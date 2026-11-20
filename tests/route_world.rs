//! Long-route harness: the production flat planner (a 65,535-node sliced A* behind a
//! budgeted path queue) measured in frames on two generated worlds, a sparse one
//! (a few polygons per tile) and a dense one (an obstacle field, hundreds per tile).
//!
//! Run explicitly in release mode:
//! `NAV_WORLD_KM=4 NAV_ROUTE_WORLD=sparse cargo test --release --features parallel --test route_world -- --ignored --nocapture`
//!
//! Environment:
//! - `NAV_WORLD_KM` (4), `NAV_ROUTE_WORLD` (`sparse` | `dense`), `NAV_ROUTE_PAIRS` (1000);
//! - `NAV_ROUTE_FIXTURE`: a directory caching the baked tile bytes (fails closed on a
//!   different world or configuration);
//! - `NAV_ROUTE_FRAME_MS` (1.0), `NAV_ROUTE_UNIT_MS` (0.25): the shared per-frame
//!   budget and the largest indivisible unit, converted to expansions by calibration;
//! - `NAV_ROUTE_FRAME_UNITS`: pins the expansions per frame (deterministic frame counts);
//! - `NAV_ROUTE_CALIBRATION_PAIRS` (32), `NAV_ROUTE_CONCURRENT` (64);
//! - `NAV_ROUTE_PERF_CTL` / `NAV_ROUTE_PERF_ACK`: sampling control FIFOs around queries;
//! - `NAV_ROUTE_REPORT`: a file receiving the summary and the acceptance bars as JSON.
//! - `NAV_ROUTE_QUALITY_ONLY=1`: parallel deterministic quality checks, without
//!   repeating CPU baselines; requires a pinned `NAV_ROUTE_FRAME_UNITS`.
//!
//! Pairs, their interleaved order and the thread-CPU clock follow the earlier
//! generated-world measurement so its numbers stay comparable.
#![cfg(all(
    feature = "parallel",
    feature = "recast",
    feature = "crowd",
    feature = "polyref64"
))]
mod common;
#[path = "common/route_world.rs"]
mod route;
use common::world::{Edit, MAX_Y, MIN_Y, World};
use cpu_time::ThreadTime;
use navmesh_rs::bake::{BakeConfig, BakeJob, JobOptions};
use navmesh_rs::crowd::PathQueue;
use navmesh_rs::detour::{DefaultFilter, NavMesh, NavMeshQuery, PolyRef, SlotPolicy, Status};
use navmesh_rs::{Limits, Progress, Workspace};
use route::{ObstacleField, RouteWorld, distance, midpoint};
use sha2::{Digest, Sha256};
use std::path::PathBuf;
use std::time::Instant;

/// Production node pool ceiling (`NavMeshQuery::new` refuses more).
const POOL: usize = 65_535;
/// Distance along the delivered corridor checked for direction agreement, metres.
const HORIZON: f32 = 50.0;
/// A delivered first segment agrees when its best completion is within this ratio.
const AGREEMENT: f32 = 1.05;
/// Iterations per calibration slice.
const CALIBRATION_SLICE: usize = 256;

fn env<T: std::str::FromStr>(name: &str, default: T) -> T {
    std::env::var(name)
        .ok()
        .and_then(|s| s.parse().ok())
        .unwrap_or(default)
}

fn profile_event(command: &[u8]) {
    if let Some(control) = std::env::var_os("NAV_ROUTE_PERF_CTL") {
        std::fs::write(control, command).unwrap();
        if let Some(ack) = std::env::var_os("NAV_ROUTE_PERF_ACK") {
            use std::io::BufRead;
            let mut line = String::new();
            std::io::BufReader::new(std::fs::File::open(ack).unwrap())
                .read_line(&mut line)
                .unwrap();
            assert_eq!(line, "ack\n", "sampling control acknowledgement");
        }
    }
}

#[derive(Clone, Copy)]
struct Pair {
    start: PolyRef,
    goal: PolyRef,
    a: [f32; 3],
    b: [f32; 3],
}

fn area_cost(nav: &NavMesh, r: PolyRef, filter: &DefaultFilter) -> f32 {
    let (_, p) = nav.tile_and_poly_by_ref(r).unwrap();
    filter.area_cost[p.area() as usize]
}

/// Cost of a corridor under the reference metric (portal midpoints, area costs).
fn path_cost(
    nav: &NavMesh,
    q: &NavMeshQuery<'_>,
    path: &[PolyRef],
    start: [f32; 3],
    goal: [f32; 3],
    filter: &DefaultFilter,
) -> f32 {
    let mut position = start;
    let mut total = 0.0;
    for (i, r) in path.iter().enumerate() {
        let next = if i + 1 == path.len() {
            goal
        } else {
            let (a, b) = q.portal_points(*r, path[i + 1]).unwrap();
            midpoint(a, b)
        };
        total += distance(position, next) * area_cost(nav, *r, filter);
        position = next;
    }
    total
}

/// Direction agreement of a delivered corridor: the cost of its first `HORIZON`
/// metres plus the exact cost from there to the goal, over the exact cost. A
/// partial corridor shorter than the horizon is unmeasured, rather than credited
/// with an undelivered exact continuation.
fn first_segment_ratio(
    nav: &NavMesh,
    q: &NavMeshQuery<'_>,
    path: &[PolyRef],
    pair: &Pair,
    filter: &DefaultFilter,
    exact: f32,
) -> Option<f32> {
    if path.is_empty() || !exact.is_finite() {
        return None;
    }
    if exact <= 0.0 {
        return Some(1.0);
    }
    let mut position = pair.a;
    let mut cost = 0.0;
    let mut travelled = 0.0;
    for (i, &here) in path.iter().enumerate() {
        let c = area_cost(nav, here, filter);
        if i + 1 == path.len() {
            return (here == pair.goal).then(|| (cost + distance(position, pair.b) * c) / exact);
        }
        let (l, r) = q.portal_points(here, path[i + 1]).unwrap();
        let next = midpoint(l, r);
        let step = distance(position, next);
        if travelled + step >= HORIZON {
            let remaining = HORIZON - travelled;
            let end = core::array::from_fn(|axis| {
                position[axis] + (next[axis] - position[axis]) * (remaining / step)
            });
            let rest = route::exact(nav, here, pair.goal, end, pair.b, filter).0;
            return Some((cost + remaining * c + rest) / exact);
        }
        cost += step * c;
        travelled += step;
        position = next;
    }
    None
}

fn done(status: u32) -> bool {
    status & (Status::SUCCESS | Status::FAILURE) != 0
}

/// Pool exhaustion leaves reachability unknown, even when the oracle knows it.
fn reachability_resolved(status: u32, reached: bool, exact: f32) -> bool {
    if exact.is_finite() {
        reached
    } else {
        !reached
            && status & Status::SUCCESS != 0
            && status & Status::PARTIAL_RESULT != 0
            && status & Status::OUT_OF_NODES == 0
    }
}

/// One request through the production queue, alone: frames until its result,
/// the result, and the thread CPU of every queue update.
struct FlatRun {
    frames: usize,
    status: Option<u32>,
    path: Vec<PolyRef>,
    calls_ms: Vec<f64>,
    frames_ms: Vec<f64>,
}

#[allow(clippy::too_many_arguments)]
fn flat_queue(
    queue: &mut PathQueue,
    ws: &mut Workspace,
    nav: &NavMesh,
    filters: &[DefaultFilter; 16],
    pair: &Pair,
    unit_iters: usize,
    calls_per_frame: usize,
) -> FlatRun {
    let handle = queue
        .request(nav, pair.start, pair.goal, pair.a, pair.b, 0)
        .unwrap();
    let mut run = FlatRun {
        frames: 0,
        status: None,
        path: Vec::new(),
        calls_ms: Vec::new(),
        frames_ms: Vec::new(),
    };
    while !done(queue.status(handle)) {
        run.frames += 1;
        assert!(run.frames < 10_000_000, "flat request never finished");
        let mut frame = 0.0;
        for _ in 0..calls_per_frame {
            let t = ThreadTime::now();
            queue.update(ws, nav, unit_iters, filters).unwrap();
            let ms = t.elapsed().as_secs_f64() * 1000.0;
            run.calls_ms.push(ms);
            frame += ms;
            if done(queue.status(handle)) {
                break;
            }
        }
        run.frames_ms.push(frame);
    }
    run.status = queue.take_result(handle, &mut run.path).ok().map(|s| s.0);
    run
}

/// `count` requests issued together at frame one; the queue holds eight, the rest
/// wait in the caller's backlog. Returns the frame each result arrived.
struct ConcurrentRun {
    arrived: Vec<usize>,
    calls_ms: Vec<f64>,
    frames_ms: Vec<f64>,
}

fn concurrent_queue(
    queue: &mut PathQueue,
    ws: &mut Workspace,
    nav: &NavMesh,
    filters: &[DefaultFilter; 16],
    pairs: &[Pair],
    unit_iters: usize,
    calls_per_frame: usize,
) -> ConcurrentRun {
    let mut arrived = vec![0; pairs.len()];
    let mut in_flight: Vec<(usize, u32)> = Vec::new();
    let mut next = 0;
    let mut frame = 0;
    let mut scratch = Vec::new();
    let mut calls_ms = Vec::new();
    let mut frames_ms = Vec::new();
    while arrived.contains(&0) {
        frame += 1;
        assert!(frame < 10_000_000, "concurrent requests never finished");
        while in_flight.len() < 8 && next < pairs.len() {
            let p = &pairs[next];
            let h = queue.request(nav, p.start, p.goal, p.a, p.b, 0).unwrap();
            in_flight.push((next, h));
            next += 1;
        }
        let mut frame_ms = 0.0;
        for _ in 0..calls_per_frame {
            let t = ThreadTime::now();
            queue.update(ws, nav, unit_iters, filters).unwrap();
            let ms = t.elapsed().as_secs_f64() * 1000.0;
            calls_ms.push(ms);
            frame_ms += ms;
            // Results expire after two updates: harvest after every unit, even
            // when several units belong to the same simulated frame.
            let finished: Vec<_> = in_flight
                .iter()
                .copied()
                .filter(|(_, h)| done(queue.status(*h)))
                .collect();
            for (i, h) in finished {
                queue
                    .take_result(h, &mut scratch)
                    .expect("concurrent result retained");
                assert!(
                    scratch.len() > 1,
                    "concurrent result has no usable direction"
                );
                arrived[i] = frame;
                in_flight.retain(|(_, x)| *x != h);
            }
        }
        frames_ms.push(frame_ms);
    }
    ConcurrentRun {
        arrived,
        calls_ms,
        frames_ms,
    }
}

fn quantile<T: Copy + PartialOrd>(sorted: &[T], q: f64) -> Option<T> {
    if sorted.is_empty() {
        return None;
    }
    let i = ((sorted.len() as f64 * q) as usize).min(sorted.len() - 1);
    Some(sorted[i])
}

fn sorted_f64(mut v: Vec<f64>) -> Vec<f64> {
    v.sort_by(f64::total_cmp);
    v
}

fn spread(v: &[f64]) -> serde_json::Value {
    let s = sorted_f64(v.to_vec());
    serde_json::json!({
        "n": s.len(),
        "p50": quantile(&s, 0.5),
        "p95": quantile(&s, 0.95),
        "p99": quantile(&s, 0.99),
        "max": s.last(),
    })
}

/// One acceptance bar: `measured <= limit` when `at_most`, else `measured >= limit`.
/// A bar with nothing measured fails.
fn bar(name: &str, measured: Option<f64>, limit: f64, at_most: bool) -> serde_json::Value {
    let pass = measured.is_some_and(|m| if at_most { m <= limit } else { m >= limit });
    serde_json::json!({
        "bar": name,
        "measured": measured,
        "limit": limit,
        "at_most": at_most,
        "pass": pass,
    })
}

struct QualityRow {
    status: u32,
    path: Vec<PolyRef>,
    exact: f32,
    ratio: Option<f32>,
    first: Option<f32>,
    greedy: Option<f32>,
    resolved: bool,
}

/// Quality-only runs can use every worker: each owns its query and queue, and
/// indexed collection preserves the seeded pair order and route digest.
fn parallel_quality(
    nav: &NavMesh,
    pairs: &[Pair],
    world: serde_json::Value,
    frame_units: usize,
) -> serde_json::Value {
    use rayon::prelude::*;
    use std::sync::atomic::{AtomicUsize, Ordering};
    assert!(frame_units >= 4 && frame_units.is_multiple_of(4));
    let completed = AtomicUsize::new(0);
    let rows: Vec<_> = pairs
        .par_iter()
        .map(|p| {
            let filter = DefaultFilter::default();
            let filters = core::array::from_fn(|_| DefaultFilter::default());
            let mut ws = Workspace::new();
            let mut queue = PathQueue::new(&mut ws, nav, POOL, POOL).unwrap();
            // Timings are deliberately discarded in a quality replay.
            let run = flat_queue(&mut queue, &mut ws, nav, &filters, p, frame_units / 4, 4);
            let status = run.status.expect("quality replay queue result");
            let exact = route::exact(nav, p.start, p.goal, p.a, p.b, &filter).0;
            let reached = status & Status::PARTIAL_RESULT == 0 && run.path.last() == Some(&p.goal);
            assert!(!reached || exact.is_finite(), "reference reachability");
            let mut q = NavMeshQuery::new(nav, POOL).unwrap();
            let ratio = (reached && exact.is_finite() && exact > 0.0)
                .then(|| path_cost(nav, &q, &run.path, p.a, p.b, &filter) / exact);
            let first = first_segment_ratio(nav, &q, &run.path, p, &filter, exact);
            q.init_sliced_find_path(p.start, p.goal, &p.a, &p.b, &filter, 0)
                .unwrap();
            q.update_sliced_find_path(frame_units, &filter).unwrap();
            let mut greedy_path = Vec::new();
            q.finalize_sliced_find_path(&filter, &mut greedy_path, POOL)
                .unwrap();
            let greedy = first_segment_ratio(nav, &q, &greedy_path, p, &filter, exact);
            let n = completed.fetch_add(1, Ordering::Relaxed) + 1;
            if n.is_multiple_of(25) || n == pairs.len() {
                println!("quality completed={n}/{}", pairs.len());
            }
            QualityRow {
                status,
                path: run.path,
                exact,
                ratio,
                first,
                greedy,
                resolved: reachability_resolved(status, reached, exact),
            }
        })
        .collect();
    let mut digest = 0xcbf2_9ce4_8422_2325u64;
    for (i, row) in rows.iter().enumerate() {
        for byte in (i as u32)
            .to_le_bytes()
            .into_iter()
            .chain(row.status.to_le_bytes())
            .chain(row.path.iter().flat_map(|r| r.to_le_bytes()))
        {
            digest = (digest ^ byte as u64).wrapping_mul(0x100_0000_01b3);
        }
    }
    let ratios: Vec<_> = rows.iter().filter_map(|r| r.ratio.map(f64::from)).collect();
    let first: Vec<_> = rows.iter().filter_map(|r| r.first.map(f64::from)).collect();
    let greedy: Vec<_> = rows
        .iter()
        .filter_map(|r| r.greedy.map(f64::from))
        .collect();
    let required = rows.iter().filter(|r| r.exact.is_finite()).count();
    let agree = |v: &[f64]| v.iter().filter(|r| **r <= AGREEMENT as f64).count();
    let filters = core::array::from_fn(|_| DefaultFilter::default());
    let mut ws = Workspace::new();
    let mut queue = PathQueue::new(&mut ws, nav, POOL, POOL).unwrap();
    let arrived = concurrent_queue(
        &mut queue,
        &mut ws,
        nav,
        &filters,
        &pairs[..64.min(pairs.len())],
        frame_units / 4,
        4,
    )
    .arrived;
    let frames: Vec<_> = arrived.iter().map(|f| *f as f64).collect();
    serde_json::json!({
        "mode": "quality-only",
        "threads": rayon::current_num_threads(),
        "world": world,
        "pairs": pairs.len(),
        "disconnected": pairs.len() - required,
        "frame_units": frame_units,
        "flat": {
            "complete": ratios.len(),
            "reachability_resolved": rows.iter().filter(|r| r.resolved).count(),
            "reachability_required": pairs.len(),
            "pool_exhausted": rows.iter().filter(|r| r.status & Status::OUT_OF_NODES != 0).count(),
            "cost_ratio": spread(&ratios),
            "first50_checked": first.len(),
            "first50_required": required,
            "first50_agree": agree(&first),
            "first50_ratio": spread(&first),
        },
        "greedy_one_frame": {
            "first50_checked": greedy.len(),
            "first50_required": required,
            "first50_agree": agree(&greedy),
            "first50_ratio": spread(&greedy),
        },
        "concurrent": {
            "requests": arrived.len(),
            "frames_to_first_move": spread(&frames),
            "within_10_frames": arrived.iter().filter(|f| **f <= 10).count(),
        },
        "route_digest": format!("{digest:016x}"),
    })
}

#[test]
#[ignore = "generated-world measurement, 1000 long pairs"]
fn long_route_flat_comparator_in_frames() {
    let size: f32 = env("NAV_WORLD_KM", 4.0f32) * 1000.0;
    let kind = std::env::var("NAV_ROUTE_WORLD").unwrap_or_else(|_| "sparse".into());
    let field = match kind.as_str() {
        "sparse" => None,
        "dense" => Some(ObstacleField::dense(7)),
        other => panic!("NAV_ROUTE_WORLD={other}: expected sparse or dense"),
    };
    let pairs: usize = env("NAV_ROUTE_PAIRS", 1000);
    let frame_ms: f64 = env("NAV_ROUTE_FRAME_MS", 1.0);
    let unit_ms: f64 = env("NAV_ROUTE_UNIT_MS", 0.25);
    let calibration_pairs: usize = env("NAV_ROUTE_CALIBRATION_PAIRS", 32);
    let concurrent: usize = env("NAV_ROUTE_CONCURRENT", 64);
    assert!(pairs > 0 && calibration_pairs > 0 && concurrent > 0);
    assert!(frame_ms.is_finite() && unit_ms.is_finite());
    assert!(frame_ms > 0.0 && unit_ms > 0.0 && unit_ms <= frame_ms);

    let mut cfg = BakeConfig::default();
    cfg.recast.tile_size = 256;
    let base = World::new(size, 1)
        .with(Edit::Wall {
            min: [size * 0.49, MIN_Y, size * 0.10],
            max: [size * 0.51, MAX_Y, size * 0.90],
        })
        .with(Edit::Wall {
            min: [size * 0.25, 16.0, size * 0.40],
            max: [size * 0.40, 18.0, size * 0.42],
        });
    // The sparse identity matches the earlier measurement, so its fixtures load here.
    let identity = match &field {
        None => format!("{base:?}\n{cfg:?}\n{}", include_str!("common/world.rs")),
        Some(f) => format!(
            "{base:?}\n{f:?}\n{cfg:?}\n{}\n{}",
            include_str!("common/world.rs"),
            include_str!("common/route_world.rs")
        ),
    };
    let source = RouteWorld { base, field };
    let grid = source.base.layout(&cfg);
    // Large fixtures can be prepared in independently verified tile ranges.
    if let Ok(batch) = std::env::var("NAV_ROUTE_BAKE_COUNT") {
        let first = env("NAV_ROUTE_BAKE_OFFSET", 0usize);
        let keys = grid.keys();
        let end = first
            .saturating_add(batch.parse::<usize>().unwrap())
            .min(keys.len());
        let dir = PathBuf::from(std::env::var_os("NAV_ROUTE_FIXTURE").unwrap());
        std::fs::create_dir_all(&dir).unwrap();
        assert!(!dir.join("complete").exists() && first < end);
        let mut job = BakeJob::new(
            &source,
            &cfg,
            &grid,
            keys[first..end].to_vec(),
            JobOptions::default(),
        )
        .unwrap();
        let mut manifest = String::new();
        loop {
            let progress = job.run_parallel_chunk().unwrap();
            while let Some(output) = job.next_ready() {
                let k = output.key;
                let hash = output.data.map_or_else(
                    || "-".into(),
                    |bytes| {
                        let hash = format!("{:x}", Sha256::digest(&bytes));
                        std::fs::write(dir.join(format!("{}-{}.dnav", k.x, k.z)), bytes).unwrap();
                        hash
                    },
                );
                manifest.push_str(&format!("{} {} {hash}\n", k.x, k.z));
            }
            if progress == Progress::Done {
                break;
            }
        }
        assert_eq!(manifest.lines().count(), end - first);
        std::fs::write(dir.join(format!("manifest-{first}-{end}")), manifest).unwrap();
        std::fs::write(dir.join(format!("identity-{first}-{end}")), identity).unwrap();
        println!("baked {first}..{end} of {}", keys.len());
        return;
    }
    let mut nav = NavMesh::with_policy(
        &grid.nav_mesh_params(grid.keys().len() as i32, 1 << 20),
        Limits::UNLIMITED,
        SlotPolicy::Keyed {
            origin: [0, 0],
            width: grid.tiles_x as u32,
            height: grid.tiles_z as u32,
            max_layers: 1,
        },
    )
    .unwrap();
    let filter = DefaultFilter::default();

    // Bake, or load a fixture whose identity and tile digests match.
    let start = Instant::now();
    let cache = std::env::var_os("NAV_ROUTE_FIXTURE").map(PathBuf::from);
    let cached = cache.as_ref().is_some_and(|p| p.join("complete").exists());
    if let Some(dir) = &cache {
        std::fs::create_dir_all(dir).unwrap();
        if cached {
            let stored = std::fs::read_to_string(dir.join("complete")).unwrap();
            // These two additive bake options had no wire effect at their defaults.
            let legacy = identity.replacen(", links: None, spare_links_per_tile: 0 }", " }", 1);
            assert!(
                stored == identity
                    || (cfg.links.is_none() && cfg.spare_links_per_tile == 0 && stored == legacy),
                "fixture identity"
            );
        } else {
            assert!(
                std::fs::read_dir(dir).unwrap().next().is_none(),
                "incomplete fixture; use a fresh directory"
            );
        }
    }
    let mut tile_bytes = 0;
    let mut tile_polys = Vec::new();
    let mut add = |nav: &mut NavMesh, bytes: Vec<u8>| {
        tile_bytes += bytes.len();
        let r = nav.add_tile(bytes).unwrap();
        tile_polys.push(nav.tile_by_ref(r).unwrap().polys.len() as f64);
    };
    if let (true, Some(dir)) = (cached, &cache) {
        let manifest = std::fs::read_to_string(dir.join("tiles.sha256")).unwrap();
        let records: Vec<_> = manifest.lines().collect();
        assert_eq!(records.len(), grid.keys().len());
        for (key, record) in grid.keys().iter().zip(records) {
            let file = dir.join(format!("{}-{}.dnav", key.x, key.z));
            let hash = record
                .strip_prefix(&format!("{} {} ", key.x, key.z))
                .expect("fixture tile identity");
            if hash == "-" {
                assert!(!file.exists(), "empty fixture tile");
                continue;
            }
            let bytes = std::fs::read(file).unwrap();
            assert_eq!(
                format!("{:x}", Sha256::digest(&bytes)),
                hash,
                "fixture tile digest"
            );
            add(&mut nav, bytes);
        }
    } else {
        let mut job =
            BakeJob::new(&source, &cfg, &grid, grid.keys(), JobOptions::default()).unwrap();
        loop {
            let progress = job.run_parallel_chunk().unwrap();
            while let Some(output) = job.next_ready() {
                if let Some(bytes) = output.data {
                    if let Some(dir) = &cache {
                        std::fs::write(
                            dir.join(format!("{}-{}.dnav", output.key.x, output.key.z)),
                            &bytes,
                        )
                        .unwrap();
                    }
                    add(&mut nav, bytes);
                }
            }
            if progress == Progress::Done {
                break;
            }
        }
        if let Some(dir) = &cache {
            let mut manifest = String::new();
            for key in grid.keys() {
                let file = dir.join(format!("{}-{}.dnav", key.x, key.z));
                let hash = if file.exists() {
                    format!("{:x}", Sha256::digest(std::fs::read(file).unwrap()))
                } else {
                    "-".into()
                };
                manifest.push_str(&format!("{} {} {hash}\n", key.x, key.z));
            }
            std::fs::write(dir.join("tiles.sha256"), manifest).unwrap();
            std::fs::write(dir.join("complete"), &identity).unwrap();
        }
    }
    // Global linking rebuilds all connections; enable it once after bulk insertion.
    nav.set_global_off_mesh_links(true);
    let load_s = start.elapsed().as_secs_f64();
    let polygons: f64 = tile_polys.iter().sum();
    if source.field.is_some() {
        let density = polygons / tile_polys.len().max(1) as f64;
        assert!(
            (300.0..=1000.0).contains(&density),
            "dense polygons per tile: {density}"
        );
    }
    let mut border_halves = 0;
    let mut off_mesh_links = 0;
    for i in 0..nav.max_tiles() as usize {
        if let Some(tile) = nav.tile(i) {
            off_mesh_links += tile.off_mesh_cons.len();
            for p in &tile.polys {
                border_halves += p.neis[..p.vert_count as usize]
                    .iter()
                    .filter(|n| **n & navmesh_rs::detour::EXT_LINK != 0 && **n & 0xff != 0xff)
                    .count();
            }
        }
    }
    let world = serde_json::json!({
        "kind": kind,
        "size_m": size,
        "field": source.field.map(|f| format!("{f:?}")),
        "grid_tiles": grid.keys().len(),
        "nonempty_tiles": tile_polys.len(),
        "polygons": polygons,
        "polygons_per_tile_mean": polygons / tile_polys.len().max(1) as f64,
        "polygons_per_tile": spread(&tile_polys),
        "polygons_per_tile_p05": quantile(&sorted_f64(tile_polys.clone()), 0.05),
        "dnav_bytes": tile_bytes,
        "border_halves": border_halves,
        "off_mesh_links": off_mesh_links,
        "fixture": cached,
        "load_s": load_s,
    });
    println!("world {world}");

    // Seeded pairs from the west fifth to the east fifth, across the wall.
    let q = NavMeshQuery::new(&nav, POOL).unwrap();
    let mut seed = 0x69af_8b31u32;
    let mut random = || {
        seed ^= seed << 13;
        seed ^= seed >> 17;
        seed ^= seed << 5;
        (seed >> 8) as f32 / 16_777_216.0
    };
    let mut set = Vec::with_capacity(pairs);
    for pair in 0..pairs {
        let a = [
            size * (0.02 + random() * 0.18),
            10.0,
            size * (0.02 + random() * 0.96),
        ];
        let b = [
            size * (0.80 + random() * 0.18),
            10.0,
            size * (0.02 + random() * 0.96),
        ];
        let (ar, a, _) = q.find_nearest_poly(&a, &[8.0, 30.0, 8.0], &filter).unwrap();
        let (br, b, _) = q.find_nearest_poly(&b, &[8.0, 30.0, 8.0], &filter).unwrap();
        assert!(ar != 0 && br != 0, "pair {pair} endpoint projection");
        set.push(Pair {
            start: ar,
            goal: br,
            a,
            b,
        });
    }
    drop(q);
    #[cfg(feature = "route")]
    if env("NAV_ROUTE_CANDIDATE", 0) == 1 {
        candidate::measure(&nav, &set, world);
        return;
    }

    if env("NAV_ROUTE_QUALITY_ONLY", 0) == 1 {
        let frame_units = std::env::var("NAV_ROUTE_FRAME_UNITS")
            .expect("quality replay requires the original iteration budget")
            .parse()
            .unwrap();
        let summary = parallel_quality(&nav, &set, world, frame_units);
        if let Some(path) = std::env::var_os("NAV_ROUTE_REPORT") {
            std::fs::write(path, serde_json::to_vec_pretty(&summary).unwrap()).unwrap();
        }
        println!("quality {summary}");
        return;
    }

    // Calibration: thread CPU per expansion of the production sliced A*.
    let mut q = NavMeshQuery::new(&nav, POOL).unwrap();
    let mut calibration_ms = 0.0;
    let mut calibration_expanded = 0usize;
    let mut slice_us = Vec::new();
    for p in set.iter().take(calibration_pairs) {
        q.init_sliced_find_path(p.start, p.goal, &p.a, &p.b, &filter, 0)
            .unwrap();
        loop {
            let t = ThreadTime::now();
            let r = q.update_sliced_find_path(CALIBRATION_SLICE, &filter);
            let ms = t.elapsed().as_secs_f64() * 1000.0;
            let Ok((status, n)) = r else { break };
            calibration_ms += ms;
            calibration_expanded += n;
            if n == CALIBRATION_SLICE {
                slice_us.push(ms * 1000.0 / n as f64);
            }
            if status.0 & Status::IN_PROGRESS == 0 {
                break;
            }
        }
    }
    let us_per_expansion = calibration_ms * 1000.0 / calibration_expanded.max(1) as f64;
    let units_per_ms = 1000.0 / us_per_expansion;
    let frame_units: usize = std::env::var("NAV_ROUTE_FRAME_UNITS")
        .ok()
        .and_then(|s| s.parse().ok())
        .unwrap_or((units_per_ms * frame_ms) as usize)
        .max(1);
    let calls_per_frame = ((frame_ms / unit_ms).round() as usize).max(1);
    let unit_iters = (frame_units / calls_per_frame).max(1);
    let frame_units = unit_iters * calls_per_frame;
    let calibration = serde_json::json!({
        "pairs": calibration_pairs.min(set.len()),
        "expanded": calibration_expanded,
        "us_per_expansion": us_per_expansion,
        "slice_us_per_expansion": spread(&slice_us),
        "units_per_ms": units_per_ms,
        "frame_ms": frame_ms,
        "unit_ms": unit_ms,
        "frame_units": frame_units,
        "frame_units_pinned": std::env::var_os("NAV_ROUTE_FRAME_UNITS").is_some(),
        "calls_per_frame": calls_per_frame,
        "unit_iters": unit_iters,
    });
    println!("calibration {calibration}");

    // Sequential pairs: each request alone through the queue, interleaved with the
    // unbounded reference, then the greedy partial first move after one frame.
    let mut ws = Workspace::new();
    let mut queue = PathQueue::new(&mut ws, &nav, POOL, POOL).unwrap();
    let filters: [DefaultFilter; 16] = core::array::from_fn(|_| DefaultFilter::default());
    let mut frames = Vec::new();
    let mut calls_ms = Vec::new();
    let mut frames_ms = Vec::new();
    let mut flat_cpu = Vec::new();
    let mut reference_cpu = Vec::new();
    let mut reference_expanded = Vec::new();
    let mut ratios = Vec::new();
    let mut first = Vec::new();
    let mut greedy = Vec::new();
    let mut complete = 0;
    let mut exhausted = 0;
    let mut failed = 0;
    let mut disconnected = 0;
    let mut resolved = 0;
    let mut greedy_complete = 0;
    let mut digest = 0xcbf2_9ce4_8422_2325u64;
    let fnv = |digest: &mut u64, bytes: &[u8]| {
        for b in bytes {
            *digest = (*digest ^ *b as u64).wrapping_mul(0x100_0000_01b3);
        }
    };
    let mut greedy_path = Vec::new();
    for (i, p) in set.iter().enumerate() {
        let reference = || {
            profile_event(b"enable\n");
            let t = ThreadTime::now();
            let r = route::exact(&nav, p.start, p.goal, p.a, p.b, &filter);
            let s = t.elapsed().as_secs_f64();
            profile_event(b"disable\n");
            (r, s)
        };
        let reference_first = (i % 2 == 0).then(reference);
        profile_event(b"enable\n");
        let run = flat_queue(
            &mut queue,
            &mut ws,
            &nav,
            &filters,
            p,
            unit_iters,
            calls_per_frame,
        );
        profile_event(b"disable\n");
        let ((exact, expanded), reference_s) = reference_first.unwrap_or_else(reference);
        let cpu: f64 = run.frames_ms.iter().sum();
        let status = run.status.unwrap_or(Status::FAILURE);
        let pool_exhausted = status & Status::OUT_OF_NODES != 0;
        let reached = run.status.is_some()
            && status & Status::PARTIAL_RESULT == 0
            && run.path.last() == Some(&p.goal);
        assert!(!reached || exact.is_finite(), "false reachable result");
        resolved += usize::from(reachability_resolved(status, reached, exact));
        fnv(&mut digest, &(i as u32).to_le_bytes());
        fnv(&mut digest, &status.to_le_bytes());
        for r in &run.path {
            fnv(&mut digest, &r.to_le_bytes());
        }
        if !exact.is_finite() {
            disconnected += 1;
        }
        if run.status.is_none() {
            failed += 1;
        }
        if pool_exhausted {
            exhausted += 1;
        }
        let ratio = (reached && exact.is_finite() && exact > 0.0)
            .then(|| path_cost(&nav, &q, &run.path, p.a, p.b, &filter) / exact);
        if let Some(r) = ratio {
            complete += 1;
            ratios.push(r as f64);
        }
        let first_ratio = first_segment_ratio(&nav, &q, &run.path, p, &filter, exact);
        if let Some(r) = first_ratio {
            first.push(r as f64);
        }

        // Option B: one frame of sliced search, then a path to the best node so far.
        q.init_sliced_find_path(p.start, p.goal, &p.a, &p.b, &filter, 0)
            .unwrap();
        let _ = q.update_sliced_find_path(frame_units, &filter);
        let greedy_status = q.finalize_sliced_find_path(&filter, &mut greedy_path, POOL);
        let greedy_ratio = match greedy_status {
            Ok(s) => {
                if s.0 & Status::PARTIAL_RESULT == 0 && greedy_path.last() == Some(&p.goal) {
                    greedy_complete += 1;
                }
                first_segment_ratio(&nav, &q, &greedy_path, p, &filter, exact)
            }
            Err(_) => None,
        };
        if let Some(r) = greedy_ratio {
            greedy.push(r as f64);
        }

        println!(
            "pair={i} frames={} status={status:#x} reached={reached} pool_exhausted={pool_exhausted} flat_cpu_ms={cpu:.4} max_call_ms={:.4} path={} reference_ms={:.4} reference_expanded={expanded} cost_ratio={ratio:?} first50={first_ratio:?} greedy_first50={greedy_ratio:?}",
            run.frames,
            run.calls_ms.iter().copied().fold(0.0, f64::max),
            run.path.len(),
            reference_s * 1000.0,
        );
        frames.push(run.frames as f64);
        calls_ms.extend(run.calls_ms);
        frames_ms.extend(run.frames_ms);
        flat_cpu.push(cpu);
        reference_cpu.push(reference_s * 1000.0);
        reference_expanded.push(expanded as f64);
    }

    let mut queue = PathQueue::new(&mut ws, &nav, POOL, POOL).unwrap();
    let concurrent_run = concurrent_queue(
        &mut queue,
        &mut ws,
        &nav,
        &filters,
        &set[..concurrent.min(set.len())],
        unit_iters,
        calls_per_frame,
    );
    let arrived = &concurrent_run.arrived;
    let arrived_f: Vec<f64> = arrived.iter().map(|f| *f as f64).collect();

    let agree = |v: &[f64]| v.iter().filter(|r| **r <= AGREEMENT as f64).count();
    let summary = serde_json::json!({
        "world": world,
        "calibration": calibration,
        "pairs": set.len(),
        "disconnected": disconnected,
        "flat": {
            "frames_to_first_move": spread(&frames),
            "within_3_frames": frames.iter().filter(|f| **f <= 3.0).count(),
            "complete": complete,
            "pool_exhausted": exhausted,
            "failed": failed,
            "reachability_resolved": resolved,
            "reachability_required": set.len(),
            "call_ms": spread(&calls_ms),
            "calls_over_unit": calls_ms.iter().filter(|c| **c > unit_ms).count(),
            "frame_ms": spread(&frames_ms),
            "frames_over_budget": frames_ms.iter().filter(|f| **f > frame_ms).count(),
            "total_cpu_ms": spread(&flat_cpu),
            "cost_ratio": spread(&ratios),
            "first50_checked": first.len(),
            "first50_required": set.len() - disconnected,
            "first50_agree": agree(&first),
            "first50_ratio": spread(&first),
        },
        "greedy_one_frame": {
            "complete": greedy_complete,
            "first50_checked": greedy.len(),
            "first50_required": set.len() - disconnected,
            "first50_agree": agree(&greedy),
            "first50_ratio": spread(&greedy),
        },
        "reference": {
            "cpu_ms": spread(&reference_cpu),
            "expanded": spread(&reference_expanded),
        },
        "concurrent": {
            "requests": arrived.len(),
            "frames_to_first_move": spread(&arrived_f),
            "within_10_frames": arrived.iter().filter(|f| **f <= 10).count(),
            "call_ms": spread(&concurrent_run.calls_ms),
            "frame_ms": spread(&concurrent_run.frames_ms),
            "calls_over_unit": concurrent_run.calls_ms.iter().filter(|c| **c > unit_ms).count(),
            "frames_over_budget": concurrent_run.frames_ms.iter().filter(|f| **f > frame_ms).count(),
        },
        "route_digest": format!("{digest:016x}"),
    });
    // The design's acceptance bars, evaluated for this planner. Reported, never
    // asserted: the flat comparator is the baseline a route planner must beat.
    let frames_sorted = sorted_f64(frames.clone());
    let calls_max = calls_ms
        .iter()
        .chain(&concurrent_run.calls_ms)
        .copied()
        .fold(0.0, f64::max);
    let ratios_sorted = sorted_f64(ratios.clone());
    let agreement = agree(&first) as f64 / (set.len() - disconnected).max(1) as f64;
    let bars = [
        bar(
            "first_move_p95_frames",
            quantile(&frames_sorted, 0.95),
            3.0,
            true,
        ),
        bar("max_unit_ms", Some(calls_max), unit_ms, true),
        bar(
            "frames_over_budget",
            Some(
                frames_ms
                    .iter()
                    .chain(&concurrent_run.frames_ms)
                    .filter(|f| **f > frame_ms)
                    .count() as f64,
            ),
            0.0,
            true,
        ),
        bar(
            "concurrent_requests",
            Some(arrived.len() as f64),
            64.0,
            false,
        ),
        bar(
            "concurrent_max_frames",
            arrived_f.iter().copied().reduce(f64::max),
            10.0,
            true,
        ),
        bar("cost_ratio_p95", quantile(&ratios_sorted, 0.95), 1.02, true),
        bar("cost_ratio_max", ratios_sorted.last().copied(), 1.10, true),
        bar(
            "complete_fraction",
            Some(complete as f64 / (set.len() - disconnected).max(1) as f64),
            1.0,
            false,
        ),
        bar("first50_agreement", Some(agreement), 0.99, false),
        bar(
            "reachability_fraction",
            Some(resolved as f64 / set.len() as f64),
            1.0,
            false,
        ),
    ];
    let bars = serde_json::Value::from(bars.to_vec());
    println!("bars {bars}");
    let report = serde_json::json!({ "summary": summary, "bars": bars });
    if let Some(path) = std::env::var_os("NAV_ROUTE_REPORT") {
        std::fs::write(path, serde_json::to_vec_pretty(&report).unwrap()).unwrap();
    }
    println!("summary {summary}");
    assert_eq!(failed, 0, "valid queue queries must produce a result");
}

#[test]
fn bars_compare_in_their_direction_and_fail_when_unmeasured() {
    assert_eq!(bar("a", Some(3.0), 3.0, true)["pass"], true);
    assert_eq!(bar("a", Some(3.5), 3.0, true)["pass"], false);
    assert_eq!(bar("b", Some(0.99), 0.99, false)["pass"], true);
    assert_eq!(bar("b", Some(0.98), 0.99, false)["pass"], false);
    assert_eq!(bar("c", None, 1.0, true)["pass"], false);
}

/// Two triangles with a long shared portal and an analytic midpoint route.
fn metric_fixture() -> (NavMesh, Pair) {
    use navmesh_rs::detour::{NavMeshCreateParams, create_nav_mesh_data};
    let n = u16::MAX;
    let bytes = create_nav_mesh_data(&NavMeshCreateParams {
        verts: &[0, 0, 0, 0, 0, 200, 200, 0, 200, 200, 0, 0],
        vert_count: 4,
        polys: &[
            0, 1, 2, n, n, n, n, n, 1, n, n, n, 0, 2, 3, n, n, n, 0, n, n, n, n, n,
        ],
        poly_count: 2,
        nvp: 6,
        poly_flags: &[1, 1],
        poly_areas: &[0, 0],
        bmin: [0.0; 3],
        bmax: [200.0, 1.0, 200.0],
        walkable_height: 2.0,
        walkable_radius: 0.6,
        walkable_climb: 0.5,
        cs: 1.0,
        ch: 1.0,
        build_bv_tree: true,
        ..Default::default()
    })
    .unwrap();
    let nav = NavMesh::new_single(bytes).unwrap().0;
    let q = NavMeshQuery::new(&nav, 8).unwrap();
    let filter = DefaultFilter::default();
    let a = [10.0, 0.0, 150.0];
    let b = [150.0, 0.0, 10.0];
    let start = q.find_nearest_poly(&a, &[1.0; 3], &filter).unwrap().0;
    let goal = q.find_nearest_poly(&b, &[1.0; 3], &filter).unwrap().0;
    assert_ne!(start, goal);
    drop(q);
    (nav, Pair { start, goal, a, b })
}

#[test]
fn direction_requires_a_delivered_horizon_and_reference_has_known_cost() {
    let (nav, pair) = metric_fixture();
    let filter = DefaultFilter::default();
    let q = NavMeshQuery::new(&nav, 8).unwrap();
    let exact = route::exact(&nav, pair.start, pair.goal, pair.a, pair.b, &filter).0;
    let expected = distance(pair.a, [100.0, 0.0, 100.0]) + distance([100.0, 0.0, 100.0], pair.b);
    assert!((exact - expected).abs() < 1e-4);
    assert_eq!(
        first_segment_ratio(&nav, &q, &[pair.start], &pair, &filter, exact),
        None
    );
    let ratio =
        first_segment_ratio(&nav, &q, &[pair.start, pair.goal], &pair, &filter, exact).unwrap();
    assert!((ratio - 1.0).abs() < 1e-5);
}

#[test]
fn concurrent_results_are_collected_before_the_next_unit_can_expire_them() {
    let (nav, pair) = metric_fixture();
    let mut ws = Workspace::new();
    let mut queue = PathQueue::new(&mut ws, &nav, 8, 8).unwrap();
    let filters = core::array::from_fn(|_| DefaultFilter::default());
    let frames = concurrent_queue(&mut queue, &mut ws, &nav, &filters, &[pair; 64], 64, 4).arrived;
    assert_eq!(frames.len(), 64);
    assert!(frames.iter().all(|frame| *frame > 0 && *frame <= 8));
}

#[test]
fn quality_replay_is_identical_with_one_or_four_workers() {
    let (nav, pair) = metric_fixture();
    let run = |threads| {
        rayon::ThreadPoolBuilder::new()
            .num_threads(threads)
            .build()
            .unwrap()
            .install(|| parallel_quality(&nav, &[pair; 64], serde_json::json!({}), 64))
    };
    let mut one = run(1);
    let mut four = run(4);
    one["threads"] = serde_json::json!(0);
    four["threads"] = serde_json::json!(0);
    assert_eq!(one, four);
    assert_eq!(one["flat"]["first50_agree"], 64);
    assert_eq!(one["concurrent"]["requests"], 64);
}

#[test]
fn unreachable_partial_is_a_result_and_pool_exhaustion_is_unresolved() {
    let (mut nav, pair) = metric_fixture();
    nav.set_poly_flags(pair.goal, 0).unwrap();
    let filters = core::array::from_fn(|_| DefaultFilter::default());
    let mut ws = Workspace::new();
    let mut queue = PathQueue::new(&mut ws, &nav, 8, 8).unwrap();
    let run = flat_queue(&mut queue, &mut ws, &nav, &filters, &pair, 16, 4);
    let status = run
        .status
        .expect("unreachable query still produces a result");
    let exact = route::exact(&nav, pair.start, pair.goal, pair.a, pair.b, &filters[0]).0;
    assert!(!exact.is_finite());
    assert_eq!(run.path, vec![pair.start]);
    assert!(reachability_resolved(status, false, exact));
    assert!(!reachability_resolved(
        status | Status::OUT_OF_NODES,
        false,
        exact
    ));
    assert!(!reachability_resolved(status, false, 1.0));
}

#[test]
fn obstacle_field_is_a_pure_function_of_its_seed() {
    let field = ObstacleField::dense(7);
    let placed: Vec<_> = (0..16)
        .flat_map(|z| (0..16).map(move |x| (x, z)))
        .filter_map(|(x, z)| field.obstacle(x, z))
        .collect();
    let again: Vec<_> = (0..16)
        .flat_map(|z| (0..16).map(move |x| (x, z)))
        .filter_map(|(x, z)| field.obstacle(x, z))
        .collect();
    assert_eq!(placed, again);
    // Fill is a rate per 256 blocks; 256 blocks land near it.
    assert!((140..210).contains(&placed.len()), "{}", placed.len());
    for (min, max) in placed {
        for axis in [0, 2] {
            let edge = max[axis] - min[axis];
            assert!((field.min_edge - 1e-3..=field.max_edge + 1e-3).contains(&edge));
            let block = (min[axis] / field.block).floor();
            assert!(
                max[axis] <= (block + 1.0) * field.block + 1e-3,
                "stays in its block"
            );
        }
        assert_eq!(max[1], field.top);
    }
    let other = ObstacleField::dense(8);
    assert_ne!(other.obstacle(3, 5), field.obstacle(3, 5));
}

#[cfg(feature = "route")]
#[path = "common/route_candidate.rs"]
mod candidate;
