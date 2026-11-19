//! Sector summaries and the super-sector graph (feature `route`).
//!
//! The ignored test measures retained bytes and edit cost on a saved bake of the
//! long-route benchmark world, in release mode:
//!
//! ```text
//! NAV_ROUTE_FIXTURE=<dir of x-z.dnav tiles> NAV_ROUTE_KM=4 cargo test --release \
//!     --features route --test route_sectors -- --ignored --nocapture
//! ```
#![cfg(all(feature = "route", feature = "recast", feature = "detour"))]

mod common;

use std::time::Instant;

use common::world::{Edit, MAX_Y, MIN_Y, World};
use navmesh_rs::bake::{BakeConfig, BakeJob, GridLayout, JobOptions, TileKey, TileOutput};
use navmesh_rs::detour::{DefaultFilter, NavMesh, TileRef};
use navmesh_rs::route::{
    SectorCollector, SectorGraph, SectorKey, SectorParams, SectorSummary, SuperKey, SuperRecord,
    UNREACHABLE, profile_hash,
};
use navmesh_rs::{Budget, Error, Limits};

fn bake(world: &World, cfg: &BakeConfig) -> (GridLayout, Vec<TileOutput>) {
    let grid = world.layout(cfg);
    let mut job = BakeJob::new(world, cfg, &grid, grid.keys(), JobOptions::default()).unwrap();
    job.run_parallel().unwrap();
    let mut out = Vec::new();
    while let Some(t) = job.next_ready() {
        out.push(t);
    }
    out.sort_by_key(|t| t.key);
    (grid, out)
}

fn small_world() -> (GridLayout, Vec<TileOutput>) {
    let mut cfg = BakeConfig::default();
    cfg.recast.tile_size = 64;
    cfg.recast.detail_sample_dist = 1.8;
    // A wall across most of the world splits sectors into regions.
    let world = World::new(120.0, 7).with(Edit::Wall {
        min: [58.0, MIN_Y, 10.0],
        max: [62.0, MAX_Y, 110.0],
    });
    bake(&world, &cfg)
}

fn mesh<'a>(grid: &GridLayout, tiles: impl Iterator<Item = &'a TileOutput>) -> NavMesh {
    let mut nav = NavMesh::new(&grid.nav_mesh_params(256, 1 << 12)).unwrap();
    for t in tiles {
        if let Some(d) = &t.data {
            nav.add_tile(d.clone()).unwrap();
        }
    }
    nav
}

fn sectors_of(grid: &GridLayout, params: &SectorParams) -> Vec<SectorKey> {
    let mut keys: Vec<_> = grid
        .keys()
        .iter()
        .map(|k| params.sector_of_tile(k.x, k.z))
        .collect();
    keys.sort();
    keys.dedup();
    keys
}

fn summary(nav: &NavMesh, key: SectorKey, params: &SectorParams) -> SectorSummary {
    let filter = DefaultFilter::default();
    SectorSummary::build(
        nav,
        key,
        params,
        &filter,
        profile_hash(&filter),
        &mut Budget::unlimited(),
    )
    .unwrap()
    .into_value()
    .unwrap()
}

#[test]
fn summaries_are_load_order_independent_and_match_the_bake_collector() {
    let (grid, tiles) = small_world();
    let params = SectorParams::new(2, 2);
    let forward = mesh(&grid, tiles.iter());
    let reverse = mesh(&grid, tiles.iter().rev());
    let filter = DefaultFilter::default();
    let mut collector = SectorCollector::new(
        &grid,
        &params,
        filter.clone(),
        profile_hash(&filter),
        Limits::default(),
    )
    .unwrap();
    let mut collected = std::collections::BTreeMap::new();
    // Bake output order differs from both meshes' insertion orders.
    for t in tiles
        .iter()
        .rev()
        .step_by(2)
        .chain(tiles.iter().rev().skip(1).step_by(2))
    {
        if let Some((k, s)) = collector.push_output(t).unwrap() {
            collected.insert(k, s);
        }
    }
    assert_eq!(collector.pending(), 0);
    let mut split_regions = 0;
    for key in sectors_of(&grid, &params) {
        let s = summary(&forward, key, &params);
        let bytes = s.to_bytes();
        assert_eq!(bytes.len(), s.encoded_len());
        assert_eq!(summary(&reverse, key, &params).to_bytes(), bytes, "{key:?}");
        assert_eq!(collected[&key].to_bytes(), bytes, "collector {key:?}");
        assert_eq!(
            SectorSummary::from_bytes(&bytes, &Limits::default()).unwrap(),
            s
        );
        assert!(!s.regions().is_empty() && s.polygon_count() > 0);
        split_regions += usize::from(s.regions().len() > 1);
        // Weak components of ground polygons are mutually reachable inside the
        // sector, and nothing else is.
        for (i, a) in s.portals().iter().enumerate() {
            for (j, b) in s.portals().iter().enumerate() {
                let q = s.quantized_cost(i, j).unwrap();
                assert_eq!(q == UNREACHABLE, a.region != b.region, "{key:?} {i} {j}");
                if i == j {
                    assert_eq!(q, 0);
                }
            }
        }
    }
    assert!(split_regions > 0, "the wall splits some sector");
}

#[test]
fn records_reject_malformed_bytes() {
    let (grid, tiles) = small_world();
    let params = SectorParams::new(2, 2);
    let nav = mesh(&grid, tiles.iter());
    let s = summary(&nav, SectorKey { x: 1, z: 1 }, &params);
    let good = s.to_bytes();
    let limits = Limits::default();
    let bad = |at: usize, v: u8| {
        let mut b = good.clone();
        b[at] = v;
        b
    };
    let n = s.transition_count();
    assert!(n > 1);
    let costs = good.len() - 2 * n * n;
    for (what, bytes) in [
        ("truncated", good[..good.len() - 1].to_vec()),
        ("trailing", [good.as_slice(), &[0]].concat()),
        ("magic", bad(0, b'X')),
        ("version", bad(4, 9)),
        ("flags", bad(6, 1)),
        ("padding", bad(19, 1)),
        ("nan quantum", {
            let mut b = good.clone();
            b[32..36].copy_from_slice(&f32::NAN.to_bits().to_le_bytes());
            b
        }),
        ("diagonal", bad(costs, 1)),
        ("tile outside the sector", bad(44, 0x7f)),
    ] {
        assert!(
            matches!(
                SectorSummary::from_bytes(&bytes, &limits),
                Err(Error::InvalidInput(_))
            ),
            "{what}"
        );
    }
    let tight = Limits {
        max_bytes: good.len() - 1,
        ..Limits::default()
    };
    assert_eq!(
        SectorSummary::from_bytes(&good, &tight),
        Err(Error::LimitExceeded)
    );

    let members: Vec<_> = sectors_of(&grid, &params)
        .into_iter()
        .filter(|k| params.super_of(*k) == SuperKey { x: 0, z: 0 })
        .map(|k| summary(&nav, k, &params))
        .collect();
    let refs: Vec<_> = members.iter().collect();
    let rec = SuperRecord::derive(SuperKey { x: 0, z: 0 }, &params, &refs).unwrap();
    let good = rec.to_bytes();
    assert_eq!(good.len(), rec.encoded_len());
    assert_eq!(SuperRecord::from_bytes(&good, &limits).unwrap(), rec);
    for bytes in [
        good[..good.len() - 1].to_vec(),
        [good.as_slice(), &[0]].concat(),
        [b"XNSR".as_slice(), &good[4..]].concat(),
    ] {
        assert!(SuperRecord::from_bytes(&bytes, &limits).is_err());
    }
}

/// Super-regions reachable from `(key, region)` over level-3 edges.
fn reach(graph: &SectorGraph, profile: u32, from: (SuperKey, u16)) -> Vec<(SuperKey, u16)> {
    let mut seen = vec![from];
    let mut i = 0;
    while i < seen.len() {
        let (k, r) = seen[i];
        for e in graph.super_edges(profile, k, r) {
            if !seen.contains(&(e.to, e.region)) {
                seen.push((e.to, e.region));
            }
        }
        i += 1;
    }
    seen.sort();
    seen
}

#[test]
fn an_edit_rebuilds_one_sector_and_one_super_sector() {
    let (grid, tiles) = small_world();
    let params = SectorParams::new(2, 2);
    let filter = DefaultFilter::default();
    let profile = profile_hash(&filter);
    let mut nav = mesh(&grid, tiles.iter());
    let mut graph = SectorGraph::new(&params, Limits::default()).unwrap();
    let sectors = sectors_of(&grid, &params);
    for &key in &sectors {
        graph.insert_summary(summary(&nav, key, &params)).unwrap();
    }
    assert_eq!(graph.sector_count(), sectors.len());
    let supers = graph.super_count();
    assert!(supers >= 4);
    // Level 3 links neighbouring super-sectors across their shared portals.
    let origin = (SuperKey { x: 0, z: 0 }, 0);
    let reached = reach(&graph, profile, origin);
    assert!(reached.len() > 1, "{reached:?}");
    let bytes = graph.retained_bytes_by_level();
    assert_eq!(bytes[0], 0);
    assert!(bytes[1] > 0 && bytes[2] > 0);
    let c = graph
        .compression(profile, SectorKey { x: 1, z: 1 })
        .unwrap();
    assert!(c.polygons > 0 && c.portals > 0);

    // Remove one tile on a sector border, then put it back.
    let key = TileKey { x: 1, z: 2 };
    let sector = params.sector_of_tile(key.x, key.z);
    let before = graph.sector(profile, sector).unwrap().to_bytes();
    let slot = nav.tiles_at(key.x, key.z, 4)[0];
    let tile_ref: TileRef = nav.tile_ref(slot);
    let data = nav.remove_tile(tile_ref).unwrap();
    let edited = summary(&nav, sector, &params);
    assert_ne!(edited.to_bytes(), before);
    let update = graph.insert_summary(edited).unwrap();
    assert_eq!((update.sectors, update.supers, update.stale), (1, 1, 0));
    assert!((1..=9).contains(&update.edge_sets));
    nav.add_tile(data).unwrap();
    graph
        .insert_summary(summary(&nav, sector, &params))
        .unwrap();
    assert_eq!(graph.sector(profile, sector).unwrap().to_bytes(), before);
    assert_eq!(reach(&graph, profile, origin), reached);

    // Level 3 loads without level 2; one resident sector cannot re-derive it.
    let sk = params.super_of(sector);
    let record = graph.super_record(profile, sk).unwrap().clone();
    assert!(record.sectors().len() > 1);
    let mut fresh = SectorGraph::new(&params, Limits::default()).unwrap();
    for k in 0..4 {
        let k = SuperKey { x: k % 2, z: k / 2 };
        if let Some(r) = graph.super_record(profile, k) {
            fresh.insert_super(&r.to_bytes()).unwrap();
        }
    }
    assert_eq!(reach(&fresh, profile, origin), reached);
    let update = fresh
        .insert_sector(&graph.sector(profile, sector).unwrap().to_bytes())
        .unwrap();
    assert_eq!((update.supers, update.stale), (0, 1));
    assert_eq!(fresh.is_stale(profile, sk), Some(true));
    assert_eq!(fresh.super_record(profile, sk), Some(&record));
}

fn percentile(v: &mut [f64], p: f64) -> f64 {
    v.sort_by(f64::total_cmp);
    v[((v.len() - 1) as f64 * p).round() as usize]
}

#[test]
#[ignore = "measurement; see the module documentation"]
fn sector_graph_memory_and_edit_cost() {
    let dir = std::path::PathBuf::from(std::env::var("NAV_ROUTE_FIXTURE").expect("fixture dir"));
    let size: f32 = std::env::var("NAV_ROUTE_KM")
        .ok()
        .and_then(|s| s.parse().ok())
        .unwrap_or(4.0)
        * 1000.0;
    let mut cfg = BakeConfig::default();
    cfg.recast.tile_size = 256;
    let grid = World::new(size, 1).layout(&cfg);
    let keys = grid.keys();
    let mut nav = NavMesh::new(&grid.nav_mesh_params(keys.len() as i32, 1 << 16)).unwrap();
    let (mut polygons, mut loaded) = (0usize, 0usize);
    for k in &keys {
        if let Ok(bytes) = std::fs::read(dir.join(format!("{}-{}.dnav", k.x, k.z))) {
            let r = nav.add_tile(bytes).unwrap();
            polygons += nav.tile_by_ref(r).unwrap().polys.len();
            loaded += 1;
        }
    }
    assert!(loaded > 0, "no fixture tiles");
    let params = SectorParams::default();
    let filter = DefaultFilter::default();
    let profile = profile_hash(&filter);
    let mut graph = SectorGraph::new(&params, Limits::UNLIMITED).unwrap();
    let sectors = sectors_of(&grid, &params);
    let start = Instant::now();
    for &key in &sectors {
        graph.insert_summary(summary(&nav, key, &params)).unwrap();
    }
    let prepare_s = start.elapsed().as_secs_f64();
    let [_, l2, l3] = graph.retained_bytes_by_level();
    let mut ratio = Vec::new();
    let (mut sum_p, mut sum_r, mut sum_t) = (0.0, 0.0, 0.0);
    for &key in &sectors {
        let c = graph.compression(profile, key).unwrap();
        ratio.push(f64::from(c.polygons) / f64::from(c.portals.max(1)));
        sum_p += f64::from(c.transitions);
        sum_r += f64::from(c.regions);
        sum_t += graph.sector(profile, key).unwrap().tiles().len() as f64;
    }
    let n = sectors.len() as f64;
    let (p, r, t) = (sum_p / n, sum_r / n, sum_t / n);
    // Published level-2 formula: per sector 44 + 16 T + 28 R + 36 P + 2 P^2.
    let formula = n * (44.0 + 16.0 * t + 28.0 * r + 36.0 * p + 2.0 * p * p);
    let design_estimate = n * (36.0 * 36.0 * 2.0 + 36.0 * 32.0);
    let compression_p05 = percentile(&mut ratio, 0.05);

    // Edits: remove and re-add an interior tile, a corner tile and a tile under a
    // sector-border portal; minimum of five repetitions each.
    let k = params.tiles_per_sector as i32;
    let mid = (grid.tiles_x / 2 / k) * k;
    let picks = [
        (
            "interior",
            TileKey {
                x: mid + k / 2,
                z: mid + k / 2,
            },
        ),
        ("corner", TileKey { x: 0, z: 0 }),
        (
            "portal",
            TileKey {
                x: mid + k - 1,
                z: mid + k / 2,
            },
        ),
    ];
    let mut edits = Vec::new();
    for (name, key) in picks {
        let sector = params.sector_of_tile(key.x, key.z);
        let before = graph.sector(profile, sector).unwrap().to_bytes();
        let mut best = f64::INFINITY;
        let mut fan = None;
        for _ in 0..5 {
            let slot = nav.tiles_at(key.x, key.z, 4)[0];
            let t0 = Instant::now();
            let data = nav.remove_tile(nav.tile_ref(slot)).unwrap();
            let up = graph
                .insert_summary(summary(&nav, sector, &params))
                .unwrap();
            nav.add_tile(data).unwrap();
            let back = graph
                .insert_summary(summary(&nav, sector, &params))
                .unwrap();
            best = best.min(t0.elapsed().as_secs_f64() / 2.0);
            assert_eq!(up, back);
            fan = Some(up);
        }
        assert_eq!(graph.sector(profile, sector).unwrap().to_bytes(), before);
        let fan = fan.unwrap();
        assert_eq!((fan.sectors, fan.supers, fan.stale), (1, 1, 0));
        edits.push(format!(
            "{{\"tile\":\"{name}\",\"edit_ms\":{:.3},\"sectors\":{},\"supers\":{},\"edge_sets\":{}}}",
            best * 1e3,
            fan.sectors,
            fan.supers,
            fan.edge_sets
        ));
    }
    let receipt = format!(
        "{{\"km\":{},\"tiles\":{},\"polygons\":{polygons},\"sectors\":{},\"supers\":{},\
         \"prepare_s\":{prepare_s:.3},\"level2_bytes\":{l2},\"level3_bytes\":{l3},\
         \"level2_bytes_per_sector\":{:.1},\"mean_transitions\":{p:.2},\"mean_regions\":{r:.3},\
         \"mean_tiles\":{t:.2},\"formula_bytes\":{formula:.0},\"formula_ratio\":{:.3},\
         \"design_estimate_bytes\":{design_estimate:.0},\"compression_p05\":{compression_p05:.2},\
         \"edits\":[{}]}}",
        size / 1000.0,
        loaded,
        sectors.len(),
        graph.super_count(),
        l2 as f64 / n,
        l2 as f64 / formula,
        edits.join(",")
    );
    println!("{receipt}");
    if let Ok(path) = std::env::var("NAV_ROUTE_RECEIPT") {
        std::fs::write(path, &receipt).unwrap();
    }
    assert!(l3 <= 16 << 20, "level 3 within 16 MB");
    let ratio = l2 as f64 / formula;
    assert!(
        (1.0 / 1.5..=1.5).contains(&ratio),
        "formula within 1.5x: {ratio}"
    );
}

#[test]
fn coarse_search_is_slice_invariant_and_invalidated_by_record_changes() {
    use navmesh_rs::route::{RegionKey, RouteIndex};
    let (grid, tiles) = small_world();
    let params = SectorParams::new(2, 2);
    let nav = mesh(&grid, tiles.iter());
    let profile = profile_hash(&DefaultFilter::default());
    let mut graph = SectorGraph::new(&params, Limits::default()).unwrap();
    for key in sectors_of(&grid, &params) {
        graph.insert_summary(summary(&nav, key, &params)).unwrap();
    }
    let from = (SuperKey { x: 0, z: 0 }, 0);
    let reachable = reach(&graph, profile, from);
    let to = *reachable.last().unwrap();
    let index = RouteIndex::build(&graph, profile).unwrap();
    let run = |slice| {
        let mut search = index
            .request(RegionKey::new(from.0, from.1), RegionKey::new(to.0, to.1))
            .unwrap();
        let mut zero = Budget::new(0);
        assert!(!search.resume(&graph, &mut zero).unwrap().is_done());
        let mut work = 0;
        loop {
            let mut budget = Budget::new(slice);
            let done = search.resume(&graph, &mut budget).unwrap().is_done();
            assert!(budget.used() <= slice);
            work += budget.used();
            assert!(work < 10000);
            if done {
                break;
            }
        }
        (search.corridor_reversed().unwrap().to_vec(), work)
    };
    let expected = run(1);
    assert!(!expected.0.is_empty());
    assert_eq!(expected, run(17));
    assert_eq!(expected, run(1000));
    let mut pending = index
        .request(RegionKey::new(from.0, from.1), RegionKey::new(to.0, to.1))
        .unwrap();
    graph.remove_sector(profile, SectorKey { x: 0, z: 0 });
    assert!(matches!(
        pending.resume(&graph, &mut Budget::new(1)),
        Err(Error::MeshChanged)
    ));
}

#[test]
fn live_window_is_slice_invariant_and_rejects_tile_edits() {
    use navmesh_rs::detour::NavMeshQuery;
    use navmesh_rs::route::{CorridorWindow, WindowPoint};
    let (grid, tiles) = small_world();
    let params = SectorParams::new(2, 2);
    let mut nav = mesh(&grid, tiles.iter());
    let filter = DefaultFilter::default();
    let q = NavMeshQuery::new(&nav, 1024).unwrap();
    let snap = |p| {
        let (r, p, _) = q.find_nearest_poly(&p, &[8.0, 20.0, 8.0], &filter).unwrap();
        assert_ne!(r, 0);
        WindowPoint::new(r, p)
    };
    let a = snap([10.0, 0.0, 10.0]);
    let b = snap([30.0, 0.0, 30.0]);
    let sectors = sectors_of(&grid, &params);
    let run = |slice| {
        let mut window =
            CorridorWindow::new(&nav, &params, &sectors, filter.clone(), a, b, 1024).unwrap();
        let mut work = 0;
        loop {
            let mut budget = Budget::new(slice);
            let done = window.resume(&nav, &mut budget, slice).unwrap().is_done();
            work += budget.used();
            assert!(work < 10000);
            if done {
                break;
            }
        }
        let status = window.finish(&nav, 1024).unwrap();
        assert!(!status.has_detail(navmesh_rs::detour::Status::PARTIAL_RESULT));
        (window.result().unwrap().0.to_vec(), work)
    };
    assert_eq!(run(1), run(17));
    assert!(CorridorWindow::new(&nav, &params, &[], filter.clone(), a, b, 1024).is_err());
    let mut window = CorridorWindow::new(&nav, &params, &sectors, filter, a, b, 1024).unwrap();
    let tile = nav.tile_ref(nav.decode_poly_id_tile(a.polygon) as usize);
    let bytes = nav.remove_tile(tile).unwrap();
    nav.add_tile(bytes).unwrap();
    assert!(matches!(
        window.resume(&nav, &mut Budget::new(17), 17),
        Err(Error::MeshChanged)
    ));
}
