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
use navmesh_rs::{Budget, Error, Limits, SnapshotError};

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
        // A finite table entry must agree with the directed region graph.
        for (i, a) in s.portals().iter().enumerate() {
            let mut seen = std::collections::BTreeSet::from([a.region]);
            let mut pending = vec![a.region];
            while let Some(region) = pending.pop() {
                for &(from, to) in s.region_arcs() {
                    if from == region && seen.insert(to) {
                        pending.push(to);
                    }
                }
            }
            for (j, b) in s.portals().iter().enumerate() {
                let q = s.quantized_cost(i, j).unwrap();
                assert_eq!(
                    q != UNREACHABLE,
                    seen.contains(&b.region),
                    "{key:?} {i} {j}"
                );
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
        ("flags", bad(6, 2)),
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
    // Another version is the typed version error, not corruption.
    assert_eq!(
        SectorSummary::from_bytes(&bad(4, 2), &limits),
        Err(Error::InvalidSnapshot(SnapshotError::WrongVersion))
    );
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
    let mut old = good.clone();
    old[4] = 2;
    assert_eq!(
        SuperRecord::from_bytes(&old, &limits),
        Err(Error::InvalidSnapshot(SnapshotError::WrongVersion))
    );
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

#[test]
fn stitched_corridors_agree_across_load_order_and_scheduler_slices() {
    use navmesh_rs::detour::NavMeshQuery;
    use navmesh_rs::route::{Navigator, RouteAtlas, RouteState, WindowPoint};
    let (grid, tiles) = small_world();
    let run = |nav: NavMesh| {
        let filter = DefaultFilter::default();
        let graph =
            SectorGraph::from_mesh(&nav, &SectorParams::new(2, 2), &filter, nav.limits()).unwrap();
        let atlas = RouteAtlas::new(&graph, &nav, filter.clone()).unwrap();
        let query = NavMeshQuery::new(&nav, 64).unwrap();
        let snap = |p| {
            let (r, pos, _) = query
                .find_nearest_poly(&p, &[8., 20., 8.], &filter)
                .unwrap();
            WindowPoint::new(r, pos)
        };
        let start = snap([12., 0., 45.]);
        let goal = snap([100., 0., 55.]);
        let route = |slice| {
            let mut scheduler = Navigator::new(&atlas, 2048).unwrap();
            let id = scheduler.request(&nav, start, goal).unwrap();
            let mut corridor = Vec::new();
            for _ in 0..100_000 {
                scheduler
                    .update(&nav, &mut Budget::new(slice), slice)
                    .unwrap();
                match scheduler.state(id).unwrap() {
                    RouteState::Segment => {
                        for &r in scheduler.segment(id).unwrap() {
                            let t = nav.tile_and_poly_by_ref(r).unwrap().0;
                            let key = (
                                t.header.x,
                                t.header.y,
                                t.header.layer,
                                nav.decode_poly_id(r).2,
                            );
                            if corridor.last() != Some(&key) {
                                corridor.push(key);
                            }
                        }
                        scheduler.advance(id).unwrap();
                    }
                    RouteState::Planning => {}
                    RouteState::Complete => return corridor,
                    other => panic!("{other:?}"),
                }
            }
            panic!("route did not terminate")
        };
        let path = route(1);
        assert_eq!(path, route(17));
        assert_eq!(path, route(1000));
        path
    };
    assert_eq!(
        run(mesh(&grid, tiles.iter())),
        run(mesh(&grid, tiles.iter().rev()))
    );
}

fn snap(nav: &NavMesh, p: [f32; 3]) -> navmesh_rs::route::WindowPoint {
    let query = navmesh_rs::detour::NavMeshQuery::new(nav, 64).unwrap();
    let (r, pos, _) = query
        .find_nearest_poly(&p, &[8., 20., 8.], &DefaultFilter::default())
        .unwrap();
    assert_ne!(r, 0);
    navmesh_rs::route::WindowPoint::new(r, pos)
}

/// An unstitched one-tile `'DNSS'` record with `regions` one-polygon regions and
/// `offmesh` off-mesh entries of region 0 whose far ends lie outside every region.
fn crafted_summary(regions: u16, offmesh: u16) -> Vec<u8> {
    let mut b = Vec::new();
    b.extend_from_slice(b"DNSS");
    b.extend_from_slice(&3u16.to_le_bytes());
    b.extend_from_slice(&0u16.to_le_bytes());
    b.extend_from_slice(&[0; 8]); // sector x, z
    b.extend_from_slice(&1u16.to_le_bytes());
    b.extend_from_slice(&[1, 0]); // layers, padding
    b.extend_from_slice(&0u32.to_le_bytes()); // profile
    for n in [1, 0, regions, offmesh] {
        b.extend_from_slice(&n.to_le_bytes()); // tiles, portals, regions, off-mesh
    }
    b.extend_from_slice(&1f32.to_le_bytes());
    b.extend_from_slice(&0f32.to_le_bytes());
    b.extend_from_slice(&u32::from(regions).to_le_bytes());
    b.extend_from_slice(&[0; 12]); // tile x, z, layer and padding
    b.extend_from_slice(&7u32.to_le_bytes());
    for r in 0..regions {
        b.extend_from_slice(&1u32.to_le_bytes());
        for v in [r as f32, 0., 0., r as f32 + 0.5, 0., 0.5] {
            b.extend_from_slice(&v.to_le_bytes());
        }
    }
    for _ in 0..offmesh {
        b.extend_from_slice(&[0; 4]); // region, flags, padding
        for v in [0.2f32, 0., 0.2, 1e6, 0., 1e6] {
            b.extend_from_slice(&v.to_le_bytes());
        }
        b.extend_from_slice(&[0; 4]); // tile index, padding
    }
    b.extend_from_slice(&0u32.to_le_bytes()); // arcs
    b.extend_from_slice(&(u32::from(regions) + 1).to_le_bytes());
    for r in 0..=regions {
        b.extend_from_slice(&0u16.to_le_bytes());
        b.extend_from_slice(&u32::from(r).to_le_bytes());
        b.extend_from_slice(&if r == regions { u16::MAX } else { r }.to_le_bytes());
    }
    b.extend_from_slice(&0u32.to_le_bytes()); // exits
    let n = usize::from(offmesh);
    for i in 0..n * n {
        let c = if i % (n + 1) == 0 { 0 } else { UNREACHABLE };
        b.extend_from_slice(&c.to_le_bytes());
    }
    b
}

#[test]
fn crafted_sector_records_end_at_the_work_limit() {
    let bytes = crafted_summary(1000, 500);
    let params = SectorParams::new(1, 1);
    // Decoding fits the limit; matching 500 off-mesh landings against 1000 region
    // boxes does not.
    let limits = Limits {
        max_work: 300_000,
        ..Limits::UNLIMITED
    };
    assert!(SectorSummary::from_bytes(&bytes, &limits).is_ok());
    let mut graph = SectorGraph::new(&params, limits).unwrap();
    assert_eq!(graph.insert_sector(&bytes), Err(Error::LimitExceeded));
    assert_eq!((graph.sector_count(), graph.super_count()), (0, 0));
    let mut unlimited = SectorGraph::new(&params, Limits::UNLIMITED).unwrap();
    assert!(unlimited.insert_sector(&bytes).is_ok());
}

#[test]
fn records_reject_inconsistent_contents() {
    let (grid, tiles) = small_world();
    let params = SectorParams::new(2, 2);
    let nav = mesh(&grid, tiles.iter());
    let limits = Limits::default();
    let summaries: Vec<_> = sectors_of(&grid, &params)
        .into_iter()
        .map(|k| summary(&nav, k, &params))
        .collect();
    // A portal moved to the opposite side no longer lies in its tile's column.
    let s = summaries.iter().find(|s| !s.portals().is_empty()).unwrap();
    let mut b = s.to_bytes();
    let at = 44 + 16 * s.tiles().len() + 28 * s.regions().len();
    b[at] = (b[at] + 4) & 7;
    assert!(matches!(
        SectorSummary::from_bytes(&b, &limits),
        Err(Error::InvalidInput(_))
    ));
    // A finite cost between regions the arcs do not connect.
    let (s, f, t) = summaries
        .iter()
        .filter(|s| s.region_arcs().is_empty())
        .find_map(|s| {
            let region = |i: usize| s.portals().get(i).map(|p| p.region);
            let n = s.transition_count();
            (0..n * n).find_map(|i| {
                let (f, t) = (i / n, i % n);
                (region(f).is_some()
                    && region(t).is_some()
                    && region(f) != region(t)
                    && s.quantized_cost(f, t) == Some(UNREACHABLE))
                .then_some((s, f, t))
            })
        })
        .expect("a sector split by the wall");
    let n = s.transition_count();
    let mut b = s.to_bytes();
    let at = b.len() - 2 * n * n + 2 * (f * n + t);
    b[at..at + 2].copy_from_slice(&1u16.to_le_bytes());
    assert!(matches!(
        SectorSummary::from_bytes(&b, &limits),
        Err(Error::InvalidInput(_))
    ));
    // Super-region members must name each sector region exactly once.
    let members: Vec<_> = summaries
        .iter()
        .filter(|s| params.super_of(s.key()) == SuperKey { x: 0, z: 0 })
        .collect();
    let rec = SuperRecord::derive(SuperKey { x: 0, z: 0 }, &params, &members).unwrap();
    let good = rec.to_bytes();
    let first = 44 + 16 * rec.sectors().len() + 36 * rec.regions().len();
    let count: usize = rec.regions().iter().map(|r| r.members.len()).sum();
    // The last member skips a region index.
    let mut gap = good.clone();
    let last = first + 4 * (count - 1) + 2;
    let g = u16::from_le_bytes([gap[last], gap[last + 1]]) + 1;
    gap[last..last + 2].copy_from_slice(&g.to_le_bytes());
    let mut cases = vec![gap];
    if rec.regions().len() > 1 {
        // The second super-region's first member repeats the first's.
        let mut twice = good.clone();
        twice.copy_within(first..first + 4, first + 4 * rec.regions()[0].members.len());
        cases.push(twice);
    }
    for b in cases {
        assert!(matches!(
            SuperRecord::from_bytes(&b, &limits),
            Err(Error::InvalidInput(_))
        ));
    }
}

#[test]
fn records_from_different_bakes_are_not_combined() {
    use navmesh_rs::route::RouteIndex;
    let (grid, tiles) = small_world();
    let nav = mesh(&grid, tiles.iter());
    let filter = DefaultFilter::default();
    let profile = profile_hash(&filter);
    // Exact index: an exit naming another content id of its destination sector.
    let params = SectorParams::new(1, 1);
    let g = SectorGraph::from_mesh(&nav, &params, &filter, nav.limits()).unwrap();
    let keys = sectors_of(&grid, &params);
    let records: Vec<Vec<u8>> = keys
        .iter()
        .map(|k| {
            g.super_record(profile, params.super_of(*k))
                .unwrap()
                .to_bytes()
        })
        .collect();
    let load = |tamper: bool| {
        let mut h = SectorGraph::new(&params, Limits::default()).unwrap();
        for (i, bytes) in records.iter().enumerate() {
            let mut b = bytes.clone();
            if tamper && i == 0 {
                b[44 + 12] ^= 1; // the listed content id of its only sector
            }
            h.insert_super(&b).unwrap();
        }
        h
    };
    assert!(RouteIndex::build_exact(&load(false), profile).is_ok());
    assert!(matches!(
        RouteIndex::build_exact(&load(true), profile),
        Err(Error::InvalidInput(_))
    ));
    // Derivation: a summary re-baked alone leaves its super-sector stale.
    let params = SectorParams::new(1, 2);
    let g = SectorGraph::from_mesh(&nav, &params, &filter, nav.limits()).unwrap();
    let keys = sectors_of(&grid, &params);
    let bytes: Vec<Vec<u8>> = keys
        .iter()
        .map(|k| g.sector(profile, *k).unwrap().to_bytes())
        .collect();
    let insert_all = |changed: Option<(usize, &[u8])>| {
        let mut h = SectorGraph::new(&params, Limits::default()).unwrap();
        for (i, b) in bytes.iter().enumerate() {
            let b = changed.filter(|c| c.0 == i).map_or(b.as_slice(), |c| c.1);
            h.insert_sector(b).unwrap();
        }
        h
    };
    let fresh = insert_all(None);
    assert!(
        keys.iter()
            .all(|k| fresh.is_stale(profile, params.super_of(*k)) == Some(false))
    );
    let shared = |k: &SectorKey| {
        keys.iter()
            .filter(|o| params.super_of(**o) == params.super_of(*k))
            .count()
            > 1
    };
    let (i, rebaked) = keys
        .iter()
        .enumerate()
        .filter(|(_, k)| shared(k))
        .find_map(|(i, _)| {
            let mut b = bytes[i].clone();
            b[44 + 12] ^= 1; // a source tile's content id
            SectorSummary::from_bytes(&b, &Limits::default())
                .is_ok()
                .then_some((i, b))
        })
        .expect("a re-bakeable sector sharing its super-sector");
    let mixed = insert_all(Some((i, &rebaked)));
    assert_eq!(
        mixed.is_stale(profile, params.super_of(keys[i])),
        Some(true)
    );
    assert_eq!(
        RouteIndex::build(&mixed, profile).err(),
        Some(Error::MeshChanged)
    );
}

#[test]
fn loaded_records_relink_like_the_built_graph() {
    let (grid, tiles) = small_world();
    let nav = mesh(&grid, tiles.iter());
    let filter = DefaultFilter::default();
    let profile = profile_hash(&filter);
    let params = SectorParams::new(1, 1);
    let g = SectorGraph::from_mesh(&nav, &params, &filter, nav.limits()).unwrap();
    let keys: Vec<SuperKey> = sectors_of(&grid, &params)
        .into_iter()
        .map(|k| params.super_of(k))
        .collect();
    // Destinations arrive after the records whose exits name them.
    let mut h = SectorGraph::new(&params, Limits::default()).unwrap();
    for k in keys.iter().rev() {
        h.insert_super(&g.super_record(profile, *k).unwrap().to_bytes())
            .unwrap();
    }
    for k in &keys {
        for r in 0..g.super_record(profile, *k).unwrap().regions().len() as u16 {
            assert_eq!(g.super_edges(profile, *k, r), h.super_edges(profile, *k, r));
        }
    }
}

#[test]
fn a_record_at_the_coordinate_limit_never_links_to_itself() {
    // One super-sector at x = i32::MAX with facing portals on its +x and -x sides.
    let mut b = Vec::new();
    b.extend_from_slice(b"DNSR");
    b.extend_from_slice(&3u16.to_le_bytes());
    b.extend_from_slice(&0u16.to_le_bytes());
    b.extend_from_slice(&i32::MAX.to_le_bytes());
    b.extend_from_slice(&0i32.to_le_bytes());
    b.extend_from_slice(&1u16.to_le_bytes());
    b.extend_from_slice(&1u16.to_le_bytes());
    b.extend_from_slice(&0u32.to_le_bytes()); // profile
    b.extend_from_slice(&0f32.to_le_bytes()); // climb
    b.extend_from_slice(&1u16.to_le_bytes()); // sectors
    b.extend_from_slice(&1u16.to_le_bytes()); // regions
    for n in [1u32, 2, 0] {
        b.extend_from_slice(&n.to_le_bytes()); // members, portals, off-mesh
    }
    b.extend_from_slice(&i32::MAX.to_le_bytes());
    b.extend_from_slice(&[0; 12]); // sector z, version, content id
    b.extend_from_slice(&0u32.to_le_bytes());
    b.extend_from_slice(&1u32.to_le_bytes());
    b.extend_from_slice(&[0; 28]); // crossing bound and box
    b.extend_from_slice(&[0; 4]); // member (0, 0)
    for side in [0u8, 4] {
        b.extend_from_slice(&[side, 0, 0, 0, 0, 0, 0, 0]);
        for v in [0f32, 0., 5., 0., 10., 0., 0.] {
            b.extend_from_slice(&v.to_le_bytes());
        }
    }
    b.extend_from_slice(&[0; 8]); // arcs, exits
    let params = SectorParams::new(1, 1);
    let mut graph = SectorGraph::new(&params, Limits::default()).unwrap();
    graph.insert_super(&b).unwrap();
    assert!(
        graph
            .super_edges(0, SuperKey { x: i32::MAX, z: 0 }, 0)
            .is_empty()
    );
}

#[test]
fn an_atlas_binds_the_whole_filter_not_its_hash() {
    use navmesh_rs::route::RouteAtlas;
    // Two filters with one profile hash, found offline by a birthday search over
    // the bit patterns of two area costs.
    let with = |(one, two): (u32, u32)| {
        let mut f = DefaultFilter::default();
        f.area_cost[1] = f32::from_bits(one);
        f.area_cost[2] = f32::from_bits(two);
        f
    };
    let (a, b) = ((0x3f45_6a57, 0x3f72_b41d), (0x3fd5_f03d, 0x3fb3_5020));
    assert_eq!(profile_hash(&with(a)), profile_hash(&with(b)));
    let (grid, tiles) = small_world();
    let nav = mesh(&grid, tiles.iter());
    let graph =
        SectorGraph::from_mesh(&nav, &SectorParams::new(2, 2), &with(a), nav.limits()).unwrap();
    assert!(RouteAtlas::new(&graph, &nav, with(a)).is_ok());
    assert!(matches!(
        RouteAtlas::new(&graph, &nav, with(b)),
        Err(Error::MeshChanged)
    ));
}

#[test]
fn navigator_pools_hold_the_largest_sector_and_failures_clear_corridors() {
    use navmesh_rs::route::{Navigator, RouteAtlas, RouteState};
    let (grid, tiles) = small_world();
    let mut nav = mesh(&grid, tiles.iter());
    let filter = DefaultFilter::default();
    let params = SectorParams::new(2, 2);
    let graph = SectorGraph::from_mesh(&nav, &params, &filter, nav.limits()).unwrap();
    let atlas = RouteAtlas::new(&graph, &nav, filter.clone()).unwrap();
    let largest = sectors_of(&grid, &params)
        .iter()
        .filter_map(|k| graph.compression(profile_hash(&filter), *k))
        .map(|c| c.polygons as usize)
        .max()
        .unwrap();
    assert!(matches!(
        Navigator::new(&atlas, largest - 1),
        Err(Error::InvalidParameter(_))
    ));
    let mut s = Navigator::new(&atlas, largest).unwrap();
    let (start, goal) = (snap(&nav, [12., 0., 45.]), snap(&nav, [100., 0., 55.]));
    let id = s.request(&nav, start, goal).unwrap();
    for _ in 0..1000 {
        if s.state(id) != Some(RouteState::Planning) {
            break;
        }
        s.update(&nav, &mut Budget::new(500), 500).unwrap();
    }
    assert_eq!(s.state(id), Some(RouteState::Segment));
    // An edit through the published corridor fails the request: no getter keeps
    // returning the closed corridor.
    let poly = s.segment(id).unwrap()[0];
    nav.set_poly_flags(poly, 0).unwrap();
    assert_eq!(
        s.update(&nav, &mut Budget::new(500), 500),
        Err(Error::MeshChanged)
    );
    assert_eq!(s.state(id), Some(RouteState::Failed(Error::MeshChanged)));
    assert!(s.segment(id).is_none() && s.corners(id).is_none() && s.toward(id).is_none());
}

#[test]
fn sync_fails_only_requests_crossing_an_unloaded_tile() {
    use navmesh_rs::route::{Navigator, RouteAtlas, RouteState};
    let (grid, tiles) = small_world();
    let mut nav = mesh(&grid, tiles.iter());
    let filter = DefaultFilter::default();
    let graph =
        SectorGraph::from_mesh(&nav, &SectorParams::new(2, 2), &filter, nav.limits()).unwrap();
    let atlas = RouteAtlas::new(&graph, &nav, filter.clone()).unwrap();
    let mut s = Navigator::new(&atlas, 4096).unwrap();
    let near = s
        .request(&nav, snap(&nav, [10., 0., 10.]), snap(&nav, [30., 0., 30.]))
        .unwrap();
    let far = s
        .request(
            &nav,
            snap(&nav, [105., 0., 20.]),
            snap(&nav, [110., 0., 100.]),
        )
        .unwrap();
    for _ in 0..1000 {
        if [near, far]
            .iter()
            .all(|id| s.state(*id) != Some(RouteState::Planning))
        {
            break;
        }
        s.update(&nav, &mut Budget::new(500), 500).unwrap();
    }
    let tile_of = |r| nav.decode_poly_id_tile(r);
    let unloaded = tile_of(s.segment(near).unwrap()[0]);
    assert!(
        s.segment(far)
            .unwrap()
            .iter()
            .all(|r| tile_of(*r) != unloaded)
    );
    nav.unload_tile(nav.tile_ref(unloaded as usize)).unwrap();
    assert_eq!(s.sync(&nav), Ok(()));
    assert_eq!(s.state(near), Some(RouteState::Failed(Error::MeshChanged)));
    assert_eq!(s.state(far), Some(RouteState::Segment));
    assert!(s.segment(far).is_some());
}
