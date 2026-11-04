#![cfg(all(feature = "recast", feature = "detour"))]
use navmesh_rs::connectivity::{Connectivity, NoPathReason, diagnose_path};
use navmesh_rs::detour::{
    self, DefaultFilter, NavMesh, NavMeshCreateParams, NavMeshParams, NavMeshQuery, PolyRef,
};
use navmesh_rs::{Budget, Budgeted, Error};

fn quad(x: i32, y: i32, variant: u16) -> Vec<u8> {
    const N: u16 = 0xffff;
    let polys = [0, 1, 2, 3, N, N, 0x8000, 0x8001, 0x8002, 0x8003, N, N];
    let (ox, oz) = (x as f32 * 10.0, y as f32 * 10.0);
    detour::create_nav_mesh_data(&NavMeshCreateParams {
        verts: &[0u16, 0, 0, 0, 0, 10, 10, 0, 10, 10, 0, 0],
        vert_count: 4,
        polys: &polys,
        poly_flags: &[1 + variant],
        poly_areas: &[0],
        poly_count: 1,
        nvp: 6,
        tile_x: x,
        tile_y: y,
        bmin: [ox, 0.0, oz],
        bmax: [ox + 10.0, 1.0, oz + 10.0],
        walkable_height: 2.0,
        walkable_radius: 0.5,
        walkable_climb: 0.5,
        cs: 1.0,
        ch: 1.0,
        build_bv_tree: true,
        ..Default::default()
    })
    .expect("valid streaming fixture")
}

fn mesh(xs: &[i32]) -> NavMesh {
    let mut nav = NavMesh::new(&NavMeshParams {
        orig: [0.0; 3],
        tile_width: 10.0,
        tile_height: 10.0,
        max_tiles: 64,
        max_polys: 64,
    })
    .unwrap();
    for &x in xs {
        nav.add_tile(quad(x, 0, 0)).unwrap();
    }
    nav
}
fn conn(nav: &NavMesh) -> Connectivity {
    match Connectivity::new(nav, &mut Budget::unlimited()).unwrap() {
        Budgeted::Complete(Some(c)) => c,
        _ => panic!("unlimited build"),
    }
}
fn at(nav: &NavMesh, x: i32) -> PolyRef {
    nav.tile_ref(nav.tile_index_at(x, 0, 0).unwrap())
}
fn pos(x: i32) -> [f32; 3] {
    [x as f32 * 10.0 + 5.0, 0.0, 5.0]
}
fn reason(
    nav: &NavMesh,
    c: &Connectivity,
    a: [f32; 3],
    b: [f32; 3],
    f: &DefaultFilter,
    nodes: usize,
    work: u64,
) -> Option<NoPathReason> {
    diagnose_path(
        &mut NavMeshQuery::new(nav, nodes).unwrap(),
        c,
        a,
        b,
        [1.0; 3],
        f,
        &mut Budget::new(work),
    )
    .unwrap()
    .reason
}
#[test]
fn labels_and_projected_area_match_components() {
    let nav = mesh(&[0, 1, 3]);
    let c = conn(&nav);
    assert_eq!(c.island(at(&nav, 0)), c.island(at(&nav, 1)));
    assert_ne!(c.island(at(&nav, 0)), c.island(at(&nav, 3)));
    assert_ne!(c.tile_island(at(&nav, 0)), c.tile_island(at(&nav, 1)));
    let r = c
        .reachability(2, &mut Budget::unlimited())
        .unwrap()
        .into_value()
        .unwrap();
    assert_eq!(
        (r.island_count, r.total_polys, r.islands_below_threshold),
        (2, 3, 1)
    );
    assert_eq!(r.largest_island_share, 2.0 / 3.0);
    assert_eq!(r.islands.iter().map(|s| s.xz_area).sum::<f64>(), 300.0);
    assert_eq!(
        c.reachable_from(&[at(&nav, 0); 2], &mut Budget::unlimited())
            .unwrap()
            .into_value()
            .unwrap()
            .reachable_polys,
        2
    );
}
#[test]
fn removal_splits_and_reload_merges_labels() {
    let mut nav = mesh(&[0, 1, 2]);
    let mut c = conn(&nav);
    let r = at(&nav, 1);
    let unloaded = nav.unload_tile(r).unwrap();
    c.update(&nav, &mut Budget::unlimited()).unwrap();
    assert_ne!(c.island(at(&nav, 0)), c.island(at(&nav, 2)));
    assert_eq!(c.island(r), None);
    let prepared = detour::prepare_tile(nav.params(), nav.limits(), unloaded.bytes).unwrap();
    nav.commit(prepared).unwrap();
    c.update(&nav, &mut Budget::unlimited()).unwrap();
    assert_eq!(c.island(at(&nav, 0)), c.island(at(&nav, 2)));
    assert_eq!(c.epoch(), nav.epoch());
}
#[test]
fn exhausted_update_is_atomic() {
    let mut nav = mesh(&[0]);
    let mut c = conn(&nav);
    let old = c.epoch();
    nav.add_tile(quad(1, 0, 0)).unwrap();
    assert!(!c.update(&nav, &mut Budget::new(2)).unwrap().is_complete());
    assert_eq!(c.epoch(), old);
    assert_eq!(c.island(at(&nav, 1)), None);
    assert!(matches!(
        reason(
            &nav,
            &c,
            pos(0),
            pos(1),
            &DefaultFilter::default(),
            64,
            u64::MAX
        ),
        Some(NoPathReason::MeshChanged)
    ));
}
#[test]
fn success_returns_complete_corridor() {
    let nav = mesh(&[0, 1, 2]);
    let c = conn(&nav);
    let r = diagnose_path(
        &mut NavMeshQuery::new(&nav, 64).unwrap(),
        &c,
        pos(0),
        pos(2),
        [1.0; 3],
        &DefaultFilter::default(),
        &mut Budget::unlimited(),
    )
    .unwrap();
    assert_eq!(r.reason, None);
    assert_eq!(r.path.len(), 3);
}
#[test]
fn start_off_mesh() {
    let nav = mesh(&[0]);
    assert!(matches!(
        reason(
            &nav,
            &conn(&nav),
            pos(5),
            pos(0),
            &DefaultFilter::default(),
            64,
            u64::MAX
        ),
        Some(NoPathReason::StartOffMesh { nearest: None })
    ));
}
#[test]
fn goal_off_mesh() {
    let nav = mesh(&[0]);
    assert!(matches!(
        reason(
            &nav,
            &conn(&nav),
            pos(0),
            pos(5),
            &DefaultFilter::default(),
            64,
            u64::MAX
        ),
        Some(NoPathReason::GoalOffMesh { nearest: None })
    ));
}
#[test]
fn start_excluded() {
    let mut nav = mesh(&[0, 1]);
    nav.set_poly_flags(at(&nav, 0), 0).unwrap();
    assert!(matches!(
        reason(
            &nav,
            &conn(&nav),
            pos(0),
            pos(1),
            &DefaultFilter::default(),
            64,
            u64::MAX
        ),
        Some(NoPathReason::StartExcluded { flags: 0, area: 0 })
    ));
}
#[test]
fn goal_excluded() {
    let mut nav = mesh(&[0, 1]);
    nav.set_poly_flags(at(&nav, 1), 0).unwrap();
    assert!(matches!(
        reason(
            &nav,
            &conn(&nav),
            pos(0),
            pos(1),
            &DefaultFilter::default(),
            64,
            u64::MAX
        ),
        Some(NoPathReason::GoalExcluded { flags: 0, area: 0 })
    ));
}
#[test]
fn disconnected_islands() {
    let nav = mesh(&[0, 2]);
    assert!(matches!(
        reason(
            &nav,
            &conn(&nav),
            pos(0),
            pos(2),
            &DefaultFilter::default(),
            64,
            u64::MAX
        ),
        Some(NoPathReason::DisconnectedIslands { .. })
    ));
}
#[test]
fn filter_cut_lists_excluded_corridor_polygon() {
    let mut nav = mesh(&[0, 1, 2]);
    let r = at(&nav, 1);
    nav.set_poly_flags(r, 0).unwrap();
    assert_eq!(
        reason(
            &nav,
            &conn(&nav),
            pos(0),
            pos(2),
            &DefaultFilter::default(),
            64,
            u64::MAX
        ),
        Some(NoPathReason::FilterCut {
            blocking: vec![(r, 0, 0)]
        })
    );
}
#[test]
fn unloaded_endpoint() {
    let mut nav = mesh(&[0, 1]);
    nav.unload_tile(at(&nav, 1)).unwrap();
    assert_eq!(
        reason(
            &nav,
            &conn(&nav),
            pos(0),
            pos(1),
            &DefaultFilter::default(),
            64,
            u64::MAX
        ),
        Some(NoPathReason::UnloadedTiles {
            frontier: vec![detour::TileKey { x: 1, y: 0 }]
        })
    );
}
#[test]
fn unloaded_frontier_precedes_disconnection() {
    let mut nav = mesh(&[0, 1, 2]);
    nav.unload_tile(at(&nav, 1)).unwrap();
    assert_eq!(
        reason(
            &nav,
            &conn(&nav),
            pos(0),
            pos(2),
            &DefaultFilter::default(),
            64,
            u64::MAX
        ),
        Some(NoPathReason::UnloadedTiles {
            frontier: vec![detour::TileKey { x: 1, y: 0 }]
        })
    );
}
#[test]
fn node_exhaustion_is_not_disconnection() {
    let nav = mesh(&[0, 1, 2]);
    assert_eq!(
        reason(
            &nav,
            &conn(&nav),
            pos(0),
            pos(2),
            &DefaultFilter::default(),
            1,
            u64::MAX
        ),
        Some(NoPathReason::NodePoolExhausted { max_nodes: 1 })
    );
}
#[test]
fn budget_exhaustion_is_not_disconnection() {
    let nav = mesh(&[0, 1]);
    assert_eq!(
        reason(
            &nav,
            &conn(&nav),
            pos(0),
            pos(1),
            &DefaultFilter::default(),
            64,
            0
        ),
        Some(NoPathReason::BudgetExhausted)
    );
    assert!(
        Connectivity::new(&nav, &mut Budget::new(0))
            .unwrap()
            .into_value()
            .is_none()
    );
}
#[test]
fn invalid_input_is_typed() {
    let nav = mesh(&[0]);
    let c = conn(&nav);
    assert!(matches!(
        diagnose_path(
            &mut NavMeshQuery::new(&nav, 64).unwrap(),
            &c,
            [f32::NAN; 3],
            pos(0),
            [1.0; 3],
            &DefaultFilter::default(),
            &mut Budget::unlimited()
        ),
        Err(Error::InvalidParameter(_))
    ));
    assert_eq!(
        c.reachable_from(&[0], &mut Budget::unlimited()),
        Err(Error::InvalidRef)
    );
}
#[test]
fn empty_statistics_have_zero_shares() {
    let c = conn(&mesh(&[]));
    let r = c
        .reachability(2, &mut Budget::unlimited())
        .unwrap()
        .into_value()
        .unwrap();
    assert_eq!((r.island_count, r.largest_island_share), (0, 0.0));
    assert_eq!(
        c.reachable_from(&[], &mut Budget::unlimited())
            .unwrap()
            .into_value()
            .unwrap()
            .share,
        0.0
    );
}

fn offmesh_nav() -> NavMesh {
    // Two disconnected ground quads in one tile, joined only left-to-right.
    const N: u16 = 0xffff;
    let polys = [
        0, 1, 2, 3, N, N, N, N, N, N, N, N, 4, 5, 6, 7, N, N, N, N, N, N, N, N,
    ];
    let bytes = detour::create_nav_mesh_data(&NavMeshCreateParams {
        verts: &[
            0, 0, 0, 0, 0, 10, 10, 0, 10, 10, 0, 0, 20, 0, 0, 20, 0, 10, 30, 0, 10, 30, 0, 0,
        ],
        vert_count: 8,
        polys: &polys,
        poly_flags: &[1, 1],
        poly_areas: &[0, 0],
        poly_count: 2,
        nvp: 6,
        off_mesh_con_verts: &[5.0, 0.0, 5.0, 25.0, 0.0, 5.0],
        off_mesh_con_rad: &[1.0],
        off_mesh_con_flags: &[1],
        off_mesh_con_areas: &[0],
        off_mesh_con_dir: &[0],
        off_mesh_con_user_id: &[1],
        off_mesh_con_count: 1,
        bmin: [0.0; 3],
        bmax: [30.0, 1.0, 10.0],
        walkable_height: 2.0,
        walkable_radius: 0.5,
        walkable_climb: 0.5,
        cs: 1.0,
        ch: 1.0,
        ..Default::default()
    })
    .unwrap();
    NavMesh::new_single(bytes).unwrap().0
}
#[test]
fn one_way_links_use_weak_labels_and_directed_coverage() {
    let nav = offmesh_nav();
    let c = conn(&nav);
    let base = at(&nav, 0);
    assert_eq!(c.island(base), c.island(base | 1));
    assert_eq!(
        c.reachable_from(&[base], &mut Budget::unlimited())
            .unwrap()
            .into_value()
            .unwrap()
            .reachable_polys,
        3
    );
    assert_eq!(
        c.reachable_from(&[base | 1], &mut Budget::unlimited())
            .unwrap()
            .into_value()
            .unwrap()
            .reachable_polys,
        1
    );
    assert_eq!(
        reason(
            &nav,
            &c,
            pos(2),
            pos(0),
            &DefaultFilter::default(),
            64,
            u64::MAX
        ),
        Some(NoPathReason::DirectedUnreachable)
    );
}
#[test]
fn edit_sequences_match_brute_flood_and_fresh_build() {
    let mut nav = mesh(&[]);
    let mut c = conn(&nav);
    let mut random = 7u32;
    for _ in 0..96 {
        random = random.wrapping_mul(1664525).wrapping_add(1013904223);
        let x = (random % 12) as i32;
        if let Some(slot) = nav.tile_index_at(x, 0, 0) {
            nav.remove_tile(nav.tile_ref(slot)).unwrap();
        } else {
            nav.add_tile(quad(x, 0, 0)).unwrap();
        }
        c.update(&nav, &mut Budget::unlimited()).unwrap();
        let fresh = conn(&nav);
        for slot in 0..nav.max_tiles() as usize {
            if nav.tile(slot).is_none() {
                continue;
            }
            let r = nav.tile_ref(slot);
            assert_eq!(c.island(r), fresh.island(r));
            let tile = nav.tile(slot).unwrap(); // brute contiguous resident columns
            let mut count = 1u32;
            for dir in [-1, 1] {
                let mut next = tile.header.x + dir;
                while nav.tile_index_at(next, 0, 0).is_some() {
                    count += 1;
                    next += dir;
                }
            }
            assert_eq!(
                c.reachable_from(&[r], &mut Budget::unlimited())
                    .unwrap()
                    .into_value()
                    .unwrap()
                    .reachable_polys,
                count
            );
        }
    }
}

fn floor_job<'a>(
    input: &'a navmesh_rs::bake::Input<'a>,
    cfg: &navmesh_rs::bake::BakeConfig,
) -> navmesh_rs::bake::BakeJob<'a, navmesh_rs::bake::Input<'a>> {
    use navmesh_rs::bake::{BakeJob, GridLayout, JobOptions};
    let grid = GridLayout::from_input(input, cfg).unwrap();
    let mut opts = JobOptions::default();
    opts.reports = true;
    BakeJob::new(input, cfg, &grid, grid.keys(), opts).unwrap()
}
#[test]
fn bake_report_matches_plain_bytes_and_coverage() {
    use navmesh_rs::bake::{self, BakeConfig, Input};
    let verts = [0., 0., 0., 0., 0., 20., 20., 0., 20., 20., 0., 0.];
    let tris = [0, 1, 2, 0, 2, 3];
    let input = Input::new(&verts, &tris);
    let cfg = BakeConfig::default();
    let plain = bake::tiles(&input, &cfg, &mut navmesh_rs::Workspace::new()).unwrap();
    let mut job = floor_job(&input, &cfg);
    job.run_parallel().unwrap();
    let r = &job.report().tiles[0];
    assert_eq!(r.schema_version, 1);
    assert_eq!(r.layers, 1);
    assert!(r.walkable_columns > 0 && r.walkable_columns <= r.total_columns);
    assert_eq!(r.polys, r.area_histogram.iter().sum::<u32>());
    assert_eq!(r.islands.iter().map(|s| s.polys).sum::<u32>(), r.polys);
    assert!(r.limits_hit.is_none());
    let output = job.next_ready().unwrap();
    assert_eq!(job.report().tiles[0].key, output.key);
    assert_eq!(output.data, plain.tiles[0].data);
}
#[test]
fn failed_bake_retains_limit_report() {
    use navmesh_rs::bake::{BakeConfig, Input};
    let verts = [0., 0., 0., 0., 0., 20., 20., 0., 20., 20., 0., 0.];
    let tris = [0, 1, 2, 0, 2, 3];
    let input = Input::new(&verts, &tris);
    let mut cfg = BakeConfig::default();
    cfg.tile_limits.max_work = 1;
    let mut job = floor_job(&input, &cfg);
    assert!(job.run_parallel().is_err());
    assert_eq!(job.report().tiles[0].limits_hit, Some(Error::LimitExceeded));
}

#[cfg(feature = "serde")]
#[test]
fn serialization_preserves_version_and_field_names() {
    let nav = mesh(&[0]);
    let c = conn(&nav);
    let r = c
        .reachability(2, &mut Budget::unlimited())
        .unwrap()
        .into_value()
        .unwrap();
    let value = serde_json::to_value(r).unwrap();
    assert_eq!(value["schema_version"], 1);
    assert_eq!(value["total_polys"], 1);
    assert_eq!(value["islands"][0]["xz_area"], 100.0);
    let report = navmesh_rs::bake::BakeReport::default();
    assert_eq!(
        serde_json::to_value(report).unwrap(),
        serde_json::json!({"schema_version":1,"tiles":[],"world":null})
    );
}

#[test]
fn excluded_nearest_does_not_hide_an_allowed_nearby_snap() {
    let mut nav = mesh(&[0, 1]);
    nav.set_poly_flags(at(&nav, 0), 0).unwrap();
    let c = conn(&nav);
    let diagnosis = diagnose_path(
        &mut NavMeshQuery::new(&nav, 64).unwrap(),
        &c,
        pos(0),
        pos(1),
        [11.0, 1.0, 1.0],
        &DefaultFilter::default(),
        &mut Budget::unlimited(),
    )
    .unwrap();
    assert_eq!(diagnosis.reason, None);
}
#[test]
fn topology_option_change_invalidates_snapshot() {
    let mut nav = mesh(&[0]);
    let c = conn(&nav);
    nav.set_global_off_mesh_links(true);
    assert!(!c.is_current(&nav));
    assert_eq!(
        reason(
            &nav,
            &c,
            pos(0),
            pos(0),
            &DefaultFilter::default(),
            64,
            u64::MAX
        ),
        Some(NoPathReason::MeshChanged)
    );
}

#[test]
fn small_regions_are_counted_when_the_output_is_empty() {
    use navmesh_rs::bake::{BakeConfig, Input, Partition};
    let verts = [0., 0., 0., 0., 0., 3., 3., 0., 3., 3., 0., 0.];
    let tris = [0, 1, 2, 0, 2, 3];
    let input = Input::new(&verts, &tris);
    for partition in [Partition::Watershed, Partition::Monotone, Partition::Layers] {
        let mut cfg = BakeConfig::default();
        cfg.partition = partition;
        cfg.recast.walkable_radius = 0;
        cfg.recast.min_region_area = 10_000;
        let mut job = floor_job(&input, &cfg);
        job.run_parallel().unwrap();
        let r = &job.report().tiles[0];
        assert_eq!(r.layers, 0);
        assert_eq!(r.walkable_columns, 0);
        assert_eq!(r.regions_removed_small, 1);
        assert!(r.area_histogram.iter().all(|n| *n == 0));
    }
}

#[test]
fn stepped_and_parallel_reports_agree() {
    use navmesh_rs::bake::{BakeConfig, Input};
    let verts = [0., 0., 0., 0., 0., 20., 20., 0., 20., 20., 0., 0.];
    let tris = [0, 1, 2, 0, 2, 3];
    let input = Input::new(&verts, &tris);
    let cfg = BakeConfig::default();
    let mut serial = floor_job(&input, &cfg);
    while serial.step(&mut Budget::new(1)).unwrap() != navmesh_rs::Progress::Done {}
    let mut parallel = floor_job(&input, &cfg);
    parallel.run_parallel().unwrap();
    assert_eq!(serial.report(), parallel.report());
}

#[test]
fn filter_cut_is_sorted_and_explicitly_capped() {
    let mut nav = NavMesh::new(&NavMeshParams {
        orig: [0.0; 3],
        tile_width: 10.0,
        tile_height: 10.0,
        max_tiles: 128,
        max_polys: 64,
    })
    .unwrap();
    for x in 0..72 {
        let r = nav.add_tile(quad(x, 0, 0)).unwrap();
        if (1..71).contains(&x) {
            nav.set_poly_flags(r, 0).unwrap();
        }
    }
    let c = conn(&nav);
    let result = diagnose_path(
        &mut NavMeshQuery::new(&nav, 512).unwrap(),
        &c,
        pos(0),
        pos(71),
        [1.0; 3],
        &DefaultFilter::default(),
        &mut Budget::unlimited(),
    )
    .unwrap();
    let Some(NoPathReason::FilterCut { blocking }) = result.reason else {
        panic!("expected filter cut");
    };
    assert_eq!(blocking.len(), 64);
    assert!(result.blocking_truncated);
    assert!(blocking.windows(2).all(|w| w[0].0 < w[1].0));
}

#[test]
fn every_refused_update_preserves_old_labels() {
    let mut nav = mesh(&[0]);
    let original = conn(&nav);
    nav.add_tile(quad(1, 0, 0)).unwrap();
    let mut updated = original.clone();
    let mut budget = Budget::unlimited();
    updated.update(&nav, &mut budget).unwrap();
    for allowance in 0..budget.used() {
        let mut c = original.clone();
        let mut b = Budget::new(allowance);
        assert!(!c.update(&nav, &mut b).unwrap().is_complete());
        assert!(b.used() <= allowance);
        assert_eq!(c.epoch(), original.epoch());
        assert_eq!(c.island(at(&nav, 0)), original.island(at(&nav, 0)));
        assert_eq!(c.island(at(&nav, 1)), None);
    }
}

#[test]
fn snapshots_reject_other_meshes_with_matching_epochs_and_refs() {
    let a = mesh(&[0, 2]);
    let c = conn(&a);
    let b = mesh(&[0, 1]);
    assert_eq!(a.epoch(), b.epoch());
    assert!(!c.is_current(&b));
    assert_eq!(
        reason(
            &b,
            &c,
            pos(0),
            pos(1),
            &DefaultFilter::default(),
            64,
            u64::MAX
        ),
        Some(NoPathReason::MeshChanged)
    );
    let copy = a.clone();
    assert!(!c.is_current(&copy));
    let moved = a;
    assert!(c.is_current(&moved));
    let mut update = c.clone();
    assert_eq!(
        update.update(&b, &mut Budget::unlimited()),
        Err(Error::MeshChanged)
    );
}
