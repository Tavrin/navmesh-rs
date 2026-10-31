#![cfg(all(feature = "recast", feature = "detour"))]

use navmesh_rs::bake::{self, AreaVolume, BakeConfig, Input, Partition};
use navmesh_rs::detour::{DefaultFilter, NavMeshQuery};
use navmesh_rs::{Error, Limits, Workspace};

fn floor() -> (Vec<f32>, Vec<i32>) {
    (
        vec![0., 0., 0., 0., 0., 24., 24., 0., 24., 24., 0., 0.],
        vec![0, 1, 2, 0, 2, 3],
    )
}

#[test]
fn partitions_areas_links_and_empty_tiles_are_identical() {
    let (verts, tris) = floor();
    let mut volume = AreaVolume::default();
    let volume_verts = [2., 0., 2., 2., 0., 7., 7., 0., 7., 7., 0., 2.];
    volume.verts = &volume_verts;
    volume.hmin = -1.;
    volume.hmax = 2.;
    volume.area = 3;
    let volumes = [volume];
    let mut input = Input::new(&verts, &tris);
    input.volumes = &volumes;
    input.areas = Some(&[63, 63]);
    let endpoints = [3., 0., 3., 18., 0., 18.];
    input.off_mesh.off_mesh_con_count = 1;
    input.off_mesh.off_mesh_con_verts = &endpoints;
    input.off_mesh.off_mesh_con_rad = &[0.5];
    input.off_mesh.off_mesh_con_flags = &[1];
    input.off_mesh.off_mesh_con_areas = &[0];
    input.off_mesh.off_mesh_con_dir = &[1];
    input.off_mesh.off_mesh_con_user_id = &[123];
    for partition in [Partition::Watershed, Partition::Monotone, Partition::Layers] {
        let mut cfg = BakeConfig::default();
        cfg.partition = partition;
        let serial = bake::tiles(&input, &cfg, &mut Workspace::new()).unwrap();
        let parallel = bake::tiles_parallel(&input, &cfg, &mut Workspace::new()).unwrap();
        assert_eq!(serial.tiles, parallel.tiles);
        assert!(serial.tiles.iter().any(|t| t.data.is_some()));
        let nav = parallel.into_navmesh(Limits::default()).unwrap();
        let query = NavMeshQuery::new(&nav, 128).unwrap();
        let (poly, _, _) = query
            .find_nearest_poly(&[3., 0., 3.], &[2., 4., 2.], &DefaultFilter::default())
            .unwrap();
        assert_ne!(poly, 0);
    }
    input.areas = Some(&[0, 0]);
    let empty =
        bake::tiles_parallel(&input, &BakeConfig::default(), &mut Workspace::new()).unwrap();
    assert!(empty.tiles.iter().all(|t| t.data.is_none()));
}

#[test]
fn errors_identify_first_tile_in_every_pool() {
    let (verts, tris) = floor();
    let input = Input::new(&verts, &tris);
    let mut cfg = BakeConfig::default();
    cfg.tile_limits.max_work = 1;
    let expected = bake::tiles(&input, &cfg, &mut Workspace::new()).unwrap_err();
    assert_eq!(expected.tile, Some([0, 0]));
    assert_eq!(expected.error, Error::LimitExceeded);
    #[cfg(feature = "parallel")]
    for n in [1, 2, 4, 8, 16] {
        let pool = rayon::ThreadPoolBuilder::new()
            .num_threads(n)
            .build()
            .unwrap();
        assert_eq!(
            pool.install(|| bake::tiles_parallel(&input, &cfg, &mut Workspace::new()))
                .unwrap_err(),
            expected
        );
    }
    #[cfg(not(feature = "parallel"))]
    assert_eq!(
        bake::tiles_parallel(&input, &cfg, &mut Workspace::new()).unwrap_err(),
        expected
    );
}

#[test]
fn cumulative_tile_budget_and_format_limits() {
    let (verts, tris) = floor();
    let input = Input::new(&verts, &tris);
    let mut cfg = BakeConfig::default();
    cfg.tile_limits.max_bytes = 1;
    assert_eq!(
        bake::tiles_parallel(&input, &cfg, &mut Workspace::new())
            .unwrap_err()
            .error,
        Error::LimitExceeded
    );
    cfg.tile_limits = Limits::default();
    cfg.tile_limits.max_tile_polys = 0;
    assert_eq!(
        bake::tiles(&input, &cfg, &mut Workspace::new())
            .unwrap_err()
            .error,
        Error::LimitExceeded
    );
    cfg.tile_limits = Limits::default();
    cfg.tile_limits.max_tile_bytes = 1;
    assert_eq!(
        bake::tiles_parallel(&input, &cfg, &mut Workspace::new())
            .unwrap_err()
            .error,
        Error::LimitExceeded
    );
    let good = bake::tiles(&input, &BakeConfig::default(), &mut Workspace::new()).unwrap();
    assert!(good.tiles.iter().any(|t| t.data.is_some()));
}

#[test]
fn setup_budget_precedes_allocation_and_retries_cleanly() {
    let (verts, tris) = floor();
    let input = Input::new(&verts, &tris);
    let mut limits = Limits {
        max_work: 0,
        ..Limits::default()
    };
    let mut ws = Workspace::with_limits(limits);
    let err = bake::tiles_parallel(&input, &BakeConfig::default(), &mut ws).unwrap_err();
    assert_eq!(err.tile, None);
    assert_eq!(err.error, Error::LimitExceeded);
    assert_eq!(ws.usage().bytes, 0);
    ws.set_limits(Limits::default());
    assert!(bake::tiles_parallel(&input, &BakeConfig::default(), &mut ws).is_ok());
    limits.max_work = u64::MAX;
    limits.max_bytes = 0;
    ws.set_limits(limits);
    assert_eq!(
        bake::tiles(&input, &BakeConfig::default(), &mut ws)
            .unwrap_err()
            .error,
        Error::LimitExceeded
    );
}

#[test]
fn invalid_input_and_extreme_config_are_rejected_before_tiles() {
    let (verts, tris) = floor();
    let mut input = Input::new(&verts, &[0, -1, 2]);
    assert!(bake::tiles_parallel(&input, &BakeConfig::default(), &mut Workspace::new()).is_err());
    input = Input::new(&verts, &tris);
    input.areas = Some(&[64, 0]);
    assert!(bake::tiles_parallel(&input, &BakeConfig::default(), &mut Workspace::new()).is_err());
    input = Input::new(&verts, &tris);
    input.off_mesh.off_mesh_con_count = i32::MAX;
    assert!(bake::tiles_parallel(&input, &BakeConfig::default(), &mut Workspace::new()).is_err());
    input = Input::new(&verts, &tris);
    for value in [-1, 0, i32::MAX] {
        let mut cfg = BakeConfig::default();
        cfg.recast.tile_size = value;
        assert!(bake::tiles_parallel(&input, &cfg, &mut Workspace::new()).is_err());
    }
    for value in [f32::NAN, f32::INFINITY, -1., 0., f32::MIN_POSITIVE] {
        let mut cfg = BakeConfig::default();
        cfg.recast.cs = value;
        assert!(bake::tiles_parallel(&input, &cfg, &mut Workspace::new()).is_err());
    }
}

#[test]
fn one_tile_layout_assembles() {
    let verts = [0., 0., 0., 0., 0., 8., 8., 0., 8., 8., 0., 0.];
    let input = Input::new(&verts, &[0, 1, 2, 0, 2, 3]);
    bake::tiles_parallel(&input, &BakeConfig::default(), &mut Workspace::new())
        .unwrap()
        .into_navmesh(Limits::default())
        .unwrap();
}

#[test]
fn aggregate_failure_is_setup_error_and_retry_preserves_identity() {
    let (verts, tris) = floor();
    let input = Input::new(&verts, &tris);
    let baseline = bake::tiles(&input, &BakeConfig::default(), &mut Workspace::new()).unwrap();
    let mut c = BakeConfig::default();
    c.tile_limits.max_bytes = 16 * 1024 * 1024;
    c.tile_limits.max_tile_bytes = 1024 * 1024;
    c.max_total_bytes = c.tile_limits.max_bytes; // no room for retained tiles
    let mut ws = Workspace::new();
    for parallel in [false, true] {
        let err = if parallel {
            bake::tiles_parallel(&input, &c, &mut ws)
        } else {
            bake::tiles(&input, &c, &mut ws)
        }
        .unwrap_err();
        assert_eq!(err.tile, None);
        assert_eq!(err.error, Error::LimitExceeded);
    }
    c.max_total_bytes = 1024 * 1024 * 1024;
    assert_eq!(
        bake::tiles(&input, &c, &mut ws).unwrap().tiles,
        baseline.tiles
    );
    #[cfg(feature = "parallel")]
    for n in [1, 2, 4, 8, 16] {
        let pool = rayon::ThreadPoolBuilder::new()
            .num_threads(n)
            .build()
            .unwrap();
        assert_eq!(
            pool.install(|| bake::tiles_parallel(&input, &c, &mut ws))
                .unwrap()
                .tiles,
            baseline.tiles
        );
    }
}
