#![cfg(feature = "tilecache")]

use navmesh_rs::bake::{
    AreaVolume, BakeConfig, BakeJob, GridLayout, Input, JobOptions, TileInstaller, TileKey,
    tile_cache_layers,
};
use navmesh_rs::detour::{DefaultFilter, NavMesh, NavMeshQuery, TileRef};
use navmesh_rs::tilecache::{
    DefaultMeshProcess, LayerHeader, ObstacleShape, RawCompressor, TileCache, TileCacheParams,
};
use navmesh_rs::{Budget, Error, Limits, Progress};

const FLOOR: [f32; 12] = [0., 0., 0., 0., 0., 9.6, 38.4, 0., 9.6, 38.4, 0., 0.];
const TRIANGLES: [i32; 6] = [0, 1, 2, 0, 2, 3];
const ZONE: [TileKey; 2] = [TileKey { x: 1, z: 0 }, TileKey { x: 2, z: 0 }];

fn setup(volumes: &[AreaVolume<'_>]) -> (TileCache, NavMesh) {
    let mut input = Input::new(&FLOOR, &TRIANGLES);
    input.volumes = volumes;
    let cfg = BakeConfig::default();
    let grid = GridLayout::from_input(&input, &cfg).unwrap();
    let mut nav = NavMesh::new(&grid.nav_mesh_params(8, 256)).unwrap();

    let normal = grid
        .keys()
        .into_iter()
        .filter(|key| !ZONE.contains(key))
        .collect();
    let mut job = BakeJob::new(&input, &cfg, &grid, normal, JobOptions::default()).unwrap();
    assert!(job.step(&mut Budget::unlimited()).unwrap().is_done());
    let mut installer = TileInstaller::new();
    while let Some(tile) = job.next_ready() {
        installer.push(tile);
    }
    assert!(
        installer
            .install(&mut nav, &mut Budget::unlimited())
            .unwrap()
            .is_done()
    );

    let mut params = TileCacheParams::new(32, 32, cfg.recast.cs, cfg.recast.ch, 8, 8);
    params.orig = grid.origin;
    params.walkable_climb = cfg.walkable_climb;
    let mut cache =
        TileCache::new(params, RawCompressor, DefaultMeshProcess, Limits::default()).unwrap();
    for key in ZONE {
        let mut layers = tile_cache_layers(&input, &cfg, &grid, key, &RawCompressor).unwrap();
        assert_eq!(layers.len(), 1);
        cache.load_tile(layers.pop().unwrap()).unwrap();
    }
    assert!(
        cache
            .update_budgeted(&mut nav, &mut Budget::new(2))
            .unwrap()
            .is_done()
    );
    (cache, nav)
}

fn resident(nav: &NavMesh, x: i32) -> TileRef {
    nav.tile_ref(nav.tile_index_at(x, 0, 0).unwrap())
}

fn aabb(x: f32) -> ObstacleShape {
    ObstacleShape::Box {
        bmin: [x - 0.6, -0.5, 3.5],
        bmax: [x + 0.6, 1.5, 5.5],
    }
}

#[test]
fn polygon_and_cache_seams_connect_both_ways() {
    let (_, nav) = setup(&[]);
    let mut query = NavMeshQuery::new(&nav, 64).unwrap();
    let filter = DefaultFilter::default();
    let left = [4., 0.2, 4.8];
    let right = [34., 0.2, 4.8];
    for (start, end) in [(left, right), (right, left)] {
        let (start_ref, ..) = query.find_nearest_poly(&start, &[0.5; 3], &filter).unwrap();
        let (end_ref, ..) = query.find_nearest_poly(&end, &[0.5; 3], &filter).unwrap();
        let mut path = Vec::new();
        query
            .find_path(start_ref, end_ref, &start, &end, &filter, &mut path, 16)
            .unwrap();
        assert_eq!(path.last(), Some(&end_ref));
        assert_eq!(path.len(), 4);
    }
    for (x, left_tile, right_tile) in [(9.6, 0, 1), (28.8, 2, 3)] {
        let seam = [x, 0.2, 4.8];
        let sample = |tile| {
            let (point, _) = query
                .closest_point_on_poly(resident(&nav, tile), &seam)
                .unwrap();
            assert!((point[0] - x).abs() < 1e-5);
            point[1]
        };
        let left_y = sample(left_tile);
        let right_y = sample(right_tile);
        assert!((left_y - right_y).abs() < 1e-5, "{left_y} vs {right_y}");
    }
}

#[test]
fn movement_rebuilds_only_old_and_new_bounds_with_one_layer_per_unit() {
    let (mut cache, mut nav) = setup(&[]);
    let original = [0, 1, 2, 3].map(|x| resident(&nav, x));
    let old = cache.add_obstacle(aabb(14.)).unwrap();
    let mut zero = Budget::new(0);
    assert_eq!(
        cache.update_budgeted(&mut nav, &mut zero).unwrap(),
        Progress::Exhausted
    );
    assert_eq!([0, 1, 2, 3].map(|x| resident(&nav, x)), original);

    let mut one = Budget::new(1);
    cache.set_limits(Limits {
        max_work: 18,
        ..Limits::default()
    });
    assert_eq!(
        cache.update_budgeted(&mut nav, &mut one),
        Err(Error::LimitExceeded)
    );
    assert_eq!([0, 1, 2, 3].map(|x| resident(&nav, x)), original);
    cache.set_limits(Limits::default());
    one.reset(1);
    assert!(cache.update_budgeted(&mut nav, &mut one).unwrap().is_done());
    assert_eq!(one.used(), 1);
    let carved = resident(&nav, 1);
    assert_ne!(carved, original[1]);
    assert_eq!(resident(&nav, 2), original[2]);

    cache.remove_obstacle(old).unwrap();
    let new = cache
        .add_obstacle(ObstacleShape::OrientedBox {
            center: [23., 0.5, 4.8],
            half_extents: [1.2, 1., 0.7],
            yaw: 0.6,
        })
        .unwrap();
    one.reset(1);
    assert_eq!(
        cache.update_budgeted(&mut nav, &mut one).unwrap(),
        Progress::Exhausted
    );
    assert_eq!(one.used(), 1);
    assert_ne!(resident(&nav, 1), carved);
    assert_eq!(resident(&nav, 2), original[2]);

    one.reset(1);
    assert!(cache.update_budgeted(&mut nav, &mut one).unwrap().is_done());
    assert_eq!(one.used(), 1);
    assert_ne!(resident(&nav, 2), original[2]);
    assert_eq!(
        cache.obstacle(new).unwrap().state(),
        navmesh_rs::tilecache::ObstacleState::Processed
    );
    for x in [0, 3] {
        assert_eq!(resident(&nav, x), original[x as usize]);
        assert!(nav.is_valid_poly_ref(original[x as usize]));
    }
}

#[test]
fn obstacle_order_and_slicing_preserve_rebuilt_bytes() {
    let (mut legacy, mut legacy_nav) = setup(&[]);
    let (mut budgeted, mut budgeted_nav) = setup(&[]);
    let shapes = [aabb(14.), aabb(19.3), aabb(24.)];
    for shape in shapes {
        legacy.add_obstacle(shape).unwrap();
    }
    for shape in shapes.into_iter().rev() {
        budgeted.add_obstacle(shape).unwrap();
    }
    legacy.set_keep_identical_tiles(true);
    assert!(!legacy.update(0., &mut legacy_nav).unwrap());
    assert!(legacy.update(0., &mut legacy_nav).unwrap());
    let mut two = Budget::new(2);
    assert!(
        budgeted
            .update_budgeted(&mut budgeted_nav, &mut two)
            .unwrap()
            .is_done()
    );
    assert_eq!(two.used(), 2);
    for key in ZONE {
        let data = |nav: &NavMesh| {
            nav.tile(nav.tile_index_at(key.x, key.z, 0).unwrap())
                .unwrap()
                .data
                .clone()
        };
        assert_eq!(data(&legacy_nav), data(&budgeted_nav));
    }
    let before = resident(&budgeted_nav, 1);
    let epoch = budgeted_nav.epoch();
    budgeted.add_obstacle(shapes[0]).unwrap();
    assert!(
        budgeted
            .update_budgeted(&mut budgeted_nav, &mut Budget::new(1))
            .unwrap()
            .is_done()
    );
    assert_eq!(resident(&budgeted_nav, 1), before);
    assert_eq!(budgeted_nav.epoch(), epoch);
}

#[test]
fn zone_grid_checks_the_border_before_baking() {
    let input = Input::new(&FLOOR, &TRIANGLES);
    let mut cfg = BakeConfig::default();
    cfg.recast.tile_size = 245;
    let grid = GridLayout::new([0.; 3], 0., 1, 1, &cfg).unwrap();
    let key = TileKey { x: 0, z: 0 };
    let layers = tile_cache_layers(&input, &cfg, &grid, key, &RawCompressor).unwrap();
    assert_eq!(LayerHeader::decode(&layers[0]).unwrap().width, 245);

    cfg.recast.tile_size += 1;
    let grid = GridLayout::new([0.; 3], 0., 1, 1, &cfg).unwrap();
    let err = tile_cache_layers(&input, &cfg, &grid, key, &RawCompressor).unwrap_err();
    assert_eq!(err.error, Error::FormatLimit("bordered tile-cache grid"));
    assert_eq!(err.tile, Some([0, 0]));
}

#[cfg(feature = "dynamic")]
#[test]
fn registered_gate_survives_carve_and_restore() {
    use navmesh_rs::dynamic::{DynamicLayer, Effect, ModifierId, ModifierSpec, Selector};

    let points = [11., 0., 2., 16., 0., 2., 16., 0., 7., 11., 0., 7.];
    let mut volume = AreaVolume::default();
    volume.verts = &points;
    volume.hmin = -1.;
    volume.hmax = 2.;
    volume.area = 4;
    let (mut cache, mut nav) = setup(&[volume]);
    let mut layer = DynamicLayer::new(1, Limits::default()).unwrap();
    let gate = ModifierId(0);
    layer
        .upsert(ModifierSpec::new(
            gate,
            Selector::Prism {
                verts_xz: vec![[10., 1.], [17., 1.], [17., 8.], [10., 8.]],
                ymin: -1.,
                ymax: 2.,
                area: Some(4),
            },
            Effect::ClearFlags(u16::MAX),
        ))
        .unwrap();
    layer.set_enabled(gate, true).unwrap();
    nav.set_dynamic_layer(Some(layer));
    let check = |nav: &NavMesh, closed| {
        let ti = nav.tile_index_at(1, 0, 0).unwrap();
        let polys = &nav.tile(ti).unwrap().polys;
        let gate: Vec<_> = polys.iter().filter(|p| p.area() == 4).collect();
        assert!(!gate.is_empty());
        assert!(gate.iter().all(|p| (p.flags == 0) == closed));
    };
    check(&nav, true);

    let mut last = resident(&nav, 1);
    let obstacle = cache.add_obstacle(aabb(14.)).unwrap();
    for remove in [false, true] {
        if remove {
            cache.remove_obstacle(obstacle).unwrap();
        }
        assert!(
            cache
                .update_budgeted(&mut nav, &mut Budget::new(1))
                .unwrap()
                .is_done()
        );
        let rebuilt = resident(&nav, 1);
        assert_ne!(last, rebuilt);
        last = rebuilt;
        check(&nav, true);
    }
    let full = cache
        .add_obstacle(ObstacleShape::Box {
            bmin: [9.601, -0.5, 0.],
            bmax: [19.199, 1.5, 9.6],
        })
        .unwrap();
    assert!(
        cache
            .update_budgeted(&mut nav, &mut Budget::new(1))
            .unwrap()
            .is_done()
    );
    assert!(nav.tile_index_at(1, 0, 0).is_none());
    cache.remove_obstacle(full).unwrap();
    assert!(
        cache
            .update_budgeted(&mut nav, &mut Budget::new(1))
            .unwrap()
            .is_done()
    );
    check(&nav, true);
    nav.set_modifier_enabled(gate, false).unwrap();
    check(&nav, false);
}
