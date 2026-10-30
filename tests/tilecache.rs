#![cfg(feature = "tilecache")]
use navmesh_rs::detour::{DefaultFilter, NavMesh, NavMeshParams, NavMeshQuery};
use navmesh_rs::tilecache::*;
use navmesh_rs::{Error, Limits, Workspace};
fn header(x: i32) -> LayerHeader {
    let mut h = LayerHeader::new(8, 8);
    h.tx = x;
    h.bmin = [x as f32 * 8.0, 0.0, 0.0];
    h.bmax = [x as f32 * 8.0 + 8.0, 4.0, 8.0];
    h.hmax = 20;
    h
}
fn data(x: i32) -> Vec<u8> {
    let mut cons = vec![0; 64];
    for y in 0..8 {
        for x in 0..8 {
            cons[x + y * 8] = u8::from(x > 0)
                | (u8::from(y < 7) * 2)
                | (u8::from(x < 7) * 4)
                | (u8::from(y > 0) * 8);
        }
    }
    build_layer(
        &mut Workspace::new(),
        &RawCompressor,
        &header(x),
        &[1; 64],
        &[63; 64],
        &cons,
    )
    .unwrap()
}
fn params(n: usize) -> TileCacheParams {
    TileCacheParams::new(8, 8, 1.0, 0.2, n, n.max(8))
}
fn nav(n: usize) -> NavMesh {
    NavMesh::new(&NavMeshParams {
        orig: [0.0; 3],
        tile_width: 8.0,
        tile_height: 8.0,
        max_tiles: i32::try_from(n.next_power_of_two()).unwrap(),
        max_polys: 256,
    })
    .unwrap()
}
fn cylinder(x: f32) -> ObstacleShape {
    ObstacleShape::Cylinder {
        pos: [x, 0.0, 4.0],
        radius: 1.0,
        height: 3.0,
    }
}
fn drain(c: &mut TileCache, n: &mut NavMesh) {
    for _ in 0..1024 {
        if c.update(0.1, n).unwrap() {
            return;
        }
    }
    panic!("updates did not drain");
}
#[test]
fn layer_roundtrip_and_malformed_bytes() {
    let good = data(0);
    assert_eq!(good.len(), 56 + 64 * 3);
    assert_eq!(&good[54..56], &[0, 0]);
    let mut ws = Workspace::new();
    let l = decompress_layer(&mut ws, &RawCompressor, &good).unwrap();
    assert_eq!(l.header(), &header(0));
    assert_eq!(l.heights(), &[1; 64]);
    for end in 0..good.len() {
        assert!(decompress_layer(&mut ws, &RawCompressor, &good[..end]).is_err());
    }
    for (offset, value) in [
        (0, 0),
        (4, 2),
        (48, 0),
        (50, 9),
        (51, 8),
        (56 + 64, 64),
        (56 + 128, 1),
    ] {
        let mut b = good.clone();
        b[offset] = value;
        assert!(
            decompress_layer(&mut ws, &RawCompressor, &b).is_err(),
            "offset {offset}"
        );
    }
    let mut extra = good;
    extra.push(0);
    assert!(decompress_layer(&mut ws, &RawCompressor, &extra).is_err());
}
#[test]
fn every_shape_rebuilds_and_restores_queries() {
    for shape in [
        cylinder(4.0),
        ObstacleShape::Box {
            bmin: [2.0, 0.0, 2.0],
            bmax: [6.0, 3.0, 6.0],
        },
        ObstacleShape::OrientedBox {
            center: [4.0, 1.0, 4.0],
            half_extents: [2.0, 1.5, 1.0],
            yaw: 0.7,
        },
    ] {
        let mut c = TileCache::new(
            params(2),
            RawCompressor,
            DefaultMeshProcess,
            Limits::default(),
        )
        .unwrap();
        let r = c.add_tile(data(0)).unwrap();
        let original = c.build_tile_data(r).unwrap().unwrap();
        let mut n = nav(2);
        c.build_nav_mesh_tile(r, &mut n).unwrap();
        let o = c.add_obstacle(shape).unwrap();
        drain(&mut c, &mut n);
        assert_eq!(c.obstacle(o).unwrap().state(), ObstacleState::Processed);
        let changed = c.build_tile_data(r).unwrap().unwrap();
        assert_ne!(original, changed);
        let q = NavMeshQuery::new(&n, 128).unwrap();
        let (_, p, _) = q
            .find_nearest_poly(
                &[4.0, 0.2, 4.0],
                &[8.0, 8.0, 8.0],
                &DefaultFilter::default(),
            )
            .unwrap();
        assert!(p[0] != 4.0 || p[2] != 4.0);
        c.remove_obstacle(o).unwrap();
        drain(&mut c, &mut n);
        assert!(c.obstacle(o).is_none());
        assert_eq!(c.build_tile_data(r).unwrap().unwrap(), original);
    }
}
#[test]
fn obstacle_slot_reuse_resets_all_state() {
    let mut c = TileCache::new(
        params(2),
        RawCompressor,
        DefaultMeshProcess,
        Limits::default(),
    )
    .unwrap();
    c.add_tile(data(0)).unwrap();
    let mut n = nav(2);
    let first = c.add_obstacle(cylinder(4.0)).unwrap();
    c.remove_obstacle(first).unwrap();
    drain(&mut c, &mut n);
    assert!(c.obstacle(first).is_none());
    let second = c.add_obstacle(cylinder(100.0)).unwrap();
    assert_ne!(first, second);
    assert_eq!(first.bits() & 65535, second.bits() & 65535);
    drain(&mut c, &mut n);
    assert_eq!(
        c.obstacle(second).unwrap().state(),
        ObstacleState::Processed
    );
    assert_eq!(c.obstacle(second).unwrap().shape(), cylinder(100.0));
    assert_eq!(c.remove_obstacle(first), Err(Error::InvalidRef));
    c.remove_obstacle(second).unwrap();
    drain(&mut c, &mut n);
    assert!(c.obstacle(second).is_none());
}
#[test]
fn tile_slot_reuse_invalidates_old_handle() {
    let mut c = TileCache::new(
        params(1),
        RawCompressor,
        DefaultMeshProcess,
        Limits::default(),
    )
    .unwrap();
    let a = c.add_tile(data(0)).unwrap();
    c.remove_tile(a).unwrap();
    let b = c.add_tile(data(1)).unwrap();
    assert_ne!(a, b);
    assert!(c.tile_header(a).is_none());
    assert_eq!(c.tile_header(b).unwrap().tx, 1);
    assert!(c.build_tile_data(a).is_err());
}
#[test]
fn invalid_shapes_capacities_and_handles_are_errors() {
    let mut p = params(1);
    p.max_obstacles = 1;
    let mut c = TileCache::new(p, RawCompressor, DefaultMeshProcess, Limits::default()).unwrap();
    c.add_tile(data(0)).unwrap();
    assert_eq!(c.add_tile(data(0)), Err(Error::TileSlotOccupied));
    assert_eq!(c.add_tile(data(1)), Err(Error::NoFreeTile));
    for s in [
        cylinder(f32::NAN),
        ObstacleShape::Cylinder {
            pos: [0.0; 3],
            radius: -1.0,
            height: 1.0,
        },
        ObstacleShape::Box {
            bmin: [1.0; 3],
            bmax: [0.0; 3],
        },
        ObstacleShape::OrientedBox {
            center: [0.0; 3],
            half_extents: [1.0; 3],
            yaw: f32::INFINITY,
        },
    ] {
        assert!(c.add_obstacle(s).is_err());
    }
    let r = c.add_obstacle(cylinder(4.0)).unwrap();
    assert_eq!(c.add_obstacle(cylinder(5.0)), Err(Error::BufferTooSmall));
    c.remove_obstacle(r).unwrap();
    assert!(c.remove_obstacle(r).is_err());
    assert!(c.update(f32::NAN, &mut nav(1)).is_err());
}
#[test]
fn touching_more_than_eight_layers_is_atomic_error() {
    let mut c = TileCache::new(
        params(16),
        RawCompressor,
        DefaultMeshProcess,
        Limits::default(),
    )
    .unwrap();
    for x in 0..10 {
        c.add_tile(data(x)).unwrap();
    }
    let s = ObstacleShape::Box {
        bmin: [0.0, 0.0, 0.0],
        bmax: [80.0, 4.0, 8.0],
    };
    assert_eq!(c.add_obstacle(s), Err(Error::BufferTooSmall));
    assert!(c.obstacle_at(0).is_none());
    let r = c.add_obstacle(cylinder(4.0)).unwrap();
    assert_eq!(r.bits(), 65536);
}
#[test]
fn request_pool_exhaustion_is_atomic() {
    let mut c = TileCache::new(
        params(128),
        RawCompressor,
        DefaultMeshProcess,
        Limits::default(),
    )
    .unwrap();
    for _ in 0..64 {
        c.add_obstacle(cylinder(100.0)).unwrap();
    }
    assert_eq!(c.add_obstacle(cylinder(100.0)), Err(Error::BufferTooSmall));
    assert!(c.obstacle_at(64).is_none());
    drain(&mut c, &mut nav(128));
    assert!(c.add_obstacle(cylinder(100.0)).is_ok());
}
#[test]
fn budget_precedes_allocations_and_rebuild_and_can_retry() {
    let mut lim = Limits {
        max_bytes: 1,
        ..Limits::default()
    };
    assert!(matches!(
        TileCache::new(params(4), RawCompressor, DefaultMeshProcess, lim),
        Err(Error::LimitExceeded)
    ));
    let mut ws = Workspace::with_limits(lim);
    assert!(matches!(
        decompress_layer(&mut ws, &RawCompressor, &data(0)),
        Err(Error::LimitExceeded)
    ));
    let mut c = TileCache::new(
        params(2),
        RawCompressor,
        DefaultMeshProcess,
        Limits::default(),
    )
    .unwrap();
    let r = c.add_tile(data(0)).unwrap();
    let mut n = nav(2);
    c.build_nav_mesh_tile(r, &mut n).unwrap();
    let before = n.tile(0).unwrap().data.clone();
    c.add_obstacle(cylinder(4.0)).unwrap();
    lim = Limits {
        max_work: 30,
        ..Limits::default()
    };
    c.set_limits(lim);
    assert_eq!(c.update(0.1, &mut n), Err(Error::LimitExceeded));
    assert_eq!(n.tile(0).unwrap().data, before);
    c.set_limits(Limits::default());
    drain(&mut c, &mut n);
}
#[test]
fn large_contour_is_rejected_before_triangulation_or_allocation() {
    let mut contour = LayerContour::new();
    contour.area = 63;
    // A simple 1,020-vertex square with collinear boundary vertices. Its
    // conservative cubic charge exceeds one billion work units.
    for i in 0..255u8 {
        contour.verts.extend_from_slice(&[0, 1, i, 15]);
    }
    for i in 0..255u8 {
        contour.verts.extend_from_slice(&[i, 1, 255, 15]);
    }
    for i in (1..=255u8).rev() {
        contour.verts.extend_from_slice(&[255, 1, i, 15]);
    }
    for i in (1..=255u8).rev() {
        contour.verts.extend_from_slice(&[i, 1, 0, 15]);
    }
    let mut set = LayerContourSet::new();
    set.contours.push(contour);
    let mut ws = Workspace::with_limits(Limits {
        max_work: 1_000_000,
        ..Limits::default()
    });
    assert!(matches!(
        build_poly_mesh(&mut ws, &set),
        Err(Error::LimitExceeded)
    ));
    assert_eq!(ws.usage().work, 1 + 1020 * 4 + 1020u64.pow(3));
    assert_eq!(ws.usage().bytes, 0);
}
#[test]
fn empty_contour_entries_are_charged_before_each_list_traversal() {
    let mut set = LayerContourSet::new();
    set.contours.resize(100_000, LayerContour::new());
    let mut ws = Workspace::with_limits(Limits {
        max_work: 1,
        ..Limits::default()
    });
    assert!(matches!(
        build_poly_mesh(&mut ws, &set),
        Err(Error::LimitExceeded)
    ));
    assert_eq!(ws.usage().work, 100_000);
    assert_eq!(ws.usage().bytes, 0);

    ws.set_limits(Limits::default());
    set.contours.truncate(1);
    build_poly_mesh(&mut ws, &set).unwrap();
    let one = ws.usage().work;
    set.contours.push(LayerContour::new());
    build_poly_mesh(&mut ws, &set).unwrap();
    assert_eq!(ws.usage().work - one, 3); // validation, construction, portals
}
#[test]
fn non_exhaustive_intermediates_support_construction_and_customization() {
    let mut contour = LayerContour::new();
    contour.reg = 1;
    contour.area = 63;
    contour.verts = vec![0, 1, 0, 15, 0, 1, 8, 15, 8, 1, 8, 15, 8, 1, 0, 15];
    let mut set = LayerContourSet::new();
    set.contours.push(contour);
    let mut mesh = build_poly_mesh(&mut Workspace::new(), &set).unwrap();
    assert_eq!(mesh.areas, [63]);
    mesh.flags[0] = 7;
    let mut empty = LayerPolyMesh::new();
    empty.verts = mesh.verts;
    assert_eq!(empty.verts.len(), 12);
    assert!(LayerContour::default().verts.is_empty());
    assert!(LayerContourSet::default().contours.is_empty());
    assert!(LayerPolyMesh::default().polys.is_empty());
}
#[test]
fn queued_add_is_processed_by_an_existing_rebuild_before_its_request() {
    let mut c = TileCache::new(
        params(2),
        RawCompressor,
        DefaultMeshProcess,
        Limits::default(),
    )
    .unwrap();
    c.add_tile(data(0)).unwrap();
    c.add_tile(data(1)).unwrap();
    let mut n = nav(2);
    let first = c.add_obstacle(cylinder(8.0)).unwrap();
    assert!(!c.update(0.1, &mut n).unwrap());
    assert_eq!(
        c.obstacle(first).unwrap().state(),
        ObstacleState::Processing
    );
    let second = c.add_obstacle(cylinder(4.0)).unwrap();
    assert_eq!(
        c.obstacle(second).unwrap().state(),
        ObstacleState::Processing
    );
    assert!(!c.update(0.1, &mut n).unwrap());
    assert_eq!(c.obstacle(first).unwrap().state(), ObstacleState::Processed);
    assert_eq!(
        c.obstacle(second).unwrap().state(),
        ObstacleState::Processed
    );
    // Its add request still runs next; subsequent removal must also finish.
    drain(&mut c, &mut n);
    c.remove_obstacle(second).unwrap();
    c.remove_obstacle(first).unwrap();
    drain(&mut c, &mut n);
    assert!(c.obstacle(first).is_none());
    assert!(c.obstacle(second).is_none());
}
#[test]
fn hash_chain_visits_are_charged_even_without_hits() {
    let mut c = TileCache::new(
        params(64),
        RawCompressor,
        DefaultMeshProcess,
        Limits::default(),
    )
    .unwrap();
    for i in 0..32 {
        c.add_tile(data(i * 16)).unwrap();
    }
    let lim = Limits {
        max_work: 8,
        ..Limits::default()
    };
    c.set_limits(lim);
    assert!(matches!(
        c.query_tiles(&[1.0, 0.0, 1.0], &[2.0, 3.0, 2.0], 64),
        Err(Error::LimitExceeded)
    ));
    assert!(c.usage().work > 8);
}
fn scale(n: usize) -> u64 {
    let mut c = TileCache::new(
        params(n * 2),
        RawCompressor,
        DefaultMeshProcess,
        Limits::default(),
    )
    .unwrap();
    let mut mesh = nav(n * 2);
    let mut total = 0;
    for i in 0..n {
        c.add_tile(data(i32::try_from(i).unwrap())).unwrap();
        total += c.usage().work;
    }
    for batch in (0..n).step_by(32) {
        for i in batch..(batch + 32).min(n) {
            c.add_obstacle(cylinder(i as f32 * 8.0 + 4.0)).unwrap();
            total += c.usage().work;
        }
        loop {
            let done = c.update(0.1, &mut mesh).unwrap();
            total += c.usage().work;
            if done {
                break;
            }
        }
    }
    total
}
#[test]
fn work_scales_for_obstacles_and_tiles() {
    let a = scale(16);
    let b = scale(128);
    println!(
        "tilecache scaling N=16 work={a}; 8N=128 work={b}; ratio={:.4}",
        b as f64 / a as f64
    );
    assert!(b <= a * 11);
}

#[test]
fn public_marking_invalidates_regions_and_rejects_invalid_settings() {
    let mut ws = Workspace::new();
    let mut l = decompress_layer(&mut ws, &RawCompressor, &data(0)).unwrap();
    build_regions(&mut ws, &mut l, 4).unwrap();
    assert!(l.region_count() > 0);
    mark_obstacle_area(&mut ws, &mut l, 1.0, 0.2, cylinder(4.0), 0).unwrap();
    assert_eq!(l.region_count(), 0);
    assert!(l.regions().iter().all(|&r| r == 255));
    assert!(mark_obstacle_area(&mut ws, &mut l, 0.0, 0.2, cylinder(4.0), 0).is_err());
    assert!(mark_obstacle_area(&mut ws, &mut l, 1.0, 0.2, cylinder(4.0), 64).is_err());
}
struct BadCodec;
impl LayerCompressor for BadCodec {
    fn max_compressed_size(&self, _: usize) -> Result<usize, Error> {
        Ok(0)
    }
    fn compress(&self, _: &[u8], _: &mut [u8]) -> Result<usize, Error> {
        Ok(usize::MAX)
    }
    fn decompress(&self, _: &[u8], _: &mut [u8]) -> Result<usize, Error> {
        Ok(usize::MAX)
    }
}
#[test]
fn invalid_codec_lengths_are_errors() {
    let mut ws = Workspace::new();
    assert!(
        build_layer(
            &mut ws,
            &BadCodec,
            &header(0),
            &[1; 64],
            &[63; 64],
            &[0; 64]
        )
        .is_err()
    );
    assert!(decompress_layer(&mut ws, &BadCodec, &data(0)).is_err());
}
#[test]
fn complete_carve_removes_tile_and_removal_restores_it() {
    let mut c = TileCache::new(
        params(2),
        RawCompressor,
        DefaultMeshProcess,
        Limits::default(),
    )
    .unwrap();
    let r = c.add_tile(data(0)).unwrap();
    let mut n = nav(2);
    c.build_nav_mesh_tile(r, &mut n).unwrap();
    let shape = ObstacleShape::Box {
        bmin: [-0.1, -1.0, -0.1],
        bmax: [8.1, 4.0, 8.1],
    };
    let o = c.add_obstacle(shape).unwrap();
    drain(&mut c, &mut n);
    assert!(n.tile_index_at(0, 0, 0).is_none());
    c.remove_obstacle(o).unwrap();
    drain(&mut c, &mut n);
    assert!(n.tile_index_at(0, 0, 0).is_some());
}

#[test]
fn contour_build_requires_successful_region_build() {
    let mut ws = Workspace::new();
    let mut l = decompress_layer(&mut ws, &RawCompressor, &data(0)).unwrap();
    assert!(build_contours(&mut ws, &l, 4, 1.3).is_err());
    build_regions(&mut ws, &mut l, 4).unwrap();
    assert!(build_contours(&mut ws, &l, 4, 1.3).is_ok());
    let lim = Limits {
        max_work: 1,
        ..Limits::default()
    };
    ws.set_limits(lim);
    assert!(build_regions(&mut ws, &mut l, 4).is_err());
    ws.set_limits(Limits::default());
    assert!(build_contours(&mut ws, &l, 4, 1.3).is_err());
}
