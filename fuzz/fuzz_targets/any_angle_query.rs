#![no_main]
use libfuzzer_sys::fuzz_target;
use navmesh_rs::detour::{
    AnyAnglePoint, AnyAngleSearch, DefaultFilter, MeshTile, NavMesh, NavMeshQuery,
};
use navmesh_rs::{Budget, Budgeted, Limits};
fuzz_target!(|data: &[u8]| {
    if data.len() < 4 {
        return;
    }
    let limits = Limits {
        max_bytes: 4 << 20,
        max_work: 100_000,
        max_tile_polys: 128,
        ..Limits::default()
    };
    let Ok(tile) = MeshTile::parse(data[4..].to_vec()) else {
        return;
    };
    let params = navmesh_rs::detour::NavMeshParams {
        orig: tile.header.bmin,
        tile_width: (tile.header.bmax[0] - tile.header.bmin[0]).max(1.0),
        tile_height: (tile.header.bmax[2] - tile.header.bmin[2]).max(1.0),
        max_tiles: 4,
        max_polys: 256,
    };
    let Ok(mut nav) = NavMesh::with_limits(&params, limits) else {
        return;
    };
    if nav.add_tile(data[4..].to_vec()).is_err() {
        return;
    }
    let Ok(q) = NavMeshQuery::new(&nav, 256) else {
        return;
    };
    let filter = DefaultFilter::default();
    let point = |x: u8, z: u8| {
        let lo = tile.header.bmin;
        let hi = tile.header.bmax;
        let p = [
            lo[0] + (hi[0] - lo[0]) * x as f32 / 255.0,
            lo[1],
            lo[2] + (hi[2] - lo[2]) * z as f32 / 255.0,
        ];
        q.find_nearest_poly(&p, &[2.0; 3], &filter)
            .ok()
            .map(|(polygon, position, _)| AnyAnglePoint { polygon, position })
    };
    let (Some(a), Some(b)) = (point(data[0], data[1]), point(data[2], data[3])) else {
        return;
    };
    let Ok(mut search) = AnyAngleSearch::begin(&q, a, b, &filter) else {
        return;
    };
    let plain = q.find_any_angle_path(a, b, &filter, &mut Budget::new(256));
    for _ in 0..256 {
        let mut budget = Budget::new(1);
        let result = search.resume(&q, &mut budget);
        assert!(budget.used() <= 1);
        match result {
            Ok(Budgeted::Complete(path)) => {
                if let Ok(Budgeted::Complete(expected)) = &plain {
                    assert_eq!(&path, expected);
                }
                break;
            }
            Err(_) => break,
            _ => {}
        }
    }
});
