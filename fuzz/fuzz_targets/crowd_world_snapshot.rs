#![no_main]
use libfuzzer_sys::fuzz_target;
use navmesh_rs::crowd::world::CrowdWorld;
use navmesh_rs::detour::{NavMesh, NavMeshCreateParams, create_nav_mesh_data};
use navmesh_rs::{Budget, Limits};

fuzz_target!(|data: &[u8]| {
    let n = 0xffff;
    let Ok(tile) = create_nav_mesh_data(&NavMeshCreateParams {
        verts: &[0, 0, 0, 0, 0, 20, 20, 0, 20, 20, 0, 0],
        vert_count: 4,
        polys: &[0, 1, 2, 3, n, n, n, n, n, n, n, n],
        poly_count: 1,
        nvp: 6,
        poly_flags: &[1],
        poly_areas: &[0],
        bmin: [0.; 3],
        bmax: [20., 1., 20.],
        cs: 1.,
        ch: 1.,
        walkable_height: 2.,
        walkable_radius: 0.6,
        walkable_climb: 0.5,
        ..Default::default()
    }) else {
        return;
    };
    let Ok((nav, _)) = NavMesh::new_single(tile) else {
        return;
    };
    let limits = Limits {
        max_bytes: 4 * 1024 * 1024,
        max_work: 100_000,
        ..Limits::UNLIMITED
    };
    if let Ok(mut world) = CrowdWorld::from_bytes(&nav, data, limits) {
        let bytes = world.to_bytes();
        let restored = CrowdWorld::from_bytes(&nav, &bytes, limits).unwrap();
        assert_eq!(world.digest(), restored.digest());
        let _ = world.step(&nav, &mut Budget::new(100_000));
    }
});
