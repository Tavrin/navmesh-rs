#![no_main]
//! Arbitrary bytes as a navmesh tile: the tile reader, a single-tile navmesh,
//! `add_tile` on a tiled navmesh, remove and restore (fresh and stale), and a set
//! of queries on whatever was accepted.
use libfuzzer_sys::fuzz_target;
use navmesh_fuzz::exercise_queries;
use navmesh_rs::detour::{MeshTile, NavMesh, NavMeshParams};

fuzz_target!(|data: &[u8]| {
    let _ = MeshTile::parse(data.to_vec());
    if let Ok((nav, _)) = NavMesh::new_single(data.to_vec()) {
        exercise_queries(&nav);
    }
    let params = NavMeshParams {
        orig: [0.0; 3],
        tile_width: 32.0,
        tile_height: 32.0,
        max_tiles: 4,
        max_polys: 1 << 14,
    };
    let Ok(mut nav) = NavMesh::new(&params) else {
        return;
    };
    if let Ok(r) = nav.add_tile(data.to_vec()) {
        exercise_queries(&nav);
        if nav.remove_tile(r).is_ok() {
            let _ = nav.restore_tile(data.to_vec(), r);
            let _ = nav.restore_tile_allow_stale(data.to_vec(), r);
            exercise_queries(&nav);
        }
    }
});
