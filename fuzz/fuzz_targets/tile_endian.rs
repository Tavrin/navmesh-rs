#![no_main]
//! Arbitrary bytes through the byte-order conversions of tile bytes and layer
//! headers, and, when the conversion for reading succeeds, through the tile reader
//! and a navmesh.
//!
//! Properties: no conversion panics or reads out of bounds, whatever the counts
//! in the header say; converting a tile to the other byte order (body, then
//! header) and back (header, then body) gives the original bytes; the same holds
//! for layer headers. A converted tile that the reader accepts can be queried.
use libfuzzer_sys::fuzz_target;
use navmesh_fuzz::exercise_queries;
use navmesh_rs::detour::{
    MeshTile, NavMesh, nav_mesh_data_swap_endian, nav_mesh_header_swap_endian,
    nav_mesh_swap_off_mesh_user_ids,
};
use navmesh_rs::tilecache::tile_cache_header_swap_endian;

fuzz_target!(|data: &[u8]| {
    // Writing direction: body first, header second, and back again.
    let mut written = data.to_vec();
    if nav_mesh_data_swap_endian(&mut written).is_ok()
        && nav_mesh_header_swap_endian(&mut written).is_ok()
    {
        let mut back = written.clone();
        nav_mesh_header_swap_endian(&mut back).expect("header of a converted tile");
        nav_mesh_data_swap_endian(&mut back).expect("body of a restored tile");
        assert_eq!(back, data);
    }
    // Reading direction: header first, body second, then the reader.
    let mut read = data.to_vec();
    if nav_mesh_header_swap_endian(&mut read).is_ok()
        && nav_mesh_data_swap_endian(&mut read).is_ok()
    {
        let _ = nav_mesh_swap_off_mesh_user_ids(&mut read);
        let _ = MeshTile::parse(read.clone());
        if let Ok((nav, _)) = NavMesh::new_single(read) {
            exercise_queries(&nav);
        }
    }
    let mut layer = data.to_vec();
    if tile_cache_header_swap_endian(&mut layer).is_ok() {
        let mut back = layer.clone();
        tile_cache_header_swap_endian(&mut back).expect("header of a converted layer");
        assert_eq!(back, data);
    }
});
