#![no_main]
//! Arbitrary bytes as stored tile state on a navmesh holding one real tile.
//!
//! Layout: one mode byte, then the blob. Mode bit 0 removes the tile and adds it
//! back first (the slot gets a new salt, so a blob stored before is stale); bit 1
//! first stores the tile's real state into the blob's first bytes; bit 2 also asks
//! for state of a tile ref read from the blob's first bytes.
//!
//! Properties: restore never panics; an accepted blob leaves exactly its flags and
//! its area ids (masked to 6 bits) in the tile, read back with `store_tile_state`;
//! a rejected blob leaves the tile's state unchanged; `store_tile_state` succeeds
//! exactly when the buffer holds `tile_state_size` bytes; refs that name no tile
//! are errors.
use libfuzzer_sys::fuzz_target;
use navmesh_fuzz::sample_tile;
use navmesh_rs::Error;
use navmesh_rs::detour::{NavMesh, NavMeshParams, PolyRef};

fn stored(nav: &NavMesh, r: PolyRef) -> Vec<u8> {
    let mut buf = vec![0u8; nav.tile_state_size(r).unwrap_or(0)];
    let _ = nav.store_tile_state(r, &mut buf);
    buf
}

fuzz_target!(|data: &[u8]| {
    let Some((&mode, blob)) = data.split_first() else {
        return;
    };
    let params = NavMeshParams {
        orig: [0.0; 3],
        tile_width: 10.0,
        tile_height: 10.0,
        max_tiles: 4,
        max_polys: 1 << 14,
    };
    let Ok(mut nav) = NavMesh::new(&params) else {
        return;
    };
    let Ok(mut tile) = nav.add_tile(sample_tile()) else {
        return;
    };
    if mode & 1 != 0 {
        let Ok(bytes) = nav.remove_tile(tile) else {
            return;
        };
        let Ok(t) = nav.add_tile(bytes) else {
            return;
        };
        tile = t;
    }
    let mut blob = blob.to_vec();
    if mode & 2 != 0 {
        let real = stored(&nav, tile);
        let n = real.len().min(blob.len());
        blob[..n].copy_from_slice(&real[..n]);
    }
    if mode & 4 != 0 {
        let mut r = [0u8; core::mem::size_of::<PolyRef>()];
        let n = r.len().min(blob.len());
        r[..n].copy_from_slice(&blob[..n]);
        let other = PolyRef::from_le_bytes(r);
        if other != tile {
            assert_eq!(
                nav.tile_state_size(other).is_ok(),
                nav.tile_by_ref(other).is_some()
            );
            let mut buf = [0u8; 64];
            let _ = nav.store_tile_state(other, &mut buf);
            let _ = nav.restore_tile_state(other, &blob);
        }
    }

    let size = nav.tile_state_size(tile).expect("resident tile");
    let before = stored(&nav, tile);
    let mut small = vec![0u8; blob.len().min(size + 8)];
    let fits = small.len() >= size;
    match nav.store_tile_state(tile, &mut small) {
        Ok(n) => assert!(fits && n == size),
        Err(e) => assert!(!fits && e == Error::BufferTooSmall),
    }
    match nav.restore_tile_state(tile, &blob) {
        Ok(()) => {
            assert!(blob.len() >= size);
            let after = stored(&nav, tile);
            let header = 8 + core::mem::size_of::<PolyRef>();
            for (a, b) in after[header..]
                .chunks_exact(4)
                .zip(blob[header..].chunks_exact(4))
            {
                assert_eq!(a[0..2], b[0..2]);
                assert_eq!(a[2], b[2] & 0x3f);
                assert_eq!(a[3], 0);
            }
        }
        Err(_) => assert_eq!(stored(&nav, tile), before),
    }
});
