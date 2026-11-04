#![no_main]
//! Arbitrary bytes as a saved residency record.
//!
//! Layout: the record bytes. Seeds are records saved from small streamed
//! navmeshes (`seeds residency`).
//!
//! Properties: parsing never panics; an accepted record re-encodes to the same
//! bytes; restoring it into a fresh navmesh of its layout (when that layout is
//! small enough to build) either fails with the navmesh untouched or leaves every
//! recorded tile unloaded, refs into it `Unloaded`, older salts `Stale`, and a
//! commit at a recorded key going back into its slot.
use libfuzzer_sys::fuzz_target;
use navmesh_fuzz::grid_quad;
use navmesh_rs::Limits;
use navmesh_rs::detour::{
    NavMesh, NavMeshParams, RefState, ResidencyRecord, SlotPolicy, prepare_tile,
};

fuzz_target!(|data: &[u8]| {
    let Ok(rec) = ResidencyRecord::from_bytes(data) else {
        return;
    };
    assert_eq!(rec.to_bytes(), data);
    let bytes = rec.to_bytes();
    let max_tiles = u32::from_le_bytes([bytes[12], bytes[13], bytes[14], bytes[15]]);
    if max_tiles == 0 || max_tiles > 4096 {
        return;
    }
    let policy = if bytes[16] == 1 {
        let r = |at: usize| {
            u32::from_le_bytes([bytes[at], bytes[at + 1], bytes[at + 2], bytes[at + 3]])
        };
        SlotPolicy::Keyed {
            origin: [r(20) as i32, r(24) as i32],
            width: r(28),
            height: r(32),
            max_layers: r(36),
        }
    } else {
        SlotPolicy::Sequential
    };
    let params = NavMeshParams {
        orig: [0.0; 3],
        tile_width: 10.0,
        tile_height: 10.0,
        max_tiles: max_tiles as i32,
        max_polys: 16,
    };
    let Ok(mut nav) = NavMesh::with_policy(&params, Limits::UNLIMITED, policy) else {
        return;
    };
    if nav.restore_residency(&rec).is_err() {
        assert_eq!(nav.epoch(), 0);
        assert!(nav.residency().is_empty());
        return;
    }
    let again = nav.residency().to_bytes();
    assert_eq!(
        again, bytes,
        "restore keeps every slot's salt and reservation"
    );
    // Entries: slot, salt, flag, layer, x, y, polys at 48 + 40 * i.
    let count = (bytes.len() - 48) / 40;
    for i in 0..count.min(64) {
        let at = 48 + 40 * i;
        let r = |o: usize| {
            u32::from_le_bytes([
                bytes[at + o],
                bytes[at + o + 1],
                bytes[at + o + 2],
                bytes[at + o + 3],
            ])
        };
        let (slot, salt, flag) = (r(0), r(4), r(8));
        let base = nav.encode_poly_id(salt, slot, 0);
        // Check the saved history before a commit can wrap the finite salt.
        if salt > 1 {
            assert_eq!(
                nav.ref_state(nav.encode_poly_id(salt - 1, slot, 0)),
                RefState::Stale
            );
        }
        if flag == 1 && r(24) > 0 {
            let (x, y) = (r(16) as i32, r(20) as i32);
            assert_eq!(
                nav.ref_state(base),
                RefState::Unloaded(navmesh_rs::detour::TileKey { x, y })
            );
            assert!(!nav.is_valid_poly_ref(base));
            if r(12) == 0 && (-1000..1000).contains(&x) && (-1000..1000).contains(&y) {
                // A tile at that key returns to the slot (with a new salt unless its
                // content hash matches the recorded one).
                if let Ok(t) = prepare_tile(&params, Limits::UNLIMITED, grid_quad(x, y, 0)) {
                    if let Ok(tr) = nav.commit(t) {
                        assert_eq!(nav.decode_poly_id_tile(tr), slot);
                    }
                }
            }
        }
    }
});
