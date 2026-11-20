#![no_main]
//! Arbitrary bytes, and byte patches of a valid record, as one of three profile records:
//! slot history (`NPRS`), a saved polygon reference (`NPRF`) or a single-profile crowd
//! world (`NPCW`).
//!
//! Layout: one mode byte (bits 0-1: record kind, 0 to 2), then the record or the patches.
//!
//! Properties: no reader panics; an accepted slot history or reference is saved back byte for
//! byte; an accepted crowd's re-encoding is a fixed point (the input itself may differ because
//! the embedded crowd reader normalises some accepted values) and steps under a budget.
use libfuzzer_sys::fuzz_target;
use navmesh_fuzz::persisted::{self, LARGE, LIMITS};
use navmesh_rs::Budget;
use navmesh_rs::bake::profiles::{ProfileCrowdWorld, ProfileMesh};

fuzz_target!(|data: &[u8]| {
    let Some((&mode, rest)) = data.split_first() else {
        return;
    };
    persisted::with_profiles(|fixture| {
        let Ok(installed) = fixture.meshes.get(LARGE) else {
            return;
        };
        let valid = match mode & 3 {
            0 => &fixture.residency,
            1 => &fixture.reference,
            _ => &fixture.crowd,
        };
        for bytes in [rest.to_vec(), persisted::patched(valid, rest, true)] {
            match mode & 3 {
                0 => {
                    // Slot history goes into a fresh mesh before any tile is committed.
                    let Some(baked) = fixture.baked.iter().find(|b| b.id == LARGE) else {
                        return;
                    };
                    let Ok(mut mesh) = ProfileMesh::new(baked.catalog.clone()) else {
                        return;
                    };
                    if mesh.restore_residency(&bytes).is_ok() {
                        assert_eq!(mesh.save_residency().ok().as_deref(), Some(&bytes[..]));
                    }
                }
                1 => {
                    if let Ok(polygon) = installed.restore_reference(&bytes) {
                        assert_eq!(
                            installed.save_reference(&polygon).ok().as_deref(),
                            Some(&bytes[..])
                        );
                    }
                }
                _ => {
                    if let Ok(mut world) = ProfileCrowdWorld::restore(installed, &bytes, LIMITS) {
                        // The embedded crowd reader normalises some accepted values, so the
                        // re-encoding, not the input, is the fixed point.
                        let once = world.save(installed).unwrap_or_default();
                        let Ok(second) = ProfileCrowdWorld::restore(installed, &once, LIMITS) else {
                            panic!("re-encoded record is rejected");
                        };
                        assert_eq!(second.save(installed).ok().as_deref(), Some(&once[..]));
                        let _ = world.step(installed, &mut Budget::new(100_000));
                    }
                }
            }
        }
    });
});
