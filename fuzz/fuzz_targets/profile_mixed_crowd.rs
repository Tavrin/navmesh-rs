#![no_main]
//! Arbitrary bytes, and byte patches of a valid record, as a mixed-profile crowd snapshot
//! (`NPCM`) bound to two installed profiles.
//!
//! Properties: the reader never panics; an accepted record's re-encoding is a fixed point (it
//! restores, and encodes to the same bytes again; the input itself may differ because the
//! embedded crowd reader accepts some out-of-range field values and normalises them); the
//! restored crowd steps under a budget without panicking.
use libfuzzer_sys::fuzz_target;
use navmesh_fuzz::persisted::{self, LIMITS};
use navmesh_rs::Budget;
use navmesh_rs::bake::profiles::MixedProfileCrowdWorld;

fuzz_target!(|data: &[u8]| {
    persisted::with_profiles(|fixture| {
        let meshes = &fixture.meshes;
        for bytes in [
            data.to_vec(),
            persisted::patched(&fixture.mixed, data, true),
        ] {
            if let Ok(mut world) = MixedProfileCrowdWorld::restore(meshes, &bytes, LIMITS) {
                let once = world.save(meshes).unwrap_or_default();
                let Ok(second) = MixedProfileCrowdWorld::restore(meshes, &once, LIMITS) else {
                    panic!("re-encoded record is rejected");
                };
                assert_eq!(second.save(meshes).ok().as_deref(), Some(&once[..]));
                let _ = world.step(meshes, &mut Budget::new(100_000));
            }
        }
    });
});
