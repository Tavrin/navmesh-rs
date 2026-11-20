#![no_main]
//! Arbitrary bytes, and byte patches of a valid record, as a mixed-profile crowd snapshot
//! (`NPCM`) bound to two installed profiles.
//!
//! Properties: the reader never panics; an accepted record is saved back byte for byte; the
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
                assert_eq!(world.save(meshes).ok().as_deref(), Some(&bytes[..]));
                let _ = world.step(meshes, &mut Budget::new(100_000));
            }
        }
    });
});
