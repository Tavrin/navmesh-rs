#![no_main]
//! Arbitrary bytes, and byte patches of a valid record, as a framed mixed-profile crowd
//! snapshot (`NPFM`) bound to a two-frame world of profile meshes.
//!
//! Properties: the reader never panics; an accepted record is saved back byte for byte; the
//! restored crowd steps under a budget without panicking.
use libfuzzer_sys::fuzz_target;
use navmesh_fuzz::persisted::{self, LIMITS};
use navmesh_rs::Budget;
use navmesh_rs::bake::profiles::FramedMixedProfileCrowdWorld;

fuzz_target!(|data: &[u8]| {
    persisted::with_framed_profiles(|frames, valid| {
        for bytes in [data.to_vec(), persisted::patched(valid, data, true)] {
            if let Ok(mut crowd) = FramedMixedProfileCrowdWorld::restore(frames, &bytes, LIMITS) {
                assert_eq!(crowd.save(frames).ok().as_deref(), Some(&bytes[..]));
                let _ = crowd.step(frames, &mut Budget::new(100_000));
            }
        }
    });
});
