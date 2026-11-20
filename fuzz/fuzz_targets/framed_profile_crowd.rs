#![no_main]
//! Arbitrary bytes, and byte patches of a valid record, as a framed mixed-profile crowd
//! snapshot (`NPFM`) bound to a two-frame world of profile meshes.
//!
//! Properties: the reader never panics; an accepted record's re-encoding is a fixed point (it
//! restores, and encodes to the same bytes again; the input itself may differ because the
//! embedded crowd reader accepts some out-of-range field values and normalises them); the
//! restored crowd steps under a budget without panicking.
use libfuzzer_sys::fuzz_target;
use navmesh_fuzz::persisted::{self, LIMITS};
use navmesh_rs::Budget;
use navmesh_rs::bake::profiles::FramedMixedProfileCrowdWorld;

fuzz_target!(|data: &[u8]| {
    persisted::with_framed_profiles(|frames, valid| {
        for bytes in [data.to_vec(), persisted::patched(valid, data, true)] {
            if let Ok(mut crowd) = FramedMixedProfileCrowdWorld::restore(frames, &bytes, LIMITS) {
                let once = crowd.save(frames).unwrap_or_default();
                let Ok(second) = FramedMixedProfileCrowdWorld::restore(frames, &once, LIMITS) else {
                    panic!("re-encoded record is rejected");
                };
                assert_eq!(second.save(frames).ok().as_deref(), Some(&once[..]));
                let _ = crowd.step(frames, &mut Budget::new(100_000));
            }
        }
    });
});
