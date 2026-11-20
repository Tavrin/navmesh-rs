#![no_main]
//! Arbitrary bytes, and byte patches of a valid record, as a framed crowd snapshot (`DCFR`)
//! bound to a lift.
//!
//! Properties: the reader never panics; an accepted record is saved back byte for byte, except
//! that a stored mesh-epoch of zero (absent) or ahead of the live mesh rebinds to it; the restored crowd steps under
//! a budget without panicking.
use libfuzzer_sys::fuzz_target;
use navmesh_fuzz::persisted::{self, LIMITS};
use navmesh_rs::Budget;
use navmesh_rs::crowd::platforms::FramedCrowd;
use navmesh_rs::crowd::world::CrowdWorld;
use navmesh_rs::frames::FrameSet;

fn check(frames: &FrameSet, bytes: &[u8]) {
    if let Ok(mut crowd) = FramedCrowd::<CrowdWorld>::from_bytes(frames, bytes, LIMITS) {
        let saved = crowd.to_bytes(frames);
        // The trailing checksum covers the epoch words, so it is compared with them.
        let body = saved.len().saturating_sub(32);
        assert!(
            saved == bytes
                || bytes
                    .get(..body)
                    .is_some_and(|input| { persisted::rebinds_only_epochs(&saved[..body], input) }),
            "record is not saved back byte for byte"
        );
        let _ = crowd.step(frames, &mut Budget::new(100_000));
    }
}

fuzz_target!(|data: &[u8]| {
    persisted::with_framed_crowd(|frames, valid| {
        check(frames, data);
        check(frames, &persisted::patched(valid, data, true));
    });
});
