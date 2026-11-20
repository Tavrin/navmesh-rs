#![no_main]
//! Arbitrary bytes, and byte patches of a valid record, as a framed crowd snapshot (`DCFR`)
//! bound to a lift.
//!
//! Properties: the reader never panics; an accepted record's re-encoding is a fixed point (it
//! restores, and encodes to the same bytes again; the input itself may differ because the
//! embedded crowd reader accepts some out-of-range field values and normalises them); the
//! restored crowd steps under a budget without panicking.
use libfuzzer_sys::fuzz_target;
use navmesh_fuzz::persisted::{self, LIMITS};
use navmesh_rs::Budget;
use navmesh_rs::crowd::platforms::FramedCrowd;
use navmesh_rs::crowd::world::CrowdWorld;
use navmesh_rs::frames::FrameSet;

fn check(frames: &FrameSet, bytes: &[u8]) {
    if let Ok(mut crowd) = FramedCrowd::<CrowdWorld>::from_bytes(frames, bytes, LIMITS) {
        let once = crowd.to_bytes(frames);
        let Ok(second) = FramedCrowd::<CrowdWorld>::from_bytes(frames, &once, LIMITS) else {
            panic!("re-encoded record is rejected");
        };
        assert_eq!(second.to_bytes(frames), once);
        let _ = crowd.step(frames, &mut Budget::new(100_000));
    }
}

fuzz_target!(|data: &[u8]| {
    persisted::with_framed_crowd(|frames, valid| {
        check(frames, data);
        check(frames, &persisted::patched(valid, data, true));
    });
});
