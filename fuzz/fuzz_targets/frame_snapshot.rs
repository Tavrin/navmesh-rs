#![no_main]
//! Arbitrary bytes, and byte patches of a valid record, as moving-platform frame state
//! (`DNFR`) restored into the matching frame set.
//!
//! Properties: the reader never panics; an accepted record is saved back byte for byte;
//! a rejected record leaves the frame set exactly as it was.
use libfuzzer_sys::fuzz_target;
use navmesh_fuzz::persisted;

fn check(bytes: &[u8]) {
    let mut frames = persisted::lift();
    let before = frames.to_bytes();
    match frames.restore(bytes) {
        Ok(()) => assert_eq!(frames.to_bytes(), bytes),
        Err(_) => assert_eq!(frames.to_bytes(), before),
    }
}

fuzz_target!(|data: &[u8]| {
    check(data);
    check(&persisted::patched(persisted::frame_record(), data, true));
});
