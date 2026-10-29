#![no_main]
//! Arbitrary meshes and settings through the whole Recast pipeline, the Detour
//! tile builder, a single-tile navmesh and a set of queries (input layout in
//! `navmesh_fuzz::RecastInput`).
use libfuzzer_sys::fuzz_target;
use navmesh_fuzz::{RecastInput, run_recast};

fuzz_target!(|data: &[u8]| {
    if let Some(input) = RecastInput::decode(data) {
        let _ = run_recast(&input);
    }
});
