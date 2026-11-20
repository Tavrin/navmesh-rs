#![no_main]
//! Arbitrary bytes, and byte patches of a valid record, as the runtime link layer (`DNLK`)
//! restored into an empty mesh.
//!
//! Properties: the reader never panics; an accepted record is saved back byte for byte;
//! ground tiles added afterwards land the restored links without panicking.
use libfuzzer_sys::fuzz_target;
use navmesh_fuzz::persisted;

fn check(bytes: &[u8]) {
    let mut nav = persisted::link_mesh();
    if nav.restore_runtime_links(bytes).is_ok() {
        assert_eq!(nav.runtime_links_to_bytes(), bytes);
        for tile in persisted::link_tiles() {
            let _ = nav.add_tile(tile);
        }
        let _ = nav.runtime_links_to_bytes();
    }
}

fuzz_target!(|data: &[u8]| {
    check(data);
    // Keep the digest valid so the definition, order and landing checks run.
    check(&persisted::patched(persisted::runtime_links_record(), data, true));
});
