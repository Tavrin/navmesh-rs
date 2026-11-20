#![no_main]
//! Arbitrary bytes, and byte patches of a valid record, as a bake-profile catalog (`NPCG`)
//! opened against its expected revision and settings.
//!
//! Properties: the reader never panics; an accepted catalog encodes back to the same bytes
//! and its digest is the digest of those bytes.
use libfuzzer_sys::fuzz_target;
use navmesh_fuzz::persisted::{self, LARGE, REV};
use navmesh_rs::bake::profiles::ProfileCatalog;
use navmesh_rs::detour::content_hash;

fuzz_target!(|data: &[u8]| {
    persisted::with_profiles(|fixture| {
        let Some(baked) = fixture.baked.iter().find(|b| b.id == LARGE) else {
            return;
        };
        let valid = baked.catalog.to_bytes();
        // The catalog has no trailing checksum, so a patch alone stays well formed.
        for bytes in [data.to_vec(), persisted::patched(&valid, data, false)] {
            if let Ok(catalog) = ProfileCatalog::from_bytes(&bytes, REV, baked.spec, baked.options)
            {
                assert_eq!(catalog.to_bytes(), bytes);
                assert_eq!(catalog.digest(), content_hash(&bytes));
            }
        }
    });
});
