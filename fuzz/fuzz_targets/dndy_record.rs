#![no_main]
use libfuzzer_sys::fuzz_target;
use navmesh_rs::Limits;
use navmesh_rs::detour::content_hash;
use navmesh_rs::dynamic::DynamicLayer;

fn check(bytes: &[u8], limits: Limits) {
    if let Ok(layer) = DynamicLayer::from_bytes(bytes, limits) {
        assert_eq!(layer.to_bytes(), bytes);
    }
}

fuzz_target!(|data: &[u8]| {
    let limits = Limits {
        max_bytes: 64 * 1024,
        max_work: 100_000,
        ..Limits::UNLIMITED
    };
    check(data, limits);
    if (48..=limits.max_bytes).contains(&data.len()) {
        // Reach selector and effect validation even when mutating a valid record.
        let mut repaired = data.to_vec();
        repaired[..8].copy_from_slice(b"DNDY\x01\x00\x00\x00");
        let footer = repaired.len() - 32;
        let digest = content_hash(&repaired[..footer]);
        repaired[footer..].copy_from_slice(&digest);
        check(&repaired, limits);
    }
});
