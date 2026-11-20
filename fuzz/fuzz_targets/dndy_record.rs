#![no_main]
use libfuzzer_sys::fuzz_target;
use navmesh_rs::Limits;
use navmesh_rs::detour::content_hash;
use navmesh_rs::dynamic::{DynamicLayer, Effect, ModifierId, ModifierSpec, Selector};

fn check(bytes: &[u8], limits: Limits) {
    if let Ok(layer) = DynamicLayer::from_bytes(bytes, limits) {
        assert_eq!(layer.to_bytes(), bytes);
    }
}

fn repair_checksum(bytes: &mut [u8]) {
    let footer = bytes.len() - 32;
    let digest = content_hash(&bytes[..footer]);
    bytes[footer..].copy_from_slice(&digest);
}

fuzz_target!(|data: &[u8]| {
    let limits = Limits {
        max_bytes: 64 * 1024,
        max_work: 100_000,
        ..Limits::UNLIMITED
    };
    check(data, limits);
    if (48..=limits.max_bytes).contains(&data.len()) {
        let mut repaired = data.to_vec();
        for version in [1u32, 2] {
            repaired[..4].copy_from_slice(b"DNDY");
            repaired[4..8].copy_from_slice(&version.to_le_bytes());
            repair_checksum(&mut repaired);
            check(&repaired, limits);
        }
    }

    // Start from a valid detail selector to reach the base table's decoder.
    let mut layer = DynamicLayer::new(1, limits).unwrap();
    layer
        .upsert(ModifierSpec::new(
            ModifierId(0),
            Selector::DetailPrism {
                verts_xz: vec![[0., 0.], [1., 0.], [0., 1.]],
                ymin: 0.,
                ymax: 1.,
                area: None,
            },
            Effect::ClearFlags(u16::MAX),
        ))
        .unwrap();
    let mut extended = layer.to_bytes();
    extended.truncate(extended.len() - 36);
    let count = (data.len() / 52).min(16);
    extended.extend_from_slice(&(count as u32).to_le_bytes());
    extended.extend_from_slice(&data[..count * 52]);
    extended.resize(extended.len() + 32, 0);
    repair_checksum(&mut extended);
    check(&extended, limits);
});
