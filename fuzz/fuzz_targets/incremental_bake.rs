#![no_main]
//! Invariant 1 of the incremental bake: after random edits, rebaking the tiles the edited
//! boxes touch (minus those whose input hash is unchanged) and merging them into the old
//! result gives exactly the tiles and the manifest of a full bake of the edited world.
//! The input layout is documented in `navmesh_fuzz::bake_world`.
use libfuzzer_sys::fuzz_target;
use navmesh_fuzz::bake_world::{bake_keys, decode, full_bake};

fuzz_target!(|data: &[u8]| {
    let Some(s) = decode(data) else { return };
    let Some(grid) = s.layout() else { return };
    let allowance = u64::from(s.param) * 97;
    let Some((mut tiles, mut manifest)) = full_bake(&s.world, &s.cfg, &grid, allowance) else {
        return;
    };
    let mut edited = s.world.clone();
    let mut boxes = Vec::new();
    for e in &s.edits {
        edited.apply(e, &mut boxes);
    }
    let dirty = manifest.dirty_from_boxes(&boxes);
    let Some((rebuilt, delta)) =
        bake_keys(&edited, &s.cfg, &grid, dirty, Some(&manifest), allowance)
    else {
        return;
    };
    for t in rebuilt {
        tiles.insert(t.key, t.data);
    }
    manifest.apply(&delta).unwrap();
    let Some((full_tiles, full_manifest)) = full_bake(&edited, &s.cfg, &grid, u64::MAX) else {
        return;
    };
    assert_eq!(tiles, full_tiles);
    assert_eq!(manifest, full_manifest);
});
