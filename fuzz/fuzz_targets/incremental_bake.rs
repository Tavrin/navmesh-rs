#![no_main]
//! Invariant 1 of the incremental bake: after random edits, and a layout change (the grid
//! grows or shrinks, the height range moves), rebaking the tiles `dirty_from_boxes` names
//! (minus those whose input hash is unchanged) and merging them into the old result gives
//! exactly the tiles and the manifest of a full bake of the edited world. Installing the
//! rebaked tiles into a navmesh that holds the old ones, behind stale queued versions of
//! the same tiles, leaves exactly the tiles of the full bake. The input layout is
//! documented in `navmesh_fuzz::bake_world`.
use libfuzzer_sys::fuzz_target;
use navmesh_fuzz::bake_world::{bake_keys, decode, full_bake};
use navmesh_rs::bake::{BakeManifest, TileInstaller};
use navmesh_rs::detour::NavMesh;
use navmesh_rs::{Budget, Progress};

fuzz_target!(|data: &[u8]| {
    let Some(s) = decode(data) else { return };
    let Some((old_grid, grid)) = s.layout_pair() else {
        return;
    };
    let allowance = u64::from(s.param) * 97;
    let Some((old_outputs, old_delta)) = bake_keys(
        &s.world,
        &s.cfg,
        &old_grid,
        old_grid.keys(),
        None,
        allowance,
    ) else {
        return;
    };
    let mut manifest = BakeManifest::new(&s.cfg, &old_grid).unwrap();
    manifest.apply(&old_delta).unwrap();
    let mut tiles: std::collections::BTreeMap<_, _> = old_outputs
        .iter()
        .map(|t| (t.key, t.data.clone()))
        .collect();
    let mut edited = s.world.clone();
    let mut boxes = Vec::new();
    for e in &s.edits {
        edited.apply(e, &mut boxes);
    }
    let dirty = manifest
        .dirty_from_boxes(&s.cfg, &grid, &boxes)
        .expect("a layout that fits the configuration");
    let same_config =
        BakeManifest::config_hash(&s.cfg, &old_grid) == BakeManifest::config_hash(&s.cfg, &grid);
    assert_eq!(dirty.full_rebake, !same_config);
    if dirty.full_rebake {
        assert_eq!(dirty.keys, grid.keys());
    }
    let mut stale = manifest.clone();
    let dropped = stale.retarget(&s.cfg, &grid).unwrap();
    assert_eq!(dropped, dirty.removed);
    let Some((rebuilt, delta)) = bake_keys(
        &edited,
        &s.cfg,
        &grid,
        dirty.keys.clone(),
        Some(&stale),
        allowance,
    ) else {
        return;
    };
    // A delta of the new layout is refused by the manifest that was not retargeted.
    if dirty.full_rebake {
        assert!(manifest.clone().apply(&delta).is_err());
    }
    stale.apply(&delta).unwrap();
    let Some((full_outputs, full_delta)) =
        bake_keys(&edited, &s.cfg, &grid, grid.keys(), None, u64::MAX)
    else {
        return;
    };
    let mut full_manifest = BakeManifest::new(&s.cfg, &grid).unwrap();
    full_manifest.apply(&full_delta).unwrap();
    let full_tiles: std::collections::BTreeMap<_, _> = full_outputs
        .iter()
        .map(|t| (t.key, t.data.clone()))
        .collect();

    tiles.retain(|k, _| grid.contains(*k) && !dirty.full_rebake);
    for t in &rebuilt {
        tiles.insert(t.key, t.data.clone());
    }
    assert_eq!(tiles, full_tiles);
    assert_eq!(stale, full_manifest);

    // The navmesh: old tiles resident, the stale versions queued before the rebaked ones.
    let capacity = (old_grid.keys().len().max(grid.keys().len())) as i32;
    let mut nav = NavMesh::new(&grid.nav_mesh_params(capacity, 1 << 12)).unwrap();
    let mut installer = TileInstaller::new();
    for t in old_outputs.iter().cloned() {
        installer.push(t);
    }
    if installer
        .install(&mut nav, &mut Budget::unlimited())
        .is_err()
    {
        return;
    }
    for t in old_outputs
        .into_iter()
        .filter(|t| dirty.keys.contains(&t.key))
    {
        installer.push(t);
    }
    for t in rebuilt {
        installer.push(t);
    }
    let slice = 1 + u64::from(s.param) % 5;
    while installer
        .install(&mut nav, &mut Budget::new(slice))
        .unwrap()
        != Progress::Done
    {}
    for key in &dirty.removed {
        if let Some(i) = nav.tile_index_at(key.x, key.z, 0) {
            nav.remove_tile(nav.tile_ref(i)).unwrap();
        }
    }
    for key in grid.keys() {
        let resident = nav
            .tile_index_at(key.x, key.z, 0)
            .map(|i| nav.remove_tile(nav.tile_ref(i)).unwrap());
        assert_eq!(resident, full_tiles[&key], "navmesh tile {key:?}");
    }
});
