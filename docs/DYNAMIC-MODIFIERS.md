# Dynamic modifiers

Enable the `dynamic` Cargo feature to add gates and cost zones to a `NavMesh`.
Register each modifier with a stable ID chosen by the application. A gate can use
`Effect::ClearFlags(u16::MAX)` with its enabled bit set when closed. A zone can
use `Effect::SetArea` and the filter's area costs. To grant access to particular
profiles, choose flag bits that the relevant filters include.

```rust
use navmesh_rs::dynamic::{DynamicLayer, Effect, ModifierId, ModifierSpec, Selector};
use navmesh_rs::detour::NavMesh;
use navmesh_rs::{Error, Limits};

fn register_gate(nav: &mut NavMesh) -> Result<ModifierId, Error> {
    let gate = ModifierId(3);
    let mut layer = DynamicLayer::new(64, Limits::UNLIMITED)?;
    layer.upsert(ModifierSpec::new(
        gate,
        Selector::Prism {
            verts_xz: vec![[2., 1.], [3., 1.], [3., 4.], [2., 4.]],
            ymin: -0.5,
            ymax: 1.5,
            area: Some(4),
        },
        Effect::ClearFlags(u16::MAX),
    ))?;
    layer.set_enabled(gate, true)?;
    nav.set_dynamic_layer(Some(layer));
    Ok(gate)
}

fn open_gate(nav: &mut NavMesh, gate: ModifierId) -> Result<(), Error> {
    nav.set_modifier_enabled(gate, false)?;
    Ok(())
}
```

A prism uses the inclusive Y range and the vertex centroid of each ground polygon
in the tile's original coordinates. Either winding is accepted for a strictly
convex XZ outline of 3 to 64 vertices. Bake the same prism as an area volume so
the bake splits polygons at the boundary. Match that baked area in the selector,
and use `modifier_selection` to count selected polygons and diagnose intersecting
polygon bounds with centroids outside the prism. `Selector::Links` selects baked
off-mesh links by their authored user ID.

The mesh recomputes governed polygons from their baked state, applying enabled
effects in ascending ID order. A disabled spec still governs its selection.
Replacing a spec preserves its enabled bit. Unregistering a spec removes its
effect and reapplies overlapping effects. Direct flag and area setters return an
error for governed polygons. Legacy tile-state restores apply to unmanaged
polygons, and governed polygons retain the current layer's result.

The mesh applies its current layer during add, commit, restore and replacement,
including install jobs and tile-cache rebuilds. Prepared tiles can be built on
workers using the existing API. The owner thread selects and applies the
modifier state as it attaches the tile. An unloaded tile needs no per-tile
modifier state. Derived flags and areas stay out of baked bytes, content hashes
and ref layout provenance.

Toggles update the resident selection immediately, with the same refs, salts
and structural epoch. The budgeted toggle checks a work unit per selected
polygon before applying the change. Set gate state before running queries or
stepping either crowd. Ordinary crowd corridor checks see the current filters
and retain their existing lookahead. Connectivity labels still describe
unfiltered topology. Use `reachable_filtered` for current filtered directed
reachability, and restart an in-flight search if its result needs to reflect a
gate change. `state_epoch` tracks modifier edits and effective state changes
separately from structure, and `state_stamp` records the last effective state
change for a resident tile.

## Saves and replay

If the application already stores authored specs, save one enabled bit per gate
against those stable IDs. Re-register the specs and apply the bits before loading
tiles. To save a standalone modifier layer, use `DynamicLayer::to_bytes` and
`DynamicLayer::from_bytes`. To preserve tile references across restart, also save
the navmesh residency record and restore it before committing any tiles.

### DNDY version 1

All multibyte fields are little-endian. The record contains:

1. Four bytes `DNDY`, a `u32` version (1), a `u32` exclusive ID limit, and a
   `u32` spec count.
2. `ceil(id_limit / 8)` enabled-bit bytes. ID `i` uses bit `i % 8` in byte
   `i / 8`, least significant bit first. Bits without registered specs are zero.
3. Specs in strictly ascending ID order. Each starts with a `u32` ID and a
   one-byte selector tag. Tag 0 is a prism with a one-byte vertex count, one-byte
   baked area (255 for any area), two `f32` Y bounds, then that many `f32` XZ
   pairs. Tag 1 is an inclusive range of two `u32` link user IDs. Each spec
   ends with a one-byte effect tag and a `u16` value. Tags 0, 1 and 2 clear
   flags, set flags and set area respectively.
4. A 32-byte SHA-256 footer over all preceding bytes. `DynamicLayer::digest`
   returns that digest.

The reader validates the version, checksum, specification order, geometry, state
bits and resource bounds. Unknown versions return
`Error::InvalidSnapshot(SnapshotError::WrongVersion)`. Tile and crowd snapshots
keep their existing formats.
