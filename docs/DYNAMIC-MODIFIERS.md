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
    nav.try_set_dynamic_layer(Some(layer))?;
    Ok(gate)
}

fn open_gate(nav: &mut NavMesh, gate: ModifierId) -> Result<(), Error> {
    nav.set_modifier_enabled(gate, false)?;
    Ok(())
}
```

Use `Selector::Prism` for centroid selection. It tests the vertex centroid of
each ground polygon in the tile's original coordinates. Bake the same prism as
an area volume so the bake splits polygons at the boundary. Match that baked
area in the selector, and use `modifier_selection` to count selected polygons and
diagnose intersecting polygon bounds with centroids outside the prism.

Use `Selector::DetailPrism` to select a ground polygon if any of its Detour
detail triangles intersects the authored volume, including contact at a face,
edge or vertex. The separating-axis test treats projected gaps within a
roundoff tolerance as contact. On each axis the tolerance is 32 times the
machine epsilon for `f64` times the sum of the absolute cross-product terms
weighted by the corresponding coordinate spans relative to the first triangle
vertex. The initial bounds check uses exact comparisons of the stored
coordinates, including the Y bounds. Ground polygons without detail triangles
use a fan of their perimeter vertices. Both prism selectors
accept a strictly convex XZ outline of 3 to 64 vertices in either winding,
inclusive Y bounds and an optional baked area filter. `Selector::Links` selects
off-mesh connections by their authored user ID.

The mesh applies enabled effects in ascending ID order over caller base values.
When a modifier first selects a polygon, its current caller flags and area
become the base. A disabled spec still selects its polygons. Enabled flag effects
own the bits in their masks and `NavMesh::set_poly_flags` preserves the previous
base value of every owned bit. An enabled area effect owns the whole area field,
so `NavMesh::set_poly_area` preserves the previous base area until the area
effect is disabled or removed. The last enabled effect on a bit or on the area wins.
Use `ClearFlags(u16::MAX)` for a gate that should block every standard filter,
and avoid a later modifier that sets the gate's bits. `poly_flags` and
`poly_area` return effective values, while
`poly_base_flags` and `poly_base_area` read the caller values before effects.
`baked_poly_state` reads the original tile.

Replacing a spec preserves its enabled bit. Unregistering a spec removes its
effect and reapplies overlapping effects over the same base. Replacing or
clearing the entire layer replaces or discards its base edits as well.
`try_set_dynamic_layer` checks the resulting snapshot against the supplied
layer's byte limit before changing the mesh. The infallible `set_dynamic_layer`
also keeps newly captured edits when they grow the snapshot beyond that limit.
Legacy tile-state restores apply to unmanaged polygons. Newly written unload
blobs mark directly edited, unmanaged polygons in the existing padding byte.
On `commit_with_state`, that mark allows a newly registered selector to capture
the caller state, with explicitly saved base fields taking precedence.
Unmarked entries on managed polygons use the current layer's result.

The mesh applies its current layer during add, commit, restore and replacement,
including install jobs and tile-cache rebuilds. Prepared tiles can be built on
workers using the existing API. The owner thread selects and applies the
modifier state as it attaches the tile. Base edits are keyed by tile column,
layer, content hash and polygon index. They survive unload and reload of the
same content. Permanently removing a tile, replacing it with different content,
or forgetting its unloaded reservation drops edits for the retired content.
Use `DynamicLayer::poly_base_edits` to list saved overrides and
`NavMesh::remove_poly_base_edit` to remove one and reapply active effects.
Derived flags and areas stay out of baked bytes, content hashes and ref
layout provenance.

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
tiles. To also save caller base edits or to save a standalone modifier layer,
use `DynamicLayer::to_bytes` and `DynamicLayer::from_bytes`. To preserve tile
references across restart, also save the navmesh residency record and restore
it before committing any tiles.

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

### DNDY version 2

Version 2 retains the header, bitset and spec layout, with version 2 in the
header and selector tag 2 for a detail prism. Its payload matches tag 0.
After the specs and before the SHA-256 footer, version 2 adds a `u32` count
of base edits and that many 52-byte entries. Each entry contains, in order:

- Three `i32` values for tile X, tile Y and layer.
- The tile's 32-byte content hash and a `u32` polygon index.
- A `u16` base flag mask, a `u8` base area (255 for baked area), and a
  `u8` flag-mask presence byte (0 or 1). An absent flag mask is encoded as
  zero and uses the baked mask.

Entries are strictly ordered by tile X, tile Y, layer, hash bytes and polygon
index. Each entry supplies at least one base value. The writer emits version 2
when the layer has a detail selector or a base edit. Otherwise it emits version
1. A layer loaded from version 1 now accepts `set_poly_flags` and
`set_poly_area` on selected polygons, subject to enabled effects' ownership.
Saving after an edit that creates a base override emits version 2. The reader
validates checksums, ordering, geometry, state fields and resource
bounds. Unknown versions return
`Error::InvalidSnapshot(SnapshotError::WrongVersion)`. Tile and crowd snapshots
keep their existing formats.
