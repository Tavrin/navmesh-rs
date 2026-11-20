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

Use `Selector::Prism` for centroid selection. It tests the vertex centroid of
each ground polygon in the tile's original coordinates. Bake the same prism as
an area volume so the bake splits polygons at the boundary. Match that baked
area in the selector, and use `modifier_selection` to count selected polygons and
diagnose intersecting polygon bounds with centroids outside the prism.

Use `Selector::DetailPrism` to select a ground polygon if any of its Detour
detail triangles intersects the authored volume, including contact at a face,
edge or vertex. Ground polygons without detail triangles use a fan of their
perimeter vertices. A prism selects only surfaces that intersect its Y range,
even when another floor shares the same XZ footprint. Both prism selectors
accept a strictly convex XZ outline of 3 to 64 vertices in either winding,
inclusive Y bounds and an optional baked area filter. `Selector::Links` selects
off-mesh connections by their authored user ID.

The mesh applies enabled effects in ascending ID order over caller base values,
which start at the polygon's baked flags and area. On a selected polygon,
`NavMesh::set_poly_flags` sets the base mask and `NavMesh::set_poly_area` sets
the base area. A disabled spec still selects its polygons. Each flag effect sets
or clears only the bits in its mask. The last enabled effect on a bit wins.
Use `ClearFlags(u16::MAX)` for a gate that should block every standard filter,
and avoid a later modifier that sets the gate's bits. The last enabled area
effect wins. The getters return effective values and `baked_poly_state` reads
the original tile.

Replacing a spec preserves its enabled bit. Unregistering a spec removes its
effect and reapplies overlapping effects over the same base. Replacing or
clearing the entire layer replaces or discards its base edits as well. Legacy
tile-state restores apply to unmanaged polygons. Selected polygons and
polygons with saved base edits use the current layer's result.

The mesh applies its current layer during add, commit, restore and replacement,
including install jobs and tile-cache rebuilds. Prepared tiles can be built on
workers using the existing API. The owner thread selects and applies the
modifier state as it attaches the tile. Base edits are keyed by tile column,
layer, content hash and polygon index. They survive unload and reload of the
same content. Different baked bytes start from their own baked state or saved
edits for that content. Derived flags and areas stay out of baked bytes, content
hashes and ref layout provenance.

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
1. The reader validates checksums, ordering, geometry, state fields and resource
bounds. Unknown versions return
`Error::InvalidSnapshot(SnapshotError::WrongVersion)`. Tile and crowd snapshots
keep their existing formats.
