# Agent-size bake profiles

Use `bake::profiles` when a map has more than one authored agent size. Assign
each size a stable `ProfileId`, and create a `ProfileBakeSpec` from its own
`BakeConfig` and `GridLayout`. World-space dimensions and voxel dimensions are
explicit. A radius of zero is valid for baking and querying. Crowds require a
positive radius.

## Baking and publishing

1. Use the same immutable `GeometrySource` and `ProfileRevision` for the jobs
   belonging to one map revision. Run `ProfileBakeSpec::bake` for each profile.
   Each job uses the ordinary bake pipeline and delivers its own row-major
   stream. Iterate profiles in ascending ID order for a canonical combined
   stream. Drain each job regularly to keep pending output bounded.
2. Use `GridLayout::new` or `GridLayout::from_input` for the native halo. For an
   explicitly authored halo, use `with_halo_cells` with each profile's config.
   It checks the radius, link-extraction reach and padded grid limits.
3. Create a `ProfileCatalogBuilder` with the profile's runtime layout options.
   Insert every delivered result, including explicit empty columns. Store
   nonempty DNAV payloads under the SHA-256 of their original bytes. The same
   blob may serve several addresses if their hashes match.
4. Keep a manifest per profile for dirty selection. For a compatible edit, use
   `ProfileCatalogBuilder::revise` to carry forward unchanged entries. Supply
   changed and newly empty columns to the new builder. A changed bake or runtime
   layout requires a new catalog and mesh.

## Loading and running

Open the catalog with the expected revision, profile specification and runtime
layout. For an external asset store, compare the catalog digest against the digest
in the trusted map index. Select a tile through its catalog, fetch its raw payload,
and use `ProfileCatalog::prepare` before handing it to `ProfileMesh::commit`.
Only the matching catalog revision accepts that prepared result. The
installation also checks the validated layer and column, the prepared byte hash
and polygon count. Empty entries carry no bytes. Missing entries have no
declared availability.

A tile can be unloaded, retaining its slot and salt. Reload the original raw
asset through its catalog. Call `forget_unloaded` to reuse a sequential slot
for another column and make the old refs stale. Prepared bytes may include a reference-layout footer.
For a compatible update, adopt the new catalog, which retires removed and empty
columns. Old resident versions of changed nonempty columns remain queryable
until they are replaced. Install the changed assets in an owner-controlled phase
if callers need to observe them together. An identical replacement preserves
the epoch and refs. Increased grid dimensions must still fit the configured
keyed slot policy if one is used.

Use `ProfileMeshes` to select a mesh by ID. A query borrows one mesh for its
lifetime, including sliced searches. A polygon or tile ref belongs to the mesh
instance that issued it, even when another profile produces the same numeric
bits. Connectivity graphs, crowd updates and crowd targets check that binding.
Structural connectivity ignores area and cost filters. Each crowd has its own
agents and avoidance neighbours. Resize an agent by removing it and admitting
it to the destination profile.

## On-disk bindings

All integer fields are little-endian. Version 1 supports ordinary baked DNAV
at layer zero. The catalog header has magic `NPCG`, version (u32), map ID
(16 bytes), revision (u64), profile ID (u32), a SHA-256 descriptor digest and
entry count (u64). The descriptor covers the effective bake settings, original
float bits, full grid, reference width and layout, mesh options and limits.
Each entry contains x and z (i32), a presence byte, two SHA-256 hashes (raw and
prepared) and polygon count (u32). Empty entries have a zero payload. Entries
are strictly ordered by z, then x. The catalog identity is the SHA-256 of the
whole encoded catalog. An address carries this identity along with the column.

Save residency before saving polygon refs or a crowd world. Save operations
require every occupied or parked slot to match the current catalog. The
wrappers start with magic `NPRS` (residency), `NPRF` (polygon ref) or `NPCW`
(crowd world), version (u32) and catalog digest (32 bytes), followed by a
payload and a SHA-256 checksum of the preceding bytes. Polygon refs and crowd
worlds prefix the payload with the exact residency-record digest, so sequential
slot assignments from different streaming schedules cannot be mixed. Checksums
check integrity. Obtain persisted records from the application's save store.

On restore, open an empty mesh, restore its residency, then commit the required
tiles. Saved polygon refs are rebound to the new mesh instance. Restore the
crowd world after loading its required tiles. An active crowd search must
still match its mesh when saved. Its epoch is rebound to the restored mesh. Unknown catalog or wrapper versions
return `ProfileError::UnsupportedVersion`.
