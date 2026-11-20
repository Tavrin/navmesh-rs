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
Structural connectivity ignores area and cost filters. Resize an agent by
removing it and admitting it to the destination profile.

## Shared avoidance

Use `MixedProfileCrowdWorld` with a `ProfileMeshes` collection and the profile
IDs to admit. Supply a profile when you add an agent and query that profile
for each point goal. Admission and parameter updates check the agent's radius
and height against that profile's bake. Registered meshes share a coordinate
frame and stay bound to the crowd across compatible catalog revisions.

One fixed step gathers all agents in global slot order. ORCA and sampled
avoidance use one neighbour grid and every neighbour's own radius and height.
Corridors, walls, off-mesh traversal and per-profile change stamps use the
agent's bound mesh. The eight-slot path queue and the replan budget serve all
profiles in the same order on every thread count. An external traversal event
carries the profile ID with its mesh-local refs. Respond through
`complete_off_mesh` or `abort_off_mesh`.

Use `ProfileCrowd` or `ProfileCrowdWorld` for a single profile with its own
avoidance neighbourhood. Their existing entry points and save layouts still
apply.

## Gates, cache zones and platforms

With the `dynamic` feature, each profile mesh owns its own modifier layer.
Install it with `set_dynamic_layer` and toggle stable IDs with
`set_modifier_enabled`. To close a gate for several sizes, apply the same
command to each eligible profile before the next query or crowd tick. The
selected polygons can differ by profile. The mesh reapplies its layer as
tiles attach after a catalog update, reload or residency restore. Save the
[modifier record](DYNAMIC-MODIFIERS.md) separately for each profile and
register it before restoring the crowd. Catalog hashes describe baked tiles.

Version 1 profile catalogs describe ordinary layer-zero bake output, including
authored off-mesh connections. Tile-cache zones and generated runtime link
tiles use the native mesh API. A [tile-cache zone](TILE-CACHE-ZONES.md) for
each size requires layers built at that size's erosion radius and individual
cache and obstacle state.

For [moving platforms](MOVING-PLATFORMS.md), use `FrameSet::with_profiles`
and `add_profile_frame` to supply each frame's catalogs and meshes. Attach a
`FramedMixedProfileCrowdWorld` simulation to each frame that riders can use.
A rider keeps its profile and stable rider ID when crossing a dock. Both
landings are resolved on that rider's profile meshes. Riders on the same
deck share avoidance. Across open seams, nearby agents are moving avoidance
obstacles, as in the native framed crowd. Supply platform poses and update
dock availability before stepping. Attach the rider's profile in each frame
along its route.

## On-disk bindings

All integer fields are little-endian. Version 1 supports ordinary baked DNAV
at layer zero. The catalog header has magic `NPCG`, version (u32), map ID
(16 bytes), revision (u64), profile ID (u32), a SHA-256 descriptor digest and
entry count (u64). The descriptor covers the effective bake settings, original
float bits, full grid, reference width and layout, mesh options and limits.
It also binds nonzero spare link reservations. A zero reservation uses the
same descriptor as a catalog created before the option was introduced.
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
still match its mesh when saved, and a crowd save fails with `MeshChanged` when
the mesh changed after the last step. The embedded `DCRW` uses zero in the
search epoch and last-step epoch fields, and restore rebinds both to the
restored mesh's epochs.

The profile `NPCW` and framed `DCFR` wrappers use the ordinary `DCRW`
reader and writer. The original base layout (version 1) loads with default
arrival and traversal settings. The extended layout (version 4) adds the
runtime-link epoch and the agents' arrival and external traversal state.
The reader also accepts the two historical layouts, version 2 (arrival state
only) and version 3 (runtime-link and traversal state only). Version 5 adds
deferred replanning state and the mesh epoch of the last step.

Mixed crowds save as `NPCM` version 1. The header carries a u64 profile count,
followed by each profile ID (u32) in ascending order and the SHA-256 of its
`NPRS` record. A `DCRW` version 6 payload follows, with a u32 profile ID before
each occupied agent's state and after each queue request handle (zero in a
free request slot). The remaining fields follow version 5. Save after stepping
following mesh edits. Restore residency and modifiers for all registered
profiles before restoring the crowd. The live sliced search is rebound to
the reconstructed epoch of its request's mesh.

Framed mixed worlds save as `NPFM` version 1. This record binds the matching
`DNFR` frame snapshot and embeds each frame's `NPCM` payload along with stable
rider IDs, local handles, destinations and offered dock wait costs. Save and
restore the frame snapshot separately. Both new wrappers end with a SHA-256
checksum of their preceding bytes.

Unknown inner versions return
`Error::InvalidSnapshot(SnapshotError::WrongVersion)`, through
`ProfileError::Navigation` for a profile restore. Unknown catalog or profile
wrapper versions return `ProfileError::UnsupportedVersion`.
