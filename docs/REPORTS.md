# Diagnostic reports

All top-level reports have `schema_version = 1`. Enable `serde` for serialization;
without that feature the same plain structs work with `no_std` and `alloc`.
Serialization names are the public Rust field and variant names. Histograms
serialize as 64-element arrays. Deserialization of arbitrary report data is not
an input API.

Set `bake::JobOptions::reports = true` before constructing a `BakeJob`. Its
`report()` accumulates completed tiles and the first failed tile in row-major
order, even after output is drained with `next_ready`. Skipped and unstarted
tiles are absent. `limits_hit` preserves the error rather than asserting that a
failed measurement is a complete result. Reporting is optional and its scans,
temporary mesh and graph are charged to the tile's work/byte allowance. Temporary
storage is conservatively charged at 16 times the output tile byte length;
retained summaries also reduce the next admission's aggregate allowance.

Bake report keys use the job's `bake::TileKey { x, z }`. Diagnostic unloaded
frontiers use the streaming API's `detour::TileKey { x, y }`, with `y` representing
world z.

Coverage counts interior raster columns with a surviving walkable region, once
per xz column regardless of overlapping floors. The denominator includes empty
interior columns. Filter transitions and removed-region counts cover the full
bordered working field. `low_hanging` counts promotions: that filter does not
remove walkability. The other two counters count cleared spans, in filter order.
Region removal counts discarded pre-merge region IDs, not spans. Area histograms,
polygon/vertex counts and local islands describe DNAV output, including owned
off-mesh polygons. Ground polygons alone contribute projected xz area; overlapping
layers contribute independently. A failed tile's counters only describe the
stages completed before its failure. `world` stays `None` until the caller attaches
reachability measured from the installed mesh; a bake cannot know residency.

Build `connectivity::Connectivity` from the runtime mesh with a `Budget`. After
committing, unloading or removing tiles, call `update`. It scans resident links
and uses union-find to relabel only affected weak components; removal recomputes
the entire old component to handle splits. Updates are transactional on budget
refusal. Each allocated slot, polygon, link, component member and union visit costs work. The
algorithm has storage proportional to the resident graph, independent of the
keyed grid's logical extent. Queries should call `is_current` before consuming
labels; `diagnose_path` does this automatically.

For hierarchy consumers, `island(ref)` is the world weak component and
`tile_island(ref)` is the tile-local weak component. Both use the minimum member
`PolyRef`, with no filter applied. An owner token rejects snapshots from another mesh, including a clone, while
surviving moves of the same mesh. The validity key is the mesh identity,
structural epoch and global off-mesh-link mode. Global-link mode changes are
observable through `is_current` even though that existing setter does not advance
the streaming epoch. A component ID may change after a merge or split. Tile
reports use a standalone tile's local reference namespace: obtain runtime labels
from `Connectivity` after installation rather than reusing those numeric IDs.

Weak components ignore link direction when grouping, including one-way off-mesh
connections. Shared labels are a necessary condition for a route, not sufficient.
`reachable_from(seeds)` follows directed links exactly. `reachability(threshold)`
reports resident island count, polygon-weighted largest share, small-island count
and geometry summaries. Empty graphs have zero shares. Neither report claims
coverage of unloaded or filtered geometry.

`diagnose_path` shares one allowance across epoch/residency checks, filtered endpoint snaps with unfiltered fallback,
endpoint filter checks, filtered search, all-pass search and explanation scans.
It returns a complete corridor on success, otherwise a typed `NoPathReason`.
A cheap weak-component check rejects fully resident disconnections before searching.
Resource exhaustion never becomes disconnection. Unloaded frontiers take
precedence over disconnection; they identify load candidates, not a guarantee
that loading creates a route. Endpoint snaps use the supplied nearest-poly box:
an empty box produces an off-mesh reason with no nearest candidate. A filter cut
requires a complete all-pass corridor and lists up to 64 excluded polygons,
sorted by reference; `blocking_truncated` reports the cap. Filters must return consistent inclusion answers throughout a diagnosis. Reverse traversal of a
one-way connection can produce `DirectedUnreachable` within a weak component.

Decisions: weak union labels provide a shared cheap structural index; directed
flood fill and searches retain directionality. Strong-component labels would
make forward one-way paths cross different labels. Reports remain explicit
snapshots owned by the caller rather than interior mutable mesh observers.
Changing label semantics later requires a report schema change; adding new
non-exhaustive reasons or fields is additive.

Validation cases are in `tests/reports.rs`. The `connectivity_ops` fuzz target
compares incremental labels and seed coverage against independent weak and
directed flood fills after bounded commit/unload/remove/reload sequences.
