# navmesh-rs

A pure-Rust port of [recastnavigation](https://github.com/recastnavigation/recastnavigation)'s
Recast (navmesh generation), Detour (navmesh runtime and queries), and DetourCrowd
(agent simulation and local avoidance).

The port follows upstream function by function and is checked against it record by record: on
x86-64 Linux with default features, every Recast stage, the navmesh tile bytes and every Detour
query give the same bits as the C++ library at the pinned commit. See
[docs/VERIFICATION.md](docs/VERIFICATION.md) for how this is checked and what is covered.

## Install

```toml
[dependencies]
navmesh-rs = "0.1"
```

The library is `navmesh_rs`. Minimum Rust version: 1.88.

## Usage

```rust
use navmesh_rs::detour::{self, DefaultFilter, NavMesh, NavMeshCreateParams, NavMeshQuery};
use navmesh_rs::recast::{self, CompactHeightfield, ContourSet, Heightfield, PolyMesh, PolyMeshDetail};
use navmesh_rs::{Error, Workspace};

fn build_and_query(verts: &[f32], tris: &[i32]) -> Result<(), Error> {
    // Generation: one call per upstream stage (upstream demo settings).
    let mut ws = Workspace::new();
    let (bmin, bmax) = recast::calc_bounds(verts);
    let (w, h) = recast::calc_grid_size(&bmin, &bmax, 0.3);
    let mut hf = Heightfield::new(&mut ws, w, h, bmin, bmax, 0.3, 0.2)?;
    let mut areas = vec![0u8; tris.len() / 3];
    recast::mark_walkable_triangles(&mut ws, 45.0, verts, tris, &mut areas)?;
    hf.rasterize_triangles(&mut ws, verts, tris, &areas, 4)?;
    let mut chf = CompactHeightfield::build(&mut ws, 10, 4, &hf)?;
    recast::erode_walkable_area(&mut ws, 2, &mut chf)?;
    recast::build_distance_field(&mut ws, &mut chf)?;
    recast::build_regions(&mut ws, &mut chf, 0, 8, 20)?;
    let cset = ContourSet::build(&mut ws, &chf, 1.3, 40, recast::CONTOUR_TESS_WALL_EDGES)?;
    let pmesh = PolyMesh::build(&mut ws, &cset, 6)?;
    let dmesh = PolyMeshDetail::build(&mut ws, &pmesh, &chf, 1.8, 0.2)?;

    // Tile bytes in upstream's DNAV layout, loaded into a single-tile navmesh.
    let flags = vec![1u16; pmesh.npolys as usize];
    let tile = detour::create_nav_mesh_data(&NavMeshCreateParams {
        verts: &pmesh.verts,
        vert_count: pmesh.nverts,
        polys: &pmesh.polys,
        poly_flags: &flags,
        poly_areas: &pmesh.areas,
        poly_count: pmesh.npolys,
        nvp: pmesh.nvp,
        detail_meshes: &dmesh.meshes,
        detail_verts: &dmesh.verts,
        detail_verts_count: (dmesh.verts.len() / 3) as i32,
        detail_tris: &dmesh.tris,
        detail_tri_count: (dmesh.tris.len() / 4) as i32,
        bmin: pmesh.bmin,
        bmax: pmesh.bmax,
        walkable_height: 2.0,
        walkable_radius: 0.6,
        walkable_climb: 0.9,
        cs: 0.3,
        ch: 0.2,
        build_bv_tree: true,
        ..Default::default()
    })?;
    let (nav, _) = NavMesh::new_single(tile)?;

    // Queries.
    let mut query = NavMeshQuery::new(&nav, 2048)?;
    let filter = DefaultFilter::default();
    let ext = [2.0, 4.0, 2.0];
    let (a, pa, _) = query.find_nearest_poly(&[1.0, 0.0, 1.0], &ext, &filter)?;
    let (b, pb, _) = query.find_nearest_poly(&[9.0, 0.0, 9.0], &ext, &filter)?;
    let mut path = Vec::new();
    query.find_path(a, b, &pa, &pb, &filter, &mut path, 256)?;
    let mut corners = Vec::new();
    query.find_straight_path(&pa, &pb, &path, &mut corners, 64, 0)?;
    Ok(())
}
```

Tiled navmeshes use `NavMesh::new` with `NavMeshParams` and `add_tile` per tile. The crate
documentation lists every stage and query.

## Crowd

Enable the default `crowd` feature to manage agents over an existing navmesh.
Targets use polygon references from `NavMeshQuery::find_nearest_poly`.

```rust
use navmesh_rs::{Error, Workspace};
use navmesh_rs::crowd::{AgentParams, Crowd};
use navmesh_rs::detour::{DefaultFilter, NavMesh, NavMeshQuery};

fn simulate(nav: &NavMesh, start: [f32; 3], goal: [f32; 3]) -> Result<(), Error> {
    let mut ws = Workspace::new();
    let mut crowd = Crowd::new(&mut ws, nav, 64, 0.6)?;
    let agent = crowd.add_agent(&mut ws, nav, start, AgentParams::default())?;
    let query = NavMeshQuery::new(nav, 512)?;
    let (poly, target, _) = query.find_nearest_poly(&goal, &[2.0, 4.0, 2.0], &DefaultFilter::default())?;
    crowd.request_move_target(agent, poly, target)?;
    for _ in 0..300 {
        crowd.update(&mut ws, nav, 1.0 / 30.0)?;
        let position = crowd.agent(agent).ok_or(Error::InvalidRef)?.npos;
        // Use position to drive the caller's presentation.
        let _ = position;
    }
    crowd.remove_agent(agent)?;
    Ok(())
}
```

`Crowd` provides target and velocity requests, target resets, sixteen query filters,
eight avoidance parameter slots, and read-only agent state. `PathCorridor`,
`LocalBoundary`, `ObstacleAvoidanceQuery`, `ProximityGrid`, and `PathQueue` can also
be used independently. Agent handles name reusable slots. The mesh is borrowed only
during calls, allowing tile changes between ticks. An update error may leave earlier
phases applied; do not consume that tick as a successful simulation result.

## What it adds over the C++ library

- No C++ toolchain or bindings; builds for `wasm32` and for `no_std` targets (with `alloc`).
- Tile bytes from outside are validated before use: section sizes, every index and every
  reference are checked, and a malformed tile is rejected with a typed error instead of being
  read out of bounds. Upstream's own checks keep their order, so its verdicts are unchanged.
- Per-call limits on memory and work (`Limits`, `Workspace`); a call that exceeds them returns
  an error. Upstream asserts and crash paths become errors as well; the library does not panic
  on bad input and contains no `unsafe` code.
- 64-bit poly refs by default (upstream defaults to 32-bit), for large tiled worlds. Turn off
  the `polyref64` feature for upstream's 32-bit tile format.

## Features

| Feature | Default | Purpose |
|---|---|---|
| `recast` | yes | navmesh generation |
| `detour` | yes | tile data, navmesh, queries |
| `crowd` | yes | Agent simulation, corridors, avoidance and path queue; implies `detour`. |
| `std` | yes | floating-point functions from the platform libm |
| `libm` | no | floating-point functions from the `libm` crate; required without `std` |
| `polyref64` | yes | 64-bit poly refs; off gives upstream's 32-bit refs |

`no_std`: `default-features = false, features = ["libm", "recast", "detour", "crowd", "polyref64"]`.

## Status and limits

- Ported: Recast, Detour and DetourCrowd.
- Not ported yet: DetourTileCache (layers,
  obstacles, incremental rebuild), merging and copying polygon meshes, and big-endian tile data.
- The API follows upstream's stage functions; there is no single-call bake helper yet.
- Bit-exactness with the C++ library is checked on x86-64 Linux with glibc. On other
  platforms the results can differ in the last bit where upstream calls `cos`/`sin`
  (the walkable-slope threshold and avoidance sampling).
- Speed: the query set and the Recast stages take about 1.1 to 1.35 times the instructions of
  the C++ library on the test corpus; no optimisation work has been done yet.

## Licence

Licensed under either of [Apache License, Version 2.0](LICENSE-APACHE) or
[MIT license](LICENSE-MIT) at your option.

This crate is an altered, translated version of recastnavigation (Copyright (c) 2009 Mikko
Mononen and the recastnavigation contributors, zlib licence). It is not the original software;
the upstream licence is reproduced in [NOTICE](NOTICE).

Unless you explicitly state otherwise, any contribution intentionally submitted for inclusion in
this crate by you, as defined in the Apache-2.0 license, shall be dual licensed as above, without
any additional terms or conditions.
