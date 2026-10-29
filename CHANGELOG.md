# Changelog

All notable changes to this project are documented in this file. The format follows
[Keep a Changelog](https://keepachangelog.com/en/1.1.0/), and the project uses
[Semantic Versioning](https://semver.org/).

## [Unreleased]

### Added

- DetourCrowd: crowd manager, corridor maintenance, local walls, proximity grid,
  sliced path queue, steering, separation, grid and adaptive avoidance, and off-mesh
  animation; full per-tick trajectory and corridor comparison with the reference.

- Recast port: every generation stage from walkable-triangle marking to the detail mesh,
  the three partitions (watershed, monotone, layers), area volumes and heightfield layers.
- Detour port: tile data creation in upstream's `DNAV` v7 layout (32- and 64-bit poly refs),
  the navmesh with a validating tile reader, off-mesh connections, and every navmesh query
  (nearest polygon, plain/sliced/partial path finding, straight paths, raycasts, distance to
  wall, move along surface, polygon searches, wall segments, random points, heights).
- `Workspace` and `Limits` for per-call memory and work limits; typed `Error` values instead
  of upstream's asserts and status codes.
- `no_std` + `alloc` support and `wasm32` builds.
