# Changelog

All notable changes to this project are documented in this file. The format follows
[Keep a Changelog](https://keepachangelog.com/en/1.1.0/), and the project uses
[Semantic Versioning](https://semver.org/).

## [Unreleased]

## [0.1.1] - 2026-10-30

### Added

- Crowd simulation: corridor maintenance, steering, separation, local avoidance,
  path queues and off-mesh animation.
- Tile cache with cylinder, box and oriented-box dynamic obstacles and incremental
  navmesh rebuilding.
- A pluggable layer compressor trait with raw storage provided by default.
- Crowd and tile-cache seed modes for the fuzz corpus generator.
- Non-exhaustive crowd parameters and state enums, with defaults and avoidance
  quality presets for construction.

### Fixed

- Clear animation, movement targets and cached state when reusing crowd slots.
- Use `as_chunks` for complete 16-byte records in the crowd fuzz target, fixing
  Clippy on current toolchains.
- Gate integration tests and the build example on their required features.

### Deliberate differences from C++

- Validate inputs and references, report exhausted capacities and work budgets,
  reset reused slots, require positive agent speeds and reject out-of-range cells.
- Define adaptive avoidance for even division counts, where upstream reads
  uninitialized samples; no C++ parity is claimed for those counts.
- Validate tile-cache layers and codec output, keep tile membership stable during
  updates, reject touched-layer overflow and retain all queued rebuilds.
- Complete zero-touch obstacle lifecycles, charge allocation and algorithm work,
  and retain failed rebuilds for retry without rolling back completed work.
- Reserve full vertex triples in tile-cache removal instead of reproducing an
  upstream buffer overrun. See [verification](docs/VERIFICATION.md) for the full
  differences and parity boundaries.

## [0.1.0]

### Added

- Recast navmesh generation, including all three partitions, area volumes and
  heightfield layers.
- Detour tile creation, tiled navmeshes, off-mesh connections and navmesh queries
  with 32-bit or 64-bit polygon references.
- Per-call memory and work limits with typed errors.
- `no_std` with `alloc` and `wasm32` support.

[Unreleased]: https://github.com/Tavrin/navmesh-rs/compare/v0.1.1...HEAD
[0.1.1]: https://github.com/Tavrin/navmesh-rs/compare/v0.1.0...v0.1.1
[0.1.0]: https://github.com/Tavrin/navmesh-rs/releases/tag/v0.1.0
