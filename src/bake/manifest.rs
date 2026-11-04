//! Tile grid layout, input hashing and the persisted bake manifest (not in recastnavigation).

use alloc::collections::BTreeSet;
use alloc::vec::Vec;
use core::cmp::Ordering;

use super::source::{OffMeshSink, TriangleSink, VolumeSink};
use super::{BakeConfig, Input, Partition};
use crate::common::{floorf, fmax, fmin};
use crate::detour::NavMeshParams;
use crate::recast::{self, Config};
use crate::{Error, Limits, Workspace};

/// Bumped whenever a crate change alters the bytes the bake pipeline produces for the
/// same input, so manifests written by an older pipeline never claim a tile is current.
pub const BAKE_REVISION: u32 = 1;

/// Version of the [`BakeManifest`] byte format.
pub const MANIFEST_VERSION: u32 = 1;

const MANIFEST_MAGIC: [u8; 4] = *b"NMBM";
const HEADER_LEN: usize = 4 + 4 + 8 + 9 * 4 + 8;
const ENTRY_LEN: usize = 4 + 4 + 8 + 1 + 8;

/// 64-bit FNV-1a, the digest of inputs, configurations and tile bytes.
#[derive(Clone, Copy, Debug)]
pub(super) struct Fnv64(u64);

impl Fnv64 {
    pub(super) const fn new() -> Self {
        Self(0xcbf2_9ce4_8422_2325)
    }
    pub(super) fn bytes(&mut self, bytes: &[u8]) {
        for &b in bytes {
            self.0 ^= u64::from(b);
            self.0 = self.0.wrapping_mul(0x0000_0100_0000_01b3);
        }
    }
    pub(super) fn u8(&mut self, v: u8) {
        self.bytes(&[v]);
    }
    pub(super) fn u16(&mut self, v: u16) {
        self.bytes(&v.to_le_bytes());
    }
    pub(super) fn u32(&mut self, v: u32) {
        self.bytes(&v.to_le_bytes());
    }
    pub(super) fn i32(&mut self, v: i32) {
        self.bytes(&v.to_le_bytes());
    }
    pub(super) fn u64(&mut self, v: u64) {
        self.bytes(&v.to_le_bytes());
    }
    pub(super) fn f32(&mut self, v: f32) {
        self.u32(v.to_bits());
    }
    pub(super) const fn finish(self) -> u64 {
        self.0
    }
}

/// Digest of tile bytes, as stored in [`ManifestEntry::output_hash`].
pub(super) fn digest_bytes(bytes: &[u8]) -> u64 {
    let mut h = Fnv64::new();
    h.bytes(bytes);
    h.finish()
}

/// A tile column of the grid; layers are per tile output (not in recastnavigation).
///
/// Ordered row-major, by `z` then `x`, the order of [`super::tiles`] and of every
/// manifest.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash)]
pub struct TileKey {
    /// Column index along x.
    pub x: i32,
    /// Column index along z (Detour's tile y).
    pub z: i32,
}

impl Ord for TileKey {
    fn cmp(&self, other: &Self) -> Ordering {
        (self.z, self.x).cmp(&(other.z, other.x))
    }
}
impl PartialOrd for TileKey {
    fn partial_cmp(&self, other: &Self) -> Option<Ordering> {
        Some(self.cmp(other))
    }
}

/// The world grid a bake job and a manifest refer to (not in recastnavigation).
///
/// [`tiles`](super::tiles) derives its grid from the bounds of the input mesh; a
/// bake from a [`GeometrySource`](super::GeometrySource) has no whole-world bounds, so
/// the caller states them. The layout, together with the [`BakeConfig`], fixes the
/// bytes of every tile: change the origin, the height range or the cell settings and
/// every tile changes, which the manifest's `config_hash` records. Growing the grid
/// (`tiles_x`, `tiles_z`) does not.
#[non_exhaustive]
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct GridLayout {
    /// World position of the minimum corner of tile `(0, 0)`; `origin[1]` is the
    /// bottom of every tile's heightfield.
    pub origin: [f32; 3],
    /// Top of every tile's heightfield.
    pub max_y: f32,
    /// Number of tile columns along x.
    pub tiles_x: i32,
    /// Number of tile columns along z.
    pub tiles_z: i32,
    /// World size of one tile, `tile_size * cs`.
    pub tile_extent: f32,
    /// World width of the border around a tile that its heightfield also covers.
    pub halo: f32,
    /// The border in cells: at least `walkable_radius + 3`.
    pub halo_cells: i32,
}

fn bake_error(message: &'static str) -> Error {
    Error::InvalidParameter(message)
}

impl GridLayout {
    /// A layout with the border [`tiles`](super::tiles) uses (`walkable_radius + 3` cells).
    ///
    /// `origin` is the world minimum of tile `(0, 0)` and `max_y` the top of the
    /// heightfields; the grid has `tiles_x` by `tiles_z` tiles.
    ///
    /// # Errors
    /// [`Error::InvalidParameter`] for an invalid `cfg`, non-finite bounds, a height
    /// range that is not above `origin[1]`, or fewer than one tile per axis.
    pub fn new(
        origin: [f32; 3],
        max_y: f32,
        tiles_x: i32,
        tiles_z: i32,
        cfg: &BakeConfig,
    ) -> Result<Self, Error> {
        super::validate_config(cfg)?;
        let halo_cells = cfg
            .recast
            .walkable_radius
            .checked_add(3)
            .ok_or(Error::SizeOverflow)?;
        let layout = Self {
            origin,
            max_y,
            tiles_x,
            tiles_z,
            tile_extent: cfg.recast.tile_size as f32 * cfg.recast.cs,
            halo: halo_cells as f32 * cfg.recast.cs,
            halo_cells,
        };
        layout.validate(cfg)?;
        Ok(layout)
    }

    /// The layout [`tiles`](super::tiles) would use for `input`, so a job over
    /// `input` returns the same tiles.
    ///
    /// # Errors
    /// As [`GridLayout::new`], plus [`Error::InvalidInput`] for an empty or non-finite mesh.
    pub fn from_input(input: &Input<'_>, cfg: &BakeConfig) -> Result<Self, Error> {
        super::validate_config(cfg)?;
        if input.verts.is_empty() || input.verts.iter().any(|v| !v.is_finite()) {
            return Err(Error::InvalidInput("empty or non-finite vertices"));
        }
        let (bmin, bmax) = recast::calc_bounds(input.verts);
        let ts = cfg.recast.tile_size;
        let cells = |lo: f32, hi: f32| -> Result<i32, Error> {
            let v = (hi - lo) / cfg.recast.cs + 0.5;
            if !v.is_finite() || v < 0.0 || f64::from(v) > f64::from(i32::MAX) {
                return Err(Error::SizeOverflow);
            }
            let g = v as i32; // finite, nonnegative and in range; truncation matches rcCalcGridSize
            Ok(g.checked_add(ts - 1).ok_or(Error::SizeOverflow)? / ts)
        };
        let (tx, tz) = (cells(bmin[0], bmax[0])?, cells(bmin[2], bmax[2])?);
        Self::new(bmin, bmax[1], tx, tz, cfg)
    }

    /// Checks the layout against a bake configuration.
    pub(super) fn validate(&self, cfg: &BakeConfig) -> Result<(), Error> {
        self.wellformed()?;
        let c = &cfg.recast;
        let border_ok = self.halo_cells >= c.walkable_radius.saturating_add(3);
        let extent = c.tile_size as f32 * c.cs;
        if !border_ok
            || self.tile_extent.to_bits() != extent.to_bits()
            || self.halo.to_bits() != (self.halo_cells as f32 * c.cs).to_bits()
        {
            return Err(bake_error(
                "grid layout does not match the bake configuration",
            ));
        }
        let dim = c
            .tile_size
            .checked_add(self.halo_cells.checked_mul(2).ok_or(Error::SizeOverflow)?)
            .ok_or(Error::SizeOverflow)?;
        if dim > i32::from(i16::MAX) {
            return Err(Error::FormatLimit("tile cell dimensions"));
        }
        if !((self.max_y - self.origin[1]) / c.ch).is_finite()
            || !(self.max_y + c.walkable_height as f32 * c.ch).is_finite()
        {
            return Err(Error::SizeOverflow);
        }
        Ok(())
    }

    /// Checks the layout on its own (a manifest read from bytes has no configuration).
    pub(super) fn wellformed(&self) -> Result<(), Error> {
        let finite = self.origin.iter().all(|v| v.is_finite())
            && self.max_y.is_finite()
            && self.tile_extent.is_finite()
            && self.halo.is_finite();
        if !finite
            || self.tiles_x < 1
            || self.tiles_z < 1
            || self.tiles_x.checked_mul(self.tiles_z).is_none()
            || self.tile_extent <= 0.0
            || self.halo < 0.0
            || self.halo_cells < 0
            || self.max_y < self.origin[1]
        {
            return Err(bake_error("grid layout"));
        }
        Ok(())
    }

    /// The navmesh layout that holds this grid's tiles: tile `(x, z)` goes to the slot
    /// keyed by its coordinates.
    ///
    /// `max_tiles` is the residency cap, the most tiles loaded at once, and `max_polys`
    /// the most polygons in one tile. Both are the caller's choice (the whole grid, or a
    /// streaming window); only the poly-ref bit layout bounds them
    /// (see [`NavMesh::with_limits`](crate::detour::NavMesh::with_limits)).
    pub fn nav_mesh_params(&self, max_tiles: i32, max_polys: i32) -> NavMeshParams {
        NavMeshParams {
            orig: self.origin,
            tile_width: self.tile_extent,
            tile_height: self.tile_extent,
            max_tiles,
            max_polys,
        }
    }

    /// Whether `key` is a tile of the grid.
    pub fn contains(&self, key: TileKey) -> bool {
        (0..self.tiles_x).contains(&key.x) && (0..self.tiles_z).contains(&key.z)
    }

    /// Every tile of the grid in row-major order.
    pub fn keys(&self) -> Vec<TileKey> {
        let mut out = Vec::new();
        for z in 0..self.tiles_z.max(0) {
            for x in 0..self.tiles_x.max(0) {
                out.push(TileKey { x, z });
            }
        }
        out
    }

    /// The tile's bordered column: the box a source is asked about, and the box a
    /// tile's heightfield covers. The y range is `[origin[1], max_y]`.
    pub fn tile_bounds(&self, key: TileKey) -> ([f32; 3], [f32; 3]) {
        let (tcs, b) = (self.tile_extent, self.halo);
        (
            [
                self.origin[0] + key.x as f32 * tcs - b,
                self.origin[1],
                self.origin[2] + key.z as f32 * tcs - b,
            ],
            [
                self.origin[0] + (key.x + 1) as f32 * tcs + b,
                self.max_y,
                self.origin[2] + (key.z + 1) as f32 * tcs + b,
            ],
        )
    }

    /// The Recast configuration of one tile, as [`tiles`](super::tiles) builds it.
    pub(super) fn tile_config(&self, cfg: &BakeConfig, key: TileKey) -> Result<Config, Error> {
        let (bmin, bmax) = self.tile_bounds(key);
        let dim = cfg
            .recast
            .tile_size
            .checked_add(self.halo_cells.checked_mul(2).ok_or(Error::SizeOverflow)?)
            .ok_or(Error::SizeOverflow)?;
        let mut c = cfg.recast;
        c.border_size = self.halo_cells;
        c.width = dim;
        c.height = dim;
        c.bmin = bmin;
        c.bmax = bmax;
        if bmin.iter().chain(&bmax).any(|v| !v.is_finite()) {
            return Err(Error::SizeOverflow);
        }
        Ok(c)
    }

    fn hash_into(&self, h: &mut Fnv64) {
        for v in self.origin {
            h.f32(v);
        }
        h.f32(self.max_y);
        h.f32(self.tile_extent);
        h.f32(self.halo);
        h.i32(self.halo_cells);
    }
}

/// One tile of a [`BakeManifest`].
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct ManifestEntry {
    /// The tile.
    pub key: TileKey,
    /// Digest of the configuration, the layout and every triangle, volume and
    /// off-mesh connection in the tile's bordered column, in source order.
    pub input_hash: u64,
    /// Digest of the tile's DNAV bytes; `None` when the tile has no polygons.
    pub output_hash: Option<u64>,
}

/// What a set of baked tiles was built from, kept beside the tiles (not in recastnavigation).
///
/// A caller persists the manifest next to the tile bytes. After an edit, it asks
/// [`dirty_from_boxes`](Self::dirty_from_boxes) which tiles the edited boxes touch,
/// then [`BakeJob::skip_unchanged`](super::BakeJob::skip_unchanged) drops those whose
/// input did not change after all.
#[non_exhaustive]
#[derive(Clone, Debug, PartialEq)]
pub struct BakeManifest {
    /// Byte format version, [`MANIFEST_VERSION`].
    pub version: u32,
    /// Digest of the [`BakeConfig`], the layout and [`BAKE_REVISION`].
    pub config_hash: u64,
    /// The grid the entries refer to.
    pub grid: GridLayout,
    /// One entry per baked tile, sorted by `(z, x)`.
    pub entries: Vec<ManifestEntry>,
}

impl BakeManifest {
    /// An empty manifest for `grid` baked with `cfg`.
    ///
    /// # Errors
    /// Those of [`GridLayout::new`] when `grid` does not fit `cfg`.
    pub fn new(cfg: &BakeConfig, grid: &GridLayout) -> Result<Self, Error> {
        grid.validate(cfg)?;
        Ok(Self {
            version: MANIFEST_VERSION,
            config_hash: Self::config_hash(cfg, grid),
            grid: *grid,
            entries: Vec::new(),
        })
    }

    /// The digest that decides whether two bakes can share tiles: every setting of
    /// `cfg` that changes tile bytes, the layout (not its size) and [`BAKE_REVISION`].
    /// Limits and `max_total_bytes` are left out, as they never change a successful tile.
    pub fn config_hash(cfg: &BakeConfig, grid: &GridLayout) -> u64 {
        // Exhaustive destructuring: a new field must be classified here.
        let BakeConfig {
            recast,
            partition,
            filter_low_hanging,
            filter_ledge,
            filter_low_height,
            walkable_height,
            walkable_radius,
            walkable_climb,
            tile_limits: _,
            max_total_bytes: _,
            strict_triangulation,
        } = *cfg;
        let Config {
            width: _,
            height: _,
            tile_size,
            border_size: _,
            cs,
            ch,
            bmin: _,
            bmax: _,
            walkable_slope_angle,
            walkable_height: cells_height,
            walkable_climb: cells_climb,
            walkable_radius: cells_radius,
            max_edge_len,
            max_simplification_error,
            min_region_area,
            merge_region_area,
            max_verts_per_poly,
            detail_sample_dist,
            detail_sample_max_error,
        } = recast;
        let mut h = Fnv64::new();
        h.u32(BAKE_REVISION);
        h.i32(tile_size);
        h.f32(cs);
        h.f32(ch);
        h.f32(walkable_slope_angle);
        h.i32(cells_height);
        h.i32(cells_climb);
        h.i32(cells_radius);
        h.i32(max_edge_len);
        h.f32(max_simplification_error);
        h.i32(min_region_area);
        h.i32(merge_region_area);
        h.i32(max_verts_per_poly);
        h.f32(detail_sample_dist);
        h.f32(detail_sample_max_error);
        h.u8(match partition {
            Partition::Watershed => 0,
            Partition::Monotone => 1,
            Partition::Layers => 2,
        });
        h.u8(u8::from(filter_low_hanging));
        h.u8(u8::from(filter_ledge));
        h.u8(u8::from(filter_low_height));
        h.f32(walkable_height);
        h.f32(walkable_radius);
        h.f32(walkable_climb);
        h.u8(u8::from(strict_triangulation));
        grid.hash_into(&mut h);
        h.finish()
    }

    /// The entry of `key`, if the manifest has one.
    pub fn entry(&self, key: TileKey) -> Option<&ManifestEntry> {
        self.entries
            .binary_search_by(|e| e.key.cmp(&key))
            .ok()
            .map(|i| &self.entries[i])
    }

    /// Replaces or inserts `delta`'s entries (a job's
    /// [`manifest_delta`](super::BakeJob::manifest_delta)), keeping the order.
    ///
    /// # Errors
    /// [`Error::InvalidInput`] if an entry names a tile outside the grid; the manifest
    /// is unchanged.
    pub fn apply(&mut self, delta: &[ManifestEntry]) -> Result<(), Error> {
        if delta.iter().any(|e| !self.grid.contains(e.key)) {
            return Err(Error::InvalidInput("manifest entry outside the grid"));
        }
        let mut sorted: Vec<ManifestEntry> = delta.to_vec();
        sorted.sort_by_key(|e| e.key);
        // The last entry for a key wins, as when applied one by one.
        let mut deduped: Vec<ManifestEntry> = Vec::with_capacity(sorted.len());
        for e in sorted {
            match deduped.last_mut() {
                Some(last) if last.key == e.key => *last = e,
                _ => deduped.push(e),
            }
        }
        let mut merged = Vec::with_capacity(self.entries.len() + deduped.len());
        let (mut a, mut b) = (self.entries.iter().peekable(), deduped.iter().peekable());
        loop {
            match (a.peek(), b.peek()) {
                (Some(x), Some(y)) => match x.key.cmp(&y.key) {
                    Ordering::Less => merged.extend(a.next().copied()),
                    Ordering::Greater => merged.extend(b.next().copied()),
                    Ordering::Equal => {
                        a.next();
                        merged.extend(b.next().copied());
                    }
                },
                (Some(_), None) => merged.extend(a.next().copied()),
                (None, Some(_)) => merged.extend(b.next().copied()),
                (None, None) => break,
            }
        }
        self.entries = merged;
        Ok(())
    }

    /// The tiles whose bordered column overlaps any of `boxes`, each
    /// `[min x, min y, min z, max x, max y, max z]`, in row-major order. Only the xz
    /// extent matters, as tiles are columns. Boxes with a non-finite coordinate are ignored.
    ///
    /// Pass each edit's box both before and after the edit (a moved object dirties
    /// where it was and where it is).
    pub fn dirty_from_boxes(&self, boxes: &[[f32; 6]]) -> Vec<TileKey> {
        let g = &self.grid;
        let mut out = BTreeSet::new();
        for b in boxes {
            if b.iter().any(|v| !v.is_finite()) {
                continue;
            }
            let (lo, hi) = (
                [fmin(b[0], b[3]), fmin(b[2], b[5])],
                [fmax(b[0], b[3]), fmax(b[2], b[5])],
            );
            // One tile of slack on each side; the exact test below decides.
            let range = |lo: f32, hi: f32, origin: f32, n: i32| {
                let first = floorf((lo - origin - g.halo) / g.tile_extent) as i32;
                let last = floorf((hi - origin + g.halo) / g.tile_extent) as i32;
                (
                    first.saturating_sub(1).clamp(0, n - 1),
                    last.saturating_add(1).clamp(0, n - 1),
                )
            };
            let (x0, x1) = range(lo[0], hi[0], g.origin[0], g.tiles_x);
            let (z0, z1) = range(lo[1], hi[1], g.origin[2], g.tiles_z);
            for z in z0..=z1 {
                for x in x0..=x1 {
                    let key = TileKey { x, z };
                    let (tmin, tmax) = g.tile_bounds(key);
                    if lo[0] <= tmax[0] && hi[0] >= tmin[0] && lo[1] <= tmax[2] && hi[1] >= tmin[2]
                    {
                        out.insert(key);
                    }
                }
            }
        }
        out.into_iter().collect()
    }

    /// Serializes the manifest (little-endian, see [`MANIFEST_VERSION`]).
    pub fn to_bytes(&self) -> Vec<u8> {
        let mut out = Vec::with_capacity(HEADER_LEN + self.entries.len() * ENTRY_LEN);
        out.extend_from_slice(&MANIFEST_MAGIC);
        out.extend_from_slice(&self.version.to_le_bytes());
        out.extend_from_slice(&self.config_hash.to_le_bytes());
        let g = &self.grid;
        for v in g.origin {
            out.extend_from_slice(&v.to_le_bytes());
        }
        for v in [g.max_y, g.tile_extent, g.halo] {
            out.extend_from_slice(&v.to_le_bytes());
        }
        for v in [g.halo_cells, g.tiles_x, g.tiles_z] {
            out.extend_from_slice(&v.to_le_bytes());
        }
        out.extend_from_slice(&(self.entries.len() as u64).to_le_bytes());
        for e in &self.entries {
            out.extend_from_slice(&e.key.x.to_le_bytes());
            out.extend_from_slice(&e.key.z.to_le_bytes());
            out.extend_from_slice(&e.input_hash.to_le_bytes());
            out.push(u8::from(e.output_hash.is_some()));
            out.extend_from_slice(&e.output_hash.unwrap_or(0).to_le_bytes());
        }
        out
    }

    /// Reads a manifest from untrusted bytes.
    ///
    /// Everything is validated: the magic and version, the layout, the exact length,
    /// the strict `(z, x)` order of the entries, that every key lies in the grid and
    /// that flags and hashes are canonical. `limits.max_bytes` bounds the input and
    /// the entry storage, `limits.max_work` the entry count.
    ///
    /// # Errors
    /// [`Error::InvalidInput`] for malformed bytes, [`Error::LimitExceeded`] over `limits`.
    pub fn from_bytes(bytes: &[u8], limits: Limits) -> Result<Self, Error> {
        if bytes.len() > limits.max_bytes {
            return Err(Error::LimitExceeded);
        }
        let mut r = Reader { bytes, at: 0 };
        if r.take(4)? != MANIFEST_MAGIC {
            return Err(Error::InvalidInput("manifest magic"));
        }
        let version = r.u32()?;
        if version != MANIFEST_VERSION {
            return Err(Error::InvalidInput("manifest version"));
        }
        let config_hash = r.u64()?;
        let origin = [r.f32()?, r.f32()?, r.f32()?];
        let (max_y, tile_extent, halo) = (r.f32()?, r.f32()?, r.f32()?);
        let (halo_cells, tiles_x, tiles_z) = (r.i32()?, r.i32()?, r.i32()?);
        let grid = GridLayout {
            origin,
            max_y,
            tiles_x,
            tiles_z,
            tile_extent,
            halo,
            halo_cells,
        };
        grid.wellformed()
            .map_err(|_| Error::InvalidInput("manifest grid layout"))?;
        let count = usize::try_from(r.u64()?).map_err(|_| Error::SizeOverflow)?;
        let body = bytes.len() - HEADER_LEN;
        if count.checked_mul(ENTRY_LEN) != Some(body) {
            return Err(Error::InvalidInput("manifest length"));
        }
        let mut ws = Workspace::with_limits(limits);
        ws.work(count as u64)?;
        let mut entries = ws.vec_with_capacity::<ManifestEntry>(count)?;
        let mut prev: Option<TileKey> = None;
        for _ in 0..count {
            let key = TileKey {
                x: r.i32()?,
                z: r.i32()?,
            };
            let input_hash = r.u64()?;
            let flag = r.take(1)?[0];
            let out = r.u64()?;
            let output_hash = match (flag, out) {
                (0, 0) => None,
                (1, h) => Some(h),
                _ => return Err(Error::InvalidInput("manifest entry flag")),
            };
            if !grid.contains(key) || prev.is_some_and(|p| p >= key) {
                return Err(Error::InvalidInput("manifest entry order"));
            }
            prev = Some(key);
            entries.push(ManifestEntry {
                key,
                input_hash,
                output_hash,
            });
        }
        Ok(Self {
            version,
            config_hash,
            grid,
            entries,
        })
    }
}

struct Reader<'a> {
    bytes: &'a [u8],
    at: usize,
}

impl<'a> Reader<'a> {
    fn take(&mut self, n: usize) -> Result<&'a [u8], Error> {
        let end = self.at.checked_add(n).ok_or(Error::SizeOverflow)?;
        let s = self
            .bytes
            .get(self.at..end)
            .ok_or(Error::InvalidInput("manifest truncated"))?;
        self.at = end;
        Ok(s)
    }
    fn array<const N: usize>(&mut self) -> Result<[u8; N], Error> {
        self.take(N)?
            .try_into()
            .map_err(|_| Error::InvalidInput("manifest truncated"))
    }
    fn u32(&mut self) -> Result<u32, Error> {
        Ok(u32::from_le_bytes(self.array()?))
    }
    fn i32(&mut self) -> Result<i32, Error> {
        Ok(i32::from_le_bytes(self.array()?))
    }
    fn u64(&mut self) -> Result<u64, Error> {
        Ok(u64::from_le_bytes(self.array()?))
    }
    fn f32(&mut self) -> Result<f32, Error> {
        Ok(f32::from_le_bytes(self.array()?))
    }
}

/// Digest of everything one tile's pipeline reads.
pub(super) fn input_hash(
    config_hash: u64,
    key: TileKey,
    triangles: &TriangleSink,
    volumes: &VolumeSink,
    off_mesh: &OffMeshSink,
) -> u64 {
    let mut h = Fnv64::new();
    h.u64(config_hash);
    h.i32(key.x);
    h.i32(key.z);
    triangles.digest(&mut h);
    volumes.digest(&mut h);
    off_mesh.digest(&mut h);
    h.finish()
}
