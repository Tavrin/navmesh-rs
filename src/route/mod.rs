//! Sector summaries for long routes (not in recastnavigation; feature `route`).
//!
//! A *sector* is a square of `tiles_per_sector` x `tiles_per_sector` tiles; a
//! *super-sector* is a square of `sectors_per_super` x `sectors_per_super` sectors.
//! Long-route search plans over three levels: polygons of resident tiles (level 1,
//! the navmesh itself), sector summaries (level 2) and a region graph over
//! super-sectors (level 3). This module builds, stores and validates levels 2 and 3;
//! it holds no polygon geometry.
//!
//! - [`SectorSummary`] (record `'DNSS'`): per sector and filter profile, the
//!   *regions* (components of reciprocal links between passable polygons inside the sector), the
//!   *portals* (maximal passable runs along each sector side, one per side, run and
//!   region; wide runs carry two transition points), off-mesh links that leave the
//!   sector, a floor-quantized `u16` table of exact transition-to-transition costs
//!   inside the sector, and the content ids of the tiles it was derived from.
//! - [`SuperRecord`] (record `'DNSR'`): per super-sector, the regions of its sectors
//!   joined across shared portals, their boundary portals and a lower-bound crossing
//!   cost per region. It can be loaded without any sector summary.
//! - [`SectorGraph`]: the resident level-2 summaries and the level-3 graph, with
//!   per-level retained bytes and per-sector compression figures.
//! - [`summarize_tiles`] and [`SectorCollector`] (with `recast`): bake-side
//!   construction from tile bytes, independent of what else is resident.
//!
//! Isolated summaries depend only on their sector and filter profile. Stitched
//! records also depend on the installed links, effective flags and areas:
//! poly refs, slots and load order never reach the records, so the bytes are
//! identical for any insertion order. Summaries never hash at query time; content ids
//! are computed when a summary is built.

mod graph;
mod summary;
mod wire;

#[cfg(feature = "recast")]
mod collect;

#[cfg(feature = "recast")]
pub use collect::SectorCollector;
pub use graph::{
    Compression, GraphUpdate, SUPER_MAGIC, SUPER_VERSION, SectorGraph, SuperEdge, SuperOffMesh,
    SuperPortal, SuperRecord, SuperRegion,
};
pub use summary::{
    OffMeshEntry, Portal, Region, SECTOR_MAGIC, SECTOR_VERSION, SectorSummary, SectorTile,
    UNREACHABLE, summarize_tiles,
};

use crate::detour::DefaultFilter;

/// Coordinates of a sector: tile `(x, z)` belongs to sector
/// `(x.div_euclid(k), z.div_euclid(k))` for `k = tiles_per_sector`.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq, PartialOrd, Ord, Hash)]
#[cfg_attr(feature = "serde", derive(serde::Serialize))]
pub struct SectorKey {
    /// Sector x.
    pub x: i32,
    /// Sector z (Detour's tile y axis).
    pub z: i32,
}

/// Coordinates of a super-sector: sector `(x, z)` belongs to super-sector
/// `(x.div_euclid(k), z.div_euclid(k))` for `k = sectors_per_super`.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq, PartialOrd, Ord, Hash)]
#[cfg_attr(feature = "serde", derive(serde::Serialize))]
pub struct SuperKey {
    /// Super-sector x.
    pub x: i32,
    /// Super-sector z.
    pub z: i32,
}

/// How tiles group into sectors and sectors into super-sectors.
#[derive(Clone, Copy, Debug, PartialEq)]
#[non_exhaustive]
pub struct SectorParams {
    /// Tiles per sector side (default 8).
    pub tiles_per_sector: u32,
    /// Sectors per super-sector side (default 8).
    pub sectors_per_super: u32,
    /// Portal runs wider than this, in metres, get two transition points instead of
    /// one. Zero or negative derives the width from the sector's tiles:
    /// `32 x walkable_radius` (4 x agent radius x 8), or 16 m when the radius is zero.
    pub portal_split_width: f32,
}

impl Default for SectorParams {
    fn default() -> Self {
        SectorParams {
            tiles_per_sector: 8,
            sectors_per_super: 8,
            portal_split_width: 0.0,
        }
    }
}

impl SectorParams {
    /// Parameters with the given grouping and a derived split width.
    pub fn new(tiles_per_sector: u32, sectors_per_super: u32) -> Self {
        SectorParams {
            tiles_per_sector,
            sectors_per_super,
            portal_split_width: 0.0,
        }
    }

    pub(crate) fn validate(&self) -> Result<(), crate::Error> {
        if !(1..=u16::MAX as u32).contains(&self.tiles_per_sector) {
            return Err(crate::Error::InvalidParameter("tiles_per_sector"));
        }
        if !(1..=u16::MAX as u32).contains(&self.sectors_per_super) {
            return Err(crate::Error::InvalidParameter("sectors_per_super"));
        }
        if !self.portal_split_width.is_finite() {
            return Err(crate::Error::InvalidParameter("portal_split_width"));
        }
        Ok(())
    }

    /// The sector holding tile `(x, z)`.
    pub fn sector_of_tile(&self, x: i32, z: i32) -> SectorKey {
        let k = self.tiles_per_sector.max(1) as i32;
        SectorKey {
            x: x.div_euclid(k),
            z: z.div_euclid(k),
        }
    }

    /// The super-sector holding `sector`.
    pub fn super_of(&self, sector: SectorKey) -> SuperKey {
        let k = self.sectors_per_super.max(1) as i32;
        SuperKey {
            x: sector.x.div_euclid(k),
            z: sector.z.div_euclid(k),
        }
    }
}

/// A stable 32-bit id for a [`DefaultFilter`]: FNV-1a over its area costs (bit
/// patterns) and flags. Store it with records so a summary is never read for the
/// wrong profile; custom [`QueryFilter`](crate::detour::QueryFilter)s choose their own id.
pub fn profile_hash(filter: &DefaultFilter) -> u32 {
    let mut h = 0x811c_9dc5u32;
    let mut eat = |bytes: &[u8]| {
        for &b in bytes {
            h ^= u32::from(b);
            h = h.wrapping_mul(0x0100_0193);
        }
    };
    for c in &filter.area_cost {
        eat(&c.to_bits().to_le_bytes());
    }
    eat(&filter.include_flags.to_le_bytes());
    eat(&filter.exclude_flags.to_le_bytes());
    h
}

/// Detour side bits of a sector side: 0 = +x, 2 = +z, 4 = -x, 6 = -z.
pub(crate) fn opposite_side(side: u8) -> u8 {
    (side + 4) & 7
}

/// Whether two portals on facing sides overlap: positive overlap along the side and
/// vertical ranges within `climb` of each other.
pub(crate) fn runs_meet(
    a_extent: [f32; 2],
    a_y: [f32; 2],
    b_extent: [f32; 2],
    b_y: [f32; 2],
    climb: f32,
    eps: f32,
) -> bool {
    let overlap = a_extent[1].min(b_extent[1]) - a_extent[0].max(b_extent[0]);
    overlap > eps && a_y[0] <= b_y[1] + climb && b_y[0] <= a_y[1] + climb
}

mod search;
pub use search::{CoarseSearch, RegionKey, RouteIndex};
mod window;
pub use window::{CorridorWindow, WindowPoint};

mod stitched;

mod navigator;
pub use navigator::{Navigator, RequestId, RouteAtlas, RouteState};
