//! Bake-side collection: sector summaries from a bake's tile output.

use super::{SectorKey, SectorParams, SectorSummary, summarize_tiles};
use crate::bake::{GridLayout, TileKey, TileOutput};
use crate::detour::QueryFilter;
use crate::{Error, Limits};
use alloc::collections::BTreeMap;
use alloc::vec::Vec;

/// Gathers baked tiles per sector and summarizes each sector once all of its grid
/// tiles have arrived, in any order, as [`BakeJob`](crate::bake::BakeJob) output or
/// read back from disk.
///
/// A sector's tiles are held until the sector completes, then dropped: memory is
/// bounded by the sectors in flight. To refresh a sector after an edit, push all of
/// its tiles again (the changed ones newly baked). Empty tiles count as arrived.
#[derive(Clone, Debug)]
pub struct SectorCollector<F> {
    grid: GridLayout,
    params: SectorParams,
    filter: F,
    profile: u32,
    limits: Limits,
    pending: BTreeMap<SectorKey, Arrived>,
}

/// Tiles of one sector so far, keyed by `(z, x)`; `None` for an empty tile.
type Arrived = BTreeMap<(i32, i32), Option<Vec<u8>>>;

impl<F: QueryFilter> SectorCollector<F> {
    /// A collector for `grid`, summarizing with `filter` under profile id `profile`.
    /// Errors: [`Error::InvalidParameter`] for bad `params`.
    pub fn new(
        grid: &GridLayout,
        params: &SectorParams,
        filter: F,
        profile: u32,
        limits: Limits,
    ) -> Result<Self, Error> {
        params.validate()?;
        Ok(SectorCollector {
            grid: *grid,
            params: *params,
            filter,
            profile,
            limits,
            pending: BTreeMap::new(),
        })
    }

    /// Grid tiles of `key`: the sector clipped to the grid.
    fn expected(&self, key: SectorKey) -> usize {
        let k = self.params.tiles_per_sector as i64;
        let span = |lo: i64, n: i32| (lo.max(0)..(lo + k).min(i64::from(n))).count();
        span(i64::from(key.x) * k, self.grid.tiles_x)
            * span(i64::from(key.z) * k, self.grid.tiles_z)
    }

    /// Records one baked tile (`None` for a tile without polygons) and returns the
    /// sector's summary when this tile completes it. Errors:
    /// [`Error::InvalidInput`] for a key outside the grid, and the errors of
    /// [`summarize_tiles`].
    pub fn push(
        &mut self,
        key: TileKey,
        data: Option<Vec<u8>>,
    ) -> Result<Option<(SectorKey, SectorSummary)>, Error> {
        if !self.grid.contains(key) {
            return Err(Error::InvalidInput("tile outside the grid"));
        }
        let sector = self.params.sector_of_tile(key.x, key.z);
        let expected = self.expected(sector);
        let tiles = self.pending.entry(sector).or_default();
        tiles.insert((key.z, key.x), data);
        if tiles.len() < expected {
            return Ok(None);
        }
        let tiles = self.pending.remove(&sector).unwrap_or_default();
        let bytes: Vec<Vec<u8>> = tiles.into_values().flatten().collect();
        let summary = summarize_tiles(
            &self.grid.nav_mesh_params(1, 1),
            sector,
            &bytes,
            &self.params,
            &self.filter,
            self.profile,
            self.limits,
        )?;
        Ok(Some((sector, summary)))
    }

    /// [`push`](Self::push) for one bake output (its bytes are copied).
    pub fn push_output(
        &mut self,
        out: &TileOutput,
    ) -> Result<Option<(SectorKey, SectorSummary)>, Error> {
        self.push(out.key, out.data.clone())
    }

    /// Sectors with some but not all of their tiles.
    pub fn pending(&self) -> usize {
        self.pending.len()
    }
}
