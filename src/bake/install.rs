//! Budgeted installation of finished tiles into a navmesh (not in recastnavigation).

use alloc::collections::VecDeque;
use alloc::vec::Vec;

use super::BakeError;
use super::job::TileOutput;
use super::manifest::TileKey;
use crate::detour::NavMesh;
use crate::{Budget, Progress};

/// Work units one tile of `len` bytes costs: one per KiB of tile data, at least one.
///
/// Installing validates the tile and links it to its neighbours, both proportional to
/// the tile's size; `docs/VERIFICATION.md` lists the measured time per unit.
pub fn install_cost(len: usize) -> u64 {
    (len.div_ceil(1024) as u64).max(1)
}

/// Queues finished tiles and installs them into a [`NavMesh`] in slices of a work
/// [`Budget`] (not in recastnavigation).
///
/// [`NavMesh::add_tile`] validates and links a tile in one call, so a tile is the
/// smallest piece: one [`install`](Self::install) call takes at most the time of the
/// last tile it started, and the budget decides how many tiles that is. A budget of one
/// unit installs exactly one tile per call. The queue and the budget are the only
/// limits; the navmesh's own `max_tiles` (set from [`GridLayout::nav_mesh_params`]) is the
/// residency cap and is a parameter of the navmesh, not of this type.
///
/// [`GridLayout::nav_mesh_params`]: super::GridLayout::nav_mesh_params
#[derive(Debug, Default)]
pub struct TileInstaller {
    pending: VecDeque<(TileKey, Vec<u8>)>,
    installed: usize,
}

impl TileInstaller {
    /// An empty queue.
    pub fn new() -> Self {
        Self::default()
    }

    /// Queues a finished tile; tiles without polygons are dropped (there is nothing to install).
    pub fn push(&mut self, tile: TileOutput) {
        if let Some(data) = tile.data {
            self.pending.push_back((tile.key, data));
        }
    }

    /// Tiles waiting to be installed.
    pub fn pending(&self) -> usize {
        self.pending.len()
    }

    /// Tiles installed so far.
    pub fn installed(&self) -> usize {
        self.installed
    }

    /// Installs queued tiles, in the order queued, until `budget` has no work left.
    ///
    /// Returns [`Progress::Done`] when the queue is empty and [`Progress::Exhausted`] when
    /// the budget ran out first. At least one tile is installed whenever the budget has
    /// work left; the tile that crosses the limit is installed whole and the budget is
    /// marked fully spent.
    ///
    /// # Errors
    /// A [`BakeError`] naming the tile whose bytes the navmesh refused (all or nothing: the
    /// navmesh is unchanged by that tile, which leaves the queue; later tiles stay queued).
    pub fn install(
        &mut self,
        nav: &mut NavMesh,
        budget: &mut Budget,
    ) -> Result<Progress, BakeError> {
        while let Some((key, data)) = self.pending.pop_front() {
            if budget.remaining() == 0 {
                self.pending.push_front((key, data));
                budget.exhaust();
                return Ok(Progress::Exhausted);
            }
            let cost = install_cost(data.len());
            nav.add_tile(data).map_err(|error| BakeError {
                tile: Some([key.x, key.z]),
                error,
            })?;
            self.installed += 1;
            if !budget.spend(cost) {
                budget.exhaust();
                return Ok(if self.pending.is_empty() {
                    Progress::Done
                } else {
                    Progress::Exhausted
                });
            }
        }
        Ok(Progress::Done)
    }
}
