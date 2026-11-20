//! Live-mesh corridor refinement without tile snapshots.
use super::{SectorKey, SectorParams};
use crate::detour::query::DetachedQuery;
use crate::detour::{DefaultFilter, MeshTile, NavMesh, Poly, PolyRef, QueryFilter, Status};
use crate::{Budget, Error, Progress};
use alloc::vec::Vec;

/// A resident endpoint for a refinement window. Re-resolve after streaming edits.
#[derive(Clone, Copy, Debug)]
#[non_exhaustive]
pub struct WindowPoint {
    /// Polygon on the live mesh.
    pub polygon: PolyRef,
    /// Position on that polygon.
    pub position: [f32; 3],
}
impl WindowPoint {
    /// Creates a resident endpoint.
    pub fn new(polygon: PolyRef, position: [f32; 3]) -> Self {
        Self { polygon, position }
    }
}

struct WindowFilter {
    inner: DefaultFilter,
    params: SectorParams,
    sectors: Vec<SectorKey>,
}
impl QueryFilter for WindowFilter {
    fn pass_filter(&self, r: PolyRef, tile: &MeshTile, poly: &Poly) -> bool {
        self.sectors
            .binary_search(&self.params.sector_of_tile(tile.header.x, tile.header.y))
            .is_ok()
            && self.inner.pass_filter(r, tile, poly)
    }
    fn cost(
        &self,
        a: &[f32; 3],
        b: &[f32; 3],
        prev: Option<(PolyRef, &MeshTile, &Poly)>,
        cur: (PolyRef, &MeshTile, &Poly),
        next: Option<(PolyRef, &MeshTile, &Poly)>,
    ) -> f32 {
        self.inner.cost(a, b, prev, cur, next)
    }
}

/// Sliced refinement restricted to caller-selected resident sectors.
///
/// Uses the live mesh and retains search state without borrowing it between
/// updates. Admission and finalization are explicit operations outside the sliced
/// expansion budget; neither has a wall-time guarantee. Partial results retain
/// their status bits and must not be treated as a completed window.
#[non_exhaustive]
pub struct CorridorWindow {
    query: Option<DetachedQuery>,
    filter: WindowFilter,
    owner: alloc::sync::Arc<()>,
    epoch: u64,
    #[cfg(feature = "dynamic")]
    state_epoch: u64,
    status: Status,
    finalized: bool,
    corridor: Vec<PolyRef>,
}
impl CorridorWindow {
    /// Starts a window with a node pool of `max_nodes` (at most 65,535).
    /// Both endpoints must pass the sector and profile filter. The caller includes
    /// its chosen margin sectors and resolves portal positions to live polygons.
    pub fn new(
        nav: &NavMesh,
        params: &SectorParams,
        sectors: &[SectorKey],
        filter: DefaultFilter,
        start: WindowPoint,
        goal: WindowPoint,
        max_nodes: usize,
    ) -> Result<Self, Error> {
        params.validate()?;
        let mut sectors = sectors.to_vec();
        sectors.sort_unstable();
        sectors.dedup();
        let filter = WindowFilter {
            inner: filter,
            params: *params,
            sectors,
        };
        for endpoint in [start, goal] {
            let (tile, poly) = nav.tile_and_poly_by_ref(endpoint.polygon)?;
            if !filter.pass_filter(endpoint.polygon, tile, poly) {
                return Err(Error::InvalidInput("window endpoint outside filter"));
            }
        }
        let mut query = DetachedQuery::new(nav, max_nodes)?.attach(nav);
        let status = query.init_sliced_find_path(
            start.polygon,
            goal.polygon,
            &start.position,
            &goal.position,
            &filter,
            0,
        )?;
        Ok(Self {
            query: Some(query.detach()),
            filter,
            owner: nav.report_identity().clone(),
            epoch: nav.epoch(),
            #[cfg(feature = "dynamic")]
            state_epoch: nav.state_epoch(),
            status,
            finalized: false,
            corridor: Vec::new(),
        })
    }

    /// Runs at most `max_unit` expansions, charged to the shared budget. Every
    /// mesh epoch change invalidates the window, including edits outside it.
    /// Errors preserve the detached state so callers can inspect or discard it.
    pub fn resume(
        &mut self,
        nav: &NavMesh,
        budget: &mut Budget,
        max_unit: u64,
    ) -> Result<Progress, Error> {
        #[cfg(feature = "dynamic")]
        if nav.state_epoch() != self.state_epoch {
            return Err(Error::MeshChanged);
        }
        if !alloc::sync::Arc::ptr_eq(&self.owner, nav.report_identity())
            || nav.epoch() != self.epoch
        {
            return Err(Error::MeshChanged);
        }
        if !self.status.is_in_progress() {
            return Ok(Progress::Done);
        }
        let allowance = budget.remaining().min(max_unit).min(i32::MAX as u64) as usize;
        if allowance == 0 {
            return Ok(Progress::Exhausted);
        }
        let detached = self.query.take().ok_or(Error::Failed)?;
        let mut query = detached.attach(nav);
        let result = query.update_sliced_find_path(allowance, &self.filter);
        self.query = Some(query.detach());
        // On errors conservatively charge the whole allowance; successful updates
        // report their exact expansion count.
        match result {
            Ok((status, used)) => {
                budget.spend(used as u64);
                self.status = status;
            }
            Err(error) => {
                budget.spend(allowance as u64);
                return Err(error);
            }
        }
        Ok(if self.status.is_in_progress() {
            Progress::Exhausted
        } else {
            Progress::Done
        })
    }

    /// Finalizes after expansion completes. This copies up to `max_path` polygon
    /// refs and is not budgeted; schedule it separately and measure its cost.
    pub fn finish(&mut self, nav: &NavMesh, max_path: usize) -> Result<Status, Error> {
        #[cfg(feature = "dynamic")]
        if nav.state_epoch() != self.state_epoch {
            return Err(Error::MeshChanged);
        }
        if !alloc::sync::Arc::ptr_eq(&self.owner, nav.report_identity())
            || nav.epoch() != self.epoch
        {
            return Err(Error::MeshChanged);
        }
        if self.finalized {
            return Ok(self.status);
        }
        if self.status.is_in_progress() {
            return Err(Error::InvalidInput("window is still planning"));
        }
        let detached = self.query.take().ok_or(Error::Failed)?;
        let mut query = detached.attach(nav);
        let result = query.finalize_sliced_find_path(&self.filter, &mut self.corridor, max_path);
        self.query = Some(query.detach());
        self.status = result?;
        self.finalized = true;
        Ok(self.status)
    }

    pub(super) fn take_corridor(&mut self) -> Vec<PolyRef> {
        core::mem::take(&mut self.corridor)
    }
    /// Final corridor and status, including partial-result or node-exhaustion bits.
    pub fn result(&self) -> Option<(&[PolyRef], Status)> {
        self.finalized
            .then_some((self.corridor.as_slice(), self.status))
    }
}
