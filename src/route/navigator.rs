//! Shared scheduling of abstract searches and live corridor windows.
use super::portal_window::PortalWindow;
use super::stitched::Crossing;
use super::{
    CoarseSearch, CorridorWindow, RegionKey, RouteIndex, SectorGraph, SectorKey, WindowPoint,
    profile_hash,
};
use crate::detour::{DefaultFilter, NavMesh, PolyRef, RefState, Status};
use crate::{Budget, Error};
use alloc::{
    boxed::Box,
    collections::{BTreeMap, BTreeSet, BinaryHeap, VecDeque},
    vec::Vec,
};
use core::cmp::Reverse;

type Region = (SectorKey, u16);
/// Euclidean distance, never infinite for finite points: far-apart huge coordinates
/// fall back to double precision and saturate at `f32::MAX`.
pub(super) fn distance(a: [f32; 3], b: [f32; 3]) -> f32 {
    let d = crate::common::sqrtf(
        (a[0] - b[0]) * (a[0] - b[0])
            + (a[1] - b[1]) * (a[1] - b[1])
            + (a[2] - b[2]) * (a[2] - b[2]),
    );
    if d.is_finite() {
        return d;
    }
    // Scale by the largest component so the square root sees values of at most 3.
    let delta = [0, 1, 2].map(|i| f64::from(a[i]) - f64::from(b[i]));
    let m = delta.iter().fold(0.0f64, |m, v| m.max(v.abs()));
    if m.is_nan() || m == 0.0 {
        return if m == 0.0 { 0.0 } else { f32::MAX };
    }
    let unit: f64 = delta.iter().map(|v| (v / m) * (v / m)).sum();
    (m * f64::from(crate::common::sqrtf(unit as f32))).min(f64::from(f32::MAX)) as f32
}

/// A label cost that stays ordered: overflow saturates at `f32::MAX` instead of
/// becoming infinite (which would read as unreachable).
fn saturated(cost: f32) -> f32 {
    if cost.is_finite() { cost } else { f32::MAX }
}

/// Charges `n` units: what the budget has left now, the rest as debt the request
/// pays before its next step.
fn owe(budget: &mut Budget, debt: &mut u64, n: u64) {
    let now = n.min(budget.remaining());
    budget.spend(now);
    *debt = debt.saturating_add(n - now);
}
#[derive(Clone, Copy)]
struct Portal {
    destination: RegionKey,
    crossing: Crossing,
    entry: Option<usize>,
    exit: Option<usize>,
}

/// Prepared search data shared by requests for one installed filter profile.
#[non_exhaustive]
pub struct RouteAtlas<'a> {
    graph: &'a SectorGraph,
    coarse: RouteIndex,
    filter: DefaultFilter,
    minimum: f32,
    profile: u32,
    portals: Vec<Portal>,
    outgoing: BTreeMap<Region, Vec<usize>>,
    slots: Vec<Option<(PolyRef, SectorKey, u64)>>,
    epoch: u64,
}
impl<'a> RouteAtlas<'a> {
    /// Prepares portal adjacency outside the frame loop. The graph must be the
    /// current result of building from this fully installed mesh.
    pub fn new(
        graph: &'a SectorGraph,
        nav: &NavMesh,
        filter: DefaultFilter,
    ) -> Result<Self, Error> {
        if filter.area_cost.iter().any(|c| !c.is_finite() || *c < 0.) {
            return Err(Error::InvalidParameter("route area cost"));
        }
        let profile = profile_hash(&filter);
        // The profile id is a 32-bit hash: bind to the whole filter, bit for bit.
        let same_filter = graph.bound_filter.as_ref().is_some_and(|f| {
            f.include_flags == filter.include_flags
                && f.exclude_flags == filter.exclude_flags
                && f.area_cost
                    .iter()
                    .zip(&filter.area_cost)
                    .all(|(a, b)| a.to_bits() == b.to_bits())
        });
        if graph.bound_epoch != Some(nav.epoch())
            || !graph
                .owner
                .as_ref()
                .is_some_and(|owner| alloc::sync::Arc::ptr_eq(owner, nav.report_identity()))
            || graph.bound_profile != Some(profile)
            || !same_filter
        {
            return Err(Error::MeshChanged);
        }
        #[cfg(feature = "dynamic")]
        if graph.state_epoch != nav.state_epoch() {
            return Err(Error::MeshChanged);
        }
        let nearest = |k, r, p| {
            graph.sector(profile, k).and_then(|s| {
                s.portals()
                    .iter()
                    .enumerate()
                    .filter(|(_, t)| t.region == r)
                    .min_by(|a, b| distance(a.1.pos, p).total_cmp(&distance(b.1.pos, p)))
                    .map(|x| x.0)
            })
        };
        let mut portals = Vec::new();
        let mut outgoing: BTreeMap<Region, Vec<usize>> = BTreeMap::new();
        for (&key, rows) in &graph.crossings {
            // Sample wide transitions at half-tile intervals in the mesh frame.
            let mut representatives = BTreeMap::new();
            for &c in rows {
                let exit = nearest(key, c.from, c.position);
                let entry = nearest(c.sector, c.region, c.position);
                let target = exit
                    .and_then(|i| graph.sector(profile, key)?.portals().get(i))
                    .map_or(c.position, |p| p.pos);
                let rank = (
                    distance(target, c.position).to_bits(),
                    c.position.map(f32::to_bits),
                    stable_key(nav, c.exit)?,
                    stable_key(nav, c.landing)?,
                );
                let slot = representatives
                    .entry((
                        c.from,
                        c.sector,
                        c.region,
                        exit,
                        entry,
                        crate::common::floorf(
                            (c.position[0] - nav.params().orig[0])
                                / (nav.params().tile_width * 0.5),
                        ) as i64,
                        crate::common::floorf(
                            (c.position[2] - nav.params().orig[2])
                                / (nav.params().tile_height * 0.5),
                        ) as i64,
                    ))
                    .or_insert((rank, c));
                if rank < slot.0 {
                    *slot = (rank, c);
                }
            }
            for ((from, _, _, exit, entry, _, _), (_, crossing)) in representatives {
                outgoing.entry((key, from)).or_default().push(portals.len());
                portals.push(Portal {
                    destination: RegionKey::new(
                        graph.params().super_of(crossing.sector),
                        graph
                            .super_region(profile, crossing.sector, crossing.region)
                            .ok_or(Error::InvalidRef)?,
                    ),
                    crossing,
                    entry,
                    exit,
                });
            }
        }
        let slots = (0..nav.max_tiles() as usize)
            .map(|slot| {
                nav.tile(slot).map(|t| {
                    (
                        nav.tile_ref(slot),
                        graph.params().sector_of_tile(t.header.x, t.header.y),
                        nav.state_stamp(nav.tile_ref(slot)).unwrap_or(0),
                    )
                })
            })
            .collect();
        Ok(Self {
            graph,
            coarse: RouteIndex::build_exact(graph, profile)?,
            profile,
            minimum: filter
                .area_cost
                .iter()
                .copied()
                .fold(f32::INFINITY, f32::min),
            filter,
            portals,
            outgoing,
            slots,
            epoch: nav.epoch(),
        })
    }
    /// Whether a tile of `sector` is not resident. A scan (only after streaming)
    /// charges one unit per tile examined.
    fn missing(
        &self,
        nav: &NavMesh,
        sector: SectorKey,
        budget: &mut Budget,
        debt: &mut u64,
    ) -> bool {
        if nav.epoch() == self.epoch {
            return false;
        }
        let Some(s) = self.graph.sector(self.profile, sector) else {
            return false;
        };
        let mut scanned = 0;
        let gone = s.tiles().iter().any(|t| {
            scanned += 1;
            nav.tile_index_at(t.x, t.z, i32::from(t.layer)).is_none()
        });
        owe(budget, debt, scanned);
        gone
    }
    fn region(&self, nav: &NavMesh, r: PolyRef) -> Result<Region, Error> {
        let (_, slot, poly) = nav.decode_poly_id(r);
        let &(base, key, _) = self
            .slots
            .get(slot as usize)
            .and_then(Option::as_ref)
            .ok_or(Error::InvalidRef)?;
        if base | poly as PolyRef != r {
            return Err(Error::InvalidRef);
        }
        let (x, z, layer) = self
            .graph
            .slot_keys
            .get(&(slot as usize))
            .ok_or(Error::InvalidRef)?;
        let region = self
            .graph
            .sector(self.profile, key)
            .and_then(|s| s.polygon_region(*x, *z, *layer, poly))
            .ok_or(Error::InvalidRef)?;
        Ok((key, region))
    }
    /// Starts a level-three reachability query for two captured polygon refs.
    /// Resume it against the same graph used to build the atlas.
    pub fn coarse_request(
        &self,
        nav: &NavMesh,
        start: PolyRef,
        goal: PolyRef,
    ) -> Result<CoarseSearch<'_>, Error> {
        self.validate(nav, self.epoch)?;
        self.coarse.request(
            self.coarse_key(self.region(nav, start)?)?,
            self.coarse_key(self.region(nav, goal)?)?,
        )
    }
    fn coarse_key(&self, r: Region) -> Result<RegionKey, Error> {
        Ok(RegionKey::new(
            self.graph.params().super_of(r.0),
            self.graph
                .super_region(self.profile, r.0, r.1)
                .ok_or(Error::InvalidRef)?,
        ))
    }
    /// Revalidates the captured slots after streaming outside the planner budget.
    /// Same-content unloads and restores keep their refs. Content replacements,
    /// new tiles and dynamic edits require a freshly built graph.
    fn validate(&self, nav: &NavMesh, last: u64) -> Result<(), Error> {
        if !self
            .graph
            .owner
            .as_ref()
            .is_some_and(|owner| alloc::sync::Arc::ptr_eq(owner, nav.report_identity()))
        {
            return Err(Error::MeshChanged);
        }
        #[cfg(feature = "dynamic")]
        if self.graph.state_epoch != nav.state_epoch() {
            return Err(Error::MeshChanged);
        }
        if nav.global_off_mesh_links() != self.graph.global_links
            || nav.runtime_links_changed_at() > self.epoch
        {
            return Err(Error::MeshChanged);
        }
        if nav.epoch() != last {
            for (i, t) in self.slots.iter().enumerate() {
                match t {
                    Some((r, _, stamp))
                        if matches!(nav.ref_state(*r), RefState::Unloaded(_))
                            || nav.state_stamp(*r) == Some(*stamp) => {}
                    None if nav.tile(i).is_none() => {}
                    _ => return Err(Error::MeshChanged),
                }
            }
        }
        Ok(())
    }
}
fn stable_key(nav: &NavMesh, r: PolyRef) -> Result<(i32, i32, i32, u32), Error> {
    let (t, _) = nav.tile_and_poly_by_ref(r)?;
    Ok((
        t.header.y,
        t.header.x,
        t.header.layer,
        nav.decode_poly_id(r).2,
    ))
}

/// State of a scheduled request. A segment stays available until acknowledged.
#[derive(Clone, Debug, PartialEq, Eq)]
#[non_exhaustive]
pub enum RouteState {
    /// Work is queued.
    Planning,
    /// A contiguous live corridor is available for the caller to follow.
    Segment,
    /// Refinement resumes after the named sector's tiles arrive.
    Waiting(SectorKey),
    /// The last segment reached the goal.
    Complete,
    /// The directed region graph cannot reach the goal.
    Disconnected,
    /// A live window could not deliver its required endpoint.
    Failed(Error),
}
/// Handle for one request, unique during the scheduler's lifetime.
#[derive(Clone, Copy, Debug, PartialEq, Eq, PartialOrd, Ord)]
#[non_exhaustive]
pub struct RequestId(u64);

// A request allocates labels only for the portal blocks its search visits.
const LABEL_BLOCK: usize = 64;
#[derive(Clone, Copy)]
struct Label {
    cost: f32,
    parent: usize,
}
const UNSEEN: Label = Label {
    cost: f32::INFINITY,
    parent: START,
};
struct PortalLabels(Vec<Option<Box<[Label; LABEL_BLOCK]>>>);
impl PortalLabels {
    fn new(portals: usize) -> Self {
        Self(alloc::vec![None;portals.div_ceil(LABEL_BLOCK)])
    }
    fn get(&self, id: usize) -> Label {
        self.0[id / LABEL_BLOCK]
            .as_ref()
            .map_or(UNSEEN, |page| page[id % LABEL_BLOCK])
    }
    fn improve(&mut self, id: usize, cost: f32, parent: usize) -> bool {
        if cost >= self.get(id).cost {
            return false;
        }
        let page = self.0[id / LABEL_BLOCK].get_or_insert_with(|| Box::new([UNSEEN; LABEL_BLOCK]));
        page[id % LABEL_BLOCK] = Label { cost, parent };
        true
    }
}
struct Segment {
    path: Vec<PolyRef>,
    corners: Vec<crate::detour::StraightPathPoint>,
    end: WindowPoint,
}
struct Expansion<'a> {
    parent: usize,
    remaining: &'a [usize],
    sector: &'a super::SectorSummary,
    position: [f32; 3],
    entry: Option<usize>,
    cost: f32,
}

struct WindowPlan {
    sectors: BTreeSet<SectorKey>,
    core: Vec<SectorKey>,
    margin_cursor: usize,
    crossings: usize,
}
struct Request<'a> {
    priority: u8,
    coarse: Option<CoarseSearch<'a>>,
    corridor: BTreeSet<RegionKey>,
    coarse_cursor: usize,
    start: WindowPoint,
    goal: WindowPoint,
    region: Region,
    destination: Region,
    open: BinaryHeap<Reverse<(u32, u32, usize)>>,
    labels: PortalLabels,
    active: Option<Expansion<'a>>,
    trace: Option<usize>,
    route: Vec<usize>,
    refining: bool,
    at_exit: bool,
    window: Option<CorridorWindow>,
    held_prefix: Option<Segment>,
    segment: Vec<PolyRef>,
    corners: Vec<crate::detour::StraightPathPoint>,
    state: RouteState,
    published: bool,
    prefix_pending: bool,
    prefix_index: usize,
    prefix_target: Option<(f32, Crossing)>,
    plan: Option<WindowPlan>,
    fine: Option<PortalWindow>,
    spare: Option<PortalWindow>,
    skip: usize,
    lookahead: usize,
    /// Work already done but not yet charged (allocation, finalization, string
    /// pulling, tile scans); paid before the next step.
    debt: u64,
    /// The next single-sector window uses the whole node pool (after a smaller
    /// window ran out of nodes).
    wide: bool,
    /// Node pool of the current single-sector window.
    pool: usize,
}
const START: usize = usize::MAX;
/// Node limit of the first-move prefix search.
const PREFIX_NODES: usize = 256;
/// Node-pool entries a work unit pays for when a refinement window allocates its
/// pool (the pool is initialized in full).
const NODES_PER_UNIT: usize = 256;
impl<'a> Request<'a> {
    fn step(
        &mut self,
        atlas: &'a RouteAtlas<'a>,
        nav: &NavMesh,
        nodes: usize,
        budget: &mut Budget,
    ) -> Result<(), Error> {
        if self.debt > 0 {
            let now = self.debt.min(budget.remaining());
            budget.spend(now);
            self.debt -= now;
            if self.debt > 0 {
                return Ok(());
            }
        }
        if let Some(coarse) = &mut self.coarse {
            if coarse.resume(atlas.graph, budget)?.is_done() {
                let path = coarse.corridor_reversed().ok_or(Error::Failed)?;
                if path.is_empty() {
                    self.state = RouteState::Disconnected;
                }
                while self.coarse_cursor < path.len() && budget.spend(1) {
                    self.corridor.insert(path[self.coarse_cursor]);
                    self.coarse_cursor += 1;
                }
                if self.coarse_cursor == path.len() {
                    self.coarse = None;
                }
            }
            return Ok(());
        }
        if self.prefix_pending {
            return self.prefix(atlas, nav, nodes, budget);
        }
        if self.refining {
            return self.refine(atlas, nav, nodes, budget);
        }
        if let Some(expansion) = &mut self.active {
            // Resolve the region and cost table once per pop. A slice can
            // stop between any two outgoing witnesses without another lookup.
            while let Some((&next, rest)) = expansion.remaining.split_first() {
                if !budget.spend(1) {
                    return Ok(());
                }
                expansion.remaining = rest;
                let p = atlas.portals[next];
                let table = expansion
                    .entry
                    .zip(p.exit)
                    .and_then(|(a, b)| expansion.sector.cost(a, b));
                // Saturating sums keep huge coordinates ordered and finite (an
                // infinite label would read as unreachable, and 0 x inf as NaN).
                let direct =
                    saturated(distance(expansion.position, p.crossing.position) * atlas.minimum);
                let d = saturated(expansion.cost + table.unwrap_or(direct).max(direct));
                if self.labels.improve(next, d, expansion.parent) {
                    let h = saturated(
                        atlas.minimum * distance(p.crossing.position, self.goal.position),
                    );
                    self.open
                        .push(Reverse((saturated(d + h).to_bits(), d.to_bits(), next)));
                }
            }
            self.active = None;
            return Ok(());
        }
        if !budget.spend(1) {
            return Ok(());
        }
        if let Some(n) = self.trace {
            if n == START {
                self.trace = None;
                self.refining = true;
            } else {
                self.route.push(n);
                self.trace = Some(self.labels.get(n).parent);
            }
            return Ok(());
        }
        if let Some(Reverse((_, cost, n))) = self.open.pop() {
            if n != START && self.labels.get(n).cost.to_bits() != cost {
                return Ok(());
            }
            let r = if n == START {
                self.region
            } else {
                let c = atlas.portals[n].crossing;
                (c.sector, c.region)
            };
            if r == self.destination {
                self.trace = Some(n);
            } else {
                let (position, entry) = if n == START {
                    (self.start.position, None)
                } else {
                    (atlas.portals[n].crossing.position, atlas.portals[n].entry)
                };
                self.active = Some(Expansion {
                    parent: n,
                    remaining: atlas.outgoing.get(&r).map_or(&[], Vec::as_slice),
                    sector: atlas
                        .graph
                        .sector(atlas.profile, r.0)
                        .ok_or(Error::MeshChanged)?,
                    position,
                    entry,
                    cost: f32::from_bits(cost),
                });
            }
        } else {
            self.state = RouteState::Disconnected;
        }
        Ok(())
    }
    fn prefix(
        &mut self,
        atlas: &RouteAtlas<'_>,
        nav: &NavMesh,
        nodes: usize,
        budget: &mut Budget,
    ) -> Result<(), Error> {
        if self.fine.is_none() {
            if atlas.missing(nav, self.region.0, budget, &mut self.debt) {
                self.state = RouteState::Waiting(self.region.0);
                return Ok(());
            }
            let target = if self.region == self.destination {
                self.goal
            } else {
                let outgoing = atlas.outgoing.get(&self.region).ok_or(Error::Failed)?;
                while let Some(&i) = outgoing.get(self.prefix_index) {
                    if !budget.spend(1) {
                        return Ok(());
                    }
                    self.prefix_index += 1;
                    let portal = atlas.portals[i];
                    if !self.corridor.contains(&portal.destination) {
                        continue;
                    }
                    let c = portal.crossing;
                    let score = distance(self.start.position, c.position)
                        + distance(c.position, self.goal.position);
                    if self.prefix_target.is_none_or(|(best, _)| score < best) {
                        self.prefix_target = Some((score, c));
                    }
                }
                let Some((_, c)) = self.prefix_target else {
                    self.prefix_pending = false;
                    return Ok(());
                };
                WindowPoint::new(c.exit, c.position)
            };
            if !budget.spend(1) {
                return Ok(());
            }
            self.fine = Some(PortalWindow::new(
                self.start,
                target,
                alloc::vec![self.region.0],
                &atlas.filter,
                PREFIX_NODES.min(nodes),
            ));
        }
        // The prefix search is sized for its hash table: it stays short.
        let result = self.fine.as_mut().ok_or(Error::Failed)?.resume(
            nav,
            atlas.graph.params(),
            &atlas.filter,
            budget,
            PREFIX_NODES.min(nodes),
            true,
            |r| atlas.region(nav, r) == Ok(self.region),
        );
        match result {
            Ok(None) => return Ok(()),
            Ok(Some((segment, point))) => {
                self.published = segment.len() > 1 || point.position != self.start.position;
                self.segment = segment;
                self.start = point;
                self.state = RouteState::Segment;
            }
            Err(Error::LimitExceeded | Error::Failed) => {}
            Err(e) => return Err(e),
        }
        self.prefix_pending = false;
        self.fine = None;
        Ok(())
    }

    fn multi_window(
        &mut self,
        atlas: &RouteAtlas<'_>,
        nav: &NavMesh,
        nodes: usize,
        budget: &mut Budget,
    ) -> Result<bool, Error> {
        if self.lookahead == 0 || self.at_exit || self.window.is_some() {
            return Ok(false);
        }
        if self.fine.is_none() {
            let plan = self.plan.get_or_insert_with(|| WindowPlan {
                sectors: BTreeSet::from([self.region.0]),
                core: alloc::vec![self.region.0],
                margin_cursor: 0,
                crossings: 0,
            });
            loop {
                if !budget.spend(1) {
                    return Ok(true);
                }
                if plan.crossings < self.lookahead.min(self.route.len()) {
                    let id = self.route[self.route.len() - 1 - plan.crossings];
                    let key = atlas.portals[id].crossing.sector;
                    if atlas.missing(nav, key, budget, &mut self.debt) {
                        self.state = RouteState::Waiting(key);
                        return Ok(true);
                    }
                    if plan.sectors.insert(key) {
                        plan.core.push(key);
                    }
                    plan.crossings += 1;
                } else if let Some(k) = plan.core.get(plan.margin_cursor / 9) {
                    let offset = (plan.margin_cursor % 9) as i32;
                    let key = SectorKey {
                        x: k.x.saturating_add(offset / 3 - 1),
                        z: k.z.saturating_add(offset % 3 - 1),
                    };
                    if atlas.graph.sector(atlas.profile, key).is_some() {
                        if atlas.missing(nav, key, budget, &mut self.debt) {
                            self.state = RouteState::Waiting(key);
                            return Ok(true);
                        }
                        plan.sectors.insert(key);
                    }
                    plan.margin_cursor += 1;
                } else {
                    break;
                }
            }
            let plan = self.plan.take().ok_or(Error::Failed)?;
            self.skip = plan.crossings;
            let sectors = plan.sectors.into_iter().collect();
            let target = if self.skip == self.route.len() {
                self.goal
            } else {
                let c = atlas.portals[self.route[self.route.len() - self.skip]].crossing;
                WindowPoint::new(c.landing, c.position)
            };
            self.fine = Some(if let Some(mut scratch) = self.spare.take() {
                scratch.restart(self.start, target, sectors);
                scratch
            } else {
                PortalWindow::new(self.start, target, sectors, &atlas.filter, nodes)
            });
        }
        match self.fine.as_mut().ok_or(Error::Failed)?.resume(
            nav,
            atlas.graph.params(),
            &atlas.filter,
            budget,
            nodes,
            false,
            |_| true,
        ) {
            Ok(None) => {}
            Ok(Some((path, end))) => {
                self.segment = path;
                self.start = end;
                self.region = atlas.region(nav, end.polygon)?;
                self.route.truncate(self.route.len() - self.skip);
                self.spare = self.fine.take();
                self.published = true;
                self.state = RouteState::Segment;
            }
            Err(Error::LimitExceeded | Error::Failed) => {
                self.spare = self.fine.take();
                self.lookahead = self.skip / 2;
            }
            Err(e) => return Err(e),
        }
        Ok(true)
    }
    fn refine(
        &mut self,
        atlas: &RouteAtlas<'_>,
        nav: &NavMesh,
        nodes: usize,
        budget: &mut Budget,
    ) -> Result<(), Error> {
        if self.multi_window(atlas, nav, nodes, budget)? {
            return Ok(());
        }
        if self.window.is_none() {
            if !budget.spend(1) {
                return Ok(());
            }
            let next = self.route.last().map(|&i| atlas.portals[i].crossing);
            let end = next.map_or(self.goal, |c| WindowPoint::new(c.exit, c.position));
            // A complete sector is needed to reproduce the full-residency window.
            let tiles = atlas
                .graph
                .sector(atlas.profile, self.region.0)
                .ok_or(Error::MeshChanged)?
                .tiles();
            owe(budget, &mut self.debt, tiles.len() as u64);
            for t in tiles {
                if nav.tile_index_at(t.x, t.z, i32::from(t.layer)).is_none() {
                    self.state = RouteState::Waiting(self.region.0);
                    return Ok(());
                }
            }
            if self.at_exit {
                let c = next.ok_or(Error::Failed)?;
                if matches!(nav.ref_state(c.landing), RefState::Unloaded(_)) {
                    self.state = RouteState::Waiting(c.sector);
                    return Ok(());
                }
                let (tile, poly) = nav.tile_and_poly_by_ref(c.exit)?;
                let mut link = poly.first_link;
                loop {
                    if link == crate::detour::NULL_LINK {
                        return Err(Error::MeshChanged);
                    }
                    let e = tile.links.get(link as usize).ok_or(Error::InvalidRef)?;
                    if e.r#ref == c.landing {
                        break;
                    }
                    link = e.next;
                }
                self.route.pop();
                self.segment = alloc::vec![c.exit, c.landing];
                self.start = WindowPoint::new(c.landing, c.position);
                self.region = (c.sector, c.region);
                self.at_exit = false;
                self.state = RouteState::Segment;
                return Ok(());
            }
            let polys = atlas
                .graph
                .sector(atlas.profile, self.region.0)
                .ok_or(Error::MeshChanged)?
                .polygon_count() as usize;
            // A small pool first; after it runs out, the whole pool, which
            // `Navigator::new` sized to hold any sector of the atlas.
            let pool = if self.wide {
                nodes
            } else {
                nodes.min(polys.saturating_mul(4).max(8))
            };
            owe(budget, &mut self.debt, pool.div_ceil(NODES_PER_UNIT) as u64);
            self.pool = pool;
            self.window = Some(CorridorWindow::new(
                nav,
                atlas.graph.params(),
                &[self.region.0],
                atlas.filter.clone(),
                self.start,
                end,
                pool,
            )?);
            return Ok(());
        }
        let w = self.window.as_mut().ok_or(Error::Failed)?;
        if !w.resume(nav, budget, budget.remaining())?.is_done() || !budget.spend(1) {
            return Ok(());
        }
        let status = w.finish(nav, nodes)?;
        owe(
            budget,
            &mut self.debt,
            w.result().map_or(0, |(corridor, _)| corridor.len() as u64),
        );
        if status
            .has_detail(Status::PARTIAL_RESULT | Status::BUFFER_TOO_SMALL | Status::OUT_OF_NODES)
        {
            self.window = None;
            // Out of nodes in a smaller pool: retry the same window with the whole
            // pool before failing. A partial result in a complete pool means the
            // endpoint is unreachable inside the window; a larger pool cannot help.
            if !self.wide && status.has_detail(Status::OUT_OF_NODES) && self.pool < nodes {
                self.wide = true;
                return Ok(());
            }
            return Err(Error::LimitExceeded);
        }
        self.wide = false;
        self.segment = w.take_corridor();
        if let Some(&i) = self.route.last() {
            let c = atlas.portals[i].crossing;
            self.start = WindowPoint::new(c.exit, c.position);
            self.at_exit = true;
        } else {
            self.start = self.goal;
        }
        self.window = None;
        self.published = true;
        self.state = RouteState::Segment;
        Ok(())
    }
}

/// A round-robin scheduler with a shared expansion budget. Requests awaiting
/// their first segment take precedence over acknowledged continuations.
#[non_exhaustive]
pub struct Navigator<'a> {
    atlas: &'a RouteAtlas<'a>,
    requests: BTreeMap<RequestId, Request<'a>>,
    ready: BTreeMap<(bool, Reverse<u8>), VecDeque<RequestId>>,
    next: u64,
    nodes: usize,
    epoch: u64,
}
impl<'a> Navigator<'a> {
    /// Creates a scheduler with a bounded Detour node pool per active window.
    ///
    /// `nodes` must hold every passable polygon of the atlas's largest sector, so a
    /// single-sector refinement window can always complete. Errors:
    /// [`Error::InvalidParameter`] when `nodes` is above 65,535 or below that
    /// polygon count (or zero).
    pub fn new(atlas: &'a RouteAtlas<'a>, nodes: usize) -> Result<Self, Error> {
        let largest = atlas
            .graph
            .sectors
            .iter()
            .filter(|((p, _), _)| *p == atlas.profile)
            .map(|(_, s)| s.polygon_count() as usize)
            .max()
            .unwrap_or(0);
        if !(largest.max(1)..=65535).contains(&nodes) {
            return Err(Error::InvalidParameter("window nodes"));
        }
        Ok(Self {
            atlas,
            requests: BTreeMap::new(),
            ready: BTreeMap::new(),
            next: 0,
            nodes,
            epoch: atlas.epoch,
        })
    }
    /// Queues a route between polygons captured in the atlas.
    pub fn request(
        &mut self,
        nav: &NavMesh,
        start: WindowPoint,
        goal: WindowPoint,
    ) -> Result<RequestId, Error> {
        self.request_with_priority(nav, start, goal, 0)
    }
    /// Sets the request's priority within the first-segment or continuation queue.
    /// Higher priorities run first. Equal priorities take turns in submission order.
    pub fn request_with_priority(
        &mut self,
        nav: &NavMesh,
        start: WindowPoint,
        goal: WindowPoint,
        priority: u8,
    ) -> Result<RequestId, Error> {
        self.atlas.validate(nav, self.epoch)?;
        if !start
            .position
            .into_iter()
            .chain(goal.position)
            .all(f32::is_finite)
        {
            return Err(Error::InvalidParameter("route point"));
        }
        let region = self.atlas.region(nav, start.polygon)?;
        let destination = self.atlas.region(nav, goal.polygon)?;
        let coarse = self.atlas.coarse.request(
            self.atlas.coarse_key(region)?,
            self.atlas.coarse_key(destination)?,
        )?;
        let id = RequestId(self.next);
        self.next = self.next.checked_add(1).ok_or(Error::SizeOverflow)?;
        self.requests.insert(
            id,
            Request {
                priority,
                coarse: Some(coarse),
                corridor: BTreeSet::new(),
                coarse_cursor: 0,
                start,
                goal,
                region,
                destination,
                open: BinaryHeap::from([Reverse((0, 0, START))]),
                labels: PortalLabels::new(self.atlas.portals.len()),
                active: None,
                trace: None,
                route: Vec::new(),
                refining: false,
                at_exit: false,
                window: None,
                held_prefix: None,
                segment: Vec::new(),
                corners: Vec::new(),
                state: RouteState::Planning,
                published: false,
                prefix_pending: true,
                prefix_index: 0,
                prefix_target: None,
                plan: None,
                fine: None,
                spare: None,
                skip: 0,
                debt: 0,
                wide: false,
                pool: 0,
                lookahead: if self
                    .atlas
                    .graph
                    .sector(self.atlas.profile, region.0)
                    .is_some_and(|s| s.polygon_count() > 1024)
                {
                    3
                } else {
                    32
                },
            },
        );
        self.ready
            .entry((false, Reverse(priority)))
            .or_default()
            .push_back(id);
        Ok(id)
    }
    /// Validates an intervening mesh edit and wakes waiting windows after streaming.
    /// Call outside the frame budget. An edit to content invalidates the atlas and
    /// fails every request with [`Error::MeshChanged`]. A request whose published
    /// segment or prefix crosses an unloaded tile fails alone, with
    /// [`RouteState::Failed`]`(`[`Error::MeshChanged`]`)`; the others continue.
    pub fn sync(&mut self, nav: &NavMesh) -> Result<(), Error> {
        if let Err(error) = self.atlas.validate(nav, self.epoch) {
            self.invalidate();
            return Err(error);
        }
        if nav.epoch() != self.epoch {
            let lost: Vec<RequestId> = self
                .requests
                .iter()
                .filter(|(_, r)| {
                    r.held_prefix.as_ref().is_some_and(|p| {
                        p.path.iter().any(|r| nav.ref_state(*r) != RefState::Valid)
                    }) || r.state == RouteState::Segment
                        && r.segment
                            .iter()
                            .any(|p| nav.ref_state(*p) != RefState::Valid)
                })
                .map(|(&id, _)| id)
                .collect();
            for id in &lost {
                self.ready.retain(|_, queue| {
                    queue.retain(|k| k != id);
                    !queue.is_empty()
                });
                if let Some(r) = self.requests.get_mut(id) {
                    Self::fail(r);
                }
            }
            for (&id, r) in &mut self.requests {
                if lost.contains(&id) {
                    continue;
                }
                r.plan = None;
                r.fine = None;
                r.window = None;
                if matches!(r.state, RouteState::Waiting(_)) {
                    r.state = RouteState::Planning;
                    self.ready
                        .entry((r.published, Reverse(r.priority)))
                        .or_default()
                        .push_back(id);
                }
            }
            self.epoch = nav.epoch();
        }
        Ok(())
    }
    fn invalidate(&mut self) {
        self.ready.clear();
        for r in self.requests.values_mut() {
            Self::fail(r);
        }
    }
    fn fail(r: &mut Request<'_>) {
        r.window = None;
        r.held_prefix = None;
        r.fine = None;
        r.segment.clear();
        r.corners.clear();
        r.state = RouteState::Failed(Error::MeshChanged);
    }
    /// Runs one request for at most the given slice of the caller's shared budget.
    /// Returns true when a request ran. False ends the update loop when the
    /// budget is empty or every request is waiting for the caller.
    pub fn update(
        &mut self,
        nav: &NavMesh,
        budget: &mut Budget,
        max_unit: u64,
    ) -> Result<bool, Error> {
        if nav.epoch() != self.epoch {
            return Err(Error::MeshChanged);
        }
        if let Err(error) = self.atlas.validate(nav, self.epoch) {
            // Never leave a corridor through the edit readable from the getters.
            self.invalidate();
            return Err(error);
        }
        let limit = budget.remaining().min(max_unit);
        if limit == 0 {
            return Ok(false);
        }
        let Some(mut queue) = self.ready.first_entry() else {
            return Ok(false);
        };
        let id = queue.get_mut().pop_front().ok_or(Error::Failed)?;
        if queue.get().is_empty() {
            queue.remove();
        }
        let r = self.requests.get_mut(&id).ok_or(Error::Failed)?;
        let origin = r.start.position;
        let was_prefix = r.prefix_pending;
        let mut slice = Budget::new(limit);
        while r.state == RouteState::Planning && slice.remaining() > 0 {
            if let Err(error) = r.step(self.atlas, nav, self.nodes, &mut slice) {
                r.state = RouteState::Failed(error);
            }
        }
        if r.state == RouteState::Segment {
            // The funnel examines each corridor portal once (plus the query setup);
            // what the slice cannot pay now is paid before the request's next step.
            owe(&mut slice, &mut r.debt, r.segment.len() as u64 + 2);
            let pulled = crate::detour::NavMeshQuery::new(nav, 1).and_then(|q| {
                let status = q.find_straight_path(
                    &origin,
                    &r.start.position,
                    &r.segment,
                    &mut r.corners,
                    r.segment.len().saturating_add(2),
                    0,
                )?;
                if status.has_detail(Status::PARTIAL_RESULT | Status::BUFFER_TOO_SMALL) {
                    return Err(Error::Failed);
                }
                Ok(())
            });
            if let Err(e) = pulled {
                r.state = RouteState::Failed(e);
            } else if was_prefix
                && !r.refining
                && (r.start.polygon != r.goal.polygon || r.start.position != r.goal.position)
            {
                // The caller can walk this prefix while the remaining portal
                // search and its first refinement window use subsequent slices.
                r.held_prefix = Some(Segment {
                    path: core::mem::take(&mut r.segment),
                    corners: core::mem::take(&mut r.corners),
                    end: r.start,
                });
                r.state = RouteState::Planning;
            }
        }
        budget.spend(slice.used());
        if r.state == RouteState::Planning {
            self.ready
                .entry((r.published, Reverse(r.priority)))
                .or_default()
                .push_back(id);
        }
        Ok(true)
    }
    /// Current state of a request.
    pub fn state(&self, id: RequestId) -> Option<RouteState> {
        self.requests.get(&id).map(|r| {
            if r.held_prefix.is_some() {
                RouteState::Segment
            } else {
                r.state.clone()
            }
        })
    }
    /// Next contiguous polygon corridor ready to follow.
    pub fn segment(&self, id: RequestId) -> Option<&[PolyRef]> {
        let r = self.requests.get(&id)?;
        if let Some(p) = &r.held_prefix {
            Some(p.path.as_slice())
        } else {
            (r.state == RouteState::Segment).then_some(r.segment.as_slice())
        }
    }
    /// String-pulled corners of the published corridor, including turns across sector borders.
    pub fn corners(&self, id: RequestId) -> Option<&[crate::detour::StraightPathPoint]> {
        let r = self.requests.get(&id)?;
        if let Some(p) = &r.held_prefix {
            Some(p.corners.as_slice())
        } else {
            (r.state == RouteState::Segment).then_some(r.corners.as_slice())
        }
    }
    /// Position to reach on the last polygon of the published segment.
    pub fn toward(&self, id: RequestId) -> Option<WindowPoint> {
        let r = self.requests.get(&id)?;
        if let Some(p) = &r.held_prefix {
            Some(p.end)
        } else {
            (r.state == RouteState::Segment).then_some(r.start)
        }
    }
    /// Acknowledges reaching the segment's endpoint and schedules its successor.
    pub fn advance(&mut self, id: RequestId) -> Result<(), Error> {
        let r = self.requests.get_mut(&id).ok_or(Error::InvalidRef)?;
        if r.held_prefix.take().is_some() {
            return Ok(());
        }
        if r.state != RouteState::Segment {
            return Err(Error::InvalidParameter("no ready segment"));
        }
        r.segment.clear();
        if r.route.is_empty()
            && r.start.polygon == r.goal.polygon
            && r.start.position == r.goal.position
        {
            r.state = RouteState::Complete;
        } else {
            r.state = RouteState::Planning;
            self.ready
                .entry((true, Reverse(r.priority)))
                .or_default()
                .push_back(id);
        }
        Ok(())
    }
    /// Releases a queued or completed request.
    pub fn cancel(&mut self, id: RequestId) -> bool {
        self.ready.retain(|_, queue| {
            queue.retain(|k| *k != id);
            !queue.is_empty()
        });
        self.requests.remove(&id).is_some()
    }
}

#[cfg(test)]
mod label_regression {
    use super::{distance, saturated};

    #[test]
    fn huge_coordinates_keep_labels_finite_and_ordered() {
        let (a, b) = ([-3e38f32, 0.0, -3e38], [3e38f32, 0.0, 3e38]);
        // The single-precision sum overflows; the distance must not.
        let d = distance(a, b);
        assert_eq!(d, f32::MAX);
        // A zero minimum area cost gives zero, not NaN (0 x inf).
        assert_eq!(saturated(d * 0.0), 0.0);
        // A label past the largest float still orders before "unreached".
        let label = saturated(d + d * 2.0);
        assert!(label.is_finite() && label < f32::INFINITY);
        assert_eq!(distance([0.0; 3], [3.0, 0.0, 4.0]), 5.0);
        assert!((distance([1e30, 0.0, 0.0], [-1e30, 0.0, 0.0]) / 2e30 - 1.0).abs() < 1e-6);
    }
}
