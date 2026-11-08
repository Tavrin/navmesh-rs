//! Shared goal fields (not in recastnavigation).
//!
//! A [`GoalField`] answers "which way to the goal?" for every polygon of a region
//! at once, so thousands of agents with one destination share one search instead
//! of running one path request each. It is a Dijkstra flood from the goal over
//! the reverse polygon graph with Detour's cost model: a polygon is entered at the
//! midpoint of its portal and the segment across it is charged by
//! [`QueryFilter::cost`], as in [`NavMeshQuery::find_path`], so a field agent and a
//! corridor agent agree on what is short.
//!
//! # Values
//!
//! For each polygon `p` of the region the field stores `cost(p)`, the cost to the
//! goal from the midpoint of the portal through which `p` is left, and `next(p)`,
//! the polygon on the other side of that portal. The goal polygon has cost 0 and
//! its anchor is the goal point. For a polygon `p` whose next polygon is `n`, with
//! `m` the midpoint of the portal from `p` to `n` and `a` the anchor of `n`:
//!
//! ```text
//! cost(p) = cost(n) + filter.cost(m, a, Some(p), n, next(n))
//! ```
//!
//! Polygons are settled in increasing `(cost, tile x, tile y, tile layer, polygon
//! index)` order and a value is replaced only by a strictly smaller one, so the
//! result does not depend on tile slots or on the order tiles were added. A
//! polygon is a candidate when it has a link to the settled polygon and passes the
//! filter (the goal polygon is not filtered, as the end of a path is not); costs
//! that are negative or not finite make the step unusable.
//!
//! Links are directed. Ground polygons are linked in both directions, so the
//! flood reads a settled polygon's own links back. Links without a partner (one-way
//! off-mesh connections) are found by a reverse index built in a scan of the region
//! before the flood.
//!
//! # Regions, work and memory
//!
//! A field covers the tile columns of a [`FieldRegion`]: a radius around the goal
//! or an explicit tile set (a coarse route plus a margin, see
//! [`FieldRegion::route`]). Storage is per polygon of the region's tiles (a cost
//! and a next index, 8 bytes), so the field has no node-pool cap; region size is
//! bounded by the caller's byte limit instead, and a region over it is refused
//! with [`Error::LimitExceeded`], never truncated. While a field is being built it
//! holds about 40 more bytes per polygon.
//!
//! [`GoalField::step`] is budgeted and resumable: one unit per polygon scanned for
//! the reverse index and one per polygon taken from the open list.
//!
//! # Invalidation
//!
//! After a structural change ([`NavMesh::epoch`] moved), the next
//! [`GoalField::step`] or [`GoalField::refresh`] compares the region's tiles with
//! the ones the field was built from. A tile that was removed, unloaded or
//! replaced makes its polygons unusable, and every polygon whose chain of next
//! polygons passes through it as well (the field is stale from that tile
//! outward); the remaining values are still served. A tile committed into the
//! region invalidates nothing but can open shorter routes. Either way the field
//! becomes [`FieldStatus::Stale`] and a fresh build starts, budgeted like the
//! first one; when it completes it replaces the served values. With
//! [`NavMesh::set_global_off_mesh_links`] on, any structural change rebuilds the
//! field (a far tile can take over an off-mesh connection's landing).
//!
//! # Steering
//!
//! [`GoalField::steer`] follows the next polygons from the agent's polygon for at
//! most three steps (a four-polygon corridor stub) and runs the string-pulling
//! funnel of [`NavMeshQuery::find_straight_path`] through them, towards the goal
//! point or the midpoint of the portal leaving the stub. The first corner is the
//! steering target: the closest point of the next portal when the straight line
//! does not pass through it, a point further along when it does.
//!
//! [`GoalFieldCache`] shares fields between requests with the same goal, filter
//! and region, counts references, and evicts unreferenced fields, least recently
//! used first, to stay within a byte cap.

use crate::detour::math::visfinite;
use crate::detour::{
    NULL_LINK, NavMesh, NavMeshQuery, PolyRef, QueryFilter, StraightPathPoint, TileKey,
};
use crate::{Budget, Budgeted, Error, Limits, Progress};
use alloc::collections::{BTreeMap, BinaryHeap};
use alloc::sync::Arc;
use alloc::vec;
use alloc::vec::Vec;
use core::cmp::Ordering;

/// No next polygon (unreached, invalidated, or the goal).
const NONE: u32 = u32::MAX;
/// Bytes per polygon of the served values (cost and next index).
const SERVED_BYTES_PER_POLY: usize = 8;
/// Upper estimate of the bytes per polygon a build holds besides the served values:
/// cost, next, anchor, settled flag, open-list entries and reverse-index entries.
const BUILD_BYTES_PER_POLY: usize = 48;
/// Bytes per region tile record.
const TILE_BYTES: usize = core::mem::size_of::<FieldTile>() + 8;
/// Polygons in a steering stub: the agent's polygon and three more.
const STUB: usize = 4;

/// The tiles a goal field covers.
#[derive(Clone, Debug, PartialEq)]
#[non_exhaustive]
pub enum FieldRegion {
    /// Every tile column whose rectangle lies within this distance (xz, metres)
    /// of the goal point, limited to the navmesh's occupied tile bounds.
    Radius(f32),
    /// These tile columns (order and duplicates do not matter).
    Tiles(Vec<TileKey>),
}

impl FieldRegion {
    /// The columns of `route` and every column within `margin` columns of them
    /// (a square neighbourhood): the region for a coarse route.
    pub fn route(route: &[TileKey], margin: u8) -> FieldRegion {
        let m = i32::from(margin);
        let mut keys = Vec::new();
        for k in route {
            for dy in -m..=m {
                for dx in -m..=m {
                    keys.push(TileKey {
                        x: k.x.saturating_add(dx),
                        y: k.y.saturating_add(dy),
                    });
                }
            }
        }
        keys.sort_unstable();
        keys.dedup();
        FieldRegion::Tiles(keys)
    }
}

/// Where a goal field is in its life.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
#[non_exhaustive]
pub enum FieldStatus {
    /// The first build is running; nothing is served yet.
    Building,
    /// Values are complete for the current navmesh.
    Ready,
    /// A structural change touched the region: values not affected by it are
    /// served while a rebuild runs.
    Stale,
    /// The goal polygon is no longer resident; nothing is served. The field
    /// rebuilds if a later change brings the polygon back.
    GoalLost,
}

/// One resident tile of a field's region.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
struct FieldTile {
    x: i32,
    y: i32,
    layer: i32,
    slot: u32,
    /// Tile ref (salt and slot) the values were built for.
    base: PolyRef,
    /// Index of the tile's first polygon in the field's arrays.
    offset: u32,
    polys: u32,
}

/// The region's resident tiles, in `(x, y, layer)` order, so field indices
/// follow `(tile key, polygon index)`.
#[derive(Clone, Debug, Default)]
struct Layout {
    tiles: Vec<FieldTile>,
    /// `(slot, index into tiles)`, sorted by slot.
    by_slot: Vec<(u32, u32)>,
    total: u32,
}

/// Result of comparing a layout with the navmesh.
struct Changes {
    /// Per layout tile: removed, unloaded or replaced.
    lost: Vec<bool>,
    /// Anything differs, including tiles added to a region column.
    any: bool,
}

impl Layout {
    fn resolve(nav: &NavMesh, columns: &[TileKey]) -> Result<Layout, Error> {
        let mut tiles = Vec::new();
        let mut offset = 0u32;
        for c in columns {
            let mut col: Vec<(i32, usize)> = nav
                .tiles_at(c.x, c.y, usize::MAX)
                .into_iter()
                .filter_map(|s| nav.tile(s).map(|t| (t.header.layer, s)))
                .collect();
            col.sort_unstable();
            for (layer, slot) in col {
                let Some(tile) = nav.tile(slot) else {
                    continue;
                };
                let polys = u32::try_from(tile.polys.len()).map_err(|_| Error::SizeOverflow)?;
                tiles.push(FieldTile {
                    x: c.x,
                    y: c.y,
                    layer,
                    slot: u32::try_from(slot).map_err(|_| Error::SizeOverflow)?,
                    base: nav.tile_ref(slot),
                    offset,
                    polys,
                });
                offset = offset
                    .checked_add(polys)
                    .filter(|&o| o < NONE)
                    .ok_or(Error::SizeOverflow)?;
            }
        }
        let mut by_slot: Vec<(u32, u32)> = tiles
            .iter()
            .enumerate()
            .map(|(i, t)| (t.slot, i as u32))
            .collect();
        by_slot.sort_unstable();
        Ok(Layout {
            tiles,
            by_slot,
            total: offset,
        })
    }

    /// Field index of `r`, when `r` names a polygon of a region tile as built.
    fn local(&self, nav: &NavMesh, r: PolyRef) -> Option<u32> {
        let (salt, it, ip) = nav.decode_poly_id(r);
        let k = self.by_slot.binary_search_by_key(&it, |e| e.0).ok()?;
        let t = self.tiles.get(self.by_slot[k].1 as usize)?;
        if t.base == 0 || t.base != nav.encode_poly_id(salt, it, 0) || ip >= t.polys {
            return None;
        }
        Some(t.offset + ip)
    }

    fn tile_of(&self, i: u32) -> Option<&FieldTile> {
        let k = self.tiles.partition_point(|t| t.offset + t.polys <= i);
        self.tiles.get(k)
    }

    fn poly_ref(&self, i: u32) -> PolyRef {
        match self.tile_of(i) {
            Some(t) => t.base | (i - t.offset) as PolyRef,
            None => 0,
        }
    }

    fn bytes(&self) -> usize {
        self.tiles.capacity() * TILE_BYTES
    }

    /// Compares the tiles with the navmesh's current ones (`since`: the epoch the
    /// layout was checked at; `all`: treat every tile as lost).
    fn changes(&self, nav: &NavMesh, columns: &[TileKey], since: u64, all: bool) -> Changes {
        let mut lost = vec![all; self.tiles.len()];
        let mut any = all;
        for c in columns {
            let mut now: Vec<(i32, u32, PolyRef)> = nav
                .tiles_at(c.x, c.y, usize::MAX)
                .into_iter()
                .filter_map(|s| {
                    nav.tile(s)
                        .map(|t| (t.header.layer, s as u32, nav.tile_ref(s)))
                })
                .collect();
            now.sort_unstable();
            let first = self.tiles.partition_point(|t| (t.x, t.y) < (c.x, c.y));
            let mut count = 0;
            for (i, t) in self.tiles[first..].iter().enumerate() {
                if (t.x, t.y) != (c.x, c.y) {
                    break;
                }
                count += 1;
                let present = now
                    .iter()
                    .any(|&(l, s, b)| l == t.layer && s == t.slot && b == t.base);
                if !present || nav.changed_since(since, &mut |s| s == t.slot) {
                    lost[first + i] = true;
                    any = true;
                }
            }
            if now.len() != count {
                any = true;
            }
        }
        Changes { lost, any }
    }
}

/// Open-list entry ordered for a min-heap on `(cost, index)`.
#[derive(Clone, Copy, Debug)]
struct Item {
    cost: f32,
    node: u32,
}

impl PartialEq for Item {
    fn eq(&self, o: &Self) -> bool {
        self.cmp(o) == Ordering::Equal
    }
}
impl Eq for Item {}
impl PartialOrd for Item {
    fn partial_cmp(&self, o: &Self) -> Option<Ordering> {
        Some(self.cmp(o))
    }
}
impl Ord for Item {
    fn cmp(&self, o: &Self) -> Ordering {
        o.cost
            .total_cmp(&self.cost)
            .then_with(|| o.node.cmp(&self.node))
    }
}

/// Values served to steering and lookups.
#[derive(Clone, Debug)]
struct Served {
    layout: Layout,
    goal: u32,
    cost: Vec<f32>,
    next: Vec<u32>,
}

impl Served {
    fn bytes(&self) -> usize {
        self.layout.bytes() + self.cost.capacity() * 4 + self.next.capacity() * 4
    }

    /// Clears every polygon in a lost tile and every polygon whose chain of next
    /// polygons reaches one.
    fn invalidate(&mut self, lost: &[bool]) {
        let n = self.cost.len();
        // 0 unknown, 1 usable, 2 cleared.
        let mut state = vec![0u8; n];
        for (t, tile) in self.layout.tiles.iter().enumerate() {
            if lost.get(t).copied().unwrap_or(true) {
                let end = (tile.offset + tile.polys) as usize;
                for s in &mut state[tile.offset as usize..end.min(n)] {
                    *s = 2;
                }
            }
        }
        let mut chain = Vec::new();
        for start in 0..n {
            let mut i = start;
            chain.clear();
            let verdict = loop {
                match state[i] {
                    0 => {}
                    v => break v,
                }
                chain.push(i);
                if i == self.goal as usize || self.next[i] == NONE || chain.len() > n {
                    break 1;
                }
                i = self.next[i] as usize;
                if i >= n {
                    break 2;
                }
            };
            for &c in &chain {
                state[c] = verdict;
            }
        }
        for (i, s) in state.iter().enumerate() {
            if *s == 2 {
                self.cost[i] = f32::INFINITY;
                self.next[i] = NONE;
            }
        }
    }
}

/// A build in progress: reverse-index scan, then the flood.
#[derive(Clone, Debug)]
struct Build {
    layout: Layout,
    goal: u32,
    cost: Vec<f32>,
    next: Vec<u32>,
    anchor: Vec<[f32; 3]>,
    done: Vec<bool>,
    /// Field index -> sources of links into it that it has no link back to.
    reverse: BTreeMap<u32, Vec<u32>>,
    heap: BinaryHeap<Item>,
    /// Next polygon of the reverse-index scan (`total` once it is complete).
    cursor: u32,
}

/// Whether `from` has a link to `to`.
fn has_link(q: &NavMeshQuery<'_>, from: PolyRef, to: PolyRef) -> bool {
    let (_, tile, poly) = q.tp(from);
    let mut li = poly.first_link;
    while li != NULL_LINK {
        let Some(l) = tile.links.get(li as usize) else {
            return false;
        };
        if l.r#ref == to {
            return true;
        }
        li = l.next;
    }
    false
}

fn mid(l: &[f32; 3], r: &[f32; 3]) -> [f32; 3] {
    [
        (l[0] + r[0]) * 0.5,
        (l[1] + r[1]) * 0.5,
        (l[2] + r[2]) * 0.5,
    ]
}

impl Build {
    /// A build over the current tiles of `columns`; `None` when the goal polygon
    /// is not resident in them.
    fn start(
        nav: &NavMesh,
        columns: &[TileKey],
        goal: PolyRef,
        goal_pos: &[f32; 3],
    ) -> Result<Option<Build>, Error> {
        if nav.tile_and_poly_by_ref(goal).is_err() {
            return Ok(None);
        }
        let layout = Layout::resolve(nav, columns)?;
        let Some(g) = layout.local(nav, goal) else {
            return Ok(None);
        };
        let n = layout.total as usize;
        let mut b = Build {
            layout,
            goal: g,
            cost: vec![f32::INFINITY; n],
            next: vec![NONE; n],
            anchor: vec![[0.0; 3]; n],
            done: vec![false; n],
            reverse: BTreeMap::new(),
            heap: BinaryHeap::new(),
            cursor: 0,
        };
        b.cost[g as usize] = 0.0;
        b.anchor[g as usize] = *goal_pos;
        b.heap.push(Item { cost: 0.0, node: g });
        Ok(Some(b))
    }

    fn bytes(&self) -> usize {
        self.layout.bytes()
            + self.cost.capacity() * 4
            + self.next.capacity() * 4
            + self.anchor.capacity() * 12
            + self.done.capacity()
            + self.heap.capacity() * core::mem::size_of::<Item>()
            + self
                .reverse
                .values()
                .map(|v| v.capacity() * 4 + 32)
                .sum::<usize>()
    }

    /// Advances the build; `true` when it is complete.
    fn run<F: QueryFilter + ?Sized>(
        &mut self,
        nav: &NavMesh,
        filter: &F,
        budget: &mut Budget,
    ) -> Result<bool, Error> {
        let q = NavMeshQuery::new(nav, 1)?;
        while self.cursor < self.layout.total {
            if !budget.spend(1) {
                return Ok(false);
            }
            let i = self.cursor;
            let r = self.layout.poly_ref(i);
            let (_, tile, poly) = q.tp(r);
            let mut li = poly.first_link;
            while li != NULL_LINK {
                let Some(l) = tile.links.get(li as usize) else {
                    break;
                };
                li = l.next;
                if let Some(j) = self.layout.local(nav, l.r#ref) {
                    if !has_link(&q, l.r#ref, r) {
                        self.reverse.entry(j).or_default().push(i);
                    }
                }
            }
            self.cursor += 1;
        }
        let Build {
            layout,
            cost,
            next,
            anchor,
            done,
            reverse,
            heap,
            ..
        } = self;
        loop {
            let Some(&top) = heap.peek() else {
                return Ok(true);
            };
            if !budget.spend(1) {
                return Ok(false);
            }
            heap.pop();
            let n = top.node as usize;
            if done[n] || top.cost.to_bits() != cost[n].to_bits() {
                continue;
            }
            done[n] = true;
            let nref = layout.poly_ref(top.node);
            let ntp = q.tp(nref);
            let after = (next[n] != NONE).then(|| q.tp(layout.poly_ref(next[n])));
            let mut relax = |p: u32| {
                let pi = p as usize;
                if done[pi] {
                    return;
                }
                let pref = layout.poly_ref(p);
                let ptp = q.tp(pref);
                if !filter.pass_filter(pref, ptp.1, ptp.2) {
                    return;
                }
                let Some((l, r)) = q.portal_points_tp(ptp, ntp) else {
                    return;
                };
                let m = mid(&l, &r);
                let c = filter.cost(&m, &anchor[n], Some(ptp), ntp, after);
                if !(c.is_finite() && c >= 0.0) {
                    return;
                }
                let total = cost[n] + c;
                if total < cost[pi] {
                    cost[pi] = total;
                    next[pi] = top.node;
                    anchor[pi] = m;
                    heap.push(Item {
                        cost: total,
                        node: p,
                    });
                }
            };
            let mut li = ntp.2.first_link;
            while li != NULL_LINK {
                let Some(l) = ntp.1.links.get(li as usize) else {
                    break;
                };
                li = l.next;
                if let Some(p) = layout.local(nav, l.r#ref) {
                    if has_link(&q, l.r#ref, nref) {
                        relax(p);
                    }
                }
            }
            if let Some(sources) = reverse.get(&top.node) {
                for &p in sources {
                    relax(p);
                }
            }
        }
    }
}

/// A goal field over a region: per-polygon cost to the goal and next polygon
/// (see the [module documentation](self)).
///
/// The field is tied to the navmesh it was created on; calls with another
/// navmesh, including a clone, return [`Error::MeshChanged`].
#[derive(Clone, Debug)]
pub struct GoalField {
    identity: Arc<()>,
    goal: PolyRef,
    goal_pos: [f32; 3],
    columns: Vec<TileKey>,
    seen_epoch: u64,
    seen_global: bool,
    served: Option<Served>,
    build: Option<Build>,
    lost: bool,
}

impl GoalField {
    /// A field towards `goal_pos` on polygon `goal` over `region`; build it with
    /// [`GoalField::step`]. The goal polygon must lie in the region.
    ///
    /// # Errors
    /// [`Error::InvalidRef`] for a goal polygon that is not resident,
    /// [`Error::InvalidParameter`] for a non-finite goal point, a negative or
    /// non-finite radius or a goal outside the region, and
    /// [`Error::LimitExceeded`] when the region's estimated peak memory (about 56
    /// bytes per polygon) exceeds `limits.max_bytes`.
    pub fn new(
        nav: &NavMesh,
        goal: PolyRef,
        goal_pos: [f32; 3],
        region: &FieldRegion,
        limits: &Limits,
    ) -> Result<GoalField, Error> {
        let (columns, estimate) = Self::plan(nav, goal, &goal_pos, region)?;
        if estimate > limits.max_bytes {
            return Err(Error::LimitExceeded);
        }
        let build = Build::start(nav, &columns, goal, &goal_pos)?.ok_or(Error::InvalidRef)?;
        Ok(GoalField {
            identity: Arc::clone(nav.report_identity()),
            goal,
            goal_pos,
            columns,
            seen_epoch: nav.epoch(),
            seen_global: nav.global_off_mesh_links(),
            served: None,
            build: Some(build),
            lost: false,
        })
    }

    /// Estimated peak memory of a field for this request: what
    /// [`GoalField::new`] compares with `limits.max_bytes` and what
    /// [`GoalFieldCache::request`] needs free.
    ///
    /// # Errors
    /// As [`GoalField::new`], without the limit check.
    pub fn peak_bytes(
        nav: &NavMesh,
        goal: PolyRef,
        goal_pos: [f32; 3],
        region: &FieldRegion,
    ) -> Result<usize, Error> {
        Ok(Self::plan(nav, goal, &goal_pos, region)?.1)
    }

    /// Validates a request; returns the region's columns and the peak byte estimate.
    fn plan(
        nav: &NavMesh,
        goal: PolyRef,
        goal_pos: &[f32; 3],
        region: &FieldRegion,
    ) -> Result<(Vec<TileKey>, usize), Error> {
        if !visfinite(goal_pos) {
            return Err(Error::InvalidParameter("goal position"));
        }
        let (tile, _) = nav
            .tile_and_poly_by_ref(goal)
            .map_err(|_| Error::InvalidRef)?;
        let goal_key = TileKey {
            x: tile.header.x,
            y: tile.header.y,
        };
        let columns = region_columns(nav, goal_pos, region)?;
        if columns.binary_search(&goal_key).is_err() {
            return Err(Error::InvalidParameter("goal outside the field region"));
        }
        let layout = Layout::resolve(nav, &columns)?;
        let estimate = (layout.total as usize)
            .checked_mul(SERVED_BYTES_PER_POLY + BUILD_BYTES_PER_POLY)
            .and_then(|b| b.checked_add(layout.tiles.len().checked_mul(TILE_BYTES)?))
            .ok_or(Error::SizeOverflow)?;
        Ok((columns, estimate))
    }

    /// The goal polygon.
    pub fn goal(&self) -> PolyRef {
        self.goal
    }

    /// The goal point.
    pub fn goal_pos(&self) -> [f32; 3] {
        self.goal_pos
    }

    /// The region's tile columns, sorted.
    pub fn columns(&self) -> &[TileKey] {
        &self.columns
    }

    /// Where the field is in its life.
    pub fn status(&self) -> FieldStatus {
        if self.lost {
            FieldStatus::GoalLost
        } else {
            match (&self.served, &self.build) {
                (None, _) => FieldStatus::Building,
                (Some(_), Some(_)) => FieldStatus::Stale,
                (Some(_), None) => FieldStatus::Ready,
            }
        }
    }

    /// Polygons in the served values (0 before the first build completes).
    pub fn polys(&self) -> u32 {
        self.served.as_ref().map_or(0, |s| s.layout.total)
    }

    /// Bytes of heap memory the field holds now (served values and any build).
    pub fn bytes(&self) -> usize {
        self.columns.capacity() * core::mem::size_of::<TileKey>()
            + self.served.as_ref().map_or(0, Served::bytes)
            + self.build.as_ref().map_or(0, Build::bytes)
    }

    fn check(&self, nav: &NavMesh) -> Result<(), Error> {
        if Arc::ptr_eq(&self.identity, nav.report_identity()) {
            Ok(())
        } else {
            Err(Error::MeshChanged)
        }
    }

    /// Applies structural changes since the last call (see the module's
    /// invalidation section). Returns whether the field went stale or restarted
    /// its build. Unbudgeted: linear in the region's tiles and served polygons.
    ///
    /// # Errors
    /// [`Error::MeshChanged`] for another navmesh; allocation and size errors of
    /// a restarted build.
    pub fn refresh(&mut self, nav: &NavMesh) -> Result<bool, Error> {
        self.check(nav)?;
        let global = nav.global_off_mesh_links();
        if nav.epoch() == self.seen_epoch && global == self.seen_global {
            return Ok(false);
        }
        let all = global || self.seen_global;
        let mut restart = self.lost;
        if let Some(s) = &mut self.served {
            let c = s.layout.changes(nav, &self.columns, self.seen_epoch, all);
            if c.lost.iter().any(|&l| l) {
                s.invalidate(&c.lost);
            }
            restart |= c.any;
        }
        if let Some(b) = &self.build {
            restart |= b
                .layout
                .changes(nav, &self.columns, self.seen_epoch, all)
                .any;
        }
        self.seen_epoch = nav.epoch();
        self.seen_global = global;
        if restart {
            self.build = Build::start(nav, &self.columns, self.goal, &self.goal_pos)?;
            self.lost = self.build.is_none();
        }
        Ok(restart)
    }

    /// Refreshes (see [`GoalField::refresh`]) and advances the build, if any, with
    /// `filter` (pass the same filter on every call). `Done` when the field is
    /// [`Ready`](FieldStatus::Ready) or [`GoalLost`](FieldStatus::GoalLost).
    ///
    /// # Errors
    /// As [`GoalField::refresh`].
    pub fn step<F: QueryFilter + ?Sized>(
        &mut self,
        nav: &NavMesh,
        filter: &F,
        budget: &mut Budget,
    ) -> Result<Progress, Error> {
        self.refresh(nav)?;
        let Some(build) = &mut self.build else {
            return Ok(Progress::Done);
        };
        if !build.run(nav, filter, budget)? {
            return Ok(Progress::Exhausted);
        }
        if let Some(b) = self.build.take() {
            self.served = Some(Served {
                layout: b.layout,
                goal: b.goal,
                cost: b.cost,
                next: b.next,
            });
        }
        Ok(Progress::Done)
    }

    /// Field index of `r` in the served values.
    fn served_index(&self, nav: &NavMesh, r: PolyRef) -> Option<(&Served, usize)> {
        if !Arc::ptr_eq(&self.identity, nav.report_identity()) {
            return None;
        }
        let s = self.served.as_ref()?;
        let i = s.layout.local(nav, r)? as usize;
        Some((s, i))
    }

    /// Cost to the goal from where polygon `r` is left (0 on the goal polygon);
    /// `None` outside the region, unreached, invalidated or not yet built.
    pub fn cost(&self, nav: &NavMesh, r: PolyRef) -> Option<f32> {
        let (s, i) = self.served_index(nav, r)?;
        let c = s.cost[i];
        c.is_finite().then_some(c)
    }

    /// The next polygon towards the goal from `r`; `None` on the goal polygon and
    /// wherever [`GoalField::cost`] is `None`.
    pub fn next(&self, nav: &NavMesh, r: PolyRef) -> Option<PolyRef> {
        let (s, i) = self.served_index(nav, r)?;
        let n = s.next[i];
        (n != NONE).then(|| s.layout.poly_ref(n))
    }

    /// Steering for an agent at `pos` on polygon `poly` (see the module's steering
    /// section): fills `out` and returns `true`, or `false` when the field has no
    /// usable value for `poly` (outside the region, unreached, invalidated, not
    /// yet built, or a polygon of the stub is no longer resident). Charges one
    /// unit per stub polygon; when they do not fit nothing is computed and the
    /// result is `Exhausted(false)`.
    ///
    /// Call [`GoalField::refresh`] or [`GoalField::step`] after structural changes
    /// before steering, so that invalidated values are not used.
    ///
    /// # Errors
    /// [`Error::MeshChanged`] when `query` reads another navmesh,
    /// [`Error::InvalidParameter`] for a non-finite position.
    pub fn steer(
        &self,
        query: &NavMeshQuery<'_>,
        poly: PolyRef,
        pos: &[f32; 3],
        out: &mut FieldSteer,
        budget: &mut Budget,
    ) -> Result<Budgeted<bool>, Error> {
        out.clear();
        let nav = query.nav();
        self.check(nav)?;
        if !visfinite(pos) {
            return Err(Error::InvalidParameter("agent position"));
        }
        let Some((s, i)) = self.served_index(nav, poly) else {
            return Ok(Budgeted::Complete(false));
        };
        let mut k = i as u32;
        if k != s.goal && s.next[i] == NONE {
            return Ok(Budgeted::Complete(false));
        }
        let mut stub = [0 as PolyRef; STUB];
        let mut len = 0;
        loop {
            stub[len] = s.layout.poly_ref(k);
            len += 1;
            if k == s.goal || len == STUB {
                break;
            }
            k = s.next[k as usize];
            if k == NONE {
                return Ok(Budgeted::Complete(false));
            }
        }
        if !budget.spend(len as u64) {
            return Ok(Budgeted::Exhausted(false));
        }
        if stub[..len]
            .iter()
            .any(|&r| nav.tile_and_poly_by_ref(r).is_err())
        {
            return Ok(Budgeted::Complete(false));
        }
        let end = if k == s.goal {
            self.goal_pos
        } else {
            let after = s.next[k as usize];
            if after == NONE {
                return Ok(Budgeted::Complete(false));
            }
            match query.portal_points(stub[len - 1], s.layout.poly_ref(after)) {
                Ok((l, r)) => mid(&l, &r),
                Err(_) => return Ok(Budgeted::Complete(false)),
            }
        };
        query.find_straight_path(pos, &end, &stub[..len], &mut out.corners, 3, 0)?;
        let Some(corner) = out.corners.get(1).or_else(|| out.corners.first()) else {
            return Ok(Budgeted::Complete(false));
        };
        out.target = corner.pos;
        out.target_flags = corner.flags;
        out.target_ref = corner.r#ref;
        out.stub = stub;
        out.stub_len = len;
        out.cost = s.cost[i];
        out.reaches_goal = k == s.goal;
        Ok(Budgeted::Complete(true))
    }
}

/// Columns of `region` around `pos`, sorted and unique.
fn region_columns(
    nav: &NavMesh,
    pos: &[f32; 3],
    region: &FieldRegion,
) -> Result<Vec<TileKey>, Error> {
    match region {
        FieldRegion::Tiles(keys) => {
            let mut keys = keys.clone();
            keys.sort_unstable();
            keys.dedup();
            Ok(keys)
        }
        FieldRegion::Radius(r) => {
            let r = *r;
            if !r.is_finite() || r < 0.0 {
                return Err(Error::InvalidParameter("field region radius"));
            }
            let Some([bx0, by0, bx1, by1]) = nav.occupied_tile_bounds() else {
                return Ok(Vec::new());
            };
            let (x0, y0) = nav.calc_tile_loc(&[pos[0] - r, pos[1], pos[2] - r]);
            let (x1, y1) = nav.calc_tile_loc(&[pos[0] + r, pos[1], pos[2] + r]);
            let p = nav.params();
            let mut keys = Vec::new();
            for x in x0.max(bx0)..=x1.min(bx1) {
                let minx = p.orig[0] + x as f32 * p.tile_width;
                let dx = (minx - pos[0]).max(pos[0] - (minx + p.tile_width)).max(0.0);
                for y in y0.max(by0)..=y1.min(by1) {
                    let minz = p.orig[2] + y as f32 * p.tile_height;
                    let dz = (minz - pos[2])
                        .max(pos[2] - (minz + p.tile_height))
                        .max(0.0);
                    if dx * dx + dz * dz <= r * r {
                        keys.push(TileKey { x, y });
                    }
                }
            }
            Ok(keys)
        }
    }
}

/// Steering output of [`GoalField::steer`], reusable across calls (it keeps its
/// corner buffer).
#[derive(Clone, Debug, Default)]
pub struct FieldSteer {
    target: [f32; 3],
    target_flags: u8,
    target_ref: PolyRef,
    stub: [PolyRef; STUB],
    stub_len: usize,
    cost: f32,
    reaches_goal: bool,
    corners: Vec<StraightPathPoint>,
}

impl FieldSteer {
    /// An empty output.
    pub fn new() -> FieldSteer {
        FieldSteer::default()
    }

    fn clear(&mut self) {
        self.target = [0.0; 3];
        self.target_flags = 0;
        self.target_ref = 0;
        self.stub = [0; STUB];
        self.stub_len = 0;
        self.cost = f32::INFINITY;
        self.reaches_goal = false;
        self.corners.clear();
    }

    /// The point to steer towards (the first corner of the funnel).
    pub fn target(&self) -> [f32; 3] {
        self.target
    }

    /// `STRAIGHTPATH_*` flags of the target corner (for example an off-mesh
    /// connection start).
    pub fn target_flags(&self) -> u8 {
        self.target_flags
    }

    /// Polygon entered at the target corner (0 at the end of the stub).
    pub fn target_ref(&self) -> PolyRef {
        self.target_ref
    }

    /// The corridor stub: the agent's polygon and up to three next polygons.
    pub fn stub(&self) -> &[PolyRef] {
        &self.stub[..self.stub_len]
    }

    /// Field cost of the agent's polygon ([`GoalField::cost`]).
    pub fn cost(&self) -> f32 {
        self.cost
    }

    /// Whether the stub ends on the goal polygon (the target may be the goal point).
    pub fn reaches_goal(&self) -> bool {
        self.reaches_goal
    }
}

/// Identifies a field in a [`GoalFieldCache`]; ids are never reused.
#[derive(Clone, Copy, Debug, PartialEq, Eq, PartialOrd, Ord, Hash)]
pub struct FieldId(u64);

#[derive(Clone, Debug, PartialEq, Eq, PartialOrd, Ord)]
struct FieldKey {
    goal: PolyRef,
    pos: [u32; 3],
    filter: usize,
    columns: Vec<TileKey>,
}

#[derive(Clone, Debug)]
struct Entry {
    field: GoalField,
    key: FieldKey,
    filter: usize,
    refs: u32,
    last_use: u64,
}

/// Goal fields shared by `(goal polygon, goal point, filter index, region)`,
/// reference-counted, with a cap in bytes.
///
/// Unreferenced fields stay cached for reuse until room is needed; they are
/// evicted least recently used first (last request or release; ties by id). A
/// request whose estimated peak memory does not fit beside the referenced fields
/// is refused with [`Error::LimitExceeded`]. A rebuild that grows a referenced
/// field past the cap is allowed; [`GoalFieldCache::bytes`] reports it.
#[derive(Clone, Debug)]
pub struct GoalFieldCache {
    max_bytes: usize,
    next_id: u64,
    clock: u64,
    entries: BTreeMap<FieldId, Entry>,
    keys: BTreeMap<FieldKey, FieldId>,
}

impl GoalFieldCache {
    /// An empty cache holding at most `max_bytes` of fields.
    pub fn new(max_bytes: usize) -> GoalFieldCache {
        GoalFieldCache {
            max_bytes,
            next_id: 0,
            clock: 0,
            entries: BTreeMap::new(),
            keys: BTreeMap::new(),
        }
    }

    /// The byte cap.
    pub fn max_bytes(&self) -> usize {
        self.max_bytes
    }

    /// Bytes held by all cached fields ([`GoalField::bytes`]).
    pub fn bytes(&self) -> usize {
        self.entries.values().map(|e| e.field.bytes()).sum()
    }

    /// Number of cached fields, referenced or not.
    pub fn len(&self) -> usize {
        self.entries.len()
    }

    /// Whether no field is cached.
    pub fn is_empty(&self) -> bool {
        self.entries.is_empty()
    }

    /// Referenced fields that are not [`Ready`](FieldStatus::Ready) or
    /// [`GoalLost`](FieldStatus::GoalLost).
    pub fn pending(&self) -> u32 {
        self.entries
            .values()
            .filter(|e| {
                e.refs > 0 && matches!(e.field.status(), FieldStatus::Building | FieldStatus::Stale)
            })
            .count() as u32
    }

    /// References held on `id` (0 for an unknown id).
    pub fn refs(&self, id: FieldId) -> u32 {
        self.entries.get(&id).map_or(0, |e| e.refs)
    }

    /// The field `id`, if cached.
    pub fn get(&self, id: FieldId) -> Option<&GoalField> {
        self.entries.get(&id).map(|e| &e.field)
    }

    fn tick(&mut self) -> u64 {
        self.clock += 1;
        self.clock
    }

    /// A reference to the field for this goal, filter index and region: the
    /// cached one when the key matches (regions compare by their resolved
    /// columns), otherwise a new field, built by [`GoalFieldCache::step`].
    ///
    /// # Errors
    /// As [`GoalField::new`], with [`Error::LimitExceeded`] when the new field's
    /// estimate does not fit beside the referenced fields after evicting every
    /// unreferenced one.
    pub fn request(
        &mut self,
        nav: &NavMesh,
        goal: PolyRef,
        goal_pos: [f32; 3],
        filter: usize,
        region: &FieldRegion,
    ) -> Result<FieldId, Error> {
        let (columns, estimate) = GoalField::plan(nav, goal, &goal_pos, region)?;
        let key = FieldKey {
            goal,
            pos: goal_pos.map(f32::to_bits),
            filter,
            columns,
        };
        if let Some(&id) = self.keys.get(&key) {
            let now = self.tick();
            if let Some(e) = self.entries.get_mut(&id) {
                e.refs = e.refs.checked_add(1).ok_or(Error::SizeOverflow)?;
                e.last_use = now;
                return Ok(id);
            }
        }
        let referenced: usize = self
            .entries
            .values()
            .filter(|e| e.refs > 0)
            .map(|e| e.field.bytes())
            .sum();
        if referenced.saturating_add(estimate) > self.max_bytes {
            return Err(Error::LimitExceeded);
        }
        self.evict_to(self.max_bytes - estimate);
        let field = GoalField::new(nav, goal, goal_pos, region, &Limits::UNLIMITED)?;
        let id = FieldId(self.next_id);
        self.next_id += 1;
        let now = self.tick();
        self.keys.insert(key.clone(), id);
        self.entries.insert(
            id,
            Entry {
                field,
                key,
                filter,
                refs: 1,
                last_use: now,
            },
        );
        Ok(id)
    }

    /// Drops one reference to `id`. The field stays cached until evicted.
    ///
    /// # Errors
    /// [`Error::InvalidParameter`] for an unknown or unreferenced id.
    pub fn release(&mut self, id: FieldId) -> Result<(), Error> {
        let now = self.tick();
        let e = self
            .entries
            .get_mut(&id)
            .ok_or(Error::InvalidParameter("unknown field"))?;
        if e.refs == 0 {
            return Err(Error::InvalidParameter("field is not referenced"));
        }
        e.refs -= 1;
        e.last_use = now;
        self.evict_to(self.max_bytes);
        Ok(())
    }

    /// Evicts unreferenced fields, least recently used first, until the cache
    /// holds at most `target` bytes or none is left to evict.
    fn evict_to(&mut self, target: usize) {
        let mut total = self.bytes();
        while total > target {
            let Some((&id, _)) = self
                .entries
                .iter()
                .filter(|(_, e)| e.refs == 0)
                .min_by_key(|(id, e)| (e.last_use, **id))
            else {
                return;
            };
            if let Some(e) = self.entries.remove(&id) {
                total -= e.field.bytes();
                self.keys.remove(&e.key);
            }
        }
    }

    /// Refreshes and builds the referenced fields in id order, with
    /// `filters[filter index]` each, until `budget` runs out. `Done` when every
    /// referenced field is ready (or has lost its goal). Afterwards evicts
    /// unreferenced fields while the cache is over its cap.
    ///
    /// # Errors
    /// [`Error::InvalidParameter`] when a field's filter index is outside
    /// `filters`; as [`GoalField::step`].
    pub fn step<F: QueryFilter>(
        &mut self,
        nav: &NavMesh,
        filters: &[F],
        budget: &mut Budget,
    ) -> Result<Progress, Error> {
        let mut progress = Progress::Done;
        for e in self.entries.values_mut().filter(|e| e.refs > 0) {
            let filter = filters
                .get(e.filter)
                .ok_or(Error::InvalidParameter("field filter index"))?;
            if e.field.step(nav, filter, budget)? == Progress::Exhausted {
                progress = Progress::Exhausted;
                break;
            }
        }
        self.evict_to(self.max_bytes);
        Ok(progress)
    }
}
