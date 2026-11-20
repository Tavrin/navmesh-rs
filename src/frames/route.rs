use super::*;
use alloc::collections::BinaryHeap;
use core::cmp::Reverse;

use crate::detour::{POLYTYPE_GROUND, QueryFilter, Status};
use crate::{Budget, Budgeted};

/// The two endpoints and the cost of one active dock seam.
#[cfg(feature = "crowd-world")]
pub(crate) type ActiveSeam = (FramePoint, FramePoint, f32);

/// One local corridor in a route, optionally ending at a dock crossing.
#[derive(Clone, Debug, PartialEq)]
#[non_exhaustive]
pub struct FrameLeg {
    /// Frame containing this entire corridor.
    pub frame: FrameId,
    /// Snapped start, in local coordinates.
    pub start: Point,
    /// Snapped destination, in local coordinates.
    pub end: Point,
    /// Ordered corridor refs, scoped to this leg's frame.
    pub path: Vec<PolyRef>,
    /// Dock to cross after walking this leg.
    pub dock: Option<LinkId>,
}

/// A complete route through one or more independently queried navmeshes.
#[derive(Clone, Debug, PartialEq)]
#[non_exhaustive]
pub struct FramePlan {
    /// Legs in travel order.
    pub legs: Vec<FrameLeg>,
    /// Sum of local corridor costs and any caller-supplied wait costs.
    pub cost: f64,
}

#[derive(Clone, Copy)]
struct Edge {
    id: LinkId,
    from: OnMesh,
    to: OnMesh,
    wait: f64,
}

struct Visit {
    open_cost: u64,
    prev: usize,
    leg: FrameLeg,
}

const SEARCH_NODES: usize = 4096;

impl FrameSet {
    /// Finds a complete route using currently enabled, aligned docks.
    ///
    /// Endpoints are local to their specified frames and snap within `half_extents`
    /// in each frame's axes. Dock landings use each dock's three-dimensional radius.
    /// The graph distinguishes arrivals via different docks in the same frame so
    /// that disconnected landings and approach costs influence routing.
    /// An exhausted search yields no partial route. Local searches are limited
    /// to 4096 nodes and report `Error::BufferTooSmall` on overflow.
    pub fn plan(
        &self,
        from: FramePoint,
        to: FramePoint,
        half_extents: Point,
        filter: &DefaultFilter,
        budget: &mut Budget,
    ) -> Result<Budgeted<Option<FramePlan>>, Error> {
        self.plan_with_waits(from, to, half_extents, filter, budget, |_| None)
    }

    /// Like [`Self::plan`], with a caller-supplied estimate for a disabled dock.
    ///
    /// Returning a finite, nonnegative cost admits the dock as a future edge
    /// even if its endpoints are currently apart. A plan is advisory. The
    /// crossing still requires an enabled dock with aligned landings.
    pub fn plan_with_waits(
        &self,
        from: FramePoint,
        to: FramePoint,
        half_extents: Point,
        filter: &DefaultFilter,
        budget: &mut Budget,
        wait_cost: impl Fn(LinkId) -> Option<f32>,
    ) -> Result<Budgeted<Option<FramePlan>>, Error> {
        self.plan_from(
            (from, None),
            to,
            half_extents,
            filter,
            budget,
            |spec, enabled| {
                if enabled {
                    Some(0.)
                } else {
                    wait_cost(spec.id)
                }
            },
        )
    }

    pub(crate) fn plan_from(
        &self,
        from: (FramePoint, Option<PolyRef>),
        to: FramePoint,
        half_extents: Point,
        filter: &DefaultFilter,
        budget: &mut Budget,
        dock_cost: impl Fn(DockSpec, bool) -> Option<f32>,
    ) -> Result<Budgeted<Option<FramePlan>>, Error> {
        FrameRouting::native(self).plan_from(from, to, half_extents, filter, budget, dock_cost)
    }

    #[cfg(feature = "crowd-world")]
    pub(crate) fn active_seams(
        &self,
        budget: &mut Budget,
    ) -> Result<Budgeted<Vec<ActiveSeam>>, Error> {
        FrameRouting::native(self).active_seams(budget)
    }

    /// Resolves an enabled dock's departure point and validates a crossing from
    /// the agent's current position. Returns the destination ref and local point
    /// once the agent is on the dock seam. No crossing is offered across a gap.
    ///
    /// The supplied polygon must be the agent's current ground polygon.
    pub fn dock_landing(
        &self,
        id: LinkId,
        from: FramePoint,
        poly: PolyRef,
        filter: &DefaultFilter,
    ) -> Result<Option<(PolyRef, FramePoint)>, Error> {
        FrameRouting::native(self).dock_landing(id, from, poly, filter)
    }

    pub(crate) fn dock_open(
        &self,
        id: LinkId,
        a: FramePoint,
        b: FramePoint,
    ) -> Result<bool, Error> {
        FrameRouting::native(self).dock_open(id, a, b)
    }

    pub(crate) fn cross_dock(
        &self,
        id: LinkId,
        from: (PolyRef, FramePoint),
        departure: FramePoint,
        dest: (PolyRef, FramePoint),
        filter: &DefaultFilter,
    ) -> Result<Option<(PolyRef, FramePoint)>, Error> {
        FrameRouting::native(self).cross_dock(id, from, departure, dest, filter)
    }
}

pub(crate) struct FrameRouting<'a> {
    frames: &'a FrameSet,
    #[cfg(all(feature = "recast", feature = "crowd-world"))]
    profile: Option<ProfileId>,
}

impl<'a> FrameRouting<'a> {
    fn native(frames: &'a FrameSet) -> Self {
        Self {
            frames,
            #[cfg(all(feature = "recast", feature = "crowd-world"))]
            profile: None,
        }
    }

    #[cfg(all(feature = "recast", feature = "crowd-world"))]
    pub(crate) fn for_profile(frames: &'a FrameSet, profile: ProfileId) -> Self {
        Self {
            frames,
            profile: Some(profile),
        }
    }

    fn mesh(&self, id: FrameId) -> Option<&'a NavMesh> {
        #[cfg(all(feature = "recast", feature = "crowd-world"))]
        if let Some(profile) = self.profile {
            return Some(&self.frames.profile_meshes(id)?.get(profile).ok()?.nav);
        }
        self.frames.mesh(id)
    }

    fn require_mesh(&self, id: FrameId) -> Result<&'a NavMesh, Error> {
        self.mesh(id)
            .ok_or(Error::InvalidParameter("unknown frame"))
    }

    pub(crate) fn plan_from(
        &self,
        from: (FramePoint, Option<PolyRef>),
        to: FramePoint,
        half_extents: Point,
        filter: &DefaultFilter,
        budget: &mut Budget,
        dock_cost: impl Fn(DockSpec, bool) -> Option<f32>,
    ) -> Result<Budgeted<Option<FramePlan>>, Error> {
        if filter.area_cost.iter().any(|v| !v.is_finite() || *v < 0.) {
            return Err(Error::InvalidParameter("filter costs"));
        }
        let (plan, exhausted) = budget.scoped(|budget| {
            let loc = |point, budget: &mut Budget| self.snap(point, half_extents, filter, budget);
            let (point, poly) = from;
            let nav = self.require_mesh(point.frame)?;
            let pinned = poly.filter(|&r| {
                nav.tile_and_poly_by_ref(r).is_ok_and(|(tile, poly)| {
                    poly.poly_type() == POLYTYPE_GROUND && filter.pass_filter(r, tile, poly)
                })
            });
            let start = if let Some(poly) = pinned {
                OnMesh { point, poly }
            } else {
                let Some(start) = loc(point, budget)? else {
                    return Ok(None);
                };
                start
            };
            let Some(goal) = loc(to, budget)? else {
                return Ok(None);
            };
            let mut edges = Vec::new();
            for dock in self.frames.docks.values() {
                if !budget.spend(1) {
                    return Ok(None);
                }
                let spec = dock.spec;
                let Some(wait) = dock_cost(spec, dock.enabled) else {
                    continue;
                };
                if !wait.is_finite() || wait < 0. {
                    return Err(Error::InvalidParameter("dock wait cost"));
                }
                let wait = f64::from(wait);
                if self.mesh(spec.a.frame).is_none() || self.mesh(spec.b.frame).is_none() {
                    continue;
                }
                let Some((a, b)) = self.landings(spec, filter, budget)? else {
                    if budget.is_exhausted() {
                        return Ok(None);
                    }
                    continue;
                };
                if dock.enabled && !self.aligned(a.point, b.point)? {
                    continue;
                }
                edges.push(Edge {
                    id: spec.id,
                    from: a,
                    to: b,
                    wait,
                });
                if spec.bidirectional {
                    edges.push(Edge {
                        id: spec.id,
                        from: b,
                        to: a,
                        wait,
                    });
                }
            }
            edges.sort_unstable_by_key(|e| (e.to.point.frame, e.id));
            self.search(start, goal, &edges, filter, budget)
        });
        let plan = plan?;
        Ok(Budgeted::of(plan, exhausted))
    }

    fn search(
        &self,
        start: OnMesh,
        goal: OnMesh,
        edges: &[Edge],
        filter: &DefaultFilter,
        budget: &mut Budget,
    ) -> Result<Option<FramePlan>, Error> {
        // Dock edges are indexed by their oriented arrivals. Two extra nodes
        // represent the origin and goal. Heap ties use the sorted edge index.
        let goal_index = edges.len();
        let start_index = goal_index + 1;
        let mut visits: Vec<Option<Visit>> = (0..=goal_index).map(|_| None).collect();
        let mut queue = BinaryHeap::from([Reverse((0, start_index))]);
        while let Some(Reverse((cost, index))) = queue.pop() {
            if !budget.spend(1) {
                return Ok(None);
            }
            let from = if index == start_index {
                start
            } else {
                if visits[index].as_ref().is_none_or(|v| v.open_cost != cost) {
                    continue;
                }
                if index == goal_index {
                    let mut legs = Vec::new();
                    let mut next = index;
                    while let Some(visit) = visits[next].take() {
                        legs.push(visit.leg);
                        if visit.prev == start_index {
                            break;
                        }
                        next = visit.prev;
                    }
                    legs.reverse();
                    return Ok(Some(FramePlan {
                        legs,
                        cost: f64::from_bits(cost),
                    }));
                }
                edges[index].to
            };

            let frame = from.point.frame;
            let mut query = NavMeshQuery::new(self.require_mesh(frame)?, SEARCH_NODES)?;
            let candidates = edges
                .iter()
                .copied()
                .map(Some)
                .chain(core::iter::once(None));
            for (next, edge) in candidates.enumerate() {
                if !budget.spend(1) {
                    return Ok(None);
                }
                let (to, dock, wait) =
                    edge.map_or((goal, None, 0.), |e| (e.from, Some(e.id), e.wait));
                if next == index || to.point.frame != frame {
                    continue;
                }
                let Some((path, leg_cost)) = local_path(&mut query, from, to, filter, budget)?
                else {
                    if budget.is_exhausted() {
                        return Ok(None);
                    }
                    continue;
                };
                // For finite nonnegative values, the bits have numeric order.
                let cost = (f64::from_bits(cost) + leg_cost + wait).to_bits();
                if visits[next].as_ref().is_some_and(|v| v.open_cost <= cost) {
                    continue;
                }
                visits[next] = Some(Visit {
                    open_cost: cost,
                    prev: index,
                    leg: FrameLeg {
                        frame,
                        start: from.point.position,
                        end: to.point.position,
                        path,
                        dock,
                    },
                });
                queue.push(Reverse((cost, next)));
            }
        }
        Ok(None)
    }

    fn snap(
        &self,
        p: FramePoint,
        extents: Point,
        filter: &DefaultFilter,
        budget: &mut Budget,
    ) -> Result<Option<OnMesh>, Error> {
        let q = NavMeshQuery::new(self.require_mesh(p.frame)?, 1)?;
        let Budgeted::Complete((poly, position, _)) =
            q.find_nearest_poly_budgeted(&p.position, &extents, filter, budget)?
        else {
            return Ok(None);
        };
        Ok((poly != 0).then_some(OnMesh {
            point: FramePoint::new(p.frame, position),
            poly,
        }))
    }

    fn landings(
        &self,
        spec: DockSpec,
        filter: &DefaultFilter,
        budget: &mut Budget,
    ) -> Result<Option<(OnMesh, OnMesh)>, Error> {
        let mut resolved = [None; 2];
        for (hit, p) in resolved.iter_mut().zip([spec.a, spec.b]) {
            let query = NavMeshQuery::new(self.require_mesh(p.frame)?, 1)?;
            let Budgeted::Complete(Some((poly, position))) =
                query.find_link_landing(p.position, spec.radius, filter, budget)?
            else {
                return Ok(None);
            };
            *hit = Some(OnMesh {
                point: FramePoint::new(p.frame, position),
                poly,
            });
        }
        Ok(resolved[0].zip(resolved[1]))
    }

    #[cfg(feature = "crowd-world")]
    pub(crate) fn active_seams(
        &self,
        budget: &mut Budget,
    ) -> Result<Budgeted<Vec<ActiveSeam>>, Error> {
        let mut seams = Vec::new();
        let filter = DefaultFilter::default();
        for dock in self.frames.docks.values().filter(|d| d.enabled) {
            if !budget.spend(1) {
                return Ok(Budgeted::Exhausted(Vec::new()));
            }
            if self.mesh(dock.spec.a.frame).is_none() || self.mesh(dock.spec.b.frame).is_none() {
                continue;
            }
            let ends = self.landings(dock.spec, &filter, budget)?;
            if budget.is_exhausted() {
                return Ok(Budgeted::Exhausted(Vec::new()));
            }
            if let Some((a, b)) = ends
                && self.aligned(a.point, b.point)?
            {
                seams.push((a.point, b.point, dock.spec.radius));
            }
        }
        Ok(Budgeted::Complete(seams))
    }

    fn aligned(&self, a: FramePoint, b: FramePoint) -> Result<bool, Error> {
        Ok(
            distance_squared(self.frames.world_wide(a)?, self.frames.world_wide(b)?)
                <= SEAM_EPSILON * SEAM_EPSILON,
        )
    }

    /// Resolves an enabled dock's departure point and validates a crossing from
    /// the agent's current position. Returns the destination ref and local point
    /// once the agent is on the dock seam. No crossing is offered across a gap.
    ///
    /// The supplied polygon must be the agent's current ground polygon.
    fn dock_landing(
        &self,
        id: LinkId,
        from: FramePoint,
        poly: PolyRef,
        filter: &DefaultFilter,
    ) -> Result<Option<(PolyRef, FramePoint)>, Error> {
        validate_point(from.position)?;
        let Some(dock) = self.frames.docks.get(&id).filter(|d| d.enabled) else {
            return Ok(None);
        };
        let Some((mut a, mut b)) = self.landings(dock.spec, filter, &mut Budget::unlimited())?
        else {
            return Ok(None);
        };
        if dock.spec.bidirectional && from.frame == b.point.frame {
            core::mem::swap(&mut a, &mut b);
        }
        self.cross_dock(id, (poly, from), a.point, (b.poly, b.point), filter)
    }

    pub(crate) fn dock_open(
        &self,
        id: LinkId,
        a: FramePoint,
        b: FramePoint,
    ) -> Result<bool, Error> {
        let Some(dock) = self.frames.docks.get(&id).filter(|d| d.enabled) else {
            return Ok(false);
        };
        let forward = (dock.spec.a.frame, dock.spec.b.frame);
        let used = (a.frame, b.frame);
        Ok(
            (used == forward || (dock.spec.bidirectional && used == (forward.1, forward.0)))
                && self.aligned(a, b)?,
        )
    }

    pub(crate) fn cross_dock(
        &self,
        id: LinkId,
        (poly, from): (PolyRef, FramePoint),
        departure: FramePoint,
        (dest_poly, dest): (PolyRef, FramePoint),
        filter: &DefaultFilter,
    ) -> Result<Option<(PolyRef, FramePoint)>, Error> {
        if departure.frame != from.frame
            || !self.aligned(from, departure)?
            || !self.dock_open(id, departure, dest)?
        {
            return Ok(None);
        }
        let nav = self.require_mesh(from.frame)?;
        let (tile, p) = nav.tile_and_poly_by_ref(poly)?;
        if p.poly_type() != POLYTYPE_GROUND || !filter.pass_filter(poly, tile, p) {
            return Ok(None);
        }
        let query = NavMeshQuery::new(nav, 1)?;
        let source = FramePoint::new(
            from.frame,
            query.closest_point_on_poly(poly, &from.position)?.0,
        );
        if !self.aligned(from, source)? {
            return Ok(None);
        }

        let dest_nav = self.require_mesh(dest.frame)?;
        let (tile, p) = dest_nav.tile_and_poly_by_ref(dest_poly)?;
        if p.poly_type() != POLYTYPE_GROUND || !filter.pass_filter(dest_poly, tile, p) {
            return Ok(None);
        }
        let pose = self
            .frames
            .pose(dest.frame)
            .ok_or(Error::InvalidParameter("unknown frame"))?;
        let local = finite_result(pose.local(self.frames.world_wide(from)?))
            .ok_or(Error::InvalidParameter("dock landing"))?;
        let q = NavMeshQuery::new(dest_nav, 1)?;
        let landing = FramePoint::new(dest.frame, q.closest_point_on_poly(dest_poly, &local)?.0);
        if !self.aligned(from, landing)? || !self.aligned(dest, landing)? {
            return Ok(None);
        }
        Ok(Some((dest_poly, landing)))
    }
}

fn local_path(
    query: &mut NavMeshQuery<'_>,
    from: OnMesh,
    to: OnMesh,
    filter: &DefaultFilter,
    budget: &mut Budget,
) -> Result<Option<(Vec<PolyRef>, f64)>, Error> {
    let mut path = Vec::new();
    let Budgeted::Complete(status) = query.find_path_budgeted(
        from.poly,
        to.poly,
        &from.point.position,
        &to.point.position,
        filter,
        &mut path,
        SEARCH_NODES,
        budget,
    )?
    else {
        return Ok(None);
    };
    if status.has_detail(Status::OUT_OF_NODES | Status::BUFFER_TOO_SMALL) {
        return Err(Error::BufferTooSmall);
    }
    if path.last() != Some(&to.poly) {
        return Ok(None);
    }
    let mut pos = from.point.position;
    let mut cost = 0.;
    for (i, &poly) in path.iter().enumerate() {
        if !budget.spend(1) {
            return Ok(None);
        }
        let next = if let Some(&next) = path.get(i + 1) {
            let (a, b) = query.portal_points(poly, next)?;
            core::array::from_fn(|axis| (0.5 * a[axis]) + (0.5 * b[axis]))
        } else {
            to.point.position
        };
        let (tile, p) = query.nav().tile_and_poly_by_ref(poly)?;
        cost += f64::from(filter.cost(&pos, &next, None, (poly, tile, p), None));
        pos = next;
    }
    if !cost.is_finite() {
        return Err(Error::InvalidParameter("frame path cost"));
    }
    Ok(Some((path, cost)))
}
