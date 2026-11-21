//! Caller-controlled traversal for authored and runtime off-mesh connections.

use super::*;
use crate::detour::{LinkId, POLYTYPE_GROUND};

/// How an agent traverses off-mesh connections.
#[non_exhaustive]
#[derive(Clone, Copy, Debug, Default, Eq, PartialEq)]
pub enum OffMeshMode {
    /// Use the crowd's built-in interpolation.
    #[default]
    Animate,
    /// Stop at the departure point until the caller completes or aborts traversal.
    External,
}

/// Information handed to the caller while an agent waits to traverse a link.
#[derive(Clone, Copy, Debug, PartialEq)]
#[non_exhaustive]
pub struct OffMeshReached {
    /// The off-mesh polygon that the agent reached.
    pub poly: PolyRef,
    /// Detour user id from the connection.
    pub user_id: u32,
    /// Stable runtime id when this is a runtime connection.
    pub link_id: Option<LinkId>,
    /// Explicit traversal kind for supported bake-extracted connections.
    pub kind: Option<crate::detour::OffMeshKind>,
    /// Snapped departure point in the direction of travel.
    pub start: [f32; 3],
    /// Snapped arrival point in the direction of travel.
    pub end: [f32; 3],
    /// Ground polygon on the departure side.
    pub departure: PolyRef,
    /// Ground polygon on the arrival side.
    pub landing: PolyRef,
}

#[derive(Clone, Copy, Debug)]
pub(super) struct PendingTraversal {
    pub(super) reached: OffMeshReached,
    pub(super) radius: f32,
}

impl CrowdAgent {
    /// The external traversal awaiting a response, if any.
    pub fn off_mesh_reached(&self) -> Option<OffMeshReached> {
        self.pending_off_mesh.map(|p| p.reached)
    }

    pub(super) fn begin_external_off_mesh(&mut self, nav: &NavMesh, poly: PolyRef) {
        let Some(&[departure, _, landing]) = self.corridor.path().windows(3).find(|w| w[1] == poly)
        else {
            return;
        };
        if self.corridor.first_poly() != departure {
            return;
        }
        let Some(con) = nav.off_mesh_connection_by_ref(poly) else {
            return;
        };
        if con.rad < 0.0 {
            return;
        }
        let Ok((start, end)) = nav.off_mesh_connection_poly_end_points(departure, poly) else {
            return;
        };
        self.pending_off_mesh = Some(PendingTraversal {
            reached: OffMeshReached {
                poly,
                user_id: con.user_id,
                link_id: nav.runtime_link_id(poly),
                kind: con.kind(),
                start,
                end,
                departure,
                landing,
            },
            radius: con.rad,
        });
        self.npos = start;
        self.state = AgentState::Offmesh;
        self.vel = [0.; 3];
        self.nvel = [0.; 3];
        self.dvel = [0.; 3];
        self.desired_speed = 0.;
        self.corners.clear();
        self.nneis = 0;
    }

    pub(super) fn complete_external(
        &mut self,
        nav: &NavMesh,
        filter: &DefaultFilter,
        landing: V3,
    ) -> Result<(), Error> {
        let pending = self
            .pending_off_mesh
            .ok_or(Error::InvalidParameter("no external traversal"))?;
        finite(landing)?;
        let r = pending.reached.landing;
        let (tile, poly) = nav.tile_and_poly_by_ref(r)?;
        if poly.poly_type() != POLYTYPE_GROUND || !filter.pass_filter(r, tile, poly) {
            return Err(Error::InvalidRef);
        }
        let end = pending.reached.end;
        if !within_radius(landing, end, pending.radius) {
            return Err(Error::InvalidParameter(
                "outside external traversal landing radius",
            ));
        }
        let (snap, _) = NavMeshQuery::new(nav, 1)?.closest_point_on_poly(r, &landing)?;
        if !within_radius(snap, end, pending.radius) {
            return Err(Error::InvalidParameter(
                "outside external traversal landing polygon",
            ));
        }
        self.resume_external(r, snap)
    }

    pub(super) fn abort_external(&mut self) -> Result<(), Error> {
        let p = self
            .pending_off_mesh
            .ok_or(Error::InvalidParameter("no external traversal"))?;
        self.resume_external(p.reached.departure, p.reached.start)
    }

    fn resume_external(&mut self, r: PolyRef, pos: V3) -> Result<(), Error> {
        self.corridor.reset(r, pos)?;
        self.npos = pos;
        self.boundary.reset();
        self.state = AgentState::Walking;
        self.pending_off_mesh = None;
        self.partial = false;
        self.pathq = 0;
        self.replan = false;
        self.replan_time = 0.;
        if self.target_ref != 0 && self.target_state != TargetState::WaitingForTile {
            self.target_state = TargetState::Requesting;
        }
        Ok(())
    }
}

fn within_radius(a: V3, b: V3, radius: f32) -> bool {
    let d: f64 = a
        .into_iter()
        .zip(b)
        .map(|(a, b)| {
            let d = f64::from(a) - f64::from(b);
            d * d
        })
        .sum();
    d <= f64::from(radius) * f64::from(radius)
}

impl Crowd {
    /// External traversal requests awaiting a response, in agent slot order.
    pub fn off_mesh_arrivals(&self) -> impl Iterator<Item = (AgentId, OffMeshReached)> + '_ {
        self.agents
            .iter()
            .enumerate()
            .filter_map(|(i, a)| a.off_mesh_reached().map(|p| (AgentId(i), p)))
    }

    /// Lands a waiting agent on its captured arrival polygon. Invalid landings
    /// leave the agent waiting and may be retried. Disabling the link blocks new
    /// arrivals and permits a traversal already handed off to finish.
    pub fn complete_off_mesh(
        &mut self,
        nav: &NavMesh,
        id: AgentId,
        landing: V3,
    ) -> Result<(), Error> {
        let filter = self
            .agent(id)
            .ok_or(Error::InvalidParameter("inactive agent handle"))?
            .params
            .query_filter_type;
        let f = self.filters[filter].clone();
        self.agent_mut(id)?.complete_external(nav, &f, landing)
    }

    /// Cancels the hand-off and replans from the departure point on the next tick.
    pub fn abort_off_mesh(&mut self, id: AgentId) -> Result<(), Error> {
        self.agent_mut(id)?.abort_external()
    }
}
