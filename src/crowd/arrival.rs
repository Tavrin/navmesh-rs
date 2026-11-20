//! Optional arrival behavior and caller-owned shared-goal reservations.
use super::*;

/// Behavior when a point target is reached. Velocity requests do not arrive.
#[non_exhaustive]
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub enum ArrivalPolicy {
    /// Keep steering and participating exactly as before (the default).
    #[default]
    Keep,
    /// Stop steering, remain in avoidance and yield to contact.
    /// With goal slots, the reservation is the parked destination.
    Park,
    /// Stop and release the agent on the update that reaches its target.
    Remove,
}

/// Deterministic reservations around a shared point goal, for either crowd manager.
/// Slots follow a square spiral on the horizontal plane, checked against the mesh
/// and every occupied reservation after projection onto the surface.
/// Keep one allocator per destination. Release a reservation when its agent is
/// removed or retargeted; parked agents retain theirs. Reservations are caller-owned
/// and are not part of crowd-world snapshots.
///
/// ```
/// use navmesh_rs::crowd::{AgentId, Crowd, GoalSlots};
/// use navmesh_rs::detour::{DefaultFilter, NavMesh};
/// use navmesh_rs::Error;
/// fn assign(crowd: &mut Crowd, nav: &NavMesh, agent: AgentId,
///           slots: &mut GoalSlots) -> Result<usize, Error> {
///     let (reservation, poly, pos) = slots.allocate(nav, &DefaultFilter::default(), 1024)?;
///     if let Err(error) = crowd.request_move_target(nav, agent, poly, pos) {
///         slots.release(reservation)?;
///         return Err(error);
///     }
///     Ok(reservation)
/// }
/// ```
#[non_exhaustive]
pub struct GoalSlots {
    goal: V3,
    spacing: f32,
    slots: Vec<Option<(PolyRef, V3)>>,
}
impl GoalSlots {
    /// Creates reservations with spacing in metres (at least 1.6).
    /// Use spacing greater than four times the agent radius to allow passage
    /// between parked neighbours. The default spacing is 3 m, leaving room
    /// for a 0.5 m arrival distance on both sides with 0.4 m radius agents.
    pub fn new(goal: V3, spacing: f32) -> Result<Self, Error> {
        finite(goal)?;
        if !spacing.is_finite() || spacing < 1.6 {
            return Err(Error::InvalidParameter("goal slot spacing"));
        }
        Ok(Self {
            goal,
            spacing,
            slots: Vec::new(),
        })
    }
    /// Creates reservations with the default 3 m spacing.
    pub fn around(goal: V3) -> Result<Self, Error> {
        Self::new(goal, 3.)
    }
    /// Allocates a free mesh-valid slot, returning its reservation index, polygon
    /// and position. The search is bounded by the smaller of one million and
    /// max_candidates; exhaustion leaves reservations unchanged.
    /// Use the agent's filter and request the returned target through
    /// Crowd::request_move_target or the world's point goal API.
    pub fn allocate(
        &mut self,
        nav: &NavMesh,
        filter: &dyn QueryFilter,
        max_candidates: usize,
    ) -> Result<(usize, PolyRef, V3), Error> {
        let q = NavMeshQuery::new(nav, 512)?;
        let (mut x, mut z, mut dx, mut dz) = (0i32, 0i32, 1i32, 0i32);
        let (mut length, mut left, mut turns) = (1usize, 1usize, 0usize);
        for _ in 0..max_candidates.min(1_000_000) {
            let p = [
                self.goal[0] + x as f32 * self.spacing,
                self.goal[1],
                self.goal[2] + z as f32 * self.spacing,
            ];
            let ext = [self.spacing * 0.25, 1., self.spacing * 0.25];
            if let Ok((r, pos, _)) = q.find_nearest_poly(&p, &ext, filter) {
                if r != 0
                    && vdist_2d(&p, &pos) <= self.spacing * 0.25
                    && self
                        .slots
                        .iter()
                        .flatten()
                        .all(|&(_, other)| vdist_2d(&pos, &other) >= self.spacing)
                {
                    let index = self
                        .slots
                        .iter()
                        .position(Option::is_none)
                        .unwrap_or(self.slots.len());
                    if index == self.slots.len() {
                        self.slots
                            .try_reserve(1)
                            .map_err(|_| Error::AllocationFailed)?;
                        self.slots.push(Some((r, pos)));
                    } else {
                        self.slots[index] = Some((r, pos));
                    }
                    return Ok((index, r, pos));
                }
            }
            x += dx;
            z += dz;
            left -= 1;
            if left == 0 {
                (dx, dz) = (-dz, dx);
                turns += 1;
                if turns.is_multiple_of(2) {
                    length += 1;
                }
                left = length;
            }
        }
        Err(Error::BufferTooSmall)
    }
    /// Releases an occupied reservation. Invalid or released indices are rejected.
    pub fn release(&mut self, index: usize) -> Result<(), Error> {
        let s = self
            .slots
            .get_mut(index)
            .filter(|s| s.is_some())
            .ok_or(Error::InvalidParameter("goal slot index"))?;
        *s = None;
        Ok(())
    }
}

/// Returns true when the owner should remove this agent.
pub(super) fn finish_arrival(a: &mut CrowdAgent) -> bool {
    if a.params.arrival_policy == ArrivalPolicy::Keep
        || a.arrived
        || a.state != AgentState::Walking
        || a.target_state != TargetState::Valid
        || a.partial
        || a.corridor.last_poly() != a.target_ref
        || vdist_2d(&a.npos, &a.target_pos) > a.params.arrival_distance
        || (a.npos[1] - a.target_pos[1]).abs() > a.params.height
    {
        return false;
    }
    let remove = a.params.arrival_policy == ArrivalPolicy::Remove;
    a.reset_target();
    a.arrived = true;
    a.vel = [0.; 3];
    a.nvel = [0.; 3];
    a.dvel = [0.; 3];
    a.desired_speed = 0.;
    remove
}
