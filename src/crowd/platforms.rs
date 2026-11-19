//! Platform navigation for the single-threaded and parallel crowds.
//!
//! Docks join local crowd simulations. Each agent has a caller-assigned id
//! that stays stable while its local slot and its navigation frame change.

use super::*;
use alloc::collections::BTreeMap;

use crate::detour::LinkId;
use crate::frames::{FrameId, FramePoint, FrameSet};

#[cfg(feature = "crowd-world")]
mod dock_avoidance;
mod simulation;
#[cfg(feature = "crowd-world")]
mod snapshot;

/// Stable, caller-assigned identity across dock crossings.
#[derive(Clone, Copy, Debug, PartialEq, Eq, PartialOrd, Ord, Hash)]
pub struct PlatformAgentId(pub u64);

#[derive(Clone, Copy)]
struct Rider<H> {
    frame: FrameId,
    handle: H,
    goal: Option<FramePoint>,
}

/// A collection of frame-local crowds with stable agent ids and dock routes.
///
/// Use `FramedCrowd<Crowd>` or `FramedCrowd<CrowdWorld>`. Attach a configured,
/// empty crowd for every frame agents may use. All frame simulations run in
/// ascending frame id order. Within a frame the native crowd runs its usual
/// path and local avoidance phases.
pub struct FramedCrowd<C: FrameCrowd> {
    crowds: BTreeMap<FrameId, C>,
    riders: BTreeMap<PlatformAgentId, Rider<C::Handle>>,
    waits: BTreeMap<LinkId, f32>,
}

impl<C: FrameCrowd> Default for FramedCrowd<C> {
    fn default() -> Self {
        Self {
            crowds: BTreeMap::new(),
            riders: BTreeMap::new(),
            waits: BTreeMap::new(),
        }
    }
}

impl<C: FrameCrowd> FramedCrowd<C> {
    /// Registers the empty simulation for a frame. Keep filter definitions
    /// consistent on frames that share agents.
    pub fn attach(&mut self, frames: &FrameSet, id: FrameId, crowd: C) -> Result<(), Error> {
        frames.require_mesh(id)?;
        if self.crowds.contains_key(&id) || !crowd.is_empty() {
            return Err(Error::InvalidParameter(
                "occupied crowd frame or nonempty crowd",
            ));
        }
        if self
            .crowds
            .values()
            .next()
            .is_some_and(|old| old.fixed_dt() != crowd.fixed_dt())
        {
            return Err(Error::InvalidParameter("crowd frame time step mismatch"));
        }
        self.crowds.insert(id, crowd);
        Ok(())
    }

    /// Reads a frame-local simulation, including its native avoidance settings.
    pub fn crowd(&self, frame: FrameId) -> Option<&C> {
        self.crowds.get(&frame)
    }

    /// Adds an agent at a local position. The id must be unused and the agent
    /// must snap onto resident, passable ground in that frame.
    pub fn add_agent(
        &mut self,
        frames: &FrameSet,
        id: PlatformAgentId,
        at: FramePoint,
        params: AgentParams,
    ) -> Result<(), Error> {
        if self.riders.contains_key(&id) {
            return Err(Error::InvalidParameter("duplicate platform agent id"));
        }
        let nav = frames.require_mesh(at.frame)?;
        let crowd = self.require_crowd_mut(at.frame)?;
        let filter = crowd
            .filter(params.query_filter_type)
            .ok_or(Error::InvalidParameter("filter index"))?;
        let query = NavMeshQuery::new(nav, 1)?;
        let (poly, pos, _) = query.find_nearest_poly(&at.position, &crowd.extents(), filter)?;
        if poly == 0 {
            return Err(Error::InvalidRef);
        }
        let handle = crowd.add_landed(poly, pos, params)?;
        self.riders.insert(
            id,
            Rider {
                frame: at.frame,
                handle,
                goal: None,
            },
        );
        Ok(())
    }

    /// Removes the agent from its current simulation.
    pub fn remove_agent(&mut self, id: PlatformAgentId) -> Result<(), Error> {
        let rider = self.require_rider(id)?;
        self.require_crowd_mut(rider.frame)?.remove(rider.handle)?;
        self.riders.remove(&id);
        Ok(())
    }

    /// Reads the agent's current frame, native handle and frame-local state.
    /// The native handle is scoped to that frame and may change at a dock.
    pub fn agent(&self, id: PlatformAgentId) -> Option<(FrameId, C::Handle, &CrowdAgent)> {
        let r = self.riders.get(&id)?;
        Some((r.frame, r.handle, self.crowds.get(&r.frame)?.get(r.handle)?))
    }

    /// Reads the agent's world-space position at the current caller-supplied pose.
    pub fn world_position(&self, frames: &FrameSet, id: PlatformAgentId) -> Option<V3> {
        let (frame, _, agent) = self.agent(id)?;
        frames.to_world(FramePoint::new(frame, agent.npos))
    }

    /// Sets a local or cross-frame goal. A goal with no currently available
    /// route is retained and retried on the next step. Clearing the goal
    /// releases the active target on the next step.
    pub fn set_goal(
        &mut self,
        frames: &FrameSet,
        id: PlatformAgentId,
        goal: Option<FramePoint>,
    ) -> Result<(), Error> {
        if let Some(goal) = goal {
            frames.require_mesh(goal.frame)?;
            finite(goal.position)?;
        }
        self.riders
            .get_mut(&id)
            .ok_or(Error::InvalidParameter("unknown platform agent"))?
            .goal = goal;
        Ok(())
    }

    /// Offers a disabled dock to route planning at a caller-estimated wait
    /// cost. Pass `None` to withdraw that offer. A disabled dock still blocks
    /// handover until the caller enables it at an aligned pose.
    pub fn set_wait_cost(
        &mut self,
        frames: &FrameSet,
        id: LinkId,
        cost: Option<f32>,
    ) -> Result<(), Error> {
        if frames.dock(id).is_none() {
            return Err(Error::InvalidParameter("unknown dock"));
        }
        match cost {
            Some(cost) => {
                nonnegative(cost)?;
                self.waits.insert(id, cost);
            }
            None => {
                self.waits.remove(&id);
            }
        }
        Ok(())
    }

    /// Completes an ordinary external off-mesh traversal in the agent's current
    /// frame. The landing position is local to that mesh.
    pub fn complete_off_mesh(
        &mut self,
        frames: &FrameSet,
        id: PlatformAgentId,
        landing: V3,
    ) -> Result<(), Error> {
        let rider = self.require_rider(id)?;
        self.require_crowd_mut(rider.frame)?.complete(
            frames.require_mesh(rider.frame)?,
            rider.handle,
            landing,
        )
    }

    /// Aborts an ordinary external off-mesh traversal in the current frame.
    pub fn abort_off_mesh(&mut self, id: PlatformAgentId) -> Result<(), Error> {
        let rider = self.require_rider(id)?;
        self.require_crowd_mut(rider.frame)?.abort(rider.handle)
    }

    fn require_rider(&self, id: PlatformAgentId) -> Result<Rider<C::Handle>, Error> {
        self.riders
            .get(&id)
            .copied()
            .ok_or(Error::InvalidParameter("unknown platform agent"))
    }

    fn require_crowd_mut(&mut self, id: FrameId) -> Result<&mut C, Error> {
        self.crowds
            .get_mut(&id)
            .ok_or(Error::InvalidParameter("missing crowd frame"))
    }
}

// Keep the adapter methods private. The public trait exposes the handle type
// and seals implementation to this crate's two crowd engines.
mod sealed {
    use super::*;

    pub trait Engine {
        type LocalHandle: Copy;

        fn is_empty(&self) -> bool;
        fn fixed_dt(&self) -> Option<f32> {
            None
        }
        fn extents(&self) -> V3;
        fn filter(&self, index: usize) -> Option<&DefaultFilter>;
        fn get(&self, handle: Self::LocalHandle) -> Option<&CrowdAgent>;
        fn add_landed(
            &mut self,
            poly: PolyRef,
            pos: V3,
            p: AgentParams,
        ) -> Result<Self::LocalHandle, Error>;
        fn remove(&mut self, handle: Self::LocalHandle) -> Result<(), Error>;
        fn target(
            &mut self,
            nav: &NavMesh,
            handle: Self::LocalHandle,
            target: Option<(PolyRef, V3)>,
        ) -> Result<(), Error>;
        fn complete(
            &mut self,
            nav: &NavMesh,
            handle: Self::LocalHandle,
            landing: V3,
        ) -> Result<(), Error>;
        fn abort(&mut self, handle: Self::LocalHandle) -> Result<(), Error>;
    }
}

/// A native simulation usable with [`FramedCrowd`].
///
/// Implemented by [`Crowd`] and, with the `crowd-world` feature, by
/// `CrowdWorld`.
pub trait FrameCrowd: sealed::Engine<LocalHandle = Self::Handle> {
    /// Agent handle scoped to one native simulation.
    type Handle: Copy;
}

impl FrameCrowd for Crowd {
    type Handle = AgentId;
}

impl sealed::Engine for Crowd {
    type LocalHandle = AgentId;

    fn is_empty(&self) -> bool {
        self.active_agents().next().is_none()
    }
    fn extents(&self) -> V3 {
        self.query_half_extents()
    }
    fn filter(&self, index: usize) -> Option<&DefaultFilter> {
        self.filter(index)
    }
    fn get(&self, h: AgentId) -> Option<&CrowdAgent> {
        self.agent(h)
    }
    fn add_landed(&mut self, r: PolyRef, pos: V3, p: AgentParams) -> Result<AgentId, Error> {
        p.validate(self.max_radius)?;
        let i = self
            .agents
            .iter()
            .position(|a| !a.active)
            .ok_or(Error::BufferTooSmall)?;
        self.agents[i].place(r, pos, p)?;
        Ok(AgentId(i))
    }
    fn remove(&mut self, h: AgentId) -> Result<(), Error> {
        self.remove_agent(h)
    }
    fn target(
        &mut self,
        nav: &NavMesh,
        h: AgentId,
        target: Option<(PolyRef, V3)>,
    ) -> Result<(), Error> {
        match target {
            Some((r, p)) => self.request_move_target(nav, h, r, p),
            None => self.reset_move_target(h),
        }
    }
    fn complete(&mut self, nav: &NavMesh, h: AgentId, p: V3) -> Result<(), Error> {
        self.complete_off_mesh(nav, h, p)
    }
    fn abort(&mut self, h: AgentId) -> Result<(), Error> {
        self.abort_off_mesh(h)
    }
}

#[cfg(feature = "crowd-world")]
use super::world::{AgentHandle, CrowdWorld, Goal};

#[cfg(feature = "crowd-world")]
impl FrameCrowd for CrowdWorld {
    type Handle = AgentHandle;
}

#[cfg(feature = "crowd-world")]
impl sealed::Engine for CrowdWorld {
    type LocalHandle = AgentHandle;

    fn is_empty(&self) -> bool {
        self.is_empty()
    }
    fn fixed_dt(&self) -> Option<f32> {
        Some(self.params().fixed_dt)
    }
    fn extents(&self) -> V3 {
        self.query_half_extents()
    }
    fn filter(&self, index: usize) -> Option<&DefaultFilter> {
        self.filter(index)
    }
    fn get(&self, h: AgentHandle) -> Option<&CrowdAgent> {
        self.agent(h)?.detail()
    }
    fn add_landed(&mut self, r: PolyRef, pos: V3, p: AgentParams) -> Result<AgentHandle, Error> {
        self.add_landed_agent(r, pos, p)
    }
    fn remove(&mut self, h: AgentHandle) -> Result<(), Error> {
        self.remove_agent(h)
    }
    fn target(
        &mut self,
        nav: &NavMesh,
        h: AgentHandle,
        target: Option<(PolyRef, V3)>,
    ) -> Result<(), Error> {
        self.set_goal(
            nav,
            h,
            target.map_or(Goal::None, |(poly, pos)| Goal::Point { poly, pos }),
        )
    }
    fn complete(&mut self, nav: &NavMesh, h: AgentHandle, p: V3) -> Result<(), Error> {
        self.complete_off_mesh(nav, h, p)
    }
    fn abort(&mut self, h: AgentHandle) -> Result<(), Error> {
        self.abort_off_mesh(h)
    }
}
