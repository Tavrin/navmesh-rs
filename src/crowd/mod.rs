//! Agent simulation and local avoidance, translated from DetourCrowd at the
//! pinned upstream revision; see NOTICE. A crowd borrows the navmesh only during
//! each call, so tiles can be replaced between updates. Slot handles are reused
//! after removal, just as in upstream; retain them only while the agent is active.
#![allow(
    clippy::needless_range_loop,
    clippy::collapsible_if,
    clippy::too_many_arguments
)]
use crate::common::sqrtf;
use crate::detour::math::*;
use crate::detour::{
    DefaultFilter, NavMesh, NavMeshQuery, PolyRef, QueryFilter, STRAIGHTPATH_END,
    STRAIGHTPATH_OFFMESH_CONNECTION, Status, StraightPathPoint,
};
use crate::{Error, Workspace};
use alloc::vec::Vec;
mod avoidance;
mod boundary;
// Public helper calls reset usage; internal `_in` calls compose within the
// manager's existing per-call budget.
mod corridor;
mod grid;
mod queue;
pub use avoidance::{ObstacleAvoidanceParams, ObstacleAvoidanceQuery};
pub use boundary::LocalBoundary;
pub use corridor::PathCorridor;
pub use grid::ProximityGrid;
pub use queue::PathQueue;

/// Anticipate corners when steering.
pub const ANTICIPATE_TURNS: u8 = 1;
/// Plan collision-avoiding velocities.
pub const OBSTACLE_AVOIDANCE: u8 = 2;
/// Add neighbour separation to steering.
pub const SEPARATION: u8 = 4;
/// Shortcut visible corridor sections.
pub const OPTIMIZE_VIS: u8 = 8;
/// Periodically optimize corridor topology.
pub const OPTIMIZE_TOPO: u8 = 16;

/// Slot in a crowd's fixed agent pool, reused after removal.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct AgentId(pub usize);
/// Locomotion state, with upstream numeric values.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
#[repr(u8)]
pub enum AgentState {
    /// No valid position.
    Invalid = 0,
    /// Moving on the surface.
    Walking = 1,
    /// Animating across a connection.
    Offmesh = 2,
}
/// Move request state, with upstream numeric values.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
#[repr(u8)]
pub enum TargetState {
    /// No request.
    None = 0,
    /// Request failed.
    Failed = 1,
    /// Corridor loaded.
    Valid = 2,
    /// Quick search pending.
    Requesting = 3,
    /// Waiting for queue capacity.
    WaitingForQueue = 4,
    /// Sliced search pending.
    WaitingForPath = 5,
    /// Direct velocity control.
    Velocity = 6,
}
/// Physical dimensions and steering settings for an agent.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct AgentParams {
    /// Positive radius, no larger than the crowd's maximum.
    pub radius: f32,
    /// Positive height.
    pub height: f32,
    /// Maximum acceleration per second.
    pub max_acceleration: f32,
    /// Maximum desired speed.
    pub max_speed: f32,
    /// Neighbour and wall search range.
    pub collision_query_range: f32,
    /// Visibility optimization range.
    pub path_optimization_range: f32,
    /// Separation steering weight.
    pub separation_weight: f32,
    /// Combination of the update flag constants.
    pub update_flags: u8,
    /// Avoidance parameter slot (0..8).
    pub obstacle_avoidance_type: usize,
    /// Filter slot (0..16).
    pub query_filter_type: usize,
}
impl Default for AgentParams {
    fn default() -> Self {
        Self {
            radius: 0.6,
            height: 2.,
            max_acceleration: 8.,
            max_speed: 3.5,
            collision_query_range: 7.2,
            path_optimization_range: 18.,
            separation_weight: 2.,
            update_flags: 31,
            obstacle_avoidance_type: 3,
            query_filter_type: 0,
        }
    }
}
impl AgentParams {
    fn validate(&self, max_radius: f32) -> Result<(), Error> {
        for v in [
            self.radius,
            self.height,
            self.collision_query_range,
            self.path_optimization_range,
        ] {
            positive(v)?;
        }
        for v in [
            self.max_acceleration,
            self.max_speed,
            self.separation_weight,
        ] {
            nonnegative(v)?;
        }
        if self.radius > max_radius
            || self.obstacle_avoidance_type >= 8
            || self.query_filter_type >= 16
            || self.update_flags & !31 != 0
        {
            return Err(Error::InvalidParameter("agent parameters"));
        }
        Ok(())
    }
}
#[derive(Clone, Copy, Debug, Default)]
struct Animation {
    active: bool,
    init: V3,
    start: V3,
    end: V3,
    t: f32,
    tmax: f32,
}
/// Read-only state of an agent slot.
#[derive(Clone, Debug)]
pub struct CrowdAgent {
    /// Whether this slot is in use.
    pub active: bool,
    /// Locomotion state.
    pub state: AgentState,
    /// Move request state.
    pub target_state: TargetState,
    /// Current position.
    pub npos: V3,
    /// Actual velocity after acceleration limiting.
    pub vel: V3,
    /// Chosen avoidance velocity.
    pub nvel: V3,
    /// Desired steering velocity.
    pub dvel: V3,
    /// Desired speed.
    pub desired_speed: f32,
    /// Current path corridor.
    pub corridor: PathCorridor,
    /// Cached local walls.
    pub boundary: LocalBoundary,
    /// Steering corners.
    pub corners: Vec<StraightPathPoint>,
    /// Physical and steering configuration.
    pub params: AgentParams,
    /// Requested polygon (zero in velocity mode).
    pub target_ref: PolyRef,
    /// Requested position or velocity.
    pub target_pos: V3,
    /// Whether the full search returned a partial path.
    pub partial: bool,
    topology_time: f32,
    replan_time: f32,
    replan: bool,
    pathq: u32,
    neis: [(usize, f32); 6],
    nneis: usize,
    disp: V3,
    anim: Animation,
}
/// Fixed-capacity single-threaded crowd manager.
pub struct Crowd {
    agents: Vec<CrowdAgent>,
    max_radius: f32,
    extents: V3,
    grid: ProximityGrid,
    avoidance: ObstacleAvoidanceQuery,
    avoidance_params: [ObstacleAvoidanceParams; 8],
    filters: [DefaultFilter; 16],
    pathq: PathQueue,
    active: Vec<usize>,
    velocity_samples: usize,
}
impl Crowd {
    /// Allocates agents and all persistent crowd components under workspace limits.
    pub fn new(
        ws: &mut Workspace,
        nav: &NavMesh,
        max_agents: usize,
        max_radius: f32,
    ) -> Result<Self, Error> {
        ws.begin();
        positive(max_radius)?;
        if max_agents == 0 || max_agents > 16383 {
            return Err(Error::InvalidParameter("max_agents must be in 1..=16383"));
        }
        let grid = ProximityGrid::new_in(ws, max_agents * 4, max_radius * 3.)?;
        let avoidance = ObstacleAvoidanceQuery::new_in(ws, 6, 8)?;
        let pathq = PathQueue::new_in(ws, nav, 256, 4096)?;
        let mut agents = ws.vec_with_capacity(max_agents)?;
        for _ in 0..max_agents {
            agents.push(CrowdAgent {
                active: false,
                state: AgentState::Invalid,
                target_state: TargetState::None,
                npos: [0.; 3],
                vel: [0.; 3],
                nvel: [0.; 3],
                dvel: [0.; 3],
                desired_speed: 0.,
                corridor: PathCorridor::new_in(ws, 256)?,
                boundary: LocalBoundary::new_in(ws)?,
                corners: ws.vec_with_capacity(4)?,
                params: AgentParams::default(),
                target_ref: 0,
                target_pos: [0.; 3],
                partial: false,
                topology_time: 0.,
                replan_time: 0.,
                replan: false,
                pathq: 0,
                neis: [(0, 0.); 6],
                nneis: 0,
                disp: [0.; 3],
                anim: Animation::default(),
            });
        }
        Ok(Self {
            agents,
            max_radius,
            extents: [max_radius * 2., max_radius * 1.5, max_radius * 2.],
            grid,
            avoidance,
            pathq,
            avoidance_params: [ObstacleAvoidanceParams::default(); 8],
            filters: core::array::from_fn(|_| DefaultFilter::default()),
            active: ws.vec_with_capacity(max_agents)?,
            velocity_samples: 0,
        })
    }
    /// Maximum number of agent slots.
    pub fn max_agents(&self) -> usize {
        self.agents.len()
    }
    /// All slots, including inactive ones.
    pub fn agents(&self) -> &[CrowdAgent] {
        &self.agents
    }
    /// Reads an active slot.
    pub fn agent(&self, id: AgentId) -> Option<&CrowdAgent> {
        self.agents.get(id.0).filter(|a| a.active)
    }
    fn agent_mut(&mut self, id: AgentId) -> Result<&mut CrowdAgent, Error> {
        self.agents
            .get_mut(id.0)
            .filter(|a| a.active)
            .ok_or(Error::InvalidParameter("inactive agent handle"))
    }
    /// Number of sampled velocities in the last update.
    pub fn velocity_sample_count(&self) -> usize {
        self.velocity_samples
    }
    /// Sets one of eight avoidance presets.
    pub fn set_obstacle_avoidance_params(
        &mut self,
        index: usize,
        p: ObstacleAvoidanceParams,
    ) -> Result<(), Error> {
        p.validate()?;
        *self
            .avoidance_params
            .get_mut(index)
            .ok_or(Error::InvalidParameter("avoidance index"))? = p;
        Ok(())
    }
    /// Reads an avoidance preset.
    pub fn obstacle_avoidance_params(&self, index: usize) -> Option<&ObstacleAvoidanceParams> {
        self.avoidance_params.get(index)
    }
    /// Sets one of sixteen query filters. Pending searches read its current value.
    pub fn set_filter(&mut self, index: usize, f: DefaultFilter) -> Result<(), Error> {
        if f.area_cost.iter().any(|v| !v.is_finite() || *v < 0.) {
            return Err(Error::InvalidParameter("filter costs"));
        }
        *self
            .filters
            .get_mut(index)
            .ok_or(Error::InvalidParameter("filter index"))? = f;
        Ok(())
    }
    /// Reads a query filter.
    pub fn filter(&self, index: usize) -> Option<&DefaultFilter> {
        self.filters.get(index)
    }
    /// Places an agent on the nearest polygon, or marks it invalid if none is near.
    pub fn add_agent(
        &mut self,
        ws: &mut Workspace,
        nav: &NavMesh,
        pos: V3,
        p: AgentParams,
    ) -> Result<AgentId, Error> {
        ws.begin();
        finite(pos)?;
        p.validate(self.max_radius)?;
        ws.work(1)?;
        ws.charge_bytes::<u8>(512 * 128 + 16384)?;
        let idx = self
            .agents
            .iter()
            .position(|a| !a.active)
            .ok_or(Error::BufferTooSmall)?;
        let q = NavMeshQuery::new(nav, 512)?;
        let (r, nearest, _) = q
            .find_nearest_poly(&pos, &self.extents, &self.filters[p.query_filter_type])
            .unwrap_or((0, pos, false));
        let nearest = if r == 0 { pos } else { nearest };
        let a = &mut self.agents[idx];
        a.params = p;
        a.corridor.reset(r, nearest)?;
        a.boundary.reset();
        a.partial = false;
        a.topology_time = 0.;
        a.replan_time = 0.;
        a.nneis = 0;
        a.dvel = [0.; 3];
        a.nvel = [0.; 3];
        a.vel = [0.; 3];
        a.npos = nearest;
        a.desired_speed = 0.;
        a.state = if r != 0 {
            AgentState::Walking
        } else {
            AgentState::Invalid
        };
        a.target_state = TargetState::None;
        a.active = true;
        Ok(AgentId(idx))
    }
    /// Releases a slot for reuse.
    pub fn remove_agent(&mut self, id: AgentId) -> Result<(), Error> {
        self.agent_mut(id)?.active = false;
        Ok(())
    }
    /// Updates physical and steering settings for an active slot.
    pub fn update_agent_parameters(&mut self, id: AgentId, p: AgentParams) -> Result<(), Error> {
        p.validate(self.max_radius)?;
        self.agent_mut(id)?.params = p;
        Ok(())
    }
    /// Requests a new target polygon and position; processed on the next update.
    pub fn request_move_target(&mut self, id: AgentId, r: PolyRef, pos: V3) -> Result<(), Error> {
        finite(pos)?;
        if r == 0 {
            return Err(Error::InvalidRef);
        }
        let a = self.agent_mut(id)?;
        a.target_ref = r;
        a.target_pos = pos;
        a.pathq = 0;
        a.replan = false;
        a.target_state = TargetState::Requesting;
        Ok(())
    }
    /// Switches to direct velocity control.
    pub fn request_move_velocity(&mut self, id: AgentId, vel: V3) -> Result<(), Error> {
        finite(vel)?;
        let a = self.agent_mut(id)?;
        a.target_ref = 0;
        a.target_pos = vel;
        a.pathq = 0;
        a.replan = false;
        a.target_state = TargetState::Velocity;
        Ok(())
    }
    /// Clears a movement request and its desired velocity.
    pub fn reset_move_target(&mut self, id: AgentId) -> Result<(), Error> {
        let a = self.agent_mut(id)?;
        a.target_ref = 0;
        a.target_pos = [0.; 3];
        a.dvel = [0.; 3];
        a.pathq = 0;
        a.replan = false;
        a.target_state = TargetState::None;
        Ok(())
    }
}
fn finite(v: V3) -> Result<(), Error> {
    if v.iter().all(|v| v.is_finite()) {
        Ok(())
    } else {
        Err(Error::InvalidInput("non-finite crowd vector"))
    }
}
fn positive(v: f32) -> Result<(), Error> {
    if v.is_finite() && v > 0. {
        Ok(())
    } else {
        Err(Error::InvalidParameter("positive crowd scalar"))
    }
}
fn nonnegative(v: f32) -> Result<(), Error> {
    if v.is_finite() && v >= 0. {
        Ok(())
    } else {
        Err(Error::InvalidParameter("nonnegative crowd scalar"))
    }
}
fn vdist_2d_sqr(a: &[f32], b: &[f32]) -> f32 {
    sqr(a[0] - b[0]) + sqr(a[2] - b[2])
}
fn vdist_2d(a: &[f32], b: &[f32]) -> f32 {
    sqrtf(vdist_2d_sqr(a, b))
}
fn vlen(a: &[f32]) -> f32 {
    sqrtf(vlen_sqr(a))
}
// Upstream appends equal-to-last entries, but inserts other ties before peers.
fn insert_pos<T>(items: &[T], value: f32, key: impl Fn(&T) -> f32) -> usize {
    if items.last().is_none_or(|v| value >= key(v)) {
        items.len()
    } else {
        items
            .iter()
            .position(|v| value <= key(v))
            .unwrap_or(items.len())
    }
}
mod update;
