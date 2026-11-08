//! A crowd store for many agents with generational handles, a world-wide
//! sorted-cell neighbour grid and data-parallel simulation phases (not in
//! recastnavigation; feature `crowd-world`).
//!
//! [`CrowdWorld`] runs the same per-agent phase arithmetic as [`Crowd`] in the same
//! phase order. Every agent simulated in full gets the result [`Crowd`] gives with
//! [`UpstreamFixes::NEAREST_NEIGHBOURS`] for the same inputs, bit for bit, as long as
//! agents are added in the same order (both take the lowest free index).
//!
//! Hot per-agent fields (position, velocities, dimensions, generation) live in
//! arrays indexed by agent index; corridor, walls, corners and neighbours live in a
//! side pool of [`CrowdAgent`] entries sized by
//! [`CrowdWorldParams::max_heavy_agents`]. Phases that read other agents read the
//! arrays, which a phase never writes; each agent writes only its own pool entry,
//! and the arrays are refreshed between phases. With the `parallel` feature these
//! phases run on the current rayon pool, and the result does not depend on the
//! number of threads. Path checks, path requests, the path queue and the one
//! topology optimisation per step stay serial, in agent-index order.
//!
//! Each [`CrowdWorld::step`] charges its work to a [`Budget`] before simulating, by
//! active agent rather than by capacity: [`STEP_WORK_PER_AGENT`] units per simulated
//! agent plus the path queue's node expansions. A step is never partly applied
//! because of the budget.
use super::*;
use alloc::collections::BinaryHeap;
use core::cmp::Reverse;

mod grid;
mod snapshot;
mod step;

use grid::CellGrid;

/// Work units a step charges per simulated agent, the same per-agent estimate
/// [`Crowd::update`] charges per slot.
pub const STEP_WORK_PER_AGENT: u64 = 4096;
/// Largest number of agents a world can hold.
pub const MAX_WORLD_AGENTS: u32 = 1 << 24;
const NONE: u32 = u32::MAX;

/// A stable reference to an agent: its index and the generation of that index.
/// A handle stops resolving once its agent is removed, even after the index is
/// reused.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash, PartialOrd, Ord)]
pub struct AgentHandle {
    index: u32,
    generation: u32,
}
impl AgentHandle {
    /// The agent index: the order of neighbour ties and of the serial phases.
    pub fn index(self) -> u32 {
        self.index
    }
    /// How many agents occupied this index before this one.
    pub fn generation(self) -> u32 {
        self.generation
    }
}

/// How much of the simulation an agent receives.
#[non_exhaustive]
#[derive(Clone, Copy, Debug, PartialEq, Eq, PartialOrd, Ord, Hash)]
pub enum Tier {
    /// Every phase: corridor, walls, neighbours, steering, avoidance, collision
    /// correction and off-mesh animation.
    Full,
}

/// What an agent moves towards.
#[non_exhaustive]
#[derive(Clone, Copy, Debug, PartialEq)]
pub enum Goal {
    /// Stand still: no target and no desired velocity.
    None,
    /// A position on polygon `poly`, reached through the path queue
    /// ([`Crowd::request_move_target`] semantics, including waiting for an unloaded
    /// tile).
    Point {
        /// Target polygon.
        poly: PolyRef,
        /// Target position.
        pos: V3,
    },
    /// A desired velocity, steered directly ([`Crowd::request_move_velocity`]).
    Velocity(V3),
}

/// Path queue sizes. The defaults are upstream's (4,096 search nodes, 100 node
/// expansions per step); the queue always has eight request slots.
#[non_exhaustive]
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct PathQueueParams {
    /// Node pool of the sliced search (1..=65,535).
    pub max_nodes: usize,
    /// Node expansions per step (at least 1).
    pub iterations: usize,
}
impl Default for PathQueueParams {
    fn default() -> Self {
        Self {
            max_nodes: 4096,
            iterations: 100,
        }
    }
}

/// Configuration of a [`CrowdWorld`]. Construct with [`Self::default`] and assign
/// the fields.
#[non_exhaustive]
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct CrowdWorldParams {
    /// Most agents the world holds (1..=[`MAX_WORLD_AGENTS`]).
    pub max_agents: u32,
    /// Size of the side pool holding corridors, walls, corners and neighbours
    /// (1..=`max_agents`). Every agent simulated in full uses one entry.
    pub max_heavy_agents: u32,
    /// Largest agent radius; also sets the grid cell size (three times it).
    pub max_radius: f32,
    /// Path queue sizes.
    pub path_queue: PathQueueParams,
    /// Positive fixed time step of [`CrowdWorld::step`], in seconds.
    pub fixed_dt: f32,
}
impl Default for CrowdWorldParams {
    fn default() -> Self {
        Self {
            max_agents: 1024,
            max_heavy_agents: 1024,
            max_radius: 0.6,
            path_queue: PathQueueParams::default(),
            fixed_dt: 1. / 30.,
        }
    }
}
impl CrowdWorldParams {
    fn validate(&self) -> Result<(), Error> {
        positive(self.max_radius)?;
        positive(self.fixed_dt)?;
        if self.max_agents == 0 || self.max_agents > MAX_WORLD_AGENTS {
            return Err(Error::InvalidParameter("max_agents"));
        }
        if self.max_heavy_agents == 0 || self.max_heavy_agents > self.max_agents {
            return Err(Error::InvalidParameter("max_heavy_agents"));
        }
        if !(1..=65535).contains(&self.path_queue.max_nodes)
            || self.path_queue.iterations == 0
            || self.path_queue.iterations > u32::MAX as usize
        {
            return Err(Error::InvalidParameter("path queue"));
        }
        Ok(())
    }
}

/// What one [`CrowdWorld::step`] did.
#[non_exhaustive]
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub struct StepReport {
    /// Agents simulated.
    pub agents: u32,
    /// Work units charged to the budget (or needed, when the step was refused).
    pub work: u64,
    /// Work units the phases counted while running; independent of the thread
    /// count. Useful to calibrate budgets.
    pub measured_work: u64,
    /// Path requests holding a queue slot after the step.
    pub path_queue_depth: u32,
    /// Velocities sampled by sampled-mode avoidance.
    pub velocity_samples: u64,
}

/// Read access to one agent.
#[derive(Clone, Copy, Debug)]
pub struct AgentView<'a> {
    handle: AgentHandle,
    tier: Tier,
    agent: &'a CrowdAgent,
}
impl<'a> AgentView<'a> {
    /// The agent's handle.
    pub fn handle(&self) -> AgentHandle {
        self.handle
    }
    /// The agent's simulation tier.
    pub fn tier(&self) -> Tier {
        self.tier
    }
    /// Current position.
    pub fn position(&self) -> V3 {
        self.agent.npos
    }
    /// Actual velocity.
    pub fn velocity(&self) -> V3 {
        self.agent.vel
    }
    /// Locomotion state.
    pub fn state(&self) -> AgentState {
        self.agent.state
    }
    /// Move request state.
    pub fn target_state(&self) -> TargetState {
        self.agent.target_state
    }
    /// Physical and steering configuration.
    pub fn params(&self) -> &'a AgentParams {
        &self.agent.params
    }
    /// The full simulation state (corridor, walls, corners, neighbours) of an agent
    /// simulated at [`Tier::Full`]. Neighbour entries name agent indices.
    pub fn detail(&self) -> Option<&'a CrowdAgent> {
        Some(self.agent)
    }
}

/// A crowd of up to [`MAX_WORLD_AGENTS`] agents with handles, a sorted-cell
/// neighbour grid and data-parallel phases; see the [module documentation](self).
pub struct CrowdWorld {
    params: CrowdWorldParams,
    limits: Limits,
    extents: V3,
    fixes: UpstreamFixes,
    filters: [DefaultFilter; 16],
    presets: [ObstacleAvoidanceParams; 8],
    pathq: PathQueue,
    // Per agent index.
    generation: Vec<u32>,
    heavy_of: Vec<u32>,
    pos: Vec<V3>,
    vel: Vec<V3>,
    dvel: Vec<V3>,
    radius: Vec<f32>,
    height: Vec<f32>,
    max_speed: Vec<f32>,
    free: BinaryHeap<Reverse<u32>>,
    count: u32,
    // Side pool.
    heavy: Vec<CrowdAgent>,
    owner: Vec<u32>,
    free_heavy: BinaryHeap<Reverse<u32>>,
    // Per-step scratch.
    grid: CellGrid,
    active: Vec<usize>,
}

impl CrowdWorld {
    /// Recommended full-tier agent settings: ORCA avoidance. Explicit settings
    /// passed to [`Self::add_agent`] are preserved, including sampled avoidance.
    pub fn default_agent_params() -> AgentParams {
        AgentParams {
            avoidance: AvoidanceMode::Orca(OrcaParams::default()),
            ..AgentParams::default()
        }
    }
    /// An empty world for agents on `nav`. `limits` bound each call's allocation and
    /// the work of each agent in each phase.
    pub fn new(nav: &NavMesh, p: &CrowdWorldParams, limits: Limits) -> Result<Self, Error> {
        p.validate()?;
        let mut ws = Workspace::with_limits(limits);
        let pathq = PathQueue::new_in(&mut ws, nav, 256, p.path_queue.max_nodes)?;
        let r = p.max_radius;
        Ok(Self {
            params: *p,
            limits,
            extents: [r * 2., r * 1.5, r * 2.],
            fixes: UpstreamFixes::NEAREST_NEIGHBOURS,
            filters: core::array::from_fn(|_| DefaultFilter::default()),
            presets: [ObstacleAvoidanceParams::default(); 8],
            pathq,
            generation: Vec::new(),
            heavy_of: Vec::new(),
            pos: Vec::new(),
            vel: Vec::new(),
            dvel: Vec::new(),
            radius: Vec::new(),
            height: Vec::new(),
            max_speed: Vec::new(),
            free: BinaryHeap::new(),
            count: 0,
            heavy: Vec::new(),
            owner: Vec::new(),
            free_heavy: BinaryHeap::new(),
            grid: CellGrid::new(r * 3.),
            active: Vec::new(),
        })
    }
    /// The configuration the world was created with.
    pub fn params(&self) -> &CrowdWorldParams {
        &self.params
    }
    /// Selects upstream defect corrections, as [`Crowd::set_fixes`]. Neighbours are
    /// always the nearest by `(distance, index)`, so
    /// [`UpstreamFixes::NEAREST_NEIGHBOURS`] is always included.
    pub fn set_fixes(&mut self, fixes: UpstreamFixes) {
        self.fixes = fixes | UpstreamFixes::NEAREST_NEIGHBOURS;
        self.pathq.set_fixes(self.fixes);
    }
    /// The corrections the world applies.
    pub fn fixes(&self) -> UpstreamFixes {
        self.fixes
    }
    /// Sets one of sixteen query filters.
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
    /// Sets one of eight sampled-avoidance presets.
    pub fn set_obstacle_avoidance_params(
        &mut self,
        index: usize,
        p: ObstacleAvoidanceParams,
    ) -> Result<(), Error> {
        p.validate()?;
        *self
            .presets
            .get_mut(index)
            .ok_or(Error::InvalidParameter("avoidance index"))? = p;
        Ok(())
    }
    /// Reads a sampled-avoidance preset.
    pub fn obstacle_avoidance_params(&self, index: usize) -> Option<&ObstacleAvoidanceParams> {
        self.presets.get(index)
    }
    /// Half extents of the box used to place agents on the navmesh.
    pub fn query_half_extents(&self) -> V3 {
        self.extents
    }
    /// Number of agents.
    pub fn len(&self) -> usize {
        self.count as usize
    }
    /// Whether the world has no agents.
    pub fn is_empty(&self) -> bool {
        self.count == 0
    }
    /// The path queue serving the agents' path requests.
    pub fn path_queue(&self) -> &PathQueue {
        &self.pathq
    }

    fn slot_of(&self, h: AgentHandle) -> Option<usize> {
        let i = h.index as usize;
        let s = *self.heavy_of.get(i)?;
        (s != NONE && self.generation.get(i) == Some(&h.generation)).then_some(s as usize)
    }
    fn heavy_mut(&mut self, h: AgentHandle) -> Result<&mut CrowdAgent, Error> {
        let s = self
            .slot_of(h)
            .ok_or(Error::InvalidParameter("stale agent handle"))?;
        Ok(&mut self.heavy[s])
    }

    /// Places an agent on the nearest polygon, or marks it invalid if none is near
    /// ([`Crowd::add_agent`] semantics). Takes the lowest free index.
    /// [`Error::BufferTooSmall`] when the world or its side pool is full.
    pub fn add_agent(
        &mut self,
        nav: &NavMesh,
        pos: V3,
        p: AgentParams,
    ) -> Result<AgentHandle, Error> {
        let mut ws = Workspace::with_limits(self.limits);
        finite(pos)?;
        p.validate(self.params.max_radius)?;
        let index = match self.free.peek() {
            Some(&Reverse(i)) => i,
            None if self.count < self.params.max_agents => self.generation.len() as u32,
            None => return Err(Error::BufferTooSmall),
        };
        let slot = match self.free_heavy.peek() {
            Some(&Reverse(s)) => s,
            None if (self.heavy.len() as u32) < self.params.max_heavy_agents => {
                self.heavy.len() as u32
            }
            None => return Err(Error::BufferTooSmall),
        };
        ws.charge_bytes::<u8>(512 * 128 + 16384)?;
        let q = NavMeshQuery::new(nav, 512)?;
        let (r, nearest, _) = q
            .find_nearest_poly(&pos, &self.extents, &self.filters[p.query_filter_type])
            .unwrap_or((0, pos, false));
        let nearest = if r == 0 { pos } else { nearest };
        if index as usize == self.generation.len() {
            self.grow_by_one()?;
            self.free.push(Reverse(index));
        }
        if slot as usize == self.heavy.len() {
            let entry = CrowdAgent::new_in(&mut ws)?;
            self.heavy
                .try_reserve(1)
                .map_err(|_| Error::AllocationFailed)?;
            self.owner
                .try_reserve(1)
                .map_err(|_| Error::AllocationFailed)?;
            self.heavy.push(entry);
            self.owner.push(NONE);
            self.free_heavy.push(Reverse(slot));
        }
        // Grown storage stays on the free lists (as their minimum) until the agent
        // is placed.
        self.heavy[slot as usize].place(r, nearest, p)?;
        self.free.pop();
        self.free_heavy.pop();
        let i = index as usize;
        self.owner[slot as usize] = index;
        self.heavy_of[i] = slot;
        self.pos[i] = nearest;
        self.vel[i] = [0.; 3];
        self.dvel[i] = [0.; 3];
        self.set_dimensions(i, &p);
        self.count += 1;
        Ok(AgentHandle {
            index,
            generation: self.generation[i],
        })
    }
    fn grow_by_one(&mut self) -> Result<(), Error> {
        fn reserve<T>(v: &mut Vec<T>) -> Result<(), Error> {
            v.try_reserve(1).map_err(|_| Error::AllocationFailed)
        }
        reserve(&mut self.generation)?;
        reserve(&mut self.heavy_of)?;
        reserve(&mut self.pos)?;
        reserve(&mut self.vel)?;
        reserve(&mut self.dvel)?;
        reserve(&mut self.radius)?;
        reserve(&mut self.height)?;
        reserve(&mut self.max_speed)?;
        self.generation.push(0);
        self.heavy_of.push(NONE);
        self.pos.push([0.; 3]);
        self.vel.push([0.; 3]);
        self.dvel.push([0.; 3]);
        self.radius.push(0.);
        self.height.push(0.);
        self.max_speed.push(0.);
        Ok(())
    }

    fn set_dimensions(&mut self, i: usize, p: &AgentParams) {
        self.radius[i] = p.radius;
        self.height[i] = p.height;
        self.max_speed[i] = p.max_speed;
    }
    /// Removes an agent; its handle stops resolving and its index becomes free.
    pub fn remove_agent(&mut self, h: AgentHandle) -> Result<(), Error> {
        let s = self
            .slot_of(h)
            .ok_or(Error::InvalidParameter("stale agent handle"))?;
        let i = h.index as usize;
        self.heavy[s].reset_slot();
        self.owner[s] = NONE;
        self.free_heavy.push(Reverse(s as u32));
        self.heavy_of[i] = NONE;
        self.generation[i] = self.generation[i].wrapping_add(1);
        self.pos[i] = [0.; 3];
        self.vel[i] = [0.; 3];
        self.dvel[i] = [0.; 3];
        self.free.push(Reverse(h.index));
        self.count -= 1;
        Ok(())
    }
    /// Updates an agent's physical and steering settings.
    pub fn set_agent_params(&mut self, h: AgentHandle, p: AgentParams) -> Result<(), Error> {
        p.validate(self.params.max_radius)?;
        self.heavy_mut(h)?.params = p;
        self.set_dimensions(h.index as usize, &p);
        Ok(())
    }
    /// Sets what the agent moves towards; processed on the next step. A point goal
    /// needs a valid polygon of `nav` or one of an unloaded tile (the agent then
    /// waits for the tile); otherwise [`Error::InvalidRef`].
    pub fn set_goal(&mut self, nav: &NavMesh, h: AgentHandle, goal: Goal) -> Result<(), Error> {
        match goal {
            Goal::None => self.heavy_mut(h)?.reset_target(),
            Goal::Velocity(v) => {
                finite(v)?;
                self.heavy_mut(h)?.request_velocity(v);
            }
            Goal::Point { poly, pos } => {
                finite(pos)?;
                let waiting = target_tile(nav, poly)?;
                self.heavy_mut(h)?.request_target(poly, pos, waiting)?;
            }
        }
        Ok(())
    }
    /// Reads an agent; `None` for a stale handle.
    pub fn agent(&self, h: AgentHandle) -> Option<AgentView<'_>> {
        let s = self.slot_of(h)?;
        Some(AgentView {
            handle: h,
            tier: Tier::Full,
            agent: &self.heavy[s],
        })
    }
    /// The handle of the agent at `index`, if one is there.
    pub fn handle_at(&self, index: u32) -> Option<AgentHandle> {
        let i = index as usize;
        (*self.heavy_of.get(i)? != NONE).then(|| AgentHandle {
            index,
            generation: self.generation[i],
        })
    }
    /// Every agent in index order.
    pub fn agents(&self) -> impl Iterator<Item = AgentView<'_>> {
        self.heavy_of
            .iter()
            .enumerate()
            .filter(|(_, s)| **s != NONE)
            .map(|(i, &s)| AgentView {
                handle: AgentHandle {
                    index: i as u32,
                    generation: self.generation[i],
                },
                tier: Tier::Full,
                agent: &self.heavy[s as usize],
            })
    }
    /// Agents whose planar position `(x, z)` lies in the box from `bmin` to `bmax`
    /// (inclusive), in index order.
    pub fn agents_in(&self, bmin: [f32; 2], bmax: [f32; 2]) -> impl Iterator<Item = AgentView<'_>> {
        self.agents().filter(move |v| {
            let p = v.position();
            (bmin[0]..=bmax[0]).contains(&p[0]) && (bmin[1]..=bmax[1]).contains(&p[2])
        })
    }
    /// A 64-bit FNV-1a digest of every agent's index, generation, states, position,
    /// velocities and corridor, in index order: equal digests across runs, thread
    /// counts and targets mean equal simulations.
    pub fn digest(&self) -> u64 {
        let mut h = 0xcbf2_9ce4_8422_2325u64;
        let mut put = |bytes: &[u8]| {
            for &b in bytes {
                h ^= u64::from(b);
                h = h.wrapping_mul(0x0100_0000_01b3);
            }
        };
        for v in self.agents() {
            let a = v.agent;
            put(&v.handle.index.to_le_bytes());
            put(&v.handle.generation.to_le_bytes());
            put(&[a.state as u8, a.target_state as u8]);
            for x in a.npos.iter().chain(&a.vel).chain(&a.nvel).chain(&a.dvel) {
                put(&x.to_bits().to_le_bytes());
            }
            put(&(a.corridor.path.len() as u32).to_le_bytes());
            for &r in &a.corridor.path {
                put(&snapshot::wide(r).to_le_bytes());
            }
        }
        h
    }
}
