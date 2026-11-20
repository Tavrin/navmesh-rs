//! Crowd admission and movement using scoped targets.

use alloc::sync::Arc;
use alloc::vec::Vec;

use super::{ProfileError, ProfileMesh, ProfilePoint};
use crate::crowd::{AgentId, AgentParams, Crowd};
use crate::detour::DefaultFilter;
use crate::{Error, UpstreamFixes, Workspace};

#[cfg(feature = "crowd-world")]
use crate::crowd::world::{AgentHandle, CrowdWorld, CrowdWorldParams, Goal, StepReport};
#[cfg(feature = "crowd-world")]
use crate::{Budget, Budgeted, Limits};

#[cfg(feature = "crowd-world")]
mod mixed;
#[cfg(feature = "crowd-world")]
pub use mixed::{FramedMixedProfileCrowdWorld, MixedProfileCrowdWorld};

fn check_envelope(mesh: &ProfileMesh, params: AgentParams) -> Result<(), ProfileError> {
    let config = mesh.catalog.spec.config();
    if params.height > config.walkable_height || params.radius > config.walkable_radius {
        return Err(Error::InvalidParameter("agent exceeds its profile dimensions").into());
    }
    Ok(())
}

/// A live agent slot in a specific profile crowd.
#[derive(Clone, Debug)]
pub struct ProfileAgent {
    slot: AgentId,
    admission: Arc<()>,
}

/// A legacy crowd whose operations are bound to one profile mesh.
pub struct ProfileCrowd {
    owner: Arc<()>,
    crowd: Crowd,
    admissions: Vec<Option<Arc<()>>>,
}

impl ProfileCrowd {
    /// Creates a crowd using the profile's world-space radius as its maximum.
    pub fn new(
        mesh: &ProfileMesh,
        ws: &mut Workspace,
        max_agents: usize,
    ) -> Result<Self, ProfileError> {
        let crowd = Crowd::new(
            ws,
            &mesh.nav,
            max_agents,
            mesh.catalog.spec.config().walkable_radius,
        )?;
        let mut admissions = ws.vec_with_capacity(max_agents)?;
        admissions.resize_with(max_agents, || None);
        Ok(Self {
            owner: Arc::clone(mesh.nav.report_identity()),
            crowd,
            admissions,
        })
    }

    /// Admits an agent whose world-space radius and height fit the bake envelope.
    pub fn add_agent(
        &mut self,
        mesh: &ProfileMesh,
        ws: &mut Workspace,
        position: [f32; 3],
        params: AgentParams,
    ) -> Result<ProfileAgent, ProfileError> {
        mesh.check_owner(&self.owner)?;
        check_envelope(mesh, params)?;
        let slot = self.crowd.add_agent(ws, &mesh.nav, position, params)?;
        let admission = Arc::new(());
        self.admissions[slot.0] = Some(Arc::clone(&admission));
        Ok(ProfileAgent { slot, admission })
    }

    fn slot(&self, agent: &ProfileAgent) -> Result<AgentId, ProfileError> {
        let same = self
            .admissions
            .get(agent.slot.0)
            .and_then(Option::as_ref)
            .is_some_and(|admission| Arc::ptr_eq(admission, &agent.admission));
        if !same {
            return Err(ProfileError::BindingMismatch);
        }
        Ok(agent.slot)
    }

    /// Removes an agent and invalidates its admission handle.
    pub fn remove_agent(&mut self, agent: &ProfileAgent) -> Result<(), ProfileError> {
        let slot = self.slot(agent)?;
        self.crowd.remove_agent(slot)?;
        self.admissions[slot.0] = None;
        Ok(())
    }

    /// Updates the settings of an admitted agent within its bake envelope.
    pub fn set_agent_params(
        &mut self,
        mesh: &ProfileMesh,
        agent: &ProfileAgent,
        params: AgentParams,
    ) -> Result<(), ProfileError> {
        mesh.check_owner(&self.owner)?;
        check_envelope(mesh, params)?;
        Ok(self
            .crowd
            .update_agent_parameters(self.slot(agent)?, params)?)
    }

    /// Sets the agent's destination using a point from the bound mesh.
    pub fn request_move_target(
        &mut self,
        mesh: &ProfileMesh,
        agent: &ProfileAgent,
        target: &ProfilePoint,
    ) -> Result<(), ProfileError> {
        mesh.check_owner(&self.owner)?;
        Ok(self.crowd.request_move_target(
            &mesh.nav,
            self.slot(agent)?,
            mesh.polygon(&target.polygon)?,
            target.position,
        )?)
    }

    /// Steers an admitted agent directly by desired velocity.
    pub fn request_move_velocity(
        &mut self,
        agent: &ProfileAgent,
        velocity: [f32; 3],
    ) -> Result<(), ProfileError> {
        Ok(self
            .crowd
            .request_move_velocity(self.slot(agent)?, velocity)?)
    }

    /// Clears an admitted agent's move request.
    pub fn reset_move_target(&mut self, agent: &ProfileAgent) -> Result<(), ProfileError> {
        Ok(self.crowd.reset_move_target(self.slot(agent)?)?)
    }

    /// Reads the current position of an admitted agent.
    pub fn position(&self, agent: &ProfileAgent) -> Result<[f32; 3], ProfileError> {
        Ok(self
            .crowd
            .agent(self.slot(agent)?)
            .ok_or(Error::InvalidRef)?
            .npos)
    }

    /// Sets a profile-local area and cost filter slot.
    pub fn set_filter(&mut self, index: usize, filter: DefaultFilter) -> Result<(), ProfileError> {
        Ok(self.crowd.set_filter(index, filter)?)
    }

    /// Enables the specified legacy-crowd behavior fixes.
    pub fn set_fixes(&mut self, fixes: UpstreamFixes) {
        self.crowd.set_fixes(fixes);
    }

    /// Advances the crowd using its bound mesh.
    pub fn update(
        &mut self,
        mesh: &ProfileMesh,
        ws: &mut Workspace,
        dt: f32,
    ) -> Result<(), ProfileError> {
        mesh.check_owner(&self.owner)?;
        Ok(self.crowd.update(ws, &mesh.nav, dt)?)
    }
}

/// A generational agent handle scoped to a profile crowd world.
#[cfg(feature = "crowd-world")]
#[derive(Clone, Debug)]
pub struct ProfileWorldAgent {
    world: Arc<()>,
    handle: AgentHandle,
}

#[cfg(feature = "crowd-world")]
impl ProfileWorldAgent {
    /// Profile-local slot index for locating an agent after restoring its crowd.
    pub fn index(&self) -> u32 {
        self.handle.index()
    }
}

/// A crowd world that always admits, routes and advances agents on one profile mesh.
#[cfg(feature = "crowd-world")]
pub struct ProfileCrowdWorld {
    owner: Arc<()>,
    world: Arc<()>,
    crowd: CrowdWorld,
}

#[cfg(feature = "crowd-world")]
impl ProfileCrowdWorld {
    /// Creates an empty crowd world within the profile's bake envelope.
    pub fn new(
        mesh: &ProfileMesh,
        params: &CrowdWorldParams,
        limits: Limits,
    ) -> Result<Self, ProfileError> {
        if params.max_radius > mesh.catalog.spec.config().walkable_radius {
            return Err(Error::InvalidParameter("crowd exceeds its profile radius").into());
        }
        Ok(Self::bind(
            mesh,
            CrowdWorld::new(&mesh.nav, params, limits)?,
        ))
    }

    fn bind(mesh: &ProfileMesh, crowd: CrowdWorld) -> Self {
        Self {
            owner: Arc::clone(mesh.nav.report_identity()),
            world: Arc::new(()),
            crowd,
        }
    }

    /// Admits an agent whose world-space dimensions fit its bake envelope.
    pub fn add_agent(
        &mut self,
        mesh: &ProfileMesh,
        position: [f32; 3],
        params: AgentParams,
    ) -> Result<ProfileWorldAgent, ProfileError> {
        mesh.check_owner(&self.owner)?;
        check_envelope(mesh, params)?;
        let handle = self.crowd.add_agent(&mesh.nav, position, params)?;
        Ok(self.scoped(handle))
    }

    fn scoped(&self, handle: AgentHandle) -> ProfileWorldAgent {
        ProfileWorldAgent {
            world: Arc::clone(&self.world),
            handle,
        }
    }

    fn handle(&self, agent: &ProfileWorldAgent) -> Result<AgentHandle, ProfileError> {
        if Arc::ptr_eq(&self.world, &agent.world) {
            Ok(agent.handle)
        } else {
            Err(ProfileError::BindingMismatch)
        }
    }

    /// Current active agent at a profile-local slot, including after a restore.
    pub fn agent_at(&self, index: u32) -> Option<ProfileWorldAgent> {
        self.crowd
            .handle_at(index)
            .map(|handle| self.scoped(handle))
    }

    /// Removes an admitted agent.
    pub fn remove_agent(&mut self, agent: &ProfileWorldAgent) -> Result<(), ProfileError> {
        Ok(self.crowd.remove_agent(self.handle(agent)?)?)
    }

    /// Updates an admitted agent's settings within its bake envelope.
    pub fn set_agent_params(
        &mut self,
        mesh: &ProfileMesh,
        agent: &ProfileWorldAgent,
        params: AgentParams,
    ) -> Result<(), ProfileError> {
        mesh.check_owner(&self.owner)?;
        check_envelope(mesh, params)?;
        Ok(self.crowd.set_agent_params(self.handle(agent)?, params)?)
    }

    /// Requests a scoped point goal, or clears the current goal with no point.
    pub fn set_goal(
        &mut self,
        mesh: &ProfileMesh,
        agent: &ProfileWorldAgent,
        target: Option<&ProfilePoint>,
    ) -> Result<(), ProfileError> {
        mesh.check_owner(&self.owner)?;
        let goal = match target {
            Some(target) => Goal::Point {
                poly: mesh.polygon(&target.polygon)?,
                pos: target.position,
            },
            None => Goal::None,
        };
        Ok(self.crowd.set_goal(&mesh.nav, self.handle(agent)?, goal)?)
    }

    /// Steers an admitted agent directly by desired velocity.
    pub fn set_velocity(
        &mut self,
        mesh: &ProfileMesh,
        agent: &ProfileWorldAgent,
        velocity: [f32; 3],
    ) -> Result<(), ProfileError> {
        mesh.check_owner(&self.owner)?;
        Ok(self
            .crowd
            .set_goal(&mesh.nav, self.handle(agent)?, Goal::Velocity(velocity))?)
    }

    /// Reads the current position of an admitted agent.
    pub fn position(&self, agent: &ProfileWorldAgent) -> Result<[f32; 3], ProfileError> {
        Ok(self
            .crowd
            .agent(self.handle(agent)?)
            .ok_or(Error::InvalidRef)?
            .position())
    }

    /// Sets a profile-local area and cost filter slot.
    pub fn set_filter(&mut self, index: usize, filter: DefaultFilter) -> Result<(), ProfileError> {
        Ok(self.crowd.set_filter(index, filter)?)
    }

    /// Enables the specified crowd-world behavior fixes.
    pub fn set_fixes(&mut self, fixes: UpstreamFixes) {
        self.crowd.set_fixes(fixes);
    }

    /// Advances the crowd on its bound mesh by one fixed tick.
    pub fn step(
        &mut self,
        mesh: &ProfileMesh,
        budget: &mut Budget,
    ) -> Result<Budgeted<StepReport>, ProfileError> {
        mesh.check_owner(&self.owner)?;
        Ok(self.crowd.step(&mesh.nav, budget)?)
    }

    /// Hashes the current simulation state in agent index order.
    pub fn digest(&self) -> u64 {
        self.crowd.digest()
    }

    /// Saves this crowd with its catalog binding and matching mesh slot history.
    pub fn save(&self, mesh: &ProfileMesh) -> Result<Vec<u8>, ProfileError> {
        mesh.check_owner(&self.owner)?;
        mesh.wrap_residency_bound(*b"NPCW", &self.crowd.profile_snapshot(&mesh.nav)?)
    }

    /// Restores after restoring residency and reloading the required mesh assets.
    pub fn restore(mesh: &ProfileMesh, bytes: &[u8], limits: Limits) -> Result<Self, ProfileError> {
        let crowd = CrowdWorld::restore_profile_snapshot(
            &mesh.nav,
            mesh.residency_bound_payload(*b"NPCW", bytes)?,
            limits,
        )?;
        if crowd.params().max_radius > mesh.catalog.spec.config().walkable_radius {
            return Err(Error::InvalidParameter("crowd exceeds its profile radius").into());
        }
        for agent in crowd.agents() {
            check_envelope(mesh, *agent.params())?;
        }
        Ok(Self::bind(mesh, crowd))
    }
}
