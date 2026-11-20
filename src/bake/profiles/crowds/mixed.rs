//! One avoidance simulation over independently baked profile meshes.

use super::super::catalog::{read_version, take};
use super::super::{ProfileId, ProfileMeshes};
use super::*;
use crate::crowd::OffMeshReached;
use crate::crowd::world::AgentView;
use crate::detour::{NavMesh, content_hash};
use alloc::collections::BTreeMap;

const MAGIC: [u8; 4] = *b"NPCM";
const VERSION: u32 = 1;

mod framed;
pub use framed::FramedMixedProfileCrowdWorld;

/// A single crowd world whose agents use different bake profiles in a common
/// coordinate frame. All profiles share the neighbour grid, path queue budget,
/// replan budget and fixed time step.
#[non_exhaustive]
pub struct MixedProfileCrowdWorld {
    owners: BTreeMap<u32, Arc<()>>,
    world: Arc<()>,
    crowd: CrowdWorld,
}

impl MixedProfileCrowdWorld {
    /// Binds a nonempty set of profile meshes. Mesh instances must stay bound
    /// for the lifetime of this crowd. Compatible catalog updates are allowed.
    pub fn new(
        meshes: &ProfileMeshes,
        profiles: &[ProfileId],
        params: &CrowdWorldParams,
        limits: Limits,
    ) -> Result<Self, ProfileError> {
        let mut navs = BTreeMap::new();
        for &id in profiles {
            let mesh = meshes.get(id)?;
            if navs.insert(id.0, &mesh.nav).is_some() {
                return Err(ProfileError::DuplicateProfile(id));
            }
        }
        let crowd = CrowdWorld::new_mixed(&navs, params, limits)?;
        Ok(Self::bind(&navs, crowd))
    }

    fn bind(navs: &BTreeMap<u32, &NavMesh>, crowd: CrowdWorld) -> Self {
        Self {
            owners: navs
                .iter()
                .map(|(&id, nav)| (id, Arc::clone(nav.report_identity())))
                .collect(),
            world: Arc::new(()),
            crowd,
        }
    }

    fn mesh<'a>(
        &self,
        meshes: &'a ProfileMeshes,
        id: ProfileId,
    ) -> Result<&'a ProfileMesh, ProfileError> {
        let owner = self
            .owners
            .get(&id.0)
            .ok_or(ProfileError::UnknownProfile(id))?;
        let mesh = meshes.get(id)?;
        mesh.check_owner(owner)?;
        Ok(mesh)
    }

    fn navs<'a>(
        &self,
        meshes: &'a ProfileMeshes,
    ) -> Result<BTreeMap<u32, &'a NavMesh>, ProfileError> {
        self.owners
            .keys()
            .map(|&id| Ok((id, &self.mesh(meshes, ProfileId::new(id))?.nav)))
            .collect()
    }

    fn scoped(&self, handle: AgentHandle) -> ProfileWorldAgent {
        ProfileWorldAgent {
            world: Arc::clone(&self.world),
            handle,
        }
    }

    fn handle(&self, agent: &ProfileWorldAgent) -> Result<AgentHandle, ProfileError> {
        if Arc::ptr_eq(&self.world, &agent.world) && self.crowd.agent(agent.handle).is_some() {
            Ok(agent.handle)
        } else {
            Err(ProfileError::BindingMismatch)
        }
    }

    /// Admits an agent within the chosen profile's baked dimensions.
    pub fn add_agent(
        &mut self,
        meshes: &ProfileMeshes,
        id: ProfileId,
        position: [f32; 3],
        params: AgentParams,
    ) -> Result<ProfileWorldAgent, ProfileError> {
        let mesh = self.mesh(meshes, id)?;
        check_envelope(mesh, params)?;
        let handle = self
            .crowd
            .add_profile_agent(id.0, &mesh.nav, position, params)?;
        Ok(self.scoped(handle))
    }

    /// Finds an active agent by its global slot, including after restore.
    pub fn agent_at(&self, index: u32) -> Option<ProfileWorldAgent> {
        self.crowd
            .handle_at(index)
            .map(|handle| self.scoped(handle))
    }

    /// Returns the profile bound at admission.
    pub fn profile(&self, agent: &ProfileWorldAgent) -> Result<ProfileId, ProfileError> {
        self.crowd
            .agent_profile(self.handle(agent)?)
            .map(ProfileId::new)
            .ok_or(ProfileError::BindingMismatch)
    }

    /// Reads an active agent's state.
    pub fn agent(&self, agent: &ProfileWorldAgent) -> Result<AgentView<'_>, ProfileError> {
        self.crowd
            .agent(self.handle(agent)?)
            .ok_or(ProfileError::BindingMismatch)
    }

    /// Removes an agent and invalidates its handle.
    pub fn remove_agent(&mut self, agent: &ProfileWorldAgent) -> Result<(), ProfileError> {
        Ok(self.crowd.remove_agent(self.handle(agent)?)?)
    }

    /// Updates settings within the dimensions of the bound profile.
    pub fn set_agent_params(
        &mut self,
        meshes: &ProfileMeshes,
        agent: &ProfileWorldAgent,
        params: AgentParams,
    ) -> Result<(), ProfileError> {
        check_envelope(self.mesh(meshes, self.profile(agent)?)?, params)?;
        Ok(self.crowd.set_agent_params(self.handle(agent)?, params)?)
    }

    /// Sets a goal issued by the agent's own profile mesh, or clears the goal.
    pub fn set_goal(
        &mut self,
        meshes: &ProfileMeshes,
        agent: &ProfileWorldAgent,
        target: Option<&ProfilePoint>,
    ) -> Result<(), ProfileError> {
        let mesh = self.mesh(meshes, self.profile(agent)?)?;
        let goal = match target {
            Some(target) => Goal::Point {
                poly: mesh.polygon(&target.polygon)?,
                pos: target.position,
            },
            None => Goal::None,
        };
        Ok(self.crowd.set_goal(&mesh.nav, self.handle(agent)?, goal)?)
    }

    /// Steers the agent by desired velocity while constraining it to its mesh.
    pub fn set_velocity(
        &mut self,
        meshes: &ProfileMeshes,
        agent: &ProfileWorldAgent,
        velocity: [f32; 3],
    ) -> Result<(), ProfileError> {
        let mesh = self.mesh(meshes, self.profile(agent)?)?;
        Ok(self
            .crowd
            .set_goal(&mesh.nav, self.handle(agent)?, Goal::Velocity(velocity))?)
    }

    /// Lists pending external traversals in global agent index order. Event
    /// polygon refs are local to the returned profile.
    pub fn off_mesh_arrivals(
        &self,
    ) -> impl Iterator<Item = (ProfileWorldAgent, ProfileId, OffMeshReached)> + '_ {
        self.crowd
            .off_mesh_arrivals()
            .filter_map(|(handle, event)| {
                let profile = self.crowd.agent_profile(handle)?;
                Some((self.scoped(handle), ProfileId::new(profile), event))
            })
    }

    /// Completes a pending external traversal on the agent's bound mesh.
    pub fn complete_off_mesh(
        &mut self,
        meshes: &ProfileMeshes,
        agent: &ProfileWorldAgent,
        landing: [f32; 3],
    ) -> Result<(), ProfileError> {
        let mesh = self.mesh(meshes, self.profile(agent)?)?;
        Ok(self
            .crowd
            .complete_off_mesh(&mesh.nav, self.handle(agent)?, landing)?)
    }

    /// Aborts a pending external traversal for a replan from its departure.
    pub fn abort_off_mesh(&mut self, agent: &ProfileWorldAgent) -> Result<(), ProfileError> {
        Ok(self.crowd.abort_off_mesh(self.handle(agent)?)?)
    }

    /// Sets a filter slot shared by all agents. Each agent applies the selected
    /// filter to the polygons of its own mesh.
    pub fn set_filter(&mut self, index: usize, filter: DefaultFilter) -> Result<(), ProfileError> {
        Ok(self.crowd.set_filter(index, filter)?)
    }

    /// Sets a sampled avoidance preset shared by all profiles.
    pub fn set_obstacle_avoidance_params(
        &mut self,
        index: usize,
        params: crate::crowd::ObstacleAvoidanceParams,
    ) -> Result<(), ProfileError> {
        Ok(self.crowd.set_obstacle_avoidance_params(index, params)?)
    }

    /// Enables the specified crowd-world behavior fixes.
    pub fn set_fixes(&mut self, fixes: UpstreamFixes) {
        self.crowd.set_fixes(fixes);
    }

    /// Sets the global number of stamp-triggered replans admitted per tick.
    pub fn set_replan_limit(&mut self, limit: u32) {
        self.crowd.set_replan_limit(limit);
    }

    /// Advances the shared avoidance simulation by one fixed tick. Like
    /// [`CrowdWorld::step`], exhaustion leaves the simulation unchanged.
    pub fn step(
        &mut self,
        meshes: &ProfileMeshes,
        budget: &mut Budget,
    ) -> Result<Budgeted<StepReport>, ProfileError> {
        Ok(self.crowd.step_mixed(
            &self.navs(meshes)?,
            budget,
            &BTreeMap::new(),
            &BTreeMap::new(),
        )?)
    }

    /// Hashes current agent state, including the profile assignments.
    pub fn digest(&self) -> u64 {
        self.crowd.digest()
    }

    /// Saves a versioned snapshot bound to all registered catalogs and slot
    /// histories. Step once after mesh changes so the snapshot captures their
    /// effect on replanning before mesh epochs are reconstructed on restore.
    pub fn save(&self, meshes: &ProfileMeshes) -> Result<Vec<u8>, ProfileError> {
        let payload = self.crowd.mixed_snapshot(&self.navs(meshes)?)?;
        let mut bytes = Vec::new();
        bytes.extend(MAGIC);
        bytes.extend(VERSION.to_le_bytes());
        bytes.extend((self.owners.len() as u64).to_le_bytes());
        for &id in self.owners.keys() {
            let mesh = self.mesh(meshes, ProfileId::new(id))?;
            bytes.extend(id.to_le_bytes());
            bytes.extend(content_hash(&mesh.save_residency()?));
        }
        bytes.extend(payload);
        bytes.extend(content_hash(&bytes));
        Ok(bytes)
    }

    /// Restores after restoring every bound profile's residency, modifier state
    /// and required tiles. Restored agents have handles scoped to the new crowd.
    pub fn restore(
        meshes: &ProfileMeshes,
        bytes: &[u8],
        limits: Limits,
    ) -> Result<Self, ProfileError> {
        let mut body = checked_payload(bytes, MAGIC, VERSION, limits)?;
        let count = u64::from_le_bytes(take(&mut body)?);
        if count == 0 || count > (body.len() / 36) as u64 {
            return Err(ProfileError::InvalidRecord);
        }
        let mut navs = BTreeMap::new();
        for _ in 0..count {
            let id = u32::from_le_bytes(take(&mut body)?);
            if navs.last_key_value().is_some_and(|(&prev, _)| id <= prev) {
                return Err(ProfileError::InvalidRecord);
            }
            let mesh = meshes.get(ProfileId::new(id))?;
            if take::<32>(&mut body)? != content_hash(&mesh.save_residency()?) {
                return Err(ProfileError::BindingMismatch);
            }
            navs.insert(id, &mesh.nav);
        }
        let crowd = CrowdWorld::restore_mixed_snapshot(&navs, body, limits)?;
        for agent in crowd.agents() {
            let profile = crowd
                .agent_profile(agent.handle())
                .ok_or(ProfileError::InvalidRecord)?;
            check_envelope(meshes.get(ProfileId::new(profile))?, *agent.params())?;
        }
        Ok(Self::bind(&navs, crowd))
    }
}

fn checked_payload(
    bytes: &[u8],
    magic: [u8; 4],
    version: u32,
    limits: Limits,
) -> Result<&[u8], ProfileError> {
    let body = read_version(bytes, magic, version)?;
    if bytes.len() > limits.max_bytes {
        return Err(Error::LimitExceeded.into());
    }
    let (payload, hash) = body
        .split_at_checked(body.len().saturating_sub(32))
        .ok_or(ProfileError::InvalidRecord)?;
    if hash.len() != 32 || hash != content_hash(&bytes[..bytes.len() - 32]) {
        return Err(ProfileError::InvalidRecord);
    }
    Ok(payload)
}
