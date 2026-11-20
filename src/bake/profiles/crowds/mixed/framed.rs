//! Profile-aware platform routing with one mixed crowd per coordinate frame.

use super::*;
mod snapshot;
use crate::crowd::AgentState;
use crate::crowd::platforms::dock_avoidance::collect_obstacles;
use crate::crowd::platforms::simulation::{DockCrossing, first_dock, same_target};
use crate::crowd::platforms::{LocalTarget, PlatformAgentId};
use crate::crowd::world::STEP_WORK_PER_AGENT;
use crate::detour::{LinkId, PolyRef};
use crate::frames::{FrameId, FramePoint, FrameRouting, FrameSet};
use alloc::collections::BTreeSet;

#[derive(Clone, Copy)]
struct Rider {
    frame: FrameId,
    handle: AgentHandle,
    goal: Option<FramePoint>,
}

struct Intent {
    id: PlatformAgentId,
    rider: Rider,
    profile: ProfileId,
    target: Option<(PolyRef, [f32; 3])>,
    dock: Option<DockCrossing>,
}

/// Mixed-profile simulations connected by platform docks. All agents sharing a
/// frame use the same avoidance grid. At open seams, agents in the adjoining
/// frame are moving avoidance obstacles, using their actual radii.
#[derive(Default)]
#[non_exhaustive]
pub struct FramedMixedProfileCrowdWorld {
    crowds: BTreeMap<FrameId, MixedProfileCrowdWorld>,
    riders: BTreeMap<PlatformAgentId, Rider>,
    waits: BTreeMap<LinkId, f32>,
}

impl FramedMixedProfileCrowdWorld {
    /// Attaches an empty simulation for the frame's selected profiles.
    /// Attached simulations use the same fixed time step.
    pub fn attach(
        &mut self,
        frames: &FrameSet,
        frame: FrameId,
        profiles: &[ProfileId],
        params: &CrowdWorldParams,
        limits: Limits,
    ) -> Result<(), ProfileError> {
        if self.crowds.contains_key(&frame) {
            return Err(Error::InvalidParameter("occupied crowd frame").into());
        }
        self.check_time_step(params.fixed_dt)?;
        let crowd =
            MixedProfileCrowdWorld::new(frame_meshes(frames, frame)?, profiles, params, limits)?;
        self.crowds.insert(frame, crowd);
        Ok(())
    }

    fn check_time_step(&self, dt: f32) -> Result<(), ProfileError> {
        if self
            .crowds
            .values()
            .next()
            .is_some_and(|old| old.crowd.params().fixed_dt != dt)
        {
            return Err(Error::InvalidParameter("crowd frame time step mismatch").into());
        }
        Ok(())
    }

    /// Returns the simulation in this frame for inspecting its state.
    pub fn crowd(&self, frame: FrameId) -> Option<&MixedProfileCrowdWorld> {
        self.crowds.get(&frame)
    }

    /// Configures a filter slot used in every attached frame.
    pub fn set_filter(&mut self, index: usize, filter: DefaultFilter) -> Result<(), ProfileError> {
        for crowd in self.crowds.values_mut() {
            crowd.set_filter(index, filter.clone())?;
        }
        Ok(())
    }

    /// Admits a rider to passable ground on its selected profile mesh.
    pub fn add_agent(
        &mut self,
        frames: &FrameSet,
        id: PlatformAgentId,
        profile: ProfileId,
        at: FramePoint,
        params: AgentParams,
    ) -> Result<(), ProfileError> {
        if self.riders.contains_key(&id) {
            return Err(Error::InvalidParameter("duplicate platform agent id").into());
        }
        let crowd = self.crowd_mut(at.frame)?;
        let mesh = crowd.mesh(frame_meshes(frames, at.frame)?, profile)?;
        check_envelope(mesh, params)?;
        let filter = crowd
            .crowd
            .filter(params.query_filter_type)
            .ok_or(Error::InvalidParameter("filter index"))?;
        let (poly, pos, _) = crate::detour::NavMeshQuery::new(&mesh.nav, 1)?.find_nearest_poly(
            &at.position,
            &crowd.crowd.query_half_extents(),
            filter,
        )?;
        if poly == 0 {
            return Err(Error::InvalidRef.into());
        }
        let handle = crowd
            .crowd
            .add_profile_landing(profile.0, poly, pos, params)?;
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

    /// Removes a rider from its current frame.
    pub fn remove_agent(&mut self, id: PlatformAgentId) -> Result<(), ProfileError> {
        let rider = self.rider(id)?;
        self.crowd_mut(rider.frame)?
            .crowd
            .remove_agent(rider.handle)?;
        self.riders.remove(&id);
        Ok(())
    }

    /// Reads the rider's frame, permanent profile and current frame-local state.
    pub fn agent(&self, id: PlatformAgentId) -> Option<(FrameId, ProfileId, AgentView<'_>)> {
        let r = self.riders.get(&id)?;
        let w = &self.crowds.get(&r.frame)?.crowd;
        Some((
            r.frame,
            ProfileId::new(w.agent_profile(r.handle)?),
            w.agent(r.handle)?,
        ))
    }

    /// Reads a rider's position transformed by its frame's current pose.
    pub fn world_position(&self, frames: &FrameSet, id: PlatformAgentId) -> Option<[f32; 3]> {
        let (frame, _, agent) = self.agent(id)?;
        frames.to_world(FramePoint::new(frame, agent.position()))
    }

    /// Sets a frame-local or cross-frame destination for this rider's profile.
    /// A goal with no available route is kept for retry.
    pub fn set_goal(
        &mut self,
        frames: &FrameSet,
        id: PlatformAgentId,
        target: Option<FramePoint>,
    ) -> Result<(), ProfileError> {
        let (_, profile, view) = self.agent(id).ok_or(Error::InvalidRef)?;
        if let Some(target) = target {
            let mesh = self
                .crowds
                .get(&target.frame)
                .ok_or(Error::InvalidRef)?
                .mesh(frame_meshes(frames, target.frame)?, profile)?;
            check_envelope(mesh, *view.params())?;
            if target.position.iter().any(|x| !x.is_finite()) {
                return Err(Error::InvalidParameter("non-finite frame point").into());
            }
        }
        self.riders.get_mut(&id).ok_or(Error::InvalidRef)?.goal = target;
        Ok(())
    }

    /// Offers or withdraws a caller-estimated wait cost for a disabled dock.
    pub fn set_wait_cost(
        &mut self,
        frames: &FrameSet,
        id: LinkId,
        cost: Option<f32>,
    ) -> Result<(), ProfileError> {
        if frames.dock(id).is_none() || cost.is_some_and(|v| !v.is_finite() || v < 0.) {
            return Err(Error::InvalidParameter("dock wait cost").into());
        }
        match cost {
            Some(cost) => {
                self.waits.insert(id, cost);
            }
            None => {
                self.waits.remove(&id);
            }
        }
        Ok(())
    }

    /// Lists external traversal requests with stable rider, frame and profile IDs.
    pub fn off_mesh_arrivals(
        &self,
    ) -> impl Iterator<Item = (PlatformAgentId, FrameId, ProfileId, OffMeshReached)> + '_ {
        self.riders.keys().filter_map(|&id| {
            let (frame, profile, agent) = self.agent(id)?;
            Some((id, frame, profile, agent.detail()?.off_mesh_reached()?))
        })
    }

    /// Completes external traversal on the rider's current frame and profile mesh.
    pub fn complete_off_mesh(
        &mut self,
        frames: &FrameSet,
        id: PlatformAgentId,
        landing: [f32; 3],
    ) -> Result<(), ProfileError> {
        let r = self.rider(id)?;
        let crowd = self.crowd_mut(r.frame)?;
        crowd.complete_off_mesh(
            frame_meshes(frames, r.frame)?,
            &crowd.scoped(r.handle),
            landing,
        )
    }

    /// Cancels external traversal from the rider's current departure point.
    pub fn abort_off_mesh(&mut self, id: PlatformAgentId) -> Result<(), ProfileError> {
        let r = self.rider(id)?;
        Ok(self.crowd_mut(r.frame)?.crowd.abort_off_mesh(r.handle)?)
    }

    /// Plans routes through profile-local meshes, advances all occupied frames,
    /// then applies aligned dock crossings in stable rider order. Budget
    /// exhaustion leaves the agents where they were before the tick.
    pub fn step(
        &mut self,
        frames: &FrameSet,
        budget: &mut Budget,
    ) -> Result<Budgeted<()>, ProfileError> {
        let mut navs = BTreeMap::new();
        let mut work = 0u64;
        for (&frame, crowd) in &self.crowds {
            navs.insert(frame, crowd.navs(frame_meshes(frames, frame)?)?);
            work = work.saturating_add(
                (crowd.crowd.len() as u64).saturating_mul(STEP_WORK_PER_AGENT)
                    + crowd.crowd.params().path_queue.iterations as u64,
            );
        }
        if !budget.spend(work) {
            return Ok(Budgeted::Exhausted(()));
        }
        let Budgeted::Complete(intents) = self.intents(frames, budget)? else {
            return Ok(Budgeted::Exhausted(()));
        };
        let mut seams = Vec::new();
        if self
            .crowds
            .values()
            .filter(|w| !w.crowd.is_empty())
            .take(2)
            .count()
            == 2
        {
            let profiles: BTreeSet<_> =
                self.crowds.values().flat_map(|w| w.owners.keys()).collect();
            for &profile in profiles {
                let routing = FrameRouting::for_profile(frames, ProfileId::new(profile));
                let Budgeted::Complete(active) = routing.active_seams(budget)? else {
                    return Ok(Budgeted::Exhausted(()));
                };
                seams.extend(active);
            }
        }
        let Budgeted::Complete(obstacles) = collect_obstacles(
            frames,
            seams,
            |frame| self.crowds.get(&frame).map(|w| &w.crowd),
            budget,
        )?
        else {
            return Ok(Budgeted::Exhausted(()));
        };
        let mut targets: BTreeMap<FrameId, BTreeMap<usize, LocalTarget>> = BTreeMap::new();
        for intent in &intents {
            let r = intent.rider;
            let crowd = self.crowd_mut(r.frame)?;
            let nav = navs
                .get(&r.frame)
                .and_then(|m| m.get(&intent.profile.0))
                .ok_or(Error::InvalidRef)?;
            let view = crowd.crowd.agent(r.handle).ok_or(Error::InvalidRef)?;
            let a = view.detail().ok_or(Error::InvalidRef)?;
            if !same_target(a, intent.target) {
                let goal = intent
                    .target
                    .map_or(Goal::None, |(poly, pos)| Goal::Point { poly, pos });
                crowd.crowd.set_goal(nav, r.handle, goal)?;
            }
            if let Some((_, point)) = intent.target {
                let target = if let Some(dock) = &intent.dock {
                    LocalTarget::Dock(
                        frames
                            .dock_open(dock.id, dock.departure, dock.landing.1)?
                            .then_some(dock.departure.position),
                    )
                } else {
                    LocalTarget::Goal(point)
                };
                targets
                    .entry(r.frame)
                    .or_default()
                    .insert(r.handle.index() as usize, target);
            }
        }
        let (empty_targets, empty_obstacles) = (BTreeMap::new(), BTreeMap::new());
        for (frame, meshes) in &navs {
            let crowd = self.crowd_mut(*frame)?;
            crowd.crowd.step_mixed(
                meshes,
                &mut Budget::unlimited(),
                targets.get(frame).unwrap_or(&empty_targets),
                obstacles.get(frame).unwrap_or(&empty_obstacles),
            )?;
        }
        for intent in intents {
            self.finish_intent(frames, intent)?;
        }
        Ok(Budgeted::Complete(()))
    }

    fn intents(
        &self,
        frames: &FrameSet,
        budget: &mut Budget,
    ) -> Result<Budgeted<Vec<Intent>>, ProfileError> {
        let mut intents = Vec::new();
        intents
            .try_reserve_exact(self.riders.len())
            .map_err(|_| Error::AllocationFailed)?;
        for (&id, &rider) in &self.riders {
            if !budget.spend(1) {
                return Ok(Budgeted::Exhausted(Vec::new()));
            }
            let (_, profile, view) = self.agent(id).ok_or(Error::InvalidRef)?;
            let a = view.detail().ok_or(Error::InvalidRef)?;
            if a.state == AgentState::Offmesh {
                continue;
            }
            let mut intent = Intent {
                id,
                rider,
                profile,
                target: None,
                dock: None,
            };
            if let Some(goal) = rider.goal {
                let crowd = &self
                    .crowds
                    .get(&rider.frame)
                    .ok_or(Error::InvalidRef)?
                    .crowd;
                let filter = crowd
                    .filter(a.params.query_filter_type)
                    .ok_or(Error::InvalidRef)?;
                let at = FramePoint::new(rider.frame, a.npos);
                let Budgeted::Complete(plan) = FrameRouting::for_profile(frames, profile)
                    .plan_from(
                        (at, Some(a.corridor.first_poly())),
                        goal,
                        crowd.query_half_extents(),
                        filter,
                        budget,
                        |dock, enabled| {
                            if ![dock.a.frame, dock.b.frame].iter().all(|id| {
                                self.crowds.get(id).is_some_and(|w| {
                                    w.crowd.filter(a.params.query_filter_type) == Some(filter)
                                        && frames.profile_meshes(*id).is_some_and(|meshes| {
                                            w.mesh(meshes, profile).is_ok_and(|mesh| {
                                                check_envelope(mesh, a.params).is_ok()
                                            })
                                        })
                                })
                            }) {
                                return None;
                            }
                            if enabled {
                                Some(0.)
                            } else {
                                self.waits.get(&dock.id).copied()
                            }
                        },
                    )?
                else {
                    return Ok(Budgeted::Exhausted(Vec::new()));
                };
                if let Some(plan) = plan {
                    if let Some(leg) = plan.legs.first() {
                        intent.target = leg.path.last().map(|&poly| (poly, leg.end));
                    }
                    intent.dock = first_dock(&plan.legs);
                }
            }
            intents.push(intent);
        }
        Ok(Budgeted::Complete(intents))
    }

    fn finish_intent(&mut self, frames: &FrameSet, intent: Intent) -> Result<(), ProfileError> {
        let Some((frame, profile, view)) = self.agent(intent.id) else {
            self.riders.remove(&intent.id);
            return Ok(());
        };
        let Some(dock) = intent.dock else {
            if intent.target.is_some() && view.arrived() {
                self.riders
                    .get_mut(&intent.id)
                    .ok_or(Error::InvalidRef)?
                    .goal = None;
            }
            return Ok(());
        };
        let a = view.detail().ok_or(Error::InvalidRef)?;
        if a.state != AgentState::Walking {
            return Ok(());
        }
        let params = a.params;
        let filter = self
            .crowds
            .get(&frame)
            .ok_or(Error::InvalidRef)?
            .crowd
            .filter(params.query_filter_type)
            .ok_or(Error::InvalidRef)?;
        let at = FramePoint::new(frame, a.npos);
        let Some((poly, landing)) = FrameRouting::for_profile(frames, profile).cross_dock(
            dock.id,
            (a.corridor.first_poly(), at),
            dock.departure,
            dock.landing,
            filter,
        )?
        else {
            return Ok(());
        };
        let dest = self.crowd_mut(landing.frame)?;
        check_envelope(
            dest.mesh(frame_meshes(frames, landing.frame)?, profile)?,
            params,
        )?;
        let handle = match dest
            .crowd
            .add_profile_landing(profile.0, poly, landing.position, params)
        {
            Ok(handle) => handle,
            Err(Error::BufferTooSmall) => return Ok(()),
            Err(e) => return Err(e.into()),
        };
        if let Err(error) = self
            .crowd_mut(frame)?
            .crowd
            .remove_agent(intent.rider.handle)
        {
            self.crowd_mut(landing.frame)?.crowd.remove_agent(handle)?;
            return Err(error.into());
        }
        self.riders.insert(
            intent.id,
            Rider {
                frame: landing.frame,
                handle,
                goal: intent.rider.goal,
            },
        );
        Ok(())
    }

    fn rider(&self, id: PlatformAgentId) -> Result<Rider, ProfileError> {
        self.riders
            .get(&id)
            .copied()
            .ok_or_else(|| Error::InvalidRef.into())
    }

    fn crowd_mut(&mut self, id: FrameId) -> Result<&mut MixedProfileCrowdWorld, ProfileError> {
        self.crowds
            .get_mut(&id)
            .ok_or_else(|| Error::InvalidParameter("missing crowd frame").into())
    }
}

fn frame_meshes(frames: &FrameSet, frame: FrameId) -> Result<&ProfileMeshes, ProfileError> {
    frames
        .profile_meshes(frame)
        .ok_or_else(|| Error::InvalidParameter("missing profile frame").into())
}
