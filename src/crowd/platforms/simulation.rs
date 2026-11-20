use super::*;
use crate::{Budget, Budgeted};

struct Intent<H> {
    id: PlatformAgentId,
    rider: Rider<H>,
    target: Option<(PolyRef, V3)>,
    dock: Option<DockCrossing>,
}

pub(crate) struct DockCrossing {
    pub(crate) id: LinkId,
    pub(crate) departure: FramePoint,
    pub(crate) landing: (PolyRef, FramePoint),
}

pub(crate) fn first_dock(legs: &[crate::frames::FrameLeg]) -> Option<DockCrossing> {
    let [leg, next, ..] = legs else { return None };
    Some(DockCrossing {
        id: leg.dock?,
        departure: FramePoint::new(leg.frame, leg.end),
        landing: (*next.path.first()?, FramePoint::new(next.frame, next.start)),
    })
}

pub(crate) fn same_target(a: &CrowdAgent, target: Option<(PolyRef, V3)>) -> bool {
    match target {
        Some((poly, pos)) => {
            a.target_ref == poly
                && a.target_pos == pos
                && matches!(
                    a.target_state,
                    TargetState::Valid
                        | TargetState::Requesting
                        | TargetState::WaitingForQueue
                        | TargetState::WaitingForPath
                )
        }
        None => a.target_state == TargetState::None,
    }
}

impl<C: FrameCrowd> FramedCrowd<C> {
    fn intents(
        &self,
        frames: &FrameSet,
        budget: &mut Budget,
    ) -> Result<Budgeted<Vec<Intent<C::Handle>>>, Error> {
        let mut intents = Vec::new();
        intents
            .try_reserve_exact(self.riders.len())
            .map_err(|_| Error::AllocationFailed)?;
        for (&id, &rider) in &self.riders {
            if !budget.spend(1) {
                return Ok(Budgeted::Exhausted(Vec::new()));
            }
            let crowd = self.crowds.get(&rider.frame).ok_or(Error::InvalidRef)?;
            frames.require_mesh(rider.frame)?;
            let agent = crowd.get(rider.handle).ok_or(Error::InvalidRef)?;
            // A pending ordinary off-mesh traversal owns the current corridor.
            if agent.state == AgentState::Offmesh {
                continue;
            }

            let mut target = None;
            let mut dock = None;
            if let Some(goal) = rider.goal {
                let filter = crowd
                    .filter(agent.params.query_filter_type)
                    .ok_or(Error::InvalidRef)?;
                let at = FramePoint::new(rider.frame, agent.npos);
                let Budgeted::Complete(plan) = frames.plan_from(
                    (at, Some(agent.corridor.first_poly())),
                    goal,
                    crowd.extents(),
                    filter,
                    budget,
                    |spec, enabled| {
                        let compatible = [spec.a.frame, spec.b.frame].iter().all(|id| {
                            self.crowds
                                .get(id)
                                .and_then(|crowd| crowd.filter(agent.params.query_filter_type))
                                == Some(filter)
                        });
                        if !compatible {
                            return None;
                        }
                        if enabled {
                            Some(0.)
                        } else {
                            self.waits.get(&spec.id).copied()
                        }
                    },
                )?
                else {
                    return Ok(Budgeted::Exhausted(Vec::new()));
                };
                if let Some(plan) = plan {
                    if let Some(leg) = plan.legs.first() {
                        target = leg.path.last().map(|&poly| (poly, leg.end));
                    }
                    dock = first_dock(&plan.legs);
                }
            }
            intents.push(Intent {
                id,
                rider,
                target,
                dock,
            });
        }
        Ok(Budgeted::Complete(intents))
    }

    fn local_targets(
        frames: &FrameSet,
        intents: &[Intent<C::Handle>],
        index: impl Fn(C::Handle) -> usize,
    ) -> Result<BTreeMap<FrameId, BTreeMap<usize, LocalTarget>>, Error> {
        let mut targets: BTreeMap<_, BTreeMap<_, _>> = BTreeMap::new();
        for intent in intents {
            let Some((_, point)) = intent.target else {
                continue;
            };
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
                .entry(intent.rider.frame)
                .or_default()
                .insert(index(intent.rider.handle), target);
        }
        Ok(targets)
    }

    fn finish_arrivals(&mut self, intents: &[Intent<C::Handle>]) {
        for intent in intents {
            if intent.dock.is_none() && intent.target.is_some() {
                match self.agent(intent.id) {
                    None => {
                        self.riders.remove(&intent.id);
                    }
                    Some((_, _, agent)) if agent.arrived() => {
                        if let Some(rider) = self.riders.get_mut(&intent.id) {
                            rider.goal = None;
                        }
                    }
                    _ => {}
                }
            }
        }
    }

    fn apply_targets(
        &mut self,
        frames: &FrameSet,
        intents: &[Intent<C::Handle>],
    ) -> Result<(), Error> {
        for i in intents {
            let nav = frames.require_mesh(i.rider.frame)?;
            let crowd = self.require_crowd_mut(i.rider.frame)?;
            let a = crowd.get(i.rider.handle).ok_or(Error::InvalidRef)?;
            if !same_target(a, i.target) {
                crowd.target(nav, i.rider.handle, i.target)?;
            }
        }
        Ok(())
    }

    fn apply_transfers(
        &mut self,
        frames: &FrameSet,
        intents: &[Intent<C::Handle>],
    ) -> Result<(), Error> {
        // Compute intents before moving any agent. Transfer after all frames
        // have stepped so a crossing agent is simulated exactly once per tick.
        for i in intents {
            let Some(dock) = &i.dock else { continue };
            let (frame, handle, agent) = self.agent(i.id).ok_or(Error::InvalidRef)?;
            if agent.state != AgentState::Walking {
                continue;
            }
            let params = agent.params;
            let filter = self
                .crowds
                .get(&frame)
                .and_then(|c| c.filter(params.query_filter_type))
                .ok_or(Error::InvalidRef)?;
            let at = FramePoint::new(frame, agent.npos);
            let Some((poly, landing)) = frames.cross_dock(
                dock.id,
                (agent.corridor.first_poly(), at),
                dock.departure,
                dock.landing,
                filter,
            )?
            else {
                continue;
            };
            let dest = self
                .crowds
                .get(&landing.frame)
                .ok_or(Error::InvalidParameter("missing destination crowd"))?;
            if dest.filter(params.query_filter_type) != Some(filter) {
                return Err(Error::InvalidParameter("dock crowd filter mismatch"));
            }
            let new_handle = match self.require_crowd_mut(landing.frame)?.add_landed(
                poly,
                landing.position,
                params,
            ) {
                Ok(handle) => handle,
                Err(Error::BufferTooSmall) => continue,
                Err(e) => return Err(e),
            };
            if let Err(error) = self.require_crowd_mut(frame)?.remove(handle) {
                self.require_crowd_mut(landing.frame)?.remove(new_handle)?;
                return Err(error);
            }
            self.riders.insert(
                i.id,
                Rider {
                    frame: landing.frame,
                    handle: new_handle,
                    ..i.rider
                },
            );
        }
        Ok(())
    }
}

impl FramedCrowd<Crowd> {
    /// Advances every frame's `Crowd` by the same elapsed time. Routes use
    /// the workspace's work limit. Simulation uses it separately in each frame.
    /// An error after simulation starts may leave an incomplete tick.
    pub fn update(&mut self, frames: &FrameSet, ws: &mut Workspace, dt: f32) -> Result<(), Error> {
        nonnegative(dt)?;
        let mut budget = Budget::from_limits(&ws.limits());
        let Budgeted::Complete(intents) = self.intents(frames, &mut budget)? else {
            return Err(Error::LimitExceeded);
        };
        let targets = Self::local_targets(frames, &intents, |h| h.0)?;
        let empty = BTreeMap::new();
        self.apply_targets(frames, &intents)?;
        for (&id, crowd) in &mut self.crowds {
            if !sealed::Engine::is_empty(crowd) {
                crowd.update_with_docks(
                    ws,
                    frames.require_mesh(id)?,
                    dt,
                    targets.get(&id).unwrap_or(&empty),
                )?;
            }
        }
        self.finish_arrivals(&intents);
        self.apply_transfers(frames, &intents)
    }
}

#[cfg(feature = "crowd-world")]
impl FramedCrowd<CrowdWorld> {
    /// Advances all occupied frames through one fixed tick. The attached
    /// simulations must agree on the fixed time step. The work budget covers
    /// planning and the native crowd step charges. On exhaustion no agent
    /// has moved and the caller can retry with a larger budget.
    /// An error after simulation starts may leave an incomplete tick.
    pub fn step(&mut self, frames: &FrameSet, budget: &mut Budget) -> Result<Budgeted<()>, Error> {
        use super::super::world::STEP_WORK_PER_AGENT;

        let mut work = 0u64;
        for (&id, crowd) in &self.crowds {
            frames.require_mesh(id)?;
            let p = crowd.params();
            if !crowd.is_empty() {
                work = work
                    .saturating_add((crowd.len() as u64).saturating_mul(STEP_WORK_PER_AGENT))
                    .saturating_add(p.path_queue.iterations as u64);
            }
        }
        if !budget.spend(work) {
            return Ok(Budgeted::Exhausted(()));
        }
        let Budgeted::Complete(intents) = self.intents(frames, budget)? else {
            return Ok(Budgeted::Exhausted(()));
        };
        let Budgeted::Complete(obstacles) = self.dock_obstacles(frames, budget)? else {
            return Ok(Budgeted::Exhausted(()));
        };
        let targets = Self::local_targets(frames, &intents, |h| h.index() as usize)?;
        let (empty, empty_obstacles) = (BTreeMap::new(), BTreeMap::new());
        self.apply_targets(frames, &intents)?;
        for (&id, crowd) in &mut self.crowds {
            if !crowd.is_empty() {
                crowd.step_with_docks(
                    frames.require_mesh(id)?,
                    &mut Budget::unlimited(),
                    targets.get(&id).unwrap_or(&empty),
                    obstacles.get(&id).unwrap_or(&empty_obstacles),
                )?;
            }
        }
        self.finish_arrivals(&intents);
        self.apply_transfers(frames, &intents)?;
        Ok(Budgeted::Complete(()))
    }
}
