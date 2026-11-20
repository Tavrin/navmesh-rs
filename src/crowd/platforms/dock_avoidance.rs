use super::*;

type DockObstacles = BTreeMap<FrameId, BTreeMap<usize, Vec<OrcaAgent>>>;

#[derive(Clone, Copy)]
struct Nearby {
    index: usize,
    pos: V3,
    vel: V3,
    params: AgentParams,
}

#[derive(Clone, Copy)]
struct Candidate {
    key: (u64, FrameId, usize),
    obstacle: OrcaAgent,
}

// Dock discs are a separate allowance from native neighbours. Keep the closest
// six per recipient so a busy dock cannot grow each worker's avoidance scratch.
const CAP: usize = 6;

impl FramedCrowd<CrowdWorld> {
    pub(super) fn dock_obstacles(
        &self,
        frames: &FrameSet,
        budget: &mut Budget,
    ) -> Result<Budgeted<DockObstacles>, Error> {
        let mut nearby: BTreeMap<FrameId, BTreeMap<usize, Vec<Candidate>>> = BTreeMap::new();
        if self
            .crowds
            .values()
            .filter(|c| !c.is_empty())
            .take(2)
            .count()
            < 2
        {
            return Ok(Budgeted::Complete(BTreeMap::new()));
        }
        let Budgeted::Complete(seams) = frames.active_seams(budget)? else {
            return Ok(Budgeted::Exhausted(BTreeMap::new()));
        };
        for (a, b, radius) in seams {
            let (Some(ca), Some(cb)) = (self.crowds.get(&a.frame), self.crowds.get(&b.frame))
            else {
                continue;
            };
            if ca.is_empty() || cb.is_empty() {
                continue;
            }
            let Some(one) = occupants(ca, a.position, radius, budget) else {
                return Ok(Budgeted::Exhausted(BTreeMap::new()));
            };
            let Some(two) = occupants(cb, b.position, radius, budget) else {
                return Ok(Budgeted::Exhausted(BTreeMap::new()));
            };
            for (dest, from, here, there) in [
                (a.frame, b.frame, &one, &two),
                (b.frame, a.frame, &two, &one),
            ] {
                for remote in there {
                    if !budget.spend(1) {
                        return Ok(Budgeted::Exhausted(BTreeMap::new()));
                    }
                    let remote_pos = frames
                        .point_between(FramePoint::new(from, remote.pos), dest)
                        .ok_or(Error::InvalidParameter("dock obstacle position"))?;
                    let remote_vel = frames
                        .vector_between(from, dest, remote.vel)
                        .ok_or(Error::InvalidParameter("dock obstacle velocity"))?;
                    let obstacle = OrcaAgent {
                        position: [remote_pos[0], remote_pos[2]],
                        velocity: [remote_vel[0], remote_vel[2]],
                        preferred_velocity: [remote_vel[0], remote_vel[2]],
                        radius: remote.params.radius,
                        max_speed: remote.params.max_speed,
                    };
                    for local in here {
                        if !budget.spend(1) {
                            return Ok(Budgeted::Exhausted(BTreeMap::new()));
                        }
                        if local.params.update_flags & OBSTACLE_AVOIDANCE == 0
                            || (f64::from(local.pos[1]) - f64::from(remote_pos[1])).abs()
                                >= (f64::from(local.params.height)
                                    + f64::from(remote.params.height))
                                    * 0.5
                        {
                            continue;
                        }
                        let d = sq_dist(local.pos, remote_pos);
                        if d > f64::from(local.params.collision_query_range)
                            * f64::from(local.params.collision_query_range)
                        {
                            continue;
                        }
                        let list = nearby
                            .entry(dest)
                            .or_default()
                            .entry(local.index)
                            .or_default();
                        let key = (d.to_bits(), from, remote.index);
                        let i = list.partition_point(|c| c.key < key);
                        if i < CAP && list.get(i).is_none_or(|c| c.key != key) {
                            if list.len() == CAP {
                                list.pop();
                            }
                            list.insert(i, Candidate { key, obstacle });
                        }
                    }
                }
            }
        }
        let obstacles = nearby
            .into_iter()
            .map(|(id, agents)| {
                let agents = agents
                    .into_iter()
                    .map(|(i, list)| (i, list.into_iter().map(|c| c.obstacle).collect()))
                    .collect();
                (id, agents)
            })
            .collect();
        Ok(Budgeted::Complete(obstacles))
    }
}

fn occupants(
    crowd: &CrowdWorld,
    center: V3,
    radius: f32,
    budget: &mut Budget,
) -> Option<Vec<Nearby>> {
    let radius2 = f64::from(radius) * f64::from(radius);
    let mut out = Vec::new();
    for agent in crowd.agents() {
        if !budget.spend(1) {
            return None;
        }
        let pos = agent.position();
        if agent.state() == AgentState::Walking && sq_dist(pos, center) <= radius2 {
            out.push(Nearby {
                index: agent.handle().index() as usize,
                pos,
                vel: agent.velocity(),
                params: *agent.params(),
            });
        }
    }
    Some(out)
}

fn sq_dist(a: V3, b: V3) -> f64 {
    a.into_iter()
        .zip(b)
        .map(|(a, b)| {
            let d = f64::from(a) - f64::from(b);
            d * d
        })
        .sum()
}
