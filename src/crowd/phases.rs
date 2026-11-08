//! Per-agent simulation phases shared by [`Crowd`] and the crowd world.
//!
//! Each function is one phase's arithmetic for one agent, moved verbatim out of the
//! upstream-ordered update so both managers evaluate identical expressions in
//! identical order. A phase that reads other agents does so through
//! [`AgentsView`], which both managers implement over state that the phase does not
//! write, so the per-agent calls of a phase are independent of each other.
use super::*;
use AgentState as S;
use TargetState as T;

/// Read access to the agent state other agents' phases consult: position, actual and
/// desired velocity, and the dimensions avoidance needs. Indexed by agent index.
pub(super) trait AgentsView {
    fn npos(&self, j: usize) -> V3;
    fn vel(&self, j: usize) -> V3;
    fn dvel(&self, j: usize) -> V3;
    fn radius(&self, j: usize) -> f32;
    fn max_speed(&self, j: usize) -> f32;
    fn orca(&self, j: usize) -> OrcaAgent {
        let (p, v, d) = (self.npos(j), self.vel(j), self.dvel(j));
        OrcaAgent {
            position: [p[0], p[2]],
            velocity: [v[0], v[2]],
            preferred_velocity: [d[0], d[2]],
            radius: self.radius(j),
            max_speed: self.max_speed(j),
        }
    }
}

impl AgentsView for [CrowdAgent] {
    fn npos(&self, j: usize) -> V3 {
        self[j].npos
    }
    fn vel(&self, j: usize) -> V3 {
        self[j].vel
    }
    fn dvel(&self, j: usize) -> V3 {
        self[j].dvel
    }
    fn radius(&self, j: usize) -> f32 {
        self[j].params.radius
    }
    fn max_speed(&self, j: usize) -> f32 {
        self[j].params.max_speed
    }
    fn orca(&self, j: usize) -> OrcaAgent {
        OrcaAgent::crowd(&self[j])
    }
}

/// Agent storage the serial phases walk by agent index.
pub(super) trait AgentSlots {
    /// One past the largest agent index.
    fn count(&self) -> usize;
    fn slot(&self, i: usize) -> Option<&CrowdAgent>;
    fn slot_mut(&mut self, i: usize) -> Option<&mut CrowdAgent>;
}

impl AgentSlots for [CrowdAgent] {
    fn count(&self) -> usize {
        self.len()
    }
    fn slot(&self, i: usize) -> Option<&CrowdAgent> {
        self.get(i)
    }
    fn slot_mut(&mut self, i: usize) -> Option<&mut CrowdAgent> {
        self.get_mut(i)
    }
}

/// Cuts an agent's corridor at its first polygon that is not resident and moves
/// the corridor's end to the point nearest the target on the last resident one.
fn trim_to_resident(a: &mut CrowdAgent, q: &NavMeshQuery<'_>) -> Result<(), Error> {
    let nav = q.nav();
    let path = a.corridor.path();
    let Some(keep) = path.iter().position(|&p| !nav.is_valid_poly_ref(p)) else {
        return Ok(());
    };
    if keep == 0 {
        let first = a.corridor.first_poly();
        return a.corridor.reset(first, a.npos);
    }
    let prefix: Vec<PolyRef> = path[..keep].to_vec();
    let target = q
        .closest_point_on_poly(prefix[keep - 1], &a.target_pos)
        .map_or(a.npos, |c| c.0);
    a.corridor.set_corridor(target, &prefix)?;
    a.boundary.reset();
    Ok(())
}

/// Path validity checks and replanning decisions for one agent.
pub(super) fn check_path(
    a: &mut CrowdAgent,
    q: &NavMeshQuery<'_>,
    filters: &[DefaultFilter; 16],
    extents: &V3,
    dt: f32,
) -> Result<(), Error> {
    if a.state != S::Walking {
        return Ok(());
    }
    a.replan_time += dt;
    let f = &filters[a.params.query_filter_type];
    let mut replan = false;
    let mut r = a.corridor.first_poly();
    let mut pos = a.npos;
    if !q.is_valid_poly_ref(r, f) {
        let (nr, np, _) = q.find_nearest_poly(&a.npos, extents, f)?;
        r = nr;
        if r != 0 {
            pos = np;
        }
        if r == 0 {
            a.corridor.reset(0, pos)?;
            a.partial = false;
            a.boundary.reset();
            a.state = S::Invalid;
            return Ok(());
        }
        a.corridor.fix_path_start(r, pos)?;
        a.boundary.reset();
        a.npos = pos;
        replan = true;
    }
    if matches!(a.target_state, T::None | T::Velocity) {
        return Ok(());
    }
    // Streaming (not in recastnavigation): a target whose tile is unloaded
    // is waited for, not dropped.
    if a.target_state != T::Failed {
        if let Some(k) = a
            .waiting_tile
            .filter(|_| a.target_state == T::WaitingForTile)
        {
            if q.nav().is_column_unloaded(k.x, k.y) {
                return trim_to_resident(a, q);
            }
        }
        if let Some(k) = a
            .corridor
            .path()
            .iter()
            .find_map(|&r| match q.nav().ref_state(r) {
                RefState::Unloaded(k) => Some(k),
                _ => None,
            })
        {
            a.target_state = T::WaitingForTile;
            a.waiting_tile = Some(k);
            a.pathq = 0;
            a.replan = false;
            return trim_to_resident(a, q);
        }
        match q.nav().ref_state(a.target_ref) {
            RefState::Unloaded(k) => {
                if a.target_state != T::WaitingForTile {
                    a.target_state = T::WaitingForTile;
                    a.waiting_tile = Some(k);
                    a.pathq = 0;
                    a.replan = false;
                }
                return trim_to_resident(a, q);
            }
            _ if a.target_state == T::WaitingForTile => {
                // Back (or replaced): plan again; a replaced target is
                // relocated by the invalid-target branch below.
                a.target_state = T::Requesting;
                replan = true;
            }
            _ => {}
        }
    }
    if !matches!(a.target_state, T::None | T::Failed) {
        if !q.is_valid_poly_ref(a.target_ref, f) {
            let (nr, np, _) = q.find_nearest_poly(&a.target_pos, extents, f)?;
            a.target_ref = nr;
            if nr != 0 {
                a.target_pos = np;
            }
            replan = true;
        }
        if a.target_ref == 0 {
            a.corridor.reset(r, pos)?;
            a.partial = false;
            a.target_state = T::None;
        }
    }
    if !a.corridor.is_valid(10, q, f) {
        replan = true;
    }
    if a.target_state == T::Valid
        && a.replan_time > 1.
        && a.corridor.path.len() < 10
        && a.corridor.last_poly() != a.target_ref
    {
        replan = true;
    }
    if replan && a.target_state != T::None {
        a.pathq = 0;
        a.replan = true;
        a.target_state = if a.target_ref != 0 {
            T::Requesting
        } else {
            T::Failed
        };
    }
    Ok(())
}

/// Quick searches, path-queue requests in upstream priority order, one queue update
/// of `max_iters` node expansions, and collection of finished results.
pub(super) fn move_requests<A: AgentSlots + ?Sized>(
    agents: &mut A,
    pathq: &mut PathQueue,
    filters: &[DefaultFilter; 16],
    ws: &mut Workspace,
    nav: &NavMesh,
    q: &mut NavMeshQuery<'_>,
    max_iters: usize,
) -> Result<(), Error> {
    let replan_time = |agents: &A, i: usize| agents.slot(i).map_or(0., |a| a.replan_time);
    let mut queue = [0usize; 8];
    let mut nqueue = 0;
    for i in 0..agents.count() {
        let Some(a) = agents.slot_mut(i) else {
            continue;
        };
        if !a.active || a.state == S::Invalid || matches!(a.target_state, T::None | T::Velocity) {
            continue;
        }
        let f = &filters[a.params.query_filter_type];
        if a.target_state == T::Requesting {
            let mut res = Vec::new();
            let mut pos = a.target_pos;
            let mut waiting = None;
            let result = (|| {
                q.init_sliced_find_path(
                    a.corridor.first_poly(),
                    a.target_ref,
                    &a.npos,
                    &a.target_pos,
                    f,
                    0,
                )?;
                q.update_sliced_find_path(20, f)?;
                let status = if a.replan {
                    q.finalize_sliced_find_path_partial(f, &a.corridor.path, &mut res, 32)?
                } else {
                    q.finalize_sliced_find_path(f, &mut res, 32)?
                };
                if status.has_detail(Status::UNLOADED_BORDER) {
                    waiting = q.search_frontier().first().copied();
                }
                if let Some(&r) = res.last() {
                    if r != a.target_ref {
                        pos = q.closest_point_on_poly(r, &a.target_pos)?.0;
                    }
                }
                Ok::<(), Error>(())
            })();
            if result.is_err() || res.is_empty() {
                res.clear();
                res.push(a.corridor.first_poly());
                pos = a.npos;
            }
            a.corridor.set_corridor(pos, &res)?;
            a.boundary.reset();
            a.partial = false;
            if let Some(k) = waiting {
                a.target_state = T::WaitingForTile;
                a.waiting_tile = Some(k);
                a.pathq = 0;
                a.replan = false;
            } else if res.last() == Some(&a.target_ref) {
                a.target_state = T::Valid;
                a.replan_time = 0.;
            } else {
                a.target_state = T::WaitingForQueue;
            }
        }
        if a.target_state == T::WaitingForQueue {
            let time = a.replan_time;
            let at = if nqueue == 0 || time <= replan_time(agents, queue[nqueue - 1]) {
                nqueue
            } else {
                (0..nqueue)
                    .find(|&j| time >= replan_time(agents, queue[j]))
                    .unwrap_or(nqueue)
            };
            if at < 8 {
                let end = nqueue.min(7);
                queue.copy_within(at..end, at + 1);
                queue[at] = i;
                nqueue = (nqueue + 1).min(8);
            }
        }
    }
    for &i in &queue[..nqueue] {
        let Some(a) = agents.slot_mut(i) else {
            continue;
        };
        match pathq.request(
            nav,
            a.corridor.last_poly(),
            a.target_ref,
            a.corridor.target,
            a.target_pos,
            a.params.query_filter_type,
        ) {
            Ok(h) => {
                a.pathq = h;
                a.target_state = T::WaitingForPath;
            }
            // Capacity backpressure is observable in the agent's target state.
            Err(Error::BufferTooSmall) => {}
            Err(e) => return Err(e),
        }
    }
    pathq.update_in(ws, nav, max_iters, filters)?;
    for i in 0..agents.count() {
        let Some(a) = agents.slot_mut(i) else {
            continue;
        };
        if !a.active || a.target_state != T::WaitingForPath {
            continue;
        }
        let status = pathq.status(a.pathq);
        if status & Status::FAILURE != 0 {
            a.pathq = 0;
            a.target_state = if a.target_ref != 0 {
                T::Requesting
            } else {
                T::Failed
            };
            a.replan_time = 0.;
        } else if status & Status::SUCCESS != 0 {
            let mut res = Vec::new();
            let status = pathq.take_result(a.pathq, &mut res)?;
            a.partial = status.0 & Status::PARTIAL_RESULT != 0;
            let mut valid = !res.is_empty() && a.corridor.last_poly() == res[0];
            let npath = a.corridor.path.len();
            if valid {
                if npath > 1 {
                    res.truncate(256 - (npath - 1));
                    let mut merged = Vec::new();
                    merged
                        .try_reserve(256)
                        .map_err(|_| Error::AllocationFailed)?;
                    merged.extend_from_slice(&a.corridor.path[..npath - 1]);
                    merged.extend_from_slice(&res);
                    res = merged;
                    let mut j = 0isize;
                    while j < res.len() as isize {
                        if j >= 1
                            && j + 1 < res.len() as isize
                            && res[(j - 1) as usize] == res[(j + 1) as usize]
                        {
                            res.drain((j - 1) as usize..(j + 1) as usize);
                            j -= 2;
                        }
                        j += 1;
                    }
                }
                let mut target = a.target_pos;
                if res.last() != Some(&a.target_ref) {
                    match q.closest_point_on_poly(*res.last().ok_or(Error::Failed)?, &target) {
                        Ok((p, _)) => target = p,
                        Err(_) => valid = false,
                    }
                }
                if valid {
                    a.corridor.set_corridor(target, &res)?;
                    a.boundary.reset();
                    a.target_state = T::Valid;
                }
            }
            if !valid {
                a.target_state = T::Failed;
            }
            a.replan_time = 0.;
        }
    }
    Ok(())
}

/// Advances topology timers and optimises the corridor of the one agent that waited
/// longest (ties: lowest index), as upstream does once per tick.
pub(super) fn optimize_topology<A: AgentSlots + ?Sized>(
    agents: &mut A,
    active: &[usize],
    q: &mut NavMeshQuery<'_>,
    filters: &[DefaultFilter; 16],
    dt: f32,
) {
    let mut opt: Option<usize> = None;
    let mut best = 0.;
    for &i in active {
        let Some(a) = agents.slot_mut(i) else {
            continue;
        };
        if a.state != S::Walking
            || matches!(a.target_state, T::None | T::Velocity)
            || a.params.update_flags & OPTIMIZE_TOPO == 0
        {
            continue;
        }
        a.topology_time += dt;
        let time = a.topology_time;
        if time >= 0.5 && (opt.is_none() || time > best) {
            opt = Some(i);
            best = time;
        }
    }
    if let Some(a) = opt.and_then(|i| agents.slot_mut(i)) {
        let _ = a
            .corridor
            .optimize_topology(q, &filters[a.params.query_filter_type]);
        a.topology_time = 0.;
    }
}

/// Refreshes the local walls when the agent moved a quarter of its query range or
/// one of the cached polygons became invalid.
pub(super) fn update_boundary(
    a: &mut CrowdAgent,
    q: &mut NavMeshQuery<'_>,
    f: &DefaultFilter,
) -> Result<(), Error> {
    let range = a.params.collision_query_range;
    if vdist_2d_sqr(&a.npos, &a.boundary.center) > sqr(range * 0.25) || !a.boundary.is_valid(q, f) {
        a.boundary
            .update(a.corridor.first_poly(), a.npos, range, q, f)?;
    }
    Ok(())
}

/// Sizes the agent's neighbour list for this tick and empties it; returns the cap.
pub(super) fn prepare_neighbours(
    ws: &mut Workspace,
    a: &mut CrowdAgent,
    active: usize,
) -> Result<usize, Error> {
    let neighbour_cap = match a.params.avoidance {
        AvoidanceMode::Orca(p) => usize::from(p.max_neighbours).min(active.saturating_sub(1)),
        _ => 6,
    };
    if a.neis.len() < neighbour_cap {
        let additional = neighbour_cap - a.neis.len();
        ws.reserve(&mut a.neis, additional)?;
        a.neis.resize(neighbour_cap, (0, 0.));
    }
    a.nneis = 0;
    Ok(neighbour_cap)
}

/// The planar squared distance to a candidate neighbour, or `None` when it is out
/// of the vertical overlap or beyond `range`.
pub(super) fn neighbour_distance(
    pos: &V3,
    height: f32,
    range: f32,
    nei_pos: &V3,
    nei_height: f32,
) -> Option<f32> {
    let mut diff = vsub(pos, nei_pos);
    if diff[1].abs() >= (height + nei_height) / 2. {
        return None;
    }
    diff[1] = 0.;
    let dist = vlen_sqr(&diff);
    if dist > sqr(range) {
        return None;
    }
    Some(dist)
}

/// Inserts candidate `j` at squared distance `dist` into the agent's sorted
/// neighbour list: by `(distance, index)` when `nearest`, else in upstream order.
pub(super) fn insert_neighbour(
    a: &mut CrowdAgent,
    nearest: bool,
    neighbour_cap: usize,
    j: usize,
    dist: f32,
) {
    let at = if nearest {
        // Candidates arrive in grid order: order by (distance, index).
        let kept = &a.neis[..a.nneis];
        if kept
            .last()
            .is_some_and(|&(index, distance)| (distance, index) <= (dist, j))
        {
            a.nneis
        } else {
            kept.partition_point(|&(index, distance)| (distance, index) < (dist, j))
        }
    } else {
        insert_pos(&a.neis[..a.nneis], dist, |v| v.1)
    };
    if at < neighbour_cap {
        let end = a.nneis.min(neighbour_cap - 1);
        a.neis.copy_within(at..end, at + 1);
        a.neis[at] = (j, dist);
        a.nneis = (a.nneis + 1).min(neighbour_cap);
    }
}

/// Steering corners and the visibility shortcut.
pub(super) fn find_corners(
    a: &mut CrowdAgent,
    q: &mut NavMeshQuery<'_>,
    filters: &[DefaultFilter; 16],
) -> Result<(), Error> {
    if a.state != S::Walking || matches!(a.target_state, T::None | T::Velocity) {
        return Ok(());
    }
    let f = &filters[a.params.query_filter_type];
    a.corridor.find_corners(q, &mut a.corners, 4)?;
    if a.params.update_flags & OPTIMIZE_VIS != 0 && !a.corners.is_empty() {
        let next = a.corners[1.min(a.corners.len() - 1)].pos;
        a.corridor
            .optimize_visibility(next, a.params.path_optimization_range, q, f)?;
    }
    Ok(())
}

/// Starts the off-mesh animation when the agent reached a connection's start.
pub(super) fn trigger_offmesh(a: &mut CrowdAgent, q: &NavMeshQuery<'_>) {
    if a.state != S::Walking || matches!(a.target_state, T::None | T::Velocity) {
        return;
    }
    if let Some(c) = a.corners.last() {
        if c.flags & STRAIGHTPATH_OFFMESH_CONNECTION != 0
            && vdist_2d_sqr(&a.npos, &c.pos) < sqr(a.params.radius * 2.25)
        {
            if let Ok((start, end, _)) = a.corridor.move_over_offmesh_connection(c.r#ref, q) {
                a.anim = Animation {
                    active: true,
                    init: a.npos,
                    start,
                    end,
                    t: 0.,
                    tmax: (vdist_2d(&start, &end) / a.params.max_speed) * 0.5,
                };
                a.state = S::Offmesh;
                a.corners.clear();
                a.nneis = 0;
            }
        }
    }
}

/// Desired velocity and speed from corners (or the requested velocity) plus
/// separation; `None` when the agent does not steer this tick.
pub(super) fn steer<V: AgentsView + ?Sized>(a: &CrowdAgent, others: &V) -> Option<(V3, f32)> {
    if a.state != S::Walking || a.target_state == T::None {
        return None;
    }
    let mut dvel = [0.; 3];
    let speed;
    if a.target_state == T::Velocity {
        dvel = a.target_pos;
        speed = vlen(&a.target_pos);
    } else {
        if !a.corners.is_empty() {
            let mut dir0 = vsub(&a.corners[0].pos, &a.npos);
            dir0[1] = 0.;
            if a.params.update_flags & ANTICIPATE_TURNS != 0 {
                let mut dir1 = vsub(&a.corners[1.min(a.corners.len() - 1)].pos, &a.npos);
                dir1[1] = 0.;
                let len0 = vlen(&dir0);
                let len1 = vlen(&dir1);
                if len1 > 0.001 {
                    dir1 = vscale(&dir1, 1. / len1);
                }
                dvel = [
                    dir0[0] - dir1[0] * len0 * 0.5,
                    0.,
                    dir0[2] - dir1[2] * len0 * 0.5,
                ];
            } else {
                dvel = dir0;
            }
            vnormalize(&mut dvel);
        }
        let slow = a.params.radius * 2.;
        let dist = if let Some(c) = a.corners.last().filter(|c| c.flags & STRAIGHTPATH_END != 0) {
            fmin(vdist_2d(&a.npos, &c.pos), slow)
        } else {
            slow
        };
        speed = a.params.max_speed;
        dvel = vscale(&dvel, speed * (dist / slow));
    }
    if a.params.update_flags & SEPARATION != 0 {
        let range = a.params.collision_query_range;
        let inv = 1. / range;
        let mut w = 0.;
        let mut disp = [0.; 3];
        for &(j, _) in &a.neis[..a.nneis] {
            let mut diff = vsub(&a.npos, &others.npos(j));
            diff[1] = 0.;
            let ds = vlen_sqr(&diff);
            if ds < 0.00001 || ds > sqr(range) {
                continue;
            }
            let dist = sqrtf(ds);
            let weight = a.params.separation_weight * (1. - sqr(dist * inv));
            disp = vmad(&disp, &diff, weight / dist);
            w += 1.;
        }
        if w > 0.0001 {
            dvel = vmad(&dvel, &disp, 1. / w);
            let ds = vlen_sqr(&dvel);
            let desired = sqr(speed);
            if ds > desired {
                dvel = vscale(&dvel, desired / ds);
            }
        }
    }
    Some((dvel, speed))
}

/// Scratch of the avoidance phase, reused across agents (one per worker).
pub(super) struct AvoidScratch<'a> {
    pub(super) query: &'a mut ObstacleAvoidanceQuery,
    pub(super) orca: &'a mut OrcaScratch,
    pub(super) orca_neighbours: &'a mut Vec<OrcaAgent>,
}

/// The avoidance velocity and the number of velocity samples taken; `None` when the
/// agent is not walking.
pub(super) fn avoid<V: AgentsView + ?Sized>(
    ws: &mut Workspace,
    a: &CrowdAgent,
    others: &V,
    dt: f32,
    presets: &[ObstacleAvoidanceParams; 8],
    s: &mut AvoidScratch<'_>,
) -> Result<Option<(V3, usize)>, Error> {
    if a.state != S::Walking {
        return Ok(None);
    }
    let mut samples = 0;
    let nvel = if a.params.update_flags & OBSTACLE_AVOIDANCE != 0
        && dt > 0.
        && let AvoidanceMode::Orca(p) = a.params.avoidance
    {
        s.orca_neighbours.clear();
        ws.reserve(s.orca_neighbours, a.nneis)?;
        for &(j, _) in &a.neis[..a.nneis] {
            s.orca_neighbours.push(others.orca(j));
        }
        let mut walls = [[0.; 4]; 8];
        for (out, &(_, seg)) in walls.iter_mut().zip(&a.boundary.segments) {
            *out = [seg[0], seg[2], seg[3], seg[5]];
        }
        let v = orca::velocity_in(
            ws,
            &OrcaAgent::crowd(a),
            s.orca_neighbours,
            &walls[..a.boundary.segments.len()],
            &p,
            dt,
            s.orca,
        )?;
        [v[0], 0., v[1]]
    } else if a.params.update_flags & OBSTACLE_AVOIDANCE != 0
        && matches!(a.params.avoidance, AvoidanceMode::Sampled)
    {
        s.query.reset();
        for &(j, _) in &a.neis[..a.nneis] {
            s.query.add_circle(
                others.npos(j),
                others.radius(j),
                others.vel(j),
                others.dvel(j),
            )?;
        }
        for &(_, seg) in &a.boundary.segments {
            if tri_area_2d(&a.npos, &seg[..3], &seg[3..]) < 0. {
                continue;
            }
            s.query.add_segment(v3(&seg[..3]), v3(&seg[3..]))?;
        }
        let (v, n) = s.query.sample_velocity_adaptive_in(
            ws,
            a.npos,
            a.params.radius,
            a.desired_speed,
            a.vel,
            a.dvel,
            &presets[a.params.obstacle_avoidance_type],
        )?;
        samples = n;
        v
    } else {
        a.dvel
    };
    Ok(Some((nvel, samples)))
}

/// Acceleration-limited velocity and the integrated position.
pub(super) fn integrate(a: &mut CrowdAgent, dt: f32) {
    if a.state != S::Walking {
        return;
    }
    let max = a.params.max_acceleration * dt;
    let mut dv = vsub(&a.nvel, &a.vel);
    let ds = vlen(&dv);
    if ds > max {
        dv = vscale(&dv, max / ds);
    }
    a.vel = vadd(&a.vel, &dv);
    if matches!(a.params.avoidance, AvoidanceMode::Orca(_)) {
        // Acceleration interpolation can round back outside the disc.
        let v = orca::clip([a.vel[0], a.vel[2]], a.params.max_speed);
        a.vel = [v[0], 0., v[1]];
    }
    if vlen(&a.vel) > 0.0001 {
        a.npos = vmad(&a.npos, &a.vel, dt);
    } else {
        a.vel = [0.; 3];
    }
}

/// One Jacobi collision-correction displacement for agent `i`; `None` when it is not
/// walking.
pub(super) fn collision_displacement<V: AgentsView + ?Sized>(
    i: usize,
    a: &CrowdAgent,
    others: &V,
) -> Option<V3> {
    if a.state != S::Walking {
        return None;
    }
    let mut disp = [0.; 3];
    let mut w = 0.;
    for &(j, _) in &a.neis[..a.nneis] {
        let nei_pos = others.npos(j);
        let nei_radius = others.radius(j);
        let mut diff = vsub(&a.npos, &nei_pos);
        diff[1] = 0.;
        let mut dist = vlen_sqr(&diff);
        if dist > sqr(a.params.radius + nei_radius) {
            continue;
        }
        dist = sqrtf(dist);
        let mut pen = (a.params.radius + nei_radius) - dist;
        if dist < 0.0001 {
            diff = if i > j {
                [-a.dvel[2], 0., a.dvel[0]]
            } else {
                [a.dvel[2], 0., -a.dvel[0]]
            };
            pen = 0.01;
        } else {
            pen = (1. / dist) * (pen * 0.5) * 0.7;
        }
        disp = vmad(&disp, &diff, pen);
        w += 1.;
    }
    if w > 0.0001 {
        disp = vscale(&disp, 1. / w);
    }
    Some(disp)
}

/// Applies the displacement stored by the last collision iteration.
pub(super) fn apply_displacement(a: &mut CrowdAgent) {
    if a.state == S::Walking {
        a.npos = vadd(&a.npos, &a.disp);
    }
}

/// Moves the corridor to the integrated position along the surface.
pub(super) fn move_on_mesh(
    a: &mut CrowdAgent,
    q: &mut NavMeshQuery<'_>,
    filters: &[DefaultFilter; 16],
) -> Result<(), Error> {
    if a.state != S::Walking {
        return Ok(());
    }
    // Upstream keeps the corridor position if the local move fails.
    let _ = a
        .corridor
        .move_position(a.npos, q, &filters[a.params.query_filter_type]);
    a.npos = a.corridor.pos;
    if matches!(a.target_state, T::None | T::Velocity) {
        a.corridor.reset(a.corridor.first_poly(), a.npos)?;
        a.partial = false;
    }
    Ok(())
}

/// Advances an off-mesh animation.
pub(super) fn animate(a: &mut CrowdAgent, dt: f32) {
    let anim = &mut a.anim;
    if !anim.active {
        return;
    }
    anim.t += dt;
    if anim.t > anim.tmax {
        anim.active = false;
        a.state = S::Walking;
        return;
    }
    let ta = anim.tmax * 0.15;
    let tb = anim.tmax;
    a.npos = if anim.t < ta {
        vlerp(&anim.init, &anim.start, fclamp(anim.t / ta, 0., 1.))
    } else {
        vlerp(
            &anim.start,
            &anim.end,
            fclamp((anim.t - ta) / (tb - ta), 0., 1.),
        )
    };
    a.vel = [0.; 3];
    a.dvel = [0.; 3];
}

/// Rejects a tick that produced a non-finite vector.
pub(super) fn check_finite(a: &CrowdAgent) -> Result<(), Error> {
    finite(a.npos)?;
    finite(a.vel)?;
    finite(a.nvel)?;
    finite(a.dvel)
}
