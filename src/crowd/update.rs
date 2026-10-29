//! Ordered simulation phases, translated from DetourCrowd.cpp; see NOTICE.
use super::*;
use AgentState as S;
use TargetState as T;
impl Crowd {
    fn check_paths(&mut self, q: &NavMeshQuery<'_>, dt: f32) -> Result<(), Error> {
        for &i in &self.active {
            let a = &mut self.agents[i];
            if a.state != S::Walking {
                continue;
            }
            a.replan_time += dt;
            let f = &self.filters[a.params.query_filter_type];
            let mut replan = false;
            let mut r = a.corridor.first_poly();
            let mut pos = a.npos;
            if !q.is_valid_poly_ref(r, f) {
                let (nr, np, _) = q.find_nearest_poly(&a.npos, &self.extents, f)?;
                r = nr;
                if r != 0 {
                    pos = np;
                }
                if r == 0 {
                    a.corridor.reset(0, pos)?;
                    a.partial = false;
                    a.boundary.reset();
                    a.state = S::Invalid;
                    continue;
                }
                a.corridor.fix_path_start(r, pos)?;
                a.boundary.reset();
                a.npos = pos;
                replan = true;
            }
            if matches!(a.target_state, T::None | T::Velocity) {
                continue;
            }
            if !matches!(a.target_state, T::None | T::Failed) {
                if !q.is_valid_poly_ref(a.target_ref, f) {
                    let (nr, np, _) = q.find_nearest_poly(&a.target_pos, &self.extents, f)?;
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
        }
        Ok(())
    }
    fn move_requests(
        &mut self,
        ws: &mut Workspace,
        nav: &NavMesh,
        q: &mut NavMeshQuery<'_>,
    ) -> Result<(), Error> {
        let mut queue = [0usize; 8];
        let mut nqueue = 0;
        for i in 0..self.agents.len() {
            let a = &mut self.agents[i];
            if !a.active || a.state == S::Invalid || matches!(a.target_state, T::None | T::Velocity)
            {
                continue;
            }
            let f = &self.filters[a.params.query_filter_type];
            if a.target_state == T::Requesting {
                let mut res = Vec::new();
                let mut pos = a.target_pos;
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
                    if a.replan {
                        q.finalize_sliced_find_path_partial(f, &a.corridor.path, &mut res, 32)?;
                    } else {
                        q.finalize_sliced_find_path(f, &mut res, 32)?;
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
                if res.last() == Some(&a.target_ref) {
                    a.target_state = T::Valid;
                    a.replan_time = 0.;
                } else {
                    a.target_state = T::WaitingForQueue;
                }
            }
            if a.target_state == T::WaitingForQueue {
                let time = a.replan_time;
                let at = if nqueue == 0 || time <= self.agents[queue[nqueue - 1]].replan_time {
                    nqueue
                } else {
                    (0..nqueue)
                        .find(|&j| time >= self.agents[queue[j]].replan_time)
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
            let a = &mut self.agents[i];
            if let Some(h) = self.pathq.request(
                a.corridor.last_poly(),
                a.target_ref,
                a.corridor.target,
                a.target_pos,
                a.params.query_filter_type,
            )? {
                a.pathq = h;
                a.target_state = T::WaitingForPath;
            }
        }
        self.pathq.update_in(ws, nav, 100, &self.filters)?;
        for a in &mut self.agents {
            if !a.active || a.target_state != T::WaitingForPath {
                continue;
            }
            let status = self.pathq.status(a.pathq);
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
                let status = self.pathq.take_result(a.pathq, &mut res)?;
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
    /// Advances the crowd in upstream phase order. `dt` must be finite and
    /// nonnegative. Workspace limits bound allocation and charged simulation work.
    /// An error can leave earlier phases applied; discard that tick's result.
    pub fn update(&mut self, ws: &mut Workspace, nav: &NavMesh, dt: f32) -> Result<(), Error> {
        ws.begin();
        nonnegative(dt)?;
        // Reserve/query scratch and conservatively charge per-agent local query
        // work before mutating the simulation. Sampling charges its own work.
        ws.work((self.agents.len() as u64).saturating_mul(4096))?;
        ws.charge_bytes::<u8>(
            self.agents
                .len()
                .checked_mul(32768)
                .and_then(|n| n.checked_add(512 * 128 + 65536))
                .ok_or(Error::SizeOverflow)?,
        )?;
        let mut q = NavMeshQuery::new(nav, 512)?;
        self.velocity_samples = 0;
        self.active.clear();
        self.active.extend(
            self.agents
                .iter()
                .enumerate()
                .filter_map(|(i, a)| a.active.then_some(i)),
        );
        self.check_paths(&q, dt)?;
        self.move_requests(ws, nav, &mut q)?;
        let mut opt: Option<usize> = None;
        for &i in &self.active {
            let a = &mut self.agents[i];
            if a.state != S::Walking
                || matches!(a.target_state, T::None | T::Velocity)
                || a.params.update_flags & OPTIMIZE_TOPO == 0
            {
                continue;
            }
            a.topology_time += dt;
            let time = a.topology_time;
            if time >= 0.5 && opt.is_none_or(|j| time > self.agents[j].topology_time) {
                opt = Some(i);
            }
        }
        if let Some(i) = opt {
            let a = &mut self.agents[i];
            let _ = a
                .corridor
                .optimize_topology(&mut q, &self.filters[a.params.query_filter_type]);
            a.topology_time = 0.;
        }
        self.grid.clear();
        for (j, &i) in self.active.iter().enumerate() {
            let a = &self.agents[i];
            let p = a.npos;
            let r = a.params.radius;
            self.grid
                .add_item(j as u16, [p[0] - r, p[2] - r, p[0] + r, p[2] + r])?;
        }
        for &i in &self.active {
            let a = &mut self.agents[i];
            if a.state != S::Walking {
                continue;
            }
            let f = &self.filters[a.params.query_filter_type];
            let range = a.params.collision_query_range;
            if vdist_2d_sqr(&a.npos, &a.boundary.center) > sqr(range * 0.25)
                || !a.boundary.is_valid(&q, f)
            {
                a.boundary
                    .update(a.corridor.first_poly(), a.npos, range, &mut q, f)?;
            }
            a.nneis = 0;
            let pos = a.npos;
            let height = a.params.height;
            let mut ids = [0u16; 32];
            ws.work(self.grid.cell_work([
                pos[0] - range,
                pos[2] - range,
                pos[0] + range,
                pos[2] + range,
            ])?)?;
            let n = self.grid.query_items(
                [
                    pos[0] - range,
                    pos[2] - range,
                    pos[0] + range,
                    pos[2] + range,
                ],
                &mut ids,
            )?;
            for &id in &ids[..n] {
                let j = self.active[id as usize];
                if i == j {
                    continue;
                }
                let nei = &self.agents[j];
                let mut diff = vsub(&pos, &nei.npos);
                if diff[1].abs() >= (height + nei.params.height) / 2. {
                    continue;
                }
                diff[1] = 0.;
                let dist = vlen_sqr(&diff);
                if dist > sqr(range) {
                    continue;
                }
                let a = &mut self.agents[i];
                let at = insert_pos(&a.neis[..a.nneis], dist, |v| v.1);
                if at < 6 {
                    let end = a.nneis.min(5);
                    a.neis.copy_within(at..end, at + 1);
                    a.neis[at] = (j, dist);
                    a.nneis = (a.nneis + 1).min(6);
                }
            }
        }
        for &i in &self.active {
            let a = &mut self.agents[i];
            if a.state != S::Walking || matches!(a.target_state, T::None | T::Velocity) {
                continue;
            }
            let f = &self.filters[a.params.query_filter_type];
            a.corridor.find_corners(&q, &mut a.corners, 4)?;
            if a.params.update_flags & OPTIMIZE_VIS != 0 && !a.corners.is_empty() {
                let next = a.corners[1.min(a.corners.len() - 1)].pos;
                a.corridor.optimize_visibility(
                    next,
                    a.params.path_optimization_range,
                    &mut q,
                    f,
                )?;
            }
        }
        for &i in &self.active {
            let a = &mut self.agents[i];
            if a.state != S::Walking || matches!(a.target_state, T::None | T::Velocity) {
                continue;
            }
            if let Some(c) = a.corners.last() {
                if c.flags & STRAIGHTPATH_OFFMESH_CONNECTION != 0
                    && vdist_2d_sqr(&a.npos, &c.pos) < sqr(a.params.radius * 2.25)
                {
                    if let Ok((start, end, _)) =
                        a.corridor.move_over_offmesh_connection(c.r#ref, &q)
                    {
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
        for &i in &self.active {
            let a = &self.agents[i];
            if a.state != S::Walking || a.target_state == T::None {
                continue;
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
                let dist =
                    if let Some(c) = a.corners.last().filter(|c| c.flags & STRAIGHTPATH_END != 0) {
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
                    let mut diff = vsub(&a.npos, &self.agents[j].npos);
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
            self.agents[i].desired_speed = speed;
            self.agents[i].dvel = dvel;
        }
        for &i in &self.active {
            let a = &self.agents[i];
            if a.state != S::Walking {
                continue;
            }
            let nvel = if a.params.update_flags & OBSTACLE_AVOIDANCE != 0 {
                self.avoidance.reset();
                for &(j, _) in &a.neis[..a.nneis] {
                    let n = &self.agents[j];
                    self.avoidance
                        .add_circle(n.npos, n.params.radius, n.vel, n.dvel)?;
                }
                for &(_, s) in &a.boundary.segments {
                    if tri_area_2d(&a.npos, &s[..3], &s[3..]) < 0. {
                        continue;
                    }
                    self.avoidance.add_segment(v3(&s[..3]), v3(&s[3..]))?;
                }
                let (v, n) = self.avoidance.sample_velocity_adaptive_in(
                    ws,
                    a.npos,
                    a.params.radius,
                    a.desired_speed,
                    a.vel,
                    a.dvel,
                    &self.avoidance_params[a.params.obstacle_avoidance_type],
                )?;
                self.velocity_samples += n;
                v
            } else {
                a.dvel
            };
            self.agents[i].nvel = nvel;
        }
        for &i in &self.active {
            let a = &mut self.agents[i];
            if a.state != S::Walking {
                continue;
            }
            let max = a.params.max_acceleration * dt;
            let mut dv = vsub(&a.nvel, &a.vel);
            let ds = vlen(&dv);
            if ds > max {
                dv = vscale(&dv, max / ds);
            }
            a.vel = vadd(&a.vel, &dv);
            if vlen(&a.vel) > 0.0001 {
                a.npos = vmad(&a.npos, &a.vel, dt);
            } else {
                a.vel = [0.; 3];
            }
        }
        for _ in 0..4 {
            for &i in &self.active {
                let a = &self.agents[i];
                if a.state != S::Walking {
                    continue;
                }
                let mut disp = [0.; 3];
                let mut w = 0.;
                for &(j, _) in &a.neis[..a.nneis] {
                    let nei = &self.agents[j];
                    let mut diff = vsub(&a.npos, &nei.npos);
                    diff[1] = 0.;
                    let mut dist = vlen_sqr(&diff);
                    if dist > sqr(a.params.radius + nei.params.radius) {
                        continue;
                    }
                    dist = sqrtf(dist);
                    let mut pen = (a.params.radius + nei.params.radius) - dist;
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
                self.agents[i].disp = disp;
            }
            for &i in &self.active {
                let a = &mut self.agents[i];
                if a.state == S::Walking {
                    a.npos = vadd(&a.npos, &a.disp);
                }
            }
        }
        for &i in &self.active {
            let a = &mut self.agents[i];
            if a.state != S::Walking {
                continue;
            }
            // Upstream keeps the corridor position if the local move fails.
            let _ =
                a.corridor
                    .move_position(a.npos, &mut q, &self.filters[a.params.query_filter_type]);
            a.npos = a.corridor.pos;
            if matches!(a.target_state, T::None | T::Velocity) {
                a.corridor.reset(a.corridor.first_poly(), a.npos)?;
                a.partial = false;
            }
        }
        for &i in &self.active {
            let a = &mut self.agents[i];
            let anim = &mut a.anim;
            if !anim.active {
                continue;
            }
            anim.t += dt;
            if anim.t > anim.tmax {
                anim.active = false;
                a.state = S::Walking;
                continue;
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
        for &i in &self.active {
            let a = &self.agents[i];
            finite(a.npos)?;
            finite(a.vel)?;
            finite(a.nvel)?;
            finite(a.dvel)?;
        }
        Ok(())
    }
}
