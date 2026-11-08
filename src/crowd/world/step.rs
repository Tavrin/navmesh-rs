//! One fixed step: serial path phases, then gather-only per-agent phases that run
//! data-parallel with the `parallel` feature.
use super::*;
use crate::crowd::phases::{self, AgentSlots, AgentsView, AvoidScratch};

/// The arrays other agents' phases read; no phase writes them.
struct View<'a> {
    pos: &'a [V3],
    vel: &'a [V3],
    dvel: &'a [V3],
    radius: &'a [f32],
    height: &'a [f32],
    max_speed: &'a [f32],
}
impl<'a> View<'a> {
    fn of(
        pos: &'a [V3],
        vel: &'a [V3],
        dvel: &'a [V3],
        radius: &'a [f32],
        height: &'a [f32],
        max_speed: &'a [f32],
    ) -> Self {
        Self {
            pos,
            vel,
            dvel,
            radius,
            height,
            max_speed,
        }
    }
}
impl AgentsView for View<'_> {
    fn npos(&self, j: usize) -> V3 {
        self.pos[j]
    }
    fn vel(&self, j: usize) -> V3 {
        self.vel[j]
    }
    fn dvel(&self, j: usize) -> V3 {
        self.dvel[j]
    }
    fn radius(&self, j: usize) -> f32 {
        self.radius[j]
    }
    fn max_speed(&self, j: usize) -> f32 {
        self.max_speed[j]
    }
}

/// The side pool seen by agent index, for the serial phases.
struct Slots<'a> {
    heavy: &'a mut [CrowdAgent],
    heavy_of: &'a [u32],
}
impl AgentSlots for Slots<'_> {
    fn count(&self) -> usize {
        self.heavy_of.len()
    }
    fn slot(&self, i: usize) -> Option<&CrowdAgent> {
        match self.heavy_of.get(i) {
            Some(&s) if s != NONE => self.heavy.get(s as usize),
            _ => None,
        }
    }
    fn slot_mut(&mut self, i: usize) -> Option<&mut CrowdAgent> {
        match self.heavy_of.get(i) {
            Some(&s) if s != NONE => self.heavy.get_mut(s as usize),
            _ => None,
        }
    }
}

/// Scratch owned by one worker of a parallel phase.
struct Worker<'n> {
    ws: Workspace,
    q: NavMeshQuery<'n>,
    avoid: ObstacleAvoidanceQuery,
    orca: OrcaScratch,
    orca_neighbours: Vec<OrcaAgent>,
}

#[derive(Clone, Copy, Default)]
struct Tally {
    work: u64,
    samples: u64,
}
impl Tally {
    fn add(self, o: Tally) -> Tally {
        Tally {
            work: self.work.saturating_add(o.work),
            samples: self.samples.saturating_add(o.samples),
        }
    }
}

struct Ctx<'n> {
    nav: &'n NavMesh,
    limits: Limits,
    fixes: UpstreamFixes,
}
impl<'n> Ctx<'n> {
    fn worker(&self) -> Result<Worker<'n>, Error> {
        // Per-step scratch bytes are charged up front by the step, so the
        // per-agent byte counter (which depends on what a worker grew before)
        // never decides an outcome; the work limit applies per agent and phase.
        let mut ws = Workspace::with_limits(Limits {
            max_bytes: usize::MAX,
            ..self.limits
        });
        let mut avoid = ObstacleAvoidanceQuery::new_in(&mut ws, 6, 8)?;
        avoid.set_fixes(self.fixes);
        Ok(Worker {
            ws,
            q: NavMeshQuery::new(self.nav, 512)?,
            avoid,
            orca: OrcaScratch::default(),
            orca_neighbours: Vec::new(),
        })
    }

    /// Runs `f` once per occupied side-pool entry with that entry's agent index.
    /// Each call writes only its own entry, so the outcome does not depend on how
    /// entries are split between threads; work is counted per call and summed.
    /// On failure the error of the lowest failing entry is returned.
    fn run<F>(&self, heavy: &mut [CrowdAgent], owner: &[u32], f: F) -> Result<Tally, Error>
    where
        F: Fn(&mut Worker<'n>, usize, &mut CrowdAgent) -> Result<u64, Error> + Sync + Send,
    {
        let call = |w: &mut Worker<'n>, i: u32, a: &mut CrowdAgent| {
            if i == NONE {
                return Ok(Tally::default());
            }
            w.ws.begin();
            let samples = f(w, i as usize, a)?;
            Ok(Tally {
                work: w.ws.usage().work,
                samples,
            })
        };
        #[cfg(feature = "parallel")]
        {
            use rayon::prelude::*;
            heavy
                .par_iter_mut()
                .zip(owner.par_iter())
                .map_init(
                    || self.worker(),
                    |w, (a, &i)| match w {
                        Ok(w) => call(w, i, a),
                        Err(e) => Err(e.clone()),
                    },
                )
                .reduce(
                    || Ok(Tally::default()),
                    |a, b| match (a, b) {
                        (Err(e), _) | (Ok(_), Err(e)) => Err(e),
                        (Ok(x), Ok(y)) => Ok(x.add(y)),
                    },
                )
        }
        #[cfg(not(feature = "parallel"))]
        {
            let mut w = self.worker()?;
            let mut total = Tally::default();
            for (a, &i) in heavy.iter_mut().zip(owner) {
                total = total.add(call(&mut w, i, a)?);
            }
            Ok(total)
        }
    }
}

/// Copies one field of every occupied side-pool entry into its agent-index array.
fn commit(heavy: &[CrowdAgent], owner: &[u32], dst: &mut [V3], field: fn(&CrowdAgent) -> V3) {
    for (a, &i) in heavy.iter().zip(owner) {
        if i != NONE {
            dst[i as usize] = field(a);
        }
    }
}

impl CrowdWorld {
    /// Advances every agent by [`CrowdWorldParams::fixed_dt`] in [`Crowd::update`]'s
    /// phase order.
    ///
    /// The step's work is [`STEP_WORK_PER_AGENT`] per agent plus the path queue's
    /// node expansions per step; it is spent from `budget` before anything is
    /// simulated. When it does not fit, the outcome is [`Budgeted::Exhausted`]
    /// (its report states the work needed) and the world is untouched. An error can
    /// leave earlier phases applied; discard the world (or restore a snapshot).
    ///
    /// With the `parallel` feature the per-agent phases run on the current rayon
    /// pool; install a pool to choose the thread count. The result is identical
    /// for every thread count.
    pub fn step(
        &mut self,
        nav: &NavMesh,
        budget: &mut Budget,
    ) -> Result<Budgeted<StepReport>, Error> {
        let dt = self.params.fixed_dt;
        self.active.clear();
        self.active
            .try_reserve(self.count as usize)
            .map_err(|_| Error::AllocationFailed)?;
        self.active.extend(
            self.heavy_of
                .iter()
                .enumerate()
                .filter(|(_, s)| **s != NONE)
                .map(|(i, _)| i),
        );
        let n = self.active.len();
        let mut report = StepReport {
            agents: n as u32,
            work: (n as u64)
                .saturating_mul(STEP_WORK_PER_AGENT)
                .saturating_add(self.params.path_queue.iterations as u64),
            ..StepReport::default()
        };
        let mut ws = Workspace::with_limits(self.limits);
        ws.charge_bytes::<u8>(
            n.checked_mul(32768)
                .and_then(|b| b.checked_add(512 * 128 + 65536))
                .ok_or(Error::SizeOverflow)?,
        )?;
        if !budget.spend(report.work) {
            return Ok(Budgeted::Exhausted(report));
        }
        let ctx = Ctx {
            nav,
            limits: self.limits,
            fixes: self.fixes,
        };
        let Self {
            params,
            extents,
            filters,
            presets,
            pathq,
            heavy_of,
            pos,
            vel,
            dvel,
            radius,
            height,
            max_speed,
            heavy,
            owner,
            grid,
            active,
            ..
        } = self;
        let (filters, presets) = (&*filters, &*presets);
        let (owner, heavy_of) = (&owner[..], &heavy_of[..]);

        // Serial phases, in agent-index order.
        let mut q = NavMeshQuery::new(nav, 512)?;
        for &i in active.iter() {
            phases::check_path(&mut heavy[heavy_of[i] as usize], &q, filters, extents, dt)?;
        }
        let mut slots = Slots {
            heavy: &mut heavy[..],
            heavy_of,
        };
        phases::move_requests(
            &mut slots,
            pathq,
            filters,
            &mut ws,
            nav,
            &mut q,
            params.path_queue.iterations,
        )?;
        phases::optimize_topology(&mut slots, active, &mut q, filters, dt);
        let mut tally = Tally {
            work: ws.usage().work,
            samples: 0,
        };
        commit(heavy, owner, pos, |a| a.npos);
        grid.rebuild(active.iter().map(|&i| (i as u32, pos[i][0], pos[i][2])))?;

        // Walls and neighbours, corners and visibility, off-mesh triggers: each
        // writes only its own agent and reads positions no phase here changes.
        let view = View::of(pos, vel, dvel, radius, height, max_speed);
        let grid = &*grid;
        tally = tally.add(ctx.run(heavy, owner, |w, i, a| {
            if a.state == AgentState::Walking {
                phases::update_boundary(a, &mut w.q, &filters[a.params.query_filter_type])?;
                let range = a.params.collision_query_range;
                let cap = phases::prepare_neighbours(&mut w.ws, a, n)?;
                let p = a.npos;
                let h = a.params.height;
                let ws = &mut w.ws;
                grid.for_each_near(p[0], p[2], range, |j| {
                    ws.work(1)?;
                    if j != i {
                        if let Some(d) =
                            phases::neighbour_distance(&p, h, range, &view.pos[j], view.height[j])
                        {
                            phases::insert_neighbour(a, true, cap, j, d);
                        }
                    }
                    Ok(())
                })?;
            }
            phases::find_corners(a, &mut w.q, filters)?;
            phases::trigger_offmesh(a, &w.q);
            Ok(0)
        })?);

        // Steering reads neighbour positions.
        tally = tally.add(ctx.run(heavy, owner, |_, _, a| {
            if let Some((v, speed)) = phases::steer(a, &view) {
                a.desired_speed = speed;
                a.dvel = v;
            }
            Ok(0)
        })?);
        commit(heavy, owner, dvel, |a| a.dvel);

        // Avoidance reads neighbours' positions, velocities and desired velocities
        // from the arrays, so integrating each agent right after it is the same as
        // integrating after all of them.
        let view = View::of(pos, vel, dvel, radius, height, max_speed);
        tally = tally.add(ctx.run(heavy, owner, |w, _, a| {
            let mut s = AvoidScratch {
                query: &mut w.avoid,
                orca: &mut w.orca,
                orca_neighbours: &mut w.orca_neighbours,
            };
            let mut samples = 0;
            if let Some((v, k)) = phases::avoid(&mut w.ws, a, &view, dt, presets, &mut s)? {
                a.nvel = v;
                samples = k as u64;
            }
            phases::integrate(a, dt);
            Ok(samples)
        })?);
        commit(heavy, owner, pos, |a| a.npos);

        // Four Jacobi collision iterations; the last one's application is fused
        // with the agent-local tail of the step.
        for iteration in 0..4 {
            let view = View::of(pos, vel, dvel, radius, height, max_speed);
            tally = tally.add(ctx.run(heavy, owner, |_, i, a| {
                if let Some(d) = phases::collision_displacement(i, a, &view) {
                    a.disp = d;
                }
                Ok(0)
            })?);
            if iteration < 3 {
                for (a, &i) in heavy.iter_mut().zip(owner.iter()) {
                    if i != NONE {
                        phases::apply_displacement(a);
                        pos[i as usize] = a.npos;
                    }
                }
            }
        }
        tally = tally.add(ctx.run(heavy, owner, |w, _, a| {
            phases::apply_displacement(a);
            phases::move_on_mesh(a, &mut w.q, filters)?;
            phases::animate(a, dt);
            Ok(0)
        })?);
        for &i in active.iter() {
            phases::check_finite(&heavy[heavy_of[i] as usize])?;
        }
        commit(heavy, owner, pos, |a| a.npos);
        commit(heavy, owner, vel, |a| a.vel);
        commit(heavy, owner, dvel, |a| a.dvel);
        report.measured_work = tally.work;
        report.velocity_samples = tally.samples;
        report.path_queue_depth = pathq.pending() as u32;
        Ok(Budgeted::Complete(report))
    }
}
