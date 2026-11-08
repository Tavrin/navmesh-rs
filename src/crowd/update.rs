//! Ordered simulation phases, translated from DetourCrowd.cpp; see NOTICE. The
//! per-agent arithmetic lives in `phases`, shared with the crowd world.
use super::phases::{self, AvoidScratch};
use super::*;
use AgentState as S;

impl Crowd {
    /// [`Crowd::update`] under a work budget (not in recastnavigation): the workspace's
    /// work limit for the tick becomes the smaller of its own and what `budget` has
    /// left, and the work the tick charged is spent from `budget`.
    ///
    /// A tick is not resumable: its phases run in upstream's order and each sees the
    /// previous one's result. If `budget` cannot cover the up-front charge of
    /// `4096` units per agent (`Workspace::usage` after any tick shows the real
    /// cost) the outcome is [`Budgeted::Exhausted`] and the crowd is untouched. If
    /// it runs out later the outcome is also `Exhausted`, with the same caveat as a
    /// [`Error::LimitExceeded`] from `update`: earlier phases were applied, so the
    /// caller should discard that tick (and give the next one a larger budget or
    /// fewer agents). Within the budget the result equals `update`'s.
    pub fn update_budgeted(
        &mut self,
        ws: &mut Workspace,
        nav: &NavMesh,
        dt: f32,
        budget: &mut Budget,
    ) -> Result<Budgeted<()>, Error> {
        let saved = ws.limits();
        let allowed = saved.max_work.min(budget.remaining());
        ws.set_limits(Limits {
            max_work: allowed,
            ..saved
        });
        let result = self.update(ws, nav, dt);
        ws.set_limits(saved);
        let used = ws.usage().work;
        match result {
            Ok(()) => {
                budget.spend(used);
                Ok(Budgeted::Complete(()))
            }
            // The work counter went over the allowance: the budget (not the byte
            // limit) stopped the tick.
            Err(Error::LimitExceeded) if used > allowed => {
                budget.exhaust();
                Ok(Budgeted::Exhausted(()))
            }
            Err(e) => Err(e),
        }
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
        for &i in &self.active {
            phases::check_path(&mut self.agents[i], &q, &self.filters, &self.extents, dt)?;
        }
        phases::move_requests(
            &mut self.agents[..],
            &mut self.pathq,
            &self.filters,
            ws,
            nav,
            &mut q,
            100,
        )?;
        phases::optimize_topology(
            &mut self.agents[..],
            &self.active,
            &mut q,
            &self.filters,
            dt,
        );
        self.grid.clear();
        for (j, &i) in self.active.iter().enumerate() {
            let a = &self.agents[i];
            let p = a.npos;
            let r = a.params.radius;
            self.grid
                .add_item(j as u16, [p[0] - r, p[2] - r, p[0] + r, p[2] + r])?;
        }
        let all_candidates = self.fixes.contains(UpstreamFixes::NEAREST_NEIGHBOURS);
        let any_orca = self
            .active
            .iter()
            .any(|&i| matches!(self.agents[i].params.avoidance, AvoidanceMode::Orca(_)));
        // One id slot per active agent holds every candidate a query can return.
        let mut wide = if all_candidates || any_orca {
            ws.vec(self.active.len(), 0u16)?
        } else {
            Vec::new()
        };
        let mut candidate_stamps = if all_candidates || any_orca {
            ws.vec(self.active.len(), 0u16)?
        } else {
            Vec::new()
        };
        for &i in &self.active {
            let a = &mut self.agents[i];
            if a.state != S::Walking {
                continue;
            }
            phases::update_boundary(a, &mut q, &self.filters[a.params.query_filter_type])?;
            let range = a.params.collision_query_range;
            let nearest = all_candidates || matches!(a.params.avoidance, AvoidanceMode::Orca(_));
            let neighbour_cap = phases::prepare_neighbours(ws, a, self.active.len())?;
            let pos = a.npos;
            let height = a.params.height;
            let mut capped = [0u16; 32];
            let ids: &mut [u16] = if nearest { &mut wide } else { &mut capped };
            let bounds = [
                pos[0] - range,
                pos[2] - range,
                pos[0] + range,
                pos[2] + range,
            ];
            let n = if nearest {
                // For ORCA, a smaller complete query proves the same nearest
                // set once it contains the cap inside its inscribed circle.
                // Otherwise discard its stamps and query the full range. The
                // corrected sampled path keeps its exhaustive collection.
                let radius = (2. * a.params.radius * sqrtf(neighbour_cap as f32)).min(range);
                let narrow = matches!(a.params.avoidance, AvoidanceMode::Orca(_))
                    && neighbour_cap > 0
                    && radius < range;
                let mut n = 0;
                let mut enough = false;
                if narrow {
                    n = self.grid.query_all_in(
                        ws,
                        [
                            pos[0] - radius,
                            pos[2] - radius,
                            pos[0] + radius,
                            pos[2] + radius,
                        ],
                        ids,
                        &mut candidate_stamps,
                        (i + 1) as u16,
                    )?;
                    let mut inside = 0;
                    for &id in &ids[..n] {
                        ws.work(1)?;
                        let j = self.active[usize::from(id)];
                        let nei = &self.agents[j];
                        if i != j
                            && (pos[1] - nei.npos[1]).abs() < (height + nei.params.height) / 2.
                            && vdist_2d_sqr(&pos, &nei.npos) <= sqr(radius)
                        {
                            inside += 1;
                        }
                    }
                    enough = inside >= neighbour_cap;
                }
                if !enough {
                    for &id in &ids[..n] {
                        ws.work(1)?;
                        candidate_stamps[usize::from(id)] = 0;
                    }
                    n = self.grid.query_all_in(
                        ws,
                        bounds,
                        ids,
                        &mut candidate_stamps,
                        (i + 1) as u16,
                    )?;
                }
                n
            } else {
                self.grid.query_items_in(ws, bounds, ids)?
            };
            for &id in &ids[..n] {
                let j = self.active[id as usize];
                if i == j {
                    continue;
                }
                let nei = &self.agents[j];
                let Some(dist) =
                    phases::neighbour_distance(&pos, height, range, &nei.npos, nei.params.height)
                else {
                    continue;
                };
                phases::insert_neighbour(&mut self.agents[i], nearest, neighbour_cap, j, dist);
            }
        }
        for &i in &self.active {
            phases::find_corners(&mut self.agents[i], &mut q, &self.filters)?;
        }
        for &i in &self.active {
            phases::trigger_offmesh(&mut self.agents[i], &q);
        }
        for &i in &self.active {
            if let Some((dvel, speed)) = phases::steer(&self.agents[i], &self.agents[..]) {
                self.agents[i].desired_speed = speed;
                self.agents[i].dvel = dvel;
            }
        }
        let mut orca_scratch = OrcaScratch::default();
        let mut orca_neighbours = Vec::new();
        let mut scratch = AvoidScratch {
            query: &mut self.avoidance,
            orca: &mut orca_scratch,
            orca_neighbours: &mut orca_neighbours,
        };
        for &i in &self.active {
            if let Some((nvel, n)) = phases::avoid(
                ws,
                &self.agents[i],
                &self.agents[..],
                dt,
                &self.avoidance_params,
                &mut scratch,
            )? {
                self.velocity_samples += n;
                self.agents[i].nvel = nvel;
            }
        }
        for &i in &self.active {
            phases::integrate(&mut self.agents[i], dt);
        }
        for _ in 0..4 {
            for &i in &self.active {
                if let Some(disp) =
                    phases::collision_displacement(i, &self.agents[i], &self.agents[..])
                {
                    self.agents[i].disp = disp;
                }
            }
            for &i in &self.active {
                phases::apply_displacement(&mut self.agents[i]);
            }
        }
        for &i in &self.active {
            phases::move_on_mesh(&mut self.agents[i], &mut q, &self.filters)?;
        }
        for &i in &self.active {
            phases::animate(&mut self.agents[i], dt);
        }
        for &i in &self.active {
            phases::check_finite(&self.agents[i])?;
        }
        Ok(())
    }
}
