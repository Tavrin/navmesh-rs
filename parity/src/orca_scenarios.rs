//! Shared, deterministic crowd qualification scenarios.
use navmesh_rs::crowd::*;
use navmesh_rs::detour::{NavMesh, NavMeshCreateParams, create_nav_mesh_data};
use navmesh_rs::{Error, Workspace};
use std::time::Instant;

pub fn floor(width: u16, height: u16) -> Result<NavMesh, Error> {
    let n = 0xffff;
    let tile = create_nav_mesh_data(&NavMeshCreateParams {
        verts: &[0, 0, 0, 0, 0, height, width, 0, height, width, 0, 0],
        vert_count: 4,
        polys: &[0, 1, 2, 3, n, n, n, n, n, n, n, n],
        poly_count: 1,
        nvp: 6,
        poly_flags: &[1],
        poly_areas: &[0],
        bmin: [0.; 3],
        bmax: [f32::from(width), 1., f32::from(height)],
        cs: 1.,
        ch: 1.,
        walkable_height: 2.,
        walkable_radius: 0.4,
        walkable_climb: 0.5,
        ..Default::default()
    })?;
    Ok(NavMesh::new_single(tile)?.0)
}
pub fn doorway() -> Result<NavMesh, Error> {
    let n = 0xffff;
    let tile = create_nav_mesh_data(&NavMeshCreateParams {
        verts: &[
            0, 0, 0, 0, 0, 240, 118, 0, 240, 118, 0, 122, 118, 0, 118, 118, 0, 0, 122, 0, 118, 122,
            0, 122, 122, 0, 0, 122, 0, 240, 240, 0, 240, 240, 0, 0,
        ],
        vert_count: 12,
        polys: &[
            0, 1, 2, 3, 4, 5, n, n, n, 1, n, n, 4, 3, 7, 6, n, n, 0, n, 2, n, n, n, 8, 6, 7, 9, 10,
            11, n, 1, n, n, n, n,
        ],
        poly_count: 3,
        nvp: 6,
        poly_flags: &[1; 3],
        poly_areas: &[0; 3],
        bmin: [0.; 3],
        bmax: [240., 1., 240.],
        cs: 1.,
        ch: 1.,
        walkable_height: 2.,
        walkable_radius: 0.4,
        walkable_climb: 0.5,
        ..Default::default()
    })?;
    Ok(NavMesh::new_single(tile)?.0)
}
pub struct Scene {
    pub crowd: Crowd,
    pub nav: NavMesh,
    pub goals: Vec<[f32; 3]>,
}
pub fn scene(name: &str, count: usize, orca: bool) -> Result<Scene, Error> {
    let nav = if name == "corridor" {
        floor(240, 8)?
    } else if name == "doorway" {
        doorway()?
    } else {
        floor(240, 240)?
    };
    let mut ws = Workspace::new();
    let mut crowd = Crowd::new(&mut ws, &nav, count, 0.4)?;
    crowd.set_obstacle_avoidance_params(3, ObstacleAvoidanceParams::quality(3)?)?;
    let mut p = AgentParams::default();
    p.radius = 0.4;
    p.height = 2.;
    p.max_speed = 2.;
    p.max_acceleration = 100.;
    p.collision_query_range = 8.;
    p.update_flags = OBSTACLE_AVOIDANCE;
    if orca {
        p.avoidance = AvoidanceMode::Orca(OrcaParams::default());
    }
    let mut goals = Vec::new();
    // Circle recurrence uses fixed constants and integer quantization, avoiding
    // platform trigonometry in initial state generation.
    let (cos, sin) = if count == 64 {
        (0.9951847_f32, 0.09801714_f32)
    } else {
        (0.9999247, 0.012271538)
    };
    let mut dir = [1_f32, 0_f32];
    for i in 0..count {
        let (pos, goal) = match name {
            "circle" => {
                let radius = if count == 64 { 20. } else { 85. };
                let d = dir.map(|v| (v * radius * 1024.).round() / 1024.);
                dir = [dir[0] * cos - dir[1] * sin, dir[0] * sin + dir[1] * cos];
                (
                    [120. + d[0], 0., 120. + d[1]],
                    [120. - d[0], 0., 120. - d[1]],
                )
            }
            "crossing" => {
                let group = i % 4;
                let k = i / 4;
                let side = if group < 2 { 30. } else { 210. };
                let offset = 100. + (k % 20) as f32 * 1.5;
                let depth = (k / 20) as f32 * 1.5;
                if group % 2 == 0 {
                    ([side + depth, 0., offset], [240. - side, 0., offset])
                } else {
                    ([offset, 0., side + depth], [offset, 0., 240. - side])
                }
            }
            "corridor" => {
                let side = if i % 2 == 0 { 30. } else { 210. };
                let x = side + (i / 12) as f32 * 1.2;
                let z = 1. + (i / 2 % 6) as f32 * 1.1;
                ([x, 0., z], [240. - x, 0., z])
            }
            "doorway" => {
                // A four-unit opening connects the two rooms.
                let x = 80. + (i / 10) as f32 * 1.1;
                let z = 114. + (i % 10) as f32 * 1.1;
                ([x, 0., z], [180., 0., 119.5 + (i % 2) as f32])
            }
            _ => {
                let x = 20. + (i % 50) as f32 * 1.1;
                let z = 20. + (i / 50) as f32 * 1.1;
                ([x, 0., z], [x + 100., 0., z])
            }
        };
        let id = crowd.add_agent(&mut ws, &nav, pos, p)?;
        if name == "doorway" {
            let q = navmesh_rs::detour::NavMeshQuery::new(&nav, 32)?;
            let (r, g, _) = q.find_nearest_poly(
                &goal,
                &[2.; 3],
                &navmesh_rs::detour::DefaultFilter::default(),
            )?;
            crowd.request_move_target(&nav, id, r, g)?;
        }
        goals.push(goal);
    }
    Ok(Scene { crowd, nav, goals })
}
pub fn tick(s: &mut Scene, ws: &mut Workspace) -> Result<(), Error> {
    for (i, goal) in s.goals.iter().enumerate() {
        let a = s.crowd.agent(AgentId(i)).ok_or(Error::Failed)?;
        if a.target_state != TargetState::None && a.target_state != TargetState::Velocity {
            continue;
        }
        let dx = goal[0] - a.npos[0];
        let dz = goal[2] - a.npos[2];
        let distance = (dx * dx + dz * dz).sqrt();
        let speed = (distance / 0.05).min(2.);
        let v = if distance > 0.001 {
            [dx / distance * speed, 0., dz / distance * speed]
        } else {
            [0.; 3]
        };
        s.crowd.request_move_velocity(AgentId(i), v)?;
    }
    s.crowd.update(ws, &s.nav, 0.05)
}
pub fn digest() -> Result<(), Error> {
    for (name, n) in [
        ("circle", 64),
        ("circle", 512),
        ("crossing", 64),
        ("corridor", 64),
        ("doorway", 64),
        ("dense", 2000),
    ] {
        let mut s = scene(name, n, true)?;
        let mut ws = Workspace::new();
        let mut hash = 14695981039346656037u64;
        for _ in 0..1200 {
            tick(&mut s, &mut ws)?;
            for a in s.crowd.active_agents() {
                for v in a.npos.iter().chain(&a.vel).chain(&a.nvel) {
                    for b in v.to_bits().to_le_bytes() {
                        hash ^= u64::from(b);
                        hash = hash.wrapping_mul(1099511628211);
                    }
                }
            }
        }
        println!("orca {name} {n} {hash:016x}");
    }
    Ok(())
}
pub fn measure() -> Result<(), Error> {
    for n in [100, 1000, 5000] {
        let mut scenes = [scene("dense", n, false)?, scene("dense", n, true)?];
        let mut ws = Workspace::new();
        for _ in 0..10 {
            for s in &mut scenes {
                tick(s, &mut ws)?;
            }
        }
        let mut times = [Vec::new(), Vec::new()];
        for pair in 0..9 {
            for mode in if pair % 2 == 0 { [0, 1] } else { [1, 0] } {
                let start = Instant::now();
                for _ in 0..10 {
                    tick(&mut scenes[mode], &mut ws)?;
                }
                times[mode].push(start.elapsed().as_secs_f64() * 1e6 / (10 * n) as f64);
            }
        }
        for t in &mut times {
            t.sort_by(f64::total_cmp);
        }
        println!(
            "cost {n} sampled_us {:.6} orca_us {:.6} ratio {:.6}",
            times[0][4],
            times[1][4],
            times[1][4] / times[0][4]
        );
    }
    Ok(())
}
pub fn invariants() -> Result<(), Error> {
    for (name, n, ticks) in [
        ("circle", 64, 1400usize),
        ("circle", 512, 12000),
        ("crossing", 128, 2400),
        ("corridor", 64, 2400),
        ("doorway", 64, 1400),
        ("dense", 2000, 1400),
    ] {
        for mode in [false, true] {
            let mut s = scene(name, n, mode)?;
            let mut ws = Workspace::new();
            let mut overlaps = 0u64;
            let mut deepest = 0_f32;
            let mut reached = vec![false; n];
            let mut finished_tick = ticks;
            let mut ids = vec![0u16; n];
            let mut metric_grid = ProximityGrid::new(&mut ws, n * 4, 1.2)?;
            let mut tail_positions: Vec<_> = s.crowd.active_agents().map(|a| a.npos).collect();
            for t in 0..ticks {
                if t == ticks.saturating_sub(400) {
                    for (saved, a) in tail_positions.iter_mut().zip(s.crowd.agents()) {
                        *saved = a.npos;
                    }
                }
                tick(&mut s, &mut ws)?;
                let agents = s.crowd.agents();
                metric_grid.clear();
                for (i, a) in agents.iter().enumerate() {
                    let [x, _, z] = a.npos;
                    metric_grid.add_item(i as u16, [x - 0.4, z - 0.4, x + 0.4, z + 0.4])?;
                }
                for (i, a) in agents.iter().enumerate() {
                    let goal = s.goals[i];
                    let dx = goal[0] - a.npos[0];
                    let dz = goal[2] - a.npos[2];
                    if dx * dx + dz * dz < 0.25 {
                        reached[i] = true;
                    }
                    // Rebuilt from final positions, independently of both the
                    // avoidance cap and the crowd's pre-integration grid.
                    let k = metric_grid.query_items(
                        &mut ws,
                        [
                            a.npos[0] - 1.,
                            a.npos[2] - 1.,
                            a.npos[0] + 1.,
                            a.npos[2] + 1.,
                        ],
                        &mut ids,
                    )?;
                    for &j in &ids[..k] {
                        let j = usize::from(j);
                        if j <= i {
                            continue;
                        }
                        let b = &agents[j];
                        let dx = a.npos[0] - b.npos[0];
                        let dz = a.npos[2] - b.npos[2];
                        let penetration = 0.8 - (dx * dx + dz * dz).sqrt();
                        if penetration > 0.001 {
                            overlaps += 1;
                            deepest = deepest.max(penetration);
                        }
                    }
                }
                if reached.iter().all(|&v| v) {
                    finished_tick = t + 1;
                    break;
                }
            }
            let unreached = reached.iter().filter(|&&v| !v).count();
            // An operational stationary-deadlock detector, distinct from failing
            // to arrive before a timeout: <5 cm displacement over the final 20 s.
            let stationary_deadlocks = s
                .crowd
                .agents()
                .iter()
                .enumerate()
                .filter(|(i, a)| {
                    let dx = a.npos[0] - tail_positions[*i][0];
                    let dz = a.npos[2] - tail_positions[*i][2];
                    !reached[*i] && dx * dx + dz * dz < 0.0025
                })
                .count();
            println!(
                "invariant {name} {n} {} overlaps {overlaps} max_penetration {deepest:.8} unreached {unreached} stationary_deadlocks {stationary_deadlocks} finish_s {:.2}",
                if mode { "orca" } else { "sampled" },
                finished_tick as f32 * 0.05
            );
        }
    }
    Ok(())
}
