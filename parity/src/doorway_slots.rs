//! Shared-goal slot qualification with identical destinations for the oracle.
use crate::scenarios;
use navmesh_rs::crowd::world::*;
use navmesh_rs::crowd::*;
use navmesh_rs::detour::DefaultFilter;
use navmesh_rs::{Budget, Limits, UpstreamFixes, Workspace};
use std::io::Write;
use std::process::{Command, Stdio};

pub fn run() -> Result<(), Box<dyn std::error::Error>> {
    let nav = scenarios::doorway()?;
    let spawns = scenarios::spawns("doorway", 64);
    // The original two points form one destination region; use one allocator so
    // adjacent shared goals cannot allocate overlapping reservations.
    let mut slots = GoalSlots::around([180., 0., 120.])?;
    let goals: Vec<_> = (0..64)
        .map(|_| slots.allocate(&nav, &DefaultFilter::default(), 1024))
        .collect::<Result<_, _>>()?;
    let mut input = String::from("doorway 64 12000\n");
    for ((pos, _), (_, _, goal)) in spawns.iter().zip(&goals) {
        input.push_str(&format!("{} {} {} {}\n", pos[0], pos[2], goal[0], goal[2]));
    }
    let oracle = std::env::args().nth(2);
    let mut reference = None;
    if let Some(path) = oracle {
        let mut child = Command::new(path)
            .stdin(Stdio::piped())
            .stdout(Stdio::piped())
            .spawn()?;
        child
            .stdin
            .take()
            .ok_or("oracle stdin")?
            .write_all(input.as_bytes())?;
        let out = child.wait_with_output()?;
        if !out.status.success() {
            return Err("oracle failed".into());
        }
        let text = String::from_utf8(out.stdout)?;
        let values: Vec<f32> = text
            .split_whitespace()
            .map(str::parse)
            .collect::<Result<_, _>>()?;
        if values.len() != 12000 * 64 * 4 {
            return Err("oracle trajectory length".into());
        }
        let mut arrivals = [None; 64];
        for t in 0..12000 {
            for i in 0..64 {
                let k = (t * 64 + i) * 4;
                let g = goals[i].2;
                if (values[k] - g[0]).powi(2) + (values[k + 1] - g[2]).powi(2) < 0.25 {
                    arrivals[i].get_or_insert((t + 1) as f32 * 0.05);
                }
            }
        }
        reference = Some(p100(&arrivals)?);
        println!(
            "oracle slots p100={:.2}",
            reference.ok_or("oracle arrival")?
        );
    }
    for policy in [
        ArrivalPolicy::Keep,
        ArrivalPolicy::Park,
        ArrivalPolicy::Remove,
    ] {
        let mut ws = Workspace::new();
        let mut crowd = Crowd::new(&mut ws, &nav, 64, 0.4)?;
        crowd.set_fixes(UpstreamFixes::NEAREST_NEIGHBOURS);
        let mut config = CrowdWorldParams::default();
        config.max_agents = 64;
        config.max_heavy_agents = 64;
        config.max_radius = 0.4;
        config.fixed_dt = 0.05;
        let mut world = CrowdWorld::new(&nav, &config, Limits::UNLIMITED)?;
        let mut p = scenarios::agent_params(true);
        p.arrival_policy = policy;
        let mut handles = Vec::new();
        for (i, (pos, _)) in spawns.iter().enumerate() {
            let a = crowd.add_agent(&mut ws, &nav, *pos, p)?;
            crowd.request_move_target(&nav, a, goals[i].1, goals[i].2)?;
            let h = world.add_agent(&nav, *pos, p)?;
            world.set_goal(
                &nav,
                h,
                Goal::Point {
                    poly: goals[i].1,
                    pos: goals[i].2,
                },
            )?;
            handles.push(h);
        }
        let mut arrivals = [None; 64];
        for t in 0..12000 {
            crowd.update(&mut ws, &nav, 0.05)?;
            world.step(&nav, &mut Budget::unlimited())?;
            for i in 0..64 {
                let a = crowd.agent(AgentId(i));
                let b = world.agent(handles[i]);
                assert_eq!(a.is_some(), b.is_some());
                if let (Some(a), Some(b)) = (a, b) {
                    assert_eq!(a.npos.map(f32::to_bits), b.position().map(f32::to_bits));
                    assert_eq!(a.vel.map(f32::to_bits), b.velocity().map(f32::to_bits));
                    let g = goals[i].2;
                    if a.arrived() || (a.npos[0] - g[0]).powi(2) + (a.npos[2] - g[2]).powi(2) < 0.25
                    {
                        arrivals[i].get_or_insert((t + 1) as f32 * 0.05);
                    }
                } else {
                    arrivals[i].get_or_insert((t + 1) as f32 * 0.05);
                }
            }
            if arrivals.iter().all(Option::is_some) {
                break;
            }
        }
        let last = p100(&arrivals)?;
        println!("crowd + world slots {policy:?} arrived=64 p100={last:.2}");
        if policy == ArrivalPolicy::Keep
            && let Some(r) = reference
        {
            assert!(last <= 1.05 * r, "slot p100 ratio {}", last / r);
        }
    }
    Ok(())
}
fn p100(arrivals: &[Option<f32>; 64]) -> Result<f32, Box<dyn std::error::Error>> {
    if arrivals.iter().any(Option::is_none) {
        return Err("unarrived agents at ceiling".into());
    }
    Ok(arrivals.iter().flatten().copied().fold(0., f32::max))
}
