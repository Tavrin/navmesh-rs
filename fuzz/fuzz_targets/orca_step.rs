#![no_main]
use libfuzzer_sys::fuzz_target;
use navmesh_rs::crowd::{OrcaAgent, OrcaParams, OrcaScratch, orca_velocity};
use navmesh_rs::{Limits, Workspace};
fuzz_target!(|data: &[u8]| {
    let values: Vec<_> = data
        .as_chunks::<4>()
        .0
        .iter()
        .take(256)
        .map(|b| f32::from_le_bytes(*b))
        .collect();
    if values.len() < 12 {
        return;
    }
    let a = OrcaAgent {
        position: [values[0], values[1]],
        velocity: [values[2], values[3]],
        preferred_velocity: [values[4], values[5]],
        radius: values[6],
        max_speed: values[7],
    };
    let mut p = OrcaParams::default();
    p.time_horizon = values[8];
    p.time_horizon_obstacles = values[9];
    p.max_neighbours = u16::from(data[0]);
    let ns: Vec<_> = values[12..]
        .as_chunks::<8>()
        .0
        .iter()
        .map(|v| OrcaAgent {
            position: [v[0], v[1]],
            velocity: [v[2], v[3]],
            preferred_velocity: [v[4], v[5]],
            radius: v[6],
            max_speed: v[7],
        })
        .collect();
    let walls = [[values[0], values[1], values[10], values[11]]];
    let mut ws = Workspace::with_limits(Limits {
        max_bytes: 1024 * 1024,
        max_work: 100000,
        ..Limits::UNLIMITED
    });
    if let Ok(v) = orca_velocity(
        &mut ws,
        &a,
        &ns,
        &walls,
        &p,
        values[10],
        &mut OrcaScratch::default(),
    ) {
        assert!(v.iter().all(|x| x.is_finite()));
        assert!(v[0] * v[0] + v[1] * v[1] <= a.max_speed * a.max_speed * (1. + 1e-5) + 1e-5);
    }
});
