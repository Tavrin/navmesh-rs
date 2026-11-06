#![cfg(feature = "crowd")]
use navmesh_rs::crowd::*;
use navmesh_rs::{Limits, Workspace};
fn agent(x: f32, z: f32) -> OrcaAgent {
    OrcaAgent {
        position: [x, z],
        velocity: [0.; 2],
        preferred_velocity: [1., 0.],
        radius: 0.5,
        max_speed: 2.,
    }
}
#[test]
fn free_head_on_wall_and_limits() {
    let mut ws = Workspace::new();
    let mut scratch = OrcaScratch::default();
    let p = OrcaParams::default();
    let a = agent(0., 0.);
    assert_eq!(
        orca_velocity(&mut ws, &a, &[], &[], &p, 0.1, &mut scratch).unwrap(),
        [1., 0.]
    );
    let mut b = agent(2., 0.);
    b.preferred_velocity = [-1., 0.];
    let va = orca_velocity(&mut ws, &a, &[b], &[], &p, 0.1, &mut scratch).unwrap();
    let vb = orca_velocity(&mut ws, &b, &[a], &[], &p, 0.1, &mut scratch).unwrap();
    assert!((va[0] - 0.25).abs() < 1e-6);
    assert!((va[0] + vb[0]).abs() < 1e-6);
    for wall in [[1., -5., 1., 5.], [1., 5., 1., -5.]] {
        let v = orca_velocity(&mut ws, &a, &[], &[wall], &p, 0.1, &mut scratch).unwrap();
        assert!((v[0] - 0.5).abs() < 1e-6);
    }
    let mut invalid = a;
    invalid.velocity[0] = f32::NAN;
    assert!(orca_velocity(&mut ws, &invalid, &[], &[], &p, 0.1, &mut scratch).is_err());
    assert!(orca_velocity(&mut ws, &a, &[], &[], &p, 0., &mut scratch).is_err());
    ws.set_limits(Limits {
        max_work: 0,
        ..Limits::UNLIMITED
    });
    assert!(orca_velocity(&mut ws, &a, &[b], &[], &p, 0.1, &mut scratch).is_err());
}
#[test]
fn random_infeasible_and_degenerate_states_stay_speed_bounded() {
    let mut rng = 7u32;
    let mut rand = || {
        rng = rng.wrapping_mul(1664525).wrapping_add(1013904223);
        (rng >> 8) as f32 / 16777216.
    };
    let mut ws = Workspace::new();
    let mut scratch = OrcaScratch::default();
    for _ in 0..10000 {
        let mut a = agent(0., 0.);
        a.max_speed = rand() * 3. + 0.01;
        a.velocity = [rand() * 8. - 4., rand() * 8. - 4.];
        a.preferred_velocity = [rand() * 8. - 4., rand() * 8. - 4.];
        let neighbours: Vec<_> = (0..16)
            .map(|_| {
                let mut b = agent(rand() * 8. - 4., rand() * 8. - 4.);
                b.velocity = [rand() * 8. - 4., rand() * 8. - 4.];
                b
            })
            .collect();
        let v = orca_velocity(
            &mut ws,
            &a,
            &neighbours,
            &[],
            &OrcaParams::default(),
            0.05,
            &mut scratch,
        )
        .unwrap();
        assert!(v.iter().all(|v| v.is_finite()));
        assert!(f64::from(v[0]).hypot(f64::from(v[1])) <= f64::from(a.max_speed));
    }
}
#[test]
fn long_pair_run_remains_separated_and_makes_progress() {
    let mut agents = [agent(-5., -0.1), agent(5., 0.1)];
    let goals = [[5., 0.1], [-5., -0.1]];
    let mut ws = Workspace::new();
    let mut scratch = OrcaScratch::default();
    for _ in 0..2000 {
        let old = agents;
        for i in 0..2 {
            let diff = [
                goals[i][0] - old[i].position[0],
                goals[i][1] - old[i].position[1],
            ];
            let len = (diff[0] * diff[0] + diff[1] * diff[1]).sqrt().max(1.);
            agents[i].preferred_velocity = [diff[0] / len, diff[1] / len];
            let v = orca_velocity(
                &mut ws,
                &agents[i],
                &[old[1 - i]],
                &[],
                &OrcaParams::default(),
                0.05,
                &mut scratch,
            )
            .unwrap();
            agents[i].velocity = v;
            agents[i].position = [
                old[i].position[0] + v[0] * 0.05,
                old[i].position[1] + v[1] * 0.05,
            ];
        }
        let d = [
            agents[0].position[0] - agents[1].position[0],
            agents[0].position[1] - agents[1].position[1],
        ];
        assert!(d[0] * d[0] + d[1] * d[1] >= 0.9999);
    }
    for i in 0..2 {
        assert!((agents[i].position[0] - goals[i][0]).abs() < 0.01);
    }
}

#[test]
fn overlapping_snapshot_keeps_the_speed_disc_when_the_lp_is_infeasible() {
    // Exact frozen-state input retained from the differential comparison.
    let agents = [
        [
            0x40abe170, 0x40c9a518, 0xbf8fbb91, 0xbf8441c2, 0xbf443d07, 0xbf6854f5, 0x3e988e9f,
            0x40000000,
        ],
        [
            0xbf360ac5, 0x400e8979, 0xbfdec22a, 0x3ed85741, 0xbb9aab94, 0x3f8e3e1d, 0x3f26b26e,
            0x40000000,
        ],
        [
            0xc0c2c08d, 0xbfd08ea3, 0x3f996239, 0xbfd0fb56, 0xbf2e55db, 0x3fbfe85a, 0x3e99b31f,
            0x40000000,
        ],
        [
            0xc05dbe7e, 0xc094db2b, 0xbe8866ab, 0x3fa8c45f, 0x3fd28471, 0xbf1dd5de, 0x3e952014,
            0x40000000,
        ],
        [
            0x40eff9f4, 0x40adbb23, 0xbf6673b2, 0x3fbd7f38, 0xbfb2c9e1, 0x3efc5dd3, 0x3e6c51e8,
            0x40000000,
        ],
        [
            0xc07d5044, 0xc09a66f3, 0xbfef1941, 0xbf417040, 0x3fbf42e4, 0xbfc65678, 0x3ef73bda,
            0x40000000,
        ],
        [
            0x40e9d636, 0xc0d5ce74, 0x3fb67e0d, 0xbf31f2c4, 0xbea7c7b1, 0x3edfa6ae, 0x3eb40a4f,
            0x40000000,
        ],
        [
            0xc04cdad7, 0x40763556, 0xbee6f021, 0x3fed1f43, 0x3fe454ce, 0x3ecb2b9f, 0x3f1e5c64,
            0x40000000,
        ],
        [
            0xc07da09e, 0xc01515f4, 0x3ffa7305, 0xbfe86426, 0x3f19e847, 0x3fc08c00, 0x3e8aa2aa,
            0x40000000,
        ],
    ]
    .map(|bits| {
        let v = bits.map(f32::from_bits);
        OrcaAgent {
            position: [v[0], v[1]],
            velocity: [v[2], v[3]],
            preferred_velocity: [v[4], v[5]],
            radius: v[6],
            max_speed: v[7],
        }
    });
    let a = agents[3];
    let neighbours: Vec<_> = agents
        .iter()
        .enumerate()
        .filter_map(|(i, a)| (i != 3).then_some(*a))
        .collect();
    let v = orca_velocity(
        &mut Workspace::new(),
        &a,
        &neighbours,
        &[],
        &OrcaParams::default(),
        0.05,
        &mut OrcaScratch::default(),
    )
    .unwrap();
    assert!(v.iter().all(|x| x.is_finite()));
    assert!(v[0] * v[0] + v[1] * v[1] <= a.max_speed * a.max_speed + 1e-6);
}

#[test]
fn finite_wall_endpoint_uses_the_visible_tangent_ray() {
    let mut a = agent(0., 0.);
    a.velocity = [2., 2.];
    a.preferred_velocity = a.velocity;
    a.max_speed = 3.;
    let mut ws = Workspace::new();
    let mut scratch = OrcaScratch::default();
    let mut params = OrcaParams::default();
    params.time_horizon_obstacles = 1.;
    // Tangent from the origin to the radius-.5 circle at (1, 1).
    let leg = 1.75_f32.sqrt();
    let tangent = [(leg - 0.5) / 2., (leg + 0.5) / 2.];
    let along = 2. * (tangent[0] + tangent[1]);
    let expected = tangent.map(|x| x * along);
    for wall in [[1., -1., 1., 1.], [1., 1., 1., -1.]] {
        let v = orca_velocity(&mut ws, &a, &[], &[wall], &params, 0.05, &mut scratch).unwrap();
        assert!((v[0] - expected[0]).abs() < 1e-6);
        assert!((v[1] - expected[1]).abs() < 1e-6);
        // The ray touches the expanded endpoint and clears the finite segment.
        let time = (v[0] + v[1]) / (v[0] * v[0] + v[1] * v[1]);
        let separation = ((time * v[0] - 1.).powi(2) + (time * v[1] - 1.).powi(2)).sqrt();
        assert!((separation - a.radius).abs() < 1e-6);
    }
}
