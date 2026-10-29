#![cfg(feature = "crowd")]
use navmesh_rs::crowd::*;
use navmesh_rs::detour::{self, DefaultFilter, NavMesh, NavMeshCreateParams, NavMeshQuery};
use navmesh_rs::{Error, Limits, Workspace};
fn floor() -> NavMesh {
    floor_width(10)
}
fn floor_width(width: u16) -> NavMesh {
    let n = 0xffff;
    let data = detour::create_nav_mesh_data(&NavMeshCreateParams {
        verts: &[0, 0, 0, 0, 0, 10, width, 0, 10, width, 0, 0],
        vert_count: 4,
        polys: &[0, 1, 2, 3, n, n, n, n, n, n, n, n],
        poly_count: 1,
        nvp: 6,
        poly_flags: &[1],
        poly_areas: &[0],
        bmin: [0.; 3],
        bmax: [f32::from(width), 1., 10.],
        walkable_height: 2.,
        walkable_radius: 0.6,
        walkable_climb: 0.5,
        cs: 1.,
        ch: 1.,
        build_bv_tree: true,
        ..Default::default()
    })
    .unwrap();
    NavMesh::new_single(data).unwrap().0
}
#[test]
fn trajectories_repeat_and_target_reset_stops() {
    fn run() -> Vec<u32> {
        let nav = floor();
        let mut ws = Workspace::new();
        let mut c = Crowd::new(&mut ws, &nav, 4, 0.6).unwrap();
        let a = c
            .add_agent(&mut ws, &nav, [2., 0., 2.], AgentParams::default())
            .unwrap();
        let b = c
            .add_agent(&mut ws, &nav, [8., 0., 8.], AgentParams::default())
            .unwrap();
        let q = NavMeshQuery::new(&nav, 32).unwrap();
        let r = q
            .find_nearest_poly(&[8., 0., 8.], &[1.; 3], &DefaultFilter::default())
            .unwrap()
            .0;
        c.request_move_target(&nav, a, r, [8., 0., 8.]).unwrap();
        c.request_move_target(&nav, b, r, [2., 0., 2.]).unwrap();
        let mut bits = Vec::new();
        for t in 0..120 {
            if t == 60 {
                c.reset_move_target(a).unwrap();
            }
            c.update(&mut ws, &nav, 1. / 30.).unwrap();
            for a in c.agents().iter().filter(|a| a.active) {
                assert!(a.npos.iter().chain(&a.vel).all(|v| v.is_finite()));
                bits.extend(a.npos.map(f32::to_bits));
                bits.extend(a.vel.map(f32::to_bits));
            }
        }
        assert_eq!(c.agent(a).unwrap().vel, [0.; 3]);
        assert!(c.agent(b).unwrap().npos[0] < 8.);
        bits
    }
    assert_eq!(run(), run());
}
#[test]
fn invalid_inputs_and_limits_are_errors() {
    let nav = floor();
    let mut ws = Workspace::new();
    assert!(Crowd::new(&mut ws, &nav, 0, 0.6).is_err());
    assert!(Crowd::new(&mut ws, &nav, 2, f32::NAN).is_err());
    ws.set_limits(Limits {
        max_bytes: 1,
        ..Limits::UNLIMITED
    });
    assert!(matches!(
        Crowd::new(&mut ws, &nav, 2, 0.6),
        Err(Error::LimitExceeded)
    ));
    ws.set_limits(Limits::UNLIMITED);
    let mut c = Crowd::new(&mut ws, &nav, 1, 0.6).unwrap();
    assert!(
        c.add_agent(&mut ws, &nav, [f32::NAN; 3], AgentParams::default())
            .is_err()
    );
    let a = c
        .add_agent(&mut ws, &nav, [2., 0., 2.], AgentParams::default())
        .unwrap();
    assert!(
        c.add_agent(&mut ws, &nav, [3., 0., 3.], AgentParams::default())
            .is_err()
    );
    assert!(c.request_move_target(&nav, a, 0, [0.; 3]).is_err());
    assert!(c.request_move_velocity(a, [f32::INFINITY; 3]).is_err());
    assert!(c.update(&mut ws, &nav, f32::NAN).is_err());
    assert!(c.update(&mut ws, &nav, -1.).is_err());
    c.update(&mut ws, &nav, 0.).unwrap();
    ws.set_limits(Limits {
        max_work: 1,
        ..Limits::UNLIMITED
    });
    assert_eq!(c.update(&mut ws, &nav, 0.1), Err(Error::LimitExceeded));
    c.remove_agent(a).unwrap();
    assert!(c.reset_move_target(a).is_err());
    ws.set_limits(Limits::UNLIMITED);
    assert_eq!(
        a,
        c.add_agent(&mut ws, &nav, [2., 0., 2.], AgentParams::default())
            .unwrap()
    );
    let mut invalid = AgentParams::default();
    invalid.query_filter_type = 16;
    assert!(c.update_agent_parameters(a, invalid).is_err());
}
#[test]
fn tile_replacement_recovers_refs() {
    let mut nav = floor();
    let mut ws = Workspace::new();
    let mut c = Crowd::new(&mut ws, &nav, 2, 0.6).unwrap();
    let a = c
        .add_agent(&mut ws, &nav, [2., 0., 2.], AgentParams::default())
        .unwrap();
    let old = c.agent(a).unwrap().corridor.first_poly();
    let data = nav.remove_tile(nav.tile_ref(0)).unwrap();
    nav.add_tile(data).unwrap();
    c.update(&mut ws, &nav, 1. / 30.).unwrap();
    let a = c.agent(a).unwrap();
    assert_ne!(a.corridor.first_poly(), old);
    assert_eq!(a.state, AgentState::Walking);
}
#[test]
fn avoidance_grid_adaptive_and_all_presets() {
    let mut ws = Workspace::new();
    let mut q = ObstacleAvoidanceQuery::new(&mut ws, 6, 8).unwrap();
    q.add_circle([2., 0., 0.], 0.5, [-1., 0., 0.], [-1., 0., 0.])
        .unwrap();
    q.add_segment([0., 0., 2.], [4., 0., 2.]).unwrap();
    for level in 0..4 {
        let p = ObstacleAvoidanceParams::quality(level).unwrap();
        let (v, n) = q
            .sample_velocity_adaptive(&mut ws, [0.; 3], 0.5, 2., [1., 0., 0.], [2., 0., 0.], &p)
            .unwrap();
        assert!(n > 0);
        assert!(v.iter().all(|v| v.is_finite()));
        let (v, n) = q
            .sample_velocity_grid(&mut ws, [0.; 3], 0.5, 2., [1., 0., 0.], [2., 0., 0.], &p)
            .unwrap();
        assert!(n > 0);
        assert!(v.iter().all(|v| v.is_finite()));
    }
    assert!(q.add_circle([0.; 3], f32::NAN, [0.; 3], [0.; 3]).is_err());
    let mut p = ObstacleAvoidanceParams::default();
    p.grid_size = 1;
    assert!(
        q.sample_velocity_grid(&mut ws, [0.; 3], 0.5, 2., [0.; 3], [0.; 3], &p)
            .is_err()
    );
}
#[test]
fn grid_deduplicates_and_checks_coordinate_limits() {
    let mut ws = Workspace::new();
    let mut grid = ProximityGrid::new(&mut ws, 32, 1.).unwrap();
    grid.add_item(7, [-1., -1., 1., 1.]).unwrap();
    grid.add_item(9, [0., 0., 0., 0.]).unwrap();
    let mut ids = [0; 8];
    let n = grid
        .query_items(&mut ws, [-2., -2., 2., 2.], &mut ids)
        .unwrap();
    assert_eq!(n, 2);
    assert_eq!(&ids[..n], &[7, 9]);
    assert!(grid.add_item(1, [1e20; 4]).is_err());
    grid.clear();
    assert_eq!(
        grid.query_items(&mut ws, [-2., -2., 2., 2.], &mut ids)
            .unwrap(),
        0
    );
}
#[test]
fn corridor_and_queue_validate_and_move() {
    let nav = floor();
    let mut ws = Workspace::new();
    let mut q = NavMeshQuery::new(&nav, 64).unwrap();
    let f = DefaultFilter::default();
    let r = q.find_nearest_poly(&[2., 0., 2.], &[1.; 3], &f).unwrap().0;
    let mut path = PathCorridor::new(&mut ws, 16).unwrap();
    assert!(path.set_corridor([0.; 3], &[]).is_err());
    path.reset(r, [2., 0., 2.]).unwrap();
    path.set_corridor([8., 0., 8.], &[r]).unwrap();
    path.move_position([3., 0., 3.], &mut q, &f).unwrap();
    path.move_target_position([9., 0., 9.], &mut q, &f).unwrap();
    assert_eq!(path.position(), [3., 0., 3.]);
    let mut queue = PathQueue::new(&mut ws, &nav, 16, 64).unwrap();
    let filters = core::array::from_fn(|_| DefaultFilter::default());
    let h = queue
        .request(&nav, r, r, [2., 0., 2.], [8., 0., 8.], 0)
        .unwrap();
    queue.update(&mut ws, &nav, 10, &filters).unwrap();
    let mut result = Vec::new();
    queue.take_result(h, &mut result).unwrap();
    assert_eq!(result, [r]);
    assert!(queue.take_result(h, &mut result).is_err());
}

#[test]
fn queue_capacity_expiry_and_invalid_requests() {
    let nav = floor();
    let mut ws = Workspace::new();
    let q = NavMeshQuery::new(&nav, 64).unwrap();
    let r = q
        .find_nearest_poly(&[2., 0., 2.], &[1.; 3], &DefaultFilter::default())
        .unwrap()
        .0;
    let mut queue = PathQueue::new(&mut ws, &nav, 16, 64).unwrap();
    assert!(queue.request(&nav, 0, r, [0.; 3], [0.; 3], 0).is_err());
    assert!(queue.request(&nav, r, r, [0.; 3], [0.; 3], 16).is_err());
    let mut handles = Vec::new();
    for _ in 0..8 {
        handles.push(
            queue
                .request(&nav, r, r, [2., 0., 2.], [3., 0., 3.], 0)
                .unwrap(),
        );
    }
    assert_eq!(
        queue.request(&nav, r, r, [2., 0., 2.], [3., 0., 3.], 0),
        Err(Error::BufferTooSmall)
    );
    let filters = core::array::from_fn(|_| DefaultFilter::default());
    queue.update(&mut ws, &nav, 10, &filters).unwrap();
    for _ in 0..3 {
        queue.update(&mut ws, &nav, 10, &filters).unwrap();
    }
    for h in handles {
        assert_ne!(queue.status(h) & detour::Status::FAILURE, 0);
    }
    assert_ne!(
        queue
            .request(&nav, r, r, [2., 0., 2.], [3., 0., 3.], 0)
            .unwrap(),
        0
    );
}

#[test]
fn standalone_helper_limits_reset_each_call() {
    let mut ws = Workspace::new();
    let _first = ObstacleAvoidanceQuery::new(&mut ws, 1, 0).unwrap();
    let bytes = ws.usage().bytes;
    ws.set_limits(Limits {
        max_bytes: bytes,
        ..Limits::UNLIMITED
    });
    let mut avoidance = ObstacleAvoidanceQuery::new(&mut ws, 1, 0).unwrap();
    avoidance
        .add_circle([2., 0., 0.], 0.5, [-1., 0., 0.], [-1., 0., 0.])
        .unwrap();
    let p = ObstacleAvoidanceParams::quality(0).unwrap();
    avoidance
        .sample_velocity_adaptive(&mut ws, [0.; 3], 0.5, 2., [1., 0., 0.], [2., 0., 0.], &p)
        .unwrap();
    let work = ws.usage().work;
    assert!(work > 0);
    ws.set_limits(Limits {
        max_work: work,
        ..Limits::UNLIMITED
    });
    for _ in 0..2 {
        avoidance
            .sample_velocity_adaptive(&mut ws, [0.; 3], 0.5, 2., [1., 0., 0.], [2., 0., 0.], &p)
            .unwrap();
    }
    ws.set_limits(Limits::UNLIMITED);
    let nav = floor();
    let mut queue = PathQueue::new(&mut ws, &nav, 16, 64).unwrap();
    let filters = core::array::from_fn(|_| DefaultFilter::default());
    queue.update(&mut ws, &nav, 10, &filters).unwrap();
    let work = ws.usage().work;
    assert!(work > 0);
    ws.set_limits(Limits {
        max_work: work,
        ..Limits::UNLIMITED
    });
    for _ in 0..2 {
        queue.update(&mut ws, &nav, 10, &filters).unwrap();
    }
}

// Two disconnected quads with a bidirectional link across their gap.
fn islands() -> NavMesh {
    let n = 0xffff;
    let data = detour::create_nav_mesh_data(&NavMeshCreateParams {
        verts: &[
            0, 0, 0, 0, 0, 10, 10, 0, 10, 10, 0, 0, 20, 0, 0, 20, 0, 10, 30, 0, 10, 30, 0, 0,
        ],
        vert_count: 8,
        polys: &[
            0, 1, 2, 3, n, n, n, n, n, n, n, n, 4, 5, 6, 7, n, n, n, n, n, n, n, n,
        ],
        poly_count: 2,
        nvp: 6,
        poly_flags: &[1, 1],
        poly_areas: &[0, 0],
        off_mesh_con_verts: &[9., 0., 5., 21., 0., 5.],
        off_mesh_con_rad: &[1.],
        off_mesh_con_flags: &[1],
        off_mesh_con_areas: &[0],
        off_mesh_con_dir: &[1],
        off_mesh_con_count: 1,
        bmin: [0.; 3],
        bmax: [30., 1., 10.],
        walkable_height: 2.,
        walkable_radius: 0.6,
        walkable_climb: 0.5,
        cs: 1.,
        ch: 1.,
        build_bv_tree: true,
        ..Default::default()
    })
    .unwrap();
    NavMesh::new_single(data).unwrap().0
}

#[test]
fn remove_mid_link_and_readd_clears_animation_and_target() {
    let nav = islands();
    let mut ws = Workspace::new();
    let mut c = Crowd::new(&mut ws, &nav, 1, 0.6).unwrap();
    let id = c
        .add_agent(&mut ws, &nav, [8., 0., 5.], AgentParams::default())
        .unwrap();
    let q = NavMeshQuery::new(&nav, 32).unwrap();
    let target = [25., 0., 5.];
    let r = q
        .find_nearest_poly(&target, &[1.; 3], &DefaultFilter::default())
        .unwrap()
        .0;
    c.request_move_target(&nav, id, r, target).unwrap();
    c.update(&mut ws, &nav, 0.1).unwrap();
    assert_eq!(
        c.agent(id).unwrap().state,
        AgentState::Offmesh,
        "{:?}",
        c.agent(id)
    );
    c.remove_agent(id).unwrap();
    let a = &c.agents()[id.0];
    assert!(!a.active);
    assert_eq!(a.state, AgentState::Invalid);
    assert_eq!(a.target_state, TargetState::None);
    assert_eq!(a.target_ref, 0);
    assert_eq!(a.target_pos, [0.; 3]);
    assert!(a.corners.is_empty());
    assert!(a.corridor.path().is_empty());
    let elsewhere = [2., 0., 2.];
    assert_eq!(
        c.add_agent(&mut ws, &nav, elsewhere, AgentParams::default())
            .unwrap(),
        id
    );
    for _ in 0..30 {
        c.update(&mut ws, &nav, 0.1).unwrap();
        let a = c.agent(id).unwrap();
        assert_eq!(a.state, AgentState::Walking);
        assert_eq!(a.target_state, TargetState::None);
        assert_eq!(a.npos, elsewhere);
        assert_eq!(a.vel, [0.; 3]);
        assert_eq!(a.target_ref, 0);
        assert_eq!(a.target_pos, [0.; 3]);
        assert!(a.corners.is_empty());
    }
}

#[test]
fn colliding_grid_entries_are_charged_even_without_hits() {
    let mut ws = Workspace::new();
    let mut grid = ProximityGrid::new(&mut ws, 64, 1.).unwrap();
    // All x coordinates are multiples of the bucket count, hence hash to zero.
    for id in 0..64 {
        let x = f32::from(id) * 64.;
        grid.add_item(id, [x, 0., x, 0.]).unwrap();
    }
    let mut ids = [0; 8];
    let absent = [4096., 0., 4096., 0.];
    ws.set_limits(Limits {
        max_work: 64,
        ..Limits::UNLIMITED
    });
    assert_eq!(
        grid.query_items(&mut ws, absent, &mut ids),
        Err(Error::LimitExceeded)
    );
    assert_eq!(ws.usage().work, 65);
    ws.set_limits(Limits {
        max_work: 65,
        ..Limits::UNLIMITED
    });
    // Repeated standalone calls each reset their budget.
    for _ in 0..2 {
        assert_eq!(grid.query_items(&mut ws, absent, &mut ids).unwrap(), 0);
        assert_eq!(ws.usage().work, 65); // One cell, all 64 chain entries.
    }
    assert_eq!(grid.query_items(&mut ws, [0.; 4], &mut ids).unwrap(), 1);
    assert_eq!(ids[0], 0);
    assert_eq!(ws.usage().work, 65);
}

#[test]
fn stale_and_fabricated_targets_are_rejected_without_changing_request() {
    let mut nav = floor();
    let mut ws = Workspace::new();
    let mut c = Crowd::new(&mut ws, &nav, 1, 0.6).unwrap();
    let id = c
        .add_agent(&mut ws, &nav, [2., 0., 2.], AgentParams::default())
        .unwrap();
    let old = c.agent(id).unwrap().corridor.first_poly();
    let target = [8., 0., 8.];
    c.request_move_target(&nav, id, old, target).unwrap();
    assert_eq!(
        c.request_move_target(&nav, id, !0, [1.; 3]),
        Err(Error::InvalidRef)
    );
    let data = nav.remove_tile(nav.tile_ref(0)).unwrap();
    nav.add_tile(data).unwrap();
    assert_eq!(
        c.request_move_target(&nav, id, old, [1.; 3]),
        Err(Error::InvalidRef)
    );
    let a = c.agent(id).unwrap();
    assert_eq!(a.target_ref, old);
    assert_eq!(a.target_pos, target);
    assert_eq!(a.target_state, TargetState::Requesting);
    c.update(&mut ws, &nav, 0.1).unwrap();
    assert!(nav.is_valid_poly_ref(c.agent(id).unwrap().target_ref));
    let mut queue = PathQueue::new(&mut ws, &nav, 16, 32).unwrap();
    let r = c.agent(id).unwrap().target_ref;
    for (start, end) in [(old, r), (r, !0)] {
        assert_eq!(
            queue.request(&nav, start, end, [0.; 3], [0.; 3], 0),
            Err(Error::InvalidRef)
        );
    }
}

#[test]
fn helper_capacity_exhaustion_is_an_error_and_grid_insert_is_atomic() {
    let mut ws = Workspace::new();
    let mut grid = ProximityGrid::new(&mut ws, 2, 1.).unwrap();
    let empty_bounds = grid.bounds();
    assert_eq!(
        grid.add_item(7, [0., 0., 2., 0.]),
        Err(Error::BufferTooSmall)
    );
    assert_eq!(grid.bounds(), empty_bounds);
    grid.add_item(9, [0., 0., 1., 0.]).unwrap();
    assert_eq!(grid.add_item(10, [9.; 4]), Err(Error::BufferTooSmall));
    assert_eq!(grid.bounds(), [0, 0, 1, 0]);
    let mut ids = [0; 8];
    let n = grid
        .query_items(&mut ws, [0., 0., 9., 9.], &mut ids)
        .unwrap();
    assert_eq!(&ids[..n], &[9]);
    let mut avoidance = ObstacleAvoidanceQuery::new(&mut ws, 1, 1).unwrap();
    avoidance.add_circle([0.; 3], 1., [0.; 3], [0.; 3]).unwrap();
    avoidance.add_segment([0.; 3], [1.; 3]).unwrap();
    assert_eq!(
        avoidance.add_circle([0.; 3], 1., [0.; 3], [0.; 3]),
        Err(Error::BufferTooSmall)
    );
    assert_eq!(
        avoidance.add_segment([0.; 3], [1.; 3]),
        Err(Error::BufferTooSmall)
    );
    avoidance.reset();
    avoidance.add_circle([0.; 3], 1., [0.; 3], [0.; 3]).unwrap();
    avoidance.add_segment([0.; 3], [1.; 3]).unwrap();
}

#[test]
fn zero_speed_is_rejected_at_add_and_at_link_entrance() {
    let nav = islands();
    let mut ws = Workspace::new();
    let mut c = Crowd::new(&mut ws, &nav, 1, 0.6).unwrap();
    let mut params = AgentParams::default();
    params.max_speed = 0.;
    assert!(matches!(
        c.add_agent(&mut ws, &nav, [8., 0., 5.], params),
        Err(Error::InvalidParameter(_))
    ));
    let id = c
        .add_agent(&mut ws, &nav, [8., 0., 5.], AgentParams::default())
        .unwrap();
    assert!(matches!(
        c.update_agent_parameters(id, params),
        Err(Error::InvalidParameter(_))
    ));
    assert_eq!(
        c.agent(id).unwrap().params.max_speed,
        AgentParams::default().max_speed
    );
    let q = NavMeshQuery::new(&nav, 32).unwrap();
    let target = [25., 0., 5.];
    let r = q
        .find_nearest_poly(&target, &[1.; 3], &DefaultFilter::default())
        .unwrap()
        .0;
    c.request_move_target(&nav, id, r, target).unwrap();
    c.update(&mut ws, &nav, 0.1).unwrap();
    assert_eq!(
        c.agent(id).unwrap().state,
        AgentState::Offmesh,
        "{:?}",
        c.agent(id)
    );
    for _ in 0..30 {
        c.update(&mut ws, &nav, 0.1).unwrap();
    }
    assert_eq!(c.agent(id).unwrap().state, AgentState::Walking);
    assert!(c.agent(id).unwrap().npos[0] > 21.);
}

#[test]
fn non_exhaustive_params_support_default_and_preset_customization() {
    let nav = floor();
    let mut ws = Workspace::new();
    let mut c = Crowd::new(&mut ws, &nav, 1, 0.6).unwrap();
    let mut params = AgentParams::default();
    params.radius = 0.5;
    params.max_speed = 2.;
    let id = c.add_agent(&mut ws, &nav, [2., 0., 2.], params).unwrap();
    let mut avoidance = ObstacleAvoidanceParams::quality(0).unwrap();
    avoidance.adaptive_depth = 2;
    c.set_obstacle_avoidance_params(0, avoidance).unwrap();
    assert_eq!(c.agent(id).unwrap().params, params);
    assert_eq!(c.obstacle_avoidance_params(0), Some(&avoidance));
    // Downstream matches must allow future states.
    assert!(matches!(c.agent(id).unwrap().state, AgentState::Walking));
}

#[test]
fn even_adaptive_divisions_are_finite_and_repeatable() {
    let mut ws = Workspace::new();
    let mut q = ObstacleAvoidanceQuery::new(&mut ws, 1, 1).unwrap();
    q.add_circle([2., 0., 0.], 0.5, [-1., 0., 0.], [-1., 0., 0.])
        .unwrap();
    q.add_segment([0., 0., 2.], [4., 0., 2.]).unwrap();
    for divs in (2..=32).step_by(2) {
        for rings in 1..=4 {
            for depth in [1, 3, 32] {
                let mut p = ObstacleAvoidanceParams::default();
                p.adaptive_divs = divs;
                p.adaptive_rings = rings;
                p.adaptive_depth = depth;
                let mut sample = || {
                    q.sample_velocity_adaptive(
                        &mut ws,
                        [0.; 3],
                        0.5,
                        2.,
                        [1., 0., 0.],
                        [2., 0., 0.],
                        &p,
                    )
                    .unwrap()
                };
                let (v, n) = sample();
                let (repeat, rn) = sample();
                assert_eq!(v.map(f32::to_bits), repeat.map(f32::to_bits));
                assert_eq!(n, rn);
                assert!(v.iter().all(|v| v.is_finite()));
                assert!(
                    n > 0 && n <= (usize::from(divs) * usize::from(rings) + 1) * usize::from(depth)
                );
                assert!(v[0] * v[0] + v[2] * v[2] <= 2.001 * 2.001);
            }
        }
    }
}

#[test]
fn crowd_tick_charges_long_collision_chains_to_its_aggregate_budget() {
    let nav = floor_width(17000);
    let mut ws = Workspace::new();
    // A cell size of one, and 256 buckets for 64 agents. Each agent touches
    // a different cell in the same bucket, with no actual neighbours.
    let mut c = Crowd::new(&mut ws, &nav, 64, 1. / 3.).unwrap();
    let mut params = AgentParams::default();
    params.radius = 0.01;
    params.collision_query_range = 0.01;
    params.update_flags = 0;
    for id in 0..64 {
        let x = id as f32 * 256. + 0.5;
        let id = c.add_agent(&mut ws, &nav, [x, 0., 0.5], params).unwrap();
        assert_eq!(c.agent(id).unwrap().state, AgentState::Walking);
    }
    // Enough for the precharge, path queue and all queried cells, but no
    // chain visits. The old cell-only accounting let this tick succeed.
    ws.set_limits(Limits {
        max_work: 64 * 4096 + 100 + 64,
        ..Limits::UNLIMITED
    });
    assert_eq!(c.update(&mut ws, &nav, 0.1), Err(Error::LimitExceeded));
    ws.set_limits(Limits::UNLIMITED);
    c.update(&mut ws, &nav, 0.1).unwrap();
    assert_eq!(ws.usage().work, 64 * 4096 + 100 + 64 * (1 + 64));
}
