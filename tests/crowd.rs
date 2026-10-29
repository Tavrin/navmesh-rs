#![cfg(feature = "crowd")]
use navmesh_rs::crowd::*;
use navmesh_rs::detour::{self, DefaultFilter, NavMesh, NavMeshCreateParams, NavMeshQuery};
use navmesh_rs::{Error, Limits, Workspace};
fn floor() -> NavMesh {
    let n = 0xffff;
    let data = detour::create_nav_mesh_data(&NavMeshCreateParams {
        verts: &[0, 0, 0, 0, 0, 10, 10, 0, 10, 10, 0, 0],
        vert_count: 4,
        polys: &[0, 1, 2, 3, n, n, n, n, n, n, n, n],
        poly_count: 1,
        nvp: 6,
        poly_flags: &[1],
        poly_areas: &[0],
        bmin: [0.; 3],
        bmax: [10., 1., 10.],
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
        c.request_move_target(a, r, [8., 0., 8.]).unwrap();
        c.request_move_target(b, r, [2., 0., 2.]).unwrap();
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
    assert!(c.request_move_target(a, 0, [0.; 3]).is_err());
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
    assert!(
        c.update_agent_parameters(
            a,
            AgentParams {
                query_filter_type: 16,
                ..Default::default()
            }
        )
        .is_err()
    );
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
    let p = ObstacleAvoidanceParams {
        grid_size: 1,
        ..Default::default()
    };
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
    let n = grid.query_items([-2., -2., 2., 2.], &mut ids).unwrap();
    assert_eq!(n, 2);
    assert_eq!(&ids[..n], &[7, 9]);
    assert!(grid.add_item(1, [1e20; 4]).is_err());
    grid.clear();
    assert_eq!(grid.query_items([-2., -2., 2., 2.], &mut ids).unwrap(), 0);
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
        .request(r, r, [2., 0., 2.], [8., 0., 8.], 0)
        .unwrap()
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
    assert!(queue.request(0, r, [0.; 3], [0.; 3], 0).is_err());
    assert!(queue.request(r, r, [0.; 3], [0.; 3], 16).is_err());
    let mut handles = Vec::new();
    for _ in 0..8 {
        handles.push(
            queue
                .request(r, r, [2., 0., 2.], [3., 0., 3.], 0)
                .unwrap()
                .unwrap(),
        );
    }
    assert_eq!(
        queue.request(r, r, [2., 0., 2.], [3., 0., 3.], 0).unwrap(),
        None
    );
    let filters = core::array::from_fn(|_| DefaultFilter::default());
    queue.update(&mut ws, &nav, 10, &filters).unwrap();
    for _ in 0..3 {
        queue.update(&mut ws, &nav, 10, &filters).unwrap();
    }
    for h in handles {
        assert_ne!(queue.status(h) & detour::Status::FAILURE, 0);
    }
    assert!(
        queue
            .request(r, r, [2., 0., 2.], [3., 0., 3.], 0)
            .unwrap()
            .is_some()
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
