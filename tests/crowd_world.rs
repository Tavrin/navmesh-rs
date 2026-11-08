#![cfg(feature = "crowd-world")]
use navmesh_rs::crowd::world::*;
use navmesh_rs::crowd::*;
use navmesh_rs::detour::{self, DefaultFilter, NavMesh, NavMeshCreateParams, NavMeshQuery};
use navmesh_rs::{Budget, Error, Limits, SnapshotError, UpstreamFixes, Workspace};

/// Two 18 x 40 rooms joined by a 4 x 4 opening: walls for the boundary phase and a
/// three-polygon corridor for the path phases.
fn rooms() -> NavMesh {
    let n = 0xffff;
    let data = detour::create_nav_mesh_data(&NavMeshCreateParams {
        verts: &[
            0, 0, 0, 0, 0, 40, 18, 0, 40, 18, 0, 22, 18, 0, 18, 18, 0, 0, 22, 0, 18, 22, 0, 22, 22,
            0, 0, 22, 0, 40, 40, 0, 40, 40, 0, 0,
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
        bmax: [40., 1., 40.],
        cs: 1.,
        ch: 1.,
        walkable_height: 2.,
        walkable_radius: 0.4,
        walkable_climb: 0.5,
        build_bv_tree: true,
        ..Default::default()
    })
    .unwrap();
    NavMesh::new_single(data).unwrap().0
}

/// Agent `i`: start in the left room on a 0.9-unit lattice, every third one ORCA,
/// even ones with a point goal in the right room, odd ones with a velocity.
fn spawn(i: usize) -> (AgentParams, [f32; 3], Option<[f32; 3]>, [f32; 3]) {
    let mut p = AgentParams::default();
    p.radius = 0.4;
    p.collision_query_range = 4.8;
    if i.is_multiple_of(3) {
        p.avoidance = AvoidanceMode::Orca(OrcaParams::default());
    }
    let pos = [
        1. + (i % 19) as f32 * 0.9,
        0.,
        2. + (i / 19 % 40) as f32 * 0.9,
    ];
    let goal = i
        .is_multiple_of(2)
        .then_some([30. + (i % 7) as f32, 0., 5. + (i % 29) as f32]);
    (p, pos, goal, [2., 0., 0.3 - (i % 5) as f32 * 0.15])
}

fn world(nav: &NavMesh, n: usize) -> CrowdWorld {
    let mut p = CrowdWorldParams::default();
    p.max_agents = n as u32;
    p.max_heavy_agents = n as u32;
    p.max_radius = 0.4;
    p.fixed_dt = 0.05;
    let mut w = CrowdWorld::new(nav, &p, Limits::UNLIMITED).unwrap();
    for i in 0..n {
        add(nav, &mut w, i);
    }
    w
}

fn add(nav: &NavMesh, w: &mut CrowdWorld, i: usize) -> AgentHandle {
    let (p, pos, goal, vel) = spawn(i);
    let h = w.add_agent(nav, pos, p).unwrap();
    let g = match goal {
        Some(pos) => Goal::Point {
            poly: nearest(nav, pos),
            pos,
        },
        None => Goal::Velocity(vel),
    };
    w.set_goal(nav, h, g).unwrap();
    h
}

fn nearest(nav: &NavMesh, pos: [f32; 3]) -> detour::PolyRef {
    let q = NavMeshQuery::new(nav, 64).unwrap();
    q.find_nearest_poly(&pos, &[1.; 3], &DefaultFilter::default())
        .unwrap()
        .0
}

fn step(nav: &NavMesh, w: &mut CrowdWorld) -> StepReport {
    w.step(nav, &mut Budget::unlimited()).unwrap().into_value()
}

fn bits(v: [f32; 3]) -> [u32; 3] {
    v.map(f32::to_bits)
}

#[test]
fn full_tier_matches_crowd_with_nearest_neighbours() {
    let nav = rooms();
    let n = 48;
    let mut ws = Workspace::new();
    let mut crowd = Crowd::new(&mut ws, &nav, n, 0.4).unwrap();
    crowd.set_fixes(UpstreamFixes::NEAREST_NEIGHBOURS);
    let crowd_add = |crowd: &mut Crowd, ws: &mut Workspace, i: usize| {
        let (p, pos, goal, vel) = spawn(i);
        let id = crowd.add_agent(ws, &nav, pos, p).unwrap();
        match goal {
            Some(g) => crowd
                .request_move_target(&nav, id, nearest(&nav, g), g)
                .unwrap(),
            None => crowd.request_move_velocity(id, vel).unwrap(),
        }
    };
    for i in 0..n {
        crowd_add(&mut crowd, &mut ws, i);
    }
    let mut w = world(&nav, n);
    for t in 0..150 {
        if t == 50 {
            for i in [3, 17, 30] {
                crowd.remove_agent(AgentId(i)).unwrap();
                w.remove_agent(w.handle_at(i as u32).unwrap()).unwrap();
            }
        }
        if t == 80 {
            for i in [5, 6] {
                crowd_add(&mut crowd, &mut ws, i);
                add(&nav, &mut w, i);
            }
        }
        crowd.update(&mut ws, &nav, 0.05).unwrap();
        step(&nav, &mut w);
        let theirs: Vec<_> = crowd
            .agents()
            .iter()
            .enumerate()
            .filter(|(_, a)| a.active)
            .collect();
        let ours: Vec<_> = w.agents().collect();
        assert_eq!(theirs.len(), ours.len(), "tick {t}");
        for ((i, a), v) in theirs.into_iter().zip(ours) {
            let b = v.detail().unwrap();
            assert_eq!(i as u32, v.handle().index(), "tick {t}");
            assert_eq!((a.state, a.target_state), (b.state, b.target_state));
            assert_eq!(bits(a.npos), bits(b.npos), "tick {t} agent {i}");
            assert_eq!(bits(a.vel), bits(b.vel), "tick {t} agent {i}");
            assert_eq!(bits(a.nvel), bits(b.nvel), "tick {t} agent {i}");
            assert_eq!(a.corridor.path(), b.corridor.path(), "tick {t} agent {i}");
            assert_eq!(a.neighbours(), b.neighbours(), "tick {t} agent {i}");
        }
    }
}

#[cfg(feature = "parallel")]
#[test]
fn digests_do_not_depend_on_the_thread_count() {
    let nav = rooms();
    let run = |threads: usize| {
        let pool = rayon::ThreadPoolBuilder::new()
            .num_threads(threads)
            .build()
            .unwrap();
        pool.install(|| {
            let mut w = world(&nav, 400);
            let mut digests = Vec::new();
            for _ in 0..120 {
                step(&nav, &mut w);
                digests.push(w.digest());
            }
            digests
        })
    };
    let one = run(1);
    for threads in [2, 4, 16] {
        assert_eq!(run(threads), one, "{threads} threads");
    }
}

#[test]
fn handles_go_stale_and_the_lowest_index_is_reused() {
    let nav = rooms();
    let mut w = world(&nav, 3);
    let old = w.handle_at(1).unwrap();
    w.remove_agent(old).unwrap();
    assert!(w.agent(old).is_none());
    assert!(w.set_goal(&nav, old, Goal::None).is_err());
    assert!(w.remove_agent(old).is_err());
    let new = add(&nav, &mut w, 1);
    assert_eq!((new.index(), new.generation()), (1, 1));
    assert_ne!(new, old);
    assert!(w.agent(old).is_none());
    assert_eq!(w.len(), 3);
    assert!(matches!(
        w.add_agent(&nav, [5., 0., 5.], AgentParams::default()),
        Err(Error::InvalidParameter(_))
    ));
}

#[test]
fn step_charges_active_agents_before_simulating() {
    let nav = rooms();
    let mut p = CrowdWorldParams::default();
    p.max_agents = 1000;
    p.max_heavy_agents = 1000;
    p.max_radius = 0.4;
    let mut w = CrowdWorld::new(&nav, &p, Limits::UNLIMITED).unwrap();
    for i in 0..10 {
        add(&nav, &mut w, i);
    }
    let need = 10 * STEP_WORK_PER_AGENT + 100;
    let before = w.digest();
    let mut budget = Budget::new(need - 1);
    let out = w.step(&nav, &mut budget).unwrap();
    assert!(!out.is_complete());
    assert_eq!(out.value().work, need);
    assert_eq!((budget.used(), w.digest()), (0, before));
    let mut budget = Budget::new(need);
    let out = w.step(&nav, &mut budget).unwrap();
    assert!(out.is_complete());
    assert_eq!((budget.used(), out.value().agents), (need, 10));
    assert_ne!(w.digest(), before);
}

#[test]
fn snapshot_restores_a_world_that_steps_identically() {
    let nav = rooms();
    let mut w = world(&nav, 48);
    w.remove_agent(w.handle_at(7).unwrap()).unwrap();
    for _ in 0..40 {
        step(&nav, &mut w);
    }
    let bytes = w.to_bytes();
    let mut r = CrowdWorld::from_bytes(&nav, &bytes, Limits::UNLIMITED).unwrap();
    assert_eq!(r.to_bytes(), bytes);
    assert_eq!(r.handle_at(7), None);
    for _ in 0..60 {
        step(&nav, &mut w);
        step(&nav, &mut r);
        assert_eq!(r.digest(), w.digest());
    }
    let err = |b: &[u8]| CrowdWorld::from_bytes(&nav, b, Limits::UNLIMITED).err();
    let snap = |e| Some(Error::InvalidSnapshot(e));
    assert_eq!(
        err(&bytes[..bytes.len() - 1]),
        snap(SnapshotError::TooShort)
    );
    assert_eq!(err(b"DNAV\x01\0\0\0"), snap(SnapshotError::WrongMagic));
    let mut long = bytes.clone();
    long.push(0);
    assert_eq!(err(&long), snap(SnapshotError::TrailingBytes));
}

#[test]
fn snapshot_preserves_in_progress_path_search() {
    let count = 100usize;
    let mut verts = Vec::new();
    for x in 0..=count {
        verts.extend_from_slice(&[x as u16, 0, 0, x as u16, 0, 2]);
    }
    let mut polys = Vec::new();
    for x in 0..count {
        let a = (x * 2) as u16;
        polys.extend_from_slice(&[a, a + 1, a + 3, a + 2]);
        polys.extend_from_slice(&[
            if x == 0 { 0xffff } else { (x - 1) as u16 },
            0xffff,
            if x + 1 == count {
                0xffff
            } else {
                (x + 1) as u16
            },
            0xffff,
        ]);
    }
    let data = detour::create_nav_mesh_data(&NavMeshCreateParams {
        verts: &verts,
        vert_count: (verts.len() / 3) as i32,
        polys: &polys,
        poly_count: count as i32,
        nvp: 4,
        poly_flags: &vec![1; count],
        poly_areas: &vec![0; count],
        bmin: [0.; 3],
        bmax: [100., 1., 2.],
        cs: 1.,
        ch: 1.,
        walkable_height: 2.,
        walkable_radius: 0.4,
        walkable_climb: 0.5,
        ..Default::default()
    })
    .unwrap();
    let nav = NavMesh::new_single(data).unwrap().0;
    let mut p = CrowdWorldParams::default();
    p.path_queue.iterations = 1;
    let mut w = CrowdWorld::new(&nav, &p, Limits::UNLIMITED).unwrap();
    let h = w
        .add_agent(&nav, [0.5, 0., 1.], AgentParams::default())
        .unwrap();
    w.set_goal(
        &nav,
        h,
        Goal::Point {
            poly: nearest(&nav, [99.5, 0., 1.]),
            pos: [99.5, 0., 1.],
        },
    )
    .unwrap();
    step(&nav, &mut w);
    assert_eq!(w.path_queue().pending(), 1);
    let mut filter = DefaultFilter::default();
    filter.area_cost[0] = 0.25;
    w.set_filter(0, filter).unwrap();
    let bytes = w.to_bytes();
    let mut restored = CrowdWorld::from_bytes(&nav, &bytes, Limits::UNLIMITED).unwrap();
    assert_eq!(restored.to_bytes(), bytes);
    for _ in 0..120 {
        step(&nav, &mut w);
        step(&nav, &mut restored);
        assert_eq!(w.digest(), restored.digest());
    }
}

#[test]
fn snapshot_rejects_overflowing_queue_retention() {
    let nav = rooms();
    let mut bytes = world(&nav, 1).to_bytes();
    let start = bytes.len() - 1 - 8 * 54;
    bytes[start..start + 4].copy_from_slice(&1u32.to_le_bytes());
    bytes[start + 45..start + 49].copy_from_slice(&(1u32 << 30).to_le_bytes());
    bytes[start + 49] = 255;
    assert!(matches!(
        CrowdWorld::from_bytes(&nav, &bytes, Limits::UNLIMITED),
        Err(Error::InvalidSnapshot(_))
    ));
}
