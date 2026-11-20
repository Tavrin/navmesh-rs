#![cfg(all(feature = "crowd-world", feature = "dynamic"))]

use navmesh_rs::crowd::world::{AgentHandle, CrowdWorld, CrowdWorldParams, Goal, StepReport};
use navmesh_rs::crowd::{AgentParams, Crowd, TargetState};
use navmesh_rs::detour::*;
use navmesh_rs::dynamic::{DynamicLayer, Effect, ModifierId, ModifierSpec, Selector};
use navmesh_rs::{Budget, Budgeted, Error, Limits, SnapshotError, Workspace};

const GATE: ModifierId = ModifierId(0);
const DOOR_X: i32 = 32;
const LINK: LinkId = LinkId(0);
const START: [f32; 3] = [2., 0., 5.];
const GOAL: [f32; 3] = [395., 0., 5.];

fn quad(x: i32, z: i32, flags: u16) -> Vec<u8> {
    let (ox, oz) = (x as f32 * 10., z as f32 * 10.);
    create_nav_mesh_data_with_spare_links(
        &NavMeshCreateParams {
            verts: &[0, 0, 0, 0, 0, 10, 10, 0, 10, 10, 0, 0],
            vert_count: 4,
            polys: &[0, 1, 2, 3, 0x8000, 0x8001, 0x8002, 0x8003],
            poly_count: 1,
            nvp: 4,
            poly_flags: &[flags],
            poly_areas: &[0],
            tile_x: x,
            tile_y: z,
            bmin: [ox, 0., oz],
            bmax: [ox + 10., 1., oz + 10.],
            cs: 1.,
            ch: 1.,
            walkable_climb: 0.5,
            build_bv_tree: true,
            ..Default::default()
        },
        2,
    )
    .unwrap()
}

fn scene(doorway: bool, detour: bool) -> NavMesh {
    let mut nav = NavMesh::new(&NavMeshParams {
        orig: [0.; 3],
        tile_width: 10.,
        tile_height: 10.,
        max_tiles: 64,
        max_polys: 64,
    })
    .unwrap();
    for x in 0..40 {
        if x != DOOR_X || doorway {
            nav.add_tile(quad(x, 0, 1)).unwrap();
        }
    }
    if detour {
        for x in DOOR_X - 1..=DOOR_X + 1 {
            nav.add_tile(quad(x, 1, 1)).unwrap();
        }
    }
    nav
}

fn install_gate(nav: &mut NavMesh) {
    let x = DOOR_X as f32 * 10.;
    let mut layer = DynamicLayer::new(1, Limits::UNLIMITED).unwrap();
    layer
        .upsert(ModifierSpec::new(
            GATE,
            Selector::Prism {
                verts_xz: vec![[x, 0.], [x + 10., 0.], [x + 10., 10.], [x, 10.]],
                ymin: -1.,
                ymax: 1.,
                area: Some(0),
            },
            Effect::ClearFlags(u16::MAX),
        ))
        .unwrap();
    layer.set_enabled(GATE, false).unwrap();
    nav.set_dynamic_layer(Some(layer));
}

fn tile(nav: &NavMesh, x: i32, z: i32, layer: i32) -> PolyRef {
    nav.tile_ref(nav.tile_index_at(x, z, layer).unwrap())
}

fn npc() -> AgentParams {
    let mut p = AgentParams::default();
    p.update_flags = 0;
    p
}

fn add(nav: &NavMesh, world: &mut CrowdWorld, pos: [f32; 3]) -> AgentHandle {
    let a = world.add_agent(nav, pos, npc()).unwrap();
    world
        .set_goal(
            nav,
            a,
            Goal::Point {
                poly: tile(nav, 39, 0, 0),
                pos: GOAL,
            },
        )
        .unwrap();
    a
}

fn step(world: &mut CrowdWorld, nav: &NavMesh) -> StepReport {
    let Budgeted::Complete(report) = world.step(nav, &mut Budget::unlimited()).unwrap() else {
        panic!("unlimited step must complete");
    };
    report
}

fn world(nav: &NavMesh) -> CrowdWorld {
    CrowdWorld::new(nav, &CrowdWorldParams::default(), Limits::UNLIMITED).unwrap()
}

fn agent(world: &CrowdWorld, a: AgentHandle) -> &navmesh_rs::crowd::CrowdAgent {
    world.agent(a).unwrap().detail().unwrap()
}

#[test]
fn distant_gate_takes_detour_then_reuses_reopened_route_in_one_step() {
    let mut nav = scene(true, true);
    install_gate(&mut nav);
    let door = tile(&nav, DOOR_X, 0, 0);
    let mut world = world(&nav);
    let a = add(&nav, &mut world, START);
    assert_eq!(step(&mut world, &nav).replans, 0);
    let route = agent(&world, a).corridor.path();
    assert!(route.iter().position(|&r| r == door).unwrap() >= 30);

    let before = nav.change_epoch();
    nav.set_modifier_enabled(GATE, true).unwrap();
    assert!(nav.tile_change_stamp(door).unwrap() > before);
    assert!(nav.tile_change_stamp(tile(&nav, DOOR_X, 1, 0)).unwrap() <= before);
    assert_eq!(step(&mut world, &nav).replans, 1);
    let state = agent(&world, a);
    assert_eq!(state.target_state, TargetState::Valid);
    assert!(state.corridor.path().contains(&tile(&nav, DOOR_X, 1, 0)));
    assert!(!state.corridor.path().contains(&door));

    nav.set_modifier_enabled(GATE, false).unwrap();
    assert_eq!(step(&mut world, &nav).replans, 1);
    assert!(agent(&world, a).corridor.path().contains(&door));
    assert_eq!(step(&mut world, &nav).replans, 0);
}

#[test]
fn crowd_flag_controls_distant_replanning() {
    for enabled in [false, true] {
        let mut nav = scene(true, false);
        install_gate(&mut nav);
        let mut ws = Workspace::new();
        let mut crowd = Crowd::new(&mut ws, &nav, 1, 0.6).unwrap();
        crowd.set_stamp_revalidation(enabled);
        let a = crowd.add_agent(&mut ws, &nav, START, npc()).unwrap();
        crowd
            .request_move_target(&nav, a, tile(&nav, 39, 0, 0), GOAL)
            .unwrap();
        crowd.update(&mut ws, &nav, 0.05).unwrap();
        let door = tile(&nav, DOOR_X, 0, 0);
        assert!(crowd.agent(a).unwrap().corridor.path().contains(&door));
        nav.set_modifier_enabled(GATE, true).unwrap();
        crowd.update(&mut ws, &nav, 0.05).unwrap();
        let state = crowd.agent(a).unwrap();
        assert_eq!(state.corridor.path().contains(&door), !enabled);
        if enabled {
            assert_eq!(state.target_state, TargetState::Failed);
            assert_eq!(state.vel, [0.; 3]);
            assert_eq!(state.dvel, [0.; 3]);
        }
        nav.set_modifier_enabled(GATE, false).unwrap();
        crowd.update(&mut ws, &nav, 0.05).unwrap();
        assert!(crowd.agent(a).unwrap().corridor.path().contains(&door));
    }
}

#[test]
fn runtime_link_is_avoided_when_disabled_mid_route() {
    let mut nav = scene(false, false);
    nav.set_link_layer(Some(1)).unwrap();
    let before = nav.change_epoch();
    let start = tile(&nav, DOOR_X - 1, 0, 0);
    let end = tile(&nav, DOOR_X + 1, 0, 0);
    let spec = LinkSpec::new(LINK, [318., 0., 5.], [332., 0., 5.], 0.6);
    assert!(nav.add_runtime_link(spec, true).unwrap().is_traversable());
    let link = tile(&nav, DOOR_X - 1, 0, 1);
    for r in [start, link, end] {
        assert!(nav.tile_change_stamp(r).unwrap() > before);
    }
    let mut world = world(&nav);
    let a = add(&nav, &mut world, START);
    step(&mut world, &nav);
    assert!(agent(&world, a).corridor.path().contains(&link));

    let before = nav.change_epoch();
    nav.set_runtime_link_enabled(LINK, false).unwrap();
    for r in [start, link, end] {
        assert!(nav.tile_change_stamp(r).unwrap() > before);
    }
    assert_eq!(step(&mut world, &nav).replans, 1);
    assert_eq!(agent(&world, a).target_state, TargetState::Failed);
    assert!(!agent(&world, a).corridor.path().contains(&link));

    nav.set_runtime_link_enabled(LINK, true).unwrap();
    assert_eq!(step(&mut world, &nav).replans, 1);
    assert!(agent(&world, a).corridor.path().contains(&link));
    let before = nav.change_epoch();
    nav.remove_runtime_link(LINK).unwrap();
    assert!(nav.tile_change_stamp(start).unwrap() > before);
    assert!(nav.tile_change_stamp(end).unwrap() > before);
    assert_eq!(nav.tile_change_stamp(link), None);
    assert_eq!(step(&mut world, &nav).replans, 1);
    assert_eq!(agent(&world, a).target_state, TargetState::Failed);
}

#[test]
fn replacement_repairs_refs_ahead_without_affecting_other_tiles() {
    let mut nav = scene(true, false);
    let old = tile(&nav, DOOR_X, 0, 0);
    let start = tile(&nav, 0, 0, 0);
    let stamp = nav.tile_change_stamp(start);
    let mut world = world(&nav);
    let a = add(&nav, &mut world, START);
    step(&mut world, &nav);
    assert!(agent(&world, a).corridor.path().contains(&old));

    let before = nav.change_epoch();
    let prepared = prepare_tile(nav.params(), Limits::UNLIMITED, quad(DOOR_X, 0, 1)).unwrap();
    assert!(matches!(
        nav.replace_tile(prepared).unwrap(),
        Replaced::Unchanged(_)
    ));
    assert_eq!(nav.change_epoch(), before);

    let prepared = prepare_tile(nav.params(), Limits::UNLIMITED, quad(DOOR_X, 0, 2)).unwrap();
    nav.replace_tile(prepared).unwrap();
    let replacement = tile(&nav, DOOR_X, 0, 0);
    assert_ne!(old, replacement);
    assert_eq!(nav.tile_change_stamp(start), stamp);
    assert!(nav.tile_change_stamp(replacement).unwrap() > before);
    assert_eq!(step(&mut world, &nav).replans, 1);
    let route = agent(&world, a).corridor.path();
    assert!(!route.contains(&old));
    assert!(route.contains(&replacement));
}

#[test]
fn budget_defers_in_index_order_and_survives_snapshot() {
    let mut nav = scene(true, false);
    install_gate(&mut nav);
    let mut world = world(&nav);
    world.set_replan_limit(1);
    let agents = [3., 5., 7.].map(|z| add(&nav, &mut world, [2., 0., z]));
    step(&mut world, &nav);
    let positions = agents.map(|a| agent(&world, a).npos);
    nav.set_modifier_enabled(GATE, true).unwrap();

    let Budgeted::Exhausted(_) = world.step(&nav, &mut Budget::new(0)).unwrap() else {
        panic!("step should leave the edit unprocessed");
    };
    assert_eq!(step(&mut world, &nav).replans, 1);
    assert_eq!(agent(&world, agents[0]).target_state, TargetState::Failed);
    for (&a, &pos) in agents[1..].iter().zip(&positions[1..]) {
        let state = agent(&world, a);
        assert!(state.replan_pending());
        assert_eq!(state.npos, pos);
        assert_eq!(state.vel, [0.; 3]);
    }

    let bytes = world.to_bytes();
    let mut restored = CrowdWorld::from_bytes(&nav, &bytes, Limits::UNLIMITED).unwrap();
    assert_eq!(restored.to_bytes(), bytes);
    let mut bad_version = bytes;
    bad_version[4..8].copy_from_slice(&u32::MAX.to_le_bytes());
    assert!(matches!(
        CrowdWorld::from_bytes(&nav, &bad_version, Limits::UNLIMITED),
        Err(Error::InvalidSnapshot(SnapshotError::WrongVersion))
    ));
    for &a in &agents[1..] {
        assert_eq!(step(&mut world, &nav).replans, 1);
        assert_eq!(step(&mut restored, &nav).replans, 1);
        assert_eq!(world.to_bytes(), restored.to_bytes());
        assert_eq!(agent(&world, a).target_state, TargetState::Failed);
    }
    assert_eq!(step(&mut world, &nav).replans, 0);
}

#[test]
fn snapshot_after_an_unstepped_edit_replans_like_the_original() {
    let mut nav = scene(true, false);
    install_gate(&mut nav);
    let mut world = world(&nav);
    add(&nav, &mut world, [2., 0., 5.]);
    step(&mut world, &nav);
    // The edit lands after the last step and before the save.
    nav.set_modifier_enabled(GATE, true).unwrap();
    let mut restored = CrowdWorld::from_bytes(&nav, &world.to_bytes(), Limits::UNLIMITED).unwrap();
    let (original, replayed) = (step(&mut world, &nav), step(&mut restored, &nav));
    assert_eq!(original.replans, 1);
    assert_eq!(replayed.replans, original.replans);
    assert_eq!(restored.digest(), world.digest());
    assert_eq!(restored.to_bytes(), world.to_bytes());
}

#[cfg(feature = "parallel")]
#[test]
fn stamp_replan_replay_is_identical_across_worker_counts() {
    let run = |threads| {
        rayon::ThreadPoolBuilder::new()
            .num_threads(threads)
            .build()
            .unwrap()
            .install(|| {
                let mut nav = scene(true, true);
                install_gate(&mut nav);
                let mut world = world(&nav);
                world.set_replan_limit(3);
                for i in 0..12 {
                    add(
                        &nav,
                        &mut world,
                        [2. + (i / 4) as f32 * 2., 0., 2. + (i % 4) as f32 * 2.],
                    );
                }
                (0..15)
                    .map(|tick| {
                        if matches!(tick, 3 | 9) {
                            nav.set_modifier_enabled(GATE, tick == 3).unwrap();
                        }
                        (step(&mut world, &nav), world.to_bytes())
                    })
                    .collect::<Vec<_>>()
            })
    };
    let serial = run(1);
    assert_eq!(serial, run(4));
    assert_eq!(serial, run(16));
}

#[test]
fn one_thousand_agents_see_a_gate_more_than_thirty_polygons_ahead() {
    let mut nav = scene(true, false);
    install_gate(&mut nav);
    let mut world = world(&nav);
    let mut p = npc();
    p.radius = 0.15;
    p.collision_query_range = 0.4;
    let goal = Goal::Point {
        poly: tile(&nav, 39, 0, 0),
        pos: GOAL,
    };
    let position = |i: usize| {
        [
            0.25 + (i % 50) as f32 * 0.39,
            0.,
            0.25 + (i / 50) as f32 * 0.45,
        ]
    };
    let agents: Vec<_> = (0..1000)
        .map(|i| {
            let a = world.add_agent(&nav, position(i), p).unwrap();
            world.set_goal(&nav, a, goal).unwrap();
            a
        })
        .collect();

    // A passable edit on the agents' start tiles installs all initial paths
    // using the same-tick search.
    for x in [0, 1] {
        nav.set_poly_flags(tile(&nav, x, 0, 0), 3).unwrap();
    }
    assert_eq!(step(&mut world, &nav).replans, 1000);
    let door = tile(&nav, DOOR_X, 0, 0);
    for &a in &agents {
        assert!(
            agent(&world, a)
                .corridor
                .path()
                .iter()
                .position(|&r| r == door)
                .unwrap()
                > 30
        );
    }
    nav.set_modifier_enabled(GATE, true).unwrap();
    assert_eq!(step(&mut world, &nav).replans, 1000);
    for &a in &agents {
        let state = agent(&world, a);
        assert_eq!(state.target_state, TargetState::Failed);
        assert_eq!(state.vel, [0.; 3]);
        assert!(!state.corridor.path().contains(&door));
    }
    nav.set_modifier_enabled(GATE, false).unwrap();
    assert_eq!(step(&mut world, &nav).replans, 1000);
    assert!(
        agents
            .iter()
            .all(|&a| agent(&world, a).corridor.path().contains(&door))
    );

    let mut ws = Workspace::new();
    let mut crowd = Crowd::new(&mut ws, &nav, 1000, 0.6).unwrap();
    crowd.set_stamp_revalidation(true);
    for i in 0..1000 {
        let a = crowd.add_agent(&mut ws, &nav, position(i), p).unwrap();
        crowd
            .request_move_target(&nav, a, tile(&nav, 39, 0, 0), GOAL)
            .unwrap();
    }
    for x in [0, 1] {
        nav.set_poly_flags(tile(&nav, x, 0, 0), 1).unwrap();
    }
    crowd.update(&mut ws, &nav, 0.05).unwrap();
    assert!(
        crowd
            .agents()
            .iter()
            .all(|a| a.corridor.path().contains(&door))
    );
    nav.set_modifier_enabled(GATE, true).unwrap();
    crowd.update(&mut ws, &nav, 0.05).unwrap();
    for a in crowd.agents() {
        assert_eq!(a.target_state, TargetState::Failed);
        assert_eq!(a.vel, [0.; 3]);
        assert!(!a.corridor.path().contains(&door));
    }
    nav.set_modifier_enabled(GATE, false).unwrap();
    crowd.update(&mut ws, &nav, 0.05).unwrap();
    assert!(
        crowd
            .agents()
            .iter()
            .all(|a| a.corridor.path().contains(&door))
    );
}
