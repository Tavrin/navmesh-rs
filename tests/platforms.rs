#![cfg(feature = "detour")]

use navmesh_rs::detour::{
    DefaultFilter, LinkId, NavMesh, NavMeshCreateParams, NavMeshParams, create_nav_mesh_data,
};
use navmesh_rs::frames::{DockSpec, FrameId, FramePoint, FramePose, FrameSet};
use navmesh_rs::{Budget, Budgeted, Error, SnapshotError};

const WORLD: FrameId = FrameId::WORLD;
const CABIN: FrameId = FrameId(1);
const BOARD: LinkId = LinkId(1);
const ALIGHT: LinkId = LinkId(2);
const LOWER: FramePoint = FramePoint::new(WORLD, [7., 0., 5.]);
const UPPER: FramePoint = FramePoint::new(WORLD, [7., 4., 5.]);
const EXTENTS: [f32; 3] = [0.6, 0.45, 0.6];

fn floor(y: f32, layer: i32, width: u16) -> Vec<u8> {
    create_nav_mesh_data(&NavMeshCreateParams {
        verts: &[0, 0, 0, 0, 0, 10, width, 0, 10, width, 0, 0],
        vert_count: 4,
        polys: &[0, 1, 2, 3, 0x800f, 0x800f, 0x800f, 0x800f],
        poly_count: 1,
        nvp: 4,
        poly_flags: &[1],
        poly_areas: &[0],
        tile_layer: layer,
        bmin: [0., y, 0.],
        bmax: [f32::from(width), y + 1., 10.],
        cs: 1.,
        ch: 1.,
        walkable_climb: 0.5,
        build_bv_tree: true,
        ..Default::default()
    })
    .unwrap()
}

fn pose(y: f32) -> FramePose {
    FramePose::new([10., y, 0.], 1., 0.).unwrap()
}

fn lift() -> FrameSet {
    let mut world = NavMesh::new(&NavMeshParams {
        orig: [0.; 3],
        tile_width: 10.,
        tile_height: 10.,
        max_tiles: 4,
        max_polys: 8,
    })
    .unwrap();
    world.add_tile(floor(0., 0, 10)).unwrap();
    world.add_tile(floor(4., 1, 10)).unwrap();
    let cabin = NavMesh::new_single(floor(0., 0, 5)).unwrap().0;
    let mut frames = FrameSet::new(world);
    frames.add_frame(CABIN, cabin, pose(0.)).unwrap();
    let deck = FramePoint::new(CABIN, [0., 0., 5.]);
    frames
        .add_dock(
            DockSpec::new(BOARD, FramePoint::new(WORLD, [10., 0., 5.]), deck, 0.25),
            true,
        )
        .unwrap();
    frames
        .add_dock(
            DockSpec::new(ALIGHT, deck, FramePoint::new(WORLD, [10., 4., 5.]), 0.25),
            false,
        )
        .unwrap();
    frames
}

#[test]
fn route_uses_reachable_directed_docks_and_caller_waits() {
    let mut frames = lift();
    let filter = DefaultFilter::default();
    let mut budget = Budget::unlimited();
    assert_eq!(
        frames
            .plan(LOWER, UPPER, EXTENTS, &filter, &mut budget)
            .unwrap(),
        Budgeted::Complete(None)
    );
    let Budgeted::Complete(Some(plan)) = frames
        .plan_with_waits(LOWER, UPPER, EXTENTS, &filter, &mut budget, |id| {
            (id == ALIGHT).then_some(3.)
        })
        .unwrap()
    else {
        panic!("expected a route over both docks");
    };
    let legs: Vec<_> = plan.legs.iter().map(|leg| (leg.frame, leg.dock)).collect();
    assert_eq!(
        legs,
        [(WORLD, Some(BOARD)), (CABIN, Some(ALIGHT)), (WORLD, None)]
    );
    assert_eq!(plan.cost, 9.);
    assert_eq!(
        frames
            .plan_with_waits(UPPER, LOWER, EXTENTS, &filter, &mut budget, |_| Some(3.))
            .unwrap(),
        Budgeted::Complete(None)
    );
    let mut small = Budget::new(1);
    assert_eq!(
        frames
            .plan(LOWER, UPPER, EXTENTS, &filter, &mut small)
            .unwrap(),
        Budgeted::Exhausted(None)
    );
    assert!(small.used() <= small.limit());

    frames.set_pose(CABIN, pose(4.)).unwrap();
    let deck = FramePoint::new(CABIN, [0., 0., 5.]);
    let (poly, world) = frames
        .find_nearest_world(CABIN, [10., 4., 5.], EXTENTS, &filter)
        .unwrap()
        .unwrap();
    assert_eq!(world, [10., 4., 5.]);
    assert_eq!(
        frames.dock_landing(ALIGHT, deck, poly, &filter).unwrap(),
        None
    );
    frames.set_dock_enabled(ALIGHT, true).unwrap();
    assert_eq!(
        frames
            .dock_landing(ALIGHT, deck, poly, &filter)
            .unwrap()
            .unwrap()
            .1
            .position,
        world
    );
    frames.set_pose(CABIN, pose(2.)).unwrap();
    assert_eq!(
        frames.dock_landing(ALIGHT, deck, poly, &filter).unwrap(),
        None
    );
}

#[cfg(feature = "dynamic")]
#[test]
fn gate_on_dock_landing_blocks_routes_until_reopened() {
    use navmesh_rs::Limits;
    use navmesh_rs::dynamic::{DynamicLayer, Effect, ModifierId, ModifierSpec, Selector};

    let mut frames = lift();
    let gate = ModifierId(0);
    let mut layer = DynamicLayer::new(1, Limits::default()).unwrap();
    layer
        .upsert(ModifierSpec::new(
            gate,
            Selector::Prism {
                verts_xz: vec![[0., 0.], [5., 0.], [5., 10.], [0., 10.]],
                ymin: -0.5,
                ymax: 0.5,
                area: None,
            },
            Effect::ClearFlags(u16::MAX),
        ))
        .unwrap();
    frames
        .mesh_mut(CABIN)
        .unwrap()
        .set_dynamic_layer(Some(layer));
    let goal = FramePoint::new(CABIN, [2., 0., 5.]);
    for closed in [false, true, false] {
        frames
            .mesh_mut(CABIN)
            .unwrap()
            .set_modifier_enabled(gate, closed)
            .unwrap();
        let result = frames
            .plan(
                LOWER,
                goal,
                EXTENTS,
                &DefaultFilter::default(),
                &mut Budget::unlimited(),
            )
            .unwrap();
        assert_eq!(matches!(result, Budgeted::Complete(Some(_))), !closed);
        assert!(frames.dock(BOARD).unwrap().1);
    }
}

#[test]
fn world_local_round_trip_at_eight_kilometres() {
    let frames = &mut lift();
    frames
        .set_pose(
            CABIN,
            FramePose::new([8_000., 90., 8_000.], 0.6, 0.8).unwrap(),
        )
        .unwrap();
    let world = [8001.25, 91., 8002.5];
    let local = frames.to_local(CABIN, world).unwrap();
    let round_trip = frames.to_world(local).unwrap();
    assert!(
        world
            .into_iter()
            .zip(round_trip)
            .all(|(a, b)| (a - b).abs() <= 0.001)
    );
}

#[test]
fn frame_snapshot_binds_meshes_and_restores_atomically() {
    let mut original = lift();
    original.set_pose(CABIN, pose(4.)).unwrap();
    original.set_dock_enabled(BOARD, false).unwrap();
    original.set_dock_enabled(ALIGHT, true).unwrap();
    let bytes = original.to_bytes();
    let mut restored = lift();
    restored.restore(&bytes).unwrap();
    assert_eq!(restored.to_bytes(), bytes);
    assert_eq!(restored.pose(CABIN), Some(pose(4.)));
    assert!(!restored.dock(BOARD).unwrap().1);
    assert!(restored.dock(ALIGHT).unwrap().1);

    let mut unknown = bytes.clone();
    unknown[4] = 2;
    assert_eq!(
        restored.restore(&unknown),
        Err(Error::InvalidSnapshot(SnapshotError::WrongVersion))
    );
    unknown[4] = 1;
    unknown[34] ^= 1;
    assert!(restored.restore(&unknown).is_err());
    assert_eq!(restored.to_bytes(), bytes);
    restored
        .mesh_mut(CABIN)
        .unwrap()
        .set_link_layer(Some(1))
        .unwrap();
    assert_eq!(
        restored.restore(&bytes),
        Err(Error::InvalidSnapshot(SnapshotError::InvalidValue(
            "frame mesh binding"
        )))
    );
}

#[cfg(feature = "crowd")]
mod rides {
    use super::*;
    use navmesh_rs::Workspace;
    use navmesh_rs::crowd::platforms::{FrameCrowd, FramedCrowd, PlatformAgentId};
    use navmesh_rs::crowd::{AgentParams, AgentState, ArrivalPolicy, Crowd};

    const RIDER: PlatformAgentId = PlatformAgentId(1);

    fn add_rider<C: FrameCrowd>(crowd: &mut FramedCrowd<C>, frames: &FrameSet) {
        let mut params = AgentParams::default();
        params.radius = 0.25;
        params.max_speed = 2.;
        // The dock test isolates locomotion and frame hand-over.
        params.update_flags = 0;
        crowd.add_agent(frames, RIDER, LOWER, params).unwrap();
        crowd.set_goal(frames, RIDER, Some(UPPER)).unwrap();
        crowd.set_wait_cost(frames, ALIGHT, Some(5.)).unwrap();
    }

    fn ride<C: FrameCrowd>(
        frames: &mut FrameSet,
        crowd: &mut FramedCrowd<C>,
        mut tick: impl FnMut(&FrameSet, &mut FramedCrowd<C>),
        ticks_in_transit: usize,
    ) -> Vec<(FrameId, [u32; 3])> {
        let mut trace = Vec::new();
        let mut next = |frames: &FrameSet, crowd: &mut FramedCrowd<C>| {
            let from = crowd.world_position(frames, RIDER).unwrap();
            tick(frames, crowd);
            let (frame, _, a) = crowd.agent(RIDER).unwrap();
            let to = crowd.world_position(frames, RIDER).unwrap();
            assert_eq!(a.state, AgentState::Walking);
            assert!(
                frames
                    .mesh(frame)
                    .unwrap()
                    .is_valid_poly_ref(a.corridor.first_poly())
            );
            assert!(
                from.into_iter()
                    .zip(to)
                    .all(|(a, b)| (a - b).abs() <= 0.102)
            );
            trace.push((frame, a.npos.map(f32::to_bits)));
            (frame, a.npos)
        };
        for _ in 0..240 {
            if next(frames, crowd).0 == CABIN {
                break;
            }
        }
        assert_eq!(crowd.agent(RIDER).unwrap().0, CABIN);
        let on_deck = crowd.agent(RIDER).unwrap().2.npos.map(f32::to_bits);
        frames.set_dock_enabled(BOARD, false).unwrap();
        // Keep the scheduled top goal. The boarding and exit markers coincide
        // in the cabin, so the stationary rider waits on the same local point.
        for i in 0..ticks_in_transit {
            let y = (i % 32) as f32 * 0.125;
            let p = if i % 2 == 0 {
                pose(y)
            } else {
                FramePose::new([15., y, 5.], 0., 1.).unwrap()
            };
            frames.set_pose(CABIN, p).unwrap();
            let (frame, local) = next(frames, crowd);
            assert_eq!(frame, CABIN);
            assert_eq!(local.map(f32::to_bits), on_deck);
        }
        frames.set_pose(CABIN, pose(4.)).unwrap();
        frames.set_dock_enabled(ALIGHT, true).unwrap();
        for _ in 0..240 {
            let (frame, local) = next(frames, crowd);
            if frame == WORLD && (local[0] - UPPER.position[0]).abs() < 0.005 {
                break;
            }
        }
        let (frame, _, a) = crowd.agent(RIDER).unwrap();
        assert_eq!(frame, WORLD);
        assert_eq!(a.npos[1], 4.);
        assert!((a.npos[0] - UPPER.position[0]).abs() < 0.005);
        trace
    }

    #[test]
    fn crowd_rides_lift_without_local_drift_for_10000_ticks() {
        let mut frames = lift();
        let mut crowd = FramedCrowd::<Crowd>::default();
        let mut ws = Workspace::new();
        for id in frames.frame_ids() {
            let local = Crowd::new(&mut ws, frames.mesh(id).unwrap(), 4, 0.3).unwrap();
            crowd.attach(&frames, id, local).unwrap();
        }
        add_rider(&mut crowd, &frames);
        ride(
            &mut frames,
            &mut crowd,
            |f, c| c.update(f, &mut ws, 0.05).unwrap(),
            10_000,
        );
    }

    fn arrival_at_final_goal<C: FrameCrowd>(
        frames: &mut FrameSet,
        crowd: &mut FramedCrowd<C>,
        policy: ArrivalPolicy,
        mut tick: impl FnMut(&FrameSet, &mut FramedCrowd<C>),
    ) {
        let mut params = AgentParams::default();
        params.radius = 0.25;
        params.max_speed = 2.;
        params.update_flags = 0;
        params.arrival_policy = policy;
        params.arrival_distance = 0.5;
        crowd.add_agent(frames, RIDER, LOWER, params).unwrap();
        crowd.set_goal(frames, RIDER, Some(UPPER)).unwrap();
        crowd.set_wait_cost(frames, ALIGHT, Some(5.)).unwrap();

        for _ in 0..240 {
            tick(frames, crowd);
        }
        let (frame, _, agent) = crowd.agent(RIDER).unwrap();
        assert_eq!(frame, CABIN);
        assert!(!agent.arrived());

        frames.set_dock_enabled(BOARD, false).unwrap();
        frames.set_pose(CABIN, pose(4.)).unwrap();
        frames.set_dock_enabled(ALIGHT, true).unwrap();
        for _ in 0..240 {
            tick(frames, crowd);
        }
        if policy == ArrivalPolicy::Park {
            let (frame, _, agent) = crowd.agent(RIDER).unwrap();
            assert_eq!(frame, WORLD);
            assert!(agent.arrived());
            assert_eq!(agent.npos[1], UPPER.position[1]);
            assert!((agent.npos[0] - UPPER.position[0]).abs() <= params.arrival_distance);
        } else {
            assert!(crowd.agent(RIDER).is_none());
            crowd.add_agent(frames, RIDER, UPPER, params).unwrap();
        }
    }

    #[test]
    fn crowd_applies_arrival_policy_only_at_the_final_goal() {
        for policy in [ArrivalPolicy::Park, ArrivalPolicy::Remove] {
            let mut frames = lift();
            let mut crowd = FramedCrowd::<Crowd>::default();
            let mut ws = Workspace::new();
            for id in frames.frame_ids() {
                let local = Crowd::new(&mut ws, frames.mesh(id).unwrap(), 4, 0.3).unwrap();
                crowd.attach(&frames, id, local).unwrap();
            }
            arrival_at_final_goal(&mut frames, &mut crowd, policy, |f, c| {
                c.update(f, &mut ws, 0.05).unwrap();
            });
        }
    }

    #[test]
    fn crowd_with_default_avoidance_can_walk_across_a_dock_wall() {
        let mut frames = lift();
        let mut crowd = FramedCrowd::<Crowd>::default();
        let mut ws = Workspace::new();
        for id in frames.frame_ids() {
            let local = Crowd::new(&mut ws, frames.mesh(id).unwrap(), 4, 0.3).unwrap();
            crowd.attach(&frames, id, local).unwrap();
        }
        let mut params = AgentParams::default();
        params.radius = 0.25;
        crowd.add_agent(&frames, RIDER, LOWER, params).unwrap();
        let on_deck = FramePoint::new(CABIN, [2., 0., 5.]);
        crowd.set_goal(&frames, RIDER, Some(on_deck)).unwrap();
        for _ in 0..240 {
            crowd.update(&frames, &mut ws, 0.05).unwrap();
            if crowd
                .agent(RIDER)
                .is_some_and(|(id, _, agent)| id == CABIN && agent.npos[0] > 1.)
            {
                break;
            }
        }
        let (id, _, agent) = crowd.agent(RIDER).unwrap();
        assert_eq!(id, CABIN);
        assert!(agent.npos[0] > 1.);
        frames.set_dock_enabled(BOARD, false).unwrap();
        frames.set_pose(CABIN, pose(4.)).unwrap();
        frames.set_dock_enabled(ALIGHT, true).unwrap();
        crowd.set_goal(&frames, RIDER, Some(UPPER)).unwrap();
        for _ in 0..240 {
            crowd.update(&frames, &mut ws, 0.05).unwrap();
            if crowd
                .agent(RIDER)
                .is_some_and(|(id, _, agent)| id == WORLD && agent.npos[0] < 8.)
            {
                break;
            }
        }
        let (id, _, agent) = crowd.agent(RIDER).unwrap();
        assert_eq!(id, WORLD);
        assert!(agent.npos[0] < 8.);
    }

    #[test]
    fn full_destination_keeps_rider_on_the_source_until_there_is_space() {
        let frames = lift();
        let mut crowd = FramedCrowd::<Crowd>::default();
        let mut ws = Workspace::new();
        for id in frames.frame_ids() {
            let local = Crowd::new(&mut ws, frames.mesh(id).unwrap(), 1, 0.3).unwrap();
            crowd.attach(&frames, id, local).unwrap();
        }
        let mut p = AgentParams::default();
        p.radius = 0.25;
        p.update_flags = 0;
        let occupant = PlatformAgentId(2);
        crowd.add_agent(&frames, RIDER, LOWER, p).unwrap();
        crowd
            .add_agent(&frames, occupant, FramePoint::new(CABIN, [3., 0., 5.]), p)
            .unwrap();
        crowd
            .set_goal(&frames, RIDER, Some(FramePoint::new(CABIN, [2., 0., 5.])))
            .unwrap();
        for _ in 0..150 {
            crowd.update(&frames, &mut ws, 0.05).unwrap();
        }
        let (frame, _, a) = crowd.agent(RIDER).unwrap();
        assert_eq!(frame, WORLD);
        assert!(a.npos[0] >= 9.999);
        crowd.remove_agent(occupant).unwrap();
        crowd.update(&frames, &mut ws, 0.05).unwrap();
        assert_eq!(crowd.agent(RIDER).unwrap().0, CABIN);
    }

    #[test]
    #[ignore = "scale qualification"]
    fn sixty_four_riders_keep_exact_deck_positions() {
        let mut frames = lift();
        let mut crowd = FramedCrowd::<Crowd>::default();
        let mut ws = Workspace::new();
        let local = Crowd::new(&mut ws, frames.mesh(CABIN).unwrap(), 64, 0.25).unwrap();
        crowd.attach(&frames, CABIN, local).unwrap();
        let mut p = AgentParams::default();
        p.radius = 0.1;
        p.collision_query_range = 0.2;
        p.update_flags = 0;
        let riders: Vec<_> = (0..64)
            .map(|i| {
                let id = PlatformAgentId(i);
                let pos = [0.5 + (i % 8) as f32 * 0.5, 0., 1. + (i / 8) as f32];
                crowd
                    .add_agent(&frames, id, FramePoint::new(CABIN, pos), p)
                    .unwrap();
                (id, crowd.agent(id).unwrap().2.npos.map(f32::to_bits))
            })
            .collect();
        frames.set_dock_enabled(BOARD, false).unwrap();
        for i in 0..10_000 {
            let p = FramePose::new([i as f32 * 0.125, 2., 8.], 0.6, 0.8).unwrap();
            frames.set_pose(CABIN, p).unwrap();
            crowd.update(&frames, &mut ws, 0.05).unwrap();
            for &(id, pos) in &riders {
                let (frame, _, a) = crowd.agent(id).unwrap();
                assert_eq!(frame, CABIN);
                assert_eq!(a.state, AgentState::Walking);
                assert_eq!(a.npos.map(f32::to_bits), pos);
            }
        }
    }

    #[cfg(feature = "crowd-world")]
    mod world {
        use super::*;
        use navmesh_rs::Limits;
        use navmesh_rs::crowd::world::{CrowdWorld, CrowdWorldParams};

        fn crowd(frames: &FrameSet) -> FramedCrowd<CrowdWorld> {
            let mut crowd = FramedCrowd::<CrowdWorld>::default();
            let mut config = CrowdWorldParams::default();
            config.max_agents = 16;
            config.max_heavy_agents = 16;
            config.max_radius = 0.3;
            config.fixed_dt = 0.05;
            for id in frames.frame_ids() {
                let local =
                    CrowdWorld::new(frames.mesh(id).unwrap(), &config, Limits::default()).unwrap();
                crowd.attach(frames, id, local).unwrap();
            }
            add_rider(&mut crowd, frames);
            crowd
        }

        fn tick(frames: &FrameSet, c: &mut FramedCrowd<CrowdWorld>) {
            assert!(matches!(
                c.step(frames, &mut Budget::unlimited()).unwrap(),
                Budgeted::Complete(())
            ));
        }

        #[test]
        fn crowd_world_applies_arrival_policy_only_at_the_final_goal() {
            for policy in [ArrivalPolicy::Park, ArrivalPolicy::Remove] {
                let mut frames = lift();
                let mut c = crowd(&frames);
                c.remove_agent(RIDER).unwrap();
                arrival_at_final_goal(&mut frames, &mut c, policy, tick);
                FramedCrowd::from_bytes(&frames, &c.to_bytes(&frames), Limits::default()).unwrap();
            }
        }

        #[cfg(not(feature = "parallel"))]
        #[test]
        fn crowd_world_rides_lift() {
            let mut frames = lift();
            let mut c = crowd(&frames);
            ride(&mut frames, &mut c, tick, 10_000);
        }

        #[cfg(feature = "parallel")]
        #[test]
        fn crowd_world_rides_lift_deterministically_across_threads() {
            let mut expected = None;
            for threads in [1, 4, 16] {
                let pool = rayon::ThreadPoolBuilder::new()
                    .num_threads(threads)
                    .build()
                    .unwrap();
                let mut frames = lift();
                let mut c = crowd(&frames);
                let mut params = AgentParams::default();
                params.radius = 0.25;
                params.update_flags = 0;
                for i in 0..7 {
                    c.add_agent(
                        &frames,
                        PlatformAgentId(10 + i),
                        FramePoint::new(CABIN, [3.5, 0., 1. + i as f32]),
                        params,
                    )
                    .unwrap();
                }
                let trace = pool.install(|| ride(&mut frames, &mut c, tick, 10_000));
                if let Some((want_trace, want_bytes)) = &expected {
                    assert_eq!(&trace, want_trace);
                    assert_eq!(&c.to_bytes(&frames), want_bytes);
                } else {
                    expected = Some((trace, c.to_bytes(&frames)));
                }
            }
        }

        #[test]
        fn crowd_world_avoids_a_cross_frame_disc_only_at_an_open_dock() {
            use navmesh_rs::crowd::{AvoidanceMode, OBSTACLE_AVOIDANCE, OrcaParams};

            let run = |enabled, avoidance| {
                let mut frames = lift();
                let mut spec = frames.remove_dock(BOARD).unwrap();
                spec.radius = 2.;
                frames.add_dock(spec, enabled).unwrap();
                let mut c = crowd(&frames);
                c.remove_agent(RIDER).unwrap();
                let mut p = AgentParams::default();
                p.radius = 0.25;
                p.max_speed = 2.;
                p.update_flags = OBSTACLE_AVOIDANCE;
                p.avoidance = avoidance;
                let approach = FramePoint::new(WORLD, [9.6, 0., 5.]);
                let obstacle = FramePoint::new(CABIN, [0.05, 0., 5.]);
                c.add_agent(&frames, RIDER, approach, p).unwrap();
                p.update_flags = 0;
                c.add_agent(&frames, PlatformAgentId(2), obstacle, p)
                    .unwrap();
                c.set_goal(&frames, RIDER, Some(FramePoint::new(WORLD, [10., 0., 5.])))
                    .unwrap();
                for _ in 0..6 {
                    tick(&frames, &mut c);
                }
                c.agent(RIDER).unwrap().2.npos[0]
            };

            for avoidance in [
                AvoidanceMode::Sampled,
                AvoidanceMode::Orca(OrcaParams::default()),
            ] {
                assert!(run(true, avoidance) < run(false, avoidance));
            }
        }

        #[test]
        fn crowd_world_restores_an_agent_aboard_with_pending_goal() {
            let mut frames = lift();
            let mut c = crowd(&frames);
            for _ in 0..240 {
                tick(&frames, &mut c);
                if c.agent(RIDER).unwrap().0 == CABIN {
                    break;
                }
            }
            assert_eq!(c.agent(RIDER).unwrap().0, CABIN);
            frames.set_dock_enabled(BOARD, false).unwrap();
            frames.set_pose(CABIN, pose(2.)).unwrap();
            let bytes = c.to_bytes(&frames);
            let mut reconstructed = lift();
            reconstructed.restore(&frames.to_bytes()).unwrap();
            let mut restored =
                FramedCrowd::from_bytes(&reconstructed, &bytes, Limits::default()).unwrap();
            assert_eq!(restored.to_bytes(&reconstructed), bytes);
            assert!(matches!(
                restored.step(&reconstructed, &mut Budget::new(1)).unwrap(),
                Budgeted::Exhausted(())
            ));
            assert_eq!(restored.to_bytes(&reconstructed), bytes);
            let mut unknown = bytes;
            unknown[4] = 2;
            assert!(matches!(
                FramedCrowd::from_bytes(&reconstructed, &unknown, Limits::default()),
                Err(Error::InvalidSnapshot(SnapshotError::WrongVersion))
            ));
            reconstructed.set_pose(CABIN, pose(4.)).unwrap();
            reconstructed.set_dock_enabled(ALIGHT, true).unwrap();
            for _ in 0..100 {
                tick(&reconstructed, &mut c);
                tick(&reconstructed, &mut restored);
            }
            assert_eq!(restored.agent(RIDER).unwrap().0, WORLD);
            assert_eq!(
                restored.to_bytes(&reconstructed),
                c.to_bytes(&reconstructed)
            );
        }
    }
}
