use super::*;
use navmesh_rs::bake::profiles::{MixedProfileCrowdWorld, ProfileCrowdWorld};
use navmesh_rs::crowd::world::{CrowdWorld, CrowdWorldParams};
use navmesh_rs::crowd::{AgentParams, AgentState, AvoidanceMode};
use navmesh_rs::{Budgeted, Limits, SnapshotError};

fn install_sizes(input: &Input<'_>) -> ProfileMeshes {
    install_sizes_with_residency(input, None)
}

fn install_sizes_with_residency(input: &Input<'_>, saved: Option<&ProfileMeshes>) -> ProfileMeshes {
    let mut meshes = ProfileMeshes::new();
    for (id, radius) in [(SMALL, 1), (LARGE, 3)] {
        let spec = spec(id, config(radius), input);
        let (tiles, _) = profile_tiles(input, spec, REV);
        let mut options = ProfileMeshOptions::new(8, 2048);
        options.global_off_mesh_links = true;
        meshes.insert(catalog(&tiles, spec, options)).unwrap();
        if let Some(saved) = saved {
            let bytes = saved.get(id).unwrap().save_residency().unwrap();
            meshes.restore_residency(id, &bytes).unwrap();
            // Reconstruct epochs independently of the original install sequence.
            // Commit every tile after restoring its saved slot and salt.
        }
        for out in tiles {
            let tile = out.into_tile();
            let catalog = meshes.get(id).unwrap().catalog();
            let ready = catalog
                .prepare(catalog.tile(tile.key).unwrap(), tile.data)
                .unwrap();
            meshes.commit(id, ready).unwrap();
        }
    }
    meshes
}

fn params() -> CrowdWorldParams {
    let mut params = CrowdWorldParams::default();
    params.max_agents = 2;
    params.max_heavy_agents = 2;
    params.max_radius = 0.7;
    params
}

fn agents() -> [AgentParams; 2] {
    let mut small = CrowdWorld::default_agent_params();
    small.radius = 0.22;
    small.max_speed = 2.;
    small.collision_query_range = 3.;
    let mut large = small;
    large.radius = 0.7;
    [small, large]
}

#[test]
fn mixed_sizes_pass_in_corridor_and_doorway_with_shared_avoidance() {
    for doorway in [false, true] {
        let mut scene = Scene::default();
        if doorway {
            scene.floor([0., 0., 10., 8.], 0.);
            scene.floor([10., 2., 12., 6.], 0.);
            scene.floor([12., 0., 22., 8.], 0.);
        } else {
            scene.floor([0., 2., 22., 6.], 0.);
        }
        let input = Input::new(&scene.verts, &scene.tris);
        let meshes = install_sizes(&input);
        for mode in [
            CrowdWorld::default_agent_params().avoidance,
            AvoidanceMode::Sampled,
        ] {
            let run = || {
                let mut world = MixedProfileCrowdWorld::new(
                    &meshes,
                    &[LARGE, SMALL],
                    &params(),
                    Limits::default(),
                )
                .unwrap();
                let [mut small, mut large] = agents();
                small.avoidance = mode;
                large.avoidance = mode;
                // Straight, opposite targets pass within the combined radius.
                // Offset the paths slightly to avoid a perfectly symmetric stop.
                let starts = [[5., 0., 3.75], [17., 0., 4.25]];
                let targets = [[17., 0., 3.75], [5., 0., 4.25]];
                let profiles = [SMALL, LARGE];
                let handles = core::array::from_fn::<_, 2, _>(|i| {
                    let h = world
                        .add_agent(&meshes, profiles[i], starts[i], [small, large][i])
                        .unwrap();
                    world
                        .set_goal(
                            &meshes,
                            &h,
                            Some(&nearest(meshes.get(profiles[i]).unwrap(), targets[i])),
                        )
                        .unwrap();
                    h
                });
                let mut max_overlap = 0.0f32;
                let mut closest = f32::INFINITY;
                let mut digests = Vec::new();
                let mut complete = false;
                let queries = profiles.map(|id| meshes.query(id, 1).unwrap());
                for _ in 0..350 {
                    let report = world.step(&meshes, &mut Budget::unlimited()).unwrap();
                    assert!(matches!(report, Budgeted::Complete(_)));
                    let positions = core::array::from_fn::<_, 2, _>(|i| {
                        let a = world.agent(&handles[i]).unwrap();
                        assert_eq!(a.state(), AgentState::Walking);
                        let pos = a.position();
                        let nearest = queries[i]
                            .find_nearest_poly(&pos, &[0.01; 3], &DefaultFilter::default())
                            .unwrap()
                            .unwrap();
                        assert!((nearest.position[0] - pos[0]).abs() <= 0.001);
                        assert!((nearest.position[2] - pos[2]).abs() <= 0.001);
                        pos
                    });
                    let d = (positions[0][0] - positions[1][0])
                        .hypot(positions[0][2] - positions[1][2]);
                    closest = closest.min(d);
                    max_overlap = max_overlap.max(small.radius + large.radius - d);
                    digests.push(world.digest());
                    if positions[0][0] > 16.65 && positions[1][0] < 5.35 {
                        complete = true;
                        break;
                    }
                }
                assert!(
                    complete,
                    "doorway={doorway}, mode={mode:?}: agents did not finish"
                );
                assert!(
                    max_overlap <= 0.001,
                    "doorway={doorway}, mode={mode:?}: overlap {max_overlap}"
                );
                (digests, closest, max_overlap)
            };
            #[cfg(feature = "parallel")]
            let (digests, closest, overlap) = {
                let pool = |threads| {
                    rayon::ThreadPoolBuilder::new()
                        .num_threads(threads)
                        .build()
                        .unwrap()
                };
                let one = pool(1).install(run);
                for threads in [4, 16] {
                    assert_eq!(one, pool(threads).install(run));
                }
                one
            };
            #[cfg(not(feature = "parallel"))]
            let (digests, closest, overlap) = run();
            println!(
                "doorway={doorway}, mode={mode:?}: {} ticks, closest {closest:.6} m, max overlap {overlap:.6} m",
                digests.len()
            );
        }
    }
}

#[test]
fn mixed_snapshot_preserves_profile_and_running_search() {
    let mut scene = Scene::default();
    for i in 0..104 {
        let x = i as f32;
        scene.floor([x, 0., x + 1., 10.], 0.);
    }
    let mut input = Input::new(&scene.verts, &scene.tris);
    let areas = (0..104).flat_map(|i| [1 + i % 2; 2]).collect::<Vec<_>>();
    input.areas = Some(&areas);
    let meshes = install_sizes(&input);
    let other_meshes = install_sizes_with_residency(&input, Some(&meshes));
    assert_ne!(
        meshes.get(LARGE).unwrap().epoch(),
        other_meshes.get(LARGE).unwrap().epoch()
    );
    let mut params = params();
    params.path_queue.iterations = 1;
    let mut world =
        MixedProfileCrowdWorld::new(&meshes, &[SMALL, LARGE], &params, Limits::default()).unwrap();
    // Lowest slot and the first sliced search are on the non-default mesh.
    let [small, large] = agents();
    let a = world
        .add_agent(&meshes, LARGE, [2., 0., 3.], large)
        .unwrap();
    let b = world
        .add_agent(&meshes, SMALL, [2., 0., 6.], small)
        .unwrap();
    let large_goal = nearest(meshes.get(LARGE).unwrap(), [102., 0., 3.]);
    assert!(matches!(
        world.set_goal(&meshes, &b, Some(&large_goal)),
        Err(ProfileError::BindingMismatch)
    ));
    assert!(matches!(
        world.step(&other_meshes, &mut Budget::unlimited()),
        Err(ProfileError::BindingMismatch)
    ));
    world.set_goal(&meshes, &a, Some(&large_goal)).unwrap();
    world
        .set_goal(
            &meshes,
            &b,
            Some(&nearest(meshes.get(SMALL).unwrap(), [102., 0., 6.])),
        )
        .unwrap();
    let report = world.step(&meshes, &mut Budget::unlimited()).unwrap();
    assert_eq!(report.value().path_queue_depth, 2);
    let bytes = world.save(&meshes).unwrap();
    let mut restored =
        MixedProfileCrowdWorld::restore(&other_meshes, &bytes, Limits::default()).unwrap();
    let restored_a = restored.agent_at(a.index()).unwrap();
    let restored_b = restored.agent_at(b.index()).unwrap();
    assert_eq!(restored.profile(&restored_a).unwrap(), LARGE);
    assert_eq!(restored.profile(&restored_b).unwrap(), SMALL);
    assert_eq!(restored.save(&other_meshes).unwrap(), bytes);

    for _ in 0..120 {
        assert_eq!(
            world.step(&meshes, &mut Budget::unlimited()).unwrap(),
            restored
                .step(&other_meshes, &mut Budget::unlimited())
                .unwrap(),
        );
        assert_eq!(world.digest(), restored.digest());
    }
    assert_eq!(
        world.save(&meshes).unwrap(),
        restored.save(&other_meshes).unwrap()
    );
    let mut unknown = bytes[..bytes.len() - 32].to_vec();
    let inner_version = 16 + 2 * 36 + 4;
    unknown[inner_version..inner_version + 4].copy_from_slice(&99u32.to_le_bytes());
    unknown.extend(content_hash(&unknown));
    assert!(matches!(
        MixedProfileCrowdWorld::restore(&other_meshes, &unknown, Limits::default()),
        Err(ProfileError::Navigation(Error::InvalidSnapshot(
            SnapshotError::WrongVersion
        )))
    ));
}

#[test]
fn single_profile_motion_matches_the_original_world_bit_for_bit() {
    let mut scene = Scene::default();
    scene.floor([0., 0., 22., 8.], 0.);
    let meshes = install_sizes(&Input::new(&scene.verts, &scene.tris));
    let large = meshes.get(LARGE).unwrap();
    let mut original = ProfileCrowdWorld::new(large, &params(), Limits::default()).unwrap();
    let mut mixed =
        MixedProfileCrowdWorld::new(&meshes, &[LARGE], &params(), Limits::default()).unwrap();
    let [small, big] = agents();
    let mut handles = Vec::new();
    for (x, z, goal, settings) in [(5., 3.5, 17., small), (17., 4.5, 5., big)] {
        let old = original.add_agent(large, [x, 0., z], settings).unwrap();
        let new = mixed
            .add_agent(&meshes, LARGE, [x, 0., z], settings)
            .unwrap();
        let target = nearest(large, [goal, 0., z]);
        original.set_goal(large, &old, Some(&target)).unwrap();
        mixed.set_goal(&meshes, &new, Some(&target)).unwrap();
        handles.push((old, new));
    }
    for _ in 0..200 {
        assert_eq!(
            original.step(large, &mut Budget::unlimited()).unwrap(),
            mixed.step(&meshes, &mut Budget::unlimited()).unwrap(),
        );
        assert_eq!(original.digest(), mixed.digest());
        for (old, new) in &handles {
            assert_eq!(
                original.position(old).unwrap().map(f32::to_bits),
                mixed.agent(new).unwrap().position().map(f32::to_bits),
            );
        }
    }
}

#[cfg(feature = "dynamic")]
#[test]
fn gate_change_replans_only_agents_on_the_edited_profile() {
    use navmesh_rs::crowd::TargetState;
    use navmesh_rs::dynamic::{DynamicLayer, Effect, ModifierId, ModifierSpec, Selector};

    let mut scene = Scene::default();
    scene.floor([0., 0., 22., 8.], 0.);
    let volume_verts = [10., 0., 0., 10., 0., 8., 12., 0., 8., 12., 0., 0.];
    let mut volume = AreaVolume::default();
    volume.verts = &volume_verts;
    volume.area = 4;
    volume.hmin = -1.;
    volume.hmax = 1.;
    let volumes = [volume];
    let mut input = Input::new(&scene.verts, &scene.tris);
    input.volumes = &volumes;
    let mut meshes = install_sizes(&input);

    let gate = ModifierId(1);
    let mut layer = DynamicLayer::new(2, Limits::UNLIMITED).unwrap();
    layer
        .upsert(ModifierSpec::new(
            gate,
            Selector::Prism {
                verts_xz: vec![[10., 0.], [12., 0.], [12., 8.], [10., 8.]],
                ymin: -1.,
                ymax: 1.,
                area: Some(4),
            },
            Effect::ClearFlags(u16::MAX),
        ))
        .unwrap();
    layer.set_enabled(gate, false).unwrap();
    meshes.set_dynamic_layer(SMALL, Some(layer)).unwrap();
    let mut world =
        MixedProfileCrowdWorld::new(&meshes, &[SMALL, LARGE], &params(), Limits::default())
            .unwrap();
    let [small, large] = agents();
    let a = world
        .add_agent(&meshes, SMALL, [4., 0., 2.], small)
        .unwrap();
    let b = world
        .add_agent(&meshes, LARGE, [4., 0., 6.], large)
        .unwrap();
    for (h, id, z) in [(&a, SMALL, 2.), (&b, LARGE, 6.)] {
        world
            .set_goal(
                &meshes,
                h,
                Some(&nearest(meshes.get(id).unwrap(), [18., 0., z])),
            )
            .unwrap();
    }
    world.step(&meshes, &mut Budget::unlimited()).unwrap();
    assert!(meshes.set_modifier_enabled(SMALL, gate, true).unwrap());
    let closed = world.step(&meshes, &mut Budget::unlimited()).unwrap();
    assert_eq!(closed.value().replans, 1);
    assert_eq!(world.agent(&a).unwrap().target_state(), TargetState::Failed);
    assert_eq!(world.agent(&b).unwrap().target_state(), TargetState::Valid);

    assert!(meshes.set_modifier_enabled(SMALL, gate, false).unwrap());
    let open = world.step(&meshes, &mut Budget::unlimited()).unwrap();
    assert_eq!(open.value().replans, 1);
    assert_eq!(world.agent(&a).unwrap().target_state(), TargetState::Valid);
}

#[test]
fn external_traversal_is_resolved_on_the_admitted_profile() {
    use navmesh_rs::crowd::OffMeshMode;

    let scene = Scene::fixture();
    let input = scene.input(&[], None);
    let meshes = install_sizes(&input);
    let mut world =
        MixedProfileCrowdWorld::new(&meshes, &[SMALL, LARGE], &params(), Limits::default())
            .unwrap();
    let [_, mut large] = agents();
    large.off_mesh = OffMeshMode::External;
    let agent = world
        .add_agent(&meshes, LARGE, [7., 0., 41.], large)
        .unwrap();
    world
        .set_goal(
            &meshes,
            &agent,
            Some(&nearest(meshes.get(LARGE).unwrap(), [26., 4., 42.])),
        )
        .unwrap();
    let event = (0..100)
        .find_map(|_| {
            world.step(&meshes, &mut Budget::unlimited()).unwrap();
            world.off_mesh_arrivals().next()
        })
        .unwrap();
    assert_eq!(event.0.index(), agent.index());
    assert_eq!(event.1, LARGE);
    assert_eq!(event.2.user_id, 19);
    let bytes = world.save(&meshes).unwrap();
    let mut world = MixedProfileCrowdWorld::restore(&meshes, &bytes, Limits::default()).unwrap();
    let agent = world.agent_at(agent.index()).unwrap();
    world
        .complete_off_mesh(&meshes, &agent, event.2.end)
        .unwrap();
    let landed = world.agent(&agent).unwrap();
    assert_eq!(landed.state(), AgentState::Walking);
    assert_eq!(landed.position(), event.2.end);
    assert_eq!(world.off_mesh_arrivals().count(), 0);
}

#[test]
fn framed_mixed_rider_keeps_profile_through_lift_and_snapshot() {
    use navmesh_rs::bake::profiles::FramedMixedProfileCrowdWorld;
    use navmesh_rs::crowd::platforms::PlatformAgentId;
    use navmesh_rs::detour::LinkId;
    use navmesh_rs::frames::{DockSpec, FrameId, FramePoint, FramePose, FrameSet};

    let mut world = Scene::default();
    world.floor([0., 0., 12., 10.], 0.);
    world.floor([0., 0., 12., 10.], 4.);
    let mut cabin = Scene::default();
    cabin.floor([0., 0., 6., 10.], 0.);
    let mut frames = FrameSet::with_profiles(install_sizes(&Input::new(&world.verts, &world.tris)));
    let world = FrameId::WORLD;
    let deck = FrameId(1);
    let pose = |y| FramePose::new([8., y, 0.], 1., 0.).unwrap();
    frames
        .add_profile_frame(
            deck,
            install_sizes(&Input::new(&cabin.verts, &cabin.tris)),
            pose(0.),
        )
        .unwrap();
    let at = FramePoint::new(deck, [2., 0.25, 5.]);
    let board = LinkId(1);
    let exit = LinkId(2);
    frames
        .add_dock(
            DockSpec::new(board, FramePoint::new(world, [10., 0.25, 5.]), at, 0.5),
            true,
        )
        .unwrap();
    frames
        .add_dock(
            DockSpec::new(exit, at, FramePoint::new(world, [10., 4.25, 5.]), 0.5),
            false,
        )
        .unwrap();

    let mut crowd = FramedMixedProfileCrowdWorld::default();
    for frame in [world, deck] {
        crowd
            .attach(
                &frames,
                frame,
                &[SMALL, LARGE],
                &params(),
                Limits::default(),
            )
            .unwrap();
    }
    crowd.set_wait_cost(&frames, exit, Some(2.)).unwrap();
    let [small, large] = agents();
    let rider = PlatformAgentId(1);
    crowd
        .add_agent(
            &frames,
            rider,
            LARGE,
            FramePoint::new(world, [7., 0.25, 5.]),
            large,
        )
        .unwrap();
    crowd
        .add_agent(
            &frames,
            PlatformAgentId(2),
            SMALL,
            FramePoint::new(deck, [4., 0.25, 8.]),
            small,
        )
        .unwrap();
    crowd
        .set_goal(&frames, rider, Some(FramePoint::new(world, [7., 4.25, 5.])))
        .unwrap();
    for _ in 0..180 {
        crowd.step(&frames, &mut Budget::unlimited()).unwrap();
        if crowd.agent(rider).unwrap().0 == deck {
            break;
        }
    }
    let (frame, profile, rider_view) = crowd.agent(rider).unwrap();
    assert_eq!((frame, profile), (deck, LARGE));
    let local = rider_view.position();
    let snapshot = crowd.save(&frames).unwrap();
    let frame_snapshot = frames.to_bytes();
    frames.restore(&frame_snapshot).unwrap();
    crowd = FramedMixedProfileCrowdWorld::restore(&frames, &snapshot, Limits::default()).unwrap();
    assert_eq!(crowd.save(&frames).unwrap(), snapshot);

    frames.set_dock_enabled(board, false).unwrap();
    for tick in 1..=40 {
        frames.set_pose(deck, pose(tick as f32 / 10.)).unwrap();
        crowd.step(&frames, &mut Budget::unlimited()).unwrap();
        let (frame, profile, agent) = crowd.agent(rider).unwrap();
        assert_eq!((frame, profile), (deck, LARGE));
        assert_eq!(agent.position(), local);
        assert!(
            (crowd.world_position(&frames, rider).unwrap()[1] - (local[1] + tick as f32 / 10.))
                .abs()
                < 0.000_01
        );
    }
    frames.set_dock_enabled(exit, true).unwrap();
    for _ in 0..180 {
        crowd.step(&frames, &mut Budget::unlimited()).unwrap();
        let (frame, _, agent) = crowd.agent(rider).unwrap();
        if frame == world && agent.position()[0] < 7.35 {
            break;
        }
    }
    let (frame, profile, agent) = crowd.agent(rider).unwrap();
    assert_eq!((frame, profile), (world, LARGE));
    assert!(agent.position()[1] > 4. && agent.position()[0] < 7.35);
}
