#![cfg(feature = "detour")]

use navmesh_rs::detour::*;
use navmesh_rs::{Error, Limits, SnapshotError};

const START: [f32; 3] = [5., 0., 5.];
const END: [f32; 3] = [35., 0., 5.];
const ID: LinkId = LinkId(73);

fn params() -> NavMeshParams {
    NavMeshParams {
        orig: [0.; 3],
        tile_width: 10.,
        tile_height: 10.,
        max_tiles: 16,
        max_polys: 64,
    }
}

fn quad(x: i32, spare: u32, flags: u16) -> Vec<u8> {
    let x0 = x as f32 * 10.;
    create_nav_mesh_data_with_spare_links(
        &NavMeshCreateParams {
            verts: &[0, 0, 0, 0, 0, 10, 10, 0, 10, 10, 0, 0],
            vert_count: 4,
            polys: &[0, 1, 2, 3, 0x800f, 0x800f, 0x800f, 0x800f],
            poly_count: 1,
            nvp: 4,
            poly_flags: &[flags],
            poly_areas: &[0],
            tile_x: x,
            bmin: [x0, 0., 0.],
            bmax: [x0 + 10., 1., 10.],
            cs: 1.,
            ch: 1.,
            walkable_climb: 0.5,
            build_bv_tree: true,
            ..Default::default()
        },
        spare,
    )
    .unwrap()
}

fn scene(a: u32, b: u32) -> (NavMesh, PolyRef, PolyRef) {
    let mut nav = NavMesh::new(&params()).unwrap();
    nav.set_link_layer(Some(1)).unwrap();
    let start = nav.add_tile(quad(0, a, 1)).unwrap();
    let end = nav.add_tile(quad(3, b, 1)).unwrap();
    (nav, start, end)
}

fn spec() -> LinkSpec {
    LinkSpec::new(ID, START, END, 0.75)
}

fn path(nav: &NavMesh, refs: [PolyRef; 2], points: [[f32; 3]; 2]) -> Vec<PolyRef> {
    let mut out = Vec::new();
    NavMeshQuery::new(nav, 64)
        .unwrap()
        .find_path(
            refs[0],
            refs[1],
            &points[0],
            &points[1],
            &DefaultFilter::default(),
            &mut out,
            16,
        )
        .unwrap();
    out
}

#[test]
fn runtime_link_changes_keep_ground_refs_and_have_no_dangling_edges() {
    let (mut nav, a, b) = scene(1, 1);
    let before = path(&nav, [a, b], [START, END]);
    assert_eq!(before, [a]);
    let mut s = spec();
    s.bidirectional = true;
    let r = nav.add_runtime_link(s, true).unwrap();
    assert_eq!(
        (r.start, r.end),
        (Landing::Connected(a), Landing::Connected(b))
    );
    let forward = path(&nav, [a, b], [START, END]);
    let link = forward[1];
    assert_eq!(forward, [a, link, b]);
    assert_eq!(path(&nav, [b, a], [END, START]), [b, link, a]);
    assert_eq!(nav.runtime_link_id(link), Some(ID));
    let original = nav.residency().to_bytes();
    nav.set_runtime_link_enabled(ID, false).unwrap();
    assert_eq!(path(&nav, [a, b], [START, END]), before);
    assert_eq!(nav.residency().to_bytes(), original);
    nav.set_runtime_link_enabled(ID, true).unwrap();
    assert_eq!(path(&nav, [a, b], [START, END]), forward);
    nav.remove_runtime_link(ID).unwrap();
    assert_eq!(nav.ref_state(link), RefState::Stale);
    for r in [a, b] {
        let (t, p) = nav.tile_and_poly_by_ref(r).unwrap();
        let mut next = p.first_link;
        while let Some(edge) = t.links.get(next as usize) {
            assert_eq!(nav.ref_state(edge.r#ref), RefState::Valid);
            next = edge.next;
        }
    }
    assert_eq!(path(&nav, [a, b], [START, END]), before);
}

#[cfg(feature = "dynamic")]
#[test]
fn closing_a_gate_on_a_runtime_landing_keeps_the_link_out_of_paths() {
    use navmesh_rs::dynamic::{DynamicLayer, Effect, ModifierId, ModifierSpec, Selector};

    let (mut nav, a, b) = scene(1, 0);
    let gate = ModifierId(0);
    let mut layer = DynamicLayer::new(1, Limits::UNLIMITED).unwrap();
    layer
        .upsert(ModifierSpec::new(
            gate,
            Selector::Prism {
                verts_xz: vec![[30., 0.], [40., 0.], [40., 10.], [30., 10.]],
                ymin: -1.,
                ymax: 1.,
                area: None,
            },
            Effect::ClearFlags(u16::MAX),
        ))
        .unwrap();
    nav.set_dynamic_layer(Some(layer));
    assert!(nav.add_runtime_link(spec(), true).unwrap().is_traversable());
    // Starting behind the connection makes its departure point a better
    // heuristic candidate than the start if the link is wrongly searched.
    let points = [[1., 0., 5.], END];
    let open = path(&nav, [a, b], points);
    assert_eq!(open.len(), 3);
    assert_eq!(nav.runtime_link_id(open[1]), Some(ID));
    assert_eq!(open[2], b);

    nav.set_modifier_enabled(gate, true).unwrap();
    assert_eq!(path(&nav, [a, b], points), [a]);
    assert!(nav.runtime_link_report(ID).unwrap().enabled);
    nav.set_modifier_enabled(gate, false).unwrap();
    assert_eq!(path(&nav, [a, b], points), open);
}

#[cfg(feature = "dynamic")]
#[test]
fn runtime_refresh_preserves_link_modifiers_and_explicit_disable_wins() {
    use navmesh_rs::dynamic::{DynamicLayer, Effect, ModifierId, ModifierSpec, Selector};

    let (mut nav, a, b) = scene(1, 0);
    let selector = Selector::Links {
        user_id_min: ID.0,
        user_id_max: ID.0,
    };
    let mut layer = DynamicLayer::new(2, Limits::UNLIMITED).unwrap();
    let gate = ModifierId(0);
    let cost = ModifierId(1);
    layer
        .upsert(ModifierSpec::new(
            gate,
            selector.clone(),
            Effect::ClearFlags(u16::MAX),
        ))
        .unwrap();
    layer
        .upsert(ModifierSpec::new(cost, selector, Effect::SetArea(7)))
        .unwrap();
    layer.set_enabled(gate, true).unwrap();
    layer.set_enabled(cost, true).unwrap();
    nav.set_dynamic_layer(Some(layer));

    nav.add_runtime_link(spec(), true).unwrap();
    let points = [START, END];
    assert_eq!(path(&nav, [a, b], points), [a]);
    nav.set_runtime_link_enabled(ID, false).unwrap();
    nav.set_modifier_enabled(gate, false).unwrap();
    assert_eq!(path(&nav, [a, b], points), [a]);
    nav.set_runtime_link_enabled(ID, true).unwrap();
    let open = path(&nav, [a, b], points);
    assert_eq!(open.len(), 3);
    let (_, poly) = nav.tile_and_poly_by_ref(open[1]).unwrap();
    assert_eq!((poly.flags, poly.area()), (1, 7));
}

#[test]
fn capacity_failure_is_atomic_and_disabled_links_keep_their_reservations() {
    let (mut nav, a, b) = scene(1, 0);
    let mut two_way = spec();
    two_way.bidirectional = true;
    let expected = Error::NoLinkCapacity {
        x: 3,
        y: 0,
        layer: 0,
    };
    let before = (
        nav.epoch(),
        nav.residency().to_bytes(),
        nav.runtime_links_to_bytes(),
    );
    assert_eq!(nav.add_runtime_link(two_way, true), Err(expected));
    assert_eq!(
        (
            nav.epoch(),
            nav.residency().to_bytes(),
            nav.runtime_links_to_bytes()
        ),
        before
    );
    nav.add_runtime_link(spec(), false).unwrap();
    let other = LinkSpec::new(LinkId(1), START, END, 0.75);
    assert_eq!(
        nav.add_runtime_link(other, true),
        Err(Error::NoLinkCapacity {
            x: 0,
            y: 0,
            layer: 0
        })
    );
    assert_eq!(nav.runtime_link_reports().count(), 1);
    nav.set_runtime_link_enabled(ID, true).unwrap();
    assert_eq!(path(&nav, [a, b], [START, END]).len(), 3);
    assert_eq!(path(&nav, [b, a], [END, START]), [b]);
}

#[test]
fn disabled_link_survives_replace_streaming_and_snapshot() {
    let (mut nav, a, b) = scene(1, 0);
    nav.add_runtime_link(spec(), true).unwrap();
    let link = path(&nav, [a, b], [START, END])[1];
    nav.set_runtime_link_enabled(ID, false).unwrap();
    let u = nav.unload_tile(b).unwrap();
    assert_eq!(
        nav.runtime_link_report(ID).unwrap().end,
        Landing::Unloaded(TileKey { x: 3, y: 0 })
    );
    let t = prepare_tile(nav.params(), nav.limits(), u.bytes).unwrap();
    assert_eq!(nav.commit_with_state(t, &u.state).unwrap(), b);
    let replacement = prepare_tile(nav.params(), nav.limits(), quad(3, 0, 3)).unwrap();
    let Replaced::Replaced { new: b, .. } = nav.replace_tile(replacement).unwrap() else {
        panic!("different ground content should replace");
    };
    assert_eq!(
        nav.runtime_link_report(ID).unwrap().end,
        Landing::Connected(b)
    );
    assert_eq!(path(&nav, [a, b], [START, END]), [a]);
    let bytes = nav.runtime_links_to_bytes();
    let mut restored = NavMesh::new(&params()).unwrap();
    restored.restore_residency(&nav.residency()).unwrap();
    let mut future = bytes.clone();
    future[4] = 2;
    assert_eq!(
        restored.restore_runtime_links(&future),
        Err(Error::InvalidSnapshot(SnapshotError::WrongVersion))
    );
    assert_eq!(
        restored.restore_runtime_links(&bytes[..bytes.len() - 36]),
        Err(Error::InvalidSnapshot(SnapshotError::TooShort))
    );
    let mut damaged = bytes.clone();
    damaged[20] ^= 1;
    assert!(matches!(
        restored.restore_runtime_links(&damaged),
        Err(Error::InvalidSnapshot(SnapshotError::InvalidValue(_)))
    ));
    restored.restore_runtime_links(&bytes).unwrap();
    assert_eq!(restored.runtime_links_to_bytes(), bytes);
    assert_eq!(restored.runtime_link_id(link), Some(ID));
    assert_eq!(restored.add_tile(quad(0, 1, 1)).unwrap(), a);
    assert_eq!(restored.add_tile(quad(3, 0, 3)).unwrap(), b);
    assert_eq!(path(&restored, [a, b], [START, END]), [a]);
    restored.set_runtime_link_enabled(ID, true).unwrap();
    assert_eq!(path(&restored, [a, b], [START, END]), [a, link, b]);
}

#[test]
fn link_with_missing_ground_is_reported_and_rechecks_capacity_on_install() {
    let (mut nav, a, b) = scene(1, 0);
    let u = nav.unload_tile(b).unwrap();
    let mut s = spec();
    s.bidirectional = true;
    assert!(!nav.add_runtime_link(s, true).unwrap().is_traversable());
    assert_eq!(nav.add_tile(u.bytes).unwrap(), b);
    assert_eq!(
        nav.runtime_link_report(ID).unwrap().end,
        Landing::NoLinkCapacity {
            key: TileKey { x: 3, y: 0 },
            layer: 0,
        }
    );
    assert_eq!(path(&nav, [a, b], [START, END]), [a]);
    let t = prepare_tile(&params(), Limits::UNLIMITED, quad(3, 1, 1)).unwrap();
    assert!(matches!(nav.replace_tile(t), Ok(Replaced::Replaced { .. })));
    assert!(nav.runtime_link_report(ID).unwrap().is_traversable());
}

#[cfg(feature = "crowd")]
#[test]
fn crowd_replans_on_toggle_and_round_trips_external_traversal() {
    use navmesh_rs::Workspace;
    use navmesh_rs::crowd::{AgentParams, AgentState, Crowd, OffMeshMode};

    let (mut nav, _, b) = scene(1, 0);
    nav.add_runtime_link(spec(), true).unwrap();
    let mut ws = Workspace::new();
    let mut p = AgentParams::default();
    p.off_mesh = OffMeshMode::External;
    let mut crowd = Crowd::new(&mut ws, &nav, 1, 0.6).unwrap();
    let a = crowd.add_agent(&mut ws, &nav, [2., 0., 5.], p).unwrap();
    crowd.request_move_target(&nav, a, b, END).unwrap();
    crowd.update(&mut ws, &nav, 0.).unwrap();
    assert_eq!(crowd.agent(a).unwrap().corridor.path().len(), 3);
    nav.set_runtime_link_enabled(ID, false).unwrap();
    crowd.update(&mut ws, &nav, 0.).unwrap();
    assert_eq!(crowd.agent(a).unwrap().corridor.path().len(), 1);
    nav.set_runtime_link_enabled(ID, true).unwrap();
    for _ in 0..80 {
        crowd.update(&mut ws, &nav, 0.05).unwrap();
        if crowd.off_mesh_arrivals().next().is_some() {
            break;
        }
    }
    let (id, r) = crowd.off_mesh_arrivals().next().unwrap();
    assert_eq!((id, r.link_id, r.start, r.end), (a, Some(ID), START, END));
    nav.set_runtime_link_enabled(ID, false).unwrap();
    crowd.update(&mut ws, &nav, 0.05).unwrap();
    assert_eq!(crowd.agent(a).unwrap().state, AgentState::Offmesh);
    assert_eq!(crowd.agent(a).unwrap().npos, START);
    assert!(crowd.complete_off_mesh(&nav, a, START).is_err());
    crowd.complete_off_mesh(&nav, a, END).unwrap();
    crowd.update(&mut ws, &nav, 0.05).unwrap();
    assert_eq!(crowd.agent(a).unwrap().npos, END);
    assert_eq!(crowd.agent(a).unwrap().state, AgentState::Walking);
}

#[cfg(feature = "crowd-world")]
#[test]
fn world_replans_and_can_abort_or_restore_a_pending_external_traversal() {
    use navmesh_rs::Budget;
    use navmesh_rs::crowd::world::{CrowdWorld, CrowdWorldParams, Goal};
    use navmesh_rs::crowd::{AgentParams, AgentState, ArrivalPolicy, OffMeshMode};

    let (mut nav, _, b) = scene(1, 0);
    nav.add_runtime_link(spec(), true).unwrap();
    let mut w = CrowdWorld::new(&nav, &CrowdWorldParams::default(), Limits::UNLIMITED).unwrap();
    let mut p = AgentParams::default();
    p.off_mesh = OffMeshMode::External;
    p.arrival_policy = ArrivalPolicy::Park;
    let a = w.add_agent(&nav, START, p).unwrap();
    w.set_goal(&nav, a, Goal::Point { poly: b, pos: END })
        .unwrap();
    nav.set_runtime_link_enabled(ID, false).unwrap();
    w.step(&nav, &mut Budget::unlimited()).unwrap();
    assert!(w.off_mesh_arrivals().next().is_none());
    nav.set_runtime_link_enabled(ID, true).unwrap();
    w.step(&nav, &mut Budget::unlimited()).unwrap();
    let (id, reached) = w.off_mesh_arrivals().next().unwrap();
    assert_eq!((id, reached.link_id), (a, Some(ID)));
    let mut restored_nav = NavMesh::new(&params()).unwrap();
    restored_nav.restore_residency(&nav.residency()).unwrap();
    restored_nav
        .restore_runtime_links(&nav.runtime_links_to_bytes())
        .unwrap();
    restored_nav.add_tile(quad(0, 1, 1)).unwrap();
    restored_nav.add_tile(quad(3, 0, 1)).unwrap();
    let mut restored =
        CrowdWorld::from_bytes(&restored_nav, &w.to_bytes(), Limits::UNLIMITED).unwrap();
    assert_eq!(restored.off_mesh_arrivals().next(), Some((a, reached)));
    restored.complete_off_mesh(&restored_nav, a, END).unwrap();
    assert_eq!(restored.agent(a).unwrap().position(), END);
    restored
        .step(&restored_nav, &mut Budget::unlimited())
        .unwrap();
    assert!(restored.agent(a).unwrap().detail().unwrap().arrived());
    nav.set_runtime_link_enabled(ID, false).unwrap();
    w.abort_off_mesh(a).unwrap();
    assert_eq!(w.agent(a).unwrap().position(), START);
    w.step(&nav, &mut Budget::unlimited()).unwrap();
    let agent = w.agent(a).unwrap();
    assert_eq!(agent.state(), AgentState::Walking);
    assert_eq!(
        agent.detail().unwrap().corridor.path(),
        &[reached.departure]
    );
    assert!(w.off_mesh_arrivals().next().is_none());
}

#[cfg(feature = "crowd-world")]
#[test]
fn world_rejects_a_pending_traversal_the_live_mesh_does_not_hold() {
    use navmesh_rs::Budget;
    use navmesh_rs::crowd::world::{CrowdWorld, CrowdWorldParams, Goal};
    use navmesh_rs::crowd::{AgentParams, ArrivalPolicy, OffMeshMode};

    let (mut nav, _, b) = scene(1, 0);
    nav.add_runtime_link(spec(), true).unwrap();
    let mut w = CrowdWorld::new(&nav, &CrowdWorldParams::default(), Limits::UNLIMITED).unwrap();
    let mut p = AgentParams::default();
    p.off_mesh = OffMeshMode::External;
    p.arrival_policy = ArrivalPolicy::Park;
    let a = w.add_agent(&nav, START, p).unwrap();
    w.set_goal(&nav, a, Goal::Point { poly: b, pos: END })
        .unwrap();
    nav.set_runtime_link_enabled(ID, false).unwrap();
    w.step(&nav, &mut Budget::unlimited()).unwrap();
    nav.set_runtime_link_enabled(ID, true).unwrap();
    w.step(&nav, &mut Budget::unlimited()).unwrap();
    assert!(w.off_mesh_arrivals().next().is_some());
    // The same ground tiles without the runtime link: the pending polygon is gone.
    let mut bare = NavMesh::new(&params()).unwrap();
    bare.restore_residency(&nav.residency()).unwrap();
    bare.add_tile(quad(0, 1, 1)).unwrap();
    bare.add_tile(quad(3, 0, 1)).unwrap();
    assert_eq!(
        CrowdWorld::from_bytes(&bare, &w.to_bytes(), Limits::UNLIMITED).err(),
        Some(Error::InvalidSnapshot(SnapshotError::InvalidValue(
            "pending external traversal"
        )))
    );
}

#[cfg(feature = "crowd-world")]
#[test]
fn framed_world_restores_pending_traversal_and_arrival_policy() {
    use navmesh_rs::Budget;
    use navmesh_rs::crowd::platforms::{FramedCrowd, PlatformAgentId};
    use navmesh_rs::crowd::world::{CrowdWorld, CrowdWorldParams};
    use navmesh_rs::crowd::{AgentParams, ArrivalPolicy, OffMeshMode};
    use navmesh_rs::frames::{FrameId, FramePoint, FramePose, FrameSet};

    let frame = FrameId(1);
    let rider = PlatformAgentId(2);
    let (mut mesh, _, _) = scene(1, 0);
    mesh.add_runtime_link(spec(), true).unwrap();
    let local = CrowdWorld::new(&mesh, &CrowdWorldParams::default(), Limits::UNLIMITED).unwrap();
    let mut frames = FrameSet::new(NavMesh::new(&params()).unwrap());
    frames.add_frame(frame, mesh, FramePose::IDENTITY).unwrap();
    let mut crowd = FramedCrowd::<CrowdWorld>::default();
    crowd.attach(&frames, frame, local).unwrap();
    let mut p = AgentParams::default();
    p.off_mesh = OffMeshMode::External;
    p.arrival_policy = ArrivalPolicy::Park;
    p.update_flags = 0;
    crowd
        .add_agent(&frames, rider, FramePoint::new(frame, [3., 0., 5.]), p)
        .unwrap();
    crowd
        .set_goal(&frames, rider, Some(FramePoint::new(frame, END)))
        .unwrap();
    let (local_handle, reached) = (0..120)
        .find_map(|_| {
            crowd.step(&frames, &mut Budget::unlimited()).unwrap();
            crowd.crowd(frame).unwrap().off_mesh_arrivals().next()
        })
        .expect("rider reaches the external link");
    assert_eq!(reached.link_id, Some(ID));
    let bytes = crowd.to_bytes(&frames);
    let mut restored = FramedCrowd::from_bytes(&frames, &bytes, Limits::UNLIMITED).unwrap();
    assert_eq!(restored.to_bytes(&frames), bytes);
    assert_eq!(restored.agent(rider).unwrap().0, frame);
    assert_eq!(
        restored.crowd(frame).unwrap().off_mesh_arrivals().next(),
        Some((local_handle, reached)),
    );
    restored.complete_off_mesh(&frames, rider, END).unwrap();
    restored.step(&frames, &mut Budget::unlimited()).unwrap();
    assert!(restored.agent(rider).unwrap().2.arrived());
}

#[test]
#[ignore = "release qualification over 1,000 runtime links"]
fn a_thousand_links_connect_and_remove_without_ground_salt_advances() {
    let mut p = params();
    p.max_tiles = 1024;
    let mut nav = NavMesh::new(&p).unwrap();
    nav.set_link_layer(Some(1)).unwrap();
    let mut ground = Vec::new();
    for column in 0..250 {
        let x = column * 6;
        let a = nav.add_tile(quad(x, 4, 1)).unwrap();
        let b = nav.add_tile(quad(x + 3, 4, 1)).unwrap();
        ground.extend([a, b]);
    }
    for id in 0..1_000 {
        let x = (id / 4 * 60) as f32;
        let z = (id % 4) as f32 + 3.;
        let start = [x + 5., 0., z];
        let end = [x + 35., 0., z];
        let mut s = LinkSpec::new(LinkId(id), start, end, 0.75);
        s.bidirectional = true;
        let a = ground[(id / 4 * 2) as usize];
        let b = ground[(id / 4 * 2 + 1) as usize];
        let r = nav.add_runtime_link(s, true).unwrap();
        assert_eq!(
            (r.start, r.end),
            (Landing::Connected(a), Landing::Connected(b))
        );
    }
    assert_eq!(
        nav.runtime_link_reports()
            .filter(LinkReport::is_traversable)
            .count(),
        1_000
    );
    assert_eq!(
        path(&nav, [ground[0], ground[1]], [[5., 0., 3.], [35., 0., 3.]]).len(),
        3
    );
    for id in 0..1_000 {
        nav.remove_runtime_link(LinkId(id)).unwrap();
    }
    for r in ground {
        assert_eq!(nav.ref_state(r), RefState::Valid);
        let (_, p) = nav.tile_and_poly_by_ref(r).unwrap();
        assert_eq!(p.first_link, NULL_LINK);
    }
    assert_eq!(nav.runtime_link_reports().count(), 0);
}
