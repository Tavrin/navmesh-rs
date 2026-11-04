//! Tile streaming: refs that survive unload and identical reload, exact ref states
//! (no stale ref is ever valid again), prepared tiles equal to `add_tile`,
//! unloaded borders in queries, checked resumable searches, residency records,
//! state and budget all-or-nothing, keyed grids, and a load/unload/rebake soak on
//! a generated world against a model. Equality of every query record with the
//! C++ oracle after unloading and recommitting every tile is checked by
//! `nav-parity` with `NAV_PORT_STREAMED=1` (see `docs/VERIFICATION.md`).

#![cfg(all(feature = "recast", feature = "detour"))]

use navmesh_rs::bake::{self, BakeConfig, Input};
use navmesh_rs::detour::{
    self, Committed, DefaultFilter, EdgeKind, NavMesh, NavMeshCreateParams, NavMeshParams,
    NavMeshQuery, PolyRef, RefState, ResidencyRecord, SlotPolicy, Status, TileKey, prepare_tile,
};
use navmesh_rs::{Budget, Error, Limits, TileStateError, Workspace};

/// A 10 x 10 quad at tile `(x, y)` whose four sides are portals; `variant`
/// changes the polygon flags (the content) but not the geometry.
fn quad(x: i32, y: i32, variant: u16) -> Vec<u8> {
    const N: u16 = 0xffff;
    let polys = [0, 1, 2, 3, N, N, 0x8000, 0x8001, 0x8002, 0x8003, N, N];
    let (ox, oz) = (x as f32 * 10.0, y as f32 * 10.0);
    detour::create_nav_mesh_data(&NavMeshCreateParams {
        verts: &[0u16, 0, 0, 0, 0, 10, 10, 0, 10, 10, 0, 0],
        vert_count: 4,
        polys: &polys,
        poly_flags: &[1 + variant],
        poly_areas: &[0],
        poly_count: 1,
        nvp: 6,
        tile_x: x,
        tile_y: y,
        bmin: [ox, 0.0, oz],
        bmax: [ox + 10.0, 1.0, oz + 10.0],
        walkable_height: 2.0,
        walkable_radius: 0.5,
        walkable_climb: 0.5,
        cs: 1.0,
        ch: 1.0,
        build_bv_tree: true,
        ..Default::default()
    })
    .expect("valid streaming fixture")
}

fn params(max_tiles: i32) -> NavMeshParams {
    NavMeshParams {
        orig: [0.0; 3],
        tile_width: 10.0,
        tile_height: 10.0,
        max_tiles,
        max_polys: 64,
    }
}

fn keyed(w: u32, h: u32) -> SlotPolicy {
    SlotPolicy::Keyed {
        origin: [0, 0],
        width: w,
        height: h,
        max_layers: 1,
    }
}

fn commit(nav: &mut NavMesh, bytes: Vec<u8>) -> PolyRef {
    let t = prepare_tile(nav.params(), nav.limits(), bytes).expect("valid streaming fixture");
    nav.commit(t).expect("valid streaming fixture")
}

fn at(nav: &NavMesh, x: i32, y: i32) -> PolyRef {
    nav.tile_ref(nav.tile_index_at(x, y, 0).expect("valid streaming fixture"))
}

fn center(x: i32, y: i32) -> [f32; 3] {
    [x as f32 * 10.0 + 5.0, 0.0, y as f32 * 10.0 + 5.0]
}

#[test]
fn identical_reload_keeps_refs_and_other_content_makes_them_stale_for_good() {
    for policy in [SlotPolicy::Sequential, keyed(4, 4)] {
        let mut nav = NavMesh::with_policy(&params(32), Limits::UNLIMITED, policy)
            .expect("valid streaming fixture");
        for k in 0..16 {
            commit(&mut nav, quad(k % 4, k / 4, 0));
        }
        let r = at(&nav, 1, 1);
        let epoch = nav.epoch();
        let u = nav.unload_tile(r).expect("valid streaming fixture");
        assert_eq!((u.key, u.layer), (TileKey { x: 1, y: 1 }, 0));
        assert!(nav.epoch() > epoch);
        assert_eq!(nav.ref_state(r), RefState::Unloaded(TileKey { x: 1, y: 1 }));
        assert!(nav.tile_and_poly_by_ref(r).is_err());
        assert_eq!(
            commit(&mut nav, u.bytes.clone()),
            r,
            "same content, same ref"
        );
        assert_eq!(nav.ref_state(r), RefState::Valid);
        assert_eq!(
            nav.tile_by_ref(r)
                .expect("valid streaming fixture")
                .header
                .x,
            1
        );

        // Other content in the reservation: the old refs are stale, and stay stale
        // through removal and a reload of the original bytes.
        nav.unload_tile(r).expect("valid streaming fixture");
        let r2 = commit(&mut nav, quad(1, 1, 1));
        assert_ne!(r2, r);
        assert_eq!(nav.ref_state(r), RefState::Stale);
        nav.remove_tile(r2).expect("valid streaming fixture");
        assert_eq!(nav.ref_state(r2), RefState::Stale);
        let r3 = commit(&mut nav, u.bytes.clone());
        assert_eq!(nav.ref_state(r), RefState::Stale);
        assert_eq!(nav.ref_state(r2), RefState::Stale);
        assert_eq!(nav.ref_state(r3), RefState::Valid);

        // A forgotten reservation also makes the refs stale.
        let f = at(&nav, 2, 2);
        nav.unload_tile(f).expect("valid streaming fixture");
        nav.forget_unloaded(TileKey { x: 2, y: 2 }, 0)
            .expect("valid streaming fixture");
        assert_eq!(nav.ref_state(f), RefState::Stale);
        commit(&mut nav, quad(2, 2, 0));
        assert_eq!(nav.ref_state(f), RefState::Stale);
        assert!(nav.forget_unloaded(TileKey { x: 2, y: 2 }, 0).is_err());

        // Refs never issued are invalid, not stale.
        let (salt, slot, _) = nav.decode_poly_id(r3);
        assert_eq!(nav.ref_state(0), RefState::Invalid);
        assert_eq!(
            nav.ref_state(nav.encode_poly_id(salt + 1, slot, 0)),
            RefState::Invalid
        );
        assert_eq!(
            nav.ref_state(nav.encode_poly_id(salt, slot, 1)),
            RefState::Invalid
        );
        assert_eq!(
            nav.ref_state(nav.encode_poly_id(1, 31, 0)),
            RefState::Invalid
        );
    }
}

#[test]
fn prepared_tiles_accept_and_reject_what_add_tile_does() {
    let good = quad(0, 0, 0);
    let mut cases = vec![good.clone(), Vec::new()];
    for cut in [4, 50, 99, 100, good.len() / 2, good.len() - 1] {
        cases.push(good[..cut].to_vec());
    }
    for at in [0, 4, 24, 100, 140] {
        let mut bad = good.clone();
        bad[at] ^= 0x5a;
        cases.push(bad);
    }
    for bytes in cases {
        let mut nav = NavMesh::new(&params(4)).expect("valid streaming fixture");
        let added = nav.add_tile(bytes.clone());
        let prepared = prepare_tile(&params(4), Limits::UNLIMITED, bytes);
        match (added, prepared) {
            (Ok(r), Ok(t)) => {
                let mut other = NavMesh::new(&params(4)).expect("valid streaming fixture");
                assert_eq!(other.commit(t.clone()), Ok(r));
                assert_eq!(other.commit(t), Err(Error::TileSlotOccupied));
            }
            (Err(a), Err(b)) => assert_eq!(a, b),
            (a, b) => panic!("add_tile {a:?}, prepare_tile {:?}", b.map(|_| ())),
        }
    }
}

/// A flat 40 x 40 floor with a wall across the middle, baked into 10-unit tiles.
fn world() -> (NavMeshParams, Vec<(i32, i32, Vec<u8>)>) {
    let mut verts = vec![0., 0., 0., 0., 0., 40., 40., 0., 40., 40., 0., 0.];
    let mut tris = vec![0, 1, 2, 0, 2, 3];
    // A box (walls 3 units high) from x 18 to 22, z 5 to 30.
    let b = verts.len() as i32 / 3;
    for (x, z) in [(18., 5.), (18., 30.), (22., 30.), (22., 5.)] {
        verts.extend([x, 3., z]);
    }
    tris.extend([b, b + 1, b + 2, b, b + 2, b + 3]);
    let input = Input::new(&verts, &tris);
    let baked = bake::tiles(&input, &BakeConfig::default(), &mut Workspace::new())
        .expect("valid streaming fixture");
    let tiles = baked
        .tiles
        .into_iter()
        .filter_map(|t| t.data.map(|d| (t.x, t.y, d)))
        .collect();
    (baked.params, tiles)
}

/// Straight paths, nearest polygons and raycasts between fixed points, without ref
/// values.
fn answers(nav: &NavMesh) -> Vec<f32> {
    let mut q = NavMeshQuery::new(nav, 2048).expect("valid streaming fixture");
    let f = DefaultFilter::default();
    let ext = [2., 4., 2.];
    let pts: Vec<[f32; 3]> = (0..12)
        .map(|i| [3. + (i * 7 % 34) as f32, 0., 2. + (i * 11 % 35) as f32])
        .collect();
    let mut out = Vec::new();
    for (i, a) in pts.iter().enumerate() {
        let b = &pts[(i * 5 + 3) % pts.len()];
        let (ra, pa, _) = q
            .find_nearest_poly(a, &ext, &f)
            .expect("valid streaming fixture");
        let (rb, pb, _) = q
            .find_nearest_poly(b, &ext, &f)
            .expect("valid streaming fixture");
        out.extend(pa);
        if ra == 0 || rb == 0 {
            continue;
        }
        let mut path = Vec::new();
        let st = q
            .find_path(ra, rb, &pa, &pb, &f, &mut path, 256)
            .expect("valid streaming fixture");
        out.push(st.0 as f32);
        let mut corners = Vec::new();
        q.find_straight_path(&pa, &pb, &path, &mut corners, 64, 0)
            .expect("valid streaming fixture");
        for c in corners {
            out.extend(c.pos);
        }
        let (hit, _) = q
            .raycast(ra, &pa, &pb, &f, 64)
            .expect("valid streaming fixture");
        out.push(hit.t);
    }
    out
}

#[test]
fn keyed_layer_ties_match_canonical_sequential_queries() {
    let p = params(8);
    let policy = SlotPolicy::Keyed {
        origin: [0, 0],
        width: 1,
        height: 1,
        max_layers: 2,
    };
    let mut sequential = NavMesh::new(&p).expect("sequential layers");
    let mut keyed = NavMesh::with_policy(&p, Limits::UNLIMITED, policy).expect("keyed layers");
    for layer in 0i32..2 {
        let mut bytes = quad(0, 0, layer as u16);
        bytes[16..20].copy_from_slice(&layer.to_le_bytes());
        sequential.add_tile(bytes.clone()).expect("add layer");
        commit(&mut keyed, bytes);
    }
    let selected = |nav: &NavMesh| {
        let q = NavMeshQuery::new(nav, 32).expect("layer query");
        let (r, point, over) = q
            .find_nearest_poly(&center(0, 0), &[1.; 3], &DefaultFilter::default())
            .expect("nearest layer");
        let (tile, poly) = nav.tile_and_poly_by_ref(r).expect("selected layer");
        (tile.header.layer, poly.flags, point, over)
    };
    let expected = selected(&sequential);
    assert_eq!(
        expected.0, 1,
        "sequential lookup breaks ties with the last installed layer"
    );
    assert_eq!(selected(&keyed), expected);
    let refs: Vec<_> = keyed
        .tiles_at(0, 0, 2)
        .iter()
        .map(|&i| keyed.tile_ref(i))
        .collect();
    let mut unloaded: Vec<_> = refs
        .into_iter()
        .map(|r| keyed.unload_tile(r).expect("unload layer"))
        .collect();
    unloaded.reverse();
    for u in unloaded {
        let t = prepare_tile(&p, Limits::UNLIMITED, u.bytes).expect("prepare layer");
        keyed.commit_with_state(t, &u.state).expect("reload layer");
    }
    assert_eq!(selected(&keyed), expected);
}

#[test]
fn streamed_navmesh_answers_like_a_one_shot_build() {
    let (p, tiles) = world();
    let mut one_shot = NavMesh::new(&p).expect("valid streaming fixture");
    for (_, _, d) in &tiles {
        one_shot
            .add_tile(d.clone())
            .expect("valid streaming fixture");
    }
    let expected = answers(&one_shot);
    let (w, h) = tiles
        .iter()
        .fold((0, 0), |(w, h), (x, y, _)| (w.max(*x + 1), h.max(*y + 1)));
    let policy = keyed(w as u32, h as u32);
    let mut nav =
        NavMesh::with_policy(&p, Limits::UNLIMITED, policy).expect("valid streaming fixture");
    for (_, _, d) in &tiles {
        commit(&mut nav, d.clone());
    }
    assert_eq!(answers(&nav), expected, "canonical commit order");
    // Unload everything and commit again in the canonical order.
    let refs: Vec<PolyRef> = tiles.iter().map(|(x, y, _)| at(&nav, *x, *y)).collect();
    let unloaded: Vec<_> = refs
        .iter()
        .map(|&r| nav.unload_tile(r).expect("valid streaming fixture"))
        .collect();
    for u in unloaded {
        let t = prepare_tile(&p, Limits::UNLIMITED, u.bytes).expect("valid streaming fixture");
        nav.commit_with_state(t, &u.state)
            .expect("valid streaming fixture");
    }
    for &r in &refs {
        assert_eq!(nav.ref_state(r), RefState::Valid);
    }
    assert_eq!(answers(&nav), expected, "after unload and recommit");
}

#[test]
fn queries_tell_unloaded_borders_from_walls_and_world_edges() {
    let mut nav = NavMesh::with_policy(&params(8), Limits::UNLIMITED, keyed(4, 1))
        .expect("valid streaming fixture");
    for x in 0..3 {
        commit(&mut nav, quad(x, 0, 0));
    }
    let (a, c) = (at(&nav, 0, 0), at(&nav, 2, 0));
    nav.unload_tile(at(&nav, 1, 0))
        .expect("valid streaming fixture");
    let f = DefaultFilter::default();
    {
        let mut q = NavMeshQuery::new(&nav, 64).expect("valid streaming fixture");
        let mut path = Vec::new();
        let (st, frontier) = q
            .find_path_report(a, c, &center(0, 0), &center(2, 0), &f, &mut path, 16)
            .expect("valid streaming fixture");
        assert!(st.has_detail(Status::PARTIAL_RESULT));
        assert!(st.has_detail(Status::UNLOADED_BORDER));
        assert_eq!(frontier, vec![TileKey { x: 1, y: 0 }]);
        q.init_sliced_find_path(a, c, &center(0, 0), &center(2, 0), &f, 0)
            .expect("valid sliced search");
        q.update_sliced_find_path(64, &f)
            .expect("valid sliced search");
        let sliced = q
            .finalize_sliced_find_path(&f, &mut path, 16)
            .expect("valid sliced path");
        assert!(sliced.has_detail(Status::UNLOADED_BORDER));
        let (hit, _) = q
            .raycast(a, &center(0, 0), &center(2, 0), &f, 8)
            .expect("valid streaming fixture");
        assert_eq!(hit.edge, EdgeKind::UnloadedBorder(TileKey { x: 1, y: 0 }));
        let (hit, _) = q
            .raycast(a, &center(0, 0), &[-5., 0., 5.], &f, 8)
            .expect("valid streaming fixture");
        assert_eq!(hit.edge, EdgeKind::WorldEdge);
        let (hit, _) = q
            .raycast(a, &center(0, 0), &[3., 0., 6.], &f, 8)
            .expect("valid streaming fixture");
        assert_eq!(hit.edge, EdgeKind::None);
        assert_eq!(
            nav.unloaded_column_at(&center(1, 0)),
            Some(TileKey { x: 1, y: 0 })
        );
    }
    // A column declared known but never loaded is a border too, until forgotten.
    nav.mark_known(TileKey { x: 3, y: 0 }, 1);
    let edge = |nav: &NavMesh| {
        let q = NavMeshQuery::new(nav, 64).expect("valid streaming fixture");
        q.raycast(c, &center(2, 0), &[35., 0., 5.], &f, 8)
            .expect("valid streaming fixture")
            .0
            .edge
    };
    assert_eq!(edge(&nav), EdgeKind::UnloadedBorder(TileKey { x: 3, y: 0 }));
    nav.mark_known(TileKey { x: 3, y: 0 }, 0);
    assert_eq!(edge(&nav), EdgeKind::WorldEdge);
    // Reloaded: the path is complete and carries no border bit.
    commit(&mut nav, quad(1, 0, 0));
    let mut q = NavMeshQuery::new(&nav, 64).expect("valid streaming fixture");
    let mut path = Vec::new();
    let (st, frontier) = q
        .find_path_report(a, c, &center(0, 0), &center(2, 0), &f, &mut path, 16)
        .expect("valid streaming fixture");
    assert_eq!(st, Status::OK);
    assert!(frontier.is_empty());
    assert_eq!(path.len(), 3);
}

#[test]
fn resumable_searches_stop_when_a_touched_tile_goes() {
    let mut nav = NavMesh::with_policy(&params(16), Limits::UNLIMITED, keyed(4, 2))
        .expect("valid streaming fixture");
    for x in 0..4 {
        commit(&mut nav, quad(x, 0, 0));
    }
    let f = DefaultFilter::default();
    let start = at(&nav, 0, 0);
    let mut q = NavMeshQuery::new(&nav, 64).expect("valid streaming fixture");
    let mut s = q
        .begin_polys_around_circle(start, &center(0, 0), 40.0, 16)
        .expect("valid streaming fixture");
    s.resume(&mut q, &f, &mut Budget::new(2))
        .expect("valid streaming fixture");
    assert!(!s.is_done());
    // An addition elsewhere does not stop the search.
    commit(&mut nav, quad(0, 1, 0));
    let mut q = NavMeshQuery::new(&nav, 64).expect("valid streaming fixture");
    s.resume(&mut q, &f, &mut Budget::new(1))
        .expect("valid streaming fixture");
    // Unloading a tile it expanded does.
    nav.unload_tile(at(&nav, 1, 0))
        .expect("valid streaming fixture");
    let mut q = NavMeshQuery::new(&nav, 64).expect("valid streaming fixture");
    assert_eq!(
        s.resume(&mut q, &f, &mut Budget::new(1))
            .expect_err("rejected streaming operation"),
        Error::MeshChanged
    );

    // Untouched removals keep it going until the removal log no longer reaches
    // back to the last check.
    let mut q = NavMeshQuery::new(&nav, 64).expect("valid streaming fixture");
    let far = at(&nav, 3, 0);
    let mut s = q
        .begin_polys_around_circle(far, &center(3, 0), 40.0, 16)
        .expect("valid streaming fixture");
    s.resume(&mut q, &f, &mut Budget::new(1))
        .expect("valid streaming fixture");
    let cycle = |nav: &mut NavMesh, n: usize| {
        for _ in 0..n {
            let u = nav
                .unload_tile(at(nav, 0, 1))
                .expect("valid streaming fixture");
            commit(nav, u.bytes);
        }
    };
    cycle(&mut nav, 100);
    let mut q = NavMeshQuery::new(&nav, 64).expect("valid streaming fixture");
    s.resume(&mut q, &f, &mut Budget::new(1))
        .expect("valid streaming fixture");
    cycle(&mut nav, detour::REMOVAL_LOG_CAPACITY + 1);
    let mut q = NavMeshQuery::new(&nav, 64).expect("valid streaming fixture");
    assert_eq!(
        s.resume(&mut q, &f, &mut Budget::new(1))
            .expect_err("rejected streaming operation"),
        Error::MeshChanged
    );
}

#[test]
fn residency_records_keep_saved_refs_across_sessions() {
    for policy in [SlotPolicy::Sequential, keyed(4, 4)] {
        let mut nav = NavMesh::with_policy(&params(32), Limits::UNLIMITED, policy)
            .expect("valid streaming fixture");
        let tiles: Vec<Vec<u8>> = (0..16).map(|k| quad(k % 4, k / 4, 0)).collect();
        let refs: Vec<PolyRef> = tiles.iter().map(|t| commit(&mut nav, t.clone())).collect();
        nav.unload_tile(refs[3]).expect("valid streaming fixture");
        nav.remove_tile(refs[4]).expect("valid streaming fixture");
        let bytes = nav.residency().to_bytes();
        let rec = ResidencyRecord::from_bytes(&bytes).expect("valid streaming fixture");
        assert_eq!(rec.to_bytes(), bytes);

        let mut next = NavMesh::with_policy(&params(32), Limits::UNLIMITED, policy)
            .expect("valid streaming fixture");
        next.restore_residency(&rec)
            .expect("valid streaming fixture");
        assert!(
            next.restore_residency(&rec).is_err(),
            "restore is single-use even before commit"
        );
        if PolyRef::BITS == 32 {
            let mut p = params(32);
            p.max_polys *= 2;
            let mut incompatible = NavMesh::with_policy(&p, Limits::UNLIMITED, policy)
                .expect("valid streaming fixture");
            assert!(
                incompatible.restore_residency(&rec).is_err(),
                "polygon layout is persisted"
            );
        }
        assert_eq!(
            next.ref_state(refs[0]),
            RefState::Unloaded(TileKey { x: 0, y: 0 })
        );
        assert_eq!(next.ref_state(refs[4]), RefState::Stale);
        for (k, t) in tiles.iter().enumerate().rev() {
            let r = commit(&mut next, t.clone());
            if k != 4 {
                assert_eq!(r, refs[k], "tile {k} keeps its ref in the next session");
            }
        }
        assert_eq!(next.ref_state(refs[4]), RefState::Stale);
        assert!(
            next.restore_residency(&rec).is_err(),
            "only into a fresh navmesh"
        );
        let other = if policy == SlotPolicy::Sequential {
            keyed(4, 4)
        } else {
            SlotPolicy::Sequential
        };
        let mut wrong = NavMesh::with_policy(&params(32), Limits::UNLIMITED, other)
            .expect("valid streaming fixture");
        assert!(wrong.restore_residency(&rec).is_err());

        for cut in 0..bytes.len() {
            assert!(ResidencyRecord::from_bytes(&bytes[..cut]).is_err());
        }
        for i in 0..bytes.len() {
            let mut bad = bytes.clone();
            bad[i] ^= 0x41;
            if let Ok(r) = ResidencyRecord::from_bytes(&bad) {
                let mut fresh = NavMesh::with_policy(&params(32), Limits::UNLIMITED, policy)
                    .expect("valid streaming fixture");
                if fresh.restore_residency(&r).is_err() {
                    assert_eq!(fresh.epoch(), 0);
                    assert!(fresh.residency().is_empty());
                }
            }
        }
    }
}

#[test]
fn state_and_budgeted_commits_are_all_or_nothing() {
    let mut nav = NavMesh::with_policy(&params(8), Limits::UNLIMITED, keyed(2, 2))
        .expect("valid streaming fixture");
    for k in 0..4 {
        commit(&mut nav, quad(k % 2, k / 2, 0));
    }
    let r = at(&nav, 0, 0);
    nav.set_poly_flags(r, 7).expect("valid streaming fixture");
    let u = nav.unload_tile(r).expect("valid streaming fixture");
    let t =
        prepare_tile(nav.params(), nav.limits(), u.bytes.clone()).expect("valid streaming fixture");
    assert_eq!(nav.commit_with_state(t, &u.state), Ok(r));
    assert_eq!(nav.poly_flags(r), Ok(7), "runtime flags survive eviction");

    // Equal initial refs in a fresh mesh must not let another tile's content
    // inherit this state. The identity is carried by the unload blob itself.
    for policy in [SlotPolicy::Sequential, keyed(2, 2)] {
        let mut fresh = NavMesh::with_policy(&params(8), Limits::UNLIMITED, policy)
            .expect("fresh state fixture");
        let other =
            prepare_tile(fresh.params(), fresh.limits(), quad(0, 0, 2)).expect("different content");
        assert_eq!(
            fresh.commit_with_state(other, &u.state),
            Err(Error::InvalidTileState(TileStateError::WrongTile))
        );
        assert_eq!(fresh.epoch(), 0);
        assert!(fresh.residency().is_empty());
        let same = prepare_tile(fresh.params(), fresh.limits(), u.bytes.clone())
            .expect("identical content");
        for cut in u.state.len() - 40..u.state.len() {
            assert!(
                fresh
                    .commit_with_state(same.clone(), &u.state[..cut])
                    .is_err()
            );
            assert_eq!(fresh.epoch(), 0);
        }
        for (offset, error) in [
            (u.state.len() - 40, TileStateError::WrongMagic),
            (u.state.len() - 36, TileStateError::WrongVersion),
            (u.state.len() - 1, TileStateError::WrongTile),
        ] {
            let mut corrupted = u.state.clone();
            corrupted[offset] ^= 1;
            assert_eq!(
                fresh.commit_with_state(same.clone(), &corrupted),
                Err(Error::InvalidTileState(error))
            );
            assert_eq!(fresh.epoch(), 0);
        }
        let mut extra = u.state.clone();
        extra.push(0);
        assert!(fresh.commit_with_state(same.clone(), &extra).is_err());
        assert_eq!(fresh.epoch(), 0);
        assert_eq!(fresh.commit_with_state(same, &u.state), Ok(r));
        assert_eq!(fresh.poly_flags(r), Ok(7));
    }

    let u = nav.unload_tile(r).expect("valid streaming fixture");
    let epoch = nav.epoch();
    let changed =
        prepare_tile(nav.params(), nav.limits(), quad(0, 0, 2)).expect("valid streaming fixture");
    assert_eq!(
        nav.commit_with_state(changed, &u.state),
        Err(Error::InvalidTileState(TileStateError::WrongTile))
    );
    assert_eq!(nav.epoch(), epoch);
    assert_eq!(nav.ref_state(r), RefState::Unloaded(TileKey { x: 0, y: 0 }));

    let t = prepare_tile(nav.params(), nav.limits(), u.bytes).expect("valid streaming fixture");
    let cost = nav.commit_cost(&t);
    assert!(cost > 1);
    let mut small = Budget::new(cost - 1);
    let Committed::Refused(t) = nav
        .commit_budgeted(t, &mut small)
        .expect("valid streaming fixture")
    else {
        panic!("a budget below the cost must refuse");
    };
    assert!(small.is_exhausted());
    assert_eq!(nav.epoch(), epoch);
    let mut enough = Budget::new(cost);
    assert!(matches!(
        nav.commit_budgeted(*t, &mut enough),
        Ok(Committed::Done(x)) if x == r
    ));
    assert_eq!(enough.used(), cost);
}

#[test]
fn keyed_grids_place_tiles_by_key_and_refuse_tiles_outside() {
    let (w, h, max_tiles) = if detour::POLYREF_BITS == 64 {
        (8192, 8192, 1 << 28)
    } else {
        (64, 64, 1 << 14)
    };
    let p = NavMeshParams {
        max_polys: 4,
        ..params(max_tiles)
    };
    let policy = SlotPolicy::Keyed {
        origin: [-(w as i32) / 2, -(h as i32) / 2],
        width: w,
        height: h,
        max_layers: 4,
    };
    let mut nav =
        NavMesh::with_policy(&p, Limits::UNLIMITED, policy).expect("valid streaming fixture");
    let corner = (w as i32 / 2 - 1, h as i32 / 2 - 1);
    let r = commit(&mut nav, quad(corner.0, corner.1, 0));
    assert_eq!(nav.decode_poly_id_tile(r), max_tiles as u32 - 4);
    let r0 = commit(&mut nav, quad(-(w as i32) / 2, -(h as i32) / 2, 0));
    assert_eq!(nav.decode_poly_id_tile(r0), 0);
    let outside = prepare_tile(&p, Limits::UNLIMITED, quad(corner.0 + 1, 0, 0))
        .expect("valid streaming fixture");
    assert_eq!(nav.commit(outside), Err(Error::TileOutsideGrid));
    assert!(
        NavMesh::with_policy(&params(16), Limits::UNLIMITED, keyed(4, 5)).is_err(),
        "the grid must fit max_tiles"
    );
    // Found by the residency_record fuzz target: a grid size whose slot count
    // overflows 64 bits is an error, not an arithmetic overflow.
    let huge = SlotPolicy::Keyed {
        origin: [0, 0],
        width: u32::MAX,
        height: u32::MAX,
        max_layers: u32::MAX,
    };
    assert!(NavMesh::with_policy(&params(16), Limits::UNLIMITED, huge).is_err());
}

/// Content generation per tile key, as the salt rules define it.
#[derive(Clone, Copy, Default)]
struct Model {
    generation: u32,
    resident: Option<u16>,
    parked: Option<u16>,
}

#[test]
fn stream_soak_matches_the_model() {
    const SIDE: i32 = 8;
    let n = (SIDE * SIDE) as usize;
    for policy in [SlotPolicy::Sequential, keyed(SIDE as u32, SIDE as u32)] {
        let mut nav = NavMesh::with_policy(&params(96), Limits::UNLIMITED, policy)
            .expect("valid streaming fixture");
        let mut model = vec![Model::default(); n];
        let mut held: Vec<(PolyRef, usize, u32)> = Vec::new();
        let mut s = 0x2545_f491u32;
        let mut rnd = |m: u32| {
            s ^= s << 13;
            s ^= s >> 17;
            s ^= s << 5;
            s % m
        };
        let f = DefaultFilter::default();
        let (mut unloads, mut reloads_kept) = (0, 0);
        for step in 0..4000 {
            let k = rnd(n as u32) as usize;
            let (x, y) = (k as i32 % SIDE, k as i32 / SIDE);
            let m = &mut model[k];
            match rnd(10) {
                0..=3 if m.resident.is_none() => {
                    let v = if rnd(4) == 0 {
                        1
                    } else {
                        m.parked.unwrap_or(0)
                    };
                    commit(&mut nav, quad(x, y, v));
                    if let Some(p) = m.parked.take() {
                        if p == v {
                            reloads_kept += 1;
                        } else {
                            m.generation += 1;
                        }
                    }
                    m.resident = Some(v);
                }
                4..=6 if m.resident.is_some() => {
                    nav.unload_tile(at(&nav, x, y))
                        .expect("valid streaming fixture");
                    m.parked = m.resident.take();
                    unloads += 1;
                }
                7 if m.resident.is_some() => {
                    nav.remove_tile(at(&nav, x, y))
                        .expect("valid streaming fixture");
                    m.resident = None;
                    m.generation += 1;
                }
                8 if m.resident.is_some() && held.len() < 256 => {
                    held.push((at(&nav, x, y), k, m.generation));
                }
                _ => {
                    // A path between two resident tiles; its frontier is unloaded.
                    let ends: Vec<usize> =
                        (0..n).filter(|&i| model[i].resident.is_some()).collect();
                    if ends.len() >= 2 {
                        let (i, j) = (ends[0], ends[ends.len() - 1]);
                        let c = |i: usize| center(i as i32 % SIDE, i as i32 / SIDE);
                        let (ri, rj) = (
                            at(&nav, i as i32 % SIDE, i as i32 / SIDE),
                            at(&nav, j as i32 % SIDE, j as i32 / SIDE),
                        );
                        let mut q = NavMeshQuery::new(&nav, 512).expect("valid streaming fixture");
                        let mut path = Vec::new();
                        let (_, frontier) = q
                            .find_path_report(ri, rj, &c(i), &c(j), &f, &mut path, 128)
                            .expect("valid streaming fixture");
                        for key in frontier {
                            assert!(nav.is_column_unloaded(key.x, key.y));
                        }
                    }
                }
            }
            if step % 25 == 0 {
                for &(r, k, g) in &held {
                    let m = model[k];
                    let want = if g < m.generation {
                        RefState::Stale
                    } else if m.resident.is_some() {
                        RefState::Valid
                    } else {
                        RefState::Unloaded(TileKey {
                            x: k as i32 % SIDE,
                            y: k as i32 / SIDE,
                        })
                    };
                    assert_eq!(nav.ref_state(r), want, "step {step}");
                    assert_eq!(nav.is_valid_poly_ref(r), want == RefState::Valid);
                }
            }
        }
        assert!(
            unloads > 500 && reloads_kept > 300,
            "{unloads} unloads, {reloads_kept} kept"
        );
        let digest = navmesh_rs::detour::content_hash(&nav.residency().to_bytes());
        let expected = match (PolyRef::BITS, policy == SlotPolicy::Sequential) {
            (64, true) => [
                0x67, 0x6d, 0xe7, 0x81, 0x3d, 0xc5, 0xa7, 0xf7, 0x5a, 0x35, 0xbe, 0x89, 0xf9, 0xc7,
                0xfc, 0x97, 0x91, 0x7e, 0x44, 0x14, 0xf5, 0x2a, 0x7e, 0x14, 0x2d, 0xda, 0x15, 0xa7,
                0xc9, 0xff, 0x40, 0xc0,
            ],
            (64, false) => [
                0x80, 0xf9, 0xd8, 0x2b, 0x9a, 0xb7, 0x91, 0x30, 0xfe, 0x1c, 0x0a, 0x6f, 0xb3, 0x1b,
                0x5d, 0x08, 0x1d, 0x4f, 0xf1, 0x2b, 0xc4, 0x1e, 0x04, 0x04, 0x03, 0x1c, 0x33, 0xb5,
                0x76, 0xc2, 0x58, 0xd2,
            ],
            (32, true) => [
                0xa0, 0x32, 0x02, 0x91, 0x81, 0x08, 0xfe, 0xf9, 0x5d, 0x8f, 0xa1, 0x06, 0x08, 0x6e,
                0x28, 0x8f, 0x7a, 0x97, 0xbb, 0x24, 0x93, 0x75, 0x15, 0xe8, 0xc0, 0x86, 0xe8, 0x6f,
                0xc8, 0x58, 0x40, 0xeb,
            ],
            (32, false) => [
                0x56, 0xa2, 0x9c, 0xc0, 0x37, 0x56, 0x69, 0x0e, 0xd9, 0x52, 0x0a, 0x27, 0x60, 0xf5,
                0x69, 0xc6, 0x7f, 0x0f, 0x48, 0x5b, 0xd4, 0x52, 0x85, 0xc9, 0x68, 0x60, 0xbc, 0x19,
                0xb4, 0xac, 0x77, 0x61,
            ],
            _ => unreachable!("supported reference widths"),
        };
        assert_eq!(digest, expected, "streaming determinism digest");
        eprintln!(
            "stream soak {policy:?}: 4000 operations, {unloads} unloads, {reloads_kept} identical reloads, digest {:02x?}",
            digest
        );
    }
}

#[cfg(feature = "crowd")]
#[test]
fn crowd_agents_wait_at_unloaded_target_and_intermediate_columns() {
    for missing in [2, 3] {
        use navmesh_rs::crowd::{AgentParams, Crowd, TargetState};
        let mut nav = NavMesh::with_policy(&params(8), Limits::UNLIMITED, keyed(4, 1))
            .expect("valid streaming fixture");
        let tiles: Vec<Vec<u8>> = (0..4).map(|x| quad(x, 0, 0)).collect();
        for t in &tiles {
            commit(&mut nav, t.clone());
        }
        let mut ws = Workspace::new();
        let mut c = Crowd::new(&mut ws, &nav, 2, 0.6).expect("valid streaming fixture");
        let a = c
            .add_agent(&mut ws, &nav, center(0, 0), AgentParams::default())
            .expect("valid streaming fixture");
        let target = at(&nav, 3, 0);
        c.request_move_target(&nav, a, target, center(3, 0))
            .expect("valid streaming fixture");
        for _ in 0..10 {
            c.update(&mut ws, &nav, 1. / 30.)
                .expect("valid streaming fixture");
        }
        nav.unload_tile(at(&nav, missing, 0))
            .expect("valid streaming fixture");
        for _ in 0..600 {
            c.update(&mut ws, &nav, 1. / 30.)
                .expect("valid streaming fixture");
        }
        let agent = c.agent(a).expect("valid streaming fixture");
        assert_eq!(agent.target_state, TargetState::WaitingForTile);
        assert_eq!(agent.waiting_tile(), Some(TileKey { x: missing, y: 0 }));
        assert!(
            agent.npos[0] > (missing * 10 - 5) as f32 && agent.npos[0] <= (missing * 10) as f32,
            "waits at the border: {:?}",
            agent.npos
        );
        commit(&mut nav, tiles[missing as usize].clone());
        for _ in 0..300 {
            c.update(&mut ws, &nav, 1. / 30.)
                .expect("valid streaming fixture");
        }
        let agent = c.agent(a).expect("valid streaming fixture");
        assert_eq!(agent.target_state, TargetState::Valid);
        assert!(
            agent.npos[0] > 33.,
            "reached the reloaded tile: {:?}",
            agent.npos
        );
    }
}

/// The old invertible hash's last-word collision must not establish identity.
#[test]
fn crafted_old_hash_collision_rejects_refs_and_saved_state() {
    fn prefix(bytes: &[u8], length: usize) -> u64 {
        let mut h = 0x243f_6a88_85a3_08d3u64 ^ (length as u64).wrapping_mul(0x9e37_79b9_7f4a_7c15);
        for c in bytes.as_chunks::<8>().0 {
            h = (h ^ u64::from_le_bytes(*c)).wrapping_mul(0x9e37_79b9_7f4a_7c15);
            h ^= h >> 29;
        }
        h
    }
    let mut a = quad(0, 0, 0);
    let mut b = a.clone();
    // The first vertex's height, after the 100-byte mesh header.
    b[104..108].copy_from_slice(&0.25f32.to_le_bytes());
    a.resize(a.len().next_multiple_of(8), 0);
    b.resize(b.len().next_multiple_of(8), 0);
    let length = a.len() + 8;
    let correction = prefix(&a, length) ^ prefix(&b, length);
    a.extend_from_slice(&0u64.to_le_bytes());
    b.extend_from_slice(&correction.to_le_bytes());
    assert_eq!(prefix(&a, length), prefix(&b, length));
    assert_ne!(detour::content_hash(&a), detour::content_hash(&b));
    let mut nav = NavMesh::new(&params(1)).expect("mesh");
    let r = commit(&mut nav, a.clone());
    let u = nav.unload_tile(r).expect("unload");
    let before = nav.residency().to_bytes();
    let epoch = nav.epoch();
    let prepared =
        prepare_tile(nav.params(), nav.limits(), b.clone()).expect("trailing bytes accepted");
    assert_eq!(
        nav.commit_with_state(prepared, &u.state),
        Err(Error::InvalidTileState(TileStateError::WrongTile))
    );
    assert_eq!(nav.residency().to_bytes(), before);
    assert_eq!(nav.epoch(), epoch);
    assert_ne!(commit(&mut nav, b), r);
    assert_eq!(nav.ref_state(r), RefState::Stale);
    let mut trailing = a.clone();
    trailing.push(1);
    assert_ne!(detour::content_hash(&a), detour::content_hash(&trailing));
    assert!(prepare_tile(nav.params(), nav.limits(), trailing).is_ok());
    assert_eq!(
        detour::content_hash(b"abc"),
        [
            0xba, 0x78, 0x16, 0xbf, 0x8f, 0x01, 0xcf, 0xea, 0x41, 0x41, 0x40, 0xde, 0x5d, 0xae,
            0x22, 0x23, 0xb0, 0x03, 0x61, 0xa3, 0x96, 0x17, 0x7a, 0x9c, 0xb4, 0x10, 0xff, 0x61,
            0xf2, 0x00, 0x15, 0xad
        ]
    );
}

#[test]
fn salt_exhaustion_never_resurrects_refs_and_is_atomic() {
    let mut nav = NavMesh::new(&params(1)).expect("mesh");
    let bytes = quad(0, 0, 0);
    let first = commit(&mut nav, bytes.clone());
    let salt_bits = if PolyRef::BITS == 64 { 16 } else { 32 - 6 };
    let max_salt = (1u32 << salt_bits) - 1;
    // Reproduce the full 64-bit wrap sequence; larger 32-bit history is restored.
    if PolyRef::BITS == 64 {
        let mut cycles = NavMesh::new(&params(1)).expect("mesh");
        let original = commit(&mut cycles, bytes.clone());
        let mut current = original;
        for _ in 0..max_salt - 1 {
            cycles.remove_tile(current).expect("advance");
            current = commit(&mut cycles, bytes.clone());
        }
        assert_eq!(cycles.ref_state(original), RefState::Stale);
        assert_eq!(cycles.remove_tile(current), Err(Error::SaltExhausted));
        assert_eq!(cycles.ref_state(current), RefState::Valid);
    }
    // Persisted history tests the boundary in both reference widths.
    let mut record = nav.residency().to_bytes();
    record[52..56].copy_from_slice(&(max_salt - 1).to_le_bytes());
    let record = ResidencyRecord::from_bytes(&record).expect("record");
    let mut nav = NavMesh::new(&params(1)).expect("mesh");
    nav.restore_residency(&record).expect("restore history");
    let penultimate = commit(&mut nav, bytes.clone());
    nav.remove_tile(penultimate).expect("last salt advance");
    let last = commit(&mut nav, bytes.clone());
    assert_eq!(nav.decode_poly_id(last).0, max_salt);
    for r in [first, penultimate] {
        assert_eq!(nav.ref_state(r), RefState::Stale);
    }
    let epoch = nav.epoch();
    let before = nav.residency().to_bytes();
    assert_eq!(nav.remove_tile(last), Err(Error::SaltExhausted));
    assert_eq!(nav.epoch(), epoch);
    assert_eq!(nav.residency().to_bytes(), before);
    assert_eq!(nav.ref_state(last), RefState::Valid);
    let u = nav.unload_tile(last).expect("unload at limit");
    let before = nav.residency().to_bytes();
    let epoch = nav.epoch();
    let other = prepare_tile(nav.params(), nav.limits(), quad(0, 0, 1)).expect("prepare");
    assert_eq!(nav.commit(other.clone()), Err(Error::SaltExhausted));
    assert_eq!(
        nav.commit_with_state(other, &u.state),
        Err(Error::SaltExhausted)
    );
    assert_eq!(
        nav.forget_unloaded(TileKey { x: 0, y: 0 }, 0),
        Err(Error::SaltExhausted)
    );
    assert_eq!(nav.epoch(), epoch);
    assert_eq!(nav.residency().to_bytes(), before);
    assert_eq!(commit(&mut nav, bytes), last);
}

#[test]
fn global_landing_layers_have_canonical_endpoints_and_link_order() {
    fn tile(x: i32, layer: i32, height: f32, owner: bool) -> Vec<u8> {
        let ox = x as f32 * 10.;
        detour::create_nav_mesh_data(&NavMeshCreateParams {
            verts: &[0, 0, 0, 0, 0, 10, 10, 0, 10, 10, 0, 0],
            vert_count: 4,
            polys: &[0, 1, 2, 3, 0xffff, 0xffff, 0, 0, 0, 0, 0, 0],
            poly_flags: &[1],
            poly_areas: &[0],
            poly_count: 1,
            nvp: 6,
            tile_x: x,
            tile_layer: layer,
            bmin: [ox, height, 0.],
            bmax: [ox + 10., height + 1., 10.],
            walkable_height: 2.,
            walkable_radius: 0.5,
            walkable_climb: 0.5,
            cs: 1.,
            ch: 1.,
            off_mesh_con_verts: if owner {
                &[5., 0., 5., 105., 0., 5.]
            } else {
                &[]
            },
            off_mesh_con_rad: if owner { &[1.] } else { &[] },
            off_mesh_con_flags: if owner { &[1] } else { &[] },
            off_mesh_con_areas: if owner { &[0] } else { &[] },
            off_mesh_con_dir: if owner { &[1] } else { &[] },
            off_mesh_con_count: i32::from(owner),
            ..Default::default()
        })
        .expect("tile")
    }
    let tiles = [
        tile(0, 0, 0., true),
        tile(10, 0, 0., false),
        tile(10, 1, 0.25, false),
    ];
    let make = |order: [usize; 3]| {
        let mut nav = NavMesh::with_policy(
            &params(32),
            Limits::UNLIMITED,
            SlotPolicy::Keyed {
                origin: [0, 0],
                width: 11,
                height: 1,
                max_layers: 2,
            },
        )
        .expect("mesh");
        nav.set_global_off_mesh_links(true);
        for i in order {
            commit(&mut nav, tiles[i].clone());
        }
        nav
    };
    let snapshot = |nav: &NavMesh| {
        let mut out = Vec::new();
        for x in [0, 10] {
            for layer in 0..2 {
                if let Some(i) = nav.tile_index_at(x, 0, layer) {
                    let t = nav.tile(i).expect("tile");
                    let links: Vec<_> = t
                        .polys
                        .iter()
                        .map(|p| {
                            let mut ls = Vec::new();
                            let mut k = p.first_link;
                            while k != detour::NULL_LINK {
                                let l = t.links[k as usize];
                                ls.push((l.r#ref, l.edge, l.side));
                                k = l.next;
                            }
                            ls
                        })
                        .collect();
                    out.push((t.verts.clone(), links));
                }
            }
        }
        out
    };
    let mut a = make([0, 1, 2]);
    let b = make([0, 2, 1]);
    assert_eq!(snapshot(&a), snapshot(&b));
    let owner = a.tile_ref(a.tile_index_at(0, 0, 0).expect("owner"));
    assert_eq!(
        a.off_mesh_connection_poly_end_points(owner, owner | 1)
            .expect("endpoints")
            .1,
        [105., 0.25, 5.]
    );
    assert_eq!(
        a.off_mesh_connection_poly_end_points(owner, owner | 1),
        b.off_mesh_connection_poly_end_points(owner, owner | 1)
    );
    let r = a.tile_ref(a.tile_index_at(10, 0, 1).expect("landing"));
    let u = a.unload_tile(r).expect("unload winner");
    commit(&mut a, u.bytes);
    assert_eq!(snapshot(&a), snapshot(&b));
    for layer in 0..2 {
        let r = a.tile_ref(a.tile_index_at(10, 0, layer).expect("landing"));
        a.unload_tile(r).expect("unload all landings");
    }
    assert_eq!(
        a.off_mesh_connection_poly_end_points(owner, owner | 1)
            .expect("authored endpoint")
            .1,
        [105., 0., 5.]
    );
}
