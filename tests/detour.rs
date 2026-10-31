//! Detour API: tile creation and the navmesh on a small mesh, upstream's verdicts
//! on conflicting tiles, typed errors for invalid queries and no panics on
//! corrupted tile bytes. Bit-exactness against the C++ oracle is checked by
//! `nav-parity` over the corpus (see `docs/VERIFICATION.md`).

#![cfg(all(feature = "recast", feature = "detour"))]

use navmesh_rs::detour::{
    self, DefaultFilter, MeshTile, NavMesh, NavMeshCreateParams, NavMeshParams, NavMeshQuery,
    Status,
};
use navmesh_rs::recast::{
    self, CompactHeightfield, ContourSet, Heightfield, PolyMesh, PolyMeshDetail,
};
use navmesh_rs::{Error, Limits, TileDataError, Workspace};

/// Tile bytes for a hand-made 10 x 10 quad at tile (x, y): one quad or two triangles,
/// optionally with two off-mesh connections.
fn mesh_tile(x: i32, y: i32, tris: bool, off_mesh: bool) -> Vec<u8> {
    const N: u16 = 0xffff;
    let verts = [0u16, 0, 0, 0, 0, 10, 10, 0, 10, 10, 0, 0];
    let quad = [0, 1, 2, 3, N, N, N, N, N, N, N, N];
    let two = [
        0, 1, 2, N, N, N, N, N, 1, N, N, N, 0, 2, 3, N, N, N, 0, N, N, N, N, N,
    ];
    let (polys, np): (&[u16], usize) = if tris { (&two, 2) } else { (&quad, 1) };
    let nc = if off_mesh { 2 } else { 0 };
    let con_verts = [2.0, 0.0, 2.0, 8.0, 0.0, 8.0, 3.0, 0.0, 3.0, 7.0, 0.0, 7.0];
    detour::create_nav_mesh_data(&NavMeshCreateParams {
        verts: &verts,
        vert_count: 4,
        polys,
        poly_flags: &[1, 1][..np],
        poly_areas: &[0, 0][..np],
        poly_count: np as i32,
        nvp: 6,
        off_mesh_con_verts: &con_verts[..nc * 6],
        off_mesh_con_rad: &[1.0, 1.0][..nc],
        off_mesh_con_flags: &[1, 1][..nc],
        off_mesh_con_areas: &[0, 0][..nc],
        off_mesh_con_dir: &[1, 1][..nc],
        off_mesh_con_count: nc as i32,
        tile_x: x,
        tile_y: y,
        bmin: [0.0; 3],
        bmax: [10.0, 1.0, 10.0],
        walkable_height: 2.0,
        walkable_radius: 0.5,
        walkable_climb: 0.5,
        cs: 1.0,
        ch: 1.0,
        build_bv_tree: true,
        ..Default::default()
    })
    .unwrap()
}

/// A 10 x 10 tile grid with `max_tiles` slots.
fn grid(max_tiles: i32) -> NavMeshParams {
    NavMeshParams {
        orig: [0.0; 3],
        tile_width: 10.0,
        tile_height: 10.0,
        max_tiles,
        max_polys: 256,
    }
}

fn rd(d: &[u8], at: usize) -> usize {
    i32::from_le_bytes([d[at], d[at + 1], d[at + 2], d[at + 3]]) as usize
}

/// Byte offsets of the bounding-volume tree and of the off-mesh connections.
fn bv_and_off_mesh_at(d: &[u8]) -> (usize, usize) {
    let link = if detour::POLYREF_BITS == 64 { 16 } else { 12 };
    let bv = 100
        + rd(d, 28) * 12
        + rd(d, 24) * 32
        + rd(d, 32) * link
        + rd(d, 36) * 12
        + rd(d, 40) * 12
        + rd(d, 44) * 4;
    (bv, bv + rd(d, 48) * 16)
}

fn rejected(d: Vec<u8>) -> Option<Error> {
    MeshTile::parse(d).err()
}

/// Tile bytes for a flat 10 x 10 square at tile (x, y).
fn square_tile(x: i32, y: i32) -> Vec<u8> {
    let mut ws = Workspace::new();
    let verts = [
        0.0, 0.0, 0.0, 0.0, 0.0, 10.0, 10.0, 0.0, 10.0, 10.0, 0.0, 0.0,
    ];
    let tris = [0, 1, 2, 0, 2, 3];
    let (bmin, bmax) = recast::calc_bounds(&verts);
    let (w, h) = recast::calc_grid_size(&bmin, &bmax, 0.3);
    let mut hf = Heightfield::new(&mut ws, w, h, bmin, bmax, 0.3, 0.2).unwrap();
    let mut areas = [0u8; 2];
    recast::mark_walkable_triangles(&mut ws, 45.0, &verts, &tris, &mut areas).unwrap();
    hf.rasterize_triangles(&mut ws, &verts, &tris, &areas, 4)
        .unwrap();
    let mut chf = CompactHeightfield::build(&mut ws, 10, 4, &hf).unwrap();
    recast::erode_walkable_area(&mut ws, 2, &mut chf).unwrap();
    recast::build_distance_field(&mut ws, &mut chf).unwrap();
    recast::build_regions(&mut ws, &mut chf, 0, 8, 20).unwrap();
    let cset = ContourSet::build(&mut ws, &chf, 1.3, 40, recast::CONTOUR_TESS_WALL_EDGES).unwrap();
    let pmesh = PolyMesh::build(&mut ws, &cset, 6).unwrap();
    let dmesh = PolyMeshDetail::build(&mut ws, &pmesh, &chf, 1.8, 0.2).unwrap();
    let flags = vec![1u16; pmesh.npolys as usize];
    detour::create_nav_mesh_data(&NavMeshCreateParams {
        verts: &pmesh.verts,
        vert_count: pmesh.nverts,
        polys: &pmesh.polys,
        poly_flags: &flags,
        poly_areas: &pmesh.areas,
        poly_count: pmesh.npolys,
        nvp: pmesh.nvp,
        detail_meshes: &dmesh.meshes,
        detail_verts: &dmesh.verts,
        detail_verts_count: (dmesh.verts.len() / 3) as i32,
        detail_tris: &dmesh.tris,
        detail_tri_count: (dmesh.tris.len() / 4) as i32,
        tile_x: x,
        tile_y: y,
        bmin: pmesh.bmin,
        bmax: pmesh.bmax,
        walkable_height: 2.0,
        walkable_radius: 0.6,
        walkable_climb: 0.9,
        cs: 0.3,
        ch: 0.2,
        build_bv_tree: true,
        ..Default::default()
    })
    .unwrap()
}

#[test]
fn square_tile_answers_queries() {
    let (nav, _) = NavMesh::new_single(square_tile(0, 0)).unwrap();
    let mut q = NavMeshQuery::new(&nav, 512).unwrap();
    let f = DefaultFilter::default();
    let (a, pa, over) = q
        .find_nearest_poly(&[2.0, 0.5, 2.0], &[2.0, 4.0, 2.0], &f)
        .unwrap();
    assert!(a != 0 && over);
    assert!((pa[1] - 0.0).abs() < 0.3);
    let (b, pb, _) = q
        .find_nearest_poly(&[8.0, 0.5, 8.0], &[2.0, 4.0, 2.0], &f)
        .unwrap();
    let mut path = Vec::new();
    let st = q.find_path(a, b, &pa, &pb, &f, &mut path, 64).unwrap();
    assert!(!st.has_detail(Status::PARTIAL_RESULT));
    assert_eq!((path[0], *path.last().unwrap()), (a, b));
    // An open square: the straight path is the segment itself.
    let mut corners = Vec::new();
    q.find_straight_path(&pa, &pb, &path, &mut corners, 16, 0)
        .unwrap();
    assert_eq!(corners.len(), 2);
    assert_eq!(corners[1].flags, detour::STRAIGHTPATH_END);
    let (hit, _) = q.raycast(a, &pa, &pb, &f, 64).unwrap();
    assert_eq!(hit.t, f32::MAX);
    let (d, _, _, _) = q.find_distance_to_wall(a, &pa, 5.0, &f).unwrap();
    assert!(d > 0.5 && d < 3.0, "distance to the eroded border: {d}");
}

#[test]
fn tile_slots_follow_upstream_verdicts() {
    let params = NavMeshParams {
        orig: [0.0; 3],
        tile_width: 10.0,
        tile_height: 10.0,
        max_tiles: 4,
        max_polys: 256,
    };
    let mut nav = NavMesh::new(&params).unwrap();
    let t = square_tile(0, 0);
    let r = nav.add_tile(t.clone()).unwrap();
    assert_eq!(nav.add_tile(t.clone()), Err(Error::TileSlotOccupied));
    let mut bad = t.clone();
    bad[3] ^= 1;
    let e = nav.add_tile(bad).unwrap_err();
    assert_eq!(e, Error::InvalidTileData(TileDataError::WrongMagic));
    assert_eq!(
        detour::status_of(&e).0,
        Status::FAILURE | Status::WRONG_MAGIC
    );
    assert_eq!(nav.remove_tile(r).unwrap(), t);
    assert_eq!(nav.remove_tile(r), Err(Error::InvalidRef));
    // Upstream's restore with the stale ref takes back the slot and the old salt.
    assert_eq!(nav.restore_tile_allow_stale(t, r).unwrap(), r);
    // A second tile next to it links across the border.
    let r2 = nav.add_tile(square_tile(1, 0)).unwrap();
    assert_ne!(r, r2);
}

#[test]
fn invalid_queries_are_typed_errors() {
    let (nav, _) = NavMesh::new_single(square_tile(0, 0)).unwrap();
    assert!(NavMeshQuery::new(&nav, 0).is_err());
    let mut q = NavMeshQuery::new(&nav, 64).unwrap();
    let f = DefaultFilter::default();
    let p = [1.0, 0.0, 1.0];
    let e = q
        .find_path(0, 0, &p, &p, &f, &mut Vec::new(), 8)
        .unwrap_err();
    assert_eq!(
        detour::status_of(&e).0,
        Status::FAILURE | Status::INVALID_PARAM
    );
    let bogus = !0;
    assert!(q.closest_point_on_poly(bogus, &p).is_err());
    assert!(
        q.find_nearest_poly(&[f32::NAN, 0.0, 0.0], &[1.0; 3], &f)
            .is_err()
    );
    // No sliced search was started: an update does nothing.
    let (st, iters) = q.update_sliced_find_path(4, &f).unwrap();
    assert!(!st.is_in_progress() && iters == 0);
}

#[test]
fn corrupted_tile_bytes_never_panic() {
    let t = square_tile(0, 0);
    let mut state = 0x2545_f491u32;
    let mut rnd = move || {
        state ^= state << 13;
        state ^= state >> 17;
        state ^= state << 5;
        state
    };
    let f = DefaultFilter::default();
    let mut accepted = 0;
    for i in 0..400 {
        let mut d = t.clone();
        for _ in 0..1 + i % 8 {
            let at = 8 + rnd() as usize % (d.len() - 8);
            d[at] = rnd() as u8;
        }
        if i % 50 == 0 {
            d.truncate(rnd() as usize % d.len());
        }
        let Ok((nav, _)) = NavMesh::new_single(d) else {
            continue;
        };
        accepted += 1;
        let mut q = NavMeshQuery::new(&nav, 128).unwrap();
        let mut frand = || 0.5f32;
        if let Ok((r, p)) = q.find_random_point(&f, &mut frand) {
            let e = [p[0] + 3.0, p[1], p[2] + 3.0];
            let _ = q.find_path(r, r, &p, &e, &f, &mut Vec::new(), 32);
            let _ = q.raycast(r, &p, &e, &f, 32);
            let _ = q.move_along_surface(r, &p, &e, &f, &mut Vec::new(), 16);
            let _ = q.find_polys_around_circle(r, &p, 4.0, &f, &mut Vec::new(), 32);
            let _ = q.find_distance_to_wall(r, &p, 4.0, &f);
        }
    }
    assert!(accepted > 0);
}

#[test]
fn tiles_at_the_grid_edges_have_no_neighbours_beyond_it() {
    let mut nav = NavMesh::new(&grid(4)).unwrap();
    let corners = [
        (i32::MAX, i32::MIN),
        (i32::MIN, i32::MAX),
        (i32::MAX, i32::MAX),
        (i32::MIN, i32::MIN),
    ];
    let refs: Vec<_> = corners
        .iter()
        .map(|&(x, y)| nav.add_tile(mesh_tile(x, y, false, false)).unwrap())
        .collect();
    // Both occupied axes span the full i32 range: the grid area is 2^64.
    let q = NavMeshQuery::new(&nav, 64).unwrap();
    let mut out = Vec::new();
    assert!(
        q.query_polygons(
            &[0.0; 3],
            &[1e30; 3],
            &DefaultFilter::default(),
            &mut out,
            8
        )
        .is_ok()
    );
    for r in refs {
        nav.remove_tile(r).unwrap();
    }
    assert_eq!(nav.occupied_tile_bounds(), None);
    // A rejected tile leaves the slot free for the next one.
    let mut one = NavMesh::new(&grid(1)).unwrap();
    let mut bad = mesh_tile(0, 0, false, false);
    bad[28] = 0xff;
    assert!(one.add_tile(bad).is_err());
    assert!(one.add_tile(mesh_tile(0, 0, false, false)).is_ok());
}

#[test]
fn tile_reader_enforces_limits_before_allocating() {
    let t = mesh_tile(0, 0, true, true);
    let n = t.len();
    let cases: [fn(&mut Limits, usize); 6] = [
        |l, n| l.max_tile_bytes = n - 1,
        |l, _| l.max_tile_polys = 0,
        |l, _| l.max_tile_verts = 0,
        |l, _| l.max_tile_links = 0,
        |l, _| l.max_tile_detail = 0,
        |l, _| l.max_bytes = 0,
    ];
    for set in cases {
        let mut l = Limits::UNLIMITED;
        set(&mut l, n);
        let r = MeshTile::parse_with_limits(t.clone(), &l);
        assert_eq!(r.err(), Some(Error::LimitExceeded));
    }
    assert!(MeshTile::parse_with_limits(t.clone(), &Limits::UNLIMITED).is_ok());
    let limits = Limits {
        max_tile_polys: 1,
        ..Limits::UNLIMITED
    };
    let mut nav = NavMesh::with_limits(&grid(4), limits).unwrap();
    assert_eq!(nav.add_tile(t.clone()), Err(Error::LimitExceeded));
    // A header claiming 2^28 links needs a buffer that holds them.
    let mut d = t;
    d[32..36].copy_from_slice(&(1i32 << 28).to_le_bytes());
    assert_eq!(
        rejected(d),
        Some(Error::InvalidTileData(TileDataError::BadSize))
    );
}

#[test]
fn stale_refs_are_never_resurrected() {
    let mut nav = NavMesh::new(&grid(4)).unwrap();
    let t = mesh_tile(0, 0, false, false);
    let r = nav.add_tile(t.clone()).unwrap();
    nav.remove_tile(r).unwrap();
    assert_eq!(nav.restore_tile(t.clone(), r), Err(Error::InvalidRef));
    assert!(!nav.is_valid_poly_ref(r));
    // The slot's current salt restores it, under a new ref.
    let slot = nav.decode_poly_id_tile(r) as usize;
    let cur = nav.poly_ref_base(slot).unwrap();
    assert_eq!(nav.restore_tile(t, cur), Ok(cur));
    assert!(cur != r && !nav.is_valid_poly_ref(r));
    // A slot that never held a tile takes any salt (rebuilding a saved navmesh).
    let saved = nav.encode_poly_id(7, 2, 0);
    let restored = nav.restore_tile(mesh_tile(1, 0, false, false), saved);
    assert_eq!(restored, Ok(saved));
}

#[test]
fn refs_with_bits_outside_the_layout_are_rejected() {
    // One tile of one polygon: with 32-bit refs the salt field cannot fill the ref.
    let params = NavMeshParams {
        max_polys: 1,
        ..grid(1)
    };
    let mut nav = NavMesh::new(&params).unwrap();
    let r = nav.add_tile(mesh_tile(0, 0, false, false)).unwrap();
    let mut aliases = 0;
    for b in 0..detour::POLYREF_BITS {
        let alias = r | 1 << b;
        if alias == r || nav.decode_poly_id(alias) != nav.decode_poly_id(r) {
            continue;
        }
        aliases += 1;
        assert!(!nav.is_valid_poly_ref(alias));
        assert!(nav.tile_by_ref(alias).is_none());
        assert_eq!(nav.remove_tile(alias), Err(Error::InvalidRef));
    }
    assert!(aliases > 0 || detour::POLYREF_BITS == 64);
    assert!(nav.is_valid_poly_ref(r));
}

#[test]
fn query_scans_and_layouts_are_bounded() {
    for (w, h) in [
        (0.0, 10.0),
        (10.0, f32::NAN),
        (f32::INFINITY, 10.0),
        (-1.0, 10.0),
    ] {
        let p = NavMeshParams {
            tile_width: w,
            tile_height: h,
            ..grid(4)
        };
        assert!(NavMesh::new(&p).is_err());
    }
    let mut nav = NavMesh::new(&grid(4)).unwrap();
    nav.add_tile(mesh_tile(0, 0, false, false)).unwrap();
    nav.add_tile(mesh_tile(1_000_000, 0, false, false)).unwrap();
    let q = NavMeshQuery::new(&nav, 64).unwrap();
    let f = DefaultFilter::default();
    let mut out = Vec::new();
    // A box over the whole i32 grid visits the two tiles, not 2^64 locations.
    let st = q
        .query_polygons(&[0.0; 3], &[1e30; 3], &f, &mut out, 8)
        .unwrap();
    assert_eq!(st, Status::OK);
    // Upstream's BV tree holds 2 nodes per polygon; the spare zeroed node repeats poly 0.
    out.dedup();
    assert_eq!(out, vec![nav.tile_ref(0), nav.tile_ref(1)]);
    let st = q
        .query_polygons(&[0.0; 3], &[1e30; 3], &f, &mut out, 0)
        .unwrap();
    assert!(out.is_empty() && st.has_detail(Status::BUFFER_TOO_SMALL));
    let st = q
        .query_polygons(&[-50.0, 0.0, -50.0], &[1.0; 3], &f, &mut out, 0)
        .unwrap();
    assert_eq!(st, Status::OK);
}

#[test]
fn malformed_bv_trees_and_off_mesh_connections_are_rejected() {
    let t = mesh_tile(0, 0, true, true);
    assert!(MeshTile::parse(t.clone()).is_ok());
    let (bv, con) = bv_and_off_mesh_at(&t);
    let bad_bv = Some(Error::InvalidTileData(TileDataError::BadBvTree));
    // The root's escape index runs past the last node.
    let mut d = t.clone();
    assert!((rd(&d, bv + 12) as i32) < 0);
    let escape = -(rd(&d, 48) as i32) - 1;
    d[bv + 12..bv + 16].copy_from_slice(&escape.to_le_bytes());
    assert_eq!(rejected(d), bad_bv);
    // A leaf with bmin above bmax.
    let mut d = t.clone();
    d[bv + 16..bv + 18].copy_from_slice(&0xffffu16.to_le_bytes());
    assert_eq!(rejected(d), bad_bv);
    // Connection 0 names connection 1's polygon.
    let mut d = t;
    d[con + 28] += 1;
    let bad_con = Some(Error::InvalidTileData(TileDataError::BadOffMeshConnection));
    assert_eq!(rejected(d), bad_con);
}

#[test]
fn non_finite_header_fields_are_rejected() {
    // walkable_height, walkable_radius, walkable_climb, bmin.x, bmax.z, bv_quant_factor.
    for at in [60, 64, 68, 72, 92, 96] {
        let mut d = mesh_tile(0, 0, false, false);
        d[at..at + 4].copy_from_slice(&f32::NAN.to_le_bytes());
        assert_eq!(
            rejected(d),
            Some(Error::InvalidTileData(TileDataError::NonFinite))
        );
    }
}

#[test]
fn accessors_return_none_or_errors_on_bad_indices_and_refs() {
    let (nav, r) = NavMesh::new_single(mesh_tile(0, 0, false, true)).unwrap();
    assert_eq!(nav.poly_ref_base(0), Some(r));
    assert_eq!(nav.poly_ref_base(1), None);
    assert_eq!(nav.poly_ref_base(usize::MAX), None);
    assert!(nav.tile(1).is_none() && nav.tile_ref(1) == 0);
    let off = r | 1;
    assert!(nav.off_mesh_connection_by_ref(off).is_some());
    let bogus = r | 7;
    assert!(nav.off_mesh_connection_by_ref(bogus).is_none());
    assert!(nav.poly_flags(bogus).is_err());
    assert!(nav.off_mesh_connection_poly_end_points(0, bogus).is_err());
    assert!(nav.tile_by_ref(!0).is_none());
}

/// A unit quad in tile (x, 0) of a 1 x 1 grid; the tile at x = 0 owns a bidirectional
/// off-mesh connection from (0.5, 0, 0.5) to (10.5, 0, 0.5), ten tiles away.
fn far_link_tile(x: i32) -> Vec<u8> {
    const N: u16 = 0xffff;
    let con = [0.5, 0.0, 0.5, 10.5, 0.0, 0.5];
    let owner = x == 0;
    detour::create_nav_mesh_data(&NavMeshCreateParams {
        verts: &[0, 0, 0, 0, 0, 1, 1, 0, 1, 1, 0, 0],
        vert_count: 4,
        polys: &[0, 1, 2, 3, N, N, N, N],
        poly_flags: &[1],
        poly_areas: &[0],
        poly_count: 1,
        nvp: 4,
        off_mesh_con_verts: if owner { &con } else { &[] },
        off_mesh_con_rad: if owner { &[0.6] } else { &[] },
        off_mesh_con_flags: if owner { &[1] } else { &[] },
        off_mesh_con_areas: if owner { &[0] } else { &[] },
        off_mesh_con_dir: if owner { &[1] } else { &[] },
        off_mesh_con_count: i32::from(owner),
        tile_x: x,
        tile_y: 0,
        bmin: [x as f32, 0.0, 0.0],
        bmax: [x as f32 + 1.0, 1.0, 1.0],
        walkable_height: 2.0,
        walkable_radius: 0.5,
        walkable_climb: 0.5,
        cs: 1.0,
        ch: 1.0,
        build_bv_tree: true,
        ..Default::default()
    })
    .unwrap()
}

/// Whether a complete path leads from the quad at x = `from` to the one at x = `to`.
fn far_path_found(nav: &NavMesh, from: f32, to: f32) -> bool {
    let mut q = NavMeshQuery::new(nav, 64).unwrap();
    let f = DefaultFilter::default();
    let ext = [0.4, 1.0, 0.4];
    let (pa, pb) = ([from + 0.5, 0.0, 0.5], [to + 0.5, 0.0, 0.5]);
    let ra = q.find_nearest_poly(&pa, &ext, &f).unwrap().0;
    let rb = q.find_nearest_poly(&pb, &ext, &f).unwrap().0;
    let mut path = Vec::new();
    if ra == 0 || rb == 0 || q.find_path(ra, rb, &pa, &pb, &f, &mut path, 16).is_err() {
        return false;
    }
    path.last() == Some(&rb) && path.len() == 3 // quad, off-mesh connection, quad
}

fn remove_at(nav: &mut NavMesh, x: i32) {
    let r = nav.tile_ref(nav.tiles_at(x, 0, 1)[0]);
    nav.remove_tile(r).unwrap();
}

#[test]
fn global_off_mesh_links_resolve_in_any_tile_and_survive_remove_and_re_add() {
    let params = NavMeshParams {
        orig: [0.0; 3],
        tile_width: 1.0,
        tile_height: 1.0,
        max_tiles: 8,
        max_polys: 8,
    };
    // Upstream looks for the far endpoint only in the adjacent tile.
    let mut plain = NavMesh::new(&params).unwrap();
    plain.add_tile(far_link_tile(0)).unwrap();
    plain.add_tile(far_link_tile(10)).unwrap();
    assert!(!far_path_found(&plain, 0.0, 10.0));

    // Opt-in before loading, in either order of loading; either tile removed and
    // added back.
    for order in [[0, 10], [10, 0]] {
        for removed in [0, 10] {
            let mut nav = NavMesh::new(&params).unwrap();
            nav.set_global_off_mesh_links(true);
            for x in order {
                nav.add_tile(far_link_tile(x)).unwrap();
            }
            assert!(far_path_found(&nav, 0.0, 10.0) && far_path_found(&nav, 10.0, 0.0));
            remove_at(&mut nav, removed);
            nav.add_tile(far_link_tile(removed)).unwrap();
            assert!(
                far_path_found(&nav, 0.0, 10.0) && far_path_found(&nav, 10.0, 0.0),
                "order {order:?}, tile {removed} re-added"
            );
            // The links to a removed tile are gone, not left dangling.
            remove_at(&mut nav, 10);
            assert!(!far_path_found(&nav, 0.0, 10.0));
        }
    }

    // Opt-in after loading relinks the resident tiles; opt-out unlinks them.
    let mut nav = NavMesh::new(&params).unwrap();
    nav.add_tile(far_link_tile(0)).unwrap();
    nav.add_tile(far_link_tile(10)).unwrap();
    nav.set_global_off_mesh_links(true);
    assert!(far_path_found(&nav, 0.0, 10.0));
    nav.set_global_off_mesh_links(false);
    assert!(!far_path_found(&nav, 0.0, 10.0));
}
