#![no_main]
//! Budgeted queries, resumable searches and global off-mesh links on a row of
//! single-polygon tiles that tiles are added to and removed from in any order.
//!
//! Layout: one flag byte (bit 0 turns global off-mesh links on before the first
//! operation), then up to 48 operations of 4 bytes `[op, a, b, c]`:
//!
//! * 0: add tile `a % 6`, with an off-mesh connection to tile `b % 6` when `c & 1`
//!   is set (bidirectional when `c & 2` is set)
//! * 1: remove tile `a % 6`
//! * 2: set global off-mesh links to `a & 1`
//! * 3: single-shot budgeted queries from tile `a % 6` to tile `b % 6` under the
//!   budget `c` (`c >= 200` is large enough for any search on this mesh)
//! * 4: resumable searches from tile `a % 6` to tile `b % 6`, resumed in steps of
//!   the budget `c` (a first step of 0 is always taken) until they finish
//!
//! Properties: a `Complete` outcome equals the unbudgeted call's result; a large
//! budget always completes; resuming to the end equals the unbudgeted call; a
//! budget is never overspent. A panic, an invalid ref accepted or a mismatch is a
//! bug.
use libfuzzer_sys::fuzz_target;
use navmesh_rs::detour::{
    DefaultFilter, NavMesh, NavMeshCreateParams, NavMeshParams, NavMeshQuery, PolyRef,
    create_nav_mesh_data,
};
use navmesh_rs::{Budget, Budgeted, Progress};

const TILES: i32 = 6;
const LARGE: u64 = 1 << 40;

fn tile(x: i32, link: Option<(i32, bool)>) -> Option<Vec<u8>> {
    const N: u16 = 0xffff;
    let (to, bidir) = link.filter(|&(t, _)| t != x).unzip();
    let con = [
        x as f32 + 0.5,
        0.0,
        0.5,
        to.unwrap_or(0) as f32 + 0.5,
        0.0,
        0.5,
    ];
    let has = to.is_some();
    let dir = [u8::from(bidir == Some(true))];
    create_nav_mesh_data(&NavMeshCreateParams {
        verts: &[0, 0, 0, 0, 0, 1, 1, 0, 1, 1, 0, 0],
        vert_count: 4,
        polys: &[0, 1, 2, 3, N, N, N, N],
        poly_flags: &[1],
        poly_areas: &[0],
        poly_count: 1,
        nvp: 4,
        off_mesh_con_verts: if has { &con } else { &[] },
        off_mesh_con_rad: if has { &[0.6] } else { &[] },
        off_mesh_con_flags: if has { &[1] } else { &[] },
        off_mesh_con_areas: if has { &[0] } else { &[] },
        off_mesh_con_dir: if has { &dir } else { &[] },
        off_mesh_con_count: i32::from(has),
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
    .ok()
}

fn budget_of(c: u8) -> u64 {
    if c >= 200 { LARGE } else { u64::from(c) }
}

/// A `Complete` outcome must equal the plain result; a large budget must complete.
fn check<T: PartialEq + core::fmt::Debug>(
    what: &str,
    b: Result<Budgeted<T>, navmesh_rs::Error>,
    plain: Result<T, navmesh_rs::Error>,
    budget: &Budget,
) {
    assert!(budget.used() <= budget.limit(), "{what}: overspent");
    match (b, plain) {
        (Ok(Budgeted::Complete(v)), Ok(p)) => assert_eq!(v, p, "{what}: complete != plain"),
        (Ok(Budgeted::Exhausted(_)), Ok(_)) => {
            assert!(budget.limit() < LARGE, "{what}: large budget exhausted");
        }
        (Ok(Budgeted::Complete(v)), Err(e)) => panic!("{what}: complete {v:?} but plain {e:?}"),
        // The budgeted call may fail on its own limit, or on the partial result.
        _ => {}
    }
}

fn lcg(seed: u32) -> impl FnMut() -> f32 {
    let mut s = seed | 1;
    move || {
        s = s.wrapping_mul(1_664_525).wrapping_add(1_013_904_223);
        (s >> 8) as f32 / (1u32 << 24) as f32
    }
}

fn centre(x: i32) -> [f32; 3] {
    [x as f32 + 0.5, 0.0, 0.5]
}

fn poly_at(q: &NavMeshQuery, x: i32, f: &DefaultFilter) -> PolyRef {
    q.find_nearest_poly(&centre(x), &[0.4, 1.0, 0.4], f)
        .map_or(0, |r| r.0)
}

fn single_shot(nav: &NavMesh, s: i32, e: i32, c: u8) {
    let Ok(mut q) = NavMeshQuery::new(nav, 64) else {
        return;
    };
    let f = DefaultFilter::default();
    let (ps, pe) = (centre(s), centre(e));
    let (rs, re) = (poly_at(&q, s, &f), poly_at(&q, e, &f));
    let ext = [0.4 + f32::from(c % 8), 1.0, 0.4 + f32::from(c % 8)];
    let mut bud = Budget::new(budget_of(c));

    let (mut a, mut b) = (Vec::new(), Vec::new());
    let r = q.query_polygons_budgeted(&ps, &ext, &f, &mut a, 16, &mut bud);
    let p = q.query_polygons(&ps, &ext, &f, &mut b, 16);
    let same = !matches!(r, Ok(Budgeted::Complete(_))) || a == b;
    assert!(same, "query_polygons: output differs");
    check("query_polygons", r, p, &bud);

    let mut bud = Budget::new(budget_of(c));
    check(
        "find_nearest_poly",
        q.find_nearest_poly_budgeted(&ps, &ext, &f, &mut bud),
        q.find_nearest_poly(&ps, &ext, &f),
        &bud,
    );

    let mut bud = Budget::new(budget_of(c));
    let (mut a, mut b) = (Vec::new(), Vec::new());
    let r = q.find_path_budgeted(rs, re, &ps, &pe, &f, &mut a, 16, &mut bud);
    let p = q.find_path(rs, re, &ps, &pe, &f, &mut b, 16);
    let same = !matches!(r, Ok(Budgeted::Complete(_))) || a == b;
    assert!(same, "find_path: path differs");
    check("find_path", r, p, &bud);

    // Straight path over the unbudgeted path, and over a truncated one.
    for n in [b.len(), b.len() / 2] {
        let mut bud = Budget::new(budget_of(c));
        let (mut a, mut o) = (Vec::new(), Vec::new());
        let r = q.find_straight_path_budgeted(&ps, &pe, &b[..n], &mut a, 8, 0, &mut bud);
        let p = q.find_straight_path(&ps, &pe, &b[..n], &mut o, 8, 0);
        let same = !matches!(r, Ok(Budgeted::Complete(_))) || a.len() == o.len();
        assert!(same, "find_straight_path: corner count differs");
        check("find_straight_path", r, p, &bud);
    }

    let mut bud = Budget::new(budget_of(c));
    check(
        "find_random_point",
        q.find_random_point_budgeted(&f, &mut lcg(u32::from(c)), &mut bud),
        q.find_random_point(&f, &mut lcg(u32::from(c)))
            .map(Some)
            .or_else(|e| {
                if e == navmesh_rs::Error::Failed {
                    Ok(None)
                } else {
                    Err(e)
                }
            }),
        &bud,
    );

    let mut bud = Budget::new(budget_of(c));
    let radius = 0.5 + f32::from(c % 16);
    let r = q.find_random_point_around_circle_budgeted(rs, &ps, radius, &f, &mut lcg(7), &mut bud);
    let p = q
        .find_random_point_around_circle(rs, &ps, radius, &f, &mut lcg(7))
        .map(Some);
    check("find_random_point_around_circle", r, p, &bud);
}

/// Resumes `step` until it reports `Done`, with a first step of zero.
fn drive(c: u8, mut step: impl FnMut(&mut Budget) -> Result<Progress, navmesh_rs::Error>) -> bool {
    let mut zero = Budget::new(0);
    match step(&mut zero) {
        Ok(_) => assert_eq!(zero.used(), 0, "zero budget spent work"),
        Err(_) => return false,
    }
    let size = u64::from(c).max(1);
    for _ in 0..100_000 {
        let mut bud = Budget::new(size);
        match step(&mut bud) {
            Ok(Progress::Done) => return true,
            Ok(Progress::Exhausted) => assert!(bud.is_exhausted(), "exhausted without refusal"),
            Err(_) => return false,
            Ok(_) => {}
        }
    }
    panic!("resumable search did not finish in 100000 steps");
}

fn resumable(nav: &NavMesh, s: i32, e: i32, c: u8) {
    let Ok(mut q) = NavMeshQuery::new(nav, 64) else {
        return;
    };
    let f = DefaultFilter::default();
    let (ps, pe) = (centre(s), centre(e));
    let rs = poly_at(&q, s, &f);
    let radius = 0.5 + f32::from(c % 24);

    if let Ok(mut search) = q.begin_polys_around_circle(rs, &ps, radius, 16) {
        if drive(c, |b| search.resume(&mut q, &f, b)) {
            let mut plain = Vec::new();
            let st = q.find_polys_around_circle(rs, &ps, radius, &f, &mut plain, 16);
            assert!(
                st.is_ok(),
                "polys_around: plain failed after search finished"
            );
            assert_eq!(search.results(), &plain[..], "polys_around_circle");
            assert_eq!(Ok(search.status()), st, "polys_around_circle status");
        }
    }
    let h = 0.25 + f32::from(c % 24) * 0.5;
    let shape = [
        ps[0] - h,
        0.0,
        ps[2] - h,
        ps[0] - h,
        0.0,
        ps[2] + h,
        ps[0] + h,
        0.0,
        ps[2] + h,
        ps[0] + h,
        0.0,
        ps[2] - h,
    ];
    if let Ok(mut search) = q.begin_polys_around_shape(rs, &shape, 16) {
        if drive(c, |b| search.resume(&mut q, &f, b)) {
            let mut plain = Vec::new();
            let st = q.find_polys_around_shape(rs, &shape, &f, &mut plain, 16);
            assert_eq!(search.results(), &plain[..], "polys_around_shape");
            assert_eq!(Ok(search.status()), st, "polys_around_shape status");
        }
    }
    if let Ok(mut search) = q.begin_distance_to_wall(rs, &ps, radius) {
        if drive(c, |b| search.resume(&mut q, &f, b)) {
            let p = q.find_distance_to_wall(rs, &ps, radius, &f);
            let (d, hp, hn) = search.result();
            assert_eq!(
                p.map(|(d, a, b, _)| (d, a, b)),
                Ok((d, hp, hn)),
                "distance_to_wall"
            );
        }
    }
    if let Ok(mut search) = q.begin_local_neighbourhood(rs, &ps, radius, 16) {
        if drive(c, |b| search.resume(&mut q, &f, b)) {
            let mut plain = Vec::new();
            let st = q.find_local_neighbourhood(rs, &ps, radius, &f, &mut plain, 16);
            assert_eq!(search.results(), &plain[..], "local_neighbourhood");
            assert_eq!(Ok(search.status()), st, "local_neighbourhood status");
        }
    }
    if let Ok(mut search) = q.begin_move_along_surface(rs, &ps, &pe, 16) {
        if drive(c, |b| search.resume(&mut q, &f, b)) {
            let (mut v1, mut v2) = (Vec::new(), Vec::new());
            let (pos, st) = search.result(&q, &mut v1);
            let p = q.move_along_surface(rs, &ps, &pe, &f, &mut v2, 16);
            assert_eq!(p, Ok((pos, st)), "move_along_surface");
            assert_eq!(v1, v2, "move_along_surface visited");
        }
    }
    if let Ok(mut search) = q.begin_raycast(rs, &ps, &pe, 0, 16, 0) {
        if drive(c, |b| search.resume(&q, &f, b)) {
            let p = q.raycast(rs, &ps, &pe, &f, 16);
            let (hit, st) = search.result();
            assert_eq!(p, Ok((hit.clone(), st)), "raycast");
        }
    }
}

fuzz_target!(|data: &[u8]| {
    let Some((&flags, ops)) = data.split_first() else {
        return;
    };
    let params = NavMeshParams {
        orig: [0.0; 3],
        tile_width: 1.0,
        tile_height: 1.0,
        max_tiles: 8,
        max_polys: 8,
    };
    let Ok(mut nav) = NavMesh::new(&params) else {
        return;
    };
    nav.set_global_off_mesh_links(flags & 1 != 0);
    let mut refs: [Option<navmesh_rs::detour::TileRef>; TILES as usize] = [None; TILES as usize];
    for op in ops.as_chunks::<4>().0.iter().take(48) {
        let x = i32::from(op[1] % TILES as u8);
        let y = i32::from(op[2] % TILES as u8);
        match op[0] % 5 {
            0 => {
                let link = (op[3] & 1 != 0).then_some((y, op[3] & 2 != 0));
                if refs[x as usize].is_none()
                    && let Some(data) = tile(x, link)
                    && let Ok(r) = nav.add_tile(data)
                {
                    refs[x as usize] = Some(r);
                }
            }
            1 => {
                if let Some(r) = refs[x as usize].take() {
                    assert!(
                        nav.remove_tile(r).is_ok(),
                        "remove of a resident tile failed"
                    );
                }
            }
            2 => nav.set_global_off_mesh_links(op[1] & 1 != 0),
            3 => single_shot(&nav, x, y, op[3]),
            _ => resumable(&nav, x, y, op[3]),
        }
    }
});
