//! Work budgets: partial results when a budget runs out, equality with the plain
//! call when it suffices, resumable searches stepped in small pieces, and the
//! budgeted crowd tick. The same equality and bound checks run over the reference
//! corpus in `nav-port budget` (see `docs/VERIFICATION.md`).

#![cfg(all(feature = "recast", feature = "detour", feature = "crowd"))]

use navmesh_rs::crowd::{AgentParams, Crowd};
use navmesh_rs::detour::{
    self, DefaultFilter, NavMesh, NavMeshCreateParams, NavMeshQuery, PolyRef, Status,
};
use navmesh_rs::{Budget, Budgeted, Progress, Workspace};

const N: u16 = 6;

/// An `N` x `N` grid of unit quads in one tile.
fn grid() -> NavMesh {
    let w = usize::from(N) + 1;
    let mut verts = Vec::new();
    for z in 0..w {
        for x in 0..w {
            verts.extend([x as u16, 0, z as u16]);
        }
    }
    let nei = |x: i32, z: i32| -> u16 {
        if (0..i32::from(N)).contains(&x) && (0..i32::from(N)).contains(&z) {
            (z * i32::from(N) + x) as u16
        } else {
            0xffff
        }
    };
    let mut polys = Vec::new();
    for z in 0..i32::from(N) {
        for x in 0..i32::from(N) {
            let v = |dx: i32, dz: i32| ((z + dz) * w as i32 + x + dx) as u16;
            polys.extend([v(0, 0), v(0, 1), v(1, 1), v(1, 0)]);
            polys.extend([nei(x - 1, z), nei(x, z + 1), nei(x + 1, z), nei(x, z - 1)]);
        }
    }
    let count = i32::from(N) * i32::from(N);
    let data = detour::create_nav_mesh_data(&NavMeshCreateParams {
        verts: &verts,
        vert_count: (w * w) as i32,
        polys: &polys,
        poly_count: count,
        nvp: 4,
        poly_flags: &vec![1; count as usize],
        poly_areas: &vec![0; count as usize],
        bmin: [0.; 3],
        bmax: [f32::from(N), 1., f32::from(N)],
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

fn poly_at(q: &NavMeshQuery, p: [f32; 3]) -> PolyRef {
    q.find_nearest_poly(&p, &[0.4, 2., 0.4], &DefaultFilter::default())
        .unwrap()
        .0
}

#[test]
fn find_path_returns_the_closest_partial_path_when_the_budget_runs_out() {
    let nav = grid();
    let mut q = NavMeshQuery::new(&nav, 64).unwrap();
    let f = DefaultFilter::default();
    let (a, b) = ([0.5, 0., 0.5], [5.5, 0., 5.5]);
    let (ra, rb) = (poly_at(&q, a), poly_at(&q, b));
    let mut plain = Vec::new();
    let st = q.find_path(ra, rb, &a, &b, &f, &mut plain, 64).unwrap();

    let mut path = Vec::new();
    let mut ample = Budget::new(1_000);
    let done = q
        .find_path_budgeted(ra, rb, &a, &b, &f, &mut path, 64, &mut ample)
        .unwrap();
    assert_eq!((done, &path), (Budgeted::Complete(st), &plain));

    let mut tight = Budget::new(3);
    let cut = q
        .find_path_budgeted(ra, rb, &a, &b, &f, &mut path, 64, &mut tight)
        .unwrap();
    assert!(matches!(cut, Budgeted::Exhausted(s) if s.0 & Status::PARTIAL_RESULT != 0));
    assert!(tight.used() <= 3 && tight.is_exhausted());
    assert_eq!(path.first(), Some(&ra));
    assert!(path.len() < plain.len() && path.len() > 1);
}

#[test]
fn polys_around_resumes_in_small_steps_and_equals_the_plain_search() {
    let nav = grid();
    let mut q = NavMeshQuery::new(&nav, 64).unwrap();
    let f = DefaultFilter::default();
    let c = [3., 0., 3.];
    let start = poly_at(&q, c);
    let mut plain = Vec::new();
    let st = q
        .find_polys_around_circle(start, &c, 2.5, &f, &mut plain, 64)
        .unwrap();

    let mut search = q.begin_polys_around_circle(start, &c, 2.5, 64).unwrap();
    let mut steps = 0;
    loop {
        // Other queries between steps do not disturb the search.
        let _ = q.find_polys_around_circle(start, &c, 0.5, &f, &mut Vec::new(), 4);
        let mut step = Budget::new(3);
        let p = search.resume(&mut q, &f, &mut step).unwrap();
        assert!(step.used() <= 3);
        steps += 1;
        if p == Progress::Done {
            break;
        }
    }
    assert!(steps > 2, "needed several steps, got {steps}");
    assert_eq!((search.results(), search.status()), (&plain[..], st));
}

#[test]
fn raycast_resumes_and_stops_charging_at_the_budget() {
    let nav = grid();
    let q = NavMeshQuery::new(&nav, 64).unwrap();
    let f = DefaultFilter::default();
    let (a, b) = ([0.5, 0., 0.5], [5.5, 0., 0.7]);
    let ra = poly_at(&q, a);
    let (plain, st) = q.raycast_with(ra, &a, &b, &f, 0, 64, 0).unwrap();
    assert!(plain.path.len() >= 6);

    let mut search = q.begin_raycast(ra, &a, &b, 0, 64, 0).unwrap();
    let mut first = Budget::new(2);
    assert_eq!(
        search.resume(&q, &f, &mut first).unwrap(),
        Progress::Exhausted
    );
    assert_eq!(first.used(), 2);
    assert_eq!(search.result().0.path.len(), 2);
    let mut rest = Budget::new(100);
    assert_eq!(search.resume(&q, &f, &mut rest).unwrap(), Progress::Done);
    assert_eq!((search.result().0, search.result().1), (&plain, st));
}

#[test]
fn crowd_tick_under_a_budget_equals_the_plain_tick_or_leaves_the_crowd_alone() {
    let nav = grid();
    let build = |ws: &mut Workspace| {
        let mut c = Crowd::new(ws, &nav, 4, 0.6).unwrap();
        let id = c
            .add_agent(ws, &nav, [0.5, 0., 0.5], AgentParams::default())
            .unwrap();
        let q = NavMeshQuery::new(&nav, 32).unwrap();
        let t = [5.5, 0., 5.5];
        c.request_move_target(&nav, id, poly_at(&q, t), t).unwrap();
        (c, id)
    };
    let (mut ws_a, mut ws_b) = (Workspace::new(), Workspace::new());
    let (mut plain, id) = build(&mut ws_a);
    let (mut budgeted, _) = build(&mut ws_b);
    let mut budget = Budget::new(10_000_000);
    for _ in 0..20 {
        plain.update(&mut ws_a, &nav, 1. / 30.).unwrap();
        let out = budgeted
            .update_budgeted(&mut ws_b, &nav, 1. / 30., &mut budget)
            .unwrap();
        assert!(out.is_complete());
    }
    assert_eq!(
        plain.agent(id).unwrap().npos,
        budgeted.agent(id).unwrap().npos
    );
    assert!(budget.used() > 0 && budget.used() <= budget.limit());

    // Not even the up-front charge fits: nothing is touched.
    let before = budgeted.agent(id).unwrap().npos;
    let mut tiny = Budget::new(100);
    let out = budgeted
        .update_budgeted(&mut ws_b, &nav, 1. / 30., &mut tiny)
        .unwrap();
    assert!(!out.is_complete() && tiny.used() <= 100 && tiny.is_exhausted());
    assert_eq!(budgeted.agent(id).unwrap().npos, before);
}
