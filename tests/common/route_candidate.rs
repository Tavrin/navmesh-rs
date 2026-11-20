//! Candidate measurements using the same fixture, seeded pairs and exact cost as the flat run.
use super::*;
use navmesh_rs::{
    Budget,
    route::{Navigator, RequestId, RouteAtlas, RouteState, SectorGraph, SectorParams, WindowPoint},
};
use rayon::prelude::*;
use serde_json::json;

fn submit(s: &mut Navigator<'_>, nav: &NavMesh, p: &Pair) -> RequestId {
    s.request(
        nav,
        WindowPoint::new(p.start, p.a),
        WindowPoint::new(p.goal, p.b),
    )
    .unwrap()
}
fn update(s: &mut Navigator<'_>, nav: &NavMesh, b: &mut Budget) {
    s.update(nav, b, 64).unwrap();
}
fn delivered(path: &[PolyRef], target: [f32; 3], p: &Pair) -> bool {
    path.len() > 1 || distance(target, p.a) > 0.001
}
struct Run {
    first: Option<usize>,
    cost: f32,
    horizon: Option<(PolyRef, [f32; 3], f32)>,
    path: Vec<PolyRef>,
    complete: bool,
    disconnected: bool,
    failure: Option<String>,
    first_units: Option<u64>,
    max_ms: f64,
    max_frame_ms: f64,
    cpu_ms: f64,
    over_frames: usize,
}
fn run(atlas: &RouteAtlas<'_>, nav: &NavMesh, p: &Pair, timed: bool) -> Run {
    let mut s = Navigator::new(atlas, POOL).unwrap();
    let start = timed.then(ThreadTime::now);
    let id = submit(&mut s, nav, p);
    let mut max_ms = start.map_or(0., |t| t.elapsed().as_secs_f64() * 1000.);
    let mut max_frame_ms: f64 = 0.;
    let mut cpu_ms = max_ms;
    let mut over_frames = 0;
    let mut path = Vec::new();
    let query = NavMeshQuery::new(nav, 1).unwrap();
    let filter = DefaultFilter::default();
    let mut position = p.a;
    let mut cost = 0.;
    let mut walked = 0.;
    let mut horizon = None;
    let mut first = None;
    let mut first_units = None;
    let mut total_units = 0;
    for frame in 1..100_000 {
        let mut frame_ms = if frame == 1 { max_ms } else { 0. };
        let mut budget = Budget::new(2720);
        while budget.remaining() > 0 {
            let unit = timed.then(ThreadTime::now);
            let before = budget.used();
            update(&mut s, nav, &mut budget);
            total_units += budget.used() - before;
            if let Some(t) = unit {
                let elapsed = t.elapsed().as_secs_f64() * 1000.;
                max_ms = max_ms.max(elapsed);
                frame_ms += elapsed;
                cpu_ms += elapsed;
            }
            match s.state(id).unwrap() {
                RouteState::Planning => {}
                RouteState::Segment => {
                    let seg = s.segment(id).unwrap();
                    assert!(!s.corners(id).unwrap().is_empty());
                    // Measure every committed endpoint, including movement within
                    // a shared polygon before the continuation is available.
                    for (i, &poly) in seg.iter().enumerate() {
                        let end = if let Some(&next) = seg.get(i + 1) {
                            let (l, r) = query.portal_points(poly, next).unwrap();
                            midpoint(l, r)
                        } else {
                            s.toward(id).unwrap().position
                        };
                        let len = distance(position, end);
                        let weight = area_cost(nav, poly, &filter);
                        if horizon.is_none() && walked + len >= HORIZON && len > 0. {
                            let remaining = HORIZON - walked;
                            let end = core::array::from_fn(|i| {
                                position[i] + (end[i] - position[i]) * remaining / len
                            });
                            horizon = Some((poly, end, cost + remaining * weight));
                        }
                        cost += len * weight;
                        walked += len;
                        position = end;
                    }
                    if path.is_empty() {
                        path.extend_from_slice(seg);
                    } else {
                        assert_eq!(path.last(), seg.first());
                        path.extend_from_slice(&seg[1..]);
                    }
                    if first.is_none() && delivered(&path, s.toward(id).unwrap().position, p) {
                        first = Some(frame);
                        first_units = Some(total_units);
                    }
                    let ack = timed.then(ThreadTime::now);
                    s.advance(id).unwrap();
                    if let Some(t) = ack {
                        let elapsed = t.elapsed().as_secs_f64() * 1000.;
                        max_ms = max_ms.max(elapsed);
                        frame_ms += elapsed;
                        cpu_ms += elapsed;
                    }
                }
                state => {
                    max_frame_ms = max_frame_ms.max(frame_ms);
                    over_frames += usize::from(frame_ms > 1.);
                    return Run {
                        first,
                        cost,
                        horizon,
                        path,
                        complete: state == RouteState::Complete,
                        disconnected: state == RouteState::Disconnected,
                        failure: if let RouteState::Failed(e) = state {
                            Some(e.to_string())
                        } else {
                            None
                        },
                        first_units,
                        max_ms,
                        max_frame_ms,
                        cpu_ms,
                        over_frames,
                    };
                }
            }
        }
        max_frame_ms = max_frame_ms.max(frame_ms);
        over_frames += usize::from(frame_ms > 1.);
    }
    panic!("candidate did not terminate")
}
pub(super) fn measure(nav: &NavMesh, pairs: &[Pair], world: serde_json::Value) {
    let prep = Instant::now();
    let filter = DefaultFilter::default();
    let params = SectorParams::new(env("NAV_ROUTE_SECTOR_TILES", 4), 8);
    let graph = SectorGraph::from_mesh(nav, &params, &filter, Limits::UNLIMITED).unwrap();
    let atlas = RouteAtlas::new(&graph, nav, filter.clone()).unwrap();
    println!("candidate prepared in {:.3}s", prep.elapsed().as_secs_f64());
    let timing = env("NAV_ROUTE_TIMING", 0) == 1;
    let offset = env("NAV_ROUTE_OFFSET", 0);
    let count = env("NAV_ROUTE_COUNT", pairs.len() - offset);
    let subset = &pairs[offset..offset + count];
    let references = env("NAV_ROUTE_REFERENCE_PAIRS", 65);
    let one = |(i, p): (usize, &Pair)| {
        let reference =
            || (i < references).then(|| route::exact(nav, p.start, p.goal, p.a, p.b, &filter).0);
        let (r, mut exact) = if i.is_multiple_of(2) {
            (run(&atlas, nav, p, timing), reference())
        } else {
            let exact = reference();
            (run(&atlas, nav, p, timing), exact)
        };
        if r.disconnected && exact.is_none() {
            exact = Some(route::exact(nav, p.start, p.goal, p.a, p.b, &filter).0);
        }
        let reached = r.complete && r.path.last() == Some(&p.goal);
        let mut coarse = atlas.coarse_request(nav, p.start, p.goal).unwrap();
        assert!(
            coarse
                .resume(&graph, &mut Budget::unlimited())
                .unwrap()
                .is_done()
        );
        let coarse_reached = !coarse.corridor_reversed().unwrap().is_empty();
        let ratio = exact
            .filter(|v| v.is_finite() && reached)
            .map(|v| r.cost / v);
        // The caller acknowledges each published segment. Score the first 50 m
        // of that committed walk even if it spans several published segments.
        let first = exact.and_then(|v| {
            if v.is_finite() && reached {
                Some(
                    r.horizon.map_or(r.cost, |(poly, position, prefix)| {
                        prefix + route::exact(nav, poly, p.goal, position, p.b, &filter).0
                    }) / v,
                )
            } else {
                None
            }
        });
        (
            r.first,
            reached,
            r.disconnected,
            exact.map(f32::is_finite),
            ratio,
            first,
            r.max_ms,
            r.max_frame_ms,
            coarse_reached,
            r.first_units,
            r.failure,
            r.cpu_ms,
            r.over_frames,
        )
    };
    profile_event(b"enable\n");
    let rows: Vec<_> = if timing {
        subset.iter().enumerate().map(one).collect()
    } else {
        subset.par_iter().enumerate().map(one).collect()
    };
    profile_event(b"disable\n");
    let mut concurrent_unit = 0.0f64;
    let mut concurrent_frame = 0.0f64;
    let mut concurrent_over = 0usize;
    let concurrent = if timing {
        let mut s = Navigator::new(&atlas, POOL).unwrap();
        let mut admission_ms = 0.;
        let pending: Vec<_> = pairs
            .iter()
            .take(64)
            .map(|p| {
                let t = ThreadTime::now();
                let id = submit(&mut s, nav, p);
                let ms = t.elapsed().as_secs_f64() * 1000.;
                concurrent_unit = concurrent_unit.max(ms);
                admission_ms += ms;
                id
            })
            .collect();
        let mut answered = vec![None; pending.len()];
        for frame in 1..100_000 {
            let mut b = Budget::new(2720);
            let mut frame_ms = if frame == 1 { admission_ms } else { 0. };
            while b.remaining() > 0 {
                let before = b.used();
                let t = ThreadTime::now();
                update(&mut s, nav, &mut b);
                let ms = t.elapsed().as_secs_f64() * 1000.;
                concurrent_unit = concurrent_unit.max(ms);
                frame_ms += ms;
                for (i, &id) in pending.iter().enumerate() {
                    if answered[i].is_some() {
                        continue;
                    }
                    match s.state(id).unwrap() {
                        RouteState::Planning => {}
                        RouteState::Segment
                            if !delivered(
                                s.segment(id).unwrap(),
                                s.toward(id).unwrap().position,
                                &pairs[i],
                            ) =>
                        {
                            let t = ThreadTime::now();
                            s.advance(id).unwrap();
                            let ms = t.elapsed().as_secs_f64() * 1000.;
                            concurrent_unit = concurrent_unit.max(ms);
                            frame_ms += ms;
                        }
                        RouteState::Segment | RouteState::Disconnected => answered[i] = Some(frame),
                        other => panic!("concurrent request failed: {other:?}"),
                    }
                }
                if before == b.used() {
                    break;
                }
            }
            concurrent_frame = concurrent_frame.max(frame_ms);
            concurrent_over += usize::from(frame_ms > 1.);
            if answered.iter().all(Option::is_some) {
                break;
            }
        }
        Some(answered)
    } else {
        None
    };
    let report = json!({"world":world,"offset":offset,"count":count,"frame_units":2720,"timed":timing,"level_bytes":graph.retained_bytes_by_level(),
        "completed":rows.iter().filter(|r|r.1).count(),"disconnected":rows.iter().filter(|r|r.2).count(),
        "reference_reachable":rows.iter().filter(|r|r.3==Some(true)).count(),
        "coarse_agreement":rows.iter().filter(|r|r.3==Some(r.8)).count(),
        "agreement":rows.iter().filter(|r|r.3.is_some_and(|reachable|if reachable{r.1}else{r.2})).count(),
        "first_frames":spread(&rows.iter().filter_map(|r|r.0.map(|v|v as f64)).collect::<Vec<_>>()),
        "ratio":spread(&rows.iter().filter_map(|r|r.4.map(f64::from)).collect::<Vec<_>>()),
        "first50_agree":rows.iter().filter(|r|r.5.is_some_and(|v|v<=AGREEMENT)).count(),
        "max_unit_ms":rows.iter().map(|r|r.6).fold(concurrent_unit,f64::max),"max_frame_ms":rows.iter().map(|r|r.7).fold(concurrent_frame,f64::max),
        "total_walk_cpu_ms":spread(&rows.iter().map(|r|r.11).collect::<Vec<_>>()),
        "frames_over_budget":concurrent_over+rows.iter().map(|r|r.12).sum::<usize>(),
        "concurrent":concurrent,"rows":rows});
    if let Some(file) = std::env::var_os("NAV_ROUTE_REPORT") {
        std::fs::write(file, serde_json::to_vec(&report).unwrap()).unwrap();
    }
    println!(
        "candidate {} completed, {} disconnected, first {:?}, cost {:?}",
        report["completed"], report["disconnected"], report["first_frames"], report["ratio"]
    );
}
