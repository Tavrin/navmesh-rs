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
    path: Vec<PolyRef>,
    initial: Vec<PolyRef>,
    complete: bool,
    disconnected: bool,
    failure: Option<String>,
    first_units: Option<u64>,
    max_ms: f64,
    max_frame_ms: f64,
}
fn run(atlas: &RouteAtlas<'_>, nav: &NavMesh, p: &Pair, timed: bool) -> Run {
    let mut s = Navigator::new(atlas, POOL).unwrap();
    let start = timed.then(ThreadTime::now);
    let id = submit(&mut s, nav, p);
    let mut max_ms = start.map_or(0., |t| t.elapsed().as_secs_f64() * 1000.);
    let mut max_frame_ms: f64 = 0.;
    let mut path = Vec::new();
    let mut first = None;
    let mut first_units = None;
    let mut total_units = 0;
    let mut initial = Vec::new();
    for frame in 1..100_000 {
        let tick = timed.then(ThreadTime::now);
        let mut budget = Budget::new(2720);
        while budget.remaining() > 0 {
            let unit = timed.then(ThreadTime::now);
            let before = budget.used();
            update(&mut s, nav, &mut budget);
            total_units += budget.used() - before;
            if let Some(t) = unit {
                max_ms = max_ms.max(t.elapsed().as_secs_f64() * 1000.);
            }
            match s.state(id).unwrap() {
                RouteState::Planning => {}
                RouteState::Segment => {
                    let seg = s.segment(id).unwrap();
                    if path.is_empty() {
                        path.extend_from_slice(seg);
                    } else {
                        assert_eq!(path.last(), seg.first());
                        path.extend_from_slice(&seg[1..]);
                    }
                    if first.is_none() && delivered(&path, s.toward(id).unwrap().position, p) {
                        first = Some(frame);
                        first_units = Some(total_units);
                        initial.clone_from(&path);
                    }
                    s.advance(id).unwrap();
                }
                state => {
                    if let Some(t) = tick {
                        max_frame_ms = max_frame_ms.max(t.elapsed().as_secs_f64() * 1000.);
                    }
                    return Run {
                        first,
                        path,
                        initial,
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
                    };
                }
            }
        }
        if let Some(t) = tick {
            max_frame_ms = max_frame_ms.max(t.elapsed().as_secs_f64() * 1000.);
        }
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
    let references = env("NAV_ROUTE_REFERENCE_PAIRS", count);
    let one = |(i, p): (usize, &Pair)| {
        let reference =
            || (i < references).then(|| route::exact(nav, p.start, p.goal, p.a, p.b, &filter).0);
        let (r, exact) = if i.is_multiple_of(2) {
            (run(&atlas, nav, p, timing), reference())
        } else {
            let exact = reference();
            (run(&atlas, nav, p, timing), exact)
        };
        let reached = r.complete && r.path.last() == Some(&p.goal);
        let mut coarse = atlas.coarse_request(nav, p.start, p.goal).unwrap();
        assert!(
            coarse
                .resume(&graph, &mut Budget::unlimited())
                .unwrap()
                .is_done()
        );
        let coarse_reached = !coarse.corridor_reversed().unwrap().is_empty();
        let q = NavMeshQuery::new(nav, 8).unwrap();
        let ratio = exact
            .filter(|v| v.is_finite() && reached)
            .map(|v| path_cost(nav, &q, &r.path, p.a, p.b, &filter) / v);
        let first = exact.and_then(|v| first_segment_ratio(nav, &q, &r.initial, p, &filter, v));
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
        )
    };
    let rows: Vec<_> = if timing {
        subset.iter().enumerate().map(one).collect()
    } else {
        subset.par_iter().enumerate().map(one).collect()
    };
    let mut concurrent_unit = 0.0f64;
    let mut concurrent_frame = 0.0f64;
    let concurrent = if timing {
        let mut s = Navigator::new(&atlas, POOL).unwrap();
        let pending: Vec<_> = pairs
            .iter()
            .take(64)
            .map(|p| {
                let t = ThreadTime::now();
                let id = submit(&mut s, nav, p);
                concurrent_unit = concurrent_unit.max(t.elapsed().as_secs_f64() * 1000.);
                id
            })
            .collect();
        let mut answered = vec![None; pending.len()];
        for frame in 1..100_000 {
            let mut b = Budget::new(2720);
            let mut frame_ms = 0.;
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
                            s.advance(id).unwrap()
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
        "concurrent":concurrent,"rows":rows});
    if let Some(file) = std::env::var_os("NAV_ROUTE_REPORT") {
        std::fs::write(file, serde_json::to_vec(&report).unwrap()).unwrap();
    }
    println!(
        "candidate {} completed, {} disconnected, first {:?}, cost {:?}",
        report["completed"], report["disconnected"], report["first_frames"], report["ratio"]
    );
}
