//! Corpus qualification and interleaved costs for the additive interval query.
use super::*;
use navmesh_rs::detour::{AnyAnglePath, AnyAnglePoint};
use navmesh_rs::{Budget, Budgeted};

fn baseline(
    q: &mut NavMeshQuery<'_>,
    a: AnyAnglePoint,
    b: AnyAnglePoint,
) -> Result<Option<f32>, Error> {
    let mut corridor = Vec::new();
    let mut points = Vec::new();
    let status = q.find_path(
        a.polygon,
        b.polygon,
        &a.position,
        &b.position,
        &DefaultFilter::default(),
        &mut corridor,
        65535,
    )?;
    if status.has_detail(Status::PARTIAL_RESULT | Status::OUT_OF_NODES) {
        return Ok(None);
    }
    q.find_straight_path(&a.position, &b.position, &corridor, &mut points, 65535, 0)?;
    Ok(Some(
        points
            .windows(2)
            .map(|p| {
                let dx = p[1].pos[0] - p[0].pos[0];
                let dz = p[1].pos[2] - p[0].pos[2];
                (dx * dx + dz * dz).sqrt()
            })
            .sum(),
    ))
}
// Qualification tolerance applies only at the segment endpoint.
const CONTAINED_T: f32 = 1.0 - 1e-6;
// Spatial slack of the closed-coverage check: eight f32 ulps of the
// coordinate magnitude, the rounding of a border vertex repeated by
// neighbouring tiles. Polygons are widened by it and smaller gaps pass.
fn rounding(a: &[f32; 3], b: &[f32; 3]) -> f64 {
    let magnitude = [a[0], a[2], b[0], b[2]]
        .iter()
        .fold(1.0f32, |m, v| m.max(v.abs()));
    f64::from(magnitude) * f64::from(f32::EPSILON) * 8.0
}
// Radius of the replacement end for pairs too far apart for the node pool.
const LOCAL_RADIUS: f32 = 48.0;

fn cross2(a: [f64; 2], b: [f64; 2], c: [f64; 2]) -> f64 {
    (b[0] - a[0]) * (c[1] - a[1]) - (b[1] - a[1]) * (c[0] - a[0])
}
fn xz(p: &[f32]) -> [f64; 2] {
    [p[0] as f64, p[2] as f64]
}
// Parameter range of segment `a..b` inside a closed convex xz polygon. A
// polygon with zero xz area (a riser seen from above) is the union of its edges.
fn clip(a: [f64; 2], b: [f64; 2], ring: &[[f64; 2]], eps: f64) -> Option<[f64; 2]> {
    let n = ring.len();
    let area: f64 = (0..n)
        .map(|i| cross2([0.0; 2], ring[i], ring[(i + 1) % n]))
        .sum();
    if area == 0.0 {
        return (0..n)
            .filter_map(|i| on_portal(a, b, ring[i], ring[(i + 1) % n], eps))
            .reduce(|x, y| [x[0].min(y[0]), x[1].max(y[1])]);
    }
    let sign = area.signum();
    let (mut lo, mut hi) = (0.0f64, 1.0f64);
    for i in 0..n {
        let (p, q) = (ring[i], ring[(i + 1) % n]);
        // Widen the polygon by `eps` along the edge normal.
        let slack = eps * ((q[0] - p[0]).powi(2) + (q[1] - p[1]).powi(2)).sqrt();
        let u = cross2(p, q, a) * sign + slack;
        let v = cross2(p, q, b) * sign + slack;
        if u < 0.0 && v < 0.0 {
            return None;
        }
        if u < 0.0 {
            lo = lo.max(u / (u - v));
        } else if v < 0.0 {
            hi = hi.min(u / (u - v));
        }
    }
    (lo <= hi).then_some([lo, hi])
}
// Parameter range of segment `a..b` lying on the closed portal `p..q`.
fn on_portal(a: [f64; 2], b: [f64; 2], p: [f64; 2], q: [f64; 2], eps: f64) -> Option<[f64; 2]> {
    let d = [b[0] - a[0], b[1] - a[1]];
    let e = [q[0] - p[0], q[1] - p[1]];
    let denom = d[0] * e[1] - d[1] * e[0];
    let w = [p[0] - a[0], p[1] - a[1]];
    if denom != 0.0 {
        let t = (w[0] * e[1] - w[1] * e[0]) / denom;
        let s = (w[0] * d[1] - w[1] * d[0]) / denom;
        let span = (d[0] * d[0] + d[1] * d[1]).sqrt();
        let portal = (e[0] * e[0] + e[1] * e[1]).sqrt();
        let (t_tol, s_tol) = (eps / span, if portal > 0.0 { eps / portal } else { 0.0 });
        return ((-t_tol..=1.0 + t_tol).contains(&t) && (-s_tol..=1.0 + s_tol).contains(&s))
            .then_some([t, t]);
    }
    if cross2(a, b, p) != 0.0 {
        return None;
    }
    let len = d[0] * d[0] + d[1] * d[1];
    if len == 0.0 {
        return None;
    }
    let tp = (w[0] * d[0] + w[1] * d[1]) / len;
    let tq = ((q[0] - a[0]) * d[0] + (q[1] - a[1]) * d[1]) / len;
    let (lo, hi) = (tp.min(tq).max(0.0), tp.max(tq).min(1.0));
    (lo <= hi).then_some([lo, hi])
}
fn ring(tile: &detour::MeshTile, poly: &detour::Poly) -> Vec<[f64; 2]> {
    poly.verts[..poly.vert_count as usize]
        .iter()
        .map(|&v| xz(&tile.verts[v as usize * 3..]))
        .collect()
}
/// Independent closed containment: the segment from `a` (in `start`) to `b`
/// must be covered, without gaps, by ground polygons reached from `start`
/// through portals that the segment touches. Unlike a raycast, a segment
/// running along a shared edge or through a shared vertex is not forced into
/// one side's walls. Returns the largest uncovered length in world units.
fn coverage(
    nav: &NavMesh,
    start: PolyRef,
    a: &[f32; 3],
    b: &[f32; 3],
    debug: bool,
) -> Result<f64, String> {
    coverage_with_slack(nav, start, a, b, debug, rounding(a, b))
}
fn coverage_with_slack(
    nav: &NavMesh,
    start: PolyRef,
    a: &[f32; 3],
    b: &[f32; 3],
    debug: bool,
    eps: f64,
) -> Result<f64, String> {
    let (pa, pb) = (xz(a), xz(b));
    let len = ((pb[0] - pa[0]).powi(2) + (pb[1] - pa[1]).powi(2)).sqrt();
    if len == 0.0 {
        return Ok(0.0);
    }
    let tol = eps / len;
    let (tile, poly) = nav.tile_and_poly_by_ref(start).map_err(|e| e.to_string())?;
    let Some(first) = clip(pa, pb, &ring(tile, poly), eps) else {
        return Ok(len);
    };
    if first[0] > tol {
        return Ok(first[0] * len);
    }
    let mut seen = vec![start];
    let mut queue = vec![(start, first)];
    let mut spans = vec![first];
    while let Some((r, span)) = queue.pop() {
        let (tile, poly) = nav.tile_and_poly_by_ref(r).map_err(|e| e.to_string())?;
        let verts = ring(tile, poly);
        let n = verts.len();
        let mut link = poly.first_link;
        while link != detour::NULL_LINK {
            let l = tile.links[link as usize];
            link = l.next;
            if l.r#ref == 0 || seen.contains(&l.r#ref) || l.edge as usize >= n {
                continue;
            }
            let (nt, np) = nav
                .tile_and_poly_by_ref(l.r#ref)
                .map_err(|e| e.to_string())?;
            if np.poly_type() != detour::POLYTYPE_GROUND {
                continue;
            }
            let (mut p, mut q) = (verts[l.edge as usize], verts[(l.edge as usize + 1) % n]);
            if l.side != 0xff {
                let s = 1.0f32 / 255.0;
                let lerp = |x: f64, y: f64, k: u8| {
                    (x as f32 + (y as f32 - x as f32) * (k as f32 * s)) as f64
                };
                let (p0, q0) = (p, q);
                p = [lerp(p0[0], q0[0], l.bmin), lerp(p0[1], q0[1], l.bmin)];
                q = [lerp(p0[0], q0[0], l.bmax), lerp(p0[1], q0[1], l.bmax)];
            }
            let gate = on_portal(pa, pb, p, q, eps);
            let next = clip(pa, pb, &ring(nt, np), eps);
            if debug && (gate.is_some() || next.is_some()) {
                eprintln!(
                    "coverage {r}->{} side={} b={}..{} portal={p:?}..{q:?} span={span:?} gate={gate:?} next={next:?} ring={:?}",
                    l.r#ref,
                    l.side,
                    l.bmin,
                    l.bmax,
                    ring(nt, np)
                );
            }
            let (Some(gate), Some(next)) = (gate, next) else {
                continue;
            };
            let lo = gate[0].max(span[0]).max(next[0]);
            let hi = gate[1].min(span[1]).min(next[1]);
            if lo > hi + tol {
                continue;
            }
            seen.push(l.r#ref);
            spans.push(next);
            queue.push((l.r#ref, next));
        }
    }
    spans.sort_by(|x, y| x[0].total_cmp(&y[0]));
    if debug {
        eprintln!("coverage spans {spans:?} visited {seen:?}");
    }
    let mut reach = 0.0f64;
    let mut gap = 0.0f64;
    for s in spans {
        if s[0] > reach {
            gap = gap.max(s[0] - reach);
        }
        reach = reach.max(s[1]);
    }
    if reach < 1.0 {
        gap = gap.max(1.0 - reach);
    }
    Ok(if gap <= tol { 0.0 } else { gap * len })
}
// An off-mesh step must be exactly one linked connection, in a direction
// the connection allows.
fn off_mesh_step(nav: &NavMesh, r: PolyRef, a: &[f32; 3], b: &[f32; 3]) -> Option<bool> {
    let (tile, poly) = nav.tile_and_poly_by_ref(r).ok()?;
    if poly.poly_type() != detour::POLYTYPE_OFFMESH_CONNECTION {
        return None;
    }
    let v = |i: usize| xz(&tile.verts[poly.verts[i] as usize * 3..]);
    let bidir = nav
        .off_mesh_connection_by_ref(r)
        .is_some_and(|c| c.flags & detour::OFFMESH_CON_BIDIR != 0);
    let (a, b) = (xz(a), xz(b));
    Some((a == v(0) && b == v(1)) || (bidir && a == v(1) && b == v(0)))
}
fn baseline_containment(
    nav: &NavMesh,
    q: &mut NavMeshQuery<'_>,
    a: AnyAnglePoint,
    b: AnyAnglePoint,
) -> Result<(bool, usize), Error> {
    let filter = DefaultFilter::default();
    let mut corridor = Vec::new();
    let mut points = Vec::new();
    q.find_path(
        a.polygon,
        b.polygon,
        &a.position,
        &b.position,
        &filter,
        &mut corridor,
        65535,
    )?;
    q.find_straight_path(&a.position, &b.position, &corridor, &mut points, 65535, 0)?;
    let mut valid = true;
    let mut tolerance_segments = 0;
    for seg in points.windows(2) {
        if let Some(step) = off_mesh_step(nav, seg[0].r#ref, &seg[0].pos, &seg[1].pos) {
            valid &= step;
            continue;
        }
        // Containment assumes the start belongs to its ref; a quantized funnel
        // corner may violate that precondition.
        let (closest, _) = q.closest_point_on_poly(seg[0].r#ref, &seg[0].pos)?;
        valid &= closest[0] == seg[0].pos[0] && closest[2] == seg[0].pos[2];
        let (hit, _) = q.raycast(seg[0].r#ref, &seg[0].pos, &seg[1].pos, &filter, 65535)?;
        let gap = coverage(nav, seg[0].r#ref, &seg[0].pos, &seg[1].pos, false)
            .map_err(|_| Error::InvalidRef)?;
        valid &= gap == 0.0;
        tolerance_segments += usize::from(hit.t < 1.0 && gap == 0.0);
    }
    Ok((valid, tolerance_segments))
}
fn candidate(
    q: &NavMeshQuery<'_>,
    a: AnyAnglePoint,
    b: AnyAnglePoint,
) -> Result<AnyAnglePath, Error> {
    match q.find_any_angle_path(a, b, &DefaultFilter::default(), &mut Budget::new(65535))? {
        Budgeted::Complete(path) => Ok(path),
        Budgeted::Exhausted(_) => Err(Error::LimitExceeded),
        _ => Err(Error::Unsupported),
    }
}
pub(super) fn verify(c: &Case, g: &Geometry) -> Result<(), String> {
    let baked = bake(c, g, &mut Sink { writer: None }).map_err(|e| e.to_string())?;
    if baked.tiles.is_empty() {
        return Err("no ground mesh".into());
    }
    let nav = assemble(&baked).map_err(|e| e.to_string())?;
    let mut q = NavMeshQuery::new(&nav, 65535).map_err(|e| e.to_string())?;
    let rng = std::cell::RefCell::new(Rng(c.seed.max(1)));
    let mut endpoints = Vec::new();
    for _ in 0..c.queries.clamp(2, 32) + 1 {
        let (polygon, position) = q
            .find_random_point(&DefaultFilter::default(), &mut || rng.borrow_mut().frand())
            .map_err(|e| e.to_string())?;
        endpoints.push(AnyAnglePoint { polygon, position });
    }
    let dump = std::env::var_os("NAV_ANY_ANGLE_DUMP").map(std::path::PathBuf::from);
    if let Some(dir) = &dump {
        export(&nav, dir).map_err(|e| e.to_string())?;
    }
    let repeats = match std::env::var("NAV_ANY_ANGLE_REPEATS") {
        Ok(v) => v.parse::<usize>().map_err(|e| e.to_string())?,
        Err(_) => 4,
    };
    if repeats == 0 || repeats > 100_000 {
        return Err("repeat count must be in 1..=100000".into());
    }
    let mut dump_queries = String::new();
    let mut digest = navmesh_parity::encode::Payload::new();
    let mut failures = 0;
    let mut skipped = 0;
    let mut invalid_baselines = 0;
    let mut tolerance_queries = 0;
    let mut tolerance_segments = 0;
    let mut baseline_tolerance_segments = 0;
    let mut passed = 0;
    let mut off_mesh_segments = 0;
    let mut local_pairs = 0;
    let mut raycast_short = 0;
    let mut max_gap = 0.0f64;
    let mut max_exact_gap = 0.0f64;
    let mut rounding_segments = 0;
    let mut timed_queries = 0;
    let mut primary_queries = 0;
    let mut primary_baseline_ns = 0;
    let mut primary_interval_ns = 0;
    let mut primary_expanded = 0;
    let mut base_ns = 0;
    let mut query_ns = 0;
    let mut max_ratio = 0.0f32;
    let mut expanded = 0;
    let mut peak_bytes = 0;
    for (i, pair) in endpoints.windows(2).enumerate() {
        let a = pair[0];
        let mut b = pair[1];
        let mut local = false;
        // A pair whose funnel baseline exhausts the node pool or is partial gets
        // one deterministic local replacement end within LOCAL_RADIUS of `a`;
        // if that baseline is incomplete too, the pair is skipped.
        if baseline(&mut q, a, b).map_err(|e| e.to_string())?.is_none() {
            let (polygon, position, _) = q
                .find_random_point_around_circle(
                    a.polygon,
                    &a.position,
                    LOCAL_RADIUS,
                    &DefaultFilter::default(),
                    &mut || rng.borrow_mut().frand(),
                )
                .map_err(|e| e.to_string())?;
            b = AnyAnglePoint { polygon, position };
            if baseline(&mut q, a, b).map_err(|e| e.to_string())?.is_none() {
                skipped += 1;
                continue;
            }
            local_pairs += 1;
            local = true;
        }
        // Alternate AB/BA, on identical endpoints, within one process. Warm each
        // implementation once; timings include its own allocations.
        let (valid_baseline, baseline_tolerance) =
            baseline_containment(&nav, &mut q, a, b).map_err(|e| e.to_string())?;
        invalid_baselines += usize::from(!valid_baseline);
        baseline_tolerance_segments += baseline_tolerance;
        let expected = candidate(&q, a, b);
        if let Err(error) = &expected {
            failures += 1;
            eprintln!("query {i}: {error} during untimed warmup; timed comparison not run");
            continue;
        }
        timed_queries += 1;
        let before_base_ns = base_ns;
        let before_query_ns = query_ns;
        let mut base = None;
        let mut result = Err(Error::Failed);
        for repeat in 0..repeats {
            if (i + repeat) % 2 == 0 {
                let t = Instant::now();
                base = baseline(&mut q, a, b).map_err(|e| e.to_string())?;
                base_ns += t.elapsed().as_nanos();
                let t = Instant::now();
                result = candidate(&q, a, b);
                query_ns += t.elapsed().as_nanos();
            } else {
                let t = Instant::now();
                result = candidate(&q, a, b);
                query_ns += t.elapsed().as_nanos();
                let t = Instant::now();
                base = baseline(&mut q, a, b).map_err(|e| e.to_string())?;
                base_ns += t.elapsed().as_nanos();
            }
        }
        if result != expected {
            return Err(format!("query {i}: repeated search differs"));
        }
        if !local {
            primary_queries += 1;
            primary_baseline_ns += base_ns - before_base_ns;
            primary_interval_ns += query_ns - before_query_ns;
        }
        match (base, result) {
            (Some(base), Ok(path)) => {
                dump_queries.push_str(&format!(
                    "{} {} {} {} {}\n",
                    a.position[0], a.position[2], b.position[0], b.position[2], path.length
                ));
                expanded += path.expanded;
                if !local {
                    primary_expanded += path.expanded;
                }
                peak_bytes = peak_bytes.max(path.search_bytes);
                if valid_baseline && base > 0.0 {
                    max_ratio = max_ratio.max(path.length / base);
                }
                if valid_baseline && path.length > base {
                    failures += 1;
                    eprintln!("query {i}: interval {} exceeds funnel {base}", path.length);
                    continue;
                }
                let mut valid = true;
                let mut used_tolerance = false;
                for (j, seg) in path.points.windows(2).enumerate() {
                    if let Some(step) = off_mesh_step(&nav, path.polygons[j], &seg[0], &seg[1]) {
                        off_mesh_segments += 1;
                        if !step {
                            valid = false;
                            eprintln!("query {i} segment {j}: off-mesh step mismatch");
                        }
                        continue;
                    }
                    let (hit, _) = q
                        .raycast(
                            path.polygons[j],
                            &seg[0],
                            &seg[1],
                            &DefaultFilter::default(),
                            65535,
                        )
                        .map_err(|e| e.to_string())?;
                    let gap = coverage(&nav, path.polygons[j], &seg[0], &seg[1], false)?;
                    let exact_gap =
                        coverage_with_slack(&nav, path.polygons[j], &seg[0], &seg[1], false, 0.0)?;
                    max_exact_gap = max_exact_gap.max(exact_gap);
                    rounding_segments += usize::from(gap == 0.0 && exact_gap > 0.0);
                    if gap > 0.0 && std::env::var_os("NAV_COVERAGE_DEBUG").is_some() {
                        coverage(&nav, path.polygons[j], &seg[0], &seg[1], true)?;
                    }
                    max_gap = max_gap.max(gap);
                    if gap == 0.0 && hit.t < 1.0 {
                        tolerance_segments += 1;
                        used_tolerance = true;
                        if hit.t < CONTAINED_T {
                            raycast_short += 1;
                            let (tile, poly) = nav
                                .tile_and_poly_by_ref(hit.path.last().copied().unwrap_or(0))
                                .map_err(|e| e.to_string())?;
                            let e = hit.hit_edge_index.max(0) as usize % poly.vert_count as usize;
                            let v = |k: usize| xz(&tile.verts[poly.verts[k] as usize * 3..]);
                            let side = |p: [f64; 2]| cross2(xz(&seg[0]), xz(&seg[1]), p);
                            eprintln!(
                                "query {i} segment {j}: covered, raycast stopped at {} on edge {:?}-{:?} (side {} {})",
                                hit.t,
                                v(e),
                                v((e + 1) % poly.vert_count as usize),
                                side(v(e)),
                                side(v((e + 1) % poly.vert_count as usize))
                            );
                        }
                    }
                    if gap > 0.0 {
                        valid = false;
                        eprintln!("query {i} segment {j}: coverage gap {gap}");
                        let mut alternatives = Vec::new();
                        q.query_polygons_with(
                            &seg[0],
                            &[0.001, 1.0, 0.001],
                            &DefaultFilter::default(),
                            &mut |_, _, r| alternatives.push(r),
                        )
                        .map_err(|e| e.to_string())?;
                        for r in alternatives {
                            let (alternative, _) = q
                                .raycast(r, &seg[0], &seg[1], &DefaultFilter::default(), 65535)
                                .map_err(|e| e.to_string())?;
                            eprintln!(
                                "query {i} segment {j}: alternative_ref={r} t={}",
                                alternative.t
                            );
                        }
                        eprintln!(
                            "query {i} segment {j}: ray stopped at {} from {:?} to {:?} ref {}",
                            hit.t, seg[0], seg[1], path.polygons[j]
                        );
                    }
                }
                tolerance_queries += usize::from(used_tolerance);
                if !valid {
                    failures += 1;
                    let mut corridor = Vec::new();
                    let mut straight = Vec::new();
                    q.find_path(
                        a.polygon,
                        b.polygon,
                        &a.position,
                        &b.position,
                        &DefaultFilter::default(),
                        &mut corridor,
                        65535,
                    )
                    .map_err(|e| e.to_string())?;
                    q.find_straight_path(
                        &a.position,
                        &b.position,
                        &corridor,
                        &mut straight,
                        65535,
                        0,
                    )
                    .map_err(|e| e.to_string())?;
                    for (j, segment) in straight.windows(2).enumerate() {
                        let (hit, _) = q
                            .raycast(
                                segment[0].r#ref,
                                &segment[0].pos,
                                &segment[1].pos,
                                &DefaultFilter::default(),
                                65535,
                            )
                            .map_err(|e| e.to_string())?;
                        eprintln!(
                            "baseline query {i} segment {j}: t={} from {:?} to {:?}",
                            hit.t, segment[0].pos, segment[1].pos
                        );
                    }
                    continue;
                }

                digest.f32(path.length);
                for point in path.points {
                    digest.v3(point);
                }
                passed += 1;
            }
            (Some(_), Err(e)) => {
                failures += 1;
                eprintln!("query {i}: {e}");
                continue;
            }
            (None, _) => {}
        }
    }
    if let Some(dir) = &dump {
        std::fs::write(dir.join("queries.txt"), dump_queries).map_err(|e| e.to_string())?;
    }
    println!(
        "queries={passed} failures={failures} max_length_ratio={max_ratio:.8} expanded={expanded} peak_search_bytes={peak_bytes} baseline_ns={base_ns} interval_ns={query_ns} digest={:016x} repeats={repeats} timed_queries={timed_queries} skipped={skipped} invalid_baselines={invalid_baselines} tolerance_queries={tolerance_queries} tolerance_segments={tolerance_segments} baseline_tolerance_segments={baseline_tolerance_segments} raycast_short={raycast_short} off_mesh_segments={off_mesh_segments} local_pairs={local_pairs} max_gap={max_gap:e} rounding_segments={rounding_segments} max_exact_gap={max_exact_gap:e} primary_queries={primary_queries} primary_baseline_ns={primary_baseline_ns} primary_interval_ns={primary_interval_ns} primary_expanded={primary_expanded}",
        navmesh_parity::fnv1a64(digest.as_bytes())
    );
    if failures > 0 || passed == 0 {
        Err(format!(
            "{failures} any-angle qualification failures, {passed} validated queries"
        ))
    } else {
        Ok(())
    }
}

// Solo-mesh adapter for the independent external oracle. It refuses partial
// portals and overlapping projected vertices rather than merging their topology.
fn export(nav: &NavMesh, dir: &std::path::Path) -> Result<(), String> {
    let mut polygons = Vec::new();
    for ti in 0..nav.max_tiles() as usize {
        if let Some(tile) = nav.tile(ti) {
            for (pi, p) in tile.polys.iter().enumerate() {
                if p.poly_type() == detour::POLYTYPE_GROUND {
                    polygons.push((
                        nav.poly_ref_base(ti).ok_or("missing tile")? | pi as PolyRef,
                        tile,
                        p,
                    ));
                }
            }
        }
    }
    let mut coords: Vec<[f32; 3]> = Vec::new();
    let mut rings = Vec::new();
    for (_, tile, p) in &polygons {
        let mut ring = Vec::new();
        for &v in p.verts[..p.vert_count as usize].iter().rev() {
            let at = v as usize * 3;
            let point = [tile.verts[at], tile.verts[at + 1], tile.verts[at + 2]];
            let index = if let Some(i) = coords
                .iter()
                .position(|c| c[0] == point[0] && c[2] == point[2])
            {
                if coords[i][1] != point[1] {
                    return Err(
                        "overlapping projected vertices unsupported by external adapter".into(),
                    );
                }
                i
            } else {
                coords.push(point);
                coords.len() - 1
            };
            ring.push(index + 1);
        }
        rings.push(ring);
    }
    let mut text = format!("mesh\n3\n{} {}\n", coords.len(), polygons.len());
    for p in &coords {
        text.push_str(&format!("{} {}\n", p[0], p[2]));
    }
    for ((_, tile, p), ring) in polygons.iter().zip(&rings) {
        text.push_str(&format!("1 {}", ring.len()));
        for i in ring {
            text.push_str(&format!(" {i}"));
        }
        for j in 0..ring.len() {
            let original_edge = (ring.len() * 2 - 2 - j) % ring.len();
            let mut neighbour = 0;
            let mut link = p.first_link;
            while link != detour::NULL_LINK {
                let l = tile.links[link as usize];
                link = l.next;
                if l.edge as usize != original_edge || l.r#ref == 0 {
                    continue;
                }
                if l.side != 0xff {
                    return Err("tile portals unsupported by external solo adapter".into());
                }
                if let Some(index) = polygons.iter().position(|&(r, _, _)| r == l.r#ref) {
                    neighbour = index + 1;
                }
            }
            text.push_str(&format!(" {neighbour}"));
        }
        text.push('\n');
    }
    std::fs::create_dir_all(dir).map_err(|e| e.to_string())?;
    std::fs::write(dir.join("mesh.mesh"), text).map_err(|e| e.to_string())
}

#[cfg(test)]
mod containment_tests {
    use super::*;

    fn squares(connected: bool) -> Result<NavMesh, Error> {
        // Four squares meet at (1, 1). Adjacency, rather than projected
        // overlap alone, determines whether a segment may cross between them.
        let mut verts = Vec::new();
        for z in 0..3 {
            for x in 0..3 {
                verts.extend([x, 0, z]);
            }
        }
        let mut polys = Vec::new();
        for cell in 0..4usize {
            let (x, z) = (cell % 2, cell / 2);
            polys.extend(
                [
                    x + z * 3,
                    x + (z + 1) * 3,
                    x + 1 + (z + 1) * 3,
                    x + 1 + z * 3,
                ]
                .map(|v| v as u16),
            );
            for next in [
                x.checked_sub(1).map(|x| x + z * 2),
                (z == 0).then_some(cell + 2),
                (x == 0).then_some(cell + 1),
                z.checked_sub(1).map(|z| x + z * 2),
            ] {
                polys.push(if connected {
                    next.map_or(u16::MAX, |p| p as u16)
                } else {
                    u16::MAX
                });
            }
        }
        let data = detour::create_nav_mesh_data(&detour::NavMeshCreateParams {
            verts: &verts,
            vert_count: 9,
            polys: &polys,
            poly_count: 4,
            nvp: 4,
            poly_flags: &[1; 4],
            poly_areas: &[0; 4],
            bmin: [0.0; 3],
            bmax: [2.0, 1.0, 2.0],
            cs: 1.0,
            ch: 1.0,
            ..Default::default()
        })?;
        let mut nav = NavMesh::with_limits(
            &detour::NavMeshParams {
                orig: [0.0; 3],
                tile_width: 2.0,
                tile_height: 2.0,
                max_tiles: 1,
                max_polys: 4,
            },
            navmesh_rs::Limits::default(),
        )?;
        nav.add_tile(data)?;
        Ok(nav)
    }

    #[test]
    fn closed_coverage_requires_portal_connectivity() -> Result<(), String> {
        for connected in [false, true] {
            let nav = squares(connected).map_err(|e| e.to_string())?;
            let start = nav.poly_ref_base(0).ok_or("missing tile")?;
            for (a, b) in [
                ([0.5, 0.0, 0.5], [1.5, 0.0, 1.5]),
                ([0.5, 0.0, 1.0], [1.5, 0.0, 1.0]),
            ] {
                assert_eq!(coverage(&nav, start, &a, &b, false)? == 0.0, connected);
            }
            // Leaving the union must fail even with all portals present.
            assert!(coverage(&nav, start, &[0.5, 0.0, 0.5], &[3.0, 0.0, 0.5], false)? > 0.9);
            // A raycast's containing-start precondition remains mandatory.
            assert!(coverage(&nav, start, &[1.5, 0.0, 0.5], &[1.8, 0.0, 0.5], false)? > 0.0);
        }
        Ok(())
    }
}
