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
    let mut dump_queries = String::new();
    let mut digest = navmesh_parity::encode::Payload::new();
    let mut failures = 0;
    let mut skipped = 0;
    let mut passed = 0;
    let mut base_ns = 0;
    let mut query_ns = 0;
    let mut max_ratio = 0.0f32;
    let mut expanded = 0;
    let mut peak_bytes = 0;
    for (i, pair) in endpoints.windows(2).enumerate() {
        let a = pair[0];
        let b = pair[1];
        // Alternate AB/BA, on identical endpoints, within one process. Warm each
        // implementation once; timings include its own allocations.
        if baseline(&mut q, a, b).map_err(|e| e.to_string())?.is_none() {
            skipped += 1;
            continue;
        }
        let expected = candidate(&q, a, b);
        if let Err(error) = &expected {
            failures += 1;
            eprintln!("query {i}: {error} during untimed warmup; timed comparison not run");
            break;
        }
        let mut base = None;
        let mut result = Err(Error::Failed);
        for repeat in 0..4 {
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
        match (base, result) {
            (Some(base), Ok(path)) => {
                dump_queries.push_str(&format!(
                    "{} {} {} {} {}\n",
                    a.position[0], a.position[2], b.position[0], b.position[2], path.length
                ));
                expanded += path.expanded;
                peak_bytes = peak_bytes.max(path.search_bytes);
                if base > 0.0 {
                    max_ratio = max_ratio.max(path.length / base);
                }
                if path.length > base {
                    failures += 1;
                    eprintln!("query {i}: interval {} exceeds funnel {base}", path.length);
                    break;
                }
                let mut valid = true;
                for (j, seg) in path.points.windows(2).enumerate() {
                    let (hit, _) = q
                        .raycast(
                            path.polygons[j],
                            &seg[0],
                            &seg[1],
                            &DefaultFilter::default(),
                            65535,
                        )
                        .map_err(|e| e.to_string())?;
                    if hit.t < 1.0 {
                        valid = false;
                        eprintln!(
                            "query {i} segment {j}: ray stopped at {} from {:?} to {:?} ref {}",
                            hit.t, seg[0], seg[1], path.polygons[j]
                        );
                    }
                }
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
                    break;
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
                break;
            }
            (None, _) => {}
        }
    }
    if let Some(dir) = &dump {
        std::fs::write(dir.join("queries.txt"), dump_queries).map_err(|e| e.to_string())?;
    }
    println!(
        "queries={passed} failures={failures} max_length_ratio={max_ratio:.8} expanded={expanded} peak_search_bytes={peak_bytes} baseline_ns={base_ns} interval_ns={query_ns} digest={:016x} skipped={skipped}",
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
