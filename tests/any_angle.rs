#![cfg(feature = "detour")]
use navmesh_rs::detour::{
    self, AnyAnglePoint, AnyAngleSearch, DefaultFilter, NavMesh, NavMeshCreateParams,
    NavMeshParams, NavMeshQuery,
};
use navmesh_rs::{Budget, Budgeted, Error, Limits};

// Grid polygons with explicit adjacency; obstacles delete cells. The independent
// oracle below works on the closed union of unit squares, without Detour queries.
fn grid(width: usize, height: usize, blocked: &[usize]) -> Result<NavMesh, Error> {
    let mut verts = Vec::new();
    for z in 0..=height {
        for x in 0..=width {
            verts.extend([x as u16, 0, z as u16]);
        }
    }
    let cells: Vec<_> = (0..width * height)
        .filter(|i| !blocked.contains(i))
        .collect();
    let mut polys = Vec::new();
    for &cell in &cells {
        let x = cell % width;
        let z = cell / width;
        polys.extend([
            (x + z * (width + 1)) as u16,
            (x + (z + 1) * (width + 1)) as u16,
            (x + 1 + (z + 1) * (width + 1)) as u16,
            (x + 1 + z * (width + 1)) as u16,
        ]);
        for next in [
            x.checked_sub(1).map(|xx| xx + z * width),
            (z + 1 < height).then_some(cell + width),
            (x + 1 < width).then_some(cell + 1),
            z.checked_sub(1).map(|zz| x + zz * width),
        ] {
            polys.push(
                next.and_then(|n| cells.iter().position(|&c| c == n))
                    .map_or(u16::MAX, |p| p as u16),
            );
        }
    }
    let data = detour::create_nav_mesh_data(&NavMeshCreateParams {
        verts: &verts,
        vert_count: (verts.len() / 3) as i32,
        polys: &polys,
        poly_count: cells.len() as i32,
        nvp: 4,
        poly_flags: &vec![1; cells.len()],
        poly_areas: &vec![0; cells.len()],
        bmin: [0.0; 3],
        bmax: [width as f32, 1.0, height as f32],
        cs: 1.0,
        ch: 1.0,
        walkable_height: 1.0,
        walkable_climb: 0.1,
        ..Default::default()
    })?;
    let mut nav = NavMesh::with_limits(
        &NavMeshParams {
            orig: [0.0; 3],
            tile_width: width as f32,
            tile_height: height as f32,
            max_tiles: 1,
            max_polys: 256,
        },
        Limits::default(),
    )?;
    nav.add_tile(data)?;
    Ok(nav)
}
fn endpoint(q: &NavMeshQuery<'_>, x: f32, z: f32) -> Result<AnyAnglePoint, Error> {
    let (polygon, position, _) =
        q.find_nearest_poly(&[x, 0.0, z], &[0.01, 1.0, 0.01], &DefaultFilter::default())?;
    Ok(AnyAnglePoint { polygon, position })
}
fn length(a: [f64; 2], b: [f64; 2]) -> f64 {
    ((a[0] - b[0]).powi(2) + (a[1] - b[1]).powi(2)).sqrt()
}
fn visible(a: [f64; 2], b: [f64; 2], width: usize, height: usize, blocked: &[usize]) -> bool {
    // Split at every grid-line crossing, then test each open subsegment's
    // midpoint for membership in a live cell. Exact for this rectilinear fixture.
    let mut ts = vec![0.0, 1.0];
    for axis in 0..2 {
        let d = b[axis] - a[axis];
        if d != 0.0 {
            for line in 0..=width.max(height) {
                let t = (line as f64 - a[axis]) / d;
                if t > 0.0 && t < 1.0 {
                    ts.push(t);
                }
            }
        }
    }
    ts.sort_by(f64::total_cmp);
    ts.windows(2).all(|w| {
        let t = (w[0] + w[1]) * 0.5;
        let p = [a[0] + (b[0] - a[0]) * t, a[1] + (b[1] - a[1]) * t];
        (0..width * height).any(|c| {
            !blocked.contains(&c)
                && p[0] >= (c % width) as f64
                && p[0] <= (c % width + 1) as f64
                && p[1] >= (c / width) as f64
                && p[1] <= (c / width + 1) as f64
        })
    })
}
fn exact(start: [f64; 2], end: [f64; 2], w: usize, h: usize, blocked: &[usize]) -> f64 {
    let mut vertices = vec![start, end];
    for z in 0..=h {
        for x in 0..=w {
            vertices.push([x as f64, z as f64]);
        }
    }
    let mut costs = vec![f64::INFINITY; vertices.len()];
    let mut closed = vec![false; vertices.len()];
    costs[0] = 0.0;
    loop {
        let next = (0..vertices.len())
            .filter(|&i| !closed[i])
            .min_by(|&a, &b| costs[a].total_cmp(&costs[b]));
        let Some(i) = next else {
            return f64::INFINITY;
        };
        if i == 1 {
            return costs[1];
        }
        closed[i] = true;
        for j in 0..vertices.len() {
            if !closed[j] && visible(vertices[i], vertices[j], w, h, blocked) {
                costs[j] = costs[j].min(costs[i] + length(vertices[i], vertices[j]));
            }
        }
    }
}
#[test]
fn exact_oracle_and_funnel_across_obstacles() -> Result<(), Error> {
    let mut cases = 0;
    for blocked in [
        vec![],
        vec![7, 13, 19],
        vec![8, 9, 14, 15],
        vec![2, 8, 14, 20],
    ] {
        let nav = grid(6, 5, &blocked)?;
        let mut q = NavMeshQuery::new(&nav, 65535)?;
        for (a, b) in [
            ([0.25, 0.25], [5.75, 4.75]),
            ([0.25, 4.75], [5.75, 0.25]),
            ([1.25, 2.25], [4.75, 2.75]),
        ] {
            if blocked.contains(&((a[0] as usize) + (a[1] as usize) * 6))
                || blocked.contains(&((b[0] as usize) + (b[1] as usize) * 6))
            {
                continue;
            }
            let start = endpoint(&q, a[0], a[1])?;
            let end = endpoint(&q, b[0], b[1])?;
            let filter = DefaultFilter::default();
            let result = q
                .find_any_angle_path(start, end, &filter, &mut Budget::unlimited())?
                .into_value();
            let oracle = exact(
                [a[0] as f64, a[1] as f64],
                [b[0] as f64, b[1] as f64],
                6,
                5,
                &blocked,
            );
            assert!(
                (result.length as f64 - oracle).abs() < 1e-4,
                "length {} oracle {oracle} blocked {blocked:?}",
                result.length
            );
            for segment in result.points.windows(2) {
                assert!(visible(
                    [segment[0][0] as f64, segment[0][2] as f64],
                    [segment[1][0] as f64, segment[1][2] as f64],
                    6,
                    5,
                    &blocked
                ));
            }
            let mut path = Vec::new();
            let status = q.find_path(
                start.polygon,
                end.polygon,
                &start.position,
                &end.position,
                &filter,
                &mut path,
                256,
            )?;
            assert!(!status.has_detail(detour::Status::PARTIAL_RESULT));
            let mut straight = Vec::new();
            q.find_straight_path(&start.position, &end.position, &path, &mut straight, 256, 0)?;
            let baseline: f32 = straight
                .windows(2)
                .map(|p| {
                    length(
                        [p[0].pos[0] as f64, p[0].pos[2] as f64],
                        [p[1].pos[0] as f64, p[1].pos[2] as f64],
                    ) as f32
                })
                .sum();
            assert!(result.length <= baseline);
            let mut search = AnyAngleSearch::begin(&q, start, end, &filter)?;
            assert!(matches!(
                search.resume(&q, &mut Budget::new(0))?,
                Budgeted::Exhausted(_)
            ));
            loop {
                let mut budget = Budget::new(1);
                let next = search.resume(&q, &mut budget)?;
                assert!(budget.used() <= 1);
                if let Budgeted::Complete(p) = next {
                    assert_eq!(p, result);
                    break;
                }
            }
            cases += 1;
        }
    }
    println!("exact_oracle_queries={cases}");
    Ok(())
}
#[test]
fn unsupported_and_mesh_identity() -> Result<(), Error> {
    let nav = grid(2, 2, &[])?;
    let q = NavMeshQuery::new(&nav, 256)?;
    let start = endpoint(&q, 0.2, 0.2)?;
    let end = endpoint(&q, 1.8, 1.8)?;
    let mut weighted = DefaultFilter::default();
    weighted.area_cost[1] = 2.0;
    assert!(matches!(
        AnyAngleSearch::begin(&q, start, end, &weighted),
        Err(Error::Unsupported)
    ));
    let filter = DefaultFilter::default();
    let mut search = AnyAngleSearch::begin(&q, start, end, &filter)?;
    let other = grid(2, 2, &[])?;
    let oq = NavMeshQuery::new(&other, 256)?;
    assert_eq!(
        search.resume(&oq, &mut Budget::unlimited()),
        Err(Error::MeshChanged)
    );
    Ok(())
}

fn portal_tile(x: i32, layer: i32, zmin: u16, zmax: u16, y: f32) -> Result<Vec<u8>, Error> {
    let n = u16::MAX;
    detour::create_nav_mesh_data(&NavMeshCreateParams {
        verts: &[0, 0, zmin, 0, 0, zmax, 10, 0, zmax, 10, 0, zmin],
        vert_count: 4,
        polys: &[0, 1, 2, 3, 0x8000, n, 0x8002, n],
        poly_count: 1,
        nvp: 4,
        poly_flags: &[1],
        poly_areas: &[0],
        tile_x: x,
        tile_layer: layer,
        bmin: [x as f32 * 10.0, y, 0.0],
        bmax: [x as f32 * 10.0 + 10.0, y + 1.0, 10.0],
        cs: 1.0,
        ch: 1.0,
        walkable_climb: 0.1,
        walkable_height: 1.0,
        ..Default::default()
    })
}
#[test]
fn partial_portals_keep_their_intervals() -> Result<(), Error> {
    let mut nav = NavMesh::with_limits(
        &NavMeshParams {
            orig: [0.0; 3],
            tile_width: 10.0,
            tile_height: 10.0,
            max_tiles: 4,
            max_polys: 16,
        },
        Limits::default(),
    )?;
    let a = nav.add_tile(portal_tile(0, 0, 0, 10, 0.0)?)?;
    let b = nav.add_tile(portal_tile(1, 0, 3, 7, 0.0)?)?;
    let q = NavMeshQuery::new(&nav, 4096)?;
    let start = AnyAnglePoint {
        polygon: a,
        position: [1., 0., 0.5],
    };
    let end = AnyAnglePoint {
        polygon: b,
        position: [19., 0., 3.5],
    };
    let result = q
        .find_any_angle_path(
            start,
            end,
            &DefaultFilter::default(),
            &mut Budget::unlimited(),
        )?
        .into_value();
    // Quantized partial portals are the actual Detour domain, including their
    // rounded endpoint rather than an assumed whole shared edge.
    let (left, right) = q.portal_points(a, b)?;
    let mut gate = if left[2] < right[2] { left } else { right };
    gate[2] = gate[2].clamp(3.0, 7.0);
    let expected = length([1., 0.5], [gate[0] as f64, gate[2] as f64])
        + length([gate[0] as f64, gate[2] as f64], [19., 3.5]);
    assert!((result.length as f64 - expected).abs() < 1e-4);
    Ok(())
}
#[test]
fn stacked_floors_and_epoch_changes() -> Result<(), Error> {
    let mut nav = NavMesh::with_limits(
        &NavMeshParams {
            orig: [0.0; 3],
            tile_width: 10.0,
            tile_height: 10.0,
            max_tiles: 4,
            max_polys: 16,
        },
        Limits::default(),
    )?;
    let lower = nav.add_tile(portal_tile(0, 0, 0, 10, 0.0)?)?;
    let upper = nav.add_tile(portal_tile(0, 1, 0, 10, 5.0)?)?;
    let filter = DefaultFilter::default();
    let start = AnyAnglePoint {
        polygon: lower,
        position: [1., 0., 1.],
    };
    let end = AnyAnglePoint {
        polygon: upper,
        position: [9., 5., 9.],
    };
    let mut search = {
        let q = NavMeshQuery::new(&nav, 4096)?;
        assert_eq!(
            q.find_any_angle_path(start, end, &filter, &mut Budget::unlimited()),
            Err(Error::Failed)
        );
        AnyAngleSearch::begin(&q, start, start, &filter)?
    };
    nav.remove_tile(lower)?;
    let q = NavMeshQuery::new(&nav, 4096)?;
    assert_eq!(
        search.resume(&q, &mut Budget::unlimited()),
        Err(Error::MeshChanged)
    );
    Ok(())
}

#[test]
fn endpoints_on_portal_interiors_and_vertices() -> Result<(), Error> {
    let nav = grid(3, 3, &[])?;
    let q = NavMeshQuery::new(&nav, 4096)?;
    let base = nav.poly_ref_base(0).ok_or(Error::InvalidRef)?;
    for start in [
        AnyAnglePoint {
            polygon: base,
            position: [0.5, 0.0, 1.0],
        },
        AnyAnglePoint {
            polygon: base,
            position: [1.0, 0.0, 1.0],
        },
    ] {
        let end = endpoint(&q, 2.8, 2.8)?;
        let result = q
            .find_any_angle_path(
                start,
                end,
                &DefaultFilter::default(),
                &mut Budget::unlimited(),
            )?
            .into_value();
        let expected = length(
            [start.position[0] as f64, start.position[2] as f64],
            [2.8, 2.8],
        );
        assert!((result.length as f64 - expected).abs() < 1e-4);
    }
    Ok(())
}

fn three_bands(
    wide_last: bool,
) -> Result<(NavMesh, detour::PolyRef, detour::PolyRef, detour::PolyRef), Error> {
    let mut nav = NavMesh::with_limits(
        &NavMeshParams {
            orig: [0.0; 3],
            tile_width: 10.0,
            tile_height: 10.0,
            max_tiles: 4,
            max_polys: 16,
        },
        Limits::default(),
    )?;
    let a = nav.add_tile(portal_tile(0, 0, 0, 10, 0.0)?)?;
    let b = nav.add_tile(portal_tile(1, 0, 3, 7, 0.0)?)?;
    let c = nav.add_tile(portal_tile(
        2,
        0,
        if wide_last { 0 } else { 3 },
        if wide_last { 10 } else { 7 },
        0.0,
    )?)?;
    Ok((nav, a, b, c))
}
#[test]
fn partial_portal_can_be_an_intermediate_polygon() -> Result<(), Error> {
    let (nav, a, _, c) = three_bands(false)?;
    let q = NavMeshQuery::new(&nav, 4096)?;
    let result = q
        .find_any_angle_path(
            AnyAnglePoint {
                polygon: a,
                position: [1., 0., 0.5],
            },
            AnyAnglePoint {
                polygon: c,
                position: [29., 0., 3.5],
            },
            &DefaultFilter::default(),
            &mut Budget::unlimited(),
        )?
        .into_value();
    for (i, segment) in result.points.windows(2).enumerate() {
        let (hit, _) = q.raycast(
            result.polygons[i],
            &segment[0],
            &segment[1],
            &DefaultFilter::default(),
            256,
        )?;
        assert!(hit.t >= 1.0);
    }
    Ok(())
}
#[test]
#[ignore = "explicit strict partial-portal qualification"]
fn quantized_portal_never_longer_qualification() -> Result<(), Error> {
    let (nav, a, b, c) = three_bands(true)?;
    let mut q = NavMeshQuery::new(&nav, 4096)?;
    let (left, right) = q.portal_points(a, b)?;
    let low = left[2].min(right[2]);
    let high = left[2].max(right[2]);
    let (z, corner) = if low < 3.0 {
        (0.5, 3.0)
    } else if high > 7.0 {
        (9.0, 7.0)
    } else {
        return Err(Error::Failed);
    };
    let start = AnyAnglePoint {
        polygon: a,
        position: [1., 0., z],
    };
    let end = AnyAnglePoint {
        polygon: c,
        position: [29., 0., z],
    };
    let filter = DefaultFilter::default();
    let result = q
        .find_any_angle_path(start, end, &filter, &mut Budget::unlimited())?
        .into_value();
    // This side's true corner belongs to both the quantized interval and the
    // receiving rectangle. Two mandatory corners give an independent analytic
    // reference over the closed union, respecting both effective portals.
    let dy = (z - corner) as f64;
    let exact = 2.0 * (81.0 + dy * dy).sqrt() + 10.0;
    let mut corridor = Vec::new();
    let mut straight = Vec::new();
    q.find_path(
        a,
        c,
        &start.position,
        &end.position,
        &filter,
        &mut corridor,
        16,
    )?;
    q.find_straight_path(
        &start.position,
        &end.position,
        &corridor,
        &mut straight,
        16,
        0,
    )?;
    let base: f32 = straight
        .windows(2)
        .map(|p| {
            let dx = p[1].pos[0] - p[0].pos[0];
            let dz = p[1].pos[2] - p[0].pos[2];
            (dx * dx + dz * dz).sqrt()
        })
        .sum();
    println!(
        "partial_portal exact={exact:.9} interval={:.9} funnel={base:.9} portal={:?} points={:?} funnel_points={:?}",
        result.length,
        q.portal_points(a, b)?,
        result.points,
        straight
    );
    assert!(
        (result.length as f64 - exact).abs() <= 1e-4,
        "analytic exact-reference mismatch"
    );
    for (i, segment) in result.points.windows(2).enumerate() {
        let (hit, _) = q.raycast(result.polygons[i], &segment[0], &segment[1], &filter, 256)?;
        assert!(hit.t >= 1.0, "candidate containment failed");
    }
    assert!(
        result.length <= base,
        "strict never-longer qualification failed on a quantized portal"
    );
    Ok(())
}
