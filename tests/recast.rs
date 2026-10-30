//! Recast stage API: a full build on a small mesh, typed errors for malformed
//! input and exhausted limits, and no panics on arbitrary input. Bit-exactness
//! against the C++ oracle is checked by `nav-parity` over the corpus (see
//! `docs/VERIFICATION.md`).

#![cfg(feature = "recast")]

use navmesh_rs::recast::{
    self, CompactHeightfield, ContourSet, Heightfield, HeightfieldLayerSet, PolyMesh,
    PolyMeshDetail,
};
use navmesh_rs::{Error, Limits, Workspace};

struct Settings {
    cs: f32,
    ch: f32,
    height: i32,
    climb: i32,
    radius: i32,
    partition: u32,
    nvp: i32,
}

const DEFAULT: Settings = Settings {
    cs: 0.3,
    ch: 0.2,
    height: 10,
    climb: 4,
    radius: 2,
    partition: 0,
    nvp: 6,
};

fn build(
    ws: &mut Workspace,
    verts: &[f32],
    tris: &[i32],
    s: &Settings,
) -> Result<(PolyMesh, PolyMeshDetail), Error> {
    let (bmin, bmax) = recast::calc_bounds(verts);
    let (w, h) = recast::calc_grid_size(&bmin, &bmax, s.cs);
    let mut hf = Heightfield::new(ws, w, h, bmin, bmax, s.cs, s.ch)?;
    let mut areas = vec![0u8; tris.len() / 3];
    recast::mark_walkable_triangles(ws, 45.0, verts, tris, &mut areas)?;
    hf.rasterize_triangles(ws, verts, tris, &areas, s.climb)?;
    hf.filter_low_hanging_walkable_obstacles(ws, s.climb)?;
    hf.filter_ledge_spans(ws, s.height, s.climb)?;
    hf.filter_walkable_low_height_spans(ws, s.height)?;
    let mut chf = CompactHeightfield::build(ws, s.height, s.climb, &hf)?;
    recast::erode_walkable_area(ws, s.radius, &mut chf)?;
    recast::mark_cylinder_area(ws, [1.0, -1.0, 1.0], 0.5, 3.0, 5, &mut chf)?;
    HeightfieldLayerSet::build(ws, &chf, 0, s.height)?;
    match s.partition % 3 {
        0 => {
            recast::build_distance_field(ws, &mut chf)?;
            recast::build_regions(ws, &mut chf, 0, 8, 20)?;
        }
        1 => recast::build_regions_monotone(ws, &mut chf, 0, 8, 20)?,
        _ => recast::build_layer_regions(ws, &mut chf, 0, 8)?,
    }
    let cset = ContourSet::build(ws, &chf, 1.3, 40, recast::CONTOUR_TESS_WALL_EDGES)?;
    let pmesh = PolyMesh::build(ws, &cset, s.nvp)?;
    let dmesh = PolyMeshDetail::build(ws, &pmesh, &chf, 1.8, 0.2)?;
    Ok((pmesh, dmesh))
}

fn square(size: f32) -> (Vec<f32>, Vec<i32>) {
    (
        vec![
            0.0, 0.0, 0.0, 0.0, 0.0, size, size, 0.0, size, size, 0.0, 0.0,
        ],
        vec![0, 1, 2, 0, 2, 3],
    )
}

#[test]
fn flat_square_builds_polygons_with_a_detail_mesh() {
    let (verts, tris) = square(10.0);
    let mut ws = Workspace::new();
    let (pmesh, dmesh) = build(&mut ws, &verts, &tris, &DEFAULT).expect("build");
    assert!(pmesh.npolys >= 1);
    assert_eq!(dmesh.meshes.len(), pmesh.npolys as usize * 4);
    // The cylinder marked area 5 on part of the square.
    assert!(pmesh.areas[..pmesh.npolys as usize].contains(&5));
    // Eroded by 2 cells: every vertex stays inside [2, 31] cells.
    let nv = pmesh.nverts as usize;
    assert!(
        pmesh.verts[..nv * 3]
            .chunks(3)
            .all(|v| (2..=31).contains(&v[0]) && (2..=31).contains(&v[2]))
    );
}

#[test]
fn malformed_input_is_a_typed_error() {
    let mut ws = Workspace::new();
    let (mut verts, tris) = square(10.0);
    let mut hf = Heightfield::new(&mut ws, 33, 33, [0.0; 3], [10.0, 1.0, 10.0], 0.3, 0.2)
        .expect("heightfield");
    assert_eq!(
        hf.rasterize_triangles(&mut ws, &verts, &[0, 1, 9], &[63], 1),
        Err(Error::InvalidInput("triangle index out of range"))
    );
    assert!(matches!(
        hf.rasterize_triangles(&mut ws, &verts, &tris, &[63], 1),
        Err(Error::InvalidInput(_))
    ));
    verts[4] = f32::NAN;
    assert!(matches!(
        hf.rasterize_triangles(&mut ws, &verts, &tris, &[63, 63], 1),
        Err(Error::InvalidInput(_))
    ));
    assert!(matches!(
        Heightfield::new(&mut ws, 10, 10, [0.0; 3], [1.0; 3], 0.0, 0.2),
        Err(Error::InvalidParameter(_))
    ));
}

#[test]
fn limits_are_enforced_per_call() {
    let (verts, tris) = square(10.0);
    let mut ws = Workspace::with_limits(Limits {
        max_bytes: usize::MAX,
        max_work: 10,
        ..Limits::UNLIMITED
    });
    assert_eq!(
        build(&mut ws, &verts, &tris, &DEFAULT).err(),
        Some(Error::LimitExceeded)
    );
    let mut ws = Workspace::with_limits(Limits {
        max_bytes: 1000,
        max_work: u64::MAX,
        ..Limits::UNLIMITED
    });
    assert_eq!(
        build(&mut ws, &verts, &tris, &DEFAULT).err(),
        Some(Error::LimitExceeded)
    );
    let mut ws = Workspace::new();
    build(&mut ws, &verts, &tris, &DEFAULT).expect("unlimited build");
    assert!(ws.usage().bytes > 0 && ws.usage().work > 0);
}

/// The polygon merge pass compares every pair of a contour's triangles, so it is
/// charged as work: a disc costs more with merging (nvp 6) than without (nvp 3),
/// although merging leaves fewer polygons for the later steps, and a limit
/// between the two stops the merging build.
#[test]
fn polygon_merging_counts_against_max_work() {
    let n = 96;
    let mut verts = vec![0.0f32, 0.0, 0.0];
    let mut tris = Vec::new();
    for i in 0..n {
        let a = i as f32 / n as f32 * core::f32::consts::TAU;
        verts.extend_from_slice(&[20.0 * a.cos(), 0.0, 20.0 * a.sin()]);
        tris.extend_from_slice(&[0, 1 + (i + 1) % n, 1 + i]);
    }
    let mut ws = Workspace::new();
    let (bmin, bmax) = recast::calc_bounds(&verts);
    let (w, h) = recast::calc_grid_size(&bmin, &bmax, 0.3);
    let mut hf = Heightfield::new(&mut ws, w, h, bmin, bmax, 0.3, 0.2).unwrap();
    let mut areas = vec![0u8; tris.len() / 3];
    recast::mark_walkable_triangles(&mut ws, 45.0, &verts, &tris, &mut areas).unwrap();
    hf.rasterize_triangles(&mut ws, &verts, &tris, &areas, 4)
        .unwrap();
    let mut chf = CompactHeightfield::build(&mut ws, 10, 4, &hf).unwrap();
    recast::build_layer_regions(&mut ws, &mut chf, 0, 8).unwrap();
    let cset = ContourSet::build(&mut ws, &chf, 0.5, 0, recast::CONTOUR_TESS_WALL_EDGES).unwrap();
    PolyMesh::build(&mut ws, &cset, 3).unwrap();
    let without_merge = ws.usage().work;
    let merged = PolyMesh::build(&mut ws, &cset, 6).unwrap();
    let with_merge = ws.usage().work;
    assert!(merged.npolys > 0);
    assert!(
        with_merge > without_merge,
        "merge work {with_merge} vs {without_merge}"
    );
    ws.set_limits(Limits {
        max_work: without_merge,
        ..Limits::UNLIMITED
    });
    PolyMesh::build(&mut ws, &cset, 3).unwrap();
    assert_eq!(
        PolyMesh::build(&mut ws, &cset, 6).err(),
        Some(Error::LimitExceeded)
    );
}

#[test]
fn arbitrary_input_never_panics() {
    let mut state = 0x9e37_79b9u32;
    let mut rnd = move || {
        state ^= state << 13;
        state ^= state >> 17;
        state ^= state << 5;
        state
    };
    for _ in 0..300 {
        let nv = (rnd() % 12 + 1) as usize;
        let scale = [0.01f32, 1.0, 10.0, 200.0][(rnd() % 4) as usize];
        let verts: Vec<f32> = (0..nv * 3)
            .map(|_| (rnd() % 2001) as f32 / 1000.0 * scale - scale)
            .collect();
        let nt = (rnd() % 10) as usize;
        let tris: Vec<i32> = (0..nt * 3)
            .map(|_| (rnd() % (nv as u32 + 1)) as i32)
            .collect();
        let s = Settings {
            cs: [0.05f32, 0.3, 1.0][(rnd() % 3) as usize],
            ch: [0.05f32, 0.2, 2.0][(rnd() % 3) as usize],
            height: (rnd() % 20) as i32 - 2,
            climb: (rnd() % 10) as i32 - 2,
            radius: (rnd() % 8) as i32 - 2,
            partition: rnd(),
            nvp: (rnd() % 10) as i32,
        };
        let mut ws = Workspace::with_limits(Limits {
            max_bytes: 64 << 20,
            max_work: 50_000_000,
            ..Limits::UNLIMITED
        });
        let _ = build(&mut ws, &verts, &tris, &s);
    }
}

#[test]
fn convex_volume_work_counts_layered_spans_times_polygon_vertices() {
    use recast::{CompactCell, CompactSpan};
    let mut chf = CompactHeightfield {
        width: 1,
        height: 1,
        span_count: 8,
        cs: 1.,
        ch: 1.,
        cells: vec![CompactCell { index: 0, count: 8 }],
        spans: (0..8)
            .map(|y| CompactSpan {
                y,
                con: 0xffffff,
                ..CompactSpan::default()
            })
            .collect(),
        areas: vec![63; 8],
        ..CompactHeightfield::default()
    };
    let polygon = [0., 0., 0., 0., 0., 1., 1., 0., 1., 1., 0., 0.];
    // Validation: 1 cell + 4*8 span connections; polygon: 12 scalars;
    // rectangle: 1 cell; marking: 8 spans * 4 vertices.
    let exact = 33 + 12 + 1 + 32;
    let mut ws = Workspace::with_limits(Limits {
        max_work: exact - 1,
        ..Limits::default()
    });
    assert_eq!(
        recast::mark_convex_poly_area(&mut ws, &polygon, 0., 8., 3, &mut chf),
        Err(Error::LimitExceeded)
    );
    assert_eq!(chf.areas, [63; 8]); // fail before visiting this cell's spans
    ws.set_limits(Limits {
        max_work: exact,
        ..Limits::default()
    });
    recast::mark_convex_poly_area(&mut ws, &polygon, 0., 8., 3, &mut chf).unwrap();
    assert_eq!(ws.usage().work, exact);
    assert_eq!(chf.areas, [3; 8]);
    ws.set_limits(Limits {
        max_work: 0,
        ..Limits::default()
    });
    assert_eq!(
        recast::mark_convex_poly_area(&mut ws, &polygon, 0., 8., 3, &mut chf),
        Err(Error::LimitExceeded)
    );
}
