//! Shared code of the fuzz targets and the seed writer.

use navmesh_rs::detour::{self, DefaultFilter, NavMesh, NavMeshCreateParams, NavMeshQuery};
use navmesh_rs::recast::{
    self, CompactHeightfield, ContourSet, Heightfield, HeightfieldLayerSet, PolyMesh,
    PolyMeshDetail,
};
use navmesh_rs::{Error, Limits, Workspace};

/// Per-call limits of the Recast target, so that a mutated cell size or a far
/// vertex ends in `LimitExceeded` instead of a multi-gigabyte grid. Sized so one
/// input stays well under libFuzzer's 10 s timeout in the instrumented build.
pub const RECAST_LIMITS: Limits = Limits {
    max_bytes: 32 << 20,
    max_work: 2_000_000,
    ..Limits::UNLIMITED
};

/// Runs a few queries between points derived from the first tile's bounds.
pub fn exercise_queries(nav: &NavMesh) {
    let Some(t) = (0..nav.max_tiles() as usize).find_map(|i| nav.tile(i)) else {
        return;
    };
    let (bmin, bmax) = (t.header.bmin, t.header.bmax);
    let c = [0, 1, 2].map(|k| (bmin[k] + bmax[k]) * 0.5);
    let e = [0, 1, 2].map(|k| (bmax[k] - bmin[k]).abs() * 0.5 + 1.0);
    if !c.iter().chain(e.iter()).all(|v| v.is_finite()) {
        return;
    }
    let Ok(mut q) = NavMeshQuery::new(nav, 256) else {
        return;
    };
    let f = DefaultFilter::default();
    let mut polys = Vec::new();
    let _ = q.query_polygons(&c, &e, &f, &mut polys, 64);
    let _ = q.query_polygons(&c, &[1e30; 3], &f, &mut polys, 0);
    let small = [e[0] * 0.25, e[1], e[2] * 0.25];
    let a = q.find_nearest_poly(&bmin.map(|v| v + 0.5), &small, &f);
    let b = q.find_nearest_poly(&bmax.map(|v| v - 0.5), &small, &f);
    if let (Ok((ra, pa, _)), Ok((rb, pb, _))) = (a, b)
        && ra != 0
        && rb != 0
    {
        let mut path = Vec::new();
        let _ = q.find_path(ra, rb, &pa, &pb, &f, &mut path, 64);
        let mut corners = Vec::new();
        let _ = q.find_straight_path(&pa, &pb, &path, &mut corners, 32, 0);
        let _ = q.raycast(ra, &pa, &pb, &f, 64);
        let _ = q.find_distance_to_wall(ra, &pa, 4.0, &f);
    }
}

/// Input of the Recast target.
///
/// Layout: 16 setting bytes, a little-endian `u16` vertex count `n`, `n` vertices
/// as three little-endian `f32` each, then the triangles as little-endian `u16`
/// index triples (indices are not checked here; the library must reject bad ones).
///
/// Setting bytes: 0 flags (bits 0-1 partition: watershed, monotone, layers;
/// bit 2 span filters; bit 3 median filter; bit 4 box area; bit 5 cylinder area;
/// bit 6 heightfield layers; bit 7 BV tree), 1 cell size `(b + 1) / 100`, 2 cell
/// height `(b + 1) / 100`, 3 walkable slope in degrees, 4 walkable height, 5 climb,
/// 6 radius (cells), 7 min region area, 8 merge region area, 9 max edge length,
/// 10 max simplification error `b / 10`, 11 vertices per polygon, 12 detail sample
/// distance `b / 10`, 13 detail max error `b / 10`, 14 border size, 15 contour
/// tessellation flags.
#[derive(Clone, Debug)]
pub struct RecastInput {
    /// Setting bytes.
    pub settings: [u8; 16],
    /// Vertices, three per point.
    pub verts: Vec<f32>,
    /// Triangle indices, three per triangle.
    pub tris: Vec<i32>,
}

/// Upstream demo settings (cell 0.3, height 0.2, agent 2.0 / 0.9 / 0.6, 45 degrees)
/// with the given partition.
pub fn demo_settings(partition: u8) -> [u8; 16] {
    [
        (partition & 3) | 0b1000_0100,
        29,
        19,
        45,
        10,
        4,
        2,
        8,
        20,
        40,
        13,
        6,
        18,
        2,
        0,
        recast::CONTOUR_TESS_WALL_EDGES as u8,
    ]
}

impl RecastInput {
    /// Decodes the layout above; `None` when the input is too short.
    pub fn decode(data: &[u8]) -> Option<RecastInput> {
        let settings: [u8; 16] = data.get(..16)?.try_into().ok()?;
        let n = usize::from(u16::from_le_bytes(data.get(16..18)?.try_into().ok()?));
        let vbytes = data.get(18..18 + n * 12)?;
        let verts = vbytes
            .as_chunks::<4>()
            .0
            .iter()
            .map(|b| f32::from_le_bytes(*b))
            .collect();
        let tris = data[18 + n * 12..]
            .as_chunks::<6>()
            .0
            .iter()
            .flat_map(|t| [0, 2, 4].map(|k| i32::from(u16::from_le_bytes([t[k], t[k + 1]]))))
            .collect();
        Some(RecastInput {
            settings,
            verts,
            tris,
        })
    }

    /// Encodes the layout above; `None` when there are more than 65,535 vertices or
    /// an index does not fit `u16`.
    pub fn encode(&self) -> Option<Vec<u8>> {
        let n = u16::try_from(self.verts.len() / 3).ok()?;
        let mut out = self.settings.to_vec();
        out.extend_from_slice(&n.to_le_bytes());
        for v in &self.verts[..usize::from(n) * 3] {
            out.extend_from_slice(&v.to_le_bytes());
        }
        for &i in &self.tris {
            out.extend_from_slice(&u16::try_from(i).ok()?.to_le_bytes());
        }
        Some(out)
    }
}

/// How far [`run_recast`] got.
#[derive(Clone, Copy, Debug, PartialEq, Eq, PartialOrd, Ord)]
pub enum Reached {
    /// Contours, polygon mesh and detail mesh built.
    DetailMesh,
    /// Tile bytes built and loaded into a navmesh.
    NavMesh,
}

/// Runs the Recast pipeline configured by `input`, builds a tile from its output,
/// loads it and runs queries on it.
pub fn run_recast(input: &RecastInput) -> Result<Reached, Error> {
    let s = &input.settings;
    let flags = s[0];
    let cs = (f32::from(s[1]) + 1.0) / 100.0;
    let ch = (f32::from(s[2]) + 1.0) / 100.0;
    let (height, climb, radius) = (i32::from(s[4]), i32::from(s[5]), i32::from(s[6]));
    let border = i32::from(s[14]);
    let mut ws = Workspace::with_limits(RECAST_LIMITS);
    let (verts, tris) = (&input.verts[..], &input.tris[..]);

    let (bmin, bmax) = recast::calc_bounds(verts);
    let (w, h) = recast::calc_grid_size(&bmin, &bmax, cs);
    let mut hf = Heightfield::new(&mut ws, w, h, bmin, bmax, cs, ch)?;
    let mut areas = vec![0u8; tris.len() / 3];
    recast::mark_walkable_triangles(&mut ws, f32::from(s[3]), verts, tris, &mut areas)?;
    hf.rasterize_triangles(&mut ws, verts, tris, &areas, climb)?;
    if flags & 4 != 0 {
        hf.filter_low_hanging_walkable_obstacles(&mut ws, climb)?;
        hf.filter_ledge_spans(&mut ws, height, climb)?;
        hf.filter_walkable_low_height_spans(&mut ws, height)?;
    }
    let mut chf = CompactHeightfield::build(&mut ws, height, climb, &hf)?;
    recast::erode_walkable_area(&mut ws, radius, &mut chf)?;
    if flags & 8 != 0 {
        recast::median_filter_walkable_area(&mut ws, &mut chf)?;
    }
    let mid = [0, 1, 2].map(|k| (bmin[k] + bmax[k]) * 0.5);
    let quarter = [0, 1, 2].map(|k| (bmax[k] - bmin[k]) * 0.25);
    if flags & 16 != 0 {
        let lo = [0, 1, 2].map(|k| mid[k] - quarter[k]);
        let hi = [0, 1, 2].map(|k| mid[k] + quarter[k]);
        recast::mark_box_area(&mut ws, lo, hi, 5, &mut chf)?;
    }
    if flags & 32 != 0 {
        let base = [mid[0], bmin[1], mid[2]];
        recast::mark_cylinder_area(&mut ws, base, quarter[0], bmax[1] - bmin[1], 7, &mut chf)?;
    }
    if flags & 64 != 0 {
        let _ = HeightfieldLayerSet::build(&mut ws, &chf, border, height);
    }
    match flags & 3 {
        0 => {
            recast::build_distance_field(&mut ws, &mut chf)?;
            recast::build_regions(&mut ws, &mut chf, border, i32::from(s[7]), i32::from(s[8]))?;
        }
        1 => recast::build_regions_monotone(
            &mut ws,
            &mut chf,
            border,
            i32::from(s[7]),
            i32::from(s[8]),
        )?,
        _ => recast::build_layer_regions(&mut ws, &mut chf, border, i32::from(s[7]))?,
    }
    let max_error = f32::from(s[10]) / 10.0;
    let cset = ContourSet::build(
        &mut ws,
        &chf,
        max_error,
        i32::from(s[9]),
        i32::from(s[15] & 3),
    )?;
    let pmesh = PolyMesh::build(&mut ws, &cset, i32::from(s[11]))?;
    let sample_dist = f32::from(s[12]) / 10.0;
    let sample_err = f32::from(s[13]) / 10.0;
    let dmesh = PolyMeshDetail::build(&mut ws, &pmesh, &chf, sample_dist, sample_err)?;

    let poly_flags = vec![1u16; usize::try_from(pmesh.npolys).unwrap_or(0)];
    let tile = detour::create_nav_mesh_data(&NavMeshCreateParams {
        verts: &pmesh.verts,
        vert_count: pmesh.nverts,
        polys: &pmesh.polys,
        poly_flags: &poly_flags,
        poly_areas: &pmesh.areas,
        poly_count: pmesh.npolys,
        nvp: pmesh.nvp,
        detail_meshes: &dmesh.meshes,
        detail_verts: &dmesh.verts,
        detail_verts_count: (dmesh.verts.len() / 3) as i32,
        detail_tris: &dmesh.tris,
        detail_tri_count: (dmesh.tris.len() / 4) as i32,
        bmin: pmesh.bmin,
        bmax: pmesh.bmax,
        walkable_height: height as f32 * ch,
        walkable_radius: radius as f32 * cs,
        walkable_climb: climb as f32 * ch,
        cs,
        ch,
        build_bv_tree: flags & 128 != 0,
        ..Default::default()
    });
    let Ok(tile) = tile else {
        return Ok(Reached::DetailMesh);
    };
    let (nav, _) = NavMesh::new_single(tile)?;
    exercise_queries(&nav);
    Ok(Reached::NavMesh)
}
