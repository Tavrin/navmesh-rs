#![no_main]
#![allow(clippy::field_reassign_with_default)]
//! Trusted-versus-validated bake identity, including errors.
//!
//! `bake::tiles` skips the per-stage compact-field validation for fields it built itself. This
//! target rebuilds every tile through the public stage functions (a fresh `Workspace` per stage,
//! so each stage validates) with the same cumulative per-tile budget, and requires the same tile
//! bytes, triangulation report and first error. Setup failures (no tile location) are skipped:
//! they run before any tile work. Detour's limit-aware tile creation is not public, so the
//! reference ends in the unlimited `create_nav_mesh_data`; when it reaches that point a bake error
//! of the limit class is accepted.
//!
//! Layout: `[0]` tile size `8 + b % 57`; `[1]` partition (`b % 3`), agent radius
//! (`(b >> 2) & 7` cells) and vertices per polygon (`3 + (b >> 5) % 4`); `[2]` bit 0 raw `f32`
//! vertices, bit 1 raw triangle indices, bits 2-3 cell size (0.3, 0.15, 0.1, 0.5); `[3]`
//! features: bit 0 switches the three span filters off, bit 1 gives triangle areas, bits 2-3
//! count the area volumes, bit 4 enables strict triangulation, bit 5 turns detail sampling off,
//! bit 6 sets tight per-tile limits from `[4]` and `[5]`, bit 7 moves the volume polygon to the
//! second vertex; `[4]`, `[5]` limit and volume parameters (`[4]` bit 0 also enables the
//! zero-sample-distance correction when detail sampling is off); `[6]` vertex count; then the vertices
//! (12 bytes each when raw, otherwise three bytes scaled to 0..16, 0..8, 0..16) and the
//! triangles, three bytes each (at most 128).
use libfuzzer_sys::fuzz_target;
use navmesh_rs::Error;
use navmesh_rs::bake::{self, AreaVolume, BakeConfig, Input, Partition};
use navmesh_rs::detour::{NavMeshCreateParams, create_nav_mesh_data};
use navmesh_rs::recast::{
    self, CompactHeightfield, Config, ContourSet, Heightfield, PolyMesh, PolyMeshDetail,
    TriangulationReport,
};
use navmesh_rs::{Limits, Usage, Workspace};

type Built = (Option<Vec<u8>>, TriangulationReport);

/// Cumulative per-tile budget, mirroring the baker's staged accounting.
struct Staged {
    limits: Limits,
    used: Usage,
}

impl Staged {
    fn remaining(&self) -> Limits {
        let mut l = self.limits;
        l.max_bytes = l.max_bytes.saturating_sub(self.used.bytes);
        l.max_work = l.max_work.saturating_sub(self.used.work);
        l
    }

    /// One public stage in a fresh, untrusted workspace.
    fn stage<T>(&mut self, f: impl FnOnce(&mut Workspace) -> Result<T, Error>) -> Result<T, Error> {
        let mut ws = Workspace::with_limits(self.remaining());
        let out = f(&mut ws);
        self.used.bytes = self.used.bytes.saturating_add(ws.usage().bytes);
        self.used.work = self.used.work.saturating_add(ws.usage().work);
        out
    }

    /// A charge the baker makes inside a stage through crate-private workspace calls.
    fn charge(&mut self, work: u64, bytes: usize) -> Result<(), Error> {
        let rem = self.remaining();
        self.used.work = self.used.work.saturating_add(work);
        self.used.bytes = self.used.bytes.saturating_add(bytes);
        if work > rem.max_work || bytes > rem.max_bytes {
            return Err(Error::LimitExceeded);
        }
        Ok(())
    }
}

/// `Err(_)` carries the reference error; `Ok((outcome, reached_create))`.
fn reference_tile(
    input: &Input<'_>,
    c: &BakeConfig,
    cfg: &Config,
    bucket: &[usize],
    volume_bucket: &[usize],
    x: i32,
    y: i32,
) -> (Result<Built, Error>, bool) {
    let mut reached = false;
    let r = (|| -> Result<Built, Error> {
        let mut t = Staged {
            limits: c.tile_limits,
            used: Usage::default(),
        };
        t.charge(bucket.len() as u64, bucket.len() * 13)?;
        let mut tris: Vec<i32> = Vec::new();
        let mut areas: Vec<u8> = Vec::new();
        for &i in bucket {
            tris.extend_from_slice(&input.tris[i * 3..i * 3 + 3]);
            areas.push(input.areas.map_or(0, |a| a[i]));
        }
        t.stage(|ws| {
            if input.areas.is_some() {
                recast::clear_unwalkable_triangles(
                    ws,
                    cfg.walkable_slope_angle,
                    input.verts,
                    &tris,
                    &mut areas,
                )
            } else {
                recast::mark_walkable_triangles(
                    ws,
                    cfg.walkable_slope_angle,
                    input.verts,
                    &tris,
                    &mut areas,
                )
            }
        })?;
        t.charge((cfg.width as u64) * (cfg.height as u64), 0)?;
        let mut hf = t.stage(|ws| {
            Heightfield::new(
                ws, cfg.width, cfg.height, cfg.bmin, cfg.bmax, cfg.cs, cfg.ch,
            )
        })?;
        if !tris.is_empty() {
            t.stage(|ws| {
                hf.rasterize_triangles(ws, input.verts, &tris, &areas, cfg.walkable_climb)
            })?;
        }
        if c.filter_low_hanging {
            t.stage(|ws| hf.filter_low_hanging_walkable_obstacles(ws, cfg.walkable_climb))?;
        }
        if c.filter_ledge {
            t.stage(|ws| hf.filter_ledge_spans(ws, cfg.walkable_height, cfg.walkable_climb))?;
        }
        if c.filter_low_height {
            t.stage(|ws| hf.filter_walkable_low_height_spans(ws, cfg.walkable_height))?;
        }
        let mut chf = t.stage(|ws| {
            CompactHeightfield::build(ws, cfg.walkable_height, cfg.walkable_climb, &hf)
        })?;
        drop(hf);
        t.stage(|ws| recast::erode_walkable_area(ws, cfg.walkable_radius, &mut chf))?;
        for &i in volume_bucket {
            let v = &input.volumes[i];
            t.stage(|ws| {
                recast::mark_convex_poly_area(ws, v.verts, v.hmin, v.hmax, v.area, &mut chf)
            })?;
        }
        match c.partition {
            Partition::Watershed => {
                t.stage(|ws| recast::build_distance_field(ws, &mut chf))?;
                t.stage(|ws| {
                    recast::build_regions(
                        ws,
                        &mut chf,
                        cfg.border_size,
                        cfg.min_region_area,
                        cfg.merge_region_area,
                    )
                })?;
            }
            Partition::Monotone => t.stage(|ws| {
                recast::build_regions_monotone(
                    ws,
                    &mut chf,
                    cfg.border_size,
                    cfg.min_region_area,
                    cfg.merge_region_area,
                )
            })?,
            Partition::Layers => t.stage(|ws| {
                recast::build_layer_regions(ws, &mut chf, cfg.border_size, cfg.min_region_area)
            })?,
            _ => return Err(Error::Failed),
        }
        let contours = t.stage(|ws| {
            ContourSet::build(
                ws,
                &chf,
                cfg.max_simplification_error,
                cfg.max_edge_len,
                recast::CONTOUR_TESS_WALL_EDGES,
            )
        })?;
        let (mut mesh, report) = t.stage(|ws| {
            PolyMesh::build_with_report(
                ws,
                &contours,
                cfg.max_verts_per_poly,
                c.strict_triangulation,
            )
        })?;
        t.charge(mesh.areas.len() as u64, 0)?;
        for (a, f) in mesh.areas.iter_mut().zip(&mut mesh.flags) {
            if *a == recast::WALKABLE_AREA {
                *a = 0;
            }
            *f = if *a == 0 || *a >= 16 {
                1
            } else {
                1 | (1 << *a)
            };
        }
        if cfg.detail_sample_dist > 0.0 {
            let samples_per_axis = cfg.width as f32 * cfg.cs / cfg.detail_sample_dist;
            if !samples_per_axis.is_finite() || samples_per_axis > (i32::MAX / 4) as f32 {
                return Err(Error::InvalidParameter("detail sample distance too small"));
            }
        }
        let detail = t.stage(|ws| {
            PolyMeshDetail::build_with_fixes(
                ws,
                &mesh,
                &chf,
                cfg.detail_sample_dist,
                cfg.detail_sample_max_error,
                c.fixes,
            )
        })?;
        if mesh.npolys == 0 {
            return Ok((None, report));
        }
        let mut params = NavMeshCreateParams::default();
        params.verts = &mesh.verts;
        params.vert_count = mesh.nverts;
        params.polys = &mesh.polys;
        params.poly_flags = &mesh.flags;
        params.poly_areas = &mesh.areas;
        params.poly_count = mesh.npolys;
        params.nvp = mesh.nvp;
        params.detail_meshes = &detail.meshes;
        params.detail_verts = &detail.verts;
        params.detail_verts_count =
            i32::try_from(detail.verts.len() / 3).map_err(|_| Error::SizeOverflow)?;
        params.detail_tris = &detail.tris;
        params.detail_tri_count =
            i32::try_from(detail.tris.len() / 4).map_err(|_| Error::SizeOverflow)?;
        params.tile_x = x;
        params.tile_y = y;
        params.bmin = mesh.bmin;
        params.bmax = mesh.bmax;
        params.walkable_height = c.walkable_height;
        params.walkable_radius = c.walkable_radius;
        params.walkable_climb = c.walkable_climb;
        params.cs = cfg.cs;
        params.ch = cfg.ch;
        params.build_bv_tree = true;
        let n = mesh.verts.len()
            + mesh.polys.len()
            + detail.verts.len()
            + detail.tris.len()
            + input.off_mesh.off_mesh_con_verts.len()
            + 1;
        let bound = n * 128;
        let polys = mesh.npolys as usize;
        let sort_work = polys * ((usize::BITS - polys.leading_zeros()) as usize).pow(2);
        t.charge((bound + sort_work) as u64, bound)?;
        reached = true;
        let data = create_nav_mesh_data(&params)?;
        Ok((Some(data), report))
    })();
    (r, reached)
}

fuzz_target!(|data: &[u8]| {
    if data.len() < 8 {
        return;
    }
    let (raw_verts, raw_tris) = (data[2] & 1 != 0, data[2] & 2 != 0);
    let stride = if raw_verts { 12 } else { 3 };
    let body = &data[7..];
    let nv = usize::from(data[6]).min(body.len() / stride);
    if nv < 3 {
        return;
    }
    let verts: Vec<f32> = if raw_verts {
        body[..nv * 12]
            .as_chunks::<4>()
            .0
            .iter()
            .map(|b| f32::from_bits(u32::from_le_bytes(*b)))
            .collect()
    } else {
        body[..nv * 3]
            .iter()
            .enumerate()
            .map(|(k, &b)| f32::from(b) / if k % 3 == 1 { 32.0 } else { 16.0 })
            .collect()
    };
    let nvi = i32::try_from(nv).unwrap();
    let tris: Vec<i32> = body[nv * stride..]
        .as_chunks::<3>()
        .0
        .iter()
        .take(128)
        .flat_map(|t| {
            t.iter().map(|&v| {
                if raw_tris {
                    i32::from(v) - 4
                } else {
                    i32::from(v) % nvi
                }
            })
        })
        .collect();
    if tris.is_empty() {
        return;
    }
    let features = data[3];
    let areas: Vec<u8> = tris
        .as_chunks::<3>()
        .0
        .iter()
        .map(|t| (t[0] & 63) as u8)
        .collect();
    let first = usize::from(features >> 7).min(nv.saturating_sub(4));
    let poly = &verts[first * 3..(first + 4).min(nv) * 3];
    let (hmin, hmax) = poly
        .as_chunks::<3>()
        .0
        .iter()
        .fold((f32::INFINITY, f32::NEG_INFINITY), |(lo, hi), p| {
            (lo.min(p[1]), hi.max(p[1]))
        });
    let volume = |area: u8| {
        let mut v = AreaVolume::default();
        v.verts = poly;
        v.hmin = hmin - 0.5;
        v.hmax = hmax + 0.5;
        v.area = area;
        v
    };
    let volumes = [volume(data[5] & 63), volume(3), volume(0)];
    let mut input = Input::new(&verts, &tris);
    if features & 2 != 0 {
        input.areas = Some(&areas);
    }
    input.volumes = &volumes[..usize::from((features >> 2) & 3)];

    let mut cfg = BakeConfig::default();
    cfg.recast.tile_size = 8 + i32::from(data[0] % 57);
    cfg.recast.walkable_radius = i32::from((data[1] >> 2) & 7);
    cfg.recast.max_verts_per_poly = 3 + i32::from((data[1] >> 5) % 4);
    cfg.recast.cs = [0.3, 0.15, 0.1, 0.5][usize::from((data[2] >> 2) & 3)];
    cfg.partition = match data[1] % 3 {
        0 => Partition::Watershed,
        1 => Partition::Monotone,
        _ => Partition::Layers,
    };
    if features & 1 != 0 {
        cfg.filter_low_hanging = false;
        cfg.filter_ledge = false;
        cfg.filter_low_height = false;
    }
    cfg.strict_triangulation = features & 0x10 != 0;
    if features & 0x20 != 0 {
        cfg.recast.detail_sample_dist = 0.0;
        if data[4] & 1 != 0 {
            cfg.fixes = navmesh_rs::UpstreamFixes::ZERO_SAMPLE_DIST;
        }
    }
    cfg.tile_limits.max_bytes = 8 << 20;
    cfg.tile_limits.max_work = 2_000_000;
    if features & 0x40 != 0 {
        // Spread the limits over the scale at which individual stages start to fail.
        cfg.tile_limits.max_bytes =
            (1 + usize::from(data[5] >> 4)) << (8 + usize::from(data[4] >> 4) % 14);
        cfg.tile_limits.max_work =
            (1 + u64::from(data[4] & 15)) << (4 + u64::from(data[5] & 15) % 18);
    }
    let setup = Limits {
        max_bytes: 8 << 20,
        max_work: 2_000_000,
        ..Limits::default()
    };

    let trusted = bake::tiles(&input, &cfg, &mut Workspace::with_limits(setup));
    let trusted = match trusted {
        Ok(b) => Ok(b.tiles),
        // Setup and aggregate failures happen before any tile stage runs.
        Err(e) if e.tile.is_none() => return,
        Err(e) => Err(e),
    };

    // Grid geometry, as the baker derives it for valid parameters.
    let base = cfg.recast;
    let (bmin, bmax) = recast::calc_bounds(&verts);
    let ts = base.tile_size;
    let gw = ((bmax[0] - bmin[0]) / base.cs + 0.5) as i32;
    let gh = ((bmax[2] - bmin[2]) / base.cs + 0.5) as i32;
    let (tw, th) = ((gw + ts - 1) / ts, (gh + ts - 1) / ts);
    assert!(tw > 0 && th > 0);
    let border = base.walkable_radius + 3;
    let dim = ts + border * 2;
    let tcs = ts as f32 * base.cs;
    let b = border as f32 * base.cs;
    let tri_bounds: Vec<([f32; 2], [f32; 2])> = tris
        .as_chunks::<3>()
        .0
        .iter()
        .map(|tri| {
            let mut lo = [f32::INFINITY; 2];
            let mut hi = [f32::NEG_INFINITY; 2];
            for &k in tri {
                let k = k as usize * 3;
                lo[0] = lo[0].min(verts[k]);
                hi[0] = hi[0].max(verts[k]);
                lo[1] = lo[1].min(verts[k + 2]);
                hi[1] = hi[1].max(verts[k + 2]);
            }
            (lo, hi)
        })
        .collect();
    let vol_bounds: Vec<([f32; 3], [f32; 3])> = input
        .volumes
        .iter()
        .map(|v| recast::calc_bounds(v.verts))
        .collect();

    let fail_at = match &trusted {
        Err(e) => e.tile.map(|[x, y]| (y * tw + x) as usize),
        Ok(t) => {
            assert_eq!(t.len(), (tw * th) as usize);
            None
        }
    };
    for y in 0..th {
        for x in 0..tw {
            let idx = (y * tw + x) as usize;
            let mut tcfg = base;
            tcfg.border_size = border;
            tcfg.width = dim;
            tcfg.height = dim;
            tcfg.bmin = [
                bmin[0] + x as f32 * tcs - b,
                bmin[1],
                bmin[2] + y as f32 * tcs - b,
            ];
            tcfg.bmax = [
                bmin[0] + (x + 1) as f32 * tcs + b,
                bmax[1],
                bmin[2] + (y + 1) as f32 * tcs + b,
            ];
            let bucket: Vec<usize> = (0..tri_bounds.len())
                .filter(|&i| {
                    let (lo, hi) = tri_bounds[i];
                    lo[0] <= tcfg.bmax[0]
                        && hi[0] >= tcfg.bmin[0]
                        && lo[1] <= tcfg.bmax[2]
                        && hi[1] >= tcfg.bmin[2]
                })
                .collect();
            let vbucket: Vec<usize> = (0..vol_bounds.len())
                .filter(|&i| {
                    let (lo, hi) = vol_bounds[i];
                    lo[0] <= tcfg.bmax[0]
                        && hi[0] >= tcfg.bmin[0]
                        && lo[2] <= tcfg.bmax[2]
                        && hi[2] >= tcfg.bmin[2]
                })
                .collect();
            let (reference, reached) = reference_tile(&input, &cfg, &tcfg, &bucket, &vbucket, x, y);
            match (&trusted, fail_at) {
                (Ok(tiles), _) => {
                    let got = &tiles[idx];
                    let (data, report) = reference.unwrap_or_else(|e| {
                        panic!("tile {x},{y}: trusted bake succeeded, validated path failed: {e:?}")
                    });
                    assert_eq!(got.data, data, "tile {x},{y} data");
                    assert_eq!(got.triangulation, report, "tile {x},{y} triangulation");
                }
                (Err(e), Some(at)) if idx == at => {
                    match reference {
                        Err(r) if !reached => assert_eq!(r, e.error, "tile {x},{y} error"),
                        // The reference passed every public stage; only Detour's limit-aware
                        // creation (or an error from creating the tile) is left to explain it.
                        _ => assert!(
                            matches!(
                                e.error,
                                Error::LimitExceeded | Error::FormatLimit(_) | Error::SizeOverflow
                            ) || reference_err(&e.error),
                            "tile {x},{y}: trusted failed with {:?} past the reference stages",
                            e.error
                        ),
                    }
                    return;
                }
                (Err(e), Some(_)) => {
                    assert!(
                        reference.is_ok(),
                        "tile {x},{y}: validated path failed ({:?}) before trusted failure {:?}",
                        reference.err(),
                        e
                    );
                }
                (Err(_), None) => return,
            }
        }
    }
});

/// Errors `create_nav_mesh_data` itself reports for invalid assembled parameters.
fn reference_err(e: &Error) -> bool {
    matches!(e, Error::InvalidParameter(_) | Error::InvalidInput(_))
}
