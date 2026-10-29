//! `nav-port`: drives the Rust port over a case file the way the reference driver
//! (`reference/harness/nav_oracle.cpp`) drives upstream, and writes the same
//! records, so `nav-parity diff --recast` can compare the two.
//!
//! ```text
//! nav-port bake  <case-file> --records FILE
//! nav-port bench <case-file> [--repeat N]
//! ```
//!
//! Only the Recast stages are produced (geometry through the detail mesh, and
//! the heightfield layers in tile-cache mode). Mesh paths that are not absolute
//! resolve against `$NAV_CORPUS_DIR`.

use std::path::Path;
use std::process::ExitCode;
use std::time::Instant;

use navmesh_parity::encode::{Payload, RecordFileWriter};
use navmesh_rs::detour::{
    self, DefaultFilter, NavMesh, NavMeshParams, NavMeshQuery, PolyRef, QueryFilter, RaycastHit,
    Status,
};
use navmesh_rs::recast::{self, CompactHeightfield, Config, ContourSet, Heightfield, PolyMesh};
use navmesh_rs::{Error, Workspace};

/// The oracle's second filter: excludes flag 0x10 (area 4), areas 1..7 cost
/// `1 + 1.5 * area`.
fn cost_filter() -> DefaultFilter {
    let mut f = DefaultFilter {
        exclude_flags: 0x10,
        ..DefaultFilter::default()
    };
    for a in 1..8 {
        f.area_cost[a] = 1.0 + 1.5 * a as f32;
    }
    f
}

const UPSTREAM: &str = "9f4ce64458dfae86e1239c525ddc219c4e9e06f1";

struct AreaVolume {
    area: u8,
    hmin: f32,
    hmax: f32,
    verts: Vec<f32>,
}

struct Case {
    mesh: String,
    mode: String,
    partition: String,
    cell_size: f32,
    cell_height: f32,
    agent_height: f32,
    agent_radius: f32,
    agent_max_climb: f32,
    agent_max_slope: f32,
    region_min_size: f32,
    region_merge_size: f32,
    edge_max_len: f32,
    edge_max_error: f32,
    verts_per_poly: i32,
    detail_sample_dist: f32,
    detail_sample_max_error: f32,
    tile_size: i32,
    filter_low_hanging: bool,
    filter_ledge: bool,
    filter_walkable_low_height: bool,
    area_volumes: Vec<AreaVolume>,
    queries: i32,
    seed: u32,
    /// x0 y0 z0 x1 y1 z1 radius bidir, repeated.
    off_mesh: Vec<f32>,
}

/// The longest prefix of `s` that C's `strtof` would consume, parsed (0 when none).
fn strtof(s: &str) -> (f32, usize) {
    let b = s.as_bytes();
    let mut i = 0;
    while i < b.len() && (b[i] == b' ' || b[i] == b'\t' || b[i] == b'\n' || b[i] == b'\r') {
        i += 1;
    }
    let start = i;
    if i < b.len() && (b[i] == b'+' || b[i] == b'-') {
        i += 1;
    }
    let digits_start = i;
    while i < b.len() && b[i].is_ascii_digit() {
        i += 1;
    }
    if i < b.len() && b[i] == b'.' {
        i += 1;
        while i < b.len() && b[i].is_ascii_digit() {
            i += 1;
        }
    }
    if i == digits_start || (i == digits_start + 1 && b[digits_start] == b'.') {
        return (0.0, 0);
    }
    if i < b.len() && (b[i] == b'e' || b[i] == b'E') {
        let mut j = i + 1;
        if j < b.len() && (b[j] == b'+' || b[j] == b'-') {
            j += 1;
        }
        if j < b.len() && b[j].is_ascii_digit() {
            while j < b.len() && b[j].is_ascii_digit() {
                j += 1;
            }
            i = j;
        }
    }
    (s[start..i].parse().unwrap_or(0.0), i)
}

/// C's `atoi`.
fn atoi(s: &str) -> i32 {
    let t = s.trim_start_matches([' ', '\t']);
    let b = t.as_bytes();
    let (neg, mut i) = match b.first() {
        Some(b'-') => (true, 1),
        Some(b'+') => (false, 1),
        _ => (false, 0),
    };
    let mut v: i64 = 0;
    while i < b.len() && b[i].is_ascii_digit() {
        v = (v * 10 + i64::from(b[i] - b'0')).min(i64::from(u32::MAX));
        i += 1;
    }
    (if neg { -v } else { v }) as i32
}

fn floats(s: &str) -> Vec<f32> {
    let mut out = Vec::new();
    let mut rest = s;
    loop {
        let (v, n) = strtof(rest);
        if n == 0 {
            break;
        }
        out.push(v);
        rest = &rest[n..];
    }
    out
}

fn parse_case(path: &Path) -> Result<Case, String> {
    let text = std::fs::read_to_string(path)
        .map_err(|e| format!("cannot open case {}: {e}", path.display()))?;
    let mut c = Case {
        mesh: String::new(),
        mode: "solo".into(),
        partition: "watershed".into(),
        cell_size: 0.3,
        cell_height: 0.2,
        agent_height: 2.0,
        agent_radius: 0.6,
        agent_max_climb: 0.9,
        agent_max_slope: 45.0,
        region_min_size: 8.0,
        region_merge_size: 20.0,
        edge_max_len: 12.0,
        edge_max_error: 1.3,
        verts_per_poly: 6,
        detail_sample_dist: 6.0,
        detail_sample_max_error: 1.0,
        tile_size: 32,
        filter_low_hanging: true,
        filter_ledge: true,
        filter_walkable_low_height: true,
        queries: 64,
        seed: 0x9e37_79b9,
        off_mesh: Vec::new(),
        area_volumes: Vec::new(),
    };
    for (n, line) in text.lines().enumerate() {
        let line = line.split('#').next().unwrap_or("");
        let Some((key, val)) = line.split_once('=') else {
            continue;
        };
        let key = key.split_whitespace().next().unwrap_or("");
        if key.is_empty() {
            continue;
        }
        let val = val
            .trim_start_matches([' ', '\t'])
            .trim_end_matches([' ', '\r', '\n']);
        let f = || strtof(val).0;
        match key {
            "mesh" => {
                c.mesh = if val.starts_with('/') {
                    val.to_string()
                } else {
                    let dir = std::env::var("NAV_CORPUS_DIR")
                        .map_err(|_| "NAV_CORPUS_DIR is not set".to_string())?;
                    format!("{dir}/{val}")
                }
            }
            "mode" => c.mode = val.into(),
            "partition" => c.partition = val.into(),
            "cell_size" => c.cell_size = f(),
            "cell_height" => c.cell_height = f(),
            "agent_height" => c.agent_height = f(),
            "agent_radius" => c.agent_radius = f(),
            "agent_max_climb" => c.agent_max_climb = f(),
            "agent_max_slope" => c.agent_max_slope = f(),
            "region_min_size" => c.region_min_size = f(),
            "region_merge_size" => c.region_merge_size = f(),
            "edge_max_len" => c.edge_max_len = f(),
            "edge_max_error" => c.edge_max_error = f(),
            "verts_per_poly" => c.verts_per_poly = atoi(val),
            "detail_sample_dist" => c.detail_sample_dist = f(),
            "detail_sample_max_error" => c.detail_sample_max_error = f(),
            "tile_size" => c.tile_size = atoi(val),
            "filter_low_hanging" => c.filter_low_hanging = atoi(val) != 0,
            "filter_ledge" => c.filter_ledge = atoi(val) != 0,
            "filter_walkable_low_height" => c.filter_walkable_low_height = atoi(val) != 0,
            "queries" => c.queries = atoi(val),
            "seed" => {
                let v = val.trim();
                c.seed = if let Some(h) = v.strip_prefix("0x") {
                    u32::from_str_radix(h, 16).unwrap_or(0)
                } else {
                    v.parse().unwrap_or(0)
                };
            }
            "off_mesh" => {
                let x = floats(val);
                if x.len() < 8 {
                    return Err(format!(
                        "{}:{}: off_mesh needs 8 numbers",
                        path.display(),
                        n + 1
                    ));
                }
                c.off_mesh.extend_from_slice(&x[..8]);
            }
            // Crowd and tile-cache keys: not used by the port yet.
            "crowd_agents" | "crowd_ticks" | "tc_obstacle" => {}
            "area_volume" => {
                let x = floats(val);
                if x.len() < 9 || !(x.len() - 3).is_multiple_of(2) || x[0] < 1.0 || x[0] > 62.0 {
                    return Err(format!("{}:{}: bad area_volume", path.display(), n + 1));
                }
                let mut verts = Vec::new();
                for p in x[3..].as_chunks::<2>().0 {
                    verts.extend_from_slice(&[p[0], 0.0, p[1]]);
                }
                c.area_volumes.push(AreaVolume {
                    area: x[0] as u8,
                    hmin: x[1],
                    hmax: x[2],
                    verts,
                });
            }
            _ => return Err(format!("{}:{}: unknown key '{key}'", path.display(), n + 1)),
        }
    }
    if c.mesh.is_empty() {
        return Err("case has no mesh".into());
    }
    Ok(c)
}

struct Geometry {
    verts: Vec<f32>,
    tris: Vec<i32>,
    bmin: [f32; 3],
    bmax: [f32; 3],
}

/// Same rules as the reference driver: rows read in 511-byte chunks (`fgets`
/// into a 512-byte buffer), `v` rows, `f` rows fan triangulated, negative
/// indices relative, out-of-range faces dropped.
fn load_obj(path: &str) -> Result<Geometry, String> {
    let bytes = std::fs::read(path).map_err(|e| format!("cannot open mesh {path}: {e}"))?;
    let mut verts = Vec::new();
    let mut tris = Vec::new();
    let mut rows = Vec::new();
    for line in bytes.split_inclusive(|&b| b == b'\n') {
        for chunk in line.chunks(511) {
            rows.push(chunk);
        }
    }
    for row in rows {
        let row = String::from_utf8_lossy(row);
        let b = row.as_bytes();
        if b.len() < 2 || (b[1] != b' ' && b[1] != b'\t') {
            continue;
        }
        if b[0] == b'v' {
            let (x, n1) = strtof(&row[1..]);
            let (y, n2) = strtof(&row[1 + n1..]);
            let (z, n3) = strtof(&row[1 + n1 + n2..]);
            if n1 > 0 && n2 > 0 && n3 > 0 {
                verts.extend_from_slice(&[x, y, z]);
            }
        } else if b[0] == b'f' {
            let vc = (verts.len() / 3) as i32;
            let mut face = Vec::new();
            let mut p = 1;
            while p < b.len() && face.len() < 32 {
                while p < b.len() && (b[p] == b' ' || b[p] == b'\t') {
                    p += 1;
                }
                if p >= b.len() || b[p] == b'\n' || b[p] == b'\r' {
                    break;
                }
                let idx = atoi(&row[p..]);
                face.push(if idx < 0 { idx + vc } else { idx - 1 });
                while p < b.len() && b[p] != b' ' && b[p] != b'\t' {
                    p += 1;
                }
            }
            for i in 2..face.len() {
                let (a, bb, cc) = (face[0], face[i - 1], face[i]);
                if [a, bb, cc].iter().any(|&v| v < 0 || v >= vc) {
                    continue;
                }
                tris.extend_from_slice(&[a, bb, cc]);
            }
        }
    }
    if verts.is_empty() || tris.is_empty() {
        return Err(format!("empty mesh {path}"));
    }
    let (bmin, bmax) = recast::calc_bounds(&verts);
    Ok(Geometry {
        verts,
        tris,
        bmin,
        bmax,
    })
}

fn make_config(c: &Case) -> Config {
    let sqr = |v: f32| v * v;
    Config {
        cs: c.cell_size,
        ch: c.cell_height,
        walkable_slope_angle: c.agent_max_slope,
        walkable_height: (c.agent_height / c.cell_height).ceil() as i32,
        walkable_climb: (c.agent_max_climb / c.cell_height).floor() as i32,
        walkable_radius: (c.agent_radius / c.cell_size).ceil() as i32,
        max_edge_len: (c.edge_max_len / c.cell_size) as i32,
        max_simplification_error: c.edge_max_error,
        min_region_area: sqr(c.region_min_size) as i32,
        merge_region_area: sqr(c.region_merge_size) as i32,
        max_verts_per_poly: c.verts_per_poly,
        detail_sample_dist: if c.detail_sample_dist < 0.9 {
            0.0
        } else {
            c.cell_size * c.detail_sample_dist
        },
        detail_sample_max_error: c.cell_height * c.detail_sample_max_error,
        ..Config::default()
    }
}

fn tile_config(base: &Config, g: &Geometry, ts: i32, x: i32, y: i32) -> Config {
    let tcs = ts as f32 * base.cs;
    let mut cfg = *base;
    cfg.tile_size = ts;
    cfg.border_size = cfg.walkable_radius + 3;
    cfg.width = ts + cfg.border_size * 2;
    cfg.height = ts + cfg.border_size * 2;
    cfg.bmin = [
        g.bmin[0] + x as f32 * tcs,
        g.bmin[1],
        g.bmin[2] + y as f32 * tcs,
    ];
    cfg.bmax = [
        g.bmin[0] + (x + 1) as f32 * tcs,
        g.bmax[1],
        g.bmin[2] + (y + 1) as f32 * tcs,
    ];
    let b = cfg.border_size as f32 * cfg.cs;
    cfg.bmin[0] -= b;
    cfg.bmin[2] -= b;
    cfg.bmax[0] += b;
    cfg.bmax[2] += b;
    cfg
}

fn bucket_triangles(g: &Geometry, base: &Config, ts: i32, tw: i32, th: i32) -> Vec<Vec<i32>> {
    let tcs = ts as f32 * base.cs;
    let mut buckets = vec![Vec::new(); (tw * th) as usize];
    let margin = (base.walkable_radius + 3) as f32 * base.cs + tcs;
    let fmin = |a: f32, b: f32| if a < b { a } else { b };
    let fmax = |a: f32, b: f32| if a > b { a } else { b };
    for t in g.tris.as_chunks::<3>().0 {
        let mut tmin = [f32::INFINITY; 2];
        let mut tmax = [f32::NEG_INFINITY; 2];
        for &k in t {
            let v = &g.verts[k as usize * 3..];
            tmin[0] = fmin(tmin[0], v[0]);
            tmax[0] = fmax(tmax[0], v[0]);
            tmin[1] = fmin(tmin[1], v[2]);
            tmax[1] = fmax(tmax[1], v[2]);
        }
        let cl = |v: f32, hi: i32| (v.floor() as i32).clamp(0, hi - 1);
        let x0 = cl((tmin[0] - g.bmin[0] - margin) / tcs, tw);
        let x1 = cl((tmax[0] - g.bmin[0] + margin) / tcs, tw);
        let y0 = cl((tmin[1] - g.bmin[2] - margin) / tcs, th);
        let y1 = cl((tmax[1] - g.bmin[2] + margin) / tcs, th);
        for y in y0..=y1 {
            for x in x0..=x1 {
                let cfg = tile_config(base, g, ts, x, y);
                if tmin[0] > cfg.bmax[0]
                    || tmax[0] < cfg.bmin[0]
                    || tmin[1] > cfg.bmax[2]
                    || tmax[1] < cfg.bmin[2]
                {
                    continue;
                }
                buckets[(y * tw + x) as usize].extend_from_slice(t);
            }
        }
    }
    buckets
}

/// Collects records when a file is being written; a no-op sink for benches.
struct Sink {
    writer: Option<RecordFileWriter>,
}

impl Sink {
    fn record(&mut self, name: &str, f: impl FnOnce(&mut Payload)) {
        if let Some(w) = self.writer.as_mut() {
            let mut p = Payload::new();
            f(&mut p);
            w.record(name, p.as_bytes());
        }
    }
}

fn stream_config(p: &mut Payload, c: &Config) {
    p.i32(c.width)
        .i32(c.height)
        .i32(c.tile_size)
        .i32(c.border_size);
    p.f32(c.cs).f32(c.ch).v3(c.bmin).v3(c.bmax);
    p.f32(c.walkable_slope_angle)
        .i32(c.walkable_height)
        .i32(c.walkable_climb)
        .i32(c.walkable_radius);
    p.i32(c.max_edge_len)
        .f32(c.max_simplification_error)
        .i32(c.min_region_area)
        .i32(c.merge_region_area);
    p.i32(c.max_verts_per_poly)
        .f32(c.detail_sample_dist)
        .f32(c.detail_sample_max_error);
}

fn stream_heightfield(p: &mut Payload, hf: &Heightfield) {
    p.i32(hf.width)
        .i32(hf.height)
        .v3(hf.bmin)
        .v3(hf.bmax)
        .f32(hf.cs)
        .f32(hf.ch);
    for i in 0..(hf.width as usize * hf.height as usize) {
        p.u32(hf.column(i).count() as u32);
        for s in hf.column(i) {
            p.u16(s.smin).u16(s.smax).u8(s.area);
        }
    }
}

fn stream_compact(p: &mut Payload, chf: &CompactHeightfield, with_dist: bool, with_regions: bool) {
    p.i32(chf.width).i32(chf.height).i32(chf.span_count);
    p.i32(chf.walkable_height)
        .i32(chf.walkable_climb)
        .i32(chf.border_size);
    p.v3(chf.bmin).v3(chf.bmax).f32(chf.cs).f32(chf.ch);
    for c in &chf.cells {
        p.u32(c.index).u8(c.count);
    }
    for (i, s) in chf.spans.iter().enumerate() {
        p.u16(s.y).u32(s.con).u8(s.h).u8(chf.areas[i]);
        if with_regions {
            p.u16(s.reg);
        }
        if with_dist {
            p.u16(chf.dist.get(i).copied().unwrap_or(0));
        }
    }
    if with_dist {
        p.u16(chf.max_distance);
    }
    if with_regions {
        p.u16(chf.max_regions);
    }
}

fn stream_contours(p: &mut Payload, cs: &ContourSet) {
    p.i32(cs.conts.len() as i32)
        .v3(cs.bmin)
        .v3(cs.bmax)
        .f32(cs.cs)
        .f32(cs.ch);
    p.i32(cs.width)
        .i32(cs.height)
        .i32(cs.border_size)
        .f32(cs.max_error);
    for c in &cs.conts {
        p.u16(c.reg).u8(c.area);
        p.i32((c.verts.len() / 4) as i32);
        for &v in &c.verts {
            p.i32(v);
        }
        p.i32((c.rverts.len() / 4) as i32);
        for &v in &c.rverts {
            p.i32(v);
        }
    }
}

fn stream_poly_mesh(p: &mut Payload, m: &PolyMesh) {
    p.i32(m.nverts)
        .i32(m.npolys)
        .i32(m.nvp)
        .v3(m.bmin)
        .v3(m.bmax);
    p.f32(m.cs)
        .f32(m.ch)
        .i32(m.border_size)
        .f32(m.max_edge_error);
    for &v in &m.verts[..m.nverts as usize * 3] {
        p.u16(v);
    }
    let nvp = m.nvp as usize;
    for i in 0..m.npolys as usize {
        for &v in &m.polys[i * nvp * 2..(i + 1) * nvp * 2] {
            p.u16(v);
        }
        p.u16(m.regs[i]).u16(m.flags[i]).u8(m.areas[i]);
    }
}

fn poly_flags_for_area(area: u8) -> u16 {
    if area == 0 || area >= 16 {
        1
    } else {
        1 | (1 << area)
    }
}

/// One Recast build over a bounding box, as `buildOne` in the reference driver.
#[allow(clippy::too_many_arguments)]
fn build_one(
    ws: &mut Workspace,
    c: &Case,
    g: &Geometry,
    cfg: &Config,
    tris: &[i32],
    sink: &mut Sink,
    prefix: &str,
    tile: (i32, i32),
) -> Result<Option<Vec<u8>>, Error> {
    let tilecache = c.mode == "tilecache";
    sink.record(&format!("{prefix}config"), |p| stream_config(p, cfg));
    let mut hf = Heightfield::new(
        ws, cfg.width, cfg.height, cfg.bmin, cfg.bmax, cfg.cs, cfg.ch,
    )?;
    let mut areas = vec![0u8; tris.len() / 3];
    recast::mark_walkable_triangles(ws, cfg.walkable_slope_angle, &g.verts, tris, &mut areas)?;
    sink.record(&format!("{prefix}tri_areas"), |p| {
        p.u32(areas.len() as u32);
        for &a in &areas {
            p.u8(a);
        }
    });
    if !tris.is_empty() {
        hf.rasterize_triangles(ws, &g.verts, tris, &areas, cfg.walkable_climb)?;
    }
    sink.record(&format!("{prefix}heightfield_initial"), |p| {
        stream_heightfield(p, &hf)
    });
    if c.filter_low_hanging {
        hf.filter_low_hanging_walkable_obstacles(ws, cfg.walkable_climb)?;
    }
    if c.filter_ledge {
        hf.filter_ledge_spans(ws, cfg.walkable_height, cfg.walkable_climb)?;
    }
    if c.filter_walkable_low_height {
        hf.filter_walkable_low_height_spans(ws, cfg.walkable_height)?;
    }
    sink.record(&format!("{prefix}heightfield_filtered"), |p| {
        stream_heightfield(p, &hf)
    });
    let mut chf = CompactHeightfield::build(ws, cfg.walkable_height, cfg.walkable_climb, &hf)?;
    drop(hf);
    sink.record(&format!("{prefix}compact_initial"), |p| {
        stream_compact(p, &chf, false, false)
    });
    recast::erode_walkable_area(ws, cfg.walkable_radius, &mut chf)?;
    sink.record(&format!("{prefix}compact_eroded"), |p| {
        stream_compact(p, &chf, false, false)
    });
    for v in &c.area_volumes {
        recast::mark_convex_poly_area(ws, &v.verts, v.hmin, v.hmax, v.area, &mut chf)?;
    }
    if !c.area_volumes.is_empty() {
        sink.record(&format!("{prefix}compact_marked"), |p| {
            stream_compact(p, &chf, false, false)
        });
    }
    if tilecache {
        let lset =
            recast::HeightfieldLayerSet::build(ws, &chf, cfg.border_size, cfg.walkable_height)?;
        sink.record(&format!("{prefix}heightfield_layers"), |p| {
            p.i32(lset.layers.len() as i32);
            for l in &lset.layers {
                p.v3(l.bmin).v3(l.bmax).f32(l.cs).f32(l.ch);
                p.i32(l.width)
                    .i32(l.height)
                    .i32(l.minx)
                    .i32(l.maxx)
                    .i32(l.miny)
                    .i32(l.maxy);
                p.i32(l.hmin).i32(l.hmax);
                for v in l.heights.iter().chain(&l.areas).chain(&l.cons) {
                    p.u8(*v);
                }
            }
        });
        return Ok(None);
    }
    match c.partition.as_str() {
        "watershed" => {
            recast::build_distance_field(ws, &mut chf)?;
            sink.record(&format!("{prefix}distance_field"), |p| {
                stream_compact(p, &chf, true, false)
            });
            recast::build_regions(
                ws,
                &mut chf,
                cfg.border_size,
                cfg.min_region_area,
                cfg.merge_region_area,
            )?;
        }
        "monotone" => recast::build_regions_monotone(
            ws,
            &mut chf,
            cfg.border_size,
            cfg.min_region_area,
            cfg.merge_region_area,
        )?,
        "layers" => {
            recast::build_layer_regions(ws, &mut chf, cfg.border_size, cfg.min_region_area)?
        }
        _ => return Err(Error::InvalidParameter("partition")),
    }
    sink.record(&format!("{prefix}regions"), |p| {
        stream_compact(p, &chf, false, true)
    });
    let cset = ContourSet::build(
        ws,
        &chf,
        cfg.max_simplification_error,
        cfg.max_edge_len,
        recast::CONTOUR_TESS_WALL_EDGES,
    )?;
    sink.record(&format!("{prefix}contours"), |p| stream_contours(p, &cset));
    let mut pmesh = PolyMesh::build(ws, &cset, cfg.max_verts_per_poly)?;
    for i in 0..pmesh.npolys as usize {
        if pmesh.areas[i] == recast::WALKABLE_AREA {
            pmesh.areas[i] = 0;
        }
        pmesh.flags[i] = poly_flags_for_area(pmesh.areas[i]);
    }
    sink.record(&format!("{prefix}poly_mesh"), |p| {
        stream_poly_mesh(p, &pmesh)
    });
    let dmesh = recast::PolyMeshDetail::build(
        ws,
        &pmesh,
        &chf,
        cfg.detail_sample_dist,
        cfg.detail_sample_max_error,
    )?;
    sink.record(&format!("{prefix}detail_mesh"), |p| {
        p.i32((dmesh.meshes.len() / 4) as i32)
            .i32((dmesh.verts.len() / 3) as i32)
            .i32((dmesh.tris.len() / 4) as i32);
        for &v in &dmesh.meshes {
            p.u32(v);
        }
        for &v in &dmesh.verts {
            p.f32(v);
        }
        for &v in &dmesh.tris {
            p.u8(v);
        }
    });
    if pmesh.npolys == 0 {
        return Ok(None);
    }
    // Off-mesh connections as the oracle passes them: area 0, flags 1, ids from 1000.
    let noff = c.off_mesh.len() / 8;
    let mut om_verts = Vec::new();
    let mut om_rad = Vec::new();
    let mut om_dir = Vec::new();
    for i in 0..noff {
        let o = &c.off_mesh[i * 8..i * 8 + 8];
        om_verts.extend_from_slice(&o[..6]);
        om_rad.push(o[6]);
        om_dir.push(u8::from(o[7] != 0.0));
    }
    let om_area = vec![0u8; noff];
    let om_flags = vec![1u16; noff];
    let om_id: Vec<u32> = (0..noff as u32).map(|i| 1000 + i).collect();
    let params = detour::NavMeshCreateParams {
        verts: &pmesh.verts,
        vert_count: pmesh.nverts,
        polys: &pmesh.polys,
        poly_flags: &pmesh.flags,
        poly_areas: &pmesh.areas,
        poly_count: pmesh.npolys,
        nvp: pmesh.nvp,
        detail_meshes: &dmesh.meshes,
        detail_verts: &dmesh.verts,
        detail_verts_count: (dmesh.verts.len() / 3) as i32,
        detail_tris: &dmesh.tris,
        detail_tri_count: (dmesh.tris.len() / 4) as i32,
        off_mesh_con_verts: &om_verts,
        off_mesh_con_rad: &om_rad,
        off_mesh_con_flags: &om_flags,
        off_mesh_con_areas: &om_area,
        off_mesh_con_dir: &om_dir,
        off_mesh_con_user_id: &om_id,
        off_mesh_con_count: noff as i32,
        user_id: 0,
        tile_x: tile.0,
        tile_y: tile.1,
        tile_layer: 0,
        bmin: pmesh.bmin,
        bmax: pmesh.bmax,
        walkable_height: c.agent_height,
        walkable_radius: c.agent_radius,
        walkable_climb: c.agent_max_climb,
        cs: cfg.cs,
        ch: cfg.ch,
        build_bv_tree: true,
    };
    detour::create_nav_mesh_data(&params).map(Some)
}

/// The navmesh of a bake: tiles in add order (x, y, layer, bytes) and the navmesh
/// layout (`None` for a solo bake, which takes it from its single tile).
struct Baked {
    tiles: Vec<(i32, i32, i32, Vec<u8>)>,
    params: Option<NavMeshParams>,
}

fn bake(c: &Case, g: &Geometry, sink: &mut Sink) -> Result<Baked, Error> {
    let mut ws = Workspace::new();
    let base = make_config(c);
    let mut out = Baked {
        tiles: Vec::new(),
        params: None,
    };
    if c.mode == "solo" {
        let mut cfg = base;
        cfg.bmin = g.bmin;
        cfg.bmax = g.bmax;
        (cfg.width, cfg.height) = recast::calc_grid_size(&cfg.bmin, &cfg.bmax, cfg.cs);
        if let Some(t) = build_one(&mut ws, c, g, &cfg, &g.tris, sink, "", (0, 0))? {
            out.tiles.push((0, 0, 0, t));
        }
        return Ok(out);
    }
    if c.mode != "tiled" && c.mode != "tilecache" {
        return Err(Error::InvalidParameter("mode"));
    }
    let (gw, gh) = recast::calc_grid_size(&g.bmin, &g.bmax, c.cell_size);
    let ts = c.tile_size;
    if ts <= 0 {
        return Err(Error::InvalidParameter("tile size"));
    }
    let tw = (gw + ts - 1) / ts;
    let th = (gh + ts - 1) / ts;
    let buckets = bucket_triangles(g, &base, ts, tw, th);
    let mut tile_bits = 0u32;
    {
        let v = (tw * th) as u32;
        let mut p = 1u32;
        while p < v {
            p <<= 1;
        }
        while (1u32 << tile_bits) < p {
            tile_bits += 1;
        }
    }
    let mut tile_bits = tile_bits.min(14);
    if c.mode == "tilecache" {
        // The demo's derivation: four expected layers per tile.
        let v = ((tw * th * 4) as u32).next_power_of_two();
        tile_bits = v.trailing_zeros().min(14);
    }
    let tcs = ts as f32 * c.cell_size;
    out.params = Some(NavMeshParams {
        orig: g.bmin,
        tile_width: tcs,
        tile_height: tcs,
        max_tiles: 1 << tile_bits,
        max_polys: 1 << (22 - tile_bits),
    });
    for y in 0..th {
        for x in 0..tw {
            let cfg = tile_config(&base, g, ts, x, y);
            let t = build_one(
                &mut ws,
                c,
                g,
                &cfg,
                &buckets[(y * tw + x) as usize],
                sink,
                &format!("tile_{x}_{y}."),
                (x, y),
            )?;
            if let Some(t) = t {
                out.tiles.push((x, y, 0, t));
            }
        }
    }
    if c.mode == "tilecache" {
        // The tile-cache tile builder is a later phase; tiles come from --tiles.
        out.tiles.clear();
    }
    Ok(out)
}

#[allow(clippy::unnecessary_cast)] // PolyRef is u32 or u64 by feature
fn put_ref(p: &mut Payload, r: PolyRef) {
    if detour::POLYREF_BITS == 64 {
        p.bytes(&(r as u64).to_le_bytes());
    } else {
        p.u32(r as u32);
    }
}

fn record_navmesh(b: &Baked, sink: &mut Sink) {
    sink.record("navmesh_tiles", |p| {
        p.u32(b.tiles.len() as u32);
        for (x, y, l, d) in &b.tiles {
            p.i32(*x).i32(*y).i32(*l).u32(d.len() as u32).bytes(d);
        }
    });
}

/// Builds the navmesh exactly as the oracle does: a solo bake through the
/// single-tile init, a tiled bake through `add_tile` in row-major order.
fn assemble(b: &Baked) -> Result<NavMesh, Error> {
    match &b.params {
        None => {
            let (_, _, _, d) = b.tiles.first().ok_or(Error::Failed)?;
            Ok(NavMesh::new_single(d.clone())?.0)
        }
        Some(params) => {
            let mut nav = NavMesh::new(params)?;
            for (_, _, _, d) in &b.tiles {
                nav.add_tile(d.clone())?;
            }
            Ok(nav)
        }
    }
}

/// The oracle's xorshift32 point generator.
struct Rng(u32);

impl Rng {
    fn frand(&mut self) -> f32 {
        self.0 ^= self.0 << 13;
        self.0 ^= self.0 >> 17;
        self.0 ^= self.0 << 5;
        (self.0 >> 8) as f32 * (1.0 / 16_777_216.0)
    }
}

fn st<T>(r: &Result<T, Error>, ok: impl FnOnce(&T) -> Status) -> u32 {
    detour::raw_status(r, ok)
}

/// One query set (reference/RECORDS.md), record for record as the oracle writes it.
fn record_queries(
    c: &Case,
    nav: &NavMesh,
    filter: &dyn QueryFilter,
    pre: &str,
    sink: &mut Sink,
) -> Result<(), Error> {
    let mut q = NavMeshQuery::new(nav, 2048)?;
    let rng = std::cell::RefCell::new(Rng(if c.seed != 0 { c.seed } else { 1 }));
    let n = c.queries.max(0) as usize;
    let mut refs = vec![0 as PolyRef; n + 1];
    let mut pts = vec![[0.0f32; 3]; n + 1];
    {
        let mut p = Payload::new();
        p.u32((n + 1) as u32);
        for i in 0..=n {
            let r = q.find_random_point(filter, &mut || rng.borrow_mut().frand());
            if let Ok((rf, pt)) = &r {
                refs[i] = *rf;
                pts[i] = *pt;
            }
            p.u32(st(&r, |_| Status::OK));
            put_ref(&mut p, refs[i]);
            p.v3(pts[i]);
        }
        sink.record(&format!("{pre}q_random_point"), |pl| {
            pl.bytes(p.as_bytes());
        });
    }
    {
        let ext = [2.0f32, 4.0, 2.0];
        let mut r1 = Payload::new();
        let mut r2 = Payload::new();
        r1.u32((n + 1) as u32);
        r2.u32((n + 1) as u32);
        let mut polys = Vec::new();
        for pt in &pts {
            let probe = [pt[0] + 0.37, pt[1] + 1.0, pt[2] - 0.21];
            let r = q.find_nearest_poly(&probe, &ext, filter);
            let (rf, np, over) = r.as_ref().map_or((0, [0.0; 3], false), |v| *v);
            r1.u32(st(&r, |_| Status::OK));
            put_ref(&mut r1, rf);
            r1.v3(np).u8(u8::from(over));
            let s = q.query_polygons(&probe, &ext, filter, &mut polys, 64);
            r2.u32(st(&s, |s| *s)).i32(polys.len() as i32);
            for &x in &polys {
                put_ref(&mut r2, x);
            }
        }
        sink.record(&format!("{pre}q_nearest_poly"), |pl| {
            pl.bytes(r1.as_bytes());
        });
        sink.record(&format!("{pre}q_query_polygons"), |pl| {
            pl.bytes(r2.as_bytes());
        });
    }
    const MAXP: usize = 256;
    const NAMES: [&str; 18] = [
        "q_find_path",
        "q_straight_path",
        "q_straight_path_all_crossings",
        "q_straight_path_area_crossings",
        "q_sliced_path",
        "q_partial_sliced_path",
        "q_raycast",
        "q_raycast_plain",
        "q_distance_to_wall",
        "q_move_along_surface",
        "q_polys_around_circle",
        "q_polys_around_shape",
        "q_local_neighbourhood",
        "q_poly_wall_segments",
        "q_closest_point_on_poly",
        "q_closest_point_on_poly_boundary",
        "q_poly_height",
        "q_random_point_around_circle",
    ];
    let mut rec: Vec<Payload> = (0..NAMES.len()).map(|_| Payload::new()).collect();
    for r in rec.iter_mut() {
        r.u32(n as u32);
    }
    let mut path = Vec::new();
    let mut path2 = Vec::new();
    let mut path3 = Vec::new();
    let mut spath = Vec::new();
    for i in 0..n {
        let (a, e) = (&pts[i], &pts[i + 1]);
        let (ra, re) = (refs[i], refs[i + 1]);
        let s = q.find_path(ra, re, a, e, filter, &mut path, MAXP);
        rec[0].u32(st(&s, |s| *s)).i32(path.len() as i32);
        for &x in &path {
            put_ref(&mut rec[0], x);
        }
        let opts = [
            0,
            detour::STRAIGHTPATH_ALL_CROSSINGS,
            detour::STRAIGHTPATH_AREA_CROSSINGS,
        ];
        for (o, &opt) in opts.iter().enumerate() {
            let s2 = q.find_straight_path(a, e, &path, &mut spath, MAXP, opt);
            let r = &mut rec[1 + o];
            r.u32(st(&s2, |s| *s)).i32(spath.len() as i32);
            for sp in &spath {
                r.v3(sp.pos).u8(sp.flags);
                put_ref(r, sp.r#ref);
            }
        }
        // Sliced, 8 iterations per update.
        {
            let mut ss = st(&q.init_sliced_find_path(ra, re, a, e, filter, 0), |s| *s);
            while ss & Status::IN_PROGRESS != 0 {
                ss = st(&q.update_sliced_find_path(8, filter), |s| s.0);
            }
            let f = q.finalize_sliced_find_path(filter, &mut path2, MAXP);
            rec[4].u32(st(&f, |s| *s)).i32(path2.len() as i32);
            for &x in &path2 {
                put_ref(&mut rec[4], x);
            }
        }
        // Partial: one update of 4 iterations, then finalize against the corridor.
        {
            let s0 = st(&q.init_sliced_find_path(ra, re, a, e, filter, 0), |s| *s);
            let mut done = 0usize;
            let s1 = if s0 & Status::IN_PROGRESS != 0 {
                let u = q.update_sliced_find_path(4, filter);
                if let Ok((_, d)) = &u {
                    done = *d;
                }
                st(&u, |s| s.0)
            } else {
                s0
            };
            let s2 = q.finalize_sliced_find_path_partial(filter, &path, &mut path3, MAXP);
            let r = &mut rec[5];
            r.u32(s0)
                .u32(s1)
                .i32(done as i32)
                .u32(st(&s2, |s| *s))
                .i32(path3.len() as i32);
            for &x in &path3 {
                put_ref(r, x);
            }
        }
        {
            let h = q.raycast_with(ra, a, e, filter, detour::RAYCAST_USE_COSTS, MAXP, 0);
            let (hit, _) = h
                .as_ref()
                .map_or((RaycastHit::default(), Status::OK), |v| v.clone());
            let r = &mut rec[6];
            r.u32(st(&h, |v| v.1))
                .f32(hit.t)
                .v3(hit.hit_normal)
                .i32(hit.hit_edge_index)
                .i32(hit.path.len() as i32);
            for &x in &hit.path {
                put_ref(r, x);
            }
            r.f32(hit.path_cost);
        }
        {
            let h = q.raycast(ra, a, e, filter, MAXP);
            let (hit, _) = h
                .as_ref()
                .map_or((RaycastHit::default(), Status::OK), |v| v.clone());
            let r = &mut rec[7];
            r.u32(st(&h, |v| v.1))
                .f32(hit.t)
                .v3(hit.hit_normal)
                .i32(hit.path.len() as i32);
            for &x in &hit.path {
                put_ref(r, x);
            }
        }
        {
            let w = q.find_distance_to_wall(ra, a, 5.0, filter);
            let (d, hp, hn, _) = w
                .as_ref()
                .map_or((0.0, [0.0; 3], [0.0; 3], Status::OK), |v| *v);
            rec[8].u32(st(&w, |v| v.3)).f32(d).v3(hp).v3(hn);
        }
        {
            let mut vis = Vec::new();
            let m = q.move_along_surface(ra, a, e, filter, &mut vis, 16);
            let res = m.as_ref().map_or([0.0; 3], |v| v.0);
            let r = &mut rec[9];
            r.u32(st(&m, |v| v.1)).v3(res).i32(vis.len() as i32);
            for &x in &vis {
                put_ref(r, x);
            }
        }
        let mut res = Vec::new();
        {
            let s = q.find_polys_around_circle(ra, a, 4.0, filter, &mut res, 64);
            let r = &mut rec[10];
            r.u32(st(&s, |s| *s)).i32(res.len() as i32);
            for &(x, pa, cost) in &res {
                put_ref(r, x);
                put_ref(r, pa);
                r.f32(cost);
            }
        }
        {
            let shape = [
                a[0] - 2.0,
                a[1],
                a[2] - 2.0,
                a[0] - 2.0,
                a[1],
                a[2] + 2.0,
                a[0] + 2.0,
                a[1],
                a[2] + 2.0,
                a[0] + 2.0,
                a[1],
                a[2] - 2.0,
            ];
            let s = q.find_polys_around_shape(ra, &shape, filter, &mut res, 64);
            let r = &mut rec[11];
            r.u32(st(&s, |s| *s)).i32(res.len() as i32);
            for &(x, pa, cost) in &res {
                put_ref(r, x);
                put_ref(r, pa);
                r.f32(cost);
            }
        }
        {
            let mut loc = Vec::new();
            let s = q.find_local_neighbourhood(ra, a, 3.0, filter, &mut loc, 32);
            let r = &mut rec[12];
            r.u32(st(&s, |s| *s)).i32(loc.len() as i32);
            for &(x, pa) in &loc {
                put_ref(r, x);
                put_ref(r, pa);
            }
        }
        {
            let mut segs = Vec::new();
            let s = q.poly_wall_segments(ra, filter, true, &mut segs, 64);
            let r = &mut rec[13];
            r.u32(st(&s, |s| *s)).i32(segs.len() as i32);
            for sg in &segs {
                r.v3([sg.seg[0], sg.seg[1], sg.seg[2]])
                    .v3([sg.seg[3], sg.seg[4], sg.seg[5]]);
                put_ref(r, sg.r#ref);
            }
        }
        {
            let cp = q.closest_point_on_poly(ra, e);
            let (pt, over) = cp.as_ref().map_or(([0.0; 3], false), |v| *v);
            rec[14]
                .u32(st(&cp, |_| Status::OK))
                .v3(pt)
                .u8(u8::from(over));
            let cb = q.closest_point_on_poly_boundary(ra, e);
            rec[15]
                .u32(st(&cb, |_| Status::OK))
                .v3(*cb.as_ref().unwrap_or(&[0.0; 3]));
        }
        {
            let ha = q.poly_height(ra, a);
            let he = q.poly_height(ra, e);
            rec[16]
                .u32(st(&ha, |_| Status::OK))
                .f32(*ha.as_ref().unwrap_or(&0.0))
                .u32(st(&he, |_| Status::OK))
                .f32(*he.as_ref().unwrap_or(&0.0));
        }
        {
            let rc = q.find_random_point_around_circle(ra, a, 3.0, filter, &mut || {
                rng.borrow_mut().frand()
            });
            let (rf, pt) = rc.as_ref().map_or((0, [0.0; 3]), |v| (v.0, v.1));
            let r = &mut rec[17];
            r.u32(st(&rc, |v| v.2));
            put_ref(r, rf);
            r.v3(pt);
        }
    }
    for (name, p) in NAMES.iter().zip(rec.iter()) {
        sink.record(&format!("{pre}{name}"), |pl| {
            pl.bytes(p.as_bytes());
        });
    }
    Ok(())
}

/// Navmesh verdicts on malformed or conflicting tiles (reference/RECORDS.md).
fn record_malformed(b: &Baked, nav: &NavMesh, sink: &mut Sink) {
    let Some((_, _, _, src)) = b.tiles.first() else {
        return;
    };
    let Ok(mut fresh) = NavMesh::new(nav.params()) else {
        return;
    };
    let mut p = Payload::new();
    p.u32(8);
    let mutate = |f: &dyn Fn(&mut Vec<u8>)| {
        let mut d = src.clone();
        f(&mut d);
        d
    };
    let add =
        |p: &mut Payload, fresh: &mut NavMesh, code: u32, d: Vec<u8>, last: PolyRef| -> PolyRef {
            // Upstream's addTile(lastRef): the stale-ref restore of case 8.
            let r = if last == 0 {
                fresh.add_tile(d)
            } else {
                fresh.restore_tile_allow_stale(d, last)
            };
            let rf = *r.as_ref().unwrap_or(&0);
            p.u32(code).u32(st(&r, |_| Status::OK));
            put_ref(p, rf);
            rf
        };
    let valid = add(&mut p, &mut fresh, 1, src.clone(), 0);
    add(&mut p, &mut fresh, 2, src.clone(), 0);
    add(&mut p, &mut fresh, 3, mutate(&|d| d[3] ^= 0x01), 0);
    add(
        &mut p,
        &mut fresh,
        4,
        mutate(&|d| d[4..8].copy_from_slice(&(detour::NAVMESH_VERSION - 1).to_le_bytes())),
        0,
    );
    add(
        &mut p,
        &mut fresh,
        5,
        mutate(&|d| d[24..28].copy_from_slice(&(1i32 << 23).to_le_bytes())),
        0,
    );
    let r6 = fresh.remove_tile(0);
    p.u32(6).u32(st(&r6, |_| Status::OK));
    put_ref(&mut p, 0);
    let r7a = fresh.remove_tile(valid);
    let r7b = fresh.remove_tile(valid);
    p.u32(7)
        .u32(st(&r7a, |_| Status::OK))
        .u32(st(&r7b, |_| Status::OK));
    add(&mut p, &mut fresh, 8, src.clone(), valid);
    sink.record("malformed_tiles", |pl| {
        pl.bytes(p.as_bytes());
    });
}

/// Tiles from a `navmesh_tiles` record of another record file (tile-cache cases,
/// whose tile builder is a later phase).
fn tiles_from(path: &str, b: &mut Baked) -> Result<(), String> {
    let mut f = navmesh_parity::container::RecordFile::open(Path::new(path)).map_err(|e| e.0)?;
    let i = f.find("navmesh_tiles").ok_or("no navmesh_tiles record")?;
    let d = f.payload(i).map_err(|e| e.0)?;
    let rd = |at: usize| i32::from_le_bytes([d[at], d[at + 1], d[at + 2], d[at + 3]]);
    let n = rd(0) as usize;
    let mut at = 4;
    b.tiles.clear();
    for _ in 0..n {
        let (x, y, l, len) = (rd(at), rd(at + 4), rd(at + 8), rd(at + 12) as usize);
        at += 16;
        b.tiles.push((x, y, l, d[at..at + len].to_vec()));
        at += len;
    }
    Ok(())
}

fn run() -> Result<(), String> {
    let args: Vec<String> = std::env::args().skip(1).collect();
    let usage = "usage: nav-port bake <case> --records FILE | bench <case> [--repeat N]";
    let (cmd, case_path) = match (args.first(), args.get(1)) {
        (Some(a), Some(b)) => (a.as_str(), b.as_str()),
        _ => return Err(usage.into()),
    };
    let opt = |name: &str| {
        args.iter()
            .position(|a| a == name)
            .and_then(|i| args.get(i + 1))
    };
    let case = parse_case(Path::new(case_path))?;
    let g = load_obj(&case.mesh)?;
    let base = Path::new(case_path)
        .file_name()
        .map(|s| s.to_string_lossy().into_owned())
        .unwrap_or_default();
    match cmd {
        "bake" => {
            let out = opt("--records").ok_or(usage)?;
            let mut writer = RecordFileWriter::new(&[
                ("producer", "nav_port"),
                ("upstream", UPSTREAM),
                ("case", base.as_str()),
                (
                    "polyref_bits",
                    if detour::POLYREF_BITS == 64 {
                        "64"
                    } else {
                        "32"
                    },
                ),
            ]);
            {
                let mut p = Payload::new();
                p.u32((g.verts.len() / 3) as u32);
                for &v in &g.verts {
                    p.f32(v);
                }
                p.u32((g.tris.len() / 3) as u32);
                for &t in &g.tris {
                    p.i32(t);
                }
                writer.record("geometry", p.as_bytes());
            }
            let mut sink = Sink {
                writer: Some(writer),
            };
            let res = bake(&case, &g, &mut sink).map_err(|e| format!("bake failed: {e}"));
            let res = res.and_then(|mut b| {
                if case.mode == "tilecache" {
                    match opt("--tiles") {
                        Some(t) => tiles_from(t, &mut b)?,
                        None => return Ok(()),
                    }
                } else {
                    record_navmesh(&b, &mut sink);
                }
                if b.tiles.is_empty() {
                    return Ok(());
                }
                let nav = assemble(&b).map_err(|e| format!("navmesh: {e}"))?;
                let costs = cost_filter();
                record_queries(&case, &nav, &DefaultFilter::default(), "", &mut sink)
                    .and_then(|()| record_queries(&case, &nav, &costs, "filtered.", &mut sink))
                    .map_err(|e| format!("queries: {e}"))?;
                record_malformed(&b, &nav, &mut sink);
                Ok(())
            });
            let bytes = sink.writer.take().map(|w| w.finish()).unwrap_or_default();
            std::fs::write(out, bytes).map_err(|e| format!("writing {out}: {e}"))?;
            res
        }
        "bench-queries" => {
            // One bake, then the plain query set `--repeat` times (no records).
            let repeat: usize = opt("--repeat").and_then(|v| v.parse().ok()).unwrap_or(1);
            let mut b = bake(&case, &g, &mut Sink { writer: None })
                .map_err(|e| format!("bake failed: {e}"))?;
            if let Some(t) = opt("--tiles") {
                tiles_from(t, &mut b)?;
            }
            let nav = assemble(&b).map_err(|e| format!("navmesh: {e}"))?;
            let t0 = Instant::now();
            for _ in 0..repeat {
                record_queries(
                    &case,
                    &nav,
                    &DefaultFilter::default(),
                    "",
                    &mut Sink { writer: None },
                )
                .map_err(|e| format!("queries: {e}"))?;
            }
            println!(
                "{{\"bench\":\"queries\",\"repeat\":{repeat},\"total_ms\":{:.3}}}",
                t0.elapsed().as_secs_f64() * 1000.0
            );
            Ok(())
        }
        "bench" => {
            let repeat: usize = opt("--repeat").and_then(|v| v.parse().ok()).unwrap_or(1);
            let mut ms = Vec::new();
            for _ in 0..repeat {
                let t0 = Instant::now();
                bake(&case, &g, &mut Sink { writer: None })
                    .map_err(|e| format!("bake failed: {e}"))?;
                ms.push(t0.elapsed().as_secs_f64() * 1000.0);
            }
            ms.sort_by(f64::total_cmp);
            println!(
                "{{\"bench\":\"bake\",\"repeat\":{repeat},\"min_ms\":{:.3},\"median_ms\":{:.3}}}",
                ms.first().copied().unwrap_or(0.0),
                ms.get(ms.len() / 2).copied().unwrap_or(0.0)
            );
            Ok(())
        }
        _ => Err(usage.into()),
    }
}

fn main() -> ExitCode {
    match run() {
        Ok(()) => ExitCode::SUCCESS,
        Err(e) => {
            eprintln!("nav-port: {e}");
            ExitCode::from(1)
        }
    }
}
