//! Writes seed corpora for the fuzz targets.
//!
//! ```text
//! seeds tiles  <out-dir> <file.navrec>...   every tile of the navmesh_tiles records
//! seeds recast <out-dir> <mesh.obj>...      each mesh with demo settings, per partition
//! seeds crowd <out-dir>                     `crowd_update` inputs, 16-byte records
//! seeds tilecache <out-dir>                 `tilecache_update` inputs (layout in the target)
//! seeds parallel <out-dir>                  `parallel_bake` inputs (layout in the target)
//! seeds recast-grid <out-dir>               two `recast_input` meshes with 50,000 polygons
//! ```
use navmesh_fuzz::{RecastInput, demo_settings, run_recast};
use navmesh_rs::Workspace;
use navmesh_rs::tilecache::{LayerHeader, RawCompressor, build_layer};
use std::path::Path;
use std::process::ExitCode;

/// Largest seed of the Recast target (libFuzzer's `-max_len` must be at least this).
const MAX_RECAST_SEED: usize = 1 << 20;

fn tiles(out: &Path, files: &[String]) -> Result<usize, String> {
    let mut written = 0;
    for path in files {
        let p = Path::new(path);
        let mut f = navmesh_parity::RecordFile::open(p).map_err(|e| format!("{path}: {e:?}"))?;
        let Some(i) = f.find("navmesh_tiles") else {
            continue;
        };
        let d = f.payload(i).map_err(|e| format!("{path}: {e:?}"))?;
        let rd = |at: usize| -> Result<usize, String> {
            let b = d.get(at..at + 4).ok_or(format!("{path}: truncated"))?;
            usize::try_from(i32::from_le_bytes([b[0], b[1], b[2], b[3]]))
                .map_err(|_| format!("{path}: negative length"))
        };
        let n = rd(0)?;
        let mut at = 4;
        let stem = p.file_stem().unwrap_or_default().to_string_lossy();
        for k in 0..n {
            let len = rd(at + 12)?;
            at += 16;
            let tile = d
                .get(at..at + len)
                .ok_or(format!("{path}: truncated tile"))?;
            std::fs::write(out.join(format!("{stem}-{k}")), tile).map_err(|e| e.to_string())?;
            at += len;
            written += 1;
        }
    }
    Ok(written)
}

/// Vertices and fan-triangulated faces of an OBJ file (`v` and `f` lines only).
fn load_obj(path: &str) -> Result<(Vec<f32>, Vec<i32>), String> {
    let text = std::fs::read_to_string(path).map_err(|e| format!("{path}: {e}"))?;
    let (mut verts, mut tris) = (Vec::new(), Vec::new());
    for line in text.lines() {
        let mut it = line.split_whitespace();
        match it.next() {
            Some("v") => {
                for _ in 0..3 {
                    let v = it.next().and_then(|x| x.parse::<f32>().ok());
                    verts.push(v.ok_or(format!("{path}: bad vertex"))?);
                }
            }
            Some("f") => {
                let nv = (verts.len() / 3) as i64;
                let idx: Vec<i32> = it
                    .filter_map(|x| x.split('/').next()?.parse::<i64>().ok())
                    .map(|i| if i < 0 { nv + i } else { i - 1 } as i32)
                    .collect();
                for k in 2..idx.len() {
                    tris.extend_from_slice(&[idx[0], idx[k - 1], idx[k]]);
                }
            }
            _ => {}
        }
    }
    Ok((verts, tris))
}

fn recast_seeds(out: &Path, files: &[String]) -> Result<usize, String> {
    let mut written = 0;
    for path in files {
        let (verts, tris) = load_obj(path)?;
        let stem = Path::new(path)
            .file_stem()
            .unwrap_or_default()
            .to_string_lossy();
        // Three partitions with demo settings, one with every optional step on, two with a
        // border, one with polygons of 70 vertices.
        for (name, settings) in [
            ("watershed", demo_settings(0)),
            ("monotone", demo_settings(1)),
            ("layers", demo_settings(2)),
            ("all-steps", {
                let mut s = demo_settings(0);
                s[0] = 0xfc;
                s[14] = 3;
                s
            }),
            // No erosion and a border: regions meet the border strip, so contour vertices
            // there are flagged and later removed from the polygon mesh.
            ("border", {
                let mut s = demo_settings(0);
                s[6] = 0;
                s[14] = 4;
                s
            }),
            ("border-small-regions", {
                let mut s = demo_settings(0);
                s[6] = 0;
                s[7] = 0;
                s[8] = 0;
                s[11] = 3;
                s[14] = 4;
                s
            }),
            // More than 64 vertices per polygon, the size of the stack buffer in polygon merging.
            ("wide-polys", {
                let mut s = demo_settings(0);
                s[11] = 70;
                s
            }),
        ] {
            let input = RecastInput {
                settings,
                verts: verts.clone(),
                tris: tris.clone(),
            };
            let Some(bytes) = input.encode().filter(|b| b.len() <= MAX_RECAST_SEED) else {
                println!("{stem}: skipped (too large for a seed)");
                break;
            };
            println!("{stem}-{name}: {:?}", run_recast(&input));
            std::fs::write(out.join(format!("{stem}-{name}")), bytes).map_err(|e| e.to_string())?;
            written += 1;
        }
    }
    Ok(written)
}

/// Two `recast_input` seeds with more polygons than a 16-bit edge index can number: a 71 x 71
/// lattice of plus-shaped islands (two overlapping bars each, one cell wide, five cells apart)
/// that the polygon mesh triangulates into ten polygons per island (`nvp = 3`, no simplification
/// error, no merging). They reach the edge-count guard of the polygon mesh adjacency, which
/// no other input in the corpus does.
fn recast_grid_seeds(out: &Path) -> Result<usize, String> {
    const SIDE: usize = 71;
    const PITCH: f64 = 5.0;
    let cs = 0.1f64;
    let mut verts = Vec::new();
    let mut tris = Vec::new();
    let mut rect = |x0: f64, x1: f64, z0: f64, z1: f64| {
        let base = (verts.len() / 3) as i32;
        for (x, z) in [(x0, z0), (x0, z1), (x1, z0), (x1, z1)] {
            verts.extend_from_slice(&[(x * cs) as f32, 0.0, (z * cs) as f32]);
        }
        // Up-facing winding: (a, b, c) and (c, b, d).
        tris.extend_from_slice(&[base, base + 1, base + 2, base + 2, base + 1, base + 3]);
    };
    for a in 0..SIDE {
        for b in 0..SIDE {
            let (x, z) = (a as f64 * PITCH, b as f64 * PITCH);
            // Bars inset from cell boundaries so each covers exactly its cells.
            rect(x + 0.1, x + 2.9, z + 1.1, z + 1.9);
            rect(x + 1.1, x + 1.9, z + 0.1, z + 2.9);
        }
    }
    let mut written = 0;
    for (name, partition) in [("watershed", 0u8), ("monotone", 1)] {
        // Flags: partition only (the span filters would remove isolated cells); cell size 0.1;
        // slope 45; height 10, climb 4, radius 0; every area, edge and error limit 0; three
        // vertices per polygon; no detail sampling; no border.
        let settings = [partition, 9, 19, 45, 10, 4, 0, 0, 0, 0, 0, 3, 0, 0, 0, 0];
        let input = RecastInput {
            settings,
            verts: verts.clone(),
            tris: tris.clone(),
        };
        let bytes = input
            .encode()
            .filter(|b| b.len() <= MAX_RECAST_SEED)
            .ok_or("grid seed does not fit the recast layout")?;
        println!(
            "grid-{name}: {} bytes, {:?}",
            bytes.len(),
            run_recast(&input)
        );
        std::fs::write(out.join(format!("grid-plus-{name}")), bytes).map_err(|e| e.to_string())?;
        written += 1;
    }
    Ok(written)
}

/// Deterministic generator (splitmix64), so seed files are reproducible.
struct Rng(u64);
impl Rng {
    fn next(&mut self) -> u64 {
        self.0 = self.0.wrapping_add(0x9e37_79b9_7f4a_7c15);
        let mut z = self.0;
        z = (z ^ (z >> 30)).wrapping_mul(0xbf58_476d_1ce4_e5b9);
        z = (z ^ (z >> 27)).wrapping_mul(0x94d0_49bb_1331_11eb);
        z ^ (z >> 31)
    }
    fn byte(&mut self) -> u8 {
        (self.next() >> 24) as u8
    }
    /// Uniform in `[lo, hi)`.
    fn range(&mut self, lo: f32, hi: f32) -> f32 {
        lo + (self.next() >> 40) as f32 / (1u64 << 24) as f32 * (hi - lo)
    }
}

/// One 16-byte `crowd_update` record: three little-endian `f32`, agent id, operation, two tail bytes.
fn crowd_record(v: [f32; 3], id: u8, op: u8, tail: [u8; 2]) -> [u8; 16] {
    let mut r = [0u8; 16];
    for (i, x) in v.iter().enumerate() {
        r[i * 4..i * 4 + 4].copy_from_slice(&x.to_le_bytes());
    }
    r[12] = id;
    r[13] = op;
    r[14..].copy_from_slice(&tail);
    r
}

/// How the two unused tail bytes of each record are filled.
#[derive(Clone, Copy)]
enum Tail {
    Zero,
    Ones,
    Counter,
    Random,
}

/// `n` records on the fuzz target's 20 x 20 mesh. `style` picks the operation mix:
/// 0 = random operations, 1 = eight agents then targets and updates, 2 = add/remove churn.
fn crowd_input(rng: &mut Rng, n: usize, style: u8, tail: Tail) -> Vec<u8> {
    let mut out = Vec::with_capacity(n * 16);
    for k in 0..n {
        let pos = [rng.range(1.0, 19.0), 0.0, rng.range(1.0, 19.0)];
        let id = rng.byte() % 8;
        let (v, op) = match style {
            1 if k < 8 => (pos, 0),
            1 => match k % 4 {
                0 => (pos, 2),
                1 => ([rng.range(0.02, 0.5), 0.0, 0.0], 5),
                2 => ([rng.range(-2.0, 2.0), 0.0, rng.range(-2.0, 2.0)], 1),
                _ => (pos, 2),
            },
            2 => match k % 5 {
                0 | 1 => (pos, 0),
                2 => (pos, 2),
                3 => ([rng.range(0.02, 0.5), 0.0, 0.0], 5),
                _ => (pos, 3 + (k / 5 % 2) as u8),
            },
            _ => match rng.byte() % 6 {
                5 => ([rng.range(0.02, 0.5), 0.0, 0.0], 5),
                o => (pos, o),
            },
        };
        let t = match tail {
            Tail::Zero => [0, 0],
            Tail::Ones => [0xff, 0xff],
            Tail::Counter => [k as u8, (k as u8).wrapping_mul(7)],
            Tail::Random => [rng.byte(), rng.byte()],
        };
        out.extend_from_slice(&crowd_record(v, id, op, t));
    }
    out
}

/// Crowd inputs of 1 to 64 records, with the 62, 63 and 64-record sizes (up to the 1,024-byte
/// maximum) written for every operation mix and tail style, and a 63-record input with a partial
/// trailing record.
fn crowd_seeds(out: &Path) -> Result<usize, String> {
    let mut written = 0;
    let mut put = |name: String, bytes: Vec<u8>| -> Result<(), String> {
        std::fs::write(out.join(name), bytes).map_err(|e| e.to_string())?;
        written += 1;
        Ok(())
    };
    let mut rng = Rng(0x6372_6f77_6400_0001);
    for n in [1usize, 4, 8, 16, 32, 48] {
        for (style, tail) in [(0, Tail::Random), (1, Tail::Zero), (2, Tail::Counter)] {
            put(
                format!("crowd-n{n}-s{style}"),
                crowd_input(&mut rng, n, style, tail),
            )?;
        }
    }
    for n in [62usize, 63, 64] {
        for style in 0..3u8 {
            for (tn, tail) in [Tail::Zero, Tail::Ones, Tail::Counter, Tail::Random]
                .into_iter()
                .enumerate()
            {
                put(
                    format!("crowd-n{n}-s{style}-t{tn}"),
                    crowd_input(&mut rng, n, style, tail),
                )?;
            }
        }
    }
    let mut partial = crowd_input(&mut rng, 63, 0, Tail::Random);
    partial.extend((0..15).map(|_| rng.byte()));
    put("crowd-n63-partial-tail".to_string(), partial)?;
    Ok(written)
}

/// `tilecache_update` header: mode, little-endian layer length, then the layer bytes.
fn tilecache_input(mode: u8, layer: &[u8], ops: &[[u8; 16]]) -> Vec<u8> {
    let mut v = vec![mode];
    v.extend_from_slice(&(layer.len() as u16).to_le_bytes());
    v.extend_from_slice(layer);
    for o in ops {
        v.extend_from_slice(o);
    }
    v
}

/// One 16-byte `tilecache_update` operation: code, selector byte, then three `f32` operands.
fn cache_op(code: u8, sel: u8, a: [f32; 3]) -> [u8; 16] {
    let mut r = [0u8; 16];
    r[0] = code;
    r[1] = sel;
    for (i, x) in a.iter().enumerate() {
        r[4 + i * 4..8 + i * 4].copy_from_slice(&x.to_le_bytes());
    }
    r
}

/// Obstacle sequences over the 8 x 8 cache: add the three shapes, update, drain, remove, update.
fn cache_ops(rng: &mut Rng, n: usize) -> Vec<[u8; 16]> {
    (0..n)
        .map(|_| {
            let (x, z) = (rng.range(0.5, 15.5), rng.range(0.5, 15.5));
            match rng.byte() % 8 {
                0 => cache_op(0, 0, [x, z, rng.range(0.5, 3.0)]),
                // The box reads bmin = (a, b) and bmax = (b, c), so the operands must ascend.
                1 => cache_op(1, 0, [x, x + rng.range(0.5, 3.0), x + rng.range(3.5, 6.0)]),
                2 => cache_op(2, 0, [x, z, rng.range(0.0, core::f32::consts::TAU)]),
                3 => cache_op(3, rng.byte(), [0.0; 3]),
                4 | 5 => cache_op(4, 0, [rng.range(0.0, 0.5), 0.0, 0.0]),
                _ => cache_op(5, 0, [0.0; 3]),
            }
        })
        .collect()
}

/// Tile-cache inputs: raw layers (built with `build_layer`, with and without holes and steps),
/// grid specifications for the synthesised-layer mode, and obstacle add/remove/update scripts.
fn tilecache_seeds(out: &Path) -> Result<usize, String> {
    let mut written = 0;
    let mut put = |name: String, bytes: Vec<u8>| -> Result<(), String> {
        std::fs::write(out.join(name), bytes).map_err(|e| e.to_string())?;
        written += 1;
        Ok(())
    };
    let mut rng = Rng(0x7469_6c65_6361_6368);
    let mut ws = Workspace::new();
    // Raw layers for tile slots (0,0), (1,0) and layer 1 of (0,0); the fixed layer occupies (0,0,0).
    let mut raw = Vec::new();
    for (name, tx, ty, tlayer, holes) in [
        ("flat-10", 1, 0, 0, false),
        ("flat-01", 0, 1, 0, false),
        ("holes-00-l1", 0, 0, 1, true),
        ("holes-11", 1, 1, 0, true),
    ] {
        let mut h = LayerHeader::new(8, 8);
        h.tx = tx;
        h.ty = ty;
        h.tlayer = tlayer;
        h.bmin = [tx as f32 * 8.0, 0.0, ty as f32 * 8.0];
        h.bmax = [h.bmin[0] + 8.0, 2.0, h.bmin[2] + 8.0];
        h.hmax = 255;
        let mut heights = [0u8; 64];
        let mut areas = [63u8; 64];
        if holes {
            for i in 0..64 {
                heights[i] = (i % 8 / 3 * 2) as u8;
                if (i % 8 == 4 && i / 8 % 3 == 1) || i == 27 {
                    areas[i] = 0;
                    heights[i] = 255;
                }
            }
        }
        let mut cons = [0u8; 64];
        for i in 0..64 {
            let (x, y) = (i % 8, i / 8);
            let near = |j: usize| areas[j] != 0 && heights[j].abs_diff(heights[i]) <= 4;
            if areas[i] != 0 {
                cons[i] = u8::from(x > 0 && near(i - 1))
                    | (u8::from(y < 7 && near(i + 8)) * 2)
                    | (u8::from(x < 7 && near(i + 1)) * 4)
                    | (u8::from(y > 0 && near(i - 8)) * 8);
            }
        }
        let bytes = build_layer(&mut ws, &RawCompressor, &h, &heights, &areas, &cons)
            .map_err(|e| format!("{name}: {e:?}"))?;
        raw.push((name, bytes));
    }
    for (name, bytes) in &raw {
        let ops = cache_ops(&mut rng, 24);
        put(format!("tc-raw-{name}"), tilecache_input(0, bytes, &ops))?;
        // The layer alone, to mutate its header and grids without an operation stream.
        put(
            format!("tc-raw-{name}-only"),
            tilecache_input(0, bytes, &[]),
        )?;
    }
    // Scripted obstacles on the fixed layer: shapes, drain, remove, add again.
    let script = [
        cache_op(0, 0, [4.0, 4.0, 1.5]),
        cache_op(1, 0, [1.0, 2.0, 4.0]),
        cache_op(2, 0, [5.5, 5.5, 0.7]),
        cache_op(4, 0, [0.1, 0.0, 0.0]),
        cache_op(5, 0, [0.0; 3]),
        cache_op(3, 0, [0.0; 3]),
        cache_op(4, 0, [0.1, 0.0, 0.0]),
        cache_op(0, 0, [10.0, 4.0, 2.0]),
        cache_op(1, 0, [5.0, 6.0, 9.0]),
        cache_op(5, 0, [0.0; 3]),
        cache_op(3, 1, [0.0; 3]),
        cache_op(3, 2, [0.0; 3]),
        cache_op(5, 0, [0.0; 3]),
    ];
    put("tc-script".to_string(), tilecache_input(0, &[], &script))?;
    // Grid specifications: derived connections (mode 1) and raw connections (mode 3).
    for k in 0..8u8 {
        let mut spec = vec![0u8; 193];
        spec[0] = k;
        for b in spec.iter_mut().skip(1) {
            *b = rng.byte();
        }
        for (i, b) in spec.iter_mut().enumerate().skip(1) {
            match k % 3 {
                // Mostly walkable and nearly flat: large regions.
                0 if (1..65).contains(&i) => *b = (*b >> 6) + 2,
                0 if (65..129).contains(&i) => *b &= 0x3f,
                // A few unwalkable cells.
                1 if (65..129).contains(&i) => *b = if *b > 230 { 200 } else { *b & 0x3f },
                _ => {}
            }
        }
        let ops = cache_ops(&mut rng, 32);
        put(
            format!("tc-synth-{k}"),
            tilecache_input(1 + 2 * (k % 2), &spec, &ops),
        )?;
    }
    Ok(written)
}

/// Vertices (three bytes each) and triangles of a `parallel_bake` seed mesh: four square
/// vertices first (the area volumes use them), then a grid of quads (at most 128 triangles).
/// `kind` 0 flat, 1 hills, 2 stairs, 3 flat with diagonal holes (each 8 x 8 quads of 3.5 units),
/// 4 a 3 x 3 plane with its centre quad missing, 5 a 5 x 5 plane with two interior holes.
fn bake_mesh(kind: u8) -> (Vec<[u8; 3]>, Vec<[u8; 3]>) {
    let (n, step): (u8, u8) = match kind {
        4 => (3, 24),
        5 => (5, 20),
        _ => (8, 28),
    };
    let (lo, hi) = (2 + step * n / 4, 2 + step * n / 4 * 3);
    let mut verts = vec![[lo, 40, lo], [lo, 40, hi], [hi, 40, hi], [hi, 40, lo]];
    let mut tris = vec![[0, 1, 2], [0, 2, 3]];
    for j in 0..=n {
        for i in 0..=n {
            let y = match kind {
                1 => 32 + 6 * ((i * j) % 4),
                2 => 20 + 10 * i,
                _ => 32,
            };
            verts.push([2 + step * i, y, 2 + step * j]);
        }
    }
    let at = |i: u8, j: u8| 4 + j * (n + 1) + i;
    for j in 0..n {
        for i in 0..n {
            let hole = match kind {
                3 => (i + j) % 5 == 2,
                4 => (i, j) == (1, 1),
                5 => (i, j) == (1, 1) || (i, j) == (3, 3),
                _ => false,
            };
            if hole {
                continue;
            }
            tris.push([at(i, j), at(i, j + 1), at(i + 1, j)]);
            tris.push([at(i + 1, j), at(i, j + 1), at(i + 1, j + 1)]);
        }
    }
    tris.truncate(128);
    (verts, tris)
}

/// `parallel_bake` inputs (layout in the target): six meshes, three partitions and six feature
/// sets (defaults, areas and volumes, filters off, tight tile limits, three aggregate budgets),
/// plus two raw-float inputs with extreme coordinates.
fn parallel_seeds(out: &Path) -> Result<usize, String> {
    let mut written = 0;
    let mut put = |name: String, bytes: Vec<u8>| -> Result<(), String> {
        std::fs::write(out.join(name), bytes).map_err(|e| e.to_string())?;
        written += 1;
        Ok(())
    };
    // (name, features, budget k, parameter byte)
    let sets: [(&str, u8, u8, u8); 6] = [
        ("plain", 0x00, 0, 0),
        ("volumes", 0x0e, 0, 5),
        ("nofilter", 0x0f, 0, 9),
        ("tight", 0x40, 0, 10),
        ("budget-fit", 0x1e, 200, 7),
        ("budget-low", 0x2e, 3, 7),
    ];
    for kind in 0..6u8 {
        let (verts, tris) = bake_mesh(kind);
        for part in 0..3u8 {
            for (n, &(name, features, k, param)) in sets.iter().enumerate() {
                // The small planes fit one tile, so their holes stay inside one contour set.
                let tile = if kind >= 4 {
                    48
                } else {
                    [24u8, 0, 48, 8, 30, 16][n]
                };
                let radius = (n as u8 + part) % 4;
                let vp = (n as u8 + kind) % 4;
                let cs = [0u8, 0, 1, 2, 0, 3][n];
                let mut v = vec![
                    tile,
                    part | radius << 2 | vp << 5,
                    cs << 2,
                    features,
                    k,
                    param,
                ];
                v.push(verts.len() as u8);
                v.extend(verts.iter().flatten());
                v.extend(tris.iter().flatten());
                put(format!("pb-k{kind}-p{part}-{name}"), v)?;
            }
        }
    }
    for (name, scale) in [("huge", 1e30f32), ("nan", f32::NAN)] {
        let mut v = vec![24u8, 0, 1, 0x1e, 0, 1, 6];
        for p in [
            [0.0, 0.0, 0.0],
            [scale, 0.0, 0.0],
            [0.0, 0.0, scale],
            [4.0, 0.0, 4.0],
            [0.0, 1.0, 4.0],
            [4.0, 0.0, 0.0],
        ] {
            v.extend(p.iter().flat_map(|f: &f32| f.to_le_bytes()));
        }
        v.extend([0u8, 1, 2, 3, 4, 5]);
        put(format!("pb-raw-{name}"), v)?;
    }
    Ok(written)
}

fn main() -> ExitCode {
    let args: Vec<String> = std::env::args().skip(1).collect();
    let no_inputs = matches!(
        args.first().map(String::as_str),
        Some("crowd" | "tilecache" | "parallel" | "recast-grid")
    );
    if args.len() < if no_inputs { 2 } else { 3 } {
        eprintln!(
            "usage: seeds tiles|recast <out-dir> <input>... | seeds crowd|tilecache|parallel|recast-grid <out-dir>"
        );
        return ExitCode::from(2);
    }
    let out = Path::new(&args[1]);
    if let Err(e) = std::fs::create_dir_all(out) {
        eprintln!("{}: {e}", out.display());
        return ExitCode::from(2);
    }
    let result = match args[0].as_str() {
        "tiles" => tiles(out, &args[2..]),
        "recast" => recast_seeds(out, &args[2..]),
        "recast-grid" => recast_grid_seeds(out),
        "crowd" => crowd_seeds(out),
        "tilecache" => tilecache_seeds(out),
        "parallel" => parallel_seeds(out),
        other => Err(format!("unknown mode {other}")),
    };
    match result {
        Ok(n) => {
            println!("{n} seeds written to {}", out.display());
            ExitCode::SUCCESS
        }
        Err(e) => {
            eprintln!("{e}");
            ExitCode::FAILURE
        }
    }
}
