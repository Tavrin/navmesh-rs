//! Writes seed corpora for the fuzz targets.
//!
//! ```text
//! seeds tiles  <out-dir> <file.navrec>...   every tile of the navmesh_tiles records
//! seeds recast <out-dir> <mesh.obj>...      each mesh with demo settings, per partition
//! seeds crowd <out-dir>                     `crowd_update` inputs, 16-byte records
//! seeds tilecache <out-dir>                 `tilecache_update` inputs (layout in the target)
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
        // Three partitions with demo settings, and one with every optional step on.
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

fn main() -> ExitCode {
    let args: Vec<String> = std::env::args().skip(1).collect();
    let no_inputs = matches!(
        args.first().map(String::as_str),
        Some("crowd" | "tilecache")
    );
    if args.len() < if no_inputs { 2 } else { 3 } {
        eprintln!(
            "usage: seeds tiles|recast <out-dir> <input>... | seeds crowd|tilecache <out-dir>"
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
        "crowd" => crowd_seeds(out),
        "tilecache" => tilecache_seeds(out),
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
