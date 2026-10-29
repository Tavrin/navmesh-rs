//! Writes seed corpora for the fuzz targets.
//!
//! ```text
//! seeds tiles  <out-dir> <file.navrec>...   every tile of the navmesh_tiles records
//! seeds recast <out-dir> <mesh.obj>...      each mesh with demo settings, per partition
//! ```
use navmesh_fuzz::{RecastInput, demo_settings, run_recast};
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

fn main() -> ExitCode {
    let args: Vec<String> = std::env::args().skip(1).collect();
    if args.len() < 3 {
        eprintln!("usage: seeds tiles|recast <out-dir> <input>...");
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
