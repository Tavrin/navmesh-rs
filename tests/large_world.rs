//! Measurements on generated worlds; run on demand, in release mode:
//!
//! ```text
//! NAV_WORLD_KM=4 RAYON_NUM_THREADS=4 cargo test --release --features parallel \
//!     --test large_world -- --ignored --nocapture
//! ```
//!
//! Prints the full-bake time and peak resident memory, the longest single step on a
//! 256-cell tile, and the latency of one-tile and region rebakes.
#![cfg(all(feature = "recast", feature = "detour", feature = "parallel"))]

mod common;

use std::time::Instant;

use common::world::{Edit, World};
use navmesh_rs::bake::{
    BakeConfig, BakeJob, BakeManifest, GridLayout, JobOptions, TileInstaller, TileKey, TileOutput,
};
use navmesh_rs::detour::NavMesh;
use navmesh_rs::{Budget, Progress, Workspace};

/// Installs `tiles` one per call (a budget of one unit) and prints the time per call.
fn install_row(grid: &GridLayout, tiles: Vec<TileOutput>) {
    let max_tiles = tiles.len().max(1) as i32;
    let mut nav = NavMesh::new(&grid.nav_mesh_params(max_tiles, 1 << 20)).unwrap();
    let bytes: usize = tiles
        .iter()
        .filter_map(|t| t.data.as_ref())
        .map(Vec::len)
        .sum();
    let mut installer = TileInstaller::new();
    for t in tiles {
        installer.push(t);
    }
    let mut micros = Vec::new();
    loop {
        let start = Instant::now();
        let progress = installer.install(&mut nav, &mut Budget::new(1)).unwrap();
        micros.push(start.elapsed().as_secs_f64() * 1e6);
        if progress == Progress::Done {
            break;
        }
    }
    micros.sort_by(f64::total_cmp);
    let n = micros.len();
    let total: f64 = micros.iter().sum();
    println!(
        "install, one tile per call: {} tiles ({:.1} KiB each on average), per call mean {:.0} us, p99 {:.0} us, max {:.0} us, {} calls over 1 ms, {:.1} us per KiB",
        installer.installed(),
        bytes as f64 / 1024.0 / installer.installed().max(1) as f64,
        total / n as f64,
        micros[(n * 99 / 100).min(n - 1)],
        micros[n - 1],
        micros.iter().filter(|&&m| m > 1000.0).count(),
        total / (bytes as f64 / 1024.0)
    );
}

fn peak_rss_mib() -> f64 {
    let status = std::fs::read_to_string("/proc/self/status").unwrap_or_default();
    status
        .lines()
        .find_map(|l| l.strip_prefix("VmHWM:"))
        .and_then(|v| v.split_whitespace().next()?.parse::<f64>().ok())
        .map_or(0.0, |kib| kib / 1024.0)
}

/// Runs `job` chunk by chunk, keeping counters and, when `keep` is given, the tiles.
fn run(job: &mut BakeJob<'_, World>, mut keep: Option<&mut Vec<TileOutput>>) -> (usize, usize) {
    let (mut tiles, mut bytes) = (0, 0);
    loop {
        let progress = job.run_parallel_chunk().unwrap();
        while let Some(t) = job.next_ready() {
            tiles += usize::from(t.data.is_some());
            bytes += t.data.as_ref().map_or(0, Vec::len);
            if let Some(k) = keep.as_deref_mut() {
                k.push(t);
            }
        }
        if progress == Progress::Done {
            return (tiles, bytes);
        }
    }
}

fn cfg() -> BakeConfig {
    let mut cfg = BakeConfig::default();
    cfg.recast.tile_size = 256;
    cfg
}

#[test]
#[ignore = "measurement; see the module documentation"]
fn large_world_bake_and_rebake_latency() {
    let km: f32 = std::env::var("NAV_WORLD_KM")
        .ok()
        .and_then(|v| v.parse().ok())
        .unwrap_or(4.0);
    let cfg = cfg();
    let base = World::new(km * 1000.0, 1);
    let grid = base.layout(&cfg);
    let total = grid.keys().len();
    println!(
        "world {km} km: {} x {} tiles of 256 cells",
        grid.tiles_x, grid.tiles_z
    );

    let start = Instant::now();
    let mut job = BakeJob::new(&base, &cfg, &grid, grid.keys(), JobOptions::default()).unwrap();
    let mut kept = Vec::new();
    // The install measurement below needs the tiles; at 16 km they are not kept.
    let keep = (km <= 4.0).then_some(&mut kept);
    let (non_empty, bytes) = run(&mut job, keep);
    let full = start.elapsed();
    let mut manifest = BakeManifest::new(&cfg, &grid).unwrap();
    manifest.apply(&job.manifest_delta()).unwrap();
    println!(
        "full bake: {:.1} s, {total} tiles ({non_empty} with polygons, {:.1} MiB of DNAV), peak RSS {:.0} MiB, threads {}",
        full.as_secs_f64(),
        bytes as f64 / 1048576.0,
        peak_rss_mib(),
        rayon::current_num_threads()
    );
    println!(
        "manifest: {} bytes for {} entries",
        manifest.to_bytes().len(),
        manifest.entries.len()
    );
    if !kept.is_empty() {
        install_row(&grid, kept);
    }

    // Longest single step: one tile in the middle, one stage per step.
    let mid = TileKey {
        x: grid.tiles_x / 2,
        z: grid.tiles_z / 2,
    };
    let mut job = BakeJob::new(&base, &cfg, &grid, vec![mid], JobOptions::default()).unwrap();
    let (mut longest, mut steps, mut sum) = (0.0f64, 0, 0.0f64);
    loop {
        let t = Instant::now();
        let progress = job.step(&mut Budget::new(1)).unwrap();
        let ms = t.elapsed().as_secs_f64() * 1e3;
        (longest, sum, steps) = (longest.max(ms), sum + ms, steps + 1);
        if progress == Progress::Done {
            break;
        }
    }
    println!("one 256-cell tile: {steps} steps, {sum:.1} ms total, longest step {longest:.1} ms");

    // Edits: one tile's worth, then regions of about 9 and 64 tiles.
    let extent = grid.tile_extent;
    let centre = [
        grid.tiles_x as f32 * extent / 2.0,
        grid.tiles_z as f32 * extent / 2.0,
    ];
    for (label, tiles_wide) in [
        ("one tile", 0.4f32),
        ("region ~9 tiles", 2.0),
        ("region ~64 tiles", 7.0),
    ] {
        let half = tiles_wide * extent / 2.0;
        let edit = Edit::Raise {
            min: [centre[0] - half, centre[1] - half],
            max: [centre[0] + half, centre[1] + half],
            dy: 2.0,
        };
        let edited = base.clone().with(edit);
        let start = Instant::now();
        let dirty = manifest
            .dirty_from_boxes(&cfg, &grid, &[edit.bounds()])
            .unwrap()
            .keys;
        let mut job =
            BakeJob::new(&edited, &cfg, &grid, dirty.clone(), JobOptions::default()).unwrap();
        job.skip_unchanged(&manifest, &mut Workspace::new())
            .unwrap();
        let skipped = job.skipped();
        run(&mut job, None);
        let ms = start.elapsed().as_secs_f64() * 1e3;
        let rebaked = dirty.len() - skipped;
        println!(
            "{label}: {} dirty, {skipped} skipped, {rebaked} rebaked, {ms:.0} ms ({:.0} ms per rebaked tile; {:.1}% of the full bake time for {:.1}% of the tiles)",
            dirty.len(),
            ms / rebaked.max(1) as f64,
            100.0 * ms / 1e3 / full.as_secs_f64(),
            100.0 * rebaked as f64 / total as f64
        );
    }
}

/// The motivating large world: 240 km across, centred on the origin, in 256 m sectors
/// (cell 0.5 m, 512-cell tiles). Bakes a spread of sample tiles, including the far
/// corners, and extrapolates the whole world from their mean cost (an estimate, labelled).
#[test]
#[ignore = "measurement; see the module documentation"]
fn huge_world_sample_row() {
    let mut cfg = BakeConfig::default();
    cfg.recast.cs = 0.5;
    cfg.recast.tile_size = 512;
    let world = World::centered(240_000.0, 1);
    let grid = world.layout(&cfg);
    let total = grid.tiles_x as u64 * grid.tiles_z as u64;
    let (x1, z1) = (grid.tiles_x - 1, grid.tiles_z - 1);
    let mut keys = vec![
        TileKey { x: 0, z: 0 },
        TileKey { x: x1, z: 0 },
        TileKey { x: 0, z: z1 },
        TileKey { x: x1, z: z1 },
        TileKey {
            x: x1 / 2,
            z: z1 / 2,
        },
    ];
    for k in 1..12 {
        keys.push(TileKey {
            x: (k * 7919) % grid.tiles_x,
            z: (k * 104729) % grid.tiles_z,
        });
    }
    let nav_params = grid.nav_mesh_params(keys.len() as i32, 1 << 20);
    let mut nav = NavMesh::new(&nav_params).unwrap();
    let (mut bake_ms, mut bytes, mut install_us) = (Vec::new(), 0usize, Vec::new());
    for key in &keys {
        let start = Instant::now();
        let mut job = BakeJob::new(&world, &cfg, &grid, vec![*key], JobOptions::default()).unwrap();
        while job.step(&mut Budget::unlimited()).unwrap() != Progress::Done {}
        bake_ms.push(start.elapsed().as_secs_f64() * 1e3);
        let tile = job.next_ready().unwrap();
        bytes += tile.data.as_ref().map_or(0, Vec::len);
        let mut installer = TileInstaller::new();
        installer.push(tile);
        let start = Instant::now();
        installer.install(&mut nav, &mut Budget::new(1)).unwrap();
        install_us.push(start.elapsed().as_secs_f64() * 1e6);
    }
    let mean = bake_ms.iter().sum::<f64>() / bake_ms.len() as f64;
    let worst = bake_ms.iter().cloned().fold(0.0, f64::max);
    println!(
        "240 km world: {} x {} sectors of {} m = {total} tiles; {} sample tiles: bake mean {mean:.0} ms, max {worst:.0} ms, {:.1} KiB DNAV each; install max {:.0} us",
        grid.tiles_x,
        grid.tiles_z,
        grid.tile_extent,
        keys.len(),
        bytes as f64 / 1024.0 / keys.len() as f64,
        install_us.iter().cloned().fold(0.0, f64::max)
    );
    println!(
        "estimate (not measured): full bake {:.1} CPU-hours, {:.1} h on 4 threads; manifest {:.1} MiB; tile keys {:.1} MiB",
        total as f64 * mean / 3.6e6,
        total as f64 * mean / 3.6e6 / 4.0,
        (60 + 25 * total) as f64 / 1048576.0,
        (total * 8) as f64 / 1048576.0
    );
    let manifest = BakeManifest::new(&cfg, &grid).unwrap();
    let start = Instant::now();
    let dirty = manifest
        .dirty_from_boxes(
            &cfg,
            &grid,
            &[[110_000.0, 0.0, 110_000.0, 110_100.0, 9.0, 110_100.0]],
        )
        .unwrap();
    println!(
        "dirty_from_boxes at the far corner: {} tiles in {:?}",
        dirty.keys.len(),
        start.elapsed()
    );
}
