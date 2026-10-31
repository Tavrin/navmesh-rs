#![no_main]
#![allow(clippy::field_reassign_with_default)]
//! Bounded serial/parallel tile identity, including errors.
//!
//! Layout: `[0]` tile size `8 + b % 57`; `[1]` partition (`b % 3`), agent radius
//! (`(b >> 2) & 7` cells) and vertices per polygon (`3 + (b >> 5) % 4`); `[2]` bit 0 raw `f32`
//! vertices, bit 1 raw triangle indices, bits 2-3 cell size (0.3, 0.15, 0.1, 0.5); `[3]`
//! features: bit 0 switches the three span filters off, bit 1 gives triangle areas, bits 2-3
//! count the area volumes, bits 4-5 pick the aggregate budget (none, `k` MiB, `k` x 256 KiB,
//! `usize::MAX - k`), bit 6 sets tight per-tile limits from `[5]`, bit 7 moves the volume
//! polygon to the second vertex; `[4]` budget parameter `k`; `[5]` volume area and tight-limit
//! parameter; `[6]` vertex count; then the vertices (12 bytes each when raw, otherwise three
//! bytes scaled to 0..32, 0..8, 0..32) and the triangles, three bytes each (at most 128).
use libfuzzer_sys::fuzz_target;
use navmesh_rs::Error;
use navmesh_rs::bake::{self, AreaVolume, BakeConfig, Input, Partition};
use navmesh_rs::{Limits, Workspace};
use std::sync::OnceLock;

fuzz_target!(|data: &[u8]| {
    if data.len() < 8 {
        return;
    }
    static POOL: OnceLock<rayon::ThreadPool> = OnceLock::new();
    let pool = POOL.get_or_init(|| {
        rayon::ThreadPoolBuilder::new()
            .num_threads(4)
            .build()
            .unwrap()
    });
    let (raw_verts, raw_tris) = (data[2] & 1 != 0, data[2] & 2 != 0);
    let stride = if raw_verts { 12 } else { 3 };
    let body = &data[7..];
    let nv = usize::from(data[6]).min(body.len() / stride);
    if nv < 3 {
        return;
    }
    // Arbitrary float bit patterns also exercise non-finite/extreme bounds.
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
            .map(|(k, &b)| f32::from(b) / if k % 3 == 1 { 32.0 } else { 8.0 })
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
    // Volumes reuse four input vertices as their polygon.
    let first = usize::from(features >> 7).min(nv.saturating_sub(4));
    let poly = &verts[first * 3..(first + 4).min(nv) * 3];
    let (hmin, hmax) = poly
        .as_chunks::<3>()
        .0
        .iter()
        .fold((f32::INFINITY, f32::NEG_INFINITY), |(lo, hi), p| {
            (lo.min(p[1]), hi.max(p[1]))
        });
    // `AreaVolume` is non-exhaustive, so it cannot be built with a struct expression.
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
    cfg.tile_limits.max_bytes = 4 << 20;
    cfg.tile_limits.max_work = 200_000;
    if features & 0x40 != 0 {
        cfg.tile_limits.max_bytes = (1 + usize::from(data[5] % 64)) * (64 << 10);
        cfg.tile_limits.max_work = 2_000 * (1 + u64::from(data[5] / 64));
    }
    let k = usize::from(data[4]);
    cfg.max_total_bytes = match (features >> 4) & 3 {
        0 => usize::MAX,
        1 => k << 20,
        2 => k << 18,
        _ => usize::MAX - k,
    };
    let setup = Limits {
        max_bytes: 4 << 20,
        max_work: 200_000,
        ..Limits::default()
    };
    let serial = bake::tiles(&input, &cfg, &mut Workspace::with_limits(setup)).map(|b| b.tiles);
    let parallel = pool
        .install(|| bake::tiles_parallel(&input, &cfg, &mut Workspace::with_limits(setup)))
        .map(|b| b.tiles);
    // The aggregate reservation counts one worker serially and the pool size in parallel, so it
    // may reject only the parallel bake (an error without a tile location).
    if cfg.max_total_bytes != usize::MAX
        && serial != parallel
        && matches!(&parallel, Err(e) if e.tile.is_none() && e.error == Error::LimitExceeded)
    {
        return;
    }
    assert_eq!(serial, parallel);
});
