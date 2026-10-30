#![no_main]
//! Source-only target: bounded serial/parallel tile identity, including errors.
use libfuzzer_sys::fuzz_target;
use navmesh_rs::bake::{self, BakeConfig, Input, Partition};
use navmesh_rs::{Limits, Workspace};
use std::sync::OnceLock;

fuzz_target!(|data: &[u8]| {
    if data.len() < 16 {
        return;
    }
    static POOL: OnceLock<rayon::ThreadPool> = OnceLock::new();
    let pool = POOL.get_or_init(|| {
        rayon::ThreadPoolBuilder::new()
            .num_threads(4)
            .build()
            .unwrap()
    });
    // Arbitrary float bit patterns also exercise non-finite/extreme bounds.
    let verts: Vec<f32> = data[4..]
        .as_chunks::<12>()
        .0
        .iter()
        .take(64)
        .flat_map(|v| {
            v.as_chunks::<4>().0.iter().map(|b| {
                if data[2] & 1 != 0 {
                    f32::from_bits(u32::from_le_bytes(*b))
                } else {
                    f32::from(b[0]) / 16.0
                }
            })
        })
        .collect();
    if verts.len() < 9 {
        return;
    }
    let nv = i32::try_from(verts.len() / 3).unwrap();
    let tris: Vec<i32> = data[4..]
        .as_chunks::<3>()
        .0
        .iter()
        .take(64)
        .flat_map(|t| {
            t.iter().map(|&v| {
                if data[2] & 2 != 0 {
                    i32::from(v) - 4
                } else {
                    i32::from(v) % nv
                }
            })
        })
        .collect();
    let input = Input::new(&verts, &tris);
    let mut cfg = BakeConfig::default();
    cfg.recast.tile_size = 8 + i32::from(data[0] % 57);
    cfg.partition = match data[1] % 3 {
        0 => Partition::Watershed,
        1 => Partition::Monotone,
        _ => Partition::Layers,
    };
    cfg.tile_limits.max_bytes = 4 << 20;
    cfg.tile_limits.max_work = 200_000;
    let setup = Limits {
        max_bytes: 4 << 20,
        max_work: 200_000,
        ..Limits::default()
    };
    let serial = bake::tiles(&input, &cfg, &mut Workspace::with_limits(setup)).map(|b| b.tiles);
    let parallel = pool
        .install(|| bake::tiles_parallel(&input, &cfg, &mut Workspace::with_limits(setup)))
        .map(|b| b.tiles);
    assert_eq!(serial, parallel);
});
