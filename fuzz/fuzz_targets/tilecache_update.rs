#![no_main]
use libfuzzer_sys::fuzz_target;
use navmesh_rs::detour::{NavMesh, NavMeshParams};
use navmesh_rs::tilecache::*;
use navmesh_rs::{Limits, Workspace};
fuzz_target!(|bytes: &[u8]| {
    let mut limits = Limits::default();
    limits.max_bytes = 8 * 1024 * 1024;
    limits.max_work = 2_000_000;
    limits.max_tile_bytes = 256 * 1024;
    let mut ws = Workspace::with_limits(limits);
    if let Ok(mut l) = decompress_layer(&mut ws, &RawCompressor, bytes) {
        let _ = build_regions(&mut ws, &mut l, 4);
        if let Ok(c) = build_contours(&mut ws, &l, 4, 1.3) {
            let _ = build_poly_mesh(&mut ws, &c);
        }
    }
    let params = TileCacheParams::new(8, 8, 1.0, 0.2, 8, 16);
    let Ok(mut cache) = TileCache::new(params, RawCompressor, DefaultMeshProcess, limits) else {
        return;
    };
    let Ok(mut nav) = NavMesh::new(&NavMeshParams {
        orig: [0.0; 3],
        tile_width: 8.0,
        tile_height: 8.0,
        max_tiles: 8,
        max_polys: 256,
    }) else {
        return;
    };
    let h = LayerHeader::new(8, 8);
    let mut con = [0u8; 64];
    for y in 0..8 {
        for x in 0..8 {
            con[x + y * 8] = u8::from(x > 0)
                | (u8::from(y < 7) * 2)
                | (u8::from(x < 7) * 4)
                | (u8::from(y > 0) * 8);
        }
    }
    if let Ok(d) = build_layer(&mut ws, &RawCompressor, &h, &[0; 64], &[63; 64], &con) {
        let _ = cache.add_tile(d);
    }
    let mut refs = Vec::new();
    for op in bytes.chunks(16).take(128) {
        if op.len() < 16 {
            break;
        }
        let f = |i| f32::from_le_bytes(op[i..i + 4].try_into().unwrap());
        match op[0] % 6 {
            0 => {
                if let Ok(r) = cache.add_obstacle(ObstacleShape::Cylinder {
                    pos: [f(4), 0.0, f(8)],
                    radius: f(12),
                    height: 3.0,
                }) {
                    refs.push(r);
                }
            }
            1 => {
                if let Ok(r) = cache.add_obstacle(ObstacleShape::Box {
                    bmin: [f(4), 0.0, f(8)],
                    bmax: [f(8), 3.0, f(12)],
                }) {
                    refs.push(r);
                }
            }
            2 => {
                if let Ok(r) = cache.add_obstacle(ObstacleShape::OrientedBox {
                    center: [f(4), 1.0, f(8)],
                    half_extents: [1.0; 3],
                    yaw: f(12),
                }) {
                    refs.push(r);
                }
            }
            3 => {
                if let Some(&r) = refs.get(usize::from(op[1]) % refs.len().max(1)) {
                    let _ = cache.remove_obstacle(r);
                }
            }
            _ => {
                let _ = cache.update(f(4), &mut nav);
            }
        }
    }
});
