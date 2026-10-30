#![no_main]
// `Limits` is non-exhaustive, so its fields cannot be set in a struct expression here.
#![allow(clippy::field_reassign_with_default)]
//! Input layout (written by `seeds tilecache`):
//!
//! ```text
//! byte 0        mode: bit 0 = layer bytes are a grid specification (else a raw DTLR layer),
//!               bit 1 = a synthesised layer takes its connections from the bytes (else derived)
//! bytes 1..3    layer length, little-endian (clamped to the input)
//! layer bytes   raw layer, or the grid specification described in `synth_layer`
//! 16-byte ops   byte 0 = operation (mod 6), bytes 4..16 = three `f32` operands
//! ```
use libfuzzer_sys::fuzz_target;
use navmesh_rs::detour::{NavMesh, NavMeshParams};
use navmesh_rs::tilecache::*;
use navmesh_rs::{Limits, Workspace};

/// Builds an 8x8 layer from `spec`: byte 0 selects the tile slot (tx, ty, layer), bytes 1..65 are
/// heights, 65..129 areas (values of 192 and above are unwalkable), 129..193 raw connections.
fn synth_layer(ws: &mut Workspace, mode: u8, spec: &[u8]) -> Option<Vec<u8>> {
    let at = |i: usize| spec.get(i).copied().unwrap_or(0);
    let tx = i32::from(at(0) & 1);
    let ty = i32::from((at(0) >> 1) & 1);
    let mut h = LayerHeader::new(8, 8);
    h.tx = tx;
    h.ty = ty;
    h.tlayer = i32::from((at(0) >> 2) % 3);
    h.bmin = [tx as f32 * 8.0, 0.0, ty as f32 * 8.0];
    h.bmax = [h.bmin[0] + 8.0, 2.0, h.bmin[2] + 8.0];
    h.hmax = 255;
    let mut heights = [0u8; 64];
    let mut areas = [0u8; 64];
    for i in 0..64 {
        let a = at(65 + i);
        areas[i] = if a >= 192 { 0 } else { a & 63 };
        heights[i] = if areas[i] == 0 {
            at(1 + i)
        } else {
            at(1 + i).min(254)
        };
    }
    let mut cons = [0u8; 64];
    for i in 0..64 {
        let (x, y) = (i % 8, i / 8);
        if mode & 2 != 0 {
            let mut c = at(129 + i) & 15;
            if x == 0 {
                c &= !1;
            }
            if y == 7 {
                c &= !2;
            }
            if x == 7 {
                c &= !4;
            }
            if y == 0 {
                c &= !8;
            }
            cons[i] = c;
        } else if areas[i] != 0 {
            let near = |j: usize| areas[j] != 0 && heights[j].abs_diff(heights[i]) <= 4;
            cons[i] = u8::from(x > 0 && near(i - 1))
                | (u8::from(y < 7 && near(i + 8)) * 2)
                | (u8::from(x < 7 && near(i + 1)) * 4)
                | (u8::from(y > 0 && near(i - 8)) * 8);
        }
    }
    build_layer(ws, &RawCompressor, &h, &heights, &areas, &cons).ok()
}

fuzz_target!(|data: &[u8]| {
    if data.len() < 3 {
        return;
    }
    let mode = data[0];
    let len = usize::from(u16::from_le_bytes([data[1], data[2]])).min(data.len() - 3);
    let (spec, ops) = data[3..].split_at(len);
    let mut limits = Limits::default();
    limits.max_bytes = 8 * 1024 * 1024;
    limits.max_work = 2_000_000;
    limits.max_tile_bytes = 256 * 1024;
    let mut ws = Workspace::with_limits(limits);
    let fuzz_layer = if mode & 1 != 0 {
        synth_layer(&mut ws, mode, spec)
    } else {
        Some(spec.to_vec())
    };
    if let Some(bytes) = &fuzz_layer
        && let Ok(mut l) = decompress_layer(&mut ws, &RawCompressor, bytes)
    {
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
    // The fuzzed layer goes through the cache's own decode, then its rebuild path on update.
    if let Some(bytes) = fuzz_layer {
        let _ = cache.add_tile(bytes);
    }
    let mut refs = Vec::new();
    for op in ops.chunks(16).take(128) {
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
            4 => {
                let _ = cache.update(f(4), &mut nav);
            }
            _ => {
                // Drain: each update rebuilds at most one layer.
                for _ in 0..32 {
                    if !matches!(cache.update(0.0, &mut nav), Ok(false)) {
                        break;
                    }
                }
            }
        }
    }
    for _ in 0..64 {
        if !matches!(cache.update(0.0, &mut nav), Ok(false)) {
            break;
        }
    }
});
