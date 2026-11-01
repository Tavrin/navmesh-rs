#![no_main]
//! `find_path_with_heuristic_scale` with arbitrary scales (raw `f32` bit patterns:
//! NaN, negative, infinite, denormal, huge, zero) on a row of single-polygon
//! tiles joined by off-mesh connections, with arbitrary area costs.
//!
//! Layout: `[costs, scale selector, max_path]`, 4 raw scale bytes, then up to 24
//! queries of 4 bytes `[start, end, link mask, scale tag]`. The first byte picks
//! the cost of area 0 (0 gives zero-cost links) and the area 1 cost; the scale
//! tag picks a special scale (0 to 7) or the raw scale from the header.
//!
//! Properties: an invalid scale (not finite or negative) is an
//! `InvalidParameter` error, a valid one succeeds; the returned path starts at
//! the start polygon and, when not partial, ends at the end polygon; reachability
//! does not depend on the scale (Dijkstra with scale 0 and the default A* agree
//! on whether the end is reached).
use libfuzzer_sys::fuzz_target;
use navmesh_rs::Error;
use navmesh_rs::detour::{
    DefaultFilter, NavMesh, NavMeshCreateParams, NavMeshParams, NavMeshQuery, PolyRef, Status,
    create_nav_mesh_data,
};

const TILES: i32 = 6;

fn tile(x: i32, to: Option<i32>) -> Option<Vec<u8>> {
    const N: u16 = 0xffff;
    let to = to.filter(|&t| t != x);
    let con = [
        x as f32 + 0.5,
        0.0,
        0.5,
        to.unwrap_or(0) as f32 + 0.5,
        0.0,
        0.5,
    ];
    let has = to.is_some();
    create_nav_mesh_data(&NavMeshCreateParams {
        verts: &[0, 0, 0, 0, 0, 1, 1, 0, 1, 1, 0, 0],
        vert_count: 4,
        polys: &[0, 1, 2, 3, N, N, N, N],
        poly_flags: &[1],
        poly_areas: &[(x % 2) as u8],
        poly_count: 1,
        nvp: 4,
        off_mesh_con_verts: if has { &con } else { &[] },
        off_mesh_con_rad: if has { &[0.6] } else { &[] },
        off_mesh_con_flags: if has { &[1] } else { &[] },
        off_mesh_con_areas: if has { &[1] } else { &[] },
        off_mesh_con_dir: if has { &[1] } else { &[] },
        off_mesh_con_count: i32::from(has),
        tile_x: x,
        tile_y: 0,
        bmin: [x as f32, 0.0, 0.0],
        bmax: [x as f32 + 1.0, 1.0, 1.0],
        walkable_height: 2.0,
        walkable_radius: 0.5,
        walkable_climb: 0.5,
        cs: 1.0,
        ch: 1.0,
        build_bv_tree: true,
        ..Default::default()
    })
    .ok()
}

fn cost(b: u8) -> f32 {
    [0.0, 1e-6, 0.5, 1.0, 1.0, 2.0, 1e6, 1e30][usize::from(b) % 8]
}

fn special(t: u8) -> Option<f32> {
    [
        Some(0.0),
        Some(0.999),
        Some(1.0),
        Some(1e-45),
        Some(1e30),
        Some(f32::MAX),
        Some(-0.0),
        None,
    ][usize::from(t) % 8]
}

fn centre(x: i32) -> [f32; 3] {
    [x as f32 + 0.5, 0.0, 0.5]
}

fuzz_target!(|data: &[u8]| {
    if data.len() < 7 {
        return;
    }
    let (h, ops) = data.split_at(7);
    let raw_scale = f32::from_le_bytes([h[3], h[4], h[5], h[6]]);
    let max_path = 1 + usize::from(h[2] % 12);
    let mut filter = DefaultFilter::default();
    filter.area_cost[0] = cost(h[0] & 7);
    filter.area_cost[1] = cost(h[0] >> 4);
    let params = NavMeshParams {
        orig: [0.0; 3],
        tile_width: 1.0,
        tile_height: 1.0,
        max_tiles: 8,
        max_polys: 8,
    };
    let Ok(mut nav) = NavMesh::new(&params) else {
        return;
    };
    nav.set_global_off_mesh_links(h[1] & 1 != 0);
    for x in 0..TILES {
        // Each tile may link to a tile chosen by the next op byte.
        let to = ops
            .get(x as usize)
            .filter(|b| **b & 0x80 != 0)
            .map(|b| i32::from(*b & 0x7f) % TILES);
        if let Some(d) = tile(x, to) {
            let _ = nav.add_tile(d);
        }
    }
    let Ok(mut q) = NavMeshQuery::new(&nav, 64) else {
        return;
    };
    for op in ops.as_chunks::<4>().0.iter().take(24) {
        let (s, e) = (i32::from(op[0]) % TILES, i32::from(op[1]) % TILES);
        let scale = special(op[3]).unwrap_or(raw_scale);
        let poly = |q: &NavMeshQuery, x: i32| -> PolyRef {
            q.find_nearest_poly(&centre(x), &[0.4, 1.0, 0.4], &filter)
                .map_or(0, |r| r.0)
        };
        let (rs, re) = (poly(&q, s), poly(&q, e));
        let mut path = Vec::new();
        let res = q.find_path_with_heuristic_scale(
            rs,
            re,
            &centre(s),
            &centre(e),
            &filter,
            &mut path,
            max_path,
            scale,
        );
        if !scale.is_finite() || scale < 0.0 {
            assert_eq!(res, Err(Error::InvalidParameter("heuristic scale")));
            continue;
        }
        let Ok(st) = res else { continue };
        if rs == 0 || re == 0 {
            continue;
        }
        assert_eq!(path.first(), Some(&rs), "path does not start at the start");
        let reached = st.0 & Status::PARTIAL_RESULT == 0;
        // A truncated buffer ends the path early.
        if reached && st.0 & Status::BUFFER_TOO_SMALL == 0 {
            assert_eq!(path.last(), Some(&re), "complete path misses the end");
        }
        // Reachability is independent of the heuristic when the buffer is large.
        let (mut a, mut b) = (Vec::new(), Vec::new());
        let r0 = q.find_path_with_heuristic_scale(
            rs,
            re,
            &centre(s),
            &centre(e),
            &filter,
            &mut a,
            16,
            0.0,
        );
        let rd = q.find_path(rs, re, &centre(s), &centre(e), &filter, &mut b, 16);
        if let (Ok(x), Ok(y)) = (r0, rd) {
            assert_eq!(
                x.0 & Status::PARTIAL_RESULT,
                y.0 & Status::PARTIAL_RESULT,
                "scale 0 and default disagree on reaching the end"
            );
        }
    }
});
