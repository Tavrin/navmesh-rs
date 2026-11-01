#![no_main]
//! World-space float tile import (`create_nav_mesh_data_f32`) with arbitrary
//! vertices, polygons, bounds, cell sizes and off-mesh connections, then a
//! single-tile navmesh and queries on whatever the importer accepted.
//!
//! Layout: 8 header bytes `[flags, vertex count, polygon count, nvp, cs, ch,
//! areas/detail, off-mesh count]`, then a byte stream the target consumes for the
//! six bounds, every vertex coordinate, polygon indices, flags, areas, off-mesh
//! connections and detail data (missing bytes read as 0).
//!
//! * flags bit 0: BV tree, bit 1: raw `f32` bit patterns for coordinates (NaN,
//!   infinities, huge values and denormals appear freely); otherwise a tag byte
//!   picks a special value (NaN, +-inf, f32::MAX/MIN, -0.0, a denormal, 1e30) or
//!   a point inside the bounds, so accepted tiles are common
//! * flags bit 2: bounds are raw `f32` too, bit 3: omit one polygon-array element
//!   (an inconsistent count), bit 4: add a detail mesh, bit 5: shift one vertex
//!   outside the bounds
//!
//! Properties: no panic or hang for any input; an accepted tile loads into a
//! navmesh, keeps every vertex float bit for bit, and answers queries.
use libfuzzer_sys::fuzz_target;
use navmesh_fuzz::exercise_queries;
use navmesh_rs::detour::{
    DefaultFilter, NavMesh, NavMeshCreateParams, NavMeshQuery, create_nav_mesh_data_f32,
};

struct Bytes<'a>(&'a [u8]);

impl Bytes<'_> {
    fn u8(&mut self) -> u8 {
        let (&b, rest) = self.0.split_first().unwrap_or((&0, &[]));
        self.0 = rest;
        b
    }
    fn u16(&mut self) -> u16 {
        u16::from_le_bytes([self.u8(), self.u8()])
    }
    fn raw(&mut self) -> f32 {
        f32::from_le_bytes([self.u8(), self.u8(), self.u8(), self.u8()])
    }
    /// A coordinate inside `lo..=hi`, or one of the special values.
    fn coord(&mut self, raw: bool, lo: f32, hi: f32) -> f32 {
        if raw {
            return self.raw();
        }
        const SPECIAL: [f32; 8] = [
            f32::NAN,
            f32::INFINITY,
            f32::NEG_INFINITY,
            f32::MAX,
            f32::MIN,
            -0.0,
            1e-45,
            1e30,
        ];
        let tag = self.u8();
        if tag < 8 {
            return SPECIAL[tag as usize];
        }
        let t = f32::from(self.u8()) / 255.0;
        lo + (hi - lo) * t
    }
}

fn size(b: u8) -> f32 {
    const SIZES: [f32; 12] = [
        1.0,
        0.5,
        0.3,
        0.01,
        2.0,
        1e-30,
        1e30,
        0.0,
        -1.0,
        f32::NAN,
        f32::INFINITY,
        0.0001,
    ];
    SIZES[usize::from(b) % SIZES.len()]
}

fuzz_target!(|data: &[u8]| {
    if data.len() < 8 {
        return;
    }
    let (h, rest) = data.split_at(8);
    let mut by = Bytes(rest);
    let flags = h[0];
    let (raw, raw_bounds) = (flags & 2 != 0, flags & 4 != 0);
    let vert_count = usize::from(h[1] % 13);
    let poly_count = usize::from(h[2] % 7);
    let nvp = 3 + usize::from(h[3] % 4);
    let (cs, ch) = (size(h[4]), size(h[5]));

    let mut bounds = [0.0f32; 6];
    for (k, b) in bounds.iter_mut().enumerate() {
        *b = if raw_bounds {
            by.raw()
        } else {
            // A tile of up to 255 x 255 x 255 around the origin, min below max.
            let v = f32::from(by.u8());
            if k < 3 { v - 128.0 } else { v }
        };
    }
    let (bmin, bmax) = (
        [bounds[0], bounds[1], bounds[2]],
        [bounds[3], bounds[4], bounds[5]],
    );
    let (lo, hi) = (
        [
            bmin[0].min(bmax[0]),
            bmin[1].min(bmax[1]),
            bmin[2].min(bmax[2]),
        ],
        [
            bmin[0].max(bmax[0]),
            bmin[1].max(bmax[1]),
            bmin[2].max(bmax[2]),
        ],
    );

    let mut verts = Vec::with_capacity(vert_count * 3);
    for _ in 0..vert_count {
        for k in 0..3 {
            verts.push(by.coord(raw, lo[k], hi[k]));
        }
    }
    if flags & 0x20 != 0 && !verts.is_empty() {
        let i = usize::from(by.u8()) % verts.len();
        verts[i] = hi[i % 3] + 1.0;
    }

    const N: u16 = 0xffff;
    let mut polys = Vec::with_capacity(poly_count * nvp * 2);
    for _ in 0..poly_count {
        let nv = 3 + usize::from(by.u8()) % (nvp - 2);
        for j in 0..nvp {
            let b = by.u16();
            polys.push(if j < nv {
                b % (vert_count as u16 + 1)
            } else {
                N
            });
        }
        for _ in 0..nvp {
            polys.push(if by.u8() & 3 == 0 {
                by.u16() % (poly_count as u16 + 1)
            } else {
                N
            });
        }
    }
    if flags & 8 != 0 {
        polys.pop();
    }
    let poly_flags: Vec<u16> = (0..poly_count).map(|_| by.u16()).collect();
    let area_mode = h[6];
    let poly_areas: Vec<u8> = (0..poly_count)
        .map(|_| {
            if area_mode & 1 != 0 {
                by.u8()
            } else {
                by.u8() & 63
            }
        })
        .collect();

    let off = usize::from(h[7] % 3);
    let mut con_verts = Vec::new();
    for _ in 0..off * 6 {
        con_verts.push(by.coord(raw, lo[0] - 1.0, hi[0] + 1.0));
    }
    let con_rad: Vec<f32> = (0..off).map(|_| by.coord(raw, -1.0, 4.0)).collect();
    let con_flags: Vec<u16> = (0..off).map(|_| by.u16()).collect();
    let con_areas: Vec<u8> = (0..off).map(|_| by.u8() & 63).collect();
    let con_dir: Vec<u8> = (0..off).map(|_| by.u8() & 1).collect();

    let (mut dmeshes, mut dverts, mut dtris) = (Vec::new(), Vec::new(), Vec::new());
    let (mut dv_count, mut dt_count) = (0, 0);
    if flags & 0x10 != 0 {
        let mut vbase = 0u32;
        let mut tbase = 0u32;
        for _ in 0..poly_count {
            let nv = u32::from(by.u8() % 4) + 3;
            let nt = u32::from(by.u8() % 3) + 1;
            dmeshes.extend([vbase, nv, tbase, nt]);
            for _ in 0..nv * 3 {
                dverts.push(by.coord(raw, lo[1], hi[1]));
            }
            for _ in 0..nt {
                dtris.extend([
                    by.u8() % (nv as u8 + 1),
                    by.u8() % (nv as u8 + 1),
                    by.u8() % (nv as u8 + 1),
                    by.u8(),
                ]);
            }
            vbase += nv;
            tbase += nt;
        }
        dv_count = vbase as i32;
        dt_count = tbase as i32;
    }

    let params = NavMeshCreateParams {
        verts: &[],
        vert_count: vert_count as i32,
        polys: &polys,
        poly_flags: &poly_flags,
        poly_areas: &poly_areas,
        poly_count: poly_count as i32,
        nvp: nvp as i32,
        detail_meshes: &dmeshes,
        detail_verts: &dverts,
        detail_verts_count: dv_count,
        detail_tris: &dtris,
        detail_tri_count: dt_count,
        off_mesh_con_verts: &con_verts,
        off_mesh_con_rad: &con_rad,
        off_mesh_con_flags: &con_flags,
        off_mesh_con_areas: &con_areas,
        off_mesh_con_dir: &con_dir,
        off_mesh_con_count: off as i32,
        bmin,
        bmax,
        walkable_height: 2.0,
        walkable_radius: 0.5,
        walkable_climb: 0.5,
        cs,
        ch,
        build_bv_tree: flags & 1 != 0,
        ..Default::default()
    };
    let Ok(tile) = create_nav_mesh_data_f32(&params, &verts) else {
        return;
    };
    let Ok((nav, _)) = NavMesh::new_single(tile) else {
        return;
    };
    exercise_queries(&nav);
    // Float vertices are stored verbatim.
    if let Some(t) = (0..nav.max_tiles() as usize).find_map(|i| nav.tile(i)) {
        let stored: Vec<u32> = t.verts.iter().map(|v| v.to_bits()).collect();
        let given: Vec<u32> = verts.iter().map(|v| v.to_bits()).collect();
        assert!(
            stored.starts_with(&given) || stored == given,
            "stored vertices differ"
        );
    }
    if let Ok(mut q) = NavMeshQuery::new(&nav, 64) {
        let f = DefaultFilter::default();
        let c = [0, 1, 2].map(|k| (lo[k] + hi[k]) * 0.5);
        if let Ok((r, p, _)) = q.find_nearest_poly(&c, &[1e4; 3], &f) {
            if r != 0 {
                let mut path = Vec::new();
                let _ = q.find_path_with_heuristic_scale(r, r, &p, &p, &f, &mut path, 8, 0.0);
            }
        }
    }
});
