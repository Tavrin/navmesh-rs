#![no_main]
//! `PolyMesh::build_with_report`, default-compatible and strict, on arbitrary
//! contour sets: random (mostly self-intersecting) contours, convex contours,
//! contours with out-of-range coordinates and degenerate sizes.
//!
//! Layout: `[nvp selector, contour count]`, then per contour `[kind, vertex
//! count, region, area]` and its vertices (2 bytes each: random mode, or the
//! radius and phase of a convex contour).
//!
//! Properties: both modes agree on any hard error; strict mode fails with
//! `PartialTriangulation` exactly when the default mode reports an incomplete
//! contour, and then reports at least one failed contour and one missing
//! triangle, no more than the default mode counts; with no incomplete contour
//! strict mode returns the same mesh sizes.
use libfuzzer_sys::fuzz_target;
use navmesh_rs::recast::{Contour, ContourSet, PolyMesh};
use navmesh_rs::{Error, Workspace};

struct Bytes<'a>(&'a [u8]);

impl Bytes<'_> {
    fn u8(&mut self) -> u8 {
        let (&b, rest) = self.0.split_first().unwrap_or((&0, &[]));
        self.0 = rest;
        b
    }
}

fn contour(by: &mut Bytes) -> Contour {
    let kind = by.u8();
    let n = usize::from(by.u8() % 24);
    let reg = u16::from(by.u8());
    let area = by.u8() % 4;
    let mut verts = Vec::with_capacity(n * 4);
    for i in 0..n {
        let (a, b) = (by.u8(), by.u8());
        let (x, z) = match kind % 4 {
            // A convex contour: points on a circle (distinct angles).
            0 | 1 => {
                let ang = (i as f64) * std::f64::consts::TAU / (n as f64);
                let r = f64::from(a) + 3.0;
                (
                    (r * ang.cos() + 300.0) as i32,
                    (r * ang.sin() + 300.0) as i32 + i32::from(b & 1),
                )
            }
            // Random coordinates.
            2 => (i32::from(a), i32::from(b)),
            // Large and out-of-range coordinates.
            _ => (i32::from(a) * 300 - 4000, i32::from(b) * 300 - 4000),
        };
        verts.extend([x, i32::from(b >> 1), z, i32::from(reg)]);
    }
    Contour {
        rverts: verts.clone(),
        verts,
        reg,
        area,
    }
}

fn describe(e: &Error) -> std::mem::Discriminant<Error> {
    std::mem::discriminant(e)
}

fuzz_target!(|data: &[u8]| {
    if data.len() < 2 {
        return;
    }
    const NVP: [i32; 11] = [0, 2, 3, 4, 5, 6, 8, 12, 32, 64, 65];
    let nvp = NVP[usize::from(data[0]) % NVP.len()];
    let mut by = Bytes(&data[2..]);
    let conts: Vec<Contour> = (0..usize::from(data[1] % 6))
        .map(|_| contour(&mut by))
        .collect();
    let cset = ContourSet {
        conts,
        bmin: [0.0; 3],
        bmax: [1000.0; 3],
        cs: 1.0,
        ch: 1.0,
        width: 1000,
        height: 1000,
        border_size: 0,
        max_error: 1.3,
    };
    let mut ws = Workspace::new();
    let lax = PolyMesh::build_with_report(&mut ws, &cset, nvp, false);
    let strict = PolyMesh::build_with_report(&mut ws, &cset, nvp, true);
    match (lax, strict) {
        (Ok((m, r)), Ok((s, sr))) => {
            assert_eq!(
                r.failed_contours, 0,
                "strict accepted an incomplete contour"
            );
            assert_eq!(sr, r);
            assert_eq!(
                (m.npolys, m.nverts),
                (s.npolys, s.nverts),
                "mesh sizes differ"
            );
        }
        (
            Ok((_, r)),
            Err(Error::PartialTriangulation {
                failed_contours,
                missing_triangles,
            }),
        ) => {
            assert!(
                r.failed_contours > 0,
                "strict failed on a complete triangulation"
            );
            assert!(failed_contours >= 1 && failed_contours <= r.failed_contours);
            assert!(missing_triangles >= 1 && missing_triangles <= r.missing_triangles);
        }
        (Err(a), Err(b)) => assert_eq!(describe(&a), describe(&b), "{a:?} vs {b:?}"),
        (a, b) => panic!(
            "modes disagree: {:?} vs {:?}",
            a.map(|x| x.1),
            b.map(|x| x.1)
        ),
    }
});
