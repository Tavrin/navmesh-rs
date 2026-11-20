use super::{MeshTile, Prism, empty_bounds, extend};
use crate::detour::Poly;

pub(super) fn overlaps(prism: &Prism<'_>, tile: &MeshTile, ip: usize, poly: &Poly) -> bool {
    let nv = usize::from(poly.vert_count);
    let vertex = |k: usize| tile.baked_vertex(usize::from(poly.verts[k]));
    if let Some(d) = tile.detail_meshes.get(ip).filter(|d| d.tri_count != 0) {
        let start = d.tri_base as usize * 4;
        let end = start + usize::from(d.tri_count) * 4;
        tile.detail_tris[start..end].chunks_exact(4).any(|tri| {
            overlaps_triangle(
                prism,
                [tri[0], tri[1], tri[2]].map(|k| {
                    let k = usize::from(k);
                    if k < nv {
                        vertex(k)
                    } else {
                        let at = (d.vert_base as usize + k - nv) * 3;
                        [
                            tile.detail_verts[at],
                            tile.detail_verts[at + 1],
                            tile.detail_verts[at + 2],
                        ]
                    }
                }),
            )
        })
    } else {
        (1..nv - 1).any(|i| overlaps_triangle(prism, [vertex(0), vertex(i), vertex(i + 1)]))
    }
}

fn overlaps_triangle(prism: &Prism<'_>, triangle: [[f32; 3]; 3]) -> bool {
    let mut bounds = empty_bounds();
    for vertex in triangle {
        extend(&mut bounds, vertex);
    }
    if !prism.bounds_overlap(bounds) {
        return false;
    }

    // Separating axis theorem for a triangle and a convex prism. Test both
    // shapes' face normals and the cross products of their edge directions.
    // Work relative to a triangle vertex and project in f64. Zero axes and
    // touching intervals overlap, including for a prism with zero height.
    let [a, b, c] = triangle.map(|v| v.map(f64::from));
    let ab = sub(b, a);
    let ac = sub(c, a);
    let edges = [ab, sub(c, b), ac];
    let up = [0.0, 1.0, 0.0];
    let span = core::array::from_fn(|i| {
        [
            bounds.0[i],
            bounds.1[i],
            prism.bounds.0[i],
            prism.bounds.1[i],
        ]
        .into_iter()
        .map(|value| (f64::from(value) - a[i]).abs())
        .fold(0.0, f64::max)
    });
    let overlaps_axis = |(axis, magnitude): ([f64; 3], [f64; 3])| {
        let p = dot(ab, axis);
        let q = dot(ac, axis);
        let tri_min = p.min(q).min(0.0);
        let tri_max = p.max(q).max(0.0);
        let mut xz_min = f64::INFINITY;
        let mut xz_max = f64::NEG_INFINITY;
        for &[x, z] in prism.vertices {
            let xz = (f64::from(x) - a[0]) * axis[0] + (f64::from(z) - a[2]) * axis[2];
            xz_min = xz_min.min(xz);
            xz_max = xz_max.max(xz);
        }
        let bottom = (f64::from(prism.bounds.0[1]) - a[1]) * axis[1];
        let top = (f64::from(prism.bounds.1[1]) - a[1]) * axis[1];
        // The absolute cross-product terms bound even a nearly cancelled
        // axis. Together with the relative spans they bound roundoff in both
        // projections, including coordinate differences and interval sums.
        let tolerance = 32.0 * f64::EPSILON * dot(span, magnitude);
        tri_min <= xz_max + bottom.max(top) + tolerance
            && tri_max >= xz_min + bottom.min(top) - tolerance
    };
    // The bounds check already covers the prism's horizontal faces.
    overlaps_axis(cross(ab, ac))
        && edges.iter().all(|&edge| overlaps_axis(cross(edge, up)))
        && prism.edges().all(|([x0, z0], [x1, z1])| {
            let along = [
                f64::from(x1) - f64::from(x0),
                0.0,
                f64::from(z1) - f64::from(z0),
            ];
            overlaps_axis(cross(up, along))
                && edges.iter().all(|&edge| overlaps_axis(cross(edge, along)))
        })
}

fn sub(a: [f64; 3], b: [f64; 3]) -> [f64; 3] {
    [a[0] - b[0], a[1] - b[1], a[2] - b[2]]
}

fn dot(a: [f64; 3], b: [f64; 3]) -> f64 {
    (a[0] * b[0] + a[2] * b[2]) + a[1] * b[1]
}

fn cross(a: [f64; 3], b: [f64; 3]) -> ([f64; 3], [f64; 3]) {
    let terms = [
        (a[1] * b[2], a[2] * b[1]),
        (a[2] * b[0], a[0] * b[2]),
        (a[0] * b[1], a[1] * b[0]),
    ];
    (
        terms.map(|(p, q)| p - q),
        terms.map(|(p, q)| p.abs() + q.abs()),
    )
}

#[cfg(test)]
mod tests {
    use super::*;

    type Point = [i128; 3];

    fn difference(a: Point, b: Point) -> Point {
        core::array::from_fn(|i| a[i] - b[i])
    }

    fn normal(a: Point, b: Point) -> Point {
        core::array::from_fn(|i| {
            let (j, k) = ((i + 1) % 3, (i + 2) % 3);
            a[j] * b[k] - a[k] * b[j]
        })
    }

    // All coordinates are integer numerators over the same power of two.
    // Project the actual box vertices and triangle vertices with exact i128
    // arithmetic. The common rational denominator cannot affect comparisons.
    fn exact_overlap(triangle: [Point; 3], lo: Point, hi: Point) -> bool {
        let corners: [Point; 8] = core::array::from_fn(|bits| {
            core::array::from_fn(|i| if bits & (1 << i) == 0 { lo[i] } else { hi[i] })
        });
        let edges =
            core::array::from_fn::<_, 3, _>(|i| difference(triangle[(i + 1) % 3], triangle[i]));
        let axes = [[1, 0, 0], [0, 1, 0], [0, 0, 1]];
        let intersects = |axis: Point| {
            let project = |point: Point| (0..3).map(|i| point[i] * axis[i]).sum::<i128>();
            let tri = triangle.map(project);
            let prism = corners.map(project);
            tri.iter().min() <= prism.iter().max() && prism.iter().min() <= tri.iter().max()
        };
        intersects(normal(edges[0], edges[1]))
            && axes.iter().all(|&axis| {
                intersects(axis) && edges.iter().all(|&edge| intersects(normal(axis, edge)))
            })
    }

    #[test]
    fn sloped_face_contact_agrees_with_exact_rational_sat() {
        let mut seed = 0x276e_8e5b_u32;
        let mut component = || {
            seed = seed.wrapping_mul(1664525).wrapping_add(1013904223);
            4 * (i128::from(seed >> 11) - (1 << 20))
        };
        for case in 0..128 {
            let origin: Point = core::array::from_fn(|_| component());
            let u: Point = core::array::from_fn(|_| component());
            let v: Point = core::array::from_fn(|_| component());
            let triangle = [
                origin,
                core::array::from_fn(|i| origin[i] + u[i]),
                core::array::from_fn(|i| origin[i] + v[i]),
            ];
            // A prism vertex lies strictly inside the triangle, at barycentric
            // coordinates (1/2, 1/4, 1/4). All other prism vertices lie on the
            // outward side of the sloped face.
            let contact: Point = core::array::from_fn(|i| origin[i] + (u[i] + v[i]) / 4);
            let face = normal(u, v);
            assert!(face.iter().all(|&x| x != 0));
            let scale = [1.0 / 4096.0, 1.0, 256.0][case % 3];
            let f = |x: i128| x as f32 * scale;
            for gap in [0, 1] {
                let near: Point = core::array::from_fn(|i| contact[i] + face[i].signum() * gap);
                let far: Point = core::array::from_fn(|i| near[i] + face[i].signum() * 16);
                let lo: Point = core::array::from_fn(|i| near[i].min(far[i]));
                let hi: Point = core::array::from_fn(|i| near[i].max(far[i]));
                let outline = [
                    [f(lo[0]), f(lo[2])],
                    [f(hi[0]), f(lo[2])],
                    [f(hi[0]), f(hi[2])],
                    [f(lo[0]), f(hi[2])],
                ];
                let prism = Prism::new(&outline, f(lo[1]), f(hi[1]), None);
                let expected = exact_overlap(triangle, lo, hi);
                assert_eq!(expected, gap == 0, "oracle case {case}");
                assert_eq!(
                    overlaps_triangle(&prism, triangle.map(|p| p.map(f))),
                    expected,
                    "case {case}, gap {gap}"
                );
            }
        }
    }
}
