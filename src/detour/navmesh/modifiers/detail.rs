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
    let overlaps_axis = |axis: [f64; 3]| {
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
        tri_min <= xz_max + bottom.max(top) && tri_max >= xz_min + bottom.min(top)
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

fn cross(a: [f64; 3], b: [f64; 3]) -> [f64; 3] {
    [
        a[1] * b[2] - a[2] * b[1],
        a[2] * b[0] - a[0] * b[2],
        a[0] * b[1] - a[1] * b[0],
    ]
}
