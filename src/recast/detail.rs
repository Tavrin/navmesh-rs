//! Detail mesh (upstream `RecastMeshDetail.cpp`).

use alloc::vec::Vec;

use super::area::check_chf;
use super::{
    CompactHeightfield, MESH_NULL_IDX, MULTIPLE_REGS, NOT_CONNECTED, PolyMesh, dir_for_offset,
    dir_offset_x, dir_offset_y,
};
use crate::common::{ceilf, clamp, floorf, fmax, fmin, sqr, sqrtf, vmax, vmin};
use crate::{Error, Workspace};

/// Height detail per polygon (`rcPolyMeshDetail`).
#[derive(Clone, Debug, Default)]
pub struct PolyMeshDetail {
    /// Per polygon: vertex base, vertex count, triangle base, triangle count.
    pub meshes: Vec<u32>,
    /// Vertices, x, y, z in world units.
    pub verts: Vec<f32>,
    /// Triangles: three vertex indices and edge flags.
    pub tris: Vec<u8>,
}

impl PolyMeshDetail {
    /// Builds the detail mesh: per polygon, a triangulation that follows the
    /// heightfield within `sample_max_error` (`rcBuildPolyMeshDetail`).
    pub fn build(
        ws: &mut Workspace,
        mesh: &PolyMesh,
        chf: &CompactHeightfield,
        sample_dist: f32,
        sample_max_error: f32,
    ) -> Result<PolyMeshDetail, Error> {
        ws.begin();
        check_chf(chf)?;
        let mut dmesh = PolyMeshDetail::default();
        if mesh.nverts == 0 || mesh.npolys == 0 {
            return Ok(dmesh);
        }
        let nvp = mesh.nvp as usize;
        let npolys = mesh.npolys as usize;
        let bs_ok = mesh.border_size >= 0
            && mesh.verts[..mesh.nverts as usize * 3]
                .chunks_exact(3)
                .all(|v| {
                    i32::from(v[0]) + 2 * mesh.border_size <= chf.width
                        && i32::from(v[2]) + 2 * mesh.border_size <= chf.height
                });
        if !(3..=126).contains(&nvp)
            || !bs_ok
            || mesh.polys.len() < npolys * nvp * 2
            || mesh.verts.len() < mesh.nverts as usize * 3
            || mesh.regs.len() < npolys
            || mesh.polys[..npolys * nvp * 2]
                .chunks_exact(nvp * 2)
                .flat_map(|p| &p[..nvp])
                .any(|&v| v != MESH_NULL_IDX && i32::from(v) >= mesh.nverts)
        {
            return Err(Error::InvalidInput("inconsistent polygon mesh"));
        }
        let cs = mesh.cs;
        let ch = mesh.ch;
        let orig = mesh.bmin;
        let bs = mesh.border_size;
        let search_radius = 1.max(ceilf(mesh.max_edge_error) as i32);
        let mut edges: Vec<i32> = Vec::new();
        let mut tris: Vec<i32> = Vec::new();
        let mut arr: Vec<i32> = Vec::new();
        let mut samples: Vec<i32> = Vec::new();
        let mut verts = [0.0f32; 256 * 3];
        let mut npoly_verts = 0usize;
        let (mut maxhw, mut maxhh) = (0i32, 0i32);
        let mut bounds = ws.vec(npolys * 4, 0i32)?;
        let mut poly = ws.vec(nvp * 3, 0.0f32)?;
        for i in 0..npolys {
            let p = &mesh.polys[i * nvp * 2..];
            let (mut xmin, mut xmax, mut ymin, mut ymax) = (chf.width, 0, chf.height, 0);
            for &pj in &p[..nvp] {
                if pj == MESH_NULL_IDX {
                    break;
                }
                let v = &mesh.verts[usize::from(pj) * 3..];
                xmin = xmin.min(i32::from(v[0]));
                xmax = xmax.max(i32::from(v[0]));
                ymin = ymin.min(i32::from(v[2]));
                ymax = ymax.max(i32::from(v[2]));
                npoly_verts += 1;
            }
            xmin = 0.max(xmin - 1);
            xmax = chf.width.min(xmax + 1);
            ymin = 0.max(ymin - 1);
            ymax = chf.height.min(ymax + 1);
            bounds[i * 4..i * 4 + 4].copy_from_slice(&[xmin, xmax, ymin, ymax]);
            if xmin >= xmax || ymin >= ymax {
                continue;
            }
            maxhw = maxhw.max(xmax - xmin);
            maxhh = maxhh.max(ymax - ymin);
        }
        let mut hp = HeightPatch {
            data: ws.vec((maxhw as usize) * (maxhh as usize), 0u16)?,
            xmin: 0,
            ymin: 0,
            width: 0,
            height: 0,
        };
        dmesh.meshes = ws.vec(npolys * 4, 0u32)?;
        let vcap = npoly_verts + npoly_verts / 2;
        dmesh.verts = ws.vec_with_capacity(vcap * 3)?;
        dmesh.tris = ws.vec_with_capacity(vcap * 2 * 4)?;
        for i in 0..npolys {
            let p = &mesh.polys[i * nvp * 2..i * nvp * 2 + nvp];
            let mut npoly = 0usize;
            for &pj in p {
                if pj == MESH_NULL_IDX {
                    break;
                }
                let v = &mesh.verts[usize::from(pj) * 3..];
                poly[npoly * 3] = f32::from(v[0]) * cs;
                poly[npoly * 3 + 1] = f32::from(v[1]) * ch;
                poly[npoly * 3 + 2] = f32::from(v[2]) * cs;
                npoly += 1;
            }
            hp.xmin = bounds[i * 4];
            hp.ymin = bounds[i * 4 + 2];
            hp.width = bounds[i * 4 + 1] - bounds[i * 4];
            hp.height = bounds[i * 4 + 3] - bounds[i * 4 + 2];
            ws.work((hp.width.max(0) as u64) * (hp.height.max(0) as u64))?;
            get_height_data(
                chf,
                &p[..npoly],
                &mesh.verts,
                bs,
                &mut hp,
                &mut arr,
                mesh.regs[i],
            );
            let nverts = build_poly_detail(
                ws,
                &poly[..npoly * 3],
                sample_dist,
                sample_max_error,
                search_radius,
                chf,
                &hp,
                &mut verts,
                &mut tris,
                &mut edges,
                &mut samples,
            )?;
            for v in verts[..nverts * 3].chunks_exact_mut(3) {
                v[0] += orig[0];
                v[1] += orig[1] + chf.ch;
                v[2] += orig[2];
            }
            let ntris = tris.len() / 4;
            let base_v = dmesh.verts.len() / 3;
            let base_t = dmesh.tris.len() / 4;
            dmesh.meshes[i * 4..i * 4 + 4].copy_from_slice(&[
                base_v as u32,
                nverts as u32,
                base_t as u32,
                ntris as u32,
            ]);
            ws.reserve(&mut dmesh.verts, nverts * 3)?;
            dmesh.verts.extend_from_slice(&verts[..nverts * 3]);
            ws.reserve(&mut dmesh.tris, ntris * 4)?;
            dmesh.tris.extend(tris.iter().map(|&t| t as u8));
        }
        Ok(dmesh)
    }
}

const UNSET_HEIGHT: u16 = 0xffff;

struct HeightPatch {
    data: Vec<u16>,
    xmin: i32,
    ymin: i32,
    width: i32,
    height: i32,
}

impl HeightPatch {
    #[inline(always)]
    fn at(&self, x: i32, y: i32) -> Option<u16> {
        if x < 0 || y < 0 || x >= self.width || y >= self.height {
            return None;
        }
        self.data.get((x + y * self.width) as usize).copied()
    }

    #[inline(always)]
    fn set(&mut self, x: i32, y: i32, v: u16) {
        if x >= 0
            && y >= 0
            && x < self.width
            && y < self.height
            && let Some(d) = self.data.get_mut((x + y * self.width) as usize)
        {
            *d = v;
        }
    }

    fn fill(&mut self, v: u16) {
        let n = (self.width.max(0) as usize * self.height.max(0) as usize).min(self.data.len());
        self.data[..n].fill(v);
    }
}

#[inline(always)]
fn vdot2(a: &[f32], b: &[f32]) -> f32 {
    a[0] * b[0] + a[2] * b[2]
}

#[inline(always)]
fn vdist_sq2(p: &[f32], q: &[f32]) -> f32 {
    let dx = q[0] - p[0];
    let dy = q[2] - p[2];
    dx * dx + dy * dy
}

#[inline(always)]
fn vdist2(p: &[f32], q: &[f32]) -> f32 {
    sqrtf(vdist_sq2(p, q))
}

#[inline(always)]
fn vcross2(p1: &[f32], p2: &[f32], p3: &[f32]) -> f32 {
    let u1 = p2[0] - p1[0];
    let v1 = p2[2] - p1[2];
    let u2 = p3[0] - p1[0];
    let v2 = p3[2] - p1[2];
    u1 * v2 - v1 * u2
}

fn circum_circle(p1: &[f32], p2: &[f32], p3: &[f32], c: &mut [f32; 3], r: &mut f32) -> bool {
    const EPS: f32 = 1e-6;
    let v1 = [0.0f32; 3];
    let v2 = [p2[0] - p1[0], p2[1] - p1[1], p2[2] - p1[2]];
    let v3 = [p3[0] - p1[0], p3[1] - p1[1], p3[2] - p1[2]];
    let cp = vcross2(&v1, &v2, &v3);
    if cp.abs() > EPS {
        let v1sq = vdot2(&v1, &v1);
        let v2sq = vdot2(&v2, &v2);
        let v3sq = vdot2(&v3, &v3);
        c[0] =
            (v1sq * (v2[2] - v3[2]) + v2sq * (v3[2] - v1[2]) + v3sq * (v1[2] - v2[2])) / (2.0 * cp);
        c[1] = 0.0;
        c[2] =
            (v1sq * (v3[0] - v2[0]) + v2sq * (v1[0] - v3[0]) + v3sq * (v2[0] - v1[0])) / (2.0 * cp);
        *r = vdist2(c, &v1);
        c[0] += p1[0];
        c[1] += p1[1];
        c[2] += p1[2];
        return true;
    }
    c.copy_from_slice(&p1[..3]);
    *r = 0.0;
    false
}

fn dist_pt_tri(p: &[f32], a: &[f32], b: &[f32], c: &[f32]) -> f32 {
    let v0 = [c[0] - a[0], c[1] - a[1], c[2] - a[2]];
    let v1 = [b[0] - a[0], b[1] - a[1], b[2] - a[2]];
    let v2 = [p[0] - a[0], p[1] - a[1], p[2] - a[2]];
    let dot00 = vdot2(&v0, &v0);
    let dot01 = vdot2(&v0, &v1);
    let dot02 = vdot2(&v0, &v2);
    let dot11 = vdot2(&v1, &v1);
    let dot12 = vdot2(&v1, &v2);
    let inv = 1.0 / (dot00 * dot11 - dot01 * dot01);
    let u = (dot11 * dot02 - dot01 * dot12) * inv;
    let v = (dot00 * dot12 - dot01 * dot02) * inv;
    const EPS: f32 = 1e-4;
    if u >= -EPS && v >= -EPS && (u + v) <= 1.0 + EPS {
        let y = a[1] + v0[1] * u + v1[1] * v;
        return (y - p[1]).abs();
    }
    f32::MAX
}

fn distance_pt_seg(pt: &[f32], p: &[f32], q: &[f32]) -> f32 {
    let pqx = q[0] - p[0];
    let pqy = q[1] - p[1];
    let pqz = q[2] - p[2];
    let mut dx = pt[0] - p[0];
    let mut dy = pt[1] - p[1];
    let mut dz = pt[2] - p[2];
    let d = pqx * pqx + pqy * pqy + pqz * pqz;
    let mut t = pqx * dx + pqy * dy + pqz * dz;
    if d > 0.0 {
        t /= d;
    }
    if t < 0.0 {
        t = 0.0;
    } else if t > 1.0 {
        t = 1.0;
    }
    dx = p[0] + t * pqx - pt[0];
    dy = p[1] + t * pqy - pt[1];
    dz = p[2] + t * pqz - pt[2];
    dx * dx + dy * dy + dz * dz
}

fn distance_pt_seg_2d(pt: &[f32], p: &[f32], q: &[f32]) -> f32 {
    let pqx = q[0] - p[0];
    let pqz = q[2] - p[2];
    let mut dx = pt[0] - p[0];
    let mut dz = pt[2] - p[2];
    let d = pqx * pqx + pqz * pqz;
    let mut t = pqx * dx + pqz * dz;
    if d > 0.0 {
        t /= d;
    }
    if t < 0.0 {
        t = 0.0;
    } else if t > 1.0 {
        t = 1.0;
    }
    dx = p[0] + t * pqx - pt[0];
    dz = p[2] + t * pqz - pt[2];
    dx * dx + dz * dz
}

fn dist_to_tri_mesh(p: &[f32], verts: &[f32], tris: &[i32]) -> f32 {
    let mut dmin = f32::MAX;
    for t in tris.chunks_exact(4) {
        let va = &verts[t[0] as usize * 3..];
        let vb = &verts[t[1] as usize * 3..];
        let vc = &verts[t[2] as usize * 3..];
        let d = dist_pt_tri(p, va, vb, vc);
        if d < dmin {
            dmin = d;
        }
    }
    if dmin == f32::MAX { -1.0 } else { dmin }
}

fn dist_to_poly(verts: &[f32], p: &[f32]) -> f32 {
    let n = verts.len() / 3;
    let mut dmin = f32::MAX;
    let mut c = false;
    let mut j = n.wrapping_sub(1);
    for i in 0..n {
        let vi = &verts[i * 3..];
        let vj = &verts[j * 3..];
        if ((vi[2] > p[2]) != (vj[2] > p[2]))
            && (p[0] < (vj[0] - vi[0]) * (p[2] - vi[2]) / (vj[2] - vi[2]) + vi[0])
        {
            c = !c;
        }
        dmin = fmin(dmin, distance_pt_seg_2d(p, vj, vi));
        j = i;
    }
    if c { -dmin } else { dmin }
}

#[allow(clippy::too_many_arguments)]
fn get_height(fx: f32, fy: f32, fz: f32, ics: f32, ch: f32, radius: i32, hp: &HeightPatch) -> u16 {
    let mut ix = floorf(fx * ics + 0.01) as i32;
    let mut iz = floorf(fz * ics + 0.01) as i32;
    // A spiral wider than the patch finds nothing more; clamping keeps huge radii finite.
    let radius = radius.min(hp.width.max(hp.height) + 1);
    ix = clamp(ix.wrapping_sub(hp.xmin), 0, hp.width - 1);
    iz = clamp(iz.wrapping_sub(hp.ymin), 0, hp.height - 1);
    let mut h = hp.at(ix, iz).unwrap_or(UNSET_HEIGHT);
    if h == UNSET_HEIGHT {
        let (mut x, mut z, mut dx, mut dz) = (1i32, 0i32, 1i32, 0i32);
        let max_size = radius * 2 + 1;
        let max_iter = max_size * max_size - 1;
        let mut next_ring_iter_start = 8;
        let mut next_ring_iters = 16;
        let mut dmin = f32::MAX;
        for i in 0..max_iter {
            let nx = ix + x;
            let nz = iz + z;
            if let Some(nh) = hp.at(nx, nz)
                && nh != UNSET_HEIGHT
            {
                let d = (f32::from(nh) * ch - fy).abs();
                if d < dmin {
                    h = nh;
                    dmin = d;
                }
            }
            if i + 1 == next_ring_iter_start {
                if h != UNSET_HEIGHT {
                    break;
                }
                next_ring_iter_start += next_ring_iters;
                next_ring_iters += 8;
            }
            if x == z || (x < 0 && x == -z) || (x > 0 && x == 1 - z) {
                let tmp = dx;
                dx = -dz;
                dz = tmp;
            }
            x += dx;
            z += dz;
        }
    }
    h
}

const EV_UNDEF: i32 = -1;
const EV_HULL: i32 = -2;

fn find_edge(edges: &[i32], nedges: usize, s: i32, t: i32) -> i32 {
    for i in 0..nedges {
        let e = &edges[i * 4..];
        if (e[0] == s && e[1] == t) || (e[0] == t && e[1] == s) {
            return i as i32;
        }
    }
    EV_UNDEF
}

#[allow(clippy::too_many_arguments)]
fn add_edge(
    edges: &mut [i32],
    nedges: &mut usize,
    max_edges: usize,
    s: i32,
    t: i32,
    l: i32,
    r: i32,
) -> i32 {
    if *nedges >= max_edges {
        return EV_UNDEF;
    }
    if find_edge(edges, *nedges, s, t) == EV_UNDEF {
        let e = &mut edges[*nedges * 4..*nedges * 4 + 4];
        e.copy_from_slice(&[s, t, l, r]);
        *nedges += 1;
        (*nedges - 1) as i32
    } else {
        EV_UNDEF
    }
}

fn update_left_face(e: &mut [i32], s: i32, t: i32, f: i32) {
    if e[0] == s && e[1] == t && e[2] == EV_UNDEF {
        e[2] = f;
    } else if e[1] == s && e[0] == t && e[3] == EV_UNDEF {
        e[3] = f;
    }
}

fn overlap_seg_seg_2d(a: &[f32], b: &[f32], c: &[f32], d: &[f32]) -> bool {
    let a1 = vcross2(a, b, d);
    let a2 = vcross2(a, b, c);
    if a1 * a2 < 0.0 {
        let a3 = vcross2(c, d, a);
        let a4 = a3 + a2 - a1;
        if a3 * a4 < 0.0 {
            return true;
        }
    }
    false
}

fn overlap_edges(pts: &[f32], edges: &[i32], nedges: usize, s1: i32, t1: i32) -> bool {
    for i in 0..nedges {
        let s0 = edges[i * 4];
        let t0 = edges[i * 4 + 1];
        if s0 == s1 || s0 == t1 || t0 == s1 || t0 == t1 {
            continue;
        }
        let p = |k: i32| &pts[k as usize * 3..k as usize * 3 + 3];
        if overlap_seg_seg_2d(p(s0), p(t0), p(s1), p(t1)) {
            return true;
        }
    }
    false
}

fn complete_facet(
    pts: &[f32],
    npts: usize,
    edges: &mut [i32],
    nedges: &mut usize,
    max_edges: usize,
    nfaces: &mut i32,
    e: usize,
) {
    const EPS: f32 = 1e-5;
    let edge = &edges[e * 4..e * 4 + 4];
    let (s, t) = if edge[2] == EV_UNDEF {
        (edge[0], edge[1])
    } else if edge[3] == EV_UNDEF {
        (edge[1], edge[0])
    } else {
        return;
    };
    let p = |k: i32| &pts[k as usize * 3..k as usize * 3 + 3];
    let mut pt = npts as i32;
    let mut c = [0.0f32; 3];
    let mut r = -1.0f32;
    for u in 0..npts as i32 {
        if u == s || u == t {
            continue;
        }
        if vcross2(p(s), p(t), p(u)) > EPS {
            if r < 0.0 {
                pt = u;
                circum_circle(p(s), p(t), p(u), &mut c, &mut r);
                continue;
            }
            let d = vdist2(&c, p(u));
            const TOL: f32 = 0.001;
            if d > r * (1.0 + TOL) {
                continue;
            } else if d < r * (1.0 - TOL) {
                pt = u;
                circum_circle(p(s), p(t), p(u), &mut c, &mut r);
            } else {
                if overlap_edges(pts, edges, *nedges, s, u)
                    || overlap_edges(pts, edges, *nedges, t, u)
                {
                    continue;
                }
                pt = u;
                circum_circle(p(s), p(t), p(u), &mut c, &mut r);
            }
        }
    }
    if pt < npts as i32 {
        update_left_face(&mut edges[e * 4..e * 4 + 4], s, t, *nfaces);
        let e2 = find_edge(edges, *nedges, pt, s);
        if e2 == EV_UNDEF {
            add_edge(edges, nedges, max_edges, pt, s, *nfaces, EV_UNDEF);
        } else {
            update_left_face(
                &mut edges[e2 as usize * 4..e2 as usize * 4 + 4],
                pt,
                s,
                *nfaces,
            );
        }
        let e3 = find_edge(edges, *nedges, t, pt);
        if e3 == EV_UNDEF {
            add_edge(edges, nedges, max_edges, t, pt, *nfaces, EV_UNDEF);
        } else {
            update_left_face(
                &mut edges[e3 as usize * 4..e3 as usize * 4 + 4],
                t,
                pt,
                *nfaces,
            );
        }
        *nfaces += 1;
    } else {
        update_left_face(&mut edges[e * 4..e * 4 + 4], s, t, EV_HULL);
    }
}

fn delaunay_hull(
    npts: usize,
    pts: &[f32],
    hull: &[i32],
    tris: &mut Vec<i32>,
    edges: &mut Vec<i32>,
) {
    let mut nfaces = 0i32;
    let mut nedges = 0usize;
    let max_edges = npts * 10;
    edges.clear();
    edges.resize(max_edges * 4, 0);
    let nhull = hull.len();
    let mut j = nhull.wrapping_sub(1);
    for i in 0..nhull {
        add_edge(
            edges,
            &mut nedges,
            max_edges,
            hull[j],
            hull[i],
            EV_HULL,
            EV_UNDEF,
        );
        j = i;
    }
    let mut cur = 0usize;
    while cur < nedges {
        if edges[cur * 4 + 2] == EV_UNDEF {
            complete_facet(pts, npts, edges, &mut nedges, max_edges, &mut nfaces, cur);
        }
        if edges[cur * 4 + 3] == EV_UNDEF {
            complete_facet(pts, npts, edges, &mut nedges, max_edges, &mut nfaces, cur);
        }
        cur += 1;
    }
    tris.clear();
    tris.resize(nfaces.max(0) as usize * 4, -1);
    for i in 0..nedges {
        let e = [
            edges[i * 4],
            edges[i * 4 + 1],
            edges[i * 4 + 2],
            edges[i * 4 + 3],
        ];
        if e[3] >= 0 {
            let t = &mut tris[e[3] as usize * 4..e[3] as usize * 4 + 4];
            if t[0] == -1 {
                t[0] = e[0];
                t[1] = e[1];
            } else if t[0] == e[1] {
                t[2] = e[0];
            } else if t[1] == e[0] {
                t[2] = e[1];
            }
        }
        if e[2] >= 0 {
            let t = &mut tris[e[2] as usize * 4..e[2] as usize * 4 + 4];
            if t[0] == -1 {
                t[0] = e[1];
                t[1] = e[0];
            } else if t[0] == e[0] {
                t[2] = e[1];
            } else if t[1] == e[1] {
                t[2] = e[0];
            }
        }
    }
    let mut i = 0usize;
    while i < tris.len() / 4 {
        if tris[i * 4] == -1 || tris[i * 4 + 1] == -1 || tris[i * 4 + 2] == -1 {
            let n = tris.len();
            let last = [tris[n - 4], tris[n - 3], tris[n - 2], tris[n - 1]];
            tris[i * 4..i * 4 + 4].copy_from_slice(&last);
            tris.truncate(n - 4);
        } else {
            i += 1;
        }
    }
}

fn poly_min_extent(verts: &[f32]) -> f32 {
    let n = verts.len() / 3;
    let mut min_dist = f32::MAX;
    for i in 0..n {
        let ni = (i + 1) % n;
        let p1 = &verts[i * 3..];
        let p2 = &verts[ni * 3..];
        let mut max_edge_dist = 0.0f32;
        for j in 0..n {
            if j == i || j == ni {
                continue;
            }
            let d = distance_pt_seg_2d(&verts[j * 3..], p1, p2);
            max_edge_dist = fmax(max_edge_dist, d);
        }
        min_dist = fmin(min_dist, max_edge_dist);
    }
    sqrtf(min_dist)
}

#[inline(always)]
fn prev(i: usize, n: usize) -> usize {
    if i >= 1 { i - 1 } else { n - 1 }
}

#[inline(always)]
fn next(i: usize, n: usize) -> usize {
    if i + 1 < n { i + 1 } else { 0 }
}

fn triangulate_hull(verts: &[f32], hull: &[i32], nin: usize, tris: &mut Vec<i32>) {
    let nhull = hull.len();
    if nhull < 3 {
        return;
    }
    let (mut start, mut left, mut right) = (0usize, 1usize, nhull - 1);
    let mut dmin = f32::MAX;
    let v = |k: i32| &verts[k as usize * 3..k as usize * 3 + 3];
    for i in 0..nhull {
        if hull[i] as usize >= nin {
            continue;
        }
        let pi = prev(i, nhull);
        let ni = next(i, nhull);
        let (pv, cv, nv) = (v(hull[pi]), v(hull[i]), v(hull[ni]));
        let d = vdist2(pv, cv) + vdist2(cv, nv) + vdist2(nv, pv);
        if d < dmin {
            start = i;
            left = ni;
            right = pi;
            dmin = d;
        }
    }
    tris.extend_from_slice(&[hull[start], hull[left], hull[right], 0]);
    while next(left, nhull) != right {
        let nleft = next(left, nhull);
        let nright = prev(right, nhull);
        let (cvl, nvl, cvr, nvr) = (
            v(hull[left]),
            v(hull[nleft]),
            v(hull[right]),
            v(hull[nright]),
        );
        let dleft = vdist2(cvl, nvl) + vdist2(nvl, cvr);
        let dright = vdist2(cvr, nvr) + vdist2(cvl, nvr);
        if dleft < dright {
            tris.extend_from_slice(&[hull[left], hull[nleft], hull[right], 0]);
            left = nleft;
        } else {
            tris.extend_from_slice(&[hull[left], hull[nright], hull[right], 0]);
            right = nright;
        }
    }
}

#[inline(always)]
fn jitter_x(i: usize) -> f32 {
    (((i as u32).wrapping_mul(0x8da6b343) & 0xffff) as f32 / 65535.0 * 2.0) - 1.0
}

#[inline(always)]
fn jitter_y(i: usize) -> f32 {
    (((i as u32).wrapping_mul(0xd8163841) & 0xffff) as f32 / 65535.0 * 2.0) - 1.0
}

fn on_hull(a: i32, b: i32, hull: &[i32]) -> bool {
    let nhull = hull.len();
    if a as usize >= nhull || b as usize >= nhull || a < 0 || b < 0 {
        return false;
    }
    let mut j = nhull - 1;
    for i in 0..nhull {
        if a == hull[j] && b == hull[i] {
            return true;
        }
        j = i;
    }
    false
}

fn set_tri_flags(tris: &mut [i32], hull: &[i32]) {
    for t in tris.chunks_exact_mut(4) {
        let (a, b, c) = (t[0], t[1], t[2]);
        let mut flags = 0i32;
        flags |= i32::from(on_hull(a, b, hull));
        flags |= i32::from(on_hull(b, c, hull)) << 2;
        flags |= i32::from(on_hull(c, a, hull)) << 4;
        t[3] = flags;
    }
}

#[allow(clippy::too_many_arguments)]
fn build_poly_detail(
    ws: &mut Workspace,
    input: &[f32],
    sample_dist: f32,
    sample_max_error: f32,
    search_radius: i32,
    chf: &CompactHeightfield,
    hp: &HeightPatch,
    verts: &mut [f32; 256 * 3],
    tris: &mut Vec<i32>,
    edges: &mut Vec<i32>,
    samples: &mut Vec<i32>,
) -> Result<usize, Error> {
    const MAX_VERTS: usize = 127;
    const MAX_TRIS: usize = 255;
    const MAX_VERTS_PER_EDGE: usize = 32;
    let nin = input.len() / 3;
    let mut edge = [0.0f32; (MAX_VERTS_PER_EDGE + 1) * 3];
    let mut hull = [0i32; MAX_VERTS];
    let mut nhull = 0usize;
    let mut nverts = nin;
    verts[..nin * 3].copy_from_slice(input);
    edges.clear();
    tris.clear();
    let cs = chf.cs;
    let ics = 1.0 / cs;
    let min_extent = poly_min_extent(&verts[..nverts * 3]);
    if sample_dist > 0.0 {
        let mut j = nin.wrapping_sub(1);
        for i in 0..nin {
            let mut vj = &input[j * 3..j * 3 + 3];
            let mut vi = &input[i * 3..i * 3 + 3];
            let mut swapped = false;
            if (vj[0] - vi[0]).abs() < 1e-6 {
                if vj[2] > vi[2] {
                    core::mem::swap(&mut vj, &mut vi);
                    swapped = true;
                }
            } else if vj[0] > vi[0] {
                core::mem::swap(&mut vj, &mut vi);
                swapped = true;
            }
            let dx = vi[0] - vj[0];
            let dy = vi[1] - vj[1];
            let dz = vi[2] - vj[2];
            let d = sqrtf(dx * dx + dz * dz);
            let mut nn = 1 + floorf(d / sample_dist) as i32;
            if nn >= MAX_VERTS_PER_EDGE as i32 {
                nn = MAX_VERTS_PER_EDGE as i32 - 1;
            }
            if nverts as i32 + nn >= MAX_VERTS as i32 {
                nn = MAX_VERTS as i32 - 1 - nverts as i32;
            }
            let nn_u = nn.max(0) as usize;
            for k in 0..=nn_u {
                let u = k as f32 / nn as f32;
                let pos = &mut edge[k * 3..k * 3 + 3];
                pos[0] = vj[0] + dx * u;
                pos[1] = vj[1] + dy * u;
                pos[2] = vj[2] + dz * u;
                pos[1] = f32::from(get_height(
                    pos[0],
                    pos[1],
                    pos[2],
                    ics,
                    chf.ch,
                    search_radius,
                    hp,
                )) * chf.ch;
            }
            let mut idx = [0usize; MAX_VERTS_PER_EDGE];
            idx[1] = nn_u;
            let mut nidx = 2usize;
            let mut k = 0usize;
            while k + 1 < nidx {
                let a = idx[k];
                let b = idx[k + 1];
                let va = &edge[a * 3..a * 3 + 3];
                let vb = &edge[b * 3..b * 3 + 3];
                let mut maxd = 0.0f32;
                let mut maxi: Option<usize> = None;
                for m in a + 1..b {
                    let dev = distance_pt_seg(&edge[m * 3..], va, vb);
                    if dev > maxd {
                        maxd = dev;
                        maxi = Some(m);
                    }
                }
                match maxi {
                    Some(m) if maxd > sqr(sample_max_error) && nidx < MAX_VERTS_PER_EDGE => {
                        for q in (k + 1..=nidx).rev() {
                            idx[q] = idx[q - 1];
                        }
                        idx[k + 1] = m;
                        nidx += 1;
                    }
                    _ => k += 1,
                }
            }
            hull[nhull] = j as i32;
            nhull += 1;
            if swapped {
                for k in (1..nidx - 1).rev() {
                    verts[nverts * 3..nverts * 3 + 3]
                        .copy_from_slice(&edge[idx[k] * 3..idx[k] * 3 + 3]);
                    hull[nhull] = nverts as i32;
                    nhull += 1;
                    nverts += 1;
                }
            } else {
                for k in 1..nidx - 1 {
                    verts[nverts * 3..nverts * 3 + 3]
                        .copy_from_slice(&edge[idx[k] * 3..idx[k] * 3 + 3]);
                    hull[nhull] = nverts as i32;
                    nhull += 1;
                    nverts += 1;
                }
            }
            j = i;
        }
    }
    let hull = &hull[..nhull];
    if min_extent < sample_dist * 2.0 {
        triangulate_hull(&verts[..nverts * 3], hull, nin, tris);
        set_tri_flags(tris, hull);
        return Ok(nverts);
    }
    triangulate_hull(&verts[..nverts * 3], hull, nin, tris);
    if tris.is_empty() {
        return Ok(nverts);
    }
    if sample_dist > 0.0 {
        let mut bmin = [input[0], input[1], input[2]];
        let mut bmax = bmin;
        for v in input.chunks_exact(3).skip(1) {
            vmin(&mut bmin, v);
            vmax(&mut bmax, v);
        }
        let x0 = floorf(bmin[0] / sample_dist) as i32;
        let x1 = ceilf(bmax[0] / sample_dist) as i32;
        let z0 = floorf(bmin[2] / sample_dist) as i32;
        let z1 = ceilf(bmax[2] / sample_dist) as i32;
        samples.clear();
        ws.work(((x1 - x0).max(0) as u64) * ((z1 - z0).max(0) as u64))?;
        for z in z0..z1 {
            for x in x0..x1 {
                let pt = [
                    x as f32 * sample_dist,
                    (bmax[1] + bmin[1]) * 0.5,
                    z as f32 * sample_dist,
                ];
                if dist_to_poly(input, &pt) > -sample_dist / 2.0 {
                    continue;
                }
                let h = get_height(pt[0], pt[1], pt[2], ics, chf.ch, search_radius, hp);
                ws.reserve(samples, 4)?;
                samples.extend_from_slice(&[x, i32::from(h), z, 0]);
            }
        }
        let nsamples = samples.len() / 4;
        for _ in 0..nsamples {
            if nverts >= MAX_VERTS {
                break;
            }
            let mut bestpt = [0.0f32; 3];
            let mut bestd = 0.0f32;
            let mut besti: Option<usize> = None;
            ws.work((nsamples * tris.len() / 4) as u64)?;
            for i in 0..nsamples {
                let s = &samples[i * 4..i * 4 + 4];
                if s[3] != 0 {
                    continue;
                }
                let pt = [
                    s[0] as f32 * sample_dist + jitter_x(i) * cs * 0.1,
                    s[1] as f32 * chf.ch,
                    s[2] as f32 * sample_dist + jitter_y(i) * cs * 0.1,
                ];
                if tris.is_empty() {
                    continue;
                }
                let d = dist_to_tri_mesh(&pt, &verts[..nverts * 3], tris);
                if d < 0.0 {
                    continue;
                }
                if d > bestd {
                    bestd = d;
                    besti = Some(i);
                    bestpt = pt;
                }
            }
            let Some(besti) = besti else {
                break;
            };
            if bestd <= sample_max_error {
                break;
            }
            samples[besti * 4 + 3] = 1;
            verts[nverts * 3..nverts * 3 + 3].copy_from_slice(&bestpt);
            nverts += 1;
            delaunay_hull(nverts, &verts[..nverts * 3], hull, tris, edges);
        }
    }
    if tris.len() / 4 > MAX_TRIS {
        tris.truncate(MAX_TRIS * 4);
    }
    set_tri_flags(tris, hull);
    Ok(nverts)
}

fn seed_array_with_poly_center(
    chf: &CompactHeightfield,
    poly: &[u16],
    verts: &[u16],
    bs: i32,
    hp: &mut HeightPatch,
    array: &mut Vec<i32>,
) {
    const OFFSET: [i32; 18] = [0, 0, -1, -1, 0, -1, 1, -1, 1, 0, 1, 1, 0, 1, -1, 1, -1, 0];
    let (mut start_x, mut start_y, mut start_i) = (0i32, 0i32, -1i32);
    let mut dmin = i32::from(UNSET_HEIGHT);
    'outer: for &pj in poly {
        for k in 0..9 {
            if dmin <= 0 {
                break 'outer;
            }
            let v = &verts[usize::from(pj) * 3..];
            let ax = i32::from(v[0]) + OFFSET[k * 2];
            let ay = i32::from(v[1]);
            let az = i32::from(v[2]) + OFFSET[k * 2 + 1];
            if ax < hp.xmin || ax >= hp.xmin + hp.width || az < hp.ymin || az >= hp.ymin + hp.height
            {
                continue;
            }
            let c = chf.cells[((ax + bs) + (az + bs) * chf.width) as usize];
            for i in c.index as usize..c.index as usize + c.count as usize {
                if dmin <= 0 {
                    break;
                }
                let d = (ay - i32::from(chf.spans[i].y)).abs();
                if d < dmin {
                    start_x = ax;
                    start_y = az;
                    start_i = i as i32;
                    dmin = d;
                }
            }
        }
    }
    let (mut pcx, mut pcy) = (0i32, 0i32);
    for &pj in poly {
        pcx += i32::from(verts[usize::from(pj) * 3]);
        pcy += i32::from(verts[usize::from(pj) * 3 + 2]);
    }
    pcx /= poly.len() as i32;
    pcy /= poly.len() as i32;
    array.clear();
    array.extend_from_slice(&[start_x, start_y, start_i]);
    let mut dirs = [0i32, 1, 2, 3];
    hp.fill(0);
    let (mut cx, mut cy, mut ci) = (-1i32, -1i32, -1i32);
    loop {
        if array.len() < 3 {
            break;
        }
        ci = array.pop().unwrap_or(-1);
        cy = array.pop().unwrap_or(-1);
        cx = array.pop().unwrap_or(-1);
        if cx == pcx && cy == pcy {
            break;
        }
        if ci < 0 {
            continue;
        }
        let direct = if cx == pcx {
            dir_for_offset(0, if pcy > cy { 1 } else { -1 })
        } else {
            dir_for_offset(if pcx > cx { 1 } else { -1 }, 0)
        } as usize;
        dirs.swap(direct, 3);
        let cs = chf.spans[ci as usize];
        for &dir in &dirs {
            let con = cs.con(dir);
            if con == NOT_CONNECTED {
                continue;
            }
            let nx = cx + dir_offset_x(dir);
            let ny = cy + dir_offset_y(dir);
            let hpx = nx - hp.xmin;
            let hpy = ny - hp.ymin;
            if hpx < 0 || hpx >= hp.width || hpy < 0 || hpy >= hp.height {
                continue;
            }
            if hp.at(hpx, hpy) != Some(0) {
                continue;
            }
            hp.set(hpx, hpy, 1);
            let idx = chf.cells[((nx + bs) + (ny + bs) * chf.width) as usize].index as i32 + con;
            array.extend_from_slice(&[nx, ny, idx]);
        }
        dirs.swap(direct, 3);
    }
    array.clear();
    array.extend_from_slice(&[cx + bs, cy + bs, ci]);
    hp.fill(UNSET_HEIGHT);
    if ci >= 0 {
        let y = chf.spans[ci as usize].y;
        hp.set(cx - hp.xmin, cy - hp.ymin, y);
    } else {
        array.clear();
    }
}

fn get_height_data(
    chf: &CompactHeightfield,
    poly: &[u16],
    verts: &[u16],
    bs: i32,
    hp: &mut HeightPatch,
    queue: &mut Vec<i32>,
    region: u16,
) {
    queue.clear();
    hp.fill(UNSET_HEIGHT);
    let mut empty = true;
    if region != MULTIPLE_REGS {
        for hy in 0..hp.height {
            let y = hp.ymin + hy + bs;
            for hx in 0..hp.width {
                let x = hp.xmin + hx + bs;
                let c = chf.cells[(x + y * chf.width) as usize];
                for i in c.index as usize..c.index as usize + c.count as usize {
                    let s = chf.spans[i];
                    if s.reg != region {
                        continue;
                    }
                    hp.set(hx, hy, s.y);
                    empty = false;
                    let mut border = false;
                    for dir in 0..4 {
                        let con = s.con(dir);
                        if con != NOT_CONNECTED {
                            let ax = x + dir_offset_x(dir);
                            let ay = y + dir_offset_y(dir);
                            let ai = chf.cells[(ax + ay * chf.width) as usize].index as usize
                                + con as usize;
                            if chf.spans[ai].reg != region {
                                border = true;
                                break;
                            }
                        }
                    }
                    if border {
                        queue.extend_from_slice(&[x, y, i as i32]);
                    }
                    break;
                }
            }
        }
    }
    if empty {
        seed_array_with_poly_center(chf, poly, verts, bs, hp, queue);
    }
    const RETRACT_SIZE: usize = 256;
    let mut head = 0usize;
    while head * 3 < queue.len() {
        let cx = queue[head * 3];
        let cy = queue[head * 3 + 1];
        let ci = queue[head * 3 + 2];
        head += 1;
        if head >= RETRACT_SIZE {
            head = 0;
            if queue.len() > RETRACT_SIZE * 3 {
                queue.copy_within(RETRACT_SIZE * 3.., 0);
            }
            queue.truncate(queue.len() - RETRACT_SIZE * 3);
        }
        let cs = chf.spans[ci as usize];
        for dir in 0..4 {
            let con = cs.con(dir);
            if con == NOT_CONNECTED {
                continue;
            }
            let ax = cx + dir_offset_x(dir);
            let ay = cy + dir_offset_y(dir);
            let hx = ax - hp.xmin - bs;
            let hy = ay - hp.ymin - bs;
            if hx < 0 || hy < 0 || hx >= hp.width || hy >= hp.height {
                continue;
            }
            if hp.at(hx, hy) != Some(UNSET_HEIGHT) {
                continue;
            }
            let ai = chf.cells[(ax + ay * chf.width) as usize].index as i32 + con;
            hp.set(hx, hy, chf.spans[ai as usize].y);
            queue.extend_from_slice(&[ax, ay, ai]);
        }
    }
}
