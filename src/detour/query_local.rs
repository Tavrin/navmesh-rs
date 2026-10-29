//! Raycasts, surface moves and the local (Dijkstra and stack) queries of
//! `dtNavMeshQuery`.

use alloc::vec::Vec;

use super::math::*;
use super::node::{NODE_CLOSED, NODE_OPEN, NodePool, no_better};
use super::query::{NavMeshQuery, QueryFilter, RaycastHit, SegmentWall, Tp};
use super::{
    EXT_LINK, NULL_LINK, POLYTYPE_OFFMESH_CONNECTION, PolyRef, RAYCAST_USE_COSTS, Status,
    VERTS_PER_POLYGON,
};
use crate::Error;

enum Region<'s> {
    Circle([f32; 3], f32),
    Shape(&'s [f32], usize),
}

impl<'a> NavMeshQuery<'a> {
    /// Moves from `start_pos` towards `end_pos` along the surface, constrained to
    /// polygons reachable within the search circle (`moveAlongSurface`): result
    /// position, visited polygons (at most `max_visited`) and status.
    pub fn move_along_surface(
        &mut self,
        start_ref: PolyRef,
        start_pos: &[f32; 3],
        end_pos: &[f32; 3],
        filter: &dyn QueryFilter,
        visited: &mut Vec<PolyRef>,
        max_visited: usize,
    ) -> Result<([f32; 3], Status), Error> {
        visited.clear();
        if !self.nav.is_valid_poly_ref(start_ref)
            || !visfinite(start_pos)
            || !visfinite(end_pos)
            || max_visited == 0
        {
            return Err(Error::InvalidParameter("ref, positions or max_visited"));
        }
        let mut status = Status::SUCCESS;
        const MAX_STACK: usize = 48;
        let mut stack: Vec<usize> = Vec::with_capacity(MAX_STACK);
        self.tiny_node_pool.clear();
        let sn = self
            .tiny_node_pool
            .get_node(start_ref, 0)
            .ok_or(Error::AllocationFailed)?;
        {
            let n = &mut self.tiny_node_pool.nodes[sn];
            n.pidx = 0;
            n.cost = 0.0;
            n.total = 0.0;
            n.id = start_ref;
            n.flags = NODE_CLOSED;
        }
        stack.push(sn);
        let mut best_pos = *start_pos;
        let mut best_dist = f32::MAX;
        let mut best_node: Option<usize> = None;
        let search_pos = vlerp(start_pos, end_pos, 0.5);
        let search_rad_sqr = sqr(vdist(start_pos, end_pos) / 2.0 + 0.001);
        let mut verts = [0.0f32; VERTS_PER_POLYGON * 3];
        while !stack.is_empty() {
            let cur = stack.remove(0);
            let cur_ref = self.tiny_node_pool.nodes[cur].id;
            let (_, tile, poly) = self.tp(cur_ref);
            let nverts = poly.vert_count as usize;
            for i in 0..nverts {
                let v = poly.verts[i] as usize * 3;
                verts[i * 3..i * 3 + 3].copy_from_slice(&tile.verts[v..v + 3]);
            }
            if point_in_polygon(end_pos, &verts, nverts) {
                best_node = Some(cur);
                best_pos = *end_pos;
                break;
            }
            let base = self
                .nav
                .base_ref(self.nav.decode_poly_id_tile(cur_ref) as usize);
            let mut j = nverts - 1;
            for i in 0..nverts {
                let mut neis: [PolyRef; 8] = [0; 8];
                let mut nneis = 0;
                if poly.neis[j] & EXT_LINK != 0 {
                    let mut k = poly.first_link;
                    while k != NULL_LINK {
                        let link = &tile.links[k as usize];
                        k = link.next;
                        if link.edge as usize == j && link.r#ref != 0 {
                            let nei = self.tp(link.r#ref);
                            if filter.pass_filter(link.r#ref, nei.1, nei.2) && nneis < 8 {
                                neis[nneis] = link.r#ref;
                                nneis += 1;
                            }
                        }
                    }
                } else if poly.neis[j] != 0 {
                    let idx = (poly.neis[j] - 1) as usize;
                    let r = base | idx as PolyRef;
                    if filter.pass_filter(r, tile, &tile.polys[idx]) {
                        neis[nneis] = r;
                        nneis += 1;
                    }
                }
                let vj = &verts[j * 3..j * 3 + 3];
                let vi = &verts[i * 3..i * 3 + 3];
                if nneis == 0 {
                    let (d, t) = distance_pt_seg_sqr_2d(end_pos, vj, vi);
                    if d < best_dist {
                        best_pos = vlerp(vj, vi, t);
                        best_dist = d;
                        best_node = Some(cur);
                    }
                } else {
                    for &nr in &neis[..nneis] {
                        let Some(nn) = self.tiny_node_pool.get_node(nr, 0) else {
                            continue;
                        };
                        if self.tiny_node_pool.nodes[nn].flags & NODE_CLOSED != 0 {
                            continue;
                        }
                        let (d, _) = distance_pt_seg_sqr_2d(&search_pos, vj, vi);
                        if d > search_rad_sqr {
                            continue;
                        }
                        if stack.len() < MAX_STACK {
                            let n = &mut self.tiny_node_pool.nodes[nn];
                            n.pidx = NodePool::idx(cur);
                            n.flags |= NODE_CLOSED;
                            stack.push(nn);
                        }
                    }
                }
                j = i;
            }
        }
        if let Some(bn) = best_node {
            let nodes = &mut self.tiny_node_pool.nodes;
            let mut prev: Option<usize> = None;
            let mut node = Some(bn);
            while let Some(n) = node {
                let next = NodePool::at_idx(nodes[n].pidx);
                nodes[n].pidx = prev.map_or(0, NodePool::idx);
                prev = Some(n);
                node = next;
            }
            let mut node = prev;
            while let Some(n) = node {
                visited.push(nodes[n].id);
                if visited.len() >= max_visited {
                    status |= Status::BUFFER_TOO_SMALL;
                    break;
                }
                node = NodePool::at_idx(nodes[n].pidx);
            }
        }
        Ok((best_pos, Status(status)))
    }

    /// Raycast along the surface (`raycast` with `dtRaycastHit`), collecting at most
    /// `max_path` visited polygons. `prev_ref` is the polygon before `start_ref`
    /// for cost purposes (0 for none).
    #[allow(clippy::too_many_arguments)]
    pub fn raycast_with(
        &self,
        start_ref: PolyRef,
        start_pos: &[f32; 3],
        end_pos: &[f32; 3],
        filter: &dyn QueryFilter,
        options: u32,
        max_path: usize,
        prev_ref: PolyRef,
    ) -> Result<(RaycastHit, Status), Error> {
        let mut hit = RaycastHit::default();
        if !self.nav.is_valid_poly_ref(start_ref)
            || !visfinite(start_pos)
            || !visfinite(end_pos)
            || (prev_ref != 0 && !self.nav.is_valid_poly_ref(prev_ref))
        {
            return Err(Error::InvalidParameter("refs or positions"));
        }
        let mut cur_pos = *start_pos;
        let dir = vsub(end_pos, start_pos);
        let mut status = Status::SUCCESS;
        let mut verts = [0.0f32; VERTS_PER_POLYGON * 3 + 3];
        let mut cur: Tp<'a> = self.tp(start_ref);
        let mut prev: Option<Tp<'a>> = if prev_ref != 0 {
            Some(self.tp(prev_ref))
        } else {
            None
        };
        loop {
            let (cur_ref, tile, poly) = cur;
            let nv = poly.vert_count as usize;
            for i in 0..nv {
                let v = poly.verts[i] as usize * 3;
                verts[i * 3..i * 3 + 3].copy_from_slice(&tile.verts[v..v + 3]);
            }
            let Some((_, tmax, _, seg_max)) =
                intersect_segment_poly_2d(start_pos, end_pos, &verts, nv)
            else {
                return Ok((hit, Status(status)));
            };
            hit.hit_edge_index = seg_max;
            if tmax > hit.t {
                hit.t = tmax;
            }
            if hit.path.len() < max_path {
                hit.path.push(cur_ref);
            } else {
                status |= Status::BUFFER_TOO_SMALL;
            }
            if seg_max == -1 {
                hit.t = f32::MAX;
                if options & RAYCAST_USE_COSTS != 0 {
                    hit.path_cost += filter.cost(&cur_pos, end_pos, prev, cur, Some(cur));
                }
                return Ok((hit, Status(status)));
            }
            let seg_max = seg_max as usize;
            let mut next: Option<Tp<'a>> = None;
            let mut k = poly.first_link;
            while k != NULL_LINK {
                let link = &tile.links[k as usize];
                k = link.next;
                if link.edge as usize != seg_max {
                    continue;
                }
                let nt = self.tp(link.r#ref);
                if nt.2.poly_type() == POLYTYPE_OFFMESH_CONNECTION
                    || !filter.pass_filter(link.r#ref, nt.1, nt.2)
                {
                    continue;
                }
                if link.side == 0xff || (link.bmin == 0 && link.bmax == 255) {
                    next = Some(nt);
                    break;
                }
                let v0 = poly.verts[link.edge as usize] as usize * 3;
                let v1 = poly.verts[(link.edge as usize + 1) % nv] as usize * 3;
                let left = &tile.verts[v0..v0 + 3];
                let right = &tile.verts[v1..v1 + 3];
                let ax = match link.side {
                    0 | 4 => 2,
                    2 | 6 => 0,
                    _ => continue,
                };
                let s = 1.0f32 / 255.0;
                let mut lmin = left[ax] + (right[ax] - left[ax]) * (link.bmin as f32 * s);
                let mut lmax = left[ax] + (right[ax] - left[ax]) * (link.bmax as f32 * s);
                if lmin > lmax {
                    core::mem::swap(&mut lmin, &mut lmax);
                }
                let c = start_pos[ax] + (end_pos[ax] - start_pos[ax]) * tmax;
                if c >= lmin && c <= lmax {
                    next = Some(nt);
                    break;
                }
            }
            if options & RAYCAST_USE_COSTS != 0 {
                let last_pos = cur_pos;
                cur_pos = vmad(start_pos, &dir, hit.t);
                let e1 = &verts[seg_max * 3..seg_max * 3 + 3];
                let e2 = &verts[((seg_max + 1) % nv) * 3..((seg_max + 1) % nv) * 3 + 3];
                let e_dir = vsub(e2, e1);
                let diff = vsub(&cur_pos, e1);
                let s = if sqr(e_dir[0]) > sqr(e_dir[2]) {
                    diff[0] / e_dir[0]
                } else {
                    diff[2] / e_dir[2]
                };
                cur_pos[1] = e1[1] + e_dir[1] * s;
                hit.path_cost += filter.cost(&last_pos, &cur_pos, prev, cur, next);
            }
            let Some(nt) = next else {
                let a = seg_max;
                let b = if seg_max + 1 < nv { seg_max + 1 } else { 0 };
                let dx = verts[b * 3] - verts[a * 3];
                let dz = verts[b * 3 + 2] - verts[a * 3 + 2];
                hit.hit_normal = [dz, 0.0, -dx];
                vnormalize(&mut hit.hit_normal);
                return Ok((hit, Status(status)));
            };
            prev = Some(cur);
            cur = nt;
            if status & Status::BUFFER_TOO_SMALL != 0 {
                status |= Status::PARTIAL_RESULT;
                break;
            }
        }
        Ok((hit, Status(status)))
    }

    /// The first upstream `raycast` form: `t`, hit normal, visited polygons.
    pub fn raycast(
        &self,
        start_ref: PolyRef,
        start_pos: &[f32; 3],
        end_pos: &[f32; 3],
        filter: &dyn QueryFilter,
        max_path: usize,
    ) -> Result<(RaycastHit, Status), Error> {
        self.raycast_with(start_ref, start_pos, end_pos, filter, 0, max_path, 0)
    }

    #[allow(clippy::too_many_arguments)]
    fn polys_around(
        &mut self,
        start_ref: PolyRef,
        center: [f32; 3],
        region: Region,
        filter: &dyn QueryFilter,
        out: &mut Vec<(PolyRef, PolyRef, f32)>,
        max: usize,
    ) -> Result<Status, Error> {
        self.node_pool.clear();
        self.open_list.clear();
        let sn = self
            .node_pool
            .get_node(start_ref, 0)
            .ok_or(Error::AllocationFailed)?;
        {
            let n = &mut self.node_pool.nodes[sn];
            n.pos = center;
            n.pidx = 0;
            n.cost = 0.0;
            n.total = 0.0;
            n.id = start_ref;
            n.flags = NODE_OPEN;
        }
        self.open_list.push(sn, &self.node_pool.nodes);
        let mut status = Status::SUCCESS;
        while !self.open_list.empty() {
            let bn = self.open_list.pop(&self.node_pool.nodes);
            self.node_pool.nodes[bn].flags &= !NODE_OPEN;
            self.node_pool.nodes[bn].flags |= NODE_CLOSED;
            let best = self.tp(self.node_pool.nodes[bn].id);
            let parent = NodePool::at_idx(self.node_pool.nodes[bn].pidx)
                .map(|i| self.node_pool.nodes[i].id)
                .filter(|&r| r != 0)
                .map(|r| self.tp(r));
            let parent_ref = parent.map_or(0, |p| p.0);
            if out.len() < max {
                out.push((best.0, parent_ref, self.node_pool.nodes[bn].total));
            } else {
                status |= Status::BUFFER_TOO_SMALL;
            }
            let mut li = best.2.first_link;
            while li != NULL_LINK {
                let link = best.1.links[li as usize];
                li = link.next;
                let nref = link.r#ref;
                if nref == 0 || nref == parent_ref {
                    continue;
                }
                let nei = self.tp(nref);
                if !filter.pass_filter(nref, nei.1, nei.2) {
                    continue;
                }
                let Some((va, vb)) = self.portal_points_tp(best, nei) else {
                    continue;
                };
                match region {
                    Region::Circle(c, r2) => {
                        if distance_pt_seg_sqr_2d(&c, &va, &vb).0 > r2 {
                            continue;
                        }
                    }
                    Region::Shape(verts, nverts) => {
                        match intersect_segment_poly_2d(&va, &vb, verts, nverts) {
                            Some((tmin, tmax, _, _)) if !(tmin > 1.0 || tmax < 0.0) => {}
                            _ => continue,
                        }
                    }
                }
                let Some(nn) = self.node_pool.get_node(nref, 0) else {
                    status |= Status::OUT_OF_NODES;
                    continue;
                };
                if self.node_pool.nodes[nn].flags & NODE_CLOSED != 0 {
                    continue;
                }
                if self.node_pool.nodes[nn].flags == 0 {
                    self.node_pool.nodes[nn].pos = vlerp(&va, &vb, 0.5);
                }
                let bpos = self.node_pool.nodes[bn].pos;
                let npos = self.node_pool.nodes[nn].pos;
                let cost = filter.cost(&bpos, &npos, parent, best, Some(nei));
                let total = self.node_pool.nodes[bn].total + cost;
                let n = &mut self.node_pool.nodes[nn];
                if n.flags & NODE_OPEN != 0 && no_better(total, n.total) {
                    continue;
                }
                n.id = nref;
                n.pidx = NodePool::idx(bn);
                n.total = total;
                if n.flags & NODE_OPEN != 0 {
                    self.open_list.modify(nn, &self.node_pool.nodes);
                } else {
                    n.flags = NODE_OPEN;
                    self.open_list.push(nn, &self.node_pool.nodes);
                }
            }
        }
        Ok(Status(status))
    }

    /// Polygons reachable from `start_ref` whose portals touch the circle
    /// (`findPolysAroundCircle`): (ref, parent, cost) per polygon, at most `max`.
    pub fn find_polys_around_circle(
        &mut self,
        start_ref: PolyRef,
        center: &[f32; 3],
        radius: f32,
        filter: &dyn QueryFilter,
        out: &mut Vec<(PolyRef, PolyRef, f32)>,
        max: usize,
    ) -> Result<Status, Error> {
        out.clear();
        if !self.nav.is_valid_poly_ref(start_ref)
            || !visfinite(center)
            || radius < 0.0
            || !radius.is_finite()
        {
            return Err(Error::InvalidParameter("ref, center or radius"));
        }
        self.polys_around(
            start_ref,
            *center,
            Region::Circle(*center, sqr(radius)),
            filter,
            out,
            max,
        )
    }

    /// Polygons reachable from `start_ref` whose portals touch the convex shape
    /// (`findPolysAroundShape`), `verts` holding 3 floats per vertex.
    pub fn find_polys_around_shape(
        &mut self,
        start_ref: PolyRef,
        verts: &[f32],
        filter: &dyn QueryFilter,
        out: &mut Vec<(PolyRef, PolyRef, f32)>,
        max: usize,
    ) -> Result<Status, Error> {
        out.clear();
        let nverts = verts.len() / 3;
        if !self.nav.is_valid_poly_ref(start_ref)
            || nverts < 3
            || !verts.iter().all(|v| v.is_finite())
        {
            return Err(Error::InvalidParameter("ref or shape"));
        }
        let mut c = [0.0f32; 3];
        for i in 0..nverts {
            c = vadd(&c, &verts[i * 3..]);
        }
        let c = vscale(&c, 1.0 / nverts as f32);
        self.polys_around(start_ref, c, Region::Shape(verts, nverts), filter, out, max)
    }

    /// Path from the start of the last Dijkstra search (circle, shape, distance to
    /// wall) to `end_ref` (`getPathFromDijkstraSearch`).
    pub fn path_from_dijkstra_search(
        &self,
        end_ref: PolyRef,
        path: &mut Vec<PolyRef>,
        max_path: usize,
    ) -> Result<Status, Error> {
        path.clear();
        if !self.nav.is_valid_poly_ref(end_ref) {
            return Err(Error::InvalidParameter("end_ref"));
        }
        let mut found = Vec::new();
        self.node_pool.find_nodes(end_ref, &mut found, 1);
        match found.first() {
            Some(&n) if self.node_pool.nodes[n].flags & NODE_CLOSED != 0 => {
                // Same walk as findPath's getPathToNode.
                let nodes = &self.node_pool.nodes;
                let mut length = 0usize;
                let mut cur = Some(n);
                while let Some(c) = cur {
                    length += 1;
                    cur = NodePool::at_idx(nodes[c].pidx);
                }
                let mut cur = Some(n);
                let mut write = length;
                while write > max_path {
                    cur = cur.and_then(|c| NodePool::at_idx(nodes[c].pidx));
                    write -= 1;
                }
                path.resize(write, 0);
                for i in (0..write).rev() {
                    let Some(c) = cur else { break };
                    path[i] = nodes[c].id;
                    cur = NodePool::at_idx(nodes[c].pidx);
                }
                Ok(Status(
                    Status::SUCCESS
                        | if length > max_path {
                            Status::BUFFER_TOO_SMALL
                        } else {
                            0
                        },
                ))
            }
            _ => Err(Error::InvalidParameter("end_ref not reached")),
        }
    }

    /// Non-overlapping polygons around `center` (`findLocalNeighbourhood`): (ref,
    /// parent) per polygon, at most `max`.
    pub fn find_local_neighbourhood(
        &mut self,
        start_ref: PolyRef,
        center: &[f32; 3],
        radius: f32,
        filter: &dyn QueryFilter,
        out: &mut Vec<(PolyRef, PolyRef)>,
        max: usize,
    ) -> Result<Status, Error> {
        out.clear();
        if !self.nav.is_valid_poly_ref(start_ref)
            || !visfinite(center)
            || radius < 0.0
            || !radius.is_finite()
        {
            return Err(Error::InvalidParameter("ref, center or radius"));
        }
        const MAX_STACK: usize = 48;
        let mut stack: Vec<usize> = Vec::with_capacity(MAX_STACK);
        self.tiny_node_pool.clear();
        let sn = self
            .tiny_node_pool
            .get_node(start_ref, 0)
            .ok_or(Error::AllocationFailed)?;
        {
            let n = &mut self.tiny_node_pool.nodes[sn];
            n.pidx = 0;
            n.id = start_ref;
            n.flags = NODE_CLOSED;
        }
        stack.push(sn);
        let radius_sqr = sqr(radius);
        let mut pa = [0.0f32; VERTS_PER_POLYGON * 3];
        let mut pb = [0.0f32; VERTS_PER_POLYGON * 3];
        let mut status = Status::SUCCESS;
        if max > 0 {
            out.push((start_ref, 0));
        } else {
            status |= Status::BUFFER_TOO_SMALL;
        }
        while !stack.is_empty() {
            let cur = stack.remove(0);
            let cur_tp = self.tp(self.tiny_node_pool.nodes[cur].id);
            let (cur_ref, ctile, cpoly) = cur_tp;
            let mut li = cpoly.first_link;
            while li != NULL_LINK {
                let link = ctile.links[li as usize];
                li = link.next;
                let nref = link.r#ref;
                if nref == 0 {
                    continue;
                }
                let Some(nn) = self.tiny_node_pool.get_node(nref, 0) else {
                    continue;
                };
                if self.tiny_node_pool.nodes[nn].flags & NODE_CLOSED != 0 {
                    continue;
                }
                let nei = self.tp(nref);
                if nei.2.poly_type() == POLYTYPE_OFFMESH_CONNECTION
                    || !filter.pass_filter(nref, nei.1, nei.2)
                {
                    continue;
                }
                let Some((va, vb)) = self.portal_points_tp(cur_tp, nei) else {
                    continue;
                };
                if distance_pt_seg_sqr_2d(center, &va, &vb).0 > radius_sqr {
                    continue;
                }
                {
                    let n = &mut self.tiny_node_pool.nodes[nn];
                    n.flags |= NODE_CLOSED;
                    n.pidx = NodePool::idx(cur);
                }
                let npa = nei.2.vert_count as usize;
                for k in 0..npa {
                    let v = nei.2.verts[k] as usize * 3;
                    pa[k * 3..k * 3 + 3].copy_from_slice(&nei.1.verts[v..v + 3]);
                }
                let mut overlap = false;
                for &(past_ref, _) in out.iter() {
                    let mut connected = false;
                    let mut k = cpoly.first_link;
                    while k != NULL_LINK {
                        if ctile.links[k as usize].r#ref == past_ref {
                            connected = true;
                            break;
                        }
                        k = ctile.links[k as usize].next;
                    }
                    if connected {
                        continue;
                    }
                    let (_, pt, pp) = self.tp(past_ref);
                    let npb = pp.vert_count as usize;
                    for k in 0..npb {
                        let v = pp.verts[k] as usize * 3;
                        pb[k * 3..k * 3 + 3].copy_from_slice(&pt.verts[v..v + 3]);
                    }
                    if overlap_poly_poly_2d(&pa, npa, &pb, npb) {
                        overlap = true;
                        break;
                    }
                }
                if overlap {
                    continue;
                }
                if out.len() < max {
                    out.push((nref, cur_ref));
                } else {
                    status |= Status::BUFFER_TOO_SMALL;
                }
                if stack.len() < MAX_STACK {
                    stack.push(nn);
                }
            }
        }
        Ok(Status(status))
    }

    /// Wall and (optionally) portal segments of a polygon (`getPolyWallSegments`),
    /// at most `max`. Portal segments carry the neighbour ref; without
    /// `store_portals` only walls are returned.
    pub fn poly_wall_segments(
        &self,
        r: PolyRef,
        filter: &dyn QueryFilter,
        store_portals: bool,
        out: &mut Vec<SegmentWall>,
        max: usize,
    ) -> Result<Status, Error> {
        out.clear();
        let (_, tile, poly) = self.valid_tp(r).map_err(|_| Error::InvalidRef)?;
        let base = self.nav.base_ref(self.nav.decode_poly_id_tile(r) as usize);
        const MAX_INTERVAL: usize = 16;
        let mut status = Status::SUCCESS;
        let push = |out: &mut Vec<SegmentWall>,
                    a: [f32; 3],
                    b: [f32; 3],
                    nr: PolyRef,
                    status: &mut u32| {
            if out.len() < max {
                out.push(SegmentWall {
                    seg: [a[0], a[1], a[2], b[0], b[1], b[2]],
                    r#ref: nr,
                });
            } else {
                *status |= Status::BUFFER_TOO_SMALL;
            }
        };
        let nv = poly.vert_count as usize;
        let mut j = nv - 1;
        for i in 0..nv {
            let mut ints: Vec<(PolyRef, i16, i16)> = Vec::with_capacity(MAX_INTERVAL);
            let insert = |ints: &mut Vec<(PolyRef, i16, i16)>, tmin: i16, tmax: i16, r: PolyRef| {
                if ints.len() + 1 > MAX_INTERVAL {
                    return;
                }
                let idx = ints
                    .iter()
                    .position(|it| tmax <= it.1)
                    .unwrap_or(ints.len());
                ints.insert(idx, (r, tmin, tmax));
            };
            let vj = v3(&tile.verts[poly.verts[j] as usize * 3..]);
            let vi = v3(&tile.verts[poly.verts[i] as usize * 3..]);
            if poly.neis[j] & EXT_LINK != 0 {
                let mut k = poly.first_link;
                while k != NULL_LINK {
                    let link = &tile.links[k as usize];
                    k = link.next;
                    if link.edge as usize == j && link.r#ref != 0 {
                        let nt = self.tp(link.r#ref);
                        if filter.pass_filter(link.r#ref, nt.1, nt.2) {
                            insert(&mut ints, link.bmin as i16, link.bmax as i16, link.r#ref);
                        }
                    }
                }
            } else {
                let mut nei_ref: PolyRef = 0;
                if poly.neis[j] != 0 {
                    let idx = (poly.neis[j] - 1) as usize;
                    nei_ref = base | idx as PolyRef;
                    if !filter.pass_filter(nei_ref, tile, &tile.polys[idx]) {
                        nei_ref = 0;
                    }
                }
                if nei_ref == 0 || store_portals {
                    push(out, vj, vi, nei_ref, &mut status);
                }
                j = i;
                continue;
            }
            insert(&mut ints, -1, 0, 0);
            insert(&mut ints, 255, 256, 0);
            for k in 1..ints.len() {
                if store_portals && ints[k].0 != 0 {
                    let tmin = ints[k].1 as f32 / 255.0;
                    let tmax = ints[k].2 as f32 / 255.0;
                    push(
                        out,
                        vlerp(&vj, &vi, tmin),
                        vlerp(&vj, &vi, tmax),
                        ints[k].0,
                        &mut status,
                    );
                }
                let imin = ints[k - 1].2 as i32;
                let imax = ints[k].1 as i32;
                if imin != imax {
                    let tmin = imin as f32 / 255.0;
                    let tmax = imax as f32 / 255.0;
                    push(
                        out,
                        vlerp(&vj, &vi, tmin),
                        vlerp(&vj, &vi, tmax),
                        0,
                        &mut status,
                    );
                }
            }
            j = i;
        }
        Ok(Status(status))
    }

    /// Distance from `center` to the nearest wall within `max_radius`
    /// (`findDistanceToWall`): distance, hit point and normal. When no wall is in
    /// range the hit point stays at the origin, as upstream leaves its output.
    pub fn find_distance_to_wall(
        &mut self,
        start_ref: PolyRef,
        center: &[f32; 3],
        max_radius: f32,
        filter: &dyn QueryFilter,
    ) -> Result<(f32, [f32; 3], [f32; 3], Status), Error> {
        if !self.nav.is_valid_poly_ref(start_ref)
            || !visfinite(center)
            || max_radius < 0.0
            || !max_radius.is_finite()
        {
            return Err(Error::InvalidParameter("ref, center or radius"));
        }
        self.node_pool.clear();
        self.open_list.clear();
        let sn = self
            .node_pool
            .get_node(start_ref, 0)
            .ok_or(Error::AllocationFailed)?;
        {
            let n = &mut self.node_pool.nodes[sn];
            n.pos = *center;
            n.pidx = 0;
            n.cost = 0.0;
            n.total = 0.0;
            n.id = start_ref;
            n.flags = NODE_OPEN;
        }
        self.open_list.push(sn, &self.node_pool.nodes);
        let mut radius_sqr = sqr(max_radius);
        let mut hit_pos = [0.0f32; 3];
        let mut status = Status::SUCCESS;
        while !self.open_list.empty() {
            let bn = self.open_list.pop(&self.node_pool.nodes);
            self.node_pool.nodes[bn].flags &= !NODE_OPEN;
            self.node_pool.nodes[bn].flags |= NODE_CLOSED;
            let best = self.tp(self.node_pool.nodes[bn].id);
            let (best_ref, btile, bpoly) = best;
            let parent_ref = NodePool::at_idx(self.node_pool.nodes[bn].pidx)
                .map_or(0, |i| self.node_pool.nodes[i].id);
            let base = self
                .nav
                .base_ref(self.nav.decode_poly_id_tile(best_ref) as usize);
            let nv = bpoly.vert_count as usize;
            let mut j = nv - 1;
            for i in 0..nv {
                let jj = j;
                j = i;
                if bpoly.neis[jj] & EXT_LINK != 0 {
                    let mut solid = true;
                    let mut k = bpoly.first_link;
                    while k != NULL_LINK {
                        let link = &btile.links[k as usize];
                        k = link.next;
                        if link.edge as usize == jj {
                            if link.r#ref != 0 {
                                let nt = self.tp(link.r#ref);
                                if filter.pass_filter(link.r#ref, nt.1, nt.2) {
                                    solid = false;
                                }
                            }
                            break;
                        }
                    }
                    if !solid {
                        continue;
                    }
                } else if bpoly.neis[jj] != 0 {
                    let idx = (bpoly.neis[jj] - 1) as usize;
                    if filter.pass_filter(base | idx as PolyRef, btile, &btile.polys[idx]) {
                        continue;
                    }
                }
                let vj = &btile.verts[bpoly.verts[jj] as usize * 3..];
                let vi = &btile.verts[bpoly.verts[i] as usize * 3..];
                let (d, tseg) = distance_pt_seg_sqr_2d(center, vj, vi);
                if d > radius_sqr {
                    continue;
                }
                radius_sqr = d;
                hit_pos = [
                    vj[0] + (vi[0] - vj[0]) * tseg,
                    vj[1] + (vi[1] - vj[1]) * tseg,
                    vj[2] + (vi[2] - vj[2]) * tseg,
                ];
            }
            let mut li = bpoly.first_link;
            while li != NULL_LINK {
                let link = btile.links[li as usize];
                li = link.next;
                let nref = link.r#ref;
                if nref == 0 || nref == parent_ref {
                    continue;
                }
                let nei = self.tp(nref);
                if nei.2.poly_type() == POLYTYPE_OFFMESH_CONNECTION {
                    continue;
                }
                let va = &btile.verts[bpoly.verts[link.edge as usize] as usize * 3..];
                let vb = &btile.verts[bpoly.verts[(link.edge as usize + 1) % nv] as usize * 3..];
                if distance_pt_seg_sqr_2d(center, va, vb).0 > radius_sqr {
                    continue;
                }
                if !filter.pass_filter(nref, nei.1, nei.2) {
                    continue;
                }
                let Some(nn) = self.node_pool.get_node(nref, 0) else {
                    status |= Status::OUT_OF_NODES;
                    continue;
                };
                if self.node_pool.nodes[nn].flags & NODE_CLOSED != 0 {
                    continue;
                }
                if self.node_pool.nodes[nn].flags == 0 {
                    if let Some(mid) = self.edge_mid_point_tp(best, nei) {
                        self.node_pool.nodes[nn].pos = mid;
                    }
                }
                let bpos = self.node_pool.nodes[bn].pos;
                let total =
                    self.node_pool.nodes[bn].total + vdist(&bpos, &self.node_pool.nodes[nn].pos);
                let n = &mut self.node_pool.nodes[nn];
                if n.flags & NODE_OPEN != 0 && no_better(total, n.total) {
                    continue;
                }
                n.id = nref;
                n.flags &= !NODE_CLOSED;
                n.pidx = NodePool::idx(bn);
                n.total = total;
                if n.flags & NODE_OPEN != 0 {
                    self.open_list.modify(nn, &self.node_pool.nodes);
                } else {
                    n.flags |= NODE_OPEN;
                    self.open_list.push(nn, &self.node_pool.nodes);
                }
            }
        }
        let mut normal = vsub(center, &hit_pos);
        vnormalize(&mut normal);
        Ok((
            crate::common::sqrtf(radius_sqr),
            hit_pos,
            normal,
            Status(status),
        ))
    }

    /// Whether a ref is valid and passes the filter (`isValidPolyRef`).
    pub fn is_valid_poly_ref(&self, r: PolyRef, filter: &dyn QueryFilter) -> bool {
        match self.nav.tile_and_poly_by_ref(r) {
            Ok((t, p)) => filter.pass_filter(r, t, p),
            Err(_) => false,
        }
    }

    /// Whether the last search closed the polygon (`isInClosedList`).
    pub fn is_in_closed_list(&self, r: PolyRef) -> bool {
        let mut found = Vec::new();
        self.node_pool.find_nodes(r, &mut found, 4);
        found
            .iter()
            .any(|&n| self.node_pool.nodes[n].flags & NODE_CLOSED != 0)
    }
}
