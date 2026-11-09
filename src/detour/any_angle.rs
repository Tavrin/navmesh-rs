//! Root/interval any-angle search, derived from Cui, Harabor and Grastien,
//! IJCAI 2017, <https://doi.org/10.24963/ijcai.2017/70>.
//!
//! Distances and visibility are in xz. Heights are sampled from detail surfaces.
//! Non-observable successors turn only at interval endpoints that are mesh
//! boundary vertices (corner pruning); no further tautness pruning is applied.
use super::{NULL_LINK, NavMeshQuery, OFFMESH_CON_BIDIR, POLYTYPE_GROUND, PolyRef, QueryFilter};
use crate::common::sqrtf;
use crate::{Budget, Budgeted, Error};
use alloc::{collections::BinaryHeap, sync::Arc, vec::Vec};
use core::cmp::Ordering;

/// A position and its containing ground polygon.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct AnyAnglePoint {
    /// Ground polygon containing `position` in xz.
    pub polygon: PolyRef,
    /// World position; y is projected onto the polygon's detail surface.
    pub position: [f32; 3],
}
/// Result of a uniform-cost search. An exhausted search may have no path yet.
#[derive(Clone, Debug, Default, PartialEq)]
pub struct AnyAnglePath {
    /// Root points followed by the goal; empty if no candidate has been found.
    pub points: Vec<[f32; 3]>,
    /// Total xz distance, independent of the uniform cost multiplier.
    pub length: f32,
    /// Polygon for each segment's departure point, followed by the goal polygon.
    /// An off-mesh segment carries its connection polygon; ground segments carry
    /// a containing ground polygon for validation and height lookup.
    pub polygons: Vec<PolyRef>,
    /// Number of root/interval states expanded.
    pub expanded: u64,
    /// Allocated search-state, dominance-index and open-queue capacity in bytes, excluding results.
    pub search_bytes: usize,
}
#[derive(Clone)]
struct Node {
    root: [f32; 3],
    root_poly: PolyRef,
    interval: Option<[[f32; 3]; 2]>,
    poly: PolyRef,
    g: f32,
    parent: Option<usize>,
    turns: [bool; 2],
    hash_next: Option<usize>,
}
#[derive(Clone, Copy)]
struct Open {
    cost: f32,
    index: usize,
}
impl PartialEq for Open {
    fn eq(&self, other: &Self) -> bool {
        self.cost.to_bits() == other.cost.to_bits() && self.index == other.index
    }
}
impl Eq for Open {}
impl Ord for Open {
    fn cmp(&self, other: &Self) -> Ordering {
        other
            .cost
            .total_cmp(&self.cost)
            .then(other.index.cmp(&self.index))
    }
}
impl PartialOrd for Open {
    fn partial_cmp(&self, other: &Self) -> Option<Ordering> {
        Some(self.cmp(other))
    }
}
/// Resumable interval search. Retained states are capped by the query's node cap.
///
/// Each resume charges one work unit per expanded state. It rejects a different
/// mesh or removal of a touched tile; additions may be missed, as in sliced A*.
/// The filter is borrowed for
/// the search lifetime and must keep its traversal decisions stable. Discard the
/// search after any error; only exhausted progress can be resumed.
pub struct AnyAngleSearch<'f, F: QueryFilter + ?Sized> {
    filter: &'f F,
    owner: Arc<()>,
    epoch: u64,
    nodes: Vec<Node>,
    open: BinaryHeap<Open>,
    buckets: Vec<Option<usize>>,
    end: AnyAnglePoint,
    cap: usize,
    best: Option<(usize, Option<[f32; 3]>, f32)>,
    expanded: u64,
    done: bool,
    failure: Option<Error>,
}
fn cross(a: [f32; 3], b: [f32; 3], c: [f32; 3]) -> f64 {
    (b[0] as f64 - a[0] as f64) * (c[2] as f64 - a[2] as f64)
        - (b[2] as f64 - a[2] as f64) * (c[0] as f64 - a[0] as f64)
}
fn distance(a: [f32; 3], b: [f32; 3]) -> f32 {
    let dx = a[0] - b[0];
    let dz = a[2] - b[2];
    sqrtf(dx * dx + dz * dz)
}
fn lerp(a: [f32; 3], b: [f32; 3], t: f64) -> [f32; 3] {
    core::array::from_fn(|i| (a[i] as f64 + (b[i] as f64 - a[i] as f64) * t) as f32)
}
fn on_segment(p: [f32; 3], a: [f32; 3], b: [f32; 3]) -> bool {
    cross(a, b, p) == 0.0
        && p[0] >= a[0].min(b[0])
        && p[0] <= a[0].max(b[0])
        && p[2] >= a[2].min(b[2])
        && p[2] <= a[2].max(b[2])
}
// Detour's point-in-polygon test excludes some exact boundary points. Use
// detail-edge height only after independently proving membership on an edge.
fn surface_height(
    query: &NavMeshQuery<'_>,
    polygon: PolyRef,
    point: [f32; 3],
) -> Result<f32, Error> {
    let height = match query.poly_height(polygon, &point) {
        Ok(height) => Ok(height),
        Err(error) => {
            let (_, tile, poly) = query.valid_tp(polygon)?;
            let vertex = |i: usize| {
                let v = poly.verts[i] as usize * 3;
                [tile.verts[v], tile.verts[v + 1], tile.verts[v + 2]]
            };
            if (0..poly.vert_count as usize)
                .any(|i| on_segment(point, vertex(i), vertex((i + 1) % poly.vert_count as usize)))
            {
                query
                    .closest_point_on_poly(polygon, &point)
                    .map(|(p, _)| p[1])
            } else {
                Err(error)
            }
        }
    }?;
    if height.is_finite() {
        Ok(height)
    } else {
        Err(Error::InvalidParameter("any-angle height range"))
    }
}
// Quantized link bounds can extend beyond the receiving polygon. Intersect
// the interval with that convex polygon before projecting or creating roots.
fn portal_in_polygon(
    edge: [[f32; 3]; 2],
    tile: &super::MeshTile,
    poly: &super::Poly,
) -> Option<[[f32; 3]; 2]> {
    let count = poly.vert_count as usize;
    let vertex = |i: usize| {
        let v = poly.verts[i] as usize * 3;
        [tile.verts[v], tile.verts[v + 1], tile.verts[v + 2]]
    };
    let winding = (0..count)
        .map(|i| cross([0.0; 3], vertex(i), vertex((i + 1) % count)))
        .sum::<f64>();
    if winding == 0.0 {
        return None;
    }
    let sign = if winding > 0.0 { 1.0 } else { -1.0 };
    // The shared edge of a neighbour in another tile is collinear with the
    // portal up to f32 rounding of each tile's vertices; it does not bound it.
    let mut skip = 0u32;
    let mut lo = 0.0f64;
    let mut hi = 1.0f64;
    for i in 0..count {
        let a = vertex(i);
        let b = vertex((i + 1) % count);
        let u = cross(a, b, edge[0]) * sign;
        let v = cross(a, b, edge[1]) * sign;
        if u >= 0.0 && v >= 0.0 {
            continue;
        }
        let tolerance = rounding(&[a, b, edge[0], edge[1]]) * f64::from(distance(a, b));
        if u.abs() <= tolerance && v.abs() <= tolerance {
            skip |= 1 << i;
            continue;
        }
        if u < 0.0 && v < 0.0 {
            return None;
        }
        if u < 0.0 {
            lo = lo.max(u / (u - v));
        }
        if v < 0.0 {
            hi = hi.min(u / (u - v));
        }
    }
    if lo == 0.0 && hi == 1.0 {
        return Some(edge);
    }
    settle(edge, lo, hi, |p| {
        (0..count).all(|i| {
            skip & (1 << i) != 0 || cross(vertex(i), vertex((i + 1) % count), p) * sign >= 0.0
        })
    })
}
// Distance under which two tiles' copies of a border vertex or edge are the
// same: eight f32 ulps of the coordinate magnitude.
fn rounding(points: &[[f32; 3]]) -> f64 {
    let magnitude = points
        .iter()
        .flat_map(|p| [p[0].abs(), p[2].abs()])
        .fold(1.0f32, f32::max);
    f64::from(magnitude) * f64::from(f32::EPSILON) * 8.0
}
// Round clipped interval endpoints inward: an f32 endpoint must itself satisfy
// every clipping constraint, so rounding can never widen a visibility cone.
fn settle(
    edge: [[f32; 3]; 2],
    mut lo: f64,
    mut hi: f64,
    inside: impl Fn([f32; 3]) -> bool,
) -> Option<[[f32; 3]; 2]> {
    if lo > hi {
        return None;
    }
    // An unclipped endpoint satisfied every constraint before clipping.
    let mut a = if lo == 0.0 {
        edge[0]
    } else {
        lerp(edge[0], edge[1], lo)
    };
    let mut step = 1e-7;
    while lo != 0.0 && !inside(a) {
        lo += step;
        step *= 4.0;
        if lo > hi || step > 1e-3 {
            return None;
        }
        a = lerp(edge[0], edge[1], lo);
    }
    let mut b = if hi == 1.0 {
        edge[1]
    } else {
        lerp(edge[0], edge[1], hi)
    };
    let mut step = 1e-7;
    while hi != 1.0 && !inside(b) {
        hi -= step;
        step *= 4.0;
        if lo > hi || step > 1e-3 {
            return None;
        }
        b = lerp(edge[0], edge[1], hi);
    }
    Some([a, b])
}
// Clip a segment to the visible cone and the far side of the entering edge.
fn project(root: [f32; 3], mut gate: [[f32; 3]; 2], edge: [[f32; 3]; 2]) -> Option<[[f32; 3]; 2]> {
    if cross(root, gate[0], gate[1]) < 0.0 {
        gate.swap(0, 1);
    }
    if on_segment(root, gate[0], gate[1]) {
        return Some(edge);
    }
    let mut lo = 0.0f64;
    let mut hi = 1.0f64;
    let signed = cross(gate[0], gate[1], root);
    let side = if signed > 0.0 {
        1.0
    } else if signed < 0.0 {
        -1.0
    } else {
        0.0
    };
    if side == 0.0 {
        let near = if distance(root, gate[0]) <= distance(root, gate[1]) {
            gate[0]
        } else {
            gate[1]
        };
        let dot = |p: [f32; 3]| {
            (p[0] as f64 - near[0] as f64) * (near[0] as f64 - root[0] as f64)
                + (p[2] as f64 - near[2] as f64) * (near[2] as f64 - root[2] as f64)
        };
        let u = dot(edge[0]);
        let v = dot(edge[1]);
        if u < 0.0 && v < 0.0 {
            return None;
        }
        if u < 0.0 {
            lo = lo.max(u / (u - v));
        }
        if v < 0.0 {
            hi = hi.min(u / (u - v));
        }
    }
    let planes = [
        (root, gate[0], 1.0),
        (root, gate[1], -1.0),
        (gate[0], gate[1], -side),
    ];
    for (a, b, sign) in planes {
        let u = cross(a, b, edge[0]) * sign;
        let v = cross(a, b, edge[1]) * sign;
        if u < 0.0 && v < 0.0 {
            return None;
        }
        if u < 0.0 {
            lo = lo.max(u / (u - v));
        }
        if v < 0.0 {
            hi = hi.min(u / (u - v));
        }
    }
    let near = if distance(root, gate[0]) <= distance(root, gate[1]) {
        gate[0]
    } else {
        gate[1]
    };
    settle(edge, lo, hi, |p| {
        planes
            .iter()
            .all(|&(a, b, sign)| cross(a, b, p) * sign >= 0.0)
            && (side != 0.0
                || (p[0] as f64 - near[0] as f64) * (near[0] as f64 - root[0] as f64)
                    + (p[2] as f64 - near[2] as f64) * (near[2] as f64 - root[2] as f64)
                    >= 0.0)
    })
}
fn estimate(node: &Node, end: [f32; 3]) -> f32 {
    let Some([a, b]) = node.interval else {
        return distance(node.root, end);
    };
    if on_segment(node.root, a, b) {
        return distance(node.root, end);
    }
    let dx = b[0] as f64 - a[0] as f64;
    let dz = b[2] as f64 - a[2] as f64;
    let norm = dx * dx + dz * dz;
    if norm == 0.0 {
        return distance(node.root, a) + distance(a, end);
    }
    let mut target = end;
    if cross(a, b, node.root) * cross(a, b, end) > 0.0 {
        let scale = 2.0 * cross(a, b, end) / norm;
        target[0] = (end[0] as f64 + dz * scale) as f32;
        target[2] = (end[2] as f64 - dx * scale) as f32;
    }
    let denom = dx * (target[2] as f64 - node.root[2] as f64)
        - dz * (target[0] as f64 - node.root[0] as f64);
    if denom != 0.0 {
        let t = cross(node.root, target, a) / denom;
        if (0.0..=1.0).contains(&t) {
            return distance(node.root, target);
        }
    }
    (distance(node.root, a) + distance(a, end)).min(distance(node.root, b) + distance(b, end))
}
// A Euclidean shortest path bends only at obstacle vertices. Walk the polygon
// fan around `vertex` of `start`: it is interior when the walk returns to
// `start` across full ground portals that pass the filter. Partial portals,
// unknown topology or long fans conservatively count as corners.
fn is_corner<F: QueryFilter + ?Sized>(
    query: &NavMeshQuery<'_>,
    filter: &F,
    start: PolyRef,
    vertex: [f32; 3],
) -> Result<bool, Error> {
    let tolerance = rounding(&[vertex]);
    let mut current = start;
    for _ in 0..64 {
        let (_, tile, poly) = query.valid_tp(current)?;
        let count = poly.vert_count as usize;
        let position = |i: usize| {
            let v = poly.verts[i] as usize * 3;
            [tile.verts[v], tile.verts[v + 2]]
        };
        // Tile neighbours repeat a border vertex up to f32 rounding.
        let Some(i) = (0..count).find(|&i| {
            let p = position(i);
            (p[0] as f64 - vertex[0] as f64).abs() <= tolerance
                && (p[1] as f64 - vertex[2] as f64).abs() <= tolerance
        }) else {
            return Ok(true);
        };
        // Cross the edge leaving the vertex through the link that touches it:
        // an internal link, or a tile link whose range starts at the vertex.
        let mut next = None;
        let mut link_index = poly.first_link;
        while link_index != NULL_LINK {
            let link = tile.links[link_index as usize];
            link_index = link.next;
            if link.edge as usize != i || link.r#ref == 0 || (link.side != 0xff && link.bmin != 0) {
                continue;
            }
            if next.is_some() {
                return Ok(true);
            }
            next = Some(link.r#ref);
        }
        let Some(next) = next else {
            return Ok(true);
        };
        let (_, nt, np) = query.valid_tp(next)?;
        if np.poly_type() != POLYTYPE_GROUND || !filter.pass_filter(next, nt, np) {
            return Ok(true);
        }
        if next == start {
            return Ok(false);
        }
        current = next;
    }
    Ok(true)
}
// Shortest route from a node's root to a point of its polygon: straight when
// the point is in the visible interval cone, otherwise via the nearer interval
// endpoint. Both legs lie in polygons already proven visible or convex.
fn reach(node: &Node, point: [f32; 3]) -> (Option<[f32; 3]>, f32) {
    let Some(gate) = node.interval else {
        return (None, node.g + distance(node.root, point));
    };
    if project(node.root, gate, [point, point]).is_some() {
        return (None, node.g + distance(node.root, point));
    }
    let a = node.g + distance(node.root, gate[0]) + distance(gate[0], point);
    let b = node.g + distance(node.root, gate[1]) + distance(gate[1], point);
    if a <= b {
        (Some(gate[0]), a)
    } else {
        (Some(gate[1]), b)
    }
}
fn bucket(poly: PolyRef, root: [f32; 3], count: usize) -> usize {
    let mut hash = (poly as u64).wrapping_mul(0x100_0000_01b3);
    for coordinate in root {
        let bits = if coordinate == 0.0 {
            0
        } else {
            coordinate.to_bits()
        };
        hash = (hash ^ bits as u64).wrapping_mul(0x100_0000_01b3);
    }
    hash ^= hash >> 33;
    hash = hash.wrapping_mul(0xff51_afd7_ed55_8ccd);
    hash ^= hash >> 33;
    hash = hash.wrapping_mul(0xc4ce_b9fe_1a85_ec53);
    hash ^= hash >> 33;
    (hash as usize) & (count - 1)
}
#[inline(always)]
fn same(a: [f32; 3], b: [f32; 3]) -> bool {
    a == b
}
#[inline(always)]
fn contains(outer: [[f32; 3]; 2], inner: [[f32; 3]; 2]) -> bool {
    inner.iter().all(|&p| {
        p[0] >= outer[0][0].min(outer[1][0])
            && p[0] <= outer[0][0].max(outer[1][0])
            && p[2] >= outer[0][2].min(outer[1][2])
            && p[2] <= outer[0][2].max(outer[1][2])
            && cross(outer[0], outer[1], p) == 0.0
    })
}
impl<'f, F: QueryFilter + ?Sized> AnyAngleSearch<'f, F> {
    /// Validate endpoints and begin without expanding a node.
    /// Non-uniform costs and off-mesh endpoints return [`Error::Unsupported`].
    /// Passable off-mesh connections are traversed in their linked direction at
    /// their xz length: the path reaches the departure vertex, then continues
    /// from the landing vertex. The departure point carries the connection
    /// polygon in [`AnyAnglePath::polygons`]; the segment after it is the
    /// connection itself, not a ground segment.
    pub fn begin(
        query: &NavMeshQuery<'_>,
        mut start: AnyAnglePoint,
        mut end: AnyAnglePoint,
        filter: &'f F,
    ) -> Result<Self, Error> {
        if !filter
            .uniform_cost()
            .is_some_and(|v| v.is_finite() && v > 0.0)
        {
            return Err(Error::Unsupported);
        }
        for point in [&mut start, &mut end] {
            if point.position.iter().any(|v| !v.is_finite()) {
                return Err(Error::InvalidParameter("any-angle endpoint"));
            }
            let (_, tile, poly) = query.valid_tp(point.polygon)?;
            if poly.poly_type() != POLYTYPE_GROUND {
                return Err(Error::Unsupported);
            }
            if !filter.pass_filter(point.polygon, tile, poly) {
                return Err(Error::InvalidParameter("filtered endpoint"));
            }
            point.position[1] = surface_height(query, point.polygon, point.position)?;
        }
        if !distance(start.position, end.position).is_finite() {
            return Err(Error::InvalidParameter("any-angle coordinate range"));
        }
        let mut nodes = Vec::new();
        nodes.try_reserve(1).map_err(|_| Error::AllocationFailed)?;
        nodes.push(Node {
            root: start.position,
            root_poly: start.polygon,
            interval: None,
            poly: start.polygon,
            g: 0.0,
            parent: None,
            turns: [false; 2],
            hash_next: None,
        });
        let bucket_count = (query.max_nodes() as usize / 4).max(1).next_power_of_two();
        let mut buckets = Vec::new();
        buckets
            .try_reserve_exact(bucket_count)
            .map_err(|_| Error::AllocationFailed)?;
        buckets.resize(bucket_count, None);
        buckets[bucket(start.polygon, start.position, bucket_count)] = Some(0);
        let mut open = BinaryHeap::new();
        open.try_reserve(1).map_err(|_| Error::AllocationFailed)?;
        open.push(Open {
            cost: distance(start.position, end.position),
            index: 0,
        });
        Ok(Self {
            open,
            buckets,
            filter,
            owner: query.nav().report_identity().clone(),
            epoch: query.nav().epoch(),
            nodes,
            end,
            cap: query.max_nodes() as usize,
            best: None,
            expanded: 0,
            done: false,
            failure: None,
        })
    }
    fn push(&mut self, mut node: Node) -> Result<(), Error> {
        if !node.g.is_finite()
            || !node.root.iter().all(|v| v.is_finite())
            || node
                .interval
                .is_some_and(|gate| gate.iter().flatten().any(|v| !v.is_finite()))
        {
            return Err(Error::InvalidParameter("any-angle path range"));
        }
        // State dominance is local to a root and an opposite polygon, never xz
        // alone: projected floors and distinct portal intervals stay separate.
        let slot = bucket(node.poly, node.root, self.buckets.len());
        let mut prior = self.buckets[slot];
        while let Some(index) = prior {
            let old = &self.nodes[index];
            if old.poly == node.poly
                && same(old.root, node.root)
                && old.g <= node.g
                && match (old.interval, node.interval) {
                    (None, _) => true,
                    (Some(a), Some(b)) => contains(a, b),
                    _ => false,
                }
            {
                return Ok(());
            }
            prior = old.hash_next;
        }
        if self.nodes.len() >= self.cap {
            return Err(Error::LimitExceeded);
        }
        self.nodes
            .try_reserve(1)
            .map_err(|_| Error::AllocationFailed)?;
        self.open
            .try_reserve(1)
            .map_err(|_| Error::AllocationFailed)?;
        self.open.push(Open {
            cost: node.g + estimate(&node, self.end.position),
            index: self.nodes.len(),
        });
        node.hash_next = self.buckets[slot];
        self.buckets[slot] = Some(self.nodes.len());
        self.nodes.push(node);
        Ok(())
    }
    // Retain a path-only state (a corner or an off-mesh departure) that is
    // never expanded; it still counts against the node cap.
    fn record(&mut self, node: Node) -> Result<usize, Error> {
        if self.nodes.len() >= self.cap {
            return Err(Error::LimitExceeded);
        }
        self.nodes
            .try_reserve(1)
            .map_err(|_| Error::AllocationFailed)?;
        self.nodes.push(node);
        Ok(self.nodes.len() - 1)
    }
    // Walk to the connection's departure vertex inside `node.poly`, traverse it
    // at its xz length, and root a new search at the landing vertex.
    fn traverse_off_mesh(
        &mut self,
        query: &NavMeshQuery<'_>,
        index: usize,
        node: &Node,
        connection: PolyRef,
    ) -> Result<(), Error> {
        let (_, tile, poly) = query.valid_tp(connection)?;
        let bidirectional = query
            .nav()
            .off_mesh_connection_by_ref(connection)
            .is_some_and(|c| c.flags & OFFMESH_CON_BIDIR != 0);
        let mut landing = [None; 2];
        let mut link_index = poly.first_link;
        while link_index != NULL_LINK {
            let link = tile.links[link_index as usize];
            link_index = link.next;
            if link.r#ref != 0 && link.edge < 2 && landing[link.edge as usize].is_none() {
                landing[link.edge as usize] = Some(link.r#ref);
            }
        }
        let vertex = |i: usize| {
            let v = poly.verts[i] as usize * 3;
            [tile.verts[v], tile.verts[v + 1], tile.verts[v + 2]]
        };
        for (from, to) in [(0usize, 1usize), (1, 0)] {
            if (from == 1 && !bidirectional) || landing[from] != Some(node.poly) {
                continue;
            }
            let Some(target) = landing[to] else {
                continue;
            };
            let (_, lt, lp) = query.valid_tp(target)?;
            if lp.poly_type() != POLYTYPE_GROUND || !self.filter.pass_filter(target, lt, lp) {
                continue;
            }
            let departure = vertex(from);
            let arrival = vertex(to);
            let (corner, cost) = reach(node, departure);
            let mut parent = index;
            if let Some(corner) = corner {
                parent = self.record(Node {
                    root: corner,
                    root_poly: node.poly,
                    interval: None,
                    poly: node.poly,
                    g: cost - distance(corner, departure),
                    parent: Some(parent),
                    turns: [false; 2],
                    hash_next: None,
                })?;
            }
            parent = self.record(Node {
                root: departure,
                root_poly: connection,
                interval: None,
                poly: connection,
                g: cost,
                parent: Some(parent),
                turns: [false; 2],
                hash_next: None,
            })?;
            self.push(Node {
                root: arrival,
                root_poly: target,
                interval: None,
                poly: target,
                g: cost + distance(departure, arrival),
                parent: Some(parent),
                turns: [false; 2],
                hash_next: None,
            })?;
        }
        Ok(())
    }
    fn result(&self) -> Result<AnyAnglePath, Error> {
        let mut points = Vec::new();
        let mut polygons = Vec::new();
        let mut length = 0.0;
        if let Some((index, corner, cost)) = self.best {
            let mut count = 2usize;
            let mut current = Some(index);
            while let Some(i) = current {
                count += 1;
                current = self.nodes[i].parent;
            }
            points
                .try_reserve_exact(count)
                .map_err(|_| Error::AllocationFailed)?;
            polygons
                .try_reserve_exact(count)
                .map_err(|_| Error::AllocationFailed)?;
            length = cost;
            let mut current = Some(index);
            while let Some(i) = current {
                if points.last() != Some(&self.nodes[i].root) {
                    points.push(self.nodes[i].root);
                    polygons.push(self.nodes[i].root_poly);
                }
                current = self.nodes[i].parent;
            }
            points.reverse();
            polygons.reverse();
            if let Some(c) = corner {
                if points.last() != Some(&c) {
                    points.push(c);
                    polygons.push(self.nodes[index].poly);
                }
            }
            if points.last() != Some(&self.end.position) {
                points.push(self.end.position);
                polygons.push(self.end.polygon);
            }
        }
        Ok(AnyAnglePath {
            points,
            polygons,
            length,
            expanded: self.expanded,
            search_bytes: self.nodes.capacity() * core::mem::size_of::<Node>()
                + self.open.capacity() * core::mem::size_of::<Open>()
                + self.buckets.capacity() * core::mem::size_of::<Option<usize>>(),
        })
    }
    /// Expand until the allowance refuses a state or the shortest path is settled.
    /// A retained-state overflow returns [`Error::LimitExceeded`], never a path
    /// labelled optimal. Empty exhausted paths mean that no upper bound exists yet.
    pub fn resume(
        &mut self,
        query: &NavMeshQuery<'_>,
        budget: &mut Budget,
    ) -> Result<Budgeted<AnyAnglePath>, Error> {
        if let Some(error) = &self.failure {
            return Err(error.clone());
        }
        let result = self.advance(query, budget);
        if let Err(error) = &result {
            self.failure = Some(error.clone());
        }
        result
    }
    fn advance(
        &mut self,
        query: &NavMeshQuery<'_>,
        budget: &mut Budget,
    ) -> Result<Budgeted<AnyAnglePath>, Error> {
        let nav = query.nav();
        if !Arc::ptr_eq(&self.owner, nav.report_identity()) {
            return Err(Error::MeshChanged);
        }
        if nav.epoch() != self.epoch
            && nav.changed_since(self.epoch, &mut |slot| {
                nav.decode_poly_id_tile(self.end.polygon) == slot
                    || self.nodes.iter().any(|n| {
                        nav.decode_poly_id_tile(n.poly) == slot
                            || nav.decode_poly_id_tile(n.root_poly) == slot
                    })
            })
        {
            return Err(Error::MeshChanged);
        }
        if self.done {
            return Ok(Budgeted::Complete(self.result()?));
        }
        loop {
            let next = self.open.peek().map(|entry| entry.index);
            let Some(index) = next else {
                self.done = true;
                return if self.best.is_some() {
                    Ok(Budgeted::Complete(self.result()?))
                } else {
                    Err(Error::Failed)
                };
            };
            let node = self.nodes[index].clone();
            if self
                .best
                .is_some_and(|(_, _, g)| node.g + estimate(&node, self.end.position) >= g)
            {
                self.done = true;
                return Ok(Budgeted::Complete(self.result()?));
            }
            if !budget.spend(1) {
                return Ok(Budgeted::Exhausted(self.result()?));
            }
            self.open.pop();
            self.expanded += 1;
            if node.poly == self.end.polygon {
                let (corner, cost) = reach(&node, self.end.position);
                if self.best.is_none_or(|(_, _, g)| cost < g) {
                    self.best = Some((index, corner, cost));
                }
                // Convexity makes leaving the goal polygon unnecessary. Other
                // open intervals may still improve this candidate.
                continue;
            }
            let (_, tile, poly) = query.valid_tp(node.poly)?;
            // Turning at a vertex surrounded by walkable polygons is never taut.
            // Turned roots are shared by every exit edge, so their height is
            // looked up once. Clipped portal endpoints are rounded to f32;
            // the closest boundary point avoids requiring exact collinearity.
            // Caller endpoints still use the strict validation in begin.
            let mut turns = node.turns;
            let mut roots = [[0.0f32; 3]; 2];
            if let Some(gate) = node.interval {
                for side in 0..2 {
                    if turns[side] {
                        turns[side] = is_corner(query, self.filter, node.poly, gate[side])?;
                    }
                    if turns[side] {
                        roots[side] = gate[side];
                        roots[side][1] = query.closest_point_on_poly(node.poly, &gate[side])?.0[1];
                        turns[side] = !same(roots[side], node.root);
                    }
                }
            }
            let mut link_index = poly.first_link;
            while link_index != NULL_LINK {
                let link = tile.links[link_index as usize];
                link_index = link.next;
                if link.r#ref == 0 {
                    continue;
                }
                let (_, tt, tp) = query.valid_tp(link.r#ref)?;
                if !self.filter.pass_filter(link.r#ref, tt, tp) {
                    continue;
                }
                if tp.poly_type() != POLYTYPE_GROUND {
                    self.traverse_off_mesh(query, index, &node, link.r#ref)?;
                    continue;
                }
                let e = link.edge as usize;
                if e >= poly.vert_count as usize {
                    continue;
                }
                let vertex = |i: usize| {
                    let v = poly.verts[i] as usize * 3;
                    [tile.verts[v], tile.verts[v + 1], tile.verts[v + 2]]
                };
                let a = vertex(e);
                let b = vertex((e + 1) % poly.vert_count as usize);
                let edge = if link.side == 0xff {
                    [a, b]
                } else {
                    let scale = 1.0f32 / 255.0;
                    [
                        super::math::vlerp(&a, &b, link.bmin as f32 * scale),
                        super::math::vlerp(&a, &b, link.bmax as f32 * scale),
                    ]
                };
                let Some(edge) = portal_in_polygon(edge, tt, tp) else {
                    continue;
                };
                let observable = match node.interval {
                    None => Some(edge),
                    Some(gate) => project(node.root, gate, edge),
                };
                if let Some(interval) = observable {
                    self.push(Node {
                        root: node.root,
                        root_poly: node.root_poly,
                        interval: Some(interval),
                        poly: link.r#ref,
                        g: node.g,
                        parent: node.parent,
                        turns: [
                            same(interval[0], edge[0]) || same(interval[0], edge[1]),
                            same(interval[1], edge[0]) || same(interval[1], edge[1]),
                        ],
                        hash_next: None,
                    })?;
                }
                if node.interval.is_some() {
                    for (side, root) in roots.into_iter().enumerate() {
                        if !turns[side] {
                            continue;
                        }
                        let g = node.g + distance(node.root, root);
                        self.push(Node {
                            root,
                            root_poly: node.poly,
                            interval: Some(edge),
                            poly: link.r#ref,
                            g,
                            parent: Some(index),
                            turns: [true; 2],
                            hash_next: None,
                        })?;
                    }
                }
            }
        }
    }
}
impl NavMeshQuery<'_> {
    /// Find an xz shortest path with a uniform-cost filter, between ground
    /// endpoints, through ground polygons and passable off-mesh connections.
    /// See [`AnyAngleSearch::begin`] for the filter and off-mesh contract.
    /// Use [`AnyAngleSearch`] to retain progress across exhausted budgets.
    pub fn find_any_angle_path<F: QueryFilter + ?Sized>(
        &self,
        start: AnyAnglePoint,
        end: AnyAnglePoint,
        filter: &F,
        budget: &mut Budget,
    ) -> Result<Budgeted<AnyAnglePath>, Error> {
        AnyAngleSearch::begin(self, start, end, filter)?.resume(self, budget)
    }
}

#[cfg(test)]
mod tests {
    use super::project;

    #[test]
    fn point_interval_keeps_only_the_far_ray() {
        let root = [0.0; 3];
        let gate = [[1.0, 0.0, 0.0]; 2];
        assert_eq!(
            project(root, gate, [[-1.0, 0.0, 0.0], [2.0, 0.0, 0.0]]),
            Some([[1.0, 0.0, 0.0], [2.0, 0.0, 0.0]])
        );
        assert_eq!(project(root, gate, [[-0.5, 0.0, 0.0]; 2]), None);
    }
}
