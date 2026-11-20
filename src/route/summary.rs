//! Level-2 sector summaries and their `'DNSS'` record.

use super::wire::{Reader, Writer, check_bytes, ordered};
use super::{SectorKey, SectorParams};
use crate::detour::{
    EXT_LINK, Link, MeshTile, NULL_LINK, NavMesh, NavMeshParams, OFFMESH_CON_BIDIR,
    POLYTYPE_OFFMESH_CONNECTION, Poly, PolyRef, QueryFilter, content_hash,
};
use crate::{Budget, Budgeted, Error, Limits};
use alloc::collections::{BTreeMap, BinaryHeap};
use alloc::vec;
use alloc::vec::Vec;
use core::cmp::Reverse;

/// Magic bytes at the start of a sector summary record.
pub const SECTOR_MAGIC: [u8; 4] = *b"DNSS";
/// Version of the sector summary record this crate writes and reads.
pub const SECTOR_VERSION: u16 = 2;
/// Quantized cost of an unreachable transition pair.
pub const UNREACHABLE: u16 = 0xFFFF;

const HEADER: usize = 44;
const TILE: usize = 16;
const REGION: usize = 28;
const PORTAL: usize = 36;
const OFFMESH: usize = 32;
/// Most transitions (portals plus off-mesh entries) of one sector; the cost table of
/// a full sector is then 32 MiB.
const MAX_TRANSITIONS: usize = 4096;
const NONE: u32 = u32::MAX;

/// A tile a summary was derived from.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
#[non_exhaustive]
#[cfg_attr(feature = "serde", derive(serde::Serialize))]
pub struct SectorTile {
    /// Tile x.
    pub x: i32,
    /// Tile z.
    pub z: i32,
    /// Tile layer.
    pub layer: u8,
    /// First four bytes (little-endian) of the tile bytes' content hash.
    pub content_id: u32,
}

/// A component connected by reciprocal polygon links inside a sector.
#[derive(Clone, Copy, Debug, PartialEq)]
#[non_exhaustive]
#[cfg_attr(feature = "serde", derive(serde::Serialize))]
pub struct Region {
    /// Passable polygons in the region (off-mesh connection polygons included).
    pub polygon_count: u32,
    /// `[min x, min y, min z, max x, max y, max z]` of the region's polygon vertices.
    pub bbox: [f32; 6],
}

/// A transition point on a maximal passable run along one sector side.
#[derive(Clone, Copy, Debug, PartialEq)]
#[non_exhaustive]
#[cfg_attr(feature = "serde", derive(serde::Serialize))]
pub struct Portal {
    /// Detour side bits of the sector side: 0 = +x, 2 = +z, 4 = -x, 6 = -z.
    pub side: u8,
    /// 0 for a run's single point; 1 and 2 for the two points of a wide run.
    pub split: u8,
    /// Region index within the sector.
    pub region: u16,
    /// The transition point, on the sector side.
    pub pos: [f32; 3],
    /// Run extent along the side (z for sides 0 and 4, x for sides 2 and 6).
    pub extent: [f32; 2],
    /// Vertical range of the run's edges.
    pub y_range: [f32; 2],
    /// Index into [`SectorSummary::tiles`] of the tile holding the point.
    pub tile_index: u16,
}

/// An off-mesh connection that starts in the sector and ends in another.
#[derive(Clone, Copy, Debug, PartialEq)]
#[non_exhaustive]
#[cfg_attr(feature = "serde", derive(serde::Serialize))]
pub struct OffMeshEntry {
    /// Region index of the connection polygon.
    pub region: u16,
    /// Whether the connection can also be travelled from `far` to `start`.
    pub bidirectional: bool,
    /// Start point, inside the sector.
    pub start: [f32; 3],
    /// End point, outside the sector.
    pub far: [f32; 3],
    /// Index into [`SectorSummary::tiles`] of the tile holding the connection.
    pub tile_index: u16,
}

/// A level-2 sector summary for one filter profile (record `'DNSS'`).
///
/// Transitions are the portals followed by the off-mesh entries; the cost table holds
/// one `u16` per ordered transition pair: the exact in-sector shortest-path cost under
/// the profile's filter, floor-quantized by [`quantum`](Self::quantum) so that
/// dequantized costs never exceed the true cost. [`UNREACHABLE`] marks pairs with no
/// path inside the sector.
///
/// Record layout (little-endian): magic `'DNSS'`, version `u16`, flags `u16` (bit zero denotes stitched exits),
/// sector x `i32`, z `i32`, tiles per sector `u16`, layers `u8`, padding `u8`, profile
/// `u32`, tile, portal, region and off-mesh counts (`u16` each), quantum `f32`, climb
/// `f32`, polygon count `u32`; then tiles (x `i32`, z `i32`, layer `u8`, 3 padding
/// bytes, content id `u32`), regions (polygon count `u32`, bbox 6 x `f32`), portals
/// (side `u8`, split `u8`, region `u16`, pos 3 x `f32`, extent 2 x `f32`, y range 2 x
/// `f32`, tile index `u16`, padding `u16`), off-mesh entries (region `u16`, flags `u8`
/// with bit 0 = bidirectional, padding `u8`, start 3 x `f32`, far 3 x `f32`, tile index
/// `u16`, padding `u16`). Three counted arrays follow, each prefixed by a `u32`:
/// directed region pairs (two `u16`), polygon membership run starts (tile `u16`,
/// polygon `u32`, region `u16`, with 65535 marking filtered or past-end), and
/// stitched exits (from `u16`, sector x and z `i32`, to `u16`). The final
/// `N x N` cost table is ordered by source row, then destination column.
#[derive(Clone, Debug, PartialEq)]
pub struct SectorSummary {
    key: SectorKey,
    tiles_per_sector: u16,
    layers: u8,
    profile: u32,
    quantum: f32,
    climb: f32,
    polygon_count: u32,
    tiles: Vec<SectorTile>,
    regions: Vec<Region>,
    portals: Vec<Portal>,
    offmesh: Vec<OffMeshEntry>,
    costs: Vec<u16>,
    pub(super) arcs: Vec<(u16, u16)>,
    membership: Vec<(u16, u32, u16)>,
    pub(super) exits: Option<Vec<(u16, SectorKey, u16)>>,
}

fn charge(budget: &mut Budget, n: u64) -> Result<(), Error> {
    if budget.spend(n) {
        Ok(())
    } else {
        Err(Error::LimitExceeded)
    }
}

fn vert(tile: &MeshTile, i: u16) -> Option<[f32; 3]> {
    let at = usize::from(i) * 3;
    let v = tile.verts.get(at..at + 3)?;
    Some([v[0], v[1], v[2]])
}

fn lerp(a: [f32; 3], b: [f32; 3], t: f32) -> [f32; 3] {
    [
        a[0] + (b[0] - a[0]) * t,
        a[1] + (b[1] - a[1]) * t,
        a[2] + (b[2] - a[2]) * t,
    ]
}

/// Midpoint of the portal between two linked polygons (Detour's `getPortalPoints`).
pub(super) fn entry_mid(
    from: (&MeshTile, &Poly),
    from_ref: PolyRef,
    link: &Link,
    to: (&MeshTile, &Poly),
) -> Option<[f32; 3]> {
    let (ft, fp) = from;
    let (tt, tp) = to;
    let e = usize::from(link.edge);
    if fp.poly_type() == POLYTYPE_OFFMESH_CONNECTION {
        return vert(ft, *fp.verts.get(e)?);
    }
    if tp.poly_type() == POLYTYPE_OFFMESH_CONNECTION {
        let mut l = tp.first_link;
        while l != NULL_LINK {
            let back = tt.links.get(l as usize)?;
            if back.r#ref == from_ref {
                return vert(tt, *tp.verts.get(usize::from(back.edge))?);
            }
            l = back.next;
        }
        return None;
    }
    let n = usize::from(fp.vert_count);
    if e >= n {
        return None;
    }
    let mut left = vert(ft, fp.verts[e])?;
    let mut right = vert(ft, fp.verts[(e + 1) % n])?;
    if link.side != 0xff && (link.bmin != 0 || link.bmax != 255) {
        let s = 1.0 / 255.0;
        let (v0, v1) = (left, right);
        left = lerp(v0, v1, f32::from(link.bmin) * s);
        right = lerp(v0, v1, f32::from(link.bmax) * s);
    }
    Some([
        (left[0] + right[0]) * 0.5,
        (left[1] + right[1]) * 0.5,
        (left[2] + right[2]) * 0.5,
    ])
}

fn find(parent: &mut [u32], mut i: u32) -> u32 {
    while parent[i as usize] != i {
        let p = parent[i as usize];
        parent[i as usize] = parent[p as usize];
        i = p;
    }
    i
}

/// One passable polygon of the sector, in canonical (tile, polygon index) order.
struct Node {
    tile: usize,
    poly: usize,
    r: PolyRef,
}

/// A border edge facing out of the sector.
struct Edge {
    side: u8,
    region: u32,
    lo: f32,
    hi: f32,
    y: [f32; 2],
    line: f32,
    a: [f32; 3],
    b: [f32; 3],
    node: u32,
    tile: usize,
}

/// A transition seed: the polygon it starts or ends in and its point.
struct Seed {
    node: u32,
    pos: [f32; 3],
}

impl SectorSummary {
    /// Summarizes sector `key` of `nav` for one filter profile; `profile` is stored
    /// with the record (see [`profile_hash`](super::profile_hash)).
    ///
    /// Reads only tiles inside the sector and links between them; neighbours being
    /// resident or not changes nothing. The result is a pure function of the sector's
    /// tile bytes, the filter and `params`. One budget unit per polygon, link, edge,
    /// heap pop and relaxation; exhaustion returns no summary (call again with a larger
    /// budget). Errors: [`Error::InvalidParameter`] for bad `params`,
    /// [`Error::FormatLimit`] for more than 65,535 tiles, regions or portals or more
    /// than 4,096 transitions.
    pub fn build<F: QueryFilter + ?Sized>(
        nav: &NavMesh,
        key: SectorKey,
        params: &SectorParams,
        filter: &F,
        profile: u32,
        budget: &mut Budget,
    ) -> Result<Budgeted<Option<Self>>, Error> {
        params.validate()?;
        match Self::build_inner(nav, key, params, filter, profile, budget) {
            Ok(s) => Ok(Budgeted::Complete(Some(s))),
            Err(Error::LimitExceeded) if budget.is_exhausted() => Ok(Budgeted::Exhausted(None)),
            Err(e) => Err(e),
        }
    }

    fn build_inner<F: QueryFilter + ?Sized>(
        nav: &NavMesh,
        key: SectorKey,
        params: &SectorParams,
        filter: &F,
        profile: u32,
        budget: &mut Budget,
    ) -> Result<Self, Error> {
        let k = params.tiles_per_sector as i32;
        let x0 = key
            .x
            .checked_mul(k)
            .ok_or(Error::InvalidParameter("sector key"))?;
        let z0 = key
            .z
            .checked_mul(k)
            .ok_or(Error::InvalidParameter("sector key"))?;
        x0.checked_add(k)
            .zip(z0.checked_add(k))
            .ok_or(Error::InvalidParameter("sector key"))?;
        // Tiles in canonical (z, x, layer) order.
        let mut slots: Vec<(i32, i32, i32, usize)> = Vec::new();
        for z in z0..z0 + k {
            for x in x0..x0 + k {
                charge(budget, 1)?;
                for slot in nav.tiles_at(x, z, 256) {
                    if let Some(t) = nav.tile(slot) {
                        slots.push((z, x, t.header.layer, slot));
                    }
                }
            }
        }
        slots.sort_unstable();
        if slots.len() > usize::from(u16::MAX) {
            return Err(Error::FormatLimit("sector tiles"));
        }
        let mut tiles = Vec::with_capacity(slots.len());
        let mut tile_refs: Vec<&MeshTile> = Vec::with_capacity(slots.len());
        let mut slot_index = BTreeMap::new();
        let (mut climb, mut radius, mut layers) = (0.0f32, 0.0f32, 0u8);
        for (i, &(z, x, layer, slot)) in slots.iter().enumerate() {
            let t = nav.tile(slot).ok_or(Error::InvalidRef)?;
            let layer = u8::try_from(layer).map_err(|_| Error::FormatLimit("tile layer"))?;
            charge(budget, (t.data.len() as u64 / 64).max(1))?;
            let hash = content_hash(&t.data);
            tiles.push(SectorTile {
                x,
                z,
                layer,
                content_id: u32::from_le_bytes([hash[0], hash[1], hash[2], hash[3]]),
            });
            tile_refs.push(t);
            slot_index.insert(slot, i);
            climb = climb.max(t.header.walkable_climb);
            radius = radius.max(t.header.walkable_radius);
            layers = layers.max(layer.saturating_add(1));
        }
        // Passable polygons.
        let mut nodes: Vec<Node> = Vec::new();
        let mut node_of: Vec<Vec<u32>> = Vec::with_capacity(tiles.len());
        for (ti, &(_, _, _, slot)) in slots.iter().enumerate() {
            let t = tile_refs[ti];
            let base = nav.poly_ref_base(slot).ok_or(Error::InvalidRef)?;
            let mut map = vec![NONE; t.polys.len()];
            for (pi, poly) in t.polys.iter().enumerate() {
                charge(budget, 1)?;
                let r = base | pi as PolyRef;
                if filter.pass_filter(r, t, poly) {
                    map[pi] = u32::try_from(nodes.len()).map_err(|_| Error::SizeOverflow)?;
                    nodes.push(Node {
                        tile: ti,
                        poly: pi,
                        r,
                    });
                }
            }
            node_of.push(map);
        }
        let lookup = |r: PolyRef| -> Option<u32> {
            let (_, it, ip) = nav.decode_poly_id(r);
            let ti = *slot_index.get(&(it as usize))?;
            let n = *node_of.get(ti)?.get(ip as usize)?;
            (n != NONE && nodes.get(n as usize)?.r == r).then_some(n)
        };
        // Adjacency inside the sector, with the entry point into the neighbour.
        let mut adj: Vec<Vec<(u32, [f32; 3])>> = Vec::with_capacity(nodes.len());
        let mut parent: Vec<u32> = (0..nodes.len() as u32).collect();
        for node in &nodes {
            let t = tile_refs[node.tile];
            let p = &t.polys[node.poly];
            let mut out = Vec::new();
            let mut l = p.first_link;
            while l != NULL_LINK {
                charge(budget, 1)?;
                let link = t.links.get(l as usize).ok_or(Error::InvalidRef)?;
                if let Some(to) = lookup(link.r#ref) {
                    let tn = &nodes[to as usize];
                    let tt = tile_refs[tn.tile];
                    if let Some(mid) = entry_mid((t, p), node.r, link, (tt, &tt.polys[tn.poly])) {
                        out.push((to, mid));
                    }
                }
                l = link.next;
            }
            out.sort_unstable_by(|a, b| {
                a.0.cmp(&b.0)
                    .then(a.1[0].total_cmp(&b.1[0]))
                    .then(a.1[1].total_cmp(&b.1[1]))
                    .then(a.1[2].total_cmp(&b.1[2]))
            });
            adj.push(out);
        }
        // Contract only reciprocal edges. Remaining arcs retain their direction.
        for (i, out) in adj.iter().enumerate() {
            for &(to, _) in out {
                charge(budget, 1)?;
                if to as usize > i
                    && adj[to as usize]
                        .binary_search_by_key(&(i as u32), |edge| edge.0)
                        .is_ok()
                {
                    let a = find(&mut parent, i as u32);
                    let b = find(&mut parent, to);
                    parent[a.max(b) as usize] = a.min(b);
                }
            }
        }
        // Regions numbered by their first polygon in canonical order.
        let mut region_of = vec![NONE; nodes.len()];
        let mut root_region = BTreeMap::new();
        let mut regions: Vec<Region> = Vec::new();
        for (ni, node) in nodes.iter().enumerate() {
            charge(budget, 1)?;
            let root = find(&mut parent, ni as u32);
            let next = regions.len();
            let r = *root_region.entry(root).or_insert(next);
            if r == next {
                if next >= usize::from(u16::MAX) {
                    return Err(Error::FormatLimit("sector regions"));
                }
                regions.push(Region {
                    polygon_count: 0,
                    bbox: [f32::MAX, f32::MAX, f32::MAX, f32::MIN, f32::MIN, f32::MIN],
                });
            }
            region_of[ni] = r as u32;
            let t = tile_refs[node.tile];
            let p = &t.polys[node.poly];
            let reg = &mut regions[r];
            reg.polygon_count += 1;
            for &vi in &p.verts[..usize::from(p.vert_count).min(p.verts.len())] {
                if let Some(v) = vert(t, vi) {
                    for (a, &c) in v.iter().enumerate() {
                        reg.bbox[a] = reg.bbox[a].min(c);
                        reg.bbox[a + 3] = reg.bbox[a + 3].max(c);
                    }
                }
            }
        }
        let mut arcs = Vec::new();
        for (i, out) in adj.iter().enumerate() {
            for &(j, _) in out {
                let edge = (region_of[i] as u16, region_of[j as usize] as u16);
                if edge.0 != edge.1 {
                    arcs.push(edge);
                }
            }
        }
        arcs.sort_unstable();
        arcs.dedup();
        // Run starts include a sentinel for filtered polygons; lookup never uses boxes.
        let mut membership = Vec::new();
        for (ti, map) in node_of.iter().enumerate() {
            let mut last = u16::MAX;
            for (pi, &node) in map.iter().enumerate() {
                let region = if node == NONE {
                    u16::MAX
                } else {
                    region_of[node as usize] as u16
                };
                if region != last {
                    membership.push((ti as u16, pi as u32, region));
                    last = region;
                }
            }
            if last != u16::MAX {
                membership.push((ti as u16, map.len() as u32, u16::MAX));
            }
        }
        for reg in &mut regions {
            if reg.bbox[0] > reg.bbox[3] {
                reg.bbox = [0.0; 6];
            }
        }
        // Border edges facing out of the sector, grouped into maximal runs.
        let mut edges = Vec::new();
        for (ni, node) in nodes.iter().enumerate() {
            let t = tile_refs[node.tile];
            let p = &t.polys[node.poly];
            if p.poly_type() == POLYTYPE_OFFMESH_CONNECTION {
                continue;
            }
            let (lx, lz) = (t.header.x - x0, t.header.y - z0);
            let n = usize::from(p.vert_count).min(p.verts.len());
            for j in 0..n {
                charge(budget, 1)?;
                let nei = p.neis[j];
                if nei & EXT_LINK == 0 {
                    continue;
                }
                let side = (nei & 0xff) as u8;
                let out = match side {
                    0 => lx == k - 1,
                    4 => lx == 0,
                    2 => lz == k - 1,
                    6 => lz == 0,
                    _ => false,
                };
                if !out {
                    continue;
                }
                let (Some(a), Some(b)) = (vert(t, p.verts[j]), vert(t, p.verts[(j + 1) % n]))
                else {
                    continue;
                };
                let (axis, line) = if side == 0 || side == 4 {
                    (2, 0)
                } else {
                    (0, 2)
                };
                edges.push(Edge {
                    side,
                    region: region_of[ni],
                    lo: a[axis].min(b[axis]),
                    hi: a[axis].max(b[axis]),
                    y: [a[1].min(b[1]), a[1].max(b[1])],
                    line: a[line],
                    a,
                    b,
                    node: ni as u32,
                    tile: node.tile,
                });
            }
        }
        edges.sort_unstable_by(|e, f| {
            (e.side, e.region)
                .cmp(&(f.side, f.region))
                .then(e.lo.total_cmp(&f.lo))
                .then(e.hi.total_cmp(&f.hi))
                .then(e.node.cmp(&f.node))
                .then(e.line.total_cmp(&f.line))
        });
        let eps = nav.params().tile_width.abs() * 1e-4;
        let split_width = if params.portal_split_width > 0.0 {
            params.portal_split_width
        } else if radius > 0.0 {
            32.0 * radius
        } else {
            16.0
        };
        let mut portals = Vec::new();
        let mut seeds = Vec::new();
        let mut start = 0;
        while start < edges.len() {
            let (side, region) = (edges[start].side, edges[start].region);
            let (mut hi, mut y) = (edges[start].hi, edges[start].y);
            let mut end = start + 1;
            while end < edges.len()
                && edges[end].side == side
                && edges[end].region == region
                && edges[end].lo <= hi + eps
            {
                hi = hi.max(edges[end].hi);
                y = [y[0].min(edges[end].y[0]), y[1].max(edges[end].y[1])];
                end += 1;
            }
            let run = &edges[start..end];
            let lo = run[0].lo;
            let w = hi - lo;
            let points: &[(u8, f32)] = if w > split_width {
                &[(1, lo + w * 0.25), (2, lo + w * 0.75)]
            } else {
                &[(0, lo + w * 0.5)]
            };
            for &(split, t) in points {
                charge(budget, run.len() as u64)?;
                // The member edge containing t, else the nearest.
                let mut best = &run[0];
                let mut best_d = f32::INFINITY;
                for e in run {
                    let d = (e.lo - t).max(t - e.hi).max(0.0);
                    if d < best_d {
                        best_d = d;
                        best = e;
                    }
                }
                let axis = if side == 0 || side == 4 { 2 } else { 0 };
                let span = best.b[axis] - best.a[axis];
                let s = if span != 0.0 {
                    ((t - best.a[axis]) / span).clamp(0.0, 1.0)
                } else {
                    0.5
                };
                let py = best.a[1] + (best.b[1] - best.a[1]) * s;
                let pos = if axis == 2 {
                    [best.line, py, t]
                } else {
                    [t, py, best.line]
                };
                if portals.len() >= usize::from(u16::MAX) {
                    return Err(Error::FormatLimit("sector portals"));
                }
                portals.push(Portal {
                    side,
                    split,
                    region: region as u16,
                    pos,
                    extent: [lo, hi],
                    y_range: y,
                    tile_index: best.tile as u16,
                });
                seeds.push(Seed {
                    node: best.node,
                    pos,
                });
            }
            start = end;
        }
        // Off-mesh connections that leave the sector.
        let mut offmesh = Vec::new();
        let mut off_seeds = Vec::new();
        for (ni, node) in nodes.iter().enumerate() {
            let t = tile_refs[node.tile];
            if t.polys[node.poly].poly_type() != POLYTYPE_OFFMESH_CONNECTION {
                continue;
            }
            charge(budget, t.off_mesh_cons.len() as u64)?;
            let Some(con) = t
                .off_mesh_cons
                .iter()
                .find(|c| usize::from(c.poly) == node.poly)
            else {
                continue;
            };
            let start = [con.pos[0], con.pos[1], con.pos[2]];
            let far = [con.pos[3], con.pos[4], con.pos[5]];
            let (fx, fz) = nav.calc_tile_loc(&far);
            if params.sector_of_tile(fx, fz) == key {
                continue;
            }
            if offmesh.len() >= usize::from(u16::MAX) {
                return Err(Error::FormatLimit("sector off-mesh entries"));
            }
            offmesh.push(OffMeshEntry {
                region: region_of[ni] as u16,
                bidirectional: con.flags & OFFMESH_CON_BIDIR != 0,
                start,
                far,
                tile_index: node.tile as u16,
            });
            off_seeds.push(Seed {
                node: ni as u32,
                pos: start,
            });
        }
        seeds.extend(off_seeds);
        let n = seeds.len();
        if n > MAX_TRANSITIONS {
            return Err(Error::FormatLimit("sector transitions"));
        }
        // Exact transition-to-transition costs: one Dijkstra per source.
        let mut raw = vec![f32::INFINITY; n * n];
        let mut dist = vec![f32::INFINITY; nodes.len()];
        let mut from = vec![NONE; nodes.len()];
        let mut at = vec![[0.0f32; 3]; nodes.len()];
        let mut closed = vec![false; nodes.len()];
        let mut touched = Vec::new();
        let poly = |i: u32| {
            let nd = &nodes[i as usize];
            let t = tile_refs[nd.tile];
            (nd.r, t, &t.polys[nd.poly])
        };
        for (si, s) in seeds.iter().enumerate() {
            for &i in &touched {
                let i: usize = i;
                dist[i] = f32::INFINITY;
                from[i] = NONE;
                closed[i] = false;
            }
            touched.clear();
            let src = s.node as usize;
            dist[src] = 0.0;
            at[src] = s.pos;
            touched.push(src);
            let mut heap = BinaryHeap::new();
            heap.push(Reverse((0u32, s.node)));
            while let Some(Reverse((bits, cur))) = heap.pop() {
                charge(budget, 1)?;
                let c = cur as usize;
                if closed[c] || dist[c].to_bits() != bits {
                    continue;
                }
                closed[c] = true;
                let prev = (from[c] != NONE).then(|| poly(from[c]));
                for &(to, mid) in &adj[c] {
                    charge(budget, 1)?;
                    let t = to as usize;
                    if closed[t] {
                        continue;
                    }
                    let step = filter.cost(&at[c], &mid, prev, poly(cur), Some(poly(to)));
                    let total = dist[c] + step;
                    if step.is_nan() || step < 0.0 || !total.is_finite() {
                        continue;
                    }
                    let better = total < dist[t] || (total == dist[t] && cur < from[t]);
                    if better {
                        if dist[t] == f32::INFINITY {
                            touched.push(t);
                        }
                        dist[t] = total;
                        from[t] = cur;
                        at[t] = mid;
                        heap.push(Reverse((total.to_bits(), to)));
                    }
                }
            }
            for (ti, target) in seeds.iter().enumerate() {
                let g = target.node as usize;
                let v = if ti == si {
                    0.0
                } else if dist[g].is_finite() {
                    let prev = (from[g] != NONE).then(|| poly(from[g]));
                    let last = filter.cost(&at[g], &target.pos, prev, poly(target.node), None);
                    if last >= 0.0 {
                        dist[g] + last
                    } else {
                        f32::INFINITY
                    }
                } else {
                    f32::INFINITY
                };
                raw[si * n + ti] = v;
            }
        }
        // Floor quantization against a per-sector quantum: never saturates, always a
        // lower bound.
        let max_cost = raw
            .iter()
            .copied()
            .filter(|c| c.is_finite())
            .fold(0.0f64, |m, c| m.max(f64::from(c)));
        let side_len = f64::from(k as f32 * nav.params().tile_width.abs());
        let diagonal = side_len * core::f64::consts::SQRT_2;
        let want = (diagonal / 65535.0)
            .max(max_cost / 65534.0)
            .max(f64::MIN_POSITIVE);
        let mut quantum = want as f32;
        if f64::from(quantum) < want {
            quantum = quantum.next_up();
        }
        if !(quantum.is_finite() && quantum > 0.0) {
            return Err(Error::InvalidParameter("tile width"));
        }
        let costs = raw
            .iter()
            .map(|&c| {
                if c.is_finite() {
                    // Costs are non-negative, so truncation is the floor.
                    ((f64::from(c) / f64::from(quantum)) as u64).min(65534) as u16
                } else {
                    UNREACHABLE
                }
            })
            .collect();
        Ok(SectorSummary {
            key,
            tiles_per_sector: params.tiles_per_sector as u16,
            layers,
            profile,
            quantum,
            climb,
            polygon_count: nodes.len() as u32,
            tiles,
            regions,
            portals,
            offmesh,
            costs,
            arcs,
            membership,
            exits: None,
        })
    }

    /// Looks up a passable polygon by stable tile coordinates, layer and polygon index.
    pub fn polygon_region(&self, x: i32, z: i32, layer: u8, poly: u32) -> Option<u16> {
        let tile = self
            .tiles
            .binary_search_by_key(&(z, x, layer), |t| (t.z, t.x, t.layer))
            .ok()? as u16;
        let end = self
            .membership
            .partition_point(|&(t, p, _)| (t, p) <= (tile, poly));
        let &(t, _, r) = self.membership.get(end.checked_sub(1)?)?;
        (t == tile && r != u16::MAX).then_some(r)
    }
    /// Directed arcs between regions inside the sector.
    pub fn region_arcs(&self) -> &[(u16, u16)] {
        &self.arcs
    }
    /// The sector.
    pub fn key(&self) -> SectorKey {
        self.key
    }
    /// Tiles per sector side the summary was built with.
    pub fn tiles_per_sector(&self) -> u16 {
        self.tiles_per_sector
    }
    /// Filter profile id.
    pub fn profile(&self) -> u32 {
        self.profile
    }
    /// Metres-times-area-cost per quantized cost unit.
    pub fn quantum(&self) -> f32 {
        self.quantum
    }
    /// Largest walkable climb of the sector's tiles (portal matching tolerance).
    pub fn climb(&self) -> f32 {
        self.climb
    }
    /// Passable polygons in the sector.
    pub fn polygon_count(&self) -> u32 {
        self.polygon_count
    }
    /// Source tiles and their content ids, ordered by (z, x, layer).
    pub fn tiles(&self) -> &[SectorTile] {
        &self.tiles
    }
    /// Regions, numbered by their first polygon in (tile, polygon index) order.
    pub fn regions(&self) -> &[Region] {
        &self.regions
    }
    /// Portals, ordered by (side, region, run start, split).
    pub fn portals(&self) -> &[Portal] {
        &self.portals
    }
    /// Off-mesh connections leaving the sector.
    pub fn offmesh(&self) -> &[OffMeshEntry] {
        &self.offmesh
    }
    /// Portals plus off-mesh entries: the side of the cost table.
    pub fn transition_count(&self) -> usize {
        self.portals.len() + self.offmesh.len()
    }
    /// Quantized cost from transition `from` to transition `to` ([`UNREACHABLE`] when
    /// there is no in-sector path); `None` for out-of-range indices.
    pub fn quantized_cost(&self, from: usize, to: usize) -> Option<u16> {
        let n = self.transition_count();
        if from >= n || to >= n {
            return None;
        }
        self.costs.get(from * n + to).copied()
    }
    /// Lower bound of the cost from transition `from` to `to`, or `None` when out of
    /// range or unreachable.
    pub fn cost(&self, from: usize, to: usize) -> Option<f32> {
        let q = self.quantized_cost(from, to)?;
        (q != UNREACHABLE).then(|| (f64::from(q) * f64::from(self.quantum)) as f32)
    }

    /// Size of the encoded record.
    pub fn encoded_len(&self) -> usize {
        let n = self.transition_count();
        HEADER
            + TILE * self.tiles.len()
            + REGION * self.regions.len()
            + PORTAL * self.portals.len()
            + OFFMESH * self.offmesh.len()
            + 8
            + self.arcs.len() * 4
            + self.membership.len() * 8
            + 4
            + self.exits.as_ref().map_or(0, |e| 12 * e.len())
            + 2 * n * n
    }

    /// Encodes the `'DNSS'` record.
    pub fn to_bytes(&self) -> Vec<u8> {
        let mut w = Writer::with_capacity(self.encoded_len());
        w.out.extend_from_slice(&SECTOR_MAGIC);
        w.u16(SECTOR_VERSION);
        w.u16(u16::from(self.exits.is_some()));
        w.i32(self.key.x);
        w.i32(self.key.z);
        w.u16(self.tiles_per_sector);
        w.u8(self.layers);
        w.u8(0);
        w.u32(self.profile);
        w.u16(self.tiles.len() as u16);
        w.u16(self.portals.len() as u16);
        w.u16(self.regions.len() as u16);
        w.u16(self.offmesh.len() as u16);
        w.f32(self.quantum);
        w.f32(self.climb);
        w.u32(self.polygon_count);
        for t in &self.tiles {
            w.i32(t.x);
            w.i32(t.z);
            w.u8(t.layer);
            w.u8(0);
            w.u16(0);
            w.u32(t.content_id);
        }
        for r in &self.regions {
            w.u32(r.polygon_count);
            w.f32s(&r.bbox);
        }
        for p in &self.portals {
            w.u8(p.side);
            w.u8(p.split);
            w.u16(p.region);
            w.f32s(&p.pos);
            w.f32s(&p.extent);
            w.f32s(&p.y_range);
            w.u16(p.tile_index);
            w.u16(0);
        }
        for o in &self.offmesh {
            w.u16(o.region);
            w.u8(u8::from(o.bidirectional));
            w.u8(0);
            w.f32s(&o.start);
            w.f32s(&o.far);
            w.u16(o.tile_index);
            w.u16(0);
        }
        w.u32(self.arcs.len() as u32);
        for &(a, b) in &self.arcs {
            w.u16(a);
            w.u16(b);
        }
        w.u32(self.membership.len() as u32);
        for &(t, p, r) in &self.membership {
            w.u16(t);
            w.u32(p);
            w.u16(r);
        }
        let exits = self.exits.as_deref().unwrap_or_default();
        w.u32(exits.len() as u32);
        for &(a, k, b) in exits {
            w.u16(a);
            w.i32(k.x);
            w.i32(k.z);
            w.u16(b);
        }
        for &c in &self.costs {
            w.u16(c);
        }
        w.out
    }

    /// Decodes and validates a `'DNSS'` record. Accepted bytes re-encode to the same
    /// bytes. Errors: [`Error::InvalidInput`] for a wrong magic, version, flags,
    /// padding, length, non-finite or unordered float, out-of-range index or side, a
    /// tile outside the sector, unsorted tiles, or a nonzero cost diagonal;
    /// [`Error::FormatLimit`] for more than 4,096 transitions;
    /// [`Error::LimitExceeded`] above `limits.max_bytes`.
    pub fn from_bytes(data: &[u8], limits: &Limits) -> Result<Self, Error> {
        check_bytes(data.len(), limits)?;
        let mut r = Reader::new(data, "sector summary");
        let magic = [r.u8()?, r.u8()?, r.u8()?, r.u8()?];
        r.require(magic == SECTOR_MAGIC)?;
        let (version, flags) = (r.u16()?, r.u16()?);
        r.require(version == SECTOR_VERSION && flags <= 1)?;
        let key = SectorKey {
            x: r.i32()?,
            z: r.i32()?,
        };
        let tiles_per_sector = r.u16()?;
        let layers = r.u8()?;
        r.pad(1)?;
        let profile = r.u32()?;
        let (nt, np, nr, nm) = (
            usize::from(r.u16()?),
            usize::from(r.u16()?),
            usize::from(r.u16()?),
            usize::from(r.u16()?),
        );
        let quantum = r.f32()?;
        let climb = r.f32()?;
        let polygon_count = r.u32()?;
        r.require(tiles_per_sector > 0 && quantum > 0.0 && climb >= 0.0)?;
        let n = np + nm;
        if n > MAX_TRANSITIONS {
            return Err(Error::FormatLimit("sector transitions"));
        }
        let expected = HEADER + TILE * nt + REGION * nr + PORTAL * np + OFFMESH * nm + 2 * n * n;
        r.require(data.len() >= expected + 12)?;
        let k = i64::from(tiles_per_sector);
        let (x0, z0) = (i64::from(key.x) * k, i64::from(key.z) * k);
        let mut tiles: Vec<SectorTile> = Vec::with_capacity(nt);
        for _ in 0..nt {
            let t = SectorTile {
                x: r.i32()?,
                z: r.i32()?,
                layer: r.u8()?,
                content_id: {
                    r.pad(3)?;
                    r.u32()?
                },
            };
            let (tx, tz) = (i64::from(t.x), i64::from(t.z));
            r.require((x0..x0 + k).contains(&tx) && (z0..z0 + k).contains(&tz))?;
            r.require(t.layer < layers)?;
            if let Some(p) = tiles.last() {
                r.require((p.z, p.x, p.layer) < (t.z, t.x, t.layer))?;
            }
            tiles.push(t);
        }
        let mut regions = Vec::with_capacity(nr);
        let mut polys = 0u64;
        for _ in 0..nr {
            let reg = Region {
                polygon_count: r.u32()?,
                bbox: r.f32s()?,
            };
            r.require(reg.polygon_count > 0)?;
            r.require((0..3).all(|a| ordered(reg.bbox[a], reg.bbox[a + 3])))?;
            polys += u64::from(reg.polygon_count);
            regions.push(reg);
        }
        r.require(polys == u64::from(polygon_count))?;
        let mut portals = Vec::with_capacity(np);
        for _ in 0..np {
            let p = Portal {
                side: r.u8()?,
                split: r.u8()?,
                region: r.u16()?,
                pos: r.f32s()?,
                extent: r.f32s()?,
                y_range: r.f32s()?,
                tile_index: r.u16()?,
            };
            r.pad(2)?;
            r.require(matches!(p.side, 0 | 2 | 4 | 6) && p.split <= 2)?;
            r.require(usize::from(p.region) < nr && usize::from(p.tile_index) < nt)?;
            r.require(ordered(p.extent[0], p.extent[1]) && ordered(p.y_range[0], p.y_range[1]))?;
            portals.push(p);
        }
        let mut offmesh = Vec::with_capacity(nm);
        for _ in 0..nm {
            let region = r.u16()?;
            let flags = r.u8()?;
            r.pad(1)?;
            let o = OffMeshEntry {
                region,
                bidirectional: flags == 1,
                start: r.f32s()?,
                far: r.f32s()?,
                tile_index: r.u16()?,
            };
            r.pad(2)?;
            r.require(flags <= 1)?;
            r.require(usize::from(o.region) < nr && usize::from(o.tile_index) < nt)?;
            offmesh.push(o);
        }
        let na = r.u32()? as usize;
        r.require(na <= (data.len() - expected - 12) / 4)?;
        let mut arcs = Vec::with_capacity(na);
        for _ in 0..na {
            let e = (r.u16()?, r.u16()?);
            r.require(
                e.0 != e.1 && usize::from(e.0.max(e.1)) < nr && arcs.last().is_none_or(|p| *p < e),
            )?;
            arcs.push(e);
        }
        let nmembers = r.u32()? as usize;
        r.require(expected as u64 + 12 + na as u64 * 4 + nmembers as u64 * 8 <= data.len() as u64)?;
        let mut membership = Vec::with_capacity(nmembers);
        for _ in 0..nmembers {
            let m = (r.u16()?, r.u32()?, r.u16()?);
            r.require(
                usize::from(m.0) < nt
                    && (m.2 == u16::MAX || usize::from(m.2) < nr)
                    && membership
                        .last()
                        .is_none_or(|&(t, p, _)| (t, p) < (m.0, m.1)),
            )?;
            membership.push(m);
        }
        let mut counts = vec![0u64; nr];
        for (i, &(tile, poly, region)) in membership.iter().enumerate() {
            if region == u16::MAX {
                r.require(i > 0 && membership[i - 1].0 == tile && membership[i - 1].2 != region)?;
                continue;
            }
            let &(next_tile, end, next_region) = membership
                .get(i + 1)
                .ok_or(Error::InvalidInput("unterminated polygon membership"))?;
            r.require(tile == next_tile && next_region != region)?;
            counts[usize::from(region)] += u64::from(end - poly);
        }
        r.require(
            regions
                .iter()
                .zip(counts)
                .all(|(region, n)| u64::from(region.polygon_count) == n),
        )?;
        let ne = r.u32()? as usize;
        r.require(
            (flags == 1 || ne == 0)
                && expected as u64 + 12 + na as u64 * 4 + nmembers as u64 * 8 + ne as u64 * 12
                    == data.len() as u64,
        )?;
        let mut exits = Vec::with_capacity(ne);
        for _ in 0..ne {
            let e = (
                r.u16()?,
                SectorKey {
                    x: r.i32()?,
                    z: r.i32()?,
                },
                r.u16()?,
            );
            r.require(
                usize::from(e.0) < nr
                    && !(e.1 == key && usize::from(e.2) >= nr)
                    && exits.last().is_none_or(|p| *p < e),
            )?;
            exits.push(e);
        }
        let exits = (flags == 1).then_some(exits);
        let mut costs = Vec::with_capacity(n * n);
        for i in 0..n * n {
            let c = r.u16()?;
            r.require(i % (n + 1) != 0 || c == 0)?;
            costs.push(c);
        }
        r.finish()?;
        Ok(SectorSummary {
            key,
            tiles_per_sector,
            layers,
            profile,
            quantum,
            climb,
            polygon_count,
            tiles,
            regions,
            portals,
            offmesh,
            costs,
            arcs,
            membership,
            exits,
        })
    }
}

/// Summarizes one sector from its tile bytes alone, as a bake does: the tiles are
/// added to a scratch navmesh laid out by `layout` (only its origin and tile size are
/// used) and [`SectorSummary::build`] runs with an unlimited budget. Tiles outside
/// the sector are rejected with [`Error::InvalidInput`]; tile bytes are validated
/// under `limits`.
pub fn summarize_tiles<F: QueryFilter + ?Sized>(
    layout: &NavMeshParams,
    key: SectorKey,
    tiles: &[Vec<u8>],
    params: &SectorParams,
    filter: &F,
    profile: u32,
    limits: Limits,
) -> Result<SectorSummary, Error> {
    params.validate()?;
    let mut max_polys = 1i32;
    let mut parsed = Vec::with_capacity(tiles.len());
    for bytes in tiles {
        let t = MeshTile::parse_with_limits(bytes.clone(), &limits)?;
        if params.sector_of_tile(t.header.x, t.header.y) != key {
            return Err(Error::InvalidInput("tile outside the sector"));
        }
        max_polys = max_polys.max(t.header.poly_count);
        parsed.push(t.data);
    }
    let scratch = NavMeshParams {
        orig: layout.orig,
        tile_width: layout.tile_width,
        tile_height: layout.tile_height,
        max_tiles: i32::try_from(parsed.len().max(1)).map_err(|_| Error::SizeOverflow)?,
        max_polys,
    };
    let mut nav = NavMesh::with_limits(&scratch, limits)?;
    for data in parsed {
        nav.add_tile(data)?;
    }
    match SectorSummary::build(&nav, key, params, filter, profile, &mut Budget::unlimited())? {
        Budgeted::Complete(Some(s)) => Ok(s),
        _ => Err(Error::LimitExceeded),
    }
}

#[cfg(test)]
mod direction_regression {
    use super::*;
    use crate::route::{SuperKey, SuperRecord};

    #[test]
    fn super_regions_preserve_one_way_reachability() {
        // Two disjoint floors connected only from the first to the second.
        let make = |x, offmesh| SectorSummary {
            key: SectorKey { x, z: 0 },
            tiles_per_sector: 1,
            layers: 1,
            profile: 0,
            quantum: 1.0,
            climb: 0.5,
            polygon_count: 1,
            tiles: vec![SectorTile {
                x,
                z: 0,
                layer: 0,
                content_id: 1,
            }],
            regions: vec![Region {
                polygon_count: 1,
                bbox: [x as f32, 0.0, 0.0, x as f32 + 0.8, 0.0, 0.8],
            }],
            portals: Vec::new(),
            offmesh,
            costs: vec![0],
            arcs: vec![],
            membership: vec![],
            exits: None,
        };
        let a = make(
            0,
            vec![OffMeshEntry {
                region: 0,
                bidirectional: false,
                start: [0.4, 0.0, 0.4],
                far: [1.4, 0.0, 0.4],
                tile_index: 0,
            }],
        );
        let b = make(1, Vec::new());
        let record =
            SuperRecord::derive(SuperKey { x: 0, z: 0 }, &SectorParams::new(1, 2), &[&a, &b])
                .unwrap();
        assert_eq!(record.regions().len(), 2);
        assert_eq!(record.region_arcs(), &[(0, 1)]);
    }
}
