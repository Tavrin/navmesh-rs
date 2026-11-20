//! Level-3 super-sector records (`'DNSR'`) and the resident [`SectorGraph`].

use super::stitched::Crossing;
use super::summary::SectorSummary;
use super::wire::{Reader, Writer, check_bytes, ordered};
use super::{SectorKey, SectorParams, SuperKey, opposite_side, runs_meet};
use crate::{Budget, Error, Limits, SnapshotError};
use alloc::collections::{BTreeMap, BTreeSet};
use alloc::vec;
use alloc::vec::Vec;

/// Magic bytes at the start of a super-sector record.
pub const SUPER_MAGIC: [u8; 4] = *b"DNSR";
/// Version of the super-sector record this crate writes and reads.
pub const SUPER_VERSION: u16 = 3;

const HEADER: usize = 44;
const SECTOR: usize = 16;
const REGION: usize = 36;
const MEMBER: usize = 4;
const PORTAL: usize = 36;
const OFFMESH: usize = 28;
const EXIT: usize = 16;

/// Level-3 edges of one record, per super-region.
type EdgeSet = Vec<Vec<SuperEdge>>;

fn charge(budget: &mut Budget, n: u64) -> Result<(), Error> {
    if budget.spend(n) {
        Ok(())
    } else {
        Err(Error::LimitExceeded)
    }
}
/// Overlap along a side below which two portal runs do not meet, metres.
const MEET_EPS: f32 = 1e-3;

/// A level-3 node: sector regions of one super-sector joined across shared portals
/// and off-mesh links that stay inside the super-sector.
#[derive(Clone, Debug, PartialEq)]
#[non_exhaustive]
#[cfg_attr(feature = "serde", derive(serde::Serialize))]
pub struct SuperRegion {
    /// Member `(sector index, sector region)` pairs, sorted.
    pub members: Vec<(u16, u16)>,
    /// Lower bound of the cost between any two distinct transitions of any member
    /// region (zero when no member has two transitions).
    pub crossing_bound: f32,
    /// Union of the member regions' boxes.
    pub bbox: [f32; 6],
}

/// A sector portal on the boundary of its super-sector.
#[derive(Clone, Copy, Debug, PartialEq)]
#[non_exhaustive]
#[cfg_attr(feature = "serde", derive(serde::Serialize))]
pub struct SuperPortal {
    /// Detour side bits of the super-sector side.
    pub side: u8,
    /// The sector portal's split index.
    pub split: u8,
    /// Super-region index.
    pub region: u16,
    /// Index into [`SuperRecord::sectors`].
    pub sector: u16,
    /// Transition point.
    pub pos: [f32; 3],
    /// Run extent along the side.
    pub extent: [f32; 2],
    /// Vertical range of the run.
    pub y_range: [f32; 2],
}

/// An off-mesh connection leaving the super-sector.
#[derive(Clone, Copy, Debug, PartialEq)]
#[non_exhaustive]
#[cfg_attr(feature = "serde", derive(serde::Serialize))]
pub struct SuperOffMesh {
    /// Super-region index of the start.
    pub region: u16,
    /// Whether it can be travelled back.
    pub bidirectional: bool,
    /// Start point.
    pub start: [f32; 3],
    /// End point, outside the super-sector.
    pub far: [f32; 3],
}

/// A level-3 record for one super-sector and profile (record `'DNSR'`).
///
/// Derived from the sector summaries of the super-sector by
/// [`derive`](Self::derive); loadable on its own, without any sector summary.
///
/// Record layout (little-endian): magic `'DNSR'`, version `u16`, flags `u16` (bit zero denotes stitched exits),
/// super x `i32`, z `i32`, sectors per super `u16`, tiles per sector `u16`, profile
/// `u32`, climb `f32`, sector and region counts (`u16` each), member, portal and
/// off-mesh counts (`u32` each); then sectors (x `i32`, z `i32`, version `u32`,
/// content id `u32`),
/// regions (first member `u32`, member count `u32`, crossing bound `f32`, bbox 6 x
/// `f32`), members (sector `u16`, region `u16`), portals (side `u8`, split `u8`,
/// region `u16`, sector `u16`, padding `u16`, pos 3 x `f32`, extent 2 x `f32`, y range
/// 2 x `f32`) and off-mesh entries (region `u16`, flags `u8`, padding `u8`, start 3 x
/// `f32`, far 3 x `f32`). The final arrays are directed local region pairs
/// (two `u16`) and remote exits (source region `u16`, destination sector x
/// and z `i32`, destination region `u16`, destination sector content id `u32`),
/// each prefixed by a `u32` count.
#[derive(Clone, Debug, PartialEq)]
pub struct SuperRecord {
    key: SuperKey,
    sectors_per_super: u16,
    tiles_per_sector: u16,
    profile: u32,
    climb: f32,
    sectors: Vec<(SectorKey, u32)>,
    contents: Vec<u32>,
    regions: Vec<SuperRegion>,
    portals: Vec<SuperPortal>,
    offmesh: Vec<SuperOffMesh>,
    arcs: Vec<(u16, u16)>,
    bound: bool,
    exits: Vec<(u16, SectorKey, u16, u32)>,
}

impl SectorSummary {
    /// Fingerprint of this profile's complete encoded summary, including directed
    /// exits, effective membership and transition costs. Computed outside searches.
    pub fn version(&self) -> u32 {
        self.to_bytes().into_iter().fold(0x811c_9dc5u32, |h, b| {
            (h ^ u32::from(b)).wrapping_mul(0x0100_0193)
        })
    }
}

fn find(parent: &mut [usize], mut i: usize) -> usize {
    while parent[i] != i {
        parent[i] = parent[parent[i]];
        i = parent[i];
    }
    i
}

fn join(parent: &mut [usize], a: usize, b: usize) {
    let (a, b) = (find(parent, a), find(parent, b));
    if a != b {
        parent[a.max(b)] = a.min(b);
    }
}

fn inside(bbox: &[f32; 6], p: &[f32; 3], climb: f32) -> bool {
    p[0] >= bbox[0] - MEET_EPS
        && p[0] <= bbox[3] + MEET_EPS
        && p[2] >= bbox[2] - MEET_EPS
        && p[2] <= bbox[5] + MEET_EPS
        && p[1] >= bbox[1] - climb
        && p[1] <= bbox[4] + climb
}

impl SuperRecord {
    /// Derives the record of super-sector `key` from the summaries of its sectors.
    ///
    /// Regions of adjacent sectors join where a portal on one side meets a portal on
    /// the facing side (positive overlap, heights within the larger climb); an
    /// off-mesh entry joins the region whose box holds its far end. Errors:
    /// [`Error::InvalidInput`] for a summary outside `key`, a duplicate sector, or
    /// mixed profiles or sector sizes, or a stitched crossing into a sector whose
    /// summary comes from another bake (its content id differs from the crossing's);
    /// [`Error::FormatLimit`] above 65,535 sectors or regions.
    pub fn derive(
        key: SuperKey,
        params: &SectorParams,
        sectors: &[&SectorSummary],
    ) -> Result<Self, Error> {
        match Self::derive_budgeted(key, params, sectors, &mut Budget::unlimited())? {
            (record, true) => Ok(record),
            (_, false) => Err(Error::InvalidInput("sector summaries from different bakes")),
        }
    }

    /// [`derive`](Self::derive) charging one unit per summary, region, portal pair,
    /// off-mesh landing test and cost-table cell it examines. Also returns whether
    /// every stitched crossing inside the super-sector names the content id of the
    /// summary it lands in.
    pub(super) fn derive_budgeted(
        key: SuperKey,
        params: &SectorParams,
        sectors: &[&SectorSummary],
        budget: &mut Budget,
    ) -> Result<(Self, bool), Error> {
        params.validate()?;
        charge(budget, sectors.len() as u64)?;
        let mut list: Vec<&SectorSummary> = sectors.to_vec();
        list.sort_unstable_by_key(|s| (s.key().z, s.key().x));
        let bound = list.first().is_some_and(|s| s.exits.is_some());
        let profile = list.first().map_or(0, |s| s.profile());
        for (i, s) in list.iter().enumerate() {
            if params.super_of(s.key()) != key
                || u32::from(s.tiles_per_sector()) != params.tiles_per_sector
                || s.profile() != profile
                || s.exits.is_some() != bound
                || (i > 0 && list[i - 1].key() == s.key())
            {
                return Err(Error::InvalidInput("sector summaries of a super-sector"));
            }
        }
        if list.len() > usize::from(u16::MAX) {
            return Err(Error::FormatLimit("super-sector sectors"));
        }
        let mut base = Vec::with_capacity(list.len());
        let mut total = 0usize;
        for s in &list {
            base.push(total);
            total += s.regions().len();
        }
        let at = |k: SectorKey| list.binary_search_by_key(&(k.z, k.x), |s| (s.key().z, s.key().x));
        let mut parent: Vec<usize> = (0..total).collect();
        let mut leaving = Vec::new();
        let mut transitions = Vec::new();
        let mut exits = Vec::new();
        let mut consistent = true;
        let contents: Vec<u32> = list.iter().map(|s| s.content_id()).collect();
        if bound {
            for (i, s) in list.iter().enumerate() {
                let out = s.exits.as_deref().unwrap_or_default();
                charge(budget, out.len() as u64)?;
                for &(a, k, b, c) in out {
                    let from = base[i] + usize::from(a);
                    if let Ok(j) = at(k) {
                        if usize::from(b) >= list[j].regions().len() {
                            return Err(Error::InvalidInput("unknown crossing region"));
                        }
                        consistent &= contents[j] == c;
                        transitions.push((from, base[j] + usize::from(b)));
                    } else {
                        exits.push((from, k, b, c));
                    }
                }
            }
            transitions.sort_unstable();
            transitions.dedup();
            for &(a, b) in &transitions {
                if a < b && transitions.binary_search(&(b, a)).is_ok() {
                    join(&mut parent, a, b);
                }
            }
        } else {
            for (i, a) in list.iter().enumerate() {
                let k = a.key();
                for (side, nk) in [
                    (0u8, k.x.checked_add(1).map(|x| SectorKey { x, z: k.z })),
                    (2u8, k.z.checked_add(1).map(|z| SectorKey { x: k.x, z })),
                ] {
                    let Some(Ok(j)) = nk.map(at) else { continue };
                    let b = list[j];
                    let climb = a.climb().max(b.climb());
                    for pa in a.portals().iter().filter(|p| p.side == side) {
                        charge(budget, b.portals().len() as u64)?;
                        for pb in b.portals().iter().filter(|p| p.side == opposite_side(side)) {
                            if runs_meet(
                                pa.extent, pa.y_range, pb.extent, pb.y_range, climb, MEET_EPS,
                            ) {
                                join(
                                    &mut parent,
                                    base[i] + usize::from(pa.region),
                                    base[j] + usize::from(pb.region),
                                );
                            }
                        }
                    }
                }
            }
            // Off-mesh entries landing inside the super-sector.
            for (i, a) in list.iter().enumerate() {
                transitions.extend(
                    a.arcs
                        .iter()
                        .map(|&(x, y)| (base[i] + usize::from(x), base[i] + usize::from(y))),
                );
                for o in a.offmesh() {
                    let mut hit = None;
                    'find: for (j, b) in list.iter().enumerate() {
                        charge(budget, b.regions().len().max(1) as u64)?;
                        for (r, reg) in b.regions().iter().enumerate() {
                            if inside(&reg.bbox, &o.far, b.climb()) {
                                hit = Some(base[j] + r);
                                break 'find;
                            }
                        }
                    }
                    match hit {
                        Some(n) if !o.bidirectional => {
                            transitions.push((base[i] + usize::from(o.region), n))
                        }
                        Some(n) => join(&mut parent, base[i] + usize::from(o.region), n),
                        None => leaving.push((i, *o)),
                    }
                }
            }
        }
        // Super-regions numbered by their first member.
        let mut number = vec![usize::MAX; total];
        let mut regions: Vec<SuperRegion> = Vec::new();
        let mut node_region = vec![0u16; total];
        for (i, s) in list.iter().enumerate() {
            charge(budget, s.regions().len() as u64)?;
            for (r, reg) in s.regions().iter().enumerate() {
                let n = base[i] + r;
                let root = find(&mut parent, n);
                if number[root] == usize::MAX {
                    if regions.len() >= usize::from(u16::MAX) {
                        return Err(Error::FormatLimit("super-sector regions"));
                    }
                    number[root] = regions.len();
                    regions.push(SuperRegion {
                        members: Vec::new(),
                        crossing_bound: f32::INFINITY,
                        bbox: reg.bbox,
                    });
                }
                let sr = &mut regions[number[root]];
                node_region[n] = number[root] as u16;
                sr.members.push((i as u16, r as u16));
                for a in 0..3 {
                    sr.bbox[a] = sr.bbox[a].min(reg.bbox[a]);
                    sr.bbox[a + 3] = sr.bbox[a + 3].max(reg.bbox[a + 3]);
                }
            }
            // Smallest off-diagonal cost per sector region.
            let n = s.transition_count();
            charge(budget, (n as u64) * (n as u64))?;
            let region_of = |t: usize| -> u16 {
                s.portals()
                    .get(t)
                    .map(|p| p.region)
                    .or_else(|| s.offmesh().get(t - s.portals().len()).map(|o| o.region))
                    .unwrap_or(0)
            };
            for f in 0..n {
                for t in 0..n {
                    if f == t || region_of(f) != region_of(t) {
                        continue;
                    }
                    if let Some(c) = s.cost(f, t) {
                        let sr = &mut regions
                            [usize::from(node_region[base[i] + usize::from(region_of(f))])];
                        sr.crossing_bound = sr.crossing_bound.min(c);
                    }
                }
            }
        }
        for r in &mut regions {
            if !r.crossing_bound.is_finite() {
                r.crossing_bound = 0.0;
            }
        }
        let k = i64::from(params.sectors_per_super);
        let (x0, z0) = (i64::from(key.x) * k, i64::from(key.z) * k);
        let mut portals = Vec::new();
        for (i, s) in list.iter().enumerate() {
            let (lx, lz) = (i64::from(s.key().x) - x0, i64::from(s.key().z) - z0);
            for p in s.portals() {
                let out = match p.side {
                    0 => lx == k - 1,
                    4 => lx == 0,
                    2 => lz == k - 1,
                    6 => lz == 0,
                    _ => false,
                };
                if out {
                    portals.push(SuperPortal {
                        side: p.side,
                        split: p.split,
                        region: node_region[base[i] + usize::from(p.region)],
                        sector: i as u16,
                        pos: p.pos,
                        extent: p.extent,
                        y_range: p.y_range,
                    });
                }
            }
        }
        let offmesh = leaving
            .into_iter()
            .map(|(i, o)| SuperOffMesh {
                region: node_region[base[i] + usize::from(o.region)],
                bidirectional: o.bidirectional,
                start: o.start,
                far: o.far,
            })
            .collect();
        let mut arcs: Vec<_> = transitions
            .into_iter()
            .map(|(a, b)| (node_region[a], node_region[b]))
            .filter(|(a, b)| a != b)
            .collect();
        arcs.sort_unstable();
        arcs.dedup();
        let mut exits: Vec<_> = exits
            .into_iter()
            .map(|(n, s, r, c)| (node_region[n], s, r, c))
            .collect();
        exits.sort_unstable();
        exits.dedup();
        let record = SuperRecord {
            bound,
            exits,
            key,
            sectors_per_super: params.sectors_per_super as u16,
            tiles_per_sector: params.tiles_per_sector as u16,
            profile,
            contents,
            climb: list.iter().fold(0.0f32, |m, s| m.max(s.climb())),
            sectors: list.iter().map(|s| (s.key(), s.version())).collect(),
            regions,
            portals,
            offmesh,
            arcs,
        };
        Ok((record, consistent))
    }

    /// Whether cross-sector connectivity was bound to actual mesh links.
    pub fn is_stitched(&self) -> bool {
        self.bound
    }
    /// Remote exit destinations: sector, region and the sector's content id.
    pub(super) fn destinations(&self) -> impl Iterator<Item = (SectorKey, u16, u32)> + '_ {
        self.exits.iter().map(|&(_, k, r, c)| (k, r, c))
    }
    /// Super-sectors the stitched exits land in (with repeats).
    fn exit_supers<'a>(&'a self, params: &'a SectorParams) -> impl Iterator<Item = SuperKey> + 'a {
        self.exits.iter().map(|&(_, k, _, _)| params.super_of(k))
    }
    /// [`SectorSummary::content_id`] of each sector in [`sectors`](Self::sectors), in
    /// the same order.
    pub fn sector_content_ids(&self) -> &[u32] {
        &self.contents
    }
    /// Directed arcs between regions of this record.
    pub fn region_arcs(&self) -> &[(u16, u16)] {
        &self.arcs
    }
    /// The super-sector.
    pub fn key(&self) -> SuperKey {
        self.key
    }
    /// Filter profile id.
    pub fn profile(&self) -> u32 {
        self.profile
    }
    /// Largest climb of the member sectors.
    pub fn climb(&self) -> f32 {
        self.climb
    }
    /// Member sectors and the [`SectorSummary::version`] each was derived from.
    pub fn sectors(&self) -> &[(SectorKey, u32)] {
        &self.sectors
    }
    /// Level-3 nodes.
    pub fn regions(&self) -> &[SuperRegion] {
        &self.regions
    }
    /// Portals on the super-sector boundary.
    pub fn portals(&self) -> &[SuperPortal] {
        &self.portals
    }
    /// Off-mesh connections leaving the super-sector.
    pub fn offmesh(&self) -> &[SuperOffMesh] {
        &self.offmesh
    }

    /// Size of the encoded record.
    pub fn encoded_len(&self) -> usize {
        let members: usize = self.regions.iter().map(|r| r.members.len()).sum();
        HEADER
            + SECTOR * self.sectors.len()
            + REGION * self.regions.len()
            + MEMBER * members
            + PORTAL * self.portals.len()
            + OFFMESH * self.offmesh.len()
            + 4
            + 4 * self.arcs.len()
            + 4
            + EXIT * self.exits.len()
    }

    /// Encodes the `'DNSR'` record.
    pub fn to_bytes(&self) -> Vec<u8> {
        let members: usize = self.regions.iter().map(|r| r.members.len()).sum();
        let mut w = Writer::with_capacity(self.encoded_len());
        w.out.extend_from_slice(&SUPER_MAGIC);
        w.u16(SUPER_VERSION);
        w.u16(u16::from(self.bound));
        w.i32(self.key.x);
        w.i32(self.key.z);
        w.u16(self.sectors_per_super);
        w.u16(self.tiles_per_sector);
        w.u32(self.profile);
        w.f32(self.climb);
        w.u16(self.sectors.len() as u16);
        w.u16(self.regions.len() as u16);
        w.u32(members as u32);
        w.u32(self.portals.len() as u32);
        w.u32(self.offmesh.len() as u32);
        for ((k, v), c) in self.sectors.iter().zip(&self.contents) {
            w.i32(k.x);
            w.i32(k.z);
            w.u32(*v);
            w.u32(*c);
        }
        let mut first = 0u32;
        for r in &self.regions {
            w.u32(first);
            w.u32(r.members.len() as u32);
            w.f32(r.crossing_bound);
            w.f32s(&r.bbox);
            first += r.members.len() as u32;
        }
        for r in &self.regions {
            for &(s, g) in &r.members {
                w.u16(s);
                w.u16(g);
            }
        }
        for p in &self.portals {
            w.u8(p.side);
            w.u8(p.split);
            w.u16(p.region);
            w.u16(p.sector);
            w.u16(0);
            w.f32s(&p.pos);
            w.f32s(&p.extent);
            w.f32s(&p.y_range);
        }
        for o in &self.offmesh {
            w.u16(o.region);
            w.u8(u8::from(o.bidirectional));
            w.u8(0);
            w.f32s(&o.start);
            w.f32s(&o.far);
        }
        w.u32(self.arcs.len() as u32);
        for &(a, b) in &self.arcs {
            w.u16(a);
            w.u16(b);
        }
        w.u32(self.exits.len() as u32);
        for &(a, k, b, c) in &self.exits {
            w.u16(a);
            w.i32(k.x);
            w.i32(k.z);
            w.u16(b);
            w.u32(c);
        }
        w.out
    }

    /// Decodes and validates a `'DNSR'` record; accepted bytes re-encode to the same
    /// bytes. Errors: [`Error::InvalidSnapshot`] with
    /// [`SnapshotError::WrongVersion`](crate::SnapshotError::WrongVersion) for another
    /// record version; [`Error::InvalidInput`] for a malformed record (magic, flags,
    /// padding, length, float, index, ordering or membership errors, a sector region
    /// listed twice or skipped);
    /// [`Error::LimitExceeded`] above `limits.max_bytes`.
    pub fn from_bytes(data: &[u8], limits: &Limits) -> Result<Self, Error> {
        check_bytes(data.len(), limits)?;
        let mut r = Reader::new(data, "super-sector record");
        let magic = [r.u8()?, r.u8()?, r.u8()?, r.u8()?];
        let (version, flags) = (r.u16()?, r.u16()?);
        r.require(magic == SUPER_MAGIC)?;
        if version != SUPER_VERSION {
            return Err(Error::InvalidSnapshot(SnapshotError::WrongVersion));
        }
        r.require(flags <= 1)?;
        let key = SuperKey {
            x: r.i32()?,
            z: r.i32()?,
        };
        let (sectors_per_super, tiles_per_sector) = (r.u16()?, r.u16()?);
        let profile = r.u32()?;
        let climb = r.f32()?;
        let (ns, nr) = (usize::from(r.u16()?), usize::from(r.u16()?));
        let (nm, np, no) = (r.u32()? as u64, r.u32()? as u64, r.u32()? as u64);
        r.require(sectors_per_super > 0 && tiles_per_sector > 0 && climb >= 0.0)?;
        let expected = (HEADER + SECTOR * ns + REGION * nr) as u64
            + MEMBER as u64 * nm
            + PORTAL as u64 * np
            + OFFMESH as u64 * no;
        r.require(data.len() as u64 >= expected + 4)?;
        let k = i64::from(sectors_per_super);
        let (x0, z0) = (i64::from(key.x) * k, i64::from(key.z) * k);
        let mut sectors: Vec<(SectorKey, u32)> = Vec::with_capacity(ns);
        let mut contents = Vec::with_capacity(ns);
        for _ in 0..ns {
            let s = SectorKey {
                x: r.i32()?,
                z: r.i32()?,
            };
            let v = r.u32()?;
            contents.push(r.u32()?);
            r.require(
                (x0..x0 + k).contains(&i64::from(s.x)) && (z0..z0 + k).contains(&i64::from(s.z)),
            )?;
            if let Some((p, _)) = sectors.last() {
                r.require((p.z, p.x) < (s.z, s.x))?;
            }
            sectors.push((s, v));
        }
        let mut regions = Vec::with_capacity(nr);
        let mut next = 0u64;
        for _ in 0..nr {
            let (first, count) = (u64::from(r.u32()?), u64::from(r.u32()?));
            let crossing_bound = r.f32()?;
            let bbox: [f32; 6] = r.f32s()?;
            r.require(first == next && count > 0 && crossing_bound >= 0.0)?;
            r.require((0..3).all(|a| ordered(bbox[a], bbox[a + 3])))?;
            next += count;
            regions.push((
                count,
                SuperRegion {
                    members: Vec::new(),
                    crossing_bound,
                    bbox,
                },
            ));
        }
        r.require(next == nm)?;
        let mut out_regions = Vec::with_capacity(nr);
        let mut all = Vec::with_capacity(nm as usize);
        for (count, mut reg) in regions {
            for _ in 0..count {
                let m = (r.u16()?, r.u16()?);
                r.require(usize::from(m.0) < ns)?;
                if let Some(p) = reg.members.last() {
                    r.require(*p < m)?;
                }
                reg.members.push(m);
                all.push(m);
            }
            out_regions.push(reg);
        }
        // Every region of a listed sector belongs to exactly one super-region, so each
        // sector's members are its regions 0, 1, ... without gaps or repeats.
        all.sort_unstable();
        r.require(all.iter().enumerate().all(|(i, &(s, g))| {
            if g == 0 {
                i == 0 || all[i - 1].0 != s
            } else {
                i > 0 && all[i - 1] == (s, g - 1)
            }
        }))?;
        let mut portals = Vec::with_capacity(np as usize);
        for _ in 0..np {
            let (side, split, region, sector) = (r.u8()?, r.u8()?, r.u16()?, r.u16()?);
            r.pad(2)?;
            let p = SuperPortal {
                side,
                split,
                region,
                sector,
                pos: r.f32s()?,
                extent: r.f32s()?,
                y_range: r.f32s()?,
            };
            r.require(matches!(p.side, 0 | 2 | 4 | 6) && p.split <= 2)?;
            r.require(usize::from(p.region) < nr && usize::from(p.sector) < ns)?;
            r.require(ordered(p.extent[0], p.extent[1]) && ordered(p.y_range[0], p.y_range[1]))?;
            portals.push(p);
        }
        let mut offmesh = Vec::with_capacity(no as usize);
        for _ in 0..no {
            let region = r.u16()?;
            let flags = r.u8()?;
            r.pad(1)?;
            r.require(flags <= 1 && usize::from(region) < nr)?;
            offmesh.push(SuperOffMesh {
                region,
                bidirectional: flags == 1,
                start: r.f32s()?,
                far: r.f32s()?,
            });
        }
        let n = r.u32()? as usize;
        r.require(expected + 8 + 4 * n as u64 <= data.len() as u64)?;
        let mut arcs = Vec::with_capacity(n);
        for _ in 0..n {
            let e = (r.u16()?, r.u16()?);
            r.require(
                e.0 != e.1 && usize::from(e.0.max(e.1)) < nr && arcs.last().is_none_or(|p| *p < e),
            )?;
            arcs.push(e);
        }
        let ne = r.u32()? as usize;
        r.require(
            (ne == 0 || flags == 1)
                && expected + 8 + 4 * n as u64 + EXIT as u64 * ne as u64 == data.len() as u64,
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
                r.u32()?,
            );
            r.require(usize::from(e.0) < nr && exits.last().is_none_or(|p| *p < e))?;
            exits.push(e);
        }
        r.finish()?;
        Ok(SuperRecord {
            key,
            sectors_per_super,
            tiles_per_sector,
            profile,
            climb,
            sectors,
            contents,
            regions: out_regions,
            bound: flags == 1,
            exits,
            portals,
            offmesh,
            arcs,
        })
    }
}

/// A level-3 edge: from a super-region to a super-region of a neighbouring
/// super-sector, with a lower bound of the cost of crossing the source region.
#[derive(Clone, Copy, Debug, PartialEq)]
#[non_exhaustive]
#[cfg_attr(feature = "serde", derive(serde::Serialize))]
pub struct SuperEdge {
    /// Target super-sector.
    pub to: SuperKey,
    /// Target super-region.
    pub region: u16,
    /// Lower bound of the cost (the source region's crossing bound).
    pub cost: f32,
}

/// Compression figures of one resident sector summary.
#[derive(Clone, Copy, Debug, PartialEq)]
#[non_exhaustive]
#[cfg_attr(feature = "serde", derive(serde::Serialize))]
pub struct Compression {
    /// Passable polygons.
    pub polygons: u32,
    /// Portal transition points.
    pub portals: u32,
    /// Regions.
    pub regions: u32,
    /// Portals plus off-mesh entries.
    pub transitions: u32,
}

/// What one graph edit rebuilt: the bound a streamed world's edits must keep.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
#[non_exhaustive]
#[cfg_attr(feature = "serde", derive(serde::Serialize))]
pub struct GraphUpdate {
    /// Sector summaries replaced.
    pub sectors: u32,
    /// Super-sector records re-derived.
    pub supers: u32,
    /// Super-sector records marked stale (their sector summaries are not all
    /// resident, so they wait for a fresh record).
    pub stale: u32,
    /// Super-sectors whose level-3 edges were recomputed.
    pub edge_sets: u32,
}

#[derive(Clone, Debug)]
pub(super) struct SuperNode {
    pub(super) record: SuperRecord,
    pub(super) stale: bool,
    pub(super) edges: Vec<Vec<SuperEdge>>,
}

/// Resident level-2 sector summaries and the always-resident level-3 graph.
///
/// Level 3 holds one [`SuperRecord`] per super-sector and profile, loaded with
/// [`insert_super`](Self::insert_super) or derived from resident summaries. Inserting
/// a summary re-derives its super-sector's record when every sector that record lists
/// is resident (or no record exists yet); otherwise the record is marked stale.
/// [`remove_sector`](Self::remove_sector) only evicts the level-2 payload: level 3 is
/// unchanged. An edit therefore touches one sector, one super-sector and the level-3
/// edges of its 3 x 3 neighbourhood and of the records whose stitched exits land in
/// it (found through an index, not a scan), whatever the world size. Every edit's
/// work is charged against the graph's [`Limits::max_work`].
///
/// Records built without [`from_mesh`](Self::from_mesh) (isolated summaries,
/// [`SuperRecord::is_stitched`] false) estimate connectivity from portal overlap and
/// off-mesh landing boxes: they are hints that can report connections that do not
/// exist and miss ones that do. Only stitched records are exact.
#[derive(Clone, Debug)]
pub struct SectorGraph {
    pub(super) slot_keys: BTreeMap<usize, (i32, i32, u8)>,
    pub(super) owner: Option<alloc::sync::Arc<()>>,
    pub(super) global_links: bool,
    pub(super) bound_profile: Option<u32>,
    /// The whole filter a stitched graph was built with: a profile id is a 32-bit
    /// hash, so an atlas compares the filter itself.
    pub(super) bound_filter: Option<crate::detour::DefaultFilter>,
    pub(super) bound_epoch: Option<u64>,
    /// Super-sectors whose stitched exits land in each super-sector, so an edit
    /// relinks only the records that name it.
    inbound: BTreeMap<(u32, SuperKey), BTreeSet<SuperKey>>,
    #[cfg(feature = "dynamic")]
    pub(super) state_epoch: u64,
    pub(super) crossings: BTreeMap<SectorKey, Vec<Crossing>>,
    revision: u64,
    params: SectorParams,
    limits: Limits,
    pub(super) sectors: BTreeMap<(u32, SectorKey), SectorSummary>,
    pub(super) supers: BTreeMap<(u32, SuperKey), SuperNode>,
}

impl SectorGraph {
    /// An empty graph. Errors: [`Error::InvalidParameter`] for bad `params`.
    pub fn new(params: &SectorParams, limits: Limits) -> Result<Self, Error> {
        params.validate()?;
        Ok(SectorGraph {
            revision: 0,
            bound_epoch: None,
            slot_keys: BTreeMap::new(),
            owner: None,
            bound_profile: None,
            bound_filter: None,
            inbound: BTreeMap::new(),
            global_links: false,
            crossings: BTreeMap::new(),
            #[cfg(feature = "dynamic")]
            state_epoch: 0,
            params: *params,
            limits,
            sectors: BTreeMap::new(),
            supers: BTreeMap::new(),
        })
    }

    /// Grouping parameters.
    pub fn params(&self) -> &SectorParams {
        &self.params
    }

    /// Decodes a `'DNSS'` record and inserts it (see [`insert_summary`](Self::insert_summary)).
    pub fn insert_sector(&mut self, bytes: &[u8]) -> Result<GraphUpdate, Error> {
        let s = SectorSummary::from_bytes(bytes, &self.limits)?;
        self.insert_summary(s)
    }

    /// Inserts or replaces a sector summary and updates level 3.
    ///
    /// The super-sector's record is re-derived only when every other sector it lists
    /// is resident with the summary version the record lists; otherwise, or when a
    /// stitched crossing inside the super-sector names a summary from another bake,
    /// the record is marked stale. The work of the update (derivation and relinking)
    /// is charged against the graph's [`Limits::max_work`]. Errors:
    /// [`Error::InvalidInput`] when the summary's sector size differs from the
    /// graph's; [`Error::LimitExceeded`] when the update would exceed the work limit,
    /// with the graph unchanged.
    pub fn insert_summary(&mut self, s: SectorSummary) -> Result<GraphUpdate, Error> {
        if u32::from(s.tiles_per_sector()) != self.params.tiles_per_sector {
            return Err(Error::InvalidInput("sector size differs from the graph's"));
        }
        let (profile, key) = (s.profile(), s.key());
        let sk = self.params.super_of(key);
        let mut budget = Budget::from_limits(&self.limits);

        let mut update = GraphUpdate {
            sectors: 1,
            ..GraphUpdate::default()
        };
        let complete = match self.supers.get(&(profile, sk)) {
            None => true,
            Some(node) => {
                charge(&mut budget, node.record.sectors().len() as u64)?;
                node.record.sectors().iter().all(|(k, v)| {
                    *k == key
                        || self
                            .sectors
                            .get(&(profile, *k))
                            .is_some_and(|resident| resident.version() == *v)
                })
            }
        };
        if complete {
            // Sector keys are ordered by (x, z): scan the super-sector's columns.
            let k = i64::from(self.params.sectors_per_super);
            let clamp = |v: i64| v.clamp(i64::from(i32::MIN), i64::from(i32::MAX)) as i32;
            let (x_lo, z_lo) = (i64::from(sk.x) * k, i64::from(sk.z) * k);
            let lo = SectorKey {
                x: clamp(x_lo),
                z: clamp(z_lo),
            };
            let hi = SectorKey {
                x: clamp(x_lo + k - 1),
                z: clamp(z_lo + k - 1),
            };
            let mut members: Vec<&SectorSummary> = Vec::new();
            for ((_, candidate), resident) in self.sectors.range((profile, lo)..=(profile, hi)) {
                charge(&mut budget, 1)?;
                if *candidate != key && (lo.z..=hi.z).contains(&candidate.z) {
                    members.push(resident);
                }
            }
            members.push(&s);
            let (record, consistent) =
                SuperRecord::derive_budgeted(sk, &self.params, &members, &mut budget)?;
            if consistent || !self.supers.contains_key(&(profile, sk)) {
                update.edge_sets = self.install(
                    profile,
                    SuperNode {
                        record,
                        stale: !consistent,
                        edges: Vec::new(),
                    },
                    &mut budget,
                )?;
                update.supers = 1;
            }
            if !consistent {
                update.stale = 1;
            }
        }
        if !complete || update.stale == 1 {
            if let Some(node) = self.supers.get_mut(&(profile, sk)) {
                node.stale = true;
            }
            update.stale = 1;
        }
        self.sectors.insert((profile, key), s);
        self.bound_epoch = None;
        self.revision = self.revision.wrapping_add(1);
        Ok(update)
    }

    /// Evicts a resident sector summary (level 2 only; level 3 is unchanged).
    pub fn remove_sector(&mut self, profile: u32, key: SectorKey) -> Option<SectorSummary> {
        self.revision = self.revision.wrapping_add(1);
        self.bound_epoch = None;
        self.sectors.remove(&(profile, key))
    }

    /// Decodes a `'DNSR'` record and installs it as the level-3 record of its
    /// super-sector, replacing any derived or stale one. The record is installed
    /// stale when a sector it lists is resident with another summary version (the
    /// two come from different bakes). Relinking is charged against the graph's
    /// [`Limits::max_work`]. Errors: as [`SuperRecord::from_bytes`];
    /// [`Error::InvalidInput`] when its grouping differs from the graph's;
    /// [`Error::LimitExceeded`] when relinking would exceed the work limit, with the
    /// graph unchanged.
    pub fn insert_super(&mut self, bytes: &[u8]) -> Result<GraphUpdate, Error> {
        let record = SuperRecord::from_bytes(bytes, &self.limits)?;
        if u32::from(record.sectors_per_super) != self.params.sectors_per_super
            || u32::from(record.tiles_per_sector) != self.params.tiles_per_sector
        {
            return Err(Error::InvalidInput("grouping differs from the graph's"));
        }
        let mut budget = Budget::from_limits(&self.limits);
        let profile = record.profile;
        charge(&mut budget, record.sectors.len() as u64)?;
        let stale = record.sectors.iter().any(|(k, v)| {
            self.sectors
                .get(&(profile, *k))
                .is_some_and(|resident| resident.version() != *v)
        });
        let edge_sets = self.install(
            profile,
            SuperNode {
                record,
                stale,
                edges: Vec::new(),
            },
            &mut budget,
        )?;
        self.revision = self.revision.wrapping_add(1);
        self.bound_epoch = None;
        Ok(GraphUpdate {
            supers: 1,
            stale: u32::from(stale),
            edge_sets,
            ..GraphUpdate::default()
        })
    }

    /// Replaces the record of `node`'s super-sector and recomputes the level-3 edges
    /// of its 3 x 3 neighbourhood and of the records whose stitched exits land in it.
    /// On an error the graph is unchanged. Returns the number of edge sets rebuilt.
    fn install(
        &mut self,
        profile: u32,
        node: SuperNode,
        budget: &mut Budget,
    ) -> Result<u32, Error> {
        let sk = node.record.key;
        let old = self.supers.insert((profile, sk), node);
        match self.relinked(profile, sk, budget) {
            Ok(sets) => {
                if let Some(old) = &old {
                    for target in old.record.exit_supers(&self.params) {
                        if let Some(sources) = self.inbound.get_mut(&(profile, target)) {
                            sources.remove(&sk);
                        }
                    }
                }
                let targets: Vec<SuperKey> = self
                    .supers
                    .get(&(profile, sk))
                    .map(|n| n.record.exit_supers(&self.params).collect())
                    .unwrap_or_default();
                for target in targets {
                    self.inbound
                        .entry((profile, target))
                        .or_default()
                        .insert(sk);
                }
                let n = sets.len() as u32;
                for (k, edges) in sets {
                    if let Some(node) = self.supers.get_mut(&(profile, k)) {
                        node.edges = edges;
                    }
                }
                Ok(n)
            }
            Err(e) => {
                match old {
                    Some(old) => self.supers.insert((profile, sk), old),
                    None => self.supers.remove(&(profile, sk)),
                };
                Err(e)
            }
        }
    }

    /// Level-3 edges of `sk`, its 3 x 3 neighbourhood and the records whose stitched
    /// exits land in `sk`, computed without changing the graph.
    fn relinked(
        &self,
        profile: u32,
        sk: SuperKey,
        budget: &mut Budget,
    ) -> Result<Vec<(SuperKey, EdgeSet)>, Error> {
        let mut keys = BTreeSet::new();
        for dz in -1..=1 {
            for dx in -1..=1 {
                if let (Some(x), Some(z)) = (sk.x.checked_add(dx), sk.z.checked_add(dz)) {
                    keys.insert(SuperKey { x, z });
                }
            }
        }
        if let Some(sources) = self.inbound.get(&(profile, sk)) {
            charge(budget, sources.len() as u64)?;
            keys.extend(sources.iter().copied());
        }
        let mut sets = Vec::new();
        for k in keys {
            if self.supers.contains_key(&(profile, k)) {
                sets.push((k, self.edges_of(profile, k, budget)?));
            }
        }
        Ok(sets)
    }

    /// Level-3 edges of one record, charging one unit per arc, exit, region lookup,
    /// portal pair and off-mesh landing test.
    pub(super) fn edges_of(
        &self,
        profile: u32,
        sk: SuperKey,
        budget: &mut Budget,
    ) -> Result<Vec<Vec<SuperEdge>>, Error> {
        let Some(node) = self.supers.get(&(profile, sk)) else {
            return Ok(Vec::new());
        };
        let rec = &node.record;
        charge(budget, (rec.regions.len() + rec.arcs.len()) as u64)?;
        let mut edges: Vec<Vec<SuperEdge>> = vec![Vec::new(); rec.regions.len()];
        let mut add = |from: u16, to: SuperKey, region: u16| {
            if let Some(list) = edges.get_mut(usize::from(from)) {
                let cost = rec.regions[usize::from(from)].crossing_bound;
                list.push(SuperEdge { to, region, cost });
            }
        };
        for &(a, b) in &rec.arcs {
            add(a, sk, b);
        }
        if rec.bound {
            for &(a, k, b, _) in &rec.exits {
                let sk = self.params.super_of(k);
                let regions = self
                    .supers
                    .get(&(profile, sk))
                    .map_or(0, |n| n.record.regions.len());
                charge(budget, 1 + regions as u64)?;
                if let Some(region) = self.super_region(profile, k, b) {
                    add(a, sk, region);
                }
            }
            return Ok(Self::sorted(edges));
        }
        for (side, d) in [(0u8, (1, 0)), (2u8, (0, 1)), (4u8, (-1, 0)), (6u8, (0, -1))] {
            let (Some(x), Some(z)) = (sk.x.checked_add(d.0), sk.z.checked_add(d.1)) else {
                continue;
            };
            let nk = SuperKey { x, z };
            let Some(other) = self.supers.get(&(profile, nk)) else {
                continue;
            };
            let climb = rec.climb.max(other.record.climb);
            for pa in rec.portals.iter().filter(|p| p.side == side) {
                charge(budget, other.record.portals.len() as u64)?;
                for pb in other
                    .record
                    .portals
                    .iter()
                    .filter(|p| p.side == opposite_side(side))
                {
                    if runs_meet(
                        pa.extent, pa.y_range, pb.extent, pb.y_range, climb, MEET_EPS,
                    ) {
                        add(pa.region, nk, pb.region);
                    }
                }
            }
        }
        // Off-mesh links: out of this super-sector, and bidirectional ones back in.
        for dz in -1..=1 {
            for dx in -1..=1 {
                let (Some(x), Some(z)) = (sk.x.checked_add(dx), sk.z.checked_add(dz)) else {
                    continue;
                };
                let nk = SuperKey { x, z };
                let Some(other) = self.supers.get(&(profile, nk)) else {
                    continue;
                };
                charge(
                    budget,
                    (rec.offmesh.len() as u64).saturating_mul(other.record.regions.len() as u64)
                        + (other.record.offmesh.len() as u64)
                            .saturating_mul(rec.regions.len() as u64),
                )?;
                if nk != sk {
                    for o in &rec.offmesh {
                        if let Some(r) = other
                            .record
                            .regions
                            .iter()
                            .position(|g| inside(&g.bbox, &o.far, other.record.climb))
                        {
                            add(o.region, nk, r as u16);
                        }
                    }
                }
                for o in other.record.offmesh.iter().filter(|o| o.bidirectional) {
                    if nk == sk {
                        continue;
                    }
                    if let Some(r) = rec
                        .regions
                        .iter()
                        .position(|g| inside(&g.bbox, &o.far, rec.climb))
                    {
                        add(r as u16, nk, o.region);
                    }
                }
            }
        }
        Ok(Self::sorted(edges))
    }

    /// Sorts each region's edges by target and drops repeats (their cost is the
    /// source region's, so repeats are identical).
    fn sorted(mut edges: Vec<Vec<SuperEdge>>) -> Vec<Vec<SuperEdge>> {
        for list in &mut edges {
            list.sort_unstable_by_key(|e| (e.to.z, e.to.x, e.region));
            list.dedup_by_key(|e| (e.to, e.region));
        }
        edges
    }

    /// Rebuilds the index of records whose stitched exits land in each super-sector.
    pub(super) fn rebuild_inbound(&mut self) {
        let mut inbound: BTreeMap<(u32, SuperKey), BTreeSet<SuperKey>> = BTreeMap::new();
        for ((profile, sk), node) in &self.supers {
            for target in node.record.exit_supers(&self.params) {
                inbound.entry((*profile, target)).or_default().insert(*sk);
            }
        }
        self.inbound = inbound;
    }

    /// Finds the level-three region containing a sector region.
    pub fn super_region(&self, profile: u32, key: SectorKey, region: u16) -> Option<u16> {
        let rec = self.super_record(profile, self.params.super_of(key))?;
        let i = rec
            .sectors
            .binary_search_by_key(&(key.z, key.x), |(k, _)| (k.z, k.x))
            .ok()?;
        rec.regions
            .iter()
            .position(|r| r.members.binary_search(&(i as u16, region)).is_ok())
            .map(|r| r as u16)
    }
    /// A resident sector summary.
    pub fn sector(&self, profile: u32, key: SectorKey) -> Option<&SectorSummary> {
        self.sectors.get(&(profile, key))
    }

    /// The level-3 record of a super-sector.
    pub fn super_record(&self, profile: u32, key: SuperKey) -> Option<&SuperRecord> {
        self.supers.get(&(profile, key)).map(|n| &n.record)
    }

    /// Whether a super-sector's record waits for a fresh one (`None` when absent).
    pub fn is_stale(&self, profile: u32, key: SuperKey) -> Option<bool> {
        self.supers.get(&(profile, key)).map(|n| n.stale)
    }

    /// Level-3 edges leaving super-region `region` of super-sector `key`.
    pub fn super_edges(&self, profile: u32, key: SuperKey, region: u16) -> &[SuperEdge] {
        self.supers
            .get(&(profile, key))
            .and_then(|n| n.edges.get(usize::from(region)))
            .map_or(&[], Vec::as_slice)
    }

    /// Resident sector summaries.
    pub fn sector_count(&self) -> usize {
        self.sectors.len()
    }

    /// Level-3 records.
    pub fn super_count(&self) -> usize {
        self.supers.len()
    }

    /// Bytes retained per level, as encoded records plus level-3 edges: level 1 is
    /// the navmesh's own tiles and always zero here.
    pub fn retained_bytes_by_level(&self) -> [usize; 3] {
        let l2 = self.sectors.values().map(SectorSummary::encoded_len).sum();
        let l3 = self
            .supers
            .values()
            .map(|n| {
                n.record.encoded_len()
                    + n.edges.iter().map(Vec::len).sum::<usize>()
                        * core::mem::size_of::<SuperEdge>()
            })
            .sum();
        [0, l2, l3]
    }

    /// Compression figures of a resident sector.
    pub fn compression(&self, profile: u32, key: SectorKey) -> Option<Compression> {
        let s = self.sectors.get(&(profile, key))?;
        Some(Compression {
            polygons: s.polygon_count(),
            portals: s.portals().len() as u32,
            regions: s.regions().len() as u32,
            transitions: s.transition_count() as u32,
        })
    }
}

impl SectorGraph {
    /// Mutation revision used to invalidate compiled views and searches.
    pub fn revision(&self) -> u64 {
        self.revision
    }

    pub(crate) fn records(&self, profile: u32) -> impl Iterator<Item = &SuperRecord> {
        self.supers
            .iter()
            .filter(move |((p, _), _)| *p == profile)
            .map(|(_, n)| &n.record)
    }
}
