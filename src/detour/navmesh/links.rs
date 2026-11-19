//! Managed off-mesh tiles and their reserved links into ground polygons.

use super::*;
use crate::SnapshotError;
use crate::detour::Link;

const RECORD_SIZE: usize = 36;
const SNAPSHOT_VERSION: u32 = 1;
use alloc::collections::BTreeMap;

/// Caller-assigned, mesh-wide identity of a runtime connection.
#[derive(Clone, Copy, Debug, Eq, Hash, Ord, PartialEq, PartialOrd)]
pub struct LinkId(pub u32);

/// An off-mesh connection in the navmesh's coordinate frame.
#[derive(Clone, Copy, Debug, PartialEq)]
#[non_exhaustive]
pub struct LinkSpec {
    /// Stable identity, also used as the connection's Detour user id.
    pub id: LinkId,
    /// Authored start. Its column owns the link-only tile.
    pub start: [f32; 3],
    /// Authored destination.
    pub end: [f32; 3],
    /// Maximum three-dimensional snap distance at either endpoint.
    pub radius: f32,
    /// Allow traversal from end to start as well.
    pub bidirectional: bool,
    /// Detour area in 0..=63.
    pub area: u8,
    /// Detour filter flags when the connection is available.
    pub flags: u16,
}

impl LinkSpec {
    /// Creates a one-way connection with area 0 and filter flags 1.
    pub fn new(id: LinkId, start: [f32; 3], end: [f32; 3], radius: f32) -> Self {
        Self {
            id,
            start,
            end,
            radius,
            bidirectional: false,
            area: 0,
            flags: 1,
        }
    }

    fn validate(&self) -> Result<(), Error> {
        if !self.radius.is_finite()
            || self.radius <= 0.0
            || !self.start.iter().chain(&self.end).all(|v| v.is_finite())
            || self.area >= 64
            || self.flags == 0
        {
            return Err(Error::InvalidParameter("runtime link definition"));
        }
        Ok(())
    }
}

/// Resolution of one endpoint of a runtime connection.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
#[non_exhaustive]
pub enum Landing {
    /// The endpoint snapped to this resident ground polygon.
    Connected(PolyRef),
    /// No eligible polygon was found within the endpoint's radius.
    NoPolygonInRadius,
    /// A required back-link has no reserved ground slot.
    NoLinkCapacity {
        /// Column of the ground polygon.
        key: TileKey,
        /// Layer of the ground polygon.
        layer: i32,
    },
    /// The endpoint's column is known and still needs ground tiles.
    Unloaded(TileKey),
}

/// Current availability and landing diagnostics for a runtime connection.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
#[non_exhaustive]
pub struct LinkReport {
    /// Stable link id.
    pub id: LinkId,
    /// The caller's requested availability.
    pub enabled: bool,
    /// Start endpoint resolution.
    pub start: Landing,
    /// End endpoint resolution.
    pub end: Landing,
}

impl LinkReport {
    /// The connection has both landings and may be traversed subject to the query filter.
    pub fn is_traversable(&self) -> bool {
        self.enabled
            && matches!(
                (self.start, self.end),
                (Landing::Connected(_), Landing::Connected(_))
            )
    }

    fn check_capacity(&self) -> Result<(), Error> {
        for endpoint in [self.start, self.end] {
            if let Landing::NoLinkCapacity { key, layer } = endpoint {
                return Err(Error::NoLinkCapacity {
                    x: key.x,
                    y: key.y,
                    layer,
                });
            }
        }
        Ok(())
    }
}

#[derive(Clone, Copy, Debug)]
struct Entry {
    key: TileKey,
    spec: LinkSpec,
    enabled: bool,
    report: LinkReport,
    landings: [(Landing, [f32; 3]); 2],
}

impl Entry {
    fn new(key: TileKey, spec: LinkSpec, enabled: bool) -> Self {
        Self {
            key,
            spec,
            enabled,
            landings: [
                (Landing::NoPolygonInRadius, spec.start),
                (Landing::NoPolygonInRadius, spec.end),
            ],
            report: LinkReport {
                id: spec.id,
                enabled,
                start: Landing::NoPolygonInRadius,
                end: Landing::NoPolygonInRadius,
            },
        }
    }
}

#[derive(Clone, Debug, Default)]
pub(super) struct RuntimeLinks {
    layer: Option<i32>,
    entries: BTreeMap<LinkId, Entry>,
    // Back-links currently installed, so cleanup never scans unrelated polygons.
    back_links: Vec<(PolyRef, PolyRef)>,
    changed_at: u64,
}

struct Resolved {
    report: LinkReport,
    points: [[f32; 3]; 2],
}

impl NavMesh {
    /// Layer reserved for runtime connection tiles.
    pub fn link_layer(&self) -> Option<i32> {
        self.runtime_links.layer
    }

    /// Reserves a nonnegative tile layer for runtime links. Ground operations
    /// cannot use this layer. A keyed grid must include the layer in its slot
    /// policy. Clear the reservation only after all runtime links are removed.
    pub fn set_link_layer(&mut self, layer: Option<i32>) -> Result<(), Error> {
        if layer == self.link_layer() {
            return Ok(());
        }
        if !self.runtime_links.entries.is_empty() {
            return Err(Error::InvalidParameter("runtime link layer is in use"));
        }
        self.validate_link_layer(layer)?;
        if let Some(l) = layer {
            if self
                .slots
                .indices()
                .any(|i| self.slots[i].parked.is_some_and(|p| p.layer == l))
            {
                return Err(Error::InvalidParameter(
                    "runtime link layer overlaps unloaded tiles",
                ));
            }
        }
        self.runtime_links.layer = layer;
        Ok(())
    }

    fn validate_link_layer(&self, layer: Option<i32>) -> Result<(), Error> {
        if let Some(l) = layer {
            if l < 0
                || matches!(self.slot_policy(), SlotPolicy::Keyed { max_layers, .. } if l as u32 >= max_layers)
            {
                return Err(Error::TileOutsideGrid);
            }
            if self
                .slots
                .indices()
                .any(|i| self.tile(i).is_some_and(|t| t.header.layer == l))
            {
                return Err(Error::InvalidParameter(
                    "runtime link layer overlaps ground tiles",
                ));
            }
        }
        Ok(())
    }

    /// Adds a runtime connection with its initial game-controlled availability.
    /// Missing landings produce a pending report. When both landings exist and
    /// a required ground slot is unavailable, the operation fails atomically.
    pub fn add_runtime_link(&mut self, spec: LinkSpec, enabled: bool) -> Result<LinkReport, Error> {
        spec.validate()?;
        let layer = self
            .link_layer()
            .ok_or(Error::InvalidParameter("no runtime link layer"))?;
        if self.runtime_links.entries.contains_key(&spec.id) {
            return Err(Error::InvalidParameter("duplicate runtime link id"));
        }
        let (x, y) = self.calc_tile_loc(&spec.start);
        let mut entry = Entry::new(TileKey { x, y }, spec, enabled);
        entry.landings = self.resolve_landings(&spec, &self.ground_in_landing_order());
        // Resolve the proposed full set before changing any tile or consuming a salt.
        let mut entries = self.runtime_links.entries.clone();
        entries.insert(spec.id, entry);
        let resolved = self.resolve_runtime_links(&entries);
        for (id, r) in resolved {
            if id == spec.id
                || self.runtime_links.entries.get(&id).is_some_and(|e| {
                    matches!(
                        (e.report.start, e.report.end),
                        (Landing::Connected(_), Landing::Connected(_))
                    )
                })
            {
                r.report.check_capacity()?;
            }
        }
        self.rebuild_link_column(entry.key, layer, entries.values())?;
        self.runtime_links.entries = entries;
        self.runtime_links.changed_at = self.epoch();
        self.refresh_runtime_links();
        Ok(self.runtime_links.entries[&spec.id].report)
    }

    /// Removes a connection. Other links in its column get new polygon refs
    /// if its link-only tile changes. Ground refs retain their salts.
    pub fn remove_runtime_link(&mut self, id: LinkId) -> Result<(), Error> {
        let layer = self
            .link_layer()
            .ok_or(Error::InvalidParameter("no runtime link layer"))?;
        let entry = self
            .runtime_links
            .entries
            .get(&id)
            .ok_or(Error::InvalidParameter("unknown runtime link id"))?;
        self.rebuild_link_column(
            entry.key,
            layer,
            self.runtime_links
                .entries
                .values()
                .filter(|e| e.spec.id != id)
                .copied()
                .collect::<Vec<_>>()
                .iter(),
        )?;
        self.runtime_links.entries.remove(&id);
        self.runtime_links.changed_at = self.epoch();
        self.refresh_runtime_links();
        Ok(())
    }

    /// Sets game-controlled availability immediately, without rebuilding tiles or
    /// advancing salts. Connected disabled links retain their slot reservations.
    pub fn set_runtime_link_enabled(
        &mut self,
        id: LinkId,
        enabled: bool,
    ) -> Result<LinkReport, Error> {
        let entry = self
            .runtime_links
            .entries
            .get_mut(&id)
            .ok_or(Error::InvalidParameter("unknown runtime link id"))?;
        if enabled {
            entry.report.check_capacity()?;
        }
        if entry.enabled != enabled {
            entry.enabled = enabled;
            self.stream.bump_epoch();
            self.refresh_runtime_links();
        }
        Ok(self.runtime_links.entries[&id].report)
    }

    /// Current landing diagnostics for the given runtime connection.
    pub fn runtime_link_report(&self, id: LinkId) -> Option<LinkReport> {
        self.runtime_links.entries.get(&id).map(|e| e.report)
    }

    /// Runtime connection reports in stable id order.
    pub fn runtime_link_reports(&self) -> impl Iterator<Item = LinkReport> + '_ {
        self.runtime_links.entries.values().map(|e| e.report)
    }

    /// Stable runtime connection id for a resident connection polygon.
    pub fn runtime_link_id(&self, poly: PolyRef) -> Option<LinkId> {
        let (tile, _) = self.tile_and_poly_by_ref(poly).ok()?;
        if self.link_layer() != Some(tile.header.layer) {
            return None;
        }
        self.off_mesh_connection_by_ref(poly)
            .map(|c| LinkId(c.user_id))
    }

    pub(super) fn check_ground_layer(&self, layer: i32) -> Result<(), Error> {
        if self.link_layer() == Some(layer) {
            return Err(Error::InvalidParameter("reserved runtime link layer"));
        }
        Ok(())
    }

    pub(super) fn ground_slots(&self) -> Vec<usize> {
        self.resident_slots()
            .into_iter()
            .filter(|&i| self.link_layer() != Some(self.tile_unchecked(i as u32).header.layer))
            .collect()
    }

    pub(crate) fn runtime_links_changed_at(&self) -> u64 {
        self.runtime_links.changed_at
    }

    pub(super) fn runtime_link_work(&self, incoming: &MeshTile) -> u64 {
        let n = self.runtime_links.entries.len() as u64;
        if n == 0 {
            return 0;
        }
        let mut candidates = 1 + incoming.polys.len() as u64 + incoming.links.len() as u64;
        for i in self.ground_slots() {
            let t = self.tile_unchecked(i as u32);
            candidates = candidates.saturating_add(1 + t.polys.len() as u64 + t.links.len() as u64);
        }
        n.saturating_mul(2).saturating_mul(candidates)
    }

    fn rebuild_link_column<'a>(
        &mut self,
        key: TileKey,
        layer: i32,
        entries: impl Iterator<Item = &'a Entry>,
    ) -> Result<(), Error> {
        let specs: Vec<_> = entries.filter(|e| e.key == key).map(|e| e.spec).collect();
        if specs.is_empty() {
            let ti = self
                .tile_index_at(key.x, key.y, layer)
                .ok_or(Error::InvalidRef)?;
            let salt = self.next_salt(self.slots[ti].salt)?;
            // The slot is only released after every check has succeeded.
            self.clear_runtime_edges();
            drop(self.detach(ti));
            self.release_slot(ti, salt);
        } else {
            let bytes = super::super::builder::encode_link_tile(key, layer, &specs, &self.limits)?;
            let tile = self.prepare_for(bytes)?;
            // The existing back-links can be cleaned up after the swap: they
            // identify their owning ground polygons and their old target refs.
            self.replace_managed_tile(tile)?;
        }
        Ok(())
    }

    fn landing(&self, point: [f32; 3], radius: f32, ground: &[usize]) -> (Landing, [f32; 3]) {
        let (x, y) = self.calc_tile_loc(&point);
        let missing = if self.is_column_unloaded(x, y) {
            Landing::Unloaded(TileKey { x, y })
        } else {
            Landing::NoPolygonInRadius
        };
        // Ground is sorted by descending key: the greatest eligible key wins,
        // matching the ordering of the global baked-link endpoint rule.
        for &ti in ground {
            let t = self.tile_unchecked(ti as u32);
            if (0..3).any(|a| {
                point[a] + radius < t.header.bmin[a] || point[a] - radius > t.header.bmax[a]
            }) {
                continue;
            }
            if let Some((ip, nearest)) = find_runtime_landing(t, point, radius) {
                return (
                    Landing::Connected(self.base_ref(ti) | ip as PolyRef),
                    nearest,
                );
            }
        }
        (missing, point)
    }

    fn ground_in_landing_order(&self) -> Vec<usize> {
        let mut ground = self.ground_slots();
        ground.sort_unstable_by_key(|&i| {
            let h = &self.tile_unchecked(i as u32).header;
            core::cmp::Reverse((h.x, h.y, h.layer))
        });
        ground
    }

    fn resolve_landings(&self, spec: &LinkSpec, ground: &[usize]) -> [(Landing, [f32; 3]); 2] {
        [spec.start, spec.end].map(|point| self.landing(point, spec.radius, ground))
    }

    pub(super) fn runtime_ground_changed(&mut self) {
        if self.runtime_links.entries.is_empty() {
            return;
        }
        let ground = self.ground_in_landing_order();
        let mut entries = core::mem::take(&mut self.runtime_links.entries);
        for e in entries.values_mut() {
            e.landings = self.resolve_landings(&e.spec, &ground);
        }
        self.runtime_links.entries = entries;
        self.refresh_runtime_links();
    }

    fn resolve_runtime_links(&self, entries: &BTreeMap<LinkId, Entry>) -> Vec<(LinkId, Resolved)> {
        let mut used = BTreeMap::<u32, u32>::new();
        let mut result = Vec::with_capacity(entries.len());
        for (&id, e) in entries {
            let [(mut start, p0), (mut end, p1)] = e.landings;
            if let (Landing::Connected(a), Landing::Connected(b)) = (start, end) {
                let ai = self.decode_poly_id_tile(a);
                let bi = self.decode_poly_id_tile(b);
                let mut check = |i| {
                    let tile = self.tile_unchecked(i);
                    let n = used.entry(i).or_default();
                    if *n == tile.spare_links {
                        let h = &tile.header;
                        Some(Landing::NoLinkCapacity {
                            key: TileKey { x: h.x, y: h.y },
                            layer: h.layer,
                        })
                    } else {
                        *n += 1;
                        None
                    }
                };
                let fail_a = check(ai);
                let fail_b = if e.spec.bidirectional {
                    check(bi)
                } else {
                    None
                };
                if fail_a.is_some() || fail_b.is_some() {
                    if let Some(fail) = fail_a {
                        start = fail;
                    } else if let Some(n) = used.get_mut(&ai) {
                        *n -= 1;
                    }
                    if let Some(fail) = fail_b {
                        end = fail;
                    } else if e.spec.bidirectional
                        && let Some(n) = used.get_mut(&bi)
                    {
                        *n -= 1;
                    }
                }
            }
            result.push((
                id,
                Resolved {
                    report: LinkReport {
                        id,
                        enabled: e.enabled,
                        start,
                        end,
                    },
                    points: [p0, p1],
                },
            ));
        }
        result
    }

    pub(super) fn clear_runtime_edges(&mut self) {
        for (land, link) in core::mem::take(&mut self.runtime_links.back_links) {
            let (_, ti, ip) = self.decode_poly_id(land);
            let t = self.tile_mut(ti as usize);
            remove_links_where(t, Some(ip as usize), |r| r == link);
        }
    }

    fn refresh_runtime_links(&mut self) {
        let Some(layer) = self.link_layer() else {
            return;
        };
        if self.runtime_links.entries.is_empty() {
            return;
        }
        self.clear_runtime_edges();
        let resolved = self.resolve_runtime_links(&self.runtime_links.entries);
        let mut owners = BTreeMap::<TileKey, (usize, usize)>::new();
        let mut changed = false;
        for (id, Resolved { report, points }) in resolved {
            let Some(e) = self.runtime_links.entries.get_mut(&id) else {
                continue;
            };
            changed |= e.report != report && (e.report.is_traversable() || report.is_traversable());
            e.report = report;
            let e = *e;
            let key = e.key;
            let (ti, ip) = match owners.entry(key) {
                alloc::collections::btree_map::Entry::Occupied(o) => o.into_mut(),
                alloc::collections::btree_map::Entry::Vacant(v) => {
                    let Some(ti) = self.tile_index_at(key.x, key.y, layer) else {
                        continue;
                    };
                    let t = self.tile_mut(ti);
                    t.links_free_list = t.reset_links();
                    for p in &mut t.polys {
                        p.first_link = NULL_LINK;
                    }
                    v.insert((ti, 0))
                }
            };
            let (ti, ip) = (*ti, {
                let next = *ip;
                *ip += 1;
                next
            });
            let owner = self.base_ref(ti) | ip as PolyRef;
            let t = self.tile_mut(ti);
            for (j, p) in points.iter().enumerate() {
                let v = t.polys[ip].verts[j] as usize * 3;
                t.verts[v..v + 3].copy_from_slice(p);
            }
            t.polys[ip].flags = if report.is_traversable() {
                e.spec.flags
            } else {
                0
            };
            if report.is_traversable()
                && let (Landing::Connected(a), Landing::Connected(b)) = (report.start, report.end)
            {
                push_edge(t, ip, a, 0, false);
                push_edge(t, ip, b, 1, false);
                self.push_runtime_back_link(a, owner);
                if e.spec.bidirectional {
                    self.push_runtime_back_link(b, owner);
                }
            }
        }
        if changed {
            self.runtime_links.changed_at = self.epoch();
        }
    }

    fn push_runtime_back_link(&mut self, ground: PolyRef, owner: PolyRef) {
        let (_, ti, ip) = self.decode_poly_id(ground);
        push_edge(self.tile_mut(ti as usize), ip as usize, owner, 0xff, true);
        self.runtime_links.back_links.push((ground, owner));
    }

    /// Writes the runtime layer and its definitions and availability as a versioned
    /// byte record. Ground tile bytes and residency are saved independently.
    pub fn runtime_links_to_bytes(&self) -> Vec<u8> {
        let mut bytes = Vec::new();
        bytes.extend_from_slice(b"DNLK");
        bytes.extend_from_slice(&SNAPSHOT_VERSION.to_le_bytes());
        bytes.extend_from_slice(&self.link_layer().unwrap_or(-1).to_le_bytes());
        bytes.extend_from_slice(&(self.runtime_links.entries.len() as u64).to_le_bytes());
        for e in self.runtime_links.entries.values() {
            let s = e.spec;
            bytes.extend_from_slice(&s.id.0.to_le_bytes());
            for v in s.start.into_iter().chain(s.end).chain([s.radius]) {
                bytes.extend_from_slice(&v.to_le_bytes());
            }
            bytes.extend_from_slice(&s.flags.to_le_bytes());
            bytes.push(s.area);
            bytes.push(u8::from(s.bidirectional) | (u8::from(e.enabled) << 1));
        }
        let digest = content_hash(&bytes);
        bytes.extend_from_slice(&digest);
        bytes
    }

    /// Restores a runtime layer into a mesh without runtime links. If restoring
    /// residency, restore it first so regenerated tiles can recover their refs.
    /// Ground tiles may be committed before or after this operation. Capacity
    /// conflicts are reported as landings while the definitions remain available.
    pub fn restore_runtime_links(&mut self, bytes: &[u8]) -> Result<(), Error> {
        let invalid = |s| Error::InvalidSnapshot(SnapshotError::InvalidValue(s));
        let read = |slice: &[u8], i: usize| {
            slice
                .get(i..i + 4)
                .and_then(|b| b.try_into().ok())
                .map(u32::from_le_bytes)
                .ok_or(Error::InvalidSnapshot(SnapshotError::TooShort))
        };
        if bytes
            .get(..4)
            .ok_or(Error::InvalidSnapshot(SnapshotError::TooShort))?
            != b"DNLK"
        {
            return Err(Error::InvalidSnapshot(SnapshotError::WrongMagic));
        }
        if read(bytes, 4)? != SNAPSHOT_VERSION {
            return Err(Error::InvalidSnapshot(SnapshotError::WrongVersion));
        }
        let layer = read(bytes, 8)? as i32;
        let count = u64::from(read(bytes, 12)?) | (u64::from(read(bytes, 16)?) << 32);
        let content_len = usize::try_from(count)
            .ok()
            .and_then(|n| n.checked_mul(RECORD_SIZE))
            .and_then(|n| n.checked_add(20))
            .ok_or(Error::SizeOverflow)?;
        let (content, digest) = bytes
            .split_at_checked(content_len)
            .ok_or(Error::InvalidSnapshot(SnapshotError::TooShort))?;
        if digest.len() < 32 {
            return Err(Error::InvalidSnapshot(SnapshotError::TooShort));
        }
        if digest.len() > 32 {
            return Err(Error::InvalidSnapshot(SnapshotError::TrailingBytes));
        }
        if layer < -1 || (layer == -1 && count != 0) {
            return Err(invalid("runtime link record layer"));
        }
        let records = &content[20..];
        if !self.runtime_links.entries.is_empty() {
            return Err(Error::InvalidParameter("runtime links already installed"));
        }
        let mut used = (records.len() / RECORD_SIZE)
            .checked_mul(core::mem::size_of::<Entry>() + 192)
            .ok_or(Error::SizeOverflow)?;
        if used > self.limits.max_bytes {
            return Err(Error::LimitExceeded);
        }
        if content_hash(content) != digest {
            return Err(invalid("runtime link record digest"));
        }
        let layer_opt = (layer >= 0).then_some(layer);
        self.validate_link_layer(layer_opt)?;
        let mut entries = BTreeMap::new();
        let mut columns = BTreeMap::<TileKey, Vec<LinkSpec>>::new();
        for r in records.chunks_exact(RECORD_SIZE) {
            let f = |i| read(r, i).map(f32::from_bits);
            let mut s = LinkSpec::new(
                LinkId(read(r, 0)?),
                [f(4)?, f(8)?, f(12)?],
                [f(16)?, f(20)?, f(24)?],
                f(28)?,
            );
            s.flags = u16::from_le_bytes([r[32], r[33]]);
            s.area = r[34];
            if r[35] > 3 {
                return Err(invalid("runtime link flags"));
            }
            s.bidirectional = r[35] & 1 != 0;
            s.validate()
                .map_err(|_| invalid("runtime link definition"))?;
            let (x, y) = self.calc_tile_loc(&s.start);
            let key = TileKey { x, y };
            if entries.last_key_value().is_some_and(|(&id, _)| id >= s.id) {
                return Err(invalid("runtime link id order"));
            }
            entries.insert(s.id, Entry::new(key, s, r[35] & 2 != 0));
            columns.entry(key).or_default().push(s);
        }
        // Plan the entire install before linking anything. Sequential slots
        // are consumed in free-list order, while parked slots keep their refs.
        let mut next_free = self.next_free;
        let mut parked = 0;
        let mut plans = Vec::new();
        plans
            .try_reserve_exact(columns.len())
            .map_err(|_| Error::AllocationFailed)?;
        for (key, specs) in columns {
            let bytes = super::super::builder::encode_link_tile(key, layer, &specs, &self.limits)?;
            used = used
                .checked_add(bytes.len().saturating_mul(2))
                .ok_or(Error::SizeOverflow)?;
            if used > self.limits.max_bytes {
                return Err(Error::LimitExceeded);
            }
            let tile = self.prepare_for(bytes)?;
            let mut target = self.choose_slot(key.x, key.y, layer, Placement::Free)?;
            if let Some(hash) = target.parked_hash {
                if hash != tile.hash {
                    return Err(invalid("runtime link residency content"));
                }
                parked += 1;
            } else if target.prev.is_some() {
                if next_free < 0 {
                    return Err(Error::NoFreeTile);
                }
                let slot = &self.slots[next_free as usize];
                target.ti = next_free as usize;
                target.salt = slot.salt;
                next_free = slot.next;
            }
            plans.push((target, tile));
        }
        if self
            .slots
            .indices()
            .filter(|&i| {
                self.slots[i]
                    .parked
                    .is_some_and(|p| Some(p.layer) == layer_opt)
            })
            .count()
            != parked
        {
            return Err(invalid("runtime link residency columns"));
        }
        for (target, _) in &plans {
            self.slots.ensure(target.ti)?;
        }
        self.runtime_links.layer = layer_opt;
        for (target, tile) in plans {
            self.attach(tile.tile, target, target.salt, Some(tile.hash), true);
        }
        self.runtime_links.entries = entries;
        self.runtime_ground_changed();
        Ok(())
    }
}

// Use the streaming polygon visitor so a dense tile never drops candidates
// due to the legacy nearest-poly scratch cap. Ties keep the first polygon.
fn find_runtime_landing(t: &MeshTile, point: [f32; 3], radius: f32) -> Option<(u32, [f32; 3])> {
    let min = point.map(|v| v - radius);
    let max = point.map(|v| v + radius);
    let mut hit = None;
    let mut best = f64::INFINITY;
    query_polygons_in_tile_with(t, &min, &max, &mut |ip| {
        let Some((snap, d)) = link_landing_candidate(t, ip as usize, point) else {
            return true;
        };
        if d <= f64::from(radius) * f64::from(radius) && d < best {
            best = d;
            hit = Some((ip, snap));
        }
        true
    });
    hit
}

fn push_edge(tile: &mut MeshTile, ip: usize, target: PolyRef, edge: u8, reserved: bool) {
    let free = if reserved {
        &mut tile.runtime_links_free_list
    } else {
        &mut tile.links_free_list
    };
    let i = *free;
    // The planner reserves all required ground links before install, and each
    // owner polygon has exactly two ordinary slots.
    let l = &mut tile.links[i as usize];
    *free = l.next;
    *l = Link {
        r#ref: target,
        next: tile.polys[ip].first_link,
        edge,
        side: 0xff,
        bmin: 0,
        bmax: 0,
    };
    tile.polys[ip].first_link = i;
}
