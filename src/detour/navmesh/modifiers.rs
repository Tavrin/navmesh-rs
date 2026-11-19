//! Resident selection and application of the mesh's modifier layer.

use alloc::collections::{BTreeMap, BTreeSet};
use alloc::vec::Vec;

use super::{MeshTile, NavMesh};
use crate::detour::{PolyRef, QueryFilter, TileRef};
use crate::dynamic::{
    DynamicLayer, Effect, ModifierId, ModifierSpec, SelectionReport, Selector, side,
};
use crate::{Budget, Budgeted, Error};

type Location = (i32, i32, i32, usize);
type Polygons = BTreeMap<usize, Vec<ModifierId>>;

#[derive(Clone, Debug, Default)]
pub(super) struct Modifiers {
    layer: Option<DynamicLayer>,
    by_modifier: BTreeMap<ModifierId, BTreeMap<Location, Vec<usize>>>,
    by_tile: BTreeMap<usize, Polygons>,
    epoch: u64,
}

fn location(tile: &MeshTile, slot: usize) -> Location {
    (tile.header.x, tile.header.y, tile.header.layer, slot)
}

type Bounds = ([f32; 3], [f32; 3]);

fn extend(bounds: &mut Bounds, v: [f32; 3]) {
    for (i, value) in v.into_iter().enumerate() {
        bounds.0[i] = bounds.0[i].min(value);
        bounds.1[i] = bounds.1[i].max(value);
    }
}

fn empty_bounds() -> Bounds {
    ([f32::INFINITY; 3], [f32::NEG_INFINITY; 3])
}

fn tile_bounds(tile: &MeshTile) -> Bounds {
    let mut bounds = empty_bounds();
    for i in 0..tile.header.vert_count as usize {
        extend(&mut bounds, tile.baked_vertex(i));
    }
    bounds
}

struct Prism<'a> {
    vertices: &'a [[f32; 2]],
    winding: f64,
    bounds: Bounds,
    area: Option<u8>,
}

impl<'a> Prism<'a> {
    fn new(vertices: &'a [[f32; 2]], ymin: f32, ymax: f32, area: Option<u8>) -> Self {
        let mut bounds = empty_bounds();
        for &[x, z] in vertices {
            extend(&mut bounds, [x, ymin, z]);
        }
        bounds.1[1] = ymax;
        let winding = side(vertices[0], vertices[1], vertices[2].map(f64::from), 1.0).signum();
        Self {
            vertices,
            winding,
            bounds,
            area,
        }
    }

    fn edges(&self) -> impl Iterator<Item = ([f32; 2], [f32; 2])> + '_ {
        let v = self.vertices.iter().copied();
        v.clone().zip(v.cycle().skip(1)).take(self.vertices.len())
    }

    fn bounds_overlap(&self, (lo, hi): Bounds) -> bool {
        (0..3).all(|i| lo[i] <= self.bounds.1[i] && hi[i] >= self.bounds.0[i])
    }

    fn contains_centroid(&self, [x, y, z]: [f64; 3], count: f64) -> bool {
        y >= f64::from(self.bounds.0[1]) * count
            && y <= f64::from(self.bounds.1[1]) * count
            && self
                .edges()
                .all(|(a, b)| side(a, b, [x, z], count) * self.winding >= 0.0)
    }

    // Separating axes for a rectangle and a convex polygon: first X and Z,
    // then the prism's edge normals.
    fn overlaps_box(&self, bounds @ ([x0, _, z0], [x1, _, z1]): Bounds) -> bool {
        self.bounds_overlap(bounds)
            && self.edges().all(|(a, b)| {
                [[x0, z0], [x0, z1], [x1, z0], [x1, z1]]
                    .into_iter()
                    .any(|corner| side(a, b, corner.map(f64::from), 1.0) * self.winding >= 0.0)
            })
    }

    fn ground_candidates<'t>(
        &'t self,
        tile: &'t MeshTile,
    ) -> impl Iterator<Item = (usize, &'t crate::detour::Poly)> {
        tile.polys.iter().enumerate().filter(|(ip, poly)| {
            poly.poly_type() != crate::detour::POLYTYPE_OFFMESH_CONNECTION
                && self.area.is_none_or(|area| tile.baked_state(*ip).1 == area)
        })
    }
}

fn vertices<'t>(
    tile: &'t MeshTile,
    poly: &'t crate::detour::Poly,
) -> impl Iterator<Item = [f32; 3]> + 't {
    poly.verts[..usize::from(poly.vert_count)]
        .iter()
        .map(|&v| tile.baked_vertex(usize::from(v)))
}

fn selected(tile: &MeshTile, bounds: Bounds, selector: &Selector) -> Vec<usize> {
    match selector {
        Selector::Prism {
            verts_xz,
            ymin,
            ymax,
            area,
        } => {
            let prism = Prism::new(verts_xz, *ymin, *ymax, *area);
            if !prism.bounds_overlap(bounds) {
                return Vec::new();
            }
            prism
                .ground_candidates(tile)
                .filter_map(|(ip, poly)| {
                    let mut sum = [0.0; 3];
                    for v in vertices(tile, poly) {
                        for (sum, component) in sum.iter_mut().zip(v) {
                            *sum += f64::from(component);
                        }
                    }
                    prism
                        .contains_centroid(sum, f64::from(poly.vert_count))
                        .then_some(ip)
                })
                .collect()
        }
        Selector::Links {
            user_id_min,
            user_id_max,
        } => tile
            .off_mesh_cons
            .iter()
            .filter(|link| (*user_id_min..=*user_id_max).contains(&link.user_id))
            .map(|link| usize::from(link.poly))
            .collect(),
    }
}

impl Modifiers {
    fn index_tile(&mut self, tile: &MeshTile, loc: Location) {
        let Some(layer) = &self.layer else { return };
        let mut specs = layer.specs().peekable();
        if specs.peek().is_none() {
            return;
        }
        let bounds = tile_bounds(tile);
        for spec in specs {
            let polys = selected(tile, bounds, &spec.selector);
            if !polys.is_empty() {
                Self::index(
                    &mut self.by_tile,
                    &mut self.by_modifier,
                    spec.id,
                    loc,
                    polys,
                );
            }
        }
    }

    fn index(
        by_tile: &mut BTreeMap<usize, Polygons>,
        by_modifier: &mut BTreeMap<ModifierId, BTreeMap<Location, Vec<usize>>>,
        id: ModifierId,
        loc: Location,
        mut polys: Vec<usize>,
    ) {
        polys.sort_unstable();
        polys.dedup();
        let tile = by_tile.entry(loc.3).or_default();
        for &ip in &polys {
            let ids = tile.entry(ip).or_default();
            let at = ids.partition_point(|&other| other < id);
            ids.insert(at, id);
        }
        by_modifier.entry(id).or_default().insert(loc, polys);
    }

    pub(super) fn detach_tile(&mut self, tile: &MeshTile, slot: usize) {
        let Some(polys) = self.by_tile.remove(&slot) else {
            return;
        };
        let loc = location(tile, slot);
        for id in polys.into_values().flatten() {
            if let Some(tiles) = self.by_modifier.get_mut(&id) {
                tiles.remove(&loc);
                if tiles.is_empty() {
                    self.by_modifier.remove(&id);
                }
            }
        }
    }

    fn unindex_modifier(&mut self, id: ModifierId) -> BTreeMap<Location, Vec<usize>> {
        let tiles = self.by_modifier.remove(&id).unwrap_or_default();
        for (&(_, _, _, ti), polys) in &tiles {
            if let Some(tile) = self.by_tile.get_mut(&ti) {
                for ip in polys {
                    if let Some(ids) = tile.get_mut(ip) {
                        ids.retain(|&other| other != id);
                        if ids.is_empty() {
                            tile.remove(ip);
                        }
                    }
                }
                if tile.is_empty() {
                    self.by_tile.remove(&ti);
                }
            }
        }
        tiles
    }

    fn poly_ids(&self, slot: usize, ip: usize) -> &[ModifierId] {
        self.by_tile
            .get(&slot)
            .and_then(|tile| tile.get(&ip))
            .map_or(&[], Vec::as_slice)
    }

    pub(super) fn governs(&self, slot: usize, ip: usize) -> bool {
        !self.poly_ids(slot, ip).is_empty()
    }

    fn apply(
        &self,
        tile: &mut MeshTile,
        slot: usize,
        polys: impl IntoIterator<Item = usize>,
    ) -> bool {
        let mut changed = false;
        for ip in polys {
            let (mut flags, mut area) = tile.baked_state(ip);
            for id in self.poly_ids(slot, ip) {
                if let Some((spec, true)) = self.layer.as_ref().and_then(|layer| layer.get(*id)) {
                    match spec.effect {
                        Effect::ClearFlags(mask) => flags &= !mask,
                        Effect::SetFlags(mask) => flags |= mask,
                        Effect::SetArea(value) => area = value,
                    }
                }
            }
            let poly = &mut tile.polys[ip];
            changed |= poly.flags != flags || poly.area() != area;
            poly.flags = flags;
            poly.set_area(area);
        }
        changed
    }

    pub(super) fn attach_tile(&mut self, tile: &mut MeshTile, slot: usize) -> u64 {
        self.index_tile(tile, location(tile, slot));
        if self.apply_tile(tile, slot) {
            self.advance()
        } else {
            0
        }
    }

    pub(super) fn apply_tile(&self, tile: &mut MeshTile, slot: usize) -> bool {
        self.by_tile
            .get(&slot)
            .is_some_and(|polys| self.apply(tile, slot, polys.keys().copied()))
    }

    fn advance(&mut self) -> u64 {
        self.epoch = self.epoch.saturating_add(1);
        self.epoch
    }
}

impl NavMesh {
    /// The world's registered specifications and enabled bits, for saving.
    pub fn dynamic_layer(&self) -> Option<&DynamicLayer> {
        self.modifiers.layer.as_ref()
    }

    /// Installs the modifier layer and applies it to every resident tile.
    /// Passing `None` unregisters the layer and restores formerly governed
    /// polygons to their baked state.
    pub fn set_dynamic_layer(&mut self, layer: Option<DynamicLayer>) {
        let old = core::mem::take(&mut self.modifiers.by_tile);
        self.modifiers.layer = layer;
        self.modifiers.by_modifier.clear();
        let mut dirty = BTreeMap::new();
        for ti in self.slots.indices() {
            if let Some(tile) = self.slots[ti].tile.as_deref() {
                let loc = location(tile, ti);
                self.modifiers.index_tile(tile, loc);
                if old.contains_key(&ti) || self.modifiers.by_tile.contains_key(&ti) {
                    dirty.insert(loc, ());
                }
            }
        }

        let stamp = self.modifiers.advance();
        for (_, _, _, ti) in dirty.into_keys() {
            let previous = old.get(&ti).into_iter().flat_map(|p| p.keys().copied());
            let current = self
                .modifiers
                .by_tile
                .get(&ti)
                .into_iter()
                .flat_map(|p| p.keys().copied());
            let added =
                current.filter(|ip| old.get(&ti).is_none_or(|polys| !polys.contains_key(ip)));
            let slot = &mut self.slots[ti];
            if let Some(tile) = slot.tile.as_deref_mut()
                && self.modifiers.apply(tile, ti, previous.chain(added))
            {
                slot.state_stamp = stamp;
            }
        }
    }

    /// Registers or replaces a spec and applies the result immediately. Install
    /// a layer with [`NavMesh::set_dynamic_layer`] before calling this method.
    pub fn upsert_modifier(&mut self, spec: ModifierSpec) -> Result<(), Error> {
        let id = spec.id;
        let layer = self
            .modifiers
            .layer
            .as_mut()
            .ok_or(Error::InvalidParameter("no modifier layer"))?;
        layer.upsert(spec)?;
        let mut dirty = self.modifiers.unindex_modifier(id);
        if let Some((spec, _)) = self
            .modifiers
            .layer
            .as_ref()
            .and_then(|layer| layer.get(id))
        {
            for ti in self.slots.indices() {
                if let Some(tile) = self.slots[ti].tile.as_deref() {
                    let polys = selected(tile, tile_bounds(tile), &spec.selector);
                    if !polys.is_empty() {
                        let loc = location(tile, ti);
                        dirty.entry(loc).or_default().extend(&polys);
                        Modifiers::index(
                            &mut self.modifiers.by_tile,
                            &mut self.modifiers.by_modifier,
                            id,
                            loc,
                            polys,
                        );
                    }
                }
            }
        }
        self.apply_modifier_edit(dirty);
        Ok(())
    }

    /// Unregisters a spec, restores its baked contribution and reapplies any
    /// remaining overlapping modifiers. Returns whether it was registered.
    pub fn remove_modifier(&mut self, id: ModifierId) -> bool {
        if !self
            .modifiers
            .layer
            .as_mut()
            .is_some_and(|layer| layer.remove(id))
        {
            return false;
        }
        let dirty = self.modifiers.unindex_modifier(id);
        self.apply_modifier_edit(dirty);
        true
    }

    fn apply_modifier_edit(&mut self, dirty: BTreeMap<Location, Vec<usize>>) {
        let stamp = self.modifiers.advance();
        for ((_, _, _, ti), polys) in dirty {
            let slot = &mut self.slots[ti];
            if let Some(tile) = slot.tile.as_deref_mut()
                && self.modifiers.apply(tile, ti, polys)
            {
                slot.state_stamp = stamp;
            }
        }
    }

    /// Enables or disables a modifier immediately. Returns whether its bit
    /// changed. The operation uses the resident selection index and keeps each
    /// tile's ref, content hash, and structural epoch.
    pub fn set_modifier_enabled(&mut self, id: ModifierId, enabled: bool) -> Result<bool, Error> {
        let layer = self
            .modifiers
            .layer
            .as_mut()
            .ok_or(Error::InvalidParameter("no modifier layer"))?;
        if !layer.set_enabled(id, enabled)? {
            return Ok(false);
        }
        let stamp = self.modifiers.advance();
        for ((_, _, _, ti), polys) in self.modifiers.by_modifier.get(&id).into_iter().flatten() {
            let slot = &mut self.slots[*ti];
            if let Some(tile) = slot.tile.as_deref_mut()
                && self.modifiers.apply(tile, *ti, polys.iter().copied())
            {
                slot.state_stamp = stamp;
            }
        }
        Ok(true)
    }

    /// Applies a modifier toggle only when the budget covers one work unit per
    /// selected resident polygon. On exhaustion the enabled bit and all polygon
    /// state stay as before. The result reports whether the enabled bit changed.
    pub fn set_modifier_enabled_budgeted(
        &mut self,
        id: ModifierId,
        enabled: bool,
        budget: &mut Budget,
    ) -> Result<Budgeted<bool>, Error> {
        let (_, current) = self
            .dynamic_layer()
            .and_then(|layer| layer.get(id))
            .ok_or(Error::InvalidParameter("unknown modifier ID"))?;
        if current == enabled {
            return Ok(Budgeted::Complete(false));
        }
        let cost = self.modifier_polygon_count(id).unwrap_or_default() as u64;
        if !budget.spend(cost) {
            return Ok(Budgeted::Exhausted(false));
        }
        self.set_modifier_enabled(id, enabled)
            .map(Budgeted::Complete)
    }

    /// Counts the resident polygons selected by a modifier, including when
    /// disabled. Returns `None` for an unregistered ID.
    pub fn modifier_polygon_count(&self, id: ModifierId) -> Option<usize> {
        self.dynamic_layer()?.get(id)?;
        Some(
            self.modifiers
                .by_modifier
                .get(&id)
                .into_iter()
                .flatten()
                .map(|(_, polys)| polys.len())
                .sum(),
        )
    }

    /// Checks a modifier's resident selection. For a prism, also scans ground
    /// polygons of the requested baked area for bounds that intersect the prism
    /// but whose centroid falls outside it. Returns `None` for an unknown ID.
    pub fn modifier_selection(&self, id: ModifierId) -> Option<SelectionReport> {
        let (spec, _) = self.dynamic_layer()?.get(id)?;
        let mut report = SelectionReport {
            selected_polygons: self.modifier_polygon_count(id)?,
            bounds_without_centroid: 0,
        };
        let Selector::Prism {
            verts_xz,
            ymin,
            ymax,
            area,
        } = &spec.selector
        else {
            return Some(report);
        };
        let prism = Prism::new(verts_xz, *ymin, *ymax, *area);
        for ti in self.slots.indices() {
            let Some(tile) = self.slots[ti].tile.as_deref() else {
                continue;
            };
            if !prism.bounds_overlap(tile_bounds(tile)) {
                continue;
            }
            for (ip, poly) in prism.ground_candidates(tile) {
                if !self.modifiers.poly_ids(ti, ip).contains(&id) {
                    let mut bounds = empty_bounds();
                    for v in vertices(tile, poly) {
                        extend(&mut bounds, v);
                    }
                    if prism.overlaps_box(bounds) {
                        report.bounds_without_centroid += 1;
                    }
                }
            }
        }
        Some(report)
    }

    /// Latest flag, area or modifier state revision. Independent of
    /// [`NavMesh::epoch`], which tracks residency and structural changes.
    pub fn state_epoch(&self) -> u64 {
        self.modifiers.epoch
    }

    /// Last revision that changed a resident tile's flags or areas, or zero
    /// if none. Returns `None` for an invalid or unloaded ref.
    pub fn state_stamp(&self, tile: TileRef) -> Option<u64> {
        let ti = self.resident_slot(tile).ok()?;
        Some(self.slots[ti].state_stamp)
    }

    /// Returns the baked flags and area from the tile's content bytes.
    pub fn baked_poly_state(&self, r: PolyRef) -> Result<(u16, u8), Error> {
        let (tile, _) = self.tile_and_poly_by_ref(r)?;
        let (_, _, ip) = self.decode_poly_id(r);
        Ok(tile.baked_state(ip as usize))
    }

    pub(super) fn check_unmanaged(&self, ti: usize, ip: usize) -> Result<(), Error> {
        if !self.modifiers.governs(ti, ip) {
            Ok(())
        } else {
            Err(Error::InvalidParameter(
                "polygon state is governed by modifiers",
            ))
        }
    }

    pub(super) fn stamp_state(&mut self, ti: usize) {
        self.slots[ti].state_stamp = self.modifiers.advance();
    }

    /// Floods the resident directed links using current polygon flags and areas.
    ///
    /// Returns a sorted set of reachable refs on completion and no partial
    /// result on budget exhaustion. An excluded start yields an empty set.
    /// Charges one unit per expanded polygon and per examined link.
    pub fn reachable_filtered<F: QueryFilter + ?Sized>(
        &self,
        start: PolyRef,
        filter: &F,
        budget: &mut Budget,
    ) -> Result<Budgeted<Option<BTreeSet<PolyRef>>>, Error> {
        let (tile, poly) = self.tile_and_poly_by_ref(start)?;
        let mut seen = BTreeSet::new();
        if !budget.spend(1) {
            return Ok(Budgeted::Exhausted(None));
        }
        if !filter.pass_filter(start, tile, poly) {
            return Ok(Budgeted::Complete(Some(seen)));
        }
        let mut pending = alloc::vec![start];
        seen.insert(start);
        while let Some(r) = pending.pop() {
            if !budget.spend(1) {
                return Ok(Budgeted::Exhausted(None));
            }
            let (tile, poly) = self.tile_and_poly_by_ref(r)?;
            let mut next = poly.first_link;
            while let Some(link) = tile.links.get(next as usize) {
                if !budget.spend(1) {
                    return Ok(Budgeted::Exhausted(None));
                }
                let to = link.r#ref;
                if !seen.contains(&to)
                    && let Ok((tile, poly)) = self.tile_and_poly_by_ref(to)
                    && filter.pass_filter(to, tile, poly)
                {
                    seen.insert(to);
                    pending.push(to);
                }
                next = link.next;
            }
        }
        Ok(Budgeted::Complete(Some(seen)))
    }
}
