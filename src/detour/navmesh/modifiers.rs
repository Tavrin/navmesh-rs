//! Resident selection and application of the mesh's modifier layer.

use alloc::collections::{BTreeMap, BTreeSet};
use alloc::vec::Vec;

use super::{MeshTile, NULL_LINK, NavMesh};
use crate::detour::{PolyRef, QueryFilter, TileRef};
use crate::dynamic::{
    BaseState, BaseTileKey, DynamicLayer, Effect, ModifierId, ModifierSpec, PolyBaseEdit,
    SelectionReport, Selector, side,
};
use crate::{Budget, Budgeted, Error};

mod detail;

// DSTM writes zero padding. Streaming state can use that byte to identify
// caller values that were not derived from an installed modifier layer.
pub(super) const DIRECT_BASE_PAD: u8 = 1;
// A polygon the layer managed when its state was stored: the stored flags and
// area are effective values, not caller values. A restore ignores them and
// derives the polygon from the layer it then has, over the saved base.
pub(super) const MANAGED_PAD: u8 = 2;

type Location = (i32, i32, i32, usize);
type Polygons = BTreeMap<usize, Vec<ModifierId>>;
pub(super) type ReloadBases = BTreeMap<usize, BaseState>;

#[derive(Clone, Debug, Default)]
pub(super) struct Modifiers {
    layer: Option<DynamicLayer>,
    by_modifier: BTreeMap<ModifierId, BTreeMap<Location, Vec<usize>>>,
    by_tile: BTreeMap<usize, Polygons>,
    base_tiles: BTreeMap<usize, BaseTileKey>,
    epoch: u64,
}

fn location(tile: &MeshTile, slot: usize) -> Location {
    (tile.header.x, tile.header.y, tile.header.layer, slot)
}

fn base_key(tile: &MeshTile, hash: Option<[u8; 32]>) -> BaseTileKey {
    (
        tile.header.x,
        tile.header.y,
        tile.header.layer,
        hash.unwrap_or_else(|| super::stream::content_hash(&tile.data)),
    )
}

fn direct_base(tile: &MeshTile, ip: usize) -> BaseState {
    let poly = &tile.polys[ip];
    let (baked_flags, baked_area) = tile.baked_state(ip);
    BaseState {
        flags: (poly.flags != baked_flags
            && !(tile.managed_link_tile && poly.first_link == NULL_LINK))
            .then_some(poly.flags),
        area: (poly.area() != baked_area).then_some(poly.area()),
    }
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

fn with_detail_bounds(tile: &MeshTile, mut bounds: Bounds) -> Bounds {
    for v in tile.detail_verts.chunks_exact(3) {
        extend(&mut bounds, [v[0], v[1], v[2]]);
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
        Selector::DetailPrism {
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
                .filter_map(|(ip, poly)| detail::overlaps(&prism, tile, ip, poly).then_some(ip))
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
        let detail_bounds = if layer.uses_detail() {
            with_detail_bounds(tile, bounds)
        } else {
            bounds
        };
        for spec in specs {
            let bounds = if spec.selector.is_detail() {
                detail_bounds
            } else {
                bounds
            };
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
        self.base_tiles.remove(&slot);
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

    pub(super) fn prune_bases(&mut self, key: BaseTileKey) {
        if let Some(layer) = &mut self.layer {
            layer.prune_tile_bases(key);
        }
    }

    pub(super) fn prune_resident_bases(&mut self, tile: &MeshTile, slot: usize) {
        if let Some(key) = self.base_tiles.get(&slot).copied() {
            self.prune_bases(key);
        } else if self.layer.as_ref().is_some_and(|layer| {
            let (x, y, l, _) = location(tile, slot);
            layer.has_bases_at((x, y, l))
        }) {
            self.prune_bases(base_key(tile, None));
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

    fn base(&self, slot: usize, ip: usize) -> BaseState {
        self.base_tiles
            .get(&slot)
            .zip(self.layer.as_ref())
            .map_or_else(BaseState::default, |(&key, layer)| layer.base(key, ip))
    }

    /// The padding byte `store_tile_state` writes for polygon `ip` of `slot`.
    pub(super) fn state_pad(&self, slot: usize, ip: usize) -> u8 {
        u8::from(self.manages_state(slot, ip)) * MANAGED_PAD
    }

    pub(super) fn manages_state(&self, slot: usize, ip: usize) -> bool {
        self.governs(slot, ip) || self.base(slot, ip) != BaseState::default()
    }

    fn index_bases(&mut self, tile: &MeshTile, slot: usize, hash: Option<[u8; 32]>) {
        let (x, y, l, _) = location(tile, slot);
        if let Some(layer) = &self.layer
            && layer.has_bases_at((x, y, l))
        {
            let key = base_key(tile, hash);
            if layer
                .base_indices(key)
                .next()
                .is_some_and(|ip| ip < tile.polys.len())
            {
                self.base_tiles.insert(slot, key);
            }
        }
    }

    fn base_indices(&self, slot: usize, polys: usize) -> impl Iterator<Item = usize> + '_ {
        self.base_tiles
            .get(&slot)
            .zip(self.layer.as_ref())
            .into_iter()
            .flat_map(|(&key, layer)| layer.base_indices(key))
            .take_while(move |&ip| ip < polys)
    }

    fn affected(&self, slot: usize, polys: usize) -> impl Iterator<Item = usize> + '_ {
        self.by_tile
            .get(&slot)
            .into_iter()
            .flat_map(|selected| selected.keys().copied())
            .chain(
                self.base_indices(slot, polys)
                    .filter(move |&ip| !self.governs(slot, ip)),
            )
    }

    fn active_effects(&self, slot: usize, ip: usize) -> impl Iterator<Item = Effect> + '_ {
        self.poly_ids(slot, ip).iter().filter_map(|&id| {
            let (spec, enabled) = self.layer.as_ref()?.get(id)?;
            enabled.then_some(spec.effect)
        })
    }

    fn base_state(&self, tile: &MeshTile, slot: usize, ip: usize) -> (u16, u8) {
        let (flags, area) = tile.baked_state(ip);
        let base = self.base(slot, ip);
        (base.flags.unwrap_or(flags), base.area.unwrap_or(area))
    }

    pub(super) fn effective_state(&self, tile: &MeshTile, slot: usize, ip: usize) -> (u16, u8) {
        let (mut flags, mut area) = self.base_state(tile, slot, ip);
        for effect in self.active_effects(slot, ip) {
            match effect {
                Effect::ClearFlags(mask) => flags &= !mask,
                Effect::SetFlags(mask) => flags |= mask,
                Effect::SetArea(value) => area = value,
            }
        }
        (flags, area)
    }

    fn apply(
        &self,
        tile: &mut MeshTile,
        slot: usize,
        polys: impl IntoIterator<Item = usize>,
    ) -> bool {
        let mut changed = false;
        for ip in polys {
            let (mut flags, area) = self.effective_state(tile, slot, ip);
            if tile.managed_link_tile && tile.polys[ip].first_link == NULL_LINK {
                flags = 0;
            }
            let poly = &mut tile.polys[ip];
            changed |= poly.flags != flags || poly.area() != area;
            poly.flags = flags;
            poly.set_area(area);
        }
        changed
    }

    pub(super) fn attach_tile(
        &mut self,
        tile: &mut MeshTile,
        slot: usize,
        hash: Option<[u8; 32]>,
    ) -> u64 {
        self.index_tile(tile, location(tile, slot));
        self.index_bases(tile, slot, hash);
        if self.apply(tile, slot, self.affected(slot, tile.polys.len())) {
            self.advance()
        } else {
            0
        }
    }

    pub(super) fn mark_direct_bases(&self, tile: &MeshTile, slot: usize, state: &mut [u8]) {
        for (ip, entry) in state
            .chunks_exact_mut(super::TILE_POLY_STATE_SIZE)
            .enumerate()
        {
            if !self.manages_state(slot, ip) && direct_base(tile, ip) != BaseState::default() {
                entry[3] = DIRECT_BASE_PAD;
            }
        }
    }

    pub(super) fn prepare_reload_bases(
        &self,
        tile: &MeshTile,
        hash: [u8; 32],
        state: &[u8],
    ) -> Result<ReloadBases, Error> {
        let mut captures = BTreeMap::new();
        let Some(layer) = &self.layer else {
            return Ok(captures);
        };
        let entries = state.chunks_exact(super::TILE_POLY_STATE_SIZE);
        if !entries.clone().any(|entry| entry[3] == DIRECT_BASE_PAD) {
            return Ok(captures);
        }
        let bounds = tile_bounds(tile);
        let detail_bounds = if layer.uses_detail() {
            with_detail_bounds(tile, bounds)
        } else {
            bounds
        };
        let key = base_key(tile, Some(hash));
        let mut new_bases = 0;
        for spec in layer.specs() {
            let bounds = if spec.selector.is_detail() {
                detail_bounds
            } else {
                bounds
            };
            for ip in selected(tile, bounds, &spec.selector) {
                let entry =
                    &state[ip * super::TILE_POLY_STATE_SIZE..][..super::TILE_POLY_STATE_SIZE];
                if entry[3] != DIRECT_BASE_PAD || captures.contains_key(&ip) {
                    continue;
                }
                let (baked_flags, baked_area) = tile.baked_state(ip);
                let (flags, area) = (u16::from_le_bytes([entry[0], entry[1]]), entry[2] & 63);
                let saved = layer.base(key, ip);
                let base = BaseState {
                    flags: saved.flags.or((flags != baked_flags).then_some(flags)),
                    area: saved.area.or((area != baked_area).then_some(area)),
                };
                if base != saved {
                    new_bases += usize::from(saved == BaseState::default());
                    captures.insert(ip, base);
                }
            }
        }
        if new_bases != 0 {
            layer.check_new_bases(new_bases)?;
        }
        Ok(captures)
    }

    pub(super) fn capture_reload_bases(
        &mut self,
        tile: &MeshTile,
        hash: [u8; 32],
        captures: ReloadBases,
    ) {
        if let Some(layer) = &mut self.layer {
            let key = base_key(tile, Some(hash));
            for (ip, base) in captures {
                layer.capture_base(key, ip, base);
            }
        }
    }

    pub(super) fn advance(&mut self) -> u64 {
        self.epoch = self.epoch.saturating_add(1);
        self.epoch
    }
}

impl NavMesh {
    /// The world's specifications, enabled bits and caller base edits, for saving.
    pub fn dynamic_layer(&self) -> Option<&DynamicLayer> {
        self.modifiers.layer.as_ref()
    }

    /// Installs the modifier layer and applies its base edits and effects to
    /// every resident tile. Passing `None` restores affected polygons to baked
    /// state and discards the previous layer, including its base edits.
    ///
    /// Previously unmanaged edits carried into the layer may grow its snapshot
    /// beyond the supplied layer's byte limit. Use
    /// [`NavMesh::try_set_dynamic_layer`] to enforce that limit.
    pub fn set_dynamic_layer(&mut self, layer: Option<DynamicLayer>) {
        let modifiers = self.prepare_dynamic_layer(layer);
        self.install_dynamic_layer(modifiers);
    }

    /// Installs a layer like [`NavMesh::set_dynamic_layer`], returning
    /// [`Error::LimitExceeded`] without changing the mesh if captured edits
    /// would exceed the layer's snapshot byte limit.
    pub fn try_set_dynamic_layer(&mut self, layer: Option<DynamicLayer>) -> Result<(), Error> {
        let modifiers = self.prepare_dynamic_layer(layer);
        if let Some(layer) = &modifiers.layer {
            layer.check_new_bases(0)?;
        }
        self.install_dynamic_layer(modifiers);
        Ok(())
    }

    fn prepare_dynamic_layer(&self, layer: Option<DynamicLayer>) -> Modifiers {
        let mut modifiers = Modifiers {
            layer,
            epoch: self.modifiers.epoch,
            ..Modifiers::default()
        };
        for ti in self.slots.indices() {
            let slot = &self.slots[ti];
            let Some(tile) = slot.tile.as_deref() else {
                continue;
            };
            modifiers.index_tile(tile, location(tile, ti));
            if let Some(polys) = modifiers.by_tile.get(&ti)
                && let Some(layer) = &mut modifiers.layer
            {
                let mut key = None;
                for &ip in polys.keys() {
                    if self.modifiers.manages_state(ti, ip) {
                        continue;
                    }
                    let direct = direct_base(tile, ip);
                    if direct != BaseState::default() {
                        let key = *key.get_or_insert_with(|| base_key(tile, slot.hash));
                        let saved = layer.base(key, ip);
                        layer.capture_base(
                            key,
                            ip,
                            BaseState {
                                flags: saved.flags.or(direct.flags),
                                area: saved.area.or(direct.area),
                            },
                        );
                    }
                }
            }
            modifiers.index_bases(tile, ti, slot.hash);
        }
        modifiers
    }

    fn install_dynamic_layer(&mut self, modifiers: Modifiers) {
        let mut dirty: BTreeMap<Location, BTreeSet<usize>> = BTreeMap::new();
        for ti in self.slots.indices() {
            let Some(tile) = self.tile(ti) else { continue };
            let mut affected = self
                .modifiers
                .affected(ti, tile.polys.len())
                .chain(modifiers.affected(ti, tile.polys.len()))
                .peekable();
            if affected.peek().is_some() {
                dirty
                    .entry(location(tile, ti))
                    .or_default()
                    .extend(affected);
            }
        }
        self.modifiers = modifiers;

        let stamp = self.modifiers.advance();
        let change = self.advance_change_epoch();
        for ((_, _, _, ti), polys) in dirty {
            let slot = &mut self.slots[ti];
            if let Some(tile) = slot.tile.as_deref_mut()
                && self.modifiers.apply(tile, ti, polys)
            {
                slot.state_stamp = stamp;
                slot.change_stamp = change;
            }
        }
    }

    /// Registers or replaces a spec and applies the result immediately. Install
    /// a layer with [`NavMesh::set_dynamic_layer`] before calling this method.
    pub fn upsert_modifier(&mut self, spec: ModifierSpec) -> Result<(), Error> {
        let id = spec.id;
        self.modifiers
            .layer
            .as_ref()
            .ok_or(Error::InvalidParameter("no modifier layer"))?
            .validate_spec(&spec)?;
        let mut selection = Vec::new();
        let mut captures = Vec::new();
        for ti in self.slots.indices() {
            let Some(tile) = self.tile(ti) else { continue };
            let mut bounds = tile_bounds(tile);
            if spec.selector.is_detail() {
                bounds = with_detail_bounds(tile, bounds);
            }
            let polys = selected(tile, bounds, &spec.selector);
            if !polys.is_empty() {
                let mut key = None;
                for &ip in &polys {
                    if !self.modifiers.manages_state(ti, ip) {
                        let base = direct_base(tile, ip);
                        if base != BaseState::default() {
                            let key =
                                *key.get_or_insert_with(|| base_key(tile, self.slots[ti].hash));
                            captures.push((ti, key, ip, base));
                        }
                    }
                }
                selection.push((location(tile, ti), polys));
            }
        }
        let layer = self
            .modifiers
            .layer
            .as_mut()
            .ok_or(Error::InvalidParameter("no modifier layer"))?;
        layer.upsert_with_bases(spec, captures.len())?;
        for (ti, key, ip, base) in captures {
            layer.capture_base(key, ip, base);
            self.modifiers.base_tiles.insert(ti, key);
        }
        let mut dirty = self.modifiers.unindex_modifier(id);
        for (loc, polys) in selection {
            dirty.entry(loc).or_default().extend(&polys);
            Modifiers::index(
                &mut self.modifiers.by_tile,
                &mut self.modifiers.by_modifier,
                id,
                loc,
                polys,
            );
        }
        self.apply_modifier_edit(dirty);
        Ok(())
    }

    /// Unregisters a spec and reapplies remaining overlapping modifiers over
    /// the base state. Returns whether the spec was registered.
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
        let change = self.advance_change_epoch();
        for ((_, _, _, ti), polys) in dirty {
            let slot = &mut self.slots[ti];
            if let Some(tile) = slot.tile.as_deref_mut()
                && self.modifiers.apply(tile, ti, polys)
            {
                slot.state_stamp = stamp;
                slot.change_stamp = change;
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
        let change = self.advance_change_epoch();
        for ((_, _, _, ti), polys) in self.modifiers.by_modifier.get(&id).into_iter().flatten() {
            let slot = &mut self.slots[*ti];
            if let Some(tile) = slot.tile.as_deref_mut()
                && self.modifiers.apply(tile, *ti, polys.iter().copied())
            {
                slot.state_stamp = stamp;
                slot.change_stamp = change;
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

    /// Returns the caller's flags before modifier effects.
    pub fn poly_base_flags(&self, r: PolyRef) -> Result<u16, Error> {
        Ok(self.poly_base_state(r)?.0)
    }

    /// Returns the caller's area before modifier effects.
    pub fn poly_base_area(&self, r: PolyRef) -> Result<u8, Error> {
        Ok(self.poly_base_state(r)?.1)
    }

    fn poly_base_state(&self, r: PolyRef) -> Result<(u16, u8), Error> {
        let (tile, poly) = self.tile_and_poly_by_ref(r)?;
        let (_, ti, ip) = self.decode_poly_id(r);
        let (ti, ip) = (ti as usize, ip as usize);
        Ok(if self.modifiers.manages_state(ti, ip) {
            self.modifiers.base_state(tile, ti, ip)
        } else {
            (poly.flags, poly.area())
        })
    }

    /// Removes a saved override by tile content and polygon index, including
    /// for unloaded tiles. Reapplies any active effects to the baked values
    /// when the matching polygon is resident.
    pub fn remove_poly_base_edit(&mut self, edit: &PolyBaseEdit) -> bool {
        if !self
            .modifiers
            .layer
            .as_mut()
            .is_some_and(|layer| layer.remove_poly_base_edit(edit))
        {
            return false;
        }
        if let Some(ti) = self.tile_index_at(edit.key.x, edit.key.y, edit.layer)
            && self.modifiers.base_tiles.get(&ti) == Some(&edit.tile_key())
            && let Some(tile) = self.slots[ti].tile.as_deref_mut()
            && (edit.polygon_index as usize) < tile.polys.len()
            && self
                .modifiers
                .apply(tile, ti, [edit.polygon_index as usize])
        {
            self.stamp_state(ti);
        } else {
            self.modifiers.advance();
        }
        true
    }

    pub(super) fn set_modifier_base(
        &mut self,
        ti: usize,
        ip: usize,
        edit: BaseState,
    ) -> Result<bool, Error> {
        if !self.modifiers.manages_state(ti, ip) {
            return Ok(false);
        }
        let mut owned_flags = 0;
        let mut owned_area = false;
        for effect in self.modifiers.active_effects(ti, ip) {
            match effect {
                Effect::ClearFlags(mask) | Effect::SetFlags(mask) => owned_flags |= mask,
                Effect::SetArea(_) => owned_area = true,
            }
        }
        let slot = &mut self.slots[ti];
        let tile = slot.tile.as_deref_mut().ok_or(Error::InvalidRef)?;
        let key = self
            .modifiers
            .base_tiles
            .get(&ti)
            .copied()
            .unwrap_or_else(|| base_key(tile, slot.hash));
        let layer = self
            .modifiers
            .layer
            .as_mut()
            .ok_or(Error::InvalidParameter("no modifier layer"))?;
        let mut base = layer.base(key, ip);
        let (baked_flags, baked_area) = tile.baked_state(ip);
        if let Some(flags) = edit.flags {
            let flags = (flags & !owned_flags) | (base.flags.unwrap_or(baked_flags) & owned_flags);
            base.flags = (flags != baked_flags).then_some(flags);
        }
        if let Some(area) = edit.area
            && !owned_area
        {
            base.area = (area != baked_area).then_some(area);
        }
        if layer.set_base(key, ip, base)? {
            self.modifiers.base_tiles.insert(ti, key);
            if self.modifiers.apply(tile, ti, [ip]) {
                self.stamp_state(ti);
            } else {
                self.modifiers.advance();
            }
        }
        Ok(true)
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
