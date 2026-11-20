//! Declarative gates and cost zones over baked polygons.
//!
//! Register specifications before loading tiles, or install a saved layer with
//! [`crate::detour::NavMesh::set_dynamic_layer`]. The mesh owns the active layer
//! and applies it on every tile installation path.

use alloc::collections::BTreeMap;
use alloc::vec::Vec;

use crate::{Error, Limits};

mod snapshot;

const MAX_PRISM_VERTICES: usize = 64;

/// A stable, caller-assigned modifier identifier.
#[derive(Clone, Copy, Debug, Eq, Hash, Ord, PartialEq, PartialOrd)]
pub struct ModifierId(pub u32);

/// Which baked polygons a modifier affects.
#[derive(Clone, Debug, PartialEq)]
#[non_exhaustive]
pub enum Selector {
    /// Ground polygons whose vertex centroid lies inside a convex prism.
    Prism {
        /// Strictly convex XZ outline in either winding, with 3 to 64 vertices.
        verts_xz: Vec<[f32; 2]>,
        /// Inclusive lower Y bound.
        ymin: f32,
        /// Inclusive upper Y bound.
        ymax: f32,
        /// Require this baked area if present.
        area: Option<u8>,
    },
    /// Ground polygons with a detail triangle intersecting a convex prism.
    ///
    /// Polygons without detail triangles use a fan of their perimeter vertices.
    DetailPrism {
        /// Strictly convex XZ outline in either winding, with 3 to 64 vertices.
        verts_xz: Vec<[f32; 2]>,
        /// Inclusive lower Y bound.
        ymin: f32,
        /// Inclusive upper Y bound.
        ymax: f32,
        /// Require this baked area if present.
        area: Option<u8>,
    },
    /// Baked off-mesh connections whose user IDs fall in an inclusive range.
    Links {
        /// First user ID.
        user_id_min: u32,
        /// Last user ID.
        user_id_max: u32,
    },
}

impl Selector {
    pub(crate) fn is_detail(&self) -> bool {
        matches!(self, Self::DetailPrism { .. })
    }

    fn prism_vertices(&self) -> Option<&[[f32; 2]]> {
        match self {
            Self::Prism { verts_xz, .. } | Self::DetailPrism { verts_xz, .. } => Some(verts_xz),
            Self::Links { .. } => None,
        }
    }
}

/// An operation applied in ascending [`ModifierId`] order.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
#[non_exhaustive]
pub enum Effect {
    /// Clear these flag bits. Clearing every bit blocks all standard filters.
    ClearFlags(u16),
    /// Set these flag bits.
    SetFlags(u16),
    /// Set an area in 0..=63 for the query filter's cost calculation.
    SetArea(u8),
}

/// Resident polygon counts for one modifier, including when disabled.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
#[non_exhaustive]
pub struct SelectionReport {
    /// Polygons selected by the modifier.
    pub selected_polygons: usize,
    /// Ground polygons whose bounds intersect the prism but whose centroid
    /// falls outside it. Counts only the requested baked area, if specified.
    /// Only centroid prisms report this diagnostic.
    pub bounds_without_centroid: usize,
}

/// A modifier's stable identity, selection and effect.
#[derive(Clone, Debug, PartialEq)]
#[non_exhaustive]
pub struct ModifierSpec {
    /// Caller-assigned identifier.
    pub id: ModifierId,
    /// Which baked polygons to affect.
    pub selector: Selector,
    /// What to do when enabled.
    pub effect: Effect,
}

impl ModifierSpec {
    /// Describes a modifier. Validation occurs when it is registered.
    pub fn new(id: ModifierId, selector: Selector, effect: Effect) -> Self {
        Self {
            id,
            selector,
            effect,
        }
    }
}

/// Serializable modifier specifications, enabled bits and polygon base edits
/// for one navigation mesh.
///
/// Specifications remain registered when their tiles are unloaded. A newly
/// registered modifier starts disabled. Replacing its specification preserves
/// its enabled bit.
#[derive(Clone, Debug)]
#[non_exhaustive]
pub struct DynamicLayer {
    max_ids: u32,
    limits: Limits,
    entries: Vec<Entry>,
    specs_size: usize,
    detail_count: usize,
    bases: BTreeMap<(BaseTileKey, u32), BaseState>,
}

pub(crate) type BaseTileKey = (i32, i32, i32, [u8; 32]);

#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub(crate) struct BaseState {
    pub(crate) flags: Option<u16>,
    pub(crate) area: Option<u8>,
}

#[derive(Clone, Debug)]
struct Entry {
    spec: ModifierSpec,
    enabled: bool,
}

impl DynamicLayer {
    /// Creates an empty layer with identifiers in `0..max_ids`. Limits bound
    /// the encoded snapshot size and work for decoding and validating specs.
    pub fn new(max_ids: u32, limits: Limits) -> Result<Self, Error> {
        let layer = Self {
            max_ids,
            limits,
            entries: Vec::new(),
            specs_size: 0,
            detail_count: 0,
            bases: BTreeMap::new(),
        };
        layer.check_size(layer.snapshot_base_len())?;
        Ok(layer)
    }

    /// Returns a registered specification and its enabled bit.
    pub fn get(&self, id: ModifierId) -> Option<(&ModifierSpec, bool)> {
        let entry = &self.entries[self.position(id).ok()?];
        Some((&entry.spec, entry.enabled))
    }

    pub(crate) fn specs(&self) -> impl Iterator<Item = &ModifierSpec> {
        self.entries.iter().map(|entry| &entry.spec)
    }

    pub(crate) fn uses_detail(&self) -> bool {
        self.detail_count != 0
    }

    /// Registers a spec or replaces the spec of an existing ID.
    pub fn upsert(&mut self, spec: ModifierSpec) -> Result<(), Error> {
        self.validate_spec(&spec)?;
        let at = self.position(spec.id);
        let old_size = at.ok().map_or(0, |i| self.entries[i].spec.encoded_len());
        let new_size = (self.specs_size - old_size)
            .checked_add(spec.encoded_len())
            .ok_or(Error::SizeOverflow)?;
        let old_detail = at
            .ok()
            .is_some_and(|i| self.entries[i].spec.selector.is_detail());
        let detail_count =
            self.detail_count - usize::from(old_detail) + usize::from(spec.selector.is_detail());
        self.check_size(self.snapshot_len(new_size, self.bases.len(), detail_count != 0)?)?;
        match at {
            Ok(i) => self.entries[i].spec = spec,
            Err(i) => {
                self.entries
                    .try_reserve(1)
                    .map_err(|_| Error::AllocationFailed)?;
                self.entries.insert(
                    i,
                    Entry {
                        spec,
                        enabled: false,
                    },
                );
            }
        }
        self.specs_size = new_size;
        self.detail_count = detail_count;
        Ok(())
    }

    /// Removes a registered spec and its enabled bit. Returns whether it existed.
    pub fn remove(&mut self, id: ModifierId) -> bool {
        let Ok(i) = self.position(id) else {
            return false;
        };
        let spec = self.entries.remove(i).spec;
        self.specs_size -= spec.encoded_len();
        self.detail_count -= usize::from(spec.selector.is_detail());
        true
    }

    /// Sets a registered modifier's enabled bit. Returns whether it changed.
    pub fn set_enabled(&mut self, id: ModifierId, enabled: bool) -> Result<bool, Error> {
        let i = self
            .position(id)
            .map_err(|_| Error::InvalidParameter("unknown modifier ID"))?;
        Ok(core::mem::replace(&mut self.entries[i].enabled, enabled) != enabled)
    }

    fn position(&self, id: ModifierId) -> Result<usize, usize> {
        self.entries
            .binary_search_by_key(&id, |entry| entry.spec.id)
    }

    pub(crate) fn has_bases_at(&self, (x, y, layer): (i32, i32, i32)) -> bool {
        self.bases
            .range(((x, y, layer, [0; 32]), 0)..=((x, y, layer, [255; 32]), u32::MAX))
            .next()
            .is_some()
    }

    pub(crate) fn base_indices(&self, tile: BaseTileKey) -> impl Iterator<Item = usize> + '_ {
        self.bases
            .range((tile, 0)..=(tile, u32::MAX))
            .map(|(&(_, ip), _)| ip as usize)
    }

    pub(crate) fn base(&self, tile: BaseTileKey, ip: usize) -> BaseState {
        self.bases
            .get(&(tile, ip as u32))
            .copied()
            .unwrap_or_default()
    }

    pub(crate) fn set_base(
        &mut self,
        tile: BaseTileKey,
        ip: usize,
        base: BaseState,
    ) -> Result<bool, Error> {
        let key = (tile, ip as u32);
        if base == BaseState::default() {
            return Ok(self.bases.remove(&key).is_some());
        }
        let old = self.bases.get(&key).copied();
        if old.is_none() {
            let count = self.bases.len().checked_add(1).ok_or(Error::SizeOverflow)?;
            self.check_size(self.snapshot_len(self.specs_size, count, false)?)?;
        }
        self.bases.insert(key, base);
        Ok(old != Some(base))
    }

    fn check_size(&self, size: usize) -> Result<(), Error> {
        if size > self.limits.max_bytes {
            return Err(Error::LimitExceeded);
        }
        Ok(())
    }

    fn validate_spec(&self, spec: &ModifierSpec) -> Result<(), Error> {
        if spec.id.0 >= self.max_ids || matches!(spec.effect, Effect::SetArea(64..)) {
            return Err(Error::InvalidParameter("modifier ID or area"));
        }
        match &spec.selector {
            Selector::Prism {
                verts_xz,
                ymin,
                ymax,
                area,
            }
            | Selector::DetailPrism {
                verts_xz,
                ymin,
                ymax,
                area,
            } => {
                let n = verts_xz.len();
                if !(3..=MAX_PRISM_VERTICES).contains(&n)
                    || !ymin.is_finite()
                    || !ymax.is_finite()
                    || ymin > ymax
                    || area.is_some_and(|a| a > 63)
                    || verts_xz.iter().flatten().any(|v| !v.is_finite())
                {
                    return Err(Error::InvalidParameter("modifier prism"));
                }
                if (n * n) as u64 > self.limits.max_work {
                    return Err(Error::LimitExceeded);
                }
                let winding = side(verts_xz[0], verts_xz[1], verts_xz[2].map(f64::from), 1.0);
                // Every other vertex must be strictly on the same side of every
                // edge. This also rejects self-intersections and duplicate vertices.
                for i in 0..n {
                    let next = (i + 1) % n;
                    for (j, p) in verts_xz.iter().enumerate() {
                        if j != i
                            && j != next
                            && side(verts_xz[i], verts_xz[next], p.map(f64::from), 1.0) * winding
                                <= 0.0
                        {
                            return Err(Error::InvalidParameter(
                                "modifier prism must be strictly convex",
                            ));
                        }
                    }
                }
            }
            Selector::Links {
                user_id_min,
                user_id_max,
            } if user_id_min > user_id_max => {
                return Err(Error::InvalidParameter("modifier link ID range"));
            }
            Selector::Links { .. } => {}
        }
        Ok(())
    }
}

// `p` can be a vertex sum. Scaling the edge origin gives the centroid
// test without dividing that sum, including for points exactly on an edge.
pub(crate) fn side(a: [f32; 2], b: [f32; 2], p: [f64; 2], count: f64) -> f64 {
    let [ax, az] = a.map(f64::from);
    let [bx, bz] = b.map(f64::from);
    (bx - ax) * (p[1] - az * count) - (bz - az) * (p[0] - ax * count)
}
