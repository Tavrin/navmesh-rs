//! Declarative gates and cost zones over baked polygons.
//!
//! Register specifications before loading tiles, or install a saved layer with
//! [`crate::detour::NavMesh::set_dynamic_layer`]. The mesh owns the active layer
//! and applies it on every tile installation path.

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
    /// Baked off-mesh connections whose user IDs fall in an inclusive range.
    Links {
        /// First user ID.
        user_id_min: u32,
        /// Last user ID.
        user_id_max: u32,
    },
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
    /// This count is zero for link selectors.
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

/// Serializable modifier specifications and enabled bits for one navigation mesh.
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

    /// Registers a spec or replaces the spec of an existing ID.
    pub fn upsert(&mut self, spec: ModifierSpec) -> Result<(), Error> {
        self.validate_spec(&spec)?;
        let at = self.position(spec.id);
        let old_size = at.ok().map_or(0, |i| self.entries[i].spec.encoded_len());
        let new_size = (self.specs_size - old_size)
            .checked_add(spec.encoded_len())
            .ok_or(Error::SizeOverflow)?;
        self.check_size(
            self.snapshot_base_len()
                .checked_add(new_size)
                .ok_or(Error::SizeOverflow)?,
        )?;
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
        Ok(())
    }

    /// Removes a registered spec and its enabled bit. Returns whether it existed.
    pub fn remove(&mut self, id: ModifierId) -> bool {
        let Ok(i) = self.position(id) else {
            return false;
        };
        self.specs_size -= self.entries.remove(i).spec.encoded_len();
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
