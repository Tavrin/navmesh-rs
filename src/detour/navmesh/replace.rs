//! Atomic tile replacement and salt headroom (not in recastnavigation).

use alloc::boxed::Box;

use super::stream::{PreparedTile, TileKey, content_hash};
use super::{NavMesh, Placement, Target};
use crate::detour::TileRef;
use crate::{Budget, Error};

/// Outcome of [`NavMesh::replace_tile`] and [`NavMesh::replace_tile_budgeted`].
#[derive(Debug)]
#[non_exhaustive]
pub enum Replaced {
    /// A tile with the same content hash was resident. Nothing changed: not its
    /// ref, salt, runtime flags and areas, nor the epoch.
    Unchanged(TileRef),
    /// The resident tile had other content and was swapped in place: same slot,
    /// next salt. Refs carrying `old_salt` are
    /// [`RefState::Stale`](super::RefState::Stale).
    Replaced {
        /// Ref of the new tile.
        new: TileRef,
        /// Salt of the tile that was replaced.
        old_salt: u32,
    },
    /// No tile was resident at the column and layer; the tile was committed as
    /// by [`NavMesh::commit`].
    Added(TileRef),
    /// [`NavMesh::replace_tile_budgeted`] only: the budget could not cover the
    /// work; nothing changed and the tile is handed back for a later frame.
    Refused(Box<PreparedTile>),
}

impl Replaced {
    /// The ref of the tile now resident at the column and layer (`None` when
    /// refused).
    pub fn tile_ref(&self) -> Option<TileRef> {
        match self {
            Replaced::Unchanged(r) | Replaced::Added(r) => Some(*r),
            Replaced::Replaced { new, .. } => Some(*new),
            Replaced::Refused(_) => None,
        }
    }
}

/// What a replacement will do, decided before anything changes.
enum Plan {
    /// Slot of the resident tile with equal content, and that content's hash.
    Unchanged(usize, [u8; 32]),
    /// Swap the resident tile of slot `ti`, advancing its salt to `salt`.
    Swap { ti: usize, old_salt: u32, salt: u32 },
    /// No resident tile: commit into `target`.
    Add(Target),
}

impl NavMesh {
    /// Makes `t` the tile at its column and layer, all or nothing (not in
    /// recastnavigation).
    ///
    /// - A resident tile with the same [`content_hash`]: nothing changes
    ///   ([`Replaced::Unchanged`]), so no salt advance, no epoch change, and runtime
    ///   flags and areas set with [`NavMesh::set_poly_flags`] stay.
    /// - A resident tile with other content: the new tile takes its slot with the
    ///   next salt ([`Replaced::Replaced`]). Refs into the old tile become stale,
    ///   the removal is logged for [`NavMesh::changed_since`], links to the
    ///   neighbours are rebuilt and flags and areas come from the new bytes. The
    ///   epoch advances once. The slot, lookup lists and free list end as after
    ///   [`NavMesh::remove_tile`]
    ///   then [`NavMesh::commit`].
    /// - No resident tile: committed as by [`NavMesh::commit`], honouring an
    ///   unloaded tile's reservation ([`Replaced::Added`]).
    ///
    /// Errors, with the navmesh unchanged: [`Error::InvalidParameter`] for a tile
    /// prepared for another ref layout, [`Error::SaltExhausted`] when the slot's
    /// salt cannot advance (see [`NavMesh`]), and otherwise as
    /// [`NavMesh::commit`]. Unlike a removal followed by a commit, a refusal
    /// leaves the resident tile with its salt, refs and runtime state.
    pub fn replace_tile(&mut self, t: PreparedTile) -> Result<Replaced, Error> {
        self.check_ground_layer(t.tile.header.layer)?;
        self.replace_managed_tile(t)
    }

    pub(super) fn replace_managed_tile(&mut self, t: PreparedTile) -> Result<Replaced, Error> {
        let plan = self.plan_replace(&t, true)?;
        self.apply_replace(t, plan)
    }

    /// [`NavMesh::replace_tile`], except that equal content is replaced too (one
    /// salt advance, flags from the bytes), as upstream's remove-then-add does.
    #[cfg(feature = "tilecache")]
    pub(crate) fn replace_tile_advancing(&mut self, t: PreparedTile) -> Result<Replaced, Error> {
        self.check_ground_layer(t.tile.header.layer)?;
        let plan = self.plan_replace(&t, false)?;
        self.apply_replace(t, plan)
    }

    /// [`NavMesh::replace_tile`] when `budget` covers [`NavMesh::replace_cost`];
    /// otherwise nothing changes, the budget is marked exhausted and the tile
    /// comes back as [`Replaced::Refused`]. Errors are reported before the budget
    /// is charged.
    pub fn replace_tile_budgeted(
        &mut self,
        t: PreparedTile,
        budget: &mut Budget,
    ) -> Result<Replaced, Error> {
        self.check_ground_layer(t.tile.header.layer)?;
        let plan = self.plan_replace(&t, true)?;
        if !budget.spend(self.plan_cost(&t, &plan)) {
            return Ok(Replaced::Refused(Box::new(t)));
        }
        self.apply_replace(t, plan)
    }

    /// Work units [`NavMesh::replace_tile_budgeted`] charges for `t`: one when the
    /// resident tile has the same content; [`NavMesh::commit_cost`] when no tile
    /// is resident; otherwise the commit cost of `t` plus the commit cost the
    /// resident tile has now (its unlinking touches the same neighbours).
    pub fn replace_cost(&self, t: &PreparedTile) -> u64 {
        let h = &t.tile.header;
        match self.tile_index_at(h.x, h.y, h.layer) {
            None => self.link_cost(&t.tile),
            Some(ti) if self.resident_hash(ti) == Some(t.hash) => 1,
            Some(ti) => self.swap_cost(t, ti),
        }
    }

    /// Salt advances left for the slot of the tile at `key` and `layer`: for a
    /// fresh slot `2^salt_bits - 2` (65,534 with the default 64-bit layout,
    /// 4,194,302 with [`RefLayout::WIDE_SALT`](super::RefLayout::WIDE_SALT)); at 0 a
    /// removal, forgotten reservation or replacement with other content fails
    /// with [`Error::SaltExhausted`]. For a keyed navmesh the slot is the key's
    /// (`None` outside the grid); for a sequential one it is the slot holding or
    /// reserved for the tile (`None` when there is none).
    pub fn salt_headroom(&self, key: TileKey, layer: i32) -> Option<u32> {
        let ti = match self.stream.keyed_slot(key.x, key.y, layer) {
            Some(slot) => slot?,
            None => self
                .tile_index_at(key.x, key.y, layer)
                .or_else(|| self.stream.parked_slot(&self.slots, key.x, key.y, layer))?,
        };
        Some(self.salt_mask().saturating_sub(self.slots[ti].salt))
    }

    /// Content hash of the resident tile in slot `ti`.
    fn resident_hash(&self, ti: usize) -> Option<[u8; 32]> {
        let s = self.slots.get(ti)?;
        s.hash
            .or_else(|| s.tile.as_deref().map(|t| content_hash(&t.data)))
    }

    fn swap_cost(&self, t: &PreparedTile, ti: usize) -> u64 {
        let old = self.tile(ti).map_or(0, |old| self.link_cost(old));
        self.link_cost(&t.tile).saturating_add(old)
    }

    fn plan_cost(&self, t: &PreparedTile, plan: &Plan) -> u64 {
        match *plan {
            Plan::Unchanged(..) => 1,
            Plan::Swap { ti, .. } => self.swap_cost(t, ti),
            Plan::Add(_) => self.link_cost(&t.tile),
        }
    }

    /// Every check of a replacement, without changing anything. With `keep_equal`
    /// false, a resident tile with equal content is swapped like any other.
    fn plan_replace(&self, t: &PreparedTile, keep_equal: bool) -> Result<Plan, Error> {
        if t.layout != self.ref_layout() {
            return Err(Error::InvalidParameter(
                "tile prepared for another ref layout",
            ));
        }
        let h = &t.tile.header;
        if let Some(ti) = self.tile_index_at(h.x, h.y, h.layer) {
            if keep_equal && self.resident_hash(ti) == Some(t.hash) {
                return Ok(Plan::Unchanged(ti, t.hash));
            }
            let old_salt = self.slots[ti].salt;
            let salt = self.next_salt(old_salt)?;
            return Ok(Plan::Swap { ti, old_salt, salt });
        }
        let target = self.choose_slot(h.x, h.y, h.layer, Placement::Free)?;
        if target.parked_hash.is_some_and(|old| old != t.hash) {
            self.next_salt(target.salt)?;
        }
        Ok(Plan::Add(target))
    }

    fn apply_replace(&mut self, t: PreparedTile, plan: Plan) -> Result<Replaced, Error> {
        match plan {
            Plan::Unchanged(ti, hash) => {
                self.slots[ti].hash = Some(hash);
                Ok(Replaced::Unchanged(self.tile_ref(ti)))
            }
            Plan::Swap { ti, old_salt, salt } => {
                // Nothing below fails: the slot's page is allocated (it holds a
                // tile) and the salt was checked.
                let retain_dynamic_bases =
                    cfg!(feature = "dynamic") && self.resident_hash(ti) == Some(t.hash);
                drop(self.detach(ti, retain_dynamic_bases));
                let target = Target {
                    ti,
                    prev: None,
                    salt,
                    parked_hash: None,
                };
                let new = self.attach(t.tile, target, salt, Some(t.hash), true);
                self.stream.record_removal(ti as u32);
                Ok(Replaced::Replaced { new, old_salt })
            }
            Plan::Add(target) => Ok(Replaced::Added(self.link_in(
                t.tile,
                target,
                Some(t.hash),
                true,
            )?)),
        }
    }
}
