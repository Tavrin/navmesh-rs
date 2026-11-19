//! Mesh ownership, runtime selection and scoped references.

use alloc::collections::BTreeMap;
use alloc::sync::Arc;
use alloc::vec::Vec;

use super::catalog::{CatalogTile, ProfileCatalog, read_version, take};
use super::{ProfileError, ProfileId};
use crate::bake::TileKey;
use crate::connectivity::{Connectivity, ReachableCoverage};
use crate::detour::{
    self, NavMesh, NavMeshQuery, PolyRef, PreparedTile, QueryFilter, RefState, ResidencyRecord,
    Status, TileRef,
};
use crate::{Budget, Budgeted, Error};

/// A polygon reference tied to the instance that issued it.
#[derive(Clone, Debug)]
#[non_exhaustive]
pub struct ProfilePolyRef {
    owner: Arc<()>,
    raw: PolyRef,
}

impl PartialEq for ProfilePolyRef {
    fn eq(&self, other: &Self) -> bool {
        self.raw == other.raw && Arc::ptr_eq(&self.owner, &other.owner)
    }
}

impl Eq for ProfilePolyRef {}

impl ProfilePolyRef {
    /// Numeric bits for diagnostics. Different mesh instances can have equal bits.
    pub fn numeric(&self) -> PolyRef {
        self.raw
    }
}

/// A tile reference tied to the instance that issued it.
#[derive(Clone, Debug)]
#[non_exhaustive]
pub struct ProfileTileRef {
    owner: Arc<()>,
    raw: TileRef,
}

impl PartialEq for ProfileTileRef {
    fn eq(&self, other: &Self) -> bool {
        self.raw == other.raw && Arc::ptr_eq(&self.owner, &other.owner)
    }
}

impl Eq for ProfileTileRef {}

/// Tile prepared against an exact catalog address, ready for installation.
pub struct ProfilePreparedTile {
    address: CatalogTile,
    tile: Option<PreparedTile>,
}

impl ProfileCatalog {
    /// Verifies the exact raw payload and prepares it against the catalog layout.
    /// Pass no payload for a catalog entry that explicitly marks a column empty.
    /// This may run on a worker before handing the result to the mesh owner.
    pub fn prepare(
        &self,
        address: CatalogTile,
        bytes: Option<Vec<u8>>,
    ) -> Result<ProfilePreparedTile, ProfileError> {
        if self.tile(address.key) != Some(address) {
            return Err(ProfileError::BindingMismatch);
        }
        let tile = match (address.payload, bytes) {
            (None, None) => None,
            (Some(payload), Some(bytes)) if detour::content_hash(&bytes) == payload.raw => {
                let options = self.options;
                let params = self
                    .spec
                    .grid
                    .nav_mesh_params(options.max_tiles, options.max_polys);
                let tile = match options.ref_layout {
                    Some(layout) => {
                        detour::prepare_tile_with_layout(&params, options.limits, layout, bytes)?
                    }
                    None => detour::prepare_tile(&params, options.limits, bytes)?,
                };
                if tile.key() != key(address.key)
                    || tile.layer() != 0
                    || tile.content_hash() != payload.prepared
                    || tile.tile().polys.len() != payload.polys as usize
                {
                    return Err(ProfileError::TileMismatch);
                }
                Some(tile)
            }
            _ => return Err(ProfileError::TileMismatch),
        };
        Ok(ProfilePreparedTile { address, tile })
    }
}

/// The meshes registered for one map. Selection always requires a profile ID.
#[derive(Default)]
pub struct ProfileMeshes {
    meshes: BTreeMap<ProfileId, ProfileMesh>,
}

impl ProfileMeshes {
    /// Creates an empty map of profile meshes.
    pub fn new() -> Self {
        Self::default()
    }

    /// Registers a catalog and its initially empty mesh.
    pub fn insert(&mut self, catalog: ProfileCatalog) -> Result<(), ProfileError> {
        let id = catalog.profile();
        if self.meshes.contains_key(&id) {
            return Err(ProfileError::DuplicateProfile(id));
        }
        if self
            .meshes
            .values()
            .next()
            .is_some_and(|mesh| mesh.catalog.revision.map != catalog.revision.map)
        {
            return Err(ProfileError::BindingMismatch);
        }
        self.meshes.insert(id, ProfileMesh::new(catalog)?);
        Ok(())
    }

    /// Selects the mesh for this profile.
    pub fn get(&self, id: ProfileId) -> Result<&ProfileMesh, ProfileError> {
        self.meshes.get(&id).ok_or(ProfileError::UnknownProfile(id))
    }

    fn mesh_mut(&mut self, id: ProfileId) -> Result<&mut ProfileMesh, ProfileError> {
        self.meshes
            .get_mut(&id)
            .ok_or(ProfileError::UnknownProfile(id))
    }

    /// Installs a verified result in the selected profile.
    pub fn commit(
        &mut self,
        id: ProfileId,
        tile: ProfilePreparedTile,
    ) -> Result<Option<ProfileTileRef>, ProfileError> {
        self.mesh_mut(id)?.commit(tile)
    }

    /// Unloads an installed tile in the selected profile.
    pub fn unload(&mut self, id: ProfileId, tile: &ProfileTileRef) -> Result<(), ProfileError> {
        self.mesh_mut(id)?.unload(tile)
    }

    /// Releases a parked slot in the selected profile.
    pub fn forget_unloaded(
        &mut self,
        id: ProfileId,
        address: CatalogTile,
    ) -> Result<(), ProfileError> {
        self.mesh_mut(id)?.forget_unloaded(address)
    }

    /// Adopts a newer catalog for its registered profile.
    pub fn advance_catalog(&mut self, next: ProfileCatalog) -> Result<(), ProfileError> {
        self.mesh_mut(next.profile())?.advance_catalog(next)
    }

    /// Restores the selected profile's residency before installing its tiles.
    pub fn restore_residency(&mut self, id: ProfileId, bytes: &[u8]) -> Result<(), ProfileError> {
        self.mesh_mut(id)?.restore_residency(bytes)
    }

    /// Binds a query to the selected profile.
    pub fn query(&self, id: ProfileId, max_nodes: usize) -> Result<ProfileQuery<'_>, ProfileError> {
        self.get(id)?.query(max_nodes)
    }
}

/// One profile's independently streamed navigation mesh.
pub struct ProfileMesh {
    pub(super) nav: NavMesh,
    pub(super) catalog: ProfileCatalog,
}

fn key(key: TileKey) -> detour::TileKey {
    detour::TileKey { x: key.x, y: key.z }
}

impl ProfileMesh {
    /// Opens an empty mesh and declares the catalog's loadable columns.
    pub fn new(catalog: ProfileCatalog) -> Result<Self, ProfileError> {
        let mut nav = catalog.options.nav(catalog.spec)?;
        for column in catalog.keys() {
            nav.mark_known(key(column), 1);
        }
        Ok(Self { nav, catalog })
    }

    /// The mesh's current catalog.
    pub fn catalog(&self) -> &ProfileCatalog {
        &self.catalog
    }

    /// The mesh's structural epoch.
    pub fn epoch(&self) -> u64 {
        self.nav.epoch()
    }

    /// Whether the column has a known missing tile.
    pub fn is_column_unloaded(&self, key: TileKey) -> bool {
        self.nav.is_column_unloaded(key.x, key.z)
    }

    /// Adopts a newer catalog with the same bake and runtime settings.
    ///
    /// Removed and empty columns are discarded, with salt exhaustion checked
    /// before any change. Install changed nonempty payloads individually after
    /// adoption. Old resident versions serve queries until replaced.
    pub fn advance_catalog(&mut self, next: ProfileCatalog) -> Result<(), ProfileError> {
        if self.catalog.revision.map != next.revision.map
            || self.catalog.profile() != next.profile()
            || self.catalog.revision.revision >= next.revision.revision
            || !self.catalog.compatible_layout(&next)
        {
            return Err(ProfileError::BindingMismatch);
        }
        let record = self.nav.residency();
        let mut removed = Vec::new();
        for (column, layer, _, _) in record.tiles() {
            let bake_key = TileKey {
                x: column.x,
                z: column.y,
            };
            if next
                .tile(bake_key)
                .and_then(CatalogTile::raw_hash)
                .is_none()
            {
                if self.nav.salt_headroom(column, layer) == Some(0) {
                    return Err(Error::SaltExhausted.into());
                }
                removed.push(bake_key);
            }
        }
        for column in removed {
            self.discard(column)?;
        }
        for column in self.catalog.keys() {
            self.nav.mark_known(key(column), 0);
        }
        for column in next.keys() {
            self.nav.mark_known(key(column), 1);
        }
        self.catalog = next;
        Ok(())
    }

    /// Installs a verified asset or applies a verified empty result.
    /// Equal content preserves the existing refs and epoch.
    pub fn commit(
        &mut self,
        prepared: ProfilePreparedTile,
    ) -> Result<Option<ProfileTileRef>, ProfileError> {
        if self.catalog.tile(prepared.address.key) != Some(prepared.address) {
            return Err(ProfileError::BindingMismatch);
        }
        let Some(tile) = prepared.tile else {
            self.discard(prepared.address.key)?;
            return Ok(None);
        };
        Ok(self
            .nav
            .replace_tile(tile)?
            .tile_ref()
            .map(|raw| ProfileTileRef {
                owner: Arc::clone(self.nav.report_identity()),
                raw,
            }))
    }

    fn discard(&mut self, column: TileKey) -> Result<(), Error> {
        if let Some(index) = self.nav.tile_index_at(column.x, column.z, 0) {
            self.nav.remove_tile(self.nav.tile_ref(index))?;
        } else if self.nav.has_parked_tile(key(column), 0) {
            self.nav.forget_unloaded(key(column), 0)?;
        }
        Ok(())
    }

    /// Unloads a tile, retaining its salt and prepared content identity.
    /// Reload through the same catalog using the original asset bytes.
    pub fn unload(&mut self, tile: &ProfileTileRef) -> Result<(), ProfileError> {
        self.check_owner(&tile.owner)?;
        self.nav.unload_tile(tile.raw)?;
        Ok(())
    }

    /// Releases a parked slot while keeping its catalog availability. Saved refs
    /// to the parked content become stale. Use this to free a sequential slot
    /// before loading a different column.
    pub fn forget_unloaded(&mut self, address: CatalogTile) -> Result<(), ProfileError> {
        if self.catalog.tile(address.key) != Some(address) {
            return Err(ProfileError::BindingMismatch);
        }
        self.nav.forget_unloaded(key(address.key), 0)?;
        Ok(())
    }

    /// State of a polygon reference issued by this mesh instance.
    pub fn ref_state(&self, polygon: &ProfilePolyRef) -> Result<RefState, ProfileError> {
        Ok(self.nav.ref_state(self.polygon(polygon)?))
    }

    /// Saves the slot history, bound to the installed catalog revision.
    /// A changed resident or parked payload must be installed before saving.
    pub fn save_residency(&self) -> Result<Vec<u8>, ProfileError> {
        let record = self.nav.residency();
        self.check_residency(&record)?;
        Ok(self.wrap_record(*b"NPRS", &record.to_bytes()))
    }

    /// Restores checked slot history into a fresh mesh before committing tiles.
    pub fn restore_residency(&mut self, bytes: &[u8]) -> Result<(), ProfileError> {
        let record = ResidencyRecord::from_bytes(self.record_payload(*b"NPRS", bytes)?)?;
        self.check_residency(&record)?;
        self.nav.restore_residency(&record)?;
        Ok(())
    }

    fn check_residency(&self, record: &ResidencyRecord) -> Result<(), ProfileError> {
        for (column, layer, hash, polys) in record.tiles() {
            let payload = self
                .catalog
                .tile(TileKey {
                    x: column.x,
                    z: column.y,
                })
                .and_then(|entry| entry.payload);
            if layer != 0
                || !payload
                    .is_some_and(|payload| payload.prepared == hash && payload.polys == polys)
            {
                return Err(ProfileError::TileMismatch);
            }
        }
        Ok(())
    }

    /// Saves a resident or parked polygon reference for use with saved residency.
    pub fn save_reference(&self, polygon: &ProfilePolyRef) -> Result<Vec<u8>, ProfileError> {
        let raw = self.polygon(polygon)?;
        self.check_saved_ref(raw)?;
        self.wrap_residency_bound(*b"NPRF", &raw.to_le_bytes())
    }

    /// Binds a saved reference to this instance after restoring its residency.
    pub fn restore_reference(&self, bytes: &[u8]) -> Result<ProfilePolyRef, ProfileError> {
        let raw = PolyRef::from_le_bytes(
            self.residency_bound_payload(*b"NPRF", bytes)?
                .try_into()
                .map_err(|_| ProfileError::InvalidRecord)?,
        );
        self.check_saved_ref(raw)?;
        Ok(scoped(&self.nav, raw))
    }

    fn check_saved_ref(&self, raw: PolyRef) -> Result<(), ProfileError> {
        match self.nav.ref_state(raw) {
            RefState::Valid | RefState::Unloaded(_) => Ok(()),
            _ => Err(Error::InvalidRef.into()),
        }
    }

    pub(super) fn wrap_residency_bound(
        &self,
        magic: [u8; 4],
        payload: &[u8],
    ) -> Result<Vec<u8>, ProfileError> {
        let record = self.nav.residency();
        self.check_residency(&record)?;
        let mut body = detour::content_hash(&record.to_bytes()).to_vec();
        body.extend_from_slice(payload);
        Ok(self.wrap_record(magic, &body))
    }

    pub(super) fn residency_bound_payload<'b>(
        &self,
        magic: [u8; 4],
        bytes: &'b [u8],
    ) -> Result<&'b [u8], ProfileError> {
        let mut body = self.record_payload(magic, bytes)?;
        let expected = take::<32>(&mut body)?;
        let record = self.nav.residency();
        self.check_residency(&record)?;
        if expected != detour::content_hash(&record.to_bytes()) {
            return Err(ProfileError::BindingMismatch);
        }
        Ok(body)
    }

    pub(super) fn wrap_record(&self, magic: [u8; 4], payload: &[u8]) -> Vec<u8> {
        let mut bytes = Vec::new();
        bytes.extend(magic);
        bytes.extend(1u32.to_le_bytes());
        bytes.extend(self.catalog.digest);
        bytes.extend_from_slice(payload);
        bytes.extend(detour::content_hash(&bytes));
        bytes
    }

    pub(super) fn record_payload<'b>(
        &self,
        magic: [u8; 4],
        bytes: &'b [u8],
    ) -> Result<&'b [u8], ProfileError> {
        let mut body = read_version(bytes, magic, 1)?;
        if take::<32>(&mut body)? != self.catalog.digest {
            return Err(ProfileError::BindingMismatch);
        }
        let (payload, hash) = body
            .split_at_checked(body.len().saturating_sub(32))
            .ok_or(ProfileError::InvalidRecord)?;
        if hash.len() != 32 || hash != detour::content_hash(&bytes[..bytes.len() - 32]) {
            return Err(ProfileError::InvalidRecord);
        }
        Ok(payload)
    }

    pub(super) fn check_owner(&self, owner: &Arc<()>) -> Result<(), ProfileError> {
        if Arc::ptr_eq(self.nav.report_identity(), owner) {
            Ok(())
        } else {
            Err(ProfileError::BindingMismatch)
        }
    }

    pub(super) fn polygon(&self, reference: &ProfilePolyRef) -> Result<PolyRef, ProfileError> {
        self.check_owner(&reference.owner)?;
        Ok(reference.raw)
    }

    /// Creates a query restricted to this mesh.
    pub fn query(&self, max_nodes: usize) -> Result<ProfileQuery<'_>, ProfileError> {
        Ok(ProfileQuery {
            query: NavMeshQuery::new(&self.nav, max_nodes)?,
        })
    }

    /// Builds a structural connectivity snapshot for the profile.
    pub fn connectivity(
        &self,
        budget: &mut Budget,
    ) -> Result<Budgeted<Option<ProfileConnectivity>>, ProfileError> {
        Ok(match Connectivity::new(&self.nav, budget)? {
            Budgeted::Complete(graph) => {
                Budgeted::Complete(graph.map(|graph| ProfileConnectivity {
                    graph,
                    owner: Arc::clone(self.nav.report_identity()),
                }))
            }
            Budgeted::Exhausted(_) => Budgeted::Exhausted(None),
        })
    }
}

fn scoped(nav: &NavMesh, raw: PolyRef) -> ProfilePolyRef {
    ProfilePolyRef {
        owner: Arc::clone(nav.report_identity()),
        raw,
    }
}

/// A nearest point with its scoped polygon.
#[derive(Clone, Debug)]
#[non_exhaustive]
pub struct ProfilePoint {
    /// Polygon containing the point.
    pub polygon: ProfilePolyRef,
    /// Point on the navigation surface.
    pub position: [f32; 3],
    /// Whether the nearest-poly input was over the polygon in xz.
    pub over_polygon: bool,
}

impl ProfilePoint {
    /// Pairs a scoped polygon (including one restored from a save) with a position.
    pub fn new(polygon: ProfilePolyRef, position: [f32; 3]) -> Self {
        Self {
            polygon,
            position,
            over_polygon: false,
        }
    }
}

/// A straight-path corner and its scoped polygon, if any.
#[derive(Clone, Debug)]
#[non_exhaustive]
pub struct ProfileCorner {
    /// Corner position.
    pub position: [f32; 3],
    /// Detour straight-path flags.
    pub flags: u8,
    /// Polygon entered here, absent at the end.
    pub polygon: Option<ProfilePolyRef>,
}

/// A query bound to one mesh for its whole lifetime, including sliced searches.
pub struct ProfileQuery<'a> {
    query: NavMeshQuery<'a>,
}

impl ProfileQuery<'_> {
    /// Enables the specified query behavior fixes.
    pub fn set_fixes(&mut self, fixes: crate::UpstreamFixes) {
        self.query.set_fixes(fixes);
    }

    /// Finds a point on the nearest polygon in this profile.
    pub fn find_nearest_poly<F: QueryFilter + ?Sized>(
        &self,
        center: &[f32; 3],
        half_extents: &[f32; 3],
        filter: &F,
    ) -> Result<Option<ProfilePoint>, ProfileError> {
        let (raw, position, over_polygon) =
            self.query.find_nearest_poly(center, half_extents, filter)?;
        Ok((raw != 0).then(|| ProfilePoint {
            polygon: scoped(self.query.nav(), raw),
            position,
            over_polygon,
        }))
    }

    fn raw(&self, reference: &ProfilePolyRef) -> Result<PolyRef, ProfileError> {
        if !Arc::ptr_eq(self.query.nav().report_identity(), &reference.owner) {
            return Err(ProfileError::BindingMismatch);
        }
        Ok(reference.raw)
    }

    /// Finds a scoped corridor, with the usual Detour status and partial-path semantics.
    pub fn find_path<F: QueryFilter + ?Sized>(
        &mut self,
        start: &ProfilePoint,
        end: &ProfilePoint,
        filter: &F,
        max_path: usize,
    ) -> Result<(Status, Vec<ProfilePolyRef>), ProfileError> {
        let mut path = Vec::new();
        let status = self.query.find_path(
            self.raw(&start.polygon)?,
            self.raw(&end.polygon)?,
            &start.position,
            &end.position,
            filter,
            &mut path,
            max_path,
        )?;
        Ok((status, self.scope_path(path)))
    }

    /// Starts a sliced path search with scoped endpoints.
    pub fn init_sliced_find_path(
        &mut self,
        start: &ProfilePoint,
        end: &ProfilePoint,
        filter: &dyn QueryFilter,
        options: u32,
    ) -> Result<Status, ProfileError> {
        Ok(self.query.init_sliced_find_path(
            self.raw(&start.polygon)?,
            self.raw(&end.polygon)?,
            &start.position,
            &end.position,
            filter,
            options,
        )?)
    }

    /// Advances the current sliced search. Use the same filter throughout the search.
    pub fn update_sliced_find_path(
        &mut self,
        max_iter: usize,
        filter: &dyn QueryFilter,
    ) -> Result<(Status, usize), ProfileError> {
        Ok(self.query.update_sliced_find_path(max_iter, filter)?)
    }

    /// Finishes a sliced search and returns its scoped corridor.
    pub fn finalize_sliced_find_path(
        &mut self,
        filter: &dyn QueryFilter,
        max_path: usize,
    ) -> Result<(Status, Vec<ProfilePolyRef>), ProfileError> {
        let mut path = Vec::new();
        let status = self
            .query
            .finalize_sliced_find_path(filter, &mut path, max_path)?;
        Ok((status, self.scope_path(path)))
    }

    fn scope_path(&self, path: Vec<PolyRef>) -> Vec<ProfilePolyRef> {
        path.into_iter()
            .map(|raw| scoped(self.query.nav(), raw))
            .collect()
    }

    /// Pulls corners through a corridor using only this mesh.
    pub fn find_straight_path(
        &self,
        start: &[f32; 3],
        end: &[f32; 3],
        path: &[ProfilePolyRef],
        max: usize,
        options: i32,
    ) -> Result<(Status, Vec<ProfileCorner>), ProfileError> {
        let path = path
            .iter()
            .map(|polygon| self.raw(polygon))
            .collect::<Result<Vec<_>, _>>()?;
        let mut corners = Vec::new();
        let status =
            self.query
                .find_straight_path(start, end, &path, &mut corners, max, options)?;
        Ok((
            status,
            corners
                .into_iter()
                .map(|corner| ProfileCorner {
                    position: corner.pos,
                    flags: corner.flags,
                    polygon: (corner.r#ref != 0).then(|| scoped(self.query.nav(), corner.r#ref)),
                })
                .collect(),
        ))
    }

    /// Reads the ordered endpoints of a link entered from the preceding polygon.
    pub fn off_mesh_endpoints(
        &self,
        previous: &ProfilePolyRef,
        link: &ProfilePolyRef,
    ) -> Result<([f32; 3], [f32; 3]), ProfileError> {
        let nav = self.query.nav();
        let previous = self.raw(previous)?;
        let link = self.raw(link)?;
        if !nav.is_valid_poly_ref(previous) {
            return Err(Error::InvalidRef.into());
        }
        Ok(nav.off_mesh_connection_poly_end_points(previous, link)?)
    }
}

/// A structural graph for one profile mesh, ignoring query filters.
pub struct ProfileConnectivity {
    graph: Connectivity,
    owner: Arc<()>,
}

impl ProfileConnectivity {
    /// Updates this snapshot after a structural change to its own mesh.
    pub fn update(
        &mut self,
        mesh: &ProfileMesh,
        budget: &mut Budget,
    ) -> Result<Budgeted<()>, ProfileError> {
        mesh.check_owner(&self.owner)?;
        Ok(self.graph.update(&mesh.nav, budget)?)
    }

    /// Checks weak connectivity using current structural labels.
    pub fn same_island(
        &self,
        mesh: &ProfileMesh,
        a: &ProfilePolyRef,
        b: &ProfilePolyRef,
    ) -> Result<bool, ProfileError> {
        self.check(mesh)?;
        let a = self.graph.island(mesh.polygon(a)?);
        let b = self.graph.island(mesh.polygon(b)?);
        Ok(a.is_some() && a == b)
    }

    /// Measures directed structural coverage from a scoped seed.
    pub fn reachable_from(
        &self,
        mesh: &ProfileMesh,
        seed: &ProfilePolyRef,
        budget: &mut Budget,
    ) -> Result<Budgeted<Option<ReachableCoverage>>, ProfileError> {
        self.check(mesh)?;
        Ok(self.graph.reachable_from(&[mesh.polygon(seed)?], budget)?)
    }

    fn check(&self, mesh: &ProfileMesh) -> Result<(), ProfileError> {
        mesh.check_owner(&self.owner)?;
        if !self.graph.is_current(&mesh.nav) {
            return Err(Error::MeshChanged.into());
        }
        Ok(())
    }
}
