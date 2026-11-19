//! Versioned addressing of ordinary baked tiles.

use alloc::collections::{BTreeMap, BTreeSet};
use alloc::sync::Arc;
use alloc::vec::Vec;

use super::{ProfileBakeSpec, ProfileError, ProfileId, ProfileRevision, ProfileTileOutput};
use crate::bake::{BAKE_REVISION, BakeConfig, Partition, TileKey};
use crate::detour::{NavMesh, PolyRef, RefLayout, SlotPolicy, content_hash};
use crate::recast::Config;
use crate::{Error, Limits};

const MAGIC: [u8; 4] = *b"NPCG";
const VERSION: u32 = 1;
const ENTRY: usize = 77;

/// Runtime parameters included in a profile's catalog binding.
#[derive(Clone, Copy, Debug)]
#[non_exhaustive]
pub struct ProfileMeshOptions {
    /// Maximum slot count, including parked tiles.
    pub max_tiles: i32,
    /// Maximum polygons per tile used for reference layout validation.
    pub max_polys: i32,
    /// Slot assignment policy for the profile.
    pub slot_policy: SlotPolicy,
    /// Explicit reference layout, when required.
    pub ref_layout: Option<RefLayout>,
    /// Enable global attachment for authored off-mesh connections.
    pub global_off_mesh_links: bool,
    /// Runtime tile reader limits.
    pub limits: Limits,
}

impl ProfileMeshOptions {
    /// Uses sequential slots, the implied ref layout and default reader limits.
    pub fn new(max_tiles: i32, max_polys: i32) -> Self {
        Self {
            max_tiles,
            max_polys,
            slot_policy: SlotPolicy::Sequential,
            ref_layout: None,
            global_off_mesh_links: false,
            limits: Limits::default(),
        }
    }

    pub(super) fn validate_grid(&self, spec: ProfileBakeSpec) -> Result<(), Error> {
        if let SlotPolicy::Keyed {
            origin,
            width,
            height,
            ..
        } = self.slot_policy
            && (origin[0] > 0
                || origin[1] > 0
                || i64::from(origin[0]) + i64::from(width) < i64::from(spec.grid.tiles_x)
                || i64::from(origin[1]) + i64::from(height) < i64::from(spec.grid.tiles_z))
        {
            return Err(Error::InvalidParameter(
                "keyed slots do not cover the profile grid",
            ));
        }
        Ok(())
    }

    pub(super) fn nav(self, spec: ProfileBakeSpec) -> Result<NavMesh, Error> {
        self.validate_grid(spec)?;
        let params = spec.grid.nav_mesh_params(self.max_tiles, self.max_polys);
        let mut nav = match self.ref_layout {
            Some(layout) => NavMesh::with_layout(&params, self.limits, self.slot_policy, layout)?,
            None => NavMesh::with_policy(&params, self.limits, self.slot_policy)?,
        };
        nav.set_global_off_mesh_links(self.global_off_mesh_links);
        Ok(nav)
    }
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub(super) struct Payload {
    pub raw: [u8; 32],
    pub prepared: [u8; 32],
    pub polys: u32,
}

/// A checked catalog address for an ordinary baked column (layer zero).
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
#[non_exhaustive]
pub struct CatalogTile {
    pub(super) catalog: [u8; 32],
    pub(super) key: TileKey,
    pub(super) payload: Option<Payload>,
}

impl CatalogTile {
    /// Column of this entry.
    pub fn key(self) -> TileKey {
        self.key
    }

    /// SHA-256 of the original DNAV bytes, or none for an empty column.
    pub fn raw_hash(self) -> Option<[u8; 32]> {
        self.payload.map(|payload| payload.raw)
    }

    /// SHA-256 of the prepared tile bytes, including any reference-layout footer.
    pub fn prepared_hash(self) -> Option<[u8; 32]> {
        self.payload.map(|payload| payload.prepared)
    }
}

type Entries = Vec<(TileKey, Option<Payload>)>;

/// Immutable, versioned bindings between one profile revision and its tile assets.
#[derive(Clone, Debug)]
pub struct ProfileCatalog {
    pub(super) revision: ProfileRevision,
    pub(super) spec: ProfileBakeSpec,
    pub(super) options: ProfileMeshOptions,
    descriptor: [u8; 32],
    pub(super) digest: [u8; 32],
    entries: Arc<Entries>,
}

impl ProfileCatalog {
    /// The revision governing these asset addresses.
    pub fn revision(&self) -> ProfileRevision {
        self.revision
    }

    /// The stable ID governing these asset addresses.
    pub fn profile(&self) -> ProfileId {
        self.spec.id
    }

    /// SHA-256 of the versioned catalog bytes.
    pub fn digest(&self) -> [u8; 32] {
        self.digest
    }

    /// Resolves a column to its checked address. Absence means no entry was published.
    pub fn tile(&self, key: TileKey) -> Option<CatalogTile> {
        let index = self
            .entries
            .binary_search_by_key(&key, |&(key, _)| key)
            .ok()?;
        Some(CatalogTile {
            catalog: self.digest,
            key,
            payload: self.entries[index].1,
        })
    }

    pub(super) fn keys(&self) -> impl Iterator<Item = TileKey> + '_ {
        self.entries
            .iter()
            .filter_map(|&(key, payload)| payload.map(|_| key))
    }

    /// Encodes the catalog in canonical row-major order.
    pub fn to_bytes(&self) -> Vec<u8> {
        let mut bytes = Vec::new();
        bytes.extend(MAGIC);
        bytes.extend(VERSION.to_le_bytes());
        bytes.extend(self.revision.map);
        bytes.extend(self.revision.revision.to_le_bytes());
        bytes.extend(self.spec.id.0.to_le_bytes());
        bytes.extend(self.descriptor);
        bytes.extend((self.entries.len() as u64).to_le_bytes());
        for (key, payload) in self.entries.iter() {
            bytes.extend(key.x.to_le_bytes());
            bytes.extend(key.z.to_le_bytes());
            bytes.push(u8::from(payload.is_some()));
            if let Some(payload) = payload {
                bytes.extend(payload.raw);
                bytes.extend(payload.prepared);
                bytes.extend(payload.polys.to_le_bytes());
            } else {
                bytes.extend([0; 68]);
            }
        }
        bytes
    }

    /// Opens a catalog using its expected revision, profile settings and runtime layout.
    /// When loading from an external store, also verify the returned digest against
    /// the published catalog digest from that store's trusted index.
    pub fn from_bytes(
        bytes: &[u8],
        revision: ProfileRevision,
        spec: ProfileBakeSpec,
        options: ProfileMeshOptions,
    ) -> Result<Self, ProfileError> {
        let mut header = read_version(bytes, MAGIC, VERSION)?;
        let descriptor = descriptor(spec, options, true);
        if take::<16>(&mut header)? != revision.map
            || u64::from_le_bytes(take(&mut header)?) != revision.revision
            || u32::from_le_bytes(take(&mut header)?) != spec.id.0
            || take::<32>(&mut header)? != descriptor
        {
            return Err(ProfileError::BindingMismatch);
        }
        let count = usize::try_from(u64::from_le_bytes(take(&mut header)?))
            .map_err(|_| ProfileError::InvalidRecord)?;
        if count.checked_mul(ENTRY) != Some(header.len())
            || count as u64 > spec.grid.tiles_x as u64 * spec.grid.tiles_z as u64
        {
            return Err(ProfileError::InvalidRecord);
        }
        options.validate_grid(spec)?;
        let mut entries = Vec::new();
        entries
            .try_reserve_exact(count)
            .map_err(|_| Error::AllocationFailed)?;
        for chunk in header.as_chunks::<ENTRY>().0 {
            let mut entry = chunk.as_slice();
            let key = TileKey {
                x: i32::from_le_bytes(take(&mut entry)?),
                z: i32::from_le_bytes(take(&mut entry)?),
            };
            if !spec.grid.contains(key) || entries.last().is_some_and(|&(last, _)| last >= key) {
                return Err(ProfileError::InvalidRecord);
            }
            let present = take::<1>(&mut entry)?[0];
            let raw = take(&mut entry)?;
            let prepared = take(&mut entry)?;
            let polys = u32::from_le_bytes(take(&mut entry)?);
            let payload = match present {
                0 if raw == [0; 32] && prepared == [0; 32] && polys == 0 => None,
                1 if polys > 0 => Some(Payload {
                    raw,
                    prepared,
                    polys,
                }),
                _ => return Err(ProfileError::InvalidRecord),
            };
            entries.push((key, payload));
        }
        Ok(Self {
            revision,
            spec,
            options,
            descriptor,
            digest: content_hash(bytes),
            entries: Arc::new(entries),
        })
    }

    pub(super) fn compatible_layout(&self, next: &Self) -> bool {
        descriptor(self.spec, self.options, false) == descriptor(next.spec, next.options, false)
    }
}

/// Assembles a catalog from completed, revision-bound profile bake results.
pub struct ProfileCatalogBuilder {
    revision: ProfileRevision,
    spec: ProfileBakeSpec,
    options: ProfileMeshOptions,
    nav: NavMesh,
    entries: BTreeMap<TileKey, Option<Payload>>,
    updated: BTreeSet<TileKey>,
}

impl ProfileCatalogBuilder {
    /// Starts a catalog for one revision and validates its runtime layout.
    pub fn new(
        revision: ProfileRevision,
        spec: ProfileBakeSpec,
        options: ProfileMeshOptions,
    ) -> Result<Self, ProfileError> {
        Ok(Self {
            revision,
            spec,
            options,
            nav: options.nav(spec)?,
            entries: BTreeMap::new(),
            updated: BTreeSet::new(),
        })
    }

    /// Starts a newer revision with entries carried over from a compatible
    /// catalog. Supply every changed column, including newly empty columns,
    /// to the returned builder. Columns outside the new grid are dropped.
    pub fn revise(
        current: &ProfileCatalog,
        revision: ProfileRevision,
        spec: ProfileBakeSpec,
    ) -> Result<Self, ProfileError> {
        if current.revision.map != revision.map
            || current.revision.revision >= revision.revision
            || current.spec.id != spec.id
            || descriptor(current.spec, current.options, false)
                != descriptor(spec, current.options, false)
        {
            return Err(ProfileError::BindingMismatch);
        }
        let mut builder = Self::new(revision, spec, current.options)?;
        builder.entries.extend(
            current
                .entries
                .iter()
                .copied()
                .filter(|&(key, _)| spec.grid.contains(key)),
        );
        Ok(builder)
    }

    /// Publishes the hashes or explicit empty marker for a completed tile.
    /// The original tile bytes are still owned by the bake result.
    pub fn insert(&mut self, output: &ProfileTileOutput) -> Result<(), ProfileError> {
        if output.revision != self.revision
            || output.spec.id != self.spec.id
            || descriptor(output.spec, self.options, true)
                != descriptor(self.spec, self.options, true)
        {
            return Err(ProfileError::BindingMismatch);
        }
        let key = output.tile.key;
        if self.updated.contains(&key) {
            return Err(ProfileError::DuplicateTile(key));
        }
        let payload = if let Some(bytes) = &output.tile.data {
            let tile = self.nav.prepare_for(bytes.clone())?;
            if tile.key().x != key.x || tile.key().y != key.z || tile.layer() != 0 {
                return Err(ProfileError::TileMismatch);
            }
            Some(Payload {
                raw: content_hash(bytes),
                prepared: tile.content_hash(),
                polys: tile.tile().polys.len() as u32,
            })
        } else {
            None
        };
        self.updated.insert(key);
        self.entries.insert(key, payload);
        Ok(())
    }

    /// Seals the entries and their versioned catalog identity.
    pub fn finish(self) -> ProfileCatalog {
        let mut catalog = ProfileCatalog {
            revision: self.revision,
            spec: self.spec,
            options: self.options,
            descriptor: descriptor(self.spec, self.options, true),
            digest: [0; 32],
            entries: Arc::new(self.entries.into_iter().collect()),
        };
        catalog.digest = content_hash(&catalog.to_bytes());
        catalog
    }
}

// Bind the effective bake settings, using the original float bits. The Recast
// dimensions/bounds and border are derived from the grid and are encoded there.
fn descriptor(spec: ProfileBakeSpec, options: ProfileMeshOptions, include_size: bool) -> [u8; 32] {
    let BakeConfig {
        recast,
        partition,
        filter_low_hanging,
        filter_ledge,
        filter_low_height,
        walkable_height,
        walkable_radius,
        walkable_climb,
        tile_limits,
        max_total_bytes,
        strict_triangulation,
        fixes,
        links,
    } = spec.config;
    let Config {
        width: _,
        height: _,
        tile_size,
        border_size: _,
        cs,
        ch,
        bmin: _,
        bmax: _,
        walkable_slope_angle,
        walkable_height: height,
        walkable_climb: climb,
        walkable_radius: radius,
        max_edge_len,
        max_simplification_error,
        min_region_area,
        merge_region_area,
        max_verts_per_poly,
        detail_sample_dist,
        detail_sample_max_error,
    } = recast;
    let grid = spec.grid;
    let mut bytes = Vec::new();
    bytes.extend(b"NPDS");
    bytes.extend(VERSION.to_le_bytes());
    bytes.extend(BAKE_REVISION.to_le_bytes());
    bytes.extend(PolyRef::BITS.to_le_bytes());
    for value in [
        tile_size,
        height,
        climb,
        radius,
        max_edge_len,
        min_region_area,
        merge_region_area,
        max_verts_per_poly,
        grid.halo_cells,
        options.max_tiles,
        options.max_polys,
    ] {
        bytes.extend(value.to_le_bytes());
    }
    if include_size {
        bytes.extend(grid.tiles_x.to_le_bytes());
        bytes.extend(grid.tiles_z.to_le_bytes());
    }
    for value in [
        cs,
        ch,
        walkable_slope_angle,
        max_simplification_error,
        detail_sample_dist,
        detail_sample_max_error,
        walkable_height,
        walkable_radius,
        walkable_climb,
        grid.origin[0],
        grid.origin[1],
        grid.origin[2],
        grid.max_y,
        grid.tile_extent,
        grid.halo,
    ] {
        bytes.extend(value.to_bits().to_le_bytes());
    }
    bytes.extend([
        match partition {
            Partition::Watershed => 0,
            Partition::Monotone => 1,
            Partition::Layers => 2,
        },
        u8::from(filter_low_hanging),
        u8::from(filter_ledge),
        u8::from(filter_low_height),
        u8::from(strict_triangulation),
        u8::from(options.global_off_mesh_links),
    ]);
    // Unlimited is the same setting on both pointer widths.
    let limit = |n: usize| if n == usize::MAX { u64::MAX } else { n as u64 };
    for value in [
        limit(tile_limits.max_bytes),
        tile_limits.max_work,
        limit(tile_limits.max_tile_bytes),
        limit(tile_limits.max_tile_polys),
        limit(tile_limits.max_tile_verts),
        limit(tile_limits.max_tile_links),
        limit(tile_limits.max_tile_detail),
        limit(max_total_bytes),
        limit(options.limits.max_bytes),
        options.limits.max_work,
        limit(options.limits.max_tile_bytes),
        limit(options.limits.max_tile_polys),
        limit(options.limits.max_tile_verts),
        limit(options.limits.max_tile_links),
        limit(options.limits.max_tile_detail),
    ] {
        bytes.extend(value.to_le_bytes());
    }
    bytes.extend(fixes.bits().to_le_bytes());
    bytes.push(u8::from(links.is_some()));
    if let Some(links) = links {
        bytes.extend(links.spacing_cells.to_le_bytes());
        bytes.extend(links.area_ids);
        for flag in links.flags {
            bytes.extend(flag.to_le_bytes());
        }
        for kind in [links.drop, links.jump, links.climb] {
            bytes.push(u8::from(kind.is_some()));
            if let Some(kind) = kind {
                for value in [kind.distance, kind.up, kind.down, kind.clearance] {
                    bytes.extend(value.to_bits().to_le_bytes());
                }
            }
        }
    }
    match options.ref_layout {
        None => bytes.push(0),
        Some(layout) => bytes.extend([1, layout.salt_bits, layout.tile_bits, layout.poly_bits]),
    }
    match options.slot_policy {
        SlotPolicy::Sequential => bytes.push(0),
        SlotPolicy::Keyed {
            origin,
            width,
            height,
            max_layers,
        } => {
            bytes.push(1);
            for axis in origin {
                bytes.extend(axis.to_le_bytes());
            }
            for value in [width, height, max_layers] {
                bytes.extend(value.to_le_bytes());
            }
        }
    }
    content_hash(&bytes)
}

pub(super) fn take<const N: usize>(bytes: &mut &[u8]) -> Result<[u8; N], ProfileError> {
    let (head, tail) = bytes
        .split_at_checked(N)
        .ok_or(ProfileError::InvalidRecord)?;
    *bytes = tail;
    head.try_into().map_err(|_| ProfileError::InvalidRecord)
}

pub(super) fn read_version(
    mut bytes: &[u8],
    magic: [u8; 4],
    expected: u32,
) -> Result<&[u8], ProfileError> {
    if take::<4>(&mut bytes)? != magic {
        return Err(ProfileError::InvalidRecord);
    }
    let version = u32::from_le_bytes(take(&mut bytes)?);
    if version != expected {
        return Err(ProfileError::UnsupportedVersion(version));
    }
    Ok(bytes)
}
