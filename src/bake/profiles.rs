//! Independent agent-size bakes and profile-qualified navigation meshes.
//!
//! Applications select a profile at admission and keep the source immutable
//! for the declared revision. Each profile has a separate tile stream and mesh.

use alloc::vec::Vec;

use super::{
    BakeConfig, BakeError, BakeJob, BakeManifest, BakeReport, GeometrySource, GridLayout,
    JobOptions, ManifestDelta, TileKey, TileOutput,
};
use crate::{Budget, CancelToken, Error, Progress, Workspace};

mod catalog;
#[cfg(feature = "crowd")]
mod crowds;
mod runtime;

pub use catalog::{CatalogTile, ProfileCatalog, ProfileCatalogBuilder, ProfileMeshOptions};
#[cfg(feature = "crowd")]
pub use crowds::{ProfileAgent, ProfileCrowd};
#[cfg(feature = "crowd-world")]
pub use crowds::{ProfileCrowdWorld, ProfileWorldAgent};
pub use runtime::{
    ProfileConnectivity, ProfileCorner, ProfileMesh, ProfileMeshes, ProfilePoint, ProfilePolyRef,
    ProfilePreparedTile, ProfileQuery, ProfileTileRef,
};

/// A stable, application-assigned agent profile identifier.
#[derive(Clone, Copy, Debug, PartialEq, Eq, PartialOrd, Ord, Hash)]
#[non_exhaustive]
pub struct ProfileId(pub u32);

impl ProfileId {
    /// Assigns a stable numeric ID.
    pub const fn new(id: u32) -> Self {
        Self(id)
    }
}

/// The application-assigned map and immutable revision of its source/catalog.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
#[non_exhaustive]
pub struct ProfileRevision {
    /// Stable ID of the navigation map.
    pub map: [u8; 16],
    /// Revision of the map's authored source and asset catalog.
    pub revision: u64,
}

impl ProfileRevision {
    /// Creates an address namespace for one revision of a map.
    pub const fn new(map: [u8; 16], revision: u64) -> Self {
        Self { map, revision }
    }
}

/// An error at a profile bake, catalog, or consumer boundary.
#[non_exhaustive]
#[derive(Clone, Debug, PartialEq, Eq)]
pub enum ProfileError {
    /// The requested profile is absent.
    UnknownProfile(ProfileId),
    /// This ID is already registered in the map.
    DuplicateProfile(ProfileId),
    /// A tile, reference or saved record belongs to another binding.
    BindingMismatch,
    /// The bake supplied more than one result for the same tile.
    DuplicateTile(TileKey),
    /// Tile bytes or their coordinates differ from the catalog entry.
    TileMismatch,
    /// A profile catalog or save has an unsupported version.
    UnsupportedVersion(u32),
    /// A profile catalog or save record is malformed.
    InvalidRecord,
    /// The underlying bake failed.
    Bake(BakeError),
    /// The underlying runtime or configuration validation failed.
    Navigation(Error),
}

impl From<BakeError> for ProfileError {
    fn from(value: BakeError) -> Self {
        Self::Bake(value)
    }
}

impl From<Error> for ProfileError {
    fn from(value: Error) -> Self {
        Self::Navigation(value)
    }
}

impl core::fmt::Display for ProfileError {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        match self {
            Self::UnknownProfile(id) => write!(f, "unknown profile {}", id.0),
            Self::DuplicateProfile(id) => write!(f, "duplicate profile {}", id.0),
            Self::BindingMismatch => f.write_str("profile binding mismatch"),
            Self::DuplicateTile(key) => write!(f, "duplicate profile tile ({}, {})", key.x, key.z),
            Self::TileMismatch => f.write_str("tile does not match its catalog entry"),
            Self::UnsupportedVersion(version) => write!(f, "unsupported profile format {version}"),
            Self::InvalidRecord => f.write_str("invalid profile record"),
            Self::Bake(error) => error.fmt(f),
            Self::Navigation(error) => error.fmt(f),
        }
    }
}

#[cfg(feature = "std")]
impl std::error::Error for ProfileError {}

/// The checked bake settings and layout for one agent profile.
#[derive(Clone, Copy, Debug)]
#[non_exhaustive]
pub struct ProfileBakeSpec {
    id: ProfileId,
    config: BakeConfig,
    grid: GridLayout,
}

impl ProfileBakeSpec {
    /// Validates the settings and exact grid. Voxel and world agent dimensions
    /// are supplied separately, as in the standalone pipeline.
    pub fn new(id: ProfileId, config: BakeConfig, grid: GridLayout) -> Result<Self, Error> {
        super::validate_config(&config)?;
        if config.recast.walkable_radius > 127 {
            return Err(Error::InvalidParameter(
                "profile erosion radius exceeds 127 cells",
            ));
        }
        grid.validate(&config)?;
        grid.tile_config(&config, TileKey { x: 0, z: 0 })?;
        grid.tile_config(
            &config,
            TileKey {
                x: grid.tiles_x - 1,
                z: grid.tiles_z - 1,
            },
        )?;
        Ok(Self { id, config, grid })
    }

    /// The stable profile ID.
    pub fn id(&self) -> ProfileId {
        self.id
    }

    /// Exact standalone bake settings.
    pub fn config(&self) -> &BakeConfig {
        &self.config
    }

    /// The native or explicitly padded grid.
    pub fn grid(&self) -> &GridLayout {
        &self.grid
    }

    /// Starts an independent bake for this profile, including incremental key sets.
    /// Drive jobs in ascending profile ID order for a canonical profile-major stream.
    pub fn bake<'s, S: GeometrySource>(
        self,
        source: &'s S,
        revision: ProfileRevision,
        keys: Vec<TileKey>,
        options: JobOptions,
    ) -> Result<ProfileBakeJob<'s, S>, BakeError> {
        Ok(ProfileBakeJob {
            spec: self,
            revision,
            job: BakeJob::new(source, &self.config, &self.grid, keys, options)?,
        })
    }
}

/// A standalone bake scoped to an immutable source revision and agent profile.
pub struct ProfileBakeJob<'s, S: GeometrySource> {
    spec: ProfileBakeSpec,
    revision: ProfileRevision,
    job: BakeJob<'s, S>,
}

impl<S: GeometrySource> ProfileBakeJob<'_, S> {
    /// Trims unchanged keys using the profile's previous manifest.
    pub fn skip_unchanged(
        &mut self,
        previous: &BakeManifest,
        workspace: &mut Workspace,
    ) -> Result<bool, BakeError> {
        self.job.skip_unchanged(previous, workspace)
    }

    /// Advances the standalone stages within its ordinary work allowance.
    pub fn step(&mut self, budget: &mut Budget) -> Result<Progress, BakeError> {
        self.job.step(budget)
    }

    /// Advances a standalone chunk and preserves row-major delivery.
    pub fn run_parallel_chunk(&mut self) -> Result<Progress, BakeError> {
        self.job.run_parallel_chunk()
    }

    /// Cancels this profile's job without affecting other profiles.
    pub fn cancel_token(&self) -> CancelToken {
        self.job.cancel_token()
    }

    /// Takes a complete result with its bake provenance.
    pub fn next_ready(&mut self) -> Option<ProfileTileOutput> {
        self.job.next_ready().map(|tile| ProfileTileOutput {
            spec: self.spec,
            revision: self.revision,
            tile,
        })
    }

    /// Entries for this profile returned so far.
    pub fn manifest_delta(&self) -> ManifestDelta {
        self.job.manifest_delta()
    }

    /// Telemetry for completed tiles if requested in the job options.
    pub fn report(&self) -> &BakeReport {
        self.job.report()
    }
}

/// A complete tile result tagged with its bake settings and source revision.
#[derive(Debug)]
#[non_exhaustive]
pub struct ProfileTileOutput {
    spec: ProfileBakeSpec,
    revision: ProfileRevision,
    tile: TileOutput,
}

impl ProfileTileOutput {
    /// The result's profile.
    pub fn profile(&self) -> ProfileId {
        self.spec.id
    }

    /// The source revision this result was baked from.
    pub fn revision(&self) -> ProfileRevision {
        self.revision
    }

    /// The standalone result, including the unmodified DNAV bytes.
    pub fn tile(&self) -> &TileOutput {
        &self.tile
    }

    /// Takes the ordinary standalone tile result.
    pub fn into_tile(self) -> TileOutput {
        self.tile
    }
}
