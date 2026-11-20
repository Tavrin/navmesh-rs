//! Moving navmesh frames and dock routes.
//!
//! Frame zero is the static world. Platform meshes, polygon refs and routes
//! stay in local coordinates while the caller updates platform poses.

use alloc::boxed::Box;
use alloc::collections::BTreeMap;
use alloc::vec::Vec;

use crate::Error;
#[cfg(all(feature = "recast", feature = "crowd-world"))]
use crate::bake::profiles::{ProfileId, ProfileMeshes};
use crate::detour::{DefaultFilter, LinkId, NavMesh, NavMeshQuery, PolyRef};

mod route;
mod snapshot;
#[cfg(feature = "crowd-world")]
pub(crate) use route::ActiveSeam;
#[cfg(all(feature = "recast", feature = "crowd-world"))]
pub(crate) use route::FrameRouting;
pub use route::{FrameLeg, FramePlan};

type Point = [f32; 3];

/// Stable identity of a navmesh coordinate frame.
#[derive(Clone, Copy, Debug, PartialEq, Eq, PartialOrd, Ord, Hash)]
pub struct FrameId(pub u32);

impl FrameId {
    /// The fixed world frame.
    pub const WORLD: Self = Self(0);
}

/// A yaw-only rigid transform from frame-local coordinates to world coordinates.
#[derive(Clone, Copy, Debug, PartialEq)]
#[non_exhaustive]
pub struct FramePose {
    /// World location of the local origin.
    pub origin: Point,
    /// Cosine of the yaw angle.
    pub cos_yaw: f32,
    /// Sine of the yaw angle.
    pub sin_yaw: f32,
}

impl FramePose {
    /// Identity pose.
    pub const IDENTITY: Self = Self {
        origin: [0.; 3],
        cos_yaw: 1.,
        sin_yaw: 0.,
    };

    /// Builds a finite pose with a unit yaw pair (squared norm within 1e-6).
    pub fn new(origin: Point, cos_yaw: f32, sin_yaw: f32) -> Result<Self, Error> {
        let pose = Self {
            origin,
            cos_yaw,
            sin_yaw,
        };
        pose.validate()?;
        Ok(pose)
    }

    fn validate(self) -> Result<(), Error> {
        validate_point(self.origin)?;
        let (c, s) = self.yaw();
        if (c * c + s * s - 1.).abs() <= 1e-6 {
            Ok(())
        } else {
            Err(Error::InvalidParameter("frame yaw"))
        }
    }

    fn yaw(self) -> (f64, f64) {
        (f64::from(self.cos_yaw), f64::from(self.sin_yaw))
    }

    fn world(self, [x, y, z]: [f64; 3]) -> [f64; 3] {
        let (c, s) = self.yaw();
        let [ox, oy, oz] = self.origin.map(f64::from);
        [ox + (c * x + s * z), oy + y, oz + (c * z - s * x)]
    }

    fn local(self, p: [f64; 3]) -> [f64; 3] {
        let [ox, oy, oz] = self.origin.map(f64::from);
        let [x, y, z] = [p[0] - ox, p[1] - oy, p[2] - oz];
        let (c, s) = self.yaw();
        let norm = c * c + s * s;
        [(c * x - s * z) / norm, y, (c * z + s * x) / norm]
    }
}

/// A point in a specific mesh's local coordinates.
#[derive(Clone, Copy, Debug, PartialEq)]
#[non_exhaustive]
pub struct FramePoint {
    /// Frame containing the point.
    pub frame: FrameId,
    /// Frame-local position.
    pub position: Point,
}

impl FramePoint {
    /// Pairs a frame identity with a local position.
    pub const fn new(frame: FrameId, position: Point) -> Self {
        Self { frame, position }
    }
}

/// A stable runtime connection between two different coordinate frames.
#[derive(Clone, Copy, Debug, PartialEq)]
#[non_exhaustive]
pub struct DockSpec {
    /// Stable id in the frame set's dock registry.
    pub id: LinkId,
    /// Departure side of the forward crossing.
    pub a: FramePoint,
    /// Arrival side of the forward crossing.
    pub b: FramePoint,
    /// Three-dimensional radius used to snap each authored endpoint to ground.
    pub radius: f32,
    /// Whether the crossing is also available from B to A.
    pub bidirectional: bool,
}

impl DockSpec {
    /// Constructs a directed dock. Set `bidirectional` for two-way traffic.
    pub const fn new(id: LinkId, a: FramePoint, b: FramePoint, radius: f32) -> Self {
        Self {
            id,
            a,
            b,
            radius,
            bidirectional: false,
        }
    }
}

enum FrameNav {
    Native(Box<NavMesh>),
    #[cfg(all(feature = "recast", feature = "crowd-world"))]
    Profiles(ProfileMeshes),
}

struct Frame {
    nav: FrameNav,
    pose: FramePose,
}

#[derive(Clone, Copy)]
struct Dock {
    spec: DockSpec,
    enabled: bool,
}

#[derive(Clone, Copy, Debug, PartialEq)]
struct OnMesh {
    point: FramePoint,
    poly: PolyRef,
}

/// The world mesh, platform meshes, caller-supplied poses and runtime docks.
pub struct FrameSet {
    frames: BTreeMap<FrameId, Frame>,
    docks: BTreeMap<LinkId, Dock>,
}

impl FrameSet {
    /// Takes ownership of the static world mesh.
    pub fn new(world: NavMesh) -> Self {
        Self::with_world(FrameNav::Native(Box::new(world)))
    }

    /// Creates a world frame with independently baked profile meshes.
    #[cfg(all(feature = "recast", feature = "crowd-world"))]
    pub fn with_profiles(world: ProfileMeshes) -> Self {
        Self::with_world(FrameNav::Profiles(world))
    }

    fn with_world(world: FrameNav) -> Self {
        Self {
            frames: BTreeMap::from([(
                FrameId::WORLD,
                Frame {
                    nav: world,
                    pose: FramePose::IDENTITY,
                },
            )]),
            docks: BTreeMap::new(),
        }
    }

    /// Installs a platform under an unused, nonzero frame id.
    pub fn add_frame(&mut self, id: FrameId, nav: NavMesh, pose: FramePose) -> Result<(), Error> {
        self.insert_frame(id, FrameNav::Native(Box::new(nav)), pose)
    }

    /// Installs a platform holding its own catalog-bound profile meshes.
    #[cfg(all(feature = "recast", feature = "crowd-world"))]
    pub fn add_profile_frame(
        &mut self,
        id: FrameId,
        meshes: ProfileMeshes,
        pose: FramePose,
    ) -> Result<(), Error> {
        self.insert_frame(id, FrameNav::Profiles(meshes), pose)
    }

    fn insert_frame(&mut self, id: FrameId, nav: FrameNav, pose: FramePose) -> Result<(), Error> {
        pose.validate()?;
        if self.frames.contains_key(&id) {
            return Err(Error::InvalidParameter("duplicate frame id"));
        }
        self.frames.insert(id, Frame { nav, pose });
        Ok(())
    }

    /// Reads the mesh in a frame. Polygon refs are scoped to that mesh.
    pub fn mesh(&self, id: FrameId) -> Option<&NavMesh> {
        match &self.frames.get(&id)?.nav {
            FrameNav::Native(nav) => Some(nav),
            #[cfg(all(feature = "recast", feature = "crowd-world"))]
            FrameNav::Profiles(_) => None,
        }
    }

    /// Updates a frame's resident tiles and its ordinary runtime off-mesh links.
    pub fn mesh_mut(&mut self, id: FrameId) -> Option<&mut NavMesh> {
        match &mut self.frames.get_mut(&id)?.nav {
            FrameNav::Native(nav) => Some(nav),
            #[cfg(all(feature = "recast", feature = "crowd-world"))]
            FrameNav::Profiles(_) => None,
        }
    }

    /// Reads the catalog-bound meshes in a profile frame.
    #[cfg(all(feature = "recast", feature = "crowd-world"))]
    pub fn profile_meshes(&self, id: FrameId) -> Option<&ProfileMeshes> {
        match &self.frames.get(&id)?.nav {
            FrameNav::Profiles(meshes) => Some(meshes),
            FrameNav::Native(_) => None,
        }
    }

    /// Updates a profile frame's catalogs, residency and modifiers.
    #[cfg(all(feature = "recast", feature = "crowd-world"))]
    pub fn profile_meshes_mut(&mut self, id: FrameId) -> Option<&mut ProfileMeshes> {
        match &mut self.frames.get_mut(&id)?.nav {
            FrameNav::Profiles(meshes) => Some(meshes),
            FrameNav::Native(_) => None,
        }
    }

    /// The registered frame ids, in ascending order.
    pub fn frame_ids(&self) -> impl Iterator<Item = FrameId> + '_ {
        self.frames.keys().copied()
    }

    /// Reads the frame's current pose.
    pub fn pose(&self, id: FrameId) -> Option<FramePose> {
        Some(self.frames.get(&id)?.pose)
    }

    /// Sets the absolute pose for the next platform tick.
    pub fn set_pose(&mut self, id: FrameId, pose: FramePose) -> Result<(), Error> {
        pose.validate()?;
        if id == FrameId::WORLD {
            return Err(Error::InvalidParameter("world frame is fixed"));
        }
        self.frames
            .get_mut(&id)
            .ok_or(Error::InvalidParameter("unknown frame"))?
            .pose = pose;
        Ok(())
    }

    /// Converts a local point to world space. Returns `None` for an unknown
    /// frame or an input or result outside the finite coordinate range.
    pub fn to_world(&self, p: FramePoint) -> Option<Point> {
        finite_result(self.pose(p.frame)?.world(p.position.map(f64::from)))
    }

    /// Converts a world point to local coordinates in the selected frame.
    pub fn to_local(&self, frame: FrameId, world: Point) -> Option<FramePoint> {
        let position = finite_result(self.pose(frame)?.local(world.map(f64::from)))?;
        Some(FramePoint::new(frame, position))
    }

    /// Finds a ground polygon near a world-space point in the selected frame.
    ///
    /// `half_extents` is the search box in local axes. The result holds the
    /// local polygon ref and its snapped position converted back to world space.
    pub fn find_nearest_world(
        &self,
        frame: FrameId,
        world: Point,
        half_extents: Point,
        filter: &DefaultFilter,
    ) -> Result<Option<(PolyRef, Point)>, Error> {
        let p = self
            .to_local(frame, world)
            .ok_or(Error::InvalidParameter("frame point"))?;
        let query = NavMeshQuery::new(self.require_mesh(frame)?, 1)?;
        let (poly, p, _) = query.find_nearest_poly(&p.position, &half_extents, filter)?;
        if poly == 0 {
            return Ok(None);
        }
        let world = self
            .to_world(FramePoint::new(frame, p))
            .ok_or(Error::InvalidParameter("frame point"))?;
        Ok(Some((poly, world)))
    }

    /// Registers a dock and its initial availability. Its landings are resolved
    /// against the resident ground on each route query or transfer.
    pub fn add_dock(&mut self, spec: DockSpec, enabled: bool) -> Result<(), Error> {
        self.validate_dock(spec)?;
        if self.docks.contains_key(&spec.id) {
            return Err(Error::InvalidParameter("duplicate dock id"));
        }
        self.docks.insert(spec.id, Dock { spec, enabled });
        Ok(())
    }

    fn validate_dock(&self, spec: DockSpec) -> Result<(), Error> {
        if spec.a.frame == spec.b.frame || !spec.radius.is_finite() || spec.radius <= 0. {
            return Err(Error::InvalidParameter("dock geometry"));
        }
        for p in [spec.a, spec.b] {
            validate_point(p.position)?;
            if !self.frames.contains_key(&p.frame) {
                return Err(Error::InvalidParameter("unknown frame"));
            }
        }
        Ok(())
    }

    /// Updates availability before the next query or crowd step.
    pub fn set_dock_enabled(&mut self, id: LinkId, enabled: bool) -> Result<(), Error> {
        self.docks
            .get_mut(&id)
            .ok_or(Error::InvalidParameter("unknown dock"))?
            .enabled = enabled;
        Ok(())
    }

    /// Reads the authored spec and caller-controlled availability.
    pub fn dock(&self, id: LinkId) -> Option<(DockSpec, bool)> {
        let d = self.docks.get(&id)?;
        Some((d.spec, d.enabled))
    }

    /// Removes a dock from the frame graph.
    pub fn remove_dock(&mut self, id: LinkId) -> Option<DockSpec> {
        Some(self.docks.remove(&id)?.spec)
    }

    #[cfg(feature = "crowd-world")]
    pub(crate) fn vector_between(&self, from: FrameId, to: FrameId, v: Point) -> Option<Point> {
        let (a, b) = self.pose(from)?.yaw();
        let (c, d) = self.pose(to)?.yaw();
        let [x, y, z] = v.map(f64::from);
        let (wx, wz) = (a * x + b * z, a * z - b * x);
        let norm = c * c + d * d;
        finite_result([(c * wx - d * wz) / norm, y, (c * wz + d * wx) / norm])
    }

    #[cfg(feature = "crowd-world")]
    pub(crate) fn point_between(&self, p: FramePoint, to: FrameId) -> Option<Point> {
        finite_result(self.pose(to)?.local(self.world_wide(p).ok()?))
    }

    pub(crate) fn require_mesh(&self, id: FrameId) -> Result<&NavMesh, Error> {
        self.mesh(id)
            .ok_or(Error::InvalidParameter("unknown frame"))
    }

    fn world_wide(&self, p: FramePoint) -> Result<[f64; 3], Error> {
        let pose = self
            .pose(p.frame)
            .ok_or(Error::InvalidParameter("unknown frame"))?;
        Ok(pose.world(p.position.map(f64::from)))
    }
}

fn validate_point(p: Point) -> Result<(), Error> {
    if p.iter().all(|x| x.is_finite()) {
        Ok(())
    } else {
        Err(Error::InvalidParameter("non-finite frame point"))
    }
}

fn finite_result(p: [f64; 3]) -> Option<Point> {
    let p = p.map(|v| v as f32);
    validate_point(p).ok().map(|()| p)
}

fn distance_squared(a: [f64; 3], b: [f64; 3]) -> f64 {
    a.into_iter().zip(b).map(|(a, b)| (a - b) * (a - b)).sum()
}

// A dock is a seam between surfaces. Only rounding and small snapping error
// may displace the agent across it.
const SEAM_EPSILON: f64 = 0.001;
