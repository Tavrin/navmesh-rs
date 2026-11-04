//! Deterministic geometry providers for the bake job (not in recastnavigation).

use alloc::vec::Vec;

use super::Input;
use super::manifest::Fnv64;
use crate::Error;
use crate::common::{fmax, fmin};
use crate::recast;

/// Identifies a triangle within a [`GeometrySource`]; fetches return ascending ids.
pub type TriangleId = u64;

/// Identifies an area volume or an off-mesh connection within a [`GeometrySource`].
pub type ItemId = u64;

/// A deterministic provider of the geometry a tile depends on (not in recastnavigation).
///
/// The bake job asks for one box at a time, the tile's bordered column, so the
/// whole world never has to be in memory. The contract that makes incremental
/// rebakes exact:
///
/// - every method returns the same items in the same order for the same box and
///   the same world state;
/// - items arrive in ascending id order (ids need not be contiguous, and the same
///   id keeps the same item across calls); the sink rejects an id that does not
///   exceed the previous one;
/// - a method may push items that lie outside the box; the sink drops them. A box
///   overlaps an item when their xz bounds touch or intersect (inclusive); the
///   height range of the box is not used.
///
/// A source that breaks the ordering rule is rejected with
/// [`Error::InvalidInput`]; one that returns different items for the same state
/// produces wrong input hashes (stale tiles after an incremental rebake) but never
/// undefined behaviour.
///
/// [`Input`] implements this trait, with the selection and order of
/// [`tiles`](super::tiles), so a borrowed mesh is a source too.
pub trait GeometrySource: Sync {
    /// Pushes every triangle whose xz bounds overlap `[bmin, bmax]`.
    fn triangles(
        &self,
        bmin: [f32; 3],
        bmax: [f32; 3],
        out: &mut TriangleSink,
    ) -> Result<(), Error>;

    /// Pushes every convex area volume whose xz bounds overlap `[bmin, bmax]`,
    /// in the order they are applied (later volumes overwrite earlier ones).
    fn volumes(&self, bmin: [f32; 3], bmax: [f32; 3], out: &mut VolumeSink) -> Result<(), Error>;

    /// Pushes every off-mesh connection with an endpoint inside `[bmin, bmax]` (xz).
    fn off_mesh(&self, bmin: [f32; 3], bmax: [f32; 3], out: &mut OffMeshSink) -> Result<(), Error>;
}

/// The xz query box shared by the three sinks, and the memory cap of one fetch.
#[derive(Clone, Copy, Debug)]
struct Window {
    lo: [f32; 2],
    hi: [f32; 2],
    cap: usize,
}

impl Window {
    fn new(bmin: [f32; 3], bmax: [f32; 3], cap: usize) -> Self {
        Self {
            lo: [bmin[0], bmin[2]],
            hi: [bmax[0], bmax[2]],
            cap,
        }
    }

    fn overlaps(&self, lo: [f32; 2], hi: [f32; 2]) -> bool {
        lo[0] <= self.hi[0] && hi[0] >= self.lo[0] && lo[1] <= self.hi[1] && hi[1] >= self.lo[1]
    }

    fn contains(&self, p: [f32; 2]) -> bool {
        self.overlaps(p, p)
    }
}

fn ascending(last: &mut Option<ItemId>, id: ItemId, what: &'static str) -> Result<(), Error> {
    if last.is_some_and(|l| id <= l) {
        return Err(Error::InvalidInput(what));
    }
    *last = Some(id);
    Ok(())
}

fn charge(used: &mut usize, cap: usize, bytes: usize) -> Result<(), Error> {
    let total = used.checked_add(bytes).ok_or(Error::SizeOverflow)?;
    if total > cap {
        return Err(Error::LimitExceeded);
    }
    *used = total;
    Ok(())
}

fn reserve<T>(v: &mut Vec<T>, n: usize) -> Result<(), Error> {
    v.try_reserve(n).map_err(|_| Error::AllocationFailed)
}

fn finite(values: &[f32]) -> bool {
    values.iter().all(|v| v.is_finite())
}

/// Receives the triangles of one fetch (not in recastnavigation).
///
/// Created by the bake job; a [`GeometrySource`] only pushes into it.
#[derive(Debug)]
pub struct TriangleSink {
    window: Window,
    verts: Vec<f32>,
    areas: Vec<u8>,
    auto: Vec<bool>,
    last: Option<TriangleId>,
    used: usize,
}

impl TriangleSink {
    pub(crate) fn new(bmin: [f32; 3], bmax: [f32; 3], cap_bytes: usize) -> Self {
        Self {
            window: Window::new(bmin, bmax, cap_bytes),
            verts: Vec::new(),
            areas: Vec::new(),
            auto: Vec::new(),
            last: None,
            used: 0,
        }
    }

    /// Adds a triangle.
    ///
    /// `area` is the Recast area id (0..=63) the triangle keeps when its slope is
    /// walkable; `None` marks it walkable by slope like a mesh given without areas.
    /// Triangles outside the query box are dropped silently.
    ///
    /// # Errors
    /// [`Error::InvalidInput`] for an id not above the previous one, a non-finite
    /// vertex or an area above 63; [`Error::LimitExceeded`] when the fetch would
    /// exceed the tile byte limit.
    pub fn push(
        &mut self,
        id: TriangleId,
        tri: [[f32; 3]; 3],
        area: Option<u8>,
    ) -> Result<(), Error> {
        ascending(&mut self.last, id, "triangle ids must ascend")?;
        if area.is_some_and(|a| a > 63) || tri.iter().any(|v| !finite(v)) {
            return Err(Error::InvalidInput("triangle"));
        }
        let mut lo = [f32::INFINITY; 2];
        let mut hi = [f32::NEG_INFINITY; 2];
        for v in &tri {
            lo[0] = fmin(lo[0], v[0]);
            hi[0] = fmax(hi[0], v[0]);
            lo[1] = fmin(lo[1], v[2]);
            hi[1] = fmax(hi[1], v[2]);
        }
        if !self.window.overlaps(lo, hi) {
            return Ok(());
        }
        charge(&mut self.used, self.window.cap, 9 * 4 + 2)?;
        reserve(&mut self.verts, 9)?;
        reserve(&mut self.areas, 1)?;
        reserve(&mut self.auto, 1)?;
        for v in &tri {
            self.verts.extend_from_slice(v);
        }
        self.areas.push(area.unwrap_or(0));
        self.auto.push(area.is_none());
        Ok(())
    }

    /// Number of triangles kept so far.
    pub fn len(&self) -> usize {
        self.areas.len()
    }

    /// Whether no triangle has been kept.
    pub fn is_empty(&self) -> bool {
        self.areas.is_empty()
    }

    pub(super) fn digest(&self, h: &mut Fnv64) {
        h.u64(self.areas.len() as u64);
        for (i, v) in self.verts.as_chunks::<9>().0.iter().enumerate() {
            for &f in v {
                h.u32(f.to_bits());
            }
            h.u8(if self.auto[i] { 0xff } else { self.areas[i] });
        }
    }

    pub(super) fn into_parts(self) -> (Vec<f32>, Vec<u8>, super::task::AreaMode) {
        let mode = if self.auto.iter().all(|&a| a) {
            super::task::AreaMode::Auto
        } else if self.auto.iter().all(|&a| !a) {
            super::task::AreaMode::Explicit
        } else {
            super::task::AreaMode::Mixed(self.auto)
        };
        (self.verts, self.areas, mode)
    }
}

/// One convex area volume as stored by a [`VolumeSink`].
#[derive(Clone, Debug)]
pub(super) struct OwnedVolume {
    pub(super) verts: Vec<f32>,
    pub(super) hmin: f32,
    pub(super) hmax: f32,
    pub(super) area: u8,
}

/// Receives the area volumes of one fetch (not in recastnavigation).
#[derive(Debug)]
pub struct VolumeSink {
    window: Window,
    volumes: Vec<OwnedVolume>,
    last: Option<ItemId>,
    used: usize,
}

impl VolumeSink {
    pub(crate) fn new(bmin: [f32; 3], bmax: [f32; 3], cap_bytes: usize) -> Self {
        Self {
            window: Window::new(bmin, bmax, cap_bytes),
            volumes: Vec::new(),
            last: None,
            used: 0,
        }
    }

    /// Adds a convex area volume: packed xyz vertices of a polygon (at least three),
    /// its height range and a Recast area id (0..=63). Volumes outside the query
    /// box are dropped silently.
    ///
    /// # Errors
    /// [`Error::InvalidInput`] for an id not above the previous one or a malformed
    /// volume; [`Error::LimitExceeded`] when the fetch would exceed the tile byte limit.
    pub fn push(
        &mut self,
        id: ItemId,
        verts: &[f32],
        hmin: f32,
        hmax: f32,
        area: u8,
    ) -> Result<(), Error> {
        ascending(&mut self.last, id, "volume ids must ascend")?;
        if verts.len() < 9
            || !verts.len().is_multiple_of(3)
            || !finite(verts)
            || !hmin.is_finite()
            || !hmax.is_finite()
            || hmin > hmax
            || area > 63
        {
            return Err(Error::InvalidInput("area volume"));
        }
        let (lo, hi) = recast::calc_bounds(verts);
        if !self.window.overlaps([lo[0], lo[2]], [hi[0], hi[2]]) {
            return Ok(());
        }
        charge(&mut self.used, self.window.cap, verts.len() * 4 + 16)?;
        reserve(&mut self.volumes, 1)?;
        let mut copy = Vec::new();
        reserve(&mut copy, verts.len())?;
        copy.extend_from_slice(verts);
        self.volumes.push(OwnedVolume {
            verts: copy,
            hmin,
            hmax,
            area,
        });
        Ok(())
    }

    /// Number of volumes kept so far.
    pub fn len(&self) -> usize {
        self.volumes.len()
    }

    /// Whether no volume has been kept.
    pub fn is_empty(&self) -> bool {
        self.volumes.is_empty()
    }

    pub(super) fn digest(&self, h: &mut Fnv64) {
        h.u64(self.volumes.len() as u64);
        for v in &self.volumes {
            h.u64(v.verts.len() as u64);
            for &f in &v.verts {
                h.u32(f.to_bits());
            }
            h.u32(v.hmin.to_bits());
            h.u32(v.hmax.to_bits());
            h.u8(v.area);
        }
    }

    pub(super) fn into_volumes(self) -> Vec<OwnedVolume> {
        self.volumes
    }
}

/// Receives the off-mesh connections of one fetch (not in recastnavigation).
#[derive(Debug)]
pub struct OffMeshSink {
    window: Window,
    verts: Vec<f32>,
    rad: Vec<f32>,
    flags: Vec<u16>,
    areas: Vec<u8>,
    dirs: Vec<u8>,
    user_ids: Vec<u32>,
    last: Option<ItemId>,
    used: usize,
}

impl OffMeshSink {
    pub(crate) fn new(bmin: [f32; 3], bmax: [f32; 3], cap_bytes: usize) -> Self {
        Self {
            window: Window::new(bmin, bmax, cap_bytes),
            verts: Vec::new(),
            rad: Vec::new(),
            flags: Vec::new(),
            areas: Vec::new(),
            dirs: Vec::new(),
            user_ids: Vec::new(),
            last: None,
            used: 0,
        }
    }

    /// Adds an off-mesh connection from `start` to `end`. Connections with
    /// neither endpoint inside the query box (xz) are dropped silently.
    ///
    /// # Errors
    /// [`Error::InvalidInput`] for an id not above the previous one, a non-finite
    /// coordinate, a negative or non-finite radius or an area above 63;
    /// [`Error::LimitExceeded`] when the fetch would exceed the tile byte limit.
    #[allow(clippy::too_many_arguments)]
    pub fn push(
        &mut self,
        id: ItemId,
        start: [f32; 3],
        end: [f32; 3],
        radius: f32,
        flags: u16,
        area: u8,
        bidirectional: bool,
        user_id: u32,
    ) -> Result<(), Error> {
        ascending(&mut self.last, id, "off-mesh ids must ascend")?;
        if !finite(&start) || !finite(&end) || !(radius.is_finite() && radius >= 0.0) || area > 63 {
            return Err(Error::InvalidInput("off-mesh connection"));
        }
        if !self.window.contains([start[0], start[2]]) && !self.window.contains([end[0], end[2]]) {
            return Ok(());
        }
        charge(&mut self.used, self.window.cap, 6 * 4 + 4 + 2 + 1 + 1 + 4)?;
        reserve(&mut self.verts, 6)?;
        reserve(&mut self.rad, 1)?;
        reserve(&mut self.flags, 1)?;
        reserve(&mut self.areas, 1)?;
        reserve(&mut self.dirs, 1)?;
        reserve(&mut self.user_ids, 1)?;
        self.verts.extend_from_slice(&start);
        self.verts.extend_from_slice(&end);
        self.rad.push(radius);
        self.flags.push(flags);
        self.areas.push(area);
        self.dirs.push(u8::from(bidirectional));
        self.user_ids.push(user_id);
        Ok(())
    }

    /// Number of connections kept so far.
    pub fn len(&self) -> usize {
        self.rad.len()
    }

    /// Whether no connection has been kept.
    pub fn is_empty(&self) -> bool {
        self.rad.is_empty()
    }

    pub(super) fn verts(&self) -> &[f32] {
        &self.verts
    }
    pub(super) fn radii(&self) -> &[f32] {
        &self.rad
    }
    pub(super) fn flags(&self) -> &[u16] {
        &self.flags
    }
    pub(super) fn areas(&self) -> &[u8] {
        &self.areas
    }
    pub(super) fn dirs(&self) -> &[u8] {
        &self.dirs
    }
    pub(super) fn user_ids(&self) -> &[u32] {
        &self.user_ids
    }

    pub(super) fn digest(&self, h: &mut Fnv64) {
        h.u64(self.rad.len() as u64);
        for i in 0..self.rad.len() {
            for &f in &self.verts[i * 6..i * 6 + 6] {
                h.u32(f.to_bits());
            }
            h.u32(self.rad[i].to_bits());
            h.u16(self.flags[i]);
            h.u8(self.areas[i]);
            h.u8(self.dirs[i]);
            h.u32(self.user_ids[i]);
        }
    }
}

fn vertex(verts: &[f32], index: i32) -> Result<[f32; 3], Error> {
    let k = usize::try_from(index)
        .ok()
        .and_then(|k| k.checked_mul(3))
        .ok_or(Error::InvalidInput("triangle index"))?;
    match verts.get(k..k + 3) {
        Some(&[x, y, z]) => Ok([x, y, z]),
        _ => Err(Error::InvalidInput("triangle index")),
    }
}

impl GeometrySource for Input<'_> {
    fn triangles(
        &self,
        _bmin: [f32; 3],
        _bmax: [f32; 3],
        out: &mut TriangleSink,
    ) -> Result<(), Error> {
        if let Some(a) = self.areas
            && a.len() != self.tris.len() / 3
        {
            return Err(Error::InvalidInput("triangle areas"));
        }
        for (i, tri) in self.tris.as_chunks::<3>().0.iter().enumerate() {
            let t = [
                vertex(self.verts, tri[0])?,
                vertex(self.verts, tri[1])?,
                vertex(self.verts, tri[2])?,
            ];
            out.push(i as TriangleId, t, self.areas.map(|a| a[i]))?;
        }
        Ok(())
    }

    fn volumes(&self, _bmin: [f32; 3], _bmax: [f32; 3], out: &mut VolumeSink) -> Result<(), Error> {
        for (i, v) in self.volumes.iter().enumerate() {
            out.push(i as ItemId, v.verts, v.hmin, v.hmax, v.area)?;
        }
        Ok(())
    }

    fn off_mesh(
        &self,
        _bmin: [f32; 3],
        _bmax: [f32; 3],
        out: &mut OffMeshSink,
    ) -> Result<(), Error> {
        let p = &self.off_mesh;
        let n = usize::try_from(p.off_mesh_con_count)
            .map_err(|_| Error::InvalidInput("off-mesh count"))?;
        if p.off_mesh_con_verts.len() != n.checked_mul(6).ok_or(Error::SizeOverflow)?
            || p.off_mesh_con_rad.len() != n
            || p.off_mesh_con_flags.len() != n
            || p.off_mesh_con_areas.len() != n
            || p.off_mesh_con_dir.len() != n
            || (!p.off_mesh_con_user_id.is_empty() && p.off_mesh_con_user_id.len() != n)
        {
            return Err(Error::InvalidInput("off-mesh connections"));
        }
        for i in 0..n {
            let v = &p.off_mesh_con_verts[i * 6..i * 6 + 6];
            out.push(
                i as ItemId,
                [v[0], v[1], v[2]],
                [v[3], v[4], v[5]],
                p.off_mesh_con_rad[i],
                p.off_mesh_con_flags[i],
                p.off_mesh_con_areas[i],
                p.off_mesh_con_dir[i] != 0,
                p.off_mesh_con_user_id.get(i).copied().unwrap_or(0),
            )?;
        }
        Ok(())
    }
}
