//! The per-tile pipeline as a stage machine, shared by [`super::tiles`] and the bake job.
//!
//! One [`TileTask::advance`] call runs one stage (or, for the mixed-area case, the
//! two stages that mark the two kinds of triangle) and moves the in-flight value of
//! that stage into the next state. A whole tile is a loop over `advance`; the job
//! API stops between stages when its budget is spent.

use alloc::borrow::Cow;
use alloc::vec::Vec;
use core::mem;

use super::source::{OffMeshSink, VolumeSink};
use super::{AreaVolume, BakeConfig, Input, Partition, work};
use crate::detour::{self, NavMeshCreateParams};
use crate::recast::{
    self, CompactHeightfield, Config, ContourSet, Heightfield, PolyMesh, PolyMeshDetail,
};
use crate::{CancelToken, Error, Limits, Usage, Workspace};

// Stage APIs reset accounting. Supply the remaining per-tile budget at each
// boundary, then accumulate usage so a later stage cannot spend it again.
pub(super) struct TileWorkspace {
    pub(super) ws: Workspace,
    pub(super) limits: Limits,
    pub(super) usage: Usage,
}

impl TileWorkspace {
    pub(super) fn stage<T>(
        &mut self,
        f: impl FnOnce(&mut Workspace) -> Result<T, Error>,
    ) -> Result<T, Error> {
        let mut limits = self.limits;
        limits.max_bytes = limits.max_bytes.saturating_sub(self.usage.bytes);
        limits.max_work = limits.max_work.saturating_sub(self.usage.work);
        self.ws.set_limits(limits);
        self.ws.begin();
        let out = f(&mut self.ws);
        self.usage.bytes = self
            .usage
            .bytes
            .checked_add(self.ws.usage().bytes)
            .ok_or(Error::SizeOverflow)?;
        self.usage.work = self
            .usage
            .work
            .checked_add(self.ws.usage().work)
            .ok_or(Error::SizeOverflow)?;
        out
    }

    /// Like [`stage`](Self::stage) for a stage whose compact field was built by
    /// this pipeline: callers never see it, so the per-stage whole-grid
    /// connection scans (a full pass per stage) are skipped. Their work is still
    /// charged, so budgets behave as before.
    pub(super) fn trusted_stage<T>(
        &mut self,
        f: impl FnOnce(&mut Workspace) -> Result<T, Error>,
    ) -> Result<T, Error> {
        self.ws.set_compact_trusted(true);
        let out = self.stage(f);
        self.ws.set_compact_trusted(false);
        out
    }
}

/// Which walkability test marks each triangle of a tile.
pub(super) enum AreaMode {
    /// Every triangle starts as walkable by slope (`areas: None` in [`Input`]).
    Auto,
    /// Every triangle carries its own area; unwalkable slopes are cleared.
    Explicit,
    /// Per triangle: `true` marks by slope, `false` keeps the stored area.
    Mixed(Vec<bool>),
}

/// Where a tile's triangles come from.
pub(super) enum TriSource<'a> {
    /// Triangles of a borrowed [`Input`] selected by index; vertices are shared.
    Bucket {
        input: &'a Input<'a>,
        bucket: &'a [usize],
    },
    /// Triangles fetched from a source: vertices `3i..3i + 3` of the task's own
    /// vertex array form triangle `i`.
    Fetched { areas: Vec<u8>, mode: AreaMode },
}

/// Off-mesh connections handed to the Detour builder.
pub(super) enum OffMesh<'a> {
    /// Every connection of a borrowed [`Input`] (the builder keeps those that touch the tile).
    Borrowed(NavMeshCreateParams<'a>),
    /// The connections a source returned for the tile's bordered column.
    Owned(OffMeshSink),
}

impl OffMesh<'_> {
    fn params(&self) -> Result<NavMeshCreateParams<'_>, Error> {
        match self {
            OffMesh::Borrowed(p) => Ok(*p),
            OffMesh::Owned(s) => Ok(NavMeshCreateParams {
                off_mesh_con_verts: s.verts(),
                off_mesh_con_rad: s.radii(),
                off_mesh_con_flags: s.flags(),
                off_mesh_con_areas: s.areas(),
                off_mesh_con_dir: s.dirs(),
                off_mesh_con_user_id: s.user_ids(),
                off_mesh_con_count: i32::try_from(s.len()).map_err(|_| Error::SizeOverflow)?,
                ..NavMeshCreateParams::default()
            }),
        }
    }
}

/// One convex area marking of a tile.
pub(super) struct VolumeRef<'a> {
    pub(super) verts: Cow<'a, [f32]>,
    pub(super) hmin: f32,
    pub(super) hmax: f32,
    pub(super) area: u8,
}

impl<'a> VolumeRef<'a> {
    pub(super) fn borrowed(v: &AreaVolume<'a>) -> Self {
        Self {
            verts: Cow::Borrowed(v.verts),
            hmin: v.hmin,
            hmax: v.hmax,
            area: v.area,
        }
    }
}

enum State {
    Mark {
        tris: Vec<i32>,
        areas: Vec<u8>,
        mode: AreaMode,
    },
    Cells {
        tris: Vec<i32>,
        areas: Vec<u8>,
    },
    NewHeightfield {
        tris: Vec<i32>,
        areas: Vec<u8>,
    },
    Rasterize {
        hf: Heightfield,
        tris: Vec<i32>,
        areas: Vec<u8>,
    },
    Filter {
        hf: Heightfield,
        which: u8,
    },
    Compact {
        hf: Heightfield,
    },
    Erode {
        chf: CompactHeightfield,
    },
    Volumes {
        chf: CompactHeightfield,
        next: usize,
    },
    Distance {
        chf: CompactHeightfield,
    },
    Regions {
        chf: CompactHeightfield,
    },
    Contours {
        chf: CompactHeightfield,
    },
    Poly {
        chf: CompactHeightfield,
        contours: ContourSet,
    },
    Fix {
        chf: CompactHeightfield,
        mesh: PolyMesh,
        report: recast::TriangulationReport,
    },
    Detail {
        chf: CompactHeightfield,
        mesh: PolyMesh,
        report: recast::TriangulationReport,
    },
    Dnav {
        mesh: PolyMesh,
        detail: PolyMeshDetail,
        report: recast::TriangulationReport,
    },
    /// Finished, or left behind by a failed stage.
    Finished,
}

/// What one [`TileTask::advance`] left the tile in.
pub(super) enum Advance {
    /// More stages remain.
    More,
    /// The tile is finished: DNAV bytes (`None` for a tile without polygons).
    Done(Option<Vec<u8>>, recast::TriangulationReport),
}

/// One tile in flight. The `'a` lifetime only matters for the borrowed (legacy
/// input) forms; a fetched tile owns everything and is `'static`.
pub(super) struct TileTask<'a> {
    tw: TileWorkspace,
    c: BakeConfig,
    cfg: Config,
    x: i32,
    y: i32,
    verts: Cow<'a, [f32]>,
    volumes: Vec<VolumeRef<'a>>,
    off_mesh: OffMesh<'a>,
    /// Consumed by the first stage.
    source: Option<TriSource<'a>>,
    state: State,
}

impl<'a> TileTask<'a> {
    #[allow(clippy::too_many_arguments)]
    pub(super) fn new(
        c: &BakeConfig,
        cfg: &Config,
        x: i32,
        y: i32,
        verts: Cow<'a, [f32]>,
        source: TriSource<'a>,
        volumes: Vec<VolumeRef<'a>>,
        off_mesh: OffMesh<'a>,
        cancel: Option<CancelToken>,
    ) -> Self {
        let mut ws = Workspace::new();
        ws.set_cancel_token(cancel);
        Self {
            tw: TileWorkspace {
                ws,
                limits: c.tile_limits,
                usage: Usage::default(),
            },
            c: *c,
            cfg: *cfg,
            x,
            y,
            verts,
            volumes,
            off_mesh,
            source: Some(source),
            state: State::Finished,
        }
    }

    /// Work units charged so far, across all stages.
    pub(super) fn work_used(&self) -> u64 {
        self.tw.usage.work
    }

    /// Runs the next stage. After an error the task must be dropped.
    pub(super) fn advance(&mut self) -> Result<Advance, Error> {
        if let Some(source) = self.source.take() {
            self.gather(source)?;
            return Ok(Advance::More);
        }
        match mem::replace(&mut self.state, State::Finished) {
            State::Finished => Err(Error::Failed),
            State::Mark { tris, areas, mode } => {
                self.mark(tris, areas, mode)?;
                Ok(Advance::More)
            }
            State::Cells { tris, areas } => {
                let cfg = self.cfg;
                self.tw.stage(|ws| {
                    let cells = usize::try_from(cfg.width)
                        .map_err(|_| Error::SizeOverflow)?
                        .checked_mul(usize::try_from(cfg.height).map_err(|_| Error::SizeOverflow)?)
                        .ok_or(Error::SizeOverflow)?;
                    work(ws, cells)
                })?;
                self.state = State::NewHeightfield { tris, areas };
                Ok(Advance::More)
            }
            State::NewHeightfield { tris, areas } => {
                let cfg = self.cfg;
                let hf = self.tw.stage(|ws| {
                    Heightfield::new(
                        ws, cfg.width, cfg.height, cfg.bmin, cfg.bmax, cfg.cs, cfg.ch,
                    )
                })?;
                self.state = State::Rasterize { hf, tris, areas };
                Ok(Advance::More)
            }
            State::Rasterize {
                mut hf,
                tris,
                areas,
            } => {
                let climb = self.cfg.walkable_climb;
                if !tris.is_empty() {
                    let verts = &self.verts;
                    self.tw
                        .stage(|ws| hf.rasterize_triangles(ws, verts, &tris, &areas, climb))?;
                }
                self.state = State::Filter { hf, which: 0 };
                Ok(Advance::More)
            }
            State::Filter { mut hf, which } => {
                let cfg = self.cfg;
                match which {
                    0 if self.c.filter_low_hanging => self.tw.stage(|ws| {
                        hf.filter_low_hanging_walkable_obstacles(ws, cfg.walkable_climb)
                    })?,
                    1 if self.c.filter_ledge => self.tw.stage(|ws| {
                        hf.filter_ledge_spans(ws, cfg.walkable_height, cfg.walkable_climb)
                    })?,
                    2 if self.c.filter_low_height => self
                        .tw
                        .stage(|ws| hf.filter_walkable_low_height_spans(ws, cfg.walkable_height))?,
                    _ => {}
                }
                self.state = if which < 2 {
                    State::Filter {
                        hf,
                        which: which + 1,
                    }
                } else {
                    State::Compact { hf }
                };
                Ok(Advance::More)
            }
            State::Compact { hf } => {
                let cfg = self.cfg;
                let chf = self.tw.stage(|ws| {
                    CompactHeightfield::build(ws, cfg.walkable_height, cfg.walkable_climb, &hf)
                })?;
                self.state = State::Erode { chf };
                Ok(Advance::More)
            }
            State::Erode { mut chf } => {
                let radius = self.cfg.walkable_radius;
                self.tw
                    .trusted_stage(|ws| recast::erode_walkable_area(ws, radius, &mut chf))?;
                self.state = self.after_volumes(chf, 0);
                Ok(Advance::More)
            }
            State::Volumes { mut chf, next } => {
                if let Some(v) = self.volumes.get(next) {
                    self.tw.trusted_stage(|ws| {
                        recast::mark_convex_poly_area(
                            ws, &v.verts, v.hmin, v.hmax, v.area, &mut chf,
                        )
                    })?;
                }
                self.state = self.after_volumes(chf, next + 1);
                Ok(Advance::More)
            }
            State::Distance { mut chf } => {
                self.tw
                    .trusted_stage(|ws| recast::build_distance_field(ws, &mut chf))?;
                self.state = State::Regions { chf };
                Ok(Advance::More)
            }
            State::Regions { mut chf } => {
                let cfg = self.cfg;
                match self.c.partition {
                    Partition::Watershed => self.tw.trusted_stage(|ws| {
                        recast::build_regions(
                            ws,
                            &mut chf,
                            cfg.border_size,
                            cfg.min_region_area,
                            cfg.merge_region_area,
                        )
                    })?,
                    Partition::Monotone => self.tw.trusted_stage(|ws| {
                        recast::build_regions_monotone(
                            ws,
                            &mut chf,
                            cfg.border_size,
                            cfg.min_region_area,
                            cfg.merge_region_area,
                        )
                    })?,
                    Partition::Layers => self.tw.trusted_stage(|ws| {
                        recast::build_layer_regions(
                            ws,
                            &mut chf,
                            cfg.border_size,
                            cfg.min_region_area,
                        )
                    })?,
                }
                self.state = State::Contours { chf };
                Ok(Advance::More)
            }
            State::Contours { chf } => {
                let cfg = self.cfg;
                let contours = self.tw.trusted_stage(|ws| {
                    ContourSet::build(
                        ws,
                        &chf,
                        cfg.max_simplification_error,
                        cfg.max_edge_len,
                        recast::CONTOUR_TESS_WALL_EDGES,
                    )
                })?;
                self.state = State::Poly { chf, contours };
                Ok(Advance::More)
            }
            State::Poly { chf, contours } => {
                let cfg = self.cfg;
                let strict = self.c.strict_triangulation;
                let (mesh, report) = self.tw.stage(|ws| {
                    PolyMesh::build_with_report(ws, &contours, cfg.max_verts_per_poly, strict)
                })?;
                self.state = State::Fix { chf, mesh, report };
                Ok(Advance::More)
            }
            State::Fix {
                chf,
                mut mesh,
                report,
            } => {
                self.tw.stage(|ws| {
                    work(ws, mesh.areas.len())?;
                    for (a, f) in mesh.areas.iter_mut().zip(&mut mesh.flags) {
                        if *a == recast::WALKABLE_AREA {
                            *a = 0;
                        }
                        *f = if *a == 0 || *a >= 16 {
                            1
                        } else {
                            1 | (1 << *a)
                        };
                    }
                    Ok(())
                })?;
                self.state = State::Detail { chf, mesh, report };
                Ok(Advance::More)
            }
            State::Detail { chf, mesh, report } => {
                let cfg = self.cfg;
                if cfg.detail_sample_dist > 0.0 {
                    let samples_per_axis = cfg.width as f32 * cfg.cs / cfg.detail_sample_dist;
                    if !samples_per_axis.is_finite() || samples_per_axis > (i32::MAX / 4) as f32 {
                        return Err(Error::InvalidParameter("detail sample distance too small"));
                    }
                }
                let detail = self.tw.trusted_stage(|ws| {
                    PolyMeshDetail::build(
                        ws,
                        &mesh,
                        &chf,
                        cfg.detail_sample_dist,
                        cfg.detail_sample_max_error,
                    )
                })?;
                if mesh.npolys == 0 {
                    return Ok(Advance::Done(None, report));
                }
                self.state = State::Dnav {
                    mesh,
                    detail,
                    report,
                };
                Ok(Advance::More)
            }
            State::Dnav {
                mesh,
                detail,
                report,
            } => {
                let data = self.dnav(&mesh, &detail)?;
                Ok(Advance::Done(Some(data), report))
            }
        }
    }

    fn after_volumes(&self, chf: CompactHeightfield, next: usize) -> State {
        if next < self.volumes.len() {
            State::Volumes { chf, next }
        } else if self.c.partition == Partition::Watershed {
            State::Distance { chf }
        } else {
            State::Regions { chf }
        }
    }

    fn gather(&mut self, source: TriSource<'a>) -> Result<(), Error> {
        let (tris, areas, mode) = match source {
            TriSource::Bucket { input, bucket } => self.tw.stage(|ws| {
                work(ws, bucket.len())?;
                let mut tris =
                    ws.vec_with_capacity(bucket.len().checked_mul(3).ok_or(Error::SizeOverflow)?)?;
                let mut areas = ws.vec_with_capacity(bucket.len())?;
                for &i in bucket {
                    tris.extend_from_slice(&input.tris[i * 3..i * 3 + 3]);
                    areas.push(input.areas.map_or(0, |a| a[i]));
                }
                let mode = if input.areas.is_some() {
                    AreaMode::Explicit
                } else {
                    AreaMode::Auto
                };
                Ok((tris, areas, mode))
            })?,
            TriSource::Fetched { areas, mode } => {
                let n = areas.len();
                let verts = self.verts.len();
                self.tw.stage(|ws| {
                    work(ws, n)?;
                    ws.charge_bytes::<f32>(verts)?;
                    ws.charge_bytes::<u8>(n)?;
                    let mut tris =
                        ws.vec_with_capacity(n.checked_mul(3).ok_or(Error::SizeOverflow)?)?;
                    for i in 0..n.checked_mul(3).ok_or(Error::SizeOverflow)? {
                        tris.push(i32::try_from(i).map_err(|_| Error::SizeOverflow)?);
                    }
                    Ok((tris, areas, mode))
                })?
            }
        };
        self.state = State::Mark { tris, areas, mode };
        Ok(())
    }

    fn mark(&mut self, tris: Vec<i32>, mut areas: Vec<u8>, mode: AreaMode) -> Result<(), Error> {
        let slope = self.cfg.walkable_slope_angle;
        let verts = &self.verts;
        match mode {
            AreaMode::Explicit => self.tw.stage(|ws| {
                recast::clear_unwalkable_triangles(ws, slope, verts, &tris, &mut areas)
            })?,
            AreaMode::Auto => self
                .tw
                .stage(|ws| recast::mark_walkable_triangles(ws, slope, verts, &tris, &mut areas))?,
            AreaMode::Mixed(auto) => {
                // Each helper resets the stage accounting, so each runs in its own stage.
                let mut by_slope = self.tw.stage(|ws| ws.vec(areas.len(), 0u8))?;
                self.tw.stage(|ws| {
                    recast::mark_walkable_triangles(ws, slope, verts, &tris, &mut by_slope)
                })?;
                self.tw.stage(|ws| {
                    recast::clear_unwalkable_triangles(ws, slope, verts, &tris, &mut areas)
                })?;
                for ((a, s), by) in areas.iter_mut().zip(&auto).zip(&by_slope) {
                    if *s {
                        *a = *by;
                    }
                }
            }
        }
        self.state = State::Cells { tris, areas };
        Ok(())
    }

    fn dnav(&mut self, mesh: &PolyMesh, detail: &PolyMeshDetail) -> Result<Vec<u8>, Error> {
        let off = self.off_mesh.params()?;
        let c = self.c;
        let cfg = self.cfg;
        let (x, y) = (self.x, self.y);
        let params = NavMeshCreateParams {
            verts: &mesh.verts,
            vert_count: mesh.nverts,
            polys: &mesh.polys,
            poly_flags: &mesh.flags,
            poly_areas: &mesh.areas,
            poly_count: mesh.npolys,
            nvp: mesh.nvp,
            detail_meshes: &detail.meshes,
            detail_verts: &detail.verts,
            detail_verts_count: i32::try_from(detail.verts.len() / 3)
                .map_err(|_| Error::SizeOverflow)?,
            detail_tris: &detail.tris,
            detail_tri_count: i32::try_from(detail.tris.len() / 4)
                .map_err(|_| Error::SizeOverflow)?,
            tile_x: x,
            tile_y: y,
            tile_layer: 0,
            user_id: 0,
            bmin: mesh.bmin,
            bmax: mesh.bmax,
            walkable_height: c.walkable_height,
            walkable_radius: c.walkable_radius,
            walkable_climb: c.walkable_climb,
            cs: cfg.cs,
            ch: cfg.ch,
            build_bv_tree: true,
            ..off
        };
        self.tw.stage(|ws| {
            // Conservative output + temporary BV arrays/sort + link/classification
            // bound, charged before entering the existing Detour builder.
            let n = mesh
                .verts
                .len()
                .checked_add(mesh.polys.len())
                .and_then(|n| n.checked_add(detail.verts.len()))
                .and_then(|n| n.checked_add(detail.tris.len()))
                .and_then(|n| n.checked_add(off.off_mesh_con_verts.len()))
                .and_then(|n| n.checked_add(1))
                .ok_or(Error::SizeOverflow)?;
            let bound = n.checked_mul(128).ok_or(Error::SizeOverflow)?;
            let polys = usize::try_from(mesh.npolys).map_err(|_| Error::SizeOverflow)?;
            let sort_work = polys
                .checked_mul(
                    usize::try_from(usize::BITS - polys.leading_zeros())
                        .map_err(|_| Error::SizeOverflow)?
                        .pow(2),
                )
                .ok_or(Error::SizeOverflow)?;
            work(ws, bound.checked_add(sort_work).ok_or(Error::SizeOverflow)?)?;
            ws.charge_bytes::<u8>(bound)?;
            let data = detour::create_with_limits(&params, &c.tile_limits)?;
            if data.len() > c.tile_limits.max_tile_bytes {
                return Err(Error::LimitExceeded);
            }
            // Validate the produced tile against all caller-specified format limits.
            detour::check_bake_tile_limits(&data, &c.tile_limits)?;
            Ok(data)
        })
    }
}

// The volume sink is consumed into `VolumeRef`s by the job.
impl VolumeRef<'static> {
    pub(super) fn from_sink(sink: VolumeSink) -> Vec<VolumeRef<'static>> {
        sink.into_volumes()
            .into_iter()
            .map(|v| VolumeRef {
                verts: Cow::Owned(v.verts),
                hmin: v.hmin,
                hmax: v.hmax,
                area: v.area,
            })
            .collect()
    }
}
