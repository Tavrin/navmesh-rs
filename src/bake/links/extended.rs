use super::{
    BakeConfig, ExtendedExtractedLink, ExtractedLink, clear, clear_with_grid, column, landing_key,
};
use crate::bake::VolumeRef;
use crate::common::{ceilf, sqrtf};
use crate::detour::OffMeshKind;
use crate::recast::{
    CompactHeightfield, Heightfield, MESH_NULL_IDX, NULL_AREA, PolyMesh, RasterGrid,
};
use crate::{Error, Workspace};
use alloc::vec::Vec;

/// Per-profile settings for the additional traversal kinds. Each kind is opt-in.
#[derive(Clone, Copy, Debug, Default, PartialEq)]
#[non_exhaustive]
pub struct ExtendedLinkExtraction {
    /// Ladders authored with dedicated ladder volumes.
    pub ladders: Option<LadderLinks>,
    /// Short, solid steps above the ordinary walkable climb.
    pub steps: Option<StepLinks>,
    /// Jumps across unsupported gaps between level landings.
    pub level_gaps: Option<GapLinks>,
}

/// Ladder authoring and landing settings, in world units.
#[derive(Clone, Copy, Debug, PartialEq)]
#[non_exhaustive]
pub struct LadderLinks {
    /// Maximum horizontal reach from the shaft axis to a landing.
    pub reach: f32,
    /// Detour area used for the ladder traversal cost.
    pub area: u8,
    /// Detour polygon flags for traversal.
    pub flags: u16,
}

impl LadderLinks {
    /// Sets the link's area, flags and landing reach.
    pub fn new(reach: f32, area: u8, flags: u16) -> Self {
        Self { reach, area, flags }
    }
}

/// Disconnected stair-step settings, in world units.
#[derive(Clone, Copy, Debug, PartialEq)]
#[non_exhaustive]
pub struct StepLinks {
    /// Maximum horizontal distance between landings, including the eroded setback.
    pub distance: f32,
    /// Maximum vertical difference between landings.
    pub height: f32,
    /// Detour area used for the step traversal cost.
    pub area: u8,
    /// Detour polygon flags for traversal.
    pub flags: u16,
}

impl StepLinks {
    /// Sets the maximum distance, height and link's area and flags.
    pub fn new(distance: f32, height: f32, area: u8, flags: u16) -> Self {
        Self {
            distance,
            height,
            area,
            flags,
        }
    }
}

/// Level gap jump settings, in world units.
#[derive(Clone, Copy, Debug, PartialEq)]
#[non_exhaustive]
pub struct GapLinks {
    /// Maximum horizontal distance between takeoff and landing, including the eroded setback.
    pub distance: f32,
    /// Height of the jump's horizontal clearance segment above the landings.
    pub clearance: f32,
    /// Detour area used for the gap jump traversal cost.
    pub area: u8,
    /// Detour polygon flags for traversal.
    pub flags: u16,
}

impl GapLinks {
    /// Sets the maximum distance, clearance height and link's area and flags.
    pub fn new(distance: f32, clearance: f32, area: u8, flags: u16) -> Self {
        Self {
            distance,
            clearance,
            area,
            flags,
        }
    }
}

impl ExtendedLinkExtraction {
    pub(in crate::bake) fn validate(self) -> Result<(), Error> {
        let valid = |v: f32| v.is_finite() && v >= 0.0;
        if self.ladders.is_some_and(|l| l.area > 63 || !valid(l.reach))
            || self
                .steps
                .is_some_and(|l| l.area > 63 || !valid(l.distance) || !valid(l.height))
            || self
                .level_gaps
                .is_some_and(|l| l.area > 63 || !valid(l.distance) || !valid(l.clearance))
        {
            return Err(Error::InvalidParameter("extended link extraction settings"));
        }
        Ok(())
    }

    pub(in crate::bake) fn enabled(self) -> bool {
        self.ladders.is_some() || self.steps.is_some() || self.level_gaps.is_some()
    }

    // Tagged suffix, absent when disabled, shared by both bake identities.
    pub(in crate::bake) fn encode(self, mut write: impl FnMut(&[u8])) {
        // An out-of-range area byte invalidates cached results from the earlier
        // extraction rules without changing identities when extensions are off.
        let revised = crate::detour::MAX_AREAS as u8;
        if let Some(l) = self.ladders {
            write(&[0xa8, revised, l.area]);
            write(&l.flags.to_le_bytes());
            write(&l.reach.to_le_bytes());
        }
        if let Some(l) = self.steps {
            write(&[0xa9, revised, l.area]);
            write(&l.flags.to_le_bytes());
            write(&l.distance.to_le_bytes());
            write(&l.height.to_le_bytes());
        }
        if let Some(l) = self.level_gaps {
            write(&[0xaa, revised, l.area]);
            write(&l.flags.to_le_bytes());
            write(&l.distance.to_le_bytes());
            write(&l.clearance.to_le_bytes());
        }
    }

    pub(super) fn reach(self) -> f32 {
        self.steps
            .map_or(0.0, |l| l.distance)
            .max(self.level_gaps.map_or(0.0, |l| l.distance))
            .max(self.ladders.map_or(0.0, |l| l.reach * 2.0))
    }
}

#[derive(Clone, Copy, PartialEq, Eq)]
enum FloorTrace {
    Gap,
    Walkable,
    Step,
    Obstructed,
}

struct Geometry<'a> {
    c: &'a BakeConfig,
    mesh: &'a PolyMesh,
    field: &'a CompactHeightfield,
    solid: &'a Heightfield,
}

#[derive(Clone, Copy)]
struct Landing {
    point: [f32; 3],
    span: usize,
    column: usize,
}

impl Geometry<'_> {
    fn local(&self, p: [f32; 3]) -> bool {
        [0, 2]
            .into_iter()
            .all(|axis| p[axis] >= self.mesh.bmin[axis] && p[axis] < self.mesh.bmax[axis])
    }

    fn landings(
        &self,
        ws: &mut Workspace,
        col: usize,
        grid: Option<LadderGrid>,
    ) -> Result<impl Iterator<Item = Landing> + '_, Error> {
        let h = self.field;
        let cell = h.cells[col];
        ws.work(u64::from(cell.count))?;
        let center = grid.map_or_else(
            || {
                [
                    h.bmin[0] + (col % h.width as usize) as f32 * h.cs + h.cs * 0.5,
                    0.0,
                    h.bmin[2] + (col / h.width as usize) as f32 * h.cs + h.cs * 0.5,
                ]
            },
            |grid| grid.center(col, h.width),
        );
        Ok(
            (cell.index as usize..cell.index as usize + cell.count as usize).filter_map(
                move |span| {
                    let s = h.spans[span];
                    (h.areas[span] != NULL_AREA
                        && (grid.is_some() || s.reg != 0)
                        && s.h as f32 * h.ch >= self.c.walkable_height)
                        .then_some(Landing {
                            point: [center[0], h.bmin[1] + s.y as f32 * h.ch, center[2]],
                            span,
                            column: col,
                        })
                },
            ),
        )
    }

    fn clears(&self, ws: &mut Workspace, a: [f32; 3], b: [f32; 3]) -> Result<bool, Error> {
        clear(
            ws,
            self.solid,
            a,
            b,
            0.0,
            self.c.walkable_height,
            self.c.walkable_radius,
        )
    }

    fn step_clear(&self, ws: &mut Workspace, a: [f32; 3], b: [f32; 3]) -> Result<bool, Error> {
        let (low, high) = if a[1] < b[1] { (a, b) } else { (b, a) };
        let over = [low[0], high[1], low[2]];
        Ok(self.clears(ws, low, over)? && self.clears(ws, over, high)?)
    }

    fn gap_clear(
        &self,
        ws: &mut Workspace,
        a: [f32; 3],
        b: [f32; 3],
        lift: f32,
    ) -> Result<bool, Error> {
        let up = |p: [f32; 3]| [p[0], p[1] + lift, p[2]];
        Ok(self.clears(ws, a, up(a))?
            && self.clears(ws, up(a), up(b))?
            && self.clears(ws, up(b), b)?)
    }

    // Trace the centerline over the solid grid. A step has support in every
    // crossed cell between the two landing heights. A level gap must lose it.
    fn trace_floor(&self, ws: &mut Workspace, a: Landing, b: Landing) -> Result<FloorTrace, Error> {
        let h = self.field;
        let locate = |col: usize| {
            let w = h.width as usize;
            [(col % w) as i32, (col / w) as i32]
        };
        let mut cell = locate(a.column);
        let end = locate(b.column);
        let mut next = [0.0f64; 2];
        let mut rate = [0.0f64; 2];
        let mut dir = [0i32; 2];
        for i in 0..2 {
            let delta = end[i] - cell[i];
            dir[i] = delta.signum();
            rate[i] = 1.0 / f64::from(delta.abs());
            next[i] = 0.5 * rate[i];
        }
        let ya = h.spans[a.span].y;
        let yb = h.spans[b.span].y;
        let mut previous = ya;
        let mut result = FloorTrace::Walkable;
        loop {
            ws.work(1)?;
            let col = (cell[0] + cell[1] * h.width) as usize;
            let mut floor = None;
            for span in self.solid.column(col) {
                ws.work(1)?;
                if ya == yb
                    && span.smax != ya
                    && (f32::from(ya) - f32::from(span.smax)) * h.ch < self.c.walkable_climb
                {
                    return Ok(FloorTrace::Obstructed);
                }
                if (ya.min(yb)..=ya.max(yb)).contains(&span.smax) {
                    floor = Some(span.smax);
                }
            }
            if let Some(floor) = floor {
                let change = previous.abs_diff(floor);
                if i32::from(change) > h.walkable_climb
                    && f32::from(change) * h.ch > self.c.walkable_climb
                {
                    result = FloorTrace::Step;
                }
                previous = floor;
            } else if ya != yb {
                return Ok(FloorTrace::Gap);
            } else {
                // Still check the remaining cells for obstacles after crossing a void.
                result = FloorTrace::Gap;
            }
            if cell == end {
                return Ok(result);
            }
            let [x, z] = next;
            for (i, t) in [x, z].into_iter().enumerate() {
                if cell[i] != end[i] && t <= [z, x][i] {
                    cell[i] += dir[i];
                    next[i] = if cell[i] == end[i] {
                        f64::INFINITY
                    } else {
                        t + rate[i]
                    };
                }
            }
        }
    }

    fn link(
        &self,
        start: Landing,
        end: Landing,
        kind: OffMeshKind,
        area: u8,
        flags: u16,
    ) -> ExtendedExtractedLink {
        ExtendedExtractedLink {
            start: start.point,
            end: end.point,
            radius: self.field.cs,
            kind,
            area,
            flags,
        }
    }
}

fn distance_sq(a: [f32; 3], b: [f32; 3]) -> f64 {
    let dx = f64::from(a[0]) - f64::from(b[0]);
    let dz = f64::from(a[2]) - f64::from(b[2]);
    dx * dx + dz * dz
}

#[derive(Clone, Copy)]
enum BoundaryKind {
    Step(StepLinks),
    Gap(GapLinks),
}

impl BoundaryKind {
    fn distance(self) -> f32 {
        match self {
            Self::Step(l) => l.distance,
            Self::Gap(l) => l.distance,
        }
    }

    fn candidate(
        self,
        g: &Geometry<'_>,
        ws: &mut Workspace,
        a: Landing,
        b: Landing,
    ) -> Result<Option<ExtendedExtractedLink>, Error> {
        let cells = g.field.spans[b.span].y.abs_diff(g.field.spans[a.span].y);
        let rise = f32::from(cells) * g.field.ch;
        let distance = f64::from(self.distance());
        if distance_sq(a.point, b.point) > distance * distance {
            return Ok(None);
        }
        match self {
            Self::Step(l)
                if i32::from(cells) > g.field.walkable_climb
                    && rise > g.c.walkable_climb
                    && rise <= l.height
                    && g.trace_floor(ws, a, b)? == FloorTrace::Step
                    && g.step_clear(ws, a.point, b.point)? =>
            {
                Ok(Some(g.link(a, b, OffMeshKind::Step, l.area, l.flags)))
            }
            Self::Gap(l)
                if cells == 0
                    && g.trace_floor(ws, a, b)? == FloorTrace::Gap
                    && g.gap_clear(ws, a.point, b.point, l.clearance)? =>
            {
                Ok(Some(g.link(a, b, OffMeshKind::LevelGap, l.area, l.flags)))
            }
            _ => Ok(None),
        }
    }
}

fn boundary_links(
    g: &Geometry<'_>,
    ws: &mut Workspace,
    kind: BoundaryKind,
    out: &mut Vec<ExtendedExtractedLink>,
) -> Result<(), Error> {
    let h = g.field;
    let mesh = g.mesh;
    let cells = ceilf(kind.distance() / h.cs) as i32;
    let nvp = mesh.nvp as usize;
    for p in 0..mesh.npolys as usize {
        let poly = &mesh.polys[p * nvp * 2..(p + 1) * nvp * 2];
        let count = poly[..nvp]
            .iter()
            .take_while(|&&v| v != MESH_NULL_IDX)
            .count();
        for e in 0..count {
            ws.work(1)?;
            if poly[nvp + e] != MESH_NULL_IDX {
                continue;
            }
            let point = |v: u16| {
                let v = v as usize * 3;
                [
                    mesh.bmin[0] + mesh.verts[v] as f32 * mesh.cs,
                    mesh.bmin[1] + mesh.verts[v + 1] as f32 * mesh.ch,
                    mesh.bmin[2] + mesh.verts[v + 2] as f32 * mesh.cs,
                ]
            };
            let a = point(poly[e]);
            let b = point(poly[(e + 1) % count]);
            let dx = b[0] - a[0];
            let dz = b[2] - a[2];
            let scale = dx.abs().max(dz.abs());
            if scale == 0.0 {
                continue;
            }
            let len = scale * sqrtf((dx / scale) * (dx / scale) + (dz / scale) * (dz / scale));
            if !len.is_finite() {
                continue;
            }
            let normal = [-dz / len, dx / len];
            let samples = ceilf(len / (h.cs * 2.0)) as i32;
            let mut run = Vec::new();
            let mut previous = None;
            for i in 0..samples {
                ws.work(1)?;
                let t = (i as f32 + 0.5) / samples as f32;
                let edge = [a[0] + dx * t, a[1] + (b[1] - a[1]) * t, a[2] + dz * t];
                let found = boundary_sample(g, ws, kind, p, cells, edge, normal)?;
                if found.as_ref().map(|(_, key)| *key) != previous {
                    super::flush(ws, &mut run, out)?;
                }
                previous = found.as_ref().map(|(_, key)| *key);
                if let Some(link) = found {
                    ws.push(&mut run, link)?;
                }
            }
            super::flush(ws, &mut run, out)?;
        }
    }
    Ok(())
}

fn boundary_sample(
    g: &Geometry<'_>,
    ws: &mut Workspace,
    kind: BoundaryKind,
    polygon: usize,
    cells: i32,
    edge: [f32; 3],
    normal: [f32; 2],
) -> Result<Option<(ExtendedExtractedLink, usize)>, Error> {
    let h = g.field;
    let inset = h.cs * 0.75;
    let origin = [
        edge[0] - normal[0] * inset,
        edge[1],
        edge[2] - normal[1] * inset,
    ];
    let Some(col) = column(h, origin) else {
        return Ok(None);
    };
    let mut source = None::<Landing>;
    for a in g.landings(ws, col, None)? {
        ws.work(1)?;
        let error = (a.point[1] - origin[1]).abs();
        if error <= 2.0 * h.ch
            && source.is_none_or(|prev| error < (prev.point[1] - origin[1]).abs())
            && g.local(a.point)
            && landing_key(ws, g.mesh, a.point, a.span)? == polygon
        {
            source = Some(a);
        }
    }
    let Some(a) = source else { return Ok(None) };
    for d in 0..=cells {
        ws.work(1)?;
        let offset = (d as f32 + 0.5) * h.cs;
        let p = [
            edge[0] + normal[0] * offset,
            edge[1],
            edge[2] + normal[1] * offset,
        ];
        let Some(col) = column(h, p) else { break };
        for b in g.landings(ws, col, None)? {
            ws.work(1)?;
            if let Some(link) = kind.candidate(g, ws, a, b)? {
                let key = landing_key(ws, g.mesh, b.point, b.span)?;
                if key != polygon && (!g.local(b.point) || key < g.mesh.npolys as usize) {
                    return Ok(Some((link, key)));
                }
            }
        }
    }
    Ok(None)
}

/// The global voxel lattice and the index of this tile's first bordered cell.
#[derive(Clone, Copy)]
pub(in crate::bake) struct LadderGrid {
    pub(in crate::bake) raster: RasterGrid,
    border: i32,
    tile_size: i32,
}

impl LadderGrid {
    pub(in crate::bake) fn new(
        origin: [f32; 3],
        tile: [i32; 2],
        cfg: &crate::recast::Config,
    ) -> Self {
        Self {
            raster: RasterGrid {
                origin: [origin[0], origin[2]],
                first: tile
                    .map(|i| i64::from(i) * i64::from(cfg.tile_size) - i64::from(cfg.border_size)),
                cell_size: cfg.cs,
            },
            border: cfg.border_size,
            tile_size: cfg.tile_size,
        }
    }

    fn coords(column: usize, width: i32) -> [i32; 2] {
        let width = width as usize;
        [(column % width) as i32, (column / width) as i32]
    }

    fn center(self, column: usize, width: i32) -> [f32; 3] {
        let [x, z] = Self::coords(column, width);
        [
            self.raster.position(0, x, 0.5) as f32,
            0.,
            self.raster.position(1, z, 0.5) as f32,
        ]
    }

    pub(super) fn covered_cells(self, axis: usize, lo: f32, hi: f32, radius: f32) -> (i32, i32) {
        (
            self.raster.cell(axis, f64::from(lo) - f64::from(radius)),
            self.raster.cell(axis, f64::from(hi) + f64::from(radius)),
        )
    }

    fn owns(self, column: usize, width: i32) -> bool {
        Self::coords(column, width)
            .into_iter()
            .all(|i| (self.border..self.border + self.tile_size).contains(&i))
    }
}

// The convex volume defines a vertical shaft and the landing search footprint.
fn ladder_links(
    g: &Geometry<'_>,
    ws: &mut Workspace,
    options: LadderLinks,
    volume: &VolumeRef<'_>,
    grid: LadderGrid,
    out: &mut Vec<ExtendedExtractedLink>,
    incoming: &mut Vec<ExtendedExtractedLink>,
) -> Result<(), Error> {
    let mut axis = [0.0f64; 2];
    let verts = volume.verts.as_chunks::<3>().0;
    for v in verts {
        ws.work(1)?;
        axis[0] += f64::from(v[0]);
        axis[1] += f64::from(v[2]);
    }
    let axis = [
        (axis[0] / verts.len() as f64) as f32,
        (axis[1] / verts.len() as f64) as f32,
    ];
    let h = g.field;
    let bounds = |a: usize, n: i32| {
        let (lo, hi) = grid.covered_cells(a, axis[a], axis[a], options.reach);
        lo.max(0)..=hi.min(n - 1)
    };
    let shaft = |y| [axis[0], y, axis[1]];
    let clears = |ws: &mut Workspace, a, b| {
        clear_with_grid(
            ws,
            g.solid,
            a,
            b,
            0.0,
            g.c.walkable_height,
            g.c.walkable_radius,
            Some(grid),
        )
    };
    let reach = f64::from(options.reach);
    let mut floors = Vec::<(u16, f64, Landing)>::new();
    for z in bounds(1, h.height) {
        for x in bounds(0, h.width) {
            ws.work(1)?;
            for landing in g.landings(ws, (x + z * h.width) as usize, Some(grid))? {
                ws.work(1 + verts.len() as u64)?;
                let p = landing.point;
                let distance = distance_sq(p, shaft(p[1]));
                if p[1] < volume.hmin
                    || p[1] > volume.hmax
                    || distance > reach * reach
                    || !crate::recast::point_in_area_polygon(&volume.verts, &p)
                {
                    continue;
                }
                let y = h.spans[landing.span].y;
                let index = floors.binary_search_by_key(&y, |&(y, _, _)| y);
                if index.is_ok_and(|i| floors[i].1 <= distance) || !clears(ws, p, shaft(p[1]))? {
                    continue;
                }
                match index {
                    Ok(i) => floors[i] = (y, distance, landing),
                    Err(i) => {
                        ws.work((floors.len() - i) as u64)?;
                        ws.push(&mut floors, (y, distance, landing))?;
                        floors[i..].rotate_right(1);
                    }
                }
            }
        }
    }
    for pair in floors.windows(2) {
        let (_, _, a) = pair[0];
        let (_, _, b) = pair[1];
        // The departure tile owns the segment. The arrival tile computes the
        // same pair from the shared lattice and only reserves its reverse slot.
        let start_local = grid.owns(a.column, h.width);
        let end_local = grid.owns(b.column, h.width);
        if !(start_local || end_local)
            || b.point[1] - a.point[1] <= g.c.walkable_climb
            || !clears(ws, shaft(a.point[1]), shaft(b.point[1]))?
        {
            continue;
        }
        let polys = g.mesh.npolys as usize;
        if (start_local && landing_key(ws, g.mesh, a.point, a.span)? >= polys)
            || (end_local && landing_key(ws, g.mesh, b.point, b.span)? >= polys)
        {
            continue;
        }
        let link = g.link(a, b, OffMeshKind::Ladder, options.area, options.flags);
        let links = if start_local {
            &mut *out
        } else {
            &mut *incoming
        };
        ws.work(links.len() as u64)?;
        if !links.contains(&link) {
            ws.push(links, link)?;
        }
    }
    Ok(())
}

#[allow(clippy::too_many_arguments)]
pub(in crate::bake) fn extract_extended(
    ws: &mut Workspace,
    c: &BakeConfig,
    mesh: &PolyMesh,
    field: &CompactHeightfield,
    solid: &Heightfield,
    volumes: &[VolumeRef<'_>],
    ladder_grid: LadderGrid,
    links: &mut super::TileLinks,
) -> Result<u32, Error> {
    let g = Geometry {
        c,
        mesh,
        field,
        solid,
    };
    let mut incoming = Vec::new();
    if let Some(options) = c.extended_links.ladders {
        for volume in volumes.iter().filter(|v| v.area.is_none()) {
            ladder_links(
                &g,
                ws,
                options,
                volume,
                ladder_grid,
                &mut links.extended,
                &mut incoming,
            )?;
        }
    }
    if let Some(options) = c.extended_links.steps {
        let first_step = links.extended.len();
        boundary_links(&g, ws, BoundaryKind::Step(options), &mut links.extended)?;
        dedupe_steps(&g, ws, &links.legacy, &mut links.extended, first_step)?;
    }
    if let Some(options) = c.extended_links.level_gaps {
        boundary_links(&g, ws, BoundaryKind::Gap(options), &mut links.extended)?;
    }
    u32::try_from(incoming.len()).map_err(|_| Error::SizeOverflow)
}

// Both endpoints of an intra-tile link can be identified here. A seam link
// is checked when its destination tile is resident and its polygon is known.
fn dedupe_steps(
    g: &Geometry<'_>,
    ws: &mut Workspace,
    legacy: &[ExtractedLink],
    out: &mut Vec<ExtendedExtractedLink>,
    first_step: usize,
) -> Result<(), Error> {
    if legacy.is_empty() || first_step == out.len() {
        return Ok(());
    }
    let polygons = g.mesh.npolys as usize;
    let mut pairs = ws.vec_with_capacity(legacy.len())?;
    for link in legacy {
        ws.work(1)?;
        if g.local(link.end) {
            pairs.push((
                landing_key(ws, g.mesh, link.start, 0)?,
                landing_key(ws, g.mesh, link.end, 0)?,
            ));
        }
    }
    let search_work = u64::from(usize::BITS - pairs.len().leading_zeros());
    ws.work(pairs.len() as u64 * search_work)?;
    pairs.sort_unstable();
    let mut kept = first_step;
    for index in first_step..out.len() {
        let link = &out[index];
        ws.work(search_work + 1)?;
        if g.local(link.end) {
            let pair = (
                landing_key(ws, g.mesh, link.start, 0)?,
                landing_key(ws, g.mesh, link.end, 0)?,
            );
            if pair.0 < polygons && pair.1 < polygons && pairs.binary_search(&pair).is_ok() {
                continue;
            }
        }
        out.swap(kept, index);
        kept += 1;
    }
    out.truncate(kept);
    Ok(())
}
