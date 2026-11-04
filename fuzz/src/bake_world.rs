//! A small editable world for the bake-job fuzz targets (`incremental_bake`, `bake_job_steps`).
//!
//! Layout of the input: `[0]` tile size `16 + b % 33` cells; `[1]` partition (`b % 3`) and agent
//! radius (`(b >> 2) & 3` cells); `[2]` bit 0 switches the span filters off, bit 1 gives every
//! triangle an explicit area; `[3]` triangle count `1 + b % 24`; `[4]` volume count `b % 4`;
//! `[5]` link count `b % 3`; `[6]` edit count `b % 9`; `[7]` driver parameter; then triangles
//! (nine coordinate bytes and an area byte), volumes (`x z w h area`), links (`fx fz tx tz`)
//! and edits (eight bytes: operation and seven parameters). Missing bytes read as zero.

use navmesh_rs::bake::{
    self, AreaVolume, BakeConfig, BakeJob, BakeManifest, GeometrySource, GridLayout, Input,
    JobOptions, ManifestEntry, OffMeshSink, Partition, TileKey, TileOutput, TriangleSink,
    VolumeSink,
};
use navmesh_rs::{Budget, Error, Progress, Workspace};
use std::collections::BTreeMap;

/// Side of the square world, metres.
pub const WORLD: f32 = 32.0;

type Tri = ([[f32; 3]; 3], Option<u8>);

/// Triangles, area volumes and off-mesh links addressed by their (stable) index.
#[derive(Clone, Debug, Default)]
pub struct FuzzWorld {
    pub tris: Vec<Option<Tri>>,
    pub volumes: Vec<Option<([f32; 12], u8)>>,
    pub links: Vec<Option<([f32; 3], [f32; 3])>>,
}

/// One change to a [`FuzzWorld`].
#[derive(Clone, Copy, Debug)]
pub enum Edit {
    AddTri(Tri),
    MoveTri(usize, [f32; 3]),
    RemoveTri(usize),
    AddVolume([f32; 12], u8),
    MoveVolume(usize, [f32; 2]),
    RemoveVolume(usize),
    AddLink([f32; 3], [f32; 3]),
    RemoveLink(usize),
}

pub struct Scenario {
    pub cfg: BakeConfig,
    pub world: FuzzWorld,
    pub edits: Vec<Edit>,
    pub param: u8,
    pub explicit_areas: bool,
}

struct Bytes<'a>(std::slice::Iter<'a, u8>);

impl Bytes<'_> {
    fn next(&mut self) -> u8 {
        self.0.next().copied().unwrap_or(0)
    }
    fn coord(&mut self) -> f32 {
        f32::from(self.next()) * (WORLD / 255.0)
    }
    fn height(&mut self) -> f32 {
        f32::from(self.next()) * (6.0 / 255.0)
    }
    fn point(&mut self) -> [f32; 3] {
        let x = self.coord();
        let y = self.height();
        [x, y, self.coord()]
    }
}

fn rect(x: f32, z: f32, w: f32, h: f32) -> [f32; 12] {
    [x, 0., z, x, 0., z + h, x + w, 0., z + h, x + w, 0., z]
}

fn bbox(points: &[[f32; 3]]) -> [f32; 6] {
    let mut b = [
        f32::INFINITY,
        -1.0,
        f32::INFINITY,
        f32::NEG_INFINITY,
        8.0,
        f32::NEG_INFINITY,
    ];
    for p in points {
        b[0] = b[0].min(p[0]);
        b[2] = b[2].min(p[2]);
        b[3] = b[3].max(p[0]);
        b[5] = b[5].max(p[2]);
    }
    b
}

fn rect_box(r: &[f32; 12]) -> [f32; 6] {
    bbox(&[[r[0], 0., r[2]], [r[6], 0., r[8]]])
}

fn shifted<const N: usize>(p: [[f32; 3]; N], d: [f32; 3]) -> [[f32; 3]; N] {
    p.map(|v| [v[0] + d[0], v[1] + d[1], v[2] + d[2]])
}

impl FuzzWorld {
    /// Applies an edit and records the boxes it changes (before and after).
    pub fn apply(&mut self, edit: &Edit, boxes: &mut Vec<[f32; 6]>) {
        match *edit {
            Edit::AddTri(t) => {
                boxes.push(bbox(&t.0));
                self.tris.push(Some(t));
            }
            Edit::MoveTri(i, d) => {
                let n = self.tris.len().max(1);
                if let Some(Some((t, _))) = self.tris.get_mut(i % n) {
                    boxes.push(bbox(t));
                    *t = shifted(*t, d);
                    boxes.push(bbox(t));
                }
            }
            Edit::RemoveTri(i) => {
                let n = self.tris.len().max(1);
                if let Some(slot) = self.tris.get_mut(i % n)
                    && let Some((t, _)) = slot.take()
                {
                    boxes.push(bbox(&t));
                }
            }
            Edit::AddVolume(r, a) => {
                boxes.push(rect_box(&r));
                self.volumes.push(Some((r, a)));
            }
            Edit::MoveVolume(i, d) => {
                let n = self.volumes.len().max(1);
                if let Some(Some((r, _))) = self.volumes.get_mut(i % n) {
                    boxes.push(rect_box(r));
                    for k in 0..4 {
                        r[k * 3] += d[0];
                        r[k * 3 + 2] += d[1];
                    }
                    boxes.push(rect_box(r));
                }
            }
            Edit::RemoveVolume(i) => {
                let n = self.volumes.len().max(1);
                if let Some(slot) = self.volumes.get_mut(i % n)
                    && let Some((r, _)) = slot.take()
                {
                    boxes.push(rect_box(&r));
                }
            }
            Edit::AddLink(a, b) => {
                boxes.push(bbox(&[a, b]));
                self.links.push(Some((a, b)));
            }
            Edit::RemoveLink(i) => {
                let n = self.links.len().max(1);
                if let Some(slot) = self.links.get_mut(i % n)
                    && let Some((a, b)) = slot.take()
                {
                    boxes.push(bbox(&[a, b]));
                }
            }
        }
    }
}

impl GeometrySource for FuzzWorld {
    fn triangles(&self, _: [f32; 3], _: [f32; 3], out: &mut TriangleSink) -> Result<(), Error> {
        for (i, t) in self.tris.iter().enumerate() {
            if let Some((t, a)) = t {
                out.push(i as u64, *t, *a)?;
            }
        }
        Ok(())
    }

    fn volumes(&self, _: [f32; 3], _: [f32; 3], out: &mut VolumeSink) -> Result<(), Error> {
        for (i, v) in self.volumes.iter().enumerate() {
            if let Some((r, a)) = v {
                out.push(i as u64, r, -1.0, 8.0, *a)?;
            }
        }
        Ok(())
    }

    fn off_mesh(&self, _: [f32; 3], _: [f32; 3], out: &mut OffMeshSink) -> Result<(), Error> {
        for (i, l) in self.links.iter().enumerate() {
            if let Some((a, b)) = l {
                out.push(i as u64, *a, *b, 0.6, 1, 0, true, i as u32)?;
            }
        }
        Ok(())
    }
}

/// Decodes a scenario; `None` only for inputs shorter than the header.
pub fn decode(data: &[u8]) -> Option<Scenario> {
    if data.len() < 8 {
        return None;
    }
    let mut cfg = BakeConfig::default();
    cfg.recast.tile_size = 16 + i32::from(data[0] % 33);
    cfg.partition = match data[1] % 3 {
        0 => Partition::Watershed,
        1 => Partition::Monotone,
        _ => Partition::Layers,
    };
    cfg.recast.walkable_radius = i32::from((data[1] >> 2) & 3);
    if data[2] & 1 != 0 {
        cfg.filter_low_hanging = false;
        cfg.filter_ledge = false;
        cfg.filter_low_height = false;
    }
    let explicit_areas = data[2] & 2 != 0;
    let mut r = Bytes(data[8..].iter());
    let mut world = FuzzWorld::default();
    for _ in 0..1 + data[3] % 24 {
        let t = [r.point(), r.point(), r.point()];
        let area = r.next() & 63;
        world.tris.push(Some((t, explicit_areas.then_some(area))));
    }
    for _ in 0..data[4] % 4 {
        let (x, z) = (r.coord(), r.coord());
        let (w, h) = (
            4.0 + f32::from(r.next() % 16),
            4.0 + f32::from(r.next() % 16),
        );
        world.volumes.push(Some((rect(x, z, w, h), r.next() & 63)));
    }
    for _ in 0..data[5] % 3 {
        let (a, b) = ([r.coord(), 1.0, r.coord()], [r.coord(), 1.0, r.coord()]);
        world.links.push(Some((a, b)));
    }
    let mut edits = Vec::new();
    for _ in 0..data[6] % 9 {
        let op = r.next() % 8;
        let id = usize::from(r.next());
        let edit = match op {
            0 => Edit::AddTri((
                [r.point(), r.point(), r.point()],
                explicit_areas.then_some(r.next() & 63),
            )),
            1 => Edit::MoveTri(
                id,
                [
                    r.coord() - WORLD / 2.0,
                    r.height() - 3.0,
                    r.coord() - WORLD / 2.0,
                ],
            ),
            2 => Edit::RemoveTri(id),
            3 => Edit::AddVolume(
                rect(r.coord(), r.coord(), 4.0 + f32::from(r.next() % 16), 6.0),
                r.next() & 63,
            ),
            4 => Edit::MoveVolume(id, [r.coord() - WORLD / 2.0, r.coord() - WORLD / 2.0]),
            5 => Edit::RemoveVolume(id),
            6 => Edit::AddLink([r.coord(), 1.0, r.coord()], [r.coord(), 1.0, r.coord()]),
            _ => Edit::RemoveLink(id),
        };
        edits.push(edit);
    }
    Some(Scenario {
        cfg,
        world,
        edits,
        param: data[7],
        explicit_areas,
    })
}

impl Scenario {
    pub fn layout(&self) -> Option<GridLayout> {
        let extent = self.cfg.recast.tile_size as f32 * self.cfg.recast.cs;
        let n = (WORLD / extent).ceil() as i32;
        GridLayout::new([0.0, -1.0, 0.0], 8.0, n, n, &self.cfg).ok()
    }
}

/// The tiles of a finished job, keyed, with its manifest; `None` if any tile failed.
pub type Baked = (BTreeMap<TileKey, Option<Vec<u8>>>, BakeManifest);

/// Bakes `keys` serially under steps of `allowance` work (at least 1), skipping tiles
/// unchanged since `prev`, and merges the result into `into`.
pub fn bake_keys(
    world: &FuzzWorld,
    cfg: &BakeConfig,
    grid: &GridLayout,
    keys: Vec<TileKey>,
    prev: Option<&BakeManifest>,
    allowance: u64,
) -> Option<(Vec<TileOutput>, Vec<ManifestEntry>)> {
    let mut job = BakeJob::new(world, cfg, grid, keys, JobOptions::default()).ok()?;
    if let Some(prev) = prev {
        job.skip_unchanged(prev, &mut Workspace::new()).ok()?;
    }
    let mut tiles = Vec::new();
    loop {
        let progress = job.step(&mut Budget::new(allowance.max(1))).ok()?;
        while let Some(t) = job.next_ready() {
            tiles.push(t);
        }
        if progress == Progress::Done {
            return Some((tiles, job.manifest_delta()));
        }
    }
}

/// A full bake of `world`.
pub fn full_bake(
    world: &FuzzWorld,
    cfg: &BakeConfig,
    grid: &GridLayout,
    allowance: u64,
) -> Option<Baked> {
    let (tiles, delta) = bake_keys(world, cfg, grid, grid.keys(), None, allowance)?;
    let mut manifest = BakeManifest::new(cfg, grid).ok()?;
    manifest.apply(&delta).ok()?;
    Some((
        tiles.into_iter().map(|t| (t.key, t.data)).collect(),
        manifest,
    ))
}

/// The world as a borrowed mesh, for the reference `bake::tiles`.
pub struct Mesh {
    pub verts: Vec<f32>,
    pub tris: Vec<i32>,
    pub areas: Vec<u8>,
    pub volumes: Vec<[f32; 12]>,
    pub volume_areas: Vec<u8>,
    pub links: Vec<f32>,
}

impl Mesh {
    pub fn of(world: &FuzzWorld) -> Self {
        let mut m = Mesh {
            verts: Vec::new(),
            tris: Vec::new(),
            areas: Vec::new(),
            volumes: Vec::new(),
            volume_areas: Vec::new(),
            links: Vec::new(),
        };
        for (t, a) in world.tris.iter().flatten() {
            for v in t {
                m.tris.push((m.verts.len() / 3) as i32);
                m.verts.extend_from_slice(v);
            }
            m.areas.push(a.unwrap_or(0));
        }
        for (r, a) in world.volumes.iter().flatten() {
            m.volumes.push(*r);
            m.volume_areas.push(*a);
        }
        for (a, b) in world.links.iter().flatten() {
            m.links.extend_from_slice(a);
            m.links.extend_from_slice(b);
        }
        m
    }

    /// Calls `f` with the borrowed input; `explicit` gives triangles their areas.
    pub fn with_input<R>(&self, explicit: bool, f: impl FnOnce(&Input<'_>) -> R) -> R {
        let volumes: Vec<AreaVolume<'_>> = self
            .volumes
            .iter()
            .zip(&self.volume_areas)
            .map(|(r, &a)| {
                let mut v = AreaVolume::default();
                v.verts = r;
                v.hmin = -1.0;
                v.hmax = 8.0;
                v.area = a;
                v
            })
            .collect();
        let n = self.links.len() / 6;
        let (rad, flags, areas, dir, ids) = (
            vec![0.6; n],
            vec![1u16; n],
            vec![0u8; n],
            vec![1u8; n],
            (0..n as u32).collect::<Vec<_>>(),
        );
        let mut input = Input::new(&self.verts, &self.tris);
        if explicit {
            input.areas = Some(&self.areas);
        }
        input.volumes = &volumes;
        input.off_mesh.off_mesh_con_count = n as i32;
        input.off_mesh.off_mesh_con_verts = &self.links;
        input.off_mesh.off_mesh_con_rad = &rad;
        input.off_mesh.off_mesh_con_flags = &flags;
        input.off_mesh.off_mesh_con_areas = &areas;
        input.off_mesh.off_mesh_con_dir = &dir;
        input.off_mesh.off_mesh_con_user_id = &ids;
        f(&input)
    }
}

/// The reference result: `bake::tiles` over the world's mesh, in row-major order.
pub fn reference(world: &FuzzWorld, s: &Scenario) -> Option<(GridLayout, Vec<Option<Vec<u8>>>)> {
    let mesh = Mesh::of(world);
    mesh.with_input(s.explicit_areas, |input| {
        let grid = GridLayout::from_input(input, &s.cfg).ok()?;
        let baked = bake::tiles(input, &s.cfg, &mut Workspace::new()).ok()?;
        Some((grid, baked.tiles.into_iter().map(|t| t.data).collect()))
    })
}
