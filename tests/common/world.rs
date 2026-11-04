//! A generated world served one tile column at a time: the source-trait test harness.
//!
//! Terrain, area volumes and off-mesh links are pure functions of a seed and a list
//! of edits, so a world of any size costs memory only for the column being asked
//! about. Heights use only `+ - *` on `f32`, so they are identical on every target.
//! Reused by the incremental-bake tests, the size measurement and (by `#[path]`) the
//! fuzz targets.
#![allow(dead_code)]

use navmesh_rs::Error;
use navmesh_rs::bake::{
    BakeConfig, GeometrySource, GridLayout, OffMeshSink, TriangleSink, VolumeSink,
};

/// Terrain quad edge, metres.
const QUAD: f32 = 2.0;
/// Edge of the blocks that may hold an area volume, metres.
const VOLUME_BLOCK: f32 = 96.0;
/// Edge of the blocks that may hold an off-mesh link, metres.
const LINK_BLOCK: f32 = 160.0;
/// Reach of a volume (resp. link) beyond its block origin, metres.
const VOLUME_REACH: f32 = 64.0;
const LINK_REACH: f32 = 96.0;
/// Bottom and top of the world's heightfields, metres.
pub const MIN_Y: f32 = -2.0;
pub const MAX_Y: f32 = 24.0;

/// A change to the base world.
#[derive(Clone, Copy, Debug)]
pub enum Edit {
    /// Adds `dy` to the terrain height of every vertex inside the xz rectangle.
    Raise {
        min: [f32; 2],
        max: [f32; 2],
        dy: f32,
    },
    /// Removes the terrain quads whose centre lies inside the xz rectangle.
    Hole { min: [f32; 2], max: [f32; 2] },
    /// Adds an axis-aligned box (twelve triangles).
    Wall { min: [f32; 3], max: [f32; 3] },
    /// Adds a convex area volume (a rectangle) with the given area id.
    Area {
        min: [f32; 2],
        max: [f32; 2],
        area: u8,
    },
    /// Adds an off-mesh link between two points.
    Link { from: [f32; 3], to: [f32; 3] },
}

impl Edit {
    /// The xz box an edit can change, as `[min x, min y, min z, max x, max y, max z]`.
    pub fn bounds(&self) -> [f32; 6] {
        match *self {
            Edit::Raise { min, max, .. }
            | Edit::Hole { min, max }
            | Edit::Area { min, max, .. } => [
                min[0] - QUAD,
                MIN_Y,
                min[1] - QUAD,
                max[0] + QUAD,
                MAX_Y,
                max[1] + QUAD,
            ],
            Edit::Wall { min, max } => [min[0], min[1], min[2], max[0], max[1], max[2]],
            Edit::Link { from, to } => [
                from[0].min(to[0]),
                MIN_Y,
                from[2].min(to[2]),
                from[0].max(to[0]),
                MAX_Y,
                from[2].max(to[2]),
            ],
        }
    }
}

/// A square world, `size` metres on a side, with its minimum corner at the origin.
#[derive(Clone, Debug)]
pub struct World {
    pub size: f32,
    pub seed: u32,
    pub edits: Vec<Edit>,
    /// World position of the minimum corner; geometry is generated in `[0, size]` and shifted.
    pub origin: [f32; 2],
}

fn hash(x: i32, z: i32, seed: u32) -> u32 {
    let mut h = (x as u32).wrapping_mul(0x9e37_79b1)
        ^ (z as u32).wrapping_mul(0x85eb_ca6b)
        ^ seed.wrapping_mul(0xc2b2_ae35);
    h ^= h >> 15;
    h = h.wrapping_mul(0x2c1b_3c6d);
    h ^= h >> 12;
    h = h.wrapping_mul(0x297a_2d39);
    h ^ (h >> 15)
}

fn unit(x: i32, z: i32, seed: u32) -> f32 {
    (hash(x, z, seed) >> 8) as f32 / 16_777_216.0
}

/// Smooth value noise in `[0, 1]` with one lattice point every `period` vertices.
fn noise(ix: i32, iz: i32, period: i32, seed: u32) -> f32 {
    let (cx, cz) = (ix.div_euclid(period), iz.div_euclid(period));
    let fx = ix.rem_euclid(period) as f32 / period as f32;
    let fz = iz.rem_euclid(period) as f32 / period as f32;
    let s = |t: f32| t * t * (3.0 - 2.0 * t);
    let (sx, sz) = (s(fx), s(fz));
    let a = unit(cx, cz, seed);
    let b = unit(cx + 1, cz, seed);
    let c = unit(cx, cz + 1, seed);
    let d = unit(cx + 1, cz + 1, seed);
    let top = a + (b - a) * sx;
    let bottom = c + (d - c) * sx;
    top + (bottom - top) * sz
}

impl World {
    pub fn new(size: f32, seed: u32) -> Self {
        Self {
            size,
            seed,
            edits: Vec::new(),
            origin: [0.0, 0.0],
        }
    }

    pub fn with(mut self, edit: Edit) -> Self {
        self.edits.push(edit);
        self
    }

    /// A world centred on the origin: `[-size / 2, size / 2]` on both axes.
    pub fn centered(size: f32, seed: u32) -> Self {
        let mut w = Self::new(size, seed);
        w.origin = [-size / 2.0, -size / 2.0];
        w
    }

    fn up(&self, v: [f32; 3]) -> [f32; 3] {
        [v[0] + self.origin[0], v[1], v[2] + self.origin[1]]
    }

    fn down(&self, v: [f32; 3]) -> [f32; 3] {
        [v[0] - self.origin[0], v[1], v[2] - self.origin[1]]
    }

    fn up_rect(&self, mut r: [f32; 12]) -> [f32; 12] {
        for k in 0..4 {
            r[k * 3] += self.origin[0];
            r[k * 3 + 2] += self.origin[1];
        }
        r
    }

    fn quads(&self) -> i32 {
        (self.size / QUAD) as i32
    }

    fn height(&self, ix: i32, iz: i32) -> f32 {
        let mut h = 7.0 * noise(ix, iz, 24, self.seed) + 2.5 * noise(ix, iz, 7, self.seed ^ 0x55);
        let (x, z) = (ix as f32 * QUAD, iz as f32 * QUAD);
        for e in &self.edits {
            if let Edit::Raise { min, max, dy } = *e
                && x >= min[0]
                && x <= max[0]
                && z >= min[1]
                && z <= max[1]
            {
                h += dy;
            }
        }
        h
    }

    fn vertex(&self, ix: i32, iz: i32) -> [f32; 3] {
        self.up([ix as f32 * QUAD, self.height(ix, iz), iz as f32 * QUAD])
    }

    fn in_hole(&self, qx: i32, qz: i32) -> bool {
        let (cx, cz) = ((qx as f32 + 0.5) * QUAD, (qz as f32 + 0.5) * QUAD);
        self.edits.iter().any(|e| {
            matches!(*e, Edit::Hole { min, max }
                if cx >= min[0] && cx <= max[0] && cz >= min[1] && cz <= max[1])
        })
    }

    /// The layout that tiles the whole world with `cfg`.
    pub fn layout(&self, cfg: &BakeConfig) -> GridLayout {
        let extent = cfg.recast.tile_size as f32 * cfg.recast.cs;
        let n = (self.size / extent).ceil() as i32;
        GridLayout::new([self.origin[0], MIN_Y, self.origin[1]], MAX_Y, n, n, cfg).expect("layout")
    }

    fn volume_blocks(&self) -> i32 {
        (self.size / VOLUME_BLOCK).ceil() as i32
    }

    fn link_blocks(&self) -> i32 {
        (self.size / LINK_BLOCK).ceil() as i32
    }

    /// Item ids of base volumes are block indices; edits follow.
    fn base_volumes(&self) -> u64 {
        (self.volume_blocks() * self.volume_blocks()) as u64
    }

    fn base_links(&self) -> u64 {
        (self.link_blocks() * self.link_blocks()) as u64
    }
}

fn box_triangles(min: [f32; 3], max: [f32; 3]) -> [[[f32; 3]; 3]; 12] {
    let c = |x: usize, y: usize, z: usize| {
        [
            if x == 0 { min[0] } else { max[0] },
            if y == 0 { min[1] } else { max[1] },
            if z == 0 { min[2] } else { max[2] },
        ]
    };
    let quad = |a, b, d, e| [[a, b, d], [a, d, e]];
    let faces = [
        quad(c(0, 1, 0), c(0, 1, 1), c(1, 1, 1), c(1, 1, 0)), // top
        quad(c(0, 0, 0), c(1, 0, 0), c(1, 0, 1), c(0, 0, 1)), // bottom
        quad(c(0, 0, 0), c(0, 1, 0), c(1, 1, 0), c(1, 0, 0)),
        quad(c(0, 0, 1), c(1, 0, 1), c(1, 1, 1), c(0, 1, 1)),
        quad(c(0, 0, 0), c(0, 0, 1), c(0, 1, 1), c(0, 1, 0)),
        quad(c(1, 0, 0), c(1, 1, 0), c(1, 1, 1), c(1, 0, 1)),
    ];
    let mut out = [[[0.0; 3]; 3]; 12];
    for (i, f) in faces.iter().enumerate() {
        out[i * 2] = f[0];
        out[i * 2 + 1] = f[1];
    }
    out
}

fn rect(min: [f32; 2], max: [f32; 2]) -> [f32; 12] {
    [
        min[0], 0.0, min[1], min[0], 0.0, max[1], max[0], 0.0, max[1], max[0], 0.0, min[1],
    ]
}

impl World {
    fn base_volume(&self, bx: i32, bz: i32) -> Option<([f32; 12], u8)> {
        let h = hash(bx, bz, self.seed ^ 0x77);
        if !h.is_multiple_of(3) {
            return None;
        }
        let ox = bx as f32 * VOLUME_BLOCK + (h >> 8 & 31) as f32;
        let oz = bz as f32 * VOLUME_BLOCK + (h >> 14 & 31) as f32;
        Some((
            rect([ox, oz], [ox + 18.0, oz + 14.0]),
            4 + (h >> 20 & 3) as u8,
        ))
    }

    fn base_link(&self, bx: i32, bz: i32) -> Option<([f32; 3], [f32; 3])> {
        let h = hash(bx, bz, self.seed ^ 0x99);
        if !h.is_multiple_of(2) {
            return None;
        }
        let x = bx as f32 * LINK_BLOCK + (h >> 8 & 63) as f32 + 8.0;
        let z = bz as f32 * LINK_BLOCK + (h >> 14 & 63) as f32 + 8.0;
        let (dx, dz) = if h >> 20 & 1 == 0 {
            (12.0, 0.0)
        } else {
            (0.0, 12.0)
        };
        let y = |x: f32, z: f32| self.height((x / QUAD).round() as i32, (z / QUAD).round() as i32);
        Some(([x, y(x, z), z], [x + dx, y(x + dx, z + dz), z + dz]))
    }
}

impl GeometrySource for World {
    fn triangles(
        &self,
        bmin: [f32; 3],
        bmax: [f32; 3],
        out: &mut TriangleSink,
    ) -> Result<(), Error> {
        let (bmin, bmax) = (self.down(bmin), self.down(bmax));
        let n = self.quads();
        let range = |lo: f32, hi: f32| {
            (
                ((lo / QUAD).floor() as i32 - 1).clamp(0, n - 1),
                ((hi / QUAD).floor() as i32 + 1).clamp(0, n - 1),
            )
        };
        let (x0, x1) = range(bmin[0], bmax[0]);
        let (z0, z1) = range(bmin[2], bmax[2]);
        for qz in z0..=z1 {
            for qx in x0..=x1 {
                if self.in_hole(qx, qz) {
                    continue;
                }
                let id = (qz as u64 * n as u64 + qx as u64) * 2;
                let (a, b) = (self.vertex(qx, qz), self.vertex(qx, qz + 1));
                let (c, d) = (self.vertex(qx + 1, qz + 1), self.vertex(qx + 1, qz));
                out.push(id, [a, b, c], None)?;
                out.push(id + 1, [a, c, d], None)?;
            }
        }
        let base = (n as u64) * (n as u64) * 2;
        for (i, e) in self.edits.iter().enumerate() {
            if let Edit::Wall { min, max } = *e
                && min[0] <= bmax[0]
                && max[0] >= bmin[0]
                && min[2] <= bmax[2]
                && max[2] >= bmin[2]
            {
                for (k, t) in box_triangles(min, max).into_iter().enumerate() {
                    out.push(
                        base + i as u64 * 12 + k as u64,
                        t.map(|v| self.up(v)),
                        Some(1),
                    )?;
                }
            }
        }
        Ok(())
    }

    fn volumes(&self, bmin: [f32; 3], bmax: [f32; 3], out: &mut VolumeSink) -> Result<(), Error> {
        let (bmin, bmax) = (self.down(bmin), self.down(bmax));
        let n = self.volume_blocks();
        let range = |lo: f32, hi: f32| {
            (
                (((lo - VOLUME_REACH) / VOLUME_BLOCK).floor() as i32).clamp(0, n - 1),
                (((hi + VOLUME_REACH) / VOLUME_BLOCK).floor() as i32).clamp(0, n - 1),
            )
        };
        let (x0, x1) = range(bmin[0], bmax[0]);
        let (z0, z1) = range(bmin[2], bmax[2]);
        for bz in z0..=z1 {
            for bx in x0..=x1 {
                if let Some((verts, area)) =
                    self.base_volume(bx, bz).map(|(v, a)| (self.up_rect(v), a))
                {
                    out.push(
                        (bz * self.volume_blocks() + bx) as u64,
                        &verts,
                        -1.0,
                        40.0,
                        area,
                    )?;
                }
            }
        }
        for (i, e) in self.edits.iter().enumerate() {
            if let Edit::Area { min, max, area } = *e {
                out.push(
                    self.base_volumes() + i as u64,
                    &self.up_rect(rect(min, max)),
                    -1.0,
                    40.0,
                    area,
                )?;
            }
        }
        Ok(())
    }

    fn off_mesh(&self, bmin: [f32; 3], bmax: [f32; 3], out: &mut OffMeshSink) -> Result<(), Error> {
        let (bmin, bmax) = (self.down(bmin), self.down(bmax));
        let n = self.link_blocks();
        let range = |lo: f32, hi: f32| {
            (
                (((lo - LINK_REACH) / LINK_BLOCK).floor() as i32).clamp(0, n - 1),
                (((hi + LINK_REACH) / LINK_BLOCK).floor() as i32).clamp(0, n - 1),
            )
        };
        let (x0, x1) = range(bmin[0], bmax[0]);
        let (z0, z1) = range(bmin[2], bmax[2]);
        for bz in z0..=z1 {
            for bx in x0..=x1 {
                if let Some((from, to)) = self.base_link(bx, bz) {
                    out.push(
                        (bz * n + bx) as u64,
                        self.up(from),
                        self.up(to),
                        0.6,
                        1,
                        0,
                        true,
                        7,
                    )?;
                }
            }
        }
        for (i, e) in self.edits.iter().enumerate() {
            if let Edit::Link { from, to } = *e {
                out.push(
                    self.base_links() + i as u64,
                    self.up(from),
                    self.up(to),
                    0.6,
                    1,
                    0,
                    true,
                    9,
                )?;
            }
        }
        Ok(())
    }
}
