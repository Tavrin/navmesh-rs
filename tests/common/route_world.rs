//! Long-route measurement worlds and the independent reference search.
//!
//! [`RouteWorld`] is the generated [`World`] plus an optional seeded obstacle field
//! (box clusters standing in for buildings and rocks). The field is a pure function
//! of a seed and block coordinates, like the base world, so a world of any size
//! costs memory only for the tile being baked. Obstacles carry the null area: they
//! block the terrain beneath them without adding walkable roofs.
#![allow(dead_code)]

use crate::common::world::{MIN_Y, World};
use navmesh_rs::Error;
use navmesh_rs::bake::{GeometrySource, OffMeshSink, TriangleSink, VolumeSink};
use navmesh_rs::detour::{DefaultFilter, NULL_LINK, NavMesh, NavMeshQuery, PolyRef, QueryFilter};
use std::{
    cmp::Reverse,
    collections::{BTreeMap, BinaryHeap},
};

/// First triangle id of the obstacle field: above every terrain and edit id of a
/// world up to 2^19 metres on a side, so ids keep ascending within one fetch.
const OBSTACLE_ID_BASE: u64 = 1 << 44;

/// A seeded field of axis-aligned boxes, at most one per square block.
#[derive(Clone, Copy, Debug)]
pub struct ObstacleField {
    /// Seed of the placement hash, independent of the terrain seed.
    pub seed: u32,
    /// Edge of the blocks that may hold one obstacle, metres.
    pub block: f32,
    /// Blocks holding an obstacle, per 256 blocks.
    pub fill: u32,
    /// Smallest and largest obstacle edge, metres (drawn per axis).
    pub min_edge: f32,
    /// Largest obstacle edge, metres (drawn per axis).
    pub max_edge: f32,
    /// Obstacle top, metres; above the terrain everywhere.
    pub top: f32,
}

impl ObstacleField {
    /// The dense benchmark field: several hundred polygons per 76.8 m tile.
    pub fn dense(seed: u32) -> Self {
        Self {
            seed,
            block: 6.0,
            fill: 176,
            min_edge: 1.2,
            max_edge: 3.6,
            top: 14.0,
        }
    }

    fn hash(&self, bx: i32, bz: i32, salt: u32) -> u32 {
        let mut h = (bx as u32).wrapping_mul(0x9e37_79b1)
            ^ (bz as u32).wrapping_mul(0x85eb_ca6b)
            ^ (self.seed ^ salt).wrapping_mul(0xc2b2_ae35);
        h ^= h >> 15;
        h = h.wrapping_mul(0x2c1b_3c6d);
        h ^= h >> 12;
        h = h.wrapping_mul(0x297a_2d39);
        h ^ (h >> 15)
    }

    fn unit(&self, bx: i32, bz: i32, salt: u32) -> f32 {
        (self.hash(bx, bz, salt) >> 8) as f32 / 16_777_216.0
    }

    /// The box of block `(bx, bz)`, if the block holds one.
    pub fn obstacle(&self, bx: i32, bz: i32) -> Option<([f32; 3], [f32; 3])> {
        if self.hash(bx, bz, 0x0b57) % 256 >= self.fill {
            return None;
        }
        let span = self.max_edge - self.min_edge;
        let ex = self.min_edge + span * self.unit(bx, bz, 0x11);
        let ez = self.min_edge + span * self.unit(bx, bz, 0x22);
        let x = bx as f32 * self.block + (self.block - ex) * self.unit(bx, bz, 0x33);
        let z = bz as f32 * self.block + (self.block - ez) * self.unit(bx, bz, 0x44);
        Some(([x, MIN_Y, z], [x + ex, self.top, z + ez]))
    }

    /// Blocks per side of a world `size` metres wide.
    pub fn blocks(&self, size: f32) -> i32 {
        (size / self.block).ceil() as i32
    }

    fn push(
        &self,
        size: f32,
        origin: [f32; 2],
        bmin: [f32; 3],
        bmax: [f32; 3],
        out: &mut TriangleSink,
    ) -> Result<(), Error> {
        let n = self.blocks(size);
        let range = |lo: f32, hi: f32| {
            (
                ((lo / self.block).floor() as i32).clamp(0, n - 1),
                ((hi / self.block).floor() as i32).clamp(0, n - 1),
            )
        };
        let (x0, x1) = range(bmin[0] - origin[0], bmax[0] - origin[0]);
        let (z0, z1) = range(bmin[2] - origin[1], bmax[2] - origin[1]);
        for bz in z0..=z1 {
            for bx in x0..=x1 {
                let Some((min, max)) = self.obstacle(bx, bz) else {
                    continue;
                };
                let id = OBSTACLE_ID_BASE + (bz as u64 * n as u64 + bx as u64) * 12;
                for (k, t) in box_triangles(min, max).into_iter().enumerate() {
                    let t = t.map(|v| [v[0] + origin[0], v[1], v[2] + origin[1]]);
                    out.push(id + k as u64, t, Some(0))?;
                }
            }
        }
        Ok(())
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
        quad(c(0, 1, 0), c(0, 1, 1), c(1, 1, 1), c(1, 1, 0)),
        quad(c(0, 0, 0), c(1, 0, 0), c(1, 0, 1), c(0, 0, 1)),
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

/// The base world with an optional obstacle field on top.
#[derive(Clone, Debug)]
pub struct RouteWorld {
    /// Terrain, spanning wall, elevated slab and directed links.
    pub base: World,
    /// Optional seeded dense obstacle field.
    pub field: Option<ObstacleField>,
}

impl GeometrySource for RouteWorld {
    fn triangles(
        &self,
        bmin: [f32; 3],
        bmax: [f32; 3],
        out: &mut TriangleSink,
    ) -> Result<(), Error> {
        self.base.triangles(bmin, bmax, out)?;
        if let Some(field) = &self.field {
            field.push(self.base.size, self.base.origin, bmin, bmax, out)?;
        }
        Ok(())
    }

    fn volumes(&self, bmin: [f32; 3], bmax: [f32; 3], out: &mut VolumeSink) -> Result<(), Error> {
        self.base.volumes(bmin, bmax, out)
    }

    fn off_mesh(&self, bmin: [f32; 3], bmax: [f32; 3], out: &mut OffMeshSink) -> Result<(), Error> {
        self.base.off_mesh(bmin, bmax, out)
    }
}

/// Unbounded A* over incoming directed portals, returning the cost and the number
/// of expanded states. It shares no code with the production query beyond portal
/// geometry; position and incoming polygon are part of the state. Quality-only:
/// it is not a planner the game could run.
pub fn exact(
    nav: &NavMesh,
    start: PolyRef,
    goal: PolyRef,
    a: [f32; 3],
    b: [f32; 3],
    filter: &DefaultFilter,
) -> (f32, usize) {
    let q = NavMeshQuery::new(nav, 8).unwrap();
    let minimum = filter
        .area_cost
        .iter()
        .copied()
        .fold(f32::INFINITY, f32::min);
    let mut costs = BTreeMap::new();
    let mut open = BinaryHeap::new();
    costs.insert((start, 0), 0.0f32);
    open.push(Reverse((
        (distance(a, b) * minimum).to_bits(),
        0.0f32.to_bits(),
        start,
        0,
    )));
    let mut expanded = 0;
    while let Some(Reverse((_, bits, cur, previous))) = open.pop() {
        let cost = f32::from_bits(bits);
        if cur == 0 {
            return (cost, expanded);
        }
        if cost > costs[&(cur, previous)] {
            continue;
        }
        expanded += 1;
        let pos = if previous == 0 {
            a
        } else {
            let (a, b) = q.portal_points(previous, cur).unwrap();
            midpoint(a, b)
        };
        let (tile, poly) = nav.tile_and_poly_by_ref(cur).unwrap();
        if cur == goal {
            let terminal = cost + distance(pos, b) * filter.area_cost[poly.area() as usize];
            open.push(Reverse((terminal.to_bits(), terminal.to_bits(), 0, 0)));
        }
        let mut index = poly.first_link;
        while index != NULL_LINK {
            let link = tile.links[index as usize];
            index = link.next;
            let next = link.r#ref;
            if next == 0 {
                continue;
            }
            let (t, p) = nav.tile_and_poly_by_ref(next).unwrap();
            if !filter.pass_filter(next, t, p) {
                continue;
            }
            let (a, z) = q.portal_points(cur, next).unwrap();
            let mid = midpoint(a, z);
            let candidate = cost + distance(pos, mid) * filter.area_cost[poly.area() as usize];
            let key = (next, cur);
            if costs.get(&key).is_some_and(|old| *old <= candidate) {
                continue;
            }
            costs.insert(key, candidate);
            let h = distance(mid, b) * minimum;
            open.push(Reverse((
                (candidate + h).to_bits(),
                candidate.to_bits(),
                next,
                cur,
            )));
        }
    }
    (f32::INFINITY, expanded)
}

/// Euclidean distance in three dimensions.
pub fn distance(a: [f32; 3], b: [f32; 3]) -> f32 {
    a.into_iter()
        .zip(b)
        .map(|(x, y)| (x - y) * (x - y))
        .sum::<f32>()
        .sqrt()
}

/// Midpoint of a portal's endpoints.
pub fn midpoint(a: [f32; 3], b: [f32; 3]) -> [f32; 3] {
    core::array::from_fn(|i| (a[i] + b[i]) * 0.5)
}
