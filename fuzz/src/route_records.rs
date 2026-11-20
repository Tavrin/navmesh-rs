//! Stitched sector and super-sector records of a small linked grid, built once per
//! process, for the `sector_summary` and `super_record` targets.

use navmesh_rs::Limits;
use navmesh_rs::detour::{DefaultFilter, NavMesh, NavMeshParams};
use navmesh_rs::route::{SectorGraph, SectorKey, SectorParams, SuperKey};
use std::sync::OnceLock;

/// A stitched graph over a 3 x 2 grid of linked quads (one tile per sector, two
/// sectors per super-sector side, so crossings stay inside a super-sector and leave
/// it), with the encoded records it holds.
pub struct Stitched {
    /// The graph built from the installed mesh.
    pub graph: SectorGraph,
    /// Its grouping.
    pub params: SectorParams,
    /// Encoded stitched sector summaries.
    pub summaries: Vec<Vec<u8>>,
    /// Encoded stitched super-sector records.
    pub supers: Vec<Vec<u8>>,
}

/// The shared fixture.
pub fn stitched() -> &'static Stitched {
    static FIXTURE: OnceLock<Stitched> = OnceLock::new();
    FIXTURE.get_or_init(|| {
        let mut nav = NavMesh::new(&NavMeshParams {
            orig: [0.0; 3],
            tile_width: 10.0,
            tile_height: 10.0,
            max_tiles: 8,
            max_polys: 4,
        })
        .expect("grid parameters");
        for y in 0..2 {
            for x in 0..3 {
                nav.add_tile(crate::grid_quad(x, y, 0)).expect("grid tile");
            }
        }
        let filter = DefaultFilter::default();
        let params = SectorParams::new(1, 2);
        let graph = SectorGraph::from_mesh(&nav, &params, &filter, Limits::UNLIMITED)
            .expect("stitched grid graph");
        let profile = navmesh_rs::route::profile_hash(&filter);
        let summaries = (0..2)
            .flat_map(|z| (0..3).map(move |x| SectorKey { x, z }))
            .filter_map(|k| graph.sector(profile, k).map(|s| s.to_bytes()))
            .collect();
        let supers = [SuperKey { x: 0, z: 0 }, SuperKey { x: 1, z: 0 }]
            .iter()
            .filter_map(|k| graph.super_record(profile, *k).map(|r| r.to_bytes()))
            .collect();
        Stitched {
            graph,
            params,
            summaries,
            supers,
        }
    })
}
