#![no_main]
//! Arbitrary bytes as tile data through `prepare_tile` and `commit`, against
//! `add_tile` on an identical empty navmesh.
//!
//! Properties: nothing panics; `prepare_tile` and `add_tile` accept and reject the
//! same bytes with the same error; an accepted tile commits, gets the same ref as
//! `add_tile` gave, the same parsed tile and the same answers to a few queries; a
//! second commit at the same location is `TileSlotOccupied`.
use libfuzzer_sys::fuzz_target;
use navmesh_fuzz::exercise_queries;
use navmesh_rs::detour::{NavMesh, NavMeshParams, prepare_tile};
use navmesh_rs::{Error, Limits};

const TILE_LIMITS: Limits = Limits::UNLIMITED;

fuzz_target!(|data: &[u8]| {
    let params = NavMeshParams {
        orig: [0.0; 3],
        tile_width: 10.0,
        tile_height: 10.0,
        max_tiles: 4,
        max_polys: 1 << 14,
    };
    let (Ok(mut a), Ok(mut b)) = (
        NavMesh::with_limits(&params, TILE_LIMITS),
        NavMesh::with_limits(&params, TILE_LIMITS),
    ) else {
        return;
    };
    let added = a.add_tile(data.to_vec());
    let prepared = prepare_tile(&params, TILE_LIMITS, data.to_vec());
    match (&added, prepared) {
        (Ok(ra), Ok(t)) => {
            let again = t.clone();
            let mut padded = data.to_vec();
            padded.extend_from_slice(&[0; 8]);
            let padded_tile =
                prepare_tile(&params, TILE_LIMITS, padded).expect("trailing bytes accepted");
            assert_ne!(t.content_hash(), padded_tile.content_hash());
            let rb = b
                .commit(t)
                .expect("a prepared tile commits into an empty navmesh");
            assert_eq!(*ra, rb);
            let (ta, tb) = (a.tile_by_ref(*ra), b.tile_by_ref(rb));
            let (Some(ta), Some(tb)) = (ta, tb) else {
                panic!("committed tile missing");
            };
            assert_eq!(ta.data, tb.data);
            assert_eq!(ta.links.len(), tb.links.len());
            exercise_queries(&b);
            assert_eq!(b.commit(again), Err(Error::TileSlotOccupied));
            let u = b.unload_tile(rb).expect("unload");
            let before = b.residency().to_bytes();
            assert!(b.commit_with_state(padded_tile.clone(), &u.state).is_err());
            assert_eq!(b.residency().to_bytes(), before);
            assert_ne!(b.commit(padded_tile).expect("changed content"), rb);
            assert_eq!(b.ref_state(rb), navmesh_rs::detour::RefState::Stale);
        }
        (Err(ea), Err(eb)) => assert_eq!(*ea, eb),
        (a, b) => panic!("add_tile {a:?} but prepare_tile {:?}", b.map(|_| ())),
    }
});
