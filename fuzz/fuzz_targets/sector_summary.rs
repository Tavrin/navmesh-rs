#![no_main]
//! Arbitrary bytes, and byte patches of a stitched record, as a sector summary record
//! (`'DNSS'`).
//!
//! Input: a mode byte. Bit 0 clear: the rest is the record. Bit 0 set: the next byte
//! picks one of the stitched fixture's summaries and the rest patches it
//! (`persisted::patched`).
//!
//! Properties: decoding under bounded limits never panics; an accepted record
//! re-encodes to the same bytes and reports its encoded length; inserting it into an
//! empty graph either fails or derives a super-sector record that itself round-trips;
//! inserting it into the stitched fixture graph either fails with the graph's counts
//! unchanged or leaves a graph whose indexes build or fail without panicking.
use libfuzzer_sys::fuzz_target;
use navmesh_fuzz::persisted::{LIMITS, patched};
use navmesh_fuzz::route_records::stitched;
use navmesh_rs::route::{RouteIndex, SectorGraph, SectorParams, SectorSummary, SuperRecord};

fuzz_target!(|data: &[u8]| {
    let Some((&mode, rest)) = data.split_first() else {
        return;
    };
    let fixture = stitched();
    let bytes = if mode & 1 == 1 {
        let Some((&pick, patches)) = rest.split_first() else {
            return;
        };
        let valid = &fixture.summaries[usize::from(pick) % fixture.summaries.len()];
        patched(valid, patches, false)
    } else {
        rest.to_vec()
    };
    let Ok(s) = SectorSummary::from_bytes(&bytes, &LIMITS) else {
        return;
    };
    assert_eq!(s.to_bytes(), bytes);
    assert_eq!(s.encoded_len(), bytes.len());
    let params = SectorParams::new(u32::from(s.tiles_per_sector()), 2);
    if let Ok(mut graph) = SectorGraph::new(&params, LIMITS) {
        if graph.insert_summary(s.clone()).is_ok() {
            let key = params.super_of(s.key());
            if let Some(rec) = graph.super_record(s.profile(), key) {
                let bytes = rec.to_bytes();
                assert_eq!(SuperRecord::from_bytes(&bytes, &LIMITS).as_ref(), Ok(rec));
            }
        }
    }
    if s.tiles_per_sector() == 1 {
        let mut graph = fixture.graph.clone();
        let before = (graph.sector_count(), graph.super_count());
        if graph.insert_summary(s.clone()).is_ok() {
            let _ = RouteIndex::build(&graph, s.profile());
            let _ = RouteIndex::build_exact(&graph, s.profile());
        } else {
            assert_eq!((graph.sector_count(), graph.super_count()), before);
        }
    }
});
