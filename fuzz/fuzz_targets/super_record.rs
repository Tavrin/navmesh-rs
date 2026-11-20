#![no_main]
//! Arbitrary bytes, and byte patches of a stitched record, as a super-sector record
//! (`'DNSR'`).
//!
//! Input: a mode byte. Bit 0 clear: the rest is the record. Bit 0 set: the next byte
//! picks one of the stitched fixture's records and the rest patches it
//! (`persisted::patched`).
//!
//! Properties: decoding under bounded limits never panics; an accepted record
//! re-encodes to the same bytes and reports its encoded length; installing it twice
//! into an empty graph is accepted or refused consistently; installing it into the
//! stitched fixture graph either fails with the previous record in place or leaves a
//! graph whose edges and indexes are computed without panicking.
use libfuzzer_sys::fuzz_target;
use navmesh_fuzz::persisted::{LIMITS, patched};
use navmesh_fuzz::route_records::stitched;
use navmesh_rs::route::{RouteIndex, SectorGraph, SectorParams, SuperRecord};

fuzz_target!(|data: &[u8]| {
    let Some((&mode, rest)) = data.split_first() else {
        return;
    };
    let fixture = stitched();
    let bytes = if mode & 1 == 1 {
        let Some((&pick, patches)) = rest.split_first() else {
            return;
        };
        let valid = &fixture.supers[usize::from(pick) % fixture.supers.len()];
        patched(valid, patches, false)
    } else {
        rest.to_vec()
    };
    let Ok(rec) = SuperRecord::from_bytes(&bytes, &LIMITS) else {
        return;
    };
    assert_eq!(rec.to_bytes(), bytes);
    assert_eq!(rec.encoded_len(), bytes.len());
    // Grouping fields sit at bytes 16..20 of the header.
    let per_super = u32::from(u16::from_le_bytes([bytes[16], bytes[17]]));
    let per_sector = u32::from(u16::from_le_bytes([bytes[18], bytes[19]]));
    let params = SectorParams::new(per_sector, per_super);
    if let Ok(mut graph) = SectorGraph::new(&params, LIMITS) {
        let first = graph.insert_super(&bytes).is_ok();
        assert_eq!(graph.insert_super(&bytes).is_ok(), first);
        if first {
            assert_eq!(graph.super_record(rec.profile(), rec.key()), Some(&rec));
            for r in 0..rec.regions().len() {
                let _ = graph.super_edges(rec.profile(), rec.key(), r as u16);
            }
        }
    }
    if params == fixture.params {
        let mut graph = fixture.graph.clone();
        let previous = graph.super_record(rec.profile(), rec.key()).cloned();
        if graph.insert_super(&bytes).is_ok() {
            for r in 0..rec.regions().len() {
                let _ = graph.super_edges(rec.profile(), rec.key(), r as u16);
            }
            let _ = RouteIndex::build(&graph, rec.profile());
            let _ = RouteIndex::build_exact(&graph, rec.profile());
        } else {
            assert_eq!(
                graph.super_record(rec.profile(), rec.key()),
                previous.as_ref()
            );
        }
    }
});
