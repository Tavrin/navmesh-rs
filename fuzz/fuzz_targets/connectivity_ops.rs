#![no_main]
//! Bounded edit sequences: incremental weak labels and directed coverage equal
//! independent flood fills of the resident links, including atomic budget refusal.
use libfuzzer_sys::fuzz_target;
use navmesh_fuzz::grid_quad;
use navmesh_rs::connectivity::Connectivity;
use navmesh_rs::detour::{NULL_LINK, NavMesh, NavMeshParams, PolyRef, prepare_tile};
use navmesh_rs::{Budget, Budgeted};
use std::collections::{BTreeMap, BTreeSet};

fn build(nav: &NavMesh) -> Connectivity {
    match Connectivity::new(nav, &mut Budget::unlimited()).expect("graph") {
        Budgeted::Complete(Some(c)) => c,
        _ => panic!("unlimited graph"),
    }
}
fn flood(edges: &BTreeMap<PolyRef, BTreeSet<PolyRef>>, seed: PolyRef) -> BTreeSet<PolyRef> {
    let mut seen = BTreeSet::from([seed]);
    let mut todo = vec![seed];
    while let Some(r) = todo.pop() {
        for &to in &edges[&r] {
            if seen.insert(to) {
                todo.push(to);
            }
        }
    }
    seen
}
fuzz_target!(|data: &[u8]| {
    let params = NavMeshParams {
        orig: [0.0; 3],
        tile_width: 10.0,
        tile_height: 10.0,
        max_tiles: 32,
        max_polys: 16,
    };
    let mut nav = NavMesh::new(&params).expect("layout");
    let mut conn = build(&nav);
    let mut parked = BTreeMap::new();
    for op in data.chunks_exact(3).take(96) {
        let (x, y) = (i32::from(op[1] % 4), i32::from(op[2] % 4));
        match (op[0] % 4, nav.tile_index_at(x, y, 0)) {
            (0, None) => {
                let _ = nav.add_tile(grid_quad(x, y, op[0] >> 2));
            }
            (1, Some(slot)) => {
                if let Ok(u) = nav.unload_tile(nav.tile_ref(slot)) {
                    parked.insert((x, y), u.bytes);
                }
            }
            (2, Some(slot)) => {
                let _ = nav.remove_tile(nav.tile_ref(slot));
            }
            (3, None) => {
                if let Some(bytes) = parked.remove(&(x, y)) {
                    if let Ok(t) = prepare_tile(nav.params(), nav.limits(), bytes) {
                        let _ = nav.commit(t);
                    }
                }
            }
            _ => {}
        }
        let old_epoch = conn.epoch();
        assert!(
            !conn
                .update(&nav, &mut Budget::new(0))
                .expect("update")
                .is_complete()
        );
        assert_eq!(conn.epoch(), old_epoch);
        assert!(
            conn.update(&nav, &mut Budget::unlimited())
                .expect("update")
                .is_complete()
        );
        let fresh = build(&nav);
        let mut directed: BTreeMap<PolyRef, BTreeSet<PolyRef>> = BTreeMap::new();
        for slot in 0..nav.max_tiles() as usize {
            if let Some(tile) = nav.tile(slot) {
                for (ip, poly) in tile.polys.iter().enumerate() {
                    let r = nav.tile_ref(slot) | ip as PolyRef;
                    let edges = directed.entry(r).or_default();
                    let mut link = poly.first_link;
                    while link != NULL_LINK {
                        let l = &tile.links[link as usize];
                        if l.r#ref != 0 {
                            edges.insert(l.r#ref);
                        }
                        link = l.next;
                    }
                }
            }
        }
        let mut weak = directed.clone();
        for (&r, edges) in &directed {
            for &to in edges {
                weak.entry(to).or_default().insert(r);
            }
        }
        for &r in directed.keys() {
            assert_eq!(conn.island(r), fresh.island(r));
            assert_eq!(
                conn.island(r).expect("label").0,
                *flood(&weak, r).first().expect("component")
            );
            let coverage = conn
                .reachable_from(&[r], &mut Budget::unlimited())
                .expect("coverage")
                .into_value()
                .expect("complete");
            assert_eq!(coverage.reachable_polys as usize, flood(&directed, r).len());
        }
    }
});
