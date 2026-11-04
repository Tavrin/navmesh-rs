// Independent corpus checker: traverse raw links rather than the labeler's union-find.
use navmesh_rs::connectivity::Connectivity;
use navmesh_rs::detour::{NULL_LINK, NavMesh, PolyRef, prepare_tile};
use navmesh_rs::{Budget, Budgeted};
use std::collections::{BTreeMap, BTreeSet};

type Graph = BTreeMap<PolyRef, BTreeSet<PolyRef>>;
fn flood(graph: &Graph, seed: PolyRef) -> BTreeSet<PolyRef> {
    let mut seen = BTreeSet::from([seed]);
    let mut todo = vec![seed];
    while let Some(r) = todo.pop() {
        for &to in &graph[&r] {
            if seen.insert(to) {
                todo.push(to);
            }
        }
    }
    seen
}
fn check(nav: &NavMesh, conn: &Connectivity) -> Result<(), String> {
    let mut directed = Graph::new();
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
    let mut visited = BTreeSet::new();
    for &r in weak.keys() {
        if visited.contains(&r) {
            continue;
        }
        let members = flood(&weak, r);
        let expected = members.first().copied();
        for &member in &members {
            if conn.island(member).map(|i| i.0) != expected {
                return Err("weak label differs from flood fill".into());
            }
        }
        visited.extend(members);
    }
    let seeds: Vec<_> = directed.keys().copied().collect();
    for index in [0, seeds.len() / 2, seeds.len().saturating_sub(1)] {
        let Some(&seed) = seeds.get(index) else {
            continue;
        };
        let coverage = conn
            .reachable_from(&[seed], &mut Budget::unlimited())
            .map_err(|e| e.to_string())?
            .into_value()
            .ok_or("missing coverage")?;
        if coverage.reachable_polys as usize != flood(&directed, seed).len() {
            return Err("directed coverage differs from flood fill".into());
        }
    }
    Ok(())
}
pub(super) fn verify(mut nav: NavMesh) -> Result<(), String> {
    let Budgeted::Complete(Some(mut conn)) =
        Connectivity::new(&nav, &mut Budget::unlimited()).map_err(|e| e.to_string())?
    else {
        return Err("unlimited build refused".into());
    };
    check(&nav, &conn)?;
    if let Some(slot) = (0..nav.max_tiles() as usize).find(|&slot| nav.tile(slot).is_some()) {
        let old = nav
            .unload_tile(nav.tile_ref(slot))
            .map_err(|e| e.to_string())?;
        conn.update(&nav, &mut Budget::unlimited())
            .map_err(|e| e.to_string())?;
        check(&nav, &conn)?;
        let tile =
            prepare_tile(nav.params(), nav.limits(), old.bytes).map_err(|e| e.to_string())?;
        nav.commit(tile).map_err(|e| e.to_string())?;
        conn.update(&nav, &mut Budget::unlimited())
            .map_err(|e| e.to_string())?;
        check(&nav, &conn)?;
    }
    println!(
        "connectivity: weak labels and directed coverage equal raw-link flood fills, including unload/reload"
    );
    Ok(())
}
