//! Binding sector regions to the directed links of the installed mesh.
use super::summary::entry_mid;
use super::{
    SectorGraph, SectorKey, SectorParams, SectorSummary, SuperKey, SuperRecord, profile_hash,
};
use crate::detour::{DefaultFilter, NULL_LINK, NavMesh, PolyRef};
use crate::{Budget, Budgeted, Error, Limits};
use alloc::{
    collections::{BTreeMap, BTreeSet},
    vec::Vec,
};

#[derive(Clone, Copy, Debug, PartialEq)]
pub(super) struct Crossing {
    pub from: u16,
    pub sector: SectorKey,
    pub region: u16,
    pub exit: PolyRef,
    pub landing: PolyRef,
    pub position: [f32; 3],
}

impl SectorGraph {
    /// Builds the two graph levels from a fully installed world. Crossings follow the
    /// installed directed links, including their actual landing polygons. Build
    /// after global linking and modifier application, before streaming tiles out.
    pub fn from_mesh(
        nav: &NavMesh,
        params: &SectorParams,
        filter: &DefaultFilter,
        limits: Limits,
    ) -> Result<Self, Error> {
        let mut graph = Self::new(params, limits)?;
        let profile = profile_hash(filter);
        let mut keys = BTreeSet::new();
        for slot in 0..nav.max_tiles() as usize {
            if let Some(t) = nav.tile(slot) {
                graph.slot_keys.insert(
                    slot,
                    (
                        t.header.x,
                        t.header.y,
                        u8::try_from(t.header.layer)
                            .map_err(|_| Error::FormatLimit("tile layer"))?,
                    ),
                );
                keys.insert(params.sector_of_tile(t.header.x, t.header.y));
            }
        }
        for &key in &keys {
            let Budgeted::Complete(Some(s)) =
                SectorSummary::build(nav, key, params, filter, profile, &mut Budget::unlimited())?
            else {
                return Err(Error::LimitExceeded);
            };
            graph.sectors.insert((profile, key), s);
        }
        let locate = |r| -> Option<(SectorKey, u16)> {
            let (tile, _) = nav.tile_and_poly_by_ref(r).ok()?;
            let key = params.sector_of_tile(tile.header.x, tile.header.y);
            let region = graph.sector(profile, key)?.polygon_region(
                tile.header.x,
                tile.header.y,
                u8::try_from(tile.header.layer).ok()?,
                nav.decode_poly_id(r).2,
            )?;
            Some((key, region))
        };
        let mut crossings: BTreeMap<SectorKey, Vec<Crossing>> = BTreeMap::new();
        for slot in 0..nav.max_tiles() as usize {
            let Some(t) = nav.tile(slot) else { continue };
            let base = nav.poly_ref_base(slot).ok_or(Error::InvalidRef)?;
            for (i, p) in t.polys.iter().enumerate() {
                let r = base | i as PolyRef;
                let Some((key, from)) = locate(r) else {
                    continue;
                };
                let mut l = p.first_link;
                while l != NULL_LINK {
                    let link = t.links.get(l as usize).ok_or(Error::InvalidRef)?;
                    l = link.next;
                    if let Some((sector, region)) = locate(link.r#ref) {
                        if (key, from) == (sector, region) {
                            continue;
                        }
                        let target = nav.tile_and_poly_by_ref(link.r#ref)?;
                        let position =
                            entry_mid((t, p), r, link, target).ok_or(Error::InvalidRef)?;
                        crossings.entry(key).or_default().push(Crossing {
                            from,
                            sector,
                            region,
                            exit: r,
                            landing: link.r#ref,
                            position,
                        });
                    }
                }
            }
        }
        // Exits name the destination summary's content id (defined before stitching).
        let contents: BTreeMap<SectorKey, u32> = graph
            .sectors
            .iter()
            .map(|((_, key), s)| (*key, s.content_id()))
            .collect();
        for ((_, key), s) in &mut graph.sectors {
            let mut exits: Vec<_> = crossings
                .get(key)
                .into_iter()
                .flatten()
                .map(|c| {
                    let content = contents.get(&c.sector).copied().ok_or(Error::InvalidRef)?;
                    Ok((c.from, c.sector, c.region, content))
                })
                .collect::<Result<_, Error>>()?;
            exits.sort_unstable();
            exits.dedup();
            s.exits = Some(exits);
        }
        let mut groups: BTreeMap<SuperKey, Vec<&SectorSummary>> = BTreeMap::new();
        for ((_, key), s) in &graph.sectors {
            groups.entry(params.super_of(*key)).or_default().push(s);
        }
        // Summaries built from the installed mesh, like the builds above, run
        // without a work limit.
        let mut budget = Budget::unlimited();
        for (sk, members) in groups {
            let (record, _) = SuperRecord::derive_budgeted(sk, params, &members, &mut budget)?;
            graph.supers.insert(
                (profile, sk),
                super::graph::SuperNode {
                    record,
                    stale: false,
                    edges: Vec::new(),
                },
            );
        }
        for sk in graph.supers.keys().map(|p| p.1).collect::<Vec<_>>() {
            let edges = graph.edges_of(profile, sk, &mut budget)?;
            if let Some(node) = graph.supers.get_mut(&(profile, sk)) {
                node.edges = edges;
            }
        }
        graph.rebuild_inbound();
        graph.global_links = nav.global_off_mesh_links();
        graph.bound_profile = Some(profile);
        graph.bound_filter = Some(filter.clone());
        graph.owner = Some(nav.report_identity().clone());
        graph.crossings = crossings;
        graph.bound_epoch = Some(nav.epoch());
        #[cfg(feature = "dynamic")]
        {
            graph.state_epoch = nav.state_epoch();
        }
        Ok(graph)
    }
}
