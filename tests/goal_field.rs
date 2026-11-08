//! Goal fields: costs equal an exact Dijkstra with the same cost model and tie rule,
//! values do not depend on tile slots, invalidation and rebuilds after streaming,
//! the byte-capped cache, and steering compared with corridor paths.

#![cfg(all(feature = "detour", feature = "crowd"))]

#[cfg(feature = "recast")]
mod common;

use std::collections::BTreeMap;

use navmesh_rs::crowd::field::{
    FieldId, FieldRegion, FieldStatus, FieldSteer, GoalField, GoalFieldCache,
};
use navmesh_rs::detour::{
    self, DefaultFilter, NULL_LINK, NavMesh, NavMeshCreateParams, NavMeshParams, NavMeshQuery,
    POLYTYPE_OFFMESH_CONNECTION, PolyRef, QueryFilter, TileKey,
};
use navmesh_rs::{Budget, Error, Limits, Progress};

const N: u16 = 0xffff;

/// A flat world of square tiles, each a lattice of quads, with holes, slow areas
/// and off-mesh connections.
struct Grid {
    tile: f32,
    quads: i32,
    tiles: i32,
    /// Inclusive rectangles of global quad coordinates `[x0, z0, x1, z1]`.
    holes: Vec<[i32; 4]>,
    /// Quads with area 1.
    slow: Vec<[i32; 4]>,
    /// `(start, end, bidirectional)`.
    links: Vec<([f32; 3], [f32; 3], bool)>,
}

fn inside(r: &[[i32; 4]], x: i32, z: i32) -> bool {
    r.iter()
        .any(|r| x >= r[0] && x <= r[2] && z >= r[1] && z <= r[3])
}

impl Grid {
    fn tile_bytes(&self, tx: i32, ty: i32) -> Option<Vec<u8>> {
        let q = self.quads;
        let n = q + 1;
        let mut verts = Vec::new();
        for iz in 0..n {
            for ix in 0..n {
                verts.extend([ix as u16, 0, iz as u16]);
            }
        }
        let (gx0, gz0) = (tx * q, ty * q);
        let mut index = vec![None; (q * q) as usize];
        let mut count = 0u16;
        for iz in 0..q {
            for ix in 0..q {
                if !inside(&self.holes, gx0 + ix, gz0 + iz) {
                    index[(iz * q + ix) as usize] = Some(count);
                    count += 1;
                }
            }
        }
        if count == 0 {
            return None;
        }
        let (mut polys, mut flags, mut areas) = (Vec::new(), Vec::new(), Vec::new());
        for iz in 0..q {
            for ix in 0..q {
                if index[(iz * q + ix) as usize].is_none() {
                    continue;
                }
                let v = |x: i32, z: i32| (z * n + x) as u16;
                let nei = |dx: i32, dz: i32, dir: u16| {
                    let (x, z) = (ix + dx, iz + dz);
                    if x < 0 || z < 0 || x >= q || z >= q {
                        0x8000 | dir
                    } else {
                        index[(z * q + x) as usize].unwrap_or(N)
                    }
                };
                polys.extend([
                    v(ix, iz),
                    v(ix, iz + 1),
                    v(ix + 1, iz + 1),
                    v(ix + 1, iz),
                    N,
                    N,
                ]);
                polys.extend([
                    nei(-1, 0, 0),
                    nei(0, 1, 1),
                    nei(1, 0, 2),
                    nei(0, -1, 3),
                    N,
                    N,
                ]);
                flags.push(1u16);
                areas.push(u8::from(inside(&self.slow, gx0 + ix, gz0 + iz)));
            }
        }
        let (x0, z0) = (tx as f32 * self.tile, ty as f32 * self.tile);
        let mine: Vec<_> = self
            .links
            .iter()
            .filter(|(s, _, _)| {
                s[0] >= x0 && s[0] < x0 + self.tile && s[2] >= z0 && s[2] < z0 + self.tile
            })
            .collect();
        let con_verts: Vec<f32> = mine
            .iter()
            .flat_map(|(s, e, _)| [*s, *e])
            .flatten()
            .collect();
        let nc = mine.len();
        let dirs: Vec<u8> = mine.iter().map(|l| u8::from(l.2)).collect();
        let data = detour::create_nav_mesh_data(&NavMeshCreateParams {
            verts: &verts,
            vert_count: n * n,
            polys: &polys,
            poly_flags: &flags,
            poly_areas: &areas,
            poly_count: i32::from(count),
            nvp: 6,
            off_mesh_con_verts: &con_verts,
            off_mesh_con_rad: &vec![0.5; nc],
            off_mesh_con_flags: &vec![1; nc],
            off_mesh_con_areas: &vec![0; nc],
            off_mesh_con_dir: &dirs,
            off_mesh_con_count: nc as i32,
            tile_x: tx,
            tile_y: ty,
            bmin: [x0, 0.0, z0],
            bmax: [x0 + self.tile, 1.0, z0 + self.tile],
            walkable_height: 2.0,
            walkable_radius: 0.5,
            walkable_climb: 0.5,
            cs: self.tile / q as f32,
            ch: 1.0,
            build_bv_tree: true,
            ..Default::default()
        })
        .unwrap();
        Some(data)
    }

    fn keys(&self) -> Vec<(i32, i32)> {
        (0..self.tiles)
            .flat_map(|y| (0..self.tiles).map(move |x| (x, y)))
            .collect()
    }

    fn nav(&self, reversed: bool) -> NavMesh {
        let mut nav = NavMesh::new(&NavMeshParams {
            orig: [0.0; 3],
            tile_width: self.tile,
            tile_height: self.tile,
            max_tiles: self.tiles * self.tiles,
            max_polys: 1 << 12,
        })
        .unwrap();
        let mut keys = self.keys();
        if reversed {
            keys.reverse();
        }
        for (x, y) in keys {
            if let Some(d) = self.tile_bytes(x, y) {
                nav.add_tile(d).unwrap();
            }
        }
        nav
    }
}

/// 4 x 4 tiles of 10 m with 2 m quads: a wall with one gap, a slow band and two
/// one-way off-mesh connections (one inside a tile, one across tiles).
fn small_grid() -> Grid {
    Grid {
        tile: 10.0,
        quads: 5,
        tiles: 4,
        holes: vec![[9, 0, 10, 14], [3, 6, 5, 7]],
        slow: vec![[0, 10, 19, 12]],
        links: vec![
            ([25.0, 0.0, 5.0], [15.0, 0.0, 5.0], false),
            ([31.0, 0.0, 33.0], [37.0, 0.0, 37.0], false),
        ],
    }
}

fn nearest(nav: &NavMesh, pos: [f32; 3]) -> (PolyRef, [f32; 3]) {
    let q = NavMeshQuery::new(nav, 64).unwrap();
    let (r, p, _) = q
        .find_nearest_poly(&pos, &[0.5, 2.0, 0.5], &DefaultFilter::default())
        .unwrap();
    assert_ne!(r, 0, "no polygon at {pos:?}");
    (r, p)
}

/// `(tile x, tile y, layer, polygon index)` of a ref.
fn key(nav: &NavMesh, r: PolyRef) -> (i32, i32, i32, u32) {
    let (_, it, ip) = nav.decode_poly_id(r);
    let h = &nav.tile(it as usize).unwrap().header;
    (h.x, h.y, h.layer, ip)
}

/// Every polygon of the region's resident tiles, in `(x, y, layer, poly)` order.
fn region_polys(nav: &NavMesh, columns: &[TileKey]) -> Vec<PolyRef> {
    let mut tiles: Vec<(i32, i32, i32, usize)> = columns
        .iter()
        .flat_map(|c| {
            nav.tiles_at(c.x, c.y, 64)
                .into_iter()
                .map(move |s| (c.x, c.y, s))
        })
        .map(|(x, y, s)| (x, y, nav.tile(s).unwrap().header.layer, s))
        .collect();
    tiles.sort_unstable();
    tiles
        .iter()
        .flat_map(|&(_, _, _, s)| {
            let base = nav.tile_ref(s);
            (0..nav.tile(s).unwrap().polys.len()).map(move |p| base | p as PolyRef)
        })
        .collect()
}

fn mid(l: [f32; 3], r: [f32; 3]) -> [f32; 3] {
    [
        (l[0] + r[0]) * 0.5,
        (l[1] + r[1]) * 0.5,
        (l[2] + r[2]) * 0.5,
    ]
}

/// Test-only exact Dijkstra: O(n^2) selection of the unsettled polygon with the
/// smallest `(cost, index)`, incoming edges from a brute-force scan of all links.
fn exact<F: QueryFilter>(
    nav: &NavMesh,
    columns: &[TileKey],
    goal: PolyRef,
    goal_pos: [f32; 3],
    filter: &F,
) -> BTreeMap<PolyRef, (f32, Option<PolyRef>)> {
    let q = NavMeshQuery::new(nav, 16).unwrap();
    let nodes = region_polys(nav, columns);
    let index: BTreeMap<PolyRef, usize> = nodes.iter().enumerate().map(|(i, &r)| (r, i)).collect();
    let n = nodes.len();
    let mut incoming = vec![Vec::new(); n];
    for (i, &r) in nodes.iter().enumerate() {
        let (t, p) = nav.tile_and_poly_by_ref(r).unwrap();
        let mut li = p.first_link;
        while li != NULL_LINK {
            let l = &t.links[li as usize];
            if let Some(&j) = index.get(&l.r#ref) {
                incoming[j].push(i);
            }
            li = l.next;
        }
    }
    for v in &mut incoming {
        v.sort_unstable();
        v.dedup();
    }
    let mut cost = vec![f32::INFINITY; n];
    let mut next: Vec<Option<usize>> = vec![None; n];
    let mut anchor = vec![[0.0f32; 3]; n];
    let mut done = vec![false; n];
    let g = index[&goal];
    cost[g] = 0.0;
    anchor[g] = goal_pos;
    loop {
        let mut best: Option<usize> = None;
        for i in 0..n {
            if !done[i] && cost[i].is_finite() && best.is_none_or(|b| cost[i] < cost[b]) {
                best = Some(i);
            }
        }
        let Some(b) = best else { break };
        done[b] = true;
        let (bt, bp) = nav.tile_and_poly_by_ref(nodes[b]).unwrap();
        let after = next[b].map(|k| {
            let (t, p) = nav.tile_and_poly_by_ref(nodes[k]).unwrap();
            (nodes[k], t, p)
        });
        for &p in &incoming[b] {
            if done[p] {
                continue;
            }
            let (pt, pp) = nav.tile_and_poly_by_ref(nodes[p]).unwrap();
            if !filter.pass_filter(nodes[p], pt, pp) {
                continue;
            }
            let Ok((l, r)) = q.portal_points(nodes[p], nodes[b]) else {
                continue;
            };
            let m = mid(l, r);
            let c = filter.cost(
                &m,
                &anchor[b],
                Some((nodes[p], pt, pp)),
                (nodes[b], bt, bp),
                after,
            );
            if !(c.is_finite() && c >= 0.0) {
                continue;
            }
            if cost[b] + c < cost[p] {
                cost[p] = cost[b] + c;
                next[p] = Some(b);
                anchor[p] = m;
            }
        }
    }
    nodes
        .iter()
        .enumerate()
        .map(|(i, &r)| (r, (cost[i], next[i].map(|k| nodes[k]))))
        .collect()
}

/// Builds `field` with `per_step` units per call; returns the number of calls.
fn build<F: QueryFilter>(field: &mut GoalField, nav: &NavMesh, filter: &F, per_step: u64) -> u32 {
    let mut calls = 0;
    loop {
        calls += 1;
        let mut b = Budget::new(per_step);
        if field.step(nav, filter, &mut b).unwrap() == Progress::Done {
            return calls;
        }
        assert!(calls < 1_000_000);
    }
}

fn assert_matches_exact<F: QueryFilter>(nav: &NavMesh, field: &GoalField, filter: &F) {
    let want = exact(nav, field.columns(), field.goal(), field.goal_pos(), filter);
    assert_eq!(field.polys() as usize, want.len());
    let mut reached = 0;
    for (&r, &(cost, next)) in &want {
        let got = field.cost(nav, r);
        if cost.is_finite() {
            reached += 1;
            assert_eq!(
                got.map(f32::to_bits),
                Some(cost.to_bits()),
                "cost of {r:#x}"
            );
        } else {
            assert_eq!(got, None, "cost of unreached {r:#x}");
        }
        assert_eq!(field.next(nav, r), next, "next of {r:#x}");
    }
    assert!(reached > want.len() / 2);
}

fn slow_filter() -> DefaultFilter {
    let mut f = DefaultFilter::default();
    f.area_cost[1] = 3.0;
    f
}

#[test]
fn field_costs_equal_exact_dijkstra_on_tiled_meshes() {
    let grid = small_grid();
    let nav = grid.nav(false);
    let filter = slow_filter();
    let (goal, goal_pos) = nearest(&nav, [5.0, 0.0, 5.0]);
    let region = FieldRegion::Radius(1000.0);
    // Resumed one unit at a time and in one call: the same values.
    let mut sliced = GoalField::new(&nav, goal, goal_pos, &region, &Limits::UNLIMITED).unwrap();
    assert_eq!(sliced.status(), FieldStatus::Building);
    assert!(build(&mut sliced, &nav, &filter, 1) > 100);
    assert_eq!(sliced.status(), FieldStatus::Ready);
    assert_matches_exact(&nav, &sliced, &filter);
    let mut whole = GoalField::new(&nav, goal, goal_pos, &region, &Limits::UNLIMITED).unwrap();
    assert_eq!(build(&mut whole, &nav, &filter, u64::MAX), 1);
    for r in region_polys(&nav, whole.columns()) {
        assert_eq!(
            whole.cost(&nav, r).map(f32::to_bits),
            sliced.cost(&nav, r).map(f32::to_bits)
        );
    }
    // The one-way connection across the wall leads towards the goal, so the
    // reverse index was used: its start polygon's next polygon is the connection.
    let (start, _) = nearest(&nav, [25.0, 0.0, 5.0]);
    let via = whole.next(&nav, start).unwrap();
    assert_eq!(
        nav.tile_and_poly_by_ref(via).unwrap().1.poly_type(),
        POLYTYPE_OFFMESH_CONNECTION
    );
    // A goal on the far side of a one-way connection: the field never leads
    // through it, and the exact search agrees.
    let (far, far_pos) = nearest(&nav, [31.0, 0.0, 31.0]);
    let tiles = FieldRegion::route(&[TileKey { x: 2, y: 2 }, TileKey { x: 1, y: 2 }], 1);
    let mut f = GoalField::new(&nav, far, far_pos, &tiles, &Limits::UNLIMITED).unwrap();
    build(&mut f, &nav, &filter, 7);
    assert_eq!(f.columns().len(), 12);
    assert_matches_exact(&nav, &f, &filter);
}

#[cfg(feature = "recast")]
#[test]
fn field_costs_equal_exact_dijkstra_on_a_baked_world() {
    use common::world::World;
    use navmesh_rs::bake::{BakeConfig, BakeJob, JobOptions};
    let mut cfg = BakeConfig::default();
    cfg.recast.tile_size = 64;
    cfg.recast.detail_sample_dist = 1.8;
    let world = World::new(120.0, 7);
    let grid = world.layout(&cfg);
    let mut job = BakeJob::new(&world, &cfg, &grid, grid.keys(), JobOptions::default()).unwrap();
    let mut tiles = Vec::new();
    loop {
        let p = job.step(&mut Budget::unlimited()).unwrap();
        while let Some(t) = job.next_ready() {
            tiles.push(t);
        }
        if p == Progress::Done {
            break;
        }
    }
    let mut nav = NavMesh::new(&grid.nav_mesh_params(tiles.len() as i32, 1 << 16)).unwrap();
    for t in tiles {
        if let Some(d) = t.data {
            nav.add_tile(d).unwrap();
        }
    }
    let q = NavMeshQuery::new(&nav, 64).unwrap();
    let filter = DefaultFilter::default();
    let (goal, goal_pos, _) = q
        .find_nearest_poly(&[60.0, 10.0, 60.0], &[4.0, 20.0, 4.0], &filter)
        .unwrap();
    assert_ne!(goal, 0);
    let mut field = GoalField::new(
        &nav,
        goal,
        goal_pos,
        &FieldRegion::Radius(45.0),
        &Limits::UNLIMITED,
    )
    .unwrap();
    build(&mut field, &nav, &filter, 64);
    println!(
        "{} columns, {} polygons",
        field.columns().len(),
        field.polys()
    );
    assert!(field.columns().len() > 4 && field.polys() > 10);
    assert_matches_exact(&nav, &field, &filter);
}

#[test]
fn field_values_do_not_depend_on_tile_slots() {
    let grid = small_grid();
    let filter = slow_filter();
    let a = grid.nav(false);
    let b = grid.nav(true);
    let fields: Vec<GoalField> = [&a, &b]
        .iter()
        .map(|nav| {
            let (goal, pos) = nearest(nav, [35.0, 0.0, 5.0]);
            let mut f = GoalField::new(
                nav,
                goal,
                pos,
                &FieldRegion::Radius(1000.0),
                &Limits::UNLIMITED,
            )
            .unwrap();
            build(&mut f, nav, &filter, 50);
            f
        })
        .collect();
    let values = |nav: &NavMesh, f: &GoalField| -> BTreeMap<_, _> {
        region_polys(nav, f.columns())
            .into_iter()
            .map(|r| {
                let next = f.next(nav, r).map(|n| key(nav, n));
                (key(nav, r), (f.cost(nav, r).map(f32::to_bits), next))
            })
            .collect()
    };
    let (va, vb) = (values(&a, &fields[0]), values(&b, &fields[1]));
    assert_ne!(
        a.decode_poly_id(fields[0].goal()).1,
        b.decode_poly_id(fields[1].goal()).1,
        "the two meshes use different slots"
    );
    assert_eq!(va, vb);
}

fn slot_at(nav: &NavMesh, x: i32, y: i32) -> usize {
    nav.tile_index_at(x, y, 0).unwrap()
}

#[test]
fn removed_tiles_invalidate_outward_and_rebuilds_match_fresh_fields() {
    let grid = small_grid();
    let filter = slow_filter();
    let mut nav = grid.nav(false);
    let (goal, goal_pos) = nearest(&nav, [5.0, 0.0, 5.0]);
    let region = FieldRegion::Radius(1000.0);
    let mut field = GoalField::new(&nav, goal, goal_pos, &region, &Limits::UNLIMITED).unwrap();
    build(&mut field, &nav, &filter, 100);
    let polys = region_polys(&nav, field.columns());
    let before: BTreeMap<PolyRef, (Option<f32>, Option<PolyRef>)> = polys
        .iter()
        .map(|&r| (r, (field.cost(&nav, r), field.next(&nav, r))))
        .collect();
    // Remove tile (1, 0): it holds the landing of the one-way connection across
    // the wall, which every polygon east of the wall and south of its gap uses.
    let gone = slot_at(&nav, 1, 0);
    let gone_ref = nav.tile_ref(gone);
    let through = |mut r: PolyRef| loop {
        if nav.decode_poly_id(r).1 as usize == gone {
            return true;
        }
        match before[&r].1 {
            Some(n) => r = n,
            None => return false,
        }
    };
    let expected: BTreeMap<PolyRef, Option<f32>> = polys
        .iter()
        .map(|&r| (r, if through(r) { None } else { before[&r].0 }))
        .collect();
    assert!(
        expected.values().filter(|v| v.is_none()).count()
            > before.values().filter(|v| v.0.is_none()).count() + 20
    );
    let data = nav.remove_tile(gone_ref).unwrap();
    assert!(field.refresh(&nav).unwrap());
    assert_eq!(field.status(), FieldStatus::Stale);
    for (&r, &want) in &expected {
        if nav.is_valid_poly_ref(r) {
            assert_eq!(field.cost(&nav, r), want, "{r:#x}");
        }
    }
    build(&mut field, &nav, &filter, 100);
    assert_eq!(field.status(), FieldStatus::Ready);
    assert_matches_exact(&nav, &field, &filter);
    // Committing the tile back: nothing is cleared, the rebuild finds the old routes.
    nav.add_tile(data).unwrap();
    assert!(field.refresh(&nav).unwrap());
    assert_eq!(field.status(), FieldStatus::Stale);
    assert!(
        field
            .cost(&nav, nearest(&nav, [35.0, 0.0, 35.0]).0)
            .is_some()
    );
    build(&mut field, &nav, &filter, 100);
    assert_matches_exact(&nav, &field, &filter);
    // No structural change: nothing to do.
    assert!(!field.refresh(&nav).unwrap());
    // Unloading the goal tile clears everything; reloading it (same refs) rebuilds.
    let home = nav.tile_ref(slot_at(&nav, 0, 0));
    nav.unload_tile(home).unwrap();
    field.step(&nav, &filter, &mut Budget::unlimited()).unwrap();
    assert_eq!(field.status(), FieldStatus::GoalLost);
    assert!(
        region_polys(&nav, field.columns())
            .iter()
            .all(|&r| field.cost(&nav, r).is_none())
    );
    let tile = detour::prepare_tile(
        nav.params(),
        Limits::UNLIMITED,
        grid.tile_bytes(0, 0).unwrap(),
    )
    .unwrap();
    assert_eq!(nav.commit(tile).unwrap(), home);
    build(&mut field, &nav, &filter, 100);
    assert_eq!(field.status(), FieldStatus::Ready);
    assert_matches_exact(&nav, &field, &filter);
    // Fields belong to their mesh.
    let other = grid.nav(false);
    assert_eq!(field.refresh(&other), Err(Error::MeshChanged));
}

#[test]
fn cache_shares_counts_refuses_and_evicts_least_recently_used() {
    let grid = small_grid();
    let nav = grid.nav(false);
    let filters = [slow_filter(), DefaultFilter::default()];
    let all = FieldRegion::Radius(1000.0);
    let goals: Vec<(PolyRef, [f32; 3])> = [[5.0, 0.0, 5.0], [35.0, 0.0, 35.0], [15.0, 0.0, 25.0]]
        .iter()
        .map(|&p| nearest(&nav, p))
        .collect();
    // Sizes of one ready field and of one request's estimate.
    let mut probe = GoalFieldCache::new(usize::MAX);
    let a = probe
        .request(&nav, goals[0].0, goals[0].1, 0, &all)
        .unwrap();
    assert_eq!(
        probe.step(&nav, &filters, &mut Budget::unlimited()),
        Ok(Progress::Done)
    );
    let ready = probe.bytes();
    let estimate = GoalField::peak_bytes(&nav, goals[1].0, goals[1].1, &all).unwrap();
    assert!(ready > 0 && ready < estimate);
    // An explicit region retains its own request identity.
    let columns: Vec<TileKey> = probe.get(a).unwrap().columns().to_vec();
    let again = probe
        .request(
            &nav,
            goals[0].0,
            goals[0].1,
            0,
            &FieldRegion::Tiles(columns),
        )
        .unwrap();
    assert_ne!(again, a);
    assert_eq!(probe.refs(a), 1);
    probe.release(again).unwrap();
    // Another filter index: another field.
    let other = probe
        .request(&nav, goals[0].0, goals[0].1, 1, &all)
        .unwrap();
    assert_ne!(other, a);
    assert_eq!(probe.pending(), 1);
    assert_eq!(
        probe.step(&nav, &filters[..1], &mut Budget::unlimited()),
        Err(Error::InvalidParameter("field filter index"))
    );

    let mut cache = GoalFieldCache::new(2 * ready + estimate - 1);
    let fa = cache
        .request(&nav, goals[0].0, goals[0].1, 0, &all)
        .unwrap();
    // Budgeted: an exhausted step leaves the field pending.
    assert_eq!(
        cache.step(&nav, &filters, &mut Budget::new(10)),
        Ok(Progress::Exhausted)
    );
    assert_eq!(cache.pending(), 1);
    cache
        .step(&nav, &filters, &mut Budget::unlimited())
        .unwrap();
    let fb = cache
        .request(&nav, goals[1].0, goals[1].1, 0, &all)
        .unwrap();
    cache
        .step(&nav, &filters, &mut Budget::unlimited())
        .unwrap();
    assert_eq!(cache.pending(), 0);
    // Both referenced: no room for a third, which is refused, not truncated.
    assert_eq!(
        cache.request(&nav, goals[2].0, goals[2].1, 0, &all),
        Err(Error::LimitExceeded)
    );
    cache.release(fa).unwrap();
    cache.release(fb).unwrap();
    assert_eq!(
        cache.release(fb),
        Err(Error::InvalidParameter("field is not referenced"))
    );
    assert_eq!(
        cache.len(),
        2,
        "unreferenced fields stay cached while they fit"
    );
    // Room for the third evicts the least recently used unreferenced field.
    let fc = cache
        .request(&nav, goals[2].0, goals[2].1, 0, &all)
        .unwrap();
    assert!(cache.get(fa).is_none());
    assert!(cache.get(fb).is_some());
    assert_eq!(cache.refs(fc), 1);
    // A re-requested cached field is reused; an evicted one gets a new id.
    assert_eq!(cache.request(&nav, goals[1].0, goals[1].1, 0, &all), Ok(fb));
    cache.release(fc).unwrap();
    let fa2: FieldId = cache
        .request(&nav, goals[0].0, goals[0].1, 0, &all)
        .unwrap();
    assert!(fa2 > fc);
    assert!(cache.get(fc).is_none());
    // Larger than the cap on its own.
    let mut tiny = GoalFieldCache::new(estimate - 1);
    assert_eq!(
        tiny.request(&nav, goals[0].0, goals[0].1, 0, &all),
        Err(Error::LimitExceeded)
    );
    assert!(tiny.is_empty());
    // A goal outside its region is a parameter error.
    let far = FieldRegion::Tiles(vec![TileKey { x: 3, y: 3 }]);
    assert!(matches!(
        GoalField::new(&nav, goals[0].0, goals[0].1, &far, &Limits::UNLIMITED),
        Err(Error::InvalidParameter(_))
    ));
}

fn dist_xz(a: [f32; 3], b: [f32; 3]) -> f32 {
    ((a[0] - b[0]).powi(2) + (a[2] - b[2]).powi(2)).sqrt()
}

/// Walks an agent along the field with fixed steps; returns the distance travelled.
fn follow(nav: &NavMesh, field: &GoalField, start: (PolyRef, [f32; 3])) -> f32 {
    let filter = DefaultFilter::default();
    let mut q = NavMeshQuery::new(nav, 256).unwrap();
    let (mut poly, mut pos) = start;
    let goal = field.goal_pos();
    let mut steer = FieldSteer::new();
    let mut visited = Vec::new();
    let mut travelled = 0.0;
    for _ in 0..4000 {
        if dist_xz(pos, goal) < 0.05 {
            return travelled;
        }
        let done = field
            .steer(&q, poly, &pos, &mut steer, &mut Budget::unlimited())
            .unwrap();
        assert_eq!(done, navmesh_rs::Budgeted::Complete(true));
        assert!(steer.stub().len() <= 4 && steer.stub()[0] == poly);
        let t = steer.target();
        let d = dist_xz(pos, t);
        let s = d.min(0.25);
        if d < 1e-5 {
            return travelled + dist_xz(pos, goal);
        }
        let want = [
            pos[0] + (t[0] - pos[0]) * s / d,
            pos[1],
            pos[2] + (t[2] - pos[2]) * s / d,
        ];
        let (res, _) = q
            .move_along_surface(poly, &pos, &want, &filter, &mut visited, 16)
            .unwrap();
        poly = *visited.last().unwrap();
        travelled += dist_xz(pos, res);
        pos = res;
    }
    panic!("agent did not arrive");
}

/// Length of the corridor path: A* polygons and the straight path through them.
fn corridor_length(nav: &NavMesh, start: (PolyRef, [f32; 3]), goal: (PolyRef, [f32; 3])) -> f32 {
    let filter = DefaultFilter::default();
    let mut q = NavMeshQuery::new(nav, 4096).unwrap();
    let mut path = Vec::new();
    q.find_path(start.0, goal.0, &start.1, &goal.1, &filter, &mut path, 1024)
        .unwrap();
    let mut pts = Vec::new();
    q.find_straight_path(&start.1, &goal.1, &path, &mut pts, 1024, 0)
        .unwrap();
    pts.windows(2).map(|w| dist_xz(w[0].pos, w[1].pos)).sum()
}

#[test]
fn field_agents_arrive_within_five_percent_of_corridor_agents() {
    // 40 m square, 1 m quads, a block in the middle and a wall stub: circle and
    // crossing scenes with one field per goal.
    let grid = Grid {
        tile: 10.0,
        quads: 10,
        tiles: 4,
        holes: vec![[16, 16, 23, 23], [30, 4, 31, 14]],
        slow: Vec::new(),
        links: Vec::new(),
    };
    let nav = grid.nav(false);
    let filter = DefaultFilter::default();
    let mut pairs = Vec::new();
    for k in 0..8 {
        let a = k as f32 * std::f32::consts::FRAC_PI_4 + 0.3;
        let at = |s: f32| [20.0 + s * 15.0 * a.cos(), 0.0, 20.0 + s * 15.0 * a.sin()];
        pairs.push((at(1.0), at(-1.0)));
    }
    for (s, g) in [
        ([2.5, 0.0, 20.5], [37.5, 0.0, 19.5]),
        ([20.5, 0.0, 2.5], [19.5, 0.0, 37.5]),
        ([37.5, 0.0, 30.5], [2.5, 0.0, 9.5]),
        ([9.5, 0.0, 37.5], [30.5, 0.0, 2.5]),
    ] {
        pairs.push((s, g));
    }
    let mut worst = 0.0f32;
    for (s, g) in pairs {
        let start = nearest(&nav, s);
        let goal = nearest(&nav, g);
        let mut field = GoalField::new(
            &nav,
            goal.0,
            goal.1,
            &FieldRegion::Radius(100.0),
            &Limits::UNLIMITED,
        )
        .unwrap();
        build(&mut field, &nav, &filter, u64::MAX);
        let walked = follow(&nav, &field, start);
        let corridor = corridor_length(&nav, start, goal);
        let ratio = walked / corridor;
        worst = worst.max((ratio - 1.0).abs());
        assert!(
            (0.95..=1.05).contains(&ratio),
            "{s:?} -> {g:?}: field {walked} m, corridor {corridor} m"
        );
    }
    println!(
        "largest field/corridor length deviation: {:.2}%",
        worst * 100.0
    );
}

/// Shared-goal cost on a 1 km world (40,000 polygons): one field plus one
/// steering call for each of 10,000 agents against 10,000 path requests of the
/// upstream path-queue size (4,096 nodes), in work units. Slow in debug builds:
/// `cargo test --release --test goal_field -- --ignored --nocapture`.
#[test]
#[ignore]
fn shared_goal_field_costs_under_ten_percent_of_path_requests() {
    let mut holes = Vec::new();
    // Walls every 200 m with gaps, so paths are not straight lines.
    for k in 1..5 {
        let x = k * 40;
        holes.push([x, 0, x, 69 + (k % 2) * 60]);
        holes.push([x, 81 + (k % 2) * 60, x, 199]);
    }
    let grid = Grid {
        tile: 50.0,
        quads: 10,
        tiles: 20,
        holes,
        slow: Vec::new(),
        links: Vec::new(),
    };
    let nav = grid.nav(false);
    let filter = DefaultFilter::default();
    let goal = nearest(&nav, [22.0, 0.0, 502.0]);
    let mut field = GoalField::new(
        &nav,
        goal.0,
        goal.1,
        &FieldRegion::Radius(1500.0),
        &Limits::UNLIMITED,
    )
    .unwrap();
    let mut field_work = Budget::unlimited();
    while field.step(&nav, &filter, &mut field_work).unwrap() != Progress::Done {}
    let mut q = NavMeshQuery::new(&nav, 4096).unwrap();
    let mut steer = FieldSteer::new();
    let mut steer_work = Budget::unlimited();
    let mut path_work = 0u64;
    let mut path = Vec::new();
    let mut agents = 0;
    for j in 0..100 {
        for i in 0..100 {
            let pos = [950.25 + 0.5 * i as f32, 0.0, 450.5 + j as f32];
            let start = nearest(&nav, pos);
            assert!(dist_xz(start.1, goal.1) > 900.0);
            let ok = field
                .steer(&q, start.0, &start.1, &mut steer, &mut steer_work)
                .unwrap();
            assert_eq!(ok, navmesh_rs::Budgeted::Complete(true));
            let mut b = Budget::unlimited();
            q.find_path_budgeted(
                start.0, goal.0, &start.1, &goal.1, &filter, &mut path, 256, &mut b,
            )
            .unwrap();
            path_work += b.used();
            agents += 1;
        }
    }
    let shared = field_work.used() + steer_work.used();
    println!(
        "{agents} agents, {} polygons: field {} + steering {} = {shared} units; path requests {path_work} units ({:.2}%)",
        field.polys(),
        field_work.used(),
        steer_work.used(),
        shared as f64 * 100.0 / path_work as f64
    );
    assert!(shared * 10 < path_work);
}

fn ready_field(nav: &NavMesh, region: &FieldRegion) -> GoalField {
    let (goal, pos) = nearest(nav, [5.0, 0.0, 5.0]);
    let mut field = GoalField::new(nav, goal, pos, region, &Limits::UNLIMITED).unwrap();
    build(&mut field, nav, &slow_filter(), 64);
    field
}

#[test]
fn bridge_added_mid_rebuild_restarts_and_tags_stale_values() {
    use navmesh_rs::crowd::field::FieldFreshness::{Fresh, Stale};
    let grid = small_grid();
    let mut nav = grid.nav(false);
    let mut field = ready_field(&nav, &FieldRegion::Radius(1000.0));
    let old_epoch = field.served_epoch();
    let tile = nav.tile_ref(slot_at(&nav, 1, 0));
    let bytes = nav.remove_tile(tile).unwrap();
    field
        .step(&nav, &slow_filter(), &mut Budget::new(1))
        .unwrap();
    assert_eq!(field.freshness(&nav), Some(Stale));
    assert_eq!(field.served_epoch(), old_epoch);
    nav.add_tile(bytes).unwrap();
    field
        .step(&nav, &slow_filter(), &mut Budget::new(1))
        .unwrap();
    assert_eq!(field.status(), FieldStatus::Stale);
    let q = NavMeshQuery::new(&nav, 16).unwrap();
    let mut out = FieldSteer::new();
    assert_eq!(
        field
            .steer(
                &q,
                field.goal(),
                &field.goal_pos(),
                &mut out,
                &mut Budget::unlimited()
            )
            .unwrap(),
        navmesh_rs::Budgeted::Complete(true)
    );
    assert_eq!(out.freshness(), Some(Stale));
    build(&mut field, &nav, &slow_filter(), 7);
    assert_eq!(field.freshness(&nav), Some(Fresh));
    assert_matches_exact(&nav, &field, &slow_filter());
}

#[test]
fn old_refs_and_dangling_next_are_refused_before_refresh_and_after_reuse() {
    let grid = small_grid();
    let mut nav = grid.nav(false);
    let field = ready_field(&nav, &FieldRegion::Radius(1000.0));
    let refs = region_polys(&nav, field.columns());
    let source = refs
        .iter()
        .copied()
        .find(|&r| field.next(&nav, r).is_some())
        .unwrap();
    let target = field.next(&nav, source).unwrap();
    let (_, slot, _) = nav.decode_poly_id(target);
    let bytes = nav.remove_tile(nav.tile_ref(slot as usize)).unwrap();
    assert_eq!(field.cost(&nav, target), None);
    assert_eq!(field.next(&nav, source), None);
    nav.add_tile(bytes).unwrap();
    assert_eq!(field.cost(&nav, target), None);
    assert_eq!(field.next(&nav, target), None);
    assert_eq!(field.next(&nav, source), None);
}

#[test]
fn zero_budget_refreshes_all_cached_fields_including_unreferenced() {
    let grid = small_grid();
    let mut nav = grid.nav(false);
    let mut cache = GoalFieldCache::new(usize::MAX);
    let mut ids = Vec::new();
    for pos in [[5.0, 0.0, 5.0], [35.0, 0.0, 35.0], [15.0, 0.0, 25.0]] {
        let (goal, pos) = nearest(&nav, pos);
        ids.push(
            cache
                .request(&nav, goal, pos, 0, &FieldRegion::Radius(1000.0))
                .unwrap(),
        );
    }
    cache
        .step(&nav, &[slow_filter()], &mut Budget::unlimited())
        .unwrap();
    cache.release(ids[2]).unwrap();
    nav.remove_tile(nav.tile_ref(slot_at(&nav, 1, 0))).unwrap();
    assert_eq!(
        cache.step(&nav, &[slow_filter()], &mut Budget::new(0)),
        Ok(Progress::Exhausted)
    );
    for id in ids {
        assert_eq!(cache.get(id).unwrap().status(), FieldStatus::Stale);
    }
}

#[test]
fn cache_keys_include_mesh_identity() {
    let grid = small_grid();
    let a = grid.nav(false);
    let b = grid.nav(false);
    let (goal, pos) = nearest(&a, [5.0, 0.0, 5.0]);
    let mut cache = GoalFieldCache::new(usize::MAX);
    let region = FieldRegion::Radius(1000.0);
    let x = cache.request(&a, goal, pos, 0, &region).unwrap();
    let y = cache.request(&b, goal, pos, 0, &region).unwrap();
    assert_ne!(x, y);
    cache
        .step(&b, &[slow_filter()], &mut Budget::unlimited())
        .unwrap();
    assert_eq!(cache.get(y).unwrap().status(), FieldStatus::Ready);
}

#[test]
fn radius_recomputes_columns_after_growth() {
    let grid = small_grid();
    // Start with narrower occupied bounds; removals intentionally retain bounds.
    let mut nav = NavMesh::new(&NavMeshParams {
        orig: [0.0; 3],
        tile_width: grid.tile,
        tile_height: grid.tile,
        max_tiles: 16,
        max_polys: 1 << 12,
    })
    .unwrap();
    for (x, y) in grid.keys().into_iter().filter(|&(x, _)| x < 3) {
        nav.add_tile(grid.tile_bytes(x, y).unwrap()).unwrap();
    }
    let bytes = grid.tile_bytes(3, 0).unwrap();
    let mut field = ready_field(&nav, &FieldRegion::Radius(1000.0));
    assert_eq!(field.columns().len(), 12);
    nav.add_tile(bytes).unwrap();
    build(&mut field, &nav, &slow_filter(), 7);
    let fresh = ready_field(&nav, &FieldRegion::Radius(1000.0));
    assert_eq!(field.columns(), fresh.columns());
    assert_eq!(field.columns().len(), 16);
    assert_matches_exact(&nav, &field, &slow_filter());
}

#[test]
fn steering_on_invalidated_goal_is_not_complete() {
    let grid = small_grid();
    let mut nav = grid.nav(false);
    let mut field = ready_field(&nav, &FieldRegion::Radius(1000.0));
    nav.unload_tile(nav.tile_ref(slot_at(&nav, 0, 0))).unwrap();
    field.refresh(&nav).unwrap();
    let tile = detour::prepare_tile(
        nav.params(),
        Limits::UNLIMITED,
        grid.tile_bytes(0, 0).unwrap(),
    )
    .unwrap();
    nav.commit(tile).unwrap();
    let q = NavMeshQuery::new(&nav, 16).unwrap();
    let mut out = FieldSteer::new();
    assert_eq!(
        field
            .steer(
                &q,
                field.goal(),
                &field.goal_pos(),
                &mut out,
                &mut Budget::unlimited()
            )
            .unwrap(),
        navmesh_rs::Budgeted::Complete(false)
    );
    assert!(!out.reaches_goal());
}

#[test]
fn columns_heavy_requests_obey_standalone_and_cache_limits() {
    let grid = small_grid();
    let nav = grid.nav(false);
    let (goal, pos) = nearest(&nav, [5.0, 0.0, 5.0]);
    let region = FieldRegion::Tiles((0..10000).map(|x| TileKey { x, y: 0 }).collect());
    let estimate = GoalField::peak_bytes(&nav, goal, pos, &region).unwrap();
    assert!(estimate >= 4 * 10000 * std::mem::size_of::<TileKey>());
    let mut limits = Limits::UNLIMITED;
    limits.max_bytes = estimate - 1;
    assert!(matches!(
        GoalField::new(&nav, goal, pos, &region, &limits),
        Err(Error::LimitExceeded)
    ));
    let mut cache = GoalFieldCache::new(estimate - 1);
    assert_eq!(
        cache.request(&nav, goal, pos, 0, &region),
        Err(Error::LimitExceeded)
    );
}

#[test]
fn custom_filter_removal_rebuilds_exactly_even_off_the_old_next_chain() {
    use navmesh_rs::detour::{MeshTile, Poly};
    struct PortalCost;
    impl QueryFilter for PortalCost {
        fn pass_filter(&self, _: PolyRef, _: &MeshTile, _: &Poly) -> bool {
            true
        }
        fn cost(
            &self,
            a: &[f32; 3],
            b: &[f32; 3],
            _: Option<(PolyRef, &MeshTile, &Poly)>,
            cur: (PolyRef, &MeshTile, &Poly),
            next: Option<(PolyRef, &MeshTile, &Poly)>,
        ) -> f32 {
            let bend = if next.is_some_and(|n| n.1.header.x < cur.1.header.x) {
                19.0
            } else {
                1.0
            };
            dist_xz(*a, *b) * bend + (a[2] - b[2]).abs() * 0.25
        }
    }
    let grid = small_grid();
    let mut nav = grid.nav(false);
    let (goal, pos) = nearest(&nav, [5.0, 0.0, 5.0]);
    let mut field = GoalField::new(
        &nav,
        goal,
        pos,
        &FieldRegion::Radius(1000.0),
        &Limits::UNLIMITED,
    )
    .unwrap();
    build(&mut field, &nav, &PortalCost, 17);
    assert_matches_exact(&nav, &field, &PortalCost);
    nav.remove_tile(nav.tile_ref(slot_at(&nav, 1, 0))).unwrap();
    field.step(&nav, &PortalCost, &mut Budget::new(0)).unwrap();
    assert_eq!(field.status(), FieldStatus::Stale);
    build(&mut field, &nav, &PortalCost, 1);
    assert_eq!(field.status(), FieldStatus::Ready);
    assert_matches_exact(&nav, &field, &PortalCost);
}
