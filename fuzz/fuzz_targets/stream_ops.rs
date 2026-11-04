#![no_main]
//! Random streaming sequences on a 4 x 4 grid of linked quad tiles, checked
//! against a reference model.
//!
//! Layout: byte 0 picks the slot policy (bit 0: keyed) and whether global off-mesh
//! links are on (bit 1), and exercises boundary regressions (bit 2); then two
//! bytes per operation (at most 256): an opcode and
//! an argument. Operations: commit (tile, content variant), unload, remove, forget
//! an unloaded tile, commit with the state saved at unload, budgeted commit,
//! declare a column known, take refs, path and raycast queries, a resumable search
//! begun or resumed, and a residency save/restore into a fresh navmesh.
//!
//! Model: per tile key a content generation, bumped whenever the slot's salt must
//! change (a removal, a forgotten reservation, a commit of other content into a
//! reservation). Properties checked after every operation, for every ref taken so
//! far: `ref_state` is exactly `Stale` when its generation is behind, otherwise
//! `Valid` (resident) or `Unloaded(key)`; a non-`Valid` ref never resolves; a
//! `Valid` ref names the same polygon of the same tile key. A resumed search that
//! does not report `MeshChanged` holds only valid refs. A path frontier names only
//! unloaded columns. Nothing panics.
use libfuzzer_sys::fuzz_target;
use navmesh_fuzz::grid_quad;
use navmesh_rs::detour::{
    Committed, DefaultFilter, NavMesh, NavMeshParams, NavMeshQuery, PolyRef, PolysAroundSearch,
    RefState, ResidencyRecord, SlotPolicy, TileKey, prepare_tile,
};
use navmesh_rs::{Budget, Error, Limits};

const SIDE: i32 = 4;

#[derive(Clone, Copy, Default)]
struct Key {
    generation: u32,
    resident: Option<u8>,
    parked: Option<u8>,
}

struct Held {
    r: PolyRef,
    key: usize,
    generation: u32,
}

fn params() -> NavMeshParams {
    NavMeshParams {
        orig: [0.0; 3],
        tile_width: 10.0,
        tile_height: 10.0,
        max_tiles: 64,
        max_polys: 16,
    }
}

fn new_nav(keyed: bool, global: bool) -> Option<NavMesh> {
    let policy = if keyed {
        SlotPolicy::Keyed {
            origin: [0, 0],
            width: SIDE as u32,
            height: SIDE as u32,
            max_layers: 1,
        }
    } else {
        SlotPolicy::Sequential
    };
    let mut nav = NavMesh::with_policy(&params(), Limits::UNLIMITED, policy).ok()?;
    nav.set_global_off_mesh_links(global);
    Some(nav)
}

fn xy(k: usize) -> (i32, i32) {
    (k as i32 % SIDE, k as i32 / SIDE)
}

fn center(k: usize) -> [f32; 3] {
    let (x, y) = xy(k);
    [x as f32 * 10.0 + 5.0, 0.0, y as f32 * 10.0 + 5.0]
}

fn check(nav: &NavMesh, keys: &[Key], held: &[Held]) {
    for h in held {
        let k = keys[h.key];
        let (x, y) = xy(h.key);
        let want = if h.generation < k.generation {
            RefState::Stale
        } else if k.resident.is_some() {
            RefState::Valid
        } else if k.parked.is_some() {
            RefState::Unloaded(TileKey { x, y })
        } else {
            panic!("model: current-generation ref of a free slot");
        };
        assert_eq!(nav.ref_state(h.r), want, "ref {:#x}", h.r);
        match nav.tile_and_poly_by_ref(h.r) {
            Ok((t, _)) => {
                assert_eq!(want, RefState::Valid);
                assert_eq!((t.header.x, t.header.y), (x, y));
                assert_eq!(nav.decode_poly_id_poly(h.r), 0);
            }
            Err(_) => assert_ne!(want, RefState::Valid),
        }
    }
}

fuzz_target!(|data: &[u8]| {
    let Some((&mode, ops)) = data.split_first() else {
        return;
    };
    if mode & 4 != 0 {
        navmesh_fuzz::streaming_boundaries(ops);
    }
    let (keyed, global) = (mode & 1 != 0, mode & 2 != 0);
    let Some(mut nav) = new_nav(keyed, global) else {
        return;
    };
    let n = (SIDE * SIDE) as usize;
    let mut keys = vec![Key::default(); n];
    let mut states: Vec<Option<Vec<u8>>> = vec![None; n];
    let mut held: Vec<Held> = Vec::new();
    let mut search: Option<PolysAroundSearch> = None;
    let filter = DefaultFilter::default();
    for op in ops.chunks_exact(2).take(256) {
        let (code, arg) = (op[0] % 12, op[1]);
        let k = arg as usize % n;
        let (x, y) = xy(k);
        let variant = (arg >> 4) & 3;
        let tref = |nav: &NavMesh| nav.tile_index_at(x, y, 0).map(|i| nav.tile_ref(i));
        match code {
            0 | 1 => {
                let Ok(t) = prepare_tile(&params(), Limits::UNLIMITED, grid_quad(x, y, variant))
                else {
                    return;
                };
                let r = if code == 0 {
                    nav.commit(t).map(Some)
                } else {
                    match nav.commit_budgeted(t, &mut Budget::new(u64::from(arg & 15))) {
                        Ok(Committed::Done(r)) => Ok(Some(r)),
                        Ok(_) => Ok(None),
                        Err(e) => Err(e),
                    }
                };
                match r {
                    Ok(Some(_)) => {
                        let m = &mut keys[k];
                        assert!(m.resident.is_none());
                        if let Some(p) = m.parked.take() {
                            if p != variant {
                                m.generation += 1;
                            }
                        }
                        m.resident = Some(variant);
                    }
                    Ok(None) => {}
                    Err(Error::TileSlotOccupied) => assert!(keys[k].resident.is_some()),
                    Err(e) => panic!("commit: {e:?}"),
                }
            }
            2 => {
                if let Some(t) = tref(&nav) {
                    let u = nav.unload_tile(t).expect("unload a resident tile");
                    assert_eq!(u.key, TileKey { x, y });
                    let m = &mut keys[k];
                    m.parked = m.resident.take();
                    states[k] = Some(u.state);
                } else {
                    assert!(keys[k].resident.is_none());
                }
            }
            3 => {
                if let Some(t) = tref(&nav) {
                    nav.remove_tile(t).expect("remove a resident tile");
                    keys[k].resident = None;
                    keys[k].generation += 1;
                }
            }
            4 => match nav.forget_unloaded(TileKey { x, y }, 0) {
                Ok(()) => {
                    assert!(keys[k].parked.take().is_some());
                    keys[k].generation += 1;
                }
                Err(_) => assert!(keys[k].parked.is_none()),
            },
            5 => {
                // Commit with the state saved at the last unload of this key.
                let Some(mut state) = states[k].clone() else {
                    continue;
                };
                let Ok(t) = prepare_tile(&params(), Limits::UNLIMITED, grid_quad(x, y, variant))
                else {
                    return;
                };
                let before = nav.epoch();
                let same = keys[k].parked == Some(variant);
                let damaged = arg & 0x40 != 0;
                if damaged {
                    if let Some(last) = state.last_mut() {
                        *last ^= 1;
                    }
                }
                match nav.commit_with_state(t, &state) {
                    Ok(_) => {
                        assert!(
                            same && !damaged,
                            "state accepted for other content or a damaged hash"
                        );
                        let m = &mut keys[k];
                        m.parked = None;
                        m.resident = Some(variant);
                    }
                    Err(_) => assert_eq!(nav.epoch(), before),
                }
            }
            6 => nav.mark_known(TileKey { x, y }, arg & 1),
            7 => {
                if let Some(t) = tref(&nav) {
                    if held.len() < 64 {
                        held.push(Held {
                            r: t,
                            key: k,
                            generation: keys[k].generation,
                        });
                    }
                }
            }
            8 | 9 => {
                let q = NavMeshQuery::new(&nav, 64).expect("query");
                let mut q = q;
                let a = held
                    .iter()
                    .find(|h| nav.is_valid_poly_ref(h.r))
                    .map(|h| (h.r, h.key));
                let b = held
                    .iter()
                    .rev()
                    .find(|h| nav.is_valid_poly_ref(h.r))
                    .map(|h| (h.r, h.key));
                if let (Some((ra, ka)), Some((rb, kb))) = (a, b) {
                    if code == 8 {
                        let mut path = Vec::new();
                        if let Ok((st, frontier)) = q.find_path_report(
                            ra,
                            rb,
                            &center(ka),
                            &center(kb),
                            &filter,
                            &mut path,
                            32,
                        ) {
                            for f in &frontier {
                                assert!(nav.is_column_unloaded(f.x, f.y));
                            }
                            assert_eq!(
                                st.has_detail(navmesh_rs::detour::Status::UNLOADED_BORDER),
                                !frontier.is_empty()
                            );
                        }
                    } else {
                        let _ = q.raycast(ra, &center(ka), &center(kb), &filter, 16);
                    }
                }
            }
            10 => {
                let mut q = NavMeshQuery::new(&nav, 64).expect("query");
                match search.as_mut() {
                    None => {
                        if let Some(h) = held.iter().find(|h| nav.is_valid_poly_ref(h.r)) {
                            search = q
                                .begin_polys_around_circle(h.r, &center(h.key), 15.0, 64)
                                .ok();
                            if let Some(s) = search.as_mut() {
                                let _ = s.resume(&mut q, &filter, &mut Budget::new(1));
                            }
                        }
                    }
                    Some(s) => match s.resume(&mut q, &filter, &mut Budget::new(u64::from(arg))) {
                        Ok(_) => {
                            for &(r, _, _) in s.results() {
                                assert!(nav.is_valid_poly_ref(r), "search kept a dead ref");
                            }
                        }
                        Err(Error::MeshChanged) => search = None,
                        Err(e) => panic!("resume: {e:?}"),
                    },
                }
            }
            _ => {
                // Save and restore the residency into a fresh navmesh, then commit
                // the resident tiles again.
                search = None;
                let bytes = nav.residency().to_bytes();
                let rec = ResidencyRecord::from_bytes(&bytes).expect("own record parses");
                assert_eq!(rec.to_bytes(), bytes);
                let Some(mut fresh) = new_nav(keyed, global) else {
                    return;
                };
                fresh.restore_residency(&rec).expect("own record restores");
                for (i, m) in keys.iter_mut().enumerate() {
                    if let Some(v) = m.resident.take() {
                        let (x, y) = xy(i);
                        let t = prepare_tile(&params(), Limits::UNLIMITED, grid_quad(x, y, v))
                            .expect("prepare");
                        fresh.commit(t).expect("commit after restore");
                        m.resident = Some(v);
                    }
                }
                nav = fresh;
            }
        }
        check(&nav, &keys, &held);
    }
});
