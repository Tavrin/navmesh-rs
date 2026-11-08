#![no_main]
//! Random tile replacements on a 4 x 4 grid of linked quad tiles, checked against a
//! reference model.
//!
//! Layout: byte 0 picks the slot policy (bit 0: keyed), global off-mesh links
//! (bit 1) and, with 64-bit refs, the wide-salt ref layout (bit 2); then two bytes
//! per operation (at most 256): an opcode and an argument. Operations: replace
//! with a content variant, budgeted replace, replace with damaged bytes, replace
//! with a tile prepared for another layout, unload, remove, set runtime flags,
//! take a ref, and a residency save/restore into a fresh navmesh.
//!
//! Model: per tile key the resident bytes, the hash of an unloaded tile, runtime
//! flags and a content generation, bumped whenever the slot's salt must change.
//! Properties: equal content is `Unchanged` (same ref, epoch, flags); other
//! content is `Replaced` in the same slot with the next salt; a refusal (budget,
//! layout, bytes) changes nothing (epoch and residency record equal); every ref
//! taken so far is exactly `Stale` when its generation is behind, otherwise
//! `Valid` or `Unloaded`. Nothing panics.
use libfuzzer_sys::fuzz_target;
use navmesh_fuzz::grid_quad;
use navmesh_rs::detour::{
    NavMesh, NavMeshParams, PolyRef, RefLayout, RefState, Replaced, ResidencyRecord, SlotPolicy,
    TileKey, content_hash, prepare_tile, prepare_tile_with_layout,
};
use navmesh_rs::{Budget, Error, Limits};

const SIDE: i32 = 4;

#[derive(Clone, Default)]
struct Key {
    generation: u32,
    resident: Option<Vec<u8>>,
    parked: Option<[u8; 32]>,
    flags: Option<u16>,
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

fn new_nav(keyed: bool, global: bool, wide: bool) -> Option<NavMesh> {
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
    let mut nav = if wide {
        NavMesh::with_layout(&params(), Limits::UNLIMITED, policy, RefLayout::WIDE_SALT).ok()?
    } else {
        NavMesh::with_policy(&params(), Limits::UNLIMITED, policy).ok()?
    };
    nav.set_global_off_mesh_links(global);
    Some(nav)
}

fn xy(k: usize) -> (i32, i32) {
    (k as i32 % SIDE, k as i32 / SIDE)
}

fn resident_ref(nav: &NavMesh, k: usize) -> Option<PolyRef> {
    let (x, y) = xy(k);
    nav.tile_index_at(x, y, 0).map(|i| nav.tile_ref(i))
}

fn check(nav: &NavMesh, keys: &[Key], held: &[Held]) {
    for (k, m) in keys.iter().enumerate() {
        let r = resident_ref(nav, k);
        assert_eq!(r.is_some(), m.resident.is_some(), "residency of key {k}");
        if let (Some(r), Some(f)) = (r, m.flags) {
            assert_eq!(nav.poly_flags(r), Ok(f), "runtime flags of key {k}");
        }
    }
    for h in held {
        let k = &keys[h.key];
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
    }
}

/// Applies a replacement outcome to the model of key `k`, checking it against the
/// model's prediction.
fn record(
    nav: &NavMesh,
    m: &mut Key,
    before: Option<PolyRef>,
    epoch: u64,
    bytes: Vec<u8>,
    out: &Replaced,
) {
    let hash = content_hash(&bytes);
    match (out, m.resident.as_deref()) {
        (Replaced::Unchanged(r), Some(old)) => {
            assert_eq!(content_hash(old), hash, "unchanged needs equal content");
            assert_eq!((Some(*r), nav.epoch()), (before, epoch));
        }
        (Replaced::Replaced { new, old_salt }, Some(old)) => {
            assert_ne!(content_hash(old), hash, "equal content must not replace");
            let (salt, slot, _) = nav.decode_poly_id(*new);
            let b = before.expect("a replaced tile was resident");
            assert_eq!(
                (*old_salt, slot),
                (nav.decode_poly_id(b).0, nav.decode_poly_id(b).1)
            );
            assert_eq!(salt, old_salt + 1);
            assert_eq!(nav.epoch(), epoch + 1);
            m.generation += 1;
            m.resident = Some(bytes);
            m.flags = None;
        }
        (Replaced::Added(_), None) => {
            if m.parked.take().is_some_and(|p| p != hash) {
                m.generation += 1;
            }
            m.resident = Some(bytes);
            m.flags = None;
        }
        (Replaced::Refused(_), _) => {}
        _ => panic!("replacement outcome disagrees with the model"),
    }
}

fuzz_target!(|data: &[u8]| {
    let Some((&mode, ops)) = data.split_first() else {
        return;
    };
    let (keyed, global) = (mode & 1 != 0, mode & 2 != 0);
    let wide = mode & 4 != 0 && PolyRef::BITS == 64;
    let Some(mut nav) = new_nav(keyed, global, wide) else {
        return;
    };
    let n = (SIDE * SIDE) as usize;
    let mut keys = vec![Key::default(); n];
    let mut held: Vec<Held> = Vec::new();
    for op in ops.chunks_exact(2).take(256) {
        let (code, arg) = (op[0] % 9, op[1]);
        let k = arg as usize % n;
        let (x, y) = xy(k);
        let variant = (arg >> 4) & 3;
        let before = resident_ref(&nav, k);
        let (epoch, record_before) = (nav.epoch(), nav.residency().to_bytes());
        let unchanged = |nav: &NavMesh| {
            assert_eq!(nav.epoch(), epoch);
            assert_eq!(nav.residency().to_bytes(), record_before);
        };
        match code {
            0..=2 => {
                let mut bytes = grid_quad(x, y, variant);
                if code == 2 && !bytes.is_empty() {
                    let at = (usize::from(arg) * 7) % bytes.len();
                    if arg & 0x80 != 0 {
                        bytes.truncate(at);
                    } else {
                        bytes[at] ^= 1 << (arg & 7);
                    }
                }
                let prepared = prepare_tile_with_layout(
                    &params(),
                    Limits::UNLIMITED,
                    nav.ref_layout(),
                    bytes.clone(),
                );
                let Ok(t) = prepared else {
                    continue;
                };
                if t.key() != (TileKey { x, y }) || t.layer() != 0 {
                    continue;
                }
                let out = if code == 1 {
                    let cost = nav.replace_cost(&t);
                    let mut budget = Budget::new(u64::from(arg & 31));
                    let out = nav.replace_tile_budgeted(t, &mut budget);
                    if matches!(out, Ok(Replaced::Refused(_))) {
                        assert!(cost > u64::from(arg & 31));
                        unchanged(&nav);
                    } else {
                        assert_eq!(budget.used(), cost);
                    }
                    out
                } else {
                    nav.replace_tile(t)
                };
                match out {
                    Ok(out) => record(&nav, &mut keys[k], before, epoch, bytes, &out),
                    Err(e) => panic!("replace: {e:?}"),
                }
            }
            3 => {
                // A tile prepared for another layout is refused without change.
                let foreign = if wide {
                    prepare_tile(&params(), Limits::UNLIMITED, grid_quad(x, y, variant))
                } else if PolyRef::BITS == 64 {
                    prepare_tile_with_layout(
                        &params(),
                        Limits::UNLIMITED,
                        RefLayout::WIDE_SALT,
                        grid_quad(x, y, variant),
                    )
                } else {
                    continue;
                };
                let Ok(t) = foreign else {
                    continue;
                };
                assert!(matches!(
                    nav.replace_tile(t),
                    Err(Error::InvalidParameter(_))
                ));
                unchanged(&nav);
            }
            4 => {
                if let Some(r) = before {
                    let u = nav.unload_tile(r).expect("unload a resident tile");
                    let m = &mut keys[k];
                    m.parked = m.resident.take().map(|b| content_hash(&b));
                    assert_eq!(m.parked, Some(u.content_hash));
                    m.flags = None;
                }
            }
            5 => {
                if let Some(r) = before {
                    nav.remove_tile(r).expect("remove a resident tile");
                    let m = &mut keys[k];
                    m.resident = None;
                    m.flags = None;
                    m.generation += 1;
                }
            }
            6 => {
                if let Some(r) = before {
                    let f = u16::from(arg) | 0x100;
                    nav.set_poly_flags(r, f).expect("flags of a resident tile");
                    keys[k].flags = Some(f);
                }
            }
            7 => {
                if let Some(r) = before {
                    if held.len() < 64 {
                        held.push(Held {
                            r,
                            key: k,
                            generation: keys[k].generation,
                        });
                    }
                }
            }
            _ => {
                // Save and restore the residency, then commit the resident tiles
                // again by replacement (each is `Added` and keeps its ref).
                let rec = ResidencyRecord::from_bytes(&record_before).expect("own record parses");
                assert_eq!(rec.to_bytes(), record_before);
                let Some(mut fresh) = new_nav(keyed, global, wide) else {
                    return;
                };
                fresh.restore_residency(&rec).expect("own record restores");
                for (i, m) in keys.iter_mut().enumerate() {
                    if let Some(b) = m.resident.clone() {
                        let old = resident_ref(&nav, i);
                        let t = prepare_tile_with_layout(
                            &params(),
                            Limits::UNLIMITED,
                            fresh.ref_layout(),
                            b,
                        )
                        .expect("prepare");
                        let out = fresh.replace_tile(t).expect("replace after restore");
                        assert!(matches!(out, Replaced::Added(r) if Some(r) == old));
                        m.flags = None;
                    }
                }
                nav = fresh;
            }
        }
        check(&nav, &keys, &held);
    }
});
