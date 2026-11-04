#![no_main]
//! Invariants 2 and 3 of the bake job: any sequence of step budgets and any thread count give
//! the bytes and the order of `bake::tiles`, and a cancel at any step returns no tile after it
//! while the tiles delivered before it equal the reference prefix. The input layout is
//! documented in `navmesh_fuzz::bake_world` (edits seed the previous world of the install
//! check); the tail bytes after the scenario are step allowances. Also: a grown grid leaves
//! the tiles of the original grid as they were, an indexed mesh is the same source as the
//! plain one, and a job cancelled over a navmesh that holds the previous world's tiles
//! leaves each tile whole, new exactly where the manifest delta says.
use libfuzzer_sys::fuzz_target;
use navmesh_fuzz::bake_world::{Mesh, bake_keys, decode, reference};
use navmesh_rs::bake::{BakeJob, GridLayout, JobOptions, TileInstaller};
use navmesh_rs::detour::NavMesh;
use navmesh_rs::{Budget, Error, Progress, Workspace};
use std::sync::OnceLock;

fuzz_target!(|data: &[u8]| {
    let Some(s) = decode(data) else { return };
    let Some((grid, expected)) = reference(&s.world, &s) else {
        return;
    };
    let new_job =
        || BakeJob::new(&s.world, &s.cfg, &grid, grid.keys(), JobOptions::default()).unwrap();
    let allowance = |k: usize| u64::from(data[(8 + k) % data.len()]) * 64 + 1;

    // Steps of data-chosen budgets, cancelled after `cancel_at` steps when it is below the total.
    let cancel_at = usize::from(s.param);
    let mut job = new_job();
    let (mut got, mut steps) = (Vec::new(), 0usize);
    let cancelled = loop {
        if steps == cancel_at {
            job.cancel_token().cancel();
        }
        match job.step(&mut Budget::new(allowance(steps))) {
            Ok(progress) => {
                while let Some(t) = job.next_ready() {
                    got.push(t.data);
                }
                if progress == Progress::Done {
                    break false;
                }
            }
            Err(e) => {
                assert_eq!(e.error, Error::Cancelled);
                assert!(job.next_ready().is_none());
                break true;
            }
        }
        steps += 1;
    };
    if cancelled {
        assert!(got.len() <= expected.len());
        assert_eq!(got[..], expected[..got.len()]);
    } else {
        assert_eq!(got, expected);
    }

    // The parallel driver at 1, 4 and 16 threads.
    static POOLS: OnceLock<[rayon::ThreadPool; 3]> = OnceLock::new();
    let pools = POOLS.get_or_init(|| {
        [1, 4, 16].map(|n| {
            rayon::ThreadPoolBuilder::new()
                .num_threads(n)
                .build()
                .unwrap()
        })
    });
    let mut job = new_job();
    pools[usize::from(s.param) % 3].install(|| job.run_parallel().unwrap());
    let mut got = Vec::new();
    while let Some(t) = job.next_ready() {
        got.push(t.data);
    }
    assert_eq!(got, expected);

    // The same world on a grid that grew: the tiles of the original grid do not change.
    let grow = |bits: u8| i32::from(bits & 3).min(2);
    let layout = GridLayout::new(
        grid.origin,
        grid.max_y,
        grid.tiles_x + grow(s.param >> 2),
        grid.tiles_z + grow(s.param >> 4),
        &s.cfg,
    );
    if let Ok(grown) = layout {
        let mut job = BakeJob::new(
            &s.world,
            &s.cfg,
            &grown,
            grown.keys(),
            JobOptions::default(),
        )
        .unwrap();
        job.run_parallel().unwrap();
        while let Some(t) = job.next_ready() {
            if grid.contains(t.key) {
                let at = (t.key.z * grid.tiles_x + t.key.x) as usize;
                assert_eq!(t.data, expected[at], "tile {:?} on the grown grid", t.key);
            }
        }
    }

    // The indexed mesh is the same source as the plain one.
    Mesh::of(&s.world).with_input(s.explicit_areas, |input| {
        let indexed = input.indexed(&grid, &mut Workspace::new()).unwrap();
        let mut job =
            BakeJob::new(&indexed, &s.cfg, &grid, grid.keys(), JobOptions::default()).unwrap();
        job.run_parallel().unwrap();
        let mut got = Vec::new();
        while let Some(t) = job.next_ready() {
            got.push(t.data);
        }
        assert_eq!(got, expected);
    });

    // A rebake cancelled part-way, installed over the previous world's tiles: every tile
    // is whole, new if the job delivered it and as before otherwise.
    let mut edited = s.world.clone();
    let mut boxes = Vec::new();
    for e in &s.edits {
        edited.apply(e, &mut boxes);
    }
    let Some((before, _)) = bake_keys(&edited, &s.cfg, &grid, grid.keys(), None, u64::MAX) else {
        return;
    };
    let capacity = grid.keys().len() as i32;
    let mut nav = NavMesh::new(&grid.nav_mesh_params(capacity, 1 << 12)).unwrap();
    let mut installer = TileInstaller::new();
    before.iter().cloned().for_each(|t| installer.push(t));
    if installer
        .install(&mut nav, &mut Budget::unlimited())
        .is_err()
    {
        return;
    }
    let mut job = new_job();
    let (mut delivered, mut steps) = (Vec::new(), 0usize);
    loop {
        if steps == cancel_at {
            job.cancel_token().cancel();
        }
        let Ok(progress) = job.step(&mut Budget::new(allowance(steps))) else {
            break;
        };
        while let Some(t) = job.next_ready() {
            delivered.push(t);
        }
        if progress == Progress::Done {
            break;
        }
        steps += 1;
    }
    let delivered_keys: Vec<_> = delivered.iter().map(|t| t.key).collect();
    assert_eq!(
        job.manifest_delta()
            .entries
            .iter()
            .map(|e| e.key)
            .collect::<Vec<_>>(),
        delivered_keys
    );
    delivered.into_iter().for_each(|t| installer.push(t));
    installer
        .install(&mut nav, &mut Budget::unlimited())
        .unwrap();
    assert!(installer.clear().is_empty());
    for key in grid.keys() {
        let at = (key.z * grid.tiles_x + key.x) as usize;
        let resident = nav
            .tile_index_at(key.x, key.z, 0)
            .map(|i| nav.remove_tile(nav.tile_ref(i)).unwrap());
        let want = if delivered_keys.contains(&key) {
            expected[at].clone()
        } else {
            before
                .iter()
                .find(|t| t.key == key)
                .and_then(|t| t.data.clone())
        };
        assert_eq!(resident, want, "tile {key:?} after the cancel");
    }
});
