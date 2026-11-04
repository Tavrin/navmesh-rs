#![no_main]
//! Invariants 2 and 3 of the bake job: any sequence of step budgets and any thread count give
//! the bytes and the order of `bake::tiles`, and a cancel at any step returns no tile after it
//! while the tiles delivered before it equal the reference prefix. The input layout is
//! documented in `navmesh_fuzz::bake_world` (edits are ignored here); the tail bytes after the
//! scenario are step allowances.
use libfuzzer_sys::fuzz_target;
use navmesh_fuzz::bake_world::{decode, reference};
use navmesh_rs::bake::{BakeJob, JobOptions};
use navmesh_rs::{Budget, Error, Progress};
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
});
