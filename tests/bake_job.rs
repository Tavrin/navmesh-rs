#![cfg(all(feature = "recast", feature = "detour"))]

mod common;

use std::collections::BTreeMap;
use std::sync::atomic::{AtomicUsize, Ordering};

use common::world::{Edit, World};
use navmesh_rs::bake::{
    self, AreaVolume, BakeConfig, BakeJob, BakeManifest, GeometrySource, GridLayout, Input,
    JobOptions, OffMeshSink, Partition, TileKey, TileOutput, TriangleSink, VolumeSink,
};
use navmesh_rs::{Budget, CancelToken, Error, Limits, Progress, Workspace};

fn drain<S: GeometrySource>(job: &mut BakeJob<'_, S>, out: &mut Vec<TileOutput>) {
    while let Some(t) = job.next_ready() {
        out.push(t);
    }
}

/// Runs a serial job to the end with `allowance` work per step.
fn serial<S: GeometrySource>(job: &mut BakeJob<'_, S>, allowance: u64) -> Vec<TileOutput> {
    let mut out = Vec::new();
    loop {
        let progress = job.step(&mut Budget::new(allowance)).unwrap();
        drain(job, &mut out);
        if progress == Progress::Done {
            return out;
        }
    }
}

fn keyed(tiles: &[TileOutput]) -> BTreeMap<TileKey, Option<Vec<u8>>> {
    tiles.iter().map(|t| (t.key, t.data.clone())).collect()
}

/// A floor with explicit areas, a volume and an off-mesh link, as an `Input`.
struct Scene {
    verts: Vec<f32>,
    tris: Vec<i32>,
    volume: Vec<f32>,
    links: [f32; 6],
}

impl Scene {
    fn new() -> Self {
        Self {
            verts: vec![0., 0., 0., 0., 0., 24., 24., 0., 24., 24., 0., 0.],
            tris: vec![0, 1, 2, 0, 2, 3],
            volume: vec![9., 0., 2., 9., 0., 12., 20., 0., 12., 20., 0., 2.],
            links: [3., 0., 3., 18., 0., 18.],
        }
    }

    fn input<'a>(&'a self, volumes: &'a [AreaVolume<'a>], areas: Option<&'a [u8]>) -> Input<'a> {
        let mut input = Input::new(&self.verts, &self.tris);
        input.volumes = volumes;
        input.areas = areas;
        input.off_mesh.off_mesh_con_count = 1;
        input.off_mesh.off_mesh_con_verts = &self.links;
        input.off_mesh.off_mesh_con_rad = &[0.5];
        input.off_mesh.off_mesh_con_flags = &[1];
        input.off_mesh.off_mesh_con_areas = &[0];
        input.off_mesh.off_mesh_con_dir = &[1];
        input.off_mesh.off_mesh_con_user_id = &[123];
        input
    }

    fn volumes(&self) -> Vec<AreaVolume<'_>> {
        let mut v = AreaVolume::default();
        v.verts = &self.volume;
        v.hmin = -1.;
        v.hmax = 2.;
        v.area = 3;
        vec![v]
    }
}

/// The scene's floor whose first triangle is marked by slope and second keeps area 5,
/// the mixed form of `Input { areas: Some([63, 5]) }`.
struct MixedFloor<'a>(Input<'a>);

impl GeometrySource for MixedFloor<'_> {
    fn triangles(&self, _: [f32; 3], _: [f32; 3], out: &mut TriangleSink) -> Result<(), Error> {
        let v = |i: usize| {
            [
                self.0.verts[i * 3],
                self.0.verts[i * 3 + 1],
                self.0.verts[i * 3 + 2],
            ]
        };
        out.push(0, [v(0), v(1), v(2)], None)?;
        out.push(1, [v(0), v(2), v(3)], Some(5))
    }
    fn volumes(&self, a: [f32; 3], b: [f32; 3], out: &mut VolumeSink) -> Result<(), Error> {
        self.0.volumes(a, b, out)
    }
    fn off_mesh(&self, a: [f32; 3], b: [f32; 3], out: &mut OffMeshSink) -> Result<(), Error> {
        self.0.off_mesh(a, b, out)
    }
}

#[test]
fn every_driver_reproduces_tiles_for_borrowed_and_mixed_area_sources() {
    let scene = Scene::new();
    let volumes = scene.volumes();
    let explicit_areas = [63u8, 5];
    for partition in [Partition::Watershed, Partition::Monotone, Partition::Layers] {
        let mut cfg = BakeConfig::default();
        cfg.partition = partition;
        for areas in [None, Some(&explicit_areas[..])] {
            let input = scene.input(&volumes, areas);
            let expected = bake::tiles(&input, &cfg, &mut Workspace::new()).unwrap();
            let grid = GridLayout::from_input(&input, &cfg).unwrap();
            assert_eq!(grid.keys().len(), expected.tiles.len());
            let expected: Vec<_> = expected.tiles.iter().map(|t| t.data.clone()).collect();
            let datas =
                |tiles: &[TileOutput]| -> Vec<_> { tiles.iter().map(|t| t.data.clone()).collect() };

            let make =
                || BakeJob::new(&input, &cfg, &grid, grid.keys(), JobOptions::default()).unwrap();
            // Invariant 2: any step sequence gives the same bytes in the same order.
            for allowance in [1, 7_000, u64::MAX] {
                assert_eq!(
                    datas(&serial(&mut make(), allowance)),
                    expected,
                    "allowance {allowance}"
                );
            }
            let mut job = make();
            job.run_parallel().unwrap();
            let mut got = Vec::new();
            drain(&mut job, &mut got);
            assert_eq!(datas(&got), expected);
            #[cfg(feature = "parallel")]
            for threads in [1, 4, 16] {
                let pool = rayon::ThreadPoolBuilder::new()
                    .num_threads(threads)
                    .build()
                    .unwrap();
                let mut job = make();
                pool.install(|| job.run_parallel().unwrap());
                let mut got = Vec::new();
                drain(&mut job, &mut got);
                assert_eq!(datas(&got), expected, "{threads} threads");
            }
        }
        // The source marks one triangle by slope and keeps area 5 on the other.
        let input = scene.input(&volumes, Some(&explicit_areas));
        let expected = bake::tiles(&input, &cfg, &mut Workspace::new()).unwrap();
        let mixed = MixedFloor(scene.input(&volumes, None));
        let grid = GridLayout::from_input(&input, &cfg).unwrap();
        let mut job =
            BakeJob::new(&mixed, &cfg, &grid, grid.keys(), JobOptions::default()).unwrap();
        let got = serial(&mut job, u64::MAX);
        let expected: Vec<_> = expected.tiles.iter().map(|t| t.data.clone()).collect();
        assert_eq!(
            got.iter().map(|t| t.data.clone()).collect::<Vec<_>>(),
            expected
        );
    }
}

fn small_cfg() -> BakeConfig {
    let mut cfg = BakeConfig::default();
    cfg.recast.tile_size = 64;
    cfg.recast.detail_sample_dist = 1.8;
    cfg
}

/// Bakes `keys` of `world` and returns the tiles and the manifest delta.
fn bake_keys(
    world: &World,
    cfg: &BakeConfig,
    grid: &GridLayout,
    keys: Vec<TileKey>,
    prev: Option<&BakeManifest>,
) -> (Vec<TileOutput>, Vec<navmesh_rs::bake::ManifestEntry>, usize) {
    let mut job = BakeJob::new(world, cfg, grid, keys, JobOptions::default()).unwrap();
    if let Some(prev) = prev {
        job.skip_unchanged(prev, &mut Workspace::new()).unwrap();
    }
    job.run_parallel().unwrap();
    let mut tiles = Vec::new();
    drain(&mut job, &mut tiles);
    (tiles, job.manifest_delta(), job.skipped())
}

#[test]
fn incremental_rebake_is_byte_identical_to_a_full_bake() {
    let cfg = small_cfg();
    let before = World::new(120.0, 11);
    let grid = before.layout(&cfg);
    let (tiles0, delta0, _) = bake_keys(&before, &cfg, &grid, grid.keys(), None);
    let mut manifest = BakeManifest::new(&cfg, &grid).unwrap();
    manifest.apply(&delta0).unwrap();
    let mut tiles = keyed(&tiles0);

    let edits = [
        Edit::Raise {
            min: [30.0, 30.0],
            max: [44.0, 52.0],
            dy: 3.0,
        },
        Edit::Hole {
            min: [70.0, 10.0],
            max: [80.0, 20.0],
        },
        Edit::Wall {
            min: [88.0, 0.0, 88.0],
            max: [96.0, 6.0, 91.0],
        },
        Edit::Area {
            min: [8.0, 90.0],
            max: [30.0, 110.0],
            area: 9,
        },
        Edit::Link {
            from: [100.0, 3.0, 40.0],
            to: [100.0, 3.0, 56.0],
        },
    ];
    let mut after = before.clone();
    after.edits.extend(edits);
    let boxes: Vec<[f32; 6]> = edits.iter().map(Edit::bounds).collect();
    let dirty = manifest.dirty_from_boxes(&boxes);
    assert!(
        !dirty.is_empty() && dirty.len() < grid.keys().len(),
        "{} dirty",
        dirty.len()
    );

    let (rebuilt, delta, skipped) = bake_keys(&after, &cfg, &grid, dirty.clone(), Some(&manifest));
    assert_eq!(rebuilt.len() + skipped, dirty.len());
    assert!(!rebuilt.is_empty());
    for t in &rebuilt {
        tiles.insert(t.key, t.data.clone());
    }
    manifest.apply(&delta).unwrap();

    let (full, full_delta, _) = bake_keys(&after, &cfg, &grid, grid.keys(), None);
    let mut full_manifest = BakeManifest::new(&cfg, &grid).unwrap();
    full_manifest.apply(&full_delta).unwrap();
    assert_eq!(tiles, keyed(&full));
    assert_eq!(manifest, full_manifest);
    assert!(tiles.values().any(Option::is_some));

    // The same world again: nothing is dirty, nothing is rebaked.
    let (none, delta, skipped) = bake_keys(&after, &cfg, &grid, grid.keys(), Some(&manifest));
    assert!(none.is_empty() && delta.is_empty());
    assert_eq!(skipped, grid.keys().len());
    // A different configuration invalidates every tile.
    let mut other = cfg;
    other.recast.max_simplification_error += 0.5;
    let other_grid = after.layout(&other);
    let job = BakeJob::new(
        &after,
        &other,
        &other_grid,
        other_grid.keys(),
        JobOptions::default(),
    );
    let mut job = job.unwrap();
    job.skip_unchanged(&manifest, &mut Workspace::new())
        .unwrap();
    assert_eq!(job.skipped(), 0);
}

/// Cancels the job the first time it is asked for geometry after `fetches` fetches.
struct CancelAfter<'a> {
    world: &'a World,
    fetches: AtomicUsize,
    after: usize,
    token: std::sync::Mutex<Option<CancelToken>>,
}

impl GeometrySource for CancelAfter<'_> {
    fn triangles(&self, a: [f32; 3], b: [f32; 3], out: &mut TriangleSink) -> Result<(), Error> {
        if self.fetches.fetch_add(1, Ordering::SeqCst) == self.after
            && let Some(t) = self.token.lock().unwrap().as_ref()
        {
            t.cancel();
        }
        self.world.triangles(a, b, out)
    }
    fn volumes(&self, a: [f32; 3], b: [f32; 3], out: &mut VolumeSink) -> Result<(), Error> {
        self.world.volumes(a, b, out)
    }
    fn off_mesh(&self, a: [f32; 3], b: [f32; 3], out: &mut OffMeshSink) -> Result<(), Error> {
        self.world.off_mesh(a, b, out)
    }
}

#[test]
fn cancel_returns_no_partial_tile_and_keeps_delivered_tiles_valid() {
    let cfg = small_cfg();
    let world = World::new(120.0, 3);
    let grid = world.layout(&cfg);
    let full = {
        let mut job =
            BakeJob::new(&world, &cfg, &grid, grid.keys(), JobOptions::default()).unwrap();
        keyed(&serial(&mut job, u64::MAX))
    };

    // Cancelled from outside between steps.
    let mut job = BakeJob::new(&world, &cfg, &grid, grid.keys(), JobOptions::default()).unwrap();
    let mut taken = Vec::new();
    while taken.len() < 2 {
        job.step(&mut Budget::new(50_000)).unwrap();
        drain(&mut job, &mut taken);
    }
    job.cancel_token().cancel();
    let err = job.step(&mut Budget::new(u64::MAX)).unwrap_err();
    assert_eq!(err.error, Error::Cancelled);
    assert!(job.next_ready().is_none() && job.run_parallel().is_err());
    assert_eq!(job.manifest_delta().len(), taken.len());
    for t in &taken {
        assert_eq!(&t.data, &full[&t.key]);
    }

    // Cancelled while a tile is being fetched: its first stage's work charge notices.
    let source = CancelAfter {
        world: &world,
        fetches: AtomicUsize::new(0),
        after: 2,
        token: std::sync::Mutex::new(None),
    };
    let mut job = BakeJob::new(&source, &cfg, &grid, grid.keys(), JobOptions::default()).unwrap();
    *source.token.lock().unwrap() = Some(job.cancel_token());
    let err = job.run_parallel().unwrap_err();
    assert_eq!(err.error, Error::Cancelled);
    assert!(err.tile.is_some(), "the in-flight tile is named");
    assert!(job.next_ready().is_none());
}

#[test]
fn manifest_round_trips_and_untrusted_bytes_are_validated() {
    let cfg = small_cfg();
    let world = World::new(60.0, 5);
    let grid = world.layout(&cfg);
    let (_, delta, _) = bake_keys(&world, &cfg, &grid, grid.keys(), None);
    let mut manifest = BakeManifest::new(&cfg, &grid).unwrap();
    manifest.apply(&delta).unwrap();
    let bytes = manifest.to_bytes();
    let limits = Limits::default();
    assert_eq!(BakeManifest::from_bytes(&bytes, limits).unwrap(), manifest);

    let invalid = |b: &[u8]| {
        matches!(
            BakeManifest::from_bytes(b, limits),
            Err(Error::InvalidInput(_))
        )
    };
    assert!(invalid(&bytes[..bytes.len() - 1]) && invalid(&bytes[..10]) && invalid(&[]));
    let mut longer = bytes.clone();
    longer.push(0);
    assert!(invalid(&longer));
    let mut wrong = bytes.clone();
    wrong[0] ^= 1;
    assert!(invalid(&wrong));
    // Entries start after the 60-byte header and take 25 bytes each; swap the first two.
    let mut unsorted = bytes.clone();
    let (a, b) = (60, 85);
    for i in 0..25 {
        unsorted.swap(a + i, b + i);
    }
    assert!(invalid(&unsorted));
    let mut flag = bytes.clone();
    flag[60 + 16] = 2;
    assert!(invalid(&flag));
    let mut outside = bytes.clone();
    outside[60..64].copy_from_slice(&1000i32.to_le_bytes());
    assert!(invalid(&outside));
    let small = Limits {
        max_bytes: bytes.len() - 1,
        ..Limits::default()
    };
    assert_eq!(
        BakeManifest::from_bytes(&bytes, small),
        Err(Error::LimitExceeded)
    );
    let few = Limits {
        max_work: 1,
        ..Limits::default()
    };
    assert_eq!(
        BakeManifest::from_bytes(&bytes, few),
        Err(Error::LimitExceeded)
    );
}

#[test]
fn dirty_boxes_include_the_halo_and_nothing_further() {
    let mut cfg = BakeConfig::default();
    cfg.recast.tile_size = 32; // 9.6 m tiles, 1.5 m halo
    let grid = GridLayout::new([0.0, 0.0, 0.0], 10.0, 4, 4, &cfg).unwrap();
    let manifest = BakeManifest::new(&cfg, &grid).unwrap();
    let keys = |boxes: &[[f32; 6]]| -> Vec<(i32, i32)> {
        manifest
            .dirty_from_boxes(boxes)
            .iter()
            .map(|k| (k.x, k.z))
            .collect()
    };
    // x = 9.0 is inside tile 0 and inside tile 1's halo (tile 1 starts at 9.6 - 1.5).
    assert_eq!(keys(&[[9.0, 0.0, 5.0, 9.0, 1.0, 5.0]]), [(0, 0), (1, 0)]);
    // x = 11.5 is past tile 0's halo (ends at 11.1) and before tile 2's (starts at 17.7).
    assert_eq!(keys(&[[11.5, 0.0, 5.0, 11.5, 1.0, 5.0]]), [(1, 0)]);
    // A box outside the grid dirties nothing; a box covering it dirties everything.
    assert!(keys(&[[-50.0, 0.0, -50.0, -40.0, 1.0, -40.0]]).is_empty());
    assert_eq!(keys(&[[-1e6, 0.0, -1e6, 1e6, 1.0, 1e6]]).len(), 16);
}

struct Unordered;

impl GeometrySource for Unordered {
    fn triangles(&self, _: [f32; 3], _: [f32; 3], out: &mut TriangleSink) -> Result<(), Error> {
        out.push(5, [[0., 0., 0.], [0., 0., 5.], [5., 0., 5.]], None)?;
        out.push(4, [[0., 0., 0.], [5., 0., 5.], [5., 0., 0.]], None)
    }
    fn volumes(&self, _: [f32; 3], _: [f32; 3], _: &mut VolumeSink) -> Result<(), Error> {
        Ok(())
    }
    fn off_mesh(&self, _: [f32; 3], _: [f32; 3], _: &mut OffMeshSink) -> Result<(), Error> {
        Ok(())
    }
}

#[test]
fn a_source_that_breaks_the_id_order_is_rejected_with_its_tile() {
    let cfg = BakeConfig::default();
    let grid = GridLayout::new([0.0, 0.0, 0.0], 10.0, 1, 1, &cfg).unwrap();
    let mut job =
        BakeJob::new(&Unordered, &cfg, &grid, grid.keys(), JobOptions::default()).unwrap();
    let err = job.step(&mut Budget::unlimited()).unwrap_err();
    assert!(matches!(err.error, Error::InvalidInput(_)));
    assert_eq!(err.tile, Some([0, 0]));
    // The failure is sticky.
    assert_eq!(job.step(&mut Budget::unlimited()).unwrap_err(), err);
}

#[test]
fn installer_adds_one_tile_per_unit_budget_and_never_caps_the_world() {
    let cfg = small_cfg();
    let world = World::new(120.0, 7);
    let grid = world.layout(&cfg);
    let (tiles, _, _) = bake_keys(&world, &cfg, &grid, grid.keys(), None);
    let non_empty = tiles.iter().filter(|t| t.data.is_some()).count();
    assert!(non_empty > 1);
    // The residency cap is the navmesh's parameter: here exactly as many slots as tiles.
    let mut nav =
        navmesh_rs::detour::NavMesh::new(&grid.nav_mesh_params(non_empty as i32, 1 << 16)).unwrap();
    let mut installer = bake::TileInstaller::new();
    tiles.into_iter().for_each(|t| installer.push(t));
    assert_eq!(installer.pending(), non_empty);
    let mut calls = 0;
    loop {
        let before = installer.installed();
        let progress = installer.install(&mut nav, &mut Budget::new(1)).unwrap();
        calls += 1;
        assert_eq!(
            installer.installed() - before,
            1,
            "one tile per unit-budget call"
        );
        if progress == Progress::Done {
            break;
        }
    }
    assert_eq!(calls, non_empty);
    let loaded = (0..nav.max_tiles() as usize)
        .filter(|&i| nav.tile(i).is_some())
        .count();
    assert_eq!(loaded, non_empty);
    // A larger budget installs several tiles per call.
    let mut nav =
        navmesh_rs::detour::NavMesh::new(&grid.nav_mesh_params(non_empty as i32, 1 << 16)).unwrap();
    let (tiles, _, _) = bake_keys(&world, &cfg, &grid, grid.keys(), None);
    tiles.into_iter().for_each(|t| installer.push(t));
    let mut budget = Budget::new(3);
    let progress = installer.install(&mut nav, &mut budget).unwrap();
    assert_eq!(progress, Progress::Exhausted);
    assert!(budget.is_exhausted() && installer.pending() > 0);
}
