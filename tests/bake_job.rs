#![cfg(all(feature = "recast", feature = "detour"))]

mod common;

use std::collections::BTreeMap;
use std::sync::atomic::{AtomicUsize, Ordering};

use common::world::{Edit, World};
use navmesh_rs::bake::{
    self, AreaVolume, BakeConfig, BakeJob, BakeManifest, GeometrySource, GridLayout, Input,
    JobOptions, ManifestDelta, OffMeshSink, Partition, TileKey, TileOutput, TriangleSink,
    VolumeSink,
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
) -> (Vec<TileOutput>, ManifestDelta, usize) {
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
    let dirty = manifest.dirty_from_boxes(&cfg, &grid, &boxes).unwrap();
    assert!(!dirty.full_rebake && dirty.removed.is_empty());
    let dirty = dirty.keys;
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
    assert!(none.is_empty() && delta.entries.is_empty());
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
    let comparable = job
        .skip_unchanged(&manifest, &mut Workspace::new())
        .unwrap();
    assert!(
        !comparable,
        "a manifest of another configuration is reported"
    );
    assert_eq!(job.skipped(), 0);
}

/// The documented edit flow for a world whose layout may change: plan with
/// `dirty_from_boxes`, retarget the manifest, bake the plan, merge the tiles.
fn rebake(
    world: &World,
    cfg: &BakeConfig,
    grid: &GridLayout,
    prev: &BakeManifest,
    prev_tiles: &Tiles,
    boxes: &[[f32; 6]],
) -> (Tiles, BakeManifest, bool) {
    let dirty = prev.dirty_from_boxes(cfg, grid, boxes).unwrap();
    let (rebuilt, delta, _) = bake_keys(world, cfg, grid, dirty.keys.clone(), Some(prev));
    let mut manifest = prev.clone();
    let mut dropped = manifest.retarget(cfg, grid).unwrap();
    dropped.sort();
    assert_eq!(dropped, dirty.removed);
    manifest.apply(&delta).unwrap();
    let mut tiles = prev_tiles.clone();
    tiles.retain(|k, _| grid.contains(*k) && !dirty.full_rebake);
    for t in &rebuilt {
        tiles.insert(t.key, t.data.clone());
    }
    (tiles, manifest, dirty.full_rebake)
}

type Tiles = BTreeMap<TileKey, Option<Vec<u8>>>;

fn baked(world: &World, cfg: &BakeConfig, grid: &GridLayout) -> (Tiles, BakeManifest) {
    let (tiles, delta, _) = bake_keys(world, cfg, grid, grid.keys(), None);
    let mut manifest = BakeManifest::new(cfg, grid).unwrap();
    manifest.apply(&delta).unwrap();
    (keyed(&tiles), manifest)
}

/// Edit boxes that cover the terrain a world gained or lost between `small` and `large`
/// metres on a side, with a margin for the terrain mesh's last quads.
fn rim_boxes(small: f32, large: f32) -> [[f32; 6]; 2] {
    use common::world::{MAX_Y, MIN_Y};
    let m = small - 4.0;
    [
        [m, MIN_Y, 0.0, large, MAX_Y, large],
        [0.0, MIN_Y, m, large, MAX_Y, large],
    ]
}

#[test]
fn growing_the_grid_dirties_the_new_tiles() {
    let cfg = small_cfg();
    let (before, after) = (World::new(120.0, 21), World::new(144.0, 21));
    let (old_grid, new_grid) = (before.layout(&cfg), after.layout(&cfg));
    assert!(new_grid.tiles_x > old_grid.tiles_x && new_grid.tiles_z > old_grid.tiles_z);
    assert_eq!(
        BakeManifest::config_hash(&cfg, &old_grid),
        BakeManifest::config_hash(&cfg, &new_grid),
        "growth alone keeps the configuration hash"
    );
    let (tiles, manifest) = baked(&before, &cfg, &old_grid);
    // The boxes alone would miss the new columns' tiles beyond the halo of the rim.
    let boxes = rim_boxes(120.0, 144.0);
    let (got, got_manifest, full) = rebake(&after, &cfg, &new_grid, &manifest, &tiles, &boxes);
    assert!(!full);
    let (want, want_manifest) = baked(&after, &cfg, &new_grid);
    assert_eq!(got, want);
    assert_eq!(got_manifest, want_manifest);
    // With no edit box at all, the new tiles are still named.
    let dirty = manifest.dirty_from_boxes(&cfg, &new_grid, &[]).unwrap();
    assert_eq!(
        dirty.keys.len(),
        new_grid.keys().len() - old_grid.keys().len()
    );
    assert!(dirty.keys.iter().all(|k| !old_grid.contains(*k)));
}

#[test]
fn shrinking_the_grid_reports_the_removed_tiles() {
    let cfg = small_cfg();
    let (before, after) = (World::new(144.0, 21), World::new(120.0, 21));
    let (old_grid, new_grid) = (before.layout(&cfg), after.layout(&cfg));
    let (tiles, manifest) = baked(&before, &cfg, &old_grid);
    let boxes = rim_boxes(120.0, 144.0);
    let dirty = manifest.dirty_from_boxes(&cfg, &new_grid, &boxes).unwrap();
    assert!(!dirty.full_rebake && !dirty.removed.is_empty());
    assert!(dirty.removed.iter().all(|k| !new_grid.contains(*k)));
    assert_eq!(
        dirty.removed.len(),
        old_grid.keys().len() - new_grid.keys().len()
    );
    let (got, got_manifest, _) = rebake(&after, &cfg, &new_grid, &manifest, &tiles, &boxes);
    let (want, want_manifest) = baked(&after, &cfg, &new_grid);
    assert_eq!(got, want);
    assert_eq!(got_manifest, want_manifest);
}

#[test]
fn a_layout_change_is_a_reported_full_rebake_and_a_stale_delta_is_refused() {
    use common::world::{MAX_Y, MIN_Y};
    let cfg = small_cfg();
    let world = World::new(120.0, 33);
    let old_grid = world.layout(&cfg);
    let (tiles, manifest) = baked(&world, &cfg, &old_grid);
    // An edit that raises the top of the world's height range changes every tile's column.
    let n = old_grid.tiles_x;
    let new_grid = GridLayout::new([0.0, MIN_Y, 0.0], MAX_Y + 8.0, n, n, &cfg).unwrap();
    assert_ne!(
        BakeManifest::config_hash(&cfg, &old_grid),
        BakeManifest::config_hash(&cfg, &new_grid)
    );
    let hit = [58.0, 0.0, 58.0, 61.0, 30.0, 61.0];
    let dirty = manifest.dirty_from_boxes(&cfg, &new_grid, &[hit]).unwrap();
    assert!(dirty.full_rebake && dirty.removed.is_empty());
    assert_eq!(dirty.keys, new_grid.keys());
    let (got, got_manifest, full) = rebake(&world, &cfg, &new_grid, &manifest, &tiles, &[hit]);
    assert!(full);
    let (want, want_manifest) = baked(&world, &cfg, &new_grid);
    assert_eq!(got, want);
    assert_eq!(got_manifest, want_manifest);
    // Merging the new bake into the old manifest without retargeting is refused.
    let (_, delta, _) = bake_keys(&world, &cfg, &new_grid, new_grid.keys(), None);
    let mut stale = manifest.clone();
    assert!(matches!(stale.apply(&delta), Err(Error::InvalidInput(_))));
    assert_eq!(stale, manifest);
}

#[test]
fn edit_boxes_must_be_finite() {
    let cfg = small_cfg();
    let world = World::new(60.0, 5);
    let grid = world.layout(&cfg);
    let (_, manifest) = baked(&world, &cfg, &grid);
    for bad in [f32::INFINITY, f32::NEG_INFINITY, f32::NAN] {
        let b = [0.0, 0.0, 0.0, bad, 1.0, 1.0];
        assert!(matches!(
            manifest.dirty_from_boxes(&cfg, &grid, &[b]),
            Err(Error::InvalidInput(_))
        ));
    }
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
    assert_eq!(job.manifest_delta().entries.len(), taken.len());
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
    // Entries start after the 68-byte header and take 25 bytes each; swap the first two.
    let mut unsorted = bytes.clone();
    let (a, b) = (68, 93);
    for i in 0..25 {
        unsorted.swap(a + i, b + i);
    }
    assert!(invalid(&unsorted));
    let mut flag = bytes.clone();
    flag[68 + 16] = 2;
    assert!(invalid(&flag));
    let mut outside = bytes.clone();
    outside[68..72].copy_from_slice(&1000i32.to_le_bytes());
    assert!(invalid(&outside));
    // A grid that disagrees with the stored configuration hash is rejected: patch the
    // header's height range (bytes 36..40), tile extent (40..44) or halo (44..48), each
    // leaving the hash alone.
    for at in [36, 40, 44] {
        let mut forged = bytes.clone();
        forged[at..at + 4].copy_from_slice(&1.0f32.to_le_bytes());
        assert!(invalid(&forged), "forged header field at {at}");
    }
    // The grid size is bounded by the work limit, whatever the entry count.
    let mut huge = BakeManifest::new(&cfg, &grid).unwrap();
    huge.grid.tiles_x = 46_340;
    huge.grid.tiles_z = 46_340;
    let tight = Limits {
        max_work: 1_000,
        ..Limits::default()
    };
    assert_eq!(
        BakeManifest::from_bytes(&huge.to_bytes(), tight),
        Err(Error::LimitExceeded)
    );
    assert!(BakeManifest::from_bytes(&huge.to_bytes(), limits).is_ok());
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
            .dirty_from_boxes(&cfg, &grid, boxes)
            .unwrap()
            .keys
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
    let total = tiles.len();
    assert!(non_empty > 1);
    // The residency cap is the navmesh's parameter: here exactly as many slots as tiles.
    let mut nav =
        navmesh_rs::detour::NavMesh::new(&grid.nav_mesh_params(non_empty as i32, 1 << 16)).unwrap();
    let mut installer = bake::TileInstaller::new();
    tiles.into_iter().for_each(|t| installer.push(t));
    // A tile without polygons is queued too: it removes a resident tile, if any.
    assert_eq!(installer.pending(), total);
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
    assert_eq!(calls, total);
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

/// A flat mesh of axis-aligned quads, for tests that need little geometry.
#[derive(Clone, Default)]
struct Quads {
    verts: Vec<f32>,
    tris: Vec<i32>,
}

impl Quads {
    fn quad(mut self, x0: f32, z0: f32, x1: f32, z1: f32, y: f32) -> Self {
        let b = (self.verts.len() / 3) as i32;
        self.verts
            .extend_from_slice(&[x0, y, z0, x0, y, z1, x1, y, z1, x1, y, z0]);
        self.tris
            .extend_from_slice(&[b, b + 1, b + 2, b, b + 2, b + 3]);
        self
    }

    fn input(&self) -> Input<'_> {
        Input::new(&self.verts, &self.tris)
    }
}

/// Bakes the given tiles of `mesh` in `grid`.
fn bake_quads(
    mesh: &Quads,
    cfg: &BakeConfig,
    grid: &GridLayout,
    keys: &[TileKey],
) -> Vec<TileOutput> {
    let input = mesh.input();
    let mut job = BakeJob::new(&input, cfg, grid, keys.to_vec(), JobOptions::default()).unwrap();
    serial(&mut job, u64::MAX)
}

/// The bytes of the tile at `key`, removing it from `nav`.
fn take_tile(nav: &mut navmesh_rs::detour::NavMesh, key: TileKey) -> Option<Vec<u8>> {
    let i = nav.tile_index_at(key.x, key.z, 0)?;
    let r = nav.tile_ref(i);
    Some(nav.remove_tile(r).unwrap())
}

#[test]
fn installer_replaces_resident_tiles_and_the_newest_queued_version_wins() {
    let cfg = BakeConfig::default();
    let grid = GridLayout::new([0.0, -1.0, 0.0], 8.0, 4, 4, &cfg).unwrap();
    let (a, b) = (Quads::default().quad(0.0, 0.0, 38.0, 38.0, 0.0), {
        Quads::default()
            .quad(0.0, 0.0, 38.0, 38.0, 0.0)
            .quad(4.0, 4.0, 6.0, 6.0, 0.6)
    });
    let key = TileKey { x: 0, z: 0 };
    let (old, new) = (
        bake_quads(&a, &cfg, &grid, &[key]).remove(0),
        bake_quads(&b, &cfg, &grid, &[key]).remove(0),
    );
    assert_ne!(old.data, new.data);
    let fresh = || navmesh_rs::detour::NavMesh::new(&grid.nav_mesh_params(16, 1 << 12)).unwrap();
    let all = || Budget::unlimited();

    // A rebaked tile replaces the resident one; refs to the old tile stop validating.
    let (mut nav, mut installer) = (fresh(), bake::TileInstaller::new());
    installer.push(old.clone());
    installer.install(&mut nav, &mut all()).unwrap();
    let old_ref = nav.tile_ref(nav.tile_index_at(0, 0, 0).unwrap());
    installer.push(new.clone());
    assert_eq!(installer.install(&mut nav, &mut all()), Ok(Progress::Done));
    let new_ref = nav.tile_ref(nav.tile_index_at(0, 0, 0).unwrap());
    assert_ne!(old_ref, new_ref);
    assert_eq!(take_tile(&mut nav, key), new.data);

    // Two versions queued: the newest is installed, the older one is dropped unseen.
    let (mut nav, mut installer) = (fresh(), bake::TileInstaller::new());
    installer.push(old.clone());
    installer.push(new.clone());
    assert_eq!(installer.pending(), 1);
    installer.install(&mut nav, &mut all()).unwrap();
    assert_eq!(take_tile(&mut nav, key), new.data);
    let (mut nav, mut installer) = (fresh(), bake::TileInstaller::new());
    installer.push(new.clone());
    installer.push(old.clone());
    installer.install(&mut nav, &mut all()).unwrap();
    assert_eq!(take_tile(&mut nav, key), old.data, "the last push wins");

    // A refused tile leaves the resident one in place.
    let (mut nav, mut installer) = (fresh(), bake::TileInstaller::new());
    installer.push(old.clone());
    installer.install(&mut nav, &mut all()).unwrap();
    let mut corrupt = new.clone();
    corrupt.data = Some(vec![1, 2, 3]);
    installer.push(corrupt);
    let err = installer.install(&mut nav, &mut all()).unwrap_err();
    assert_eq!(err.tile, Some([0, 0]));
    assert_eq!(installer.pending(), 0);
    assert_eq!(take_tile(&mut nav, key), old.data);

    // A tile rebaked to nothing removes the resident tile; clear() reports what it drops.
    let far = TileKey { x: 3, z: 3 };
    let small = Quads::default().quad(0.0, 0.0, 5.0, 5.0, 0.0);
    let (full, empty) = (
        bake_quads(&a, &cfg, &grid, &[far]).remove(0),
        bake_quads(&small, &cfg, &grid, &[far]).remove(0),
    );
    assert!(full.data.is_some() && empty.data.is_none());
    let (mut nav, mut installer) = (fresh(), bake::TileInstaller::new());
    installer.push(full);
    installer.install(&mut nav, &mut all()).unwrap();
    installer.push(empty);
    installer.push(old);
    assert_eq!(installer.clear(), [key, far]);
    assert_eq!(installer.pending(), 0);
    assert!(
        nav.tile_index_at(3, 3, 0).is_some(),
        "clear leaves the navmesh as it is"
    );
    let (mut installer, mut nav) = (bake::TileInstaller::new(), fresh());
    installer.push(bake_quads(&a, &cfg, &grid, &[far]).remove(0));
    installer.install(&mut nav, &mut all()).unwrap();
    installer.push(bake_quads(&small, &cfg, &grid, &[far]).remove(0));
    installer.install(&mut nav, &mut all()).unwrap();
    assert!(nav.tile_index_at(3, 3, 0).is_none());
}

#[test]
fn a_cancelled_rebake_leaves_whole_old_or_new_tiles_and_the_delta_names_the_new_ones() {
    let cfg = small_cfg();
    let before = World::new(120.0, 9);
    let edit = Edit::Raise {
        min: [10.0, 10.0],
        max: [60.0, 60.0],
        dy: 3.0,
    };
    let after = before.clone().with(edit);
    let grid = before.layout(&cfg);
    let (tiles, manifest) = baked(&before, &cfg, &grid);
    let (want, _) = baked(&after, &cfg, &grid);
    let mut nav = navmesh_rs::detour::NavMesh::new(&grid.nav_mesh_params(64, 1 << 16)).unwrap();
    let mut installer = bake::TileInstaller::new();
    for (key, data) in &tiles {
        installer.push(tile_output(&before, &cfg, &grid, *key, data.clone()));
    }
    installer
        .install(&mut nav, &mut Budget::unlimited())
        .unwrap();

    let dirty = manifest
        .dirty_from_boxes(&cfg, &grid, &[edit.bounds()])
        .unwrap()
        .keys;
    let mut job = BakeJob::new(&after, &cfg, &grid, dirty.clone(), JobOptions::default()).unwrap();
    let mut delivered = Vec::new();
    while delivered.len() < 2 {
        job.step(&mut Budget::new(50_000)).unwrap();
        drain(&mut job, &mut delivered);
    }
    job.cancel_token().cancel();
    let delivered_count = delivered.len();
    delivered.into_iter().for_each(|t| installer.push(t));
    installer
        .install(&mut nav, &mut Budget::unlimited())
        .unwrap();
    assert!(job.step(&mut Budget::unlimited()).is_err());
    let new: Vec<TileKey> = job.manifest_delta().entries.iter().map(|e| e.key).collect();
    assert_eq!(new.len(), delivered_count);
    assert!(
        new.len() < dirty.len(),
        "the cancel left some dirty tiles stale"
    );
    for key in grid.keys() {
        let resident = take_tile(&mut nav, key);
        let expected = if new.contains(&key) {
            &want[&key]
        } else {
            &tiles[&key]
        };
        assert_eq!(&resident, expected, "tile {key:?}");
    }
}

/// A finished-tile value for `key` carrying `data`, as a job would return it.
fn tile_output(
    world: &World,
    cfg: &BakeConfig,
    grid: &GridLayout,
    key: TileKey,
    data: Option<Vec<u8>>,
) -> TileOutput {
    let mut job = BakeJob::new(world, cfg, grid, vec![key], JobOptions::default()).unwrap();
    let mut tile = serial(&mut job, u64::MAX).remove(0);
    tile.data = data;
    tile
}

#[test]
fn a_manifest_with_a_foreign_layout_is_not_used_to_skip_tiles() {
    let cfg = small_cfg();
    let world = World::new(60.0, 4);
    let grid = world.layout(&cfg);
    let (_, mut manifest) = baked(&world, &cfg, &grid);
    let mut job = BakeJob::new(&world, &cfg, &grid, grid.keys(), JobOptions::default()).unwrap();
    assert!(
        job.skip_unchanged(&manifest, &mut Workspace::new())
            .unwrap()
    );
    assert_eq!(job.skipped(), grid.keys().len());
    // The stored hash still matches the configuration, but the stored layout does not.
    manifest.grid.halo = 100.0;
    manifest.grid.tile_extent = 1.0;
    let mut job = BakeJob::new(&world, &cfg, &grid, grid.keys(), JobOptions::default()).unwrap();
    assert!(
        !job.skip_unchanged(&manifest, &mut Workspace::new())
            .unwrap()
    );
    assert_eq!(job.skipped(), 0);
}

#[test]
fn byte_limits_mean_the_same_for_a_job_and_tiles() {
    let mut mesh = Quads::default();
    for i in 0..30 {
        for j in 0..30 {
            let (x, z) = (i as f32 * 0.3, j as f32 * 0.3);
            mesh = mesh.quad(x, z, x + 0.3, z + 0.3, 0.0);
        }
    }
    let with_bytes = |bytes: usize| {
        let mut cfg = BakeConfig::default();
        cfg.tile_limits.max_bytes = bytes;
        cfg
    };
    let tiles_ok = |bytes: usize| {
        bake::tiles(&mesh.input(), &with_bytes(bytes), &mut Workspace::new()).is_ok()
    };
    let job_ok = |bytes: usize| {
        let (input, cfg) = (mesh.input(), with_bytes(bytes));
        let grid = GridLayout::from_input(&input, &cfg).unwrap();
        let mut job =
            BakeJob::new(&input, &cfg, &grid, grid.keys(), JobOptions::default()).unwrap();
        loop {
            match job.step(&mut Budget::unlimited()) {
                Ok(Progress::Done) => return true,
                Ok(_) => {}
                Err(_) => return false,
            }
        }
    };
    let (mut lo, mut hi) = (1usize, 1usize << 30);
    while lo < hi {
        let mid = lo + (hi - lo) / 2;
        if tiles_ok(mid) {
            hi = mid;
        } else {
            lo = mid + 1;
        }
    }
    assert!(job_ok(lo), "the job needs no more than tiles at {lo} bytes");
    assert!(!job_ok(lo - 1), "and no less");

    // The aggregate limit applies to a job as it does to `tiles`.
    let mut cfg = with_bytes(1 << 30);
    cfg.tile_limits.max_tile_bytes = 1 << 20;
    cfg.max_total_bytes = 1 << 12;
    let input = mesh.input();
    assert_eq!(
        bake::tiles(&input, &cfg, &mut Workspace::new())
            .err()
            .map(|e| e.error),
        Some(Error::LimitExceeded)
    );
    let grid = GridLayout::from_input(&input, &cfg).unwrap();
    let mut job = BakeJob::new(&input, &cfg, &grid, grid.keys(), JobOptions::default()).unwrap();
    assert_eq!(
        job.step(&mut Budget::unlimited()).unwrap_err().error,
        Error::LimitExceeded
    );
    cfg.max_total_bytes = 1 << 40;
    let mut job = BakeJob::new(&input, &cfg, &grid, grid.keys(), JobOptions::default()).unwrap();
    assert_eq!(job.step(&mut Budget::unlimited()), Ok(Progress::Done));
}

#[test]
fn an_indexed_input_gives_the_tiles_and_hashes_of_the_plain_input() {
    let scene = Scene::new();
    let volumes = scene.volumes();
    let areas = [63u8, 5];
    for areas in [None, Some(&areas[..])] {
        let input = scene.input(&volumes, areas);
        let cfg = small_cfg();
        let grid = GridLayout::from_input(&input, &cfg).unwrap();
        let indexed = input.indexed(&grid, &mut Workspace::new()).unwrap();
        let opts = JobOptions::default;
        let mut job = BakeJob::new(&input, &cfg, &grid, grid.keys(), opts()).unwrap();
        let plain = serial(&mut job, u64::MAX);
        let mut job = BakeJob::new(&indexed, &cfg, &grid, grid.keys(), opts()).unwrap();
        let fast = serial(&mut job, u64::MAX);
        assert_eq!(plain, fast);
        assert!(plain.iter().any(|t| t.data.is_some()) && grid.keys().len() > 1);
    }
    // The index is charged to the workspace, and a bad mesh fails when it is built.
    let input = scene.input(&volumes, None);
    let cfg = small_cfg();
    let grid = GridLayout::from_input(&input, &cfg).unwrap();
    let mut tight = Workspace::with_limits(Limits {
        max_bytes: 8,
        ..Limits::default()
    });
    assert_eq!(
        input.indexed(&grid, &mut tight).err(),
        Some(Error::LimitExceeded)
    );
    let bad = [0, 1, 99];
    let broken = Input::new(&scene.verts, &bad);
    assert!(matches!(
        broken.indexed(&grid, &mut Workspace::new()),
        Err(Error::InvalidInput(_))
    ));
}

#[test]
fn fixes_flow_through_jobs_and_invalidate_manifests() {
    use navmesh_rs::UpstreamFixes;

    let scene = Scene::new();
    let input = Input::new(&scene.verts, &scene.tris);
    let mut cfg = BakeConfig::default();
    cfg.recast.detail_sample_dist = 0.0;
    let grid = GridLayout::from_input(&input, &cfg).unwrap();
    let mut previous: Option<BakeManifest> = None;
    let mut baseline = None;
    for fixes in [
        UpstreamFixes::NONE,
        UpstreamFixes::ZERO_SAMPLE_DIST,
        UpstreamFixes::ALL,
    ] {
        cfg.fixes = fixes;
        let expected = bake::tiles(&input, &cfg, &mut Workspace::new()).unwrap();
        let expected: Vec<_> = expected.tiles.into_iter().map(|t| t.data).collect();
        assert!(expected.iter().any(Option::is_some));
        if let Some(ref baseline) = baseline {
            assert_ne!(&expected, baseline);
        } else {
            baseline = Some(expected.clone());
        }
        for allowance in [1, u64::MAX] {
            let mut job =
                BakeJob::new(&input, &cfg, &grid, grid.keys(), JobOptions::default()).unwrap();
            let got: Vec<_> = serial(&mut job, allowance)
                .into_iter()
                .map(|t| t.data)
                .collect();
            assert_eq!(got, expected);
        }
        let mut job =
            BakeJob::new(&input, &cfg, &grid, grid.keys(), JobOptions::default()).unwrap();
        job.run_parallel().unwrap();
        let mut got = Vec::new();
        drain(&mut job, &mut got);
        assert_eq!(
            got.into_iter().map(|t| t.data).collect::<Vec<_>>(),
            expected
        );
        if let Some(ref previous) = previous {
            let dirty = previous.dirty_from_boxes(&cfg, &grid, &[]).unwrap();
            assert!(dirty.full_rebake);
            assert_eq!(dirty.keys, grid.keys());
            let mut job =
                BakeJob::new(&input, &cfg, &grid, grid.keys(), JobOptions::default()).unwrap();
            assert!(!job.skip_unchanged(previous, &mut Workspace::new()).unwrap());
            assert_eq!(job.skipped(), 0);
        }
        let mut manifest = BakeManifest::new(&cfg, &grid).unwrap();
        manifest.apply(&job.manifest_delta()).unwrap();
        previous = Some(manifest);
    }
}
