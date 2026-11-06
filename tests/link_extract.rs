#![cfg(all(feature = "recast", feature = "detour"))]
use navmesh_rs::bake::{
    self, BakeConfig, BakeJob, BakeManifest, GridLayout, Input, JobOptions, LinkExtraction,
    LinkKind, LinkLimits,
};
use navmesh_rs::{Budget, Error, Workspace};
fn config() -> BakeConfig {
    let mut c = BakeConfig::default();
    c.recast.cs = 0.5;
    c.recast.ch = 0.1;
    c.recast.tile_size = 32;
    c.recast.walkable_height = 10;
    c.recast.walkable_radius = 1;
    c.recast.walkable_climb = 2;
    c.recast.min_region_area = 0;
    c.recast.merge_region_area = 0;
    c.recast.detail_sample_dist = 0.0;
    c.walkable_height = 1.0;
    c.walkable_radius = 0.5;
    c.walkable_climb = 0.2;
    c
}
fn options() -> LinkExtraction {
    let drop = LinkLimits {
        distance: 6.0,
        up: 0.0,
        down: 4.0,
        clearance: 0.0,
    };
    LinkExtraction {
        drop: Some(drop),
        jump: None,
        climb: Some(LinkLimits {
            up: 4.0,
            down: 0.0,
            ..drop
        }),
        spacing_cells: 2,
        area_ids: [5, 6, 7],
        flags: [8, 16, 32],
    }
}
fn scene() -> (Vec<f32>, Vec<i32>) {
    // Raised departure slab, a gap, and a lower landing slab.
    (
        vec![
            0., 3., 0., 0., 3., 12., 8., 3., 12., 8., 3., 0., 10., 0., 0., 10., 0., 12., 24., 0.,
            12., 24., 0., 0.,
        ],
        vec![0, 1, 2, 0, 2, 3, 4, 5, 6, 4, 6, 7],
    )
}
#[test]
fn extraction_installs_links_and_all_drivers_agree() -> Result<(), Box<dyn std::error::Error>> {
    let (v, t) = scene();
    let input = Input::new(&v, &t);
    let mut c = config();
    let plain = bake::tiles(&input, &c, &mut Workspace::new())?;
    assert!(plain.tiles.iter().all(|t| t.links.is_empty()));
    c.links = Some(options());
    let linked = bake::tiles(&input, &c, &mut Workspace::new())?;
    let links: Vec<_> = linked.tiles.iter().flat_map(|t| t.links.iter()).collect();
    assert!(!links.is_empty(), "expected links between slabs");
    assert!(links.iter().any(|l| l.kind == LinkKind::Drop));
    for l in &links {
        assert!((l.start[1] - l.end[1]).abs() > c.walkable_climb);
        let dx = l.start[0] - l.end[0];
        let dz = l.start[2] - l.end[2];
        assert!(dx * dx + dz * dz <= 36.01);
        match l.kind {
            LinkKind::Drop => {
                assert!(l.start[1] > l.end[1]);
                assert_eq!((l.area, l.flags), (5, 8));
            }
            LinkKind::Climb => {
                assert!(l.end[1] > l.start[1]);
                assert_eq!((l.area, l.flags), (7, 32));
            }
            LinkKind::Jump => panic!("disabled kind"),
        }
    }
    let mut digest = 0xcbf2_9ce4_8422_2325u64;
    for bytes in linked.tiles.iter().filter_map(|t| t.data.as_ref()) {
        for &byte in bytes {
            digest = (digest ^ byte as u64).wrapping_mul(0x100_0000_01b3);
        }
    }
    println!("extraction_digest={digest:016x}");
    let nav = linked.into_navmesh(Default::default())?;
    assert!(
        (0..nav.max_tiles() as usize)
            .filter_map(|i| nav.tile(i))
            .any(|tile| !tile.off_mesh_cons.is_empty())
    );
    let linked = bake::tiles(&input, &c, &mut Workspace::new())?;
    let grid = GridLayout::from_input(&input, &c)?;
    let plain_grid = GridLayout::from_input(&input, &config())?;
    assert!(grid.halo_cells > plain_grid.halo_cells);
    assert_ne!(
        BakeManifest::config_hash(&c, &grid),
        BakeManifest::config_hash(&config(), &plain_grid)
    );
    let keys = grid.keys();
    let mut job = BakeJob::new(&input, &c, &grid, keys, JobOptions::default())?;
    while !job.step(&mut Budget::new(1))?.is_done() {}
    let mut i = 0;
    while let Some(out) = job.next_ready() {
        assert_eq!(out.data, linked.tiles[i].data);
        assert_eq!(out.links, linked.tiles[i].links);
        i += 1;
    }
    assert_eq!(i, linked.tiles.len());
    #[cfg(feature = "parallel")]
    for threads in [1, 4, 16] {
        let pool = rayon::ThreadPoolBuilder::new()
            .num_threads(threads)
            .build()?;
        let parallel = pool.install(|| bake::tiles_parallel(&input, &c, &mut Workspace::new()))?;
        assert_eq!(parallel.tiles, linked.tiles);
    }
    Ok(())
}
#[test]
fn malformed_settings_are_rejected() {
    let (v, t) = scene();
    let input = Input::new(&v, &t);
    let mut c = config();
    let mut opts = options();
    opts.spacing_cells = 0;
    c.links = Some(opts);
    assert!(matches!(
        bake::tiles(&input, &c, &mut Workspace::new()),
        Err(bake::BakeError {
            error: Error::InvalidParameter(_),
            ..
        })
    ));
}

#[test]
fn dirty_rebake_with_extraction_equals_full_job() -> Result<(), Box<dyn std::error::Error>> {
    use std::collections::BTreeMap;
    let (mut v, t) = scene();
    let mut c = config();
    c.links = Some(options());
    let before = Input::new(&v, &t);
    let grid = GridLayout::from_input(&before, &c)?;
    let mut job = BakeJob::new(&before, &c, &grid, grid.keys(), JobOptions::default())?;
    job.run_parallel()?;
    let mut retained = BTreeMap::new();
    while let Some(out) = job.next_ready() {
        retained.insert(out.key, (out.data, out.links));
    }
    let mut manifest = BakeManifest::new(&c, &grid)?;
    manifest.apply(&job.manifest_delta())?;
    for i in [1, 4, 7, 10] {
        v[i] = 2.5;
    }
    let after = Input::new(&v, &t);
    let dirty = manifest.dirty_from_boxes(&c, &grid, &[[-0.5, -1., -0.5, 9., 4., 13.]])?;
    let mut incremental = BakeJob::new(&after, &c, &grid, dirty.keys, JobOptions::default())?;
    incremental.skip_unchanged(&manifest, &mut Workspace::new())?;
    incremental.run_parallel()?;
    while let Some(out) = incremental.next_ready() {
        retained.insert(out.key, (out.data, out.links));
    }
    let mut full = BakeJob::new(&after, &c, &grid, grid.keys(), JobOptions::default())?;
    full.run_parallel()?;
    let mut expected = BTreeMap::new();
    while let Some(out) = full.next_ready() {
        expected.insert(out.key, (out.data, out.links));
    }
    assert_eq!(retained, expected);
    Ok(())
}

#[test]
#[ignore = "explicit interleaved cost measurement"]
fn extraction_costs() -> Result<(), Box<dyn std::error::Error>> {
    use std::time::Instant;
    let (v, t) = scene();
    let input = Input::new(&v, &t);
    let plain = config();
    let mut linked = plain;
    linked.links = Some(options());
    let mut ns = [0u128; 2];
    let mut bytes = [0usize; 2];
    let mut counts = [0usize; 2];
    for warmup in [true, false] {
        for repeat in 0..16 {
            for arm in if repeat % 2 == 0 { [0, 1] } else { [1, 0] } {
                let c = if arm == 0 { &plain } else { &linked };
                let start = Instant::now();
                let output = bake::tiles(&input, c, &mut Workspace::new())?;
                let elapsed = start.elapsed().as_nanos();
                if !warmup {
                    ns[arm] += elapsed;
                }
                bytes[arm] = output
                    .tiles
                    .iter()
                    .filter_map(|t| t.data.as_ref())
                    .map(Vec::len)
                    .sum();
                counts[arm] = output.tiles.iter().map(|t| t.links.len()).sum();
            }
        }
    }
    println!(
        "fixture=two_slabs repeats=16 default_ns={} extraction_ns={} default_dnav_bytes={} extraction_dnav_bytes={} links={}",
        ns[0], ns[1], bytes[0], bytes[1], counts[1]
    );
    assert!(counts[1] > 0);
    Ok(())
}
