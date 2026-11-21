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

#[test]
fn bake_reserve_changes_tile_bytes_and_manifest_identity() -> Result<(), Box<dyn std::error::Error>>
{
    use navmesh_rs::detour::MeshTile;

    let (v, t) = scene();
    let input = Input::new(&v, &t);
    let mut c = config();
    let grid = GridLayout::from_input(&input, &c)?;
    let before = BakeManifest::config_hash(&c, &grid);
    let plain = bake::tiles(&input, &c, &mut Workspace::new())?;
    c.spare_links_per_tile = 4;
    let reserved = bake::tiles(&input, &c, &mut Workspace::new())?;
    assert_ne!(BakeManifest::config_hash(&c, &grid), before);
    assert!(plain.tiles.iter().any(|t| t.data.is_some()));
    assert_eq!(plain.tiles.len(), reserved.tiles.len());
    for (a, b) in plain.tiles.into_iter().zip(reserved.tiles) {
        match (a.data, b.data) {
            (Some(a), Some(b)) => {
                let a = MeshTile::parse(a)?;
                let b = MeshTile::parse(b)?;
                assert_eq!(a.polys, b.polys);
                assert_eq!(b.links.len(), a.links.len() + 4);
            }
            (None, None) => {}
            _ => panic!("adding spare slots changed ground tile coverage"),
        }
    }
    Ok(())
}

#[derive(Default)]
struct Floors {
    verts: Vec<f32>,
    tris: Vec<i32>,
}

impl Floors {
    fn quad(&mut self, x0: f32, x1: f32, y: f32, is_floor: bool) {
        let i = (self.verts.len() / 3) as i32;
        self.verts
            .extend([x0, y, 0., x0, y, 12., x1, y, 12., x1, y, 0.]);
        if is_floor {
            self.tris.extend([i, i + 1, i + 2, i, i + 2, i + 3]);
        } else {
            self.tris.extend([i, i + 2, i + 1, i, i + 3, i + 2]);
        }
    }

    fn input(&self) -> Input<'_> {
        Input::new(&self.verts, &self.tris)
    }
}

use navmesh_rs::bake::{AreaVolume, BakedTiles, GapLinks, LadderLinks, StepLinks};
use navmesh_rs::detour::{DefaultFilter, NavMesh, NavMeshQuery, OffMeshKind, PolyRef};

fn find_ground(nav: &NavMesh, pos: [f32; 3]) -> PolyRef {
    let (ground, _, _) = NavMeshQuery::new(nav, 128)
        .unwrap()
        .find_nearest_poly(&pos, &[0.6, 0.6, 0.6], &DefaultFilter::default())
        .unwrap();
    assert_ne!(ground, 0, "no ground at {pos:?}");
    ground
}

fn path(nav: &NavMesh, a: [f32; 3], b: [f32; 3]) -> Vec<PolyRef> {
    let mut out = Vec::new();
    NavMeshQuery::new(nav, 128)
        .unwrap()
        .find_path(
            find_ground(nav, a),
            find_ground(nav, b),
            &a,
            &b,
            &DefaultFilter::default(),
            &mut out,
            32,
        )
        .unwrap();
    out
}

fn traversals(nav: &NavMesh, path: &[PolyRef]) -> Vec<OffMeshKind> {
    path.iter()
        .filter_map(|&r| nav.off_mesh_connection_by_ref(r)?.kind())
        .collect()
}

fn has_kind(bake: &BakedTiles, kind: OffMeshKind) -> bool {
    bake.tiles
        .iter()
        .flat_map(|t| &t.extended_links)
        .any(|l| l.kind == kind)
}

fn ladder_scene(blocked: bool) -> Floors {
    let mut scene = Floors::default();
    scene.quad(0., 14., 0., true);
    scene.quad(18., 30., 2., true);
    scene.quad(0., 14., 4., true);
    if blocked {
        scene.quad(15., 17., 1.5, false);
        scene.quad(15., 17., 3.5, false);
    }
    scene
}

fn ladder_marker() -> AreaVolume<'static> {
    AreaVolume::ladder(
        &[10., 0., 2., 10., 0., 10., 22., 0., 10., 22., 0., 2.],
        0.,
        4.3,
    )
}

fn ladder_config() -> BakeConfig {
    let mut c = config();
    c.extended_links.ladders = Some(LadderLinks::new(6., 8, 1));
    c
}

#[test]
fn ladder_connects_three_floors_in_both_directions_and_reports_kind() {
    let scene = ladder_scene(false);
    let volumes = [ladder_marker()];
    let mut input = scene.input();
    input.volumes = &volumes;
    let c = ladder_config();
    let baked = bake::tiles(&input, &c, &mut Workspace::new()).unwrap();
    let links: Vec<_> = baked.tiles.iter().flat_map(|t| &t.extended_links).collect();
    assert_eq!(links.len(), 2, "{links:?}");
    assert!(
        links
            .iter()
            .all(|l| l.kind == OffMeshKind::Ladder && l.area == 8)
    );
    let bottom = links[0].start;
    let top = links[1].end;
    assert!(links[0].start[0] < 16. && links[0].end[0] > 16.);
    assert!(links[1].start[0] > 16. && links[1].end[0] < 16.);
    assert_eq!(links[0].end, links[1].start);
    let nav = baked.into_navmesh(Default::default()).unwrap();
    for (a, b) in [(bottom, top), (top, bottom)] {
        let route = path(&nav, a, b);
        assert_eq!(route.last(), Some(&find_ground(&nav, b)), "{route:?}");
        assert_eq!(traversals(&nav, &route), [OffMeshKind::Ladder; 2]);
    }
    #[cfg(feature = "crowd")]
    {
        use navmesh_rs::crowd::{AgentParams, Crowd, OffMeshMode};
        let mut ws = Workspace::new();
        let mut params = AgentParams::default();
        params.off_mesh = OffMeshMode::External;
        let mut crowd = Crowd::new(&mut ws, &nav, 1, 0.6).unwrap();
        let walk_up = [bottom[0] - 2., bottom[1], bottom[2]];
        let id = crowd.add_agent(&mut ws, &nav, walk_up, params).unwrap();
        crowd
            .request_move_target(&nav, id, find_ground(&nav, top), top)
            .unwrap();
        for _ in 0..120 {
            crowd.update(&mut ws, &nav, 0.05).unwrap();
            if crowd.off_mesh_arrivals().next().is_some() {
                break;
            }
        }
        let (waiting, arrival) = crowd.off_mesh_arrivals().next().expect("ladder arrival");
        assert_eq!(waiting, id);
        assert_eq!(arrival.kind, Some(OffMeshKind::Ladder));
        crowd.complete_off_mesh(&nav, id, arrival.end).unwrap();
        assert_eq!(crowd.agent(id).unwrap().npos, arrival.end);
    }
    #[cfg(feature = "crowd-world")]
    {
        use navmesh_rs::crowd::{
            AgentParams, OffMeshMode,
            world::{CrowdWorld, CrowdWorldParams, Goal},
        };
        let mut crowd =
            CrowdWorld::new(&nav, &CrowdWorldParams::default(), Default::default()).unwrap();
        let mut params = AgentParams::default();
        params.off_mesh = OffMeshMode::External;
        let walk_up = [bottom[0] - 2., bottom[1], bottom[2]];
        let id = crowd.add_agent(&nav, walk_up, params).unwrap();
        crowd
            .set_goal(
                &nav,
                id,
                Goal::Point {
                    poly: find_ground(&nav, top),
                    pos: top,
                },
            )
            .unwrap();
        for _ in 0..240 {
            crowd.step(&nav, &mut Budget::unlimited()).unwrap();
            if crowd.off_mesh_arrivals().next().is_some() {
                break;
            }
        }
        let (waiting, arrival) = crowd.off_mesh_arrivals().next().expect("ladder arrival");
        assert_eq!(waiting, id);
        assert_eq!(arrival.kind, Some(OffMeshKind::Ladder));
        let mut restored =
            CrowdWorld::from_bytes(&nav, &crowd.to_bytes(), Default::default()).unwrap();
        assert_eq!(
            restored.off_mesh_arrivals().next(),
            Some((waiting, arrival))
        );
        restored.complete_off_mesh(&nav, id, arrival.end).unwrap();
    }
}

#[test]
fn ladder_rejects_a_blocked_shaft() {
    let scene = ladder_scene(true);
    let volumes = [ladder_marker()];
    let mut input = scene.input();
    input.volumes = &volumes;
    let baked = bake::tiles(&input, &ladder_config(), &mut Workspace::new()).unwrap();
    assert!(!has_kind(&baked, OffMeshKind::Ladder));
}

fn stair_scene() -> Floors {
    let mut scene = Floors::default();
    scene.quad(0., 8., 0., true);
    scene.quad(8., 16., 0.2, true);
    scene.quad(16., 28., 1.2, true);
    scene
}

fn step_config(height: f32) -> BakeConfig {
    let mut c = config();
    c.recast.walkable_climb = 3;
    c.walkable_climb = 0.3;
    c.extended_links.steps = Some(StepLinks::new(4.5, height, 9, 1));
    c
}

#[test]
fn step_crosses_a_split_stair_top_without_replacing_the_walkable_step() {
    let scene = stair_scene();
    let c = step_config(1.2);
    let baked = bake::tiles(&scene.input(), &c, &mut Workspace::new()).unwrap();
    assert!(has_kind(&baked, OffMeshKind::Step));
    let links: Vec<_> = baked.tiles.iter().flat_map(|t| &t.extended_links).collect();
    assert!(links.iter().all(|l| l.start[0].max(l.end[0]) > 16.));
    let nav = baked.into_navmesh(Default::default()).unwrap();
    let a = [4., 0.1, 6.];
    let b = [22., 1.3, 6.];
    let route = path(&nav, a, b);
    assert_eq!(route.last(), Some(&find_ground(&nav, b)), "{route:?}");
    assert_eq!(traversals(&nav, &route), [OffMeshKind::Step]);
}

#[test]
fn step_rejects_a_rise_above_the_limit() {
    let scene = stair_scene();
    let baked = bake::tiles(&scene.input(), &step_config(0.6), &mut Workspace::new()).unwrap();
    assert!(!has_kind(&baked, OffMeshKind::Step));
}

fn gap_scene() -> Floors {
    let mut scene = Floors::default();
    scene.quad(0., 8., 0., true);
    scene.quad(16., 24., 0., true);
    scene
}

fn gap_config(distance: f32) -> BakeConfig {
    let mut c = config();
    c.recast.tile_size = 16;
    c.extended_links.level_gaps = Some(GapLinks::new(distance, 0.4, 10, 1));
    c
}

#[test]
fn level_gap_crosses_an_empty_column_and_reconnects_on_reload() {
    let scene = gap_scene();
    let input = scene.input();
    let c = gap_config(12.);
    let baked = bake::tiles(&input, &c, &mut Workspace::new()).unwrap();
    assert!(has_kind(&baked, OffMeshKind::LevelGap));
    let grid = GridLayout::from_input(&input, &c).unwrap();
    let mut job = BakeJob::new(&input, &c, &grid, grid.keys(), JobOptions::default()).unwrap();
    while !job.step(&mut Budget::new(1)).unwrap().is_done() {}
    for tile in &baked.tiles {
        let ready = job.next_ready().unwrap();
        assert_eq!(ready.data, tile.data);
        assert_eq!(ready.extended_links, tile.extended_links);
    }
    #[cfg(feature = "parallel")]
    for threads in [1, 4, 16] {
        let pool = rayon::ThreadPoolBuilder::new()
            .num_threads(threads)
            .build()
            .unwrap();
        let parallel = pool
            .install(|| bake::tiles_parallel(&input, &c, &mut Workspace::new()))
            .unwrap();
        assert_eq!(parallel.tiles, baked.tiles);
    }
    let mut nav = baked.into_navmesh(Default::default()).unwrap();
    let a = [4., 0.1, 3.];
    let b = [20., 0.1, 3.];
    let b_ref = find_ground(&nav, b);
    let route = path(&nav, a, b);
    assert_eq!(route.last(), Some(&b_ref));
    assert_eq!(traversals(&nav, &route), [OffMeshKind::LevelGap]);
    let (x, z) = nav.calc_tile_loc(&b);
    let slot = nav.tile_index_at(x, z, 0).unwrap();
    let data = nav.remove_tile(nav.tile_ref(slot)).unwrap();
    assert!(nav.off_mesh_connection_by_ref(route[1]).is_some());
    nav.set_global_off_mesh_links(true);
    nav.add_tile(data).unwrap();
    let route = path(&nav, a, b);
    assert_eq!(route.last(), Some(&find_ground(&nav, b)));
    assert_eq!(traversals(&nav, &route), [OffMeshKind::LevelGap]);
    nav.set_global_off_mesh_links(false);
    assert_eq!(path(&nav, a, b), route);
}

#[test]
fn bake_profiles_use_their_own_link_limits_and_agent_size() {
    use navmesh_rs::bake::profiles::{ProfileBakeSpec, ProfileId, ProfileRevision};

    let scene = gap_scene();
    let input = scene.input();
    let small = gap_config(12.);
    let mut large = gap_config(9.);
    large.recast.walkable_radius = 3;
    large.walkable_radius = 1.5;
    for (id, c, can_jump) in [(1, small, true), (2, large, false)] {
        let grid = GridLayout::from_input(&input, &c).unwrap();
        let profile = ProfileBakeSpec::new(ProfileId::new(id), c, grid).unwrap();
        let plain = bake::tiles(&input, &c, &mut Workspace::new()).unwrap();
        assert_eq!(has_kind(&plain, OffMeshKind::LevelGap), can_jump);
        let mut job = profile
            .bake(
                &input,
                ProfileRevision::new([0; 16], 1),
                grid.keys(),
                JobOptions::default(),
            )
            .unwrap();
        while !job.step(&mut Budget::unlimited()).unwrap().is_done() {}
        for tile in plain.tiles {
            let ready = job.next_ready().unwrap();
            assert_eq!(ready.tile().data, tile.data);
            assert_eq!(ready.tile().extended_links, tile.extended_links);
        }
        assert!(job.next_ready().is_none());
    }
}

#[test]
fn level_gap_rejects_a_jump_above_the_distance_limit() {
    let scene = gap_scene();
    let baked = bake::tiles(&scene.input(), &gap_config(7.), &mut Workspace::new()).unwrap();
    assert!(!has_kind(&baked, OffMeshKind::LevelGap));
}

#[test]
fn enabling_ladders_preserves_null_and_walkable_area_paint() {
    let mut scene = Floors::default();
    scene.quad(0., 16., 0., true);
    let mut hole = AreaVolume::default();
    hole.verts = &[2., 0., 2., 2., 0., 10., 6., 0., 10., 6., 0., 2.];
    hole.hmax = 1.;
    let mut walkable = hole;
    walkable.verts = &[10., 0., 2., 10., 0., 10., 14., 0., 10., 14., 0., 2.];
    walkable.area = 63;
    let volumes = [hole, walkable];
    let mut input = scene.input();
    input.areas = Some(&[7, 7]);
    input.volumes = &volumes;
    let baked = bake::tiles(&input, &ladder_config(), &mut Workspace::new()).unwrap();
    assert!(!has_kind(&baked, OffMeshKind::Ladder));
    let nav = baked.into_navmesh(Default::default()).unwrap();
    let (at_hole, _, _) = NavMeshQuery::new(&nav, 32)
        .unwrap()
        .find_nearest_poly(&[4., 0.1, 6.], &[0.2; 3], &DefaultFilter::default())
        .unwrap();
    assert_eq!(at_hole, 0);
    assert_eq!(nav.poly_area(find_ground(&nav, [12., 0.1, 6.])).unwrap(), 0);
    assert_eq!(nav.poly_area(find_ground(&nav, [8., 0.1, 6.])).unwrap(), 7);
}

#[test]
fn ladder_volume_never_paints_in_a_profile_without_ladders() {
    use navmesh_rs::bake::profiles::{ProfileBakeSpec, ProfileId, ProfileRevision};

    let scene = ladder_scene(false);
    let bare = scene.input();
    let volumes = [ladder_marker()];
    let mut with_shaft = bare;
    with_shaft.volumes = &volumes;
    let bake_profile = |id, cfg, input: &Input<'_>| {
        let grid = GridLayout::from_input(input, &cfg).unwrap();
        let profile = ProfileBakeSpec::new(ProfileId::new(id), cfg, grid).unwrap();
        let index = input.indexed(&grid, &mut Workspace::new()).unwrap();
        let mut job = profile
            .bake(
                &index,
                ProfileRevision::new([0; 16], 1),
                grid.keys(),
                JobOptions::default(),
            )
            .unwrap();
        while !job.step(&mut Budget::unlimited()).unwrap().is_done() {}
        std::iter::from_fn(|| job.next_ready().map(|item| item.tile().clone())).collect::<Vec<_>>()
    };

    let enabled = bake_profile(1, ladder_config(), &with_shaft);
    assert_eq!(
        enabled
            .iter()
            .map(|tile| tile.extended_links.len())
            .sum::<usize>(),
        2
    );
    assert_eq!(
        bake_profile(2, config(), &with_shaft),
        bake_profile(2, config(), &bare),
    );
}

#[test]
fn ladder_footprint_and_ordinary_area_can_overlap() {
    let scene = ladder_scene(false);
    let shaft = ladder_marker();
    let mut paint = AreaVolume::default();
    paint.verts = shaft.verts;
    paint.hmin = shaft.hmin;
    paint.hmax = shaft.hmax;
    paint.area = 42;
    let volumes = [shaft, paint];
    let mut input = scene.input();
    input.volumes = &volumes;
    let baked = bake::tiles(&input, &ladder_config(), &mut Workspace::new()).unwrap();
    assert!(has_kind(&baked, OffMeshKind::Ladder));
    let nav = baked.into_navmesh(Default::default()).unwrap();
    assert_eq!(
        nav.poly_area(find_ground(&nav, [12., 0.1, 6.])).unwrap(),
        42
    );
    assert_eq!(nav.poly_area(find_ground(&nav, [6., 0.1, 6.])).unwrap(), 0);
}

#[test]
fn level_gap_rejects_a_wall_below_jump_clearance() {
    let mut scene = gap_scene();
    let c = gap_config(12.);
    assert!(has_kind(
        &bake::tiles(&scene.input(), &c, &mut Workspace::new()).unwrap(),
        OffMeshKind::LevelGap
    ));
    scene.quad(11., 12., 0.3, false);
    // The wall leaves unsupported cells on either side and fits under the
    // raised sweep. The centerline floor trace must still reject it.
    let blocked = bake::tiles(&scene.input(), &c, &mut Workspace::new()).unwrap();
    assert!(!has_kind(&blocked, OffMeshKind::LevelGap));
}

#[test]
fn legacy_climb_wins_over_a_step_in_the_same_direction() {
    let scene = stair_scene();
    let a = [5., 0.1, 6.];
    let b = [22., 1.3, 6.];
    for tile_size in [64, 32] {
        let mut c = step_config(1.2);
        c.recast.tile_size = tile_size;
        let mut legacy = options();
        legacy.drop = None;
        c.links = Some(legacy);
        let baked = bake::tiles(&scene.input(), &c, &mut Workspace::new()).unwrap();
        assert!(
            baked
                .tiles
                .iter()
                .flat_map(|t| &t.links)
                .any(|l| l.kind == LinkKind::Climb)
        );
        if tile_size == 64 {
            assert!(
                baked
                    .tiles
                    .iter()
                    .flat_map(|t| &t.extended_links)
                    .all(|l| l.start[1] > l.end[1])
            );
        }
        let mut nav = baked.into_navmesh(Default::default()).unwrap();
        for global in [false, true, false] {
            nav.set_global_off_mesh_links(global);
            for tile in (0..nav.max_tiles() as usize).filter_map(|i| nav.tile(i)) {
                for con in &tile.off_mesh_cons {
                    if con.kind() != Some(OffMeshKind::Step) || con.pos[1] >= con.pos[4] {
                        continue;
                    }
                    let mut link = tile.polys[usize::from(con.poly)].first_link;
                    while let Some(entry) = tile.links.get(link as usize) {
                        assert_ne!(
                            entry.edge, 1,
                            "duplicated upward traversal across a tile seam"
                        );
                        link = entry.next;
                    }
                }
            }
            let up = path(&nav, a, b);
            assert_eq!(up.last(), Some(&find_ground(&nav, b)));
            assert!(
                traversals(&nav, &up).is_empty(),
                "upward route must use legacy climb"
            );
            let down = path(&nav, b, a);
            assert_eq!(down.last(), Some(&find_ground(&nav, a)));
            assert_eq!(traversals(&nav, &down), [OffMeshKind::Step]);
        }
    }
}

#[test]
fn border_ladder_has_one_owner_per_segment_and_matching_incoming_slots() {
    let scene = ladder_scene(false);
    let mut c = ladder_config();
    c.recast.cs = 0.4;
    c.recast.tile_size = 40;
    c.walkable_radius = 0.4;
    let mut input = scene.input();
    let bare = bake::tiles(&input, &c, &mut Workspace::new()).unwrap();
    let volumes = [ladder_marker(), ladder_marker()];
    input.volumes = &volumes;
    let baked = bake::tiles(&input, &c, &mut Workspace::new()).unwrap();
    let links: Vec<_> = baked.tiles.iter().flat_map(|t| &t.extended_links).collect();
    assert_eq!(links.len(), 2);
    assert_eq!(
        links[0].end.map(f32::to_bits),
        links[1].start.map(f32::to_bits)
    );
    assert!(links.iter().all(|l| (l.start[0] < 16.) != (l.end[0] < 16.)));
    for (tile, bare) in baked.tiles.iter().zip(&bare.tiles) {
        assert_eq!(tile.extended_links.len(), 1);
        let cap = |data: &[u8]| {
            navmesh_rs::detour::MeshTile::parse(data.to_vec())
                .unwrap()
                .header
                .max_link_count
        };
        // One outbound link uses three slots; one inbound link uses one.
        assert_eq!(
            cap(tile.data.as_deref().unwrap()) - cap(bare.data.as_deref().unwrap()),
            4
        );
    }
    let (bottom, top) = (links[0].start, links[1].end);
    let nav = baked.into_navmesh(Default::default()).unwrap();
    for (start, end) in [(bottom, top), (top, bottom)] {
        let path = path(&nav, start, end);
        assert_eq!(path.last(), Some(&find_ground(&nav, end)));
        assert_eq!(traversals(&nav, &path), [OffMeshKind::Ladder; 2]);
    }
}

#[test]
fn extended_options_off_preserve_tile_bytes_and_legacy_links() {
    let scene = ladder_scene(false);
    let bare = scene.input();
    let mut c = config();
    c.links = Some(options());
    let baseline = bake::tiles(&bare, &c, &mut Workspace::new()).unwrap();
    let volumes = [ladder_marker()];
    let mut with_shaft = bare;
    with_shaft.volumes = &volumes;
    c.extended_links = Default::default();
    let unchanged = bake::tiles(&with_shaft, &c, &mut Workspace::new()).unwrap();
    assert_eq!(baseline.tiles, unchanged.tiles);
}

#[cfg(feature = "parallel")]
#[test]
fn extended_link_bytes_and_order_match_across_thread_counts() {
    let volumes = [ladder_marker()];
    let ladder = ladder_scene(false);
    let mut shaft = ladder.input();
    shaft.volumes = &volumes;
    let stair = stair_scene();
    let gap = gap_scene();
    for (input, c, kind) in [
        (shaft, ladder_config(), OffMeshKind::Ladder),
        (stair.input(), step_config(1.2), OffMeshKind::Step),
        (gap.input(), gap_config(12.), OffMeshKind::LevelGap),
    ] {
        let serial = bake::tiles(&input, &c, &mut Workspace::new()).unwrap();
        assert!(has_kind(&serial, kind));
        for threads in [1, 4, 16] {
            let pool = rayon::ThreadPoolBuilder::new()
                .num_threads(threads)
                .build()
                .unwrap();
            let parallel = pool
                .install(|| bake::tiles_parallel(&input, &c, &mut Workspace::new()))
                .unwrap();
            assert_eq!(
                serial.tiles, parallel.tiles,
                "{kind:?} at {threads} threads"
            );
        }
    }
}
