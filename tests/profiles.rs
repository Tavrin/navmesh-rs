#![cfg(all(feature = "recast", feature = "detour"))]

use navmesh_rs::bake::profiles::{
    ProfileBakeJob, ProfileBakeSpec, ProfileCatalog, ProfileCatalogBuilder, ProfileError,
    ProfileId, ProfileMesh, ProfileMeshOptions, ProfileMeshes, ProfilePoint, ProfileRevision,
    ProfileTileOutput,
};
use navmesh_rs::bake::{
    self, AreaVolume, BakeConfig, BakeJob, BakeManifest, GeometrySource, GridLayout, Input,
    JobOptions, LinkExtraction, LinkKind, LinkLimits, ManifestDelta, OffMeshSink, Partition,
    TileKey, TileOutput, TriangleSink, VolumeSink,
};
use navmesh_rs::detour::{DefaultFilter, RefState, Status, content_hash};
use navmesh_rs::{Budget, Error, Progress, Workspace};

const REV: ProfileRevision = ProfileRevision::new([7; 16], 1);
const FIRST: TileKey = TileKey { x: 0, z: 0 };
const NARROW: TileKey = TileKey { x: 1, z: 0 };
const SMALL: ProfileId = ProfileId::new(1);
const LARGE: ProfileId = ProfileId::new(2);

#[derive(Clone, Default)]
struct Scene {
    verts: Vec<f32>,
    tris: Vec<i32>,
}

impl Scene {
    fn floor(&mut self, [x, z, x1, z1]: [f32; 4], y: f32) {
        let i = (self.verts.len() / 3) as i32;
        self.verts.extend([x, y, z, x, y, z1, x1, y, z1, x1, y, z]);
        self.tris.extend([i, i + 1, i + 2, i, i + 2, i + 3]);
    }

    fn input<'a>(&'a self, volumes: &'a [AreaVolume<'a>], areas: Option<&'a [u8]>) -> Input<'a> {
        let mut input = Input::new(&self.verts, &self.tris);
        input.volumes = volumes;
        input.areas = areas;
        input.off_mesh.off_mesh_con_count = 1;
        input.off_mesh.off_mesh_con_verts = &[8., 0., 41., 25., 4., 42.];
        input.off_mesh.off_mesh_con_rad = &[1.5];
        input.off_mesh.off_mesh_con_dir = &[0];
        input.off_mesh.off_mesh_con_areas = &[1];
        input.off_mesh.off_mesh_con_flags = &[1];
        input.off_mesh.off_mesh_con_user_id = &[19];
        input
    }

    fn fixture() -> Self {
        let mut scene = Self::default();
        scene.floor([0., 0., 12., 30.], 0.);
        scene.floor([20., 0., 32., 30.], 0.);
        scene.floor([12., 7., 20., 10.], 0.); // three-meter shortcut
        scene.floor([12., 20., 20., 28.], 0.); // eight-meter detour
        scene.floor([32., 7., 59., 10.], 0.); // the large agent loses this tile
        scene.floor([2., 34., 15., 48.], 0.);
        scene.floor([17., 35., 31., 49.], 4.); // one-way authored lift
        scene.floor([2., 34., 15., 48.], 5.); // stacked floor
        scene.floor([8., 15., 10., 19.], 1.25); // low ceiling
        let i = (scene.verts.len() / 3) as i32;
        scene
            .verts
            .extend([33., 0., 7., 34.75, 0., 10., 35.5, 0., 9.]);
        scene.tris.extend([i, i + 1, i + 2]); // diagonal crossing a native halo
        scene
    }
}

const SEAM_VOLUME: [f32; 12] = [31., 0., 7., 31., 0., 11., 34., 0., 11., 34., 0., 7.];
const INNER_VOLUME: [f32; 12] = [30., 0., 8., 30., 0., 9., 33., 0., 9., 33., 0., 8.];

fn volumes() -> [AreaVolume<'static>; 2] {
    let mut first = AreaVolume::default();
    first.verts = &SEAM_VOLUME;
    first.hmin = -0.5;
    first.hmax = 0.5;
    first.area = 4;
    let mut second = first;
    second.verts = &INNER_VOLUME;
    second.area = 6;
    [first, second]
}

struct Mixed<'a>(Input<'a>);

impl GeometrySource for Mixed<'_> {
    fn triangles(&self, _: [f32; 3], _: [f32; 3], out: &mut TriangleSink) -> Result<(), Error> {
        for (id, tri) in self.0.tris.as_chunks::<3>().0.iter().enumerate() {
            let v = |index| {
                let i = index as usize * 3;
                [self.0.verts[i], self.0.verts[i + 1], self.0.verts[i + 2]]
            };
            let area = (id % 5 == 1).then_some(5);
            out.push(id as u64, [v(tri[0]), v(tri[1]), v(tri[2])], area)?;
        }
        Ok(())
    }

    fn volumes(&self, min: [f32; 3], max: [f32; 3], out: &mut VolumeSink) -> Result<(), Error> {
        self.0.volumes(min, max, out)
    }

    fn off_mesh(&self, min: [f32; 3], max: [f32; 3], out: &mut OffMeshSink) -> Result<(), Error> {
        self.0.off_mesh(min, max, out)
    }
}

fn config(radius: i32) -> BakeConfig {
    let mut config = BakeConfig::default();
    config.recast.cs = 0.25;
    config.recast.ch = 0.25;
    config.recast.tile_size = 128;
    config.recast.walkable_radius = radius;
    config.walkable_radius = radius as f32 * config.recast.cs;
    config.recast.walkable_height = 8;
    config.recast.walkable_climb = 2;
    config.walkable_climb = 0.5;
    config.recast.min_region_area = 2;
    config.recast.merge_region_area = 4;
    config
}

fn spec(id: ProfileId, config: BakeConfig, input: &Input<'_>) -> ProfileBakeSpec {
    ProfileBakeSpec::new(id, config, GridLayout::from_input(input, &config).unwrap()).unwrap()
}

fn options() -> JobOptions {
    let mut options = JobOptions::default();
    options.reports = true;
    options.max_in_flight = 2;
    options
}

fn output<S: GeometrySource>(
    job: &mut ProfileBakeJob<'_, S>,
    allowance: Option<u64>,
) -> Vec<ProfileTileOutput> {
    let mut tiles = Vec::new();
    loop {
        let progress = match allowance {
            Some(units) => job.step(&mut Budget::new(units)),
            None => job.run_parallel_chunk(),
        }
        .unwrap();
        tiles.extend(std::iter::from_fn(|| job.next_ready()));
        if progress == Progress::Done {
            return tiles;
        }
    }
}

fn profile_tiles<S: GeometrySource>(
    source: &S,
    spec: ProfileBakeSpec,
    revision: ProfileRevision,
) -> (Vec<ProfileTileOutput>, ManifestDelta) {
    let mut job = spec
        .bake(source, revision, spec.grid().keys(), options())
        .unwrap();
    let tiles = output(&mut job, None);
    (tiles, job.manifest_delta())
}

fn standalone<S: GeometrySource>(
    source: &S,
    spec: ProfileBakeSpec,
) -> (Vec<TileOutput>, ManifestDelta) {
    let mut job = BakeJob::new(
        source,
        spec.config(),
        spec.grid(),
        spec.grid().keys(),
        options(),
    )
    .unwrap();
    job.run_parallel().unwrap();
    let tiles = std::iter::from_fn(|| job.next_ready()).collect();
    (tiles, job.manifest_delta())
}

fn compare<S: GeometrySource>(source: &S, spec: ProfileBakeSpec) -> Vec<ProfileTileOutput> {
    let mut oracle = BakeJob::new(
        source,
        spec.config(),
        spec.grid(),
        spec.grid().keys(),
        options(),
    )
    .unwrap();
    oracle.run_parallel().unwrap();
    let expected: Vec<_> = std::iter::from_fn(|| oracle.next_ready()).collect();
    let mut job = spec
        .bake(source, REV, spec.grid().keys(), options())
        .unwrap();
    let actual = output(&mut job, None);
    assert_eq!(job.manifest_delta(), oracle.manifest_delta());
    assert_eq!(job.report(), oracle.report());
    assert!(actual.iter().map(ProfileTileOutput::tile).eq(&expected));
    actual
}

fn extraction() -> LinkExtraction {
    LinkExtraction {
        drop: Some(LinkLimits {
            distance: 5.,
            up: 0.,
            down: 6.,
            clearance: 0.0,
        }),
        jump: None,
        climb: None,
        spacing_cells: 4,
        area_ids: [2, 3, 4],
        flags: [1, 1, 1],
    }
}

#[test]
fn native_and_authored_halos_match_independent_bakes() {
    let scene = Scene::fixture();
    let marks = volumes();
    let areas: Vec<u8> = (0..scene.tris.len() / 3)
        .map(|id| if id % 5 == 1 { 5 } else { 63 })
        .collect();
    let input = scene.input(&marks, Some(&areas));
    let source = Mixed(input);
    let mut compared = 0;
    let mut empty = 0;
    let mut links = 0;
    let mut canonical = Vec::new();
    for partition in [Partition::Watershed, Partition::Monotone, Partition::Layers] {
        for extract in [false, true] {
            for radius in [2, 4, 8] {
                let mut cfg = config(radius);
                cfg.partition = partition;
                cfg.links = extract.then(extraction);
                let spec = spec(ProfileId::new(radius as u32), cfg, &input);
                let native = compare(&source, spec);
                let in_memory = bake::tiles(&input, &cfg, &mut Workspace::new()).unwrap();
                for (profile, plain) in native.iter().zip(&in_memory.tiles) {
                    let tile = profile.tile();
                    assert_eq!((tile.key.x, tile.key.z), (plain.x, plain.y));
                    assert_eq!(tile.data, plain.data);
                    assert_eq!(tile.triangulation, plain.triangulation);
                    assert_eq!(tile.links, plain.links);
                }
                assert_eq!(in_memory.tiles.len(), native.len());
                let common_cells = if extract { 31 } else { 11 };
                let common = spec.grid().with_halo_cells(&cfg, common_cells).unwrap();
                let common = ProfileBakeSpec::new(spec.id(), cfg, common).unwrap();
                for tile in native.into_iter().chain(compare(&source, common)) {
                    compared += 1;
                    canonical.extend(tile.profile().0.to_le_bytes());
                    let tile = tile.into_tile();
                    empty += usize::from(tile.data.is_none());
                    links += tile.links.len();
                    canonical.extend(tile.key.x.to_le_bytes());
                    canonical.extend(tile.key.z.to_le_bytes());
                    canonical.push(u8::from(tile.data.is_some()));
                    canonical.extend(tile.input_hash.to_le_bytes());
                    canonical.extend(tile.output_hash.unwrap_or(0).to_le_bytes());
                    let data = tile.data.unwrap_or_default();
                    canonical.extend((data.len() as u64).to_le_bytes());
                    canonical.extend(data);
                    canonical.extend((tile.links.len() as u64).to_le_bytes());
                    for link in tile.links {
                        for value in link.start.into_iter().chain(link.end).chain([link.radius]) {
                            canonical.extend(value.to_bits().to_le_bytes());
                        }
                        canonical.push(match link.kind {
                            LinkKind::Drop => 0,
                            LinkKind::Jump => 1,
                            LinkKind::Climb => 2,
                        });
                        canonical.push(link.area);
                        canonical.extend(link.flags.to_le_bytes());
                    }
                }
            }
        }
    }
    assert!(empty > 0);
    assert!(links > 0);
    println!(
        "{compared} matching results, {empty} empty, {links} generated links, SHA-256 {:02x?}",
        content_hash(&canonical)
    );
}

#[test]
fn zero_radius_same_voxel_bucket_and_translated_grid_keep_exact_bytes() {
    let mut scene = Scene::default();
    scene.floor([200.25, -49.75, 250.25, -20.75], -7.5);
    let input = Input::new(&scene.verts, &scene.tris);
    for (cells, world, height) in [(0, 0., 8), (2, 0.5, 8), (2, 0.51, 8), (2, 0.5, 10)] {
        let mut cfg = config(cells);
        cfg.recast.cs = 0.3;
        cfg.recast.walkable_height = height;
        cfg.walkable_height = height as f32 * cfg.recast.ch;
        cfg.walkable_radius = world;
        let spec = spec(SMALL, cfg, &input);
        let native = compare(&input, spec);
        let plain = bake::tiles(&input, &cfg, &mut Workspace::new()).unwrap();
        assert!(
            native
                .iter()
                .map(|tile| &tile.tile().data)
                .eq(plain.tiles.iter().map(|tile| &tile.data))
        );
    }
    let mut cfg = config(8);
    let grid = GridLayout::from_input(&input, &cfg).unwrap();
    assert!(grid.with_halo_cells(&cfg, 10).is_err());
    assert!(grid.with_halo_cells(&cfg, i32::MAX).is_err());
    cfg.links = Some(extraction());
    assert!(grid.with_halo_cells(&cfg, 30).is_err());
    assert!(grid.with_halo_cells(&cfg, 31).is_ok());
    cfg.recast.walkable_radius = 128;
    let grid = GridLayout::from_input(&input, &cfg).unwrap();
    assert!(ProfileBakeSpec::new(SMALL, cfg, grid).is_err());
}

#[test]
fn delivery_and_manifest_are_independent_of_stage_budgets_and_workers() {
    let scene = Scene::fixture();
    let marks = volumes();
    let input = scene.input(&marks, None);
    for (id, radius) in [(SMALL, 2), (LARGE, 8)] {
        let mut cfg = config(radius);
        cfg.links = Some(extraction());
        let spec = spec(id, cfg, &input);
        let (expected, delta) = standalone(&input, spec);
        let run = |allowance| {
            let mut job = spec
                .bake(&input, REV, spec.grid().keys(), options())
                .unwrap();
            let tiles = output(&mut job, allowance);
            assert!(
                tiles
                    .iter()
                    .all(|tile| tile.profile() == id && tile.revision() == REV)
            );
            assert!(tiles.iter().map(ProfileTileOutput::tile).eq(&expected));
            assert_eq!(job.manifest_delta(), delta);
        };
        for budget in [1, 17, 1000] {
            run(Some(budget));
        }
        #[cfg(feature = "parallel")]
        for workers in [1, 4, 16] {
            rayon::ThreadPoolBuilder::new()
                .num_threads(workers)
                .build()
                .unwrap()
                .install(|| run(None));
        }
    }
}

fn catalog(
    tiles: &[ProfileTileOutput],
    spec: ProfileBakeSpec,
    options: ProfileMeshOptions,
) -> ProfileCatalog {
    let mut builder = ProfileCatalogBuilder::new(REV, spec, options).unwrap();
    for tile in tiles.iter().rev() {
        builder.insert(tile).unwrap();
    }
    let made = builder.finish();
    let restored = ProfileCatalog::from_bytes(&made.to_bytes(), REV, spec, options).unwrap();
    assert_eq!(made.digest(), restored.digest());
    restored
}

fn install(
    mesh: &mut ProfileMesh,
    tiles: &[ProfileTileOutput],
) -> Vec<navmesh_rs::bake::profiles::ProfileTileRef> {
    tiles
        .iter()
        .filter_map(|out| {
            let tile = out.tile();
            let catalog = mesh.catalog();
            let address = catalog.tile(tile.key).unwrap();
            let ready = catalog.prepare(address, tile.data.clone()).unwrap();
            mesh.commit(ready).unwrap()
        })
        .collect()
}

fn nearest(mesh: &ProfileMesh, xyz: [f32; 3]) -> ProfilePoint {
    mesh.query(2048)
        .unwrap()
        .find_nearest_poly(&xyz, &[0.6, 1., 0.6], &DefaultFilter::default())
        .unwrap()
        .unwrap()
}

#[cfg(feature = "dynamic")]
#[test]
fn profile_gate_is_local_and_survives_streaming_and_residency_restore() {
    use navmesh_rs::Limits;
    use navmesh_rs::dynamic::{DynamicLayer, Effect, ModifierId, ModifierSpec, Selector};

    let mut scene = Scene::default();
    scene.floor([0., 0., 28., 28.], 0.);
    let input = Input::new(&scene.verts, &scene.tris);
    let make = |id, radius| {
        let spec = spec(id, config(radius), &input);
        let (tiles, _) = profile_tiles(&input, spec, REV);
        let mesh =
            ProfileMesh::new(catalog(&tiles, spec, ProfileMeshOptions::new(4, 2048))).unwrap();
        (mesh, tiles)
    };
    let (mut small, tiles) = make(SMALL, 2);
    let (mut large, large_tiles) = make(LARGE, 4);
    let installed = install(&mut small, &tiles);
    install(&mut large, &large_tiles);

    let center = [14., 0., 14.];
    let original = nearest(&small, center).polygon;
    let peer = nearest(&large, center).polygon;
    let gate = ModifierId(1);
    let mut layer = DynamicLayer::new(2, Limits::UNLIMITED).unwrap();
    layer
        .upsert(ModifierSpec::new(
            gate,
            Selector::Prism {
                verts_xz: vec![[0., 0.], [28., 0.], [28., 28.], [0., 28.]],
                ymin: -1.,
                ymax: 1.,
                area: None,
            },
            Effect::ClearFlags(u16::MAX),
        ))
        .unwrap();
    layer.set_enabled(gate, false).unwrap();
    small.set_dynamic_layer(Some(layer));
    let epoch = small.epoch();
    let peer_epoch = large.epoch();
    let residency = small.save_residency().unwrap();
    assert!(small.set_modifier_enabled(gate, true).unwrap());
    assert_eq!(small.epoch(), epoch);
    assert_eq!(small.save_residency().unwrap(), residency);
    let closed = |mesh: &ProfileMesh| {
        mesh.query(32)
            .unwrap()
            .find_nearest_poly(&center, &[1.; 3], &DefaultFilter::default())
            .unwrap()
            .is_none()
    };
    assert!(closed(&small));
    small.unload(&installed[0]).unwrap();
    install(&mut small, &tiles);
    assert!(closed(&small));
    assert_eq!(small.ref_state(&original).unwrap(), RefState::Valid);

    let state = small.dynamic_layer().unwrap().to_bytes();
    let mut restored = ProfileMesh::new(small.catalog().clone()).unwrap();
    restored.restore_residency(&residency).unwrap();
    restored.set_dynamic_layer(Some(
        DynamicLayer::from_bytes(&state, Limits::UNLIMITED).unwrap(),
    ));
    install(&mut restored, &tiles);
    assert!(closed(&restored));
    assert!(restored.set_modifier_enabled(gate, false).unwrap());
    assert_eq!(
        nearest(&restored, center).polygon.numeric(),
        original.numeric()
    );
    assert_eq!(large.epoch(), peer_epoch);
    assert_eq!(nearest(&large, center).polygon, peer);
}

#[test]
fn routes_and_directed_links_are_profile_local() {
    let scene = Scene::fixture();
    let input = scene.input(&[], None);
    let mut meshes = ProfileMeshes::new();
    for (id, radius) in [(SMALL, 2), (LARGE, 8)] {
        let spec = spec(id, config(radius), &input);
        let (tiles, _) = profile_tiles(&input, spec, REV);
        let mut options = ProfileMeshOptions::new(8, 2048);
        options.global_off_mesh_links = true;
        let catalog = catalog(&tiles, spec, options);
        let narrow = catalog.tile(NARROW).unwrap();
        assert_eq!(narrow.raw_hash().is_some(), id == SMALL);
        meshes.insert(catalog).unwrap();
        assert_eq!(
            meshes.get(id).unwrap().is_column_unloaded(NARROW),
            id == SMALL
        );
        for result in &tiles {
            let output = result.tile();
            let catalog = meshes.get(id).unwrap().catalog();
            let address = catalog.tile(output.key).unwrap();
            let prepared = catalog.prepare(address, output.data.clone()).unwrap();
            meshes.commit(id, prepared).unwrap();
        }
        let mesh = meshes.get(id).unwrap();
        assert!(!mesh.is_column_unloaded(NARROW));

        let (start, end) = (nearest(mesh, [6., 0., 8.5]), nearest(mesh, [25., 0., 8.5]));
        let mut query = mesh.query(2048).unwrap();
        let filter = DefaultFilter::default();
        let (status, path) = query.find_path(&start, &end, &filter, 64).unwrap();
        assert!(!status.has_detail(Status::PARTIAL_RESULT));
        query
            .init_sliced_find_path(&start, &end, &filter, 0)
            .unwrap();
        while query
            .update_sliced_find_path(1, &filter)
            .unwrap()
            .0
            .is_in_progress()
        {}
        let (_, sliced) = query.finalize_sliced_find_path(&filter, 64).unwrap();
        assert_eq!(sliced.last(), Some(&end.polygon));
        let (_, corners) = query
            .find_straight_path(&start.position, &end.position, &path, 64, 0)
            .unwrap();
        assert_eq!(corners.iter().any(|p| p.position[2] >= 19.0), id == LARGE);
        let graph = mesh
            .connectivity(&mut Budget::unlimited())
            .unwrap()
            .into_value()
            .unwrap();
        assert!(
            graph
                .same_island(mesh, &start.polygon, &end.polygon)
                .unwrap()
        );

        let (base, top) = (nearest(mesh, [8., 0., 41.]), nearest(mesh, [25., 4., 42.]));
        let (status, lift) = query.find_path(&base, &top, &filter, 32).unwrap();
        assert!(!status.has_detail(Status::PARTIAL_RESULT));
        assert_eq!(lift.len(), 3);
        assert!(query.off_mesh_endpoints(&lift[0], &lift[1]).is_ok());
        let (reverse, _) = query.find_path(&top, &base, &filter, 32).unwrap();
        assert!(reverse.has_detail(Status::PARTIAL_RESULT));
        let a = graph
            .reachable_from(mesh, &base.polygon, &mut Budget::unlimited())
            .unwrap()
            .into_value()
            .unwrap();
        let b = graph
            .reachable_from(mesh, &top.polygon, &mut Budget::unlimited())
            .unwrap()
            .into_value()
            .unwrap();
        assert!(a.reachable_polys > b.reachable_polys);
    }
    let foreign = nearest(meshes.get(SMALL).unwrap(), [6., 0., 8.5]);
    assert!(matches!(
        meshes.query(LARGE, 64).unwrap().find_path(
            &foreign,
            &foreign,
            &DefaultFilter::default(),
            8
        ),
        Err(ProfileError::BindingMismatch)
    ));
}

#[test]
fn catalog_and_residency_reject_equal_payloads_from_other_bindings() {
    let mut scene = Scene::default();
    scene.floor([0., 0., 62., 30.], 0.);
    let input = Input::new(&scene.verts, &scene.tris);
    let settings = config(2);
    let first = spec(SMALL, settings, &input);
    let other = spec(LARGE, settings, &input);
    let (tiles, _) = profile_tiles(&input, first, REV);
    let (other_tiles, _) = profile_tiles(&input, other, REV);

    use navmesh_rs::detour::{RefLayout, SlotPolicy};

    let layouts = if cfg!(feature = "polyref64") {
        vec![None, Some(RefLayout::default()), Some(RefLayout::WIDE_SALT)]
    } else {
        vec![None, Some(RefLayout::new(19, 2, 11))]
    };
    for (layout, keyed) in layouts
        .into_iter()
        .map(|layout| (layout, false))
        .chain([(None, true)])
    {
        let mut options = ProfileMeshOptions::new(4, 2048);
        options.ref_layout = layout;
        if keyed {
            options.slot_policy = SlotPolicy::Keyed {
                origin: [0, 0],
                width: 2,
                height: 1,
                max_layers: 1,
            };
        }
        let a = catalog(&tiles, first, options);
        let b = catalog(&other_tiles, other, options);
        let address = a.tile(FIRST).unwrap();
        let raw = tiles[0].tile().data.clone();
        assert_eq!(address.raw_hash(), b.tile(FIRST).unwrap().raw_hash());
        assert_eq!(
            address.raw_hash() != address.prepared_hash(),
            layout == Some(navmesh_rs::detour::RefLayout::WIDE_SALT)
        );
        assert!(matches!(
            b.prepare(address, raw.clone()),
            Err(ProfileError::BindingMismatch)
        ));
        let mut broken = raw.clone().unwrap();
        broken[4] ^= 1;
        assert!(matches!(
            a.prepare(address, Some(broken)),
            Err(ProfileError::TileMismatch)
        ));

        let mut bad_version = a.to_bytes();
        bad_version[4] = 50;
        assert!(matches!(
            ProfileCatalog::from_bytes(&bad_version, REV, first, options),
            Err(ProfileError::UnsupportedVersion(50))
        ));
        let mut wrong_coordinate = a.to_bytes();
        // Preserve the original address but point its hashes/count at the second tile.
        let entry = wrong_coordinate[76 + 77 + 9..].to_vec();
        wrong_coordinate[76 + 9..76 + 77].copy_from_slice(&entry);
        let crafted = ProfileCatalog::from_bytes(&wrong_coordinate, REV, first, options).unwrap();
        let wrong = crafted.tile(FIRST).unwrap();
        assert!(matches!(
            crafted.prepare(wrong, tiles[1].tile().data.clone()),
            Err(ProfileError::TileMismatch)
        ));

        let mut one = ProfileMesh::new(a.clone()).unwrap();
        let mut two = ProfileMesh::new(b.clone()).unwrap();
        let ticket = a.prepare(address, raw.clone()).unwrap();
        assert!(matches!(
            two.commit(ticket),
            Err(ProfileError::BindingMismatch)
        ));
        let one_refs = install(&mut one, &tiles);
        install(&mut two, &other_tiles);
        let one_poly = nearest(&one, [10., 0., 10.]).polygon;
        let two_poly = nearest(&two, [10., 0., 10.]).polygon;
        assert_eq!(one_poly.numeric(), two_poly.numeric());
        assert_ne!(one_poly, two_poly);
        assert!(matches!(
            two.unload(&one_refs[0]),
            Err(ProfileError::BindingMismatch)
        ));
        let mut graph = one
            .connectivity(&mut Budget::unlimited())
            .unwrap()
            .into_value()
            .unwrap();
        assert!(matches!(
            graph.update(&two, &mut Budget::unlimited()),
            Err(ProfileError::BindingMismatch)
        ));
        let epoch = two.epoch();
        one.unload(&one_refs[0]).unwrap();
        assert!(matches!(
            graph.same_island(&one, &one_poly, &one_poly),
            Err(ProfileError::Navigation(Error::MeshChanged))
        ));
        assert!(
            graph
                .update(&one, &mut Budget::unlimited())
                .unwrap()
                .is_complete()
        );
        assert!(matches!(
            one.ref_state(&one_poly),
            Ok(RefState::Unloaded(_))
        ));
        assert_eq!(two.ref_state(&two_poly).unwrap(), RefState::Valid);
        assert_eq!(two.epoch(), epoch);

        let record = one.save_residency().unwrap();
        let saved_ref = one.save_reference(&one_poly).unwrap();
        let mut wrong = ProfileMesh::new(b).unwrap();
        assert!(matches!(
            wrong.restore_residency(&record),
            Err(ProfileError::BindingMismatch)
        ));
        assert_eq!(wrong.epoch(), 0);
        let mut restored = ProfileMesh::new(a.clone()).unwrap();
        restored.restore_residency(&record).unwrap();
        let scoped = restored.restore_reference(&saved_ref).unwrap();
        assert_eq!(scoped.numeric(), one_poly.numeric());
        install(&mut restored, &tiles);
        assert_eq!(restored.ref_state(&scoped).unwrap(), RefState::Valid);
        let epoch = restored.epoch();
        install(&mut restored, &tiles);
        assert_eq!(restored.epoch(), epoch);
        assert_eq!(restored.save_residency().unwrap(), record);

        let mut reordered = ProfileMesh::new(a).unwrap();
        for tile in tiles.iter().rev() {
            install(&mut reordered, core::slice::from_ref(tile));
        }
        if keyed {
            assert_eq!(
                reordered.restore_reference(&saved_ref).unwrap().numeric(),
                scoped.numeric()
            );
        } else {
            assert!(matches!(
                reordered.restore_reference(&saved_ref),
                Err(ProfileError::BindingMismatch)
            ));
        }
    }
}

#[test]
fn catalog_binds_spare_link_reservations() {
    let mut scene = Scene::default();
    scene.floor([0., 0., 28., 28.], 0.);
    let input = Input::new(&scene.verts, &scene.tris);
    let original = config(2);
    let first = spec(SMALL, original, &input);
    let mut reserved = original;
    reserved.spare_links_per_tile = 2;
    let second = spec(SMALL, reserved, &input);
    let options = ProfileMeshOptions::new(4, 2048);
    let (tiles, _) = profile_tiles(&input, first, REV);
    let old = catalog(&tiles, first, options);
    assert!(matches!(
        ProfileCatalog::from_bytes(&old.to_bytes(), REV, second, options),
        Err(ProfileError::BindingMismatch)
    ));
}

#[test]
fn parked_slots_can_be_released_for_a_profile_streaming_window() {
    let mut scene = Scene::default();
    scene.floor([0., 0., 62., 30.], 0.);
    let input = Input::new(&scene.verts, &scene.tris);
    let spec = spec(SMALL, config(2), &input);
    let (tiles, _) = profile_tiles(&input, spec, REV);
    let catalog = catalog(&tiles, spec, ProfileMeshOptions::new(1, 2048));
    let mut mesh = ProfileMesh::new(catalog.clone()).unwrap();
    let here = catalog.tile(FIRST).unwrap();
    let next = catalog.tile(NARROW).unwrap();
    let tile = mesh
        .commit(catalog.prepare(here, tiles[0].tile().data.clone()).unwrap())
        .unwrap()
        .unwrap();
    let polygon = nearest(&mesh, [10., 0., 10.]).polygon;
    mesh.unload(&tile).unwrap();
    let prepare_next = || catalog.prepare(next, tiles[1].tile().data.clone()).unwrap();
    assert!(matches!(
        mesh.commit(prepare_next()),
        Err(ProfileError::Navigation(Error::NoFreeTile))
    ));
    mesh.forget_unloaded(here).unwrap();
    assert!(mesh.is_column_unloaded(FIRST));
    assert!(mesh.commit(prepare_next()).unwrap().is_some());
    assert_eq!(mesh.ref_state(&polygon).unwrap(), RefState::Stale);
}

#[test]
fn incremental_catalogs_match_full_rebakes_and_preserve_other_profiles() {
    let mut before = Scene::default();
    before.floor([0., 0., 63., 28.], 0.);
    let mut after = before.clone();
    after.floor([5., 5., 9., 9.], 1.);
    after.floor([32.4, 12., 34., 14.], 1.);
    after.floor([70., 12., 85., 22.], 0.);
    let before = Input::new(&before.verts, &before.tris);
    let after = Input::new(&after.verts, &after.tris);
    let cfg = config(2);
    let grid = GridLayout::new([0.; 3], 4., 2, 1, &cfg).unwrap();
    let grow = GridLayout::new(grid.origin, grid.max_y, 3, 1, &cfg).unwrap();
    let mut current = ProfileBakeSpec::new(SMALL, cfg, grid).unwrap();
    let other = ProfileBakeSpec::new(
        LARGE,
        config(8),
        GridLayout::new(grid.origin, grid.max_y, 2, 1, &config(8)).unwrap(),
    )
    .unwrap();
    let opts = ProfileMeshOptions::new(8, 2048);
    let (before_tiles, delta) = profile_tiles(&before, current, REV);
    let (other_tiles, _) = profile_tiles(&before, other, REV);
    let mut a = ProfileMesh::new(catalog(&before_tiles, current, opts)).unwrap();
    let mut b = ProfileMesh::new(catalog(&other_tiles, other, opts)).unwrap();
    install(&mut a, &before_tiles);
    install(&mut b, &other_tiles);
    let old = nearest(&a, [15., 0., 15.]).polygon;
    let elsewhere = nearest(&b, [15., 0., 15.]).polygon;
    let b_epoch = b.epoch();
    let stale_ticket = a
        .catalog()
        .prepare(
            a.catalog().tile(FIRST).unwrap(),
            before_tiles[0].tile().data.clone(),
        )
        .unwrap();
    let mut manifest = BakeManifest::new(current.config(), current.grid()).unwrap();
    manifest.apply(&delta).unwrap();
    let boxes = [
        [5., 1., 5., 9., 1., 9.],      // interior
        [32.4, 1., 12., 34., 1., 14.], // just beyond the first tile's edge
        [70., 0., 12., 85., 0., 22.],  // outside the original grid and its halo
    ];
    for (version, layout, edits) in [(2, grow, boxes.as_slice()), (3, grid, &[][..])] {
        let revision = ProfileRevision::new(REV.map, version);
        let next = ProfileBakeSpec::new(SMALL, cfg, layout).unwrap();
        let dirty = manifest
            .dirty_from_boxes(next.config(), next.grid(), edits)
            .unwrap();
        let mut job = next.bake(&after, revision, dirty.keys, options()).unwrap();
        job.skip_unchanged(&manifest, &mut Workspace::new())
            .unwrap();
        let baked = output(&mut job, Some(17));
        let mut builder = ProfileCatalogBuilder::revise(a.catalog(), revision, next).unwrap();
        assert!(matches!(
            builder.insert(&before_tiles[0]),
            Err(ProfileError::BindingMismatch)
        ));
        for tile in &baked {
            builder.insert(tile).unwrap();
        }
        let next_catalog = builder.finish();
        let (full, expected_delta) = profile_tiles(&after, next, revision);
        let mut oracle = ProfileCatalogBuilder::new(revision, next, opts).unwrap();
        for tile in &full {
            oracle.insert(tile).unwrap();
        }
        assert_eq!(next_catalog.to_bytes(), oracle.finish().to_bytes());
        manifest.retarget(next.config(), next.grid()).unwrap();
        manifest.apply(&job.manifest_delta()).unwrap();
        assert_eq!(manifest.entries, expected_delta.entries);
        a.advance_catalog(next_catalog).unwrap();
        install(&mut a, &baked);
        assert_eq!(b.epoch(), b_epoch);
        assert_eq!(b.ref_state(&elsewhere).unwrap(), RefState::Valid);
        assert_eq!(a.ref_state(&old).unwrap(), RefState::Stale);
        assert!(a.save_residency().is_ok());
        current = next;
    }
    assert!(matches!(
        a.commit(stale_ticket),
        Err(ProfileError::BindingMismatch)
    ));

    let changed = ProfileBakeSpec::new(
        SMALL,
        config(4),
        GridLayout::new(grid.origin, grid.max_y, 2, 1, &config(4)).unwrap(),
    )
    .unwrap();
    let dirty = manifest
        .dirty_from_boxes(changed.config(), changed.grid(), &[])
        .unwrap();
    assert!(dirty.full_rebake);
    assert_eq!(dirty.keys, changed.grid().keys());
    assert_ne!(
        current.config().walkable_radius,
        changed.config().walkable_radius
    );
    let result = compare(&after, changed);
    assert_eq!(result.len(), 2);
}

#[test]
fn cancellation_and_limits_publish_only_delivered_tiles() {
    let scene = Scene::fixture();
    let input = scene.input(&[], None);
    let spec0 = spec(SMALL, config(2), &input);
    let mut job = spec0
        .bake(&input, REV, spec0.grid().keys(), options())
        .unwrap();
    assert_eq!(job.step(&mut Budget::new(0)).unwrap(), Progress::Exhausted);
    assert!(job.next_ready().is_none());
    loop {
        job.step(&mut Budget::new(1)).unwrap();
        if job.next_ready().is_some() {
            break;
        }
    }
    job.cancel_token().cancel();
    assert!(
        matches!(job.step(&mut Budget::new(17)), Err(error) if error.error == Error::Cancelled)
    );
    assert!(job.next_ready().is_none());
    assert_eq!(job.manifest_delta().entries.len(), 1);

    let mut cfg = config(2);
    cfg.tile_limits.max_work = 10;
    let limited = spec(SMALL, cfg, &input);
    let mut reference = BakeJob::new(
        &input,
        &cfg,
        limited.grid(),
        limited.grid().keys(),
        options(),
    )
    .unwrap();
    let mut profile = limited
        .bake(&input, REV, limited.grid().keys(), options())
        .unwrap();
    assert_eq!(
        reference.run_parallel_chunk().unwrap_err(),
        profile.run_parallel_chunk().unwrap_err()
    );
    assert!(profile.next_ready().is_none());
    assert!(profile.manifest_delta().entries.is_empty());
}

#[cfg(feature = "crowd")]
#[test]
fn legacy_crowds_select_profile_for_admission_goals_and_ticks() {
    use navmesh_rs::bake::profiles::ProfileCrowd;
    use navmesh_rs::crowd::AgentParams;

    let scene = Scene::fixture();
    let input = scene.input(&[], None);
    let make = |id| {
        let spec = spec(id, config(2), &input);
        let (tiles, _) = profile_tiles(&input, spec, REV);
        let mut mesh =
            ProfileMesh::new(catalog(&tiles, spec, ProfileMeshOptions::new(8, 2048))).unwrap();
        install(&mut mesh, &tiles);
        mesh
    };
    let (a, b) = (make(SMALL), make(LARGE));
    let mut ws = Workspace::new();
    let mut crowd = ProfileCrowd::new(&a, &mut ws, 2).unwrap();
    let mut agent = AgentParams::default();
    agent.radius = 0.5;
    agent.height = 2.1;
    assert!(crowd.add_agent(&a, &mut ws, [6., 0., 8.5], agent).is_err());
    agent.height = 2.0;
    let first = crowd.add_agent(&a, &mut ws, [6., 0., 8.5], agent).unwrap();
    let target = nearest(&a, [25., 0., 8.5]);
    let foreign = nearest(&b, [25., 0., 8.5]);
    assert!(matches!(
        crowd.request_move_target(&a, &first, &foreign),
        Err(ProfileError::BindingMismatch)
    ));
    assert!(matches!(
        crowd.update(&b, &mut ws, 0.1),
        Err(ProfileError::BindingMismatch)
    ));
    crowd.request_move_target(&a, &first, &target).unwrap();
    let before = crowd.position(&first).unwrap();
    for _ in 0..90 {
        crowd.update(&a, &mut ws, 0.1).unwrap();
    }
    assert!(crowd.position(&first).unwrap()[0] > before[0] + 12.0);
    crowd.remove_agent(&first).unwrap();
    let second = crowd.add_agent(&a, &mut ws, [6., 0., 8.5], agent).unwrap();
    assert!(crowd.position(&second).is_ok());
    assert!(matches!(
        crowd.position(&first),
        Err(ProfileError::BindingMismatch)
    ));
}

#[cfg(feature = "crowd-world")]
#[test]
fn crowd_world_snapshots_are_bound_to_profile_and_replay_with_restored_refs() {
    use navmesh_rs::bake::profiles::ProfileCrowdWorld;
    use navmesh_rs::crowd::world::CrowdWorldParams;
    use navmesh_rs::crowd::{AgentParams, ArrivalPolicy, OffMeshMode};
    use navmesh_rs::{Limits, SnapshotError};

    let mut scene = Scene::default();
    for stripe in 0..104 {
        let x = stripe as f32;
        scene.floor([x, 0., x + 1., 10.], 0.);
    }
    let areas = (0..104)
        .flat_map(|stripe| [1 + stripe % 2; 2])
        .collect::<Vec<_>>();
    let mut input = Input::new(&scene.verts, &scene.tris);
    input.areas = Some(&areas);
    let make = |id| {
        let spec = spec(id, config(2), &input);
        let (tiles, _) = profile_tiles(&input, spec, REV);
        let mut mesh =
            ProfileMesh::new(catalog(&tiles, spec, ProfileMeshOptions::new(8, 2048))).unwrap();
        install(&mut mesh, &tiles);
        (mesh, tiles)
    };
    let ((a, tiles), (b, _)) = (make(SMALL), make(LARGE));
    let mut params = CrowdWorldParams::default();
    params.max_agents = 2;
    params.max_heavy_agents = 2;
    params.max_radius = 0.5;
    params.path_queue.iterations = 1;
    for (arrival_policy, off_mesh, version) in [
        (ArrivalPolicy::Keep, OffMeshMode::Animate, 1u32),
        (ArrivalPolicy::Park, OffMeshMode::Animate, 3),
        (ArrivalPolicy::Keep, OffMeshMode::External, 3),
        (ArrivalPolicy::Park, OffMeshMode::External, 3),
    ] {
        let mut world = ProfileCrowdWorld::new(&a, &params, Limits::default()).unwrap();
        let mut p = AgentParams::default();
        p.radius = 0.5;
        p.arrival_policy = arrival_policy;
        p.off_mesh = off_mesh;
        let agent = world.add_agent(&a, [2., 0., 5.], p).unwrap();
        assert!(matches!(
            world.set_goal(&a, &agent, Some(&nearest(&b, [102., 0., 5.]))),
            Err(ProfileError::BindingMismatch)
        ));
        assert!(matches!(
            world.step(&b, &mut Budget::unlimited()),
            Err(ProfileError::BindingMismatch)
        ));
        world
            .set_goal(&a, &agent, Some(&nearest(&a, [102., 0., 5.])))
            .unwrap();
        let tick = world.step(&a, &mut Budget::unlimited()).unwrap();
        assert_eq!(tick.value().path_queue_depth, 1);
        let state = world.save(&a).unwrap();
        // The NPCW header and residency digest take 72 bytes.
        let (prefix, _) = state.split_at(state.len() - 32);
        assert_eq!(&prefix[72..76], b"DCRW");
        assert_eq!(&prefix[76..80], &version.to_le_bytes());
        assert!(matches!(
            ProfileCrowdWorld::restore(&b, &state, Limits::default()),
            Err(ProfileError::BindingMismatch)
        ));
        let mut restored = ProfileMesh::new(a.catalog().clone()).unwrap();
        restored
            .restore_residency(&a.save_residency().unwrap())
            .unwrap();
        install(&mut restored, &tiles);
        assert_ne!(a.epoch(), restored.epoch());
        let mut unknown = prefix.to_vec();
        unknown[76..80].copy_from_slice(&u32::MAX.to_le_bytes());
        unknown.extend(content_hash(&unknown));
        assert!(matches!(
            ProfileCrowdWorld::restore(&restored, &unknown, Limits::default()),
            Err(ProfileError::Navigation(Error::InvalidSnapshot(
                SnapshotError::WrongVersion
            )))
        ));
        let mut other = ProfileCrowdWorld::restore(&restored, &state, Limits::default()).unwrap();
        let other_agent = other.agent_at(agent.index()).unwrap();
        assert_eq!(other.save(&restored).unwrap(), state);
        assert!(matches!(
            other.position(&agent),
            Err(ProfileError::BindingMismatch)
        ));
        for _ in 0..48 {
            assert_eq!(
                world.step(&a, &mut Budget::unlimited()).unwrap(),
                other.step(&restored, &mut Budget::unlimited()).unwrap(),
            );
            assert_eq!(
                world.position(&agent).unwrap().map(f32::to_bits),
                other.position(&other_agent).unwrap().map(f32::to_bits)
            );
        }
        let final_snapshot = world.save(&a).unwrap();
        assert_eq!(final_snapshot, other.save(&restored).unwrap());
        println!(
            "profile replay SHA-256 {:02x?}",
            content_hash(&final_snapshot)
        );
    }
}

#[cfg(all(target_os = "linux", feature = "parallel"))]
#[allow(dead_code)]
mod common;

#[cfg(all(target_os = "linux", feature = "parallel"))]
#[test]
fn generated_terrain_matches_the_standalone_profile_oracle() {
    let world = common::world::World::new(128., 17);
    for radius in [2, 4, 8] {
        let cfg = config(radius);
        let spec =
            ProfileBakeSpec::new(ProfileId::new(radius as u32), cfg, world.layout(&cfg)).unwrap();
        assert_eq!(compare(&world, spec).len(), spec.grid().keys().len());
    }
}

#[cfg(all(target_os = "linux", feature = "parallel"))]
#[test]
#[ignore = "pinned, interleaved release measurement"]
fn independent_profile_bake_cost() {
    use std::hint::black_box;
    use std::time::{Duration, Instant};

    use common::world::World;

    let world = World::new(256., 17);
    // Frozen 128-cell columns spanning flat, sloped and obstructed terrain.
    let keys = [
        FIRST,
        TileKey { x: 2, z: 2 },
        TileKey { x: 5, z: 3 },
        TileKey { x: 7, z: 7 },
    ];
    let specs = [2, 4, 8].map(|radius| {
        let cfg = config(radius);
        ProfileBakeSpec::new(ProfileId::new(radius as u32), cfg, world.layout(&cfg)).unwrap()
    });
    let run = |profiled: bool, count: usize| {
        let mut totals = [0u64; 3];
        for spec in &specs[..count] {
            let mut budget = Budget::unlimited();
            if profiled {
                let mut job = spec
                    .bake(&world, REV, keys.to_vec(), JobOptions::default())
                    .unwrap();
                assert_eq!(job.step(&mut budget).unwrap(), Progress::Done);
                while let Some(tile) = job.next_ready() {
                    totals[0] += 1;
                    totals[1] += black_box(tile).tile().data.as_ref().map_or(0, Vec::len) as u64;
                }
            } else {
                let mut job = BakeJob::new(
                    &world,
                    spec.config(),
                    spec.grid(),
                    keys.to_vec(),
                    JobOptions::default(),
                )
                .unwrap();
                assert_eq!(job.step(&mut budget).unwrap(), Progress::Done);
                while let Some(tile) = job.next_ready() {
                    totals[0] += 1;
                    totals[1] += black_box(tile).data.as_ref().map_or(0, Vec::len) as u64;
                }
            }
            totals[2] += budget.used();
        }
        totals
    };
    for count in [1, 3] {
        let baseline = run(false, count);
        assert_eq!(baseline, run(true, count));
        let mut repeats = 1;
        loop {
            let start = Instant::now();
            for _ in 0..repeats {
                assert_eq!(baseline, run(false, count));
            }
            if start.elapsed() >= Duration::from_millis(350) {
                break;
            }
            repeats *= 2;
        }
        for pair in 0..9 {
            for profiled in [pair % 2 == 1, pair % 2 == 0] {
                let wall = Instant::now();
                let cpu = cpu_time::ProcessTime::now();
                for _ in 0..repeats {
                    assert_eq!(baseline, run(profiled, count));
                }
                println!(
                    "profiles={count} pair={pair} profiled={profiled} repeats={repeats} wall_ns={} cpu_ns={} tiles={} bytes={} work={}",
                    wall.elapsed().as_nanos(),
                    cpu.elapsed().as_nanos(),
                    baseline[0],
                    baseline[1],
                    baseline[2],
                );
            }
        }
    }
}
