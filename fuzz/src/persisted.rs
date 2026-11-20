//! Valid persisted records for the format-reader targets (`runtime_links`, `frame_snapshot`,
//! `framed_crowd_snapshot`, `profile_catalog`, `profile_records`, `profile_mixed_crowd`,
//! `framed_profile_crowd`), built once per process, plus the byte mutator they share.

use navmesh_rs::bake::profiles::{
    FramedMixedProfileCrowdWorld, MixedProfileCrowdWorld, ProfileBakeSpec, ProfileCatalog,
    ProfileCatalogBuilder, ProfileCrowdWorld, ProfileId, ProfileMeshOptions, ProfileMeshes,
    ProfilePoint, ProfileRevision,
};
use navmesh_rs::bake::{BakeConfig, GridLayout, Input, JobOptions, TileKey};
use navmesh_rs::crowd::platforms::{FramedCrowd, PlatformAgentId};
use navmesh_rs::crowd::world::{CrowdWorld, CrowdWorldParams};
use navmesh_rs::crowd::AgentParams;
use navmesh_rs::detour::{
    DefaultFilter, LinkId, LinkSpec, NavMesh, NavMeshCreateParams, NavMeshParams, content_hash,
    create_nav_mesh_data, create_nav_mesh_data_with_spare_links,
};
use navmesh_rs::frames::{DockSpec, FrameId, FramePoint, FramePose, FrameSet};
use navmesh_rs::{Budget, Limits, Progress};
use std::sync::OnceLock;

/// Limits for the readers: small enough that a mutated count ends in `LimitExceeded`.
pub const LIMITS: Limits = Limits {
    max_bytes: 4 * 1024 * 1024,
    max_work: 100_000,
    ..Limits::UNLIMITED
};

/// Recomputes the trailing SHA-256 of a record that ends in one, so a mutated body
/// still reaches the checks behind the checksum.
pub fn repaired(mut bytes: Vec<u8>) -> Vec<u8> {
    if let Some(end) = bytes.len().checked_sub(32) {
        let digest = content_hash(&bytes[..end]);
        bytes[end..].copy_from_slice(&digest);
    }
    bytes
}

/// Whether `saved` is `input` with only mesh-epoch words lowered. A crowd snapshot restored
/// onto a mesh whose epoch is behind the stored one rebinds that word to the live epoch, so
/// the re-encoding differs from the input in exactly those little-endian `u64` words.
pub fn rebinds_only_epochs(saved: &[u8], input: &[u8]) -> bool {
    if saved.len() != input.len() {
        return false;
    }
    let word = |b: &[u8], at: usize| u64::from_le_bytes(b[at..at + 8].try_into().unwrap_or([0; 8]));
    let mut i = 0;
    while i < saved.len() {
        if saved[i] == input[i] {
            i += 1;
            continue;
        }
        let Some(at) = (i.saturating_sub(7)..=i)
            .find(|&t| t + 8 <= saved.len() && word(saved, t) < word(input, t))
        else {
            return false;
        };
        i = at + 8;
    }
    true
}

/// Applies byte patches to a valid record. `patches` is one window byte (0: the whole
/// body, 1: the first 64 bytes, 2: the last 160 bytes) and then 3-byte patches
/// `[position lo, position hi, value]` inside the window. With `rehash` the last 32
/// bytes are treated as a checksum, excluded from the window and recomputed.
pub fn patched(valid: &[u8], patches: &[u8], rehash: bool) -> Vec<u8> {
    let mut out = valid.to_vec();
    let body = out.len().saturating_sub(if rehash { 32 } else { 0 });
    let Some((&window, rest)) = patches.split_first() else {
        return out;
    };
    let (lo, hi) = match window % 3 {
        0 => (0, body),
        1 => (0, body.min(64)),
        _ => (body.saturating_sub(160), body),
    };
    if hi > lo {
        for c in rest.chunks_exact(3) {
            let at = lo + usize::from(u16::from_le_bytes([c[0], c[1]])) % (hi - lo);
            out[at] = c[2];
        }
    }
    if rehash { repaired(out) } else { out }
}

// ---------------------------------------------------------------------------
// Runtime link layer (DNLK)
// ---------------------------------------------------------------------------

fn link_params() -> NavMeshParams {
    NavMeshParams {
        orig: [0.; 3],
        tile_width: 10.,
        tile_height: 10.,
        max_tiles: 16,
        max_polys: 64,
    }
}

fn link_quad(x: i32, spare: u32) -> Vec<u8> {
    let x0 = x as f32 * 10.;
    create_nav_mesh_data_with_spare_links(
        &NavMeshCreateParams {
            verts: &[0, 0, 0, 0, 0, 10, 10, 0, 10, 10, 0, 0],
            vert_count: 4,
            polys: &[0, 1, 2, 3, 0x800f, 0x800f, 0x800f, 0x800f],
            poly_count: 1,
            nvp: 4,
            poly_flags: &[1],
            poly_areas: &[0],
            tile_x: x,
            bmin: [x0, 0., 0.],
            bmax: [x0 + 10., 1., 10.],
            cs: 1.,
            ch: 1.,
            walkable_climb: 0.5,
            build_bv_tree: true,
            ..Default::default()
        },
        spare,
    )
    .unwrap_or_default()
}

/// A mesh without tiles or links, the target of `restore_runtime_links`.
pub fn link_mesh() -> NavMesh {
    NavMesh::new(&link_params()).expect("link mesh parameters")
}

/// Ground tiles to add after a restore, so definitions land on real polygons.
pub fn link_tiles() -> [Vec<u8>; 2] {
    [link_quad(0, 2), link_quad(3, 2)]
}

/// A saved layer of two links, one enabled and one disabled.
pub fn runtime_links_record() -> &'static [u8] {
    static RECORD: OnceLock<Vec<u8>> = OnceLock::new();
    RECORD.get_or_init(|| {
        let mut nav = link_mesh();
        nav.set_link_layer(Some(1)).expect("link layer");
        for tile in link_tiles() {
            nav.add_tile(tile).expect("ground tile");
        }
        let first = LinkSpec::new(LinkId(73), [5., 0., 5.], [35., 0., 5.], 0.75);
        let mut second = LinkSpec::new(LinkId(90), [2., 0., 2.], [38., 0., 8.], 0.5);
        second.bidirectional = true;
        second.area = 3;
        second.flags = 5;
        nav.add_runtime_link(first, true).expect("first link");
        nav.add_runtime_link(second, false).expect("second link");
        nav.runtime_links_to_bytes()
    })
}

// ---------------------------------------------------------------------------
// Moving-platform frames (DNFR) and framed crowds (DCFR)
// ---------------------------------------------------------------------------

const WORLD: FrameId = FrameId::WORLD;
const CABIN: FrameId = FrameId(1);
const BOARD: LinkId = LinkId(1);
const ALIGHT: LinkId = LinkId(2);
const LOWER: FramePoint = FramePoint::new(WORLD, [7., 0., 5.]);
const UPPER: FramePoint = FramePoint::new(WORLD, [7., 4., 5.]);

fn floor(y: f32, layer: i32, width: u16) -> Vec<u8> {
    create_nav_mesh_data(&NavMeshCreateParams {
        verts: &[0, 0, 0, 0, 0, 10, width, 0, 10, width, 0, 0],
        vert_count: 4,
        polys: &[0, 1, 2, 3, 0x800f, 0x800f, 0x800f, 0x800f],
        poly_count: 1,
        nvp: 4,
        poly_flags: &[1],
        poly_areas: &[0],
        tile_layer: layer,
        bmin: [0., y, 0.],
        bmax: [f32::from(width), y + 1., 10.],
        cs: 1.,
        ch: 1.,
        walkable_climb: 0.5,
        build_bv_tree: true,
        ..Default::default()
    })
    .unwrap_or_default()
}

fn pose(y: f32) -> FramePose {
    FramePose::new([10., y, 0.], 1., 0.).expect("finite pose")
}

/// A lift: a two-floor world, a cabin frame and two docks (boarding enabled, exit not).
pub fn lift() -> FrameSet {
    let mut world = NavMesh::new(&NavMeshParams {
        orig: [0.; 3],
        tile_width: 10.,
        tile_height: 10.,
        max_tiles: 4,
        max_polys: 8,
    })
    .expect("world parameters");
    world.add_tile(floor(0., 0, 10)).expect("lower floor");
    world.add_tile(floor(4., 1, 10)).expect("upper floor");
    let cabin = NavMesh::new_single(floor(0., 0, 5)).expect("cabin tile").0;
    let mut frames = FrameSet::new(world);
    frames.add_frame(CABIN, cabin, pose(0.)).expect("cabin frame");
    let deck = FramePoint::new(CABIN, [0., 0., 5.]);
    frames
        .add_dock(
            DockSpec::new(BOARD, FramePoint::new(WORLD, [10., 0., 5.]), deck, 0.25),
            true,
        )
        .expect("boarding dock");
    frames
        .add_dock(
            DockSpec::new(ALIGHT, deck, FramePoint::new(WORLD, [10., 4., 5.]), 0.25),
            false,
        )
        .expect("exit dock");
    frames
}

/// A saved frame state with a raised cabin and the boarding dock closed.
pub fn frame_record() -> &'static [u8] {
    static RECORD: OnceLock<Vec<u8>> = OnceLock::new();
    RECORD.get_or_init(|| {
        let mut frames = lift();
        frames.set_pose(CABIN, pose(4.)).expect("pose");
        frames.set_dock_enabled(BOARD, false).expect("dock");
        frames.set_dock_enabled(ALIGHT, true).expect("dock");
        frames.to_bytes()
    })
}

struct FramedCrowdFixture {
    frames: FrameSet,
    bytes: Vec<u8>,
}

thread_local! {
    static FRAMED_CROWD: FramedCrowdFixture = {
        let frames = lift();
        let mut crowd = FramedCrowd::<CrowdWorld>::default();
        let mut config = CrowdWorldParams::default();
        config.max_agents = 16;
        config.max_heavy_agents = 16;
        config.max_radius = 0.3;
        config.fixed_dt = 0.05;
        for id in frames.frame_ids() {
            let mesh = frames.mesh(id).expect("frame mesh");
            let local = CrowdWorld::new(mesh, &config, Limits::default()).expect("local crowd");
            crowd.attach(&frames, id, local).expect("attach");
        }
        let mut params = AgentParams::default();
        params.radius = 0.25;
        params.max_speed = 2.;
        params.update_flags = 0;
        let rider = PlatformAgentId(1);
        crowd.add_agent(&frames, rider, LOWER, params).expect("rider");
        crowd.set_goal(&frames, rider, Some(UPPER)).expect("goal");
        crowd.set_wait_cost(&frames, ALIGHT, Some(5.)).expect("wait cost");
        for _ in 0..4 {
            let _ = crowd.step(&frames, &mut Budget::unlimited());
        }
        let bytes = crowd.to_bytes(&frames);
        FramedCrowdFixture { frames, bytes }
    };
}

/// Runs `f` on the lift and a valid framed-crowd record bound to it.
pub fn with_framed_crowd<R>(f: impl FnOnce(&FrameSet, &[u8]) -> R) -> R {
    FRAMED_CROWD.with(|x| f(&x.frames, &x.bytes))
}

// ---------------------------------------------------------------------------
// Bake profiles: catalog, residency and reference records, crowds
// ---------------------------------------------------------------------------

/// Revision every fixture catalog is baked at.
pub const REV: ProfileRevision = ProfileRevision::new([7; 16], 1);
/// The small-agent profile.
pub const SMALL: ProfileId = ProfileId::new(1);
/// The large-agent profile.
pub const LARGE: ProfileId = ProfileId::new(2);

/// One baked profile: its settings, catalog and tile payloads.
pub struct Baked {
    pub id: ProfileId,
    pub spec: ProfileBakeSpec,
    pub options: ProfileMeshOptions,
    pub catalog: ProfileCatalog,
    pub tiles: Vec<(TileKey, Option<Vec<u8>>)>,
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

/// Bakes both profiles over flat floors, each `([x, z, x1, z1], y)`.
pub fn bake(floors: &[([f32; 4], f32)]) -> Vec<Baked> {
    let mut verts = Vec::new();
    let mut tris = Vec::new();
    for &([x, z, x1, z1], y) in floors {
        let i = (verts.len() / 3) as i32;
        verts.extend([x, y, z, x, y, z1, x1, y, z1, x1, y, z]);
        tris.extend([i, i + 1, i + 2, i, i + 2, i + 3]);
    }
    let input = Input::new(&verts, &tris);
    let mut out = Vec::new();
    for (id, radius) in [(SMALL, 1), (LARGE, 3)] {
        let cfg = config(radius);
        let grid = GridLayout::from_input(&input, &cfg).expect("grid");
        let spec = ProfileBakeSpec::new(id, cfg, grid).expect("spec");
        let mut options = ProfileMeshOptions::new(8, 2048);
        options.global_off_mesh_links = true;
        let mut job = spec
            .bake(&input, REV, spec.grid().keys(), JobOptions::default())
            .expect("bake job");
        let mut done = Vec::new();
        loop {
            let progress = job.step(&mut Budget::unlimited()).expect("bake step");
            done.extend(std::iter::from_fn(|| job.next_ready()));
            if progress == Progress::Done {
                break;
            }
        }
        let mut builder = ProfileCatalogBuilder::new(REV, spec, options).expect("builder");
        for tile in done.iter().rev() {
            builder.insert(tile).expect("catalog entry");
        }
        let tiles = done
            .into_iter()
            .map(|t| {
                let t = t.into_tile();
                (t.key, t.data)
            })
            .collect();
        out.push(Baked {
            id,
            spec,
            options,
            catalog: builder.finish(),
            tiles,
        });
    }
    out
}

/// Installs every baked profile with all its tiles resident.
pub fn install(baked: &[Baked]) -> ProfileMeshes {
    let mut meshes = ProfileMeshes::new();
    for b in baked {
        meshes.insert(b.catalog.clone()).expect("insert catalog");
        for (key, data) in &b.tiles {
            let catalog = meshes.get(b.id).expect("mesh").catalog();
            let ready = catalog
                .prepare(catalog.tile(*key).expect("entry"), data.clone())
                .expect("prepare");
            meshes.commit(b.id, ready).expect("commit");
        }
    }
    meshes
}

fn world_floors() -> [([f32; 4], f32); 2] {
    [([0., 0., 12., 10.], 0.), ([0., 0., 12., 10.], 4.)]
}

fn crowd_params() -> CrowdWorldParams {
    let mut params = CrowdWorldParams::default();
    params.max_agents = 2;
    params.max_heavy_agents = 2;
    params.max_radius = 0.7;
    params
}

fn agent_pair() -> [AgentParams; 2] {
    let mut small = CrowdWorld::default_agent_params();
    small.radius = 0.22;
    small.max_speed = 2.;
    small.collision_query_range = 3.;
    let mut large = small;
    large.radius = 0.7;
    [small, large]
}

/// Valid profile records over the two-floor world.
pub struct ProfileFixture {
    pub baked: Vec<Baked>,
    pub meshes: ProfileMeshes,
    pub point: ProfilePoint,
    /// `NPRS`, `NPRF`, `NPCW` and `NPCM` records of the large profile.
    pub residency: Vec<u8>,
    pub reference: Vec<u8>,
    pub crowd: Vec<u8>,
    pub mixed: Vec<u8>,
}

thread_local! {
    static PROFILES: ProfileFixture = {
        let baked = bake(&world_floors());
        let meshes = install(&baked);
        let large = meshes.get(LARGE).expect("large mesh");
        let point = large
            .query(2048)
            .expect("query")
            .find_nearest_poly(&[6., 0., 5.], &[0.6, 1., 0.6], &DefaultFilter::default())
            .expect("nearest")
            .expect("a polygon");
        let residency = large.save_residency().expect("residency");
        let reference = large.save_reference(&point.polygon).expect("reference");
        let [small_agent, large_agent] = agent_pair();

        let mut world = ProfileCrowdWorld::new(large, &crowd_params(), Limits::default())
            .expect("profile crowd");
        let a = world.add_agent(large, [2., 0., 3.], large_agent).expect("agent");
        world.add_agent(large, [2., 0., 6.], small_agent).expect("agent");
        world.set_goal(large, &a, Some(&point)).expect("goal");
        let _ = world.step(large, &mut Budget::unlimited());
        let crowd = world.save(large).expect("crowd record");

        let mut mixed = MixedProfileCrowdWorld::new(
            &meshes,
            &[SMALL, LARGE],
            &crowd_params(),
            Limits::default(),
        )
        .expect("mixed crowd");
        let a = mixed.add_agent(&meshes, LARGE, [2., 0., 3.], large_agent).expect("agent");
        mixed.add_agent(&meshes, SMALL, [2., 0., 6.], small_agent).expect("agent");
        mixed.set_goal(&meshes, &a, Some(&point)).expect("goal");
        let _ = mixed.step(&meshes, &mut Budget::unlimited());
        let mixed = mixed.save(&meshes).expect("mixed record");

        ProfileFixture { baked, meshes, point, residency, reference, crowd, mixed }
    };
}

/// Runs `f` on the shared profile fixture.
pub fn with_profiles<R>(f: impl FnOnce(&ProfileFixture) -> R) -> R {
    PROFILES.with(f)
}

struct FramedProfileFixture {
    frames: FrameSet,
    bytes: Vec<u8>,
}

thread_local! {
    static FRAMED_PROFILES: FramedProfileFixture = {
        let world = bake(&world_floors());
        let cabin = bake(&[([0., 0., 6., 10.], 0.)]);
        let mut frames = FrameSet::with_profiles(install(&world));
        let deck = FrameId(1);
        let place = |y| FramePose::new([8., y, 0.], 1., 0.).expect("finite pose");
        frames
            .add_profile_frame(deck, install(&cabin), place(0.))
            .expect("profile frame");
        let at = FramePoint::new(deck, [2., 0.25, 5.]);
        let (board, exit) = (LinkId(1), LinkId(2));
        frames
            .add_dock(
                DockSpec::new(board, FramePoint::new(WORLD, [10., 0.25, 5.]), at, 0.5),
                true,
            )
            .expect("dock");
        frames
            .add_dock(
                DockSpec::new(exit, at, FramePoint::new(WORLD, [10., 4.25, 5.]), 0.5),
                false,
            )
            .expect("dock");
        let mut crowd = FramedMixedProfileCrowdWorld::default();
        for frame in [WORLD, deck] {
            crowd
                .attach(&frames, frame, &[SMALL, LARGE], &crowd_params(), Limits::default())
                .expect("attach");
        }
        crowd.set_wait_cost(&frames, exit, Some(2.)).expect("wait cost");
        let [small_agent, large_agent] = agent_pair();
        let rider = PlatformAgentId(1);
        crowd
            .add_agent(
                &frames,
                rider,
                LARGE,
                FramePoint::new(WORLD, [7., 0.25, 5.]),
                large_agent,
            )
            .expect("rider");
        crowd
            .add_agent(
                &frames,
                PlatformAgentId(2),
                SMALL,
                FramePoint::new(deck, [4., 0.25, 8.]),
                small_agent,
            )
            .expect("second rider");
        crowd
            .set_goal(&frames, rider, Some(FramePoint::new(WORLD, [7., 4.25, 5.])))
            .expect("goal");
        for _ in 0..8 {
            let _ = crowd.step(&frames, &mut Budget::unlimited());
        }
        let bytes = crowd.save(&frames).expect("framed record");
        FramedProfileFixture { frames, bytes }
    };
}

/// Runs `f` on a two-frame profile world and a valid `NPFM` record bound to it.
pub fn with_framed_profiles<R>(f: impl FnOnce(&FrameSet, &[u8]) -> R) -> R {
    FRAMED_PROFILES.with(|x| f(&x.frames, &x.bytes))
}
