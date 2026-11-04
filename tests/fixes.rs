//! Each upstream defect correction (`UpstreamFixes`) next to the default behaviour
//! that shows the defect. Defaults match the reference driver's records; the
//! corrected behaviour is opt-in.

use navmesh_rs::UpstreamFixes;

#[test]
fn flag_set_algebra() {
    let all = UpstreamFixes::SLICED_CROSS_SIDE
        | UpstreamFixes::NEAREST_NEIGHBOURS
        | UpstreamFixes::ADAPTIVE_PATTERN
        | UpstreamFixes::ZERO_SAMPLE_DIST;
    assert_eq!(all, UpstreamFixes::ALL);
    assert!(UpstreamFixes::default().is_empty());
    assert!(all.contains(UpstreamFixes::ADAPTIVE_PATTERN));
    assert!(!UpstreamFixes::NONE.contains(UpstreamFixes::ADAPTIVE_PATTERN));
    assert_eq!(UpstreamFixes::from_bits(all.bits()), Some(all));
    assert_eq!(UpstreamFixes::from_bits(1 << 31), None);
}

#[cfg(all(feature = "recast", feature = "detour"))]
mod bake_and_query {
    use super::UpstreamFixes;
    use navmesh_rs::bake::{self, BakeConfig, Input};
    use navmesh_rs::detour::{DefaultFilter, NavMesh, NavMeshQuery, PolyRef};
    use navmesh_rs::recast::{
        self, CompactHeightfield, ContourSet, Heightfield, PolyMesh, PolyMeshDetail,
    };
    use navmesh_rs::{Limits, Workspace};

    fn tiled_floor() -> NavMesh {
        // Tiles of 7.2 world units: the 40 x 40 floor spans a 6 x 6 tile grid.
        let verts = [0., 0., 0., 0., 0., 40., 40., 0., 40., 40., 0., 0.];
        let input = Input::new(&verts, &[0, 1, 2, 0, 2, 3]);
        let mut cfg = BakeConfig::default();
        cfg.recast.tile_size = 24;
        bake::tiles(&input, &cfg, &mut Workspace::new())
            .unwrap()
            .into_navmesh(Limits::default())
            .unwrap()
    }

    fn plain(
        q: &mut NavMeshQuery,
        s: PolyRef,
        e: PolyRef,
        sp: [f32; 3],
        ep: [f32; 3],
    ) -> Vec<PolyRef> {
        let mut path = Vec::new();
        q.find_path(s, e, &sp, &ep, &DefaultFilter::default(), &mut path, 512)
            .unwrap();
        path
    }

    fn sliced(
        q: &mut NavMeshQuery,
        s: PolyRef,
        e: PolyRef,
        sp: [f32; 3],
        ep: [f32; 3],
    ) -> Vec<PolyRef> {
        let f = DefaultFilter::default();
        q.init_sliced_find_path(s, e, &sp, &ep, &f, 0).unwrap();
        while q
            .update_sliced_find_path(10_000, &f)
            .unwrap()
            .0
            .is_in_progress()
        {}
        let mut path = Vec::new();
        q.finalize_sliced_find_path(&f, &mut path, 512).unwrap();
        path
    }

    /// (pairs whose plain and sliced corridors differ, pairs tried).
    fn corridor_disagreements(fixes: UpstreamFixes) -> (usize, usize) {
        let nav = tiled_floor();
        let mut q = NavMeshQuery::new(&nav, 4096).unwrap();
        q.set_fixes(fixes);
        let f = DefaultFilter::default();
        let mut pts = Vec::new();
        for i in 0..9 {
            for j in 0..9 {
                let p = [1. + i as f32 * 4.5, 0.2, 1. + j as f32 * 4.5];
                if let Ok((r, c, _)) = q.find_nearest_poly(&p, &[1., 4., 1.], &f)
                    && r != 0
                {
                    pts.push((r, c));
                }
            }
        }
        let (mut diff, mut total) = (0, 0);
        for a in pts.iter().step_by(3) {
            for b in pts.iter().skip(1).step_by(4) {
                total += 1;
                if plain(&mut q, a.0, b.0, a.1, b.1) != sliced(&mut q, a.0, b.0, a.1, b.1) {
                    diff += 1;
                }
            }
        }
        (diff, total)
    }

    #[test]
    fn sliced_search_matches_the_plain_search_with_cross_side_keys() {
        let (upstream, total) = corridor_disagreements(UpstreamFixes::NONE);
        let (fixed, _) = corridor_disagreements(UpstreamFixes::SLICED_CROSS_SIDE);
        assert!(upstream > 0, "no pair shows the defect in {total} tried");
        assert_eq!(fixed, 0, "{fixed} of {total} corridors still differ");
    }

    #[test]
    fn zero_detail_sample_distance_builds_the_detail_mesh_from_the_hull() {
        let mut ws = Workspace::new();
        let verts = [
            0.0, 0.0, 0.0, 0.0, 0.0, 10.0, 10.0, 0.0, 10.0, 10.0, 0.0, 0.0,
        ];
        let tris = [0, 1, 2, 0, 2, 3];
        let (bmin, bmax) = recast::calc_bounds(&verts);
        let (w, h) = recast::calc_grid_size(&bmin, &bmax, 0.3);
        let mut hf = Heightfield::new(&mut ws, w, h, bmin, bmax, 0.3, 0.2).unwrap();
        let mut areas = [0u8; 2];
        recast::mark_walkable_triangles(&mut ws, 45.0, &verts, &tris, &mut areas).unwrap();
        hf.rasterize_triangles(&mut ws, &verts, &tris, &areas, 4)
            .unwrap();
        let mut chf = CompactHeightfield::build(&mut ws, 10, 4, &hf).unwrap();
        recast::build_distance_field(&mut ws, &mut chf).unwrap();
        recast::build_regions(&mut ws, &mut chf, 0, 8, 20).unwrap();
        let cset =
            ContourSet::build(&mut ws, &chf, 1.3, 40, recast::CONTOUR_TESS_WALL_EDGES).unwrap();
        let pmesh = PolyMesh::build(&mut ws, &cset, 6).unwrap();
        assert!(pmesh.npolys > 0);

        let upstream = PolyMeshDetail::build(&mut ws, &pmesh, &chf, 0.0, 0.2).unwrap();
        assert!(
            upstream.meshes.chunks(4).all(|m| m[3] == 0),
            "no detail triangles"
        );

        let fixed = PolyMeshDetail::build_with_fixes(
            &mut ws,
            &pmesh,
            &chf,
            0.0,
            0.2,
            UpstreamFixes::ZERO_SAMPLE_DIST,
        )
        .unwrap();
        let nvp = pmesh.nvp as usize;
        for (i, m) in fixed.meshes.chunks(4).enumerate() {
            let corners = pmesh.polys[i * nvp * 2..i * nvp * 2 + nvp]
                .iter()
                .filter(|&&v| v != u16::MAX)
                .count() as u32;
            assert_eq!(
                (m[1], m[3]),
                (corners, corners - 2),
                "polygon {i}: hull only"
            );
        }

        // Through the bake: the corrected tile carries detail triangles, upstream's none.
        let input = Input::new(&verts, &tris);
        let mut cfg = BakeConfig::default();
        cfg.recast.detail_sample_dist = 0.0;
        let default = bake::tiles(&input, &cfg, &mut Workspace::new()).unwrap();
        cfg.fixes = UpstreamFixes::ZERO_SAMPLE_DIST;
        let corrected = bake::tiles(&input, &cfg, &mut Workspace::new()).unwrap();
        assert_ne!(default.tiles, corrected.tiles);
    }
}

#[cfg(feature = "crowd")]
mod crowd {
    use super::UpstreamFixes;
    use navmesh_rs::Workspace;
    use navmesh_rs::crowd::{
        AgentId, AgentParams, Crowd, ObstacleAvoidanceParams, ObstacleAvoidanceQuery,
    };
    use navmesh_rs::detour::{self, NavMesh, NavMeshCreateParams};

    fn floor_30x10() -> NavMesh {
        let n = 0xffff;
        let data = detour::create_nav_mesh_data(&NavMeshCreateParams {
            verts: &[0, 0, 0, 0, 0, 10, 30, 0, 10, 30, 0, 0],
            vert_count: 4,
            polys: &[0, 1, 2, 3, n, n, n, n, n, n, n, n],
            poly_count: 1,
            nvp: 6,
            poly_flags: &[1],
            poly_areas: &[0],
            bmin: [0.; 3],
            bmax: [30., 1., 10.],
            walkable_height: 2.,
            walkable_radius: 0.6,
            walkable_climb: 0.5,
            cs: 1.,
            ch: 1.,
            build_bv_tree: true,
            ..Default::default()
        })
        .unwrap();
        NavMesh::new_single(data).unwrap().0
    }

    /// Slots of the neighbours the first agent keeps: 40 agents farther away are
    /// scanned before the 6 nearest ones.
    fn neighbours_of_first(fixes: UpstreamFixes) -> (Vec<usize>, Vec<usize>) {
        let nav = floor_30x10();
        let mut ws = Workspace::new();
        let mut crowd = Crowd::new(&mut ws, &nav, 64, 0.6).unwrap();
        crowd.set_fixes(fixes);
        let mut add = |x: f32, z: f32| {
            crowd
                .add_agent(&mut ws, &nav, [x, 0., z], AgentParams::default())
                .unwrap()
        };
        let first = add(15., 6.);
        for zi in 0..4 {
            for xi in 0..10 {
                add(10.5 + xi as f32, 1.5 + zi as f32);
            }
        }
        let near: Vec<usize> = [
            (14.6, 7.),
            (15., 7.),
            (15.4, 7.),
            (14.6, 6.6),
            (15.4, 6.6),
            (15., 7.4),
        ]
        .iter()
        .map(|&(x, z)| add(x, z).0)
        .collect();
        crowd.update(&mut ws, &nav, 1. / 30.).unwrap();
        let kept = crowd
            .agent(AgentId(first.0))
            .unwrap()
            .neighbours()
            .iter()
            .map(|n| n.0)
            .collect();
        (kept, near)
    }

    #[test]
    fn crowd_keeps_the_nearest_neighbours_among_more_than_32_candidates() {
        let (upstream, near) = neighbours_of_first(UpstreamFixes::NONE);
        let (fixed, _) = neighbours_of_first(UpstreamFixes::NEAREST_NEIGHBOURS);
        let mut sorted = fixed.clone();
        sorted.sort_unstable();
        let mut expected = near.clone();
        expected.sort_unstable();
        assert_eq!(sorted, expected, "all six nearest agents are kept");
        assert!(
            near.iter().any(|n| !upstream.contains(n)),
            "upstream kept {upstream:?}, which misses none of {near:?}"
        );
    }

    fn adaptive_sample(divs: u8, fixes: UpstreamFixes) -> [f32; 3] {
        let mut ws = Workspace::new();
        let mut q = ObstacleAvoidanceQuery::new(&mut ws, 1, 1).unwrap();
        q.set_fixes(fixes);
        // A wall of one circle 1.2 ahead: forward and standing still hit it, retreating does not.
        q.add_circle([1.2, 0., 0.], 0.5, [0.; 3], [0.; 3]).unwrap();
        let mut p = ObstacleAvoidanceParams::default();
        p.adaptive_divs = divs;
        p.adaptive_rings = 1;
        p.adaptive_depth = 1;
        q.sample_velocity_adaptive(&mut ws, [0.; 3], 0.5, 2., [0.; 3], [1., 0., 0.], &p)
            .unwrap()
            .0
    }

    #[test]
    fn even_division_pattern_has_its_last_sample_only_with_the_fix() {
        // Two divisions: the pattern is the centre, the desired direction and its opposite.
        let upstream = adaptive_sample(2, UpstreamFixes::NONE);
        let fixed = adaptive_sample(2, UpstreamFixes::ADAPTIVE_PATTERN);
        assert!(
            upstream[0] >= 0.,
            "the retreat sample is missing: {upstream:?}"
        );
        assert!(
            fixed[0] < 0.,
            "retreat is the only clear velocity: {fixed:?}"
        );
        // Odd counts were never affected.
        assert_eq!(
            adaptive_sample(3, UpstreamFixes::NONE),
            adaptive_sample(3, UpstreamFixes::ADAPTIVE_PATTERN)
        );
    }
}
