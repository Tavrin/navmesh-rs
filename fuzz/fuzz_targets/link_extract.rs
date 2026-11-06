#![no_main]
use libfuzzer_sys::fuzz_target;
use navmesh_rs::bake::{self, BakeConfig, Input, LinkExtraction, LinkLimits};
use navmesh_rs::{Limits, Workspace};
fuzz_target!(|data: &[u8]| {
    if data.len() < 6 {
        return;
    }
    let y = data[0] as f32 / 32.0;
    let gap = (data[1] % 8) as f32 / 2.0;
    let verts = [
        0.,
        y,
        0.,
        0.,
        y,
        8.,
        8.,
        y,
        8.,
        8.,
        y,
        0.,
        8. + gap,
        0.,
        0.,
        8. + gap,
        0.,
        8.,
        20.,
        0.,
        8.,
        20.,
        0.,
        0.,
    ];
    let tris = [0, 1, 2, 0, 2, 3, 4, 5, 6, 4, 6, 7];
    let input = Input::new(&verts, &tris);
    let mut cfg = BakeConfig::default();
    cfg.recast.cs = 0.5;
    cfg.recast.ch = 0.1;
    cfg.recast.tile_size = 16;
    cfg.recast.min_region_area = 0;
    cfg.recast.merge_region_area = 0;
    cfg.tile_limits = Limits {
        max_bytes: 16 << 20,
        max_work: 2_000_000,
        ..Limits::default()
    };
    let limits = LinkLimits {
        distance: (data[2] % 16) as f32 / 2.,
        up: (data[3] % 16) as f32 / 2.,
        down: (data[4] % 16) as f32 / 2.,
        clearance: 1.0,
    };
    cfg.links = Some(LinkExtraction {
        drop: Some(limits),
        jump: Some(limits),
        climb: Some(limits),
        spacing_cells: (data[5] % 8 + 1) as u16,
        area_ids: [1, 2, 3],
        flags: [1, 2, 4],
    });
    if let Ok(serial) = bake::tiles(&input, &cfg, &mut Workspace::new()) {
        if let Ok(repeat) = bake::tiles_parallel(&input, &cfg, &mut Workspace::new()) {
            assert_eq!(serial.tiles, repeat.tiles);
        }
        for l in serial.tiles.iter().flat_map(|t| &t.links) {
            assert!(l.start.iter().chain(&l.end).all(|v| v.is_finite()));
        }
    }
});
