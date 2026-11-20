#![cfg(feature = "dynamic")]

use navmesh_rs::detour::{
    DefaultFilter, NavMesh, NavMeshCreateParams, NavMeshParams, create_nav_mesh_data_f32,
    prepare_tile,
};
use navmesh_rs::dynamic::{DynamicLayer, Effect, ModifierId, ModifierSpec, Selector};
use navmesh_rs::{Budget, Error, Limits, SnapshotError};

const GATE: ModifierId = ModifierId(0);
const ZONE: ModifierId = ModifierId(1);
const OUTLINE: [[f32; 2]; 4] = [[0.45, 0.45], [0.45, 0.55], [0.55, 0.55], [0.55, 0.45]];

fn detail_prism(ymin: f32, ymax: f32) -> Selector {
    Selector::DetailPrism {
        verts_xz: OUTLINE.to_vec(),
        ymin,
        ymax,
        area: Some(1),
    }
}

fn tile(layer: i32, raised: bool) -> Vec<u8> {
    tile_variant(layer, raised, 0)
}

fn tile_variant(layer: i32, raised: bool, user_id: u32) -> Vec<u8> {
    let verts = [0., 0., 0., 0., 0., 1., 1., 0., 1., 1., 0., 0.];
    let mut detail = verts.to_vec();
    detail.extend_from_slice(&[0.5, 1., 0.5]);
    let mut params = NavMeshCreateParams {
        vert_count: 4,
        polys: &[0, 1, 2, 3, 0xffff, 0xffff, 0xffff, 0xffff],
        poly_flags: &[1],
        poly_areas: &[1],
        poly_count: 1,
        nvp: 4,
        bmin: [0., 0., 0.],
        bmax: [1., 1., 1.],
        cs: 1.,
        ch: 1.,
        tile_layer: layer,
        user_id,
        ..Default::default()
    };
    if raised {
        params.detail_meshes = &[0, 5, 0, 4];
        params.detail_verts = &detail;
        params.detail_verts_count = 5;
        params.detail_tris = &[0, 1, 4, 1, 1, 2, 4, 1, 2, 3, 4, 1, 3, 0, 4, 1];
        params.detail_tri_count = 4;
    }
    create_nav_mesh_data_f32(&params, &verts).unwrap()
}

fn mesh() -> NavMesh {
    NavMesh::new(&NavMeshParams {
        orig: [0.; 3],
        tile_width: 1.,
        tile_height: 1.,
        max_tiles: 2,
        max_polys: 1,
    })
    .unwrap()
}

fn closed_gate(selector: Selector) -> DynamicLayer {
    let mut layer = DynamicLayer::new(2, Limits::default()).unwrap();
    layer
        .upsert(ModifierSpec::new(
            GATE,
            selector,
            Effect::ClearFlags(u16::MAX),
        ))
        .unwrap();
    layer.set_enabled(GATE, true).unwrap();
    layer
}

#[test]
fn detail_prism_blocks_the_raised_surface_above_a_flat_perimeter() {
    let mut nav = mesh();
    nav.set_dynamic_layer(Some(closed_gate(Selector::Prism {
        verts_xz: OUTLINE.to_vec(),
        ymin: 0.75,
        ymax: 1.25,
        area: Some(1),
    })));
    let poly = nav.add_tile(tile(0, true)).unwrap();
    assert_eq!(nav.modifier_polygon_count(GATE), Some(0));
    assert_eq!(nav.poly_flags(poly), Ok(1));

    // Register before installing, as a streamed gate would be.
    let mut nav = mesh();
    nav.set_dynamic_layer(Some(closed_gate(detail_prism(0.75, 1.25))));
    let poly = nav.add_tile(tile(0, true)).unwrap();
    assert_eq!(nav.modifier_selection(GATE).unwrap().selected_polygons, 1);
    assert_eq!(nav.poly_flags(poly), Ok(0));
    let reached = nav
        .reachable_filtered(poly, &DefaultFilter::default(), &mut Budget::unlimited())
        .unwrap()
        .into_value()
        .unwrap();
    assert!(reached.is_empty());
}

#[test]
fn detail_prism_leaves_the_lower_layer_open() {
    let mut nav = mesh();
    nav.set_dynamic_layer(Some(closed_gate(detail_prism(0.75, 2.0))));
    let lower = nav.add_tile(tile(0, false)).unwrap();
    let upper = nav.add_tile(tile(1, true)).unwrap();
    assert_eq!(nav.modifier_polygon_count(GATE), Some(1));
    assert_eq!(nav.poly_flags(lower), Ok(1));
    assert_eq!(nav.poly_flags(upper), Ok(0));
}

#[test]
fn detail_prism_tests_surface_contact_in_both_windings() {
    let mut nav = mesh();
    nav.set_dynamic_layer(Some(closed_gate(detail_prism(0.1, 0.2))));
    let poly = nav.add_tile(tile(0, true)).unwrap();
    // The sloped triangles have overlapping bounds but are above this box.
    assert_eq!(nav.poly_flags(poly), Ok(1));

    let apex_corner = [[0.5, 0.5], [0.6, 0.5], [0.6, 0.6], [0.5, 0.6]];
    for outline in [
        apex_corner.to_vec(),
        apex_corner.into_iter().rev().collect(),
    ] {
        for (height, flags) in [(1.0, 0), (f32::from_bits(1.0f32.to_bits() + 1), 1)] {
            nav.upsert_modifier(ModifierSpec::new(
                GATE,
                Selector::DetailPrism {
                    verts_xz: outline.clone(),
                    ymin: height,
                    ymax: height,
                    area: None,
                },
                Effect::ClearFlags(u16::MAX),
            ))
            .unwrap();
            assert_eq!(nav.poly_flags(poly), Ok(flags));
        }
    }
}

#[test]
fn replacement_reselects_the_surface_and_reload_uses_the_current_gate_bit() {
    let mut nav = mesh();
    nav.set_dynamic_layer(Some(closed_gate(detail_prism(0.75, 1.25))));
    let prepare =
        |nav: &NavMesh, raised| prepare_tile(nav.params(), nav.limits(), tile(0, raised)).unwrap();
    let flat = nav.commit(prepare(&nav, false)).unwrap();
    assert_eq!(nav.poly_flags(flat), Ok(1));
    let raised = nav
        .replace_tile(prepare(&nav, true))
        .unwrap()
        .tile_ref()
        .unwrap();
    assert_eq!(nav.poly_flags(raised), Ok(0));
    nav.set_modifier_enabled(GATE, false).unwrap();
    nav.set_poly_flags(raised, 3).unwrap();
    assert_eq!(nav.poly_flags(raised), Ok(3));
    let parked = nav.unload_tile(raised).unwrap();
    nav.set_modifier_enabled(GATE, true).unwrap();
    assert_eq!(
        nav.commit_with_state(prepare(&nav, true), &parked.state),
        Ok(raised)
    );
    assert_eq!(nav.poly_flags(raised), Ok(0));
    let flat = nav
        .replace_tile(prepare(&nav, false))
        .unwrap()
        .tile_ref()
        .unwrap();
    assert_eq!(nav.poly_flags(flat), Ok(1));
    let raised_again = nav
        .replace_tile(prepare(&nav, true))
        .unwrap()
        .tile_ref()
        .unwrap();
    assert_eq!(nav.poly_flags(raised_again), Ok(0));
    nav.set_modifier_enabled(GATE, false).unwrap();
    assert_eq!(nav.poly_flags(raised_again), Ok(1));
}

#[test]
fn base_edits_are_folded_under_active_effects_and_saved_with_the_layer() {
    let mut nav = mesh();
    let mut layer = closed_gate(detail_prism(0.75, 1.25));
    layer
        .upsert(ModifierSpec::new(
            ZONE,
            detail_prism(0.75, 1.25),
            Effect::SetArea(9),
        ))
        .unwrap();
    layer.set_enabled(GATE, false).unwrap();
    nav.set_dynamic_layer(Some(layer));
    let bytes = tile(0, true);
    let poly = nav.add_tile(bytes.clone()).unwrap();
    nav.set_poly_flags(poly, 0x8021).unwrap();
    nav.set_poly_area(poly, 79).unwrap();
    nav.set_modifier_enabled(GATE, true).unwrap();
    nav.set_modifier_enabled(ZONE, true).unwrap();
    assert_eq!(nav.poly_flags(poly), Ok(0));
    assert_eq!(nav.poly_area(poly), Ok(9));
    assert_eq!(nav.baked_poly_state(poly), Ok((1, 1)));

    let saved = nav.dynamic_layer().unwrap().to_bytes();
    assert_eq!(&saved[..8], b"DNDY\x02\0\0\0");
    let layer = DynamicLayer::from_bytes(&saved, Limits::UNLIMITED).unwrap();
    assert_eq!(layer.to_bytes(), saved);
    assert_eq!(
        layer.get(GATE).unwrap().0.selector,
        detail_prism(0.75, 1.25)
    );
    let mut restored = mesh();
    let poly = restored.add_tile(bytes).unwrap();
    restored.set_dynamic_layer(Some(layer));
    assert_eq!(restored.poly_flags(poly), Ok(0));
    assert_eq!(restored.poly_area(poly), Ok(9));
    restored.set_modifier_enabled(GATE, false).unwrap();
    restored.set_modifier_enabled(ZONE, false).unwrap();
    assert_eq!(restored.poly_flags(poly), Ok(0x8021));
    assert_eq!(restored.poly_area(poly), Ok(15));
    restored.set_modifier_enabled(GATE, true).unwrap();
    assert_eq!(restored.poly_flags(poly), Ok(0));
    assert!(restored.remove_modifier(GATE));
    assert!(restored.remove_modifier(ZONE));
    let parked = restored.unload_tile(poly).unwrap();
    let prepared = prepare_tile(restored.params(), restored.limits(), parked.bytes).unwrap();
    assert_eq!(restored.commit(prepared), Ok(poly));
    assert_eq!(restored.poly_flags(poly), Ok(0x8021));
    assert_eq!(restored.poly_area(poly), Ok(15));

    let mut unknown = saved;
    unknown[4] = 3;
    assert!(matches!(
        DynamicLayer::from_bytes(&unknown, Limits::UNLIMITED),
        Err(Error::InvalidSnapshot(SnapshotError::WrongVersion))
    ));
}

#[test]
fn effective_read_modify_write_preserves_owned_base_fields() {
    let mut layer = DynamicLayer::new(2, Limits::UNLIMITED).unwrap();
    let selector = Selector::Prism {
        verts_xz: OUTLINE.to_vec(),
        ymin: 0.0,
        ymax: 1.0,
        area: None,
    };
    for (id, effect) in [(GATE, Effect::ClearFlags(1)), (ZONE, Effect::SetArea(9))] {
        layer
            .upsert(ModifierSpec::new(id, selector.clone(), effect))
            .unwrap();
        layer.set_enabled(id, true).unwrap();
    }
    let v1 = layer.to_bytes();
    assert_eq!(&v1[..8], b"DNDY\x01\0\0\0");
    let mut nav = mesh();
    nav.set_dynamic_layer(Some(
        DynamicLayer::from_bytes(&v1, Limits::UNLIMITED).unwrap(),
    ));
    let poly = nav.add_tile(tile(0, true)).unwrap();
    nav.set_poly_flags(poly, nav.poly_flags(poly).unwrap() | 4)
        .unwrap();
    nav.set_poly_area(poly, nav.poly_area(poly).unwrap())
        .unwrap();
    nav.set_poly_area(poly, 13).unwrap();
    assert_eq!(nav.poly_flags(poly), Ok(4));
    assert_eq!(nav.poly_area(poly), Ok(9));
    assert_eq!(nav.poly_base_flags(poly), Ok(5));
    assert_eq!(nav.poly_base_area(poly), Ok(1));
    let v2 = nav.dynamic_layer().unwrap().to_bytes();
    assert_eq!(&v2[..8], b"DNDY\x02\0\0\0");
    nav.set_modifier_enabled(GATE, false).unwrap();
    nav.set_modifier_enabled(ZONE, false).unwrap();
    assert_eq!(nav.poly_flags(poly), Ok(5));
    assert_eq!(nav.poly_area(poly), Ok(1));
}

#[test]
fn newly_selected_polygons_keep_prior_direct_writes() {
    for path in 0..3 {
        let mut nav = mesh();
        let poly = nav.add_tile(tile(0, true)).unwrap();
        nav.set_poly_flags(poly, 5).unwrap();
        // Exercise tile-state restore as another way to edit an unmanaged polygon.
        nav.set_poly_area(poly, 7).unwrap();
        let mut state = vec![0; nav.tile_state_size(poly).unwrap()];
        nav.store_tile_state(poly, &mut state).unwrap();
        nav.set_poly_area(poly, 1).unwrap();
        nav.restore_tile_state(poly, &state).unwrap();
        let selector = detail_prism(0.75, 1.25);
        match path {
            0 => {
                nav.set_dynamic_layer(Some(DynamicLayer::new(2, Limits::UNLIMITED).unwrap()));
                nav.upsert_modifier(ModifierSpec::new(
                    GATE,
                    selector,
                    Effect::ClearFlags(u16::MAX),
                ))
                .unwrap();
                nav.set_modifier_enabled(GATE, true).unwrap();
            }
            1 => nav.set_dynamic_layer(Some(closed_gate(selector))),
            _ => {
                let parked = nav.unload_tile(poly).unwrap();
                nav.set_dynamic_layer(Some(closed_gate(selector)));
                let prepared = prepare_tile(nav.params(), nav.limits(), parked.bytes).unwrap();
                assert_eq!(nav.commit_with_state(prepared, &parked.state), Ok(poly));
            }
        }
        assert_eq!(nav.poly_flags(poly), Ok(0));
        assert_eq!(nav.poly_base_flags(poly), Ok(5));
        assert_eq!(nav.poly_base_area(poly), Ok(7));
        nav.set_modifier_enabled(GATE, false).unwrap();
        assert_eq!(nav.poly_flags(poly), Ok(5));
        assert_eq!(nav.poly_area(poly), Ok(7));
        assert_eq!(nav.dynamic_layer().unwrap().poly_base_edits().count(), 1);
    }
}

#[test]
fn replacing_or_removing_content_reclaims_the_base_edit_budget() {
    let mut nav = mesh();
    let limits = Limits {
        max_bytes: 180,
        ..Limits::UNLIMITED
    };
    nav.set_dynamic_layer(Some(DynamicLayer::new(1, limits).unwrap()));
    nav.upsert_modifier(ModifierSpec::new(
        GATE,
        detail_prism(0.75, 1.25),
        Effect::ClearFlags(1),
    ))
    .unwrap();
    let mut poly = nav.add_tile(tile(0, true)).unwrap();
    for variant in 1..=8 {
        nav.set_poly_flags(poly, 5).unwrap();
        let layer = nav.dynamic_layer().unwrap();
        assert_eq!(layer.poly_base_edits().count(), 1);
        assert!(layer.to_bytes().len() <= limits.max_bytes);
        let data = tile_variant(0, true, variant);
        if variant % 2 == 0 {
            nav.remove_tile(poly).unwrap();
            poly = nav.add_tile(data).unwrap();
        } else {
            let prepared = prepare_tile(nav.params(), nav.limits(), data).unwrap();
            poly = nav.replace_tile(prepared).unwrap().tile_ref().unwrap();
        }
        assert_eq!(nav.poly_base_flags(poly), Ok(1));
        assert_eq!(nav.dynamic_layer().unwrap().poly_base_edits().count(), 0);
    }
    nav.set_poly_flags(poly, 5).unwrap();
    let edit = nav
        .dynamic_layer()
        .unwrap()
        .poly_base_edits()
        .next()
        .unwrap();
    let parked = nav.unload_tile(poly).unwrap();
    let prepared = prepare_tile(nav.params(), nav.limits(), parked.bytes).unwrap();
    assert_eq!(nav.commit(prepared), Ok(poly));
    assert_eq!(nav.poly_base_flags(poly), Ok(5));
    nav.set_modifier_enabled(GATE, true).unwrap();
    assert!(nav.remove_poly_base_edit(&edit));
    assert!(!nav.remove_poly_base_edit(&edit));
    assert_eq!(nav.poly_base_flags(poly), Ok(1));
    assert_eq!(nav.poly_flags(poly), Ok(0));
}
