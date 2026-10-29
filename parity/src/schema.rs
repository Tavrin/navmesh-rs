//! Field layouts of every record kind, as `reference/harness/nav_oracle.cpp`
//! writes them and `reference/RECORDS.md` documents them.
//!
//! A record's kind is the part of its name after the last `.`: `tile_1_2.contours`
//! and `contours` are both `contours`, `filtered.q_find_path` is `q_find_path`.

use crate::decode::{Cursor, DecodeResult};

/// Every record kind the decoders know, in the order the driver writes them.
/// `crowd_*` kinds share one layout; `crowd_quality_<q>` covers q = 0..3.
pub const KINDS: &[&str] = &[
    "geometry",
    "config",
    "tri_areas",
    "heightfield_initial",
    "heightfield_filtered",
    "compact_initial",
    "compact_eroded",
    "compact_marked",
    "distance_field",
    "regions",
    "contours",
    "poly_mesh",
    "detail_mesh",
    "heightfield_layers",
    "layer_data",
    "layer_grid",
    "layer_regions",
    "layer_contours",
    "layer_poly_mesh",
    "navmesh_tiles",
    "q_random_point",
    "q_nearest_poly",
    "q_query_polygons",
    "q_find_path",
    "q_straight_path",
    "q_straight_path_all_crossings",
    "q_straight_path_area_crossings",
    "q_sliced_path",
    "q_partial_sliced_path",
    "q_raycast",
    "q_raycast_plain",
    "q_distance_to_wall",
    "q_move_along_surface",
    "q_polys_around_circle",
    "q_polys_around_shape",
    "q_local_neighbourhood",
    "q_poly_wall_segments",
    "q_closest_point_on_poly",
    "q_closest_point_on_poly_boundary",
    "q_poly_height",
    "q_random_point_around_circle",
    "crowd_trajectories",
    "crowd_quality_0",
    "crowd_quality_1",
    "crowd_quality_2",
    "crowd_quality_3",
    "crowd_move_velocity",
    "crowd_remove_readd",
    "crowd_tile_swap",
    "malformed_tiles",
    "tilecache_obstacles",
];

/// The kind of a record name: the part after the last `.`.
pub fn kind_of(name: &str) -> &str {
    name.rsplit('.').next().unwrap_or(name)
}

/// Record kinds produced by the Recast stages (input handling through the detail
/// mesh, and the heightfield layers). `nav-parity diff --recast` compares only these.
pub const RECAST_KINDS: &[&str] = &[
    "geometry",
    "config",
    "tri_areas",
    "heightfield_initial",
    "heightfield_filtered",
    "compact_initial",
    "compact_eroded",
    "compact_marked",
    "distance_field",
    "regions",
    "contours",
    "poly_mesh",
    "detail_mesh",
    "heightfield_layers",
];

/// Whether a kind is one of [`RECAST_KINDS`].
pub fn is_recast(kind: &str) -> bool {
    RECAST_KINDS.contains(&kind)
}

/// Whether a kind belongs to the Recast + Detour phases: the Recast stages, the
/// navmesh tiles, the malformed-tile verdicts and every query family (not the
/// crowd, tile-cache layer or obstacle records).
pub fn is_detour_phase(kind: &str) -> bool {
    is_recast(kind)
        || kind == "navmesh_tiles"
        || kind == "malformed_tiles"
        || kind.starts_with("q_")
}

/// Whether a kind is a crowd trajectory (the only kind with a tolerance rule).
pub fn is_crowd(kind: &str) -> bool {
    kind.starts_with("crowd_")
}

/// The record type used in reports: the name without its per-tile or per-layer
/// prefix (`tile_X_Y.`, `layer_X_Y_L.`). The `filtered.` prefix is kept, because
/// filtered queries are a separate family.
pub fn type_of(name: &str) -> &str {
    if let Some((first, rest)) = name.split_once('.') {
        let indexed = |p: &str| {
            first.strip_prefix(p).is_some_and(|t| {
                !t.is_empty()
                    && t.bytes()
                        .all(|b| b.is_ascii_digit() || b == b'_' || b == b'-')
            })
        };
        if indexed("tile_") || indexed("layer_") {
            return rest;
        }
    }
    name
}

/// Decodes a whole payload of the given kind; fails if the layout does not
/// match or bytes are left over.
pub fn decode(kind: &str, c: &mut Cursor) -> DecodeResult<()> {
    match kind {
        "geometry" => geometry(c)?,
        "config" => config(c)?,
        "tri_areas" => {
            let n = c.count_u32("count", 1)?;
            c.array("areas", n, |c, _| c.u8("").map(drop))?;
        }
        "heightfield_initial" | "heightfield_filtered" => heightfield(c)?,
        "compact_initial" | "compact_eroded" | "compact_marked" => compact(c, false, false)?,
        "distance_field" => compact(c, true, false)?,
        "regions" => compact(c, false, true)?,
        "contours" => contours(c)?,
        "poly_mesh" => poly_mesh(c)?,
        "detail_mesh" => detail_mesh(c)?,
        "heightfield_layers" => heightfield_layers(c)?,
        "layer_data" => layer_data(c)?,
        "layer_grid" => layer_grid(c)?,
        "layer_regions" => {
            let n = c.count_i32("cells", 1)?;
            c.u8("reg_count")?;
            c.array("regs", n, |c, _| c.u8("").map(drop))?;
        }
        "layer_contours" => layer_contours(c)?,
        "layer_poly_mesh" => layer_poly_mesh(c)?,
        "navmesh_tiles" => navmesh_tiles(c)?,
        "q_random_point" | "q_random_point_around_circle" => per_entry(c, 20, |c| {
            c.u32("status")?;
            c.poly_ref("ref")?;
            c.v3("pos").map(drop)
        })?,
        "q_nearest_poly" => per_entry(c, 21, |c| {
            c.u32("status")?;
            c.poly_ref("ref")?;
            c.v3("nearest")?;
            c.u8("over_poly").map(drop)
        })?,
        "q_query_polygons" | "q_find_path" | "q_sliced_path" => per_entry(c, 8, |c| {
            c.u32("status")?;
            refs(c, "path")
        })?,
        "q_partial_sliced_path" => per_entry(c, 20, |c| {
            c.u32("init_status")?;
            c.u32("update_status")?;
            c.i32("iterations")?;
            c.u32("status")?;
            refs(c, "path")
        })?,
        "q_straight_path" | "q_straight_path_all_crossings" | "q_straight_path_area_crossings" => {
            per_entry(c, 8, |c| {
                c.u32("status")?;
                let n = c.count_i32("count", 17)?;
                c.array("corners", n, |c, _| {
                    c.v3("pos")?;
                    c.u8("flags")?;
                    c.poly_ref("ref").map(drop)
                })
            })?
        }
        "q_raycast" => per_entry(c, 28, |c| {
            c.u32("status")?;
            c.f32("t")?;
            c.v3("normal")?;
            c.i32("edge")?;
            refs(c, "path")?;
            c.f32("cost").map(drop)
        })?,
        "q_raycast_plain" => per_entry(c, 24, |c| {
            c.u32("status")?;
            c.f32("t")?;
            c.v3("normal")?;
            refs(c, "path")
        })?,
        "q_distance_to_wall" => per_entry(c, 32, |c| {
            c.u32("status")?;
            c.f32("distance")?;
            c.v3("hit")?;
            c.v3("normal").map(drop)
        })?,
        "q_move_along_surface" => per_entry(c, 20, |c| {
            c.u32("status")?;
            c.v3("result")?;
            refs(c, "visited")
        })?,
        "q_polys_around_circle" | "q_polys_around_shape" => per_entry(c, 8, |c| {
            c.u32("status")?;
            let n = c.count_i32("count", 12)?;
            c.array("polys", n, |c, _| {
                c.poly_ref("ref")?;
                c.poly_ref("parent")?;
                c.f32("cost").map(drop)
            })
        })?,
        "q_local_neighbourhood" => per_entry(c, 8, |c| {
            c.u32("status")?;
            let n = c.count_i32("count", 8)?;
            c.array("polys", n, |c, _| {
                c.poly_ref("ref")?;
                c.poly_ref("parent").map(drop)
            })
        })?,
        "q_poly_wall_segments" => per_entry(c, 8, |c| {
            c.u32("status")?;
            let n = c.count_i32("count", 28)?;
            c.array("segments", n, |c, _| {
                c.v3("a")?;
                c.v3("b")?;
                c.poly_ref("ref").map(drop)
            })
        })?,
        "q_closest_point_on_poly" => per_entry(c, 17, |c| {
            c.u32("status")?;
            c.v3("point")?;
            c.u8("over_poly").map(drop)
        })?,
        "q_closest_point_on_poly_boundary" => per_entry(c, 16, |c| {
            c.u32("status")?;
            c.v3("point").map(drop)
        })?,
        "q_poly_height" => per_entry(c, 16, |c| {
            c.u32("status_start")?;
            c.f32("height_start")?;
            c.u32("status_end")?;
            c.f32("height_end").map(drop)
        })?,
        "malformed_tiles" => {
            let n = c.count_u32("count", 12)?;
            c.array("verdicts", n, |c, _| {
                // Case 7 removes a tile twice and holds both statuses; the
                // others hold a status and a tile ref.
                let case = c.u32("case")?;
                c.u32("status")?;
                if case == 7 {
                    c.u32("second_status").map(drop)
                } else {
                    c.poly_ref("ref").map(drop)
                }
            })?;
        }
        "tilecache_obstacles" => obstacles(c)?,
        k if is_crowd(k) => crowd(c)?,
        _ => return Err(c.error("kind", format!("unknown record kind '{kind}'"))),
    }
    c.finish()
}

/// `u32` entry count, then `count` entries of at least `min` bytes.
fn per_entry(
    c: &mut Cursor,
    min: usize,
    mut f: impl FnMut(&mut Cursor) -> DecodeResult<()>,
) -> DecodeResult<()> {
    let n = c.count_u32("count", min)?;
    c.array("entries", n, |c, _| f(c))
}

/// `i32` count, then that many poly refs.
fn refs(c: &mut Cursor, name: &'static str) -> DecodeResult<()> {
    let n = c.count_i32("count", 4)?;
    c.array(name, n, |c, _| c.poly_ref("").map(drop))
}

fn dims(c: &mut Cursor, w: i32, h: i32, per_cell: usize) -> DecodeResult<usize> {
    let cells = i64::from(w) * i64::from(h);
    if w < 0 || h < 0 {
        return Err(c.error("width", format!("negative grid size {w} x {h}")));
    }
    c.check_count("cells", cells, per_cell)
}

fn bounds(c: &mut Cursor) -> DecodeResult<()> {
    c.v3("bmin")?;
    c.v3("bmax")?;
    c.f32("cs")?;
    c.f32("ch").map(drop)
}

fn geometry(c: &mut Cursor) -> DecodeResult<()> {
    let nv = c.count_u32("vert_count", 12)?;
    c.array("verts", nv, |c, _| {
        c.f32("x")?;
        c.f32("y")?;
        c.f32("z").map(drop)
    })?;
    let nt = c.count_u32("tri_count", 12)?;
    c.array("tris", nt, |c, _| {
        c.i32("a")?;
        c.i32("b")?;
        c.i32("c").map(drop)
    })
}

fn config(c: &mut Cursor) -> DecodeResult<()> {
    for name in ["width", "height", "tile_size", "border_size"] {
        c.i32(name)?;
    }
    c.f32("cs")?;
    c.f32("ch")?;
    c.v3("bmin")?;
    c.v3("bmax")?;
    c.f32("walkable_slope_angle")?;
    for name in [
        "walkable_height",
        "walkable_climb",
        "walkable_radius",
        "max_edge_len",
    ] {
        c.i32(name)?;
    }
    c.f32("max_simplification_error")?;
    for name in ["min_region_area", "merge_region_area", "max_verts_per_poly"] {
        c.i32(name)?;
    }
    c.f32("detail_sample_dist")?;
    c.f32("detail_sample_max_error").map(drop)
}

fn heightfield(c: &mut Cursor) -> DecodeResult<()> {
    let w = c.i32("width")?;
    let h = c.i32("height")?;
    bounds(c)?;
    let cells = dims(c, w, h, 4)?;
    c.array("columns", cells, |c, _| {
        let n = c.count_u32("span_count", 5)?;
        c.array("spans", n, |c, _| {
            c.u16("smin")?;
            c.u16("smax")?;
            c.u8("area").map(drop)
        })
    })
}

fn compact(c: &mut Cursor, with_dist: bool, with_regions: bool) -> DecodeResult<()> {
    let w = c.i32("width")?;
    let h = c.i32("height")?;
    let spans = c.i32("span_count")?;
    c.i32("walkable_height")?;
    c.i32("walkable_climb")?;
    c.i32("border_size")?;
    bounds(c)?;
    let cells = dims(c, w, h, 5)?;
    c.array("cells", cells, |c, _| {
        c.u32("index")?;
        c.u8("count").map(drop)
    })?;
    let span_size = 8 + if with_dist { 2 } else { 0 } + if with_regions { 2 } else { 0 };
    let spans = c.check_count("span_count", i64::from(spans), span_size)?;
    c.array("spans", spans, |c, _| {
        c.u16("y")?;
        c.u32("con")?;
        c.u8("h")?;
        c.u8("area")?;
        if with_regions {
            c.u16("reg")?;
        }
        if with_dist {
            c.u16("dist")?;
        }
        Ok(())
    })?;
    if with_dist {
        c.u16("max_distance")?;
    }
    if with_regions {
        c.u16("max_regions")?;
    }
    Ok(())
}

fn contours(c: &mut Cursor) -> DecodeResult<()> {
    let n = c.i32("contour_count")?;
    bounds(c)?;
    c.i32("width")?;
    c.i32("height")?;
    c.i32("border_size")?;
    c.f32("max_error")?;
    let n = c.check_count("contour_count", i64::from(n), 11)?;
    c.array("contours", n, |c, _| {
        c.u16("reg")?;
        c.u8("area")?;
        for name in ["verts", "raw_verts"] {
            let nv = c.count_i32("count", 16)?;
            c.array(name, nv, |c, _| {
                c.i32("x")?;
                c.i32("y")?;
                c.i32("z")?;
                c.i32("flags").map(drop)
            })?;
        }
        Ok(())
    })
}

fn poly_mesh(c: &mut Cursor) -> DecodeResult<()> {
    let nv = c.i32("vert_count")?;
    let np = c.i32("poly_count")?;
    let nvp = c.i32("nvp")?;
    bounds(c)?;
    c.i32("border_size")?;
    c.f32("max_edge_error")?;
    let nv = c.check_count("vert_count", i64::from(nv), 6)?;
    c.array("verts", nv, |c, _| {
        c.u16("x")?;
        c.u16("y")?;
        c.u16("z").map(drop)
    })?;
    let slots = poly_slots(c, nvp)?;
    let np = c.check_count("poly_count", i64::from(np), slots * 2 + 5)?;
    c.array("polys", np, |c, _| {
        c.array("slots", slots, |c, _| c.u16("").map(drop))?;
        c.u16("reg")?;
        c.u16("flags")?;
        c.u8("area").map(drop)
    })
}

/// Vertex plus neighbour slots per polygon (`2 * nvp`); Recast allows at most
/// 255 vertices per polygon.
fn poly_slots(c: &Cursor, nvp: i32) -> DecodeResult<usize> {
    match usize::try_from(nvp) {
        Ok(n) if (1..=255).contains(&n) => Ok(2 * n),
        _ => Err(c.error("nvp", format!("vertices per polygon {nvp} outside 1..255"))),
    }
}

fn detail_mesh(c: &mut Cursor) -> DecodeResult<()> {
    let nm = c.i32("mesh_count")?;
    let nv = c.i32("vert_count")?;
    let nt = c.i32("tri_count")?;
    let nm = c.check_count("mesh_count", i64::from(nm), 16)?;
    c.array("meshes", nm, |c, _| {
        c.u32("vert_base")?;
        c.u32("vert_count")?;
        c.u32("tri_base")?;
        c.u32("tri_count").map(drop)
    })?;
    let nv = c.check_count("vert_count", i64::from(nv), 12)?;
    c.array("verts", nv, |c, _| c.v3("").map(drop))?;
    let nt = c.check_count("tri_count", i64::from(nt), 4)?;
    c.array("tris", nt, |c, _| {
        c.u8("a")?;
        c.u8("b")?;
        c.u8("c")?;
        c.u8("flags").map(drop)
    })
}

fn grid_bytes(c: &mut Cursor, cells: usize) -> DecodeResult<()> {
    c.array("heights", cells, |c, _| c.u8("").map(drop))?;
    c.array("areas", cells, |c, _| c.u8("").map(drop))?;
    c.array("cons", cells, |c, _| c.u8("").map(drop))
}

fn heightfield_layers(c: &mut Cursor) -> DecodeResult<()> {
    let n = c.count_i32("layer_count", 64)?;
    c.array("layers", n, |c, _| {
        bounds(c)?;
        let w = c.i32("width")?;
        let h = c.i32("height")?;
        for name in ["minx", "maxx", "miny", "maxy", "hmin", "hmax"] {
            c.i32(name)?;
        }
        let cells = dims(c, w, h, 3)?;
        grid_bytes(c, cells)
    })
}

/// `dtTileCacheLayerHeader` fields as the driver streams them (no padding).
fn layer_header(c: &mut Cursor) -> DecodeResult<(u8, u8)> {
    for name in ["magic", "version", "tx", "ty", "tlayer"] {
        c.i32(name)?;
    }
    c.v3("bmin")?;
    c.v3("bmax")?;
    c.u16("hmin")?;
    c.u16("hmax")?;
    let w = c.u8("width")?;
    let h = c.u8("height")?;
    for name in ["minx", "maxx", "miny", "maxy"] {
        c.u8(name)?;
    }
    Ok((w, h))
}

/// Tile-cache layer bytes as stored in the cache: the header struct (54 bytes
/// plus 2 padding bytes), then the uncompressed grid (the driver's raw
/// compressor): heights, areas, connections.
fn layer_data(c: &mut Cursor) -> DecodeResult<()> {
    let len = c.count_u32("len", 1)?;
    let start = c.pos();
    let (w, h) = c.group("header", |c| {
        let wh = layer_header(c)?;
        c.bytes("pad", 2)?;
        Ok(wh)
    })?;
    let cells = usize::from(w) * usize::from(h);
    if len != 56 + 3 * cells {
        return Err(c.error(
            "len",
            format!("layer data of {len} bytes is not a raw {w} x {h} layer"),
        ));
    }
    grid_bytes(c, cells)?;
    debug_assert_eq!(c.pos() - start, len);
    Ok(())
}

fn layer_grid(c: &mut Cursor) -> DecodeResult<()> {
    let (w, h) = c.group("header", layer_header)?;
    grid_bytes(c, usize::from(w) * usize::from(h))
}

fn layer_contours(c: &mut Cursor) -> DecodeResult<()> {
    let n = c.count_i32("contour_count", 6)?;
    c.array("contours", n, |c, _| {
        c.u8("reg")?;
        c.u8("area")?;
        let nv = c.count_i32("count", 4)?;
        c.array("verts", nv, |c, _| {
            c.u8("x")?;
            c.u8("y")?;
            c.u8("z")?;
            c.u8("flags").map(drop)
        })
    })
}

fn layer_poly_mesh(c: &mut Cursor) -> DecodeResult<()> {
    let nvp = c.i32("nvp")?;
    let nv = c.i32("vert_count")?;
    let np = c.i32("poly_count")?;
    let nv = c.check_count("vert_count", i64::from(nv), 6)?;
    c.array("verts", nv, |c, _| {
        c.u16("x")?;
        c.u16("y")?;
        c.u16("z").map(drop)
    })?;
    let slots = poly_slots(c, nvp)?;
    let np = c.check_count("poly_count", i64::from(np), slots * 2 + 3)?;
    c.array("polys", np, |c, _| {
        c.array("slots", slots, |c, _| c.u16("").map(drop))?;
        c.u16("flags")?;
        c.u8("area").map(drop)
    })
}

fn navmesh_tiles(c: &mut Cursor) -> DecodeResult<()> {
    let n = c.count_u32("tile_count", 16)?;
    c.array("tiles", n, |c, _| {
        c.i32("x")?;
        c.i32("y")?;
        c.i32("layer")?;
        let len = c.count_u32("len", 1)?;
        c.group("data", |c| dnav(c, len))
    })
}

/// A Detour navmesh tile (`'DNAV'` version 7, links at the cursor's poly-ref width), byte for byte as
/// `dtCreateNavMeshData` lays it out (or as `dtNavMesh` holds it once linked).
pub fn dnav(c: &mut Cursor, len: usize) -> DecodeResult<()> {
    let start = c.pos();
    let counts = c.group("header", |c| {
        for name in ["magic", "version", "x", "y", "layer"] {
            c.i32(name)?;
        }
        c.u32("user_id")?;
        let mut counts = [0i32; 9];
        let names = [
            "poly_count",
            "vert_count",
            "max_link_count",
            "detail_mesh_count",
            "detail_vert_count",
            "detail_tri_count",
            "bv_node_count",
            "off_mesh_con_count",
            "off_mesh_base",
        ];
        for (slot, name) in counts.iter_mut().zip(names) {
            *slot = c.i32(name)?;
        }
        c.f32("walkable_height")?;
        c.f32("walkable_radius")?;
        c.f32("walkable_climb")?;
        c.v3("bmin")?;
        c.v3("bmax")?;
        c.f32("bv_quant_factor")?;
        Ok(counts)
    })?;
    let [
        polys,
        verts,
        links,
        dmeshes,
        dverts,
        dtris,
        bvnodes,
        offmesh,
        _,
    ] = counts;
    let n = |c: &Cursor, name: &'static str, v: i32, size: usize| {
        c.check_count(name, i64::from(v), size)
    };
    let verts = n(c, "vert_count", verts, 12)?;
    c.array("verts", verts, |c, _| c.v3("").map(drop))?;
    let polys = n(c, "poly_count", polys, 32)?;
    c.array("polys", polys, |c, _| {
        c.u32("first_link")?;
        c.array("verts", 6, |c, _| c.u16("").map(drop))?;
        c.array("neis", 6, |c, _| c.u16("").map(drop))?;
        c.u16("flags")?;
        c.u8("vert_count")?;
        c.u8("area_and_type").map(drop)
    })?;
    let links = n(c, "max_link_count", links, 12)?;
    c.array("links", links, |c, _| {
        c.poly_ref("ref")?;
        c.u32("next")?;
        c.u8("edge")?;
        c.u8("side")?;
        c.u8("bmin")?;
        c.u8("bmax").map(drop)
    })?;
    let dmeshes = n(c, "detail_mesh_count", dmeshes, 12)?;
    c.array("detail_meshes", dmeshes, |c, _| {
        c.u32("vert_base")?;
        c.u32("tri_base")?;
        c.u8("vert_count")?;
        c.u8("tri_count")?;
        c.bytes("pad", 2).map(drop)
    })?;
    let dverts = n(c, "detail_vert_count", dverts, 12)?;
    c.array("detail_verts", dverts, |c, _| c.v3("").map(drop))?;
    let dtris = n(c, "detail_tri_count", dtris, 4)?;
    c.array("detail_tris", dtris, |c, _| {
        c.u8("a")?;
        c.u8("b")?;
        c.u8("c")?;
        c.u8("flags").map(drop)
    })?;
    let bvnodes = n(c, "bv_node_count", bvnodes, 16)?;
    c.array("bv_tree", bvnodes, |c, _| {
        c.array("bmin", 3, |c, _| c.u16("").map(drop))?;
        c.array("bmax", 3, |c, _| c.u16("").map(drop))?;
        c.i32("i").map(drop)
    })?;
    let offmesh = n(c, "off_mesh_con_count", offmesh, 36)?;
    c.array("off_mesh_cons", offmesh, |c, _| {
        c.v3("start")?;
        c.v3("end")?;
        c.f32("rad")?;
        c.u16("poly")?;
        c.u8("flags")?;
        c.u8("side")?;
        c.u32("user_id").map(drop)
    })?;
    let used = c.pos() - start;
    if used != len {
        return Err(c.error(
            "data",
            format!("tile sections cover {used} bytes, tile length is {len}"),
        ));
    }
    Ok(())
}

fn crowd(c: &mut Cursor) -> DecodeResult<()> {
    let ticks = c.count_u32("tick_count", 8)?;
    c.array("ticks", ticks, |c, _| {
        c.u32("tick")?;
        let n = c.count_u32("active", 50)?;
        c.array("agents", n, |c, _| {
            c.u32("index")?;
            c.u8("state")?;
            c.u8("target_state")?;
            c.v3("npos")?;
            c.v3("vel")?;
            c.v3("nvel")?;
            c.poly_ref("first_poly")?;
            c.i32("corners")?;
            let n = c.count_u32("corridor_count", 4)?;
            c.array("corridor", n, |c, _| c.poly_ref("poly").map(drop))
        })
    })
}

fn obstacles(c: &mut Cursor) -> DecodeResult<()> {
    let n = c.count_u32("op_count", 1)?;
    c.array("ops", n, |c, _| {
        let op = c.u8("op")?;
        let args: &[&'static str] = match op {
            1 => &["x", "y", "z", "radius", "height"],
            2 => &["x0", "y0", "z0", "x1", "y1", "z1"],
            3 => &["cx", "cy", "cz", "hx", "hy", "hz", "yaw"],
            4 => &["index"],
            5 => &[],
            _ => return Err(c.error("op", format!("unknown obstacle op {op}"))),
        };
        for &a in args {
            c.f32(a)?;
        }
        match op {
            1..=3 => {
                c.u32("status")?;
                c.u32("ref")?;
                c.u32("touched").map(drop)
            }
            4 => {
                c.u32("ref")?;
                c.u32("status")?;
                c.u32("touched").map(drop)
            }
            _ => {
                c.u32("iterations")?;
                c.u32("status")?;
                c.u8("up_to_date")?;
                let slots = c.count_u32("slot_count", 12)?;
                c.array("slots", slots, |c, _| {
                    c.u32("index")?;
                    c.u32("ref")?;
                    c.u8("state")?;
                    c.u8("type")?;
                    c.u16("salt").map(drop)
                })?;
                let tiles = c.count_u32("tile_count", 13)?;
                c.array("tiles", tiles, |c, _| {
                    c.i32("tx")?;
                    c.i32("ty")?;
                    c.i32("tlayer")?;
                    if c.u8("present")? != 0 {
                        let len = c.count_u32("len", 1)?;
                        c.group("data", |c| dnav(c, len))?;
                    }
                    Ok(())
                })
            }
        }
    })
}
