use super::*;
use std::collections::HashSet;
pub(super) struct Process {
    verts: Vec<f32>,
    rad: Vec<f32>,
    dir: Vec<u8>,
    flags: Vec<u16>,
    areas: Vec<u8>,
    ids: Vec<u32>,
}
impl Process {
    pub(super) fn new(c: &Case) -> Self {
        let mut s = Self {
            verts: Vec::new(),
            rad: Vec::new(),
            dir: Vec::new(),
            flags: Vec::new(),
            areas: Vec::new(),
            ids: Vec::new(),
        };
        for (i, o) in c.off_mesh.as_chunks::<8>().0.iter().enumerate() {
            s.verts.extend_from_slice(&o[..6]);
            s.rad.push(o[6]);
            s.dir.push(u8::from(o[7] != 0.0));
            s.flags.push(1);
            s.areas.push(0);
            s.ids.push(1000 + u32::try_from(i).unwrap());
        }
        s
    }
}
impl MeshProcess for Process {
    fn process(&self, areas: &mut [u8], flags: &mut [u16]) {
        for (a, f) in areas.iter_mut().zip(flags) {
            if *a == 63 {
                *a = 0;
            }
            *f = poly_flags_for_area(*a);
        }
    }
    fn configure<'a>(&'a self, p: &mut detour::NavMeshCreateParams<'a>) {
        p.off_mesh_con_verts = &self.verts;
        p.off_mesh_con_rad = &self.rad;
        p.off_mesh_con_dir = &self.dir;
        p.off_mesh_con_flags = &self.flags;
        p.off_mesh_con_areas = &self.areas;
        p.off_mesh_con_user_id = &self.ids;
        p.off_mesh_con_count = i32::try_from(self.rad.len()).unwrap();
    }
}
pub(super) fn record_layer(
    ws: &mut Workspace,
    cache: &TileCache<RawCompressor, Process>,
    r: tc::CompressedTileRef,
    c: &Case,
    sink: &mut Sink,
) -> Result<(), Error> {
    let mut l = tc::decompress_layer(ws, &RawCompressor, cache.tile_data(r).unwrap())?;
    let h = l.header();
    let pre = format!("layer_{}_{}_{}.", h.tx, h.ty, h.tlayer);
    sink.record(&format!("{pre}layer_grid"), |p| {
        let h = l.header();
        p.i32(tc::LAYER_MAGIC)
            .i32(1)
            .i32(h.tx)
            .i32(h.ty)
            .i32(h.tlayer)
            .v3(h.bmin)
            .v3(h.bmax)
            .u16(h.hmin)
            .u16(h.hmax)
            .u8(h.width)
            .u8(h.height)
            .u8(h.minx)
            .u8(h.maxx)
            .u8(h.miny)
            .u8(h.maxy);
        p.bytes(l.heights()).bytes(l.areas()).bytes(l.connections());
    });
    let climb_value = c.agent_max_climb / c.cell_height;
    if !climb_value.is_finite() || !(0.0..2147483648.0).contains(&climb_value) {
        return Err(Error::InvalidParameter("walkable climb"));
    }
    let climb = climb_value as i32; // Validated nonnegative i32 range.
    tc::build_regions(ws, &mut l, climb)?;
    sink.record(&format!("{pre}layer_regions"), |p| {
        p.i32(i32::try_from(l.regions().len()).unwrap())
            .u8(l.region_count())
            .bytes(l.regions());
    });
    let cs = tc::build_contours(ws, &l, climb, c.edge_max_error)?;
    sink.record(&format!("{pre}layer_contours"), |p| {
        p.i32(i32::try_from(cs.contours.len()).unwrap());
        for cc in &cs.contours {
            p.u8(cc.reg)
                .u8(cc.area)
                .i32(i32::try_from(cc.verts.len() / 4).unwrap())
                .bytes(&cc.verts);
        }
    });
    let mut m = tc::build_poly_mesh(ws, &cs)?;
    cache_process(c, &mut m);
    sink.record(&format!("{pre}layer_poly_mesh"), |p| {
        p.i32(6)
            .i32(i32::try_from(m.verts.len() / 3).unwrap())
            .i32(i32::try_from(m.areas.len()).unwrap());
        for &v in &m.verts {
            p.u16(v);
        }
        for (i, poly) in m.polys.as_chunks::<12>().0.iter().enumerate() {
            for &v in poly {
                p.u16(v);
            }
            p.u16(m.flags[i]).u8(m.areas[i]);
        }
    });
    Ok(())
}
fn cache_process(c: &Case, m: &mut tc::LayerPolyMesh) {
    Process::new(c).process(&mut m.areas, &mut m.flags);
}
pub(super) fn record_obstacles(
    c: &Case,
    b: &mut Baked,
    nav: &mut NavMesh,
    sink: &mut Sink,
) -> Result<(), Error> {
    let Some(cache) = b.cache.as_mut() else {
        return Ok(());
    };
    if c.obstacle_ops.is_empty() {
        return Ok(());
    }
    let mut p = Payload::new();
    p.u32(u32::try_from(c.obstacle_ops.len()).unwrap());
    let mut added = Vec::new();
    let mut touched = Vec::new();
    let mut seen = HashSet::new();
    let mut updates = 0;
    for (op, args) in &c.obstacle_ops {
        p.u8(*op);
        for &v in args {
            p.f32(v);
        }
        if *op <= 3 {
            let a = args;
            let shape = match op {
                1 => ObstacleShape::Cylinder {
                    pos: [a[0], a[1], a[2]],
                    radius: a[3],
                    height: a[4],
                },
                2 => ObstacleShape::Box {
                    bmin: [a[0], a[1], a[2]],
                    bmax: [a[3], a[4], a[5]],
                },
                _ => ObstacleShape::OrientedBox {
                    center: [a[0], a[1], a[2]],
                    half_extents: [a[3], a[4], a[5]],
                    yaw: a[6],
                },
            };
            let r = cache.add_obstacle(shape)?;
            added.push(r);
            let (a, bb) = shape.bounds();
            let ts = cache.query_tiles(&a, &bb, 64)?;
            for &t in &ts {
                if seen.insert(t.bits()) {
                    touched.push(t);
                }
            }
            p.u32(0x40000000)
                .u32(r.bits())
                .u32(u32::try_from(ts.len()).unwrap());
        } else if *op == 4 {
            let k = args[0]
                .to_string()
                .parse::<usize>()
                .map_err(|_| Error::InvalidParameter("obstacle index"))?;
            let r = *added.get(k).ok_or(Error::InvalidRef)?;
            let shape = cache.obstacle(r).ok_or(Error::InvalidRef)?.shape();
            let (a, bb) = shape.bounds();
            let ts = cache.query_tiles(&a, &bb, 64)?;
            for &t in &ts {
                if seen.insert(t.bits()) {
                    touched.push(t);
                }
            }
            cache.remove_obstacle(r)?;
            p.u32(r.bits())
                .u32(0x40000000)
                .u32(u32::try_from(ts.len()).unwrap());
        } else {
            let mut iters = 0;
            loop {
                iters += 1;
                if cache.update(1.0 / 30.0, nav)? {
                    break;
                }
                if iters >= 4096 {
                    return Err(Error::Failed);
                }
            }
            updates += 1;
            p.u32(iters).u32(0x40000000).u8(1);
            let used = (0..cache.params().max_obstacles)
                .filter(|&i| cache.obstacle_at(i).is_some())
                .count();
            p.u32(u32::try_from(used).unwrap());
            for i in 0..cache.params().max_obstacles {
                if let Some((r, o)) = cache.obstacle_at(i) {
                    p.u32(u32::try_from(i).unwrap())
                        .u32(r.bits())
                        .u8(o.state().code())
                        .u8(o.shape().type_code())
                        .u16(o.salt());
                }
            }
            p.u32(u32::try_from(touched.len()).unwrap());
            for t in touched.drain(..) {
                let h = cache.tile_header(t).unwrap();
                p.i32(h.tx).i32(h.ty).i32(h.tlayer);
                if let Some(i) = nav.tile_index_at(h.tx, h.ty, h.tlayer) {
                    let mt = nav.tile(i).unwrap();
                    let data = resident_bytes(mt);
                    p.u8(1).u32(u32::try_from(data.len()).unwrap()).bytes(&data);
                } else {
                    p.u8(0);
                }
            }
            seen.clear();
            // Query every resulting mesh, under both filters, after each script update.
            record_queries(
                c,
                nav,
                &DefaultFilter::default(),
                &format!("obstacle_update_{updates}."),
                sink,
            )?;
            record_queries(
                c,
                nav,
                &cost_filter(),
                &format!("obstacle_update_{updates}.filtered."),
                sink,
            )?;
        }
    }
    if let Some(w) = sink.writer.as_mut() {
        w.record("tilecache_obstacles", p.as_bytes());
    }
    Ok(())
}

// MeshTile.data intentionally retains the input. Upstream mutates its raw
// allocation in place. Serialize the mutable runtime sections for resident-byte
// comparisons, including unused/free link slots and snapped off-mesh vertices.
fn resident_bytes(t: &detour::MeshTile) -> Vec<u8> {
    let mut data = t.data.clone();
    for (i, v) in t.verts.iter().enumerate() {
        data[100 + i * 4..104 + i * 4].copy_from_slice(&v.to_le_bytes());
    }
    let poly_base = 100 + t.verts.len() * 4;
    for (i, p) in t.polys.iter().enumerate() {
        let b = poly_base + i * 32;
        data[b..b + 4].copy_from_slice(&p.first_link.to_le_bytes());
        for (j, v) in p.verts.iter().chain(&p.neis).enumerate() {
            data[b + 4 + j * 2..b + 6 + j * 2].copy_from_slice(&v.to_le_bytes());
        }
        data[b + 28..b + 30].copy_from_slice(&p.flags.to_le_bytes());
        data[b + 30] = p.vert_count;
        data[b + 31] = p.area_and_type;
    }
    let link_base = poly_base + t.polys.len() * 32;
    let ref_size = std::mem::size_of::<PolyRef>();
    let link_size = ref_size + 8;
    for (i, l) in t.links.iter().enumerate() {
        let b = link_base + i * link_size;
        data[b..b + ref_size].copy_from_slice(&l.r#ref.to_le_bytes());
        data[b + ref_size..b + ref_size + 4].copy_from_slice(&l.next.to_le_bytes());
        data[b + ref_size + 4..b + ref_size + 8].copy_from_slice(&[l.edge, l.side, l.bmin, l.bmax]);
    }
    data
}
