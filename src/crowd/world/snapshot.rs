//! The `'DCRW'` snapshot: a little-endian byte image of a [`CrowdWorld`] for
//! replays and saves (not in recastnavigation).
//!
//! Version 1 (base): magic `DCRW`, version, world parameters, fixes, the sixteen
//! filters, the eight avoidance presets, the generation of every agent index, then
//! each agent's full state in index order, then the path queue's eight request
//! slots. Every float is stored as its bits, so a restored world steps bit for bit
//! like the original, including the node pool and heap of a sliced path search in progress.
//! Version 2 (read-only) is the base layout plus the arrival policy, distance and
//! arrived flag at the start of each agent. Version 3 (read-only) is the base layout
//! plus the runtime-link epoch after the version number and, at the end of each
//! agent, the external traversal mode, replan flag and optional pending traversal.
//! Version 4 (extended) has both extensions.
//! Version 5 adds the replan limit, cursor and the navmesh change epoch of the last
//! completed step after the runtime-link epoch, and each agent's deferred-replan bit
//! and watched obstruction after its agent record. The last-step epoch lets a
//! restored world replan exactly where the original would after edits that followed
//! its last step; restore keeps it when it does not exceed the live mesh's epoch
//! (the live epoch is used when it does, as for a mesh rebuilt from scratch).
//! Version 6 uses the version 5 layout with a profile ID before each occupied
//! agent record and after each queue handle. It requires a mixed profile binding.
//!
//! The format is canonical: a reader accepts only the version the restored state
//! encodes with, a fixes word that includes the always-on nearest-neighbour
//! correction, so a snapshot of version 1 or 4 to 6 re-encodes to its own bytes on
//! the mesh it was saved from. The two mesh-epoch fields are the exception: they
//! rebind to a live mesh that is behind them. Versions 2 and 3 migrate to version 4.
//!
//! The profile (`NPCW`) and framed (`DCFR`) records embed the same `DCRW` layouts.
//! A profile record binds the catalog and slot history, refuses to save while the
//! mesh has changed since the last step, and writes zero in the sliced-search epoch
//! and last-step epoch fields. Restore validates the refs and substitutes the
//! reconstructed mesh's epochs. A framed record stores rider bindings and
//! goals outside its frame-local `DCRW` payloads.
use super::*;
use crate::SnapshotError as E;
use crate::detour::Status;

const MAGIC: &[u8; 4] = b"DCRW";
const BASE_VERSION: u32 = 1;
const ARRIVAL_VERSION: u32 = 2;
const LINKS_VERSION: u32 = 3;
const EXTENDED_VERSION: u32 = 4;
const REPLAN_VERSION: u32 = 5;
const MIXED_VERSION: u32 = 6;

#[derive(Clone, Copy)]
struct AgentLayout {
    arrival: bool,
    links: bool,
}

/// A polygon reference as 64 bits, whatever the reference width.
// The conversion is the identity with 64-bit references.
#[allow(clippy::useless_conversion)]
pub(super) fn wide(r: PolyRef) -> u64 {
    u64::from(r)
}

struct Out(Vec<u8>);
impl Out {
    fn bytes(&mut self, b: &[u8]) {
        self.0.extend_from_slice(b);
    }
    fn u8(&mut self, v: u8) {
        self.0.push(v);
    }
    fn u16(&mut self, v: u16) {
        self.bytes(&v.to_le_bytes());
    }
    fn u32(&mut self, v: u32) {
        self.bytes(&v.to_le_bytes());
    }
    fn u64(&mut self, v: u64) {
        self.bytes(&v.to_le_bytes());
    }
    fn f32(&mut self, v: f32) {
        self.u32(v.to_bits());
    }
    fn v3(&mut self, v: V3) {
        v.iter().for_each(|&x| self.f32(x));
    }
    fn r(&mut self, r: PolyRef) {
        self.u64(wide(r));
    }
    fn len(&mut self, n: usize) {
        self.u32(n as u32);
    }
}

struct In<'a>(&'a [u8]);
fn bad(what: &'static str) -> Error {
    Error::InvalidSnapshot(E::InvalidValue(what))
}
impl In<'_> {
    fn take<const N: usize>(&mut self) -> Result<[u8; N], Error> {
        let (head, rest) = self
            .0
            .split_first_chunk::<N>()
            .ok_or(Error::InvalidSnapshot(E::TooShort))?;
        self.0 = rest;
        Ok(*head)
    }
    fn u8(&mut self) -> Result<u8, Error> {
        Ok(self.take::<1>()?[0])
    }
    fn flag(&mut self, what: &'static str) -> Result<bool, Error> {
        match self.u8()? {
            0 => Ok(false),
            1 => Ok(true),
            _ => Err(bad(what)),
        }
    }
    fn u16(&mut self) -> Result<u16, Error> {
        Ok(u16::from_le_bytes(self.take()?))
    }
    fn u32(&mut self) -> Result<u32, Error> {
        Ok(u32::from_le_bytes(self.take()?))
    }
    fn u64(&mut self) -> Result<u64, Error> {
        Ok(u64::from_le_bytes(self.take()?))
    }
    fn f32(&mut self, what: &'static str) -> Result<f32, Error> {
        let v = f32::from_bits(self.u32()?);
        if v.is_finite() { Ok(v) } else { Err(bad(what)) }
    }
    fn v3(&mut self, what: &'static str) -> Result<V3, Error> {
        Ok([self.f32(what)?, self.f32(what)?, self.f32(what)?])
    }
    fn r(&mut self) -> Result<PolyRef, Error> {
        PolyRef::try_from(self.u64()?).map_err(|_| bad("polygon reference width"))
    }
    /// A count no larger than `max`.
    fn len(&mut self, max: usize, what: &'static str) -> Result<usize, Error> {
        let n = self.u32()? as usize;
        if n > max { Err(bad(what)) } else { Ok(n) }
    }
}

fn write_params(o: &mut Out, p: &AgentParams) {
    for v in [
        p.radius,
        p.height,
        p.max_acceleration,
        p.max_speed,
        p.collision_query_range,
        p.path_optimization_range,
        p.separation_weight,
    ] {
        o.f32(v);
    }
    o.u8(p.update_flags);
    o.u8(p.obstacle_avoidance_type as u8);
    o.u8(p.query_filter_type as u8);
    match p.avoidance {
        AvoidanceMode::Orca(q) => {
            o.u8(1);
            o.f32(q.time_horizon);
            o.f32(q.time_horizon_obstacles);
            o.u16(q.max_neighbours);
        }
        _ => o.u8(0),
    }
}

fn read_params(i: &mut In<'_>, max_radius: f32) -> Result<AgentParams, Error> {
    let mut p = AgentParams::default();
    for v in [
        &mut p.radius,
        &mut p.height,
        &mut p.max_acceleration,
        &mut p.max_speed,
        &mut p.collision_query_range,
        &mut p.path_optimization_range,
        &mut p.separation_weight,
    ] {
        *v = i.f32("agent parameters")?;
    }
    p.update_flags = i.u8()?;
    p.obstacle_avoidance_type = usize::from(i.u8()?);
    p.query_filter_type = usize::from(i.u8()?);
    p.avoidance = match i.u8()? {
        0 => AvoidanceMode::Sampled,
        1 => AvoidanceMode::Orca(OrcaParams {
            time_horizon: i.f32("ORCA parameters")?,
            time_horizon_obstacles: i.f32("ORCA parameters")?,
            max_neighbours: i.u16()?,
        }),
        _ => return Err(bad("avoidance mode")),
    };
    p.validate(max_radius)
        .map_err(|_| bad("agent parameters"))?;
    Ok(p)
}

fn write_agent(o: &mut Out, a: &CrowdAgent, layout: AgentLayout) {
    if layout.arrival {
        o.u8(match a.params.arrival_policy {
            ArrivalPolicy::Keep => 0,
            ArrivalPolicy::Park => 1,
            ArrivalPolicy::Remove => 2,
        });
        o.f32(a.params.arrival_distance);
        o.u8(u8::from(a.arrived));
    }
    o.u8(a.state as u8);
    o.u8(a.target_state as u8);
    for v in [a.npos, a.vel, a.nvel, a.dvel] {
        o.v3(v);
    }
    o.f32(a.desired_speed);
    o.v3(a.corridor.pos);
    o.v3(a.corridor.target);
    o.len(a.corridor.path.len());
    a.corridor.path.iter().for_each(|&r| o.r(r));
    o.v3(a.boundary.center);
    o.len(a.boundary.segments.len());
    for (d, s) in &a.boundary.segments {
        o.f32(*d);
        s.iter().for_each(|&x| o.f32(x));
    }
    o.len(a.boundary.polys.len());
    a.boundary.polys.iter().for_each(|&r| o.r(r));
    o.len(a.corners.len());
    for c in &a.corners {
        o.v3(c.pos);
        o.u8(c.flags);
        o.r(c.r#ref);
    }
    write_params(o, &a.params);
    o.r(a.target_ref);
    o.v3(a.target_pos);
    o.u8(u8::from(a.partial));
    match a.waiting_tile {
        Some(k) => {
            o.u8(1);
            o.u32(k.x as u32);
            o.u32(k.y as u32);
        }
        None => o.u8(0),
    }
    o.f32(a.topology_time);
    o.f32(a.replan_time);
    o.u8(u8::from(a.replan));
    o.u32(a.pathq);
    o.len(a.neis.len());
    for &(j, d) in &a.neis {
        o.u32(j as u32);
        o.f32(d);
    }
    o.len(a.nneis);
    o.v3(a.disp);
    o.u8(u8::from(a.anim.active));
    for v in [a.anim.init, a.anim.start, a.anim.end] {
        o.v3(v);
    }
    o.f32(a.anim.t);
    o.f32(a.anim.tmax);
    if layout.links {
        o.u8(u8::from(a.params.off_mesh == OffMeshMode::External));
        o.u8(u8::from(a.pending_link_replan));
        o.u8(u8::from(a.pending_off_mesh.is_some()));
        if let Some(p) = a.pending_off_mesh {
            let r = p.reached;
            o.r(r.poly);
            o.u32(r.user_id);
            o.u8(u8::from(r.link_id.is_some()));
            o.v3(r.start);
            o.v3(r.end);
            o.r(r.departure);
            o.r(r.landing);
            o.f32(p.radius);
        }
    }
}

fn read_agent(
    i: &mut In<'_>,
    nav: &NavMesh,
    a: &mut CrowdAgent,
    agents: usize,
    max_radius: f32,
    ws: &mut Workspace,
    layout: AgentLayout,
) -> Result<(), Error> {
    let (policy, distance, arrived) = if layout.arrival {
        let policy = match i.u8()? {
            0 => ArrivalPolicy::Keep,
            1 => ArrivalPolicy::Park,
            2 => ArrivalPolicy::Remove,
            _ => return Err(bad("arrival policy")),
        };
        let distance = i.f32("arrival distance")?;
        if distance <= 0. {
            return Err(bad("arrival distance"));
        }
        (policy, distance, i.flag("arrived")?)
    } else {
        (ArrivalPolicy::Keep, 0.5, false)
    };
    a.state = match i.u8()? {
        0 => AgentState::Invalid,
        1 => AgentState::Walking,
        2 => AgentState::Offmesh,
        _ => return Err(bad("agent state")),
    };
    a.target_state = match i.u8()? {
        0 => TargetState::None,
        1 => TargetState::Failed,
        2 => TargetState::Valid,
        3 => TargetState::Requesting,
        4 => TargetState::WaitingForQueue,
        5 => TargetState::WaitingForPath,
        6 => TargetState::Velocity,
        7 => TargetState::WaitingForTile,
        _ => return Err(bad("target state")),
    };
    a.npos = i.v3("position")?;
    a.vel = i.v3("velocity")?;
    a.nvel = i.v3("velocity")?;
    a.dvel = i.v3("velocity")?;
    a.desired_speed = i.f32("desired speed")?;
    a.corridor.pos = i.v3("corridor")?;
    a.corridor.target = i.v3("corridor")?;
    let n = i.len(256, "corridor length")?;
    a.corridor.path.clear();
    for _ in 0..n {
        a.corridor.path.push(i.r()?);
    }
    a.boundary.center = i.v3("wall center")?;
    let n = i.len(8, "wall count")?;
    a.boundary.segments.clear();
    for _ in 0..n {
        let d = i.f32("wall")?;
        let mut s = [0.; 6];
        for x in &mut s {
            *x = i.f32("wall")?;
        }
        a.boundary.segments.push((d, s));
    }
    let n = i.len(16, "wall polygon count")?;
    a.boundary.polys.clear();
    for _ in 0..n {
        a.boundary.polys.push(i.r()?);
    }
    let n = i.len(4, "corner count")?;
    a.corners.clear();
    for _ in 0..n {
        let pos = i.v3("corner")?;
        let flags = i.u8()?;
        let r#ref = i.r()?;
        a.corners.push(StraightPathPoint { pos, flags, r#ref });
    }
    a.params = read_params(i, max_radius)?;
    a.params.arrival_policy = policy;
    a.params.arrival_distance = distance;
    a.arrived = arrived;
    a.target_ref = i.r()?;
    a.target_pos = i.v3("target")?;
    a.partial = i.flag("partial flag")?;
    a.waiting_tile = if i.flag("waiting tile")? {
        Some(TileKey {
            x: i.u32()? as i32,
            y: i.u32()? as i32,
        })
    } else {
        None
    };
    a.topology_time = i.f32("timer")?;
    a.replan_time = i.f32("timer")?;
    a.replan = i.flag("replan flag")?;
    a.pathq = i.u32()?;
    let n = i.len(usize::from(u16::MAX), "neighbour capacity")?;
    ws.charge_bytes::<(usize, f32)>(n)?;
    a.neis.clear();
    a.neis.try_reserve(n).map_err(|_| Error::AllocationFailed)?;
    for _ in 0..n {
        let j = i.u32()? as usize;
        if j >= agents {
            return Err(bad("neighbour index"));
        }
        a.neis.push((j, i.f32("neighbour distance")?));
    }
    a.nneis = i.len(a.neis.len(), "neighbour count")?;
    a.disp = i.v3("displacement")?;
    a.anim.active = i.flag("animation flag")?;
    a.anim.init = i.v3("animation")?;
    a.anim.start = i.v3("animation")?;
    a.anim.end = i.v3("animation")?;
    a.anim.t = i.f32("animation")?;
    a.anim.tmax = i.f32("animation")?;
    if layout.links {
        if i.flag("external traversal mode")? {
            a.params.off_mesh = OffMeshMode::External;
        }
        a.pending_link_replan = i.flag("runtime link replan")?;
        if i.flag("pending external traversal")? {
            let poly = i.r()?;
            let user_id = i.u32()?;
            let link_id = i
                .flag("runtime traversal")?
                .then_some(crate::detour::LinkId(user_id));
            let start = i.v3("external traversal")?;
            let end = i.v3("external traversal")?;
            let departure = i.r()?;
            let landing = i.r()?;
            let radius = i.f32("external traversal radius")?;
            if a.state != AgentState::Offmesh
                || a.anim.active
                || poly == 0
                || departure == 0
                || landing == 0
                || radius < 0.0
                || a.npos != start
                || !matches!(
                    nav.off_mesh_connection_by_ref(poly),
                    Some(con) if con.user_id == user_id && con.rad.to_bits() == radius.to_bits()
                )
                || nav.runtime_link_id(poly) != link_id
                || nav
                    .off_mesh_connection_poly_end_points(departure, poly)
                    .ok()
                    != Some((start, end))
            {
                return Err(bad("pending external traversal"));
            }
            a.pending_off_mesh = Some(PendingTraversal {
                reached: OffMeshReached {
                    poly,
                    user_id,
                    link_id,
                    kind: nav
                        .off_mesh_connection_by_ref(poly)
                        .and_then(|con| con.kind()),
                    start,
                    end,
                    departure,
                    landing,
                },
                radius,
            });
        }
    }
    a.active = true;
    Ok(())
}

impl CrowdWorld {
    /// The world as `'DCRW'` snapshot bytes (see the format in the module source).
    /// In-progress searches retain their node pool and heap for exact continuation.
    pub fn to_bytes(&self) -> Vec<u8> {
        self.snapshot_bytes(false)
    }

    /// `bound` writes zero in the mesh-epoch fields: profile and mixed records
    /// rebuild their mesh, so restore substitutes the reconstructed mesh's epochs.
    fn snapshot_bytes(&self, bound: bool) -> Vec<u8> {
        let (version, layout) = self.canonical_version(bound);
        self.write_snapshot(version, layout, bound)
    }

    /// The version and agent layout `snapshot_bytes` writes for this state. A reader
    /// accepts a current version only when it is this one, so every accepted
    /// snapshot re-encodes to its own bytes.
    fn canonical_version(&self, bound: bool) -> (u32, AgentLayout) {
        let replan = self.replan_limit != replan::DEFAULT_LIMIT
            || self.replan_cursor != 0
            || (!bound && self.replan_epoch != 0)
            || self
                .heavy
                .iter()
                .any(|a| a.active && (a.pending_stamp_replan || a.blocked_poly != 0));
        let extended = self.mixed.is_some()
            || replan
            || self.link_epoch != 0
            || self.heavy.iter().any(|a| {
                a.active
                    && (a.params.off_mesh == OffMeshMode::External
                        || a.pending_off_mesh.is_some()
                        || a.pending_link_replan
                        || a.params.arrival_policy != ArrivalPolicy::Keep
                        || a.params.arrival_distance != 0.5
                        || a.arrived)
            });
        let layout = AgentLayout {
            arrival: extended,
            links: extended,
        };
        let version = if self.mixed.is_some() {
            MIXED_VERSION
        } else if replan {
            REPLAN_VERSION
        } else if extended {
            EXTENDED_VERSION
        } else {
            BASE_VERSION
        };
        (version, layout)
    }

    fn write_snapshot(&self, version: u32, layout: AgentLayout, bound: bool) -> Vec<u8> {
        let mut o = Out(Vec::new());
        o.bytes(MAGIC);
        o.u32(version);
        if layout.links {
            o.u64(self.link_epoch);
        }
        if version >= REPLAN_VERSION {
            o.u32(self.replan_limit);
            o.u32(self.replan_cursor);
            o.u64(if bound { 0 } else { self.replan_epoch });
        }
        let p = &self.params;
        o.u32(p.max_agents);
        o.u32(p.max_heavy_agents);
        o.f32(p.max_radius);
        o.u32(p.path_queue.max_nodes as u32);
        o.u32(p.path_queue.iterations as u32);
        o.f32(p.fixed_dt);
        o.u64(params_hash(p));
        o.u32(self.fixes.bits());
        for f in &self.filters {
            f.area_cost.iter().for_each(|&c| o.f32(c));
            o.u16(f.include_flags);
            o.u16(f.exclude_flags);
        }
        for a in &self.presets {
            for v in [
                a.vel_bias,
                a.weight_des_vel,
                a.weight_cur_vel,
                a.weight_side,
                a.weight_toi,
                a.horiz_time,
            ] {
                o.f32(v);
            }
            for v in [
                a.grid_size,
                a.adaptive_divs,
                a.adaptive_rings,
                a.adaptive_depth,
            ] {
                o.u8(v);
            }
        }
        o.len(self.generation.len());
        for (i, &g) in self.generation.iter().enumerate() {
            o.u32(g);
            o.u8(u8::from(self.heavy_of[i] != NONE));
        }
        for (index, &s) in self.heavy_of.iter().enumerate() {
            if s != NONE {
                let a = &self.heavy[s as usize];
                if let Some(mixed) = &self.mixed {
                    o.u32(mixed.profiles[index]);
                }
                write_agent(&mut o, a, layout);
                if version >= REPLAN_VERSION {
                    o.u8(u8::from(a.pending_stamp_replan));
                    o.r(a.blocked_poly);
                }
            }
        }
        let q = &self.pathq;
        o.u32((q.head % 8) as u32);
        o.u32(q.next);
        for r in &q.requests {
            o.u32(r.handle);
            if version == MIXED_VERSION {
                o.u32(if r.handle == 0 { 0 } else { r.profile });
            }
            o.r(r.start);
            o.r(r.end);
            o.v3(r.start_pos);
            o.v3(r.end_pos);
            o.u8(r.filter as u8);
            o.u32(r.status);
            o.u8(r.keep);
            o.len(r.path.len());
            r.path.iter().for_each(|&x| o.r(x));
        }
        write_search(&mut o, q, bound);
        o.0
    }

    #[cfg(feature = "recast")]
    pub(crate) fn profile_snapshot(&self, nav: &NavMesh) -> Result<Vec<u8>, Error> {
        // The profile wrapper binds the exact slot history. Its local mesh epochs
        // are reconstructed when that history is installed in a new instance.
        if nav.change_epoch() != self.replan_epoch {
            return Err(Error::MeshChanged);
        }
        if let Some(query) = self.profile_search(nav)?
            && crate::detour::query::pool_changed(nav, query.query.epoch, &query.node_pool)
        {
            return Err(Error::MeshChanged);
        }
        Ok(self.snapshot_bytes(true))
    }

    #[cfg(feature = "recast")]
    pub(crate) fn restore_profile_snapshot(
        nav: &NavMesh,
        bytes: &[u8],
        limits: Limits,
    ) -> Result<Self, Error> {
        let mut world = Self::read_snapshot(nav, bytes, limits, None, true)?;
        world.bind_replan_epoch(nav, true)?;
        if let Some(query) = world.profile_search(nav)? {
            if query.query.epoch != 0 {
                return Err(bad("profile search epoch"));
            }
            let query = world.pathq.query.as_mut().ok_or(bad("queue query"))?;
            query.query.epoch = nav.epoch();
        }
        Ok(world)
    }

    /// The running search's query when every ref it holds is resident on `nav`.
    fn profile_search(
        &self,
        nav: &NavMesh,
    ) -> Result<Option<&crate::detour::query::DetachedQuery>, Error> {
        if !has_running_search(&self.pathq) {
            return Ok(None);
        }
        let query = self.pathq.query.as_ref().ok_or(bad("queue query"))?;
        if !nav.is_valid_poly_ref(query.query.start_ref)
            || !nav.is_valid_poly_ref(query.query.end_ref)
            || query
                .node_pool
                .used()
                .iter()
                .any(|node| !nav.is_valid_poly_ref(node.id))
        {
            return Err(Error::MeshChanged);
        }
        Ok(Some(query))
    }

    #[cfg(feature = "recast")]
    pub(crate) fn mixed_snapshot(
        &self,
        meshes: &BTreeMap<u32, &NavMesh>,
    ) -> Result<Vec<u8>, Error> {
        let mixed = self.mixed.as_ref().ok_or(bad("profile binding"))?;
        for (&id, &epoch) in &mixed.epochs {
            let mesh = meshes.get(&id).ok_or(bad("profile binding"))?;
            if mesh.change_epoch() != epoch {
                return Err(Error::MeshChanged);
            }
        }
        if has_running_search(&self.pathq) {
            let request = &self.pathq.requests[self.pathq.head % 8];
            let mesh = meshes.get(&request.profile).ok_or(bad("queue profile"))?;
            if let Some(query) = self.profile_search(mesh)?
                && crate::detour::query::pool_changed(mesh, query.query.epoch, &query.node_pool)
            {
                return Err(Error::MeshChanged);
            }
        }
        Ok(self.snapshot_bytes(true))
    }

    #[cfg(feature = "recast")]
    pub(crate) fn restore_mixed_snapshot(
        meshes: &BTreeMap<u32, &NavMesh>,
        bytes: &[u8],
        limits: Limits,
    ) -> Result<Self, Error> {
        let (_, nav) = meshes.first_key_value().ok_or(bad("empty profile set"))?;
        let mut world = Self::read_snapshot(nav, bytes, limits, Some(meshes), true)?;
        world.bind_replan_epoch(nav, true)?;
        if has_running_search(&world.pathq) {
            let request = &world.pathq.requests[world.pathq.head % 8];
            let mesh = meshes.get(&request.profile).ok_or(bad("queue profile"))?;
            if world
                .profile_search(mesh)?
                .is_none_or(|q| q.query.epoch != 0)
            {
                return Err(bad("profile search epoch"));
            }
            let query = world.pathq.query.as_mut().ok_or(bad("queue query"))?;
            query.query.epoch = mesh.epoch();
        }
        Ok(world)
    }

    /// Restores a world from untrusted `'DCRW'` bytes for agents on `nav`. Rejects
    /// malformed input with [`Error::InvalidSnapshot`]; `limits` bound the
    /// restored world's allocation as in [`CrowdWorld::new`]. Restore the
    /// navmesh to the state it had at the snapshot's last completed crowd step.
    pub fn from_bytes(nav: &NavMesh, b: &[u8], limits: Limits) -> Result<Self, Error> {
        let mut world = Self::read_snapshot(nav, b, limits, None, false)?;
        world.bind_replan_epoch(nav, false)?;
        // A running search keeps its saved epoch only when the live mesh has not
        // fallen behind it, so a restored search still sees later edits.
        if let Some(saved) = world.profile_search(nav)?.map(|q| q.query.epoch) {
            let query = world.pathq.query.as_mut().ok_or(bad("queue query"))?;
            query.query.epoch = saved.min(nav.epoch());
        }
        Ok(world)
    }

    /// Sets the last-step epoch after a read. A bound (profile or mixed) record
    /// stores zero and takes the reconstructed mesh's epoch. Otherwise a stored
    /// epoch ahead of the live mesh, or an absent one, means the mesh has no
    /// comparable history and is taken to match the last step.
    fn bind_replan_epoch(&mut self, nav: &NavMesh, bound: bool) -> Result<(), Error> {
        let live = nav.change_epoch();
        if bound && self.replan_epoch != 0 {
            return Err(bad("profile replan epoch"));
        }
        if self.replan_epoch == 0 || self.replan_epoch > live {
            self.replan_epoch = live;
        }
        Ok(())
    }

    fn read_snapshot(
        nav: &NavMesh,
        b: &[u8],
        limits: Limits,
        profiles: Option<&BTreeMap<u32, &NavMesh>>,
        bound: bool,
    ) -> Result<Self, Error> {
        let mut i = In(b);
        if &i.take::<4>()? != MAGIC {
            return Err(Error::InvalidSnapshot(E::WrongMagic));
        }
        let version = i.u32()?;
        if (version == MIXED_VERSION) != profiles.is_some() {
            return Err(Error::InvalidSnapshot(E::WrongVersion));
        }
        let layout = match version {
            BASE_VERSION => AgentLayout {
                arrival: false,
                links: false,
            },
            ARRIVAL_VERSION => AgentLayout {
                arrival: true,
                links: false,
            },
            LINKS_VERSION => AgentLayout {
                arrival: false,
                links: true,
            },
            EXTENDED_VERSION | REPLAN_VERSION | MIXED_VERSION => AgentLayout {
                arrival: true,
                links: true,
            },
            _ => return Err(Error::InvalidSnapshot(E::WrongVersion)),
        };
        let link_epoch = if layout.links { i.u64()? } else { 0 };
        let (replan_limit, replan_cursor, replan_epoch) = if version >= REPLAN_VERSION {
            (i.u32()?, i.u32()?, i.u64()?)
        } else {
            (replan::DEFAULT_LIMIT, 0, 0)
        };
        let p = read_world_params(&mut i)?;
        if replan_cursor > p.max_agents {
            return Err(bad("replan cursor"));
        }
        let mut w = CrowdWorld::new(nav, &p, limits)?;
        w.mixed = profiles.map(Self::mixed_state);
        w.link_epoch = link_epoch;
        w.replan_limit = replan_limit;
        w.replan_cursor = replan_cursor;
        // Zero (absent) is resolved by `bind_replan_epoch`.
        w.replan_epoch = replan_epoch;
        let fixes = UpstreamFixes::from_bits(i.u32()?).ok_or(bad("fixes"))?;
        // `set_fixes` always adds this flag, so a record without it is not canonical.
        if !fixes.contains(UpstreamFixes::NEAREST_NEIGHBOURS) {
            return Err(bad("fixes"));
        }
        w.set_fixes(fixes);
        for k in 0..16 {
            let mut f = DefaultFilter::default();
            for c in &mut f.area_cost {
                *c = i.f32("filter")?;
            }
            f.include_flags = i.u16()?;
            f.exclude_flags = i.u16()?;
            w.set_filter(k, f).map_err(|_| bad("filter"))?;
        }
        for k in 0..8 {
            let mut a = ObstacleAvoidanceParams::default();
            for v in [
                &mut a.vel_bias,
                &mut a.weight_des_vel,
                &mut a.weight_cur_vel,
                &mut a.weight_side,
                &mut a.weight_toi,
                &mut a.horiz_time,
            ] {
                *v = i.f32("avoidance preset")?;
            }
            for v in [
                &mut a.grid_size,
                &mut a.adaptive_divs,
                &mut a.adaptive_rings,
                &mut a.adaptive_depth,
            ] {
                *v = i.u8()?;
            }
            w.set_obstacle_avoidance_params(k, a)
                .map_err(|_| bad("avoidance preset"))?;
        }
        let n = i.len(p.max_agents as usize, "agent count")?;
        if n > i.0.len() / 5 {
            return Err(Error::InvalidSnapshot(E::TooShort));
        }
        let mut ws = Workspace::with_limits(limits);
        ws.charge_bytes::<u8>(
            p.path_queue.max_nodes * 128 + 16384 + 8 * 256 * core::mem::size_of::<PolyRef>(),
        )?;
        // SoA, occupancy, free indices, and side-pool headers.
        ws.charge_bytes::<u8>(
            n.checked_mul(80 + core::mem::size_of::<CrowdAgent>())
                .ok_or(Error::SizeOverflow)?,
        )?;
        let mut occupied = Vec::new();
        occupied
            .try_reserve(n)
            .map_err(|_| Error::AllocationFailed)?;
        for _ in 0..n {
            w.grow_by_one()?;
            let k = w.generation.len() - 1;
            w.generation[k] = i.u32()?;
            occupied.push(i.flag("occupancy")?);
        }
        if occupied.iter().filter(|&&o| o).count() > p.max_heavy_agents as usize {
            return Err(bad("agent count"));
        }
        for (k, &o) in occupied.iter().enumerate() {
            if !o {
                w.free.push(Reverse(k as u32));
                continue;
            }
            let mut agent_nav = nav;
            if let Some(mixed) = &mut w.mixed {
                let profile = i.u32()?;
                if !mixed.epochs.contains_key(&profile) {
                    return Err(bad("agent profile"));
                }
                mixed.profiles[k] = profile;
                if let Some(&mesh) = profiles.and_then(|meshes| meshes.get(&profile)) {
                    agent_nav = mesh;
                }
            }
            let mut a = CrowdAgent::new_in(&mut ws)?;
            read_agent(&mut i, agent_nav, &mut a, n, p.max_radius, &mut ws, layout)?;
            if version >= REPLAN_VERSION {
                a.pending_stamp_replan = i.flag("deferred replan")?;
                a.blocked_poly = i.r()?;
                if (a.pending_stamp_replan || a.blocked_poly != 0)
                    && (a.target_ref == 0
                        || matches!(a.target_state, TargetState::None | TargetState::Velocity))
                {
                    return Err(bad("replan target"));
                }
            }
            let s = w.heavy.len() as u32;
            w.heavy_of[k] = s;
            w.pos[k] = a.npos;
            w.vel[k] = a.vel;
            w.dvel[k] = a.dvel;
            w.set_dimensions(k, &a.params);
            w.heavy
                .try_reserve(1)
                .map_err(|_| Error::AllocationFailed)?;
            w.owner
                .try_reserve(1)
                .map_err(|_| Error::AllocationFailed)?;
            w.heavy.push(a);
            w.owner.push(k as u32);
            w.count += 1;
        }
        let q = &mut w.pathq;
        q.head = i.u32()? as usize;
        if q.head >= 8 {
            return Err(bad("queue head"));
        }
        q.next = i.u32()?;
        if q.next == 0 {
            return Err(bad("queue handle"));
        }
        let max_path = q.max_path;
        for r in &mut q.requests {
            r.handle = i.u32()?;
            if let Some(profiles) = profiles {
                r.profile = i.u32()?;
                if (r.handle == 0 && r.profile != 0)
                    || (r.handle != 0 && !profiles.contains_key(&r.profile))
                {
                    return Err(bad("queue profile"));
                }
            }
            r.start = i.r()?;
            r.end = i.r()?;
            r.start_pos = i.v3("queue request")?;
            r.end_pos = i.v3("queue request")?;
            r.filter = usize::from(i.u8()?);
            if r.filter >= 16 {
                return Err(bad("queue filter"));
            }
            r.status = i.u32()?;
            let primary = r.status & (Status::IN_PROGRESS | Status::SUCCESS | Status::FAILURE);
            if r.status != 0 && primary.count_ones() != 1 {
                return Err(bad("queue status"));
            }
            r.keep = i.u8()?;
            if r.keep > 3 || (r.handle != 0 && r.keep > 2) {
                return Err(bad("queue retention"));
            }
            let n = i.len(max_path, "queue path length")?;
            r.path.clear();
            for _ in 0..n {
                r.path.push(i.r()?);
            }
        }
        let mut running = None;
        for (index, r) in q.requests.iter().enumerate() {
            if r.handle != 0
                && q.requests[..index]
                    .iter()
                    .any(|other| other.handle == r.handle)
            {
                return Err(bad("duplicate queue handle"));
            }
            if r.handle != 0 && r.status & Status::IN_PROGRESS != 0 {
                if running.replace(index).is_some() || index != q.head % 8 {
                    return Err(bad("queue searches"));
                }
            }
        }
        if q.requests
            .iter()
            .any(|r| r.handle != 0 && r.handle == q.next)
        {
            return Err(bad("queue handle"));
        }
        read_search(&mut i, q, running, &mut ws)?;
        if let Some(mixed) = &w.mixed {
            for (a, &index) in w.heavy.iter().zip(&w.owner) {
                if a.target_state == TargetState::WaitingForPath
                    && let Some(r) = q
                        .requests
                        .iter()
                        .find(|r| r.handle != 0 && r.handle == a.pathq)
                    && r.profile != mixed.profiles[index as usize]
                {
                    return Err(bad("agent queue profile"));
                }
            }
        }
        if !i.0.is_empty() {
            return Err(Error::InvalidSnapshot(E::TrailingBytes));
        }
        // Versions 2 and 3 are read-only layouts that migrate to version 4. Every
        // other version must be the one the restored state encodes with.
        if !matches!(version, ARRIVAL_VERSION | LINKS_VERSION)
            && w.canonical_version(bound).0 != version
        {
            return Err(bad("layout version"));
        }
        Ok(w)
    }
}

fn has_running_search(queue: &PathQueue) -> bool {
    queue
        .requests
        .iter()
        .any(|r| r.handle != 0 && r.status & Status::IN_PROGRESS != 0)
}

fn write_search(o: &mut Out, queue: &PathQueue, bound: bool) {
    let running = has_running_search(queue);
    o.u8(u8::from(running));
    if !running {
        return;
    }
    let q = queue
        .query
        .as_ref()
        .expect("queue owns its query between updates");
    o.len(q.node_pool.used().len());
    for n in q.node_pool.used() {
        o.v3(n.pos);
        o.f32(n.cost);
        o.f32(n.total);
        o.u32(n.pidx);
        o.u8(n.state);
        o.u8(n.flags);
        o.r(n.id);
    }
    o.len(q.open_list.entries().len());
    for (node, total) in q.open_list.entries() {
        o.len(node);
        o.f32(total);
    }
    let d = &q.query;
    o.u32(d.status);
    o.u32(d.last_best_node.map_or(0, |n| n as u32 + 1));
    o.f32(d.last_best_node_cost);
    o.r(d.start_ref);
    o.r(d.end_ref);
    o.v3(d.end_pos);
    o.u32(d.options);
    o.f32(d.raycast_limit_sqr);
    o.u64(if bound { 0 } else { d.epoch });
}

fn read_search(
    i: &mut In<'_>,
    queue: &mut PathQueue,
    running: Option<usize>,
    ws: &mut Workspace,
) -> Result<(), Error> {
    use crate::detour::node::{NODE_OPEN, Node};
    use crate::detour::query::QueryData;
    if i.flag("search present")? != running.is_some() {
        return Err(bad("search present"));
    }
    let Some(slot) = running else {
        return Ok(());
    };
    let q = queue.query.as_mut().ok_or(bad("queue query"))?;
    let n = i.len(q.node_pool.max_nodes(), "search nodes")?;
    if n == 0 {
        return Err(bad("search nodes"));
    }
    ws.work(n as u64)?;
    ws.charge_bytes::<u8>(n * 2)?;
    q.node_pool.clear();
    for _ in 0..n {
        let node = Node {
            pos: i.v3("node position")?,
            cost: i.f32("node cost")?,
            total: i.f32("node total")?,
            pidx: i.u32()?,
            state: i.u8()?,
            flags: i.u8()?,
            id: i.r()?,
        };
        if node.pidx as usize > n
            || node.flags & !7 != 0
            || node.flags & 3 == 3
            || node.cost < 0.
            || node.total < node.cost
            || !q.node_pool.restore_node(node)
        {
            return Err(bad("search node"));
        }
    }
    for node in q.node_pool.used() {
        if let Some(parent) = node.pidx.checked_sub(1) {
            if q.node_pool.nodes[parent as usize].cost > node.cost {
                return Err(bad("node parent cost"));
            }
        }
    }
    // Validate parent chains in linear time before any path extraction can follow them.
    let mut color = ws.vec(n, 0u8)?;
    for start in 0..n {
        let mut at = Some(start);
        while let Some(j) = at {
            if color[j] == 2 {
                break;
            }
            if color[j] == 1 {
                return Err(bad("search parent cycle"));
            }
            color[j] = 1;
            at = q.node_pool.nodes[j].pidx.checked_sub(1).map(|v| v as usize);
        }
        let mut at = Some(start);
        while let Some(j) = at {
            if color[j] != 1 {
                break;
            }
            color[j] = 2;
            at = q.node_pool.nodes[j].pidx.checked_sub(1).map(|v| v as usize);
        }
    }
    let len = i.len(n, "search heap")?;
    let mut seen = ws.vec(n, false)?;
    q.open_list.clear();
    for _ in 0..len {
        let node = i.len(n - 1, "heap node")?;
        let total = i.f32("heap total")?;
        if seen[node]
            || q.node_pool.nodes[node].flags & NODE_OPEN == 0
            || total.to_bits() != q.node_pool.nodes[node].total.to_bits()
        {
            return Err(bad("search heap"));
        }
        seen[node] = true;
        q.open_list.restore_entry(node, total);
    }
    if !q.open_list.valid_heap() {
        return Err(bad("heap order"));
    }
    if q.node_pool
        .used()
        .iter()
        .enumerate()
        .any(|(j, node)| (node.flags & NODE_OPEN != 0) != seen[j])
    {
        return Err(bad("search heap membership"));
    }
    let status = i.u32()?;
    let best = i.len(n, "best node")?;
    let d = QueryData {
        status,
        last_best_node: best.checked_sub(1),
        last_best_node_cost: i.f32("best cost")?,
        start_ref: i.r()?,
        end_ref: i.r()?,
        end_pos: i.v3("search end")?,
        options: i.u32()?,
        raycast_limit_sqr: i.f32("raycast limit")?,
        epoch: i.u64()?,
    };
    let r = &queue.requests[slot];
    if d.options != 0
        || status != r.status
        || best == 0
        || d.start_ref != r.start
        || d.end_ref != r.end
        || d.end_pos.map(f32::to_bits) != r.end_pos.map(f32::to_bits)
    {
        return Err(bad("search state"));
    }
    q.query = d;
    Ok(())
}

fn read_world_params(i: &mut In<'_>) -> Result<CrowdWorldParams, Error> {
    let p = CrowdWorldParams {
        max_agents: i.u32()?,
        max_heavy_agents: i.u32()?,
        max_radius: i.f32("world parameters")?,
        path_queue: PathQueueParams {
            max_nodes: i.u32()? as usize,
            iterations: i.u32()? as usize,
        },
        fixed_dt: i.f32("world parameters")?,
    };
    p.validate().map_err(|_| bad("world parameters"))?;
    if i.u64()? != params_hash(&p) {
        return Err(bad("parameters hash"));
    }
    Ok(p)
}

fn params_hash(p: &CrowdWorldParams) -> u64 {
    let mut hash = 0xcbf29ce484222325u64;
    for word in [
        p.max_agents,
        p.max_heavy_agents,
        p.max_radius.to_bits(),
        p.path_queue.max_nodes as u32,
        p.path_queue.iterations as u32,
        p.fixed_dt.to_bits(),
    ] {
        for byte in word.to_le_bytes() {
            hash = (hash ^ u64::from(byte)).wrapping_mul(0x100000001b3);
        }
    }
    hash
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::detour::NavMeshParams;

    #[test]
    fn reads_both_legacy_layouts_and_rejects_unknown_versions() {
        let nav = NavMesh::new(&NavMeshParams {
            orig: [0.; 3],
            tile_width: 10.,
            tile_height: 10.,
            max_tiles: 4,
            max_polys: 32,
        })
        .unwrap();
        let mut w = CrowdWorld::new(&nav, &CrowdWorldParams::default(), Limits::UNLIMITED).unwrap();
        let params = AgentParams {
            arrival_policy: ArrivalPolicy::Park,
            arrival_distance: 0.75,
            off_mesh: OffMeshMode::External,
            ..AgentParams::default()
        };
        let h = w.add_agent(&nav, [0.; 3], params).unwrap();
        w.link_epoch = 17;

        for arrival in [true, false] {
            let mut bytes = w.write_snapshot(
                if arrival {
                    ARRIVAL_VERSION
                } else {
                    LINKS_VERSION
                },
                AgentLayout {
                    arrival,
                    links: !arrival,
                },
                false,
            );
            let mut restored = CrowdWorld::from_bytes(&nav, &bytes, Limits::UNLIMITED).unwrap();
            let agent = restored.heavy_mut(h).unwrap();
            if arrival {
                assert_eq!(agent.params.arrival_policy, ArrivalPolicy::Park);
                assert_eq!(agent.params.arrival_distance, 0.75);
                assert_eq!(agent.params.off_mesh, OffMeshMode::Animate);
                assert_eq!(restored.link_epoch, 0);
            } else {
                assert_eq!(agent.params.arrival_policy, ArrivalPolicy::Keep);
                assert_eq!(agent.params.off_mesh, OffMeshMode::External);
                assert_eq!(restored.link_epoch, 17);
            }
            assert_eq!(restored.to_bytes()[4..8], EXTENDED_VERSION.to_le_bytes());
            // Each layout has its own version: the other number is not a guess away.
            let (own, other) = if arrival {
                (ARRIVAL_VERSION, LINKS_VERSION)
            } else {
                (LINKS_VERSION, ARRIVAL_VERSION)
            };
            assert_eq!(bytes[4..8], own.to_le_bytes());
            let mut swapped = bytes.clone();
            swapped[4..8].copy_from_slice(&other.to_le_bytes());
            assert!(CrowdWorld::from_bytes(&nav, &swapped, Limits::UNLIMITED).is_err());
            bytes[4..8].copy_from_slice(&u32::MAX.to_le_bytes());
            assert!(matches!(
                CrowdWorld::from_bytes(&nav, &bytes, Limits::UNLIMITED),
                Err(Error::InvalidSnapshot(E::WrongVersion))
            ));
        }
    }
    fn empty_nav() -> NavMesh {
        NavMesh::new(&NavMeshParams {
            orig: [0.; 3],
            tile_width: 10.,
            tile_height: 10.,
            max_tiles: 4,
            max_polys: 32,
        })
        .unwrap()
    }

    #[test]
    fn rejects_a_fixes_word_the_writer_never_produces() {
        let nav = empty_nav();
        let mut w = CrowdWorld::new(&nav, &CrowdWorldParams::default(), Limits::UNLIMITED).unwrap();
        w.set_fixes(UpstreamFixes::ALL);
        let bytes = w.to_bytes();
        let back = CrowdWorld::from_bytes(&nav, &bytes, Limits::UNLIMITED).unwrap();
        assert_eq!(back.to_bytes(), bytes);
        // Version 1 header: magic, version, 24 bytes of parameters, their hash, then the fixes.
        let at = 8 + 24 + 8;
        assert_eq!(bytes[at..at + 4], UpstreamFixes::ALL.bits().to_le_bytes());
        let mut bad_bytes = bytes;
        bad_bytes[at..at + 4].copy_from_slice(&UpstreamFixes::NONE.bits().to_le_bytes());
        assert!(matches!(
            CrowdWorld::from_bytes(&nav, &bad_bytes, Limits::UNLIMITED),
            Err(Error::InvalidSnapshot(E::InvalidValue("fixes")))
        ));
    }

    #[test]
    fn rejects_a_current_version_the_state_does_not_encode_with() {
        let nav = empty_nav();
        let w = CrowdWorld::new(&nav, &CrowdWorldParams::default(), Limits::UNLIMITED).unwrap();
        let wide = AgentLayout {
            arrival: true,
            links: true,
        };
        for version in [EXTENDED_VERSION, REPLAN_VERSION] {
            let bytes = w.write_snapshot(version, wide, false);
            assert!(matches!(
                CrowdWorld::from_bytes(&nav, &bytes, Limits::UNLIMITED),
                Err(Error::InvalidSnapshot(E::InvalidValue("layout version")))
            ));
        }
    }
}
