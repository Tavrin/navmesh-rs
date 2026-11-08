//! The `'DCRW'` snapshot: a little-endian byte image of a [`CrowdWorld`] for
//! replays and saves (not in recastnavigation).
//!
//! Layout, version 1: magic `DCRW`, version, world parameters, fixes, the sixteen
//! filters, the eight avoidance presets, the generation of every agent index, then
//! each agent's full state in index order, then the path queue's eight request
//! slots. Every float is stored as its bits, so a restored world steps bit for bit
//! like the original, including the node pool and heap of a sliced path search in progress.
use super::*;
use crate::SnapshotError as E;
use crate::detour::Status;

const MAGIC: &[u8; 4] = b"DCRW";
const VERSION: u32 = 1;

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

fn write_agent(o: &mut Out, a: &CrowdAgent) {
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
}

fn read_agent(
    i: &mut In<'_>,
    a: &mut CrowdAgent,
    agents: usize,
    max_radius: f32,
    ws: &mut Workspace,
) -> Result<(), Error> {
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
    a.active = true;
    Ok(())
}

impl CrowdWorld {
    /// The world as `'DCRW'` snapshot bytes (see the format in the module source).
    /// In-progress searches retain their node pool and heap for exact continuation.
    pub fn to_bytes(&self) -> Vec<u8> {
        let mut o = Out(Vec::new());
        o.bytes(MAGIC);
        o.u32(VERSION);
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
        for &s in &self.heavy_of {
            if s != NONE {
                write_agent(&mut o, &self.heavy[s as usize]);
            }
        }
        let q = &self.pathq;
        o.u32((q.head % 8) as u32);
        o.u32(q.next);
        for r in &q.requests {
            o.u32(r.handle);
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
        write_search(&mut o, q);
        o.0
    }

    /// Restores a world from untrusted `'DCRW'` bytes for agents on `nav`. Rejects
    /// malformed input with [`Error::InvalidSnapshot`]; `limits` bound the
    /// restored world's allocation as in [`CrowdWorld::new`].
    pub fn from_bytes(nav: &NavMesh, b: &[u8], limits: Limits) -> Result<Self, Error> {
        let mut i = In(b);
        if &i.take::<4>()? != MAGIC {
            return Err(Error::InvalidSnapshot(E::WrongMagic));
        }
        if i.u32()? != VERSION {
            return Err(Error::InvalidSnapshot(E::WrongVersion));
        }
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
        let mut w = CrowdWorld::new(nav, &p, limits)?;
        w.set_fixes(UpstreamFixes::from_bits(i.u32()?).ok_or(bad("fixes"))?);
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
            let mut a = CrowdAgent::new_in(&mut ws)?;
            read_agent(&mut i, &mut a, n, p.max_radius, &mut ws)?;
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
        read_search(&mut i, q, running, &mut ws)?;
        if !i.0.is_empty() {
            return Err(Error::InvalidSnapshot(E::TrailingBytes));
        }
        Ok(w)
    }
}

fn write_search(o: &mut Out, queue: &PathQueue) {
    let running = queue
        .requests
        .iter()
        .any(|r| r.handle != 0 && r.status & Status::IN_PROGRESS != 0);
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
    o.u64(d.epoch);
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
