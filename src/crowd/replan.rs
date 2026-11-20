//! Same-tick corridor repair after a mesh edit, shared by both crowds.
use super::*;
use AgentState as S;
use TargetState as T;

pub(super) const DEFAULT_LIMIT: u32 = 1024;

/// Walks active indices in order, starting where the previous tick's budget
/// ran out. The cursor is an agent index so removals never reset queue order.
pub(super) fn order(active: &[usize], cursor: u32) -> impl Iterator<Item = &usize> {
    let first = if cursor == 0 {
        0
    } else {
        active.partition_point(|&i| i < cursor as usize)
    };
    let (before, after) = active.split_at(first);
    after.iter().chain(before)
}

/// Returns true when this edit replaces the ordinary lookahead check.
pub(super) fn revalidate(
    a: &mut CrowdAgent,
    q: &mut NavMeshQuery<'_>,
    pathq: &mut PathQueue,
    filters: &[DefaultFilter; 16],
    extents: &V3,
    since: u64,
    remaining: &mut u32,
) -> Result<bool, Error> {
    if a.target_ref == 0 || matches!(a.target_state, T::None | T::Velocity) {
        return Ok(false);
    }
    let nav = q.nav();
    if !a.pending_stamp_replan {
        if nav.change_epoch() == since {
            return Ok(false);
        }
        // A failed or partial route can lack the tile that becomes passable.
        // After a detour, keep watching the first obstruction on the old route.
        let changed = |poly| poly != 0 && nav.corridor_tile_changed(poly, since);
        let mut last_slot = None;
        let affected = a.partial
            || a.target_state == T::Failed
            || changed(a.blocked_poly)
            || changed(a.target_ref)
            || a.corridor.path().iter().copied().any(|poly| {
                let slot = nav.decode_poly_id_tile(poly);
                let first = last_slot != Some(slot);
                last_slot = Some(slot);
                first && changed(poly)
            });
        if !affected {
            return Ok(false);
        }
        a.pending_stamp_replan = true;
        pathq.cancel(core::mem::take(&mut a.pathq));
    }
    if a.state == S::Offmesh {
        return Ok(true);
    }
    if *remaining == 0 {
        stop(a);
        return Ok(true);
    }
    *remaining -= 1;
    a.pending_stamp_replan = false;
    a.pending_link_replan = false;

    let filter = &filters[a.params.query_filter_type];
    let path = a.corridor.path();
    let unloaded = path
        .iter()
        .position(|&poly| matches!(nav.ref_state(poly), RefState::Unloaded(_)));
    let target_unloaded = match nav.ref_state(a.target_ref) {
        RefState::Unloaded(tile) => Some(tile),
        _ => None,
    };
    let obstruction = first_obstruction(&path[..unloaded.unwrap_or(path.len())], q, filter);
    // Keep ordinary streaming waits when the prefix is still usable.
    if obstruction.is_none() && (unloaded.is_some() || target_unloaded.is_some()) {
        return Ok(false);
    }
    if let Some(blocked) = obstruction {
        a.blocked_poly = blocked;
    } else if q.is_valid_poly_ref(a.blocked_poly, filter) {
        a.blocked_poly = 0;
    }

    let mut start = a.corridor.first_poly();
    if !q.is_valid_poly_ref(start, filter) {
        let (poly, pos, _) = q.find_nearest_poly(&a.npos, extents, filter)?;
        start = poly;
        if start != 0 {
            a.npos = pos;
        }
    }
    a.corridor.reset(start, a.npos)?;
    a.boundary.reset();
    a.partial = false;
    a.replan = false;
    a.replan_time = 0.;
    a.waiting_tile = None;
    a.state = if start == 0 { S::Invalid } else { S::Walking };

    if let Some(tile) = target_unloaded {
        a.target_state = T::WaitingForTile;
        a.waiting_tile = Some(tile);
        stop(a);
        return Ok(true);
    }

    // A closed but resident goal is kept for retry. Project a stale goal
    // onto the replacement geometry when possible.
    if !nav.is_valid_poly_ref(a.target_ref) {
        let (poly, pos, _) = q.find_nearest_poly(&a.target_pos, extents, filter)?;
        if poly != 0 {
            a.target_ref = poly;
            a.target_pos = pos;
        }
    }
    if start == 0 || !q.is_valid_poly_ref(a.target_ref, filter) {
        fail(a);
        return Ok(true);
    }

    let mut path = Vec::new();
    let status = q.find_path(
        start,
        a.target_ref,
        &a.npos,
        &a.target_pos,
        filter,
        &mut path,
        256,
    )?;
    let last = *path.last().ok_or(Error::Failed)?;
    if last == a.target_ref {
        a.corridor.set_corridor(a.target_pos, &path)?;
        a.target_state = T::Valid;
        // A stopped agent can already be on the reopened connection's first
        // portal, which the straight-path funnel skips at its start point.
        let start_tolerance = fmin(0.001, a.params.radius * 2.25);
        if let Some(&poly) = path.get(1)
            && let Ok((departure, _)) = nav.off_mesh_connection_poly_end_points(start, poly)
            && vdist_2d_sqr(&a.npos, &departure) < sqr(start_tolerance)
        {
            phases::start_offmesh(a, q, poly);
        }
    } else if status.has_detail(Status::UNLOADED_BORDER)
        && let Some(tile) = q.search_frontier().first().copied()
    {
        a.corridor
            .set_corridor(q.closest_point_on_poly(last, &a.target_pos)?.0, &path)?;
        a.target_state = T::WaitingForTile;
        a.waiting_tile = Some(tile);
    } else if status.has_detail(Status::OUT_OF_NODES | Status::BUFFER_TOO_SMALL) {
        a.corridor
            .set_corridor(q.closest_point_on_poly(last, &a.target_pos)?.0, &path)?;
        a.partial = true;
        a.target_state = T::WaitingForQueue;
    } else {
        fail(a);
    }
    Ok(true)
}

fn first_obstruction(
    path: &[PolyRef],
    q: &NavMeshQuery<'_>,
    filter: &DefaultFilter,
) -> Option<PolyRef> {
    // Prefer the actual filtered polygon over an off-mesh link that is
    // indirectly blocked by that polygon at its landing.
    if let Some(&poly) = path.iter().find(|&&poly| {
        !q.nav()
            .tile_and_poly_by_ref(poly)
            .is_ok_and(|(tile, p)| filter.pass_filter(poly, tile, p))
    }) {
        return Some(poly);
    }
    for pair in path.windows(2) {
        let (tile, polygon) = q.nav().tile_and_poly_by_ref(pair[0]).ok()?;
        let mut edge = polygon.first_link;
        loop {
            let Some(link) = tile.links.get(edge as usize) else {
                return Some(pair[0]);
            };
            if link.r#ref == pair[1] {
                break;
            }
            edge = link.next;
        }
    }
    None
}

fn stop(a: &mut CrowdAgent) {
    a.vel = [0.; 3];
    a.nvel = [0.; 3];
    a.dvel = [0.; 3];
    a.desired_speed = 0.;
    a.corners.clear();
    a.nneis = 0;
}

fn fail(a: &mut CrowdAgent) {
    a.target_state = T::Failed;
    if a.blocked_poly == 0 {
        a.blocked_poly = a.target_ref;
    }
    stop(a);
}
