//! Paper-derived reciprocal half-planes and incremental linear programs.
//!
//! Source: van den Berg et al., “Reciprocal n-Body Collision Avoidance” (2011),
//! sections 4.2, 5.1, 5.3 and 5.4, doi:10.1007/978-3-642-19457-3_1.
//! No reference-library implementation is used here.
use super::*;
type V2 = [f32; 2];
const EPS: f32 = 0.000001;

/// Local velocity planner. The default preserves the existing sampled planner.
#[derive(Clone, Copy, Debug, Default, PartialEq)]
#[non_exhaustive]
pub enum AvoidanceMode {
    /// Use [`AgentParams::obstacle_avoidance_type`] to select the sampled preset.
    #[default]
    Sampled,
    /// Use reciprocal velocity half-planes.
    Orca(OrcaParams),
}

/// ORCA horizons and neighbour cap. Neither horizon may be zero or non-finite.
#[derive(Clone, Copy, Debug, PartialEq)]
#[non_exhaustive]
pub struct OrcaParams {
    /// Prediction horizon for other agents, in seconds.
    pub time_horizon: f32,
    /// Prediction horizon for two-sided static segments, in seconds.
    pub time_horizon_obstacles: f32,
    /// Nearest agents retained, in ascending (distance squared, input index) order.
    pub max_neighbours: u16,
}
impl Default for OrcaParams {
    fn default() -> Self {
        Self {
            time_horizon: 2.,
            time_horizon_obstacles: 1.,
            max_neighbours: 16,
        }
    }
}
impl OrcaParams {
    pub(super) fn validate(&self) -> Result<(), Error> {
        positive(self.time_horizon)?;
        positive(self.time_horizon_obstacles)?;
        if self.max_neighbours == 0 {
            return Err(Error::InvalidParameter("ORCA neighbour cap"));
        }
        Ok(())
    }
}

/// Observable planar agent state and its preferred velocity.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct OrcaAgent {
    /// Planar position (x, z).
    pub position: V2,
    /// Current velocity (x, z).
    pub velocity: V2,
    /// Preferred velocity (x, z), clipped to `max_speed` by the solver.
    pub preferred_velocity: V2,
    /// Positive disc radius.
    pub radius: f32,
    /// Positive speed limit.
    pub max_speed: f32,
}
impl OrcaAgent {
    fn validate(&self) -> Result<(), Error> {
        positive(self.radius)?;
        positive(self.max_speed)?;
        if self
            .position
            .iter()
            .chain(&self.velocity)
            .chain(&self.preferred_velocity)
            .any(|v| !v.is_finite() || v.abs() > 1.0e6)
            || self.radius > 1.0e6
            || self.max_speed > 1.0e6
        {
            return Err(Error::InvalidParameter("ORCA finite bounded state"));
        }
        Ok(())
    }
    pub(super) fn crowd(a: &CrowdAgent) -> Self {
        Self {
            position: [a.npos[0], a.npos[2]],
            velocity: [a.vel[0], a.vel[2]],
            preferred_velocity: [a.dvel[0], a.dvel[2]],
            radius: a.params.radius,
            max_speed: a.params.max_speed,
        }
    }
}

// Feasible side: n.v >= c; n is a unit outward normal.
#[derive(Clone, Copy, Debug)]
struct Plane {
    n: V2,
    c: f32,
}
/// Reusable solver storage. Capacity growth is fallible and charged to the workspace.
#[derive(Default)]
pub struct OrcaScratch {
    planes: Vec<Plane>,
    projected: Vec<Plane>,
    order: Vec<(usize, f32)>,
}
fn add(a: V2, b: V2) -> V2 {
    [a[0] + b[0], a[1] + b[1]]
}
fn sub(a: V2, b: V2) -> V2 {
    [a[0] - b[0], a[1] - b[1]]
}
fn mul(a: V2, s: f32) -> V2 {
    [a[0] * s, a[1] * s]
}
fn dot(a: V2, b: V2) -> f32 {
    a[0] * b[0] + a[1] * b[1]
}
fn det(a: V2, b: V2) -> f32 {
    a[0] * b[1] - a[1] * b[0]
}
fn norm(a: V2) -> f32 {
    sqrtf(dot(a, a))
}
fn unit(a: V2, fallback: V2) -> V2 {
    let l = norm(a);
    if l > 0. { mul(a, 1. / l) } else { fallback }
}
fn clip(v: V2, speed: f32) -> V2 {
    if dot(v, v) > speed * speed {
        mul(unit(v, [1., 0.]), speed)
    } else {
        v
    }
}

// Eq. (5): nearest point on a disc's truncated velocity cone. In overlap,
// use the next integration step rather than the prediction horizon.
fn agent_plane(a: &OrcaAgent, b: &OrcaAgent, horizon: f32, dt: f32) -> Plane {
    let p = sub(b.position, a.position);
    let v = sub(a.velocity, b.velocity);
    let r = a.radius + b.radius;
    let d2 = dot(p, p);
    let (n, u) = if d2 > r * r {
        let w = sub(v, mul(p, 1. / horizon));
        let dp = dot(w, p);
        if dp < 0. && dp * dp > r * r * dot(w, w) {
            let n = unit(w, [1., 0.]);
            (n, mul(n, r / horizon - norm(w)))
        } else {
            let leg = sqrtf((d2 - r * r).max(0.));
            let t = if det(p, w) > 0. {
                [(p[0] * leg - p[1] * r) / d2, (p[0] * r + p[1] * leg) / d2]
            } else {
                [-(p[0] * leg + p[1] * r) / d2, (p[0] * r - p[1] * leg) / d2]
            };
            let n = [-t[1], t[0]];
            (n, sub(mul(t, dot(v, t)), v))
        }
    } else {
        let w = sub(v, mul(p, 1. / dt));
        // Coincident stationary centres have no observable separating direction.
        // The deterministic axis is a recovery policy, not a collision guarantee.
        let n = unit(w, unit(mul(p, -1.), [1., 0.]));
        (n, mul(n, r / dt - norm(w)))
    };
    Plane {
        n,
        c: dot(n, add(a.velocity, mul(u, 0.5))),
    }
}

// Section 5.4 explicitly chooses optimization velocity zero for walls. Its
// nearest capsule boundary is the closest segment point, expanded by the radius.
// Reversing the endpoints therefore leaves the half-plane unchanged.
fn wall_plane(a: &OrcaAgent, wall: [f32; 4], horizon: f32, dt: f32) -> Plane {
    let start = [wall[0], wall[1]];
    let edge = sub([wall[2], wall[3]], start);
    let d2 = dot(edge, edge);
    let t = if d2 > 0. {
        (dot(sub(a.position, start), edge) / d2).clamp(0., 1.)
    } else {
        0.
    };
    let away = sub(a.position, add(start, mul(edge, t)));
    let d = norm(away);
    let n = unit(away, [1., 0.]);
    Plane {
        n,
        c: (a.radius - d) / if d < a.radius { dt } else { horizon },
    }
}

// Incremental 2D LP. A violated boundary reduces the problem to an interval
// on that line; intersect every earlier half-plane before selecting its optimum.
fn solve(
    ws: &mut Workspace,
    planes: &[Plane],
    speed: f32,
    objective: V2,
    directional: bool,
) -> Result<(V2, usize), Error> {
    let mut v = if directional {
        mul(objective, speed)
    } else {
        clip(objective, speed)
    };
    for (i, p) in planes.iter().enumerate() {
        ws.work(1)?;
        if dot(p.n, v) >= p.c {
            continue;
        }
        ws.work(1)?; // pivot, before constructing the interval
        let disc = speed * speed - p.c * p.c;
        if disc < 0. {
            return Ok((v, i));
        }
        let base = mul(p.n, p.c);
        let tangent = [-p.n[1], p.n[0]];
        let mut lo = -sqrtf(disc);
        let mut hi = -lo;
        for q in &planes[..i] {
            ws.work(1)?;
            let slope = dot(q.n, tangent);
            let need = q.c - dot(q.n, base);
            if slope.abs() < EPS {
                if need > 0. {
                    return Ok((v, i));
                }
            } else if slope > 0. {
                lo = lo.max(need / slope);
            } else {
                hi = hi.min(need / slope);
            }
            if lo > hi {
                return Ok((v, i));
            }
        }
        let t = if directional {
            if dot(objective, tangent) > 0. { hi } else { lo }
        } else {
            dot(sub(objective, base), tangent).clamp(lo, hi)
        };
        v = add(base, mul(tangent, t));
    }
    Ok((v, planes.len()))
}

/// Selects a planar velocity using the paper's reciprocal half-planes.
///
/// `dt` must be positive; it controls overlap recovery. Walls are two-sided
/// `[start_x, start_z, end_x, end_z]` segments, in caller order. Neighbours are
/// sorted by (distance squared, slice index) and capped by `max_neighbours`.
/// Non-finite inputs, magnitudes above `1e6`, horizons outside `1e-5..=1e5`,
/// and steps outside that interval are rejected to avoid arithmetic overflow.
///
/// Returns a speed-bounded velocity, including when constraints are infeasible.
/// In that case the 3D epigraph LP minimizes the worst agent-plane violation;
/// static planes retain priority. Acceleration limits, missing neighbours and
/// infeasible constraints prevent an unconditional nonpenetration guarantee.
/// The workspace bounds storage, candidate visits, constraint additions and LP
/// pivots. On an error no velocity is returned; scratch may contain partial work.
pub fn orca_velocity(
    ws: &mut Workspace,
    agent: &OrcaAgent,
    neighbours: &[OrcaAgent],
    walls: &[[f32; 4]],
    params: &OrcaParams,
    dt: f32,
    scratch: &mut OrcaScratch,
) -> Result<V2, Error> {
    ws.begin();
    velocity_in(ws, agent, neighbours, walls, params, dt, scratch)
}
pub(super) fn velocity_in(
    ws: &mut Workspace,
    a: &OrcaAgent,
    neighbours: &[OrcaAgent],
    walls: &[[f32; 4]],
    params: &OrcaParams,
    dt: f32,
    s: &mut OrcaScratch,
) -> Result<V2, Error> {
    a.validate()?;
    params.validate()?;
    for t in [dt, params.time_horizon, params.time_horizon_obstacles] {
        if !(1.0e-5..=1.0e5).contains(&t) {
            return Err(Error::InvalidParameter("ORCA time range"));
        }
    }
    s.planes.clear();
    s.projected.clear();
    s.order.clear();
    let cap = usize::from(params.max_neighbours).min(neighbours.len());
    ws.reserve(&mut s.order, cap)?;
    for (i, b) in neighbours.iter().enumerate() {
        ws.work(1)?;
        b.validate()?;
        let distance = dot(sub(a.position, b.position), sub(a.position, b.position));
        let at = s
            .order
            .partition_point(|&(j, d)| d < distance || (d == distance && j < i));
        if at < cap {
            if s.order.len() == cap {
                s.order.pop();
            }
            s.order.insert(at, (i, distance));
        }
    }
    let count = walls.len().checked_add(cap).ok_or(Error::SizeOverflow)?;
    ws.reserve(&mut s.planes, count)?;
    ws.reserve(&mut s.projected, count)?;
    for &wall in walls {
        ws.work(1)?;
        if wall.iter().any(|v| !v.is_finite() || v.abs() > 1.0e6) {
            return Err(Error::InvalidParameter("ORCA wall coordinates"));
        }
        s.planes
            .push(wall_plane(a, wall, params.time_horizon_obstacles, dt));
    }
    let static_count = s.planes.len();
    for &(i, _) in &s.order {
        ws.work(1)?;
        s.planes
            .push(agent_plane(a, &neighbours[i], params.time_horizon, dt));
    }
    let (mut v, failed) = solve(ws, &s.planes, a.max_speed, a.preferred_velocity, false)?;
    let mut violation = 0.;
    // Section 5.3: on plane i of the (vx,vz,slack) epigraph, substituting
    // slack=c_i-n_i.v projects earlier planes to (n_j-n_i).v >= c_j-c_i.
    for i in failed..s.planes.len() {
        ws.work(1)?;
        let p = s.planes[i];
        if p.c - dot(p.n, v) <= violation {
            continue;
        }
        ws.work(1)?;
        s.projected.clear();
        s.projected
            .extend_from_slice(&s.planes[..static_count.min(i)]);
        for q in &s.planes[static_count.min(i)..i] {
            ws.work(1)?;
            let diff = sub(q.n, p.n);
            let length = norm(diff);
            if length > EPS {
                s.projected.push(Plane {
                    n: mul(diff, 1. / length),
                    c: (q.c - p.c) / length,
                });
            }
        }
        let (candidate, end) = solve(ws, &s.projected, a.max_speed, p.n, true)?;
        if end == s.projected.len() {
            v = candidate;
        }
        violation = p.c - dot(p.n, v);
    }
    v = clip(v, a.max_speed);
    if v.iter().any(|x| !x.is_finite()) {
        return Err(Error::InvalidInput("ORCA arithmetic range"));
    }
    Ok(v)
}
