//! Bit-for-bit comparison of a crowd world with a crowd driven by the same inputs.

use navmesh_rs::crowd::Crowd;
use navmesh_rs::crowd::world::CrowdWorld;

fn bits(v: [f32; 3]) -> [u32; 3] {
    v.map(f32::to_bits)
}

/// Compares every active agent of `crowd` with the world agent at the same index:
/// states, position, the three velocities, corridor, corner count and neighbours,
/// bit for bit. Returns the number of agents compared, or the first difference.
pub fn compare(crowd: &Crowd, world: &CrowdWorld) -> Result<usize, String> {
    let mut a = crowd.agents().iter().enumerate().filter(|(_, a)| a.active);
    let mut b = world.agents();
    let mut n = 0;
    loop {
        match (a.next(), b.next()) {
            (None, None) => return Ok(n),
            (Some((i, x)), Some(v)) => {
                if i as u32 != v.handle().index() {
                    return Err(format!("agent index {i} vs {}", v.handle().index()));
                }
                let y = v.detail().ok_or("agent without full state")?;
                let neighbours = |n: &[(usize, f32)]| -> Vec<(usize, u32)> {
                    n.iter().map(|&(j, d)| (j, d.to_bits())).collect()
                };
                let field = if x.state != y.state || x.target_state != y.target_state {
                    "state"
                } else if bits(x.npos) != bits(y.npos) {
                    "position"
                } else if bits(x.vel) != bits(y.vel) {
                    "velocity"
                } else if bits(x.nvel) != bits(y.nvel) {
                    "avoidance velocity"
                } else if bits(x.dvel) != bits(y.dvel) {
                    "desired velocity"
                } else if x.corridor.path() != y.corridor.path() {
                    "corridor"
                } else if x.corners.len() != y.corners.len() {
                    "corners"
                } else if neighbours(x.neighbours()) != neighbours(y.neighbours()) {
                    "neighbours"
                } else {
                    n += 1;
                    continue;
                };
                return Err(format!("agent {i}: {field} differs"));
            }
            _ => return Err("active agent counts differ".into()),
        }
    }
}
