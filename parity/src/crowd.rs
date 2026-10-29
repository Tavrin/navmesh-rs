//! Crowd trajectory records and their tolerance rule.
//!
//! On the oracle platform crowd records are compared bit for bit like every
//! other record. Under the cross-platform `libm` contract (`reference/RECORDS.md`,
//! "What must be exact") the obstacle-avoidance `sinf`/`cosf` may differ in the
//! last bit, after which trajectories diverge. The rule is then: the first
//! [`EXACT_TICKS`] ticks bit-exact; afterwards the same agents active per tick,
//! identical agent and target states, and each agent's position within one cell
//! size of the reference.

use crate::decode::{Cursor, DecodeResult};

/// Ticks that must stay bit-exact under the tolerance rule.
pub const EXACT_TICKS: u32 = 30;

/// One active agent in one tick.
#[derive(Clone, Debug, PartialEq)]
pub struct Agent {
    /// Agent slot index.
    pub index: u32,
    /// `dtCrowdAgent::state`.
    pub state: u8,
    /// `dtCrowdAgent::targetState`.
    pub target_state: u8,
    /// Position after the tick.
    pub npos: [f32; 3],
    /// Actual velocity.
    pub vel: [f32; 3],
    /// Velocity chosen by obstacle avoidance.
    pub nvel: [f32; 3],
    /// First polygon of the path corridor.
    pub first_poly: u64,
    /// Number of steering corners.
    pub corners: i32,
    /// Every polygon of the path corridor.
    pub corridor: Vec<u64>,
}

/// All active agents after one tick.
#[derive(Clone, Debug, PartialEq)]
pub struct Tick {
    /// Tick number, from 0.
    pub tick: u32,
    /// Active agents in slot order.
    pub agents: Vec<Agent>,
}

/// Parses a crowd record payload whose poly refs are `ref_bits` wide.
pub fn parse(payload: &[u8], ref_bits: u32) -> DecodeResult<Vec<Tick>> {
    let mut c = Cursor::new(payload).with_ref_bits(ref_bits);
    let n = c.count_u32("tick_count", 8)?;
    let mut ticks = Vec::with_capacity(n);
    c.array("ticks", n, |c, _| {
        let tick = c.u32("tick")?;
        let na = c.count_u32("active", 50)?;
        let mut agents = Vec::with_capacity(na);
        c.array("agents", na, |c, _| {
            agents.push(Agent {
                index: c.u32("index")?,
                state: c.u8("state")?,
                target_state: c.u8("target_state")?,
                npos: c.v3("npos")?,
                vel: c.v3("vel")?,
                nvel: c.v3("nvel")?,
                first_poly: c.poly_ref("first_poly")?,
                corners: c.i32("corners")?,
                corridor: {
                    let n = c.count_u32("corridor_count", 4)?;
                    let mut path = Vec::with_capacity(n);
                    c.array("corridor", n, |c, _| {
                        path.push(c.poly_ref("poly")?);
                        Ok(())
                    })?;
                    path
                },
            });
            Ok(())
        })?;
        ticks.push(Tick { tick, agents });
        Ok(())
    })?;
    c.finish()?;
    Ok(ticks)
}

fn bits(v: [f32; 3]) -> [u32; 3] {
    v.map(f32::to_bits)
}

fn bit_equal(a: &Agent, b: &Agent) -> bool {
    a.index == b.index
        && a.state == b.state
        && a.target_state == b.target_state
        && bits(a.npos) == bits(b.npos)
        && bits(a.vel) == bits(b.vel)
        && bits(a.nvel) == bits(b.nvel)
        && a.first_poly == b.first_poly
        && a.corners == b.corners
        && a.corridor == b.corridor
}

/// Outcome of a tolerance comparison: the largest position error seen, or the
/// first violation.
#[derive(Clone, Debug, PartialEq)]
pub struct Tolerance {
    /// Largest per-agent position error after the exact ticks.
    pub max_error: f32,
    /// Whether any field differed at all (false means bit-identical).
    pub any_difference: bool,
}

/// Whether a cell size can bound a position error: finite and positive.
pub fn valid_cell_size(cell_size: f32) -> bool {
    cell_size.is_finite() && cell_size > 0.0
}

/// Applies the tolerance rule; `Err` names the first violation. A cell size that
/// is not finite and positive is itself an error (a NaN bound would pass every
/// position difference).
pub fn compare_with_tolerance(a: &[Tick], b: &[Tick], cell_size: f32) -> Result<Tolerance, String> {
    if !valid_cell_size(cell_size) {
        return Err(format!("cell size {cell_size} is not finite and positive"));
    }
    if a.len() != b.len() {
        return Err(format!("tick count {} vs {}", a.len(), b.len()));
    }
    let mut out = Tolerance {
        max_error: 0.0,
        any_difference: false,
    };
    for (ta, tb) in a.iter().zip(b) {
        if ta.tick != tb.tick {
            return Err(format!("tick numbers {} vs {}", ta.tick, tb.tick));
        }
        let t = ta.tick;
        if ta.agents.len() != tb.agents.len() {
            return Err(format!(
                "tick {t}: {} vs {} active agents",
                ta.agents.len(),
                tb.agents.len()
            ));
        }
        for (aa, ab) in ta.agents.iter().zip(&tb.agents) {
            if aa.index != ab.index {
                return Err(format!(
                    "tick {t}: active agent {} vs {}",
                    aa.index, ab.index
                ));
            }
            let i = aa.index;
            if bit_equal(aa, ab) {
                continue;
            }
            out.any_difference = true;
            if t < EXACT_TICKS {
                return Err(format!(
                    "tick {t} agent {i}: differs within the first {EXACT_TICKS} exact ticks"
                ));
            }
            if aa.state != ab.state || aa.target_state != ab.target_state {
                return Err(format!(
                    "tick {t} agent {i}: state {}/{} vs {}/{} (agent/target)",
                    aa.state, aa.target_state, ab.state, ab.target_state
                ));
            }
            let d = [
                aa.npos[0] - ab.npos[0],
                aa.npos[1] - ab.npos[1],
                aa.npos[2] - ab.npos[2],
            ];
            let err = (d[0] * d[0] + d[1] * d[1] + d[2] * d[2]).sqrt();
            if err.is_nan() || err >= cell_size {
                return Err(format!(
                    "tick {t} agent {i}: position error {err} is not under the cell size {cell_size}"
                ));
            }
            out.max_error = out.max_error.max(err);
        }
    }
    Ok(out)
}
