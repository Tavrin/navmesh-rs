//! Matched fixture collection; the optional reference executable uses public APIs.
use crate::scenarios;
use navmesh_rs::Workspace;
use navmesh_rs::crowd::ProximityGrid;
use std::collections::VecDeque;
use std::io::{BufRead, BufReader, Write};
use std::process::{Command, Stdio};

type State = ([f32; 3], [f32; 3]);
struct Metrics {
    arrivals: Vec<Option<usize>>,
    penetrations: Vec<f32>,
    history: VecDeque<Vec<[f32; 3]>>,
    grid: ProximityGrid,
    ids: Vec<u16>,
    speed_violations: usize,
    max_speed_excess: f64,
    strict_speed_violations: usize,
}
impl Metrics {
    fn new(n: usize, ws: &mut Workspace) -> Result<Self, navmesh_rs::Error> {
        Ok(Self {
            arrivals: vec![None; n],
            penetrations: Vec::new(),
            history: VecDeque::new(),
            grid: ProximityGrid::new(ws, n * 4, 1.2)?,
            ids: vec![0; n],
            speed_violations: 0,
            max_speed_excess: 0.,
            strict_speed_violations: 0,
        })
    }
    fn observe(
        &mut self,
        states: &[State],
        goals: &[[f32; 3]],
        tick: usize,
        ws: &mut Workspace,
    ) -> Result<bool, navmesh_rs::Error> {
        self.grid.clear();
        for (i, (p, _)) in states.iter().enumerate() {
            self.grid
                .add_item(i as u16, [p[0] - 0.4, p[2] - 0.4, p[0] + 0.4, p[2] + 0.4])?;
        }
        for (i, (p, v)) in states.iter().enumerate() {
            let dx = p[0] - goals[i][0];
            let dz = p[2] - goals[i][2];
            if dx * dx + dz * dz < 0.25 && self.arrivals[i].is_none() {
                self.arrivals[i] = Some(tick);
            }
            let excess = f64::from(v[0]).hypot(f64::from(v[2])) - 2.;
            self.max_speed_excess = self.max_speed_excess.max(excess);
            self.speed_violations += usize::from(excess > 1e-6);
            self.strict_speed_violations += usize::from(excess > 0.);
            let k = self.grid.query_items(
                ws,
                [p[0] - 1., p[2] - 1., p[0] + 1., p[2] + 1.],
                &mut self.ids,
            )?;
            for &j in &self.ids[..k] {
                let j = usize::from(j);
                if j <= i {
                    continue;
                }
                let q = states[j].0;
                let dx = p[0] - q[0];
                let dz = p[2] - q[2];
                let penetration = 0.8 - (dx * dx + dz * dz).sqrt();
                if penetration > 0. {
                    self.penetrations.push(penetration);
                }
            }
        }
        self.history.push_back(states.iter().map(|s| s.0).collect());
        if self.history.len() > 401 {
            self.history.pop_front();
        }
        Ok(self.arrivals.iter().all(Option::is_some))
    }
    fn report(&mut self, name: &str, n: usize, mode: &str, ticks: usize, states: &[State]) {
        self.penetrations.sort_by(f32::total_cmp);
        let quantile = |p: f64| {
            self.penetrations
                .get(((self.penetrations.len() as f64 * p).ceil() as usize).saturating_sub(1))
                .copied()
                .unwrap_or(0.)
        };
        let mut arrivals: Vec<_> = self.arrivals.iter().flatten().copied().collect();
        arrivals.sort_unstable();
        let arrival = |p: f64| {
            let index = (n as f64 * p).ceil() as usize - 1;
            arrivals
                .get(index)
                .map_or("null".to_owned(), |t| format!("{:.2}", *t as f64 * 0.05))
        };
        let deadlocks = if self.history.len() == 401 {
            self.history.front().map_or(0, |old| {
                states
                    .iter()
                    .enumerate()
                    .filter(|(i, s)| {
                        let dx = s.0[0] - old[*i][0];
                        let dz = s.0[2] - old[*i][2];
                        self.arrivals[*i].is_none() && dx * dx + dz * dz < 0.0025
                    })
                    .count()
            })
        } else {
            0
        };
        println!(
            "{{\"scene\":\"{name}\",\"agents\":{n},\"mode\":\"{mode}\",\"ticks\":{ticks},\"positive_pair_ticks\":{},\"max_penetration\":{:.9},\"p99_positive_penetration\":{:.9},\"arrival_p50\":{},\"arrival_p95\":{},\"arrival_p99\":{},\"arrival_p100\":{},\"unreached\":{},\"deadlocks\":{deadlocks},\"speed_violations\":{},\"strict_speed_violations\":{},\"max_speed_excess\":{:.12}}}",
            self.penetrations.len(),
            quantile(1.),
            quantile(0.99),
            arrival(0.5),
            arrival(0.95),
            arrival(0.99),
            arrival(1.),
            n - arrivals.len(),
            self.speed_violations,
            self.strict_speed_violations,
            self.max_speed_excess
        );
    }
}
pub fn run() -> Result<(), Box<dyn std::error::Error>> {
    let oracle = std::env::args().nth(2);
    for (name, n, limit) in [
        ("circle", 64, 1400),
        ("circle", 512, 12000),
        ("crossing", 128, 2400),
        ("corridor", 64, 2400),
        ("doorway", 64, 1400),
        ("dense", 2000, 1400),
    ] {
        let mut s = scenarios::scene(name, n, true)?;
        let mut ws = Workspace::new();
        let mut metrics = Metrics::new(n, &mut ws)?;
        let mut states: Vec<State> = s.crowd.agents().iter().map(|a| (a.npos, a.vel)).collect();
        let mut child = if let Some(binary) = &oracle {
            Some(
                Command::new(binary)
                    .stdin(Stdio::piped())
                    .stdout(Stdio::piped())
                    .spawn()?,
            )
        } else {
            None
        };
        let mut reader = None;
        if let Some(c) = &mut child {
            let mut input = c.stdin.take().ok_or("reference stdin")?;
            writeln!(input, "{name} {n} {limit}")?;
            for (state, goal) in states.iter().zip(&s.goals) {
                writeln!(
                    input,
                    "{} {} {} {}",
                    state.0[0], state.0[2], goal[0], goal[2]
                )?;
            }
            drop(input);
            reader = Some(BufReader::new(c.stdout.take().ok_or("reference stdout")?));
        }
        let mut ticks = 0;
        for t in 1..=limit {
            if let Some(reader) = &mut reader {
                for state in &mut states {
                    let mut line = String::new();
                    if reader.read_line(&mut line)? == 0 {
                        return Err("short reference trajectory".into());
                    }
                    let values: Vec<f32> = line
                        .split_whitespace()
                        .map(str::parse)
                        .collect::<Result<_, _>>()?;
                    if values.len() != 4 {
                        return Err("invalid reference trajectory".into());
                    }
                    *state = ([values[0], 0., values[1]], [values[2], 0., values[3]]);
                }
            } else {
                scenarios::tick(&mut s, &mut ws)?;
                for (state, a) in states.iter_mut().zip(s.crowd.agents()) {
                    *state = (a.npos, a.vel);
                }
            }
            ticks = t;
            if metrics.observe(&states, &s.goals, t, &mut ws)? {
                break;
            }
        }
        // The reference produces the complete timeout stream. Drain before wait.
        if let Some(mut r) = reader {
            std::io::copy(&mut r, &mut std::io::sink())?;
        }
        if let Some(mut c) = child
            && !c.wait()?.success()
        {
            return Err("reference failed".into());
        }
        metrics.report(
            name,
            n,
            if oracle.is_some() {
                "reference"
            } else {
                "candidate"
            },
            ticks,
            &states,
        );
    }
    Ok(())
}
