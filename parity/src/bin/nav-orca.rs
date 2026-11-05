//! Independent ORCA qualification driver: frozen-state protocol and trajectory digest.
use navmesh_rs::Workspace;
use navmesh_rs::crowd::{OrcaAgent, OrcaParams, OrcaScratch, orca_velocity};
use std::io::{self, Read};
fn next<T: std::str::FromStr>(tokens: &mut std::str::SplitWhitespace<'_>) -> T {
    tokens
        .next()
        .and_then(|s| s.parse().ok())
        .unwrap_or_else(|| panic!("invalid protocol input"))
}
#[path = "../orca_scenarios.rs"]
mod scenarios;
fn main() {
    match std::env::args().nth(1).as_deref() {
        Some("digest") => {
            scenarios::digest().unwrap();
            return;
        }
        Some("measure") => {
            scenarios::measure().unwrap();
            return;
        }
        Some("invariants") => {
            scenarios::invariants().unwrap();
            return;
        }
        _ => {}
    }
    let mut input = String::new();
    io::stdin().read_to_string(&mut input).unwrap();
    let mut tokens = input.split_whitespace();
    let mut ws = Workspace::new();
    let mut scratch = OrcaScratch::default();
    while let Some(n) = tokens.next() {
        let n: usize = n.parse().unwrap();
        let m: usize = next(&mut tokens);
        let dt = next(&mut tokens);
        let mut p = OrcaParams::default();
        p.time_horizon = next(&mut tokens);
        p.time_horizon_obstacles = next(&mut tokens);
        p.max_neighbours = next(&mut tokens);
        let agents: Vec<_> = (0..n)
            .map(|_| OrcaAgent {
                position: [next(&mut tokens), next(&mut tokens)],
                velocity: [next(&mut tokens), next(&mut tokens)],
                preferred_velocity: [next(&mut tokens), next(&mut tokens)],
                radius: next(&mut tokens),
                max_speed: next(&mut tokens),
            })
            .collect();
        let walls: Vec<_> = (0..m)
            .map(|_| {
                [
                    next(&mut tokens),
                    next(&mut tokens),
                    next(&mut tokens),
                    next(&mut tokens),
                ]
            })
            .collect();
        for (i, a) in agents.iter().enumerate() {
            let ns: Vec<_> = agents
                .iter()
                .enumerate()
                .filter_map(|(j, b)| (i != j).then_some(*b))
                .collect();
            let v = orca_velocity(&mut ws, a, &ns, &walls, &p, dt, &mut scratch).unwrap();
            println!("{:.9} {:.9}", v[0], v[1]);
        }
    }
}
