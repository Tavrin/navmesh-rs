#![no_main]
use libfuzzer_sys::fuzz_target;
use navmesh_rs::crowd::{AgentId, AgentParams, Crowd};
use navmesh_rs::detour::{NavMesh, NavMeshCreateParams, create_nav_mesh_data};
use navmesh_rs::{Budget, Budgeted, Limits, Workspace};
fuzz_target!(|data: &[u8]| {
    let n = 0xffff;
    let Ok(tile) = create_nav_mesh_data(&NavMeshCreateParams {
        verts: &[0, 0, 0, 0, 0, 20, 20, 0, 20, 20, 0, 0],
        vert_count: 4,
        polys: &[0, 1, 2, 3, n, n, n, n, n, n, n, n],
        poly_count: 1,
        nvp: 6,
        poly_flags: &[1],
        poly_areas: &[0],
        bmin: [0.; 3],
        bmax: [20., 1., 20.],
        cs: 1.,
        ch: 1.,
        walkable_height: 2.,
        walkable_radius: 0.6,
        walkable_climb: 0.5,
        ..Default::default()
    }) else {
        return;
    };
    let Ok((nav, _)) = NavMesh::new_single(tile) else {
        return;
    };
    let mut ws = Workspace::with_limits(Limits {
        max_bytes: 4 * 1024 * 1024,
        max_work: 1_000_000,
        ..Limits::UNLIMITED
    });
    let Ok(mut crowd) = Crowd::new(&mut ws, &nav, 8, 0.6) else {
        return;
    };
    for chunk in data.as_chunks::<16>().0.iter().take(64) {
        let v = core::array::from_fn(|i| {
            f32::from_bits(u32::from_le_bytes(
                chunk[i * 4..i * 4 + 4].try_into().unwrap(),
            ))
        });
        let id = AgentId((chunk[12] % 8) as usize);
        match chunk[13] % 7 {
            0 => {
                let _ = crowd.add_agent(&mut ws, &nav, v, AgentParams::default());
            }
            1 => {
                let _ = crowd.request_move_velocity(id, v);
            }
            2 => {
                let r = crowd.agent(id).map_or(0, |a| a.corridor.first_poly());
                let _ = crowd.request_move_target(&nav, id, r, v);
            }
            3 => {
                let _ = crowd.remove_agent(id);
            }
            4 => {
                let _ = crowd.reset_move_target(id);
            }
            5 => {
                let _ = crowd.update(&mut ws, &nav, v[0]);
            }
            _ => {
                // A tick under a budget taken from the two tail bytes: 0 and 1 refuse
                // the up-front charge; larger values reach later phases.
                let t = u64::from(u16::from_le_bytes([chunk[14], chunk[15]]));
                let mut budget = Budget::new(if t < 4 { t } else { t * 256 });
                let r = crowd.update_budgeted(&mut ws, &nav, v[0], &mut budget);
                assert!(budget.used() <= budget.limit(), "budget overspent");
                if matches!(r, Ok(Budgeted::Exhausted(()))) {
                    assert!(budget.is_exhausted(), "exhausted without a refusal");
                }
            }
        }
        let _ = crowd.update(&mut ws, &nav, 1. / 30.);
    }
});
