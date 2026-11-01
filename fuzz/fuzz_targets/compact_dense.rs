#![no_main]
//! Dense single-layer compact-field validation: the same small mesh run through
//! the Recast pipeline under unlimited work (the one-pass bit-field validation)
//! and under a very large finite work limit (the generic validation). The polygon
//! and detail meshes, or the error, must be identical. Input layout is
//! `navmesh_fuzz::RecastInput`; inputs whose grid or triangle count is large are
//! skipped so unlimited work stays fast.
use libfuzzer_sys::fuzz_target;
use navmesh_fuzz::{RECAST_LIMITS, RecastInput, run_recast_with};
use navmesh_rs::{Limits, recast};

fuzz_target!(|data: &[u8]| {
    let Some(input) = RecastInput::decode(data) else {
        return;
    };
    if input.tris.len() / 3 > 300 || input.verts.len() / 3 > 600 {
        return;
    }
    let cs = (f32::from(input.settings[1]) + 1.0) / 100.0;
    let (bmin, bmax) = recast::calc_bounds(&input.verts);
    if !bmin.iter().chain(&bmax).all(|v| v.is_finite()) {
        return;
    }
    let (w, h) = recast::calc_grid_size(&bmin, &bmax, cs);
    if w <= 0 || h <= 0 || i64::from(w) * i64::from(h) > 16_384 {
        return;
    }
    let dense = Limits {
        max_work: u64::MAX,
        ..RECAST_LIMITS
    };
    let generic = Limits {
        max_work: 1 << 40,
        ..RECAST_LIMITS
    };
    match (
        run_recast_with(&input, dense),
        run_recast_with(&input, generic),
    ) {
        (Ok((ra, a)), Ok((rb, b))) => {
            assert_eq!(ra, rb, "reached stage differs");
            assert_eq!(a, b, "meshes differ between dense and generic validation");
        }
        (Err(a), Err(b)) => assert_eq!(
            std::mem::discriminant(&a),
            std::mem::discriminant(&b),
            "{a:?} vs {b:?}"
        ),
        (a, b) => panic!(
            "dense and generic validation disagree: {:?} vs {:?}",
            a.map(|x| x.0),
            b.map(|x| x.0)
        ),
    }
});
