//! Comparator behaviour on synthetic record files: identical files pass; a
//! flipped bit, a missing record, an out-of-tolerance crowd position, a bad
//! digest and a malformed layout each fail.

use navmesh_parity::diff::{self, Options, Outcome};
use navmesh_parity::encode::{Payload, RecordFileWriter};
use navmesh_parity::schema::{self, KINDS};
use navmesh_parity::{RecordFile, compare};

const META: &[(&str, &str)] = &[
    ("producer", "test"),
    ("upstream", "9f4ce64458dfae86e1239c525ddc219c4e9e06f1"),
    ("case", "synthetic.case"),
];
const CELL: f32 = 0.3;

fn config() -> Vec<u8> {
    config_with_cs(CELL)
}

fn config_with_cs(cs: f32) -> Vec<u8> {
    let mut p = Payload::new();
    p.i32(100).i32(80).i32(0).i32(0);
    p.f32(cs)
        .f32(0.2)
        .v3([0.0; 3])
        .v3([30.0, 5.0, 24.0])
        .f32(45.0);
    p.i32(10)
        .i32(4)
        .i32(2)
        .i32(40)
        .f32(1.3)
        .i32(64)
        .i32(400)
        .i32(6)
        .f32(1.8)
        .f32(0.2);
    p.into_bytes()
}

/// A poly mesh with 4 vertices and 2 polygons of 6 slots each.
fn poly_mesh() -> Vec<u8> {
    let mut p = Payload::new();
    p.i32(4).i32(2).i32(6);
    p.v3([0.0; 3]).v3([30.0, 5.0, 24.0]).f32(CELL).f32(0.2);
    p.i32(0).f32(1.3);
    for v in 0..4u16 {
        p.u16(v * 3).u16(1).u16(v * 5);
    }
    for poly in 0..2u16 {
        for slot in 0..12u16 {
            p.u16(if slot < 3 { poly + slot } else { 0xffff });
        }
        p.u16(poly + 1).u16(1).u8(0);
    }
    p.into_bytes()
}

/// Two agents walking along x; `bump` moves agent 1's position at one tick.
fn crowd(ticks: u32, bump: Option<(u32, f32)>) -> Vec<u8> {
    let mut p = Payload::new();
    p.u32(ticks);
    for t in 0..ticks {
        p.u32(t).u32(2);
        for i in 0..2u32 {
            let mut x = t as f32 * 0.1 + i as f32;
            if let Some((bt, dx)) = bump
                && i == 1
                && t == bt
            {
                x += dx;
            }
            p.u32(i).u8(1).u8(3);
            p.v3([x, 0.5, 2.0]).v3([3.0, 0.0, 0.0]).v3([3.0, 0.0, 0.0]);
            p.u32(0x1000 + i).i32(2).u32(1).u32(0x1000 + i);
        }
    }
    p.into_bytes()
}

fn file(records: &[(&str, Vec<u8>)]) -> RecordFile {
    let mut w = RecordFileWriter::new(META);
    for (name, payload) in records {
        w.record(name, payload);
    }
    RecordFile::from_bytes("synthetic", w.finish()).expect("valid record file")
}

fn base() -> Vec<(&'static str, Vec<u8>)> {
    vec![
        ("config", config()),
        ("poly_mesh", poly_mesh()),
        ("crowd_trajectories", crowd(60, None)),
    ]
}

fn outcome_of(
    a: &[(&str, Vec<u8>)],
    b: &[(&str, Vec<u8>)],
    opts: Options,
    name: &str,
) -> (bool, Outcome) {
    let report = compare(&mut file(a), &mut file(b), &opts);
    let outcome = report
        .records
        .iter()
        .find(|r| r.name == name)
        .expect("record compared")
        .outcome
        .clone();
    (report.passed(), outcome)
}

fn tolerant() -> Options {
    Options {
        crowd_tolerance: true,
        ..Options::default()
    }
}

#[test]
fn identical_files_are_equal() {
    let report = compare(&mut file(&base()), &mut file(&base()), &Options::default());
    assert!(report.passed());
    assert!(report.records.iter().all(|r| r.outcome == Outcome::Equal));
    assert_eq!(report.records.len(), 3);
}

#[test]
fn flipped_bit_fails_and_names_the_field() {
    let mut b = base();
    // Header 52 bytes, 4 vertices of 6 bytes, polygons of 29 bytes: polys[1].slots[2] is at 109.
    b[1].1[109] ^= 1;
    let (passed, outcome) = outcome_of(&base(), &b, Options::default(), "poly_mesh");
    assert!(!passed);
    match outcome {
        Outcome::Different(msg) => assert!(msg.starts_with("polys[1].slots[2]: 3 vs 2"), "{msg}"),
        other => panic!("expected a difference, got {other:?}"),
    }

    let mut b = base();
    b[0].1[16] ^= 1; // lowest bit of cs
    let (passed, outcome) = outcome_of(&base(), &b, Options::default(), "config");
    assert!(!passed);
    assert!(
        matches!(&outcome, Outcome::Different(m) if m.starts_with("cs: 0.3 ")),
        "{outcome:?}"
    );
}

#[test]
fn missing_record_fails() {
    let b: Vec<_> = base()
        .into_iter()
        .filter(|(n, _)| *n != "poly_mesh")
        .collect();
    let (passed, outcome) = outcome_of(&base(), &b, Options::default(), "poly_mesh");
    assert!(!passed);
    assert_eq!(outcome, Outcome::Missing { in_a: true });
    let (passed, outcome) = outcome_of(&b, &base(), Options::default(), "poly_mesh");
    assert!(!passed);
    assert_eq!(outcome, Outcome::Missing { in_a: false });
}

#[test]
fn crowd_position_beyond_one_cell_fails_under_tolerance() {
    let mut b = base();
    b[2].1 = crowd(60, Some((40, 2.0 * CELL)));
    let (passed, outcome) = outcome_of(&base(), &b, tolerant(), "crowd_trajectories");
    assert!(!passed);
    assert!(
        matches!(&outcome, Outcome::Different(m) if m.contains("tick 40 agent 1: position error")),
        "{outcome:?}"
    );
}

#[test]
fn crowd_within_tolerance_passes_only_under_the_tolerance_rule() {
    let mut b = base();
    b[2].1 = crowd(60, Some((40, 0.1 * CELL)));
    let (passed, outcome) = outcome_of(&base(), &b, tolerant(), "crowd_trajectories");
    assert!(passed, "{outcome:?}");
    assert!(
        matches!(outcome, Outcome::WithinTolerance { max_error } if max_error > 0.0 && max_error < CELL)
    );
    let (passed, outcome) = outcome_of(&base(), &b, Options::default(), "crowd_trajectories");
    assert!(!passed);
    assert!(
        matches!(&outcome, Outcome::Different(m) if m.starts_with("ticks[40].agents[1].npos.x")),
        "{outcome:?}"
    );
}

#[test]
fn crowd_difference_in_the_exact_ticks_fails_under_tolerance() {
    let mut b = base();
    b[2].1 = crowd(60, Some((5, 0.001)));
    let (passed, outcome) = outcome_of(&base(), &b, tolerant(), "crowd_trajectories");
    assert!(!passed);
    assert!(
        matches!(&outcome, Outcome::Different(m) if m.contains("tick 5 agent 1")),
        "{outcome:?}"
    );
}

#[test]
fn corrupted_payload_fails_its_digest() {
    let mut w = RecordFileWriter::new(META);
    w.record("poly_mesh", &poly_mesh());
    let mut bytes = w.finish();
    let header = 8
        + 4
        + META
            .iter()
            .map(|(k, v)| k.len() + v.len() + 2)
            .sum::<usize>();
    bytes[header + 4 + "poly_mesh".len() + 8 + 20] ^= 0x80; // inside the payload, digest unchanged
    let mut bad = RecordFile::from_bytes("bad", bytes).expect("framing is intact");
    let report = compare(
        &mut file(&[("poly_mesh", poly_mesh())]),
        &mut bad,
        &Options::default(),
    );
    assert!(!report.passed());
    assert!(matches!(&report.records[0].outcome, Outcome::Invalid(m) if m.contains("digest")));
}

#[test]
fn malformed_layout_fails_even_when_both_sides_agree() {
    let mut extra = poly_mesh();
    extra.push(0);
    let recs = [("poly_mesh", extra)];
    let report = compare(&mut file(&recs), &mut file(&recs), &Options::default());
    assert!(!report.passed());
    assert!(
        matches!(&report.records[0].outcome, Outcome::Invalid(m) if m.contains("unread bytes"))
    );
}

#[test]
fn truncated_file_is_rejected() {
    let mut w = RecordFileWriter::new(META);
    w.record("config", &config());
    let mut bytes = w.finish();
    bytes.truncate(bytes.len() - 6);
    assert!(RecordFile::from_bytes("truncated", bytes).is_err());
}

/// With `NAV_RECORDS_DIR` set to a directory of oracle record files (see
/// `reference/run-corpus.sh`), every record decodes and every known kind occurs.
#[test]
fn oracle_records_decode() {
    let Some(dir) = std::env::var_os("NAV_RECORDS_DIR") else {
        eprintln!("NAV_RECORDS_DIR not set; skipping");
        return;
    };
    let mut seen = std::collections::BTreeSet::new();
    for entry in std::fs::read_dir(&dir).expect("NAV_RECORDS_DIR is readable") {
        let path = entry.expect("directory entry").path();
        if path.extension().is_none_or(|x| x != "navrec") {
            continue;
        }
        let mut f = RecordFile::open(&path).expect("record file opens");
        for e in &f.entries {
            seen.insert(schema::kind_of(&e.name).to_string());
        }
        let errors = diff::check(&mut f);
        assert!(errors.is_empty(), "{}: {errors:?}", path.display());
    }
    let missing: Vec<_> = KINDS.iter().filter(|k| !seen.contains(**k)).collect();
    assert!(
        missing.is_empty(),
        "kinds absent from the corpus: {missing:?}"
    );
}

/// A `q_find_path` payload with one entry holding two refs at `bits` width.
fn find_path(bits: u32) -> Vec<u8> {
    let mut p = Payload::new();
    p.u32(1).u32(0x4000_0000).i32(2);
    for r in [0x1_0000_0003u64, 0x2_0000_0004] {
        if bits == 64 {
            p.u32(r as u32).u32((r >> 32) as u32);
        } else {
            p.u32(r as u32);
        }
    }
    p.into_bytes()
}

fn file_with_bits(bits: &str, payload: Vec<u8>) -> RecordFile {
    let mut meta = META.to_vec();
    meta.push(("polyref_bits", bits));
    let mut w = RecordFileWriter::new(&meta);
    w.record("q_find_path", &payload);
    RecordFile::from_bytes("synthetic", w.finish()).expect("valid record file")
}

#[test]
fn refs_decode_at_the_stated_width_and_widths_are_not_mixed() {
    let mut a = file_with_bits("64", find_path(64));
    assert!(diff::check(&mut a).is_empty());
    let mut b = file_with_bits("64", find_path(64));
    assert!(compare(&mut a, &mut b, &Options::default()).passed());
    // A 32-bit file decoded as 64-bit leaves bytes over.
    let mut wrong = file_with_bits("64", find_path(32));
    assert!(!diff::check(&mut wrong).is_empty());
    // Files of different widths compare by decoded value: refs that do not fit
    // 32 bits differ, with no metadata error.
    let mut c = file_with_bits("32", find_path(32));
    let report = compare(&mut a, &mut c, &Options::default());
    assert!(!report.passed());
    assert!(report.meta_errors.is_empty(), "{:?}", report.meta_errors);
    assert!(
        matches!(&report.records[0].outcome, Outcome::Different(m) if m.contains("path[0]")),
        "{:?}",
        report.records[0].outcome
    );
}

/// A file stating `bits` (`None`: no `polyref_bits` key) holding `records`.
fn file_stating(bits: Option<&str>, records: &[(&str, Vec<u8>)]) -> RecordFile {
    let mut meta = META.to_vec();
    if let Some(bits) = bits {
        meta.push(("polyref_bits", bits));
    }
    let mut w = RecordFileWriter::new(&meta);
    for (name, payload) in records {
        w.record(name, payload);
    }
    RecordFile::from_bytes("synthetic", w.finish()).expect("valid record file")
}

/// A `q_find_path` payload with one entry holding `refs` at `bits` width.
fn find_path_of(bits: u32, refs: &[u64]) -> Vec<u8> {
    let mut p = Payload::new();
    p.u32(1).u32(0x4000_0000).i32(refs.len() as i32);
    for &r in refs {
        p.u32(r as u32);
        if bits == 64 {
            p.u32((r >> 32) as u32);
        }
    }
    p.into_bytes()
}

#[test]
fn a_file_without_polyref_bits_decodes_as_32_bit_beside_a_64_bit_file() {
    let mut narrow = file_stating(None, &[("q_find_path", find_path_of(32, &[3, 4]))]);
    let mut wide = file_stating(Some("64"), &[("q_find_path", find_path_of(64, &[3, 4]))]);
    let report = compare(&mut narrow, &mut wide, &Options::default());
    assert!(report.passed(), "{report:?}");
    let mut other = file_stating(Some("64"), &[("q_find_path", find_path_of(64, &[3, 5]))]);
    let mut narrow = file_stating(None, &[("q_find_path", find_path_of(32, &[3, 4]))]);
    let report = compare(&mut narrow, &mut other, &Options::default());
    assert!(
        matches!(&report.records[0].outcome, Outcome::Different(m) if m.contains("path[1]: 4 vs 5")),
        "{:?}",
        report.records[0].outcome
    );
}

#[test]
fn width_independent_records_compare_across_32_and_64_bit_files() {
    let records = [("config", config()), ("poly_mesh", poly_mesh())];
    let mut a = file_stating(Some("32"), &records);
    let mut b = file_stating(Some("64"), &records);
    let report = compare(&mut a, &mut b, &Options::default());
    assert!(report.meta_errors.is_empty(), "{:?}", report.meta_errors);
    assert!(report.passed(), "{report:?}");
}

#[test]
fn crowd_tolerance_rejects_a_cell_size_that_is_not_finite_and_positive() {
    for cs in [f32::NAN, f32::INFINITY, 0.0, -CELL] {
        let mut a = base();
        a[0].1 = config_with_cs(cs);
        let mut b = a.clone();
        b[2].1 = crowd(60, Some((40, 2.0 * CELL)));
        let (passed, outcome) = outcome_of(&a, &b, tolerant(), "crowd_trajectories");
        assert!(!passed, "cs {cs}: {outcome:?}");
        assert!(
            matches!(&outcome, Outcome::Invalid(m) if m.contains("cell size")),
            "cs {cs}: {outcome:?}"
        );
    }
}

#[test]
fn crowd_full_corridor_difference_names_the_polygon() {
    let mut b = base();
    // Tick header, first agent's original 50-byte state, corridor count,
    // then the first corridor ref. This changes neither its first_poly nor pose.
    b[2].1[4 + 8 + 50 + 4] ^= 1;
    let (passed, outcome) = outcome_of(&base(), &b, Options::default(), "crowd_trajectories");
    assert!(!passed);
    assert!(
        matches!(&outcome, Outcome::Different(m) if m.starts_with("ticks[0].agents[0].corridor[0].poly")),
        "{outcome:?}"
    );
}
