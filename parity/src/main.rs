//! `nav-parity`: compare, check and list record files.
//!
//! ```text
//! nav-parity diff  <a> <b> [--crowd-tolerance] [--no-validate] [--recast]
//! nav-parity check <file-or-dir>...
//! nav-parity list  <file>
//! ```
//!
//! `diff` takes two record files, or two directories whose `*.navrec` files are
//! paired by name. `--recast` compares only the Recast stage records (both files
//! may hold other records); `--detour` compares the Recast stages, navmesh tiles,
//! malformed-tile verdicts and query records. Exit status: 0 when everything matches, 1 on any difference,
//! missing record or decode failure, 2 on usage or I/O errors.

use std::collections::BTreeMap;
use std::path::{Path, PathBuf};
use std::process::ExitCode;

use navmesh_parity::diff::{self, Options, Outcome, Report};
use navmesh_parity::schema::{kind_of, type_of};
use navmesh_parity::{Error, RecordFile};

const USAGE: &str = "usage:
  nav-parity diff  <a> <b> [--crowd-tolerance] [--no-validate] [--recast | --detour]
  nav-parity check <file-or-dir>...
  nav-parity list  <file>";

fn record_files(dir: &Path) -> Result<Vec<PathBuf>, Error> {
    let mut files: Vec<PathBuf> = std::fs::read_dir(dir)
        .map_err(|e| Error(format!("{}: {e}", dir.display())))?
        .filter_map(|e| e.ok().map(|e| e.path()))
        .filter(|p| p.extension().is_some_and(|x| x == "navrec"))
        .collect();
    files.sort();
    Ok(files)
}

fn describe(outcome: &Outcome) -> String {
    match outcome {
        Outcome::Equal => "equal".into(),
        Outcome::WithinTolerance { max_error } => {
            format!("within tolerance (max position error {max_error})")
        }
        Outcome::Different(msg) => format!("DIFFERENT: {msg}"),
        Outcome::Missing { in_a: true } => "MISSING from the second file".into(),
        Outcome::Missing { in_a: false } => "MISSING from the first file".into(),
        Outcome::Invalid(msg) => format!("INVALID: {msg}"),
    }
}

#[derive(Default)]
struct Totals {
    total: usize,
    passed: usize,
    tolerated: usize,
}

fn print_report(report: &Report, totals: &mut BTreeMap<String, Totals>) {
    let failed = report
        .records
        .iter()
        .filter(|r| !r.outcome.passed())
        .count();
    println!(
        "{}  {} records, {} failed{}",
        Path::new(&report.a)
            .file_name()
            .map_or(report.a.clone(), |n| n.to_string_lossy().into_owned()),
        report.records.len(),
        failed,
        if report.meta_errors.is_empty() {
            ""
        } else {
            ", metadata mismatch"
        }
    );
    for e in &report.meta_errors {
        println!("  metadata: {e}");
    }
    for n in &report.meta_notes {
        println!("  note: {n}");
    }
    for s in report.by_type() {
        let t = totals.entry(s.ty.clone()).or_default();
        t.total += s.total;
        t.passed += s.passed;
        t.tolerated += s.tolerated;
        for (name, outcome) in &s.failures {
            println!("  {name}: {}", describe(outcome));
        }
    }
}

fn run_diff(a: &Path, b: &Path, opts: &Options) -> Result<bool, Error> {
    let pairs: Vec<(PathBuf, PathBuf)> = if a.is_dir() {
        let fa = record_files(a)?;
        let fb = record_files(b)?;
        let mut ok = true;
        for f in &fb {
            if !a.join(f.file_name().unwrap_or_default()).exists() {
                println!("{}: only in the second directory", f.display());
                ok = false;
            }
        }
        let mut pairs = Vec::new();
        for f in fa {
            let other = b.join(f.file_name().unwrap_or_default());
            if other.exists() {
                pairs.push((f, other));
            } else {
                println!("{}: only in the first directory", f.display());
                ok = false;
            }
        }
        if !ok {
            return Ok(false);
        }
        pairs
    } else {
        vec![(a.to_path_buf(), b.to_path_buf())]
    };
    let mut totals: BTreeMap<String, Totals> = BTreeMap::new();
    let mut all_passed = true;
    for (pa, pb) in &pairs {
        let mut fa = RecordFile::open(pa)?;
        let mut fb = RecordFile::open(pb)?;
        let report = diff::compare(&mut fa, &mut fb, opts);
        all_passed &= report.passed();
        print_report(&report, &mut totals);
    }
    println!();
    println!(
        "{:<44} {:>8} {:>8} {:>8}",
        "record type", "records", "equal", "failed"
    );
    for (ty, t) in &totals {
        let equal = if t.tolerated > 0 {
            format!("{}+{}t", t.passed - t.tolerated, t.tolerated)
        } else {
            t.passed.to_string()
        };
        println!(
            "{:<44} {:>8} {:>8} {:>8}",
            ty,
            t.total,
            equal,
            t.total - t.passed
        );
    }
    let records: usize = totals.values().map(|t| t.total).sum();
    let failed: usize = totals.values().map(|t| t.total - t.passed).sum();
    println!(
        "{} files, {} record types, {} records, {} failed: {}",
        pairs.len(),
        totals.len(),
        records,
        failed,
        if all_passed { "EQUAL" } else { "DIFFERENT" }
    );
    Ok(all_passed)
}

fn run_check(paths: &[PathBuf]) -> Result<bool, Error> {
    let mut files = Vec::new();
    for p in paths {
        if p.is_dir() {
            files.extend(record_files(p)?);
        } else {
            files.push(p.clone());
        }
    }
    let mut kinds: BTreeMap<String, usize> = BTreeMap::new();
    let mut ok = true;
    let mut records = 0usize;
    for f in &files {
        let mut file = RecordFile::open(f)?;
        for e in &file.entries {
            *kinds.entry(kind_of(&e.name).to_string()).or_default() += 1;
        }
        records += file.entries.len();
        for (name, err) in diff::check(&mut file) {
            println!("{}: {name}: {err}", f.display());
            ok = false;
        }
    }
    for (kind, n) in &kinds {
        println!("{kind:<40} {n:>8}");
    }
    println!(
        "{} files, {} records, {} kinds decoded: {}",
        files.len(),
        records,
        kinds.len(),
        if ok { "OK" } else { "ERRORS" }
    );
    Ok(ok)
}

fn run_list(path: &Path) -> Result<bool, Error> {
    let file = RecordFile::open(path)?;
    for (k, v) in &file.meta {
        println!("# {k}={v}");
    }
    for e in &file.entries {
        println!(
            "{:<48} {:<36} {:>12} {:016x}",
            e.name,
            type_of(&e.name),
            e.len,
            e.digest
        );
    }
    Ok(true)
}

fn main() -> ExitCode {
    let args: Vec<String> = std::env::args().skip(1).collect();
    let result = match args.first().map(String::as_str) {
        Some("diff") => {
            let mut opts = Options::default();
            let mut paths = Vec::new();
            for a in &args[1..] {
                match a.as_str() {
                    "--crowd-tolerance" => opts.crowd_tolerance = true,
                    "--no-validate" => opts.validate = false,
                    "--recast" => opts.only = Some(navmesh_parity::schema::is_recast),
                    "--detour" => opts.only = Some(navmesh_parity::schema::is_detour_phase),
                    _ if a.starts_with("--") => {
                        eprintln!("unknown option {a}\n{USAGE}");
                        return ExitCode::from(2);
                    }
                    _ => paths.push(PathBuf::from(a)),
                }
            }
            if paths.len() != 2 {
                eprintln!("{USAGE}");
                return ExitCode::from(2);
            }
            run_diff(&paths[0], &paths[1], &opts)
        }
        Some("check") if args.len() > 1 => {
            run_check(&args[1..].iter().map(PathBuf::from).collect::<Vec<_>>())
        }
        Some("list") if args.len() == 2 => run_list(Path::new(&args[1])),
        _ => {
            eprintln!("{USAGE}");
            return ExitCode::from(2);
        }
    };
    match result {
        Ok(true) => ExitCode::SUCCESS,
        Ok(false) => ExitCode::from(1),
        Err(e) => {
            eprintln!("error: {e}");
            ExitCode::from(2)
        }
    }
}
