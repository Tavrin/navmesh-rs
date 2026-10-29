//! Record-by-record comparison of two record files.
//!
//! Every record must be present in both files. Bit-exact records must have equal
//! payloads; on a mismatch the report names the first differing field. Crowd
//! records are bit-exact too unless [`Options::crowd_tolerance`] selects the
//! cross-platform rule in [`crate::crowd`]. With [`Options::validate`] every
//! payload is also decoded, so a record whose layout does not match its kind
//! fails even when both sides agree.

use crate::container::RecordFile;
use crate::crowd;
use crate::decode::Cursor;
use crate::schema::{self, kind_of, type_of};

/// Comparison options.
#[derive(Clone, Copy, Debug)]
pub struct Options {
    /// Apply the crowd tolerance rule instead of bit equality to `crowd_*` records.
    pub crowd_tolerance: bool,
    /// Decode every payload, not only mismatching ones.
    pub validate: bool,
    /// Compare only records whose kind passes this predicate (`None`: all records).
    /// Records of other kinds are skipped in both files, so a file that holds a
    /// subset of the stages can be compared with a full oracle file.
    pub only: Option<fn(&str) -> bool>,
}

impl Default for Options {
    fn default() -> Self {
        Options {
            crowd_tolerance: false,
            validate: true,
            only: None,
        }
    }
}

/// The verdict for one record.
#[derive(Clone, Debug, PartialEq)]
pub enum Outcome {
    /// Payloads are byte-identical.
    Equal,
    /// A crowd record differs but stays within the tolerance rule.
    WithinTolerance {
        /// Largest agent position error.
        max_error: f32,
    },
    /// Payloads differ; the message names the first differing field.
    Different(String),
    /// The record is in only one file (`in_a` tells which).
    Missing {
        /// True when the record is present in the first file only.
        in_a: bool,
    },
    /// The payload could not be read or does not decode as its kind.
    Invalid(String),
}

impl Outcome {
    /// Whether this verdict passes.
    pub fn passed(&self) -> bool {
        matches!(self, Outcome::Equal | Outcome::WithinTolerance { .. })
    }
}

/// One record's verdict.
#[derive(Clone, Debug)]
pub struct RecordResult {
    /// Record name.
    pub name: String,
    /// Verdict.
    pub outcome: Outcome,
}

/// Per-type totals for the report.
#[derive(Clone, Debug)]
pub struct TypeSummary {
    /// Record type (see [`schema::type_of`]).
    pub ty: String,
    /// Records of this type.
    pub total: usize,
    /// Records that passed.
    pub passed: usize,
    /// Of those, records that passed only under the tolerance rule.
    pub tolerated: usize,
    /// Failing records with their verdicts.
    pub failures: Vec<(String, Outcome)>,
}

/// The result of comparing two record files.
#[derive(Clone, Debug)]
pub struct Report {
    /// First file's label.
    pub a: String,
    /// Second file's label.
    pub b: String,
    /// Metadata differences that make the files incomparable.
    pub meta_errors: Vec<String>,
    /// Metadata differences that are reported but allowed.
    pub meta_notes: Vec<String>,
    /// Verdicts in file order (first file, then records only in the second).
    pub records: Vec<RecordResult>,
}

impl Report {
    /// Whether the metadata is compatible and every record passed.
    pub fn passed(&self) -> bool {
        self.meta_errors.is_empty() && self.records.iter().all(|r| r.outcome.passed())
    }

    /// Totals per record type, in order of first appearance.
    pub fn by_type(&self) -> Vec<TypeSummary> {
        let mut out: Vec<TypeSummary> = Vec::new();
        for r in &self.records {
            let ty = type_of(&r.name);
            let pos = match out.iter().position(|s| s.ty == ty) {
                Some(p) => p,
                None => {
                    out.push(TypeSummary {
                        ty: ty.to_string(),
                        total: 0,
                        passed: 0,
                        tolerated: 0,
                        failures: Vec::new(),
                    });
                    out.len() - 1
                }
            };
            let s = &mut out[pos];
            s.total += 1;
            match &r.outcome {
                Outcome::Equal => s.passed += 1,
                Outcome::WithinTolerance { .. } => {
                    s.passed += 1;
                    s.tolerated += 1;
                }
                other => s.failures.push((r.name.clone(), other.clone())),
            }
        }
        out
    }
}

/// Keys that identify the oracle build; two oracle files must agree on them.
const ORACLE_KEYS: &[&str] = &["compiler", "flags", "libc"];

/// Poly-ref width of a record file: its `polyref_bits` metadata, 32 when absent.
/// A file without the key holds no ref-carrying records (the port's Recast-only
/// files) or predates the key (32-bit oracle files).
pub fn ref_bits(file: &RecordFile) -> Result<u32, String> {
    match file.meta_value("polyref_bits") {
        None | Some("32") => Ok(32),
        Some("64") => Ok(64),
        Some(v) => Err(format!("polyref_bits: unsupported value '{v}'")),
    }
}

fn compare_meta(a: &RecordFile, b: &RecordFile, errors: &mut Vec<String>, notes: &mut Vec<String>) {
    for key in ["upstream", "case"] {
        if let (Some(va), Some(vb)) = (a.meta_value(key), b.meta_value(key))
            && va != vb
        {
            errors.push(format!("{key}: '{va}' vs '{vb}'"));
        }
    }
    // Differing widths are not a metadata error: only records that carry poly refs
    // depend on the width, and those are compared by decoded value (see
    // `compare_record`).
    for file in [a, b] {
        if let Err(e) = ref_bits(file) {
            errors.push(e);
        }
    }
    let both_oracle = a.meta_value("producer") == Some("nav_oracle")
        && b.meta_value("producer") == Some("nav_oracle");
    for &key in ORACLE_KEYS {
        let (va, vb) = (a.meta_value(key), b.meta_value(key));
        if va != vb {
            let msg = format!("{key}: '{}' vs '{}'", va.unwrap_or("-"), vb.unwrap_or("-"));
            if both_oracle {
                errors.push(format!(
                    "{msg} (records from different oracle builds are not mixed)"
                ));
            } else {
                notes.push(msg);
            }
        }
    }
}

/// Cell size of the first `config` record, for the crowd tolerance rule. `None`
/// when the file has no `config` record; an unreadable record or a cell size that
/// is not finite and positive is an error, because the tolerance is measured in
/// cells.
fn cell_size(file: &mut RecordFile) -> Option<Result<f32, String>> {
    let idx = file
        .entries
        .iter()
        .position(|e| kind_of(&e.name) == "config")?;
    let read = |file: &mut RecordFile| -> Result<f32, String> {
        let payload = file.payload(idx).map_err(|e| e.to_string())?;
        let mut c = Cursor::new(&payload);
        for name in ["width", "height", "tile_size", "border_size"] {
            c.i32(name).map_err(|e| e.to_string())?;
        }
        c.f32("cs").map_err(|e| e.to_string())
    };
    Some(read(file).and_then(|cs| {
        if crowd::valid_cell_size(cs) {
            Ok(cs)
        } else {
            Err(format!(
                "config cs: {cs} is not a finite positive cell size (the crowd tolerance is measured in cells)"
            ))
        }
    }))
}

/// The payload with every poly ref widened to 64 bits, decoded at `bits`.
fn normalize(kind: &str, payload: &[u8], bits: u32) -> Result<Vec<u8>, String> {
    let mut c = Cursor::new(payload).with_ref_bits(bits).normalizing();
    schema::decode(kind, &mut c).map_err(|e| e.to_string())?;
    Ok(c.into_normalized())
}

/// Names the first differing field of two payloads of the same kind whose poly
/// refs are `bits` wide.
pub fn first_difference(kind: &str, a: &[u8], b: &[u8], bits: u32) -> String {
    let at = a
        .iter()
        .zip(b)
        .position(|(x, y)| x != y)
        .unwrap_or(a.len().min(b.len()));
    let locate = |p: &[u8]| {
        let mut c = Cursor::with_probe(p, at).with_ref_bits(bits);
        let res = schema::decode(kind, &mut c);
        (c.found().cloned(), res.err())
    };
    let (fa, ea) = locate(a);
    let (fb, eb) = locate(b);
    match (fa, fb) {
        (Some(x), Some(y)) if x.path == y.path => {
            format!("{}: {} vs {} (byte {at})", x.path, x.value, y.value)
        }
        (Some(x), Some(y)) => format!(
            "byte {at}: {} = {} vs {} = {}",
            x.path, x.value, y.path, y.value
        ),
        (None, Some(y)) => format!(
            "first payload ends at byte {at}; second continues with {} = {}",
            y.path, y.value
        ),
        (Some(x), None) => format!(
            "second payload ends at byte {at}; first continues with {} = {}",
            x.path, x.value
        ),
        (None, None) => {
            let why = ea
                .or(eb)
                .map(|e| e.to_string())
                .unwrap_or_else(|| "no field covers it".into());
            format!("byte {at} (lengths {} vs {}): {why}", a.len(), b.len())
        }
    }
}

fn validate(kind: &str, payload: &[u8], bits: u32) -> Result<(), String> {
    let mut c = Cursor::new(payload).with_ref_bits(bits);
    schema::decode(kind, &mut c).map_err(|e| e.to_string())
}

/// Per-file settings used for every record of a comparison.
struct Ctx<'a> {
    opts: &'a Options,
    /// Cell size for the crowd tolerance rule (`None`: no `config` record).
    cell: Option<Result<f32, String>>,
    /// Poly-ref width of the first and the second file, each its own.
    bits: (u32, u32),
}

fn compare_record(
    kind: &str,
    a: &mut RecordFile,
    ia: usize,
    b: &mut RecordFile,
    ib: usize,
    ctx: &Ctx,
) -> Outcome {
    let (opts, (bits_a, bits_b)) = (ctx.opts, ctx.bits);
    let pa = match a.payload(ia) {
        Ok(p) => p,
        Err(e) => return Outcome::Invalid(format!("first file: {e}")),
    };
    let pb = match b.payload(ib) {
        Ok(p) => p,
        Err(e) => return Outcome::Invalid(format!("second file: {e}")),
    };
    // Each side decodes at its own width. Raw bytes are comparable only at equal
    // widths; otherwise both payloads are widened to 64-bit refs first, which
    // leaves a payload without refs (every Recast stage, geometry, ref-free
    // queries) byte-identical, so only ref-carrying records feel the difference.
    let widened = if bits_a == bits_b {
        None
    } else {
        let wa = match normalize(kind, &pa, bits_a) {
            Ok(w) => w,
            Err(e) => return Outcome::Invalid(format!("first file: {e}")),
        };
        let wb = match normalize(kind, &pb, bits_b) {
            Ok(w) => w,
            Err(e) => return Outcome::Invalid(format!("second file: {e}")),
        };
        Some((wa, wb))
    };
    let equal = match &widened {
        None => pa == pb,
        Some((wa, wb)) => wa == wb,
    };
    if equal {
        // With differing widths the decode above already validated both sides.
        if widened.is_none()
            && opts.validate
            && let Err(e) = validate(kind, &pa, bits_a)
        {
            return Outcome::Invalid(e);
        }
        return Outcome::Equal;
    }
    if opts.crowd_tolerance && schema::is_crowd(kind) {
        let cell = match &ctx.cell {
            None => return Outcome::Invalid("no config record to take the cell size from".into()),
            Some(Err(e)) => return Outcome::Invalid(e.clone()),
            Some(Ok(cell)) => *cell,
        };
        let ta = match crowd::parse(&pa, bits_a) {
            Ok(t) => t,
            Err(e) => return Outcome::Invalid(format!("first file: {e}")),
        };
        let tb = match crowd::parse(&pb, bits_b) {
            Ok(t) => t,
            Err(e) => return Outcome::Invalid(format!("second file: {e}")),
        };
        return match crowd::compare_with_tolerance(&ta, &tb, cell) {
            Ok(t) => Outcome::WithinTolerance {
                max_error: t.max_error,
            },
            Err(msg) => Outcome::Different(msg),
        };
    }
    Outcome::Different(match &widened {
        None => first_difference(kind, &pa, &pb, bits_a),
        Some((wa, wb)) => first_difference(kind, wa, wb, 64),
    })
}

/// Compares two record files.
pub fn compare(a: &mut RecordFile, b: &mut RecordFile, opts: &Options) -> Report {
    let mut meta_errors = Vec::new();
    let mut meta_notes = Vec::new();
    compare_meta(a, b, &mut meta_errors, &mut meta_notes);
    let cell = if opts.crowd_tolerance {
        cell_size(a)
    } else {
        None
    };
    // A bad cell size fails the report even when no crowd record needs it.
    if let Some(Err(e)) = &cell {
        meta_errors.push(e.clone());
    }
    // Each file's own width (absent: 32); an unsupported value is already a
    // metadata error.
    let bits = (ref_bits(a).unwrap_or(32), ref_bits(b).unwrap_or(32));
    let ctx = Ctx { opts, cell, bits };
    let selected = |name: &str| opts.only.is_none_or(|only| only(kind_of(name)));
    let mut records = Vec::new();
    for ia in 0..a.entries.len() {
        let name = a.entries[ia].name.clone();
        if !selected(&name) {
            continue;
        }
        let outcome = match b.find(&name) {
            None => Outcome::Missing { in_a: true },
            Some(ib) => compare_record(kind_of(&name), a, ia, b, ib, &ctx),
        };
        records.push(RecordResult { name, outcome });
    }
    for e in &b.entries {
        if selected(&e.name) && a.find(&e.name).is_none() {
            records.push(RecordResult {
                name: e.name.clone(),
                outcome: Outcome::Missing { in_a: false },
            });
        }
    }
    Report {
        a: a.label().to_string(),
        b: b.label().to_string(),
        meta_errors,
        meta_notes,
        records,
    }
}

/// Decodes every record of one file; returns per-record decode errors.
pub fn check(file: &mut RecordFile) -> Vec<(String, String)> {
    let mut errors = Vec::new();
    let bits = match ref_bits(file) {
        Ok(b) => b,
        Err(e) => return vec![("metadata".into(), e)],
    };
    for i in 0..file.entries.len() {
        let name = file.entries[i].name.clone();
        let res = file
            .payload(i)
            .map_err(|e| e.to_string())
            .and_then(|p| validate(kind_of(&name), &p, bits));
        if let Err(e) = res {
            errors.push((name, e));
        }
    }
    errors
}
