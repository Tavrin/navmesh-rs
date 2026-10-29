//! Record encoder, decoders and comparator for the differential tests against the
//! recastnavigation reference driver (`reference/harness/nav_oracle.cpp`).
//!
//! The driver writes one record file per case (format in `reference/RECORDS.md`):
//! a list of named records, each a canonical little-endian byte stream with an
//! FNV-1a 64 digest. This crate reads those files ([`container`]), decodes every
//! record kind field by field ([`schema`]), applies the crowd tolerance rule
//! ([`crowd`]) and compares two files record by record ([`diff`]). [`encode`]
//! writes the same format, for the port and for tests.

pub mod container;
pub mod crowd;
pub mod decode;
pub mod diff;
pub mod encode;
pub mod schema;

pub use container::{Error, RecordFile, fnv1a64};
pub use diff::{Options, Outcome, Report, compare};
