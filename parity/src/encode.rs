//! Writers for record payloads and record files, following the encoding rules
//! in `reference/RECORDS.md`: little-endian, no padding, `f32` as raw bits.

use crate::container::{MAGIC, fnv1a64};

/// A canonical record payload under construction.
#[derive(Clone, Debug, Default)]
pub struct Payload {
    bytes: Vec<u8>,
}

impl Payload {
    /// An empty payload.
    pub fn new() -> Self {
        Self::default()
    }

    /// Appends a `u8`.
    pub fn u8(&mut self, v: u8) -> &mut Self {
        self.bytes.push(v);
        self
    }

    /// Appends a little-endian `u16`.
    pub fn u16(&mut self, v: u16) -> &mut Self {
        self.bytes.extend_from_slice(&v.to_le_bytes());
        self
    }

    /// Appends a little-endian `u32`.
    pub fn u32(&mut self, v: u32) -> &mut Self {
        self.bytes.extend_from_slice(&v.to_le_bytes());
        self
    }

    /// Appends a little-endian `i32`.
    pub fn i32(&mut self, v: i32) -> &mut Self {
        self.bytes.extend_from_slice(&v.to_le_bytes());
        self
    }

    /// Appends an `f32` as its IEEE-754 bits.
    pub fn f32(&mut self, v: f32) -> &mut Self {
        self.u32(v.to_bits())
    }

    /// Appends three `f32` values.
    pub fn v3(&mut self, v: [f32; 3]) -> &mut Self {
        self.f32(v[0]).f32(v[1]).f32(v[2])
    }

    /// Appends raw bytes (used for upstream tile bytes, which are written verbatim).
    pub fn bytes(&mut self, b: &[u8]) -> &mut Self {
        self.bytes.extend_from_slice(b);
        self
    }

    /// The encoded bytes.
    pub fn as_bytes(&self) -> &[u8] {
        &self.bytes
    }

    /// Takes the encoded bytes.
    pub fn into_bytes(self) -> Vec<u8> {
        self.bytes
    }
}

/// Builds a record file in memory.
#[derive(Debug)]
pub struct RecordFileWriter {
    out: Vec<u8>,
    count: u32,
}

impl RecordFileWriter {
    /// Starts a file with the given metadata pairs (written as `key=value` lines).
    pub fn new(meta: &[(&str, &str)]) -> Self {
        let mut text = String::new();
        for (k, v) in meta {
            text.push_str(k);
            text.push('=');
            text.push_str(v);
            text.push('\n');
        }
        let mut out = MAGIC.to_vec();
        out.extend_from_slice(&(text.len() as u32).to_le_bytes());
        out.extend_from_slice(text.as_bytes());
        RecordFileWriter { out, count: 0 }
    }

    /// Appends a record. Names must be non-empty and unique within a file.
    pub fn record(&mut self, name: &str, payload: &[u8]) -> &mut Self {
        self.out
            .extend_from_slice(&(name.len() as u32).to_le_bytes());
        self.out.extend_from_slice(name.as_bytes());
        self.out
            .extend_from_slice(&(payload.len() as u64).to_le_bytes());
        self.out.extend_from_slice(payload);
        self.out.extend_from_slice(&fnv1a64(payload).to_le_bytes());
        self.count += 1;
        self
    }

    /// Writes the end marker and returns the file bytes.
    pub fn finish(mut self) -> Vec<u8> {
        self.out.extend_from_slice(&0u32.to_le_bytes());
        self.out.extend_from_slice(&self.count.to_le_bytes());
        self.out
    }
}
