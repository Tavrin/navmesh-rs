//! A cursor over a canonical record payload that names every field it reads.
//!
//! Decoders in [`crate::schema`] walk a payload with this cursor. Run plainly it
//! validates the structure (every count fits, every byte is consumed). Run with a
//! probe offset it also reports which field covers that byte and its value, which
//! is how a mismatch is reported as a field path instead of a byte offset.

use std::fmt;

/// A decoded primitive value.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Value {
    /// An unsigned integer field.
    U(u64),
    /// A signed integer field.
    I(i64),
    /// An `f32` field, kept as its bits so equal means bit-equal.
    F(u32),
    /// One byte of an opaque byte run.
    Byte(u8),
}

impl fmt::Display for Value {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match *self {
            Value::U(v) => write!(f, "{v}"),
            Value::I(v) => write!(f, "{v}"),
            Value::F(bits) => write!(f, "{:?} (0x{bits:08x})", f32::from_bits(bits)),
            Value::Byte(b) => write!(f, "0x{b:02x}"),
        }
    }
}

/// A field located by a probe offset.
#[derive(Clone, Debug, PartialEq)]
pub struct Located {
    /// Field path, such as `polys[3].slots[2]`.
    pub path: String,
    /// The field's value.
    pub value: Value,
    /// Byte offset of the field's first byte in the payload.
    pub offset: usize,
}

/// A payload that does not match its record kind's layout.
#[derive(Clone, Debug, PartialEq)]
pub struct DecodeError {
    /// Byte offset where decoding failed.
    pub offset: usize,
    /// Path of the field being read.
    pub path: String,
    /// What was wrong.
    pub message: String,
}

impl fmt::Display for DecodeError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        write!(
            f,
            "{} at byte {} ({})",
            self.message, self.offset, self.path
        )
    }
}

impl std::error::Error for DecodeError {}

/// Result of a decoding step.
pub type DecodeResult<T> = Result<T, DecodeError>;

#[derive(Clone, Copy, Debug)]
enum Seg {
    Field(&'static str),
    Index(usize),
}

/// Reads primitives from a payload, tracking the field path.
#[derive(Debug)]
pub struct Cursor<'a> {
    data: &'a [u8],
    pos: usize,
    stack: Vec<Seg>,
    probe: Option<usize>,
    found: Option<Located>,
    ref_bits: u32,
    /// The bytes read so far with every poly ref widened to 64 bits (see
    /// [`Cursor::normalizing`]).
    norm: Option<Vec<u8>>,
}

impl<'a> Cursor<'a> {
    /// A cursor that validates structure only.
    pub fn new(data: &'a [u8]) -> Self {
        Cursor {
            data,
            pos: 0,
            stack: Vec::new(),
            probe: None,
            found: None,
            ref_bits: 32,
            norm: None,
        }
    }

    /// Records the payload as it is read with every poly ref widened to 64 bits,
    /// so two payloads written at different ref widths can be compared by value
    /// ([`Cursor::into_normalized`]). A payload without refs normalizes to itself.
    pub fn normalizing(mut self) -> Self {
        self.norm = Some(Vec::with_capacity(self.data.len()));
        self
    }

    /// The bytes recorded by [`Cursor::normalizing`] (empty when it was not requested).
    pub fn into_normalized(self) -> Vec<u8> {
        self.norm.unwrap_or_default()
    }

    /// Sets the poly-ref width (32 or 64 bits, the record file's `polyref_bits`).
    pub fn with_ref_bits(mut self, bits: u32) -> Self {
        self.ref_bits = bits;
        self
    }

    /// A cursor that also locates the field covering byte `offset`.
    pub fn with_probe(data: &'a [u8], offset: usize) -> Self {
        Cursor {
            probe: Some(offset),
            ..Cursor::new(data)
        }
    }

    /// The field found by the probe, once decoding has passed it.
    pub fn found(&self) -> Option<&Located> {
        self.found.as_ref()
    }

    /// Current byte offset.
    pub fn pos(&self) -> usize {
        self.pos
    }

    /// Bytes left to read.
    pub fn remaining(&self) -> usize {
        self.data.len() - self.pos
    }

    fn path_with(&self, leaf: Option<&'static str>) -> String {
        let mut s = String::new();
        let segs = self.stack.iter().copied().chain(leaf.map(Seg::Field));
        for seg in segs {
            match seg {
                // Unnamed fields: array elements that are a single value or a vector.
                Seg::Field("") => {}
                Seg::Field(name) => {
                    if !s.is_empty() {
                        s.push('.');
                    }
                    s.push_str(name);
                }
                Seg::Index(i) => {
                    s.push('[');
                    s.push_str(&i.to_string());
                    s.push(']');
                }
            }
        }
        if s.is_empty() {
            "(record)".to_string()
        } else {
            s
        }
    }

    /// An error at the current position.
    pub fn error(&self, name: &'static str, message: impl Into<String>) -> DecodeError {
        DecodeError {
            offset: self.pos,
            path: self.path_with(Some(name)),
            message: message.into(),
        }
    }

    fn take<const N: usize>(&mut self, name: &'static str) -> DecodeResult<[u8; N]> {
        if self.remaining() < N {
            return Err(self.error(
                name,
                format!("payload ends: {N} bytes needed, {} left", self.remaining()),
            ));
        }
        let start = self.pos;
        let mut b = [0u8; N];
        b.copy_from_slice(&self.data[start..start + N]);
        self.pos += N;
        if let Some(n) = &mut self.norm {
            n.extend_from_slice(&b);
        }
        Ok(b)
    }

    fn note(&mut self, name: &'static str, start: usize, len: usize, value: Value) {
        if let Some(p) = self.probe
            && self.found.is_none()
            && p >= start
            && p < start + len
        {
            self.found = Some(Located {
                path: self.path_with(Some(name)),
                value,
                offset: start,
            });
        }
    }

    /// Reads a `u8`.
    pub fn u8(&mut self, name: &'static str) -> DecodeResult<u8> {
        let start = self.pos;
        let v = self.take::<1>(name)?[0];
        self.note(name, start, 1, Value::U(u64::from(v)));
        Ok(v)
    }

    /// Reads a `u16`.
    pub fn u16(&mut self, name: &'static str) -> DecodeResult<u16> {
        let start = self.pos;
        let v = u16::from_le_bytes(self.take::<2>(name)?);
        self.note(name, start, 2, Value::U(u64::from(v)));
        Ok(v)
    }

    /// Reads a `u32`.
    pub fn u32(&mut self, name: &'static str) -> DecodeResult<u32> {
        let start = self.pos;
        let v = u32::from_le_bytes(self.take::<4>(name)?);
        self.note(name, start, 4, Value::U(u64::from(v)));
        Ok(v)
    }

    /// Reads a poly or tile ref at the cursor's ref width.
    pub fn poly_ref(&mut self, name: &'static str) -> DecodeResult<u64> {
        if self.ref_bits == 64 {
            let start = self.pos;
            let v = u64::from_le_bytes(self.take::<8>(name)?);
            self.note(name, start, 8, Value::U(v));
            Ok(v)
        } else {
            let v = self.u32(name).map(u64::from)?;
            if let Some(n) = &mut self.norm {
                n.extend_from_slice(&[0; 4]);
            }
            Ok(v)
        }
    }

    /// Reads an `i32`.
    pub fn i32(&mut self, name: &'static str) -> DecodeResult<i32> {
        let start = self.pos;
        let v = i32::from_le_bytes(self.take::<4>(name)?);
        self.note(name, start, 4, Value::I(i64::from(v)));
        Ok(v)
    }

    /// Reads an `f32`.
    pub fn f32(&mut self, name: &'static str) -> DecodeResult<f32> {
        let start = self.pos;
        let bits = u32::from_le_bytes(self.take::<4>(name)?);
        self.note(name, start, 4, Value::F(bits));
        Ok(f32::from_bits(bits))
    }

    /// Reads three `f32` values as a group named `name` with fields x, y, z.
    pub fn v3(&mut self, name: &'static str) -> DecodeResult<[f32; 3]> {
        self.group(name, |c| Ok([c.f32("x")?, c.f32("y")?, c.f32("z")?]))
    }

    /// Reads `n` opaque bytes (padding, or bytes with no finer structure).
    pub fn bytes(&mut self, name: &'static str, n: usize) -> DecodeResult<&'a [u8]> {
        if self.remaining() < n {
            return Err(self.error(
                name,
                format!("payload ends: {n} bytes needed, {} left", self.remaining()),
            ));
        }
        let start = self.pos;
        self.pos += n;
        if let Some(norm) = &mut self.norm {
            norm.extend_from_slice(&self.data[start..start + n]);
        }
        if let Some(p) = self.probe
            && self.found.is_none()
            && p >= start
            && p < start + n
        {
            let i = p - start;
            self.stack.push(Seg::Field(name));
            self.stack.push(Seg::Index(i));
            self.found = Some(Located {
                path: self.path_with(None),
                value: Value::Byte(self.data[p]),
                offset: p,
            });
            self.stack.pop();
            self.stack.pop();
        }
        Ok(&self.data[start..start + n])
    }

    /// Checks that a count read from the payload is non-negative and that `n`
    /// elements of at least `min_elem` bytes each fit in what is left.
    pub fn check_count(&self, name: &'static str, n: i64, min_elem: usize) -> DecodeResult<usize> {
        if n < 0 {
            return Err(self.error(name, format!("negative count {n}")));
        }
        let n = usize::try_from(n).map_err(|_| self.error(name, "count too large"))?;
        if n.checked_mul(min_elem)
            .is_none_or(|need| need > self.remaining())
        {
            return Err(self.error(
                name,
                format!(
                    "count {n} needs at least {} bytes, {} left",
                    n.saturating_mul(min_elem),
                    self.remaining()
                ),
            ));
        }
        Ok(n)
    }

    /// Reads an `i32` count and checks it with [`Cursor::check_count`].
    pub fn count_i32(&mut self, name: &'static str, min_elem: usize) -> DecodeResult<usize> {
        let n = self.i32(name)?;
        self.check_count(name, i64::from(n), min_elem)
    }

    /// Reads a `u32` count and checks it with [`Cursor::check_count`].
    pub fn count_u32(&mut self, name: &'static str, min_elem: usize) -> DecodeResult<usize> {
        let n = self.u32(name)?;
        self.check_count(name, i64::from(n), min_elem)
    }

    /// Decodes a named group of fields.
    pub fn group<T>(
        &mut self,
        name: &'static str,
        f: impl FnOnce(&mut Self) -> DecodeResult<T>,
    ) -> DecodeResult<T> {
        self.stack.push(Seg::Field(name));
        let r = f(self);
        self.stack.pop();
        r
    }

    /// Decodes `n` elements of a named array.
    pub fn array(
        &mut self,
        name: &'static str,
        n: usize,
        mut f: impl FnMut(&mut Self, usize) -> DecodeResult<()>,
    ) -> DecodeResult<()> {
        self.stack.push(Seg::Field(name));
        for i in 0..n {
            self.stack.push(Seg::Index(i));
            let r = f(self, i);
            self.stack.pop();
            if let Err(e) = r {
                self.stack.pop();
                return Err(e);
            }
        }
        self.stack.pop();
        Ok(())
    }

    /// Checks that the whole payload was consumed.
    pub fn finish(&self) -> DecodeResult<()> {
        if self.remaining() != 0 {
            return Err(DecodeError {
                offset: self.pos,
                path: "(end)".into(),
                message: format!("{} unread bytes after the last field", self.remaining()),
            });
        }
        Ok(())
    }
}
