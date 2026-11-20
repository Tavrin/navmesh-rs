//! Little-endian record reading and writing shared by the sector formats.

use crate::Error;
use alloc::vec::Vec;

pub(crate) struct Writer {
    pub(crate) out: Vec<u8>,
}

impl Writer {
    pub(crate) fn with_capacity(n: usize) -> Self {
        Writer {
            out: Vec::with_capacity(n),
        }
    }
    pub(crate) fn u8(&mut self, v: u8) {
        self.out.push(v);
    }
    pub(crate) fn u16(&mut self, v: u16) {
        self.out.extend_from_slice(&v.to_le_bytes());
    }
    pub(crate) fn u32(&mut self, v: u32) {
        self.out.extend_from_slice(&v.to_le_bytes());
    }
    pub(crate) fn i32(&mut self, v: i32) {
        self.out.extend_from_slice(&v.to_le_bytes());
    }
    pub(crate) fn f32(&mut self, v: f32) {
        self.out.extend_from_slice(&v.to_bits().to_le_bytes());
    }
    pub(crate) fn f32s(&mut self, v: &[f32]) {
        for &x in v {
            self.f32(x);
        }
    }
}

pub(crate) struct Reader<'a> {
    data: &'a [u8],
    at: usize,
    what: &'static str,
}

impl<'a> Reader<'a> {
    pub(crate) fn new(data: &'a [u8], what: &'static str) -> Self {
        Reader { data, at: 0, what }
    }
    fn take<const N: usize>(&mut self) -> Result<[u8; N], Error> {
        let end = self.at.checked_add(N).ok_or(Error::SizeOverflow)?;
        let bytes = self
            .data
            .get(self.at..end)
            .ok_or(Error::InvalidInput(self.what))?;
        self.at = end;
        let mut out = [0u8; N];
        out.copy_from_slice(bytes);
        Ok(out)
    }
    pub(crate) fn u8(&mut self) -> Result<u8, Error> {
        Ok(self.take::<1>()?[0])
    }
    pub(crate) fn u16(&mut self) -> Result<u16, Error> {
        Ok(u16::from_le_bytes(self.take()?))
    }
    pub(crate) fn u32(&mut self) -> Result<u32, Error> {
        Ok(u32::from_le_bytes(self.take()?))
    }
    pub(crate) fn i32(&mut self) -> Result<i32, Error> {
        Ok(i32::from_le_bytes(self.take()?))
    }
    /// A finite `f32`; NaN and infinities are rejected.
    pub(crate) fn f32(&mut self) -> Result<f32, Error> {
        let v = f32::from_bits(u32::from_le_bytes(self.take()?));
        if v.is_finite() {
            Ok(v)
        } else {
            Err(Error::InvalidInput(self.what))
        }
    }
    pub(crate) fn f32s<const N: usize>(&mut self) -> Result<[f32; N], Error> {
        let mut out = [0.0; N];
        for v in &mut out {
            *v = self.f32()?;
        }
        Ok(out)
    }
    /// A zero padding byte (nonzero padding is rejected so accepted bytes re-encode
    /// identically).
    pub(crate) fn pad(&mut self, n: usize) -> Result<(), Error> {
        for _ in 0..n {
            if self.u8()? != 0 {
                return Err(Error::InvalidInput(self.what));
            }
        }
        Ok(())
    }
    pub(crate) fn require(&self, ok: bool) -> Result<(), Error> {
        if ok {
            Ok(())
        } else {
            Err(Error::InvalidInput(self.what))
        }
    }
    pub(crate) fn finish(&self) -> Result<(), Error> {
        self.require(self.at == self.data.len())
    }
}

/// `[min, max]` pairs must be ordered.
pub(crate) fn ordered(lo: f32, hi: f32) -> bool {
    lo <= hi
}

/// Checks a decoded record's size against the caller's byte limit.
pub(crate) fn check_bytes(len: usize, limits: &crate::Limits) -> Result<(), Error> {
    if len > limits.max_bytes {
        Err(Error::LimitExceeded)
    } else {
        Ok(())
    }
}
