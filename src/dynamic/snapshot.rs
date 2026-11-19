//! Canonical modifier snapshots.

use alloc::vec::Vec;

use super::{DynamicLayer, Effect, MAX_PRISM_VERTICES, ModifierId, ModifierSpec, Selector};
use crate::detour::content_hash;
use crate::{Error, Limits, SnapshotError};

const MAGIC: &[u8; 4] = b"DNDY";
const VERSION: u32 = 1;
const HEADER: usize = 16;
const DIGEST: usize = 32;

const SHORT: Error = Error::InvalidSnapshot(SnapshotError::TooShort);

fn invalid(reason: &'static str) -> Error {
    Error::InvalidSnapshot(SnapshotError::InvalidValue(reason))
}

impl ModifierSpec {
    pub(super) fn encoded_len(&self) -> usize {
        // ID, selector tag, selector payload, effect tag and value.
        8 + match &self.selector {
            Selector::Prism { verts_xz, .. } => 10 + 8 * verts_xz.len(),
            Selector::Links { .. } => 8,
        }
    }

    fn encode(&self, out: &mut Vec<u8>) {
        out.extend_from_slice(&self.id.0.to_le_bytes());
        match &self.selector {
            Selector::Prism {
                verts_xz,
                ymin,
                ymax,
                area,
            } => {
                out.extend_from_slice(&[0, verts_xz.len() as u8, area.unwrap_or(255)]);
                out.extend_from_slice(&ymin.to_le_bytes());
                out.extend_from_slice(&ymax.to_le_bytes());
                for [x, z] in verts_xz {
                    out.extend_from_slice(&x.to_le_bytes());
                    out.extend_from_slice(&z.to_le_bytes());
                }
            }
            Selector::Links {
                user_id_min,
                user_id_max,
            } => {
                out.push(1);
                out.extend_from_slice(&user_id_min.to_le_bytes());
                out.extend_from_slice(&user_id_max.to_le_bytes());
            }
        }
        let (tag, value) = match self.effect {
            Effect::ClearFlags(flags) => (0, flags),
            Effect::SetFlags(flags) => (1, flags),
            Effect::SetArea(area) => (2, u16::from(area)),
        };
        out.push(tag);
        out.extend_from_slice(&value.to_le_bytes());
    }

    fn decode(input: &mut &[u8]) -> Result<Self, Error> {
        let id = ModifierId(u32::from_le_bytes(read(input)?));
        let selector = match read(input)? {
            [0] => {
                let [n, area] = read(input)?;
                let n = usize::from(n);
                if !(3..=MAX_PRISM_VERTICES).contains(&n) {
                    return Err(invalid("prism vertex count"));
                }
                let ymin = f32::from_le_bytes(read(input)?);
                let ymax = f32::from_le_bytes(read(input)?);
                let mut verts_xz = Vec::new();
                verts_xz
                    .try_reserve_exact(n)
                    .map_err(|_| Error::AllocationFailed)?;
                for _ in 0..n {
                    verts_xz.push([
                        f32::from_le_bytes(read(input)?),
                        f32::from_le_bytes(read(input)?),
                    ]);
                }
                Selector::Prism {
                    verts_xz,
                    ymin,
                    ymax,
                    area: (area != 255).then_some(area),
                }
            }
            [1] => Selector::Links {
                user_id_min: u32::from_le_bytes(read(input)?),
                user_id_max: u32::from_le_bytes(read(input)?),
            },
            _ => return Err(invalid("selector tag")),
        };
        let [effect] = read(input)?;
        let value = u16::from_le_bytes(read(input)?);
        let effect = match effect {
            0 => Effect::ClearFlags(value),
            1 => Effect::SetFlags(value),
            2 if value <= 63 => Effect::SetArea(value as u8),
            _ => return Err(invalid("modifier effect")),
        };
        Ok(Self::new(id, selector, effect))
    }
}

fn read<const N: usize>(input: &mut &[u8]) -> Result<[u8; N], Error> {
    let (bytes, rest) = input.split_first_chunk().ok_or(SHORT)?;
    *input = rest;
    Ok(*bytes)
}

impl DynamicLayer {
    fn bitset_len(&self) -> usize {
        self.max_ids.div_ceil(8) as usize
    }

    pub(super) fn snapshot_base_len(&self) -> usize {
        HEADER + self.bitset_len() + DIGEST
    }

    fn encode_payload(&self) -> Vec<u8> {
        let mut out = Vec::with_capacity(self.snapshot_base_len() + self.specs_size);
        out.extend_from_slice(MAGIC);
        out.extend_from_slice(&VERSION.to_le_bytes());
        out.extend_from_slice(&self.max_ids.to_le_bytes());
        out.extend_from_slice(&(self.entries.len() as u32).to_le_bytes());
        out.resize(HEADER + self.bitset_len(), 0);
        for entry in &self.entries {
            let id = entry.spec.id.0 as usize;
            if entry.enabled {
                out[HEADER + id / 8] |= 1 << (id % 8);
            }
            entry.spec.encode(&mut out);
        }
        out
    }

    /// Encodes the registered specs and enabled bits in canonical DNDY v1 format.
    pub fn to_bytes(&self) -> Vec<u8> {
        let mut out = self.encode_payload();
        let digest = content_hash(&out);
        out.extend_from_slice(&digest);
        out
    }

    /// Returns the SHA-256 digest of the canonical snapshot payload.
    pub fn digest(&self) -> [u8; 32] {
        content_hash(&self.encode_payload())
    }

    /// Decodes untrusted modifier state. Unknown versions return
    /// [`Error::InvalidSnapshot`] with [`SnapshotError::WrongVersion`].
    pub fn from_bytes(bytes: &[u8], limits: Limits) -> Result<Self, Error> {
        if bytes.len() > limits.max_bytes {
            return Err(Error::LimitExceeded);
        }
        let mut input = bytes;
        if &read(&mut input)? != MAGIC {
            return Err(Error::InvalidSnapshot(SnapshotError::WrongMagic));
        }
        if u32::from_le_bytes(read(&mut input)?) != VERSION {
            return Err(Error::InvalidSnapshot(SnapshotError::WrongVersion));
        }
        let max_ids = u32::from_le_bytes(read(&mut input)?);
        let count = u32::from_le_bytes(read(&mut input)?);
        if count > max_ids {
            return Err(invalid("modifier count"));
        }
        let mut layer = Self::new(max_ids, limits)?;
        let (body, footer) = bytes
            .split_at_checked(bytes.len().checked_sub(DIGEST).ok_or(SHORT)?)
            .ok_or(SHORT)?;
        let (bits, mut specs) = body
            .get(HEADER..)
            .ok_or(SHORT)?
            .split_at_checked(layer.bitset_len())
            .ok_or(SHORT)?;
        if count as usize > specs.len() / 16 {
            return Err(SHORT);
        }
        let mut work = u64::from(count).saturating_add(bytes.len() as u64);
        if work > limits.max_work {
            return Err(Error::LimitExceeded);
        }
        if footer != content_hash(body) {
            return Err(invalid("modifier snapshot checksum"));
        }
        let mut workspace = crate::Workspace::with_limits(limits);
        layer.entries = workspace.vec_with_capacity(count as usize)?;
        let mut enabled_count: u64 = bits.iter().map(|byte| u64::from(byte.count_ones())).sum();
        let mut previous = None;
        for _ in 0..count {
            let spec = ModifierSpec::decode(&mut specs)?;
            if previous.is_some_and(|old| old >= spec.id) {
                return Err(invalid("modifier IDs out of order"));
            }
            if let Selector::Prism { verts_xz, .. } = &spec.selector {
                workspace.charge_bytes::<[f32; 2]>(verts_xz.len())?;
                work = work.saturating_add((verts_xz.len() * verts_xz.len()) as u64);
                if work > limits.max_work {
                    return Err(Error::LimitExceeded);
                }
            }
            layer
                .validate_spec(&spec)
                .map_err(|_| invalid("modifier specification"))?;
            let id = spec.id.0 as usize;
            let enabled = bits[id / 8] & (1 << (id % 8)) != 0;
            enabled_count -= u64::from(enabled);
            previous = Some(spec.id);
            layer.specs_size += spec.encoded_len();
            layer.entries.push(super::Entry { spec, enabled });
        }
        if !specs.is_empty() {
            return Err(Error::InvalidSnapshot(SnapshotError::TrailingBytes));
        }
        if enabled_count != 0 {
            return Err(invalid("enabled bit without a specification"));
        }
        Ok(layer)
    }
}
