//! Canonical modifier snapshots.

use alloc::vec::Vec;

use super::{
    BaseState, DynamicLayer, Effect, MAX_PRISM_VERTICES, ModifierId, ModifierSpec, Selector,
};
use crate::detour::content_hash;
use crate::{Error, Limits, SnapshotError};

const MAGIC: &[u8; 4] = b"DNDY";
const LEGACY: u32 = 1;
const EXTENDED: u32 = 2;
const HEADER: usize = 16;
const DIGEST: usize = 32;
const BASE_ENTRY: usize = 52;

const SHORT: Error = Error::InvalidSnapshot(SnapshotError::TooShort);

fn invalid(reason: &'static str) -> Error {
    Error::InvalidSnapshot(SnapshotError::InvalidValue(reason))
}

impl ModifierSpec {
    pub(super) fn encoded_len(&self) -> usize {
        // ID, selector tag, selector payload, effect tag and value.
        8 + match &self.selector {
            Selector::Prism { verts_xz, .. } | Selector::DetailPrism { verts_xz, .. } => {
                10 + 8 * verts_xz.len()
            }
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
            }
            | Selector::DetailPrism {
                verts_xz,
                ymin,
                ymax,
                area,
            } => {
                let tag = if self.selector.is_detail() { 2 } else { 0 };
                out.extend_from_slice(&[tag, verts_xz.len() as u8, area.unwrap_or(255)]);
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

    fn decode(input: &mut &[u8], version: u32) -> Result<Self, Error> {
        let id = ModifierId(u32::from_le_bytes(read(input)?));
        let [tag] = read(input)?;
        let selector = match tag {
            0 | 2 if tag == 0 || version == EXTENDED => {
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
                let area = (area != 255).then_some(area);
                if tag == 0 {
                    Selector::Prism {
                        verts_xz,
                        ymin,
                        ymax,
                        area,
                    }
                } else {
                    Selector::DetailPrism {
                        verts_xz,
                        ymin,
                        ymax,
                        area,
                    }
                }
            }
            1 => Selector::Links {
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

    pub(super) fn snapshot_len(
        &self,
        specs_size: usize,
        base_count: usize,
        detail: bool,
    ) -> Result<usize, Error> {
        u32::try_from(base_count).map_err(|_| Error::SizeOverflow)?;
        let base_bytes = base_count
            .checked_mul(BASE_ENTRY)
            .ok_or(Error::SizeOverflow)?;
        self.snapshot_base_len()
            .checked_add(specs_size)
            .and_then(|len| len.checked_add(4 * usize::from(detail || base_count != 0)))
            .and_then(|len| len.checked_add(base_bytes))
            .ok_or(Error::SizeOverflow)
    }

    fn encode_payload(&self) -> Vec<u8> {
        let extended = self.detail_count != 0 || !self.bases.is_empty();
        let version = if extended { EXTENDED } else { LEGACY };
        let size = self.snapshot_base_len()
            + self.specs_size
            + 4 * usize::from(extended)
            + self.bases.len() * BASE_ENTRY;
        let mut out = Vec::with_capacity(size);
        out.extend_from_slice(MAGIC);
        out.extend_from_slice(&version.to_le_bytes());
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
        if extended {
            out.extend_from_slice(&(self.bases.len() as u32).to_le_bytes());
            for (&((x, y, layer, hash), ip), base) in &self.bases {
                for coordinate in [x, y, layer] {
                    out.extend_from_slice(&coordinate.to_le_bytes());
                }
                out.extend_from_slice(&hash);
                out.extend_from_slice(&ip.to_le_bytes());
                out.extend_from_slice(&base.flags.unwrap_or_default().to_le_bytes());
                out.push(base.area.unwrap_or(255));
                out.push(u8::from(base.flags.is_some()));
            }
        }
        out
    }

    /// Encodes the layer in canonical DNDY format. Layers with detail selectors
    /// or caller base edits use version 2. Other layers use version 1.
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
        let version = u32::from_le_bytes(read(&mut input)?);
        if !matches!(version, LEGACY | EXTENDED) {
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
            let spec = ModifierSpec::decode(&mut specs, version)?;
            if previous.is_some_and(|old| old >= spec.id) {
                return Err(invalid("modifier IDs out of order"));
            }
            if let Some(verts_xz) = spec.selector.prism_vertices() {
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
            layer.detail_count += usize::from(spec.selector.is_detail());
            layer.entries.push(super::Entry { spec, enabled });
        }
        if version == EXTENDED {
            layer.decode_bases(&mut specs, &mut workspace)?;
        }
        if !specs.is_empty() {
            return Err(Error::InvalidSnapshot(SnapshotError::TrailingBytes));
        }
        if enabled_count != 0 {
            return Err(invalid("enabled bit without a specification"));
        }
        Ok(layer)
    }

    fn decode_bases(
        &mut self,
        input: &mut &[u8],
        workspace: &mut crate::Workspace,
    ) -> Result<(), Error> {
        let count = u32::from_le_bytes(read(input)?) as usize;
        if count > input.len() / BASE_ENTRY {
            return Err(SHORT);
        }
        if count == 0 && self.detail_count == 0 {
            return Err(invalid("empty modifier snapshot extension"));
        }
        workspace.charge_bytes::<((super::BaseTileKey, u32), BaseState)>(count)?;
        let mut previous = None;
        for _ in 0..count {
            let x = i32::from_le_bytes(read(input)?);
            let y = i32::from_le_bytes(read(input)?);
            let layer = i32::from_le_bytes(read(input)?);
            let hash = read(input)?;
            let ip = u32::from_le_bytes(read(input)?);
            let flags = u16::from_le_bytes(read(input)?);
            let [area, has_flags] = read(input)?;
            let key = ((x, y, layer, hash), ip);
            if previous.is_some_and(|old| old >= key) {
                return Err(invalid("polygon base keys out of order"));
            }
            if has_flags > 1
                || (has_flags == 0 && (flags != 0 || area == 255))
                || !(area <= 63 || area == 255)
            {
                return Err(invalid("polygon base state"));
            }
            self.bases.insert(
                key,
                BaseState {
                    flags: (has_flags == 1).then_some(flags),
                    area: (area != 255).then_some(area),
                },
            );
            previous = Some(key);
        }
        Ok(())
    }
}
