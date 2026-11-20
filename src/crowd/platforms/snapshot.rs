use super::*;
use crate::SnapshotError;
use crate::detour::content_hash;
use alloc::collections::BTreeSet;

const MAGIC: &[u8; 4] = b"DCFR";
const VERSION: u32 = 1;

impl FramedCrowd<CrowdWorld> {
    /// Saves each local simulation, stable agent ids, goals, and dock wait
    /// offers. Save the matching [`FrameSet::to_bytes`] snapshot alongside it.
    pub fn to_bytes(&self, frames: &FrameSet) -> Vec<u8> {
        let mut out = Vec::new();
        out.extend_from_slice(MAGIC);
        put32(&mut out, VERSION);
        out.extend_from_slice(&content_hash(&frames.to_bytes()));
        put64(&mut out, self.crowds.len() as u64);
        put64(&mut out, self.riders.len() as u64);
        put64(&mut out, self.waits.len() as u64);
        for (&id, crowd) in &self.crowds {
            put32(&mut out, id.0);
            let local = crowd.to_bytes();
            put64(&mut out, local.len() as u64);
            out.extend_from_slice(&local);
        }
        for (&id, r) in &self.riders {
            put64(&mut out, id.0);
            put32(&mut out, r.frame.0);
            put32(&mut out, r.handle.index());
            put32(&mut out, r.handle.generation());
            out.push(u8::from(r.goal.is_some()));
            if let Some(goal) = r.goal {
                put32(&mut out, goal.frame.0);
                for v in goal.position {
                    put32(&mut out, v.to_bits());
                }
            }
        }
        for (&dock, &cost) in &self.waits {
            put32(&mut out, dock.0);
            put32(&mut out, cost.to_bits());
        }
        let checksum = content_hash(&out);
        out.extend_from_slice(&checksum);
        out
    }

    /// Restores a framed crowd after restoring the matching frame state.
    /// Unsupported versions return `InvalidSnapshot(WrongVersion)`. The limits
    /// apply to the serialized input and to each native crowd's restoration.
    pub fn from_bytes(frames: &FrameSet, bytes: &[u8], limits: Limits) -> Result<Self, Error> {
        let mut input = Input(bytes);
        if input.take(4)? != MAGIC {
            return Err(Error::InvalidSnapshot(SnapshotError::WrongMagic));
        }
        if input.u32()? != VERSION {
            return Err(Error::InvalidSnapshot(SnapshotError::WrongVersion));
        }
        if bytes.len() > limits.max_bytes {
            return Err(Error::LimitExceeded);
        }
        let end = bytes
            .len()
            .checked_sub(32)
            .filter(|&n| n >= 8)
            .ok_or(Error::InvalidSnapshot(SnapshotError::TooShort))?;
        if content_hash(&bytes[..end]) != bytes[end..] {
            return Err(bad("framed crowd checksum"));
        }
        input.0 = &bytes[8..end];
        if input.take(32)? != content_hash(&frames.to_bytes()) {
            return Err(bad("crowd frame binding"));
        }
        let crowd_count = input.count()?;
        let rider_count = input.count()?;
        let wait_count = input.count()?;
        let mut restored = Self::default();
        let mut dt = None;
        for _ in 0..crowd_count {
            let frame = FrameId(input.u32()?);
            if restored
                .crowds
                .last_key_value()
                .is_some_and(|(&old, _)| old >= frame)
            {
                return Err(bad("crowd frame order"));
            }
            let nav = frames.mesh(frame).ok_or_else(|| bad("crowd frame"))?;
            let count = input.count()?;
            let crowd = CrowdWorld::from_bytes(nav, input.take(count)?, limits)?;
            if dt
                .replace(crowd.params().fixed_dt)
                .is_some_and(|old| old != crowd.params().fixed_dt)
            {
                return Err(bad("crowd frame time step"));
            }
            restored.crowds.insert(frame, crowd);
        }
        if rider_count != restored.crowds.values().map(CrowdWorld::len).sum::<usize>() {
            return Err(bad("rider count"));
        }

        let mut handles = BTreeSet::new();
        for _ in 0..rider_count {
            let id = PlatformAgentId(input.u64()?);
            if restored
                .riders
                .last_key_value()
                .is_some_and(|(&old, _)| old >= id)
            {
                return Err(bad("rider order"));
            }
            let frame = FrameId(input.u32()?);
            let index = input.u32()?;
            let generation = input.u32()?;
            let handle = restored
                .crowds
                .get(&frame)
                .and_then(|c| c.handle_at(index))
                .filter(|h| h.generation() == generation)
                .ok_or_else(|| bad("rider handle"))?;
            if !handles.insert((frame, handle)) {
                return Err(bad("duplicate rider handle"));
            }
            let goal = match input.take(1)?[0] {
                0 => None,
                1 => {
                    let dest = FrameId(input.u32()?);
                    frames.mesh(dest).ok_or_else(|| bad("goal frame"))?;
                    let pos = [input.float()?, input.float()?, input.float()?];
                    finite(pos).map_err(|_| bad("rider goal"))?;
                    Some(FramePoint::new(dest, pos))
                }
                _ => return Err(bad("rider goal flag")),
            };
            restored.riders.insert(
                id,
                Rider {
                    frame,
                    handle,
                    goal,
                },
            );
        }

        for _ in 0..wait_count {
            let id = LinkId(input.u32()?);
            if restored
                .waits
                .last_key_value()
                .is_some_and(|(&old, _)| old >= id)
            {
                return Err(bad("dock wait order"));
            }
            let cost = input.float()?;
            nonnegative(cost).map_err(|_| bad("dock wait cost"))?;
            restored
                .set_wait_cost(frames, id, Some(cost))
                .map_err(|_| bad("dock wait dock"))?;
        }
        if !input.0.is_empty() {
            return Err(Error::InvalidSnapshot(SnapshotError::TrailingBytes));
        }
        Ok(restored)
    }
}

fn put32(out: &mut Vec<u8>, n: u32) {
    out.extend_from_slice(&n.to_le_bytes());
}

fn put64(out: &mut Vec<u8>, n: u64) {
    out.extend_from_slice(&n.to_le_bytes());
}

fn bad(message: &'static str) -> Error {
    Error::InvalidSnapshot(SnapshotError::InvalidValue(message))
}

struct Input<'a>(&'a [u8]);

impl<'a> Input<'a> {
    fn take(&mut self, n: usize) -> Result<&'a [u8], Error> {
        let (head, rest) = self
            .0
            .split_at_checked(n)
            .ok_or(Error::InvalidSnapshot(SnapshotError::TooShort))?;
        self.0 = rest;
        Ok(head)
    }

    fn u32(&mut self) -> Result<u32, Error> {
        let v = self.take(4)?.try_into().map_err(|_| bad("integer width"))?;
        Ok(u32::from_le_bytes(v))
    }

    fn u64(&mut self) -> Result<u64, Error> {
        let v = self.take(8)?.try_into().map_err(|_| bad("integer width"))?;
        Ok(u64::from_le_bytes(v))
    }

    fn count(&mut self) -> Result<usize, Error> {
        usize::try_from(self.u64()?).map_err(|_| bad("snapshot count"))
    }

    fn float(&mut self) -> Result<f32, Error> {
        Ok(f32::from_bits(self.u32()?))
    }
}
