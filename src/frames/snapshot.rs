use super::*;
use crate::SnapshotError;
use crate::detour::content_hash;

const MAGIC: &[u8; 4] = b"DNFR";
const VERSION: u32 = 1;
const FRAME_BYTES: usize = 56;
const DOCK_BYTES: usize = 41;

impl FrameSet {
    /// Saves frame identities, absolute poses, and dock specs and availability.
    ///
    /// Save tile content, residency and any ordinary runtime off-mesh links
    /// separately. The snapshot binds to each mesh's grid, residency and link records.
    pub fn to_bytes(&self) -> Vec<u8> {
        let mut out = Vec::new();
        out.extend_from_slice(MAGIC);
        write_u32(&mut out, VERSION);
        out.extend_from_slice(&(self.frames.len() as u64).to_le_bytes());
        out.extend_from_slice(&(self.docks.len() as u64).to_le_bytes());
        for (&id, frame) in &self.frames {
            write_u32(&mut out, id.0);
            for v in frame
                .pose
                .origin
                .into_iter()
                .chain([frame.pose.cos_yaw, frame.pose.sin_yaw])
            {
                write_u32(&mut out, v.to_bits());
            }
            out.extend_from_slice(&binding(&frame.nav));
        }
        for dock in self.docks.values() {
            write_u32(&mut out, dock.spec.id.0);
            for p in [dock.spec.a, dock.spec.b] {
                write_u32(&mut out, p.frame.0);
                for v in p.position {
                    write_u32(&mut out, v.to_bits());
                }
            }
            write_u32(&mut out, dock.spec.radius.to_bits());
            out.push(u8::from(dock.enabled) | (u8::from(dock.spec.bidirectional) << 1));
        }
        let checksum = content_hash(&out);
        out.extend_from_slice(&checksum);
        out
    }

    /// Restores poses and docks atomically into the same set of restored meshes.
    /// An unsupported version returns `InvalidSnapshot(WrongVersion)`.
    pub fn restore(&mut self, bytes: &[u8]) -> Result<(), Error> {
        let mut in_bytes = Input(bytes);
        if &in_bytes.take::<4>()? != MAGIC {
            return Err(Error::InvalidSnapshot(SnapshotError::WrongMagic));
        }
        if in_bytes.u32()? != VERSION {
            return Err(Error::InvalidSnapshot(SnapshotError::WrongVersion));
        }
        let frame_count = in_bytes.count()?;
        let dock_count = in_bytes.count()?;
        if frame_count != self.frames.len() {
            return Err(bad("frame count"));
        }
        let expected = frame_count
            .checked_mul(FRAME_BYTES)
            .and_then(|f| {
                dock_count
                    .checked_mul(DOCK_BYTES)
                    .and_then(|d| f.checked_add(d))
            })
            .and_then(|n| n.checked_add(32))
            .ok_or_else(|| bad("snapshot size"))?;
        if in_bytes.0.len() < expected {
            return Err(Error::InvalidSnapshot(SnapshotError::TooShort));
        }
        if in_bytes.0.len() > expected {
            return Err(Error::InvalidSnapshot(SnapshotError::TrailingBytes));
        }
        let checksum_at = bytes.len() - 32;
        if content_hash(&bytes[..checksum_at]) != bytes[checksum_at..] {
            return Err(bad("frame checksum"));
        }

        let mut poses = Vec::new();
        poses
            .try_reserve_exact(frame_count)
            .map_err(|_| Error::AllocationFailed)?;
        for (&id, frame) in &self.frames {
            if in_bytes.u32()? != id.0 {
                return Err(bad("frame id"));
            }
            let pose = FramePose::new(in_bytes.point()?, in_bytes.float()?, in_bytes.float()?)
                .map_err(|_| bad("frame pose"))?;
            if id == FrameId::WORLD && pose != FramePose::IDENTITY {
                return Err(bad("world pose"));
            }
            if binding(&frame.nav) != in_bytes.take::<32>()? {
                return Err(bad("frame mesh binding"));
            }
            poses.push(pose);
        }

        let mut docks = BTreeMap::new();
        for _ in 0..dock_count {
            let id = LinkId(in_bytes.u32()?);
            if docks.last_key_value().is_some_and(|(&last, _)| last >= id) {
                return Err(bad("dock order"));
            }
            let a = in_bytes.frame_point()?;
            let b = in_bytes.frame_point()?;
            let mut spec = DockSpec::new(id, a, b, in_bytes.float()?);
            let flags = in_bytes.take::<1>()?[0];
            if flags > 3 {
                return Err(bad("dock flags"));
            }
            spec.bidirectional = flags & 2 != 0;
            self.validate_dock(spec).map_err(|_| bad("dock geometry"))?;
            docks.insert(
                id,
                Dock {
                    spec,
                    enabled: flags & 1 != 0,
                },
            );
        }
        self.docks = docks;
        for (frame, pose) in self.frames.values_mut().zip(poses) {
            frame.pose = pose;
        }
        Ok(())
    }
}

fn binding(frame: &FrameNav) -> [u8; 32] {
    let nav = match frame {
        FrameNav::Native(nav) => nav,
        #[cfg(all(feature = "recast", feature = "crowd-world"))]
        FrameNav::Profiles(meshes) => return meshes.frame_binding(),
    };
    let mut data = nav.residency().to_bytes();
    let params = nav.params();
    for v in params
        .orig
        .into_iter()
        .chain([params.tile_width, params.tile_height])
    {
        write_u32(&mut data, v.to_bits());
    }
    data.extend_from_slice(&nav.runtime_links_to_bytes());
    content_hash(&data)
}

fn write_u32(out: &mut Vec<u8>, v: u32) {
    out.extend_from_slice(&v.to_le_bytes());
}

fn bad(what: &'static str) -> Error {
    Error::InvalidSnapshot(SnapshotError::InvalidValue(what))
}

struct Input<'a>(&'a [u8]);

impl Input<'_> {
    fn take<const N: usize>(&mut self) -> Result<[u8; N], Error> {
        let (head, rest) = self
            .0
            .split_first_chunk::<N>()
            .ok_or(Error::InvalidSnapshot(SnapshotError::TooShort))?;
        self.0 = rest;
        Ok(*head)
    }

    fn u32(&mut self) -> Result<u32, Error> {
        Ok(u32::from_le_bytes(self.take()?))
    }

    fn count(&mut self) -> Result<usize, Error> {
        usize::try_from(u64::from_le_bytes(self.take()?)).map_err(|_| bad("snapshot count"))
    }

    fn float(&mut self) -> Result<f32, Error> {
        Ok(f32::from_bits(self.u32()?))
    }

    fn point(&mut self) -> Result<Point, Error> {
        Ok([self.float()?, self.float()?, self.float()?])
    }

    fn frame_point(&mut self) -> Result<FramePoint, Error> {
        Ok(FramePoint::new(FrameId(self.u32()?), self.point()?))
    }
}
