use super::*;

const MAGIC: [u8; 4] = *b"NPFM";
const VERSION: u32 = 1;

impl FramedMixedProfileCrowdWorld {
    /// Saves the attached simulations and stable rider bindings. Save the
    /// matching frame state and profile residency records alongside this record.
    pub fn save(&self, frames: &FrameSet) -> Result<Vec<u8>, ProfileError> {
        let mut bytes = Vec::new();
        bytes.extend(MAGIC);
        bytes.extend(VERSION.to_le_bytes());
        bytes.extend(content_hash(&frames.to_bytes()));
        bytes.extend((self.crowds.len() as u64).to_le_bytes());
        bytes.extend((self.riders.len() as u64).to_le_bytes());
        bytes.extend((self.waits.len() as u64).to_le_bytes());
        for (&frame, crowd) in &self.crowds {
            let local = crowd.save(frame_meshes(frames, frame)?)?;
            bytes.extend(frame.0.to_le_bytes());
            bytes.extend((local.len() as u64).to_le_bytes());
            bytes.extend(local);
        }
        for (&id, rider) in &self.riders {
            bytes.extend(id.0.to_le_bytes());
            bytes.extend(rider.frame.0.to_le_bytes());
            bytes.extend(rider.handle.index().to_le_bytes());
            bytes.extend(rider.handle.generation().to_le_bytes());
            bytes.push(u8::from(rider.goal.is_some()));
            if let Some(goal) = rider.goal {
                bytes.extend(goal.frame.0.to_le_bytes());
                for coord in goal.position {
                    bytes.extend(coord.to_bits().to_le_bytes());
                }
            }
        }
        for (&id, &cost) in &self.waits {
            bytes.extend(id.0.to_le_bytes());
            bytes.extend(cost.to_bits().to_le_bytes());
        }
        bytes.extend(content_hash(&bytes));
        Ok(bytes)
    }

    /// Restores a framed crowd after restoring its frame state and all required
    /// catalogs, resident tiles and modifiers.
    pub fn restore(frames: &FrameSet, bytes: &[u8], limits: Limits) -> Result<Self, ProfileError> {
        let mut body = checked_payload(bytes, MAGIC, VERSION, limits)?;
        if take::<32>(&mut body)? != content_hash(&frames.to_bytes()) {
            return Err(ProfileError::BindingMismatch);
        }
        let frame_count = u64::from_le_bytes(take(&mut body)?);
        let rider_count = u64::from_le_bytes(take(&mut body)?);
        let wait_count = u64::from_le_bytes(take(&mut body)?);
        let mut out = Self::default();
        for _ in 0..frame_count {
            let id = FrameId(u32::from_le_bytes(take(&mut body)?));
            if out
                .crowds
                .last_key_value()
                .is_some_and(|(&last, _)| last >= id)
            {
                return Err(ProfileError::InvalidRecord);
            }
            let size = usize::try_from(u64::from_le_bytes(take(&mut body)?))
                .map_err(|_| ProfileError::InvalidRecord)?;
            let (local, rest) = body
                .split_at_checked(size)
                .ok_or(ProfileError::InvalidRecord)?;
            body = rest;
            let crowd = MixedProfileCrowdWorld::restore(frame_meshes(frames, id)?, local, limits)?;
            out.check_time_step(crowd.crowd.params().fixed_dt)?;
            out.crowds.insert(id, crowd);
        }
        if rider_count != out.crowds.values().map(|w| w.crowd.len() as u64).sum() {
            return Err(ProfileError::InvalidRecord);
        }
        let mut handles = BTreeSet::new();
        for _ in 0..rider_count {
            let id = PlatformAgentId(u64::from_le_bytes(take(&mut body)?));
            if out
                .riders
                .last_key_value()
                .is_some_and(|(&last, _)| last >= id)
            {
                return Err(ProfileError::InvalidRecord);
            }
            let frame = FrameId(u32::from_le_bytes(take(&mut body)?));
            let index = u32::from_le_bytes(take(&mut body)?);
            let generation = u32::from_le_bytes(take(&mut body)?);
            let handle = out
                .crowds
                .get(&frame)
                .and_then(|w| w.crowd.handle_at(index))
                .filter(|h| h.generation() == generation)
                .ok_or(ProfileError::InvalidRecord)?;
            if !handles.insert((frame, handle)) {
                return Err(ProfileError::InvalidRecord);
            }
            out.riders.insert(
                id,
                Rider {
                    frame,
                    handle,
                    goal: None,
                },
            );
            match take::<1>(&mut body)? {
                [0] => {}
                [1] => {
                    let dest = FrameId(u32::from_le_bytes(take(&mut body)?));
                    let mut position = [0.; 3];
                    for coord in &mut position {
                        *coord = f32::from_bits(u32::from_le_bytes(take(&mut body)?));
                    }
                    out.set_goal(frames, id, Some(FramePoint::new(dest, position)))?;
                }
                _ => return Err(ProfileError::InvalidRecord),
            }
        }
        for _ in 0..wait_count {
            let id = LinkId(u32::from_le_bytes(take(&mut body)?));
            if out
                .waits
                .last_key_value()
                .is_some_and(|(&last, _)| last >= id)
            {
                return Err(ProfileError::InvalidRecord);
            }
            let cost = f32::from_bits(u32::from_le_bytes(take(&mut body)?));
            out.set_wait_cost(frames, id, Some(cost))?;
        }
        if !body.is_empty() {
            return Err(ProfileError::InvalidRecord);
        }
        Ok(out)
    }
}
