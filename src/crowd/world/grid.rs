//! Sorted-cell neighbour grid: every agent is one `(cell key, agent index)` entry,
//! stable radix-sorted by key each step. Keys order cells row by row, so the cells a
//! query box covers in one row are one contiguous run found by binary search. No
//! hash chains, no per-cell capacity and no 16-bit id cap.
use crate::Error;
use crate::common::floor_i32;
use alloc::vec::Vec;

pub(super) struct CellGrid {
    inv: f32,
    entries: Vec<(u64, u32)>,
    scratch: Vec<(u64, u32)>,
}

fn key(cx: i32, cz: i32) -> u64 {
    (u64::from(cz as u32 ^ 0x8000_0000) << 32) | u64::from(cx as u32 ^ 0x8000_0000)
}

impl CellGrid {
    pub(super) fn new(cell_size: f32) -> Self {
        Self {
            inv: 1. / cell_size,
            entries: Vec::new(),
            scratch: Vec::new(),
        }
    }

    fn cell(&self, x: f32) -> i32 {
        floor_i32(x * self.inv)
    }

    /// Rebuilds the grid from agent indices and planar positions `(x, z)`.
    pub(super) fn rebuild(
        &mut self,
        agents: impl ExactSizeIterator<Item = (u32, f32, f32)>,
    ) -> Result<(), Error> {
        self.entries.clear();
        self.entries
            .try_reserve(agents.len())
            .map_err(|_| Error::AllocationFailed)?;
        for (i, x, z) in agents {
            let k = key(self.cell(x), self.cell(z));
            self.entries.push((k, i));
        }
        self.scratch.clear();
        self.scratch
            .try_reserve(self.entries.len())
            .map_err(|_| Error::AllocationFailed)?;
        self.scratch.resize(self.entries.len(), (0, 0));
        radix_sort(&mut self.entries, &mut self.scratch);
        Ok(())
    }

    /// Calls `f` with every agent whose position lies in the box `pos ± range`
    /// (planar), possibly with some outside it; each agent at most once.
    pub(super) fn for_each_near(
        &self,
        x: f32,
        z: f32,
        range: f32,
        mut f: impl FnMut(usize) -> Result<(), Error>,
    ) -> Result<(), Error> {
        // Pad the box by a few float ulps of the coordinates so an agent whose
        // rounded distance passes the exact range test is never outside it.
        let pad = range * (1. / 1024.) + x.abs().max(z.abs()) * (1. / 1_048_576.) + 1.0e-6;
        let r = range + pad;
        let (x0, x1) = (self.cell(x - r), self.cell(x + r));
        let (z0, z1) = (self.cell(z - r), self.cell(z + r));
        // Jump between occupied rows; a wide query must not scan empty space.
        let mut row = self.entries.partition_point(|e| e.0 < key(i32::MIN, z0));
        while row < self.entries.len() {
            let cz = ((self.entries[row].0 >> 32) as u32 ^ 0x8000_0000) as i32;
            if cz > z1 {
                break;
            }
            let end = row + self.entries[row..].partition_point(|e| e.0 <= key(i32::MAX, cz));
            let start = row + self.entries[row..end].partition_point(|e| e.0 < key(x0, cz));
            for &(k, i) in &self.entries[start..end] {
                if k > key(x1, cz) {
                    break;
                }
                f(i as usize)?;
            }
            row = end;
        }
        Ok(())
    }
}

/// Stable least-significant-digit radix sort on the 64-bit key, one byte per pass;
/// passes whose byte is equal for every entry are skipped.
fn radix_sort(v: &mut [Entry], scratch: &mut [Entry]) {
    let mut src_is_v = true;
    for pass in 0..8 {
        let moved = if src_is_v {
            radix_pass(v, scratch, pass * 8)
        } else {
            radix_pass(scratch, v, pass * 8)
        };
        if moved {
            src_is_v = !src_is_v;
        }
    }
    if !src_is_v {
        v.copy_from_slice(scratch);
    }
}

type Entry = (u64, u32);

/// Stable counting pass on the key byte at `shift` from `src` into `dst`; `false`
/// (nothing written) when every entry has the same byte.
fn radix_pass(src: &[Entry], dst: &mut [Entry], shift: u32) -> bool {
    let mut count = [0usize; 256];
    for e in src {
        count[((e.0 >> shift) & 0xff) as usize] += 1;
    }
    if count.contains(&src.len()) {
        return false;
    }
    let mut sum = 0;
    for c in &mut count {
        let n = *c;
        *c = sum;
        sum += n;
    }
    for e in src {
        let d = ((e.0 >> shift) & 0xff) as usize;
        dst[count[d]] = *e;
        count[d] += 1;
    }
    true
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn wide_queries_visit_only_occupied_rows() {
        let mut g = CellGrid::new(1.);
        g.rebuild([(0, 0., -1.0e9), (1, 0., 1.0e9)].into_iter())
            .unwrap();
        let mut found = Vec::new();
        g.for_each_near(0., 0., f32::MAX, |i| {
            found.push(i);
            Ok(())
        })
        .unwrap();
        assert_eq!(found, [0, 1]);
    }

    #[test]
    fn sorts_by_cell_and_keeps_index_order_within_a_cell() {
        let mut g = CellGrid::new(1.);
        let pts = [
            (0, 5.5, -3.2),
            (1, -1.5, 0.5),
            (2, 5.1, -3.9),
            (3, -1.2, 0.1),
        ];
        g.rebuild(pts.into_iter()).unwrap();
        let order: Vec<u32> = g.entries.iter().map(|e| e.1).collect();
        assert_eq!(order, [0, 2, 1, 3]);
        let mut near = Vec::new();
        g.for_each_near(-1., 0., 0.6, |i| {
            near.push(i);
            Ok(())
        })
        .unwrap();
        assert_eq!(near, [1, 3]);
    }
}
