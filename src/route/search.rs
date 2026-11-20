//! Resumable coarse search. Coarse connectivity is a hint, not proof of a path.
use super::{SectorGraph, SuperKey};
use crate::{Budget, Error, Progress};
use alloc::{collections::BinaryHeap, vec, vec::Vec};
use core::cmp::Reverse;

/// Stable address of a region in a super-sector record.
#[derive(Clone, Copy, Debug, PartialEq, Eq, PartialOrd, Ord)]
#[non_exhaustive]
pub struct RegionKey {
    /// Super-sector coordinates.
    pub sector: SuperKey,
    /// Region index in that record.
    pub region: u16,
}
impl RegionKey {
    /// Addresses a region in a super-sector record.
    pub fn new(sector: SuperKey, region: u16) -> Self {
        Self { sector, region }
    }
}

/// Immutable, array-indexed view of one profile's level-3 graph.
///
/// Build outside the frame loop after loading records. Searches borrow this view;
/// replacing it invalidates outstanding searches through an integer revision.
/// The region graph is an optimistic corridor hint: directed connections and
/// overlapping region boxes in summary construction can overestimate connectivity.
#[derive(Clone, Debug)]
#[non_exhaustive]
pub struct RouteIndex {
    keys: Vec<RegionKey>,
    edges: Vec<Vec<(usize, f32)>>,
    revision: u64,
    profile: u32,
}
impl RouteIndex {
    /// Builds the directed index for stitched records. Missing destinations and
    /// records derived solely from isolated sectors are rejected.
    pub fn build_exact(graph: &SectorGraph, profile: u32) -> Result<Self, Error> {
        for record in graph.records(profile) {
            if !record.is_stitched()
                || record
                    .destinations()
                    .any(|(k, r)| graph.super_region(profile, k, r).is_none())
            {
                return Err(Error::InvalidInput("incomplete stitched graph"));
            }
        }
        Self::build(graph, profile)
    }
    /// Compiles the current graph, rejecting stale records and invalid costs.
    pub fn build(graph: &SectorGraph, profile: u32) -> Result<Self, Error> {
        let mut keys = Vec::new();
        for record in graph.records(profile) {
            if graph.is_stale(profile, record.key()) == Some(true) {
                return Err(Error::MeshChanged);
            }
            for region in 0..record.regions().len() {
                keys.push(RegionKey::new(record.key(), region as u16));
            }
        }
        keys.sort_unstable();
        let mut edges = vec![Vec::new(); keys.len()];
        for (i, key) in keys.iter().enumerate() {
            for edge in graph.super_edges(profile, key.sector, key.region) {
                if !edge.cost.is_finite() || edge.cost < 0.0 {
                    return Err(Error::InvalidInput("coarse edge cost"));
                }
                if let Ok(j) = keys.binary_search(&RegionKey::new(edge.to, edge.region)) {
                    edges[i].push((j, edge.cost));
                }
            }
            edges[i].sort_unstable_by_key(|e| e.0);
        }
        Ok(Self {
            keys,
            edges,
            revision: graph.revision(),
            profile,
        })
    }

    /// Starts a coarse query. Admission allocates labels and is not frame-budgeted.
    /// Missing records are reported as invalid input rather than disconnection.
    pub fn request(&self, start: RegionKey, goal: RegionKey) -> Result<CoarseSearch<'_>, Error> {
        let locate = |key| {
            self.keys
                .binary_search(&key)
                .map_err(|_| Error::InvalidInput("missing coarse region"))
        };
        let start = locate(start)?;
        let goal = locate(goal)?;
        let mut distance = vec![f32::INFINITY; self.keys.len()];
        distance[start] = 0.0;
        let mut open = BinaryHeap::new();
        open.push(Reverse((0u32, start)));
        Ok(CoarseSearch {
            index: self,
            revision: self.revision,
            profile: self.profile,
            goal,
            distance,
            parent: vec![usize::MAX; self.keys.len()],
            open,
            active: None,
            trace: None,
            path: Vec::new(),
            done: false,
        })
    }
}

/// A coarse query with stable tie-breaking and array-indexed labels.
///
/// One work unit pops a heap entry, relaxes one edge, or emits one path node.
/// Dijkstra is deliberate: Euclidean distance is not an admissible heuristic for
/// region crossing lower bounds. Results are invariant to budget slice sizes.
#[derive(Clone, Debug)]
#[non_exhaustive]
pub struct CoarseSearch<'a> {
    index: &'a RouteIndex,
    revision: u64,
    profile: u32,
    goal: usize,
    distance: Vec<f32>,
    parent: Vec<usize>,
    open: BinaryHeap<Reverse<(u32, usize)>>,
    active: Option<(usize, usize)>,
    trace: Option<usize>,
    path: Vec<RegionKey>,
    done: bool,
}
impl CoarseSearch<'_> {
    /// Resumes, rejecting any intervening graph mutation. Zero budget does no work.
    pub fn resume(&mut self, graph: &SectorGraph, budget: &mut Budget) -> Result<Progress, Error> {
        let index = self.index;
        if graph.revision() != self.revision
            || index.revision != self.revision
            || index.profile != self.profile
        {
            return Err(Error::MeshChanged);
        }
        while !self.done && budget.spend(1) {
            if let Some(node) = self.trace {
                self.path.push(index.keys[node]);
                let parent = self.parent[node];
                if parent == usize::MAX {
                    self.done = true;
                    self.trace = None;
                } else {
                    self.trace = Some(parent);
                }
            } else if let Some((node, edge)) = self.active {
                if let Some(&(next, cost)) = index.edges[node].get(edge) {
                    self.active = Some((node, edge + 1));
                    let candidate = self.distance[node] + cost;
                    if candidate < self.distance[next] {
                        self.distance[next] = candidate;
                        self.parent[next] = node;
                        self.open.push(Reverse((candidate.to_bits(), next)));
                    }
                } else {
                    self.active = None;
                }
            } else if let Some(Reverse((cost, node))) = self.open.pop() {
                if cost != self.distance[node].to_bits() {
                    continue;
                }
                if node == self.goal {
                    self.trace = Some(node);
                } else {
                    self.active = Some((node, 0));
                }
            } else {
                self.done = true;
            }
        }
        Ok(if self.done {
            Progress::Done
        } else {
            Progress::Exhausted
        })
    }

    /// Completed coarse corridor, in goal-to-start order to avoid an unbudgeted
    /// reversal. Empty means no route in the available coarse graph, not a proof
    /// that the live mesh is disconnected. `None` means still planning.
    pub fn corridor_reversed(&self) -> Option<&[RegionKey]> {
        self.done.then_some(self.path.as_slice())
    }
}
