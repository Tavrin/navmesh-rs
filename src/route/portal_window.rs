//! Live search with one label per incoming polygon portal.
use super::{SectorKey, SectorParams, WindowPoint, summary::entry_mid};
use crate::detour::{DefaultFilter, NULL_LINK, NavMesh, PolyRef, QueryFilter};
use crate::{Budget, Error};
use alloc::{boxed::Box, collections::BinaryHeap, vec::Vec};
use core::cmp::Reverse;

struct Node {
    point: WindowPoint,
    previous: PolyRef,
    next: u32,
    bucket: u32,
    cost: f32,
    parent: usize,
}

// Retain populated pages between windows. A new search writes over old nodes
// in index order, so restarting costs the same regardless of its predecessor.
const NODES_PER_PAGE: usize = 64;
const HEADS_PER_PAGE: usize = 256;

#[derive(Default)]
struct NodePages {
    pages: Vec<Vec<Node>>,
    len: usize,
}
impl NodePages {
    fn push(&mut self, node: Node) {
        let page_index = self.len / NODES_PER_PAGE;
        let offset = self.len % NODES_PER_PAGE;
        if page_index == self.pages.len() {
            self.pages.push(Vec::with_capacity(NODES_PER_PAGE));
        }
        let page = &mut self.pages[page_index];
        if offset == page.len() {
            page.push(node);
        } else {
            page[offset] = node;
        }
        self.len += 1;
    }
}
impl core::ops::Index<usize> for NodePages {
    type Output = Node;
    fn index(&self, i: usize) -> &Self::Output {
        &self.pages[i / NODES_PER_PAGE][i % NODES_PER_PAGE]
    }
}
impl core::ops::IndexMut<usize> for NodePages {
    fn index_mut(&mut self, i: usize) -> &mut Self::Output {
        &mut self.pages[i / NODES_PER_PAGE][i % NODES_PER_PAGE]
    }
}

pub(super) struct PortalWindow {
    sectors: Vec<SectorKey>,
    target: WindowPoint,
    minimum: f32,
    head_mask: usize,
    heads: Vec<Option<Box<[u32; HEADS_PER_PAGE]>>>,
    nodes: NodePages,
    open: BinaryHeap<Reverse<(u32, u32, usize)>>,
    best: usize,
    expanded: usize,
    active: Option<(usize, u32)>,
}

impl PortalWindow {
    pub(super) fn new(
        start: WindowPoint,
        target: WindowPoint,
        sectors: Vec<SectorKey>,
        filter: &DefaultFilter,
        limit: usize,
    ) -> Self {
        let buckets = (limit / 2).max(1).next_power_of_two();
        let mut window = Self {
            sectors: Vec::new(),
            target,
            minimum: filter
                .area_cost
                .iter()
                .copied()
                .fold(f32::INFINITY, f32::min),
            head_mask: buckets - 1,
            heads: alloc::vec![None; buckets.div_ceil(HEADS_PER_PAGE)],
            nodes: NodePages::default(),
            open: BinaryHeap::new(),
            best: 0,
            expanded: 0,
            active: None,
        };
        window.restart(start, target, sectors);
        window
    }
    pub(super) fn restart(
        &mut self,
        start: WindowPoint,
        target: WindowPoint,
        sectors: Vec<SectorKey>,
    ) {
        self.target = target;
        self.sectors = sectors;
        self.best = 0;
        self.expanded = 0;
        self.active = None;
        self.nodes.len = 0;
        self.nodes.push(Node {
            point: start,
            previous: 0,
            next: 0,
            bucket: u32::MAX,
            cost: 0.,
            parent: usize::MAX,
        });
        self.open.clear();
        self.open.push(Reverse((0, 0, 0)));
    }
    /// During the initial slice, return a short walkable prefix inside the
    /// reciprocal start region. Later windows search across all included borders.
    #[allow(clippy::too_many_arguments)]
    pub(super) fn resume(
        &mut self,
        nav: &NavMesh,
        params: &SectorParams,
        filter: &DefaultFilter,
        budget: &mut Budget,
        limit: usize,
        prefix: bool,
        allowed: impl Fn(PolyRef) -> bool,
    ) -> Result<Option<(Vec<PolyRef>, WindowPoint)>, Error> {
        while budget.spend(1) {
            // Popping a state and examining one outgoing link are separate
            // work units, just as they are on the abstract portal graph.
            let Some((id, link)) = self.active else {
                let Reverse((_, bits, id)) = self.open.pop().ok_or(Error::Failed)?;
                let node = &self.nodes[id];
                if node.cost.to_bits() != bits {
                    continue;
                }
                if node.point.polygon == self.target.polygon {
                    return Ok(Some((self.corridor(id), self.target)));
                }
                if prefix && self.expanded >= 24 && self.best != 0 {
                    return Ok(Some((
                        self.corridor(self.best),
                        self.nodes[self.best].point,
                    )));
                }
                self.expanded += 1;
                let first = nav.tile_and_poly_by_ref(node.point.polygon)?.1.first_link;
                self.active = (first != NULL_LINK).then_some((id, first));
                continue;
            };
            let node = &self.nodes[id];
            let point = node.point;
            let g = node.cost;
            let (tile, poly) = nav.tile_and_poly_by_ref(point.polygon)?;
            let e = tile.links.get(link as usize).ok_or(Error::InvalidRef)?;
            self.active = (e.next != NULL_LINK).then_some((id, e.next));
            if e.r#ref == 0 {
                continue;
            }
            let dest = nav.tile_and_poly_by_ref(e.r#ref)?;
            if !filter.pass_filter(e.r#ref, dest.0, dest.1)
                || self
                    .sectors
                    .binary_search(&params.sector_of_tile(dest.0.header.x, dest.0.header.y))
                    .is_err()
                || !allowed(e.r#ref)
            {
                continue;
            }
            let mid = entry_mid((tile, poly), point.polygon, e, dest).ok_or(Error::InvalidRef)?;
            let mut cost = g + super::navigator::distance(point.position, mid)
                * filter.area_cost[poly.area() as usize];
            let h = super::navigator::distance(mid, self.target.position);
            let goal = e.r#ref == self.target.polygon;
            if goal {
                cost += h * filter.area_cost[dest.1.area() as usize];
            }
            // Each state names an incoming directed edge. Collisions link
            // through node indices and never affect the search order.
            #[allow(clippy::unnecessary_cast)]
            let hash = (e.r#ref as u64).wrapping_mul(0x9e3779b97f4a7c15)
                ^ (point.polygon as u64).rotate_left(27);
            let bucket = ((hash ^ (hash >> 32)) as usize) & self.head_mask;
            // After a restart the first insertion into a bucket must find
            // an empty chain. A head can name a current node of this bucket
            // only if an insertion has already overwritten it in this window.
            let page = self.heads[bucket / HEADS_PER_PAGE]
                .get_or_insert_with(|| Box::new([0; HEADS_PER_PAGE]));
            let slot = &mut page[bucket % HEADS_PER_PAGE];
            let head = *slot;
            let head = if (head as usize) < self.nodes.len
                && self.nodes[head as usize].bucket == bucket as u32
            {
                head
            } else {
                0
            };
            let mut index = head;
            while index != 0 {
                let old = &self.nodes[index as usize];
                if old.point.polygon == e.r#ref && old.previous == point.polygon {
                    break;
                }
                index = old.next;
            }
            if index == 0 {
                if self.nodes.len >= limit {
                    return Err(Error::LimitExceeded);
                }
                index = self.nodes.len as u32;
                self.nodes.push(Node {
                    point: WindowPoint::new(e.r#ref, mid),
                    previous: point.polygon,
                    next: head,
                    bucket: bucket as u32,
                    cost,
                    parent: id,
                });
                *slot = index;
            } else if self.nodes[index as usize].cost <= cost {
                continue;
            }
            let index = index as usize;
            self.nodes[index].cost = cost;
            self.nodes[index].parent = id;
            self.open.push(Reverse((
                (cost + if goal { 0. } else { h * self.minimum }).to_bits(),
                cost.to_bits(),
                index,
            )));
            if h < super::navigator::distance(
                self.nodes[self.best].point.position,
                self.target.position,
            ) {
                self.best = index;
            }
        }
        Ok(None)
    }

    fn corridor(&self, mut id: usize) -> Vec<PolyRef> {
        let mut path = Vec::new();
        while id != usize::MAX {
            path.push(self.nodes[id].point.polygon);
            id = self.nodes[id].parent;
        }
        path.reverse();
        path
    }
}
