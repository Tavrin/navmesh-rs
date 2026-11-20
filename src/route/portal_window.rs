//! Live search with one label per incoming polygon portal.
use super::{SectorKey, SectorParams, WindowPoint, summary::entry_mid};
use crate::detour::{DefaultFilter, NULL_LINK, NavMesh, PolyRef, QueryFilter};
use crate::{Budget, Error};
use alloc::{collections::BinaryHeap, vec::Vec};
use core::cmp::Reverse;

struct Node {
    point: WindowPoint,
    previous: PolyRef,
    next: u32,
    bucket: u32,
    cost: f32,
    parent: usize,
}

pub(super) struct PortalWindow {
    sectors: Vec<SectorKey>,
    target: WindowPoint,
    minimum: f32,
    heads: Vec<u32>,
    nodes: Vec<Node>,
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
        let mut window = Self {
            sectors: Vec::new(),
            target,
            minimum: filter
                .area_cost
                .iter()
                .copied()
                .fold(f32::INFINITY, f32::min),
            heads: alloc::vec![0; (limit / 2).max(1).next_power_of_two()],
            nodes: Vec::with_capacity(limit),
            open: BinaryHeap::with_capacity(limit.min(4096)),
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
        self.nodes.clear();
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
            let bucket = ((hash ^ (hash >> 32)) as usize) & (self.heads.len() - 1);
            // After a restart the first insertion into a bucket must find
            // an empty chain. A head can name a current node of this bucket
            // only if an insertion has already overwritten it in this window.
            let head = self.heads[bucket];
            let head = if self
                .nodes
                .get(head as usize)
                .is_some_and(|n| n.bucket == bucket as u32)
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
                if self.nodes.len() >= limit {
                    return Err(Error::LimitExceeded);
                }
                index = self.nodes.len() as u32;
                self.nodes.push(Node {
                    point: WindowPoint::new(e.r#ref, mid),
                    previous: point.polygon,
                    next: head,
                    bucket: bucket as u32,
                    cost,
                    parent: id,
                });
                self.heads[bucket] = index;
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
