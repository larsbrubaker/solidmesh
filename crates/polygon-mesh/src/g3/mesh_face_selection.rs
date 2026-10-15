//! A triangle selection (derived from geometry3Sharp), the part the mesh repair's erosion step
//! uses: selecting triangles and the triangles on an edge, then reading them back in the order
//! they were selected. That is the order `mesh_editor::remove_triangles` then removes them in,
//! and removal order decides which ids the mesh reuses, so the selection keeps it.

use std::collections::HashSet;

use super::dmesh3::{DMesh3, INVALID_ID};

/// A set of triangle ids that only grows, read back in selection order.
#[derive(Debug, Clone, Default)]
pub(crate) struct MeshFaceSelection {
    order: Vec<i32>,
    selected: HashSet<i32>,
}

impl MeshFaceSelection {
    /// An empty selection.
    pub(crate) fn new() -> Self {
        Self::default()
    }

    fn add(&mut self, tid: i32) {
        if self.selected.insert(tid) {
            self.order.push(tid);
        }
    }

    #[allow(dead_code)] // mesh API only the tests call so far
    /// Selects triangle `tid` if it is live.
    pub(crate) fn select(&mut self, mesh: &DMesh3, tid: i32) {
        if mesh.is_triangle(tid) {
            self.add(tid);
        }
    }

    /// Selects the triangles on either side of edge `eid`.
    pub(crate) fn select_edge_tris(&mut self, mesh: &DMesh3, eid: i32) {
        let [a, b] = mesh.get_edge_t(eid);
        self.add(a);
        if b != INVALID_ID {
            self.add(b);
        }
    }

    #[allow(dead_code)] // mesh API only the tests call so far
    /// Whether triangle `tid` is selected.
    pub(crate) fn is_selected(&self, tid: i32) -> bool {
        self.selected.contains(&tid)
    }

    #[allow(dead_code)] // mesh API only the tests call so far
    /// The number of selected triangles.
    pub(crate) fn count(&self) -> usize {
        self.order.len()
    }

    /// The selected triangles, in selection order.
    pub(crate) fn iter(&self) -> impl Iterator<Item = i32> + '_ {
        self.order.iter().copied()
    }
}
