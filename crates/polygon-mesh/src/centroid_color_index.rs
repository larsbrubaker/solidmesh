//! A kd-tree over saved face centroids ([`CentroidColor`]s) answering "the colour of the nearest
//! one" exactly as [`Mesh::find_nearest_centroid_color`]'s scan does, for
//! [`Mesh::restore_face_colors_from_centroids`] in `face_data.rs`: a rebuilt mesh of a hundred
//! thousand faces asks once per face, and the scan over every saved centroid made that quadratic.
//!
//! Same answer, bit for bit, as the scan:
//! - each candidate's distance is the scan's own `(point - centroid).length_squared()`;
//! - the winner is the smallest distance, ties going to the lowest saved-list index, and nothing
//!   wins unless it is under `f64::MAX` (the scan's starting best), so an empty list, or one
//!   whose every distance overflows, gives [`Mesh::UNKNOWN_FACE_COLOR`];
//! - a subtree is skipped only when its lower bound is strictly greater than the best distance
//!   so far, so an equally near centroid with a lower index is still visited. The bound is the
//!   squared gap from the point to the subtree's box, built from its members' own coordinates
//!   and summed in the same order as `length_squared`: rounding is monotonic, so it never
//!   exceeds the distance computed for any member. A NaN bound never prunes.
//!
//! The tree splits at the median of its widest axis (keys made unique by the saved index), so
//! duplicates, a single point, a flat or line-shaped box and huge coordinates all give a balanced
//! tree, and its shape is the same on every run.

use solidmesh_math::{Color, Vector3};

use crate::face_data::CentroidColor;
use crate::mesh::Mesh;

/// The most centroids a leaf holds before it is split.
const LEAF_SIZE: usize = 8;

/// A saved centroid and its index in the saved list.
type IndexedCentroid = (Vector3, usize);

enum NodeKind {
    /// The centroids in `points[start..end]`.
    Leaf { start: usize, end: usize },
    /// Indices into `nodes` of the two halves.
    Branch { low: usize, high: usize },
}

struct Node {
    /// The smallest and largest coordinate of the node's centroids on each axis.
    min: [f64; 3],
    max: [f64; 3],
    kind: NodeKind,
}

/// The nearest-centroid index over one saved list; build once, ask once per face.
pub(crate) struct CentroidColorIndex<'a> {
    saved: &'a [CentroidColor],
    /// The saved centroids, reordered so each leaf's are contiguous.
    points: Vec<IndexedCentroid>,
    /// The tree, root first; empty for an empty saved list.
    nodes: Vec<Node>,
}

/// The best candidate so far, under the scan's rule.
struct Best {
    dist_sq: f64,
    index: Option<usize>,
}

impl Best {
    fn offer(&mut self, dist_sq: f64, index: usize) {
        let better = match self.index {
            None => dist_sq < self.dist_sq,
            Some(best_index) => {
                dist_sq < self.dist_sq || (dist_sq == self.dist_sq && index < best_index)
            }
        };
        if better {
            self.dist_sq = dist_sq;
            self.index = Some(index);
        }
    }

    /// Whether nothing at or beyond `lower_bound` can win.
    fn rules_out(&self, lower_bound: f64) -> bool {
        lower_bound > self.dist_sq
    }
}

impl<'a> CentroidColorIndex<'a> {
    pub(crate) fn new(saved: &'a [CentroidColor]) -> Self {
        let mut points: Vec<IndexedCentroid> = saved
            .iter()
            .enumerate()
            .map(|(index, &(centroid, _))| (centroid, index))
            .collect();
        let mut nodes = Vec::new();
        if !points.is_empty() {
            build(&mut points, 0, &mut nodes);
        }
        Self {
            saved,
            points,
            nodes,
        }
    }

    /// The colour [`Mesh::find_nearest_centroid_color`] gives for `point` over the same list.
    pub(crate) fn nearest_color(&self, point: Vector3) -> Color {
        let mut best = Best {
            dist_sq: f64::MAX,
            index: None,
        };
        if !self.nodes.is_empty() {
            self.search(0, point, &mut best);
        }
        best.index
            .map_or(Mesh::UNKNOWN_FACE_COLOR, |index| self.saved[index].1)
    }

    fn search(&self, node: usize, point: Vector3, best: &mut Best) {
        match self.nodes[node].kind {
            NodeKind::Leaf { start, end } => {
                for &(centroid, index) in &self.points[start..end] {
                    best.offer((point - centroid).length_squared(), index);
                }
            }
            NodeKind::Branch { low, high } => {
                let low_bound = lower_bound_sq(&self.nodes[low], point);
                let high_bound = lower_bound_sq(&self.nodes[high], point);
                // Nearer half first so the farther one is more often skipped; the answer does
                // not depend on the order.
                let (first, first_bound, second, second_bound) = if high_bound < low_bound {
                    (high, high_bound, low, low_bound)
                } else {
                    (low, low_bound, high, high_bound)
                };
                if !best.rules_out(first_bound) {
                    self.search(first, point, best);
                }
                if !best.rules_out(second_bound) {
                    self.search(second, point, best);
                }
            }
        }
    }
}

/// Builds the subtree over `points` (which start at `offset` in the full list), returning its
/// node index.
fn build(points: &mut [IndexedCentroid], offset: usize, nodes: &mut Vec<Node>) -> usize {
    let (min, max) = bounds(points);
    let id = nodes.len();
    nodes.push(Node {
        min,
        max,
        kind: NodeKind::Leaf {
            start: offset,
            end: offset + points.len(),
        },
    });
    if points.len() > LEAF_SIZE {
        let axis = widest_axis(min, max);
        let mid = points.len() / 2;
        points.select_nth_unstable_by(mid, |a, b| {
            coordinate(a.0, axis)
                .total_cmp(&coordinate(b.0, axis))
                .then(a.1.cmp(&b.1))
        });
        let (low_points, high_points) = points.split_at_mut(mid);
        let low = build(low_points, offset, nodes);
        let high = build(high_points, offset + mid, nodes);
        nodes[id].kind = NodeKind::Branch { low, high };
    }
    id
}

fn coordinate(v: Vector3, axis: usize) -> f64 {
    match axis {
        0 => v.x,
        1 => v.y,
        _ => v.z,
    }
}

/// The per-axis smallest and largest coordinates of a non-empty slice.
fn bounds(points: &[IndexedCentroid]) -> ([f64; 3], [f64; 3]) {
    let first = points[0].0;
    let mut min = [first.x, first.y, first.z];
    let mut max = min;
    for &(p, _) in &points[1..] {
        for (axis, value) in [p.x, p.y, p.z].into_iter().enumerate() {
            if value < min[axis] {
                min[axis] = value;
            }
            if value > max[axis] {
                max[axis] = value;
            }
        }
    }
    (min, max)
}

/// The axis the box is longest along; the first of equal ones.
fn widest_axis(min: [f64; 3], max: [f64; 3]) -> usize {
    let extent = [max[0] - min[0], max[1] - min[1], max[2] - min[2]];
    let mut axis = 0;
    for candidate in 1..3 {
        if extent[candidate] > extent[axis] {
            axis = candidate;
        }
    }
    axis
}

/// A lower bound on the scan's squared distance from `point` to any centroid in `node`: the
/// gap to its box on each axis, squared and summed as `length_squared` sums. Each gap is the
/// rounded difference to the box face, never more than the rounded difference to a member.
fn lower_bound_sq(node: &Node, point: Vector3) -> f64 {
    let gap = |p: f64, axis: usize| {
        if p > node.max[axis] {
            p - node.max[axis]
        } else if p < node.min[axis] {
            node.min[axis] - p
        } else {
            0.0
        }
    };
    let gx = gap(point.x, 0);
    let gy = gap(point.y, 1);
    let gz = gap(point.z, 2);
    gx * gx + gy * gy + gz * gz
}
