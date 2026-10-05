//! The part of a picked edge's round that runs out through its END FACE at a mitred pair corner
//! ([`PairCornerEndFaceField`]), and the scope that turns it off for a flow
//! ([`PairCornerEndFaceFieldScope`]). Ports the types of
//! `Bevel/LevelLine/PairCornerEndFaceField.cs`, with the bridges of a vertex graph ([`bridges`]); the
//! construction and field queries are in `pair_corner_end_face_field_queries.rs`, and the finder
//! lands with a later package. The per-flow gate is a switch on the build's [`WorkBudget`](super::WorkBudget).

use std::collections::{HashMap, HashSet};

use vector_math::Vector3;

use super::{triangle_edges, LevelLineWork};

/// C# `PairCornerEndFaceField`: the part of a picked edge's round that runs out through its END
/// FACE at a mitred pair corner - negative in the prism the round's cylinder leaves in front of it
/// (between the two tangent planes through the round's axis, outside the cylinder), so a group
/// holding that end face reads the least of it and its pick field (`BandGroupFields`) and cuts the
/// end face along the round.
///
/// **Where it applies** (`Find`): two members meeting at a corner vertex with a turn between them,
/// sharing exactly ONE face, whose other sides are two different faces joined through the corner
/// by an unselected convex crease - Bevel to Center Feature's right tip (x = 20 face shared, slope
/// and top unshared). The shared face is mitred as it always was; nothing cut the unshared END face
/// with the member's round, so the top face kept a wedge in front of the tip's round
/// (`BevelToCenterPairCornerTests`).
///
/// **The field.** `max(size - r, -(q . m1), -(q . m2), bounds)`, where q is the point's offset
/// from the round's axis across the edge, r its length, m1, m2 the directions from the axis to the
/// two tangent lines, and bounds four linear pieces that end it at the corner's end region.
/// Continuous, and zero on the cylinder inside the prism. Not convex (size - r is concave), so the
/// least and greatest over a piece are not golden-section searched as a run-out's are: they are
/// read exactly at the piece's corners, where pieces tie, and where it comes nearest the axis.
#[derive(Debug, Clone, PartialEq, Default)]
pub struct PairCornerEndFaceField {
    /// A point on the round's axis.
    pub axis_point: Vector3,
    /// The round's unit axis direction.
    pub axis_direction: Vector3,
    /// The round's radius.
    pub size: f64,
    /// The linear pieces, each `gradient . p + constant`: the two tangent planes, then the bounds
    /// of the corner's end region.
    pub planes: Vec<(Vector3, f64)>,
}

/// C# `PairCornerEndFaceField.Scope`: turns the end-face field off for the build in force until
/// it is dropped, which puts back what was there before. The switch lives on the build's
/// [`WorkBudget`](super::WorkBudget) (C#'s `AsyncLocal`, per the bevel-core decision that per-flow switches travel
/// with the build); with no budget in force there is no build to switch and the scope does nothing.
#[derive(Debug)]
#[must_use = "the field stays off only while the scope is alive"]
pub struct PairCornerEndFaceFieldScope {
    /// Whether the field was off when the scope was opened - restored when it closes.
    pub was: bool,
}

impl PairCornerEndFaceFieldScope {
    /// C#'s `Scope()`: remembers the setting and turns the field off.
    pub fn new() -> Self {
        let was = LevelLineWork::current()
            .is_some_and(|budget| budget.pair_corner_end_face_field_off.replace(true));
        PairCornerEndFaceFieldScope { was }
    }
}

impl Default for PairCornerEndFaceFieldScope {
    fn default() -> Self {
        Self::new()
    }
}

impl Drop for PairCornerEndFaceFieldScope {
    /// C#'s `Dispose`: puts back what was there before.
    fn drop(&mut self) {
        if let Some(budget) = LevelLineWork::current() {
            budget.pair_corner_end_face_field_off.set(self.was);
        }
    }
}

/// The bridges of an undirected vertex graph (each vertex's neighbours listed both ways round),
/// as (low, high) keys: the edges whose removal disconnects their component. An iterative
/// Tarjan walk; the set is a property of the graph, so the order the roots are taken in does not
/// reach it.
pub fn bridges(neighbours: &HashMap<i32, Vec<i32>>) -> HashSet<(i32, i32)> {
    let mut bridges = HashSet::new();
    let mut order: HashMap<i32, i32> = HashMap::new();
    let mut low: HashMap<i32, i32> = HashMap::new();
    let mut counter = 0;
    for &root in neighbours.keys() {
        if order.contains_key(&root) {
            continue;
        }

        // (vertex, parent, next neighbour index)
        let mut stack: Vec<(i32, i32, usize)> = Vec::new();
        order.insert(root, counter);
        low.insert(root, counter);
        counter += 1;
        stack.push((root, -1, 0));
        while let Some((at, parent, next)) = stack.pop() {
            let list = &neighbours[&at];
            if next < list.len() {
                stack.push((at, parent, next + 1));
                let to = list[next];
                if to == parent {
                    continue;
                }

                if let Some(&seen) = order.get(&to) {
                    let here = low[&at].min(seen);
                    low.insert(at, here);
                } else {
                    order.insert(to, counter);
                    low.insert(to, counter);
                    counter += 1;
                    stack.push((to, at, 0));
                }

                continue;
            }

            if parent >= 0 {
                let here = low[&parent].min(low[&at]);
                low.insert(parent, here);
                if low[&at] > order[&parent] {
                    bridges.insert(triangle_edges::key(parent, at));
                }
            }
        }
    }

    bridges
}

impl PairCornerEndFaceField {
    /// Whether the end-face field is on for the build in force (on wherever no budget is, as C#'s
    /// flow default is).
    pub fn on() -> bool {
        LevelLineWork::current().is_none_or(|budget| !budget.pair_corner_end_face_field_off.get())
    }

    /// Turns the field off for the build in force until the returned scope is dropped.
    pub fn turn_off() -> PairCornerEndFaceFieldScope {
        PairCornerEndFaceFieldScope::new()
    }

    /// A point on the round's axis.
    pub fn axis_point(&self) -> Vector3 {
        self.axis_point
    }

    /// The triangles of `indices` that use `vertex`, in order.
    pub fn triangles_at(
        indices: &[(i32, i32, i32)],
        vertex: i32,
    ) -> impl Iterator<Item = usize> + '_ {
        indices
            .iter()
            .enumerate()
            .filter(move |(_, &(a, b, c))| a == vertex || b == vertex || c == vertex)
            .map(|(t, _)| t)
    }
}
