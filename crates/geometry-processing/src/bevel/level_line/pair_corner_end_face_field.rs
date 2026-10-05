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

impl PairCornerEndFaceField {
    /// C# `PairCornerEndFaceField.Find`: at every corner where exactly two picked chain edges meet
    /// on a bridge of the pick graph, share one face and turn CONVEX across an unpicked feature
    /// crease between their own faces, the end-face field of each edge whose round runs out
    /// through the other's face - filed under every group a triangle of that end face at the
    /// corner belongs to. `None` with the field off or nothing found. `standing` is the input's
    /// standing map where the caller already has one.
    #[allow(clippy::too_many_arguments)]
    pub fn find(
        positions: &[Vector3],
        indices: &[(i32, i32, i32)],
        chains: &[Vec<i32>],
        closed: &[bool],
        is_feature_edge: &dyn Fn(i32, i32) -> bool,
        size: f64,
        tags: &[i32],
        standing: Option<&triangle_edges::StandingMap>,
    ) -> Option<super::BandGroupExtraFields> {
        if !Self::on() {
            return None;
        }

        let mut chain_edges = HashSet::new();
        let mut neighbours: indexmap::IndexMap<i32, Vec<i32>> = indexmap::IndexMap::new();
        let mut add = |at: i32, to: i32| {
            let list = neighbours.entry(at).or_default();
            if !list.contains(&to) {
                list.push(to);
            }
        };

        for (k, chain) in chains.iter().enumerate() {
            let count = if closed[k] && chain.len() > 2 {
                chain.len()
            } else {
                chain.len() - 1
            };
            for i in 0..count {
                let next = chain[(i + 1) % chain.len()];
                chain_edges.insert(triangle_edges::key(chain[i], next));
                add(chain[i], next);
                add(next, chain[i]);
            }
        }

        let mut candidates: Vec<i32> = neighbours
            .iter()
            .filter(|(_, n)| n.len() == 2)
            .map(|(&k, _)| k)
            .collect();
        if candidates.is_empty() {
            return None;
        }

        let graph: HashMap<i32, Vec<i32>> =
            neighbours.iter().map(|(&k, v)| (k, v.clone())).collect();
        let bridges = bridges(&graph);
        candidates.retain(|&corner| {
            bridges.contains(&triangle_edges::key(corner, neighbours[&corner][0]))
        });
        if candidates.is_empty() {
            return None;
        }

        let built;
        let standing = match standing {
            Some(standing) => standing,
            None => {
                built = triangle_edges::standing_map(indices);
                &built
            }
        };

        let candidate_set: HashSet<i32> = candidates.iter().copied().collect();
        let mut edges_at: HashMap<i32, Vec<(i32, i32)>> = HashMap::new();
        for &edge in standing.keys() {
            for end in [edge.0, edge.1] {
                if candidate_set.contains(&end) {
                    edges_at.entry(end).or_default().push(edge);
                }
            }
        }

        let normal = |t: i32| {
            let (a, b, c) = indices[t as usize];
            let pa = positions[a as usize];
            (positions[b as usize] - pa)
                .cross(positions[c as usize] - pa)
                .get_normal()
        };
        let same = |n: Vector3, m: Vector3| n.dot(m) > 0.9999;
        let faces_on = |a: i32, b: i32| -> Vec<Vector3> {
            standing
                .get(&triangle_edges::key(a, b))
                .map_or_else(Vec::new, |ts| ts.iter().map(|&t| normal(t)).collect())
        };
        let triangles_at = |corner: i32| -> Vec<i32> {
            let mut seen = Vec::new();
            if let Some(edges) = edges_at.get(&corner) {
                for edge in edges {
                    for &t in &standing[edge] {
                        if !seen.contains(&t) {
                            seen.push(t);
                        }
                    }
                }
            }
            seen
        };

        let mut found: Option<super::BandGroupExtraFields> = None;
        for &corner in &candidates {
            let near = &neighbours[&corner];
            let faces0 = faces_on(corner, near[0]);
            let faces1 = faces_on(corner, near[1]);
            if faces0.len() != 2 || faces1.len() != 2 {
                continue;
            }

            let shared_pairs: Vec<Vector3> = faces0
                .iter()
                .flat_map(|&f0| {
                    faces1
                        .iter()
                        .filter(move |&&f1| same(f0, f1))
                        .map(move |_| f0)
                })
                .collect();
            if shared_pairs.len() != 1 {
                continue;
            }

            let shared = shared_pairs[0];
            let own0 = *faces0
                .iter()
                .find(|&&f| !same(f, shared))
                .expect("two faces");
            let own1 = *faces1
                .iter()
                .find(|&&f| !same(f, shared))
                .expect("two faces");
            if same(own0, own1) {
                continue;
            }

            let crease = edges_at.get(&corner).and_then(|edges| {
                edges
                    .iter()
                    .copied()
                    .filter(|e| !chain_edges.contains(e) && is_feature_edge(e.0, e.1))
                    .find(|e| {
                        let normals: Vec<Vector3> =
                            standing[e].iter().map(|&t| normal(t)).collect();
                        normals.len() == 2
                            && ((same(normals[0], own0) && same(normals[1], own1))
                                || (same(normals[0], own1) && same(normals[1], own0)))
                    })
            });
            let Some(crease) = crease else {
                continue;
            };

            let other = if crease.0 == corner {
                crease.1
            } else {
                crease.0
            };
            let crease_mid = (positions[corner as usize] + positions[other as usize]) / 2.0;
            let convex = (positions[near[0] as usize] - crease_mid).dot(own1) < 0.0
                && (positions[near[1] as usize] - crease_mid).dot(own0) < 0.0;
            if !convex {
                continue;
            }

            for member in 0..2 {
                let own = if member == 0 { own0 } else { own1 };
                let end_face = if member == 0 { own1 } else { own0 };
                if shared.dot(own) > -0.05 {
                    continue;
                }

                let field: std::rc::Rc<dyn super::ChainDistanceField> =
                    std::rc::Rc::new(PairCornerEndFaceField::of(
                        positions[corner as usize],
                        positions[near[member] as usize] - positions[corner as usize],
                        shared,
                        own,
                        end_face,
                        size,
                    ));
                let mut tags_here = Vec::new();
                for t in triangles_at(corner) {
                    if !same(normal(t), end_face) {
                        continue;
                    }
                    let tag = tags[t as usize];
                    if !tags_here.contains(&tag) {
                        tags_here.push(tag);
                    }
                }

                for tag in tags_here {
                    found
                        .get_or_insert_with(Default::default)
                        .entry(tag)
                        .or_default()
                        .push(field.clone());
                }
            }
        }

        found
    }
}
