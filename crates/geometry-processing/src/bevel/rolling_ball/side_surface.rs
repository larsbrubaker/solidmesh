//! One side of a picked edge as the rolling ball reads it: the side's triangles, a box tree over
//! them, and the one question the ball asks of a side - its closest point to a point in space, and
//! what that point lies on. The overview is the module header (`rolling_ball/mod.rs`).
//!
//! What the closest point lies on matters to the band built from it: a contact inside a face, or
//! on an edge between two of the side's own triangles, is the ball resting on that surface; a
//! contact on the side's BOUNDARY (an edge only one of the side's triangles uses, or a vertex on
//! one) is the ball resting on a rim, where the face beside the edge ran out before the ball could
//! touch it.

use std::collections::{HashMap, HashSet};

use vector_math::Vector3;

use crate::bevel::level_line::ChainField;

/// The most triangles a leaf of the box tree holds.
const LEAF_SIZE: usize = 4;

/// How close a barycentric coordinate must come to zero, relative to one, for a contact to count as
/// on that corner's opposite edge (or, with two of them, at a vertex).
const ON_A_BOUNDARY_RELATIVE: f64 = 1e-9;

/// What a contact point lies on.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum ContactFeature {
    /// Inside the side's triangle `triangle` (its position in the side's list).
    Face { triangle: i32 },
    /// On the edge between vertices `from` and `to` (`from < to`); `rim` when only one of the
    /// side's triangles uses it, so the side ends there.
    Edge { from: i32, to: i32, rim: bool },
    /// At vertex `vertex`; `rim` when it stands on the side's boundary.
    Vertex { vertex: i32, rim: bool },
    /// On the plane a half of space was cut at ([`RollingBallSide::closest_in`]), inside the
    /// side's triangle `triangle`: no surface the ball rests on, only where the half ends.
    Cut { triangle: i32 },
}

impl ContactFeature {
    /// Whether the contact stands on the side's boundary - the ball resting on a rim.
    pub fn is_rim(&self) -> bool {
        match *self {
            ContactFeature::Face { .. } | ContactFeature::Cut { .. } => false,
            ContactFeature::Edge { rim, .. } | ContactFeature::Vertex { rim, .. } => rim,
        }
    }

    /// Whether the contact is only where a half of space was cut, not the side's own surface.
    pub fn is_cut(&self) -> bool {
        matches!(self, ContactFeature::Cut { .. })
    }
}

/// A half of space: the points `p` with `(p - origin) . normal >= 0`.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct HalfSpace {
    /// A point on its plane.
    pub origin: Vector3,
    /// The unit normal pointing into it.
    pub normal: Vector3,
}

impl HalfSpace {
    /// How far `p` stands inside (negative outside).
    pub fn depth(&self, p: Vector3) -> f64 {
        (p - self.origin).dot(self.normal)
    }

    /// Whether the box from `min` to `max` lies wholly outside.
    fn leaves_box(&self, min: Vector3, max: Vector3) -> bool {
        let mut deepest = f64::NEG_INFINITY;
        for x in [min.x, max.x] {
            for y in [min.y, max.y] {
                for z in [min.z, max.z] {
                    deepest =
                        vector_math::ieee_math::max_f64(deepest, self.depth(Vector3::new(x, y, z)));
                }
            }
        }

        deepest < 0.0
    }
}

/// The closest point to `point` of the triangle `corners` within every one of `halves` (the whole
/// triangle for none), and whether it lies on a half's plane where that plane cuts the triangle;
/// `None` where nothing of the triangle is inside them all. The triangle is cut by each half in
/// turn, so one half reads exactly as a lone cut does.
pub(super) fn closest_on_cut_triangle(
    point: Vector3,
    corners: [Vector3; 3],
    halves: &[HalfSpace],
) -> Option<(Vector3, bool)> {
    let [a, b, c] = corners;
    let mut inside: Vec<Vector3> = corners.to_vec();
    let mut cutting: Vec<HalfSpace> = Vec::new();
    for half in halves {
        let depths: Vec<f64> = inside.iter().map(|&p| half.depth(p)).collect();
        if depths.iter().all(|&d| d >= 0.0) {
            continue;
        }
        if depths.iter().all(|&d| d < 0.0) {
            return None;
        }

        // The part inside, walked corner by corner. A crossing is added only where the edge's
        // ends lie strictly on opposite sides: a corner exactly on the plane is kept as itself,
        // and a crossing there too would repeat it and give the fan a triangle with no area.
        let mut kept: Vec<Vector3> = Vec::with_capacity(inside.len() + 1);
        for i in 0..inside.len() {
            let j = (i + 1) % inside.len();
            let (p, q) = (inside[i], inside[j]);
            let (dp, dq) = (depths[i], depths[j]);
            if dp >= 0.0 {
                kept.push(p);
            }
            if (dp > 0.0 && dq < 0.0) || (dp < 0.0 && dq > 0.0) {
                kept.push(p + ((q - p) * (dp / (dp - dq))));
            }
        }
        // Rounding can still land a crossing on its neighbour; a repeated point adds nothing.
        kept.dedup();
        while kept.len() > 1 && kept.first() == kept.last() {
            kept.pop();
        }
        if kept.len() < 3 {
            return None;
        }
        inside = kept;
        cutting.push(*half);
    }
    if cutting.is_empty() {
        return Some((closest_on_any_triangle(point, a, b, c), false));
    }

    let mut best: Option<(f64, Vector3)> = None;
    for i in 1..inside.len() - 1 {
        let on = closest_on_any_triangle(point, inside[0], inside[i], inside[i + 1]);
        let d = (on - point).length_squared();
        if best.is_none_or(|(bd, _)| d < bd) {
            best = Some((d, on));
        }
    }

    let (_, on) = best?;
    // On the cut where the point stands on a cutting plane but not on a corner the triangle had
    // there.
    let scale = 1.0 + (b - a).length() + (c - a).length();
    let on_plane = cutting
        .iter()
        .any(|half| half.depth(on).abs() <= ON_A_BOUNDARY_RELATIVE * scale);
    Some((on, on_plane))
}

/// The closest point of the triangle `a`, `b`, `c` to `point`, a triangle with no area included:
/// the barycentric region test divides by the triangle's area terms, so a triangle whose corners
/// coincide or line up is read as its three edges instead, never as a point that is not a number.
fn closest_on_any_triangle(point: Vector3, a: Vector3, b: Vector3, c: Vector3) -> Vector3 {
    if (b - a).cross(c - a).length_squared() > 0.0 {
        return ChainField::closest_on_triangle(point, a, b, c);
    }

    let mut best = ChainField::closest_on_segment(point, a, b);
    for (p, q) in [(b, c), (c, a)] {
        let on = ChainField::closest_on_segment(point, p, q);
        if (on - point).length_squared() < (best - point).length_squared() {
            best = on;
        }
    }
    best
}

/// A side's closest point to a point in space.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct SideContact {
    /// The closest point of the side.
    pub point: Vector3,
    /// Its distance from the point asked about.
    pub distance: f64,
    /// What it lies on.
    pub feature: ContactFeature,
    /// The side's triangle it was found on, by its position in the side's list (of two equally
    /// near, the lower), as [`ContactFeature::Face`] names it.
    pub triangle: i32,
}

/// One node of the box tree: a box, and either two children or a run of `order`.
#[derive(Debug, Clone, Copy)]
struct Node {
    min: Vector3,
    max: Vector3,
    /// Child node indices, or -1 for a leaf.
    left: i32,
    right: i32,
    /// A leaf's run of `order`.
    start: usize,
    count: usize,
}

/// One side of a picked edge: the triangles of the part on that side, with a box tree.
#[derive(Debug, Clone)]
pub struct RollingBallSide {
    pub(super) positions: Vec<Vector3>,
    pub(super) triangles: Vec<[i32; 3]>,
    /// Triangle indices in tree order.
    order: Vec<usize>,
    nodes: Vec<Node>,
    /// How many of the side's triangles use each edge (keyed low, high).
    edge_uses: HashMap<(i32, i32), u32>,
    /// The vertices on the side's boundary: an end of any edge only one of its triangles uses.
    rim_vertices: HashSet<i32>,
    /// The side's triangles (indices into `triangles`) at each vertex, in index order.
    pub(super) vertex_triangles: HashMap<i32, Vec<usize>>,
}

/// An edge's key: its two vertices, low first.
fn key(a: i32, b: i32) -> (i32, i32) {
    if a < b {
        (a, b)
    } else {
        (b, a)
    }
}

/// The squared distance from a point to a box (zero inside it).
fn box_distance_squared(point: Vector3, min: Vector3, max: Vector3) -> f64 {
    let gap = |p: f64, low: f64, high: f64| {
        if p < low {
            low - p
        } else if p > high {
            p - high
        } else {
            0.0
        }
    };
    let dx = gap(point.x, min.x, max.x);
    let dy = gap(point.y, min.y, max.y);
    let dz = gap(point.z, min.z, max.z);
    (dx * dx) + (dy * dy) + (dz * dz)
}

impl RollingBallSide {
    /// A side from the part's vertex positions and the side's triangles (indices into
    /// `positions`). Triangles keep their indices, so a contact names the part's own triangle,
    /// edge or vertex.
    pub fn new(positions: &[Vector3], triangles: &[[i32; 3]]) -> RollingBallSide {
        let mut edge_uses = HashMap::new();
        for t in triangles {
            for (a, b) in [(t[0], t[1]), (t[1], t[2]), (t[2], t[0])] {
                *edge_uses.entry(key(a, b)).or_insert(0) += 1;
            }
        }

        let rim_vertices = edge_uses
            .iter()
            .filter(|(_, &uses)| uses < 2)
            .flat_map(|(&(a, b), _)| [a, b])
            .collect();
        let mut vertex_triangles: HashMap<i32, Vec<usize>> = HashMap::new();
        for (i, t) in triangles.iter().enumerate() {
            for v in *t {
                let around = vertex_triangles.entry(v).or_default();
                if around.last() != Some(&i) {
                    around.push(i);
                }
            }
        }

        let mut side = RollingBallSide {
            positions: positions.to_vec(),
            triangles: triangles.to_vec(),
            order: (0..triangles.len()).collect(),
            nodes: Vec::new(),
            edge_uses,
            rim_vertices,
            vertex_triangles,
        };
        if !side.triangles.is_empty() {
            side.build(0, side.triangles.len());
        }

        side
    }

    /// How many triangles the side holds.
    pub fn triangle_count(&self) -> usize {
        self.triangles.len()
    }

    /// A triangle's three corners.
    pub(super) fn corners(&self, triangle: usize) -> [Vector3; 3] {
        let t = self.triangles[triangle];
        [
            self.positions[t[0] as usize],
            self.positions[t[1] as usize],
            self.positions[t[2] as usize],
        ]
    }

    /// Builds the node over `order[start..end]` and returns its index: the run is sorted along its
    /// box's longest axis by centroid (ties by triangle index, so the tree is the same on every
    /// run) and split at the median.
    fn build(&mut self, start: usize, end: usize) -> i32 {
        let mut min = Vector3::POSITIVE_INFINITY;
        let mut max = Vector3::NEGATIVE_INFINITY;
        for &t in &self.order[start..end] {
            for corner in self.corners(t) {
                min = Vector3::component_min(min, corner);
                max = Vector3::component_max(max, corner);
            }
        }

        let index = self.nodes.len();
        self.nodes.push(Node {
            min,
            max,
            left: -1,
            right: -1,
            start,
            count: end - start,
        });
        if end - start <= LEAF_SIZE {
            return index as i32;
        }

        let extent = max - min;
        let axis = if extent.x >= extent.y && extent.x >= extent.z {
            0
        } else if extent.y >= extent.z {
            1
        } else {
            2
        };
        let centroid_on_axis = |side: &RollingBallSide, t: usize| {
            let [a, b, c] = side.corners(t);
            let centroid = (a + b + c) / 3.0;
            match axis {
                0 => centroid.x,
                1 => centroid.y,
                _ => centroid.z,
            }
        };
        let mut run: Vec<(f64, usize)> = self.order[start..end]
            .iter()
            .map(|&t| (centroid_on_axis(self, t), t))
            .collect();
        run.sort_by(|a, b| a.0.total_cmp(&b.0).then(a.1.cmp(&b.1)));
        for (i, (_, t)) in run.into_iter().enumerate() {
            self.order[start + i] = t;
        }

        let middle = start + ((end - start) / 2);
        let left = self.build(start, middle);
        let right = self.build(middle, end);
        self.nodes[index].left = left;
        self.nodes[index].right = right;
        index as i32
    }

    /// The side's closest point to `point`, with what it lies on; `None` for a side with no
    /// triangles. Exact for the triangles as given; of two triangles equally near, the lower index.
    pub fn closest(&self, point: Vector3) -> Option<SideContact> {
        self.closest_in(point, None)
    }

    /// [`closest`](Self::closest) among the side's surface within `half` (all of it for `None`):
    /// each triangle cut by the half's plane and read on the part inside. A closest point on the
    /// cut itself is [`ContactFeature::Cut`]; `None` where nothing of the side is inside.
    pub fn closest_in(&self, point: Vector3, half: Option<HalfSpace>) -> Option<SideContact> {
        self.closest_within(point, half.as_slice())
    }

    /// [`closest_in`](Self::closest_in) within every one of `halves` at once (all of the side for
    /// none): a contact on any half's cut is [`ContactFeature::Cut`].
    pub fn closest_within(&self, point: Vector3, halves: &[HalfSpace]) -> Option<SideContact> {
        if self.nodes.is_empty() {
            return None;
        }

        // (squared distance, triangle, point, on the cut) of the best so far.
        let mut best: Option<(f64, usize, Vector3, bool)> = None;
        let mut stack = vec![0usize];
        while let Some(n) = stack.pop() {
            let node = self.nodes[n];
            let reach = box_distance_squared(point, node.min, node.max);
            if best.is_some_and(|(d, _, _, _)| reach > d) {
                continue;
            }
            if halves.iter().any(|h| h.leaves_box(node.min, node.max)) {
                continue;
            }

            if node.left < 0 {
                for &t in &self.order[node.start..node.start + node.count] {
                    let corners = self.corners(t);
                    let Some((on, cut)) = closest_on_cut_triangle(point, corners, halves) else {
                        continue;
                    };
                    let d = (on - point).length_squared();
                    let better = match best {
                        None => true,
                        Some((bd, bt, _, _)) => d < bd || (d == bd && t < bt),
                    };
                    if better {
                        best = Some((d, t, on, cut));
                    }
                }
                continue;
            }

            // The nearer child last, so it is searched first.
            let left = self.nodes[node.left as usize];
            let right = self.nodes[node.right as usize];
            let to_left = box_distance_squared(point, left.min, left.max);
            let to_right = box_distance_squared(point, right.min, right.max);
            if to_left <= to_right {
                stack.push(node.right as usize);
                stack.push(node.left as usize);
            } else {
                stack.push(node.left as usize);
                stack.push(node.right as usize);
            }
        }

        let (d, t, on, cut) = best?;
        // A side's list position; a side has far fewer than `i32::MAX` triangles.
        let triangle = t as i32;
        Some(SideContact {
            point: on,
            distance: d.sqrt(),
            feature: if cut {
                ContactFeature::Cut { triangle }
            } else {
                self.feature_of(t, on)
            },
            triangle,
        })
    }

    /// How far `point` stands off the side where `contact` touches it, on the side asked: in
    /// front (along the outward normal, free space for a part's outward-wound triangles) or, with
    /// `behind`, behind (inside the solid). It is the largest such distance over the side's
    /// triangles holding the contact - the one face for a face contact, the faces on the edge or
    /// around the vertex otherwise - so it is negative only where the point stands on the other
    /// side of every one of them: a ball reaching round a rim from the wrong side. `None` where
    /// every triangle holding the contact has no area: none of them faces any way, so nothing says
    /// which side of the surface the point stands on.
    pub fn standing_off(&self, contact: &SideContact, point: Vector3, behind: bool) -> Option<f64> {
        let holding: Vec<usize> = match contact.feature {
            ContactFeature::Face { .. } | ContactFeature::Cut { .. } => {
                vec![contact.triangle as usize]
            }
            ContactFeature::Edge { from, to, .. } => self
                .vertex_triangles
                .get(&from)
                .map(|around| {
                    around
                        .iter()
                        .copied()
                        .filter(|&t| self.triangles[t].contains(&to))
                        .collect()
                })
                .unwrap_or_default(),
            ContactFeature::Vertex { vertex, .. } => self
                .vertex_triangles
                .get(&vertex)
                .cloned()
                .unwrap_or_default(),
        };

        let toward = point - contact.point;
        let mut best: Option<f64> = None;
        for t in holding {
            let [a, b, c] = self.corners(t);
            let normal = (b - a).cross(c - a);
            let length = normal.length();
            if length > 0.0 {
                let along = toward.dot(normal / length);
                let off = if behind { -along } else { along };
                best = Some(best.map_or(off, |b| vector_math::ieee_math::max_f64(b, off)));
            }
        }

        best
    }

    /// What a point on triangle `triangle` lies on: a corner whose two neighbours' barycentric
    /// weights vanish is a vertex, one weight vanishing is the opposite edge, none is the face.
    pub(super) fn feature_of(&self, triangle: usize, on: Vector3) -> ContactFeature {
        let t = self.triangles[triangle];
        let [a, b, c] = self.corners(triangle);
        let (v0, v1, v2) = (b - a, c - a, on - a);
        let d00 = v0.dot(v0);
        let d01 = v0.dot(v1);
        let d11 = v1.dot(v1);
        let d20 = v2.dot(v0);
        let d21 = v2.dot(v1);
        let denominator = (d00 * d11) - (d01 * d01);
        if denominator <= 0.0 {
            // A degenerate triangle has no inside: its corners lie on one line, along its longest
            // edge, which spans the other two, so the point lies on that edge (the first listed of
            // equally long ones).
            let mut longest = (t[0], t[1], (b - a).length_squared());
            for (from, to, length) in [
                (t[1], t[2], (c - b).length_squared()),
                (t[2], t[0], (a - c).length_squared()),
            ] {
                if length > longest.2 {
                    longest = (from, to, length);
                }
            }
            return self.edge(longest.0, longest.1);
        }

        let wb = ((d11 * d20) - (d01 * d21)) / denominator;
        let wc = ((d00 * d21) - (d01 * d20)) / denominator;
        let wa = 1.0 - wb - wc;
        let zero = |w: f64| w <= ON_A_BOUNDARY_RELATIVE;
        match (zero(wa), zero(wb), zero(wc)) {
            (false, false, false) => ContactFeature::Face {
                triangle: triangle as i32,
            },
            (true, false, false) => self.edge(t[1], t[2]),
            (false, true, false) => self.edge(t[2], t[0]),
            (false, false, true) => self.edge(t[0], t[1]),
            (false, true, true) => self.vertex(t[0]),
            (true, false, true) => self.vertex(t[1]),
            _ => self.vertex(t[2]),
        }
    }

    /// The edge `a`-`b` as a contact feature, a rim where one of the side's triangles uses it.
    fn edge(&self, a: i32, b: i32) -> ContactFeature {
        let (from, to) = key(a, b);
        ContactFeature::Edge {
            from,
            to,
            rim: self.edge_uses.get(&(from, to)).copied().unwrap_or(0) < 2,
        }
    }

    /// Vertex `v` as a contact feature, a rim where any of the side's edges at it is one.
    fn vertex(&self, v: i32) -> ContactFeature {
        ContactFeature::Vertex {
            vertex: v,
            rim: self.rim_vertices.contains(&v),
        }
    }
}
