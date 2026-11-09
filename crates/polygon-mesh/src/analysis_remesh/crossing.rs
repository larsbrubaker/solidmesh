//! The no-crossing promise of a returned surface: how many pairs of triangles sharing no vertex
//! actually pass through each other.
//!
//! Pairs are found by self-joining a bounding-volume hierarchy over the triangles' boxes, split at
//! the median so its size is two nodes per triangle however uneven the triangle sizes are; a
//! uniform grid sized by the mean triangle would instead fill a number of cells that grows with
//! the cube of the largest triangle's size over the mean. The join carries only an explicit stack
//! of node pairs and polls the cancel flag as it goes.
//!
//! Each test decides on exact signs: `manifold_rust`'s filtered orientation predicates evaluate
//! the determinant in floating point and fall back to exact arithmetic when rounding could change
//! the sign, so a planar pair whose determinants are all rounding noise reads as what it is.
//! Contact is closed: two triangles with no vertex in common that touch at all are a crossing,
//! through an edge, through a vertex or by coplanar overlap, because a surface that touches
//! itself cannot bound a volume mesh. Two triangles sharing one vertex cross only if they meet
//! beyond it; two sharing an edge are neighbours, and `verify` holds their folding separately.

use manifold_rust::linalg::{Vec2, Vec3};
use manifold_rust::robust::exact::filtered::{orient2d, orient3d};
use manifold_rust::robust::exact::Sign;

use super::engine::{Engine, P};
use super::{AnalysisRemeshError, AnalysisSurface};

/// Triangles in a leaf of the hierarchy: below this, a leaf's pairs cost less than more nodes.
const LEAF_TRIANGLES: usize = 8;
/// Node pairs visited between cancel polls.
const CANCEL_STRIDE: usize = 4096;

fn point(p: P) -> Vec3 {
    Vec3 {
        x: p[0],
        y: p[1],
        z: p[2],
    }
}

fn orient(a: P, b: P, c: P, d: P) -> Sign {
    orient3d(point(a), point(b), point(c), point(d))
}

fn flat(p: P, drop_axis: usize) -> Vec2 {
    let (u, v) = [(1, 2), (2, 0), (0, 1)][drop_axis];
    Vec2 { x: p[u], y: p[v] }
}

fn on_segment_box(p: Vec2, q: Vec2, x: Vec2) -> bool {
    x.x >= p.x.min(q.x) && x.x <= p.x.max(q.x) && x.y >= p.y.min(q.y) && x.y <= p.y.max(q.y)
}

/// Whether closed 2D segments `p`-`q` and `a`-`b` share a point, collinear contact included.
fn segments_meet(p: Vec2, q: Vec2, a: Vec2, b: Vec2) -> bool {
    let (o1, o2) = (orient2d(p, q, a), orient2d(p, q, b));
    let (o3, o4) = (orient2d(a, b, p), orient2d(a, b, q));
    (o1 != o2 && o3 != o4)
        || (o1 == Sign::Zero && on_segment_box(p, q, a))
        || (o2 == Sign::Zero && on_segment_box(p, q, b))
        || (o3 == Sign::Zero && on_segment_box(a, b, p))
        || (o4 == Sign::Zero && on_segment_box(a, b, q))
}

/// Whether `x` lies in the closed 2D triangle `a b c`.
fn point_in_triangle(x: Vec2, a: Vec2, b: Vec2, c: Vec2) -> bool {
    let sides = [orient2d(a, b, x), orient2d(b, c, x), orient2d(c, a, x)];
    !(sides.contains(&Sign::Pos) && sides.contains(&Sign::Neg))
}

/// Whether closed segment `p`-`q`, lying in the plane of triangle `a b c`, touches the triangle.
/// A triangle of no area in every projection holds nothing.
fn segment_in_plane_hits(p: P, q: P, a: P, b: P, c: P) -> bool {
    let Some(axis) =
        (0..3).find(|&axis| orient2d(flat(a, axis), flat(b, axis), flat(c, axis)) != Sign::Zero)
    else {
        return false;
    };
    let (fp, fq) = (flat(p, axis), flat(q, axis));
    let (fa, fb, fc) = (flat(a, axis), flat(b, axis), flat(c, axis));
    point_in_triangle(fp, fa, fb, fc)
        || point_in_triangle(fq, fa, fb, fc)
        || segments_meet(fp, fq, fa, fb)
        || segments_meet(fp, fq, fb, fc)
        || segments_meet(fp, fq, fc, fa)
}

/// Whether closed segment `p`-`q` touches closed triangle `a b c`.
fn segment_hits(p: P, q: P, a: P, b: P, c: P) -> bool {
    let (dp, dq) = (orient(a, b, c, p), orient(a, b, c, q));
    if dp == Sign::Zero && dq == Sign::Zero {
        return segment_in_plane_hits(p, q, a, b, c);
    }
    if dp == dq {
        return false;
    }
    // The line crosses the plane once, within the segment; closed containment allows the sides'
    // zero signs, which mean it passes exactly through an edge or a vertex.
    let sides = [orient(p, q, a, b), orient(p, q, b, c), orient(p, q, c, a)];
    !(sides.contains(&Sign::Pos) && sides.contains(&Sign::Neg))
}

const EDGES: [(usize, usize); 3] = [(0, 1), (1, 2), (2, 0)];

/// Whether two triangles with no vertex in common share any point. Two triangles that meet at
/// all have a point on an edge of one of them, touching or coplanar overlap included.
fn triangles_touch(a: &[P; 3], b: &[P; 3]) -> bool {
    let any_edge = |t: &[P; 3], o: &[P; 3]| {
        EDGES
            .iter()
            .any(|&(i, j)| segment_hits(t[i], t[j], o[0], o[1], o[2]))
    };
    any_edge(a, b) || any_edge(b, a)
}

/// Whether two triangles sharing exactly one vertex meet anywhere else. Their contact beyond the
/// shared vertex must reach the far edge of one of them, so only those two edges are tested.
fn fan_pair_crosses(a: &[P; 3], shared_a: usize, b: &[P; 3], shared_b: usize) -> bool {
    let far = |t: &[P; 3], at: usize| (t[(at + 1) % 3], t[(at + 2) % 3]);
    let (p, q) = far(a, shared_a);
    let (r, s) = far(b, shared_b);
    segment_hits(p, q, b[0], b[1], b[2]) || segment_hits(r, s, a[0], a[1], a[2])
}

fn boxes_overlap(a: &(P, P), b: &(P, P)) -> bool {
    (0..3).all(|k| a.0[k] <= b.1[k] && b.0[k] <= a.1[k])
}

struct Node {
    bounds: (P, P),
    /// Children, for an inner node; `None` marks a leaf.
    children: Option<[u32; 2]>,
    /// The triangles of a leaf: a range of `order`.
    start: u32,
    count: u32,
}

/// A median-split hierarchy over triangle boxes. Sizes are fixed by the triangle count alone.
struct Hierarchy {
    nodes: Vec<Node>,
    order: Vec<u32>,
}

impl Hierarchy {
    fn build(boxes: &[(P, P)]) -> Self {
        let mut hierarchy = Self {
            nodes: Vec::with_capacity(boxes.len() * 2),
            order: (0..boxes.len() as u32).collect(),
        };
        hierarchy.split(boxes, 0, boxes.len());
        hierarchy
    }

    fn range_bounds(&self, boxes: &[(P, P)], start: usize, end: usize) -> (P, P) {
        let mut bounds = boxes[self.order[start] as usize];
        for &t in &self.order[start + 1..end] {
            for k in 0..3 {
                bounds.0[k] = bounds.0[k].min(boxes[t as usize].0[k]);
                bounds.1[k] = bounds.1[k].max(boxes[t as usize].1[k]);
            }
        }
        bounds
    }

    fn split(&mut self, boxes: &[(P, P)], start: usize, end: usize) -> u32 {
        let index = self.nodes.len() as u32;
        let bounds = self.range_bounds(boxes, start, end);
        self.nodes.push(Node {
            bounds,
            children: None,
            start: start as u32,
            count: (end - start) as u32,
        });
        if end - start <= LEAF_TRIANGLES {
            return index;
        }
        let extents = [
            bounds.1[0] - bounds.0[0],
            bounds.1[1] - bounds.0[1],
            bounds.1[2] - bounds.0[2],
        ];
        let axis = if extents[0] >= extents[1] && extents[0] >= extents[2] {
            0
        } else if extents[1] >= extents[2] {
            1
        } else {
            2
        };
        let middle = (start + end) / 2;
        // Box centres, tie-broken by triangle id, so the split is the same every run.
        self.order[start..end].select_nth_unstable_by(middle - start, |&x, &y| {
            let centre = |t: u32| boxes[t as usize].0[axis] + boxes[t as usize].1[axis];
            centre(x).total_cmp(&centre(y)).then(x.cmp(&y))
        });
        let left = self.split(boxes, start, middle);
        let right = self.split(boxes, middle, end);
        self.nodes[index as usize].children = Some([left, right]);
        index
    }

    fn leaf(&self, node: u32) -> &[u32] {
        let n = &self.nodes[node as usize];
        &self.order[n.start as usize..(n.start + n.count) as usize]
    }
}

/// How many triangle pairs sharing no vertex cross.
pub(super) fn crossing_pairs(
    surface: &AnalysisSurface,
    engine: &Engine<'_>,
) -> Result<usize, AnalysisRemeshError> {
    let corners: Vec<[P; 3]> = surface
        .faces
        .iter()
        .map(|f| f.map(|v| surface.vertices[v as usize]))
        .collect();
    if corners.is_empty() {
        return Ok(0);
    }
    let boxes: Vec<(P, P)> = corners
        .iter()
        .map(|t| {
            let (mut lo, mut hi) = (t[0], t[0]);
            for p in &t[1..] {
                for k in 0..3 {
                    lo[k] = lo[k].min(p[k]);
                    hi[k] = hi[k].max(p[k]);
                }
            }
            (lo, hi)
        })
        .collect();
    let hierarchy = Hierarchy::build(&boxes);

    let test_pair = |a: u32, b: u32| -> bool {
        let (ta, tb) = (a as usize, b as usize);
        if !boxes_overlap(&boxes[ta], &boxes[tb]) {
            return false;
        }
        let shared: Vec<(usize, usize)> = (0..3)
            .flat_map(|i| (0..3).map(move |j| (i, j)))
            .filter(|&(i, j)| surface.faces[ta][i] == surface.faces[tb][j])
            .collect();
        match shared.as_slice() {
            [] => triangles_touch(&corners[ta], &corners[tb]),
            [(i, j)] => fan_pair_crosses(&corners[ta], *i, &corners[tb], *j),
            // Triangles sharing an edge are neighbours: their one possible fault is folding,
            // which `verify` holds to its own promise.
            _ => false,
        }
    };

    let mut pairs = 0;
    let mut visited = 0;
    let mut stack = vec![(0_u32, 0_u32)];
    while let Some((a, b)) = stack.pop() {
        visited += 1;
        if visited % CANCEL_STRIDE == 0 {
            engine.check_cancel()?;
        }
        if !boxes_overlap(
            &hierarchy.nodes[a as usize].bounds,
            &hierarchy.nodes[b as usize].bounds,
        ) {
            continue;
        }
        let (a_children, b_children) = (
            hierarchy.nodes[a as usize].children,
            hierarchy.nodes[b as usize].children,
        );
        match (a_children, b_children) {
            (None, None) => {
                let (la, lb) = (hierarchy.leaf(a), hierarchy.leaf(b));
                if a == b {
                    for (i, &x) in la.iter().enumerate() {
                        for &y in &la[i + 1..] {
                            if test_pair(x, y) {
                                pairs += 1;
                            }
                        }
                    }
                } else {
                    for &x in la {
                        for &y in lb {
                            if test_pair(x, y) {
                                pairs += 1;
                            }
                        }
                    }
                }
            }
            (Some([al, ar]), Some(_)) if a == b => {
                stack.push((al, al));
                stack.push((ar, ar));
                stack.push((al, ar));
            }
            (Some([al, ar]), None) => {
                stack.push((al, b));
                stack.push((ar, b));
            }
            (None, Some([bl, br])) => {
                stack.push((a, bl));
                stack.push((a, br));
            }
            (Some([al, ar]), Some([bl, br])) => {
                // Descend the larger box so the pairs shrink evenly.
                let span = |n: u32| {
                    let (lo, hi) = hierarchy.nodes[n as usize].bounds;
                    (hi[0] - lo[0]) + (hi[1] - lo[1]) + (hi[2] - lo[2])
                };
                if span(a) >= span(b) {
                    stack.push((al, b));
                    stack.push((ar, b));
                } else {
                    stack.push((a, bl));
                    stack.push((a, br));
                }
            }
        }
    }
    Ok(pairs)
}
