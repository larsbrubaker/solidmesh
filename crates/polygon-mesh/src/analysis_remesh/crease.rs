//! The sharp creases of a remesh in progress: which mesh edges lie on a crease of the input and,
//! for each, the polyline of original crease points it stands for and the input crease segment
//! each polyline piece came from. A crease edge may be split (the new vertex goes on the
//! polyline) or merged with its neighbour along the chain when the chord stays within the
//! deviation bound, so a crease is never lost and a coarsened chain is always checked against
//! the original crease rather than its last approximation. Survival is read off these chains:
//! each input segment is counted by the chain edges that carry it, in time linear in the
//! polylines, with no search of unrelated creases.
//!
//! A crease is measured only between two faces whose normals are their own: the thin faces that
//! have none were taken out before the net was built (`clean`), so nothing here stands in for
//! one. The net also records the input angle of the sectors the creases cut each vertex's fan
//! into and the smallest input corner at each vertex, the measures `sliver` needs to tell a
//! corner the input forces from one the remesh narrowed.
//!
//! Edges are keyed by their sorted vertex pair: splits and collapses renumber `DMesh3`'s edge
//! ids, but a crease edge's two end vertices never change identity. `BTreeMap` keeps every
//! walk of the net in a fixed order.

use std::collections::BTreeMap;

use crate::g3::dmesh3::DMesh3;
use crate::g3::vector3d::{cross, distance, dot, length, sub};

use super::sheets::reliable_normal;

fn key(a: i32, b: i32) -> (i32, i32) {
    if a < b {
        (a, b)
    } else {
        (b, a)
    }
}

fn face_normal(mesh: &DMesh3, t: i32) -> Option<[f64; 3]> {
    let [a, b, c] = mesh.get_triangle(t).map(|v| mesh.vertex(v));
    reliable_normal(a, b, c)
}

/// The angle of the corner at `at` of a triangle, in degrees.
fn corner_angle_degrees(at: [f64; 3], p: [f64; 3], q: [f64; 3]) -> f64 {
    let (u, v) = (sub(p, at), sub(q, at));
    libm::atan2(length(cross(u, v)), dot(u, v)).to_degrees()
}

/// The fan of triangles around `v` in winding order: `(triangle, entering side, leaving side)`,
/// where the sides are the two neighbours of `v` in that triangle, walking `v`, entering side,
/// leaving side as the triangle does. Closed input gives a closed cycle.
fn ordered_fan(mesh: &DMesh3, v: i32) -> Vec<(i32, i32, i32)> {
    let triangles = mesh.vtx_triangles(v);
    let Some(&first) = triangles.first() else {
        return Vec::new();
    };
    let sides = |t: i32| {
        let verts = mesh.get_triangle(t);
        let at = (0..3).find(|&i| verts[i] == v).unwrap_or(0);
        (verts[(at + 1) % 3], verts[(at + 2) % 3])
    };
    let mut fan = Vec::with_capacity(triangles.len());
    let mut current = first;
    for _ in 0..triangles.len() {
        let (enter, leave) = sides(current);
        fan.push((current, enter, leave));
        let next = mesh.edge_other_t(mesh.find_edge(v, leave), current);
        if next < 0 || next == first {
            break;
        }
        current = next;
    }
    fan
}

/// One crease edge's polyline, running from the key's first vertex to its second, and the input
/// segment id of each polyline piece (one fewer than the points).
#[derive(Clone)]
struct Span {
    points: Vec<[f64; 3]>,
    segments: Vec<u32>,
}

impl Span {
    fn reversed(mut self) -> Self {
        self.points.reverse();
        self.segments.reverse();
        self
    }
}

/// The crease edges of a mesh and the original polyline each stands for.
pub(super) struct CreaseNet {
    spans: BTreeMap<(i32, i32), Span>,
    degree: Vec<u32>,
    input_segments: u32,
    /// The total input angle of each sector at an input vertex, keyed by the vertex and the input
    /// segments that bound the sector, in winding order (a two-crease vertex has two sectors over
    /// the same pair of segments, told apart by the direction).
    sectors: BTreeMap<(i32, u32, u32), f64>,
    /// For a vertex with no sector to measure (no crease, or a crease that ends there), the input
    /// angle of its whole fan; infinite elsewhere.
    closed_fan: Vec<f64>,
    /// Vertices the fan angle marks as feature points in their own right: never moved or merged.
    pinned: Vec<bool>,
    /// The smallest input corner at each vertex, degrees.
    smallest_corner: Vec<f64>,
}

impl CreaseNet {
    /// Every edge whose faces bend by more than `sharp_degrees`, and the input angle at every
    /// vertex, against `floor_degrees` (see `sliver` for the rule these feed).
    pub(super) fn from_mesh(mesh: &DMesh3, sharp_degrees: f64, floor_degrees: f64) -> Self {
        let cos_sharp = libm::cos(sharp_degrees.to_radians());
        let vertex_slots = mesh.max_vertex_id().max(0) as usize;
        let mut net = Self {
            spans: BTreeMap::new(),
            degree: vec![0; vertex_slots],
            input_segments: 0,
            sectors: BTreeMap::new(),
            closed_fan: vec![f64::INFINITY; vertex_slots],
            pinned: vec![false; vertex_slots],
            smallest_corner: vec![f64::INFINITY; vertex_slots],
        };
        for eid in mesh.edge_indices() {
            let [a, b, t0, t1] = mesh.get_edge(eid);
            if t1 < 0 {
                continue;
            }
            let (Some(n0), Some(n1)) = (face_normal(mesh, t0), face_normal(mesh, t1)) else {
                continue;
            };
            if dot(n0, n1) < cos_sharp {
                let k = key(a, b);
                net.spans.insert(
                    k,
                    Span {
                        points: vec![mesh.vertex(k.0), mesh.vertex(k.1)],
                        segments: vec![net.input_segments],
                    },
                );
                net.input_segments += 1;
                net.degree[a as usize] += 1;
                net.degree[b as usize] += 1;
            }
        }
        net.measure_input_sectors(mesh, floor_degrees);
        net
    }

    /// Records each sector's input angle at each vertex, and pins the vertices whose whole fan
    /// is too narrow to be triangulated at the floor. A closed fan needs at least three corners,
    /// so a fan totalling under three floors cannot be fixed by any triangulation.
    fn measure_input_sectors(&mut self, mesh: &DMesh3, floor_degrees: f64) {
        for v in mesh.vertex_indices() {
            let fan = ordered_fan(mesh, v);
            let corner = |t: i32| {
                let [a, b, c] = mesh.get_triangle(t);
                let (at, other_a, other_b) = if a == v {
                    (a, b, c)
                } else if b == v {
                    (b, c, a)
                } else {
                    (c, a, b)
                };
                corner_angle_degrees(mesh.vertex(at), mesh.vertex(other_a), mesh.vertex(other_b))
            };
            let total: f64 = fan.iter().map(|&(t, _, _)| corner(t)).sum();
            self.smallest_corner[v as usize] = fan
                .iter()
                .map(|&(t, _, _)| corner(t))
                .fold(f64::INFINITY, f64::min);
            if self.degree(v) < 2 {
                self.closed_fan[v as usize] = total;
                self.pinned[v as usize] = total < 3.0 * floor_degrees;
                continue;
            }
            let Some(start) = fan.iter().position(|&(_, _, leave)| self.has(v, leave)) else {
                continue;
            };
            let mut angle = 0.0;
            let mut entering = fan[start].2;
            for step in 1..=fan.len() {
                let (t, _, leave) = fan[(start + step) % fan.len()];
                angle += corner(t);
                if self.has(v, leave) {
                    if let (Some(from), Some(to)) = (
                        self.first_segment_from(v, entering),
                        self.first_segment_from(v, leave),
                    ) {
                        self.sectors.insert((v, from, to), angle);
                    }
                    angle = 0.0;
                    entering = leave;
                }
            }
        }
    }

    /// The input segment the edge `v`-`to` starts with, at `v`'s end.
    fn first_segment_from(&self, v: i32, to: i32) -> Option<u32> {
        self.oriented(v, to)
            .and_then(|s| s.segments.first().copied())
    }

    /// The input angle of the sector between the crease edges `at`-`x` and `at`-`y`, for a corner
    /// wound `at`, `x`, `y`; `None` if `at` has no such sector.
    pub(super) fn input_sector_angle(&self, at: i32, x: i32, y: i32) -> Option<f64> {
        let from = self.first_segment_from(at, x)?;
        let to = self.first_segment_from(at, y)?;
        self.sectors.get(&(at, from, to)).copied()
    }

    /// The smallest corner the input had at `v`, degrees; infinite for a vertex the input lacks.
    pub(super) fn smallest_input_corner(&self, v: i32) -> f64 {
        self.smallest_corner
            .get(v as usize)
            .copied()
            .unwrap_or(f64::INFINITY)
    }

    /// The input angle of `v`'s whole fan, if `v` is a vertex with no sector of its own.
    pub(super) fn closed_fan_angle(&self, v: i32) -> Option<f64> {
        self.closed_fan
            .get(v as usize)
            .copied()
            .filter(|angle| angle.is_finite())
    }

    /// How many input crease segments there are.
    pub(super) fn input_segments(&self) -> u32 {
        self.input_segments
    }

    /// Whether the edge `a`-`b` lies on a crease.
    pub(super) fn has(&self, a: i32, b: i32) -> bool {
        self.spans.contains_key(&key(a, b))
    }

    /// How many crease edges meet at `v`.
    pub(super) fn degree(&self, v: i32) -> u32 {
        self.degree.get(v as usize).copied().unwrap_or(0)
    }

    /// Whether `v` lies on a crease or is a pinned feature point; such a vertex is never smoothed
    /// or merged off where the input put it.
    pub(super) fn is_vertex(&self, v: i32) -> bool {
        self.degree(v) > 0 || self.pinned.get(v as usize).copied().unwrap_or(false)
    }

    /// Every crease edge with its polyline and the input segment of each polyline piece.
    pub(super) fn chains(&self) -> impl Iterator<Item = ((i32, i32), &[[f64; 3]], &[u32])> + '_ {
        self.spans
            .iter()
            .map(|(&k, s)| (k, s.points.as_slice(), s.segments.as_slice()))
    }

    /// The span of edge `a`-`b`, running from `a` to `b`.
    fn oriented(&self, a: i32, b: i32) -> Option<Span> {
        let k = key(a, b);
        let span = self.spans.get(&k)?;
        Some(if k.0 == a {
            span.clone()
        } else {
            span.clone().reversed()
        })
    }

    /// Where a split of `a`-`b` goes: half way along its polyline, so the new vertex is on the
    /// original crease even when this edge is a chord over several original segments.
    pub(super) fn split_position(&self, a: i32, b: i32) -> Option<[f64; 3]> {
        let span = self.oriented(a, b)?;
        Some(point_at_half_length(&span.points).0)
    }

    /// How far the edge's polyline strays from the straight chord, the crease error of this edge.
    pub(super) fn deviation(&self, a: i32, b: i32) -> f64 {
        self.oriented(a, b)
            .map_or(0.0, |span| polyline_deviation(&span.points))
    }

    /// Records the split of `a`-`b` at new vertex `v`.
    pub(super) fn record_split(&mut self, a: i32, b: i32, v: i32) {
        let Some(span) = self.oriented(a, b) else {
            return;
        };
        let (position, index) = point_at_half_length(&span.points);
        self.spans.remove(&key(a, b));

        // `position` sits inside segment `index - 1`: both halves carry its id.
        let cut = index.saturating_sub(1).min(span.segments.len() - 1);
        let mut left_points = span.points[..index].to_vec();
        left_points.push(position);
        let mut left_segments = span.segments[..cut].to_vec();
        left_segments.push(span.segments[cut]);
        self.insert_oriented(
            a,
            v,
            Span {
                points: left_points,
                segments: left_segments,
            },
        );

        let mut right_points = vec![position];
        right_points.extend_from_slice(&span.points[index..]);
        self.insert_oriented(
            v,
            b,
            Span {
                points: right_points,
                segments: span.segments[cut..].to_vec(),
            },
        );

        if self.degree.len() <= v as usize {
            self.degree.resize(v as usize + 1, 0);
        }
        self.degree[v as usize] = 2;
    }

    /// The chord deviation of the merged edge `keep`-`next` if `remove` left the chain.
    pub(super) fn merged_deviation(&self, keep: i32, remove: i32, next: i32) -> Option<f64> {
        let mut points = self.oriented(keep, remove)?.points;
        points.pop();
        points.extend(self.oriented(remove, next)?.points);
        Some(polyline_deviation(&points))
    }

    /// Replaces `keep`-`remove` and `remove`-`next` with the merged edge `keep`-`next`.
    pub(super) fn record_chain_collapse(&mut self, keep: i32, remove: i32, next: i32) {
        let (Some(mut first), Some(second)) =
            (self.oriented(keep, remove), self.oriented(remove, next))
        else {
            return;
        };
        first.points.pop();
        first.points.extend(second.points);
        first.segments.extend(second.segments);
        self.spans.remove(&key(keep, remove));
        self.spans.remove(&key(remove, next));
        self.insert_oriented(keep, next, first);
        if let Some(d) = self.degree.get_mut(remove as usize) {
            *d = 0;
        }
    }

    fn insert_oriented(&mut self, a: i32, b: i32, span: Span) {
        let k = key(a, b);
        self.spans
            .insert(k, if k.0 == a { span } else { span.reversed() });
    }

    /// The other end of the crease chain at chain-interior `v`, past the edge to `from`.
    pub(super) fn chain_neighbour(&self, mesh: &DMesh3, v: i32, from: i32) -> Option<i32> {
        if self.degree(v) != 2 {
            return None;
        }
        mesh.vtx_vertices(v)
            .into_iter()
            .find(|&n| n != from && self.has(v, n))
    }
}

/// The point half way along `points` and the index of the first polyline point after it.
fn point_at_half_length(points: &[[f64; 3]]) -> ([f64; 3], usize) {
    let total: f64 = points.windows(2).map(|s| distance(s[0], s[1])).sum();
    let half = total * 0.5;
    let mut walked = 0.0;
    for (i, s) in points.windows(2).enumerate() {
        let length_here = distance(s[0], s[1]);
        if walked + length_here >= half && length_here > 0.0 {
            let t = ((half - walked) / length_here).clamp(0.0, 1.0);
            let d = sub(s[1], s[0]);
            return (
                [s[0][0] + d[0] * t, s[0][1] + d[1] * t, s[0][2] + d[2] * t],
                i + 1,
            );
        }
        walked += length_here;
    }
    (points[points.len() / 2], points.len() / 2)
}

/// The largest distance from `points` to the straight chord between its ends.
pub(super) fn polyline_deviation(points: &[[f64; 3]]) -> f64 {
    let (first, last) = (points[0], points[points.len() - 1]);
    let chord = sub(last, first);
    let chord_len_sq = dot(chord, chord);
    let mut worst = 0.0_f64;
    for &p in &points[1..points.len() - 1] {
        let offset = sub(p, first);
        let t = if chord_len_sq > 0.0 {
            (dot(offset, chord) / chord_len_sq).clamp(0.0, 1.0)
        } else {
            0.0
        };
        let nearest = [
            first[0] + chord[0] * t,
            first[1] + chord[1] * t,
            first[2] + chord[2] * t,
        ];
        worst = worst.max(distance(p, nearest));
    }
    worst
}
