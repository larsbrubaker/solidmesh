//! A side as the ball reads it at a chain that ends on an end face ([`extended_past`]): cut at
//! each end face's plane and carried on past it, each of its triangles that meets the plane
//! extended as its own plane, as far as the chain's own line carried on past its end vertex. The
//! walk (`walk.rs`) reads a chain's sides through this wherever the chain has an end face; the
//! overview is the module header (`rolling_ball/mod.rs`).
//!
//! **Why extend.** At a chain end on an end face the round a part gets is the one its sides
//! would make if they ran on past the end face, cut flat by the face: extend and trim. Read only
//! up to the end face's plane, a side that the face leans across ends short of where the ball
//! would touch its continuation, so near a leaning end face the end stations' contacts land on the
//! side's edge with the face and fall short of the rails (the 10 degree sheared wedge's walls,
//! whose round is 2.7235 and 2.3416 from the chain everywhere, read 2.70 and 2.32 at its ends).
//! Read as extended, the end station touches each side where its continuation lies, and on prism
//! walls that is the wall's own plane, so every station reads the same distances.
//!
//! **The construction.** Each end face's plane bounds the side: triangles wholly on the chain's
//! side are kept as they are, triangles wholly beyond are dropped, and triangles the plane crosses
//! are cut there, one new vertex per crossed edge, so neighbouring triangles share it. That clip
//! is why nothing beyond the end face that is not the side's own is read: a side region can wrap
//! on past the plane (the T Handle's pocket wall runs through the web), and only what lies on the
//! chain's side of the plane is the side up to the face. Then every edge of the cut side that lies
//! in the plane and that only one of its triangles uses - the side's trace in the end face - gets
//! a strip in that triangle's own plane, running out square to the edge away from the triangle,
//! far enough that a point the ball's radius beyond the plane is inside it; a facet extends as
//! itself. Where two such strips leave one vertex in different directions a triangle fills the
//! wedge between them, and where they leave it in the same direction (collinear trace edges, or a
//! prism's walls past a square end) they share their far vertex, so the extension is one surface
//! with the side, sharing its vertices: the side's trace in the plane is no rim any more, and
//! contact tracking walks on over it.
//!
//! **Bounded at the chain.** Extended and trimmed, a side ends where the other side's extension
//! meets it: on the chain's own line carried on past its end vertex ([`ChainEnd`]). So each end's
//! extension is cut by the plane through that line holding the normal of the side's triangle
//! whose edge in the end plane leaves the end vertex (in that triangle's plane, the line itself),
//! keeping the side's half (the half its trace leaves the end vertex into). Where the face leans
//! so the strip at the end vertex runs out across the chain's line, the cut stops it there; a
//! side read beyond it would answer, on curved sides close together, with surface the trim
//! removes and a contact too near. Where the face leans the other way, the strip at the end vertex
//! runs out away from the chain's line, and one triangle in the strip's own plane fills the wedge
//! between the line and the strip, out as far past the plane as the strip, so a contact beside
//! the chain's line past the end vertex reads the side's plane, not a strip's open edge. A square
//! end has no wedge and nothing to cut: its strip at the end vertex runs along the line.
//!
//! **What it keeps.** The strips lie wholly beyond the plane, so a centre on the chain's side
//! whose ball does not reach the plane is nearer the side's own surface than any strip, and its
//! contact lies where the side alone puts it. At a square end the ball held in the plane touches
//! each side on its trace in the plane, which is the strip's edge as well: the same position, but
//! named `Edge { rim: false }` (the side now runs on past it) where it was a rim, and the cut side
//! is a new triangle list, so a solve can differ from the side's own in the last bits.
//!
//! **Its holes.** The extension's open edges are of two kinds. The side's own boundary carried on:
//! its rims on the chain's side of the plane (the cut side's open edges in no end plane, touching
//! no chain end vertex) and, where such a rim reaches the plane at the trace's far end (one strip
//! leaves it), the strip's edge leaving there, which carries that rim on. And holes, each reading
//! as a rim where a contact lands on it, though no surface of the side ends there: what lies past
//! a strip's reach (two radii past the plane), a vertex three or more strips leave (no fill; the
//! pairing round it is not decided), a strip left out because its triangle lies along the plane,
//! and an end vertex where the side's trace in the plane leaves by no edge or by two (no bound and
//! no wedge there); with them the chain's own edge at the end vertex and its line carried on past
//! it, where the band starts and no rail can end. Every vertex a hole's edge touches is a hole.
//!
//! **Which end contacts are setbacks** ([`ExtendedSide::end_contact_fault`]). A contact on a face
//! is. A contact on a hole is not. A contact on the side's own rim is only where the ball touches
//! one of the side's faces there square to it (a wall exactly as tall as the fillet, its end
//! station on the wall's bottom corner): a ball that only rests on the rim (a size past the side's
//! width: a cube edge of 2 asked for 2.2 would build a 2.0 round) gives no round of the size
//! asked. The pass refuses an end station whose contact is no setback
//! (`level_line::rolling_ball_legs`). The own rim is defined by what it is not (a hole), so a side
//! that curves back or is cut short near the end (a chamfer between the side and the end face) is
//! kept from building wrong by the square-touch rule and, before it, by the walk's reach check:
//! the ball whose round would cut into the chamfer is refused as `ReachesPastThePart`.

use std::collections::{HashMap, HashSet};

use vector_math::Vector3;

use super::side_surface::{ContactFeature, HalfSpace, RollingBallSide, SideContact};
use super::walk::EndContactFault;

/// How nearly a strip must run along the plane, as the cosine between its direction out and the
/// plane's normal, before it is left out: a triangle lying in the plane extends nowhere past it.
const LEAST_STRIP_COSINE: f64 = 1e-3;

/// How many radii past the plane a strip reaches, at least: the ball's contact beyond the plane is
/// at most a radius from its centre, which stands on the chain's side; twice leaves room.
const STRIP_RADII: f64 = 2.0;

/// How nearly the line from a rim contact to the ball's centre must stand square to a face of
/// the side there, as the sine of the angle off its normal, for the ball to touch that face: the
/// solve places a tangent contact to about a billionth of the part's size, and a ball resting on
/// the rim off the face's normal by an angle that changes the round's size stands far past this.
const TANGENT_SINE: f64 = 1e-6;

/// One end of a chain on an end face, as the extension reads it.
#[derive(Debug, Clone, Copy, PartialEq)]
pub(super) struct ChainEnd {
    /// The chain's side of the end face's plane.
    pub(super) plane: HalfSpace,
    /// The chain's end vertex, on the plane.
    pub(super) vertex: Vector3,
    /// The chain's direction at the end vertex, carried on past it (out of `plane`'s half); its
    /// length does not matter.
    pub(super) along: Vector3,
}

/// Where a cut makes its new vertices: keyed on a crossed edge's two ends, low first, and on the
/// plane, so every polygon sharing the edge shares the vertex exactly.
type Crossings = HashMap<(i32, i32, usize), i32>;

/// `polygon` cut to the half of space `keep` (vertices within `tolerance` of the plane count as
/// on it): unchanged when wholly kept, empty when wholly beyond, else cut at the plane, each new
/// vertex placed from its edge's low end and keyed on `(low, high, plane)` in `crossings`.
fn cut(
    polygon: Vec<i32>,
    keep: &HalfSpace,
    plane: usize,
    tolerance: f64,
    positions: &mut Vec<Vector3>,
    crossings: &mut Crossings,
) -> Vec<i32> {
    let depths: Vec<f64> = polygon
        .iter()
        .map(|&v| {
            let d = keep.depth(positions[v as usize]);
            if d.abs() <= tolerance {
                0.0
            } else {
                d
            }
        })
        .collect();
    if depths.iter().all(|&d| d >= 0.0) {
        return polygon;
    }
    if depths.iter().all(|&d| d <= 0.0) {
        return Vec::new();
    }
    let mut kept: Vec<i32> = Vec::with_capacity(polygon.len() + 1);
    for i in 0..polygon.len() {
        let j = (i + 1) % polygon.len();
        let (p, q) = (polygon[i], polygon[j]);
        let (dp, dq) = (depths[i], depths[j]);
        if dp >= 0.0 {
            kept.push(p);
        }
        if (dp > 0.0 && dq < 0.0) || (dp < 0.0 && dq > 0.0) {
            let (lo, hi, dlo, dhi) = if p < q {
                (p, q, dp, dq)
            } else {
                (q, p, dq, dp)
            };
            let at = *crossings.entry((lo, hi, plane)).or_insert_with(|| {
                let (a, b) = (positions[lo as usize], positions[hi as usize]);
                positions.push(a + ((b - a) * (dlo / (dlo - dhi))));
                // A part's vertex count fits an i32 (the mesh's own indices are i32).
                (positions.len() - 1) as i32
            });
            kept.push(at);
        }
    }
    kept
}

/// The side's edge in an end plane that leaves the chain's end vertex: its triangle's unit normal
/// and the edge's unit direction away from the end vertex.
#[derive(Debug, Clone, Copy)]
struct TraceAtEnd {
    normal: Vector3,
    away: Vector3,
}

/// A side cut at its chain's end faces and extended past them ([`extended_past`]), with the holes
/// its extension leaves.
pub(super) struct ExtendedSide {
    /// The side as the ball reads it.
    pub(super) side: RollingBallSide,
    holes: Holes,
}

impl ExtendedSide {
    /// Why a contact on `feature` is no setback at an end station whose ball's centre is
    /// `centre`; `None` where it is one: a contact on a face, or on the side's own rim (the far
    /// corner of its trace in the end face, its rim on the chain's side, or that rim carried on
    /// as a strip's edge) where the ball touches one of the side's faces there square to it, as a
    /// wall exactly as tall as the fillet is touched at its bottom rim.
    pub(super) fn end_contact_fault(
        &self,
        contact: &SideContact,
        centre: Vector3,
    ) -> Option<EndContactFault> {
        if !contact.feature.is_rim() {
            return None;
        }
        if self.on_a_hole(&contact.feature) {
            return Some(EndContactFault::OnAHole);
        }
        (!self.touched_square(contact, centre)).then_some(EndContactFault::OffTheSidesFaces)
    }

    /// Whether a contact on `feature` rests on a hole of the extension (the file header's list):
    /// a rim that is not the side's own boundary carried on past the end face, where the ball
    /// rests on no surface of the side, so its distance is no setback.
    pub(super) fn on_a_hole(&self, feature: &ContactFeature) -> bool {
        match *feature {
            ContactFeature::Edge { from, to, rim } => {
                rim && self.holes.edges.contains(&(from.min(to), from.max(to)))
            }
            ContactFeature::Vertex { vertex, rim } => rim && self.holes.vertices.contains(&vertex),
            ContactFeature::Face { .. } | ContactFeature::Cut { .. } => false,
        }
    }

    /// Whether the line from `contact` to `centre` stands square to one of the side's triangles
    /// using the contact's edge or vertex, within [`TANGENT_SINE`]: the ball touches that face at
    /// its rim, rather than only resting on the rim.
    fn touched_square(&self, contact: &SideContact, centre: Vector3) -> bool {
        let (v, also) = match contact.feature {
            ContactFeature::Edge { from, to, .. } => (from, Some(to)),
            ContactFeature::Vertex { vertex, .. } => (vertex, None),
            ContactFeature::Face { .. } | ContactFeature::Cut { .. } => return true,
        };
        let out = centre - contact.point;
        let length = out.length();
        if length.is_nan() || length <= 0.0 {
            return false;
        }
        let side = &self.side;
        side.vertex_triangles.get(&v).is_some_and(|around| {
            around.iter().any(|&t| {
                let [a, b, c] = side.triangles[t];
                if also.is_some_and(|w| a != w && b != w && c != w) {
                    return false;
                }
                let (pa, pb, pc) = (
                    side.positions[a as usize],
                    side.positions[b as usize],
                    side.positions[c as usize],
                );
                let normal = (pb - pa).cross(pc - pa);
                let area = normal.length();
                area > 0.0 && out.cross(normal).length() <= TANGENT_SINE * length * area
            })
        })
    }
}

/// The extension's open edges (low, high) that are holes, and every vertex they touch.
#[derive(Default)]
struct Holes {
    edges: HashSet<(i32, i32)>,
    vertices: HashSet<i32>,
}

/// One strip: its edge `a -> b` in the side's order, its end plane, and its two far vertices.
struct Strip {
    a: i32,
    b: i32,
    end: usize,
    far_a: i32,
    far_b: i32,
}

/// `side` cut at each of `ends`' planes and extended past each, bounded at the chain's line, as
/// the file header says, for a ball of `radius`; `tolerance` is how near a vertex must stand to a
/// plane to count as on it. Vertices keep their indices; new ones follow them.
#[allow(clippy::neg_cmp_op_on_partial_ord)] // `!(x > 0)` on purpose: a NaN extends nothing
pub(super) fn extended_past(
    side: &RollingBallSide,
    ends: &[ChainEnd],
    radius: f64,
    tolerance: f64,
) -> ExtendedSide {
    let mut positions = side.positions.clone();
    let mut crossings: Crossings = HashMap::new();

    // Cut each triangle at each end plane in turn (plane keys 0..ends.len()).
    let mut kept: Vec<[i32; 3]> = Vec::with_capacity(side.triangles.len());
    for &triangle in &side.triangles {
        let mut polygon: Vec<i32> = triangle.to_vec();
        for (h, end) in ends.iter().enumerate() {
            polygon = cut(
                polygon,
                &end.plane,
                h,
                tolerance,
                &mut positions,
                &mut crossings,
            );
            if polygon.is_empty() {
                break;
            }
        }
        for i in 1..polygon.len().saturating_sub(1) {
            kept.push([polygon[0], polygon[i], polygon[i + 1]]);
        }
    }
    // The cut side's vertices are these; the strips', the fills' and the bound's come after.
    let cut_count = positions.len();

    // The cut side's edges in a plane that one triangle uses, each with that triangle's far
    // corner, in triangle order.
    let mut uses: HashMap<(i32, i32), u32> = HashMap::new();
    for t in &kept {
        for (a, b) in [(t[0], t[1]), (t[1], t[2]), (t[2], t[0])] {
            *uses.entry((a.min(b), a.max(b))).or_insert(0) += 1;
        }
    }
    let in_plane =
        |v: i32, end: &ChainEnd| end.plane.depth(positions[v as usize]).abs() <= tolerance;
    // Per end, the trace edges leaving its end vertex (one is what bounds and fills there).
    let mut at_end: Vec<Vec<TraceAtEnd>> = vec![Vec::new(); ends.len()];
    // Each strip as its edge `a -> b`, its end, and how far out each end goes.
    let mut runs: Vec<(i32, i32, usize, Vector3, Vector3)> = Vec::new();
    for t in &kept {
        for (a, b, c) in [(t[0], t[1], t[2]), (t[1], t[2], t[0]), (t[2], t[0], t[1])] {
            if uses.get(&(a.min(b), a.max(b))) != Some(&1) {
                continue;
            }
            let Some(h) = ends.iter().position(|e| in_plane(a, e) && in_plane(b, e)) else {
                continue;
            };
            let end = &ends[h];
            let (pa, pb, pc) = (
                positions[a as usize],
                positions[b as usize],
                positions[c as usize],
            );
            let normal = (pb - pa).cross(pc - pa);
            let along = pb - pa;
            if !(normal.length() > 0.0) || !(along.length() > 0.0) {
                continue;
            }
            if at_vertex(&positions, a, end, tolerance) || at_vertex(&positions, b, end, tolerance)
            {
                let away = if at_vertex(&positions, a, end, tolerance) {
                    along
                } else {
                    -along
                };
                at_end[h].push(TraceAtEnd {
                    normal: normal.get_normal(),
                    away: away.get_normal(),
                });
            }
            // Away from the triangle, square to the edge, in the triangle's plane.
            let out = along.get_normal().cross(normal.get_normal()).get_normal();
            let past = -out.dot(end.plane.normal);
            if !(past > LEAST_STRIP_COSINE) {
                continue;
            }
            let reach = (STRIP_RADII * radius) / past;
            runs.push((a, b, h, pa + (out * reach), pb + (out * reach)));
        }
    }

    // Each end's bound: the plane through the chain's line holding the normal of the triangle
    // whose trace edge leaves the end vertex, kept on the side its trace leaves into. With no
    // such edge, or two, the end is left unbounded (a documented hole, see the file header).
    let bounds: Vec<Option<(HalfSpace, TraceAtEnd)>> = ends
        .iter()
        .zip(&at_end)
        .map(|(end, traces)| {
            let [trace] = traces.as_slice() else {
                return None;
            };
            let across = trace.normal.cross(end.along);
            if !(across.length() > 0.0) {
                return None;
            }
            let across = across.get_normal();
            let normal = if across.dot(trace.away) < 0.0 {
                -across
            } else {
                across
            };
            Some((
                HalfSpace {
                    origin: end.vertex,
                    normal,
                },
                *trace,
            ))
        })
        .collect();

    // Each strip leaves each end `v` of its edge toward a new far vertex; `into` says whether the
    // side's edge runs into `v` (its `b`) or out of it (its `a`), so the fills can orient.
    let mut extension: Vec<([i32; 3], usize)> = Vec::new();
    let mut strips: Vec<Strip> = Vec::new();
    let mut leaving: HashMap<i32, Vec<(i32, bool, usize)>> = HashMap::new();
    let mut leaving_order: Vec<i32> = Vec::new();
    for (a, b, end, far_a, far_b) in runs {
        // Two strips leaving one vertex in the same direction (a prism's walls past a square end,
        // collinear trace edges) share their far vertex there, so the seam between them is no
        // open edge: no wedge fits between them, and an unshared seam would read as a rim.
        let mut far_of = |v: i32, at: Vector3| -> i32 {
            let shared = leaving.get(&v).and_then(|around| {
                around.iter().find_map(|&(far, _, on)| {
                    (on == end && (positions[far as usize] - at).length() <= tolerance)
                        .then_some(far)
                })
            });
            shared.unwrap_or_else(|| {
                positions.push(at);
                // A part's vertex count fits an i32 (the mesh's own indices are i32).
                (positions.len() - 1) as i32
            })
        };
        let (fa, fb) = (far_of(a, far_a), far_of(b, far_b));
        extension.push(([b, a, fa], end));
        extension.push(([b, fa, fb], end));
        for (v, far, into) in [(a, fa, false), (b, fb, true)] {
            let around = leaving.entry(v).or_default();
            if around.is_empty() {
                leaving_order.push(v);
            }
            around.push((far, into, end));
        }
        strips.push(Strip {
            a,
            b,
            end,
            far_a: fa,
            far_b: fb,
        });
    }

    // Fill the wedge where two strips leave one vertex, one whose side edge runs into it and one
    // out of it. The strip into `v` runs `far -> v` and the one out of it `v -> far`, so the wedge
    // runs `v, far_in, far_out` and every edge it shares is run both ways.
    for &v in &leaving_order {
        let around = &leaving[&v];
        if around.len() != 2 {
            continue;
        }
        let (Some(&(far_in, _, end)), Some(&(far_out, _, _))) = (
            around.iter().find(|(_, into, _)| *into),
            around.iter().find(|(_, into, _)| !*into),
        ) else {
            continue;
        };
        if !wide(&positions, v, far_in, far_out, tolerance) {
            continue;
        }
        extension.push(([v, far_in, far_out], end));
    }

    // Fill the wedge between the chain's line and the strip at the end vertex, where the strip
    // runs out away from the line (inside the bound): in the strip's plane, out along the line
    // as far past the end plane as the strip reaches.
    for strip in &strips {
        let Some((bound, trace)) = bounds[strip.end] else {
            continue;
        };
        let end = &ends[strip.end];
        let (v, far, into) = if at_vertex(&positions, strip.a, end, tolerance) {
            (strip.a, strip.far_a, false)
        } else if at_vertex(&positions, strip.b, end, tolerance) {
            (strip.b, strip.far_b, true)
        } else {
            continue;
        };
        let p = positions[v as usize];
        if !(bound.depth(positions[far as usize]) > tolerance) {
            continue;
        }
        let along = end.along.get_normal();
        let in_strip = along - (trace.normal * along.dot(trace.normal));
        if !(in_strip.length() > 0.0) {
            continue;
        }
        let in_strip = in_strip.get_normal();
        let past = -in_strip.dot(end.plane.normal);
        if !(past > LEAST_STRIP_COSINE) {
            continue;
        }
        positions.push(p + (in_strip * ((STRIP_RADII * radius) / past)));
        // A part's vertex count fits an i32 (the mesh's own indices are i32).
        let on_line = (positions.len() - 1) as i32;
        // The strip runs `v -> far` where the side's edge leaves `v`, `far -> v` where it runs in.
        let fill = if into {
            [v, far, on_line]
        } else {
            [v, on_line, far]
        };
        if wide(&positions, fill[0], fill[1], fill[2], tolerance) {
            extension.push((fill, strip.end));
        }
    }

    // Cut every piece of an end's extension at that end's bound (plane keys after the ends').
    for (triangle, end) in extension {
        let polygon = match bounds[end] {
            Some((bound, _)) => cut(
                triangle.to_vec(),
                &bound,
                ends.len() + end,
                tolerance,
                &mut positions,
                &mut crossings,
            ),
            None => triangle.to_vec(),
        };
        for i in 1..polygon.len().saturating_sub(1) {
            kept.push([polygon[0], polygon[i], polygon[i + 1]]);
        }
    }
    // The strips leaving the trace's far ends, each `(v, far)`: where the side's own rim reaches
    // the end plane, its continuation past it.
    let carried: Vec<(i32, i32)> = strips
        .iter()
        .flat_map(|strip| [(strip.a, strip.far_a), (strip.b, strip.far_b)])
        .filter(|(v, _)| leaving.get(v).is_some_and(|around| around.len() == 1))
        .collect();
    let holes = holes_of(&positions, &kept, ends, cut_count, &carried, tolerance);
    ExtendedSide {
        side: RollingBallSide::new(&positions, &kept),
        holes,
    }
}

/// The open edges of the extended side `kept` that are holes in it, not the side's own boundary
/// carried on (the file header's list), and every vertex such an edge touches. The side's own
/// boundary is an edge of the cut side (both ends below `cut_count`) that lies in no end plane
/// and touches no chain end vertex (the chain's own edge, or the line carried on past it), and
/// the strip edge leaving such a rim where it reaches an end plane (one of `carried`, or the part
/// of it the bound keeps).
fn holes_of(
    positions: &[Vector3],
    kept: &[[i32; 3]],
    ends: &[ChainEnd],
    cut_count: usize,
    carried: &[(i32, i32)],
    tolerance: f64,
) -> Holes {
    let mut uses: HashMap<(i32, i32), u32> = HashMap::new();
    let mut open: Vec<(i32, i32)> = Vec::new();
    for t in kept {
        for (a, b) in [(t[0], t[1]), (t[1], t[2]), (t[2], t[0])] {
            let key = (a.min(b), a.max(b));
            let count = uses.entry(key).or_insert(0);
            if *count == 0 {
                open.push(key);
            }
            *count += 1;
        }
    }
    open.retain(|key| uses.get(key) == Some(&1));
    let at_an_end = |v: i32| {
        ends.iter()
            .any(|end| at_vertex(positions, v, end, tolerance))
    };
    let in_one_plane = |a: i32, b: i32| {
        ends.iter().any(|end| {
            end.plane.depth(positions[a as usize]).abs() <= tolerance
                && end.plane.depth(positions[b as usize]).abs() <= tolerance
        })
    };
    // A vertex index a part never reaches is past every cut vertex (the cut side's indices fit
    // an i32, as the mesh's do).
    let of_the_cut = |v: i32| (v as usize) < cut_count;
    let own_rim = |a: i32, b: i32| {
        of_the_cut(a) && of_the_cut(b) && !in_one_plane(a, b) && !at_an_end(a) && !at_an_end(b)
    };
    // Where the side's own rim reaches each vertex, so a strip carries it on only from there.
    let mut rim_reaches: HashSet<i32> = HashSet::new();
    for &(a, b) in &open {
        if own_rim(a, b) {
            rim_reaches.insert(a);
            rim_reaches.insert(b);
        }
    }
    let carried_on = |a: i32, b: i32| {
        carried.iter().any(|&(v, far)| {
            if !rim_reaches.contains(&v) || at_an_end(v) {
                return false;
            }
            let other = if a == v {
                b
            } else if b == v {
                a
            } else {
                return false;
            };
            other == far || on_segment(positions, other, v, far, tolerance)
        })
    };
    let mut holes = Holes::default();
    for (a, b) in open {
        if own_rim(a, b) || carried_on(a, b) {
            continue;
        }
        holes.edges.insert((a, b));
        holes.vertices.insert(a);
        holes.vertices.insert(b);
    }

    holes
}

/// Whether vertex `p` lies on the segment from `v` to `far`, within `tolerance`.
fn on_segment(positions: &[Vector3], p: i32, v: i32, far: i32, tolerance: f64) -> bool {
    let (p, from, to) = (
        positions[p as usize],
        positions[v as usize],
        positions[far as usize],
    );
    let along = to - from;
    let length_squared = along.length_squared();
    if length_squared.is_nan() || length_squared <= 0.0 {
        return false;
    }
    let t = (p - from).dot(along) / length_squared;
    let nearest = from + (along * t.clamp(0.0, 1.0));
    (p - nearest).length() <= tolerance
}

/// Whether vertex `v` is the chain's end vertex at `end`, within `tolerance`.
fn at_vertex(positions: &[Vector3], v: i32, end: &ChainEnd, tolerance: f64) -> bool {
    (positions[v as usize] - end.vertex).length() <= tolerance
}

/// Whether the triangle `a, b, c` has an area worth a fill (twice its area past `tolerance`
/// squared).
fn wide(positions: &[Vector3], a: i32, b: i32, c: i32, tolerance: f64) -> bool {
    let (p, q, r) = (
        positions[a as usize],
        positions[b as usize],
        positions[c as usize],
    );
    (q - p).cross(r - p).length() > tolerance * tolerance
}

#[cfg(test)]
mod tests {
    use super::*;

    /// A floor `z = 0` from x = 0 to 4 and y = -4 to 4, wound up, as a side; the chain runs along
    /// its edge x = 0, toward +y.
    fn floor() -> RollingBallSide {
        let positions = vec![
            Vector3::new(0.0, -4.0, 0.0),
            Vector3::new(4.0, -4.0, 0.0),
            Vector3::new(4.0, 4.0, 0.0),
            Vector3::new(0.0, 4.0, 0.0),
        ];
        RollingBallSide::new(&positions, &[[0, 1, 2], [0, 2, 3]])
    }

    /// The chain's end at the origin on an end plane leaning `degrees` off square to the chain
    /// (the y axis), its normal into the chain's side, the chain carried on along +y. At a
    /// negative lean the floor's trace in it runs from the origin down to y = x tan(degrees), so
    /// the strip at the origin runs out away from the chain's line; at a positive lean up to
    /// y = x tan(degrees), so the strip at the origin runs out across it.
    fn leaning(degrees: f64) -> ChainEnd {
        let lean = degrees.to_radians();
        ChainEnd {
            plane: HalfSpace {
                origin: Vector3::ZERO,
                normal: Vector3::new(lean.sin(), -lean.cos(), 0.0),
            },
            vertex: Vector3::ZERO,
            along: Vector3::new(0.0, 1.0, 0.0),
        }
    }

    /// Cut at a plane leaning either way, the floor, its strip and the fills are one surface
    /// wound one way: every edge two triangles share is run once each way, and no edge twice the
    /// same way.
    #[test]
    fn the_extended_side_is_wound_one_way() {
        for degrees in [-25.0, 25.0] {
            let side = extended_past(&floor(), &[leaning(degrees)], 2.0, 1e-9).side;
            let mut runs: HashMap<(i32, i32), u32> = HashMap::new();
            for t in &side.triangles {
                for (a, b) in [(t[0], t[1]), (t[1], t[2]), (t[2], t[0])] {
                    *runs.entry((a, b)).or_insert(0) += 1;
                }
            }
            assert!(side.triangles.len() > 2, "the strip was added");
            for (&(a, b), &count) in &runs {
                assert_eq!(
                    count, 1,
                    "{degrees}: edge {a}-{b} run {count} times one way"
                );
            }
            // The floor's trace in the plane is shared with the strip: run once each way.
            let shared = runs
                .keys()
                .filter(|&&(a, b)| runs.contains_key(&(b, a)))
                .count();
            assert!(shared >= 2, "{degrees}: {runs:?}");
        }
    }

    /// Past the plane the strip carries the floor on, so a point above it touches the floor's own
    /// plane there, on no rim; and beside the chain's line carried on past the end vertex
    /// (x = 0), where the strip runs square to the slanted trace and not along the chain, the
    /// wedge between them is filled: a point above it touches the floor's plane too, a radius
    /// below it.
    #[test]
    fn past_the_plane_the_floor_runs_on_beside_the_chain_too() {
        let side = extended_past(&floor(), &[leaning(-25.0)], 2.0, 1e-9).side;
        // Above the strip: past the trace, which falls 25 degrees in y along x from the origin.
        let above = side
            .closest(Vector3::new(2.0, 0.0, 1.0))
            .expect("the side has triangles");
        assert!((above.distance - 1.0).abs() < 1e-12, "{above:?}");
        assert!(!above.feature.is_rim(), "{above:?}");
        // In the wedge between the chain's line past the origin and the strip.
        for at in [Vector3::new(0.1, 1.0, 1.0), Vector3::new(0.01, 3.0, 1.0)] {
            let wedge = side.closest(at).expect("the side has triangles");
            assert!(!wedge.feature.is_rim(), "{at:?}: {wedge:?}");
            assert!((wedge.distance - 1.0).abs() < 1e-12, "{at:?}: {wedge:?}");
            assert!(
                (wedge.point - Vector3::new(at.x, at.y, 0.0)).length() < 1e-12,
                "{at:?}: {wedge:?}"
            );
        }
    }

    /// Leaning so the strip at the end vertex runs out across the chain's line, the extension
    /// stops at the line: no triangle of it reaches x < 0, where the trim at the chain removes
    /// the floor's continuation.
    #[test]
    fn no_strip_reaches_across_the_chains_line() {
        let side = extended_past(&floor(), &[leaning(25.0)], 2.0, 1e-9).side;
        assert!(side.triangles.len() > 2, "the strip was added");
        for t in &side.triangles {
            for &v in t {
                let p = side.positions[v as usize];
                assert!(p.x >= -1e-9, "triangle {t:?} reaches {p:?}");
            }
        }
    }

    /// The hand answer past the chain's line: above (-0.5, 1) the floor carried on unbounded is a
    /// radius below at (-0.5, 1, 0), a contact nearer than the trimmed side gives. The trimmed
    /// side's nearest point is on the chain's line, (0, 1, 0), sqrt(1.25) away, and it is the
    /// side's boundary there (a rim, as the chain edge is).
    #[test]
    fn past_the_chains_line_the_side_reads_only_up_to_the_line() {
        let side = extended_past(&floor(), &[leaning(25.0)], 2.0, 1e-9).side;
        let beyond = side
            .closest(Vector3::new(-0.5, 1.0, 1.0))
            .expect("the side has triangles");
        assert!(
            (beyond.distance - 1.25f64.sqrt()).abs() < 1e-12,
            "{beyond:?}"
        );
        assert!(
            (beyond.point - Vector3::new(0.0, 1.0, 0.0)).length() < 1e-12,
            "{beyond:?}"
        );
        assert!(beyond.feature.is_rim(), "{beyond:?}");
        // Inside the line the cut strip still carries the floor on.
        let inside = side
            .closest(Vector3::new(0.5, 1.0, 1.0))
            .expect("the side has triangles");
        assert!((inside.distance - 1.0).abs() < 1e-12, "{inside:?}");
        assert!(!inside.feature.is_rim(), "{inside:?}");
    }

    /// Cut square at y = 0 (the chain along x = 0 ending at the origin), the floor's trace in the
    /// plane runs from the origin through (2, 0, 0), where its diagonal crosses the plane, to its
    /// far corner (4, 0, 0), where the floor's own rim x = 4
    /// reaches the plane, and the strip carries that rim on past it. A contact on the rim, on its
    /// corner in the plane or on the strip's edge carrying it on rests on the floor's own boundary;
    /// one on the strip's far edge (past its reach) or on the chain's line past the end vertex
    /// rests on a hole of the extension, as does one on the chain's own edge into the end vertex.
    #[test]
    fn the_sides_own_rim_carried_on_is_no_hole_and_the_strips_far_edge_is() {
        let extended = extended_past(&floor(), &[leaning(0.0)], 2.0, 1e-9);
        for (at, hole) in [
            (Vector3::new(5.0, -1.0, 1.0), false),
            (Vector3::new(5.0, 0.0, 1.0), false),
            (Vector3::new(5.0, 1.0, 1.0), false),
            (Vector3::new(3.0, 5.0, 1.0), true),
            (Vector3::new(-1.0, 1.0, 1.0), true),
            // The chain's own edge, cut at the plane, into the end vertex.
            (Vector3::new(-1.0, -1.0, 1.0), true),
        ] {
            let contact = extended.side.closest(at).expect("the side has triangles");
            assert!(contact.feature.is_rim(), "{at:?}: {contact:?}");
            assert_eq!(
                extended.on_a_hole(&contact.feature),
                hole,
                "{at:?}: {contact:?}"
            );
        }
        // Over the strip itself the contact is on its face, no rim at all; and over the seam
        // where the floor's diagonal meets the plane at (2, 0, 0), whose two trace edges' strips
        // leave in the same direction and share their far vertex, it is on no rim either.
        for at in [Vector3::new(3.0, 1.0, 1.0), Vector3::new(2.0, 1.0, 1.0)] {
            let over = extended.side.closest(at).expect("the side has triangles");
            assert!(!over.feature.is_rim(), "{at:?}: {over:?}");
            assert!(!extended.on_a_hole(&over.feature), "{at:?}: {over:?}");
        }
    }
}
