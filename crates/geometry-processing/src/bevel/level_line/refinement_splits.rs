//! The mesh edits the refinement makes and the bookkeeping that goes with them: where a point
//! stands in a triangle ([`PointPlace`],
//! [`where_the_point_stands`]) the triangle-size readings the refinement splits by
//! ([`cannot_be_divided_further`], [`longest_edge_length`], [`longest_edge`], [`aspect`],
//! [`is_new_sliver`]) and the queue bookkeeping ([`enqueue`]); the splits themselves are in
//! `refinement_splits_enqueue`.

use std::collections::{HashMap, VecDeque};

use indexmap::IndexSet;
use vector_math::{ieee_math, Vector3};

use super::{LevelLineMesh, LevelLineTriangle};

/// True when a triangle stands no higher than `floor` over its longest edge, with no area that
/// any division can improve. One implementation, and it is the MESH's: the mesh asks the same
/// question of itself inside its split backstop and the repair, and two copies of a tolerance
/// test are two things to keep in step. This is the name the rules read it by.
pub fn has_no_height(mesh: &LevelLineMesh, t: LevelLineTriangle, floor: f64) -> bool {
    mesh.has_no_height(t, floor)
}

/// The triangles standing on the edges from `parent`'s corners to a vertex `inserted` inside it -
/// the triangles round a point the split just put in.
pub fn neighbours_of_vertex(
    mesh: &LevelLineMesh,
    parent: LevelLineTriangle,
    inserted: i32,
) -> Vec<i32> {
    let mut around = mesh.triangles_on(parent.a, inserted).to_vec();
    around.extend_from_slice(mesh.triangles_on(parent.b, inserted));
    around.extend_from_slice(mesh.triangles_on(parent.c, inserted));
    around
}

/// Where a point stands in a triangle - the three answers, and they are not
/// overlapping.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, Default)]
pub enum PointPlace {
    /// Off every edge: the triangle itself is divided about it.
    #[default]
    Interior = 0,

    /// On one edge, away from both its ends: that edge is split, and its neighbour with it.
    OnAnEdge,

    /// At a corner the mesh already carries: there is nothing to insert.
    AtACorner,
}

/// True when a triangle is already below the size any accuracy asked for, so dividing it again
/// cannot make the line through it better - the one state the refinement refuses in rather than
/// loop.
pub fn cannot_be_divided_further(mesh: &LevelLineMesh, triangle: i32, floor: f64) -> bool {
    longest_edge_length(mesh, mesh.triangles()[triangle as usize]) <= floor
}

/// Where `point` stands in triangle `t`, with the corner (`from == to`), the edge's two ends, or
/// (-1, -1) in the interior.
///
/// `same_place` is how near an existing corner the point has to stand to BE that corner, as a
/// length; `None` asks by the triangle's own millionth, which is what the field rules want.
///
/// The corner is tested FIRST and it is the whole point of this routine. Every corner is also at
/// the END of two edges, and an on-edge test whose parameter is clamped to [0, 1] reads a corner
/// as standing on both of them - so a search answering with a corner of the asking triangle had
/// its answer "split that edge", and the edge was split at its own end: a second vertex id at a
/// place the mesh already carried, a zero-length edge and a zero-area triangle every time. It is
/// not a rare case: the apex of a fan round a hub is minted by an on-edge split of a neighbour, so
/// it LIES on the edge about to be split, and the searches keep landing there (127 exact
/// coincidences on the Many Holes witness).
///
/// How WIDE "the same place" is, though, is the caller's to say, because the callers do not all
/// work at one scale. A field rule's point comes from a bounded search over a whole triangle and
/// may miss the corner it means by a millionth of the size, so a millionth is what catches it. A
/// caller whose point is the last surviving scrap of something - the side witness - is working in
/// a strip only a couple of snaps wide, and a millionth of the triangle would swallow the very
/// point it was sent to find.
pub fn where_the_point_stands(
    mesh: &LevelLineMesh,
    t: LevelLineTriangle,
    point: Vector3,
    same_place: Option<f64>,
) -> (PointPlace, i32, i32) {
    let size = longest_edge_length(mesh, t);
    let at_the_corner = same_place.unwrap_or(1e-6 * size);
    for corner in 0..3 {
        let v = t.corner(corner);
        if (point - mesh.position(v)).length() <= at_the_corner {
            return (PointPlace::AtACorner, v, v);
        }
    }

    for corner in 0..3 {
        let v = t.corner(corner);
        let w = t.corner((corner + 1) % 3);
        let a = mesh.position(v);
        let b = mesh.position(w);
        let along = b - a;
        let length_squared = along.length_squared();
        if length_squared <= 0.0 {
            continue;
        }

        // NaN passes through the clamp, so a broken reading stays visible.
        let s = ((point - a).dot(along) / length_squared).clamp(0.0, 1.0);
        if (point - (a + (along * s))).length() <= 1e-6 * size {
            return (PointPlace::OnAnEdge, v, w);
        }
    }

    (PointPlace::Interior, -1, -1)
}

/// The length of a triangle's longest side.
pub fn longest_edge_length(mesh: &LevelLineMesh, t: LevelLineTriangle) -> f64 {
    let mut longest = 0.0;
    for corner in 0..3 {
        longest = ieee_math::max_f64(
            longest,
            (mesh.position(t.corner((corner + 1) % 3)) - mesh.position(t.corner(corner))).length(),
        );
    }

    longest
}

/// A triangle's longest edge as (low, high). Ties break on the index pair so the same mesh always
/// splits the same edge - and a "tie" has to mean nearly equal, because a part that has been moved
/// carries a rounding in its lengths and would otherwise split a symmetric triangle differently
/// from the same part left where it was.
pub fn longest_edge(mesh: &LevelLineMesh, triangle: i32) -> (i32, i32) {
    let t = mesh.triangles()[triangle as usize];
    let mut best = (i32::MAX, i32::MAX);
    let mut best_length = -1.0;
    for corner in 0..3 {
        let v = t.corner(corner);
        let w = t.corner((corner + 1) % 3);
        let low = v.min(w);
        let high = v.max(w);
        let length = mesh.length(v, w);
        let tie = (length - best_length).abs()
            <= 1e-12 * ieee_math::max_f64(length, f64::abs(best_length));
        if (length > best_length && !tie)
            || (tie && (low < best.0 || (low == best.0 && high < best.1)))
        {
            best_length = length;
            best = (low, high);
        }
    }

    best
}

/// What a split that has nothing to insert reports in place of a vertex.
pub const NOTHING_TO_INSERT: i32 = -1;

/// Queues each of `triangles` once (in first-seen order, so the refinement runs the same way every
/// time) at
/// refinement depth `at`.
pub fn enqueue(
    triangles: impl IntoIterator<Item = i32>,
    pending: &mut VecDeque<i32>,
    depth: &mut HashMap<i32, i32>,
    at: i32,
) {
    let distinct: IndexSet<i32> = triangles.into_iter().collect();

    for child in distinct {
        depth.insert(child, at);
        pending.push_back(child);
    }
}

/// Whether a child triangle of aspect `child` is a NEW sliver: past the limit and worse than its
/// `parent` was.
pub fn is_new_sliver(child: f64, parent: f64, aspect_limit: f64) -> bool {
    child > aspect_limit && child > parent
}

/// The aspect of the triangle abc: its longest edge squared over twice its area - infinity for a
/// triangle of no area.
pub fn aspect(a: Vector3, b: Vector3, c: Vector3) -> f64 {
    let longest = ieee_math::max_f64(
        (b - a).length(),
        ieee_math::max_f64((c - b).length(), (a - c).length()),
    );
    let twice_area = (b - a).cross(c - a).length();

    if twice_area > 0.0 {
        longest * longest / twice_area
    } else {
        f64::INFINITY
    }
}

/// An edge at least this share of its triangle's longest is a long
/// side, the only kind a split is allowed to thin.
pub const LONG_SIDE: f64 = 0.9;

/// How far a longest-edge path is followed.
pub const LONGEST_EDGE_PATH_CAP: i32 = 32;

/// Whether splitting edge `v`-`w` at `point` would
/// leave a new sliver in a triangle standing on it.
pub fn would_thin_a_child_on_edge(
    mesh: &LevelLineMesh,
    v: i32,
    w: i32,
    point: Vector3,
    aspect_limit: f64,
    shortest_guarded: f64,
) -> bool {
    triangle_thinned_by_on_edge(mesh, v, w, point, aspect_limit, shortest_guarded, None) >= 0
}

/// The first triangle on edge `v`-`w` (of
/// `tag`, where one is named) that splitting the edge at `point` would thin, or `-1`.
pub fn triangle_thinned_by_on_edge(
    mesh: &LevelLineMesh,
    v: i32,
    w: i32,
    point: Vector3,
    aspect_limit: f64,
    shortest_guarded: f64,
    tag: Option<i32>,
) -> i32 {
    let a = mesh.position(v);
    let b = mesh.position(w);
    let along = b - a;
    let length_squared = along.length_squared();
    if length_squared <= 0.0 {
        return -1;
    }

    let s = (point - a).dot(along) / length_squared;
    if s <= 1.0 / aspect_limit || s >= 1.0 - (1.0 / aspect_limit) {
        return -1;
    }

    for &parent in mesh.triangles_on(v, w) {
        let face = mesh.triangles()[parent as usize];
        if tag.is_some_and(|tag| face.tag != tag) {
            continue;
        }

        let longest = longest_edge_length(mesh, face);
        if longest <= shortest_guarded || length_squared.sqrt() < LONG_SIDE * longest {
            continue;
        }

        let x = mesh.position(if face.a != v && face.a != w {
            face.a
        } else if face.b != v && face.b != w {
            face.b
        } else {
            face.c
        });
        let before = aspect(a, b, x);
        if is_new_sliver(aspect(a, point, x), before, aspect_limit)
            || is_new_sliver(aspect(point, b, x), before, aspect_limit)
        {
            return parent;
        }
    }

    -1
}

/// Whether splitting `triangle` at `point`
/// would leave a new sliver.
pub fn would_thin_a_child(
    mesh: &LevelLineMesh,
    triangle: i32,
    point: Vector3,
    aspect_limit: f64,
    shortest_guarded: f64,
) -> bool {
    triangle_thinned_by(mesh, triangle, point, aspect_limit, shortest_guarded, false) >= 0
}

/// The triangle a split at
/// `point` would thin - `triangle` itself for an interior point, a triangle on the edge for a
/// point on one (only of `triangle`'s own tag when `own_tag_only`) - or `-1`.
pub fn triangle_thinned_by(
    mesh: &LevelLineMesh,
    triangle: i32,
    point: Vector3,
    aspect_limit: f64,
    shortest_guarded: f64,
    own_tag_only: bool,
) -> i32 {
    let t = mesh.triangles()[triangle as usize];
    match where_the_point_stands(mesh, t, point, None) {
        (PointPlace::AtACorner, _, _) => return -1,
        (PointPlace::OnAnEdge, from, to) => {
            return triangle_thinned_by_on_edge(
                mesh,
                from,
                to,
                point,
                aspect_limit,
                shortest_guarded,
                own_tag_only.then_some(t.tag),
            );
        }
        (PointPlace::Interior, _, _) => {}
    }

    let longest = longest_edge_length(mesh, t);
    if longest <= shortest_guarded {
        return -1;
    }

    for corner in 0..3 {
        if (point - mesh.position(t.corner(corner))).length() <= longest / aspect_limit {
            return -1;
        }
    }

    let before = aspect(mesh.position(t.a), mesh.position(t.b), mesh.position(t.c));
    for corner in 0..3 {
        let a = mesh.position(t.corner(corner));
        let b = mesh.position(t.corner((corner + 1) % 3));
        if (b - a).length() >= LONG_SIDE * longest
            && is_new_sliver(aspect(a, b, point), before, aspect_limit)
        {
            return triangle;
        }
    }

    -1
}

/// Under the meter, which of the
/// would-thin limits splitting edge `v`-`w` at `point` would trip, counted against `rule`.
pub fn meter_would_thin_on_edge(
    rule: super::field_counters::LevelLineRule,
    mesh: &LevelLineMesh,
    v: i32,
    w: i32,
    point: Vector3,
    chord_tolerance: f64,
) {
    use super::field_counters::{FieldCounters, WOULD_THIN_LIMITS};

    if !FieldCounters::meter_on() {
        return;
    }

    for (i, &limit) in WOULD_THIN_LIMITS.iter().enumerate() {
        if would_thin_a_child_on_edge(mesh, v, w, point, limit, 2.0 * chord_tolerance) {
            FieldCounters::note_would_thin(rule, i as i32);
        }
    }
}

/// The same for a split of
/// `triangle` at `point`.
pub fn meter_would_thin(
    rule: super::field_counters::LevelLineRule,
    mesh: &LevelLineMesh,
    triangle: i32,
    point: Vector3,
    chord_tolerance: f64,
) {
    use super::field_counters::{FieldCounters, WOULD_THIN_LIMITS};

    if !FieldCounters::meter_on() {
        return;
    }

    for (i, &limit) in WOULD_THIN_LIMITS.iter().enumerate() {
        if would_thin_a_child(mesh, triangle, point, limit, 2.0 * chord_tolerance) {
            FieldCounters::note_would_thin(rule, i as i32);
        }
    }
}

/// The edge the longest-edge path from `start` ends on - where two
/// triangles share their longest edge, or the path meets a boundary or another tag's fold -
/// or `None` where it runs onto a non-manifold or chain edge or past
/// [`LONGEST_EDGE_PATH_CAP`] steps.
pub fn terminal_edge(mesh: &LevelLineMesh, start: i32) -> Option<(i32, i32)> {
    let mut at = start;
    let mut terminal = longest_edge(mesh, at);

    for _ in 0..LONGEST_EDGE_PATH_CAP {
        let on = mesh.triangles_on(terminal.0, terminal.1);
        if on.len() > 2 || mesh.edge_lies_on_a_chain(terminal.0, terminal.1) {
            return None;
        }

        // A fold between two tags ends the path as a boundary edge does.
        let across = if on.len() == 2 {
            if on[0] == at {
                on[1]
            } else {
                on[0]
            }
        } else {
            -1
        };
        if across < 0 || mesh.triangles()[across as usize].tag != mesh.triangles()[at as usize].tag
        {
            return Some(terminal);
        }

        let next = longest_edge(mesh, across);
        if next == terminal {
            return Some(terminal);
        }

        at = across;
        terminal = next;
    }

    None
}
