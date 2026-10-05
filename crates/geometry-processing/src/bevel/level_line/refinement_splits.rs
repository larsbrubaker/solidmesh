//! The mesh edits the refinement makes and the bookkeeping that goes with them. Ports
//! `Bevel/LevelLine/RefinementSplits.cs`: where a point stands in a triangle ([`PointPlace`],
//! [`where_the_point_stands`]) the triangle-size readings the refinement splits by
//! ([`cannot_be_divided_further`], [`longest_edge_length`], [`longest_edge`], [`aspect`],
//! [`is_new_sliver`]) and the queue bookkeeping ([`enqueue`]); the splits themselves land with later
//! packages.

use std::collections::{HashMap, VecDeque};

use indexmap::IndexSet;
use vector_math::{cs_math, Vector3};

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

/// C# `PointPlace`: where a point stands in a triangle - the three answers, and they are not
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

        // NaN passes through the clamp, as C#'s two comparisons let it.
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
        longest = cs_math::max_f64(
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
        let tie =
            (length - best_length).abs() <= 1e-12 * cs_math::max_f64(length, f64::abs(best_length));
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

/// Queues each of `triangles` once (in first-seen order, as C#'s `Distinct` keeps it) at
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
    let longest = cs_math::max_f64(
        (b - a).length(),
        cs_math::max_f64((c - b).length(), (a - c).length()),
    );
    let twice_area = (b - a).cross(c - a).length();

    if twice_area > 0.0 {
        longest * longest / twice_area
    } else {
        f64::INFINITY
    }
}
