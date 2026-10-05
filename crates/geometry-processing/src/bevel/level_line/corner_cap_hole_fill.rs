//! The corner cap's hole fill. Ports `Bevel/LevelLine/CornerCap.HoleFill.cs` (a partial of the C#
//! static class `CornerCap`): so far its geometry readings - a row's cross-tangent
//! ([`arc_cross_tangent`]), the direction away from a triangle's third corner
//! ([`away_from_third_corner`]), a facet normal ([`triangle_normal`]), the removed triangles on a
//! loop ([`removed_triangles_on_the_loop`]) and the fill vertex standing furthest through the faces
//! the rows leave along ([`bulging_fill_vertex`]), the cross-tangent laid into the crease faces
//! ([`leaves_through_convex_crease`], [`inside_the_creases`], [`faded`], [`out_of_the_planes`]),
//! an edge's direction on a triangle ([`runs_forward`]) and the patch placed on the rolling ball
//! ([`on_the_ball`], [`covers_the_ball_once`]); the fill itself lands with later packages. Its
//! switch is in `corner_cap_hole_fill_switch.rs`.

use indexmap::IndexMap;
use std::collections::HashSet;

use vector_math::{cs_math, Vector3};

use super::{
    ChainField, CornerCapLastRow, HoleFillResult, HoleFillTriangle, LevelLineMesh,
    LevelLineTriangle,
};

/// The unit cross-tangent of a row's arc at its end `p`, turned toward `v`: a skew row's axis (its
/// fillet's ruling), else its plane's normal; zero for an arc with no plane.
pub fn arc_cross_tangent(
    mesh: &LevelLineMesh,
    row: &CornerCapLastRow,
    p: Vector3,
    v: Vector3,
) -> Vector3 {
    // a skew row (LastRow.Axis) is not a plane arc: its cross-tangent is the fillet's ruling
    let mut normal = if row.axis.length_squared() > 0.0 {
        row.axis
    } else {
        (mesh.position(row.arc[0]) - row.centre)
            .cross(mesh.position(row.arc[row.arc.len() - 1]) - row.centre)
    };
    if normal.length_squared() < 1e-24 {
        return Vector3::ZERO;
    }

    normal = normal.get_normal();
    let mut toward = v - row.chain_point;
    if toward.length_squared() < 1e-24 {
        toward = v - p;
    }

    if normal.dot(toward) < 0.0 {
        normal * -1.0
    } else {
        normal
    }
}

/// The unit direction square to edge `a b`, away from `face`'s third corner; zero where the corner
/// stands on the edge's line.
pub fn away_from_third_corner(
    mesh: &LevelLineMesh,
    face: LevelLineTriangle,
    a: i32,
    b: i32,
) -> Vector3 {
    let c = if face.a != a && face.a != b {
        face.a
    } else if face.b != a && face.b != b {
        face.b
    } else {
        face.c
    };
    let d = (mesh.position(b) - mesh.position(a)).get_normal();
    let off = mesh.position(c) - mesh.position(a);
    let perpendicular = off - (d * off.dot(d));
    if perpendicular.length_squared() < 1e-24 {
        Vector3::ZERO
    } else {
        (perpendicular * -1.0).get_normal()
    }
}

/// A triangle's unit normal, or zero for one with no area.
pub fn triangle_normal(mesh: &LevelLineMesh, face: LevelLineTriangle) -> Vector3 {
    let normal = (mesh.position(face.b) - mesh.position(face.a))
        .cross(mesh.position(face.c) - mesh.position(face.a));
    if normal.length_squared() < 1e-30 {
        Vector3::ZERO
    } else {
        normal.get_normal()
    }
}

/// Per edge of the closed `cycle` (keyed low, high), the removed triangles standing on it, in
/// triangle order; the map keeps the order the edges were first met in.
pub fn removed_triangles_on_the_loop(
    mesh: &LevelLineMesh,
    cycle: &[i32],
) -> IndexMap<(i32, i32), Vec<i32>> {
    let mut edges = HashSet::new();
    for i in 0..cycle.len() {
        let a = cycle[i];
        let b = cycle[(i + 1) % cycle.len()];
        edges.insert((a.min(b), a.max(b)));
    }

    let mut found: IndexMap<(i32, i32), Vec<i32>> = IndexMap::new();
    for t in 0..mesh.triangle_count() {
        if !mesh.is_removed(t) {
            continue;
        }

        let face = mesh.triangles()[t as usize];
        for (u, w) in [(face.a, face.b), (face.b, face.c), (face.c, face.a)] {
            let key = (u.min(w), u.max(w));
            if edges.contains(&key) {
                found.entry(key).or_default().push(t);
            }
        }
    }

    found
}

/// One removed face near the hole: its centroid, unit normal and corners.
type NearFace = (Vector3, Vector3, Vector3, Vector3, Vector3);

/// A face plane as (origin, unit normal).
type FacePlane = (Vector3, Vector3);

/// C#'s `Comparer<double>.Default`: NaN sorts below every number.
fn cs_compare(a: f64, b: f64) -> std::cmp::Ordering {
    match (a.is_nan(), b.is_nan()) {
        (true, true) => std::cmp::Ordering::Equal,
        (true, false) => std::cmp::Ordering::Less,
        (false, true) => std::cmp::Ordering::Greater,
        _ => a.partial_cmp(&b).unwrap_or(std::cmp::Ordering::Equal),
    }
}

/// The added fill vertex (index `n` on) standing furthest through the faces the rows leave along -
/// in front of a convex row's face, behind a concave row's - past a tenth of the rows' mean width;
/// -1 for none, no rows, nothing added, or no removed face near the hole.
pub fn bulging_fill_vertex(
    mesh: &LevelLineMesh,
    fill: &[Vector3],
    n: i32,
    rows: &[CornerCapLastRow],
    points: &[Vector3],
) -> i32 {
    if rows.is_empty() || fill.len() as i32 == n {
        return -1;
    }

    let mean = points.iter().fold(Vector3::ZERO, |s, &p| s + p) / points.len() as f64;
    // C#'s `Enumerable.Max`: a NaN is displaced by any number.
    let reach = 2.0
        * points
            .iter()
            .map(|&p| (p - mean).length())
            .fold(
                f64::NAN,
                |best, x| {
                    if x > best || best.is_nan() {
                        x
                    } else {
                        best
                    }
                },
            );
    // Near the hole is measured to the triangle, not its centroid: a face that is one big facet has
    // its centroid millimetres off while the triangle itself runs under the arc's end - measured
    // by centroid it dropped out, the convex row's arc end on it read the neighbouring wall's plane
    // instead, and a sound fill was refused for standing 0.02 in front of a wall it never replaced.
    let mut planes: Vec<NearFace> = Vec::new();
    for t in 0..mesh.triangle_count() {
        if !mesh.is_removed(t) {
            continue;
        }

        let triangle = mesh.triangles()[t as usize];
        let a = mesh.position(triangle.a);
        let b = mesh.position(triangle.b);
        let c = mesh.position(triangle.c);
        let normal = (b - a).cross(c - a);
        if normal.length_squared() > 0.0
            && (ChainField::closest_on_triangle(mean, a, b, c) - mean).length() <= reach
        {
            planes.push(((a + b + c) / 3.0, normal.get_normal(), a, b, c));
        }
    }

    if planes.is_empty() {
        return -1;
    }

    // The face an arc end lies on: the removed triangle under it, measured to the triangle. Where
    // the end sits on a crease of the part, two faces are under it, and the one the round leaves
    // along is the face its first chord lies in - a fillet is tangent to that face. Ties within a
    // hundredth of the row's width are one place.
    let face_at = |row: &CornerCapLastRow, at_start: bool| -> FacePlane {
        let last = row.arc.len() - 1;
        let p = mesh.position(if at_start { row.arc[0] } else { row.arc[last] });
        let along = if row.arc.len() < 2 {
            Vector3::ZERO
        } else {
            (mesh.position(if at_start {
                row.arc[1]
            } else {
                row.arc[last - 1]
            }) - p)
                .get_normal()
        };
        let tie = 1e-2 * (mesh.position(row.arc[last]) - mesh.position(row.arc[0])).length();
        let under: Vec<(NearFace, f64)> = planes
            .iter()
            .map(|&plane| {
                (
                    plane,
                    (p - ChainField::closest_on_triangle(p, plane.2, plane.3, plane.4)).length(),
                )
            })
            .collect();
        // C#'s `Enumerable.Min`: any NaN makes the least NaN.
        let nearest = under.iter().map(|u| u.1).fold(f64::INFINITY, |least, x| {
            if x.is_nan() || least.is_nan() {
                f64::NAN
            } else {
                cs_math::min_f64(least, x)
            }
        });
        // OrderBy then ThenBy, stable: the first of the least (key1, key2) pair.
        let mut best: Option<(NearFace, f64, f64)> = None;
        for &(plane, off) in &under {
            if off > nearest + tie {
                continue;
            }

            let key1 = plane.1.dot(along).abs();
            let key2 = plane.1.dot(p - plane.0).abs();
            let better = match best {
                None => true,
                Some((_, b1, b2)) => cs_compare(key1, b1).then(cs_compare(key2, b2)).is_lt(),
            };
            if better {
                best = Some((plane, key1, key2));
            }
        }

        let face = best.expect("the nearest face is within its own tie").0;
        (face.0, face.1)
    };

    let faces: Vec<(&CornerCapLastRow, FacePlane, FacePlane)> = rows
        .iter()
        .map(|r| (r, face_at(r, true), face_at(r, false)))
        .collect();
    let allowance = 0.1
        * (rows
            .iter()
            .map(|r| (mesh.position(r.arc[r.arc.len() - 1]) - mesh.position(r.arc[0])).length())
            .sum::<f64>()
            / rows.len() as f64);
    let mixed = rows.iter().any(|r| r.concave) && rows.iter().any(|r| !r.concave);
    let mut worst = allowance;
    let mut at = -1;
    for (i, &point) in fill.iter().enumerate().skip(n as usize) {
        let mut through = if mixed { f64::MAX } else { f64::MIN };
        for &(row, a, b) in &faces {
            // in front of a face is positive; a convex row's fill must not stand in front, a
            // concave row's must not stand behind
            let sign = if row.concave { -1.0 } else { 1.0 };
            let by_row = cs_math::max_f64(sign * a.1.dot(point - a.0), sign * b.1.dot(point - b.0));
            through = if mixed {
                cs_math::min_f64(through, by_row)
            } else {
                cs_math::max_f64(through, by_row)
            };
        }

        if through > worst {
            worst = through;
            at = i as i32;
        }
    }

    at
}

/// C# `CornerCap.CreaseDegrees`: a loop edge that is a crease of the part - the live triangle on
/// it and the removed one across it turn by more than this - keeps its crease: the fill leaves
/// along the removed face, not the live one.
pub const CREASE_DEGREES: f64 = 20.0;

/// C# `CornerCap.LeavesThroughConvexCrease`: true when a fill leaving an arc end along the row's
/// cross-tangent `along`, signed toward the hole (`into_hole`, from the end toward the loop's
/// centroid), heads out through a face - its outward normal `outward` - by more than
/// [`CREASE_DEGREES`]. False where the chain runs square to the hole's side, since then neither
/// sign is the way in.
pub fn leaves_through_convex_crease(along: Vector3, into_hole: Vector3, outward: Vector3) -> bool {
    let toward = along.get_normal().dot(into_hole.get_normal());
    // The centroid sits well up out of the chain's line (G3's hole: 0.33 along it, the rest across
    // and up), so the sign is taken wherever the chain holds a tenth of the way in - past
    // round-off, short of a guess.
    if toward.abs() < 0.1 {
        return false;
    }

    let leaving = if toward < 0.0 { along * -1.0 } else { along };
    leaving.dot(outward) > (CREASE_DEGREES * std::f64::consts::PI / 180.0).sin()
}

/// C# `CornerCap.InsideTheCreases`: the cross-tangent at arc point `index` of a row of `count`
/// points, laid into the planes of the convex crease faces at its ends with a weight fading from 1
/// at the crease end to 0 at the far end - or at the row's middle where both ends have creases - so
/// the far end keeps its own cross-tangent, and its own face's plane, whatever the angle.
pub fn inside_the_creases(
    cross_tangent: Vector3,
    index: i32,
    count: i32,
    at_first: &[Vector3],
    at_last: &[Vector3],
) -> Vector3 {
    if count < 2 || (at_first.is_empty() && at_last.is_empty()) {
        return cross_tangent;
    }

    let from_first = index as f64 / (count - 1) as f64;
    let both_ends = !at_first.is_empty() && !at_last.is_empty();
    let first_weight = if at_first.is_empty() {
        0.0
    } else if both_ends {
        cs_math::max_f64(0.0, 1.0 - (2.0 * from_first))
    } else {
        1.0 - from_first
    };
    let last_weight = if at_last.is_empty() {
        0.0
    } else if both_ends {
        cs_math::max_f64(0.0, (2.0 * from_first) - 1.0)
    } else {
        from_first
    };
    let mut faded_tangent = cross_tangent;
    faded_tangent = faded(faded_tangent, at_first, first_weight);
    faded_tangent = faded(faded_tangent, at_last, last_weight);
    faded_tangent
}

/// C# `CornerCap.Faded`: `cross_tangent` moved `weight` of the way to its direction out of the
/// crease planes ([`out_of_the_planes`]), and renormalised; unchanged where there is none.
pub fn faded(cross_tangent: Vector3, creases: &[Vector3], weight: f64) -> Vector3 {
    if weight <= 0.0 || creases.is_empty() {
        return cross_tangent;
    }

    let inside = out_of_the_planes(cross_tangent, creases);
    if inside.length_squared() == 0.0 {
        cross_tangent
    } else {
        ((cross_tangent * (1.0 - weight)) + (inside * weight)).get_normal()
    }
}

/// C# `CornerCap.OutOfThePlanes`: the unit direction left of `cross_tangent` once every crease
/// normal's component is taken out JOINTLY - the normals orthonormalised first, so two faces at a
/// slant (n1.n2 = 0.5) leave a direction in both planes, their line n1 x n2, where removing them
/// one after the other would put the first back. Zero where the faces leave no direction (the
/// chain square to them).
pub fn out_of_the_planes(cross_tangent: Vector3, creases: &[Vector3]) -> Vector3 {
    let mut basis: Vec<Vector3> = Vec::new();
    for &normal in creases {
        let mut e = normal;
        for &b in &basis {
            e -= b * e.dot(b);
        }

        if e.length_squared() > 1e-12 {
            basis.push(e.get_normal());
        }
    }

    let mut inside = cross_tangent;
    for &b in &basis {
        inside -= b * cross_tangent.dot(b);
    }

    if inside.length_squared() < 1e-12 {
        Vector3::ZERO
    } else {
        inside.get_normal()
    }
}

/// C# `CornerCap.RunsForward`: true when the triangle runs its edge a to b in that direction.
pub fn runs_forward(face: LevelLineTriangle, a: i32, b: i32) -> bool {
    (face.a == a && face.b == b) || (face.b == a && face.c == b) || (face.c == a && face.a == b)
}

/// C# `CornerCap.OnTheBall`: the filler's patch laid on the rolling ball where the hole's whole rim
/// is one: every row a convex arc about the same centre at the same radius, and every loop point on
/// that sphere - three equal fillets meeting at a cube corner, whose true blend is the ball's
/// octant, tangent to each fillet along its end circle. The filler is fair, not a sphere (it held a
/// 20 mm cube's 2 mm corner 0.088 off the ball, six sagittas), so there its vertices are placed on
/// the ball along their own radial direction; the topology is the filler's. Three equal CONCAVE
/// fillets meeting at an inside corner are the same ball seen from inside
/// (HoleFillOneLoopCornerTests.TheThreeConcaveBeadsMeetOnTheBall). Any other hole - unequal radii,
/// a chamfer, a mixed corner, a face piece on the loop - is returned as filled.
#[allow(clippy::neg_cmp_op_on_partial_ord)]
pub fn on_the_ball(
    filled: HoleFillResult,
    points: &[Vector3],
    rows: &[CornerCapLastRow],
) -> HoleFillResult {
    if rows.is_empty() || rows.iter().any(|row| row.concave != rows[0].concave) {
        return filled;
    }

    let centre = rows[0].centre;
    let radius = rows[0].radius;

    // Relative to the coordinates' size as well as the radius: the rows' centres are sums of
    // float-rounded positions, so a corner far from the origin carries errors that grow with its
    // distance from it.
    let tolerance = 1e-6 * cs_math::max_f64(radius, centre.length());
    if !(radius > tolerance)
        || rows.iter().any(|row| {
            (row.centre - centre).length() > tolerance || (row.radius - radius).abs() > tolerance
        })
        || points
            .iter()
            .any(|&p| ((p - centre).length() - radius).abs() > tolerance)
    {
        return filled;
    }

    let n = points.len();
    let mut vertices = filled.vertices.clone();
    for vertex in vertices.iter_mut().skip(n) {
        let outward = *vertex - centre;

        // A fill vertex near the centre has no radial direction worth trusting; leave the hole as
        // filled.
        if outward.length() < radius / 2.0 {
            return filled;
        }

        *vertex = centre + (outward.get_normal() * radius);
    }

    if covers_the_ball_once(centre, &vertices, n as i32, &filled.triangles) {
        HoleFillResult::new(filled.failure, vertices, filled.triangles)
    } else {
        filled
    }
}

/// C# `CornerCap.CoversTheBallOnce`: whether a patch on a ball, its first `loop_count` vertices the
/// hole's loop, covers the loop's cap once: every triangle turns the same way seen from `centre` as
/// the loop does about its mean direction. A triangle turning the other way is a fold, which the
/// radial placement on the ball would keep (it never changes a triangle's turn seen from the
/// centre), so a folded patch is left as the filler laid it.
#[allow(clippy::neg_cmp_op_on_partial_ord)]
pub fn covers_the_ball_once(
    centre: Vector3,
    vertices: &[Vector3],
    loop_count: i32,
    triangles: &[HoleFillTriangle],
) -> bool {
    let n = loop_count as usize;
    let mut axis = Vector3::ZERO;
    for vertex in vertices.iter().take(n) {
        axis += (*vertex - centre).get_normal();
    }

    let mut loop_turn = 0.0;
    for i in 0..n {
        loop_turn += (vertices[i] - centre)
            .cross(vertices[(i + 1) % n] - centre)
            .dot(axis);
    }

    if loop_turn == 0.0 || loop_turn.is_nan() {
        return false;
    }

    // C#'s Math.Sign: -1, 0 or 1 as an int (loop_turn is neither 0 nor NaN here).
    let sign = if loop_turn > 0.0 { 1.0 } else { -1.0 };
    for t in triangles {
        let a = vertices[t.a as usize] - centre;
        let turn = a
            .cross(vertices[t.b as usize] - centre)
            .dot(vertices[t.c as usize] - centre);
        if !(turn * sign > 0.0) {
            return false;
        }
    }

    true
}
