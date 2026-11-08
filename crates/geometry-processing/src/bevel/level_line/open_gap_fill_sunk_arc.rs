//! The open fill's sunk arc: a triangle's doubled area ([`area2`]), the planes of a side's
//! removed triangles ([`planes`]), whether a spoke stands past an end ([`past_an_end`]) and whether
//! a step turns into the left side's nearest plane ([`concave`]), and the first sunk arc point
//! ([`sunk_arc_point`]).

use solidmesh_math::Vector3;

use super::{ChainField, GapLoop, LevelLineMesh, LevelLineTriangle, OpenGapFillOpenRail};

/// Twice a triangle's area.
pub fn area2(mesh: &LevelLineMesh, t: LevelLineTriangle) -> f64 {
    (mesh.position(t.b) - mesh.position(t.a))
        .cross(mesh.position(t.c) - mesh.position(t.a))
        .length()
}

/// The planes - centroid and unit normal - of the side's REMOVED triangles with area, in triangle
/// order: the faces the band took out of the side, which the arc is read against.
pub fn planes(mesh: &LevelLineMesh, side: i32) -> Vec<(Vector3, Vector3)> {
    let mut planes = Vec::new();
    for t in 0..mesh.triangle_count() {
        let triangle = mesh.triangles()[t as usize];
        if !mesh.is_removed(t) || triangle.tag != side {
            continue;
        }

        let a = mesh.position(triangle.a);
        let normal = (mesh.position(triangle.b) - a).cross(mesh.position(triangle.c) - a);
        if normal.length() > 0.0 {
            planes.push((
                (a + mesh.position(triangle.b) + mesh.position(triangle.c)) / 3.0,
                normal.get_normal(),
            ));
        }
    }

    planes
}

/// True when a spoke's mean extended parameter stands past either end of the chain.
pub fn past_an_end(
    chain: &ChainField,
    left: &OpenGapFillOpenRail,
    right: &OpenGapFillOpenRail,
    spoke: (i32, i32),
) -> bool {
    let mean = (left.parameters[spoke.0 as usize] + right.parameters[spoke.1 as usize]) / 2.0;
    mean < 0.0 || mean > chain.length
}

/// True when the step from `p` to `q` runs into the left side's plane nearest `p` (the first of a
/// tie); false with no planes.
pub fn concave(left_planes: &[(Vector3, Vector3)], p: Vector3, q: Vector3) -> bool {
    let mut nearest: Option<((Vector3, Vector3), f64)> = None;
    for &plane in left_planes {
        let reach = (plane.0 - p).length_squared();
        if nearest.is_none_or(|(_, best)| reach < best) {
            nearest = Some((plane, reach));
        }
    }

    nearest.is_some_and(|((_, normal), _)| normal.dot(q - p) > 0.0)
}

/// The first arc point of a CONCAVE chain's strip that stands inside the solid -
/// behind every plane of the triangles the band removed on one side - or -1. Exact for a flat side
/// and a convex curved one; on a concave curved side it can only miss, never refuse wrongly. It
/// catches an arc whose rail stands far round a convex boss (at 0.9 of the room at a radius-5
/// boss's foot the fill removed 119.8 where the fillet adds).
///
/// Concave is read PER SPOKE, from the left side only, and a spoke past a chain end
/// (`past_an_end`) is skipped. A point may stand a chord's sag behind the facets: the allowance is
/// a tenth of the spoke's own chord.
pub fn sunk_arc_point(
    mesh: &LevelLineMesh,
    left_loop: &GapLoop,
    right_loop: &GapLoop,
    arcs: &[Vec<i32>],
    past_an_end: &[bool],
) -> i32 {
    if arcs.iter().all(|arc| arc.len() <= 2) {
        return -1;
    }

    let sides = [planes(mesh, left_loop.side), planes(mesh, right_loop.side)];
    let mut deepest = 0.0;
    let mut at = -1;

    for (row, arc) in arcs.iter().enumerate() {
        if past_an_end[row] {
            continue;
        }

        // An empty spoke has no first or last point and is skipped.
        let (Some(&first), Some(&last)) = (arc.first(), arc.last()) else {
            continue;
        };

        if first == last || !concave(&sides[0], mesh.position(first), mesh.position(last)) {
            continue;
        }

        let allowance = 0.1 * (mesh.position(last) - mesh.position(first)).length();

        for &vertex in &arc[1..arc.len().saturating_sub(1).max(1)] {
            let point = mesh.position(vertex);

            for side_planes in &sides {
                if side_planes.is_empty() {
                    continue;
                }

                // How far behind the LEAST-behind plane: positive only when behind every one.
                // A NaN depth wins, so a broken plane is never read as behind.
                let depth = side_planes
                    .iter()
                    .map(|&(origin, normal)| normal.dot(origin - point))
                    .fold(f64::INFINITY, |least, d| {
                        if d < least || d.is_nan() {
                            d
                        } else {
                            least
                        }
                    });

                if depth > allowance && depth > deepest {
                    deepest = depth;
                    at = vertex;
                }
            }
        }
    }

    at
}
