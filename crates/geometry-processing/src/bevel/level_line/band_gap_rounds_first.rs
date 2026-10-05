//! The band built rounds first. Ports `Bevel/LevelLine/BandGap.RoundsFirst.cs` (a partial of the C#
//! static class `BandGap`): so far the kept triangles as index triples ([`kept`]), the concave
//! chain's surviving piece ([`surviving_piece`]), the crease walk on the end face
//! ([`crease_on_the_end_face`]) and a triangle's oriented plane ([`plane_of`], [`same_plane`]); the
//! build lands with later packages.

use std::collections::{HashMap, HashSet};

use vector_math::{cs_math, Vector3};

use super::end_face_sense::COPLANAR_TOLERANCE;
use super::LevelLineMesh;

/// The mesh's live triangles as index triples, in triangle order.
pub fn kept(mesh: &LevelLineMesh) -> Vec<(i32, i32, i32)> {
    (0..mesh.triangle_count())
        .filter(|&t| !mesh.is_removed(t))
        .map(|t| {
            let face = mesh.triangles()[t as usize];
            (face.a, face.b, face.c)
        })
        .collect()
}

/// C# `BandGap.SurvivingPiece`: a concave chain oriented to end at its knife tip, carried onto the
/// rounds' mesh: from its far end, through any vertices the rounds' cut laid on its edges, to the
/// vertex where it enters the rounds' band (the band took the tip). `None` where the piece cannot
/// be walked, or the band never reached the chain. A vertex missing from `neighbours` (where C#'s
/// indexer throws) is a walk that cannot be made, as in [`super::band_gap_in_turn::walk_along`].
pub fn surviving_piece(
    to_tip: &[i32],
    at: &[Vector3],
    is_crease: &dyn Fn(i32, i32) -> bool,
    neighbours: &HashMap<i32, Vec<i32>>,
) -> Option<Vec<i32>> {
    if !neighbours.contains_key(&to_tip[0]) {
        return None;
    }

    // Stepped along each edge's straight line from vertex to nearest vertex ahead, while the step
    // is still crease: the tip itself may stay (it lies in the end face the round grew), so the
    // piece ends where the crease leaves the line - the rail's entry - not where a vertex is
    // missing.
    let mut piece = vec![to_tip[0]];
    let mut i = 0;
    while i + 1 < to_tip.len() {
        let start = at[to_tip[i] as usize];
        let along = at[to_tip[i + 1] as usize] - start;
        let length = along.length();
        let direction = along / length;
        let on_the_line = 1e-7 * length;
        let mut here = piece[piece.len() - 1];
        let mut reached = (at[here as usize] - start).dot(direction);
        while here != to_tip[i + 1] {
            let mut next = -1;
            let mut next_reach = f64::INFINITY;
            for &n in neighbours.get(&here)? {
                let offset = at[n as usize] - start;
                let t = offset.dot(direction);
                if t > reached + on_the_line
                    && t <= length + on_the_line
                    && t < next_reach
                    && (offset - direction * t).length() <= on_the_line
                {
                    next = n;
                    next_reach = t;
                }
            }

            if next < 0 || !is_crease(here, next) {
                return if piece.len() > 1 { Some(piece) } else { None };
            }

            piece.push(next);
            here = next;
            reached = next_reach;
        }

        i += 1;
    }

    // The whole chain is still crease up to the tip: the round never reached it.
    None
}

/// C# `BandGap.CreaseOnTheEndFace`: the crease from `start` to `end` along edges where a triangle
/// on the end face meets one off it, leaving `start` away from `came_from`: the vertices after
/// `start`, ending with `end`. `None` where the walk forks or stops short (or reaches a vertex
/// missing from `neighbours`, where C#'s indexer throws).
pub fn crease_on_the_end_face(
    start: i32,
    came_from: i32,
    end: i32,
    is_crease: &dyn Fn(i32, i32) -> bool,
    neighbours: &HashMap<i32, Vec<i32>>,
) -> Option<Vec<i32>> {
    let mut walk = Vec::new();
    let mut visited: HashSet<i32> = HashSet::from([start, came_from]);
    let mut here = start;
    while here != end {
        let next: Vec<i32> = neighbours
            .get(&here)?
            .iter()
            .copied()
            .filter(|&n| (n == end || !visited.contains(&n)) && is_crease(here, n))
            .collect();
        if next.len() != 1 {
            return None;
        }

        here = next[0];
        visited.insert(here);
        walk.push(here);
    }

    Some(walk)
}

/// A triangle's oriented plane: its unit normal, the normal's offset and the centroid (C#'s
/// `(Vector3 Normal, double Offset, Vector3 Point)`).
pub type OrientedPlane = (Vector3, f64, Vector3);

/// C# `BandGap.PlaneOf`: the plane of triangle `t` over the positions `at`, through its centroid.
pub fn plane_of(at: &[Vector3], t: (i32, i32, i32)) -> OrientedPlane {
    let (a, b, c) = (at[t.0 as usize], at[t.1 as usize], at[t.2 as usize]);
    let normal = (b - a).cross(c - a).get_normal();
    let centroid = (a + b + c) / 3.0;
    (normal, normal.dot(centroid), centroid)
}

/// C# `BandGap.SamePlane`: the same oriented plane, to the catalogue's float noise
/// ([`COPLANAR_TOLERANCE`]): the facings agree and `q`'s centroid stands on `p`. Not the two
/// offsets: a triangle tilted by float noise 115 from the origin moves its offset by the tilt times
/// that distance - a wall triangle of the rounds' run-out tilted 2.5e-5 read 1.2e-3 off the wall,
/// was taken for a facet of the round, and the joined crease's side flooded over the wall
/// (RoundsFirstTests.TheWallStaysOffTheCreasesSide).
pub fn same_plane(p: OrientedPlane, q: OrientedPlane) -> bool {
    1.0 - p.0.dot(q.0) < COPLANAR_TOLERANCE
        && (p.0.dot(q.2) - p.1).abs() <= COPLANAR_TOLERANCE * cs_math::max_f64(1.0, p.1.abs())
}
