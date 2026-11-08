//! One level of refinement of a corner patch's spherical polygon that leaves its BOUNDARY exactly
//! where it was: every interior edge is split at its midpoint on the sphere, every boundary edge is
//! untouched, and the triangles are re-cut to suit. The surface corner's rows
//! (`SurfaceCornerPatchRows`) refine with it.
//!
//! **Why the interior alone.** The boundary arcs of that polygon are the SEAM
//! ([`super::bevel_seam_ring`]): the runs ending at the corner carry those very points, so moving
//! one, or adding one between two of them, is two solids meeting along two different polylines -
//! which is a union the kernel refuses. The interior is nobody else's business.
//!
//! **What it buys, measured.** The fan is a chorded sampling of the ball, so it sags inside the
//! sphere - and the sag grows with the SQUARE of the step, which is why a sharp corner suffers
//! most: the 45 degree knife tip of "Bevel to Center Feature.mcx" measured 0.055 above the fan
//! against a tolerance of 0.05; refined, 0.036, no worse than the runs' own chording.
//!
//! **What it does NOT buy.** Not a quartering: the band along the boundary keeps its coarse chords,
//! because those chords are the seam, and the fan's skinny triangles near its ribs stay skinny. If
//! the corner ever needs to be rounder than this, the answer is a better-shaped fan rather than
//! another level of midpoints. One level, not a loop: it costs very nearly four times the patch's
//! triangles (192 on the ball becomes 744 at eight segments over three sectors).
//!
//! The kept-edge set and the midpoint memo are only looked up, never iterated, so a `HashSet` /
//! `HashMap` order cannot reach the output; the midpoints are appended in triangle order.

use std::collections::{HashMap, HashSet};

use vector_math::Vector3;

/// The refined triangle list, with the midpoints appended to `directions` as unit vectors - so the
/// caller's own indices, and every direction it already had, keep their meaning.
///
/// `boundary` is the polygon's boundary walks - the seam arcs - whose edges are the ones that must
/// not be split; `None` or empty refines everything, which is what a caller with no seam to keep
/// means.
pub fn refine(
    directions: &mut Vec<Vector3>,
    patch: &[(i32, i32, i32)],
    boundary: Option<&[Vec<i32>]>,
) -> Vec<(i32, i32, i32)> {
    let mut kept = HashSet::new();

    if let Some(boundary) = boundary {
        for walk in boundary {
            let mut step = 0;
            while step + 1 < walk.len() {
                kept.insert(edge(walk[step], walk[step + 1]));
                step += 1;
            }
        }
    }

    let mut midpoints: HashMap<(i32, i32), i32> = HashMap::new();
    let mut refined = Vec::with_capacity(4 * patch.len());

    let mut split = |directions: &mut Vec<Vector3>, from: i32, to: i32| -> i32 {
        let edge = edge(from, to);

        if kept.contains(&edge) {
            return -1;
        }

        if let Some(&midpoint) = midpoints.get(&edge) {
            return midpoint;
        }

        // The midpoint ON THE SPHERE rather than on the chord - the sum of two unit vectors bisects
        // the angle between them, so normalizing it is the great-circle midpoint without an arc's
        // worth of trigonometry, and it is exactly what a slerp at a half would give.
        let midpoint_direction = (directions[from as usize] + directions[to as usize]).get_normal();
        directions.push(midpoint_direction);

        let midpoint = directions.len() as i32 - 1;
        midpoints.insert(edge, midpoint);

        midpoint
    };

    for &(first, second, third) in patch {
        let (mut a, mut b, mut c) = (first, second, third);

        let mut ab = split(directions, a, b);
        let mut bc = split(directions, b, c);
        let mut ca = split(directions, c, a);

        // Rotated so the cases below only ever have to be written once: with one split it is AB, and
        // with two they are AB and BC. A rotation of a triangle's corners is the same triangle wound
        // the same way, so nothing about the surface depends on which way round it came in.
        while (ab < 0 && (bc >= 0 || ca >= 0)) || (ab >= 0 && bc < 0 && ca >= 0) {
            (a, b, c, ab, bc, ca) = (b, c, a, bc, ca, ab);
        }

        if ab < 0 {
            // Nothing to split: a triangle all of whose edges are the seam, which only happens where
            // the polygon is a single band.
            refined.push((a, b, c));
        } else if bc < 0 {
            refined.push((a, ab, c));
            refined.push((ab, b, c));
        } else if ca < 0 {
            // Two split edges and a boundary one: the corner between them is cut off, and the quad
            // that is left is cut by the diagonal from A - the boundary edge CA is not touched either
            // way, which is the whole point.
            refined.push((a, ab, bc));
            refined.push((ab, b, bc));
            refined.push((a, bc, c));
        } else {
            refined.push((a, ab, ca));
            refined.push((ab, b, bc));
            refined.push((ca, bc, c));
            refined.push((ab, bc, ca));
        }
    }

    refined
}

/// An edge as a key, read the same way from either end.
fn edge(from: i32, to: i32) -> (i32, i32) {
    if from < to {
        (from, to)
    } else {
        (to, from)
    }
}
