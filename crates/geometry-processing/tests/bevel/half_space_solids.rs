//! Convex solids described the way a chamfer is, as the walls that keep a region rather than as a
//! tessellation. A list of half spaces goes in and the polytope they bound comes out, either as a
//! mesh to bevel or as a volume to measure a bevel against. `skew_ridge_solid.rs` builds its
//! fixture with it.
//!
//! **Built by brute force, which is what keeps it honest.** Every triple of walls is solved for its
//! point and kept when every other wall keeps it too - the definition of a vertex of an
//! intersection of half spaces, with no hull algorithm in between to have a bug of its own. Each
//! wall's own vertices are then turned about its normal and fanned.

use polygon_mesh::Mesh;
use vector_math::Vector3;

use crate::mesh_measurements::signed_volume;

/// How near a point has to come to a wall to be standing on it, over parts tens of mm across.
const ON_THE_WALL: f64 = 1e-7;

/// One half space: the material is where `facing . x <= offset`.
#[derive(Clone, Copy, Debug)]
pub struct Wall {
    pub facing: Vector3,
    pub offset: f64,
}

/// The wall through `at` whose outward normal is `facing`.
pub fn through(at: Vector3, facing: Vector3) -> Wall {
    let normal = facing.get_normal();

    Wall {
        facing: normal,
        offset: normal.dot(at),
    }
}

/// The chamfer wall for the edge along `direction` through `on` between the two faces, set back
/// `size` along each of them: the plane through the two points the bevel's tangent lines stand at.
/// Chamfering every edge of a CONVEX solid is exactly intersecting it with one of these per edge,
/// so the polytope of the walls plus the chamfer walls is the answer a bevel test measures against.
pub fn chamfer(on: Vector3, direction: Vector3, first: Wall, second: Wall, size: f64) -> Wall {
    let along = direction.get_normal();
    let touches_first = on + (into_the_face(along, first, second) * size);
    let touches_second = on + (into_the_face(along, second, first) * size);

    let mut facing = (touches_second - touches_first).cross(along);

    // Out of the material, which is the way both of the faces it is set back from look.
    if facing.dot(first.facing + second.facing) < 0.0 {
        facing = -facing;
    }

    through(touches_first, facing)
}

/// The unit step from a point of the edge into `face`, square to the edge: the direction the
/// setback is measured in. Of the two ways along the face it is the one that leaves `other`'s wall
/// behind, which is what makes it the inward one.
fn into_the_face(along: Vector3, face: Wall, other: Wall) -> Vector3 {
    let across = along.cross(face.facing).get_normal();

    if across.dot(other.facing) > 0.0 {
        -across
    } else {
        across
    }
}

/// The polytope the walls bound, as a welded mesh with one fan per wall.
pub fn build(walls: &[Wall]) -> Mesh {
    let corners = corners(walls);
    let mut mesh = Mesh::new();

    for wall in walls {
        let on: Vec<Vector3> = corners
            .iter()
            .copied()
            .filter(|&at| (wall.facing.dot(at) - wall.offset).abs() < 1e-6)
            .collect();

        if on.len() < 3 {
            continue;
        }

        for (a, b, c) in turned(&on, wall.facing) {
            mesh.create_face(&[a, b, c]);
        }
    }

    // create_face adds fresh vertices every call, so what is built above is triangle soup - the
    // same weld TopologyTestMeshes does for the same reason.
    mesh.clean_and_merge();

    mesh
}

/// The volume the walls keep between them, read off the polytope they bound.
pub fn volume(walls: &[Wall]) -> f64 {
    signed_volume(&build(walls))
}

/// Every vertex of the polytope: the point of each triple of walls that every other wall also
/// keeps, each once.
fn corners(walls: &[Wall]) -> Vec<Vector3> {
    let mut found: Vec<Vector3> = Vec::new();

    for first in 0..walls.len() {
        for second in (first + 1)..walls.len() {
            for third in (second + 1)..walls.len() {
                let Some(at) = try_meet(walls[first], walls[second], walls[third]) else {
                    continue;
                };

                if walls
                    .iter()
                    .any(|wall| wall.facing.dot(at) - wall.offset > 1e-6)
                {
                    continue;
                }

                if !found.iter().any(|&already| (already - at).length() < 1e-6) {
                    found.push(at);
                }
            }
        }
    }

    found
}

// Written as a negated `>` so a NaN determinant is refused as well.
#[allow(clippy::neg_cmp_op_on_partial_ord)]
/// The one point on three walls, by Cramer, or nothing for three that share a direction.
fn try_meet(first: Wall, second: Wall, third: Wall) -> Option<Vector3> {
    let determinant = first.facing.dot(second.facing.cross(third.facing));

    if !(determinant.abs() > 1e-9) {
        return None;
    }

    Some(
        ((second.facing.cross(third.facing) * first.offset)
            + (third.facing.cross(first.facing) * second.offset)
            + (first.facing.cross(second.facing) * third.offset))
            / determinant,
    )
}

/// One wall's corners turned into triangles: sorted about the wall's own normal, which orders a
/// convex ring, then fanned from the first of them and wound to face out.
fn turned(on: &[Vector3], facing: Vector3) -> Vec<(Vector3, Vector3, Vector3)> {
    let mut middle = Vector3::ZERO;

    for &at in on {
        middle += at;
    }

    middle /= on.len() as f64;

    let across = (on[0] - middle).get_normal();
    let up = facing.cross(across);

    // A stable sort, so ties keep their order.
    let mut ordered = on.to_vec();
    ordered.sort_by(|a, b| {
        let angle = |at: &Vector3| ((*at - middle).dot(up)).atan2((*at - middle).dot(across));
        angle(a).total_cmp(&angle(b))
    });

    let mut fan = Vec::new();

    for i in 1..ordered.len().saturating_sub(1) {
        let normal = (ordered[i] - ordered[0]).cross(ordered[i + 1] - ordered[0]);

        if normal.length() < ON_THE_WALL {
            continue;
        }

        fan.push(if normal.dot(facing) > 0.0 {
            (ordered[0], ordered[i], ordered[i + 1])
        } else {
            (ordered[0], ordered[i + 1], ordered[i])
        });
    }

    fan
}
