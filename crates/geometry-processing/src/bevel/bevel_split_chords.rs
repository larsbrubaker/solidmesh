//! Split chords: a vertex a boolean left standing part way along a straight crease, reading as a
//! corner of nothing. What is here is the mesh's own lattice ([`lattice_for`]) and the
//! straight-line test ([`stands_on_the_line_between`]) the sections' room reading and split blend
//! share.

use solidmesh_math::{ieee_math, Vector3};
use solidmesh_mesh::Mesh;

/// The step between neighbouring floats at magnitude one, which is how a mesh stores a point.
const FLOAT_STORAGE_EPSILON: f64 = 1.1920929e-7;

/// How many of those steps still count as one straight line. Eight rather than one because a vertex
/// a boolean wrote has been through several roundings on its way here, and rather than thirty-two
/// because the smallest turn these rims really make is thousands of steps wide.
const LATTICE_FACTOR: f64 = 8.0;

/// The mesh's own lattice - `LATTICE_FACTOR` float steps at its furthest coordinate - or zero for
/// a mesh with no extent (or an infinite one).
#[allow(clippy::neg_cmp_op_on_partial_ord)]
pub fn lattice_for(mesh: &Mesh) -> f64 {
    let bounds = mesh.get_axis_aligned_bounding_box();

    let furthest = ieee_math::max_f64(
        ieee_math::max_f64(bounds.min_xyz.x.abs(), bounds.max_xyz.x.abs()),
        ieee_math::max_f64(
            ieee_math::max_f64(bounds.min_xyz.y.abs(), bounds.max_xyz.y.abs()),
            ieee_math::max_f64(bounds.min_xyz.z.abs(), bounds.max_xyz.z.abs()),
        ),
    );

    if !(furthest > 0.0) || furthest.is_infinite() {
        return 0.0;
    }

    furthest * FLOAT_STORAGE_EPSILON * LATTICE_FACTOR
}

/// Whether `point` stands on the straight line from `from` to `to`, within the lattice, and BETWEEN
/// them - a chord a boolean split carries its extra vertex INSIDE the chord, and one standing past
/// an end would make the merged edge longer than either piece.
#[allow(clippy::neg_cmp_op_on_partial_ord)]
pub fn stands_on_the_line_between(
    point: Vector3,
    from: Vector3,
    to: Vector3,
    lattice: f64,
) -> bool {
    let along = to - from;
    let length = along.length();

    if !(length > 0.0) {
        return false;
    }

    along.cross(point - from).length() / length <= lattice && (point - from).dot(point - to) < 0.0
}
