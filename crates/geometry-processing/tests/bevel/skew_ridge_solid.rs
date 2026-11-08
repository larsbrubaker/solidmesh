//! A ROOF with two skew ends - one ridge running along x, the
//! two roof faces falling away from it, and an end face at either end leaning so that the three
//! faces meeting at each end of the ridge do so at wedges of the caller's choosing. It stands on a
//! flat bottom, and every one of its five faces is a plane, so `half_space_solids` both builds it
//! and states what bevelling it comes to.
//!
//! **What it is for.** At an OBLIQUE corner (wedges around 112, 142 and 146 degrees) the chamfer
//! planes' meeting point stands past the chord between the two rail ends, and the bent end is
//! reflex; one strip is mitred at both ends. **The two ends are mirrors with their wedges swapped
//! over**, which puts the LONG rail of the ridge strip on one face at one end and on the other face
//! at the other end.

use solidmesh_math::{math_helper, Vector3};
use solidmesh_mesh::Mesh;

use crate::half_space_solids::{self, Wall};

/// The wedge between the two roof faces, along the ridge.
pub const RIDGE_WEDGE_DEGREES: f64 = 142.0;

/// The wedge between an end face and the roof face it leans toward.
pub const NEAR_WEDGE_DEGREES: f64 = 112.0;

/// The wedge between an end face and the roof face it leans away from.
pub const FAR_WEDGE_DEGREES: f64 = 146.0;

/// How far the bottom stands below the ridge.
pub const DEPTH: f64 = 12.0;

/// The ridge's own end at +x, where three faces meet at the skew wedges.
pub fn near_corner(ridge_length: f64) -> Vector3 {
    Vector3::new(ridge_length / 2.0, 0.0, 0.0)
}

/// The ridge's own end at -x.
pub fn far_corner(ridge_length: f64) -> Vector3 {
    Vector3::new(-ridge_length / 2.0, 0.0, 0.0)
}

/// The five walls of the solid: two roof faces, two end faces and the bottom.
pub fn walls(ridge_length: f64) -> Vec<Wall> {
    let half = math_helper::degrees_to_radians(180.0 - RIDGE_WEDGE_DEGREES) / 2.0;
    let left = Vector3::new(0.0, half.sin(), half.cos());
    let right = Vector3::new(0.0, -half.sin(), half.cos());

    vec![
        half_space_solids::through(Vector3::ZERO, left),
        half_space_solids::through(Vector3::ZERO, right),
        half_space_solids::through(
            near_corner(ridge_length),
            leaning(left, right, NEAR_WEDGE_DEGREES, FAR_WEDGE_DEGREES, 1.0),
        ),
        half_space_solids::through(
            far_corner(ridge_length),
            leaning(left, right, FAR_WEDGE_DEGREES, NEAR_WEDGE_DEGREES, -1.0),
        ),
        half_space_solids::through(Vector3::new(0.0, 0.0, -DEPTH), Vector3::new(0.0, 0.0, -1.0)),
    ]
}

/// The solid itself.
pub fn create(ridge_length: f64) -> Mesh {
    half_space_solids::build(&walls(ridge_length))
}

/// The five edges a bevel of this fixture is asked for: the ridge, and the two edges running down
/// from each of its ends - every edge that touches either end of the ridge.
pub fn is_selected(from: Vector3, to: Vector3, ridge_length: f64) -> bool {
    touches(from, ridge_length) || touches(to, ridge_length)
}

fn touches(at: Vector3, ridge_length: f64) -> bool {
    (at - near_corner(ridge_length)).length() < 1e-5
        || (at - far_corner(ridge_length)).length() < 1e-5
}

/// The unit normal of an end face: the one standing at the asked-for wedge from each roof face, on
/// the `side` end of the ridge. The roof normals have no x of their own, so the two wedges fix the
/// normal's y and z outright and the x is what is left of a unit vector.
fn leaning(
    left: Vector3,
    right: Vector3,
    to_the_left_degrees: f64,
    to_the_right_degrees: f64,
    side: f64,
) -> Vector3 {
    let to_left = math_helper::degrees_to_radians(180.0 - to_the_left_degrees).cos();
    let to_right = math_helper::degrees_to_radians(180.0 - to_the_right_degrees).cos();

    let determinant = (left.y * right.z) - (left.z * right.y);
    let y = ((to_left * right.z) - (to_right * left.z)) / determinant;
    let z = ((left.y * to_right) - (right.y * to_left)) / determinant;
    let square = 1.0 - (y * y) - (z * z);

    assert!(
        square > 0.0,
        "no face stands at both of those wedges from the two roof faces"
    );

    Vector3::new(side * square.sqrt(), y, z)
}
