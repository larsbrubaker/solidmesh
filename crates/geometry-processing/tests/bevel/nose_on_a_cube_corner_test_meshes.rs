//! Port of the test fixture `Fixtures/NoseOnACubeCornerTestMeshes.cs`: a 20 mm cube with a
//! knife-edge NOSE standing proud of its front face, placed so that the band on the top front edge
//! runs UNDER the nose's root and the extension has to run off through the band's end at the cube's
//! own corner (20, 0, 20) - where three convex edges meet.
//!
//! The same standing feature `nose_in_notch_test_meshes` carries, moved to a corner three chains can
//! gather at. That is the whole point of it: the nose fixture's notch runs off at (10, 10, 20), where
//! one convex edge meets a CONCAVE crease, so no corner PATCH can ever stand there and the case a
//! patch would cap over cannot be asked of that part.
//!
//! **The shape.** The cube, plus the tetrahedron on `y < 0` whose ridge runs from
//! [`ridge_on_the_face`] = (10, 0, 10) out to [`ridge_at_the_end`] = (20, -10, 10), and whose two
//! walls come back to the face at [`upper_crease_end`] = (20, 0, 13.333) and [`lower_crease_end`] =
//! (20, 0, 6.667). The two creases are concave, both end in the plane x = 20 - which is the band's
//! own end plane - and both lie within 13.334 of the top, so a fillet of 15 on the top front edge
//! undercuts the whole of the nose's root.

#![allow(dead_code)]

use polygon_mesh::Mesh;
use vector_math::Vector3;

/// The cube's edge.
pub const SIZE: f64 = 20.0;

/// Where the nose's ridge stands on the front face - the end of both creases inside the face.
pub fn ridge_on_the_face() -> Vector3 {
    Vector3::new(10.0, 0.0, 10.0)
}

/// Where the ridge runs out, proud of the face and in the band's end plane.
pub fn ridge_at_the_end() -> Vector3 {
    Vector3::new(SIZE, -10.0, 10.0)
}

/// Where the upper crease dies, on the cube's own front-right vertical edge.
pub fn upper_crease_end() -> Vector3 {
    Vector3::new(SIZE, 0.0, 40.0 / 3.0)
}

/// Where the lower crease dies.
pub fn lower_crease_end() -> Vector3 {
    Vector3::new(SIZE, 0.0, 20.0 / 3.0)
}

/// The corner the band ends at, where three convex edges of the cube meet.
pub fn the_corner() -> Vector3 {
    Vector3::new(SIZE, 0.0, SIZE)
}

/// The corner the band starts at.
pub fn the_tip() -> Vector3 {
    Vector3::new(0.0, 0.0, SIZE)
}

/// The solid, hand triangulated - every polygon emitted is one its own outline can be seen whole
/// from, and every vertex standing on another polygon's rim is a corner of that polygon too, so the
/// mesh has no T junction in it.
pub fn create() -> Mesh {
    let mut mesh = Mesh::new();
    let v = Vector3::new;

    face(
        &mut mesh,
        v(0.0, 0.0, 1.0),
        &[
            v(0.0, 0.0, SIZE),
            v(SIZE, 0.0, SIZE),
            v(SIZE, SIZE, SIZE),
            v(0.0, SIZE, SIZE),
        ],
    );
    face(
        &mut mesh,
        v(0.0, 0.0, -1.0),
        &[
            v(0.0, 0.0, 0.0),
            v(SIZE, 0.0, 0.0),
            v(SIZE, SIZE, 0.0),
            v(0.0, SIZE, 0.0),
        ],
    );

    face(
        &mut mesh,
        v(0.0, 1.0, 0.0),
        &[
            v(0.0, SIZE, 0.0),
            v(SIZE, SIZE, 0.0),
            v(SIZE, SIZE, SIZE),
            v(0.0, SIZE, SIZE),
        ],
    );
    face(
        &mut mesh,
        v(-1.0, 0.0, 0.0),
        &[
            v(0.0, 0.0, 0.0),
            v(0.0, SIZE, 0.0),
            v(0.0, SIZE, SIZE),
            v(0.0, 0.0, SIZE),
        ],
    );

    // The right face, x = 20: the square PLUS the triangle the nose adds to it, which is coplanar
    // with it. The square's own rim carries the two crease ends, so the two pieces share them.
    face(
        &mut mesh,
        v(1.0, 0.0, 0.0),
        &[
            v(SIZE, SIZE, SIZE),
            v(SIZE, 0.0, SIZE),
            upper_crease_end(),
            lower_crease_end(),
            v(SIZE, 0.0, 0.0),
            v(SIZE, SIZE, 0.0),
        ],
    );
    face(
        &mut mesh,
        v(1.0, 0.0, 0.0),
        &[ridge_at_the_end(), lower_crease_end(), upper_crease_end()],
    );

    // The front face, y = 0, less the triangle the nose's root stands on: two pieces either side of
    // the ridge's own foot, each fanned from the far corner that sees the whole of it past the notch
    // that root takes out of the face. Nothing is added to the face's RIM, so the top front edge -
    // the one that is beveled - stays the single edge of the cube it always was.
    face(
        &mut mesh,
        v(0.0, -1.0, 0.0),
        &[
            v(0.0, 0.0, 0.0),
            v(SIZE, 0.0, 0.0),
            lower_crease_end(),
            ridge_on_the_face(),
        ],
    );
    face(
        &mut mesh,
        v(0.0, -1.0, 0.0),
        &[
            v(0.0, 0.0, SIZE),
            v(0.0, 0.0, 0.0),
            ridge_on_the_face(),
            upper_crease_end(),
            v(SIZE, 0.0, SIZE),
        ],
    );

    // And the nose's own two walls.
    face(
        &mut mesh,
        v(0.0, -1.0, 1.0),
        &[ridge_on_the_face(), ridge_at_the_end(), upper_crease_end()],
    );
    face(
        &mut mesh,
        v(0.0, -1.0, -1.0),
        &[ridge_on_the_face(), lower_crease_end(), ridge_at_the_end()],
    );

    mesh.clean_and_merge();

    mesh
}

/// One polygon, fanned from the corner it is LISTED first from and wound to the way it is to face.
/// Written rather than trusted to the order the corners are given in: the mesh fans every polygon
/// from its first vertex, so the apex has to see the whole of its own outline, and a face laid the
/// other way about is an inside-out solid with every measurement on it the wrong sign.
fn face(mesh: &mut Mesh, facing: Vector3, corners: &[Vector3]) {
    let mut ring = corners.to_vec();
    let mut normal = Vector3::ZERO;

    for at in 1..ring.len() - 1 {
        normal += (ring[at] - ring[0]).cross(ring[at + 1] - ring[0]);
    }

    if normal.dot(facing) < 0.0 {
        // Turned about the apex, which stays at the head of the list - reversing the whole of it
        // would fan from some other corner, which is not the one measured to see the outline whole.
        ring[1..].reverse();
    }

    mesh.create_face(&ring);
}
