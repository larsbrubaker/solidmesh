//! A zero area face is topologically real - the seams where a solid touches itself are made of
//! them - so the mesh has to be able to hold one without its normal turning into NaN.

use solidmesh_math::Vector3Float;
use solidmesh_mesh::Face;

#[test]
fn zero_area_face_gets_a_finite_normal() {
    let vertices = vec![
        Vector3Float::new(0.0, 0.0, 0.0),
        Vector3Float::new(10.0, 0.0, 0.0),
        // A third corner on the line through the first two: three distinct vertices, no area.
        Vector3Float::new(5.0, 0.0, 0.0),
        // And a fourth at a position already used, which is what a self-touch seam looks like.
        Vector3Float::new(10.0, 0.0, 0.0),
    ];

    for face in [Face::new(0, 1, 2, &vertices), Face::new(0, 1, 3, &vertices)] {
        assert!(
            !(face.normal.x.is_nan() || face.normal.y.is_nan() || face.normal.z.is_nan()),
            "a NaN normal compares false against every tolerance, so it silently breaks vertex merging and coplanar walks"
        );
        assert_eq!(
            face.normal,
            Vector3Float::ZERO,
            "a face with no area has no direction to point in"
        );
    }
}

#[test]
fn face_with_area_still_gets_a_unit_normal() {
    let vertices = vec![
        Vector3Float::new(0.0, 0.0, 0.0),
        Vector3Float::new(10.0, 0.0, 0.0),
        Vector3Float::new(0.0, 10.0, 0.0),
    ];

    let face = Face::new(0, 1, 2, &vertices);
    assert_eq!(
        face.normal,
        Vector3Float::new(0.0, 0.0, 1.0),
        "guarding the degenerate case must not change the normal of a real face"
    );
}
