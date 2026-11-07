//! Whether the end
//! face HOLDS a chamfer's section or stands BESIDE it
//! (`surface_end_gains_the_section::how_the_face_stands_to_it`), read off what of the face the
//! section's chords take - so a far lobe of the same coplanar patch inside the wedge extended counts
//! for nothing.

use geometry_processing::bevel::surface::{
    surface_end_gains_the_section, surface_end_notch, SurfaceEndGainsTheSectionReading,
    SurfaceVertexPool,
};
use indexmap::IndexSet;
use vector_math::{Vector3, Vector3Float};

const REACH: f64 = 4.0;

fn the_corner() -> Vector3 {
    Vector3::ZERO
}

fn the_end_faces_normal() -> Vector3 {
    Vector3::new(0.0, 0.0, 1.0)
}

#[test]
fn a_face_beside_the_section_with_a_far_lobe_inside_the_wedge_still_gains_it() {
    let (mut pool, mut faces, profile) = the_section();

    let in_the_end = added(
        &mut pool,
        &mut faces,
        &[
            // Beside it: the third quadrant, touching the corner and nothing else of the wedge.
            (
                Vector3::new(0.0, 0.0, 0.0),
                Vector3::new(0.0, -20.0, 0.0),
                Vector3::new(-20.0, -20.0, 0.0),
            ),
            (
                Vector3::new(0.0, 0.0, 0.0),
                Vector3::new(-20.0, -20.0, 0.0),
                Vector3::new(-20.0, 0.0, 0.0),
            ),
            // And a lobe of the same coplanar patch 30 mm out, square inside the wedge extended.
            (
                Vector3::new(30.0, 30.0, 0.0),
                Vector3::new(40.0, 30.0, 0.0),
                Vector3::new(40.0, 40.0, 0.0),
            ),
            (
                Vector3::new(30.0, 30.0, 0.0),
                Vector3::new(40.0, 40.0, 0.0),
                Vector3::new(30.0, 40.0, 0.0),
            ),
        ],
    );

    let sides = surface_end_notch::the_sides_of_the_section(
        &pool,
        &profile,
        the_corner(),
        the_end_faces_normal(),
    );

    assert_eq!(
        sides.len(),
        2,
        "the chamfer's two straight sides are the two axes, which name a wedge"
    );

    let reading = surface_end_gains_the_section::how_the_face_stands_to_it(
        &pool,
        &faces,
        &in_the_end,
        &profile,
        the_corner(),
        the_end_faces_normal(),
        &sides,
    );

    assert_eq!(
        reading,
        SurfaceEndGainsTheSectionReading::TheFaceIsBesideIt,
        "the far lobe covers none of the section, which reaches {REACH} mm - instead {reading:?}"
    );
}

#[test]
fn a_face_the_section_stands_in_holds_it() {
    let (mut pool, mut faces, profile) = the_section();

    let in_the_end = added(
        &mut pool,
        &mut faces,
        &[
            (
                Vector3::new(0.0, 0.0, 0.0),
                Vector3::new(20.0, 0.0, 0.0),
                Vector3::new(20.0, 20.0, 0.0),
            ),
            (
                Vector3::new(0.0, 0.0, 0.0),
                Vector3::new(20.0, 20.0, 0.0),
                Vector3::new(0.0, 20.0, 0.0),
            ),
        ],
    );

    let sides = surface_end_notch::the_sides_of_the_section(
        &pool,
        &profile,
        the_corner(),
        the_end_faces_normal(),
    );

    let reading = surface_end_gains_the_section::how_the_face_stands_to_it(
        &pool,
        &faces,
        &in_the_end,
        &profile,
        the_corner(),
        the_end_faces_normal(),
        &sides,
    );

    assert_eq!(
        reading,
        SurfaceEndGainsTheSectionReading::TheFaceHoldsIt,
        "the whole section stands in the face, which is what the cut takes off - instead {reading:?}"
    );
}

#[test]
fn a_face_covering_only_half_the_section_still_holds_it() {
    let (mut pool, mut faces, profile) = the_section();

    let in_the_end = added(
        &mut pool,
        &mut faces,
        &[
            // The half of the first quadrant below y = x.
            (
                Vector3::new(0.0, 0.0, 0.0),
                Vector3::new(20.0, 0.0, 0.0),
                Vector3::new(20.0, 20.0, 0.0),
            ),
        ],
    );

    let sides = surface_end_notch::the_sides_of_the_section(
        &pool,
        &profile,
        the_corner(),
        the_end_faces_normal(),
    );

    let reading = surface_end_gains_the_section::how_the_face_stands_to_it(
        &pool,
        &faces,
        &in_the_end,
        &profile,
        the_corner(),
        the_end_faces_normal(),
        &sides,
    );

    assert_eq!(
        reading,
        SurfaceEndGainsTheSectionReading::TheFaceHoldsIt,
        "half the section is the face's, which is the cut's own business - instead {reading:?}"
    );
}

/// A pool seeded with the corners of the 20 mm square the faces are drawn in (the weld is read off
/// what it already holds), and the chamfer's two-point profile.
fn the_section() -> (SurfaceVertexPool, Vec<(i32, i32, i32)>, Vec<i32>) {
    let mut pool = SurfaceVertexPool::new(&[
        Vector3Float::new(-20.0, -20.0, 0.0),
        Vector3Float::new(20.0, 20.0, 0.0),
    ]);

    let profile = vec![
        pool.add(Vector3::new(REACH, 0.0, 0.0)),
        pool.add(Vector3::new(0.0, REACH, 0.0)),
    ];

    (pool, Vec::new(), profile)
}

fn added(
    pool: &mut SurfaceVertexPool,
    faces: &mut Vec<(i32, i32, i32)>,
    triangles: &[(Vector3, Vector3, Vector3)],
) -> IndexSet<i32> {
    let mut in_the_end = IndexSet::new();

    for &(a, b, c) in triangles {
        in_the_end.insert(faces.len() as i32);
        faces.push((pool.add(a), pool.add(b), pool.add(c)));
    }

    in_the_end
}
