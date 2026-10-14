//! A plateau or a ridge has a whole LINE of equally greatest value, so "the greatest point" is only
//! a statement about the part if the search names the same one of them wherever the part stands.

use std::f64::consts::PI;

use geometry_processing::bevel::level_line::band_gap_entry_points::{self, BandGapBuildOptions};
use geometry_processing::bevel::level_line::{BandGapRefusal, BandProfile, ChainField};
use vector_math::{Matrix4X4, Vector3};

use crate::level_line_feature_edges::feature_edges_over;

/// Five proper rotations with translations - no reflections, so the part is not
/// changed.
fn motions() -> Vec<Matrix4X4> {
    vec![
        Matrix4X4::create_rotation(Vector3::new(0.3, -0.7, 1.1))
            * Matrix4X4::create_translation(11.5, -3.25, 7.125),
        Matrix4X4::create_rotation(Vector3::new(-1.2, 0.4, 0.9))
            * Matrix4X4::create_translation(-4.5, 8.25, -2.5),
        Matrix4X4::create_rotation(Vector3::new(0.0, 0.0, PI / 2.0))
            * Matrix4X4::create_translation(100.0, 0.0, 0.0),
        Matrix4X4::create_rotation(Vector3::new(PI / 3.0, 0.0, 0.0))
            * Matrix4X4::create_translation(0.5, 0.25, -0.125),
        Matrix4X4::create_translation(-1000.0, 1000.0, 250.0),
    ]
}

fn field(chain: &[Vector3], setbacks: &[f64]) -> ChainField {
    ChainField::from_polyline(chain, false, setbacks).expect("a valid polyline")
}

/// The field of a straight chain
/// has a RIDGE parallel to it; the greatest point must be the same point of the part in every
/// placement.
#[test]
fn the_greatest_point_on_a_plateau_is_the_same_wherever_the_part_stands() {
    // A straight chain along y, and a triangle straddling it: the field rises with |x|, so the
    // whole segment x = 2 of the triangle is equally greatest - a ridge, with no single maximiser
    // of its own.
    let chain = [Vector3::new(0.0, -20.0, 0.0), Vector3::new(0.0, 20.0, 0.0)];
    let setbacks = [0.5, 0.5];
    let a = Vector3::new(2.0, -10.0, 0.0);
    let b = Vector3::new(2.0, 10.0, 0.0);
    let c = Vector3::new(0.0, 0.0, 0.0);

    let still = field(&chain, &setbacks)
        .max_over_triangle(a, b, c, 0.05)
        .expect("no budget in force");

    for motion in motions() {
        let moved_chain: Vec<Vector3> = chain.iter().map(|p| p.transform(motion)).collect();
        let moved = field(&moved_chain, &setbacks)
            .max_over_triangle(
                a.transform(motion),
                b.transform(motion),
                c.transform(motion),
                0.05,
            )
            .expect("no budget in force");

        let back = moved.point.transform(motion.inverted());
        assert!((moved.value - still.value).abs() < 1e-9);
        assert!(
            (back - still.point).length() < 1e-9,
            "the ridge's greatest point is {:?} still and {back:?} moved back",
            still.point
        );
    }
}

/// The same claim in one
/// dimension - a segment lying ALONG the ridge is equally greatest everywhere.
#[test]
fn the_greatest_point_on_a_ridge_segment_is_the_same_wherever_the_part_stands() {
    let chain = [Vector3::new(0.0, -20.0, 0.0), Vector3::new(0.0, 20.0, 0.0)];
    let setbacks = [0.5, 0.5];
    let a = Vector3::new(2.0, -6.0, 0.0);
    let b = Vector3::new(2.0, 9.0, 0.0);

    let still = field(&chain, &setbacks)
        .max_along_segment(a, b, 0.05)
        .expect("no budget in force");

    for motion in motions() {
        let moved_chain: Vec<Vector3> = chain.iter().map(|p| p.transform(motion)).collect();
        let moved = field(&moved_chain, &setbacks)
            .max_along_segment(a.transform(motion), b.transform(motion), 0.05)
            .expect("no budget in force");

        let back = moved.point.transform(motion.inverted());
        assert!((moved.value - still.value).abs() < 1e-9);
        assert!(
            (back - still.point).length() < 1e-9,
            "the ridge segment's greatest point is {:?} still and {back:?} moved back",
            still.point
        );
    }
}

/// A two-triangle strip whose spine is
/// a ridge is cut the same way in every placement - same triangle and vertex counts.
#[test]
fn a_strip_whose_spine_is_a_ridge_is_cut_the_same_wherever_it_stands() {
    let positions = vec![
        Vector3::new(-2.0, -10.0, -2.0),
        Vector3::new(2.0, -10.0, -2.0),
        Vector3::new(2.0, 10.0, -2.0),
        Vector3::new(-2.0, 10.0, -2.0),
        Vector3::new(-2.0, -10.0, 2.0),
        Vector3::new(2.0, -10.0, 2.0),
        Vector3::new(2.0, 10.0, 2.0),
        Vector3::new(-2.0, 10.0, 2.0),
    ];
    let indices = vec![
        (0, 2, 1),
        (0, 3, 2),
        (4, 5, 6),
        (4, 6, 7),
        (0, 1, 5),
        (0, 5, 4),
        (1, 2, 6),
        (1, 6, 5),
        (2, 3, 7),
        (2, 7, 6),
        (3, 0, 4),
        (3, 4, 7),
    ];
    let rim = [4, 5, 6, 7];

    let still = cut(&positions, &indices, &rim);

    for motion in motions() {
        let moved_positions: Vec<Vector3> = positions.iter().map(|p| p.transform(motion)).collect();
        let moved = cut(&moved_positions, &indices, &rim);

        assert_eq!(moved.0, still.0);
        assert_eq!(
            moved.1, still.1,
            "the cut of a ridge may not depend on where the part stands"
        );
        assert_eq!(moved.2, still.2);
    }
}

/// The refusal and, where it built, the mesh's vertex and triangle counts.
fn cut(
    positions: &[Vector3],
    indices: &[(i32, i32, i32)],
    rim: &[i32],
) -> (BandGapRefusal, i32, usize) {
    let gap = band_gap_entry_points::build(
        positions,
        indices,
        rim,
        true,
        Some(feature_edges_over(positions, indices, 20.0)),
        0.8,
        BandProfile::Chamfer,
        &BandGapBuildOptions::default(),
    )
    .expect("the pick builds or refuses by name");

    if gap.built() {
        let mesh = gap.mesh.as_ref().expect("a built mesh").borrow();
        (gap.refusal, mesh.vertex_count(), mesh.triangles().len())
    } else {
        (gap.refusal, 0, 0)
    }
}
