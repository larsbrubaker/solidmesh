//! Ports `MatterCADTests/Standard/BevelVoidWedgeSectionTests.cs`: the two things the void wedge
//! has to be, held against each other at every dihedral the builder accepts. It has to CONTAIN the
//! part of the bead that lies in the void, or the clip would trim the bead it exists to protect;
//! and it has to stay no wider than the bead's own section, or it gets refused where the bead
//! sweeps fine (an over-large wedge fails silently: a refused wedge is a bead added unclipped).
//! The test lives with the kernel it exercises (`bevel_edge_tool_builder`) rather than with the app
//! tests, since it touches nothing else. C# `[MethodDataSource(Corners)]` rows are a loop.

use geometry_processing::bevel::bevel_edge_tool_builder as builder;
use geometry_processing::topology::EdgeConvexity;
use vector_math::math_helper::degrees_to_radians;
use vector_math::{Vector2, Vector3};

/// A right angle, a sharp corner and a flat one - all well inside the builder's interior angle
/// bounds, and covering both sides of the point where `cross_section_reach` stops being
/// `1 / sin(halfAngle)` and becomes the flat `sqrt(2)`.
fn corners() -> Vec<(f64, bool)> {
    let mut corners = Vec::new();
    for round_profile in [true, false] {
        corners.push((90.0, round_profile));
        corners.push((30.0, round_profile));
        corners.push((160.0, round_profile));
    }
    corners
}

#[test]
fn the_wedge_is_no_wider_than_the_section_it_clips() {
    const SIZE: f64 = 3.0;

    for (wedge_angle_degrees, round_profile) in corners() {
        let (wedge, tangent_distance) = build_wedge(wedge_angle_degrees, round_profile, SIZE);

        // The bead's own furthest reach from the edge, which is what BevelChainSweep's fold test
        // is effectively measuring when it decides whether a section fits the curvature.
        let bead_reach = builder::cross_section_reach(SIZE, wedge_angle_degrees);
        let mut wedge_reach = 0.0_f64;

        for point in &wedge {
            wedge_reach = wedge_reach.max(point.length());
        }

        assert!(
            wedge_reach <= builder::CONTAINMENT_MARGIN * bead_reach,
            "a wedge wider than the bead is refused on curvature the bead follows, and a refused \
             wedge leaks - at {wedge_angle_degrees} degrees the wedge reaches {wedge_reach:.4} \
             against the section's own {bead_reach:.4}"
        );

        // The other half of the pincer: the tangent points are the furthest the void side of the
        // bead goes, and the wedge has to be outside them.
        assert!(
            wedge_reach > tangent_distance,
            "the wedge has to reach past the bead's tangent points at {tangent_distance:.4} to \
             contain the arc between them"
        );

        // And the closed form of that same setback, which BevelChainEnds reads before anything
        // has been swept: the one number two pieces of the pipeline compute independently.
        let closed_form = builder::tangent_distance(SIZE, wedge_angle_degrees, round_profile);
        assert!(
            (closed_form - tangent_distance).abs() <= 1e-9,
            "the setback derived from the wedge angle is the setback the section was actually \
             built with ({closed_form} against {tangent_distance} at {wedge_angle_degrees})"
        );
    }
}

#[test]
fn the_wedge_contains_every_part_of_the_bead_that_lies_in_the_void() {
    const SIZE: f64 = 3.0;
    const SEGMENTS: i32 = 16;

    for (wedge_angle_degrees, round_profile) in corners() {
        let (wedge, _) = build_wedge(wedge_angle_degrees, round_profile, SIZE);
        let (profile, face0_out, face1_out) =
            build_profile(wedge_angle_degrees, round_profile, SIZE, SEGMENTS);

        // Which of the tool's points are in the VOID, asked the way the two faces answer it: the
        // profile normals point into the material on a concave edge, so a point in the void is
        // one that is behind both of them. That excludes the clearance corners - the buried back,
        // which trimming to the wall is the entire purpose of the clip.
        let mut checked_any = false;

        for point in &profile {
            if point.dot(face0_out) > 1e-9 || point.dot(face1_out) > 1e-9 {
                continue;
            }

            checked_any = true;

            assert!(
                is_inside(&wedge, *point),
                "the bead's ({:.4}, {:.4}) lies in the void at {wedge_angle_degrees} degrees, so \
                 clipping to the wedge would cut the bead itself",
                point.x,
                point.y
            );
        }

        assert!(
            checked_any,
            "the bead has a void side at every dihedral - a run that checked nothing is a broken \
             test"
        );
    }
}

/// The wedge section for a concave corner whose VOID opens by `wedge_angle_degrees`, built the way
/// `BevelChainSweep` builds it: real face normals through `try_get_profile_normals` and the frame
/// that follows from them. The bare triangle (walls on the faces), because what these measure is
/// the construction, and the inset would move every point.
fn build_wedge(wedge_angle_degrees: f64, round_profile: bool, size: f64) -> (Vec<Vector2>, f64) {
    let frame = build_frame(wedge_angle_degrees);

    builder::try_build_void_wedge_cross_section(
        frame.normal0,
        frame.normal1,
        frame.x_axis,
        frame.y_axis,
        frame.z_axis,
        size,
        round_profile,
        false,
    )
    .unwrap_or_else(|reason| {
        panic!("the wedge at {wedge_angle_degrees} degrees would not build: {reason}")
    })
}

/// The bead's own cross section at the same corner, and its two profile normals in the frame.
fn build_profile(
    wedge_angle_degrees: f64,
    round_profile: bool,
    size: f64,
    segments: i32,
) -> (Vec<Vector2>, Vector2, Vector2) {
    let frame = build_frame(wedge_angle_degrees);

    let (points, _) = builder::try_build_cross_section(
        frame.normal0,
        frame.normal1,
        frame.x_axis,
        frame.y_axis,
        frame.z_axis,
        size,
        segments,
        round_profile,
        0.0,
    )
    .unwrap_or_else(|reason| {
        panic!("the tool at {wedge_angle_degrees} degrees would not build: {reason}")
    });

    (points, frame.face0_out, frame.face1_out)
}

struct Frame {
    normal0: Vector3,
    normal1: Vector3,
    x_axis: Vector3,
    y_axis: Vector3,
    z_axis: Vector3,
    face0_out: Vector2,
    face1_out: Vector2,
}

/// An inside corner running along z whose void opens by the given angle about +x: two faces whose
/// own outward normals point into that void, put through the concave sign flip and the frame
/// builder exactly as a swept segment does. `face0_out`/`face1_out` are the profile normals in the
/// 2D frame, the way the bead's clearance wall runs out past each face.
fn build_frame(wedge_angle_degrees: f64) -> Frame {
    let half = degrees_to_radians(wedge_angle_degrees) / 2.0;

    // Each face runs away from the edge along a[i] and its outward normal is a[i] turned square
    // toward the void, which is the +x side.
    let face_normal0 = Vector3::new(half.sin(), -half.cos(), 0.0);
    let face_normal1 = Vector3::new(half.sin(), half.cos(), 0.0);

    let (normal0, normal1) =
        builder::try_get_profile_normals(EdgeConvexity::Concave, face_normal0, face_normal1)
            .unwrap_or_else(|reason| panic!("{reason}"));

    let (x_axis, y_axis, z_axis) =
        builder::try_build_frame(normal0, normal1, Vector3::new(0.0, 0.0, 1.0))
            .unwrap_or_else(|reason| panic!("{reason}"));

    Frame {
        normal0,
        normal1,
        x_axis,
        y_axis,
        z_axis,
        face0_out: Vector2::new(normal0.dot(x_axis), normal0.dot(y_axis)),
        face1_out: Vector2::new(normal1.dot(x_axis), normal1.dot(y_axis)),
    }
}

/// Whether a point is inside the wedge polygon, which is convex and may be wound either way.
fn is_inside(polygon: &[Vector2], point: Vector2) -> bool {
    let mut saw_positive = false;
    let mut saw_negative = false;

    for i in 0..polygon.len() {
        let edge = polygon[(i + 1) % polygon.len()] - polygon[i];
        let to_point = point - polygon[i];
        let side = (edge.x * to_point.y) - (edge.y * to_point.x);

        // On an edge counts as inside: the bead is tangent to both faces, so its two tangent
        // points sit exactly on the wedge's own sides by construction.
        if side > 1e-9 {
            saw_positive = true;
        } else if side < -1e-9 {
            saw_negative = true;
        }
    }

    !(saw_positive && saw_negative)
}
