//! Port of `Agg.PolygonMesh/RevolveTests.cs`: Revolve turns a 2D profile into a solid, and
//! every primitive that spins a profile around an axis reaches the same code. The profile can be
//! degenerate while a user is mid-edit, and it may or may not have material on the negative side
//! of the axis.

use agg_path::VertexStorage;
use polygon_mesh::processors::vertex_source_to_mesh::revolve;
use polygon_mesh::Mesh;
use vector_math::math_helper::TAU;
use vector_math::Vector3;

/// A rectangular profile from (left, 0) to (right, height), authored the way the ring and
/// cylinder primitives author theirs - four line segments, left implicitly closed.
fn rectangle_profile(left: f64, right: f64, height: f64) -> VertexStorage {
    let mut path = VertexStorage::new();
    path.move_to(left, 0.0);
    path.line_to(right, 0.0);
    path.line_to(right, height);
    path.line_to(left, height);

    path
}

/// The same rectangle authored the other way around - up the left edge first, so the profile
/// winds clockwise.
fn clockwise_rectangle_profile(left: f64, right: f64, height: f64) -> VertexStorage {
    let mut path = VertexStorage::new();
    path.move_to(left, 0.0);
    path.line_to(left, height);
    path.line_to(right, height);
    path.line_to(right, 0.0);

    path
}

/// C#'s default sweep: a full turn around z.
fn revolve_full(profile: &VertexStorage, angle_steps: i32) -> Mesh {
    revolve(profile, angle_steps, 0.0, TAU, true).expect("angles in range")
}

/// The signed volume of a closed mesh, positive when its faces wind outward: the tetrahedron
/// each triangle forms with the origin, summed.
fn signed_volume(mesh: &Mesh) -> f64 {
    let mut total = 0.0;

    for face in mesh.faces.iter() {
        let a = Vector3::from_vector3_float(mesh.vertices[face.v0 as usize]);
        let b = Vector3::from_vector3_float(mesh.vertices[face.v1 as usize]);
        let c = Vector3::from_vector3_float(mesh.vertices[face.v2 as usize]);

        total += a.dot(b.cross(c)) / 6.0;
    }

    total
}

fn assert_within(actual: f64, expected: f64, tolerance: f64, because: &str) {
    assert!(
        (actual - expected).abs() <= tolerance,
        "expected {expected} within {tolerance}, got {actual}: {because}"
    );
}

#[test]
fn a_zero_area_profile_revolves_to_an_empty_mesh() {
    // A zero height profile encloses no area, so it converts to no clipper polygons at all;
    // nothing to revolve is an empty solid, not a crash.
    let mesh = revolve_full(&rectangle_profile(5.0, 15.0, 0.0), 30);

    assert_eq!(mesh.faces.len(), 0);
    assert_eq!(mesh.vertices.len(), 0);
}

#[test]
fn a_profile_right_of_the_axis_revolves_unchanged() {
    // The counts are the ones the C#'s pre-fix code produced for this profile.
    let mesh = revolve_full(&rectangle_profile(5.0, 15.0, 10.0), 30);

    assert_eq!(mesh.faces.len(), 240);
    assert_eq!(mesh.vertices.len(), 120);

    let bounds = mesh.get_axis_aligned_bounding_box();
    assert_within(bounds.x_size(), 30.0, 0.001, "outer diameter");
    assert_within(bounds.z_size(), 10.0, 0.001, "height");
}

#[test]
fn a_clockwise_profile_right_of_the_axis_revolves_right_side_out() {
    // Revolve is what makes profile winding not matter to its callers.
    let clockwise = revolve_full(&clockwise_rectangle_profile(10.0, 20.0, 5.0), 30);
    let counter_clockwise = revolve_full(&rectangle_profile(10.0, 20.0, 5.0), 30);

    assert!(
        signed_volume(&counter_clockwise) > 0.0,
        "a counter clockwise profile has always revolved right side out"
    );
    assert_within(
        signed_volume(&clockwise),
        signed_volume(&counter_clockwise),
        0.001,
        "Revolve normalizes profile winding, so both orders make the same solid",
    );
}

#[test]
fn a_partial_revolve_welds_its_end_caps_to_the_wall() {
    // CleanAndMerge welds on exact float equality, so a cap corner computed any other way than
    // the wall's stays its own vertex and leaves the solid open along both end seams.
    const ANGLE_STEPS: i32 = 8;
    let mesh = revolve(
        &rectangle_profile(5.0, 15.0, 10.0),
        ANGLE_STEPS,
        0.0,
        TAU / 4.0,
        true,
    )
    .expect("angles in range");

    // four profile corners at each of the nine ring angles and nothing else
    assert_eq!(mesh.vertices.len(), 4 * (ANGLE_STEPS as usize + 1));
    assert!(
        mesh.is_manifold(),
        "a partial revolve is a closed solid, so every edge has exactly two faces"
    );

    // The faceted quarter wedge is eight prisms of chord width:
    // 8 * height * sin(step) * (R^2 - r^2) / 2.
    let step_angle = TAU / 4.0 / f64::from(ANGLE_STEPS);
    let expected_volume =
        f64::from(ANGLE_STEPS) * 10.0 * step_angle.sin() * ((15.0 * 15.0) - (5.0 * 5.0)) / 2.0;
    assert_within(
        signed_volume(&mesh),
        expected_volume,
        0.001,
        "both caps close the wedge, facing outward",
    );
}

#[test]
fn a_full_revolve_welds_its_wrap_around_seam() {
    // A full turn has no end caps, and the closing strip ends on the exact angle the first
    // strip started from, so its seam welds.
    const ANGLE_STEPS: i32 = 8;
    let mesh = revolve_full(&rectangle_profile(5.0, 15.0, 10.0), ANGLE_STEPS);

    assert_eq!(mesh.vertices.len(), 4 * ANGLE_STEPS as usize);
    assert!(mesh.is_manifold(), "a full revolve closes on itself");
}

#[test]
fn a_profile_left_of_the_axis_is_mirrored_onto_the_right() {
    // The same rectangle authored on the negative side must revolve into the same solid.
    let mirrored = revolve_full(&rectangle_profile(-15.0, -5.0, 10.0), 30);
    let direct = revolve_full(&rectangle_profile(5.0, 15.0, 10.0), 30);

    assert_eq!(mirrored.faces.len(), direct.faces.len());

    let mirrored_bounds = mirrored.get_axis_aligned_bounding_box();
    let direct_bounds = direct.get_axis_aligned_bounding_box();
    assert_within(mirrored_bounds.x_size(), direct_bounds.x_size(), 0.001, "");
    assert_within(mirrored_bounds.z_size(), direct_bounds.z_size(), 0.001, "");
}
