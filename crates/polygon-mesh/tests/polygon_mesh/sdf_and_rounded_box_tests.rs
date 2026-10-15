// The distance functions' shapes past the cylinder's rim and inside near its caps, empty
// combinations, a rounded box whose radius is larger than half its smallest side, and the
// plane shape's debug text.

use polygon_mesh::dual_contouring::{Cylinder, Intersection, Sdf, Subtraction, Union};
use polygon_mesh::ray_tracer::PlaneShape;
use polygon_mesh::solids::RoundedCornerBox;
use vector_math::{AxisAlignedBoundingBox, Plane, Vector3};

#[test]
fn cylinder_distance_is_measured_to_the_nearest_surface() {
    let cylinder = Cylinder::new(2.0, 0.5);
    // Past the cap and outside the radius: the nearest point is on the rim.
    assert_eq!(cylinder.sdf(Vector3::new(1.5, 0.0, 2.0)), 2f64.sqrt());
    // Inside, nearer the cap than the side.
    assert!((cylinder.sdf(Vector3::new(0.0, 0.0, 0.9)) - -0.1).abs() < 1e-12);
    // Inside, nearer the side than the caps.
    assert!((cylinder.sdf(Vector3::new(0.4, 0.0, 0.0)) - -0.1).abs() < 1e-12);
}

#[test]
fn empty_combinations_are_empty_rather_than_panicking() {
    let p = Vector3::new(1.0, 2.0, 3.0);
    let union = Union::default();
    assert_eq!(union.sdf(p), f64::INFINITY);
    assert_eq!(union.bounds(), AxisAlignedBoundingBox::empty());
    let intersection = Intersection::default();
    assert_eq!(intersection.sdf(p), f64::INFINITY);
    assert_eq!(intersection.bounds(), AxisAlignedBoundingBox::empty());
    let subtraction = Subtraction::default();
    assert_eq!(subtraction.sdf(p), f64::INFINITY);
    assert_eq!(subtraction.bounds(), AxisAlignedBoundingBox::empty());
}

#[test]
fn rounded_box_with_an_oversized_radius_stays_inside_its_size() {
    let size = Vector3::new(10.0, 10.0, 2.0);
    let mesh = RoundedCornerBox::create(4, size, 5.0);
    let bounds = mesh.get_axis_aligned_bounding_box();
    assert!(
        bounds.equals_within(&AxisAlignedBoundingBox::new(-size / 2.0, size / 2.0), 1e-9),
        "{bounds:?}"
    );
}

#[test]
fn plane_shape_text_names_a_plane() {
    let shape = PlaneShape::new(Plane::new(Vector3::new(0.0, 0.0, 1.0), 2.5));
    assert_eq!(shape.to_string(), "Plane (0x+0y+1z=2.5)");
}
