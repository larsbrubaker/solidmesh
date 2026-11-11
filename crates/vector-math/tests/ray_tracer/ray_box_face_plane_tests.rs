// A ray whose direction is exactly 0 on one axis and whose origin lies exactly in one of the box's
// faces on that axis: it runs along the face, touches the box and must hit it, for the boolean
// `Ray::intersection` and for `Ray::get_closest_intersection`, on x, y and z alike. A ray parallel
// to an axis with its origin strictly outside that slab must still miss, and a ray whose
// direction is NaN still misses everything. A zero-thickness box the ray lies in is touched, so
// hit, and a `BvhTree` pick along a face plane finds the item it runs along.

use solidmesh_math::bvh::{BvhTreeItemData, TradeOffBvhConstructor};
use solidmesh_math::{AxisAlignedBoundingBox, IntersectionType, Ray, Vector3};

fn unit_box() -> AxisAlignedBoundingBox {
    AxisAlignedBoundingBox::new(Vector3::new(0.0, 0.0, 0.0), Vector3::new(1.0, 1.0, 1.0))
}

fn axis_vector(axis: usize, value: f64) -> Vector3 {
    let mut v = Vector3::new(0.0, 0.0, 0.0);
    v[axis] = value;
    v
}

/// A ray in the plane `plane_axis = face` (0 or 1), started 2 units before the box on
/// `travel_axis` and pointing along it; the third axis sits mid-box. Every component of the
/// direction other than `travel_axis` is exactly 0.
fn ray_in_face(plane_axis: usize, face: f64, travel_axis: usize) -> Ray {
    let mut origin = Vector3::new(0.5, 0.5, 0.5);
    origin[plane_axis] = face;
    origin[travel_axis] = -2.0;
    Ray::new(origin, axis_vector(travel_axis, 1.0))
}

fn assert_touching_hit(plane_axis: usize, face: f64, travel_axis: usize) {
    let bounds = unit_box();
    let ray = ray_in_face(plane_axis, face, travel_axis);
    let case = format!("plane axis {plane_axis} at {face}, travelling along {travel_axis}");

    assert!(ray.intersection(&bounds), "intersection missed: {case}");

    let (hit, found) = ray.intersect(&bounds);
    assert!(hit, "intersect missed: {case}");
    assert_eq!(found.min_dist_found, 2.0, "entry distance: {case}");
    assert_eq!(found.max_dist_found, 3.0, "exit distance: {case}");

    let info = ray.get_closest_intersection(&bounds);
    assert_eq!(
        info.hit_type,
        IntersectionType::FrontFace,
        "closest hit: {case}"
    );
    assert_eq!(info.distance_to_hit, 2.0, "closest distance: {case}");
    assert_eq!(
        info.closest_hit_object,
        Some(bounds),
        "closest object: {case}"
    );
    let mut expected_position = ray.origin;
    expected_position[travel_axis] = 0.0;
    assert_eq!(
        info.hit_position, expected_position,
        "closest position: {case}"
    );
    assert_eq!(
        info.normal_at_hit,
        axis_vector(travel_axis, -1.0),
        "closest normal: {case}"
    );
}

#[test]
fn ray_in_the_min_x_face_hits() {
    assert_touching_hit(0, 0.0, 1);
    assert_touching_hit(0, 0.0, 2);
}

#[test]
fn ray_in_the_max_x_face_hits() {
    assert_touching_hit(0, 1.0, 1);
    assert_touching_hit(0, 1.0, 2);
}

#[test]
fn ray_in_a_y_face_hits() {
    for face in [0.0, 1.0] {
        assert_touching_hit(1, face, 0);
        assert_touching_hit(1, face, 2);
    }
}

#[test]
fn ray_in_a_z_face_hits() {
    for face in [0.0, 1.0] {
        assert_touching_hit(2, face, 0);
        assert_touching_hit(2, face, 1);
    }
}

#[test]
fn ray_in_a_face_plane_with_negative_zero_direction_hits() {
    // -0.0 makes the reciprocal -infinity and flips which face the slab test enters through.
    let bounds = unit_box();
    for face in [0.0, 1.0] {
        let ray = Ray::new(Vector3::new(face, -2.0, 0.5), Vector3::new(-0.0, 1.0, 0.0));
        assert!(
            ray.intersection(&bounds),
            "intersection missed at x = {face}"
        );
        let (hit, found) = ray.intersect(&bounds);
        assert!(hit, "intersect missed at x = {face}");
        assert_eq!(found.min_dist_found, 2.0, "entry distance at x = {face}");
        assert_eq!(found.max_dist_found, 3.0, "exit distance at x = {face}");
        let info = ray.get_closest_intersection(&bounds);
        assert_eq!(info.hit_type, IntersectionType::FrontFace, "x = {face}");
        assert_eq!(info.distance_to_hit, 2.0, "x = {face}");
    }
}

#[test]
fn ray_starting_inside_the_box_in_a_face_plane_reports_its_exit() {
    // Started inside on y, so the front face is behind it; the back face is 0.5 ahead. -0.0
    // flips which x face the slab test enters through.
    let bounds = unit_box();
    for direction_x in [0.0, -0.0] {
        for face in [0.0, 1.0] {
            let ray = Ray::with_limits(
                Vector3::new(face, 0.5, 0.5),
                Vector3::new(direction_x, 1.0, 0.0),
                0.0,
                f64::INFINITY,
                IntersectionType::BackFace,
            );
            let case = format!("x = {face}, direction x {direction_x:?}");
            assert!(ray.intersection(&bounds), "intersection missed: {case}");
            let (hit, found) = ray.intersect(&bounds);
            assert!(hit, "intersect missed: {case}");
            assert_eq!(found.min_dist_found, -0.5, "entry distance: {case}");
            assert_eq!(found.max_dist_found, 0.5, "exit distance: {case}");
            let info = ray.get_closest_intersection(&bounds);
            assert_eq!(info.hit_type, IntersectionType::BackFace, "{case}");
            assert_eq!(info.distance_to_hit, 0.5, "{case}");
            assert_eq!(info.normal_at_hit, Vector3::new(0.0, 1.0, 0.0), "{case}");
        }
    }
}

#[test]
fn ray_parallel_to_a_slab_outside_it_misses() {
    let bounds = unit_box();
    for plane_axis in 0..3 {
        let travel_axis = (plane_axis + 1) % 3;
        for outside in [-0.5, 1.5, -f64::MIN_POSITIVE, 1.0 + f64::EPSILON] {
            let ray = ray_in_face(plane_axis, outside, travel_axis);
            let case = format!("axis {plane_axis} at {outside}");
            assert!(!ray.intersection(&bounds), "intersection hit: {case}");
            assert!(!ray.intersect(&bounds).0, "intersect hit: {case}");
            let info = ray.get_closest_intersection(&bounds);
            assert_eq!(info.hit_type, IntersectionType::None, "closest hit: {case}");
        }
    }
}

#[test]
fn ray_with_a_nan_direction_misses() {
    let bounds = unit_box();
    let ray = Ray::new(
        Vector3::new(0.5, 0.5, -2.0),
        Vector3::new(f64::NAN, f64::NAN, f64::NAN),
    );
    assert!(!ray.intersection(&bounds));
    assert!(!ray.intersect(&bounds).0);
    assert_eq!(
        ray.get_closest_intersection(&bounds).hit_type,
        IntersectionType::None
    );
}

#[test]
fn ray_lying_in_a_zero_thickness_box_hits() {
    // The box is flat on `plane_axis` at 0.5 and the ray lies in that plane.
    for plane_axis in 0..3 {
        let mut min = Vector3::new(0.0, 0.0, 0.0);
        let mut max = Vector3::new(1.0, 1.0, 1.0);
        min[plane_axis] = 0.5;
        max[plane_axis] = 0.5;
        let bounds = AxisAlignedBoundingBox::new(min, max);
        let travel_axis = (plane_axis + 1) % 3;
        let ray = ray_in_face(plane_axis, 0.5, travel_axis);
        let case = format!("flat on axis {plane_axis}");

        assert!(ray.intersection(&bounds), "intersection missed: {case}");
        let (hit, found) = ray.intersect(&bounds);
        assert!(hit, "intersect missed: {case}");
        assert_eq!(found.min_dist_found, 2.0, "entry distance: {case}");
        assert_eq!(found.max_dist_found, 3.0, "exit distance: {case}");
        let info = ray.get_closest_intersection(&bounds);
        assert_eq!(info.hit_type, IntersectionType::FrontFace, "{case}");
        assert_eq!(info.distance_to_hit, 2.0, "{case}");
    }
}

#[test]
fn bvh_pick_along_an_items_x_face_finds_it() {
    // Two items side by side; the pick ray runs along x = 0, the near item's min x face and the
    // tree's own min x face, so the tree's root box is the first thing it must not miss.
    let near =
        AxisAlignedBoundingBox::new(Vector3::new(0.0, 0.0, 0.0), Vector3::new(1.0, 1.0, 1.0));
    let far = AxisAlignedBoundingBox::new(Vector3::new(5.0, 0.0, 0.0), Vector3::new(6.0, 1.0, 1.0));
    let tree = TradeOffBvhConstructor::create_new_hierarchy(
        vec![
            BvhTreeItemData::new("near", near),
            BvhTreeItemData::new("far", far),
        ],
        TradeOffBvhConstructor::DEFAULT_MAX_RECURSION,
        0,
        None,
        TradeOffBvhConstructor::DEFAULT_DO_SIMPLE_SORT_SIZE,
    )
    .expect("two items build a tree");
    let pick = Ray::new(Vector3::new(0.0, -2.0, 0.5), Vector3::new(0.0, 1.0, 0.0));

    let mut along = Vec::new();
    tree.along_ray(&pick, &mut along);
    assert_eq!(along, vec![&"near"]);

    let mut ray = pick.clone();
    let hit = tree
        .get_closest_intersection(&mut ray)
        .expect("the pick along the near item's face finds it");
    assert_eq!(hit.closest_hit_object, Some(&"near"));
    assert_eq!(hit.hit_type, IntersectionType::FrontFace);
    assert_eq!(hit.distance_to_hit, 2.0);
    assert_eq!(hit.hit_position, Vector3::new(0.0, 0.0, 0.5));
}
