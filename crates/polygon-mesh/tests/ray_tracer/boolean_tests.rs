//! View-space CSG with `BoxShape` and `Difference`, and `IntersectInfo::subtract` on a hit list
//! that once failed. Not run here: a subtract through a BVH of boxes.

use std::sync::Arc;

use solidmesh_math::ColorF;
use solidmesh_math::{IntersectionType, Ray, Vector3};
use solidmesh_mesh::ray_tracer::{
    BoxShape, Difference, IntersectInfo, MaterialRef, SolidMaterial, Traceable, TraceableRef,
};

#[test]
fn difference_tests_for_box() {
    let red_material: MaterialRef = Arc::new(SolidMaterial::new(ColorF::RED, 0.0, 0.0, 0.0));
    let blue_material: MaterialRef = Arc::new(SolidMaterial::new(ColorF::BLUE, 0.0, 0.0, 0.0));
    let cast_ray = Ray::new(Vector3::new(0.0, -1.0, 0.0), Vector3::UNIT_Y);

    let box1_x1 = Arc::new(
        BoxShape::new(
            Vector3::new(-0.5, -0.5, -0.5),
            Vector3::new(0.5, 0.5, 0.5),
            Some(blue_material),
        )
        .unwrap(),
    );

    // just a box all by itself
    {
        let test_info = box1_x1
            .get_closest_intersection(&mut cast_ray.clone())
            .unwrap_or_default();

        assert!(test_info.hit_type == IntersectionType::FrontFace);
        assert!(test_info.hit_object_is(box1_x1.as_ref()));
        assert!(test_info.hit_position == Vector3::new(0.0, -0.5, 0.0));
        assert!(test_info.distance_to_hit == 0.5);
        assert!(test_info.normal_at_hit == -Vector3::UNIT_Y);
    }

    // one subtract from the front of a box, the front faces are aligned
    {
        let subtract_box = Arc::new(
            BoxShape::new(
                Vector3::new(-0.5, -0.5, -0.5),
                Vector3::new(0.5, 0.0, 0.5),
                Some(red_material),
            )
            .unwrap(),
        );
        let primary: TraceableRef = box1_x1.clone();
        let subtract: TraceableRef = subtract_box.clone();
        let merge = Difference::new(primary, subtract);
        let test_info = merge
            .get_closest_intersection(&mut cast_ray.clone())
            .unwrap_or_default();

        assert!(test_info.hit_type == IntersectionType::FrontFace);
        assert!(test_info.hit_object_is(subtract_box.as_ref()));
        assert!(test_info.hit_position == Vector3::new(0.0, 0.0, 0.0));
        assert!(test_info.distance_to_hit == 1.0);
        assert!(test_info.normal_at_hit == -Vector3::UNIT_Y);
    }
}

#[test]
fn discovered_bad_intersect_info_list_subtraction() {
    let primary_string = "2
FrontFace, 6.55505298172777
BackFace, 7.05554361306285";
    let subtract_string = "4
FrontFace, 7.05554387355765
BackFace, 7.14478176419901
FrontFace, 7.28926063619785
BackFace, 7.36209329430552";

    let mut all_primary = Vec::new();
    IntersectInfo::read_in_list(&mut all_primary, &mut primary_string.lines()).unwrap();

    let mut all_subtract = Vec::new();
    IntersectInfo::read_in_list(&mut all_subtract, &mut subtract_string.lines()).unwrap();

    let mut result = Vec::new();

    // The case passes when the subtract returns without an error.
    IntersectInfo::subtract(&all_primary, &all_subtract, &mut result).unwrap();
}
