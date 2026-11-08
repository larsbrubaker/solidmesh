// Frustum tests: transforming a frustum, and clipping boxes and lines against its planes. They
// use only vector-math's Frustum, Plane, Matrix4X4 and AxisAlignedBoundingBox.
// `ray_bundle_same_result_as_individual_rays` has no checks yet: ray bundles against single rays
// is still an open case.

use solidmesh_math::{
    math_helper, AxisAlignedBoundingBox, Frustum, FrustumIntersection, Matrix4X4, Plane, Vector3,
};

fn v3(x: f64, y: f64, z: f64) -> Vector3 {
    Vector3::new(x, y, z)
}

fn aabb(min: [f64; 3], max: [f64; 3]) -> AxisAlignedBoundingBox {
    AxisAlignedBoundingBox::new(v3(min[0], min[1], min[2]), v3(max[0], max[1], max[2]))
}

#[test]
fn ray_bundle_same_result_as_individual_rays() {}

#[test]
fn frustum_transform_tests() {
    let frustum = Frustum::from_planes(
        Plane::new(v3(1.0, 0.0, 0.0), 20.0),
        Plane::new(v3(-1.0, 0.0, 0.0), 20.0),
        Plane::new(v3(0.0, 1.0, 0.0), 20.0),
        Plane::new(v3(0.0, -1.0, 0.0), 20.0),
        Plane::new(v3(0.0, 0.0, 1.0), 20.0),
        Plane::new(v3(0.0, 0.0, -1.0), 20.0),
    );

    // moved right
    {
        let moved_right_frustum =
            Frustum::transform(&frustum, Matrix4X4::create_translation(10.0, 0.0, 0.0));
        assert!(moved_right_frustum.planes[0].equals(&Plane::new(v3(1.0, 0.0, 0.0), 30.0)));
        assert!(moved_right_frustum.planes[1].equals(&Plane::new(v3(-1.0, 0.0, 0.0), 10.0)));
        assert!(moved_right_frustum.planes[2].equals(&frustum.planes[2]));
        assert!(moved_right_frustum.planes[3].equals(&frustum.planes[3]));
        assert!(moved_right_frustum.planes[4].equals(&frustum.planes[4]));
        assert!(moved_right_frustum.planes[5].equals(&frustum.planes[5]));
    }

    // rotated right
    {
        let moved_right_frustum = Frustum::transform(
            &frustum,
            Matrix4X4::create_rotation_y(math_helper::degrees_to_radians(45.0)),
        );
        let test_matrix = Matrix4X4::create_rotation_y(math_helper::degrees_to_radians(45.0));
        let control = Plane::new(
            frustum.planes[0].normal.transform_normal(test_matrix),
            frustum.planes[0].distance_from_origin,
        );
        assert!(moved_right_frustum.planes[0].equals(&control));
        assert!(moved_right_frustum.planes[1].equals(&Plane::new(
            frustum.planes[1].normal.transform_normal(test_matrix),
            frustum.planes[1].distance_from_origin
        )));
        assert!(moved_right_frustum.planes[2].equals(&frustum.planes[2]));
        assert!(moved_right_frustum.planes[3].equals(&frustum.planes[3]));
        assert!(moved_right_frustum.planes[4].equals(&Plane::new(
            frustum.planes[4].normal.transform_normal(test_matrix),
            frustum.planes[4].distance_from_origin
        )));
        assert!(moved_right_frustum.planes[5].equals(&Plane::new(
            frustum.planes[5].normal.transform_normal(test_matrix),
            frustum.planes[5].distance_from_origin
        )));
    }
}

#[test]
fn frustum_interset_aabb_tests() {
    {
        let frustum = Frustum::from_planes(
            Plane::new(v3(1.0, 0.0, 0.0), 20.0),
            Plane::new(v3(-1.0, 0.0, 0.0), 20.0),
            Plane::new(v3(0.0, 1.0, 0.0), 20.0),
            Plane::new(v3(0.0, -1.0, 0.0), 20.0),
            Plane::new(v3(0.0, 0.0, 1.0), 20.0),
            Plane::new(v3(0.0, 0.0, -1.0), 20.0),
        );

        // outside to left
        let intersection = frustum.get_intersect(&aabb([-30.0, -10.0, -10.0], [-25.0, 10.0, 10.0]));
        assert!(intersection == FrustumIntersection::Outside);

        // intersect
        let intersection = frustum.get_intersect(&aabb([-25.0, 0.0, -10.0], [-15.0, 10.0, 10.0]));
        assert!(intersection == FrustumIntersection::Intersect);

        // not intersect
        let intersection = frustum.get_intersect(&aabb([-25.0, 0.0, 30.0], [-15.0, 10.0, 35.0]));
        assert!(intersection == FrustumIntersection::Outside);

        // inside
        let intersection = frustum.get_intersect(&aabb([-5.0, -5.0, -5.0], [5.0, 5.0, 5.0]));
        assert!(intersection == FrustumIntersection::Inside);
    }

    {
        let frustum = Frustum::from_planes(
            Plane::new(v3(-1.0, -1.0, 0.0), 0.0),
            Plane::new(v3(1.0, -1.0, 0.0), 0.0),
            Plane::new(v3(0.0, -1.0, -1.0), 0.0),
            Plane::new(v3(0.0, -1.0, 1.0), 0.0),
            Plane::new(v3(0.0, -1.0, 0.0), 0.0),
            Plane::new(v3(0.0, 1.0, 0.0), 10000.0),
        );

        // outside to left
        let intersection = frustum.get_intersect(&aabb([-110.0, 0.0, -10.0], [-100.0, 10.0, 10.0]));
        assert!(intersection == FrustumIntersection::Outside);

        // intersect with origin (front)
        let intersection = frustum.get_intersect(&aabb([-10.0, -10.0, -10.0], [10.0, 10.0, 10.0]));
        assert!(intersection == FrustumIntersection::Intersect);

        // inside
        let intersection = frustum.get_intersect(&aabb([-5.0, 100.0, -5.0], [5.0, 110.0, 5.0]));
        assert!(intersection == FrustumIntersection::Inside);
    }

    {
        // looking down -z
        let frustum5_plane_neg_z = Frustum::from_normals(
            v3(-1.0, 0.0, 1.0),
            v3(-1.0, 0.0, 1.0),
            v3(0.0, 1.0, 1.0),
            v3(0.0, -1.0, 1.0),
            v3(0.0, 0.0, -1.0),
            10000.0,
        );

        // outside to left
        let intersection =
            frustum5_plane_neg_z.get_intersect(&aabb([-110.0, 0.0, -10.0], [-100.0, 10.0, 10.0]));
        assert!(intersection == FrustumIntersection::Outside);

        // intersect with origin (front)
        let intersection =
            frustum5_plane_neg_z.get_intersect(&aabb([-10.0, -10.0, -10.0], [10.0, 10.0, 10.0]));
        assert!(intersection == FrustumIntersection::Intersect);

        // inside
        let intersection =
            frustum5_plane_neg_z.get_intersect(&aabb([-5.0, -5.0, -110.0], [5.0, 5.0, -100.0]));
        assert!(intersection == FrustumIntersection::Inside);
    }
}
