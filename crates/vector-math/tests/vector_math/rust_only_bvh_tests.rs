// Rust-only checks for the BVH (`vector_math::bvh`), `Ray::get_closest_intersection`, the
// estimator and the track ball. The expected values of the `*_matches_csharp` tests are recorded
// output for these 30 boxes, with `BvhTreeItemData::center` left at zero as every caller leaves
// it; they pin tree shape, search order and hits, so any change to them shows up here.

use vector_math::bvh::{
    morton_codes, BvhBuilderLocallyOrderedClustering, BvhTree, BvhTreeItemData,
};
use vector_math::bvh::{SortingAccelerator, TradeOffBvhConstructor};
use vector_math::{
    estimator, AxisAlignedBoundingBox, IntersectionType, Matrix4X4, Quaternion, Ray,
    TrackBallController, TrackBallError, TrackBallTransformType, Vector2, Vector3,
};

fn test_boxes() -> Vec<AxisAlignedBoundingBox> {
    (0..30)
        .map(|i| {
            let x = f64::from((i * 37) % 11);
            let y = f64::from((i * 13) % 7);
            let z = f64::from((i * 5) % 3);
            AxisAlignedBoundingBox::new(
                Vector3::new(x, y, z),
                Vector3::new(x + 1.5, y + 1.0, z + 0.5),
            )
        })
        .collect()
}

fn test_tree() -> BvhTree<i32> {
    let items = test_boxes()
        .into_iter()
        .enumerate()
        .map(|(i, aabb)| BvhTreeItemData::new(i as i32, aabb))
        .collect();
    TradeOffBvhConstructor::create_new_hierarchy(
        items,
        TradeOffBvhConstructor::DEFAULT_MAX_RECURSION,
        0,
        None,
        10,
    )
    .expect("30 items make a tree")
}

fn copied(results: Vec<&i32>) -> Vec<i32> {
    results.into_iter().copied().collect()
}

#[test]
fn rust_only_trade_off_tree_matches_csharp() {
    let tree = test_tree();
    assert_eq!(tree.count_branches(), 7);

    let mut results = Vec::new();
    tree.search_bounds(
        &AxisAlignedBoundingBox::new(
            Vector3::new(-100.0, -100.0, -100.0),
            Vector3::new(100.0, 100.0, 100.0),
        ),
        &mut results,
    );
    assert_eq!(
        copied(results),
        vec![
            8, 7, 6, 5, 4, 3, 2, 1, 28, 29, 10, 9, 12, 11, 0, 27, 26, 25, 24, 23, 22, 21, 20, 19,
            18, 17, 16, 15, 14, 13
        ]
    );

    let mut results = Vec::new();
    tree.search_bounds(
        &AxisAlignedBoundingBox::new(Vector3::new(2.0, 2.0, 0.0), Vector3::new(5.0, 4.0, 1.0)),
        &mut results,
    );
    assert_eq!(copied(results), vec![6, 3, 12, 26, 20, 17]);
}

#[test]
fn rust_only_closest_intersection_matches_csharp() {
    let tree = test_tree();
    let expected = [
        (Vector3::new(3.2, 2.5, 10.0), None),
        (Vector3::new(0.7, 0.5, 10.0), Some((0, 9.5))),
        (Vector3::new(5.2, 3.5, 10.0), Some((4, 7.5))),
        (Vector3::new(9.9, 6.3, 10.0), None),
    ];
    for (origin, expected) in expected {
        let mut ray = Ray::new(origin, Vector3::new(0.0, 0.0, -1.0));
        let hit = tree.get_closest_intersection(&mut ray);
        match expected {
            None => assert!(hit.is_none(), "{origin}"),
            Some((item, distance)) => {
                let hit = hit.expect("a recorded hit here");
                assert_eq!(hit.closest_hit_object, Some(&item));
                assert_eq!(hit.distance_to_hit, distance);
                assert_eq!(hit.hit_type, IntersectionType::FrontFace);
                assert_eq!(hit.normal_at_hit, Vector3::new(0.0, 0.0, 1.0));
                assert_eq!(ray.max_distance_to_consider, distance);
            }
        }
    }
}

#[test]
fn rust_only_locally_ordered_clustering_matches_csharp() {
    let boxes = test_boxes();
    let nodes = BvhBuilderLocallyOrderedClustering::build_bvh(&boxes);
    let described: Vec<String> = nodes
        .iter()
        .map(|n| format!("{},{}", n.left, n.right))
        .collect();
    assert_eq!(
        described.join(";"),
        "1,40;2,23;3,16;4,7;5,6;6,6;12,12;8,11;9,10;14,14;20,20;12,13;11,11;14,15;26,26;0,0;\
         17,22;18,21;19,20;21,21;27,27;18,18;5,5;24,33;25,30;26,27;3,3;28,29;9,9;15,15;31,32;\
         17,17;23,23;34,35;24,24;36,37;29,29;38,39;2,2;8,8;41,52;42,47;43,44;28,28;45,46;25,25;\
         4,4;48,51;49,50;7,7;13,13;19,19;53,56;54,55;22,22;1,1;57,58;10,10;16,16"
    );
    // Building the tree leaves the caller's boxes as they were: changing an input as a side
    // effect of building a tree would surprise the caller.
    assert_eq!(boxes, test_boxes());
}

#[test]
fn rust_only_tree_searches_and_all() {
    let tree = test_tree();
    let mut all = Vec::new();
    tree.all(&mut all);
    let mut all = copied(all);
    all.sort();
    assert_eq!(all, (0..30).collect::<Vec<_>>());

    let mut at_point = Vec::new();
    tree.search_point(Vector3::new(0.7, 0.5, 0.25), &mut at_point);
    assert_eq!(copied(at_point), vec![0]);

    let ray = Ray::new(Vector3::new(0.7, 0.5, 10.0), Vector3::new(0.0, 0.0, -1.0));
    let mut along = Vec::new();
    tree.along_ray(&ray, &mut along);
    assert!(along.contains(&&0));
    for item in along {
        assert!(ray.intersection(&test_boxes()[*item as usize]));
    }

    let empty: Option<BvhTree<i32>> =
        TradeOffBvhConstructor::create_new_hierarchy(Vec::new(), i32::MAX, 0, None, 5000);
    assert!(empty.is_none());
}

#[test]
fn rust_only_sorting_accelerator_rotates_from_x() {
    let mut accelerator = SortingAccelerator::new();
    assert_eq!(
        [
            accelerator.next_axis(),
            accelerator.next_axis(),
            accelerator.next_axis()
        ],
        [0, 1, 2]
    );
}

#[test]
fn rust_only_morton_codes_interleave_xyz() {
    assert_eq!(morton_codes::encode3(1, 0, 0), 1);
    assert_eq!(morton_codes::encode3(0, 1, 0), 2);
    assert_eq!(morton_codes::encode3(0, 0, 1), 4);
    assert_eq!(morton_codes::encode3(1, 1, 1), 7);
    assert_eq!(morton_codes::encode3(2, 0, 0), 8);
    assert_eq!(morton_codes::SIZE, 1 << 21);
}

#[test]
fn rust_only_ray_closest_intersection_front_and_back() {
    let bounds =
        AxisAlignedBoundingBox::new(Vector3::new(-1.0, -1.0, -1.0), Vector3::new(1.0, 1.0, 1.0));
    let front = Ray::new(Vector3::new(0.25, 0.25, 5.0), Vector3::new(0.0, 0.0, -1.0));
    let hit = front.get_closest_intersection(&bounds);
    assert_eq!(hit.hit_type, IntersectionType::FrontFace);
    assert_eq!(hit.distance_to_hit, 4.0);
    assert_eq!(hit.hit_position, Vector3::new(0.25, 0.25, 1.0));
    assert_eq!(hit.normal_at_hit, Vector3::new(0.0, 0.0, 1.0));
    assert_eq!(hit.closest_hit_object, Some(bounds));

    let back = Ray::with_limits(
        Vector3::new(0.25, 0.25, 5.0),
        Vector3::new(0.0, 0.0, -1.0),
        0.0,
        f64::INFINITY,
        IntersectionType::BackFace,
    );
    let hit = back.get_closest_intersection(&bounds);
    assert_eq!(hit.hit_type, IntersectionType::BackFace);
    assert_eq!(hit.distance_to_hit, 6.0);

    let miss = Ray::new(Vector3::new(5.0, 5.0, 5.0), Vector3::new(0.0, 0.0, -1.0));
    let hit = miss.get_closest_intersection(&bounds);
    assert_eq!(hit.hit_type, IntersectionType::None);
    assert_eq!(hit.distance_to_hit, f64::MAX);
    assert!(hit.closest_hit_object.is_none());
}

#[test]
fn rust_only_estimator_trapezoid_and_triangle() {
    // accelerate 5 mm to 100 mm/s in 0.1 s, cruise 90 mm in 0.9 s, decelerate in 0.1 s
    let seconds = estimator::get_seconds_for_movement(100.0, 100.0, 1000.0, 200.0, 0.0, 1.0);
    assert!((seconds - 1.1).abs() < 1e-12, "{seconds}");
    // too short to reach 100 mm/s: accelerate over half, decelerate over half
    let seconds = estimator::get_seconds_for_movement(2.0, 100.0, 1000.0, 200.0, 0.0, 1.0);
    assert!((seconds - 2.0 * (2.0f64 * 1000.0).sqrt() / 1000.0).abs() < 1e-12);
    assert_eq!(
        estimator::get_seconds_for_movement(0.0, 1.0, 1.0, 1.0, 1.0, 1.0),
        0.0
    );
}

#[test]
fn rust_only_track_ball_rotation_and_errors() {
    let start = Vector2::new(10.0, 10.0);
    assert_eq!(
        TrackBallController::get_rotation_for_move(Vector2::ZERO, 100.0, start, start, false),
        Quaternion::IDENTITY
    );

    let mut controller = TrackBallController::new(1.0);
    assert_eq!(controller.on_mouse_up(), Err(TrackBallError::NotTracking));
    controller
        .on_mouse_down(start, Matrix4X4::IDENTITY, TrackBallTransformType::Scale)
        .unwrap();
    let mut world = vector_math::WorldView::new(100.0, 100.0);
    controller
        .on_mouse_move(&mut world, Vector2::new(10.0, 20.0))
        .unwrap();
    assert!((world.scale() - 1.1).abs() < 1e-12);
    controller.on_mouse_up().unwrap();
    assert_eq!(
        controller.current_tracking_type(),
        TrackBallTransformType::None
    );
}
