//! Rust-only tests for the bevel-core package R1.P04's standing-feature and notch readings, split
//! from `rust_only_bevel_r1p04_tests.rs` by responsibility: the feature's ridge, crease and wedge
//! (`SurfaceStandingFeatureWithheld`), the end cap's walk round a notch
//! (`SurfaceEndCapAroundAFeature.TryWalkAround`), the notch's own readings off the strip
//! (`SurfaceStripNotchedByAFeature`) and the hull end's turning point
//! (`SurfaceStripEndsOnTheHull`). Each pins the rule on a hand-built case.

use std::cell::RefCell;
use std::rc::Rc;

use solidmesh::bevel::surface::{
    surface_end_cap_around_a_feature, SurfaceEndCapAroundAFeatureMeeting,
    SurfaceStandingFeatureWithheld, SurfaceStandingFeatureWithheldFace, SurfaceStrip,
    SurfaceStripEndsOnTheHull, SurfaceStripEndsOnTheHullSupportingPlane,
    SurfaceStripNotchedByAFeature, SurfaceStripNotchedByAFeatureBetween,
    SurfaceStripTrimmedByTheFaceBeyond, SurfaceVertexPool,
};
use solidmesh_math::Vector3;

use crate::rust_only_bevel_r1p06_tests::square_edge_plan;

/// A two-faced feature: the floor z = 0 and the wall x = 0, its one crease running from the ridge
/// at (0, 5, 0) out to (3, 5, 0) on the floor.
fn two_faced_feature(
    pool: &mut SurfaceVertexPool,
) -> (Rc<SurfaceStandingFeatureWithheld>, i32, i32) {
    let on_the_ridge = pool.add(Vector3::new(0.0, 5.0, 0.0));
    let on_the_end = pool.add(Vector3::new(3.0, 5.0, 0.0));
    let face = |normal: Vector3| {
        Rc::new(SurfaceStandingFeatureWithheldFace {
            creases: RefCell::new(vec![(on_the_end, on_the_ridge)]),
            plane_point: Vector3::ZERO,
            plane_normal: normal,
            ..Default::default()
        })
    };
    let feature = SurfaceStandingFeatureWithheld {
        faces: vec![
            face(Vector3::new(0.0, 0.0, 1.0)),
            face(Vector3::new(0.0, 0.0, 1.0)),
            face(Vector3::new(1.0, 0.0, 0.0)),
        ],
        ..Default::default()
    };
    (Rc::new(feature), on_the_ridge, on_the_end)
}

/// The ridge is where the two planes meet, the crease's ends are told apart by it, and a strip's
/// rail is cut by the face whose crossing stands inside the other's half space.
#[test]
fn rust_only_a_standing_feature_reads_its_ridge_and_wedge() {
    let mut pool = SurfaceVertexPool::new(&[]);
    let (feature, on_the_ridge, on_the_end) = two_faced_feature(&mut pool);

    let (point, along) = feature.try_the_ridge().expect("two planes meet in a line");
    assert!(point.length() < 1e-12);
    assert!((along - Vector3::new(0.0, 1.0, 0.0)).length() < 1e-12);
    assert_eq!(
        feature.try_the_ends_of_the_crease(&pool, &feature.faces[0], 0),
        Some((on_the_ridge, on_the_end))
    );

    let one_plane = SurfaceStandingFeatureWithheld {
        faces: feature.faces[..2].to_vec(),
        ..Default::default()
    };
    assert!(one_plane.try_the_ridge().is_none());

    // A rail running down -z at x = -1: it crosses the floor (both of the floor's facets) inside the
    // wall's half space x <= 0, and runs parallel to the wall, which therefore cuts it nowhere.
    let strip = SurfaceStrip {
        origin: Vector3::ZERO,
        along: Vector3::new(0.0, 0.0, -1.0),
        rail_points: vec![Vector3::new(-1.0, 2.0, 1.0)],
        rail_step_degrees: 0.0,
        outward_reference: Vector3::new(0.0, 1.0, 0.0),
    };
    let mut found = Vec::new();
    feature.crossings_on_the_wedge(&strip, 0, 1e-9, &mut found);
    assert_eq!(found.len(), 2);
    assert!(found
        .iter()
        .all(|(face, at)| face.plane_normal.z == 1.0 && (*at - 1.0).abs() < 1e-12));
}

/// The walk round a section's rim breaks at the notch: the rails before it are the head, the
/// rails after it the tail, and each meets the feature at the Q standing on the chord it crossed.
#[test]
fn rust_only_the_end_cap_walks_round_the_notch() {
    let mut pool = SurfaceVertexPool::new(&[]);
    let (feature, _, on_the_end) = two_faced_feature(&mut pool);
    let q = |vertex: i32, below: i32| SurfaceStripNotchedByAFeatureBetween {
        vertex,
        below,
        param: 0.0,
        face: Some(feature.faces[0].clone()),
    };
    let notch = SurfaceStripNotchedByAFeature {
        strip: 0,
        at_the_start: false,
        feature: feature.clone(),
        reach: 0,
        rail_of: vec![1, 2],
        param_of: vec![0.0, 0.0],
        face_of: vec![],
        end_param: 0.0,
        exit: SurfaceStripNotchedByAFeatureBetween {
            vertex: -1,
            below: -1,
            ..Default::default()
        },
        at_the_low_rail: q(20, 0),
        at_the_high_rail: q(21, 2),
        the_crease_rail: -1,
    };
    assert!(!notch.is_one_faced() && notch.notches(2) && !notch.notches(3));
    assert!(!notch.exit.stands() && notch.at_the_low_rail.stands());

    let (head, head_step, tail, at_the_head, at_the_tail) =
        surface_end_cap_around_a_feature::try_walk_around(
            &pool,
            &[0, 1, 2, 3],
            -1,
            &notch,
            &[10, 11, 12, 13],
            Vector3::ZERO,
        )
        .expect("one notch in the middle of the walk");
    assert_eq!((head, head_step, tail), (vec![10], vec![0], vec![13]));
    assert_eq!(
        at_the_head,
        SurfaceEndCapAroundAFeatureMeeting::new(20, on_the_end)
    );
    assert_eq!(
        at_the_tail,
        SurfaceEndCapAroundAFeatureMeeting::new(21, on_the_end)
    );

    // A notch on the first rail of the walk leaves no chord for a Q.
    assert!(surface_end_cap_around_a_feature::try_walk_around(
        &pool,
        &[1, 2, 3],
        -1,
        &notch,
        &[10, 11, 12, 13],
        Vector3::ZERO
    )
    .is_err());
}

/// The notch's small readings and the hull end's turning point between two planes.
#[test]
fn rust_only_the_notch_and_hull_readings() {
    let mut pool = SurfaceVertexPool::new(&[]);
    let (feature, _, _) = two_faced_feature(&mut pool);
    let notch = SurfaceStripNotchedByAFeature::new(
        0,
        true,
        feature.clone(),
        0,
        vec![2, 3],
        vec![0.5, 0.75],
        vec![feature.faces[0].clone(), feature.faces[0].clone()],
        1.0,
        SurfaceStripNotchedByAFeatureBetween::none(),
        SurfaceStripNotchedByAFeatureBetween::none(),
        SurfaceStripNotchedByAFeatureBetween::none(),
        -1,
    );
    assert_eq!(notch.param_on(3), Some(0.75));
    assert_eq!(notch.param_on(0), None);
    assert!(SurfaceStripNotchedByAFeature::on_one_face_only(
        &notch.face_of
    ));
    assert!(!SurfaceStripNotchedByAFeature::on_one_face_only(
        &feature.faces[1..]
    ));
    assert!(!SurfaceStripNotchedByAFeatureBetween::none().stands());

    let near = pool.position_of(0);
    let count = pool.count();
    assert_eq!(
        SurfaceStripNotchedByAFeature::share(&mut pool, near, 1e-6),
        0
    );
    assert_eq!(
        SurfaceStripNotchedByAFeature::share(&mut pool, Vector3::new(7.0, 7.0, 7.0), 1e-6),
        count
    );

    let trim = SurfaceStripTrimmedByTheFaceBeyond::new(
        1,
        true,
        0.0,
        vec![],
        2,
        0,
        0,
        Vector3::ZERO,
        Vector3::ZERO,
    );
    assert!(SurfaceStripNotchedByAFeature::is_collapsed(
        &[None, Some(&trim)],
        5,
        4
    ));
    assert!(!SurfaceStripNotchedByAFeature::is_collapsed(
        &[None, Some(&trim)],
        5,
        2
    ));

    let table = vec![[Some(Rc::new(notch)), None]];
    assert!(SurfaceStripNotchedByAFeature::standing_at(Some(&table), 0, true).is_some());
    assert!(SurfaceStripNotchedByAFeature::standing_at(Some(&table), 0, false).is_none());

    // A strip along x ruled by rails at y = 0 and y = 2, between the planes x + y = 1 and x - y = -1:
    // they meet the band at (0, 1, 0), half way across, at the edge's start.
    let strip = SurfaceStrip {
        origin: Vector3::ZERO,
        along: Vector3::new(1.0, 0.0, 0.0),
        rail_points: vec![Vector3::new(0.0, 0.0, 0.0), Vector3::new(0.0, 2.0, 0.0)],
        rail_step_degrees: 0.0,
        outward_reference: Vector3::new(0.0, 0.0, 1.0),
    };
    let plane =
        |at: Vector3, normal: Vector3| SurfaceStripEndsOnTheHullSupportingPlane { at, normal };
    let planes = [
        plane(
            Vector3::new(1.0, 0.0, 0.0),
            Vector3::new(1.0, 1.0, 0.0).get_normal(),
        ),
        plane(
            Vector3::new(-1.0, 0.0, 0.0),
            Vector3::new(1.0, -1.0, 0.0).get_normal(),
        ),
    ];
    let between = SurfaceStripEndsOnTheHull::try_between(&strip, &planes, 0, 10.0, 1e-9)
        .expect("the two planes cross the band between its rails");
    assert!((between - Vector3::new(0.0, 1.0, 0.0)).length() < 1e-12);
    assert!(
        SurfaceStripEndsOnTheHull::try_between(&strip, &[planes[0], planes[0]], 0, 10.0, 1e-9)
            .is_none()
    );
}

/// The notch's readings off the strip: the ridge meets a chord's quad, a face's plane cuts the end
/// ring's chord at a Q, the feature leaves through the end its crease reaches, a single wedge
/// crossing names the face that cuts, the crease runs out on one seam, and the exit stands where
/// the faces change over.
#[test]
fn rust_only_the_notch_reads_the_strip() {
    let plan = square_edge_plan(1.0, false);
    let chamfer_strip = SurfaceStrip::try_create(&plan, 4).expect("a chamfer");
    let x = Vector3::new(1.0, 0.0, 0.0);
    let y = Vector3::new(0.0, 1.0, 0.0);
    let z = Vector3::new(0.0, 0.0, 1.0);

    let on_the_chord = SurfaceStripNotchedByAFeature::try_the_ridge_on_the_chord(
        &chamfer_strip,
        &plan,
        Vector3::new(3.0, 0.0, 0.0),
        z,
        0,
    )
    .expect("the ridge leaves the quad");
    assert!((on_the_chord - Vector3::new(3.0, 0.0, -1.0)).length() < 1e-12);
    assert!(SurfaceStripNotchedByAFeature::try_the_ridge_on_the_chord(
        &chamfer_strip,
        &plan,
        Vector3::ZERO,
        (y + z).get_normal(),
        0
    )
    .is_none());

    let mut pool = SurfaceVertexPool::new(&[]);
    let (feature, on_the_ridge, on_the_end) = two_faced_feature(&mut pool);
    let ruled = SurfaceStrip {
        origin: Vector3::ZERO,
        along: x,
        rail_points: (0..3).map(|rail| y * rail as f64).collect(),
        rail_step_degrees: 30.0,
        outward_reference: z,
    };
    let cutting = Rc::new(SurfaceStandingFeatureWithheldFace {
        plane_point: y * 1.5,
        plane_normal: y,
        ..Default::default()
    });
    let q = SurfaceStripNotchedByAFeature::try_the_q(&mut pool, &ruled, 1, 2, &cutting, 4.0, 1e-6)
        .expect("the plane cuts the chord");
    assert_eq!((q.below, q.param), (1, 4.0));
    assert!((pool.position_of(q.vertex) - Vector3::new(4.0, 1.5, 0.0)).length() < 1e-6);
    assert!(
        SurfaceStripNotchedByAFeature::try_the_q(&mut pool, &ruled, 2, 3, &cutting, 4.0, 1e-6)
            .is_err()
    );

    // The crease runs from (3, 5, 0) to the ridge at (0, 5, 0); a strip along y meets both at 5.
    let along_y = SurfaceStrip {
        origin: Vector3::ZERO,
        along: y,
        rail_points: vec![Vector3::ZERO, x],
        rail_step_degrees: 0.0,
        outward_reference: z,
    };
    assert_eq!(
        SurfaceStripNotchedByAFeature::try_the_end_it_leaves_through(
            &pool, &along_y, &feature, 0, 5.0, 0.1
        ),
        Some((false, 5.0))
    );
    assert_eq!(
        SurfaceStripNotchedByAFeature::try_the_end_it_leaves_through(
            &pool, &along_y, &feature, 0, 10.0, 0.1
        ),
        None
    );

    // A rail running down -z at x = -1: the two floor facets both cross it (no one face), the floor
    // and wall alone cross it once.
    let down = SurfaceStrip {
        origin: Vector3::ZERO,
        along: -z,
        rail_points: vec![Vector3::new(-1.0, 2.0, 1.0)],
        rail_step_degrees: 0.0,
        outward_reference: y,
    };
    assert!(
        SurfaceStripNotchedByAFeature::try_the_face_that_cuts(&down, &feature, 0, 1e-9).is_none()
    );
    let floor_and_wall = SurfaceStandingFeatureWithheld {
        faces: feature.faces[1..].to_vec(),
        ..Default::default()
    };
    let (face, at) =
        SurfaceStripNotchedByAFeature::try_the_face_that_cuts(&down, &floor_and_wall, 0, 1e-9)
            .expect("one face cuts the rail");
    assert!(Rc::ptr_eq(&face, &feature.faces[1]) && (at - 1.0).abs() < 1e-12);

    // The seam on side 1 ends on the crease (at its far end, param 7): the crease runs out there.
    let seams = vec![[None, Some(vec![(on_the_ridge, 2.0), (on_the_end, 7.0)])]];
    let (crease_rail, crossing) = SurfaceStripNotchedByAFeature::try_the_crease_crossing(
        &pool,
        Some(&seams),
        0,
        4,
        false,
        &feature.faces[0],
        0,
        1e-6,
    )
    .expect("one seam ends on the crease");
    assert_eq!(
        (crease_rail, crossing.vertex, crossing.below, crossing.param),
        (3, on_the_end, -1, 7.0)
    );
    let both = vec![[
        Some(vec![(on_the_end, 7.0)]),
        Some(vec![(on_the_ridge, 7.0)]),
    ]];
    assert!(SurfaceStripNotchedByAFeature::try_the_crease_crossing(
        &pool,
        Some(&both),
        0,
        4,
        false,
        &feature.faces[0],
        0,
        1e-6
    )
    .is_none());

    // The faces change over between the floor and the wall: the ridge (along y through the origin)
    // meets the chamfer's quad at (0, 1, 0). One face all the way is no changeover and no exit.
    let exit = SurfaceStripNotchedByAFeature::try_the_exit(
        &mut pool,
        &chamfer_strip,
        &plan,
        &feature,
        &[0, 1],
        &[feature.faces[0].clone(), feature.faces[2].clone()],
        1e-6,
    )
    .expect("one changeover, one ridge");
    assert_eq!(exit.below, 0);
    assert!((pool.position_of(exit.vertex) - y).length() < 1e-6);
    assert!(SurfaceStripNotchedByAFeature::try_the_exit(
        &mut pool,
        &chamfer_strip,
        &plan,
        &feature,
        &[0, 1],
        &[feature.faces[0].clone(), feature.faces[0].clone()],
        1e-6
    )
    .is_err());
}
