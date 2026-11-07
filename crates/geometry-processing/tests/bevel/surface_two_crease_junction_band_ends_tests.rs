//! What the faces give up at the two-crease junction of `surface_two_crease_junction_reading_tests` -
//! which bands are bounded there, and what the walk back to the tangent lines leaves standing.
//!
//! **One rule, read off the face.** A band is bounded at a junction by its edge's own stopping
//! plane unless the face it lies on is the face some arm SHARES (`surface_junction_band_ends`).
//! Here that turns the one-crease junction's answer over: the CUT's two faces are BOTH shared - one
//! with each crease - so the cut is bounded on neither and the two beads' bands are what end it;
//! and each bead is bounded only on the wall the two creases stand against, which is nobody's
//! shared face, on its own square section.

use std::collections::HashMap;
use std::rc::Rc;
use std::sync::Arc;

use geometry_processing::bevel::surface::{
    surface_bevel_builder, surface_face_walkback, SurfaceBevelPlan, SurfaceBuildOptions,
    SurfaceWalkbackResult,
};
use geometry_processing::topology::{FaceRegionCache, FeatureEdgeSet};
use polygon_mesh::CancelToken;
use vector_math::Vector3;

use crate::mesh_measurements::shell_count;
use crate::nose_in_notch_test_meshes;
use crate::surface_two_crease_junction_reading_tests::{
    the_junction, the_plans_of_the_right_ridge_end, RADIUS,
};
use crate::surface_walkback_checks::regions_of;

/// Where the ball touches the nose's UPPER face - the one point S is left with at the corner.
fn the_upper_tangency() -> Vector3 {
    Vector3::new(10.784822, 7.496151, 11.664101)
}

/// Its mirror on the lower face.
fn the_lower_tangency() -> Vector3 {
    Vector3::new(10.784822, 7.496151, 8.335899)
}

/// Where the upper bead's square end plane cuts its own crease - the foot the wall keeps.
fn the_upper_crease_foot() -> Vector3 {
    Vector3::new(11.800611, 8.199389, 12.132926)
}

/// Its mirror.
fn the_lower_crease_foot() -> Vector3 {
    Vector3::new(11.800611, 8.199389, 7.867074)
}

/// The CUT is bounded on NEITHER of its faces and each bead only on the wall - the whole of what
/// the generalised rule says about this corner.
#[test]
fn the_cut_is_bounded_on_neither_face_and_each_bead_only_on_the_wall() {
    let (_, _, plans) = the_junction();

    for left in [true, false] {
        assert_eq!(
            plans[0].band_ends_of(0, left).len(),
            0,
            "both of the ridge's faces are shared with a crease, so both of its bands run on to \
             meet theirs"
        );
    }

    for (plan, tangency) in [(1usize, the_upper_tangency()), (2, the_lower_tangency())] {
        let wall = plans[plan].edges[0].left_region_key() == plans[0].edges[0].left_region_key()
            || plans[plan].edges[0].left_region_key() == plans[0].edges[0].right_region_key();

        assert_eq!(
            plans[plan].band_ends_of(0, wall).len(),
            0,
            "the face a crease shares with the ridge is ended by the ridge's own band"
        );

        let ends = plans[plan].band_ends_of(0, !wall);

        assert_eq!(
            ends.len(),
            1,
            "and on the wall nobody picked the crease has to stop by itself"
        );

        // SQUARE, not the leaning plane the cut's own end ring lies in: the canal's first ball is
        // centred on the bead's axis, so the ring it is tangent to is the bead's own section.
        assert!(
            (ends[0].point - tangency).length() < 1e-5,
            "through the point the two cylinders touch at - instead {:?}",
            ends[0].point
        );
        let square = ends[0]
            .inward
            .dot(plans[plan].edges[0].edge.direction())
            .abs();
        assert!(
            (square - 1.0).abs() <= 1e-6,
            "square across the crease - instead {:?}",
            ends[0].inward
        );
    }
}

/// On each of the nose's two faces the ridge's band and the crease's band meet at ONE point - the
/// tangency - which is the whole of what that face is left with at the corner.
#[test]
fn the_two_bands_on_each_nose_face_meet_at_the_one_tangency() {
    let (walked, plans) = the_walkback();

    for (bead, tangency) in [(1usize, the_upper_tangency()), (2, the_lower_tangency())] {
        let bead_left = plans[bead].edges[0].left_region_key();
        let shared = if bead_left == plans[0].edges[0].left_region_key()
            || bead_left == plans[0].edges[0].right_region_key()
        {
            bead_left
        } else {
            plans[bead].edges[0].right_region_key()
        };

        let on_the_cut =
            the_run_ending_at_the_corner(&walked, 0, plans[0].edges[0].left_region_key() == shared);
        let on_the_bead = the_run_ending_at_the_corner(&walked, bead as i32, bead_left == shared);

        assert_eq!(
            on_the_cut, on_the_bead,
            "the two tangent runs on that face end on ONE vertex, pooled once, rather than two a \
             hair apart"
        );
        let at = Vector3::from_vector3_float(walked.mesh.vertices[on_the_cut as usize]);
        assert!(
            (at - tangency).length() < 1e-5,
            "at the tangency - instead {at:?}"
        );
    }
}

/// The WALL keeps its apex: the outline of what it gave up runs from the upper bead's crease foot,
/// along the crease to the tip at (15, 5, 10), and back down the lower crease to its own foot. The
/// blend has still to be laid across that, and the two points between are where the ridge's own
/// tangent lines cross the creases.
///
/// The design, written off the control, quoted this outline as the crease foot, the apex and the
/// mirrored foot. Measured, it carries two more vertices - at (12.5038, 7.4962, 11.6641) and its
/// mirror - which are not corners of the shape but where the band the RIDGE took off its own two
/// faces crosses the creases, and the crease is the boundary those faces share with the wall. They
/// are on the straight run between the foot and the apex, so the outline is the design's; it is
/// split.
#[test]
fn the_wall_keeps_its_apex_between_the_two_crease_feet() {
    let (walked, _) = the_walkback();

    let apex = nose_in_notch_test_meshes::right_ridge_end();
    let on_the_upper_crease = Vector3::new(12.503849, 7.496151, 11.664101);
    let on_the_lower_crease = Vector3::new(12.503849, 7.496151, 8.335899);

    let outline = the_walls_outline_near_the_apex(&walked);
    let described = outline
        .iter()
        .map(|(from, to)| format!("{from:?}-{to:?}"))
        .collect::<Vec<_>>()
        .join(", ");

    for (from, to) in [
        (
            Vector3::new(12.199036, 7.800964, 13.328201),
            the_upper_crease_foot(),
        ),
        (the_upper_crease_foot(), on_the_upper_crease),
        (on_the_upper_crease, apex),
        (apex, on_the_lower_crease),
        (on_the_lower_crease, the_lower_crease_foot()),
        (
            the_lower_crease_foot(),
            Vector3::new(12.199036, 7.800964, 6.671799),
        ),
    ] {
        assert!(
            outline.iter().any(|(edge_from, edge_to)| {
                ((*edge_from - from).length() < 1e-5 && (*edge_to - to).length() < 1e-5)
                    || ((*edge_to - from).length() < 1e-5 && (*edge_from - to).length() < 1e-5)
            }),
            "the wall's outline runs {from:?} to {to:?} - instead it is {described}"
        );
    }

    assert_eq!(
        outline.len(),
        6,
        "and nothing else stands on it near the tip - instead {described}"
    );
}

/// The pass BUILDS, and what it publishes is one closed manifold shell - which is the whole of what
/// the patch between the two beads' rings had to be worth.
#[test]
fn the_build_lays_the_patch_and_publishes_one_closed_shell() {
    let mesh = Arc::new(nose_in_notch_test_meshes::create());
    let features = FeatureEdgeSet::create(&mesh);
    let regions = Arc::new(FaceRegionCache::new(
        mesh.clone(),
        Some(mesh.get_mesh_edge_graph()),
        features.flat_angle_degrees(),
    ));
    let plans = the_plans_of_the_right_ridge_end(&features, &regions, RADIUS, None);

    let result = surface_bevel_builder::try_build(
        &mesh,
        &mesh.get_mesh_edge_graph(),
        &regions,
        &plans,
        16,
        &CancelToken::new(),
        SurfaceBuildOptions::default(),
    )
    .unwrap_or_else(|refused| {
        panic!("the two-crease junction has a patch of its own now - instead {refused:?}")
    })
    .mesh;

    assert!(
        result.is_manifold(),
        "every edge of the patch is a curve one of the three strips already owned, so none of them \
         may be left used once or three times"
    );
    assert_eq!(
        shell_count(&result),
        1,
        "the blend is grown onto the part and nothing of it floats free"
    );
}

/// The walkback of the whole pass, which is what the junction's band ends bound.
fn the_walkback() -> (SurfaceWalkbackResult, Vec<Rc<SurfaceBevelPlan>>) {
    let (_, mesh, plans) = the_junction();
    let (regions, _) = regions_of(&mesh);

    let walked =
        surface_face_walkback::try_apply(&mesh, &mesh.get_mesh_edge_graph(), &regions, &plans)
            .unwrap_or_else(|why| panic!("the walkback refused: {why}"));

    (walked, plans)
}

/// The vertex one edge's tangent run on one side ENDS at, which at this corner is the end standing
/// at the junction - the runs are walked away from it.
fn the_run_ending_at_the_corner(
    walked: &SurfaceWalkbackResult,
    edge_index: i32,
    left: bool,
) -> i32 {
    let runs = &walked.tangent_runs[&(edge_index, left)];
    let corner = nose_in_notch_test_meshes::right_ridge_end();
    let distance = |vertex: i32| {
        (Vector3::from_vector3_float(walked.mesh.vertices[vertex as usize]) - corner).length()
    };
    let mut nearest = -1;

    for run in runs {
        for vertex in [run[0], run[run.len() - 1]] {
            if nearest < 0 || distance(vertex) < distance(nearest) {
                nearest = vertex;
            }
        }
    }

    nearest
}

/// The boundary of what the WALL kept, near the tip: the edges used by exactly one of the faces
/// standing in the wall's own plane, both ends of which are within reach of the corner.
fn the_walls_outline_near_the_apex(walked: &SurfaceWalkbackResult) -> Vec<(Vector3, Vector3)> {
    let at = |vertex: i32| Vector3::from_vector3_float(walked.mesh.vertices[vertex as usize]);
    let in_the_wall = |vertex: i32| {
        let at = at(vertex);

        (at.x + at.y - nose_in_notch_test_meshes::SIZE).abs() < 1e-5
    };

    // Kept in insertion order; only the count reaches an assertion.
    let mut order: Vec<(i32, i32)> = Vec::new();
    let mut used: HashMap<(i32, i32), i32> = HashMap::new();

    for face in walked.mesh.faces.iter() {
        let corners = [face.v0, face.v1, face.v2];

        if !corners.iter().all(|&corner| in_the_wall(corner)) {
            continue;
        }

        for side in 0..3 {
            let one = corners[side];
            let other = corners[(side + 1) % 3];
            let key = (one.min(other), one.max(other));

            let count = used.entry(key).or_insert(0);
            if *count == 0 {
                order.push(key);
            }
            *count += 1;
        }
    }

    let apex = nose_in_notch_test_meshes::right_ridge_end();

    order
        .into_iter()
        .filter(|key| used[key] == 1)
        .map(|(low, high)| (at(low), at(high)))
        .filter(|(from, to)| (*from - apex).length() < 7.0 && (*to - apex).length() < 7.0)
        .collect()
}
