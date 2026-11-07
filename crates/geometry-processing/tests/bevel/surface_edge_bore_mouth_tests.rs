//! The mouth DRAPED OVER A CONVEX EDGE - a 20 mm block
//! with a 40 sided bore of radius 5 drilled at 45 degrees through its own corner
//! (`bore_test_meshes_edge_bore`), so each mouth crosses the block's 90 degree crease twice. The
//! chain's two neighbours at a crossing land on faces 90 degrees apart while the rim goes on being
//! a chorded curve, and the crossing carries the LONGEST chord of the loop and the SHALLOWEST
//! setback on it - so the drape is not what stops "Inner Face Should Bevel.mcx".
//!
//! `TheDrapedMouthTakesTheFillet` is not run, still open: it plans and publishes, but the
//! answer is open in 24 places down the wall below the mouth. The parameter rows are a loop.

use geometry_processing::bevel::surface::{surface_mesh_closure, SurfaceEdgePlan};
use geometry_processing::topology::{
    bevel_edge_group_resolver, BevelEdgeGroup, FaceGroup, FeatureEdgeSet,
};
use polygon_mesh::{CancelToken, Mesh};
use vector_math::{math_helper, Vector3};

use crate::bore_test_meshes_edge_bore::{
    create_the_edge_bore_fixture, edge_bore_drill_axis, EDGE_BORE_BLOCK, EDGE_BORE_RADIUS,
};
use crate::mesh_measurements::shell_count;
use crate::surface_test_build::{try_bevel, TryBevelOptions};

const RADIUS: f64 = EDGE_BORE_RADIUS;

/// Half the block, which is where both of the crossings stand on every axis.
const HALF: f64 = EDGE_BORE_BLOCK / 2.0;

const SEGMENTS: i32 = 8;

/// The part's own ten degree bend threshold, at which a 40 sided bore is one surface.
const BEND_THRESHOLD_DEGREES: f64 = 10.0;

/// The fixture itself, before any bevel: a closed solid whose mouth crosses the block's crease at
/// two of its own vertices, exactly.
#[test]
fn the_bore_is_drilled_clean_through_the_blocks_edge() {
    let part = create_the_edge_bore_fixture();

    let (open, open_at) = surface_mesh_closure::open_edge_count(&part);
    assert_eq!(
        open, 0,
        "the six block faces and the bore close on each other, and they do not at {open_at:?}"
    );
    assert!(part.is_manifold());
    assert_eq!(shell_count(&part), 1);

    let standing_on_the_crease = part
        .vertices
        .iter()
        .map(|vertex| vertex.as_vector3())
        .filter(|at| (at.x - HALF).abs() < 1e-9 && (at.z - HALF).abs() < 1e-9)
        .count();

    // The mouth's two minor axis ends and the crease's own two block corners.
    assert_eq!(
        standing_on_the_crease, 4,
        "the upper mouth crosses the crease at two of its own vertices, (10, +-5, 10)"
    );
}

/// The numbers at the crossing, at the part's own 2 mm and at the 1 mm Lars also tries: what the
/// two chords either side of it are, how far each of their mitres eats along them, and that the two
/// of them land on faces a right angle apart.
#[test]
fn the_draped_mouths_crossing_carries_its_longest_chord() {
    for (size, setback) in [(2.0, 2.114214), (1.0, 1.057107)] {
        let part = create_the_edge_bore_fixture();
        let features = FeatureEdgeSet::create_with_graph(
            &part,
            &part.get_mesh_edge_graph(),
            BEND_THRESHOLD_DEGREES,
        );
        let (group, rim_edges) = the_bore_walls_rim(&part, &features);

        // Both mouths, 40 chords each, where the part's own boolean leaves 116 on the same two
        // loops.
        assert_eq!(rim_edges, 80);

        let plans = try_bevel(
            &part,
            &[group],
            size,
            SEGMENTS,
            &CancelToken::new(),
            TryBevelOptions::default(),
        )
        .plans_built;

        assert_eq!(plans.len(), 2);

        for plan in &plans {
            assert_eq!(plan.edges.len(), 40);
            assert!(plan.is_closed);
        }

        let at_the_crossing: Vec<&SurfaceEdgePlan> = plans
            .iter()
            .flat_map(|plan| plan.edges.iter())
            .filter(|edge| stands_on_a_crossing(edge.from()) || stands_on_a_crossing(edge.to()))
            .collect();

        // Four crossings, two chords running into each.
        assert_eq!(at_the_crossing.len(), 8);

        for edge in &at_the_crossing {
            let chord = (edge.to() - edge.from()).length();

            assert!(
                (chord - 1.10787).abs() <= 1e-5,
                "the crossing's two chords are the longest of the loop, and equal - {chord}"
            );
            assert!(
                (edge.left_setback - setback).abs() <= 1e-5,
                "{} against {setback} at {size}",
                edge.left_setback
            );
            assert!(
                (edge.right_setback - setback).abs() <= 1e-5,
                "{} against {setback} at {size}",
                edge.right_setback
            );
        }

        // The two that meet at one crossing lie on the top face and the side face, a right angle
        // apart - which is the one thing this fixture adds to a mouth.
        let faces: Vec<Vector3> = at_the_crossing
            .iter()
            .map(|edge| {
                let (from, to) = (edge.from(), edge.to());

                if (from.z - to.z).abs() < 1e-9 {
                    Vector3::new(0.0, 0.0, sign(from.z + to.z))
                } else {
                    Vector3::new(sign(from.x + to.x), 0.0, 0.0)
                }
            })
            .collect();

        for pair in faces.chunks(2) {
            let degrees = math_helper::radians_to_degrees(pair[0].dot(pair[1]).acos());

            assert!((degrees - 90.0).abs() <= 1e-6, "{degrees}");
        }
    }
}

/// The sign as a double: -1, 0 or 1.
fn sign(value: f64) -> f64 {
    if value > 0.0 {
        1.0
    } else if value < 0.0 {
        -1.0
    } else {
        0.0
    }
}

/// Whether a point is one of the four places a mouth crosses a block crease.
fn stands_on_a_crossing(at: Vector3) -> bool {
    (at.x.abs() - HALF).abs() < 1e-9
        && (at.z.abs() - HALF).abs() < 1e-9
        && (at.y.abs() - RADIUS).abs() < 1e-9
}

/// The FACE PICK the part's own file carries: a coplanar-region flood off one bore wall facet,
/// which at the ten degree bend threshold takes the whole wall and hands back its rim - both
/// mouths. Answers the group and how many rim edges the region has.
fn the_bore_walls_rim(part: &Mesh, features: &FeatureEdgeSet) -> (BevelEdgeGroup, usize) {
    let wall = part
        .faces
        .iter()
        .position(|face| {
            [face.v0, face.v1, face.v2].iter().all(|&vertex| {
                off_the_axis_by_a_radius(part.vertices[vertex as usize].as_vector3())
            })
        })
        .expect("the fixture carries no bore wall facet");

    let region = FaceGroup::create_with_graph(
        part,
        &part.get_mesh_edge_graph(),
        wall as i32,
        FaceGroup::DEFAULT_CURVE_TOLERANCE_DEGREES,
    )
    .expect("the wall facet is a face of the part");

    let rim_edges = region.boundary_edges().len();
    let group = bevel_edge_group_resolver::create_from_face_group(
        &region,
        &region.boundary_edges()[0],
        features.flat_angle_degrees(),
    )
    .expect("the rim's edge is the region's own");

    (group, rim_edges)
}

/// Whether a point stands exactly a bore radius off the drill's axis, as the wall's do.
fn off_the_axis_by_a_radius(at: Vector3) -> bool {
    let axis = edge_bore_drill_axis();

    ((at - (axis * at.dot(axis))).length() - RADIUS).abs() < 1e-6
}
