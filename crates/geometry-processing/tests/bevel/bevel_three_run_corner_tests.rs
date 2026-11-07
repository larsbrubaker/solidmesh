//! Three runs meeting at one corner where their setbacks are NOT equal - the rolling ball vertex
//! blend on a 45 degree knife tip, the corner "Bevel to Center Feature.mcx" measures against
//! Onshape. The tip's tool touches down `R / tan(22.5) = 8.208` along its faces while the top edges
//! set back only `R = 3.4`; a point on a 90 degree edge retreats `R(sqrt(2) - 1)` under its fillet,
//! so the top edge at the tip's own setback has to be 1.408 below where it started. At an
//! ALL-CONVEX corner of exactly three runs each run drives one ring PAST the centre plane
//! (`bevel_corner_seam`, `bevel_overshoot_rings`), so the patch's wall is interior to the union
//! rather than coincident with a cap.

use geometry_processing::bevel::bevel_arc_chording::effective_segments;
use geometry_processing::bevel::bevel_corner_patch;
use geometry_processing::bevel::bevel_group_tools::{self, BevelGroupToolsOptions};
use geometry_processing::topology::bevel_edge_group_resolver;
use geometry_processing::topology::{
    BevelEdgeGroup, BevelEdgeGroupResolution, FeatureEdge, FeatureEdgeSet,
};
use polygon_mesh::csg::CsgModes;
use polygon_mesh::Mesh;
use std::collections::HashSet;
use vector_math::math_helper::degrees_to_radians;
use vector_math::{Vector2, Vector3};

use crate::kernel_cut_helpers::{boolean, edge_group};
use crate::mesh_surface_distance;
use crate::topology_test_meshes::create_isosceles_prism;

const RADIUS: f64 = 3.4;
const SEGMENTS: i32 = 8;
const APEX_DEGREES: f64 = 45.0;
const LENGTH: f64 = 20.0;
const HEIGHT: f64 = 20.0;

/// Two chordings of one surface disagree by the sum of their sagittas, and this rebuild steps
/// 11.25 degrees round a radius of 3.4 (0.02) and the same round the ball. A fifth of a millimetre
/// leaves room for both and is still a seventh of the 1.408 the corner is being held to.
const SURFACE_TOLERANCE: f64 = 0.2;

/// The top of the prism, and so the z of the corner under test.
fn top() -> f64 {
    HEIGHT / 2.0
}

/// Half the prism's base, which is where its two top edges end.
fn spread() -> f64 {
    LENGTH * (degrees_to_radians(APEX_DEGREES) / 2.0).tan()
}

/// How far a 45 degree tip's tool touches down along each of its two faces.
fn tip_setback() -> f64 {
    RADIUS / (degrees_to_radians(APEX_DEGREES) / 2.0).tan()
}

/// How far a point on a 90 degree edge retreats under its own fillet.
fn ninety_degree_retreat() -> f64 {
    RADIUS * (2.0_f64.sqrt() - 1.0)
}

#[test]
fn the_three_runs_meet_in_the_rolling_ball_at_a_knife_tip() {
    let prism = create_isosceles_prism(APEX_DEGREES, LENGTH, HEIGHT);
    let result =
        round_the_top_corner(&prism).expect("the three runs and their corner have to build at all");

    // The headline number. The ball's centre projects onto each top edge exactly at the TIP's
    // setback, so this is the station where the cylinder hands over to the sphere - and whichever
    // of the two arrives, the edge has retreated by the same R(sqrt(2) - 1).
    for (name, along) in [("upper", 1.0), ("lower", -1.0)] {
        let on_the_edge = top_edge_point(along, tip_setback());
        let retreat = mesh_surface_distance::from(&result, on_the_edge);

        assert!(
            (retreat - ninety_degree_retreat()).abs() <= SURFACE_TOLERANCE,
            "the {name} top edge at the tip's own setback of {:.3} from the corner has to have \
             retreated {:.3} under the blend, and it measures {retreat:.3}. Zero here means the \
             original sharp edge is still standing - the runs stopped at each other's tangent lines \
             and nothing filled the corner between them",
            tip_setback(),
            ninety_degree_retreat()
        );
    }

    // And the shape as a whole: nothing in the corner may be anywhere except on the ball, on one
    // of the three cylinders, or on one of the three flat faces it is all tangent to.
    let centre = ball_centre();
    let mut strays: Vec<(Vector3, f64)> = result
        .vertices
        .iter()
        .map(|vertex| vertex.as_vector3())
        .filter(|&vertex| (vertex - Vector3::new(0.0, 0.0, top())).length() <= 3.0 * RADIUS)
        .map(|vertex| (vertex, off_the_blend(vertex, centre)))
        .filter(|&(_, off)| off > SURFACE_TOLERANCE)
        .collect();
    strays.sort_by(|a, b| b.1.total_cmp(&a.1));

    assert_eq!(
        strays.len(),
        0,
        "every vertex near the corner has to lie on the rolling ball of radius {RADIUS:.3} about \
         {centre:?}, on one of the three cylinders that run into it, or on one of the three faces \
         they are tangent to. These are on none of them: {:?}",
        strays.iter().take(6).collect::<Vec<_>>()
    );
}

/// The seam: every vertex of the corner patch that sits on the ball-centre plane of a run is also
/// a vertex of that run's own last ring. The knife tip's 135 degree arc pulls the whole operation
/// to 12 segments, so the patch's arcs are sampled at 12 as well and land exactly on the runs'
/// points.
#[test]
fn the_patch_and_the_runs_share_their_vertices_on_the_seam() {
    let prism = create_isosceles_prism(APEX_DEGREES, LENGTH, HEIGHT);
    let features = FeatureEdgeSet::create(&prism);
    let (resolutions, _) =
        bevel_edge_group_resolver::resolve_all(&the_three_groups(&features), &prism);
    let eligible: Vec<bool> = resolutions.iter().map(|r| r.is_resolved()).collect();
    let all_edges: Vec<FeatureEdge> = selected(&resolutions);
    let effective = effective_segments(Some(&all_edges), SEGMENTS);

    assert!(
        effective > SEGMENTS,
        "the fixture only says anything if its {APEX_DEGREES:.3} degree tip really does pull the \
         operation past its own {SEGMENTS} - it answered {effective}"
    );

    let patches = bevel_corner_patch::create(&resolutions, &eligible, true, RADIUS, effective);
    let drive_through: HashSet<i32> = patches.runs_may_drive_through.iter().copied().collect();

    assert_eq!(
        patches.patches.len(),
        1,
        "the three runs meet at one corner, which is the seam under test"
    );

    let patch_points: Vec<Vector3> = patches.patches[0]
        .mesh
        .vertices
        .iter()
        .map(|v| v.as_vector3())
        .collect();
    let mut shared = 0;

    for resolution in &resolutions {
        let built = bevel_group_tools::try_build(
            resolution,
            true,
            RADIUS,
            effective,
            BevelGroupToolsOptions {
                selected_across_groups: Some(&all_edges),
                patched_corners: Some(&patches.ball_centres),
                runs_may_drive_through: Some(&drive_through),
                patch_seam_rings: Some(&patches.seam_rings),
                ..BevelGroupToolsOptions::default()
            },
        )
        .unwrap_or_else(|refusal| panic!("{:?}", refusal.reason));

        let runs: Vec<&Mesh> = built
            .tools
            .iter()
            .filter(|tool| !tool.is_bead)
            .map(|tool| &tool.mesh)
            .collect();
        assert_eq!(runs.len(), 1, "one cutter per run");
        assert_eq!(patches.ball_centres.len(), 1);
        let centre = *patches
            .ball_centres
            .values()
            .next()
            .expect("one ball centre");

        // The run's own ring on the ball-centre plane: its vertices at that plane's distance along
        // the run. Each has to be a vertex of the patch too, or the two solids do not meet there.
        for vertex in runs[0].vertices.iter().map(|v| v.as_vector3()) {
            if (vertex - centre).dot(resolution.edges[0].direction).abs() > 1e-6 {
                continue;
            }

            if patch_points
                .iter()
                .any(|&point| (point - vertex).length() < 1e-5)
            {
                shared += 1;
            }
        }
    }

    assert!(
        shared > 0,
        "the patch and the runs have to carry the SAME points where they meet - a run ring on the \
         ball-centre plane with no matching patch vertex is two polylines on one circle, which is \
         the seam failing to meet rather than a corner meeting badly"
    );
}

/// The ball: the one point at RADIUS from all three faces. On the bisector of the tip's two faces
/// at `R / sin(half the apex)` from the tip's own edge, and one radius under the top.
fn ball_centre() -> Vector3 {
    Vector3::new(
        RADIUS / (degrees_to_radians(APEX_DEGREES) / 2.0).sin(),
        0.0,
        top() - RADIUS,
    )
}

/// A point on one of the two top edges, measured from the corner.
fn top_edge_point(side: f64, from_the_corner: f64) -> Vector3 {
    let direction = Vector3::new(LENGTH, side * spread(), 0.0).get_normal();
    Vector3::new(0.0, 0.0, top()) + (direction * from_the_corner)
}

/// How far a point is from the surface the corner is supposed to be: the ball, the three
/// cylinders, or the three planes. Stated in closed form so a failure names the shape rather than
/// a distance.
fn off_the_blend(vertex: Vector3, centre: Vector3) -> f64 {
    // The three faces: the top, and the tip's two sides.
    let mut best = (vertex.z - top()).abs();

    for side in [1.0, -1.0] {
        let face_normal = Vector3::new(-spread(), side * LENGTH, 0.0).get_normal();
        best = best.min(vertex.dot(face_normal).abs());

        // That side's top edge cylinder: one radius in from the face and one below the top.
        let axis_point =
            Vector3::new(0.0, 0.0, top()) - (face_normal * RADIUS) - (Vector3::UNIT_Z * RADIUS);
        let axis_direction = Vector3::new(LENGTH, side * spread(), 0.0).get_normal();
        let off_axis = vertex - axis_point;
        best = best.min(
            ((off_axis - (axis_direction * off_axis.dot(axis_direction))).length() - RADIUS).abs(),
        );
    }

    // The vertical tip cylinder, whose axis runs up through the ball's centre in plan.
    let in_plan = Vector2::new(vertex.x - centre.x, vertex.y - centre.y);
    best = best.min((in_plan.length() - RADIUS).abs());

    // And the ball itself.
    best.min(((vertex - centre).length() - RADIUS).abs())
}

/// The vertical tip edge and the two top edges, each its own group.
fn the_three_groups(features: &FeatureEdgeSet) -> Vec<BevelEdgeGroup> {
    vec![
        edge_group(
            features,
            Vector3::new(0.0, 0.0, -top()),
            Vector3::new(0.0, 0.0, top()),
        ),
        edge_group(
            features,
            Vector3::new(0.0, 0.0, top()),
            Vector3::new(LENGTH, spread(), top()),
        ),
        edge_group(
            features,
            Vector3::new(0.0, 0.0, top()),
            Vector3::new(LENGTH, -spread(), top()),
        ),
    ]
}

fn selected(resolutions: &[BevelEdgeGroupResolution]) -> Vec<FeatureEdge> {
    resolutions
        .iter()
        .flat_map(|resolution| resolution.edges.iter().copied())
        .collect()
}

/// Selects the three edges meeting at the top corner, builds their tools and the corner patch
/// exactly as a rebuild does, and takes the union of the lot out of the part.
fn round_the_top_corner(prism: &Mesh) -> Option<Mesh> {
    let features = FeatureEdgeSet::create(prism);
    let (resolutions, _) =
        bevel_edge_group_resolver::resolve_all(&the_three_groups(&features), prism);
    let eligible: Vec<bool> = resolutions.iter().map(|r| r.is_resolved()).collect();
    let patches = bevel_corner_patch::create(&resolutions, &eligible, true, RADIUS, SEGMENTS);
    let all_edges = selected(&resolutions);
    let drive_through: HashSet<i32> = patches.runs_may_drive_through.iter().copied().collect();
    let mut solids: Vec<Mesh> = Vec::new();

    for resolution in &resolutions {
        let built = bevel_group_tools::try_build(
            resolution,
            true,
            RADIUS,
            SEGMENTS,
            BevelGroupToolsOptions {
                selected_across_groups: Some(&all_edges),
                patched_corners: Some(&patches.ball_centres),
                runs_may_drive_through: Some(&drive_through),
                patch_seam_rings: Some(&patches.seam_rings),
                ..BevelGroupToolsOptions::default()
            },
        )
        .ok()?;
        solids.extend(built.tools.into_iter().map(|tool| tool.mesh));
    }

    solids.extend(patches.patches.iter().map(|patch| patch.mesh.clone()));

    let cutter = if solids.len() == 1 {
        solids.remove(0)
    } else {
        boolean(&solids.iter().collect::<Vec<_>>(), CsgModes::Union)
    };

    Some(boolean(&[prism, &cutter], CsgModes::Subtract))
}
