//! Port of `SurfaceMixedJunctionOverACurvedWallTests.cs`: the JUNCTION on a CHORDED seam - the
//! lobed prism's own, where the flat side's top rim, the barrel's top rim and the seam between
//! them all meet at one vertex and all three are picked.
//!
//! The L prism's junction is the same corner with every face flat
//! (`surface_mixed_three_edge_corner_tests`). Here the bead ROLLS (the turn is 67.49 degrees, not
//! the 76.5 the seam facet's normal names), the barrel arm's cut is not the chord arriving at the
//! corner (the chords between are swallowed), and that chord hands over on the MITRE ring before
//! it, which leans. The frame: `lobed_prism_meshes` at `r = 3.756`, junction vertex
//! `V = (10.2277, 0, 20)`, bead axis at `(7.5282, -3.756, z)`, tangencies
//! `P = (7.5282, 0, 16.244)` and `Q = (10.9979, -2.3178, 16.244)`.

use std::rc::Rc;
use std::sync::Arc;

use geometry_processing::bevel::bevel_chain_linker;
use geometry_processing::bevel::surface::{
    surface_bevel_builder, surface_mesh_closure, surface_mixed_corner_ring_schedule,
    SurfaceBevelPlan, SurfaceBuildOptions, SurfaceMixedJunction, SurfaceVertexPool,
};
use geometry_processing::topology::{FaceRegionCache, FeatureEdge, FeatureEdgeSet};
use polygon_mesh::{CancelToken, Mesh};
use vector_math::{math_helper, Vector3};

use crate::lobed_prism_meshes;
use crate::mesh_measurements::{shell_count, signed_volume};
use crate::surface_test_chains::join_and_open;

/// The fixture's own fillet radius, which is what "Every Chamfer Works.mcx" is filleted at.
const RADIUS: f64 = 3.756;

/// The turn the facet the bead rolled to names, which both arms have to read the same.
const TURN_DEGREES: f64 = 67.4868;

/// How far a rail reused across the seam may stand from where the turn puts it: the lobe's own
/// chord sagitta, which is what the published mesh already stands off the true barrel there.
const CHORD_SAGITTA: f64 = 0.031;

/// How far the patch's worst vertex really stands off the ideal torus, at eight segments: two
/// orders inside `CHORD_SAGITTA`, and pinned so that a drift in the grid is visible as one.
const WORST_OFF_THE_TORUS: f64 = 1.2833e-4;

fn the_corner() -> Vector3 {
    Vector3::new(
        lobed_prism_meshes::left_crossing().x,
        0.0,
        lobed_prism_meshes::HEIGHT,
    )
}

/// The junction READS on a chorded seam: one bead, one axis, one turn - with one arm handing over
/// on its own tangency and the other on a mitre ring two chords along.
#[test]
fn the_chorded_junction_reads_as_two_arms_of_one_bead() {
    let (junction, _, _, _) = the_junction();

    // Both arms read the ROLLED bead's own turn, not the seam facet's normal, and they read it the
    // same: the two are the same pair of feet measured from either end.
    let turn = math_helper::radians_to_degrees(junction.turn_radians());
    assert!((turn - TURN_DEGREES).abs() <= 0.01, "instead {turn}");
    assert!(
        (junction.first.mixed.turn_radians() - junction.second.mixed.turn_radians()).abs() < 1e-9,
        "one bead, read from either side, turns through one angle"
    );

    // The ball the flat arm hands over at IS the ball the barrel arm starts from.
    assert!(
        (junction
            .first
            .mixed
            .ball_centre_at(junction.first.mixed.turn_radians())
            - junction.second.mixed.ball_centre_at(0.0))
        .length()
            < 1e-6
    );

    let (flat, barrel) = if junction.first.mixed.rolled_is_the_shared_face() {
        (&junction.second, &junction.first)
    } else {
        (&junction.first, &junction.second)
    };

    // The FLAT arm: its own chain's one edge, nothing swallowed, handing over on the tangency.
    assert_eq!(flat.swallowed.len(), 0);
    assert!(!flat.mixed.hands_over_at_a_mitre);
    assert!(
        flat.mixed
            .tangent_point
            .equals_within(Vector3::new(7.5282, 0.0, 16.244), 1e-3),
        "instead {:?}",
        flat.mixed.tangent_point
    );

    // The BARREL arm: the cut is two chords along, both of them swallowed, and the ring is the
    // mitre ring before it, which leans.
    assert_eq!(barrel.cut_index, 2);
    assert_eq!(
        barrel.swallowed.len(),
        2,
        "the ball reaches over the chords between"
    );
    assert!(
        barrel.mixed.hands_over_at_a_mitre,
        "the chord that touches has no band of its own left to carry a ring"
    );
    assert!(
        barrel
            .mixed
            .tangent_point
            .equals_within(Vector3::new(10.9979, -2.3178, 16.244), 1e-3),
        "instead {:?}",
        barrel.mixed.tangent_point
    );

    // Both tangencies stand a radius under the cap, which is what lets one stop plane cut the
    // bead for both arms.
    assert!((flat.mixed.tangent_point.z - (lobed_prism_meshes::HEIGHT - RADIUS)).abs() <= 1e-3);
    assert!((barrel.mixed.tangent_point.z - (lobed_prism_meshes::HEIGHT - RADIUS)).abs() <= 1e-3);
}

/// And it BUILDS: closed, manifold, one shell at every sampling.
fn the_chorded_junction_closes(segments: i32) {
    let built = the_build(segments);

    let (closed, open_at) = surface_mesh_closure::is_closed(&built);
    assert!(
        closed,
        "nothing may be left open, and something is at {open_at:?}"
    );
    assert!(built.is_manifold());
    assert_eq!(shell_count(&built), 1);
}

#[test]
fn the_chorded_junction_closes_4() {
    the_chorded_junction_closes(4);
}

#[test]
fn the_chorded_junction_closes_8() {
    the_chorded_junction_closes(8);
}

#[test]
fn the_chorded_junction_closes_16() {
    the_chorded_junction_closes(16);
}

/// What it removes converges from BELOW as the sampling doubles - the chords of the patch and of
/// the strips alike fall inside the surfaces they stand in, so a coarser patch takes more.
#[test]
fn what_the_junction_removes_converges_as_the_sampling_doubles() {
    let mut last = 0.0;

    for segments in [4, 8, 16] {
        let volume = signed_volume(&the_build(segments)).abs();

        assert!(
            volume > last,
            "a finer patch leaves more standing - {segments} gave {volume} against {last}"
        );
        assert!(
            volume < lobed_prism_meshes::HEIGHT * 1000.0,
            "and never more than the part itself - instead {volume}"
        );

        last = volume;
    }
}

/// Every vertex of the patch stands on the blend's own torus to within the lobe's OWN chord
/// sagitta, and the measured worst is pinned.
#[test]
fn the_patch_stands_on_the_torus_to_within_the_lobes_own_chords() {
    let (junction, prism, regions, plans) = the_junction();
    let axis = junction.axis();
    let centre = junction.outer_rail_centre();
    let mut worst = 0.0_f64;

    let built = build(&prism, &regions, &plans, 8)
        .unwrap_or_else(|reason| panic!("the build refused the junction: {reason}"));

    for face in built.faces.iter() {
        for vertex in [face.v0, face.v1, face.v2] {
            let at = Vector3::from_vector3_float(built.vertices[vertex as usize]);

            // Only the patch's own vertices: between the cap and the bead's own stop plane a radius
            // under it, inside the ball's reach, and strictly WITHIN the turn - which is what tells
            // the patch from the three strips, every one of which stands at one end of it or the
            // other.
            if at.z < lobed_prism_meshes::HEIGHT - RADIUS - 1e-6
                || (at - axis.closest_point_to(at)).length() > 2.0 * RADIUS
            {
                continue;
            }

            let angle = surface_mixed_corner_ring_schedule::angle_of(&junction.first.mixed, at);

            if angle < 0.02 || angle > junction.turn_radians() - 0.02 {
                continue;
            }

            let out_of_the_axis = (at - axis.closest_point_to(at)).length();

            let along_the_cap =
                (at - centre).dot(junction.first.mixed.out_of_the_cap_face) + RADIUS;
            let from_the_centre_circle = (((out_of_the_axis - (2.0 * RADIUS))
                * (out_of_the_axis - (2.0 * RADIUS)))
                + (along_the_cap * along_the_cap))
                .sqrt();

            worst = worst.max((from_the_centre_circle - RADIUS).abs());
        }
    }

    assert!(
        worst < CHORD_SAGITTA,
        "the patch's own vertices stand on the torus to within the lobe's chords - instead {worst}"
    );

    // And the MEASURED number, not merely a positive one: a drift is a fact about the build to be
    // explained, not a pin to be widened.
    assert!(
        (worst - WORST_OFF_THE_TORUS).abs() <= 1e-6,
        "the reading really did find the patch and it stands where it stood - instead {worst}"
    );
}

fn build(
    prism: &Mesh,
    regions: &FaceRegionCache,
    plans: &[Rc<SurfaceBevelPlan>],
    segments: i32,
) -> Result<Mesh, String> {
    surface_bevel_builder::try_build(
        prism,
        &prism.get_mesh_edge_graph(),
        regions,
        plans,
        segments,
        &CancelToken::new(),
        SurfaceBuildOptions {
            crossings: None,
            ran_off_over_a_stretch: None,
            lay_the_oblique_rows_anyway: false,
            planless_loops: &[],
            features: None,
        },
    )
    .map(|built| built.mesh)
    .map_err(|reason| format!("{reason:?}"))
}

/// The prism built with all three edges at the seam's cap corner picked.
fn the_build(segments: i32) -> Mesh {
    let (_, prism, regions, plans) = the_junction();

    build(&prism, &regions, &plans, segments).unwrap_or_else(|reason| {
        panic!("the build refused the junction at {segments} segments: {reason}")
    })
}

/// The junction as the pass reads it, and the plans it was read from - which the reading alone
/// leaves UNTOUCHED: only `try_apply_to` records the corner on them, and the build does that for
/// itself (`try_read_all`).
fn the_junction() -> (
    SurfaceMixedJunction,
    Mesh,
    Arc<FaceRegionCache>,
    Vec<Rc<SurfaceBevelPlan>>,
) {
    let prism = lobed_prism_meshes::create();
    let graph = prism.get_mesh_edge_graph();

    let regions = Arc::new(FaceRegionCache::new(
        Arc::new(prism.clone()),
        Some(graph),
        lobed_prism_meshes::THRESHOLD_DEGREES,
    ));

    let features = FeatureEdgeSet::create_with_angle(&prism, lobed_prism_meshes::THRESHOLD_DEGREES);

    let plans: Vec<Rc<SurfaceBevelPlan>> =
        join_and_open(&bevel_chain_linker::link(&three_seam_edges(&features)))
            .iter()
            .map(|chain| {
                SurfaceBevelPlan::try_create(chain, &regions, RADIUS, true)
                    .unwrap_or_else(|reason| panic!("the plan refused a chain: {reason}"))
            })
            .collect();

    let junction = SurfaceMixedJunction::try_read(
        &plans,
        the_vertex(&plans),
        the_corner(),
        SurfaceVertexPool::near_enough_for(&prism.vertices),
    )
    .unwrap_or_else(|reason| panic!("the junction would not read: {reason:?}"));

    (junction, prism, regions, plans)
}

/// The three picked runs: the flat side's top rim up to the seam, the seam itself, and the
/// barrel's whole top rim.
fn three_seam_edges(features: &FeatureEdgeSet) -> Vec<FeatureEdge> {
    let seam = lobed_prism_meshes::left_crossing();
    let mut picked = Vec::new();

    for edge in features.edges() {
        let on_the_cap = (edge.start.z - lobed_prism_meshes::HEIGHT).abs() < 1e-4
            && (edge.end.z - lobed_prism_meshes::HEIGHT).abs() < 1e-4;

        let on_the_flat_side = edge.start.y.abs() < 1e-4 && edge.end.y.abs() < 1e-4;

        // The lobe pokes out to NEGATIVE y, so its own rim is the only run of the cap's that leaves
        // the rectangle - which is what tells the barrel's top rim from the other three sides'.
        let on_the_lobe = edge.start.y < -1e-4 || edge.end.y < -1e-4;

        let is_the_seam = (edge.start.x - seam.x).abs() < 1e-3
            && (edge.end.x - seam.x).abs() < 1e-3
            && on_the_flat_side;

        if is_the_seam
            || (on_the_cap && on_the_lobe)
            || (on_the_cap
                && on_the_flat_side
                && edge.start.x < seam.x + 0.01
                && edge.end.x < seam.x + 0.01)
        {
            picked.push(*edge);
        }
    }

    picked
}

/// The welded mesh's own index for the corner, off the plans that meet there.
fn the_vertex(plans: &[Rc<SurfaceBevelPlan>]) -> i32 {
    for plan in plans {
        for edge in &plan.edges {
            if (edge.from() - the_corner()).length() < 1e-3 {
                return edge.edge.from_vertex();
            }

            if (edge.to() - the_corner()).length() < 1e-3 {
                return edge.edge.to_vertex();
            }
        }
    }

    panic!("no picked edge stands at {:?}", the_corner());
}
