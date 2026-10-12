//! Port of `SurfaceMixedCornerOverACurvedWallTests.cs`: the mixed corner where the wall nobody
//! picked is CURVED - the rim of the flat side the lobe pokes through, whose concave seam is a
//! bead ROLLED onto the lobe's facets (`SurfaceRollAcrossTheWall`).
//!
//! The kernel reading of "Every Chamfer Works.mcx"'s `y = 28` cube face, built from the merged
//! outline alone (`lobed_prism_meshes`). Each cap corner of that face's rim is a mixed corner with
//! the lobe as the wall W, a run of 9 degree facets. The two cylinders are still tangent at one
//! point and the cap still flat; what changes is the TURN: a rolled bead is tangent to the facet
//! its profile rolled to, so the turn is 67.5 degrees and not the 76.5 the seam facet's normal
//! names. The frame: S is `y = 0`, the seam at `x = 10.2277`, the cap `z = 20`; at `r = 3.756`
//! the bead's axis runs at `(7.5282, -3.756)` and `P = (7.5282, 0, 16.244)`.

use std::rc::Rc;
use std::sync::Arc;

use geometry_processing::bevel::bevel_chain_linker;
use geometry_processing::bevel::surface::{
    surface_bevel_builder, surface_mesh_closure, surface_mixed_corner_run_out, SurfaceBevelPlan,
    SurfaceBuildFailure, SurfaceBuildOptions, SurfaceEdgePlan, SurfaceMixedCorner,
    SurfaceMixedCornerWall, SurfaceStrip, SurfaceVertexPool,
};
use geometry_processing::topology::{EdgeConvexity, FaceRegionCache, FeatureEdge, FeatureEdgeSet};
use polygon_mesh::{CancelToken, Mesh};
use vector_math::{math_helper, Vector2, Vector3};

use crate::lobed_prism_meshes;
use crate::mesh_measurements::{shell_count, signed_volume};

/// The fixture's own fillet radius, which is what "Every Chamfer Works.mcx" is filleted at.
const RADIUS: f64 = 3.756;

/// The turn the facet the bead rolled to names: 180 less the void wedge at facet 1.
const TURN_DEGREES: f64 = 67.5;

/// And the turn the seam facet's own normal would have named, which is one facet more.
const TURN_AT_THE_SEAM_FACET_DEGREES: f64 = 76.5;

const EXACT: f64 = 1e-9;

fn assert_within(actual: f64, expected: f64, tolerance: f64, because: &str) {
    assert!(
        (actual - expected).abs() <= tolerance,
        "{actual} is not {expected} within {tolerance}: {because}"
    );
}

/// The bead on the seam really is ROLLED - planned against the lobe, with a band of facets - and
/// tangent to facet 1, a whole facet past the one the seam lies in.
#[test]
fn the_seams_bead_rolls_onto_the_lobe() {
    let read = the_rim();
    let bead = &read.bead;

    assert_eq!(bead.convexity(), EdgeConvexity::Concave);
    assert!(
        bead.left_band.is_some() || bead.right_band.is_some(),
        "a bead planned against a curved wall carries the band of facets it may take material off"
    );

    // The axis: a radius under the flat face, and a radius outside the plane of facet 1.
    assert_within(
        bead.center_line.point.y,
        -RADIUS,
        1e-6,
        "the ball sits in the void a radius off S",
    );
    assert_within(bead.center_line.point.x, 7.5282, 1e-3, "the axis's x");
    assert_within(
        the_seam().x - bead.center_line.point.x,
        2.6995,
        1e-3,
        "which is how far the bead sets back from the seam in the flat face",
    );
}

/// The corner reads its turn from the facet the bead rolled to, so that the patch's stations ARE
/// the bead's own rails: the turn is the bead strip's own sweep, sample for sample.
#[test]
fn the_turn_is_the_beads_own_sweep_and_not_the_seam_facets_normal() {
    let read = the_rim();
    let corner = &read.corner_at_the_cap;
    let turn = math_helper::radians_to_degrees(corner.turn_radians());

    // Within a hundredth of a degree rather than exactly: the facets are read off float vertices.
    assert_within(
        turn,
        TURN_DEGREES,
        0.02,
        "facet 1's outward normal stands at 202.5 degrees",
    );
    assert!(
        turn != TURN_AT_THE_SEAM_FACET_DEGREES,
        "that is the turn the seam facet's own normal names, and the bead does not stand on it"
    );

    // The invariant the patch depends on, read off the bead's OWN rails.
    let bead = SurfaceStrip::try_create(&read.bead, 8)
        .unwrap_or_else(|reason| panic!("the bead has to rule: {reason:?}"));

    assert_within(
        sweep_of(&bead, corner),
        corner.turn_radians(),
        1e-9,
        "the patch turns the cut's ring one bead rail per station, so a turn that is not the bead's sweep lands off every one of them",
    );
}

/// The two cylinders are still tangent at one point on the shared face, and each strip still
/// stops on the plane through it square to the other edge - a curved W changes neither.
#[test]
fn the_two_strips_still_touch_at_one_point_on_the_shared_face() {
    let read = the_rim();
    let corner = &read.corner_at_the_cap;

    assert_within(corner.tangent_point.y, 0.0, 1e-6, "P stands on S");
    assert_within(
        corner.tangent_point.z,
        lobed_prism_meshes::HEIGHT - RADIUS,
        1e-6,
        "a radius under the cap",
    );

    assert_within(
        corner.cut_center_line().distance_to(corner.tangent_point),
        RADIUS,
        1e-4,
        "on the cut's cylinder",
    );
    assert_within(
        corner.axis().distance_to(corner.tangent_point),
        RADIUS,
        1e-4,
        "and on the bead's",
    );

    assert_within(
        corner.outer_rail_centre.z,
        lobed_prism_meshes::HEIGHT,
        EXACT,
        "the outer rail runs in the cap face",
    );

    // And the way out of the axis at the full turn is the way to the bead's own foot on facet 1.
    let foot_of_the_bead = corner.axis().closest_point_to(corner.corner)
        + (corner.out_of_the_axis(corner.turn_radians()) * RADIUS);

    assert_within(
        (foot_of_the_bead - corner.axis().closest_point_to(corner.corner)).length(),
        RADIUS,
        EXACT,
        "the foot is a radius out of the axis",
    );

    assert_within(
        math_helper::radians_to_degrees(
            (-corner.out_of_the_wall)
                .dot(Vector3::new(1.0, 0.0, 0.0))
                .clamp(-1.0, 1.0)
                .acos(),
        ),
        22.5,
        0.02,
        &format!(
            "facet 1's inward normal stands at 22.5 degrees - instead {:?}",
            corner.out_of_the_wall
        ),
    );
}

/// And the two end rings MEET: the build gets as far as sewing the corner at all.
#[test]
fn the_corner_sews_rather_than_refusing_its_own_end_rings() {
    let read = the_rim();

    let built = build(&read, 8);

    assert!(
        built.is_ok(),
        "the patch's stations are the bead's own rails now: {:?}",
        built.err()
    );
}

/// The wall the corner reads is the LOBE's own facets, from the seam outward, and the strip the
/// blend stands over between the bead's stop plane and the cap is taken off facet by facet.
#[test]
fn the_strip_under_the_blend_is_taken_facet_by_facet() {
    let read = the_rim();
    let corner = &read.corner_at_the_cap;
    let wall = the_wall(corner);

    assert_eq!(
        wall.touched_facet, 3,
        "the bead rolls onto the second chord of the lobe, which is its fourth triangle"
    );

    assert_within(
        wall.entry_of(0).x,
        the_seam().x,
        1e-3,
        "the walk starts at the seam itself",
    );

    let pieces = wall.strip_under_the_blend();

    assert_eq!(
        pieces.len(),
        4,
        "facets 0 to 3, the last of them cut at the bead's own foot"
    );

    let mut across = 0.0;

    for (piece, normal) in &pieces {
        assert_eq!(piece.len(), 4);

        // Every corner of the piece in the facet's OWN plane.
        for at in piece {
            assert!(
                (*at - piece[0]).dot(*normal).abs() < 1e-9,
                "{at:?} stands off the facet it is bitten from"
            );
        }

        assert_within(
            (piece[0] - piece[3]).length(),
            RADIUS,
            1e-9,
            "the strip is a radius deep - the stop plane stands that far under the cap",
        );

        across += (piece[1] - piece[0]).length();
    }

    // And the run of them is the section the bead walked: from the seam along the lobe to the foot.
    assert_within(across, 2.4503, 1e-3, "the section the bead walked");

    assert!(
        (pieces[pieces.len() - 1].0[1] - corner.cap_corner()).length() < 1e-9,
        "the far end of the last piece is the bead's foot lifted into the cap"
    );
}

/// Past the bead the trim curve is the meridian circle met with the FACET it is over, and it ends
/// where the outer rail crosses the wall - 42.84 degrees on this chorded wall, not the flat wall's
/// sixty.
#[test]
fn the_trim_curve_follows_the_facets_and_dies_where_the_rail_crosses_them() {
    let read = the_rim();
    let corner = &read.corner_at_the_cap;
    let wall = the_wall(corner);

    let meets = wall
        .try_where_the_rail_meets_the_wall()
        .expect("the rail meets the wall");

    let clears = math_helper::radians_to_degrees(meets - corner.turn_radians());

    assert_within(clears, 42.8435, 1e-3, "where the rail crosses the facets");
    assert!(
        clears < 60.0,
        "sixty is the flat wall's answer, and a wall that bends away from the blend is met sooner"
    );

    // Every station's trim point on the wall: on the facet the ray at that turn lands on, and never
    // further out than the rail itself.
    let past = surface_mixed_corner_run_out::try_turns_past_the_bead(
        corner,
        &wall,
        corner.turn_radians() / 8.0,
        1e-4,
        None,
        None,
    )
    .unwrap_or_else(|reason| panic!("{reason:?}"));

    for &station in &past {
        let at = surface_mixed_corner_run_out::try_scallop_point(corner, &wall, station)
            .expect("the trim point reads");
        let (stands_off, facet) = wall
            .try_out_of_the_axis(corner.turn_radians() + station)
            .expect("the wall stands at that turn");

        assert!(
            (at - wall.entry_of(facet)).dot(wall.normal_of(facet)).abs() < 1e-9,
            "the trim point {at:?} has to stand ON the facet it was read from"
        );

        assert!(
            stands_off < corner.outer_rail_radius() + 1e-9,
            "the wall never stands further out than the rail before the rail has met it"
        );
    }

    let dies = surface_mixed_corner_run_out::try_scallop_point(corner, &wall, past[past.len() - 1])
        .expect("the last trim point reads");

    assert_within(
        (dies - corner.outer_rail_centre).length(),
        corner.outer_rail_radius(),
        1e-6,
        "the curve dies where the ball's touch point on the cap meets the wall",
    );
    assert_within(
        dies.z,
        lobed_prism_meshes::HEIGHT,
        1e-6,
        "which is in the cap face itself",
    );
}

/// And the whole rim CLOSES: one shell, manifold, with the material it takes off coming down on
/// the smooth answer from above as the chording is refined (near 116.36, stated not pinned).
fn the_whole_rim_closes_and_its_volume_comes_down_on_the_smooth_answer(
    segments: i32,
    takes_off: f64,
    coarser: f64,
) {
    let read = the_rim();

    let result = build(&read, segments).unwrap_or_else(|reason| panic!("{reason:?}"));

    let (open, open_at) = surface_mesh_closure::open_edge_count(&result);
    assert_eq!(open, 0, "open at {open_at:?}");
    assert!(result.is_manifold());
    assert_eq!(shell_count(&result), 1);

    let whole = signed_volume(&read.prism);

    let took = whole - signed_volume(&result);

    assert_within(took, takes_off, 1e-3, "what the rim takes off");

    // And each reading takes LESS off than the one before it - the row above in this very table.
    assert!(
        took < coarser,
        "at {segments} segments it took {took}, and half the stations took {coarser}"
    );
}

#[test]
fn the_whole_rim_closes_and_its_volume_comes_down_on_the_smooth_answer_4() {
    the_whole_rim_closes_and_its_volume_comes_down_on_the_smooth_answer(4, 124.2256, f64::MAX);
}

#[test]
fn the_whole_rim_closes_and_its_volume_comes_down_on_the_smooth_answer_8() {
    the_whole_rim_closes_and_its_volume_comes_down_on_the_smooth_answer(8, 118.3609, 124.2256);
}

#[test]
fn the_whole_rim_closes_and_its_volume_comes_down_on_the_smooth_answer_16() {
    the_whole_rim_closes_and_its_volume_comes_down_on_the_smooth_answer(16, 116.8563, 118.3609);
}

#[test]
fn the_whole_rim_closes_and_its_volume_comes_down_on_the_smooth_answer_32() {
    the_whole_rim_closes_and_its_volume_comes_down_on_the_smooth_answer(32, 116.4862, 116.8563);
}

fn build(read: &TheRim, segments: i32) -> Result<Mesh, SurfaceBuildFailure> {
    surface_bevel_builder::try_build(
        &read.prism,
        &read.prism.get_mesh_edge_graph(),
        &read.regions,
        std::slice::from_ref(&read.plan),
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
}

/// The wall as the corner reads it, or a panic where it will not read.
fn the_wall(corner: &Rc<SurfaceMixedCorner>) -> SurfaceMixedCornerWall {
    SurfaceMixedCornerWall::try_read(corner)
        .unwrap_or_else(|failure| panic!("the wall has to read: {:?}", failure.code()))
}

/// How far the strip's own rails sweep about the bead's axis, from the first to the last.
fn sweep_of(strip: &SurfaceStrip, corner: &SurfaceMixedCorner) -> f64 {
    let axis = corner.axis();
    let first = strip.point_on(0, 0.0);
    let last = strip.point_on(strip.rail_count() - 1, 0.0);

    let from_centre = first - axis.closest_point_to(first);
    let to_centre = last - axis.closest_point_to(last);

    from_centre
        .get_normal()
        .dot(to_centre.get_normal())
        .clamp(-1.0, 1.0)
        .acos()
}

/// Where the lobe crosses the flat side - the foot of the concave seam.
fn the_seam() -> Vector2 {
    lobed_prism_meshes::left_crossing()
}

/// C#'s `TheRim` tuple.
struct TheRim {
    prism: Mesh,
    regions: Arc<FaceRegionCache>,
    plan: Rc<SurfaceBevelPlan>,
    bead: SurfaceEdgePlan,
    corner_at_the_cap: Rc<SurfaceMixedCorner>,
}

/// The rim of the flat side's LEFT piece - the one the outline walks first - planned whole, with
/// the mixed corner at its cap end read off the two edges that meet there.
fn the_rim() -> TheRim {
    let prism = lobed_prism_meshes::create();
    let graph = prism.get_mesh_edge_graph();
    let regions = Arc::new(FaceRegionCache::new(
        Arc::new(prism.clone()),
        Some(graph),
        lobed_prism_meshes::THRESHOLD_DEGREES,
    ));
    let features = FeatureEdgeSet::create_with_angle(&prism, lobed_prism_meshes::THRESHOLD_DEGREES);

    let seam = the_seam();

    let rim: Vec<FeatureEdge> = features
        .edges()
        .iter()
        .filter(|edge| on_the_flat_side(edge.start, seam) && on_the_flat_side(edge.end, seam))
        .copied()
        .collect();

    let chains = bevel_chain_linker::link(&rim);
    assert_eq!(chains.len(), 1);

    let plan = SurfaceBevelPlan::try_create(&chains[0], &regions, RADIUS, true)
        .unwrap_or_else(|reason| panic!("the rim has to plan: {reason}"));

    let beads: Vec<&SurfaceEdgePlan> = plan
        .edges
        .iter()
        .filter(|edge| edge.convexity() == EdgeConvexity::Concave)
        .collect();
    assert_eq!(beads.len(), 1);
    let bead = beads[0].clone();

    let at_the_cap = Vector3::new(seam.x, seam.y, lobed_prism_meshes::HEIGHT);

    let cuts: Vec<&SurfaceEdgePlan> = plan
        .edges
        .iter()
        .filter(|edge| {
            edge.convexity() == EdgeConvexity::Convex
                && ((edge.from() - at_the_cap).length() < 1e-3
                    || (edge.to() - at_the_cap).length() < 1e-3)
        })
        .collect();
    assert_eq!(cuts.len(), 1);

    let corner = SurfaceMixedCorner::try_read(
        cuts[0],
        &bead,
        at_the_cap,
        RADIUS,
        SurfaceVertexPool::near_enough_for(&prism.vertices),
    )
    .unwrap_or_else(|refusal| panic!("the corner has to read: {:?}", refusal.code()));

    TheRim {
        prism,
        regions,
        plan,
        bead,
        corner_at_the_cap: Rc::new(corner),
    }
}

/// Whether a point stands on the flat side's left piece, which is what the rim is picked by.
fn on_the_flat_side(point: Vector3, seam: Vector2) -> bool {
    point.y.abs() < 1e-4 && point.x < seam.x + 0.01
}
