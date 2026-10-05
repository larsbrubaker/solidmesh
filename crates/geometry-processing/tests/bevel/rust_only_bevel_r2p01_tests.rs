//! Rust-only checks of the R2.P01 package on a SQUARE mixed corner (the R1.P12 fixture - corner at
//! (7, 7, 7), size 1, cap normal +z - with the bead's axis stood on +z through the origin): the carries into the cap and down
//! to the bead's end, which plane bounds each edge's band, the wall read off the corner and its
//! strip under the blend, the run-out's rigid carry, and the band ends at a junction and at a
//! chorded mitre; a chain's band ends and its chorded run's joint; which chord carries a mixed
//! corner's ring, and a strip's end ring cut and won from the pool.

use std::collections::HashMap;
use std::rc::Rc;

use indexmap::IndexMap;

use vector_math::Vector3;

use geometry_processing::bevel::surface::surface_handover_leaves_a_strip::leaves_a_band_of;
use geometry_processing::bevel::surface::surface_mitre_joint_step::SideSeams;
use geometry_processing::bevel::surface::surface_mitre_on_a_chorded_run::{
    try_the_point, where_the_neighbours_plane_crosses,
};
use geometry_processing::bevel::surface::surface_mixed_corner_bites::under_the_cap;
use geometry_processing::bevel::surface::surface_mixed_corner_hands_over_at_the_mitre as hands_over;
use geometry_processing::bevel::surface::surface_mixed_corner_patch::{try_cut_the_ring, try_ring};
use geometry_processing::bevel::surface::surface_mixed_corner_which_canal::{
    the_corner_takes_the_dual, try_how_far_the_duals_crease_runs,
};
use geometry_processing::bevel::surface::surface_plan_band_ends::{
    band_end_where_the_chain_arrives, band_end_where_the_chain_leaves,
};
use geometry_processing::bevel::surface::{
    surface_junction_band_ends, SurfaceBandEnd, SurfaceMixedCornerFaces, SurfaceMixedCornerWall,
    SurfaceRunOutCarry, SurfaceVertexPool,
};
use geometry_processing::bevel::surface::{
    SurfaceBevelPlan, SurfaceEdgePlan, SurfaceEdgeRoles, SurfacePlanRecipe,
};
use geometry_processing::bevel::surface::{SurfaceCrossingAtTheMitre, SurfaceStrip};
use geometry_processing::bevel::{BevelAdvisory, BevelAdvisoryCode, BevelSweptChain};
use geometry_processing::topology::{ChainEdge, EdgeConvexity, FeatureEdge};

use crate::rust_only_bevel_r1p06_tests::square_edge_plan;
use geometry_processing::bevel::surface::{SurfaceLine, SurfaceMixedCorner};

/// The R1.P12 mixed corner with its bead's axis (its centre line) stood square to the cap: +z
/// through the origin, so nothing about the corner leans.
fn mixed_corner(hands_over_at_a_mitre: bool) -> SurfaceMixedCorner {
    let mut corner = crate::rust_only_bevel_r1p12_tests::mixed_corner(hands_over_at_a_mitre);
    corner.bead.center_line = SurfaceLine::new(Vector3::ZERO, Vector3::new(0.0, 0.0, 1.0));
    corner
}

fn close(a: Vector3, b: Vector3) -> bool {
    (a - b).length() < 1e-12
}

/// On a square cap a point is carried along the cap's normal: into the cap's plane through the
/// outer rail centre (z = 0), or through the corner vertex (z = 7); down to the bead's end a radius
/// under the cap; the cap corner a radius out of the axis at the end of the turn, in the cap.
#[test]
fn rust_only_a_square_corner_carries_points_along_the_cap_normal() {
    let corner = mixed_corner(false);
    assert!(!corner.the_cap_leans() && !corner.the_beads_ring_leans());

    let at = Vector3::new(3.0, 4.0, 5.0);
    assert!(close(
        corner.up_into_the_cap(at),
        Vector3::new(3.0, 4.0, 0.0)
    ));
    assert!(close(
        under_the_cap(&corner, at),
        Vector3::new(3.0, 4.0, 7.0)
    ));
    assert!(close(
        corner.down_to_the_beads_end(Vector3::new(3.0, 4.0, 0.0)),
        Vector3::new(3.0, 4.0, -1.0)
    ));

    let cap_corner = corner.cap_corner();
    assert!(cap_corner.z.abs() < 1e-12);
    assert!((corner.axis().distance_to(cap_corner) - corner.size).abs() < 1e-12);

    // The shared face's foot: the cut's tangent line in the shared face met with the cut's stop
    // plane (x = 0), which on this fixture is the tangent line's own point, lifted into the cap.
    let expected = if corner.cut.left_region_key() == corner.shared_region_key {
        Vector3::new(0.0, 1.0, 0.0)
    } else {
        Vector3::new(0.0, 0.0, 0.0)
    };
    assert!(close(corner.shared_face_foot(), expected));
}

/// A square corner takes the cap-riding canal (and caches the answer); the convex cut's band stops
/// on the handover, the concave bead's on the bead's own square section.
#[test]
fn rust_only_a_square_corner_bounds_each_band_on_its_own_plane() {
    let corner = mixed_corner(false);
    assert!(!the_corner_takes_the_dual(&corner));
    assert!(corner.takes_the_dual_canal.get().is_none());
    assert!(!corner.takes_the_dual_canal());
    assert_eq!(corner.takes_the_dual_canal.get(), Some(false));
    assert_eq!(corner.bead_ring_stands_on(), corner.bead_stops_at);

    let convex = square_edge_plan(1.0, true);
    assert_eq!(convex.convexity(), EdgeConvexity::Convex);
    assert_eq!(corner.band_end_for(&convex), corner.cut_stops_at);

    let mut concave = square_edge_plan(1.0, true);
    concave.edge.edge.convexity = EdgeConvexity::Concave;
    assert_eq!(corner.band_end_for(&concave), corner.bead_stops_at);

    // And the fixture reads no reach for the dual's crease out of this corner (no dual, or no rim
    // edge of its wall region on that crease).
    assert_eq!(try_how_far_the_duals_crease_runs(&corner), None);
}

/// The wall nobody picked, where nothing was rolled, is ONE PLANE from the corner to the cap
/// corner; its strip under the blend is that segment and the same segment a radius down. One face
/// read twice gives one strip.
#[test]
fn rust_only_a_flat_wall_is_one_plane_with_one_strip_under_the_blend() {
    let corner = Rc::new(mixed_corner(false));
    assert!(corner.unpicked_section().is_none());

    let wall = SurfaceMixedCornerWall::try_read(&corner).expect("one plane");
    assert_eq!(wall.count(), 1);
    assert_eq!(wall.touched_facet, 0);
    assert!(close(wall.foot, corner.cap_corner()));
    assert!(wall.exit_turn[0].is_infinite(), "a plane runs on");
    assert_eq!(
        wall.try_where_the_rail_meets_the_wall(),
        wall.try_where_a_circle_meets_the_wall(corner.outer_rail_radius(), corner.turn_radians())
    );

    let strip = wall.strip_under_the_blend();
    assert_eq!(strip.len(), 1);
    let (rim, normal) = strip[0];
    assert!(close(rim[0], corner.corner) && close(rim[1], wall.foot));
    assert!(close(rim[2], wall.foot - Vector3::new(0.0, 0.0, 1.0)));
    assert!(close(rim[3], corner.corner - Vector3::new(0.0, 0.0, 1.0)));
    assert!(close(normal, corner.out_of_the_wall));

    // A wall whose foot IS its corner gives up nothing - one piece fewer, not a refusal.
    let pinched = SurfaceMixedCornerWall::try_read_section(&corner, None, corner.corner).unwrap();
    assert!(pinched.strip_under_the_blend().is_empty());

    let wall = Rc::new(wall);
    let one = SurfaceMixedCornerFaces::new(wall.clone(), wall.clone(), true);
    assert_eq!(one.strips_under_the_blend().len(), 1);
    let two = SurfaceMixedCornerFaces::new(wall.clone(), wall, false);
    assert_eq!(two.strips_under_the_blend().len(), 2);
    assert_eq!(
        SurfaceMixedCornerWall::in_the_cap(&corner, Vector3::new(1.0, 2.0, 3.0)),
        corner.up_into_the_cap(Vector3::new(1.0, 2.0, 3.0))
    );
}

/// On a square corner the run-out's carry is the RIGID turn whatever rows it is handed: a row
/// stands where it sat at no turn past, every station reads the one array of radii, and the touch
/// row stands off the ideal rail's `2r`; with no rows at all the cap is touched on the outer rail.
#[test]
fn rust_only_a_square_corners_run_out_is_carried_rigidly() {
    let corner = Rc::new(mixed_corner(false));
    let mut pool = SurfaceVertexPool::new(&[]);
    let seats = [Vector3::new(2.0, 0.0, 0.0), Vector3::new(0.0, 3.0, 1.0)];
    let column: Vec<i32> = seats.iter().map(|&s| pool.add(s)).collect();

    let carry = SurfaceRunOutCarry::of(corner.clone(), None, &pool, &column);
    assert!(!carry.on_the_spine());
    assert_eq!(carry.rows(), 2);
    assert!(close(carry.at(1, 0.0), pool.position_of(column[1])));
    assert!(close(
        carry.touches_the_cap_at(0.0),
        pool.position_of(column[0])
    ));
    let radii = carry.out_of_the_axis_at(0.7);
    assert!(matches!(radii, std::borrow::Cow::Borrowed(_)));
    assert!((radii[0] - 2.0).abs() < 1e-6 && (radii[1] - 3.0).abs() < 1e-6);
    assert_eq!(
        carry.the_touch_row_stands_off_at(0.7),
        corner.outer_rail_radius()
    );

    // Turned a quarter about the axis, a seat keeps its radius.
    let turned = carry.at(0, std::f64::consts::FRAC_PI_2);
    assert!((corner.axis().distance_to(turned) - radii[0]).abs() < 1e-6);

    let empty = SurfaceRunOutCarry::of(corner.clone(), None, &pool, &[]);
    assert!(close(
        empty.touches_the_cap_at(0.25),
        corner.outer_rail_point(corner.turn_radians() + 0.25)
    ));
}

/// At a junction a band on an arm's SHARED face is ended by the other edge's band unless the arm
/// hands over at a mitre; on nobody's shared face it stops on the edge's own plane.
#[test]
fn rust_only_a_junction_bounds_a_band_off_the_face_it_lies_on() {
    let plain = Rc::new(mixed_corner(false));
    let mitre = Rc::new(mixed_corner(true));
    let edge = square_edge_plan(1.0, true);
    let shared = plain.shared_region_key;

    let mut ends = Vec::new();
    surface_junction_band_ends::add(&mut ends, std::slice::from_ref(&plain), &edge, shared);
    assert!(ends.is_empty(), "the other edge's band ends it");

    assert!(mitre.both_bands_stop_on_the_shared_face());
    surface_junction_band_ends::add(&mut ends, std::slice::from_ref(&mitre), &edge, shared);
    assert_eq!(ends, vec![mitre.band_end_for(&edge)]);
    ends.clear();

    surface_junction_band_ends::add(&mut ends, &[plain.clone(), mitre], &edge, shared + 1);
    assert_eq!(ends, vec![plain.band_end_for(&edge)]);
}

/// The band ends read off two plans stand where they are told to and face into their own bands:
/// along the edge where the chain arrives, back along it where it leaves.
#[test]
fn rust_only_the_plan_band_ends_face_into_their_bands() {
    let plan = square_edge_plan(1.0, true);
    let through = Vector3::new(4.0, 0.5, -0.5);

    let arrives = band_end_where_the_chain_arrives(&plan, &plan, Some(through));
    assert_eq!(
        arrives,
        SurfaceBandEnd::new(through, Vector3::new(1.0, 0.0, 0.0))
    );
    let leaves = band_end_where_the_chain_leaves(&plan, &plan, Some(through));
    assert_eq!(
        leaves,
        SurfaceBandEnd::new(through, Vector3::new(-1.0, 0.0, 0.0))
    );
}

/// One piece of the square edge along +x, from `from` to `to` (both on the x axis): the same two
/// tangent lines as the whole edge, so consecutive pieces are collinear.
fn piece(index: i32, from: f64, to: f64) -> SurfaceEdgePlan {
    let mut plan = square_edge_plan(1.0, true);
    plan.edge = ChainEdge::new(
        index,
        FeatureEdge {
            start: Vector3::new(from, 0.0, 0.0),
            end: Vector3::new(to, 0.0, 0.0),
            ..plan.edge.edge
        },
        false,
    );
    plan
}

fn chain_plan(edges: Vec<SurfaceEdgePlan>, end_trim: Option<SurfaceBandEnd>) -> SurfaceBevelPlan {
    SurfaceBevelPlan::new(
        edges,
        false,
        1.0,
        true,
        None,
        HashMap::new(),
        true,
        false,
        None,
        end_trim,
        IndexMap::new(),
        SurfaceEdgeRoles::default(),
        SurfacePlanRecipe {
            chain: BevelSweptChain::default(),
            regions: None,
            curve_regions: None,
            round_profile: true,
        },
    )
}

/// Two pieces of one chord with no neighbours beyond them: the run's band IS the run (0 to 10 on
/// both tangent lines), so the joint keeps its corner and no point is read.
#[test]
fn rust_only_a_chorded_run_with_no_neighbours_keeps_its_corner() {
    let edges = vec![piece(0, 0.0, 4.0), piece(1, 4.0, 10.0)];
    let x = Vector3::new(1.0, 0.0, 0.0);

    for line in [edges[0].left_tangent_line, edges[0].right_tangent_line] {
        assert_eq!(
            where_the_neighbours_plane_crosses(&edges, false, 0, true, Vector3::ZERO, x, line),
            0.0
        );
        assert_eq!(
            where_the_neighbours_plane_crosses(&edges, false, 1, false, Vector3::ZERO, x, line),
            10.0
        );
    }
    // The neighbour's plane at the joint, square to the run, crosses each line at the joint.
    let at_the_joint = where_the_neighbours_plane_crosses(
        &edges,
        false,
        1,
        true,
        Vector3::ZERO,
        x,
        edges[0].left_tangent_line,
    );
    assert!((at_the_joint - 4.0).abs() < 1e-12);

    assert_eq!(try_the_point(&edges, false, 0, 1e-9), None);
    assert_eq!(
        try_the_point(&edges, false, 1, 1e-9),
        None,
        "no edge after the last"
    );
    assert_eq!(
        chain_plan(edges, None).the_chorded_runs_joint_after(0),
        None
    );
}

/// The band ends of a two-piece open chain: the start runs out (the square plane through the first
/// corner), the joint is the mitre between the pieces facing into each one's band, and the end is
/// trimmed; a band whose ends cross leaves nothing.
#[test]
fn rust_only_a_chains_band_ends_are_its_run_out_mitre_and_trim() {
    let x = Vector3::new(1.0, 0.0, 0.0);
    let trim = SurfaceBandEnd::new(Vector3::new(10.0, 0.0, 0.0), -x);
    let plan = chain_plan(vec![piece(0, 0.0, 4.0), piece(1, 4.0, 10.0)], Some(trim));
    let joint = Vector3::new(4.0, 0.0, 0.0);

    for left in [true, false] {
        assert_eq!(
            plan.band_ends_of(0, left),
            vec![
                SurfaceBandEnd::new(Vector3::ZERO, x),
                SurfaceBandEnd::new(joint, -x)
            ]
        );
        assert_eq!(
            plan.band_ends_of(1, left),
            vec![SurfaceBandEnd::new(joint, x), trim]
        );
    }

    let line = plan.edges[0].left_tangent_line;
    assert!(leaves_a_band_of(&plan, 0, true, &line));
    let crossed = chain_plan(
        vec![piece(0, 0.0, 4.0), piece(1, 4.0, 10.0)],
        Some(SurfaceBandEnd::new(Vector3::new(-1.0, 0.0, 0.0), -x)),
    );
    assert!(!leaves_a_band_of(&crossed, 1, true, &line));
}

/// The mitre crossing is refused by name, at the mitre corner, where a strip is missing or the two
/// strips do not have the same rails to make one ring of.
#[test]
fn rust_only_a_crossing_needs_two_strips_with_the_same_rails() {
    let corner = Rc::new(mixed_corner(false));
    let wall = SurfaceMixedCornerWall::try_read(&corner).expect("one plane");
    let plan = square_edge_plan(1.0, true);
    let two = SurfaceStrip::try_create(&plan, 2).expect("a quarter in two");
    let three = SurfaceStrip::try_create(&plan, 3).expect("a quarter in three");
    assert_ne!(two.rail_count(), three.rail_count());
    let at = Vector3::new(5.0, 0.0, 0.0);

    for (cap, on_the_wall) in [
        (None, Some(&two)),
        (Some(&two), None),
        (Some(&two), Some(&three)),
    ] {
        let refused =
            SurfaceCrossingAtTheMitre::try_read(&corner, &wall, cap, on_the_wall, at, 1e-9)
                .expect_err("no ring to hand over on");
        assert_eq!(
            refused,
            BevelAdvisory::new(
                BevelAdvisoryCode::SurfaceCrossingMissesTheMitre,
                vec![at.into()]
            )
        );
    }
}

/// Which chord carries the ring: the touching chord while its cap band survives between the
/// handover and its own mitre, the chord before it (cut on its own mitre) where it does not, and
/// the touching chord at an open chain's end with nothing before it.
#[test]
fn rust_only_the_ring_goes_to_the_mitre_when_the_touching_chord_has_no_band() {
    let x = Vector3::new(1.0, 0.0, 0.0);
    let planned = vec![piece(0, 0.0, 4.0), piece(1, 4.0, 10.0)];
    let mut corner = mixed_corner(false);

    // The handover keeps x >= 0 and the touching chord's mitre x >= 4: a band is left.
    assert_eq!(
        hands_over::try_read(&planned, false, 1, true, &corner),
        (1, None)
    );
    assert_eq!(
        hands_over::try_read(&planned, false, 0, true, &corner),
        (0, None)
    );

    // A handover keeping x <= 2 against the mitre's x >= 4 leaves nothing.
    corner.cut_stops_at = SurfaceBandEnd::new(Vector3::new(2.0, 0.0, 0.0), -x);
    let (carries, stops_at) = hands_over::try_read(&planned, false, 1, true, &corner);
    assert_eq!(carries, 0);
    assert_eq!(stops_at, Some(hands_over::mitre_with(&planned, 0, 1, true)));
    assert_eq!(
        stops_at,
        Some(SurfaceBandEnd::new(Vector3::new(4.0, 0.0, 0.0), -x))
    );
}

/// A strip's end ring cut at a named plane lands every rail on it; a plane the rails run along is
/// refused at the corner. Won from the pool, the outer rails are the tangent runs' own vertices and
/// the interior rails new ones; a missing run is a seam that did not meet.
#[test]
fn rust_only_a_strips_end_ring_is_cut_and_won_from_the_pool() {
    let x = Vector3::new(1.0, 0.0, 0.0);
    let corner = mixed_corner(true);
    let strip = SurfaceStrip::try_create(&square_edge_plan(1.0, true), 2).expect("a strip");
    let rails = strip.rail_count() as usize;
    let plane = SurfaceBandEnd::new(Vector3::new(3.0, 0.0, 0.0), x);

    let (meets, at) = try_cut_the_ring(&corner, &strip, true, Some(plane)).expect("crosses");
    assert_eq!((meets.len(), at.len()), (rails, rails));
    assert!(at.iter().all(|p| (p.x - 3.0).abs() < 1e-9));
    assert_eq!(
        try_cut_the_ring(
            &corner,
            &strip,
            true,
            Some(SurfaceBandEnd::new(
                Vector3::ZERO,
                Vector3::new(0.0, 1.0, 0.0)
            ))
        ),
        Err(BevelAdvisory::new(
            BevelAdvisoryCode::SurfaceCornerNotSupported,
            vec![corner.corner.into()]
        ))
    );

    let mut pool = SurfaceVertexPool::new(&[]);
    let first = pool.add(at[0]);
    let last = pool.add(at[rails - 1]);
    let seams: Vec<SideSeams> = vec![[Some(vec![(first, 0.0)]), Some(vec![(last, 0.0)])]];
    let before = pool.count();

    // The CUT handing over at a mitre is checked against its own seams alone.
    let (ring, meets_at) = try_ring(
        &mut pool,
        &corner,
        &strip,
        true,
        &seams,
        0,
        true,
        Some(plane),
    )
    .expect("meets");
    assert_eq!(meets_at, meets);
    assert_eq!((ring[0], ring[rails - 1]), (first, last));
    assert_eq!(pool.count(), before + rails as i32 - 2);

    let missing: Vec<SideSeams> = vec![[None, None]];
    assert_eq!(
        try_ring(
            &mut pool,
            &corner,
            &strip,
            true,
            &missing,
            0,
            true,
            Some(plane)
        )
        .map_err(|failure| failure.code()),
        Err(BevelAdvisoryCode::SurfaceStripSeamDidNotMeet)
    );
}
