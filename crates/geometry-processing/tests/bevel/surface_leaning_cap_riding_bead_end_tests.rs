//! The first rung of the LEANING CAP-RIDING
//! corner. The bead's own end stands on the ring the rolling ball really traces on it, and the
//! strip of wall under the blend follows that ring down - so the patch's grid, the two rings it is
//! sewn to and the faces under it all close over the turn.
//!
//! The bead used to end on its own SQUARE section through the tangency (`bead_stops_at`), which is
//! right for a square corner and for the DUAL and wrong for a leaning corner that rides the cap:
//! the patch's last rail at every station is where the ball touches the bead, in the spine's own
//! leaning plane (`SurfaceMixedCornerSpine::tube_ring_plane`), so a bead cut square left that row
//! up to 0.197 mm off its own strip at 5.625 degrees of lean and 0.297 at 8.4375. The strip of wall
//! under the blend follows the leaning plane down the axis (`down_to_the_beads_end`). A square cap
//! drops by the radius as the constant and ends on its own square section, so every number of the
//! square case is unmoved. The parameter rows are a loop.

use std::rc::Rc;

use solidmesh::bevel::surface::surface_mixed_corner_patch::try_cut_the_ring;
use solidmesh::bevel::surface::{
    surface_bevel_builder, surface_mesh_closure, surface_mixed_corner_run_out, SurfaceBevelPlan,
    SurfaceBuildOptions, SurfaceMixedCorner, SurfaceMixedCornerRows, SurfaceMixedCornerWall,
    SurfaceStrip,
};
use solidmesh::topology::{EdgeConvexity, FaceRegionCache, FeatureEdgeSet};
use solidmesh_math::math_helper;
use solidmesh_mesh::{CancelToken, Mesh};
use std::sync::Arc;

use crate::mesh_measurements::{shell_count, signed_volume};
use crate::surface_mixed_corner_fixture as fixture;
use crate::surface_mixed_corner_reading_tests::the_mixed_corner;
use crate::surface_oblique_mixed_corner_reading_tests::the_oblique_corner;
use crate::topology_test_meshes::create_sheared_l_shaped_grid_prism;

/// Two thirds of the seam budget at 8 segments, and its whole limit - the two leans the app-side
/// cases carry.
const TWO_THIRDS_OF_THE_BUDGET: f64 = 5.625;

const THE_WHOLE_BUDGET: f64 = 8.4375;

const RADIUS: f64 = 1.0;

const SEGMENTS: i32 = 8;

/// The bead's end ring IS the curve the ball traces on the bead: every rail of it lands on
/// `SurfaceMixedCornerSpine::bead_contact_at` at its own station, to the last bits of a double,
/// where the square section it used to be cut at stands a fifth to a third of a millimetre off
/// that curve at a fillet radius of one.
#[test]
fn the_bead_ends_on_the_ring_the_ball_traces_on_it() {
    for (lean_degrees, along_the_rim, ring_leans_by, the_square_section_stands_off) in [
        (TWO_THIRDS_OF_THE_BUDGET, false, 11.1436, 0.19698),
        (TWO_THIRDS_OF_THE_BUDGET, true, 11.1436, 0.19698),
        (THE_WHOLE_BUDGET, false, 16.5242, 0.29667),
        (THE_WHOLE_BUDGET, true, 16.5242, 0.29667),
    ] {
        let row = format!("{lean_degrees} degrees, along the rim {along_the_rim}");
        let (corner, plan) = the_sheared_corner(lean_degrees, along_the_rim);

        assert!(
            corner.the_beads_ring_leans(),
            "{row}: the cap leans and this corner's canal is the cap-riding one, which is the \
             whole of what moves the bead's end"
        );

        let stands_on = corner.bead_ring_stands_on();

        assert!(
            stands_on.point.equals_within(corner.tangent_point, 1e-9),
            "{row}: the ring still stands on the point the two cylinders touch at - instead {:?}",
            stands_on.point
        );

        let leaning = math_helper::radians_to_degrees(
            stands_on
                .inward
                .dot(corner.axis().direction)
                .abs()
                .clamp(-1.0, 1.0)
                .acos(),
        );

        assert!(
            (leaning - ring_leans_by).abs() <= 1e-3,
            "{row}: and it LEANS off the bead's square section by more than the cap does - \
             instead {leaning:.4}"
        );

        // The two readings of the same rails: cut at the plane the corner now names, and cut at
        // the square section it used to be.
        let strip = the_beads_strip(&plan);

        let (_, on_the_ring) =
            try_cut_the_ring(&corner, &strip, false, None).unwrap_or_else(|why| {
                panic!("{row}: the bead's strip crosses that plane rail for rail - instead {why:?}")
            });
        let (_, square) = try_cut_the_ring(&corner, &strip, false, Some(corner.bead_stops_at))
            .unwrap_or_else(|why| {
                panic!("{row}: and it crosses the square section too - instead {why:?}")
            });

        let step = corner.turn_radians() / (on_the_ring.len() - 1) as f64;
        let mut worst_on_the_ring: f64 = 0.0;
        let mut worst_square: f64 = 0.0;

        for rail in 0..on_the_ring.len() {
            let contact = corner.spine.bead_contact_at(rail as f64 * step);

            worst_on_the_ring = worst_on_the_ring.max((on_the_ring[rail] - contact).length());
            worst_square = worst_square.max((square[rail] - contact).length());
        }

        assert!(
            worst_on_the_ring < 1e-12,
            "{row}: every rail of the ring is where the ball touches the bead at that station - \
             instead one stands {worst_on_the_ring:.3e} off it"
        );
        assert!(
            (worst_square - the_square_section_stands_off).abs() <= 1e-4,
            "{row}: while the square section's own rails stand that far off the curve the patch \
             is sewn to - instead {worst_square:.5}"
        );
    }
}

/// A SQUARE corner and the nose's DUAL both still end on the bead's own square section, and their
/// strip of wall still drops by the radius as the constant - so nothing that builds today is read
/// through the new arithmetic at all.
#[test]
fn the_square_corner_and_the_dual_both_still_end_square() {
    let square = Rc::new(the_mixed_corner());
    let dual = the_oblique_corner();

    for corner in [&square, &dual] {
        assert!(
            !corner.the_beads_ring_leans(),
            "neither a square corner nor a corner whose canal is the dual moves its bead's end"
        );

        let stands_on = corner.bead_ring_stands_on();

        assert!(
            stands_on.point == corner.bead_stops_at.point
                && stands_on.inward == corner.bead_stops_at.inward,
            "so the plane is the square section itself and not arithmetic that lands near it - \
             instead {:?} / {:?}",
            stands_on.point,
            stands_on.inward
        );

        let in_the_cap = corner.cap_corner();

        assert!(
            corner.down_to_the_beads_end(in_the_cap)
                == in_the_cap - (corner.out_of_the_cap_face * corner.size),
            "and the strip under the blend drops by the radius, bit for bit as it did"
        );
    }

    assert!(
        !square.the_cap_leans(),
        "the square L is the corner every number of the ladder was measured on"
    );
    assert!(
        dual.takes_the_dual_canal(),
        "and the nose's corner is the one the control measured onto the dual"
    );
}

/// THE WHOLE CORNER CLOSES. With the run-out carried on the spine as well, a leaning cap-riding
/// corner publishes a closed, manifold, single shell at both leans and both shear directions -
/// where the rung before it left 14 and 21 edges open, and where shearing ALONG the rim folded the
/// run-out onto a face it had already drawn.
///
/// The gate is lifted for this measurement (`lay_the_oblique_rows_anyway`) because the product's
/// own gate is lifted at the rung after this one; everything the corner draws is on the spine here.
#[test]
fn the_whole_corner_closes_on_the_spine() {
    for (lean_degrees, along_the_rim, faces) in [
        (TWO_THIRDS_OF_THE_BUDGET, false, 450),
        (TWO_THIRDS_OF_THE_BUDGET, true, 452),
        (THE_WHOLE_BUDGET, false, 484),
        (THE_WHOLE_BUDGET, true, 454),
    ] {
        let row = format!("{lean_degrees} degrees, along the rim {along_the_rim}");
        let mesh = create_sheared_l_shaped_grid_prism(lean_degrees, along_the_rim);
        let (regions, features) = fixture::regions_of(&mesh);
        let plan = fixture::the_plan_on(&regions, &features);

        let result = surface_bevel_builder::try_build(
            &mesh,
            &mesh.get_mesh_edge_graph(),
            &regions,
            &[plan],
            SEGMENTS,
            &CancelToken::new(),
            SurfaceBuildOptions {
                lay_the_oblique_rows_anyway: true,
                ..crate::surface_test_build::build_options()
            },
        )
        .unwrap_or_else(|why| {
            panic!(
                "{row}: the patch, the bites and the run-out are all laid on the spine now - \
                 instead {why:?}"
            )
        })
        .mesh;

        assert_eq!(
            result.faces.len(),
            faces,
            "{row}: instead {} faces",
            result.faces.len()
        );

        let (open, open_at) = surface_mesh_closure::open_edge_count(&result);
        assert_eq!(
            open, 0,
            "{row}: and nothing of it is left open - instead at {open_at:?}"
        );

        let as_list: Vec<(i32, i32, i32)> = result
            .faces
            .iter()
            .map(|face| (face.v0, face.v1, face.v2))
            .collect();

        let (folds, folded) = surface_mesh_closure::folded_edge_count(&as_list);
        assert_eq!(
            folds, 0,
            "{row}: nor folded over on itself - instead between {} and {}",
            folded.0, folded.1
        );
        assert_eq!(shell_count(&result), 1, "{row}: and it is one solid");

        // And it took material off rather than adding it, by less than the same corner at a
        // smaller radius does - the blend's own volume, read the only way a whole-part measurement
        // can say it.
        let smaller = the_build_at(&mesh, &regions, &features, 0.9);

        assert!(
            signed_volume(&result) < signed_volume(&smaller),
            "{row}: a fillet of radius one takes more off this corner than one of 0.9 does"
        );
        assert!(
            signed_volume(&smaller) < signed_volume(&mesh),
            "{row}: and both of them take material off the part rather than adding it"
        );
    }
}

/// Every vertex the run-out draws stands ON THE BALL at its own station of the spine, a radius
/// from that ball's centre - which is what says the columns past the bead's turn are the same
/// canal the patch is and not a torus carried on past it.
#[test]
fn the_run_outs_own_columns_stand_on_the_ball_at_their_station() {
    for (lean_degrees, along_the_rim) in [
        (TWO_THIRDS_OF_THE_BUDGET, false),
        (TWO_THIRDS_OF_THE_BUDGET, true),
        (THE_WHOLE_BUDGET, false),
        (THE_WHOLE_BUDGET, true),
    ] {
        let row = format!("{lean_degrees} degrees, along the rim {along_the_rim}");
        let (corner, plan) = the_sheared_corner(lean_degrees, along_the_rim);
        let (rows, rails) = the_rows_of(&corner, &plan);

        let wall = SurfaceMixedCornerWall::try_read(&corner).unwrap_or_else(|why| {
            panic!("{row}: the L's unpicked wall is one plane - instead {why:?}")
        });
        let meets = wall
            .try_where_the_rail_meets_the_wall()
            .unwrap_or_else(|| panic!("{row}: and the contact curve reaches it"));

        let mut worst_off_the_ball: f64 = 0.0;
        let mut worst_off_the_wall: f64 = 0.0;

        for station in 0..=32 {
            let past = (meets - corner.turn_radians()) * station as f64 / 32.0;
            let turn = corner.turn_radians() + past;
            let centre = corner.spine.centre_at(turn);

            for rail in 0..rails {
                worst_off_the_ball = worst_off_the_ball
                    .max(((rows.at(rail, turn) - centre).length() - RADIUS).abs());
            }

            // And the point the column ENDS on stands on the wall it is trimmed by, which is the
            // other half of the same statement: the run-out is the ball's own surface cut by that
            // plane.
            let trim = surface_mixed_corner_run_out::try_scallop_point(&corner, &wall, past)
                .unwrap_or_else(|| panic!("{row}: the station's ring meets the wall at {past}"));
            let (stands_in, facing) = wall
                .try_plane_at(turn)
                .unwrap_or_else(|| panic!("{row}: and the wall answers for that turn"));

            worst_off_the_wall = worst_off_the_wall.max((trim - stands_in).dot(facing).abs());

            let off = ((trim - centre).length() - RADIUS).abs();
            assert!(
                off < 1e-9,
                "{row}: and the trim point is a point of the ball as much as the rows are - \
                 instead {off:.3e} off at {past}"
            );
        }

        assert!(
            worst_off_the_ball < 1e-9,
            "{row}: every row of every station is a radius from that station's centre - instead \
             one stands {worst_off_the_ball:.3e} off"
        );
        assert!(
            worst_off_the_wall < 1e-9,
            "{row}: and every trim point stands in the wall's own plane - instead one stands \
             {worst_off_the_wall:.3e} out of it"
        );
    }
}

/// And BOTH general forms reduce to the closed forms the square corner was written in: the
/// station's ring met with the wall is the same point as `r - sqrt(r^2 - (2r - d)^2)` under the
/// cap, and the contact curve reaches the wall at sixty degrees past the bead whatever the wedge.
///
/// The square corner keeps the closed forms - it is routed round the general ones by
/// `SurfaceMixedCorner::the_beads_ring_leans`, so that none of its published numbers moves in the
/// last bits - and this is what says the two are one construction all the same.
#[test]
fn the_square_corners_own_closed_forms_are_what_the_general_ones_reduce_to() {
    let corner = Rc::new(the_mixed_corner());

    let wall =
        SurfaceMixedCornerWall::try_read(&corner).unwrap_or_else(|why| panic!("instead {why:?}"));

    let meets = wall.try_where_the_contact_meets_the_wall().expect(
        "the contact curve of a square cap is the circle of radius 2r the run-out was written about",
    );
    let past_the_bead = math_helper::radians_to_degrees(meets - corner.turn_radians());
    assert!(
        (past_the_bead - 60.0).abs() <= 1e-9,
        "so it reaches the wall at sixty degrees past the bead - instead {past_the_bead}"
    );

    let mut worst: f64 = 0.0;

    for station in 0..=32 {
        let past = surface_mixed_corner_run_out::clears_the_wall() * station as f64 / 32.0;

        let by_the_closed_form =
            surface_mixed_corner_run_out::try_scallop_point(&corner, &wall, past)
                .unwrap_or_else(|| panic!("the closed form answers at {past}"));
        let by_the_ring =
            surface_mixed_corner_run_out::try_scallop_on_the_ring(&corner, &wall, past)
                .unwrap_or_else(|| panic!("and so does the station's own ring at {past}"));

        worst = worst.max((by_the_closed_form - by_the_ring).length());
    }

    assert!(
        worst < 1e-9,
        "and the two are the same point at every station of the turn - instead they part by \
         {worst:.3e}"
    );
}

/// The corner the sheared L's two picked edges meet at, and the plan it was read from.
fn the_sheared_corner(
    lean_degrees: f64,
    along_the_rim: bool,
) -> (Rc<SurfaceMixedCorner>, Rc<SurfaceBevelPlan>) {
    let mesh = create_sheared_l_shaped_grid_prism(lean_degrees, along_the_rim);
    let (regions, features) = fixture::regions_of(&mesh);

    let plan = fixture::the_plan_on(&regions, &features);

    (fixture::the_corner(&plan), plan)
}

/// The same corner built at another radius, for a measurement that needs two of them.
fn the_build_at(
    part: &Mesh,
    regions: &Arc<FaceRegionCache>,
    features: &FeatureEdgeSet,
    size: f64,
) -> Mesh {
    let plan =
        SurfaceBevelPlan::try_create(&fixture::the_joined_chain(features), regions, size, true)
            .unwrap_or_else(|why| panic!("the plan refused the chain at {size}: {why:?}"));

    surface_bevel_builder::try_build(
        part,
        &part.get_mesh_edge_graph(),
        regions,
        &[plan],
        SEGMENTS,
        &CancelToken::new(),
        SurfaceBuildOptions {
            lay_the_oblique_rows_anyway: true,
            ..crate::surface_test_build::build_options()
        },
    )
    .unwrap_or_else(|why| panic!("the build refused the corner at {size}: {why:?}"))
    .mesh
}

/// The rows the patch lays its grid with - the cut's own end ring read from the cap face round,
/// and carried on the spine - which is what the run-out carries on past the bead's turn.
fn the_rows_of(
    corner: &SurfaceMixedCorner,
    plan: &SurfaceBevelPlan,
) -> (SurfaceMixedCornerRows, usize) {
    let cut = fixture::strip_of(
        plan,
        if plan.edges[0].convexity() == EdgeConvexity::Convex {
            0
        } else {
            1
        },
        SEGMENTS,
    );

    let (_, ring) = try_cut_the_ring(corner, &cut, true, None)
        .unwrap_or_else(|why| panic!("the cut's end ring refused: {why:?}"));

    // Rail zero of a strip is whichever side the walk left it on, and the rows are read from the
    // CAP face round - the end standing nearest where the ball touches the cap at turn zero.
    let start = corner.spine.t_contact_at(0.0);
    let rails = ring.len();
    let from_the_cap_face: Vec<_> =
        if (ring[0] - start).length() < (ring[ring.len() - 1] - start).length() {
            ring
        } else {
            ring.into_iter().rev().collect()
        };

    let rows = SurfaceMixedCornerRows::try_read_corner(corner, &from_the_cap_face)
        .unwrap_or_else(|why| panic!("the rows refused the ring: {why:?}"));

    (rows, rails)
}

/// The bead's own strip - the concave edge's, whichever of the plan's two edges that is.
fn the_beads_strip(plan: &SurfaceBevelPlan) -> SurfaceStrip {
    let bead = if plan.edges[0].convexity() == EdgeConvexity::Concave {
        0
    } else {
        1
    };

    fixture::strip_of(plan, bead, SEGMENTS)
}
