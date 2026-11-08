//! The patch's rows at the OBLIQUE mixed corner of
//! the nose-in-notch part's left ridge end, laid along the spine (`SurfaceMixedCornerRows`) rather
//! than turned rigidly about the bead's axis.
//!
//! A rigid turn keeps the cut's own 112.62 degree arc all the way round, and the ball's does not:
//! at the end of the turn the ring is the BEAD's 66.91 degree arc, and the turned ring stands up to
//! 1.4072 mm off the ball at a fillet radius of one. Laid along the spine the rows stand on it to
//! the last bits of a double. The corner itself closes on the DUAL; asked for these cap-riding
//! rows the build publishes nothing. The rows and the station count are the ones
//! `surface_oblique_mixed_corner_bites_tests` reads the same corner with.

use std::sync::Arc;

use geometry_processing::bevel::surface::{
    surface_bevel_builder, SurfaceBevelPlan, SurfaceBuildFailure, SurfaceBuildOptions,
    SurfaceMixedCorner, SurfaceMixedCornerRows,
};
use geometry_processing::bevel::{BevelAdvisoryArg, BevelAdvisoryCode};
use geometry_processing::topology::{FaceRegionCache, FeatureEdgeSet};
use polygon_mesh::CancelToken;
use std::rc::Rc;
use vector_math::{math_helper, Vector3};

use crate::nose_in_notch_test_meshes;
use crate::surface_mixed_corner_fixture as fixture;
use crate::surface_oblique_mixed_corner_bites_tests::{the_rows_of, the_station_count_of};
use crate::surface_oblique_mixed_corner_reading_tests::the_oblique_corner;
use crate::surface_two_crease_junction_reading_tests::{the_edge_from, the_plans_of};

const RADIUS: f64 = 1.0;

/// The sampling every measurement here is quoted at.
const SEGMENTS: i32 = 8;

/// Every row point stands a radius from the ball's centre at its own station, and in that ball's
/// characteristic plane - the two conditions that say the patch IS the rolling ball's surface.
/// The plane is read as the spine's own TANGENT, by difference, so the second condition checks the
/// construction rather than restating it.
#[test]
fn every_row_point_stands_on_the_ball_at_its_own_station() {
    let corner = the_oblique_corner();
    let (rows, ring) = the_rows_of(&corner);
    let turns = the_station_count_of(&corner);

    let mut worst_off_the_ball = 0.0_f64;
    let mut worst_out_of_the_plane = 0.0_f64;
    let mut worst_rigid = 0.0_f64;

    for (rail, &on_the_ring) in ring.iter().enumerate() {
        for station in 0..turns {
            let turn = station as f64 * corner.turn_radians() / (turns - 1) as f64;
            let centre = corner.spine.centre_at(turn);
            let along = (corner.spine.centre_at(turn + 1e-6) - corner.spine.centre_at(turn - 1e-6))
                .get_normal();

            let from = rows.at(rail, turn) - centre;

            worst_off_the_ball = worst_off_the_ball.max((from.length() - RADIUS).abs());
            worst_out_of_the_plane = worst_out_of_the_plane.max(from.dot(along).abs());

            worst_rigid = worst_rigid
                .max(((corner.turned_about(on_the_ring, turn) - centre).length() - RADIUS).abs());
        }
    }

    assert!(
        worst_off_the_ball < 1e-12,
        "every row point is a point of the ball at its station - instead one stands {worst_off_the_ball:e} off it"
    );
    assert!(
        worst_out_of_the_plane < 1e-9,
        "and in the plane square to the spine, which is where a constant radius canal's own ring lies - instead one stands {worst_out_of_the_plane:e} out of it"
    );

    assert!(
        (worst_rigid - 1.4072).abs() <= 1e-4,
        "while the rigid turn these rows replaced stands that far off the ball at r = 1 - instead {worst_rigid:.4}"
    );
}

/// The rows' own sweep SHRINKS along the spine, from the cut's arc at turn zero to the bead's at the
/// end of the turn - 112.62 closing to 66.91 degrees.
#[test]
fn the_rows_sweep_shrinks_from_the_cuts_arc_to_the_beads() {
    let corner = the_oblique_corner();
    let (rows, ring) = the_rows_of(&corner);

    let at_zero = swept_at(&corner, &rows, ring.len(), 0.0);
    assert!(
        (at_zero - 112.6199).abs() <= 1e-3,
        "at turn zero the row is the cut's own end ring - instead {at_zero:.4}"
    );
    let at_the_end = swept_at(&corner, &rows, ring.len(), corner.turn_radians());
    assert!(
        (at_the_end - 66.9065).abs() <= 1e-3,
        "and at the end of the turn it is the bead's - instead {at_the_end:.4}"
    );

    // And it closes all the way down, so no station's row crosses another's.
    let mut swept = 180.0;
    let mut narrowest = 180.0_f64;

    for station in 0..=64 {
        let next = swept_at(
            &corner,
            &rows,
            ring.len(),
            station as f64 * corner.turn_radians() / 64.0,
        );

        assert!(
            next < swept,
            "the sweep closes at every station - at {station} it went from {swept:.4} to {next:.4}"
        );

        swept = next;
        narrowest = narrowest.min(next);
    }

    // Closing is not vanishing: the narrowest row is the bead's own arc, well clear of the bound
    // the corner's own guard puts under it - the arc sine of how square the axis stands to the cap,
    // 0.709575 here, which no station can close past (SurfaceMixedCornerRows.AcrossAt).
    let that_guards_bound = math_helper::radians_to_degrees(0.709575_f64.asin());

    assert!(
        (narrowest - 66.9065).abs() <= 1e-3,
        "the narrowest row of the turn is the bead's own arc - instead {narrowest:.4}"
    );
    assert!(
        (that_guards_bound - 45.2003).abs() <= 1e-3,
        "instead the guard's bound is {that_guards_bound:.4}"
    );
    assert!(
        narrowest > that_guards_bound,
        "and every row of the turn stands clear of it, so none of them can collapse"
    );
}

/// The rows end on the two curves the patch is sewn to: the cut's own end ring at turn zero, and
/// the bead's contact at the last rail of every station.
#[test]
fn the_rows_end_on_the_two_rings_they_are_sewn_to() {
    let corner = the_oblique_corner();
    let (rows, ring) = the_rows_of(&corner);
    let turns = the_station_count_of(&corner);

    for (rail, &on_the_ring) in ring.iter().enumerate() {
        assert!(
            (rows.at(rail, 0.0) - on_the_ring).length() < 1e-12,
            "column zero IS the cut's end ring - rail {rail} stands off it"
        );
    }

    for station in 0..turns {
        let turn = station as f64 * corner.turn_radians() / (turns - 1) as f64;

        assert!(
            (rows.at(0, turn) - corner.spine.t_contact_at(turn)).length() < 1e-9,
            "the first rail of every row is where the ball touches the cap - station {station} is not"
        );
        assert!(
            (rows.at(ring.len() - 1, turn) - corner.spine.bead_contact_at(turn)).length() < 1e-6,
            "and the last is where it touches the bead - station {station} is not"
        );
    }
}

/// THIS CORNER CLOSES, and it closes on the DUAL: the product builds it, and the CAP-RIDING rows
/// measured above still cannot - the two canals are different surfaces and only one of them is
/// this corner's.
#[test]
fn the_oblique_corner_closes_on_the_dual_and_cannot_on_the_cap_riding_rows() {
    let mesh = nose_in_notch_test_meshes::create();
    let (regions, features) = fixture::regions_of(&mesh);
    let plans = the_plans_of_the_left_ridge_end(&features, &regions);
    let graph = mesh.get_mesh_edge_graph();

    let options = |lay_the_oblique_rows_anyway| SurfaceBuildOptions {
        crossings: None,
        ran_off_over_a_stretch: None,
        lay_the_oblique_rows_anyway,
        planless_loops: &[],
        features: None,
        use_sections: crate::surface_test_build::use_sections(),
    };

    let on_the_dual = surface_bevel_builder::try_build(
        &mesh,
        &graph,
        &regions,
        &plans,
        SEGMENTS,
        &CancelToken::new(),
        options(false),
    );
    assert!(
        on_the_dual.is_ok(),
        "the product closes a leaning corner on the dual now - instead {:?}",
        on_the_dual.err()
    );

    let built = surface_bevel_builder::try_build(
        &mesh,
        &graph,
        &regions,
        &plans,
        SEGMENTS,
        &CancelToken::new(),
        options(true),
    );

    let reason = match built {
        Ok(_) => panic!(
            "and asked for the CAP-RIDING rows instead it publishes nothing, rather than a surface built of two canals at once - instead it published something"
        ),
        Err(SurfaceBuildFailure::Refused(reason)) => reason,
        Err(other) => panic!("refused by name - instead {other:?}"),
    };

    assert_eq!(
        reason.code(),
        BevelAdvisoryCode::SurfaceStripSeamDidNotMeet,
        "naming the seam between that patch's last column and the bead's square end - instead {reason:?}"
    );
    let BevelAdvisoryArg::Vector3(at) = reason.args()[0] else {
        panic!("the seam is named by where it is - instead {reason:?}");
    };
    assert!(
        (at - Vector3::new(6.74, 6.1216, 10.7704)).length() < 1e-3,
        "on the bead's own seam, where the two curves part - instead {at:?}"
    );
}

/// How far the row at `turn` sweeps, from its first rail to its last, seen from the ball's centre.
fn swept_at(
    corner: &SurfaceMixedCorner,
    rows: &SurfaceMixedCornerRows,
    rails: usize,
    turn: f64,
) -> f64 {
    let centre = corner.spine.centre_at(turn);
    let first = (rows.at(0, turn) - centre).get_normal();
    let last = (rows.at(rails - 1, turn) - centre).get_normal();

    math_helper::radians_to_degrees(first.cross(last).length().atan2(first.dot(last)))
}

/// The ridge and the upper left crease, as the one chain they make
/// at the left ridge end.
pub fn the_plans_of_the_left_ridge_end(
    features: &FeatureEdgeSet,
    regions: &Arc<FaceRegionCache>,
) -> Vec<Rc<SurfaceBevelPlan>> {
    the_plans_of(
        regions,
        RADIUS,
        &[
            the_edge_from(
                features,
                nose_in_notch_test_meshes::left_ridge_end(),
                nose_in_notch_test_meshes::right_ridge_end(),
            ),
            the_edge_from(
                features,
                nose_in_notch_test_meshes::left_ridge_end(),
                nose_in_notch_test_meshes::upper_crease_end(),
            ),
        ],
    )
}
