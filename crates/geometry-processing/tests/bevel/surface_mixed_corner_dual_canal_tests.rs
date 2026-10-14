//! The leaning one-crease mixed corner of the
//! nose-in-notch part read WHOLE - the canal carried past the end of the cut's own strip until it
//! dies on the crease the cap and the wall meet in.

use std::rc::Rc;

use geometry_processing::bevel::surface::SurfaceMixedCornerDualCanal;
use geometry_processing::bevel::BevelAdvisoryCode;
use vector_math::math_helper::{degrees_to_radians, radians_to_degrees};
use vector_math::Vector3;

use crate::nose_in_notch_mixed_corners;
use crate::nose_in_notch_test_meshes;

const RADIUS: f64 = 1.0;

/// Four decimals, which is what the control's own measurements are quoted to.
const AS_MEASURED: f64 = 1e-4;

const EXACT: f64 = 1e-9;

/// How far the crease between the cap and the wall really runs on this part.
fn the_creases_own_length() -> f64 {
    (nose_in_notch_test_meshes::lower_crease_end() - nose_in_notch_test_meshes::left_ridge_end())
        .length()
}

fn the_canal() -> SurfaceMixedCornerDualCanal {
    let corner = nose_in_notch_mixed_corners::at_the_ridge_end(
        nose_in_notch_test_meshes::left_ridge_end(),
        RADIUS,
    );

    SurfaceMixedCornerDualCanal::try_read(&corner, the_creases_own_length())
        .unwrap_or_else(|why| panic!("the canal of the oblique corner would not read: {why:?}"))
}

/// The crease the blend runs out onto is the edge the CAP and the WALL meet in, derived from the
/// corner's own two planes rather than looked for in the mesh.
#[test]
fn the_crease_is_where_the_cap_and_the_wall_meet() {
    let canal = the_canal();
    let lower = (nose_in_notch_test_meshes::lower_crease_end()
        - nose_in_notch_test_meshes::left_ridge_end())
    .get_normal();

    assert!(
        canal
            .crease
            .point
            .equals_within(nose_in_notch_test_meshes::left_ridge_end(), EXACT),
        "it runs from the corner - instead {:?}",
        canal.crease.point
    );

    assert!(
        canal.crease.direction.equals_within(lower, 1e-6),
        "and out along the nose's LOWER crease - instead {:?} against {lower:?}",
        canal.crease.direction
    );

    assert!(
        (the_creases_own_length() - 7.8174).abs() <= AS_MEASURED,
        "instead {}",
        the_creases_own_length()
    );
}

/// The blend dies at 158.5036 degrees - 45.88 past the 112.62 the cut's own strip ends at - with
/// the ball's centre exactly one radius from the crease's LINE.
#[test]
fn it_dies_tangentially_on_that_crease() {
    let canal = the_canal();

    let dies_at = radians_to_degrees(canal.dies_at);
    assert!((dies_at - 158.5036).abs() <= 1e-3, "instead {dies_at}");

    let strip_ends = radians_to_degrees(canal.the_cut_strip_ends_at());
    assert!(
        (strip_ends - 112.6199).abs() <= 1e-3,
        "which is past where the CUT's strip ends - instead that is at {strip_ends}"
    );

    let off = canal
        .crease
        .distance_to(canal.dual.spine.centre_at(canal.dies_at));
    assert!(
        (off - RADIUS).abs() <= EXACT,
        "and the ball stands one radius off the crease's own line there, touching it at a point - instead {off}"
    );

    assert!(
        canal
            .dies_on()
            .equals_within(Vector3::new(7.9323, 7.9323, 8.0451), AS_MEASURED),
        "instead {:?}",
        canal.dies_on()
    );

    assert!(
        (canal.reach_along_the_crease() - 4.5846).abs() <= AS_MEASURED,
        "of the crease's own {:.4} - instead {}",
        the_creases_own_length(),
        canal.reach_along_the_crease()
    );
}

/// While the CUT's strip is still there the row ends ON that strip's cylinder; once it has ended
/// the row ends on the CAP's own plane instead, and the two agree exactly at the changeover.
#[test]
fn the_row_ends_on_the_cut_while_there_is_cut_and_on_the_cap_afterwards() {
    let canal = the_canal();
    let corner = Rc::clone(canal.corner());

    for station in 0..=32 {
        let turn = station as f64 * canal.the_cut_strip_ends_at() / 32.0;
        let ends = canal.row_ends_at(turn);

        let off = corner.cut_center_line().distance_to(ends);
        assert!(
            (off - RADIUS).abs() <= EXACT,
            "at {:.2} degrees the row ends on the cut's own cylinder - instead {ends:?}",
            radians_to_degrees(turn)
        );
    }

    for station in 1..=32 {
        let turn = canal.the_cut_strip_ends_at()
            + (station as f64 * (canal.dies_at - canal.the_cut_strip_ends_at()) / 32.0);
        let ends = canal.row_ends_at(turn);

        let off = (ends - corner.corner).dot(corner.out_of_the_cap_face);
        assert!(
            off.abs() <= EXACT,
            "and at {:.2} degrees it ends on the cap's own plane - instead {ends:?}",
            radians_to_degrees(turn)
        );
    }

    // The changeover point itself, which both readings have to give.
    let changeover = canal.row_ends_at(canal.the_cut_strip_ends_at());

    assert!(
        changeover.equals_within(Vector3::new(7.10759, 6.24808, 9.16795), AS_MEASURED),
        "instead {changeover:?}"
    );

    let off = (changeover - corner.corner).dot(corner.out_of_the_cap_face);
    assert!(
        off.abs() <= 1e-9,
        "and it stands on the cap's plane as well as on the cut's tube - instead {changeover:?}"
    );
}

/// The row's extent closes to NOTHING as the turn runs out, so the patch's last column is a point
/// on the crease and the blend fades away rather than ending on an edge of its own.
#[test]
fn the_rows_extent_closes_to_nothing_at_the_end() {
    let canal = the_canal();

    for (degrees, reaches) in [
        (112.9, 66.6),
        (115.0, 64.2),
        (117.5, 61.3),
        (119.9, 58.3),
        (123.3, 53.7),
        (126.8, 49.5),
        (130.5, 44.4),
        (133.3, 40.4),
        (139.8, 31.0),
        (145.5, 22.4),
        (151.9, 11.8),
        (157.8, 1.3),
    ] {
        let runs_to = radians_to_degrees(canal.row_runs_to(degrees_to_radians(degrees)));

        assert!(
            (runs_to - reaches).abs() <= 0.5,
            "at {degrees} degrees of turn the control's own row reaches {reaches} - instead this one runs to {runs_to:.3}"
        );
    }

    let last = radians_to_degrees(canal.row_runs_to(canal.dies_at));
    assert!(last.abs() <= 1e-6, "instead {last}");
}

/// A crease too short for the blend to die on is refused BY NAME rather than run off the end of.
#[test]
fn a_crease_too_short_to_die_on_is_refused() {
    let corner = nose_in_notch_mixed_corners::at_the_ridge_end(
        nose_in_notch_test_meshes::left_ridge_end(),
        RADIUS,
    );

    let read = SurfaceMixedCornerDualCanal::try_read(&corner, 4.5);

    let why = match read {
        Ok(_) => {
            panic!("the blend dies 4.5846 along the crease and the crease was said to run 4.5")
        }
        Err(why) => why,
    };

    assert_eq!(
        why.code(),
        BevelAdvisoryCode::SurfaceMixedCornerRunOutPastTheCrease,
        "instead {why:?}"
    );

    assert!(
        SurfaceMixedCornerDualCanal::try_read(&corner, 4.6).is_ok(),
        "and a crease a tenth longer than the reach carries it"
    );
}
