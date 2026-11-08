//! WHICH CANAL a LEANING mixed corner draws,
//! read off the sheared L prism (`topology_test_meshes::create_sheared_l_shaped_grid_prism`).
//!
//! Both canals die on the SAME line - the cap's plane met with the wall's - and they travel
//! OPPOSITE ways along it. On the L the dual's way out of the corner is `-x`, into the notch's void,
//! where the part carries nothing; the cap-riding run-out's way is `+x`, ten millimetres of the
//! part's own convex rim. So the selector is by crease reach, and it routes this corner to the
//! cap-riding path. The lean is not an epsilon either: the cap-riding ball's centre runs on an
//! ellipse off the torus's circle by `2r (sec phi - 1)`.

use std::rc::Rc;

use solidmesh::bevel::surface::surface_mixed_corner_which_canal;
use solidmesh::bevel::surface::{SurfaceMixedCorner, SurfaceMixedCornerDualCanal};
use solidmesh_math::Vector3;

use crate::surface_mixed_corner_fixture as fixture;
use crate::surface_mixed_corner_reading_tests::the_mixed_corner;
use crate::topology_test_meshes::create_sheared_l_shaped_grid_prism;

/// Two thirds of the seam budget at 8 segments, and its whole limit - the two leans the app-side
/// cases carry.
const TWO_THIRDS_OF_THE_BUDGET: f64 = 5.625;

const THE_WHOLE_BUDGET: f64 = 8.4375;

/// A ten-thousandth, because the sheared cap goes through float storage and comes back.
const AS_SHEARED: f64 = 1e-4;

/// The reading, both ways round and at both leans: the cap leans, the DUAL's crease runs off the
/// corner into the notch's void with nothing to die on, and the CAP-RIDING run-out's crease is the
/// part's own rim running ten millimetres the other way.
fn the_ls_dual_runs_into_the_void_and_its_cap_riding_crease_is_the_rim(
    lean_degrees: f64,
    along_the_rim: bool,
) {
    let corner = the_sheared_corner(lean_degrees, along_the_rim);

    assert!(
        corner.the_cap_leans(),
        "shearing the prism tilts the bead's axis off the cap it rolls under, which is the whole \
         fixture"
    );
    let lean = corner.spine.cap_lean_degrees();
    assert!(
        (lean - lean_degrees).abs() <= AS_SHEARED,
        "and by exactly what it was sheared by - instead {lean}"
    );

    let dual = corner.try_read_the_dual_canal().unwrap_or_else(|why| {
        panic!(
            "the dual still READS here - what it has not got is anywhere to go - instead {why:?}"
        )
    });
    let duals_crease = SurfaceMixedCornerDualCanal::try_the_crease(&corner, &dual)
        .expect("the cap and the wall do meet in a line, so the dual has a direction to travel");

    assert!(
        duals_crease
            .direction
            .equals_within(Vector3::new(-1.0, 0.0, 0.0), AS_SHEARED),
        "and it is the way INTO the notch, where the L has no part at all - instead {:?}",
        duals_crease.direction
    );

    let duals_reach = surface_mixed_corner_which_canal::try_how_far_the_duals_crease_runs(&corner);
    assert!(
        duals_reach.is_none(),
        "so the dual's crease has no forward reach and the dual cannot be this corner's canal - \
         instead {duals_reach:?}"
    );

    let cap_riding_crease = surface_mixed_corner_which_canal::try_the_cap_riding_crease(&corner)
        .expect("the cap-riding run-out reads the same line signed by its own contact on the cap");
    assert!(
        cap_riding_crease
            .direction
            .equals_within(Vector3::new(1.0, 0.0, 0.0), AS_SHEARED),
        "which is the other way out of the corner - instead {:?}",
        cap_riding_crease.direction
    );

    let cap_riding_reach =
        surface_mixed_corner_which_canal::try_how_far_the_cap_riding_crease_runs(&corner).expect(
            "and THAT way the part carries the rim the square corner's own run-out dies on",
        );
    assert!(
        (cap_riding_reach - 10.0).abs() <= AS_SHEARED,
        "all ten millimetres of it, from the corner to the far end of the notch - instead \
         {cap_riding_reach}"
    );

    assert!(
        !corner.takes_the_dual_canal(),
        "so the selector routes this corner to the cap-riding path, and it does so by crease reach \
         rather than by how far the cap leans"
    );

    // And the plane the cut's band and end ring are cut at follows the routing, or a crescent of
    // the face is left standing between where the band ended and where the ring really is.
    let band_end = corner.band_end_for(&corner.cut);

    assert!(
        band_end
            .point
            .equals_within(corner.cut_stops_at.point, AS_SHEARED)
            && band_end
                .inward
                .equals_within(corner.cut_stops_at.inward, AS_SHEARED),
        "the cap-riding ring stands on the SQUARE handover - instead {:?} / {:?}",
        band_end.point,
        band_end.inward
    );

    if along_the_rim {
        // Which has teeth only where the two planes are two planes. Leaning ACROSS the rim the
        // dual's cut ring lies in the square handover anyway - it is the plane the cut's own
        // cylinder touches the cap along - and the routing moves nothing; leaning ALONG it the
        // dual's plane tilts 0.28 out of square, and THAT is the crescent a mis-routed corner
        // would leave.
        let ring = dual.cut_ring_plane();
        assert!(
            !ring.inward.equals_within(corner.cut_stops_at.inward, 1e-3),
            "the dual's own cut ring leans here, so taking the square plane is a choice and not a \
             coincidence - instead {:?}",
            ring.inward
        );
    }
}

#[test]
fn the_ls_dual_runs_into_the_void_and_its_cap_riding_crease_is_the_rim_5_625_false() {
    the_ls_dual_runs_into_the_void_and_its_cap_riding_crease_is_the_rim(
        TWO_THIRDS_OF_THE_BUDGET,
        false,
    );
}

#[test]
fn the_ls_dual_runs_into_the_void_and_its_cap_riding_crease_is_the_rim_5_625_true() {
    the_ls_dual_runs_into_the_void_and_its_cap_riding_crease_is_the_rim(
        TWO_THIRDS_OF_THE_BUDGET,
        true,
    );
}

#[test]
fn the_ls_dual_runs_into_the_void_and_its_cap_riding_crease_is_the_rim_8_4375_false() {
    the_ls_dual_runs_into_the_void_and_its_cap_riding_crease_is_the_rim(THE_WHOLE_BUDGET, false);
}

#[test]
fn the_ls_dual_runs_into_the_void_and_its_cap_riding_crease_is_the_rim_8_4375_true() {
    the_ls_dual_runs_into_the_void_and_its_cap_riding_crease_is_the_rim(THE_WHOLE_BUDGET, true);
}

/// The SQUARE L is not routed to the dual either, and for the same reason read at zero lean: its
/// crease runs the same way out of the corner whatever the shear is.
#[test]
fn the_square_l_is_not_the_dual_either() {
    let corner = the_mixed_corner();

    assert!(
        !corner.the_cap_leans(),
        "the square L's axis stands square to its cap, which is where the two canals are one torus"
    );
    assert!(
        !corner.takes_the_dual_canal(),
        "and a square corner keeps the cap-riding rows it was measured with"
    );
    assert!(
        surface_mixed_corner_which_canal::try_how_far_the_duals_crease_runs(&corner).is_none(),
        "the dual's crease runs into the notch's void here as it does at every lean - the \
         selector's answer is the shape's, not the angle's"
    );
}

/// The corner the sheared L's two picked edges meet at, read off their own plan.
fn the_sheared_corner(lean_degrees: f64, along_the_rim: bool) -> Rc<SurfaceMixedCorner> {
    let mesh = create_sheared_l_shaped_grid_prism(lean_degrees, along_the_rim);
    let (regions, features) = fixture::regions_of(&mesh);
    let plan = fixture::the_plan_on(&regions, &features);

    fixture::the_corner(&plan)
}
