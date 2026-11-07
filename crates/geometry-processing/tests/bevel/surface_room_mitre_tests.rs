//! The room two bands take between them where they MITRE at a
//! REFLEX corner of the cap they share - a 40 by 20 prism with a V notched into one side, whose
//! apex stands at 296 degrees of material and 12 mm from the far side.
//!
//! Two bands meeting at a reflex corner reach as far as their two offset lines CROSS, which stands
//! PAST the corner at `setback / sin(half the interior angle)` - 1.8868 times the setback, up the
//! notch's bisector. The fit charges that wedge against the far side's band. The cap's rim and its
//! regions are read with the helpers `surface_room_mitre_window_tests` reads its own notch with.

use crate::path_solids::extrude;
use crate::path_solids::Outline;
use geometry_processing::bevel::surface::{
    surface_bevel_builder, surface_mesh_closure, surface_room, surface_room_measure,
    SurfaceBevelPlan, SurfaceBuildOptions, SurfaceRoomFootprint,
};
use geometry_processing::bevel::{BevelAdvisoryArg, BevelAdvisoryCode};
use polygon_mesh::{CancelToken, Mesh};
use vector_math::Vector3;

use crate::mesh_measurements::shell_count;
use crate::surface_room_mitre_window_tests::{regions_of, the_cap, the_caps_rim};

/// Where the notch's apex stands, which is the reflex corner this class is about.
fn the_apex() -> Vector3 {
    Vector3::new(20.0, 8.0, 0.0)
}

/// How far the wedge reaches out of the apex per unit of setback: 1 / sin(148) = 1.8868.
const REACH_PER_SETBACK: f64 = 1.8868;

/// How far the apex stands from the side the cap runs across to, which is the gap they share.
const ACROSS_THE_CAP: f64 = 12.0;

/// The size the two of them meet at: the gap over what the two of them take per unit of size.
const THEY_MEET_AT: f64 = ACROSS_THE_CAP / (1.0 + REACH_PER_SETBACK);

/// The corner reads as a MITRE TIP on the cap, seated at the apex, reaching up the notch's
/// bisector by 1.8868 of the setback. Its room is infinite: only another cut stops the tip.
#[test]
fn the_reflex_corners_mitre_is_a_cut_on_the_cap() {
    let prism = notched_prism();
    let cuts =
        surface_room_measure::measure(&the_caps_rim(&prism, &regions_of(&prism), 3.0), None, None);

    let tips: Vec<_> = cuts
        .iter()
        .filter(|cut| cut.shape == SurfaceRoomFootprint::MitreTip)
        .collect();
    assert_eq!(tips.len(), 1);
    let tip = tips[0];

    assert_eq!(
        tip.region_key,
        SurfaceBevelPlan::key_of(&the_cap(&prism)),
        "the wedge is laid on the face the two bands share"
    );

    assert!(
        (tip.from - the_apex()).length() < 1e-6,
        "the corner is the base the reach is measured from - instead {:?}",
        tip.from
    );

    assert!(
        (tip.inward.dot(Vector3::UNIT_Y) - 1.0).abs() <= 1e-9,
        "the notch is symmetric, so its two offset lines cross straight up its bisector - instead {:?}",
        tip.inward
    );

    assert!(
        (tip.setback_per_size - REACH_PER_SETBACK).abs() <= 1e-4,
        "a 296 degree corner reaches 1 / sin(148) of the setback past itself - instead {}",
        tip.setback_per_size
    );

    assert!(
        tip.room.is_infinite() && tip.room > 0.0,
        "the bands it belongs to have already been measured against the face; only another cut stops the tip"
    );
}

/// So the fit REFUSES the size whose wedge reaches the far side's band, and names the size the two
/// of them share the cap at.
#[test]
fn the_tip_and_the_far_sides_band_share_the_cap_between_them() {
    let prism = notched_prism();
    let plans = the_caps_rim(&prism, &regions_of(&prism), 4.2);

    let fit = surface_room::try_fit(&plans, 4.2, &CancelToken::new(), None)
        .expect("nothing cancels the fit");

    assert!(
        !fit.fits,
        "1.8868 of wedge and 1 of band do not both fit in {ACROSS_THE_CAP} at 4.2 - it allowed {}",
        fit.largest_fitting_size
    );

    let refusal = fit.refusal.expect("a refused fit names why");

    assert_eq!(
        refusal.code(),
        BevelAdvisoryCode::SizeCollidesAcrossFace,
        "the face is wide enough; what is in the way is the selection's own wedge arriving from the notch"
    );

    let BevelAdvisoryArg::Vector3(at) = refusal.args()[1] else {
        panic!("the refusal is reported at a point - instead {refusal:?}");
    };
    assert!(
        (at - the_apex()).length() < 1e-6,
        "and the refusal is reported at the corner the wedge stands at - instead {at:?}"
    );

    assert!(
        (fit.largest_fitting_size - THEY_MEET_AT).abs() <= 1e-3,
        "which comes to 12 / (1 + 1.8868) = {THEY_MEET_AT} - instead {}",
        fit.largest_fitting_size
    );
}

/// And the refusal is the BUILD's own boundary rather than a margin taken for safety: at 4.15 the
/// cap rim closes into one manifold shell, and at 4.2 the build itself cannot sew the seam.
fn the_size_the_fit_accepts_is_the_size_that_closes(size: f64, fits: bool) {
    let prism = notched_prism();
    let graph = prism.get_mesh_edge_graph();
    let regions = regions_of(&prism);
    let plans = the_caps_rim(&prism, &regions, size);

    let fit = surface_room::try_fit(&plans, size, &CancelToken::new(), None)
        .expect("nothing cancels the fit");
    assert_eq!(fit.fits, fits);

    let built = surface_bevel_builder::try_build(
        &prism,
        &graph,
        &regions,
        &plans,
        8,
        &CancelToken::new(),
        SurfaceBuildOptions {
            crossings: None,
            ran_off_over_a_stretch: None,
            lay_the_oblique_rows_anyway: false,
            planless_loops: &[],
            features: None,
        },
    );

    if fits {
        let mesh = built
            .unwrap_or_else(|reason| panic!("the size the fit accepts has to build - {reason:?}"))
            .mesh;

        let (open, open_at) = surface_mesh_closure::open_edge_count(&mesh);
        assert_eq!(open, 0, "open at {open_at:?}");
        assert!(mesh.is_manifold());
        assert_eq!(shell_count(&mesh), 1);
    } else {
        assert!(
            built.is_err(),
            "the size the fit refuses is a size whose wedge stands on cap the far side's band has taken"
        );
    }
}

#[test]
fn the_size_the_fit_accepts_is_the_size_that_closes_4_15() {
    the_size_the_fit_accepts_is_the_size_that_closes(4.15, true);
}

#[test]
fn the_size_the_fit_accepts_is_the_size_that_closes_4_2() {
    the_size_the_fit_accepts_is_the_size_that_closes(4.2, false);
}

/// A 40 by 20 rectangle with a V notched 8 mm into the middle of one side, extruded 10 tall. The
/// apex of the notch is a corner of 296 degrees of material and stands 12 mm from the far side.
fn notched_prism() -> Mesh {
    let mut path = Outline::new();

    path.move_to(0.0, 0.0);
    path.line_to(15.0, 0.0);
    path.line_to(20.0, 8.0);
    path.line_to(25.0, 0.0);
    path.line_to(40.0, 0.0);
    path.line_to(40.0, 20.0);
    path.line_to(0.0, 20.0);
    path.close_polygon();

    extrude(&path, 10.0).expect("the prism extrudes")
}
