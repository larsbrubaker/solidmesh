//! The window a band really occupies along its own edge -
//! not the edge's span, but the span its MITRES leave it - read on a cap with a V notched into one
//! side and a tongue hanging over the other, whose foot stands 10 mm short of the notch's apex.
//!
//! Two strips meeting at a corner are cut at the plane between them, which carries the band on the
//! OUTSIDE of the turn past the end of its own edge - a whole setback of it at a right-angled reflex
//! corner, 1.6 of one at the 296 degree apex here. And what stands in the window has to be read as
//! the footprint it is: a mitre tip reaches ONE way, and the tip at this fixture's apex stands 10 mm
//! past the tongue's foot with nothing leaning towards it, so the cap takes 5 mm and closes.
//!
//! Legacy-only: every test pins the sections router off (`pinned_to_the_legacy_construction`).
//! The parameter rows are a loop.

use std::rc::Rc;
use std::sync::Arc;

use crate::path_solids::extrude;
use crate::path_solids::Outline;
use solidmesh::bevel::surface::{
    surface_bevel_builder, surface_mesh_closure, surface_room, surface_room_measure,
    SurfaceBevelPlan, SurfaceRoomCut, SurfaceRoomFootprint,
};
use solidmesh::bevel::{bevel_chain_linker, BevelAdvisoryCode};
use solidmesh::topology::{FaceGroup, FaceRegionCache};
use solidmesh_math::Vector3;
use solidmesh_mesh::{CancelToken, Mesh};

use crate::mesh_measurements::shell_count;
use crate::surface_test_build::pinned_to_the_legacy_construction;
use crate::surface_test_chains::join_and_open;

/// The bend threshold the fixture is read at - every corner of it is square or sharper.
const THRESHOLD_DEGREES: f64 = 10.0;

/// How far past its own end the notch legs' bands reach at the apex: tan(58) = 1.6003 of the
/// setback at 296 degrees of material.
const PAST_THE_APEX: f64 = 1.6003;

/// A band's window is its edge's span widened by its own mitre at each end - one setback past the
/// tongue's right-angled reflex foot, 1.6 past the notch's apex, and nothing at all at the corners
/// the mitre cuts it BACK at.
#[test]
fn a_bands_window_is_its_edges_span_widened_by_its_own_mitres() {
    let _pinned = pinned_to_the_legacy_construction();

    let prism = tongued_prism();
    let cuts = the_caps_cuts(&prism, 4.0);

    let tongue = the_caps_band_from(&cuts, the_tongues_foot());

    assert!(
        (tongue.overhang_before_from - tongue.setback_per_size).abs() <= 1e-6,
        "the foot is a 270 degree corner of the cap, so the band reaches a whole setback past it - \
         instead {}",
        tongue.overhang_before_from
    );

    assert_eq!(
        tongue.overhang_past_to, 0.0,
        "and at its far end the cap turns the other way, where the mitre cuts the band back rather \
         than carrying it on"
    );

    let leg = the_caps_band_from(&cuts, Vector3::new(15.0, 0.0, 0.0));

    assert!(
        (leg.overhang_past_to - leg.setback_per_size * PAST_THE_APEX).abs() <= 1e-3,
        "the apex is 296 degrees of material, so the leg's band reaches tan(58) of the setback past \
         it - instead {}",
        leg.overhang_past_to
    );

    assert_eq!(
        leg.overhang_before_from, 0.0,
        "where the leg leaves the side it is a square corner, and a square corner takes the band back"
    );
}

/// And a footprint standing past the end of a band is charged against it only where it REACHES it.
/// The apex's wedge stands 10 mm past the tongue's foot and reaches straight up the notch's
/// bisector, so the two never meet: the cap takes 5 mm and closes into one manifold shell.
#[test]
fn a_wedge_leaning_past_a_band_is_not_a_collision_with_it() {
    let _pinned = pinned_to_the_legacy_construction();

    let prism = tongued_prism();
    let graph = prism.get_mesh_edge_graph();
    let regions = regions_of(&prism);
    let plans = the_caps_rim(&prism, &regions, 5.0);

    let tips: Vec<SurfaceRoomCut> = surface_room_measure::measure(&plans, None, None)
        .into_iter()
        .filter(|cut| {
            cut.shape == SurfaceRoomFootprint::MitreTip && (cut.from - the_apex()).length() < 1e-6
        })
        .collect();

    assert_eq!(tips.len(), 1, "one mitre tip stands at the apex");

    let tip = tips[0];

    assert!(
        (tip.inward.dot(Vector3::UNIT_Y) - 1.0).abs() <= 1e-9,
        "the notch is symmetric, so its wedge reaches straight up the bisector - instead {:?}",
        tip.inward
    );

    assert!(
        ((the_apex() - the_tongues_foot()).dot(Vector3::UNIT_X) - 10.0).abs() <= 1e-6,
        "and it stands 10 mm along the cap from the foot of the tongue's band"
    );

    let fit = surface_room::try_fit(&plans, 5.0, &CancelToken::new(), None)
        .expect("nothing cancels this fit");

    assert!(
        fit.fits,
        "nothing the wedge reaches has been taken, so 5 fits - {:?}, which allowed {}",
        fit.refusal, fit.largest_fitting_size
    );

    let built = surface_bevel_builder::try_build(
        &prism,
        &graph,
        &regions,
        &plans,
        8,
        &CancelToken::new(),
        crate::surface_test_build::build_options(),
    )
    .unwrap_or_else(|reason| panic!("and the build says the same - {reason:?}"))
    .mesh;

    assert_closed_manifold_one_shell(&built);
}

/// And the size a refusal NAMES here is one the user can then ask for: planned at it and measured
/// again, the fit accepts it and the cap builds.
#[test]
fn the_size_the_refusal_names_is_a_size_the_fit_then_accepts() {
    let _pinned = pinned_to_the_legacy_construction();

    for asked in [7.0, 10.0] {
        let prism = tongued_prism();
        let graph = prism.get_mesh_edge_graph();
        let regions = regions_of(&prism);

        let fit = surface_room::try_fit(
            &the_caps_rim(&prism, &regions, asked),
            asked,
            &CancelToken::new(),
            None,
        )
        .expect("nothing cancels this fit");

        assert!(
            !fit.fits,
            "{asked} is more cap than there is - instead it fit"
        );

        let refusal = fit
            .refusal
            .expect("a fit that does not fit names its reason");

        assert_eq!(
            refusal.code(),
            BevelAdvisoryCode::SizeCollidesAcrossFace,
            "the two bands meet across the cap - instead {refusal:?}"
        );

        let named = fit.largest_fitting_size;

        assert!(
            named > 0.0 && named < asked,
            "the named size {named} is a size below {asked}"
        );

        let at_the_named_size = the_caps_rim(&prism, &regions, named);

        let again = surface_room::try_fit(&at_the_named_size, named, &CancelToken::new(), None)
            .expect("nothing cancels this fit");

        assert!(
            again.fits,
            "the named size, asked for, has to fit - instead {:?}",
            again.refusal
        );

        let built = surface_bevel_builder::try_build(
            &prism,
            &graph,
            &regions,
            &at_the_named_size,
            8,
            &CancelToken::new(),
            crate::surface_test_build::build_options(),
        )
        .unwrap_or_else(|reason| panic!("and build - {reason:?}"))
        .mesh;

        assert_closed_manifold_one_shell(&built);
    }
}

fn assert_closed_manifold_one_shell(built: &Mesh) {
    let (open, open_at) = surface_mesh_closure::open_edge_count(built);

    assert_eq!(open, 0, "open at {open_at:?}");
    assert!(built.is_manifold());
    assert_eq!(shell_count(built), 1);
}

/// Where the notch's apex stands, which is the reflex corner the wedge reaches out of.
fn the_apex() -> Vector3 {
    Vector3::new(20.0, 8.0, 0.0)
}

/// Where the tongue's band starts: its own 270 degree reflex foot, 10 mm along from the apex.
fn the_tongues_foot() -> Vector3 {
    Vector3::new(10.0, 20.0, 0.0)
}

pub fn regions_of(prism: &Mesh) -> Arc<FaceRegionCache> {
    Arc::new(FaceRegionCache::new(
        Arc::new(prism.clone()),
        Some(prism.get_mesh_edge_graph()),
        THRESHOLD_DEGREES,
    ))
}

/// The band this cap gives up beside the edge leaving `from`.
fn the_caps_band_from(cuts: &[SurfaceRoomCut], from: Vector3) -> SurfaceRoomCut {
    let found: Vec<&SurfaceRoomCut> = cuts
        .iter()
        .filter(|cut| {
            cut.shape == SurfaceRoomFootprint::Band
                && cut.inward.z.abs() < 1e-6
                && (cut.from - from).length() < 1e-6
        })
        .collect();

    assert_eq!(found.len(), 1, "one band of the cap leaves {from:?}");

    *found[0]
}

/// Every cut the cap's own rim makes, planned at one size.
fn the_caps_cuts(prism: &Mesh, size: f64) -> Vec<SurfaceRoomCut> {
    surface_room_measure::measure(&the_caps_rim(prism, &regions_of(prism), size), None, None)
}

/// The cap's rim, joined into chains and planned at one size.
pub fn the_caps_rim(
    prism: &Mesh,
    regions: &Arc<FaceRegionCache>,
    size: f64,
) -> Vec<Rc<SurfaceBevelPlan>> {
    let cap = the_cap(prism);

    join_and_open(&bevel_chain_linker::link(cap.boundary_edges()))
        .iter()
        .map(|chain| {
            SurfaceBevelPlan::try_create(chain, regions, size, true).unwrap_or_else(|reason| {
                panic!("the cap's rim has to plan at {size}: {:?}", reason.code())
            })
        })
        .collect()
}

/// The cap both readings are made on: the prism's bottom, which is where the notch's apex is.
pub fn the_cap(prism: &Mesh) -> FaceGroup {
    let seed = (0..prism.faces.len())
        .find(|&face| {
            (Vector3::from_vector3_float(prism.faces[face].normal) + Vector3::UNIT_Z).length()
                < 1e-6
        })
        .expect("the prism has a bottom") as i32;

    FaceGroup::create_with_tolerance(prism, seed, THRESHOLD_DEGREES).expect("the cap floods")
}

/// A 40 by 36 rectangle with a V notched 8 mm into the middle of one side and a tongue of void
/// hanging 16 mm down over the other, extruded 10 tall.
fn tongued_prism() -> Mesh {
    let mut path = Outline::new();

    path.move_to(0.0, 0.0);
    path.line_to(15.0, 0.0);
    path.line_to(20.0, 8.0);
    path.line_to(25.0, 0.0);
    path.line_to(40.0, 0.0);

    // 36 rather than 40 only because the apex, the notch's own left leg and the far corner are
    // collinear at 40, which leaves the cap a triangle of no area to be flooded through.
    path.line_to(40.0, 36.0);
    path.line_to(10.0, 36.0);
    path.line_to(10.0, 20.0);
    path.line_to(0.0, 20.0);
    path.close_polygon();

    extrude(&path, 10.0).expect("the prism extrudes")
}
