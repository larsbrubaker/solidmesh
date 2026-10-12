//! Ports `GeometryProcessingTests/BevelShadowClipTests.cs`: a RIB standing on the face a fillet is
//! landing on - the general case of "Encroaching Side.mcx". The rolling ball never collides with
//! the rib; what goes wrong is only the CUTTER, a solid that fills the whole corner and so takes
//! the column under the rib's foot. The clip is the rib's FOOTPRINT on the face extruded down
//! through the cutter (a SHADOW), not a half space through one of its faces, which would hand back
//! most of the cut. The fixture is `create_rib_near_a_top_edge`: a 20 mm cube, the top edge at
//! x = 0 rounded at 5, and a rib 2 wide and 4 tall on the top face - inside the setback at 1, beyond
//! it at 8.

use std::rc::Rc;
use std::sync::Arc;

use geometry_processing::bevel::bevel_face_fit::DEFAULT_SURFACE_TOLERANCE_DEGREES;
use geometry_processing::bevel::bevel_group_tools::{self, BevelGroupToolsOptions};
use geometry_processing::bevel::bevel_shadow_clip::{self, Obstruction};
use geometry_processing::bevel::{BevelFaceFit, BevelTool};
use geometry_processing::topology::bevel_edge_group_resolver;
use geometry_processing::topology::{BevelSelectionKind, FaceGroup, FeatureEdgeSet};
use polygon_mesh::Mesh;
use vector_math::{Vector2, Vector3};

use crate::bevel_obstruction_test_meshes::{
    create_rib_and_bridge_near_a_top_edge, create_rib_near_a_top_edge,
};
use crate::bevel_volume_probes::{probe_cube, volume_inside};
use crate::kernel_cut_helpers::{cut, edge_index};

const CUBE_SIZE: f64 = 20.0;

/// The fillet's radius, which on this 90 degree edge is also its setback across each face.
const RADIUS: f64 = 5.0;
const SEGMENTS: i32 = 8;

/// The rib's near wall, one millimetre from the selected edge - well inside the setback.
const RIB_INSIDE_THE_BAND: f64 = 1.0;

/// The same rib beyond the setback, where the walk never meets it.
const RIB_OUTSIDE_THE_BAND: f64 = 8.0;

/// How big a cube is used to ask whether a point is inside the solid.
const PROBE_SIDE: f64 = 0.05;

/// What a probe reads where the point is well inside. Half of a full probe is the threshold either
/// way, which is far outside anything a chorded cylinder or a boolean's rounding could produce at
/// these distances from a surface.
const FULL_PROBE: f64 = PROBE_SIDE * PROBE_SIDE * PROBE_SIDE;

/// The round's axis: one radius in from each of the two faces it is tangent to.
fn axis() -> Vector2 {
    Vector2::new(RADIUS, CUBE_SIZE - RADIUS)
}

#[test]
fn a_rib_inside_the_band_is_kept_and_the_round_is_trimmed_to_it() {
    let part = create_rib_near_a_top_edge(RIB_INSIDE_THE_BAND);
    let (admitted, obstructions, tools) = round(&part);

    assert!(
        admitted,
        "the rib is material STANDING ON the top face, not the top face running out - the ball \
         never touches it, so there is nothing here to refuse"
    );
    assert!(
        !obstructions.is_empty(),
        "admitting the group is only half the answer: unless the obstruction is recorded, the \
         cutter is never clipped and the round saws the rib off"
    );

    let result = cut(
        &part,
        &tools.expect("admitted")[0].mesh,
        &shadows(&part, &obstructions),
    );

    // The rib's own material, well above the face it stands on. A cutter that reached past the top
    // face - which it does, by its own stand-off - shaves this off at the tangent line.
    assert!(
        inside(&result, Vector3::new(2.0, 10.0, 22.0)) > FULL_PROBE / 2.0,
        "the rib stands 4 mm above the face and none of it is the fillet's to take"
    );

    // The column under the rib, between the top face's plane at z = 20 and the cylinder at
    // z = 19.0. This is the undercut, and it is the whole of what the shadow clip is for.
    assert!(
        inside(&result, Vector3::new(2.0, 10.0, 19.4)) > FULL_PROBE / 2.0,
        "the cylinder about ({}, {}) passes z = {:.3} at x = 2, so the material between it and \
         z = 20 is the column the rib stands on - a rib whose foot has been undercut is hanging in \
         the air, which is not what any kernel does",
        axis().x,
        axis().y,
        cylinder_top(2.0)
    );

    // And the round itself, on both sides of the rib, which is what says the clip took the column
    // and not the cut. Nearer the edge than the rib:
    assert!(
        inside(&result, Vector3::new(0.5, 10.0, 18.0)) < FULL_PROBE / 2.0,
        "at x = 0.5 the cylinder is at z = {:.3} and there is no rib overhead, so this is cut away \
         - a shadow that spread past the rib's own footprint would have handed the round back here",
        cylinder_top(0.5)
    );
    assert!(
        inside(&result, Vector3::new(0.5, 10.0, 16.5)) > FULL_PROBE / 2.0,
        "below the cylinder the part is untouched, whatever the clip did above it"
    );

    // And beyond it, where the round runs on to its tangent line at x = 5:
    assert!(
        inside(&result, Vector3::new(4.0, 10.0, 19.95)) < FULL_PROBE / 2.0,
        "at x = 4 the cylinder is at z = {:.3} and the rib's footprint ended at x = 3, so the round \
         is still cutting here",
        cylinder_top(4.0)
    );
}

#[test]
fn a_rib_beyond_the_setback_is_not_an_obstruction_and_nothing_is_clipped() {
    let part = create_rib_near_a_top_edge(RIB_OUTSIDE_THE_BAND);
    let (admitted, obstructions, _) = round(&part);

    assert!(
        admitted,
        "the walk reaches its whole setback of 5 before it ever meets a rib standing at 8"
    );
    assert_eq!(
        obstructions.len(),
        0,
        "nothing stopped the walk short, so no clip is asked for and every part that has always \
         built goes on building exactly as it did - the gate is what keeps the clip off them, not \
         the emptiness of some footprint"
    );
}

#[test]
fn a_roof_over_the_band_casts_no_shadow_though_a_rib_beside_it_does() {
    let part = create_rib_and_bridge_near_a_top_edge();
    let (admitted, obstructions, tools) = round(&part);

    assert!(
        admitted,
        "the rib is standing on the face, which is admitted"
    );
    assert!(
        !obstructions.is_empty(),
        "the rib is what makes the clip run at all - without a recorded obstruction the footprint \
         rule is never asked anything and this fixture would pass on a technicality"
    );

    let result = cut(
        &part,
        &tools.expect("admitted")[0].mesh,
        &shadows(&part, &obstructions),
    );

    // Under the rib: kept, exactly as in the rib-only fixture.
    assert!(
        inside(&result, Vector3::new(2.0, 10.0, 19.4)) > FULL_PROBE / 2.0,
        "the rib stands on this column, so it is the rib's"
    );

    // Under the DECK but clear of the rib: cut. This is the assertion the whole fixture exists for.
    assert!(
        inside(&result, Vector3::new(4.0, 10.0, 19.95)) < FULL_PROBE / 2.0,
        "at x = 4 the cylinder is at z = {:.3}, the rib ended at x = 3, and the deck 6 mm overhead \
         rests on a post outside the band - it is not standing on this material and cannot be \
         undercut by this cut, so it has no claim on the column beneath it",
        cylinder_top(4.0)
    );
    assert!(
        inside(&result, Vector3::new(4.0, 10.0, 28.0)) > FULL_PROBE / 2.0,
        "the deck itself is nobody's to cut - it is nowhere near the fillet"
    );
}

/// The round's surface over the top face: the cylinder tangent to z = 20 at x = 5.
fn cylinder_top(x: f64) -> f64 {
    let axis = axis();
    axis.y
        + ((RADIUS * RADIUS) - ((x - axis.x) * (x - axis.x)))
            .max(0.0)
            .sqrt()
}

/// Resolves the top edge at x = 0, asks `BevelFaceFit` whether it fits and what stands in its way,
/// and sweeps the tools - the same three steps a rebuild takes, in the same order.
fn round(part: &Mesh) -> (bool, Vec<Obstruction>, Option<Vec<BevelTool>>) {
    let features = FeatureEdgeSet::create(part);
    let edge = edge_index(
        &features,
        Vector3::new(0.0, 0.0, CUBE_SIZE),
        Vector3::new(0.0, CUBE_SIZE, CUBE_SIZE),
        1e-5,
    );
    let group = bevel_edge_group_resolver::create_from_edge(
        &features,
        edge,
        BevelSelectionKind::Edge,
        FaceGroup::DEFAULT_CURVE_TOLERANCE_DEGREES,
    )
    .expect("an edge of the set");
    let resolution = bevel_edge_group_resolver::resolve(&group, part);

    let check = BevelFaceFit::create(
        Arc::new(part.clone()),
        Rc::new(vec![resolution.clone()]),
        true,
        RADIUS,
        DEFAULT_SURFACE_TOLERANCE_DEGREES,
        None,
    )
    .try_check_group_fully(0);
    let admitted = check.fits();

    let tools = admitted.then(|| {
        bevel_group_tools::try_build(
            &resolution,
            true,
            RADIUS,
            SEGMENTS,
            BevelGroupToolsOptions::default(),
        )
        .map(|built| built.tools)
        .unwrap_or_default()
    });

    (admitted, check.standing_on_a_face, tools)
}

/// `BevelShadowClip.Build` with the bodies, as the C# test asks for it.
fn shadows(part: &Mesh, obstructions: &[Obstruction]) -> Vec<Mesh> {
    bevel_shadow_clip::build(part, &part.get_mesh_edge_graph(), obstructions, true, 1.0)
        .expect("the clipper takes the outline")
        .0
}

fn inside(solid: &Mesh, at: Vector3) -> f64 {
    volume_inside(solid, &probe_cube(at, PROBE_SIDE))
}
