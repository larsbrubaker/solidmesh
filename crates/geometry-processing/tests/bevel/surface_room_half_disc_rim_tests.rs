//! A RIM IS NOT ITS OWN WALL. The flat end of a half
//! disc prism has one closed rim - the diameter and the nineteen chords of the arc. Picked whole,
//! every chord is one the same pass is beveling, so none of them is a wall the diameter's band
//! could run off at; what the two sides take out of the face between them is the pair
//! negotiation's to answer (`surface_room::largest_fitting`), not the room reading's.

use std::f64::consts::PI;
use std::rc::Rc;
use std::sync::Arc;

use solidmesh::bevel::bevel_chain_linker;
use solidmesh::bevel::surface::{
    surface_room, surface_room_measure, SurfaceBevelPlan, SurfaceRoomFootprint,
};
use solidmesh::bevel::BevelAdvisoryCode;
use solidmesh::topology::{FaceRegionCache, FeatureEdge, FeatureEdgeSet};
use solidmesh_mesh::CancelToken;

use crate::surface_mitre_consumes_a_chord_tests::half_disc_prism;

/// A mesh keeps its vertices as floats, so a distance of 10 is a distance of 10 to about this.
const NEAR: f64 = 1e-4;

/// How many chords the arc is drawn with, which is the fixture's own tessellation.
const FACETS: i32 = 19;

/// The disc's radius. The arc is inscribed, so the face is 9.966 deep rather than 10.
const RADIUS: f64 = 10.0;

/// How deep the face really is, which is the inscribed arc's rise over the diameter.
fn face_depth() -> f64 {
    RADIUS * (PI / (2.0 * FACETS as f64)).cos()
}

#[test]
fn the_diameter_has_the_whole_face_in_front_of_it_and_not_the_arcs_second_chord() {
    let plan = plan_of_the_rim(2.0);

    let cuts = surface_room_measure::measure(std::slice::from_ref(&plan), None, None);

    let diameters: Vec<_> = cuts
        .iter()
        .filter(|cut| {
            cut.shape == SurfaceRoomFootprint::Band
                && (cut.length - (2.0 * RADIUS)).abs() < NEAR
                && cut.inward.y > 0.99
        })
        .collect();
    assert_eq!(diameters.len(), 1);
    let diameter = diameters[0];

    assert!(
        diameter.room >= face_depth() - NEAR,
        "the only thing in front of the diameter on that face is the arc, which the same pass is beveling, and the face is {:.3} deep - it measured {:.3}",
        face_depth(),
        diameter.room
    );

    let fit = surface_room::try_fit(&[plan], 2.0, &CancelToken::new(), None).expect("no cancel");
    assert!(
        fit.fits,
        "a 2 mm fillet fits in a face a 5 mm circle fits in - {:?}",
        fit.refusal
    );
    assert!((fit.largest_fitting_size - 2.0).abs() <= NEAR);
}

/// And the size that does NOT fit is still refused by the reading that owns it: the diameter's band
/// and the far side of the arc's band split the face depth between them.
#[test]
fn too_large_a_fillet_is_refused_by_what_the_two_sides_of_the_face_leave_each_other() {
    let plan = plan_of_the_rim(6.0);

    let fit = surface_room::try_fit(&[plan], 6.0, &CancelToken::new(), None).expect("no cancel");
    assert!(
        !fit.fits,
        "a 6 mm fillet cannot fit in a face that holds a {:.3} mm circle",
        face_depth() / 2.0
    );
    assert!(
        (fit.largest_fitting_size - (face_depth() / 2.0)).abs() <= NEAR,
        "the two sides of the face split its {:.3} between them, and it said {}",
        face_depth(),
        fit.largest_fitting_size
    );
    assert_eq!(
        fit.refusal.expect("a refusal").code(),
        BevelAdvisoryCode::SizeCollidesAcrossFace,
        "it is the bevel coming the other way across the face that is in the way, not a wall"
    );
}

/// The whole rim of the flat face, planned as the one closed chain it is.
fn plan_of_the_rim(size: f64) -> Rc<SurfaceBevelPlan> {
    let mesh = half_disc_prism(FACETS, RADIUS, 20.0);
    let features = FeatureEdgeSet::create(&mesh);
    let top = mesh
        .vertices
        .iter()
        .map(|vertex| vertex.z as f64)
        .fold(f64::NEG_INFINITY, f64::max);

    let rim: Vec<FeatureEdge> = features
        .edges()
        .iter()
        .filter(|edge| (edge.start.z - top).abs() < 1e-4 && (edge.end.z - top).abs() < 1e-4)
        .copied()
        .collect();

    assert_eq!(
        rim.len(),
        (FACETS + 1) as usize,
        "the flat end's rim is the diameter and {FACETS} chords"
    );

    let regions = Arc::new(FaceRegionCache::new(
        Arc::new(mesh.clone()),
        Some(mesh.get_mesh_edge_graph()),
        features.flat_angle_degrees(),
    ));
    let mut chains = bevel_chain_linker::link(&rim);
    assert_eq!(chains.len(), 1);
    let chain = chains.remove(0);

    SurfaceBevelPlan::try_create(&chain, &regions, size, true).unwrap_or_else(|reason| {
        panic!("the rim has to plan before its room can be measured: {reason:?}")
    })
}
