//! A FOOTPRINT ON THE FAR SIDE OF A CURVED WALL IS
//! NOT IN FRONT OF A BAND. The legacy room (`surface_room::largest_fitting`) once read across the
//! tapered barrel and refused the wall's own 0.5 fillet, quoting 0.4098, for a pair of cuts a
//! quarter turn apart: a BAND off one bore mouth on the +x side paired against another plan's
//! MITRE TIP on the -x side, both on region key 0 (the wall is one region at the operation's 10
//! degree bend threshold, though it is 48 facets of 7.5 degrees each), 25.94 apart.
//!
//! The corner rule read the gap as the corner's offset along the BAND's inward direction, which is
//! exact on a flat face; here that projection threw away 25.9 of the separation. What tells the two
//! apart is the component OUT of the band's plane: zero on a flat region, a sagitta for a near
//! neighbour on a gently curved one, and the whole 25.9 here.

use std::rc::Rc;
use std::sync::Arc;

use solidmesh::bevel::surface::{
    surface_room, surface_room_measure, surface_shared_edge_network, SurfaceBevelPlan,
    SurfaceRoomCut,
};
use solidmesh::topology::FaceRegionCache;
use solidmesh_mesh::CancelToken;

use crate::surface_test_chains::join_and_open;
use crate::tapered_barrel_test_meshes;

/// The reading itself: the legacy room fits the barrel wall's own 0.5, and nothing at all answers -
/// no cut's own room and no pair.
#[test]
fn the_far_side_mitre_tip_is_not_a_collision() {
    let part = Arc::new(tapered_barrel_test_meshes::the_working_part());
    let resolution = tapered_barrel_test_meshes::the_wall(&part);
    let graph = part.get_mesh_edge_graph();
    let regions = Arc::new(FaceRegionCache::new(
        part.clone(),
        Some(graph.clone()),
        resolution.features.flat_angle_degrees(),
    ));
    let curve_regions = Arc::new(FaceRegionCache::new(
        part.clone(),
        Some(graph.clone()),
        resolution.group.curve_tolerance_degrees,
    ));

    let plans: Vec<Rc<SurfaceBevelPlan>> = join_and_open(
        &surface_shared_edge_network::link_together(std::slice::from_ref(&resolution)),
    )
    .iter()
    .map(|chain| {
        SurfaceBevelPlan::try_create_with_curve_regions(
            chain,
            &regions,
            Some(&curve_regions),
            tapered_barrel_test_meshes::SIZE,
            true,
        )
        .unwrap_or_else(|why| panic!("{:?}", why.code()))
    })
    .collect();

    let cuts = surface_room_measure::measure(&plans, Some(&mut Vec::new()), None);
    let mut blame: Vec<SurfaceRoomCut> = Vec::new();

    let (largest, worst_at, _) = surface_room::largest_fitting(
        &cuts,
        tapered_barrel_test_meshes::SIZE,
        &CancelToken::new(),
        tapered_barrel_test_meshes::SIZE,
        Some(&mut Vec::new()),
        Some(&mut blame),
    )
    .expect("an uncancelled reading finishes");

    assert!(
        (largest - tapered_barrel_test_meshes::SIZE).abs() <= 1e-9,
        "the wall's own size fits: {worst_at:?} answered otherwise ({largest})"
    );

    assert_eq!(
        blame.len(),
        0,
        "nothing answers at all - the 0.4098 this used to read was the far-side mitre tip"
    );

    // The pair that used to answer, still on one region key and still a quarter turn apart: what
    // changed is only that the rule no longer reads one as standing in front of the other.
    let on_the_wall = cuts.iter().filter(|cut| cut.region_key == 0).count();

    assert!(
        on_the_wall > 300,
        "the whole tessellated wall is ONE region at the 10 degree bend threshold, which is what \
         let two cuts 25.94 apart be read as sharing a gap ({on_the_wall})"
    );
}
