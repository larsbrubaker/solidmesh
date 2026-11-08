//! The WHOLE upper face of `nose_in_notch_test_meshes`'
//! nose, built - a convex ridge cut between two concave creases beaded, which puts a LEANING mixed
//! corner at each end of that ridge and asks the dual canal to close both of them.
//!
//! The same selection `BevelTopCenterFaceRegressionTests` holds against the Onshape control, read
//! here off the part built in code so that the kernel can say the mesh CLOSES without the app being
//! built at all. What the control decides is the shape; what this decides is that the shape is a
//! solid.

use std::rc::Rc;
use std::sync::Arc;

use geometry_processing::bevel::bevel_chain_linker;
use geometry_processing::bevel::surface::{
    surface_bevel_builder, surface_mesh_closure, SurfaceBevelPlan, SurfaceEdgePlan,
    SurfaceMixedCornerDualCanal, SurfaceMixedCornerDualGrid, SurfaceStrip,
};
use geometry_processing::topology::{FaceRegionCache, FeatureEdge, FeatureEdgeSet};
use polygon_mesh::{CancelToken, Mesh};
use vector_math::Vector3;

use crate::mesh_measurements::{shell_count, signed_volume};
use crate::nose_in_notch_mixed_corners;
use crate::nose_in_notch_test_meshes as nose;

const SEGMENTS: i32 = 16;

#[test]
fn the_upper_faces_rim_closes_at_radius_one() {
    the_rim_closes(1.0);
}

/// And at HALF the radius, where every part of the corner is half the size but the part is not -
/// so the canal's turn, the run-out's reach along the crease and the bites all change together and
/// nothing that closed at one size closed by coincidence.
#[test]
fn the_upper_faces_rim_closes_at_half_that_radius() {
    the_rim_closes(0.5);
}

fn the_rim_closes(radius: f64) {
    let built = the_build(radius).unwrap_or_else(|reason| {
        panic!("the rim of the nose's upper face at r {radius} refused: {reason}")
    });

    assert_eq!(
        shell_count(&built),
        1,
        "the cut ridge and the two beads are one solid with the part"
    );

    let (open, open_at) = surface_mesh_closure::open_edge_count(&built);
    assert_eq!(
        open, 0,
        "and nothing of it is left open - instead the first gap is at {open_at:?}"
    );

    assert!(
        built.is_manifold(),
        "and no edge of it carries more than two faces"
    );

    // A fillet on a rim that is convex along the ridge and concave along both creases adds where it
    // beads and takes where it cuts, so the sign is not derivable - what is, is that it stays within
    // a few percent of the part it was cut from. A corner that came back capped, gapped or doubled
    // over itself moves it far further than that.
    let volume = signed_volume(&built);

    assert!(
        (volume - nose::VOLUME).abs() <= nose::VOLUME / 25.0,
        "instead {volume} against the part's {}",
        nose::VOLUME
    );
}

/// Every interior point of both corners' grids is PUBLISHED - the patch owns its own middle, and a
/// grid point the emission dropped would leave a hole the shell count alone might not see.
#[test]
fn every_interior_grid_point_is_published() {
    let built = the_build(1.0).unwrap_or_else(|reason| panic!("the rim refused: {reason}"));

    let vertices: Vec<Vector3> = built
        .vertices
        .iter()
        .map(|&v| Vector3::from_vector3_float(v))
        .collect();

    for ridge_end in [nose::left_ridge_end(), nose::right_ridge_end()] {
        let grid = the_grid_at(ridge_end);

        for station in 1..grid.station_count().saturating_sub(1) {
            for rail in 1..grid.rails_per_row().saturating_sub(1) {
                let want = grid.at(rail, station);
                let nearest = vertices
                    .iter()
                    .map(|v| (*v - want).length())
                    .fold(f64::INFINITY, f64::min);

                assert!(
                    nearest < 1e-4,
                    "rail {rail} of station {station} at {ridge_end:?} stands at {want:?}, and the \
                     nearest published vertex is {nearest} away"
                );
            }
        }
    }
}

fn the_grid_at(ridge_end: Vector3) -> SurfaceMixedCornerDualGrid {
    let corner = nose_in_notch_mixed_corners::at_the_ridge_end(ridge_end, 1.0);
    // From either ridge end the crease left unpicked runs down to the lower crease end (the
    // picked legs go up to the upper one), so the canal's reach is that leg's length at both ends.
    let to = nose::lower_crease_end();

    let canal = SurfaceMixedCornerDualCanal::try_read(&corner, (to - ridge_end).length())
        .unwrap_or_else(|why| panic!("the canal refused: {why:?}"));

    SurfaceMixedCornerDualGrid::try_read_strips(
        Rc::new(canal),
        &strip_of(&corner.cut),
        &strip_of(&corner.bead),
    )
    .unwrap_or_else(|why| panic!("the grid refused: {why:?}"))
}

fn strip_of(edge: &SurfaceEdgePlan) -> SurfaceStrip {
    SurfaceStrip::try_create(edge, SEGMENTS)
        .unwrap_or_else(|why| panic!("the strip refused: {why:?}"))
}

/// The nose's upper face's whole rim, built: the ridge and the two creases that die on the notch's
/// slopes, which is the FaceGroup "Top Center Face.mcx" picks.
fn the_build(radius: f64) -> Result<Mesh, String> {
    let part = nose::create();
    let features = FeatureEdgeSet::create(&part);
    let graph = part.get_mesh_edge_graph();
    let regions = Arc::new(FaceRegionCache::new(
        Arc::new(part.clone()),
        Some(graph.clone()),
        features.flat_angle_degrees(),
    ));

    let rim = vec![
        edge_between(&features, nose::left_ridge_end(), nose::right_ridge_end()),
        edge_between(&features, nose::left_ridge_end(), nose::upper_crease_end()),
        edge_between(&features, nose::right_ridge_end(), nose::upper_crease_end()),
    ];

    let plans: Vec<Rc<SurfaceBevelPlan>> = bevel_chain_linker::link(&rim)
        .iter()
        .map(|chain| {
            SurfaceBevelPlan::try_create(chain, &regions, radius, true)
                .unwrap_or_else(|why| panic!("the plan refused a chain: {why}"))
        })
        .collect();

    surface_bevel_builder::try_build(
        &part,
        &graph,
        &regions,
        &plans,
        SEGMENTS,
        &CancelToken::new(),
        crate::surface_test_build::build_options(),
    )
    .map(|built| built.mesh)
    .map_err(|reason| format!("{reason:?}"))
}

fn edge_between(features: &FeatureEdgeSet, from: Vector3, to: Vector3) -> FeatureEdge {
    let found: Vec<&FeatureEdge> = features
        .edges()
        .iter()
        .filter(|edge| {
            ((edge.start - from).length() < 1e-6 && (edge.end - to).length() < 1e-6)
                || ((edge.start - to).length() < 1e-6 && (edge.end - from).length() < 1e-6)
        })
        .collect();
    assert_eq!(found.len(), 1);

    *found[0]
}
