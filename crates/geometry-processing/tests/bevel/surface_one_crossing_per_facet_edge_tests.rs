//! Port of `SurfaceOneCrossingPerFacetEdgeTests.cs`: TWO CUTS CROSSING ONE MESH EDGE AT ONE PLACE
//! MAKE ONE VERTEX. Where a closed chain turns, the two bands either side of the corner are cut off
//! on tangent planes that meet at the corner's mitre point, and where that point stands on a
//! facet's own edge - the fan edge of a half disc's cap, at an odd chord count - each band used to
//! compute ITS crossing of that edge from its own plane.
//!
//! On the three chord half disc the two land (8.2679367, 0.9999780) and (8.2678986, 1.0000000),
//! 4.4e-5 apart where the part's weld is 2.96e-5, and the shell came out OPEN along the tangent
//! line. A split on a mesh edge is a function of the EDGE and not of which plane asked for it
//! (`surface_walkback_rewrite::try_find_crossings`). C# `[Arguments]` rows are a loop.

use geometry_processing::bevel::surface::{
    surface_bevel_builder, surface_mesh_closure, SurfaceBuildOptions,
};
use polygon_mesh::CancelToken;
use vector_math::Vector3;

use crate::mesh_measurements::shell_count;
use crate::surface_band_end_along_a_facet_edge_tests::plan_of_the_rim;
use crate::surface_mitre_consumes_a_chord_tests::half_disc_prism;

const RADIUS: f64 = 10.0;

const HEIGHT: f64 = 20.0;

/// The odd chord counts that put a fan edge through the mitre point, at the two sizes the
/// duplicate was measured at.
#[test]
fn the_mitre_point_on_a_fan_edge_is_one_vertex() {
    for (facets, size) in [(3, 1.0), (9, 1.0), (11, 1.0), (19, 0.1)] {
        let mesh = half_disc_prism(facets, RADIUS, HEIGHT);
        let (plan, regions) = plan_of_the_rim(&mesh, facets, size);

        let result = surface_bevel_builder::try_build(
            &mesh,
            &mesh.get_mesh_edge_graph(),
            &regions,
            &[plan],
            8,
            &CancelToken::new(),
            SurfaceBuildOptions::default(),
        )
        .unwrap_or_else(|reason| {
            panic!(
                "{facets} chords at {size}: the two bands' feet meet at one point on the fan edge: \
                 {reason:?}"
            )
        })
        .mesh;

        let (closed, open_at) = surface_mesh_closure::is_closed(&result);
        assert!(
            closed,
            "{facets} chords at {size}: a crossing made twice leaves the shell open, and it is at \
             {open_at:?}"
        );
        assert!(
            result.is_manifold(),
            "{facets} chords at {size}: a fillet of a closed solid stays a closed solid"
        );
        assert_eq!(shell_count(&result), 1, "{facets} chords at {size}");
    }
}

/// And the duplicate itself, at the place it was measured: ONE vertex stands where the diameter's
/// tangent line and the arc's cross the cap's fan edge.
#[test]
fn the_two_bands_cross_the_fan_edge_at_one_vertex() {
    let mesh = half_disc_prism(3, RADIUS, HEIGHT);
    let (plan, regions) = plan_of_the_rim(&mesh, 3, 1.0);

    let result = surface_bevel_builder::try_build(
        &mesh,
        &mesh.get_mesh_edge_graph(),
        &regions,
        &[plan],
        8,
        &CancelToken::new(),
        SurfaceBuildOptions::default(),
    )
    .unwrap_or_else(|reason| panic!("{reason:?}"))
    .mesh;

    let crossing = Vector3::new(8.26792, 1.0, HEIGHT);

    let there: Vec<Vector3> = result
        .vertices
        .iter()
        .map(|&vertex| Vector3::from_vector3_float(vertex))
        .filter(|&vertex| (vertex - crossing).length() < 1e-3)
        .collect();

    assert_eq!(
        there.len(),
        1,
        "the mitre point on the fan edge is one place, and one place is one vertex - {there:?}"
    );
}
