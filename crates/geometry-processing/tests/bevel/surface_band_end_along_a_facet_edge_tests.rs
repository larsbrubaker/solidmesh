//! Port of `SurfaceBandEndAlongAFacetEdgeTests.cs`: A BAND'S END PLANE LYING ALONG A FACET'S OWN
//! EDGE DOES NOT CUT THAT FACET IN TWO. The flat end of a half disc prism is fanned into triangles
//! from the corner where the diameter meets the arc, and for an ODD number of chords one of those
//! fan edges lies exactly in the plane the diameter's band ends on at that corner - the mitre plane
//! with the arc's first chord, which by the inscribed angle theorem runs from a cyclic polygon's
//! corner through the midpoint of the arc facing it, and at an odd count that midpoint IS a vertex.
//!
//! A plane through an edge decides nothing about which side the edge is on; the floats put it
//! 1.2e-4 on the removed side at nineteen chords, the clip kept a zero-area sliver beyond it, and
//! the edge came out with three faces on it - a fold refused as `SurfaceResultNotClosed`. It fell
//! on 3, 7, 9, 11 and 19 chords, the signature of a rounding deciding a side rather than of a
//! shape. C# `[Arguments]` rows are a loop over the same-named test.

use std::rc::Rc;
use std::sync::Arc;

use geometry_processing::bevel::bevel_chain_linker;
use geometry_processing::bevel::surface::{
    surface_bevel_builder, surface_mesh_closure, SurfaceBevelPlan, SurfaceBuildOptions,
};
use geometry_processing::topology::{FaceRegionCache, FeatureEdge, FeatureEdgeSet};
use polygon_mesh::{CancelToken, Mesh};

use crate::mesh_measurements::{shell_count, signed_volume};
use crate::surface_mitre_consumes_a_chord_tests::half_disc_prism;

const RADIUS: f64 = 10.0;

const HEIGHT: f64 = 20.0;

/// The whole rim of the flat end at nineteen chords, at the two sizes the fold was found at.
#[test]
fn the_whole_rim_of_a_nineteen_chord_half_disc_builds() {
    for size in [0.5, 1.0] {
        let mesh = half_disc_prism(19, RADIUS, HEIGHT);
        let (plan, regions) = plan_of_the_rim(&mesh, 19, size);

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
                "at {size} the cap's own fan edge lying in the band's end plane is not a piece of \
                 the cap: {reason:?}"
            )
        })
        .mesh;

        let (closed, open_at) = surface_mesh_closure::is_closed(&result);
        assert!(
            closed,
            "at {size} nothing may be left open, and something is at {open_at:?}"
        );
        assert!(
            result.is_manifold(),
            "a fillet of a closed solid stays a closed solid"
        );
        assert_eq!(shell_count(&result), 1);

        let whole = signed_volume(&mesh);
        let left = signed_volume(&result);

        assert!(
            left < whole,
            "a convex fillet takes material off, and this took none"
        );
        assert!(
            left > whole * 0.95,
            "and it takes a bead off one rim, not a twentieth of the solid: {whole} became {left}"
        );
    }
}

/// And every size up to the one the chord itself runs out at: the fold was never about the size
/// fitting, so no size under that may be refused for it, nor built into something open.
///
/// A TENTH of a millimetre is here since the crossing was made once. At that size the diameter's
/// own tangent line crosses the fan edge within a rounding of the mitre point the two bands' lines
/// meet at, and the crossing used to be constructed twice a weld apart - now that a split on a mesh
/// edge is a function of the edge (`surface_one_crossing_per_facet_edge_tests`).
#[test]
fn no_size_up_to_the_one_the_chord_runs_out_at_is_refused_for_it() {
    let mut wrong = Vec::new();

    for size in [0.1, 0.25, 0.5, 0.75, 1.0, 1.25, 1.41] {
        let mesh = half_disc_prism(19, RADIUS, HEIGHT);
        let (plan, regions) = plan_of_the_rim(&mesh, 19, size);

        let result = match surface_bevel_builder::try_build(
            &mesh,
            &mesh.get_mesh_edge_graph(),
            &regions,
            &[plan],
            8,
            &CancelToken::new(),
            SurfaceBuildOptions::default(),
        ) {
            Ok(built) => built.mesh,
            Err(reason) => {
                wrong.push(format!("{size}: {reason:?}"));
                continue;
            }
        };

        let (closed, open_at) = surface_mesh_closure::is_closed(&result);

        if !closed {
            wrong.push(format!("{size}: open at {open_at:?}"));
        } else if !result.is_manifold() {
            wrong.push(format!("{size}: not manifold"));
        } else if shell_count(&result) != 1 {
            wrong.push(format!("{size}: {} shells", shell_count(&result)));
        }
    }

    assert!(
        wrong.is_empty(),
        "the fold is a rounding on a plane through an edge, and it is there at every size - {}",
        wrong.join("; ")
    );
}

/// And another odd chord count, where the same fan edge stands in the same plane at another angle.
///
/// Three, nine and eleven chords carry a SECOND consequence of the same coincidence: it puts the
/// two bands' tangent lines across that fan edge at ONE point, the mitre point at the corner, and
/// each band used to construct that crossing from its own plane - 4.4e-5 apart where the part's
/// weld is 2.96e-5 - so the shell came out open along the tangent line. A duplicated crossing
/// rather than a fold, kept out by `surface_one_crossing_per_facet_edge_tests`.
#[test]
fn an_odd_chord_count_puts_a_fan_edge_in_the_bands_end_plane() {
    for facets in [3, 7, 9, 11] {
        let mesh = half_disc_prism(facets, RADIUS, HEIGHT);
        let (plan, regions) = plan_of_the_rim(&mesh, facets, 1.0);

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
                "{facets} chords put the bisector at the corner through a vertex, which is not a \
                 cut: {reason:?}"
            )
        })
        .mesh;

        let (closed, open_at) = surface_mesh_closure::is_closed(&result);
        assert!(
            closed,
            "at {facets} chords nothing may be left open, and something is at {open_at:?}"
        );
        assert!(result.is_manifold(), "{facets} chords");
        assert_eq!(shell_count(&result), 1, "{facets} chords");
    }
}

/// The whole rim of the flat end, planned as the one closed chain it is. Shared with
/// `surface_one_crossing_per_facet_edge_tests` (the same C# helper in both classes).
pub fn plan_of_the_rim(
    mesh: &Mesh,
    facets: i32,
    size: f64,
) -> (Rc<SurfaceBevelPlan>, Arc<FaceRegionCache>) {
    let features = FeatureEdgeSet::create(mesh);
    let top = mesh
        .vertices
        .iter()
        .map(|vertex| vertex.z as f64)
        .fold(f64::NEG_INFINITY, f64::max);

    let rim: Vec<FeatureEdge> = features
        .edges()
        .iter()
        .filter(|edge| (edge.start.z - top).abs() < 1e-4 && (edge.end.z - top).abs() < 1e-4)
        .cloned()
        .collect();

    assert_eq!(
        rim.len() as i32,
        facets + 1,
        "the flat end's rim is the diameter and {facets} chords, not {} edges",
        rim.len()
    );

    let regions = Arc::new(FaceRegionCache::new(
        Arc::new(mesh.clone()),
        Some(mesh.get_mesh_edge_graph()),
        features.flat_angle_degrees(),
    ));

    let chains = bevel_chain_linker::link(&rim);
    assert_eq!(chains.len(), 1, "the rim is one chain");

    let plan =
        SurfaceBevelPlan::try_create(&chains[0], &regions, size, true).unwrap_or_else(|reason| {
            panic!(
                "the rim has to plan before it can be built: {:?}",
                reason.code()
            )
        });

    (plan, regions)
}
