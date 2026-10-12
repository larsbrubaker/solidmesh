//! Port of `SurfaceMitreOverAShortChordTests.cs`: the whole rim of a flat top whose straight edge
//! runs into a chorded arc. The corner where they meet is REFLEX in the top face's own plan - the
//! arc bulges out past the straight edge's line - and the chord it meets is a part chord, shorter
//! than the fillet itself.
//!
//! The kernel reading of "Every Chamfer Works.mcx"'s two merged cap faces, built directly by
//! `lobed_prism_meshes` to the same numbers with no boolean; the part chord the merge leaves at the
//! seam is 1.1211 long there and here. At a reflex corner a mitre carries the rails on the outside
//! of the turn PAST it, by the rail's own offset over the cosine of half the turn, whatever the
//! edge's length; whether two mitres leave a window to sew through is the stitch's own reading.

use std::rc::Rc;
use std::sync::Arc;

use geometry_processing::bevel::bevel_chain_linker;
use geometry_processing::bevel::surface::{
    surface_bevel_builder, surface_mesh_closure, SurfaceBevelPlan,
};
use geometry_processing::topology::{FaceRegionCache, FeatureEdge, FeatureEdgeSet};
use polygon_mesh::{CancelToken, Mesh};
use vector_math::Vector3;

use crate::lobed_prism_meshes;
use crate::mesh_measurements::{shell_count, signed_volume};

/// The bend threshold the fixture is read at, which the arc's 9 degree facet joins are under.
const THRESHOLD_DEGREES: f64 = lobed_prism_meshes::THRESHOLD_DEGREES;

const HEIGHT: f64 = lobed_prism_meshes::HEIGHT;

/// The rim builds, and it closes: one shell, manifold, with a bead of material off the rim.
///
/// The volume is read as bounds rather than as a closed form - what a fillet takes off a
/// tessellated arc is not the ideal torus's answer, and every one of the rim's 23 corners has its
/// own mitre. What it may not be is unchanged, which is what a rim nobody cut would say.
#[test]
fn a_reflex_corner_mitres_over_a_chord_shorter_than_the_fillet() {
    let size = 3.0;

    let prism = lobed_prism_meshes::create();
    let (plan, regions) = top_rim_plan_of(&prism, size);

    assert!(
        plan.is_closed,
        "the rim of a face is a closed run, so every one of its corners is a mitre and none of \
         them an end"
    );

    let part_chord = plan
        .edges
        .iter()
        .map(|edge| (edge.to() - edge.from()).length())
        .fold(f64::INFINITY, f64::min);

    assert!(
        (part_chord - 1.1211).abs() <= 1e-3,
        "the fixture is the merged outline's own part chord at the seam, and it measured \
         {part_chord}"
    );
    assert!(
        part_chord < size,
        "and it is only worth anything while that chord is shorter than the fillet on it"
    );

    let result = surface_bevel_builder::try_build_one(
        &prism,
        &prism.get_mesh_edge_graph(),
        &regions,
        plan,
        8,
        &CancelToken::new(),
    )
    .unwrap_or_else(|reason| {
        panic!(
            "a mitre carries the rails on the outside of the turn past the corner, however short \
             the edge: {reason:?}"
        )
    });

    let (closed, open_at) = surface_mesh_closure::is_closed(&result);
    assert!(
        closed,
        "nothing may be left open, and something is at {open_at:?}"
    );
    assert!(
        result.is_manifold(),
        "a fillet of a closed solid stays a closed solid"
    );
    assert_eq!(shell_count(&result), 1);

    let whole = signed_volume(&prism);
    let left = signed_volume(&result);

    assert!(
        left < whole,
        "a convex fillet takes material off, and this took none"
    );
    assert!(
        left > whole * 0.95,
        "and it takes a 3 mm bead off one rim, not a twentieth of the solid: {whole} became {left}"
    );
}

/// The plan for the whole rim of the top face, flooded at the fixture's own 10 degree threshold.
fn top_rim_plan_of(mesh: &Mesh, size: f64) -> (Rc<SurfaceBevelPlan>, Arc<FaceRegionCache>) {
    let features = FeatureEdgeSet::create_with_angle(mesh, THRESHOLD_DEGREES);

    let regions = Arc::new(FaceRegionCache::new(
        Arc::new(mesh.clone()),
        Some(mesh.get_mesh_edge_graph()),
        THRESHOLD_DEGREES,
    ));

    let rim: Vec<FeatureEdge> = features
        .edges()
        .iter()
        .filter(|edge| at_the_top(edge.start) && at_the_top(edge.end))
        .copied()
        .collect();

    let chains = bevel_chain_linker::link(&rim);
    assert_eq!(chains.len(), 1, "the top rim is one chain");

    let plan = SurfaceBevelPlan::try_create(&chains[0], &regions, size, true)
        .unwrap_or_else(|reason| panic!("the rim has to plan: {:?}", reason.code()));

    (plan, regions)
}

/// Whether a point stands on the top face.
fn at_the_top(point: Vector3) -> bool {
    (point.z - HEIGHT).abs() < 1e-3
}
