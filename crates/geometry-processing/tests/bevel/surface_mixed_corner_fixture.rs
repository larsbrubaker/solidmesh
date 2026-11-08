//! The L prism's mixed corner as every test of it asks for
//! it - the two picked edges as one chain, the plan they make, the build, and the two numbers the
//! corner's own closed form is written in.
//!
//! Shared because several classes read this one corner from several sides - the blend
//! (`surface_mixed_corner_tests`), the runout (`surface_mixed_corner_run_out_tests`) and the
//! sections and sectors built on it - and a fixture that each of them built again would be a fixture
//! per class to keep in step.

use std::rc::Rc;
use std::sync::Arc;

use solidmesh::bevel::bevel_chain_linker::{self, BevelSweptChain};
use solidmesh::bevel::surface::{
    surface_bevel_builder, surface_mixed_corner_patch, SurfaceBevelPlan, SurfaceMixedCorner,
    SurfaceStrip,
};
use solidmesh::topology::{EdgeConvexity, FaceRegionCache, FeatureEdgeSet};
use solidmesh_math::Vector3;
use solidmesh_mesh::{CancelToken, Mesh};

use crate::surface_test_chains::join_and_open;
use crate::topology_test_meshes::create_l_shaped_grid_prism;

/// The radius the corner is derived at, and the one every closed form here is written for.
pub const RADIUS: f64 = 1.0;

/// The corner the two picked edges meet at - the top of the L's reflex edge.
pub fn top_corner() -> Vector3 {
    Vector3::new(10.0, 10.0, 10.0)
}

/// The plan for the two picked edges as one chain, on regions already read.
pub fn the_plan_on(
    regions: &Arc<FaceRegionCache>,
    features: &FeatureEdgeSet,
) -> Rc<SurfaceBevelPlan> {
    SurfaceBevelPlan::try_create(&the_joined_chain(features), regions, RADIUS, true)
        .unwrap_or_else(|reason| panic!("the plan refused the chain: {reason}"))
}

/// The same, on a fresh L prism: the plan, the mesh and its regions.
pub fn the_plan() -> (Rc<SurfaceBevelPlan>, Mesh, Arc<FaceRegionCache>) {
    let mesh = create_l_shaped_grid_prism();
    let (regions, features) = regions_of(&mesh);
    let plan = the_plan_on(&regions, &features);

    (plan, mesh, regions)
}

/// The corner as the plan recorded it.
pub fn the_corner(plan: &SurfaceBevelPlan) -> Rc<SurfaceMixedCorner> {
    plan.try_mixed_corner_after(0)
        .expect("the plan did not record the corner")
}

/// The L prism built at this corner, and the plan it was built from.
pub fn the_build(segments: i32) -> (Mesh, Rc<SurfaceBevelPlan>) {
    let (plan, mesh, regions) = the_plan();

    let built = surface_bevel_builder::try_build_one(
        &mesh,
        &mesh.get_mesh_edge_graph(),
        &regions,
        plan.clone(),
        segments,
        &CancelToken::new(),
    )
    .unwrap_or_else(|reason| {
        panic!("the build refused the corner at {segments} segments: {reason:?}")
    });

    (built, plan)
}

/// One planned edge's strip, at the sampling the build uses.
pub fn strip_of(plan: &SurfaceBevelPlan, edge_index: usize, segments: i32) -> SurfaceStrip {
    SurfaceStrip::try_create(&plan.edges[edge_index], segments)
        .unwrap_or_else(|reason| panic!("the strip refused the edge: {reason}"))
}

/// The two picked edges as the one chain they are, end to end at the corner.
pub fn the_joined_chain(features: &FeatureEdgeSet) -> BevelSweptChain {
    let mut chains = join_and_open(&[
        chain_of(features, the_convex_edge(features)),
        chain_of(features, the_concave_edge(features)),
    ]);

    assert_eq!(
        chains.len(),
        1,
        "the two picked edges are end to end, so they are one chain"
    );

    chains.remove(0)
}

/// The one reflex edge of the L - the bead's.
pub fn the_concave_edge(features: &FeatureEdgeSet) -> usize {
    single(
        (0..features.count()).filter(|&i| features.edges()[i].convexity == EdgeConvexity::Concave),
    )
}

/// The top rim of the step wall, from the corner out to (10, 20, 10) - the cut's.
pub fn the_convex_edge(features: &FeatureEdgeSet) -> usize {
    the_convex_edge_to(features, Vector3::new(10.0, 20.0, 10.0))
}

/// The top rim of the VOID wall, from the corner out to (20, 10, 10) - the third edge at the
/// corner, which the two-edge corner leaves unpicked and the JUNCTION picks as its second cut.
pub fn the_second_convex_edge(features: &FeatureEdgeSet) -> usize {
    the_convex_edge_to(features, Vector3::new(20.0, 10.0, 10.0))
}

/// The one feature edge running from [`top_corner`] out to `far`.
fn the_convex_edge_to(features: &FeatureEdgeSet, far: Vector3) -> usize {
    let top = top_corner();

    single((0..features.count()).filter(|&i| {
        let edge = &features.edges()[i];

        ((edge.start - top).length() < 1e-6 && (edge.end - far).length() < 1e-6)
            || ((edge.end - top).length() < 1e-6 && (edge.start - far).length() < 1e-6)
    }))
}

fn single(mut found: impl Iterator<Item = usize>) -> usize {
    let first = found.next().expect("exactly one edge");
    assert!(found.next().is_none(), "exactly one edge");

    first
}

pub fn chain_of(features: &FeatureEdgeSet, edge_index: usize) -> BevelSweptChain {
    let mut chains = bevel_chain_linker::link(&[features.edges()[edge_index]]);
    assert_eq!(chains.len(), 1);

    chains.remove(0)
}

pub fn regions_of(mesh: &Mesh) -> (Arc<FaceRegionCache>, FeatureEdgeSet) {
    let features = FeatureEdgeSet::create(mesh);
    let regions = Arc::new(FaceRegionCache::new(
        Arc::new(mesh.clone()),
        Some(mesh.get_mesh_edge_graph()),
        features.flat_angle_degrees(),
    ));

    (regions, features)
}

/// The corner's two numbers, read off the ring the build itself cut: the area of the cut's end
/// section - the ring closed back to the corner the two faces would have met at - and what that
/// section sweeps as it is carried round the bead's axis in the patch's own `n` ruled steps,
/// `n sin(dt) M` with `M` its first moment about that axis.
///
/// The two are the whole of the corner up to the last turn. Over the corner's own height the void
/// square the bead used to fill straight gains that section once - which is the `+ S` - while the
/// blend takes the same section round through the turn, which is the sweep. `n sin(dt) M` and not
/// `t_W M`, because the patch is a grid of PLANAR trapezoids rather than a torus: the ruled sweep's
/// Pappus. At eight segments it is 0.42047 against the ideal 0.41233.
pub fn the_corners_own_section(plan: &SurfaceBevelPlan, segments: i32) -> (f64, f64) {
    let cut_first = plan.edges[0].convexity() == EdgeConvexity::Convex;

    the_corners_own_section_of(
        &the_corner(plan),
        &strip_of(plan, if cut_first { 0 } else { 1 }, segments),
    )
}

/// The same two numbers, read off a corner and the cut's strip directly rather than off a joined
/// chain's plan - which is how the JUNCTION asks for them, where the cut and the bead are planned on
/// chains of their own and no one plan holds both.
pub fn the_corners_own_section_of(
    corner: &SurfaceMixedCorner,
    cut_strip: &SurfaceStrip,
) -> (f64, f64) {
    let (_, ring) = surface_mixed_corner_patch::try_cut_the_ring(corner, cut_strip, true, None)
        .unwrap_or_else(|reason| panic!("the cut's ring would not cut: {reason}"));

    // The section in the meridian half plane: the ring, and the corner the cap and the shared face
    // would have met at, which is where the two tangent lines' own planes cross.
    let mut section = ring.clone();
    section.push(Vector3::new(10.0, 10.0 + RADIUS, 10.0));

    // In that half plane, measured from the axis: out of it, and along it. Shoelace for both the
    // area and the first moment, which is the same walk.
    let axis = corner.axis();
    let mut area = 0.0;
    let mut moment = 0.0;

    for i in 0..section.len() {
        let here = section[i];
        let there = section[(i + 1) % section.len()];

        let first_out = axis.distance_to(here);
        let first_along = (here - axis.point).dot(axis.direction);
        let next_out = axis.distance_to(there);
        let next_along = (there - axis.point).dot(axis.direction);

        let crossed = (first_out * next_along) - (next_out * first_along);

        area += crossed;
        moment += (first_out + next_out) * crossed;
    }

    let steps = (ring.len() - 1) as f64;

    (
        area.abs() / 2.0,
        steps * (corner.turn_radians() / steps).sin() * moment.abs() / 6.0,
    )
}

/// What the L is left with at the JUNCTION - both top rims picked as well as the crease - as a
/// closed form in the cut's own chorded section, at the patch's own sampling.
///
/// The 20 square less the 10 square bite, 10 tall, is 3000 before anything is filleted. Four things
/// change it and every one of them is the SAME section `a`, because all three edges are right
/// angles at one radius: `-8a` in all, and the blend takes the section round the turn once more,
/// which is the ruled sweep's Pappus. In the smooth limit the volume is 2997.87079; chorded it is
/// under that, and climbs to it as the sampling doubles.
pub fn the_junctions_volume(corner: &SurfaceMixedCorner, cut_strip: &SurfaceStrip) -> f64 {
    let (section, swept) = the_corners_own_section_of(corner, cut_strip);

    3000.0 - (8.0 * section) - swept
}

/// What the junction leaves of the L's CAP: 300 less the two 1 by 9 bands the two cuts take, less
/// the blend's own bite out of the 2 by 2 corner - the chorded quarter disc of radius `2r` about the
/// bead's axis, `2n sin(pi / 2n)`, less the unit square of it that was never cap at all because the
/// void bite had it already.
pub fn the_junctions_cap_area(segments: i32) -> f64 {
    let segments = f64::from(segments);

    300.0 - 18.0 - ((2.0 * segments * (std::f64::consts::PI / (2.0 * segments)).sin()) - 1.0)
}
