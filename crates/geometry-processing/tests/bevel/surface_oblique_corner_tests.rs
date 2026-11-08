//! Three chamfers mitring at a corner that is NOT square -
//! `skew_ridge_solid`'s two skew ends, where the three faces stand at 112, 142 and 146 degrees to
//! each other and the point their chamfer planes meet at is nowhere near the middle of the strip it
//! bends.
//!
//! A chamfer of a CONVEX solid is the solid intersected with one plane per bevelled edge, so the
//! answer is the polytope of those planes, which `half_space_solids` builds from the same walls the
//! fixture is made of. `surface_corner_tests.rs` measures the square corner; this is where the
//! corner leans.

use std::sync::Arc;

use solidmesh::bevel::surface::{
    surface_bevel_builder, surface_mesh_closure, SurfaceBevelPlan, SurfaceBuildFailure,
};
use solidmesh::bevel::{bevel_chain_linker, BevelAdvisoryArg, BevelAdvisoryCode};
use solidmesh::topology::{FaceRegionCache, FeatureEdge, FeatureEdgeSet};
use solidmesh_math::Vector3;
use solidmesh_mesh::{CancelToken, Mesh};

use crate::half_space_solids::{self, Wall};
use crate::mesh_measurements::{shell_count, signed_volume};
use crate::skew_ridge_solid;

const SIZE: f64 = 3.0;

/// A ridge long enough that its two mitres have room between them, chamfered along with the four
/// edges running down from its ends: the solid is exactly what the ten planes keep, area for area as
/// well as in volume. The ridge strip is ONE ring of six vertices, the widest the stitch ever hands
/// to the fan; it comes out convex because a chamfer face of a convex solid is a convex polygon, and
/// the area reading is what pins that - a ring fanned into a fold covers part of itself twice.
#[test]
fn an_oblique_three_chamfer_corner_keeps_exactly_what_the_three_planes_keep() {
    let ridge_length = 25.0;

    let built = try_chamfer(ridge_length)
        .unwrap_or_else(|failure| panic!("the build refused the corner: {failure:?}"));

    assert!(
        built.is_manifold(),
        "a chamfer of a closed solid stays a closed solid, however the corner leans"
    );
    let (closed, open_at) = surface_mesh_closure::is_closed(&built);
    assert!(
        closed,
        "nothing may be left open, and something is at {open_at:?}"
    );
    assert_eq!(shell_count(&built), 1);

    let kept = half_space_solids::volume(&walls_with_the_chamfers(ridge_length));
    let volume = signed_volume(&built);

    assert!(
        (volume - kept).abs() <= 1e-3,
        "chamfering a convex solid is intersecting it with one plane per edge, and that is the \
         same solid: {volume} against {kept}"
    );

    let polytope = half_space_solids::build(&walls_with_the_chamfers(ridge_length));
    let skin = surface_of(&built);
    let expected = surface_of(&polytope);

    assert!(
        (skin - expected).abs() <= 1e-3,
        "the same solid has the same skin, and a ring fanned into a fold would carry more of it \
         than that: {skin} against {expected}"
    );
}

/// The same corners at either end of a ridge too SHORT to hold them: the two meeting points cross
/// over inside the strip while the rails themselves still have room. There is no ring to sew
/// there, so it is refused with the place named rather than built inside out.
#[test]
fn two_mitres_crossing_inside_a_short_ridge_are_refused() {
    let ridge_length = 10.5;

    let failure = match try_chamfer(ridge_length) {
        Ok(_) => panic!("the two bent ends of the ridge want the same stretch of it twice over"),
        Err(failure) => failure,
    };
    let reason = failure.advisory().expect("the refusal names its advisory");

    assert_eq!(reason.code(), BevelAdvisoryCode::SurfaceStripsDoNotMitre);
    assert!(
        reason
            .args()
            .iter()
            .any(|arg| matches!(arg, BevelAdvisoryArg::Vector3(_))),
        "a refusal of this kind says where"
    );
}

/// How much skin a mesh has. Two meshes of one shape agree on it; a tessellation that doubles back
/// over itself keeps the volume it had and carries more skin than the shape needs.
fn surface_of(mesh: &Mesh) -> f64 {
    let mut area = 0.0;

    for face in mesh.faces.iter() {
        let first = Vector3::from_vector3_float(mesh.vertices[face.v0 as usize]);

        area += (Vector3::from_vector3_float(mesh.vertices[face.v1 as usize]) - first)
            .cross(Vector3::from_vector3_float(mesh.vertices[face.v2 as usize]) - first)
            .length()
            / 2.0;
    }

    area
}

/// The fixture with its five edges chamfered by the surface construction, as one chain per linked
/// run - which here is one chain per edge, three of them ending at each of the two skew corners.
fn try_chamfer(ridge_length: f64) -> Result<Mesh, SurfaceBuildFailure> {
    let mesh = skew_ridge_solid::create(ridge_length);
    let graph = mesh.get_mesh_edge_graph();
    let features = FeatureEdgeSet::create(&mesh);
    let regions = Arc::new(FaceRegionCache::new(
        Arc::new(mesh.clone()),
        Some(graph.clone()),
        features.flat_angle_degrees(),
    ));

    let picked: Vec<FeatureEdge> = features
        .edges()
        .iter()
        .filter(|edge| skew_ridge_solid::is_selected(edge.start, edge.end, ridge_length))
        .copied()
        .collect();

    assert_eq!(
        picked.len(),
        5,
        "the fixture should offer the ridge and its four wings"
    );

    let mut plans = Vec::new();

    for chain in bevel_chain_linker::link(&picked) {
        plans.push(
            SurfaceBevelPlan::try_create(&chain, &regions, SIZE, false)
                .unwrap_or_else(|reason| panic!("the plan refused the chain: {reason}")),
        );
    }

    surface_bevel_builder::try_build(
        &mesh,
        &graph,
        &regions,
        &plans,
        1,
        &CancelToken::new(),
        crate::surface_test_build::build_options(),
    )
    .map(|built| built.mesh)
}

/// The fixture's own five walls and the five chamfer planes, one per bevelled edge: the walls of
/// the chamfered solid, which is what the surface construction has to build.
fn walls_with_the_chamfers(ridge_length: f64) -> Vec<Wall> {
    let walls = skew_ridge_solid::walls(ridge_length);
    let mut found = walls.clone();

    let bevelled = [
        (0, 1, Vector3::ZERO),
        (0, 2, skew_ridge_solid::near_corner(ridge_length)),
        (1, 2, skew_ridge_solid::near_corner(ridge_length)),
        (0, 3, skew_ridge_solid::far_corner(ridge_length)),
        (1, 3, skew_ridge_solid::far_corner(ridge_length)),
    ];

    for (first, second, on) in bevelled {
        let first = walls[first];
        let second = walls[second];

        found.push(half_space_solids::chamfer(
            on,
            first.facing.cross(second.facing),
            first,
            second,
            SIZE,
        ));
    }

    found
}
