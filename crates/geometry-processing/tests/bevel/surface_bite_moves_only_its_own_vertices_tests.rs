//! A snap taken after a piece is cut may only
//! move vertices the pass that cut it MINTED - an older vertex is a vertex some face outside this
//! pass still uses, and moving it off that face's corner opens the seam between them
//! (`surface_end_notch::snapped_to_the_profile`).
//!
//! The lobe wall's rim at `r = 2` is where the part says so. One barrel chord's extruded quad has a
//! diagonal, and three lines cut it, which leaves two construction points near `z = 2` a hair under
//! the pool's snap distance apart. The mixed corner's bite re-clipped only the two faces on its own
//! side of the diagonal and snapped the crossing onto the band corner in both, while the third face
//! went on using the crossing: six open edges over the rim, and the group's last
//! `SurfaceDidNotClose`.

use std::sync::Arc;

use geometry_processing::bevel::bevel_chain_linker;
use geometry_processing::bevel::surface::{
    surface_bevel_builder, surface_end_notch, surface_mesh_closure, SurfaceBevelPlan,
    SurfaceVertexPool,
};
use geometry_processing::topology::FaceRegionCache;
use polygon_mesh::csg::boolean_processing::{classify_boolean_operand, BooleanOperandVerdict};
use polygon_mesh::{CancelToken, Mesh};
use vector_math::{Matrix4X4, Vector3, Vector3Float};

use crate::lobed_prism_meshes;
use crate::mesh_measurements::shell_count;

/// The size of the lobe wall's rim whose bite reached back for a walkback vertex: the whole rim
/// builds closed, manifold, one shell and an operand the kernel takes.
#[test]
fn the_lobe_walls_rim_builds_where_the_bite_met_the_walkbacks_own_vertex() {
    let radius = 2.0;
    let prism = lobed_prism_meshes::create();
    let graph = prism.get_mesh_edge_graph();
    let regions = Arc::new(FaceRegionCache::new(
        Arc::new(prism.clone()),
        Some(graph.clone()),
        lobed_prism_meshes::THRESHOLD_DEGREES,
    ));

    let region = regions
        .region(a_facet_of_the_lobes_wall(&prism))
        .expect("every facet has a region");
    let chains = bevel_chain_linker::link(region.boundary_edges());
    assert_eq!(chains.len(), 1, "the wall's rim is one chain");

    let plan = SurfaceBevelPlan::try_create(&chains[0], &regions, radius, true)
        .unwrap_or_else(|why| panic!("the wall's rim has to plan at {radius}: {:?}", why.code()));

    let built = surface_bevel_builder::try_build_one(
        &prism,
        &graph,
        &regions,
        plan,
        8,
        &CancelToken::new(),
    )
    .unwrap_or_else(|reason| panic!("the wall's rim has to build at {radius}: {reason:?}"));

    let (open, open_at) = surface_mesh_closure::open_edge_count(&built);
    assert_eq!(open, 0, "open at {open_at:?}");
    assert!(built.is_manifold());
    assert_eq!(shell_count(&built), 1);

    assert_eq!(
        classify_boolean_operand(&built, Matrix4X4::IDENTITY, false),
        BooleanOperandVerdict::Clean,
        "a Combine after a Fillet hands this very mesh to the kernel"
    );
}

/// And the snap itself says so: of two vertices of a piece standing the same hair from one rim
/// vertex, the one this pass minted lands on it and the one that was already in the pool stays
/// where it is.
#[test]
fn the_snap_moves_the_minted_vertex_and_leaves_the_older_one() {
    // A part a hundred millimetres across, so the pool's own ruler is far above a float's ulp and
    // the offsets below are real distances rather than rounding.
    let mut pool = SurfaceVertexPool::new(&[
        Vector3Float::new(0.0, 0.0, 0.0),
        Vector3Float::new(100.0, 0.0, 0.0),
        Vector3Float::new(0.0, 100.0, 0.0),
    ]);

    let rim = pool.add(Vector3::new(50.0, 50.0, 0.0));
    let a_hair = pool.snap_near() / 4.0;

    let older = pool.add(Vector3::new(50.0 + a_hair, 50.0, 0.0));
    let elsewhere = pool.add(Vector3::new(20.0, 20.0, 0.0));

    let minted_from = pool.vertices().len() as i32;
    let minted = pool.add(Vector3::new(50.0, 50.0 + a_hair, 0.0));

    assert!(
        a_hair > pool.weld_distance(),
        "the two neighbours have to be two vertices, or the pool would have welded them"
    );
    assert_ne!(minted, older, "and the pool did not weld them");

    let snapped = surface_end_notch::snapped_to_the_profile(
        &[older, minted, elsewhere],
        &[rim],
        &pool,
        minted_from,
    );

    assert_eq!(
        snapped,
        vec![older, rim, elsewhere],
        "only the vertex minted from {minted_from} on may move - instead {snapped:?}"
    );
}

/// Any one facet of the lobe's own wall - a side face standing well outside the flat side.
fn a_facet_of_the_lobes_wall(prism: &Mesh) -> i32 {
    for (face, facet) in prism.faces.iter().enumerate() {
        let middle = (Vector3::from_vector3_float(prism.vertices[facet.v0 as usize])
            + Vector3::from_vector3_float(prism.vertices[facet.v1 as usize])
            + Vector3::from_vector3_float(prism.vertices[facet.v2 as usize]))
            / 3.0;

        if middle.y < -lobed_prism_meshes::RADIUS / 2.0
            && middle.z > 1.0
            && middle.z < lobed_prism_meshes::HEIGHT - 1.0
        {
            return face as i32;
        }
    }

    panic!("the prism has lost the lobe it pokes through its side");
}
