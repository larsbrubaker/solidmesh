//! Two chains picked at once whose bands share a face. A corner of one chain bites that face, and
//! the bite's clip lands NEW vertices on the OTHER chain's tangent line. The sewing knits those
//! vertices into the face that keeps them, so the strip across that line has to sew to them too -
//! and the strip's seam was read from the walkback, before the bite happened.
//!
//! The fixture is the lobed prism's barrel wall and its `x = 0` end wall, both picked as faces:
//! "Every Chamfer Works.mcx"'s own cube-and-cylinder, and its own two picks. At the fixture's own
//! 3.756 the two bands also OVERLAP where the chains meet at the prism's corner, which leaves a
//! hole of its own; 2.5 is the largest of the sizes tried that is clear of that second cause and
//! still bites the shared face, so it is what this test measures.

use std::rc::Rc;
use std::sync::Arc;

use geometry_processing::bevel::bevel_chain_linker;
use geometry_processing::bevel::surface::{
    surface_bevel_builder, surface_mesh_closure, SurfaceBevelPlan, SurfaceBuildOptions,
};
use geometry_processing::topology::FaceRegionCache;
use polygon_mesh::csg::{boolean_processing, BooleanOperandVerdict};
use polygon_mesh::{CancelToken, Mesh};
use vector_math::{Matrix4X4, Vector3};

use crate::lobed_prism_meshes;
use crate::mesh_measurements::shell_count;
use crate::surface_run_out_crosses_a_band_tests::lobed_regions;

/// Big enough for the corner's bite to reach the other chain's tangent line, small enough that the
/// two bands do not overlap where the chains meet.
const RADIUS: f64 = 2.5;

const SEGMENTS: i32 = 8;

#[test]
fn the_barrel_and_the_side_rim_together_close_over_their_shared_face() {
    let built = the_barrel_and_the_side_rim();

    let (open, open_at) = surface_mesh_closure::open_edge_count(&built);
    assert_eq!(open, 0, "open at {open_at:?}");
    assert!(built.is_manifold());
    assert_eq!(shell_count(&built), 1);

    assert_eq!(
        t_junctions(&built),
        0,
        "every vertex on a seam has a station of the strip at it, so none stands part way along \
         another face's edge"
    );

    assert_eq!(
        boolean_processing::classify_boolean_operand(&built, Matrix4X4::IDENTITY, false),
        BooleanOperandVerdict::Clean,
        "a Combine after a Fillet hands this very mesh to the kernel"
    );
}

/// How many (vertex, face edge) pairs have the vertex standing strictly inside the edge - the
/// signature of a seam stitched as the walkback left it rather than as the mesh now stands.
// The negated comparison `!(x > 0)` skips a NaN as well as zero.
#[allow(clippy::neg_cmp_op_on_partial_ord)]
fn t_junctions(mesh: &Mesh) -> i32 {
    let mut found = 0;

    for face in mesh.faces.iter() {
        let corners = [face.v0, face.v1, face.v2];

        for corner in 0..3 {
            let from = Vector3::from_vector3_float(mesh.vertices[corners[corner] as usize]);
            let run =
                Vector3::from_vector3_float(mesh.vertices[corners[(corner + 1) % 3] as usize])
                    - from;

            // Skips a zero-length edge, and a NaN as well.
            if !(run.length_squared() > 0.0) {
                continue;
            }

            for vertex in 0..mesh.vertices.len() as i32 {
                if corners.contains(&vertex) {
                    continue;
                }

                let at = Vector3::from_vector3_float(mesh.vertices[vertex as usize]) - from;
                let place = at.dot(run) / run.length_squared();

                if place > 0.0 && place < 1.0 && (at - (run * place)).length() < 1e-4 {
                    found += 1;
                }
            }
        }
    }

    found
}

/// The barrel wall and the prism's `x = 0` end wall, both picked as faces and built.
fn the_barrel_and_the_side_rim() -> Mesh {
    let prism = lobed_prism_meshes::create();
    let graph = prism.get_mesh_edge_graph();
    let regions = lobed_regions(&prism);

    let plans = vec![
        rim_plan_of(&prism, &regions, |middle| {
            middle.y < -lobed_prism_meshes::RADIUS / 2.0
                && middle.z > 1.0
                && middle.z < lobed_prism_meshes::HEIGHT - 1.0
        }),
        rim_plan_of(&prism, &regions, |middle| {
            middle.x < 1e-6 && middle.z > 1.0 && middle.z < lobed_prism_meshes::HEIGHT - 1.0
        }),
    ];

    surface_bevel_builder::try_build(
        &prism,
        &graph,
        &regions,
        &plans,
        SEGMENTS,
        &CancelToken::new(),
        SurfaceBuildOptions::default(),
    )
    .unwrap_or_else(|reason| panic!("the build refused the two rims: {reason:?}"))
    .mesh
}

/// The rim of the region of the first facet whose middle answers `is_on_it`.
fn rim_plan_of(
    prism: &Mesh,
    regions: &Arc<FaceRegionCache>,
    is_on_it: impl Fn(Vector3) -> bool,
) -> Rc<SurfaceBevelPlan> {
    let found = (0..prism.faces.len())
        .find(|&face| {
            let facet = prism.faces[face];
            let middle = (Vector3::from_vector3_float(prism.vertices[facet.v0 as usize])
                + Vector3::from_vector3_float(prism.vertices[facet.v1 as usize])
                + Vector3::from_vector3_float(prism.vertices[facet.v2 as usize]))
                / 3.0;

            is_on_it(middle)
        })
        .expect("no facet of the prism stands where the pick says");

    let rim = regions
        .region(found as i32)
        .expect("a region")
        .boundary_edges()
        .to_vec();
    let chains = bevel_chain_linker::link(&rim);
    assert_eq!(chains.len(), 1);

    SurfaceBevelPlan::try_create(&chains[0], regions, RADIUS, true)
        .unwrap_or_else(|reason| panic!("the plan refused the rim: {reason}"))
}
