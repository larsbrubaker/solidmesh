//! The sliver a POKE-THROUGH leaves, measured on the kernel's own boolean output rather than
//! described - `gusset_test_meshes` is the left half of "Chain and Face Should Both work.mcx" as
//! the union left it, and how its top face reads beside it.
//!
//! The gusset stands 0.1 mm proud of the plate's y = -6 face and carries a 1.3113417e-07 term in
//! its own rotation, so its top misses z = 20 by a hair. Between the two the boolean leaves a
//! triangle 0.1414 long and two MICRONS thick, whose normal is (-0.696, 0.696, 0.174) - nothing
//! like either surface it lies between. Its corners are placed only to about the size of its
//! thickness, so that normal is rounding noise: the sliver reads the face across its long edge as
//! its own and is no face region of its own, and a 0.4 fillet of the top face fits past it.

use std::rc::Rc;
use std::sync::Arc;

use solidmesh::bevel::bevel_chain_linker;
use solidmesh::bevel::surface::{
    surface_chain_join, surface_chain_opens_at_the_junction, surface_room, SurfaceBevelPlan,
};
use solidmesh::topology::{FaceGroup, FaceRegionCache};
use solidmesh_math::Vector3;
use solidmesh_mesh::{CancelToken, Face, Mesh, MeshEdgeGraph};

use crate::gusset_test_meshes;

/// The fillet the fixture's own operation asks for.
const RADIUS: f64 = 0.4;

/// The bend threshold the fixture's operation carries, which its face group was picked at.
const FLAT_ANGLE_DEGREES: f64 = 10.0;

/// The sliver is there, and it is the one measured under the running application: three corners,
/// two of them two microns apart, and a normal belonging to neither surface it lies between.
#[test]
fn the_poke_through_leaves_a_triangle_two_microns_thick() {
    let part = gusset_test_meshes::create_plate_with_gusset_poking_through();
    let sliver = the_sliver_face(&part);

    let corners = corners(&part, &sliver);

    assert_eq!(
        corners
            .iter()
            .filter(|corner| (**corner - the_buried_tip()).length() < 1e-5)
            .count(),
        2,
        "two of its three corners are the gusset's buried tip read twice - {corners:?}"
    );

    assert_eq!(
        corners
            .iter()
            .filter(|corner| (**corner - the_seams_far_end()).length() < 1e-5)
            .count(),
        1,
        "and the third is where the hypotenuse leaves the plate - {corners:?}"
    );

    assert!(
        smallest_altitude(&corners) < 1e-5,
        "the whole of it is two microns thick, which is what makes it a sliver rather than a face - it measures {}",
        smallest_altitude(&corners)
    );

    assert!(
        Vector3::from_vector3_float(sliver.normal)
            .dot(Vector3::UNIT_Z)
            .abs()
            < 0.2,
        "and its normal stands nearly square to the two tops it lies between - it is {:?}",
        sliver.normal
    );
}

/// And why it costs nothing: two microns is less than the part's single-precision corners are
/// placed to, so the sliver has no normal of its own (`FeatureEdge::try_classify` reads the face
/// across its long edge in its place), no crease bounds it and it is no face region of its own.
/// The part's top face, picked whole, has room for the 0.4 its operation asks for.
#[test]
fn the_top_face_fits_past_a_sliver_too_thin_for_a_normal() {
    let part = gusset_test_meshes::create_plate_with_gusset_poking_through();
    let graph = part.get_mesh_edge_graph();
    let regions = Arc::new(FaceRegionCache::new(
        Arc::new(part.clone()),
        Some(part.get_mesh_edge_graph()),
        FLAT_ANGLE_DEGREES,
    ));
    let plans = the_top_faces_rim(&part, &graph, &regions);

    let fit =
        surface_room::try_fit(&plans, RADIUS, &CancelToken::new(), None).expect("nothing cancels");

    assert!(
        fit.fits && fit.largest_fitting_size >= RADIUS,
        "the sliver is no face of its own, so it is not in the way of the top face's rim: it allowed {} ({:?})",
        fit.largest_fitting_size,
        fit.refusal
    );
}

/// The gusset's buried tip on z = 20, which the boolean wrote three times over.
fn the_buried_tip() -> Vector3 {
    Vector3::new(25.5, -6.099996, 20.0)
}

/// Where the gusset's hypotenuse leaves the plate's y = -6 face.
fn the_seams_far_end() -> Vector3 {
    Vector3::new(25.599997, -6.0, 20.0)
}

/// The top face picked whole at the operation's own bend threshold and planned edge by edge, which
/// is what the pass does with a face group.
fn the_top_faces_rim(
    part: &Mesh,
    graph: &MeshEdgeGraph,
    regions: &Arc<FaceRegionCache>,
) -> Vec<Rc<SurfaceBevelPlan>> {
    let top =
        FaceGroup::create_with_graph(part, graph, the_top_seed_face(part), FLAT_ANGLE_DEGREES)
            .expect("the seed is a face of the part");
    let chains = surface_chain_opens_at_the_junction::open(surface_chain_join::join(
        &bevel_chain_linker::link(top.boundary_edges()),
    ));

    chains
        .iter()
        .map(|chain| {
            SurfaceBevelPlan::try_create(chain, regions, RADIUS, true)
                .unwrap_or_else(|why| panic!("the top face's rim has to plan at {RADIUS}: {why:?}"))
        })
        .collect()
}

/// A face in the middle of the plate's own top, well away from anything the gusset did.
fn the_top_seed_face(part: &Mesh) -> i32 {
    let mut best = -1;
    let mut nearest = f64::INFINITY;

    for (i, face) in part.faces.iter().enumerate() {
        if (face.normal.z as f64) < 0.99 {
            continue;
        }

        let centroid = (Vector3::from_vector3_float(part.vertices[face.v0 as usize])
            + Vector3::from_vector3_float(part.vertices[face.v1 as usize])
            + Vector3::from_vector3_float(part.vertices[face.v2 as usize]))
            / 3.0;

        let away = (centroid - Vector3::new(0.0, -8.0, 20.0)).length();

        if away < nearest {
            nearest = away;
            best = i as i32;
        }
    }

    best
}

/// The one triangle of the fixture whose corners are the seam, found by its own geometry.
fn the_sliver_face(part: &Mesh) -> Face {
    let found: Vec<Face> = part
        .faces
        .iter()
        .filter(|face| {
            let corners = corners(part, face);
            corners
                .iter()
                .filter(|corner| (**corner - the_buried_tip()).length() < 1e-5)
                .count()
                == 2
                && corners
                    .iter()
                    .filter(|corner| (**corner - the_seams_far_end()).length() < 1e-5)
                    .count()
                    == 1
        })
        .cloned()
        .collect();
    assert_eq!(found.len(), 1, "exactly one face is the sliver");
    found[0]
}

fn corners(part: &Mesh, face: &Face) -> [Vector3; 3] {
    [
        Vector3::from_vector3_float(part.vertices[face.v0 as usize]),
        Vector3::from_vector3_float(part.vertices[face.v1 as usize]),
        Vector3::from_vector3_float(part.vertices[face.v2 as usize]),
    ]
}

/// The shortest distance from a corner to the line of the opposite side - a triangle's thickness.
fn smallest_altitude(corners: &[Vector3; 3]) -> f64 {
    let twice_the_area = (corners[1] - corners[0])
        .cross(corners[2] - corners[0])
        .length();
    let longest = (corners[1] - corners[0]).length().max(
        (corners[2] - corners[1])
            .length()
            .max((corners[0] - corners[2]).length()),
    );

    if longest > 0.0 {
        twice_the_area / longest
    } else {
        0.0
    }
}
