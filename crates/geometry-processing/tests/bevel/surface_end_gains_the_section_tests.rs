//! A CONVEX end whose corner is a reentrant vertex of
//! the face beyond it gains the section as a cap (`surface_end_gains_the_section`). The
//! nose-in-notch part's ridge rounded at 2 is the fixture: both of its ends die where the nose
//! bites into a slope, the section stands in that bite rather than in the slope, and the cut that
//! was supposed to take it out of the slope found nothing there to take.
//!
//! The ring's two rails are the seam ends the strip already stands on and the corner is welded
//! into the pool, so the cap shares every vertex of its boundary - 11 ring vertices fanned from
//! one apex are 10 triangles over 12 vertices, and nothing is repaired after the fact.

use std::rc::Rc;
use std::sync::Arc;

use geometry_processing::bevel::surface::{
    surface_bevel_builder, surface_mesh_closure, surface_room, SurfaceBevelPlan,
    SurfaceBuildOptions,
};
use geometry_processing::bevel::BevelSweptChain;
use geometry_processing::topology::{ChainEdge, FaceRegionCache, FeatureEdgeSet};
use indexmap::IndexSet;
use polygon_mesh::csg::manifold_import;
use polygon_mesh::{CancelToken, Mesh};
use vector_math::Vector3;

use crate::mesh_measurements::{shell_count, signed_volume};
use crate::nose_in_notch_test_meshes;
use crate::surface_walkback_checks::{boundary_loops, regions_of};

const SIZE: f64 = 2.0;

const SEGMENTS: i32 = 8;

/// What the part measures before anything is beaded: the round takes material off it.
const BEFORE_THE_BEAD: f64 = 8000.0 - 2000.0 + (500.0 / 9.0);

/// How far from the corner the section's own ring stands: 2.27 at the arc's apex and 3.90 at the
/// two tangent points, measured on the slope the ring is cut into.
const NEAREST_ON_THE_RING: f64 = 2.27;

const FURTHEST_ON_THE_RING: f64 = 3.91;

#[test]
fn the_ridges_ends_gain_their_sections_and_the_shell_closes() {
    let part = nose_in_notch_test_meshes::create();
    let (regions, features) = regions_of(&Arc::new(part.clone()));
    let regions = Arc::new(regions);
    let built = the_ridge_rounded(&part, &regions, &features, SIZE);

    let (open, open_at) = surface_mesh_closure::open_edge_count(&built);
    assert_eq!(
        open, 0,
        "each end's ring stands on the cap its own section is - open at {open_at:?}"
    );

    assert_eq!(
        boundary_loops(&built).len(),
        0,
        "so there is no boundary left at either end"
    );

    assert!(built.is_manifold(), "and it closes manifold");
    assert_eq!(shell_count(&built), 1, "in one piece");

    assert!(
        signed_volume(&built) < BEFORE_THE_BEAD,
        "a convex round takes material off"
    );

    for (name, corner, normal) in [
        (
            "left",
            nose_in_notch_test_meshes::left_ridge_end(),
            nose_in_notch_test_meshes::left_slope_normal(),
        ),
        (
            "right",
            nose_in_notch_test_meshes::right_ridge_end(),
            nose_in_notch_test_meshes::right_slope_normal(),
        ),
    ] {
        let cap = the_cap_at(&built, corner, normal);

        assert_eq!(
            cap.len(),
            10,
            "the {name} end's section is the ring's 11 vertices fanned from its corner, which is \
             10 triangles - instead {}",
            cap.len()
        );

        let vertices: Vec<Vector3> = cap
            .iter()
            .flat_map(|&(v0, v1, v2)| [v0, v1, v2])
            .collect::<IndexSet<i32>>()
            .into_iter()
            .map(|vertex| Vector3::from_vector3_float(built.vertices[vertex as usize]))
            .collect();

        assert_eq!(
            vertices.len(),
            12,
            "over the corner and the ring's 11 and nothing else - instead {}",
            vertices.len()
        );

        for at in vertices {
            let away = (at - corner).length();

            assert!(
                away < 1e-6 || (away > NEAREST_ON_THE_RING && away < FURTHEST_ON_THE_RING),
                "every corner of the {name} cap is the corner itself or one of the ring's, which \
                 stand {NEAREST_ON_THE_RING} to {FURTHEST_ON_THE_RING} out - instead {at:?} at \
                 {away:.4}"
            );
        }
    }
}

/// And with the cube's knife edge at the origin picked as well - the pair a directional notch
/// reach lets fit (`surface_room_notch_reach_tests`) - the two really are clear of each other: the
/// build closes, the boolean kernel takes it, and the knife's own foot on the slope at 4.8284 from
/// its edge is where it always was.
#[test]
fn the_ridge_and_the_knife_edge_build_together_at_the_size_the_fit_allows() {
    let part = nose_in_notch_test_meshes::create();
    let (regions, features) = regions_of(&Arc::new(part.clone()));
    let regions = Arc::new(regions);

    let mut plans: Vec<Rc<SurfaceBevelPlan>> = Vec::new();

    for chain in [
        edge_between(
            &features,
            nose_in_notch_test_meshes::left_ridge_end(),
            nose_in_notch_test_meshes::right_ridge_end(),
        ),
        edge_between(
            &features,
            Vector3::ZERO,
            Vector3::new(0.0, 0.0, nose_in_notch_test_meshes::SIZE),
        ),
    ] {
        match SurfaceBevelPlan::try_create(&chain, &regions, SIZE, true) {
            Ok(plan) => plans.push(plan),
            Err(why) => panic!("{:?}", why.code()),
        }
    }

    let fit =
        surface_room::try_fit(&plans, SIZE, &CancelToken::new(), None).expect("not cancelled");
    assert!(
        fit.fits,
        "the ridge's section leans away from the knife's band - instead {:?} at {:.5}",
        fit.refusal.as_ref().map(|refusal| refusal.code()),
        fit.largest_fitting_size
    );

    let built = surface_bevel_builder::try_build(
        &part,
        &part.get_mesh_edge_graph(),
        &regions,
        &plans,
        SEGMENTS,
        &CancelToken::new(),
        SurfaceBuildOptions {
            crossings: Some(&fit.crossings),
            ..SurfaceBuildOptions::default()
        },
    )
    .unwrap_or_else(|reason| panic!("and so the two build together - {reason:?}"))
    .mesh;

    let (open, open_at) = surface_mesh_closure::open_edge_count(&built);
    assert_eq!(open, 0, "closed - open at {open_at:?}");
    assert_eq!(
        boundary_loops(&built).len(),
        0,
        "with no boundary loop left"
    );
    assert!(built.is_manifold(), "and manifold");
    assert_eq!(shell_count(&built), 1, "in one piece");

    assert!(
        manifold_import::import_strict(&built).is_ok(),
        "the boolean kernel has to take it"
    );

    // The knife's foot on the slope, 4.8284 from its own edge: the straight rail (3.4142, 3.4142,
    // z) running the whole height of the cube. The ridge's section starts 9.34 out along the same
    // slope, so the foot is untouched by it - the two are 4.5 mm of bare slope apart, which is the
    // clearance the omnidirectional reach used to read as a 0.757 mm overlap.
    for z in [0.0, nose_in_notch_test_meshes::SIZE] {
        #[allow(clippy::excessive_precision)]
        let foot = Vector3::new(3.4142135623730951, 3.4142135623730951, z);
        let carried = built
            .vertices
            .iter()
            .filter(|&&vertex| (Vector3::from_vector3_float(vertex) - foot).length() < 1e-4)
            .count();

        assert_eq!(
            carried, 1,
            "the knife's tangent foot ends at {foot:?}, once - instead {carried}"
        );
    }

    // And nothing of the mesh stands on the bare slope between them: the knife's foot at 4.8284
    // and the ridge's own corner at 7.0711, with 2.2427 of untouched slope in between.
    for at in [5.5, 6.0, 6.5] {
        let between = built
            .vertices
            .iter()
            .filter(|&&vertex| {
                let vertex = Vector3::from_vector3_float(vertex);
                (vertex.x - vertex.y).abs() < 1e-6
                    && ((vertex.x + vertex.y) / 2.0_f64.sqrt() - at).abs() < 0.25
            })
            .count();

        assert_eq!(
            between, 0,
            "the slope is bare {at} out from the knife edge - instead {between} vertices stand there"
        );
    }
}

/// The ridge alone, planned and built at one size.
fn the_ridge_rounded(
    part: &Mesh,
    regions: &Arc<FaceRegionCache>,
    features: &FeatureEdgeSet,
    size: f64,
) -> Mesh {
    let chain = edge_between(
        features,
        nose_in_notch_test_meshes::left_ridge_end(),
        nose_in_notch_test_meshes::right_ridge_end(),
    );

    let plan = SurfaceBevelPlan::try_create(&chain, regions, size, true)
        .unwrap_or_else(|why| panic!("the ridge would not plan: {:?}", why.code()));

    surface_bevel_builder::try_build_one(
        part,
        &part.get_mesh_edge_graph(),
        regions,
        plan,
        SEGMENTS,
        &CancelToken::new(),
    )
    .unwrap_or_else(|reason| panic!("the ridge would not build: {reason:?}"))
}

/// The cap's triangles at one end: those lying in the end face's own plane and wholly within the
/// section's reach of the corner, which is the section and nothing of the slope beyond it.
fn the_cap_at(mesh: &Mesh, corner: Vector3, normal: Vector3) -> Vec<(i32, i32, i32)> {
    let mut cap = Vec::new();

    for face in mesh.faces.iter() {
        let mut on_the_cap = true;

        for vertex in [face.v0, face.v1, face.v2] {
            let at = Vector3::from_vector3_float(mesh.vertices[vertex as usize]);

            on_the_cap &= (at - corner).dot(normal).abs() < 1e-6
                && (at - corner).length() < FURTHEST_ON_THE_RING;
        }

        if on_the_cap {
            cap.push((face.v0, face.v1, face.v2));
        }
    }

    cap
}

/// The fixture's one feature edge between two named vertices, as a chain of its own.
fn edge_between(features: &FeatureEdgeSet, from: Vector3, to: Vector3) -> BevelSweptChain {
    for (i, edge) in features.edges().iter().enumerate() {
        if (edge.start - from).length() < 1e-6 && (edge.end - to).length() < 1e-6 {
            return BevelSweptChain::new(vec![ChainEdge::new(i as i32, *edge, false)], false);
        }

        if (edge.end - from).length() < 1e-6 && (edge.start - to).length() < 1e-6 {
            return BevelSweptChain::new(vec![ChainEdge::new(i as i32, *edge, true)], false);
        }
    }

    panic!("the fixture has no edge from {from:?} to {to:?}");
}
