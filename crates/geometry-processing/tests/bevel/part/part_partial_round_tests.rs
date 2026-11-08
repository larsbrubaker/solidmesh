// Rounds that overlap across a narrow face meet in a crest, and the rest of the pick rounds with
// them: the surface part builder, run whole on a thin slab where two picked edges' rounds overlap
// across the narrow face between them and a third picked edge has all the room it needs. Each of
// the three rounds alone at the size, so the three round together: the face between the two goes
// and their rounds meet in a crest, and the third rounds on what they left.

use std::rc::Rc;
use std::sync::Arc;

use solidmesh::bevel::part::surface_part_builder::BevelSurfacePartBuilder;
use solidmesh::bevel::BevelRebuildAdvisories;
use solidmesh::topology::{BevelEdgeGroup, BevelEdgeGroupResolution, FeatureEdge, FeatureEdgeSet};
use solidmesh_math::Color;
use solidmesh_math::Matrix4X4;
use solidmesh_mesh::csg::boolean_processing::{classify_boolean_operand, BooleanOperandVerdict};
use solidmesh_mesh::solids::platonic_solids::create_cube;
use solidmesh_mesh::CancelToken;

/// The slab's thickness: the front face between the two colliding edges is this tall.
const THICKNESS: f64 = 2.0;

/// The round: wider than half the front face, so the top-front and bottom-front rounds overlap
/// on it, and narrower than the face, so each fits on its own.
const SIZE: f64 = 1.5;

#[test]
fn edges_whose_rounds_overlap_across_a_narrow_face_meet_in_a_crest_and_the_rest_of_the_pick_rounds()
{
    let mesh = create_cube(20.0, 20.0, THICKNESS);
    let features = Arc::new(FeatureEdgeSet::create(&mesh));
    let half = THICKNESS / 2.0;
    let the_edge = |y: f64, z: f64| -> Vec<FeatureEdge> {
        features
            .edges()
            .iter()
            .filter(|edge| {
                (edge.start.y - y).abs() < 1e-9
                    && (edge.end.y - y).abs() < 1e-9
                    && (edge.start.z - z).abs() < 1e-9
                    && (edge.end.z - z).abs() < 1e-9
            })
            .copied()
            .collect()
    };
    // Top-front and bottom-front overlap across the front face; top-back stands alone.
    let picks = [
        the_edge(-10.0, half),
        the_edge(-10.0, -half),
        the_edge(10.0, half),
    ];
    for edges in &picks {
        assert_eq!(edges.len(), 1, "each pick is one edge of the slab");
    }
    let resolutions: Vec<BevelEdgeGroupResolution> = picks
        .iter()
        .map(|edges| BevelEdgeGroupResolution {
            group: BevelEdgeGroup::default(),
            features: features.clone(),
            failure_reason: None,
            seed_edge_index: 0,
            edges: edges.clone(),
            chain: None,
            face_group: None,
        })
        .collect();
    let graph = mesh.get_mesh_edge_graph();

    let mut builder = BevelSurfacePartBuilder::new(
        vec![BevelEdgeGroup::default(); 3],
        resolutions,
        vec![true; 3],
        Rc::new(BevelRebuildAdvisories::new(None::<Vec<_>>)),
        Arc::new(mesh),
        graph,
        SIZE,
        8,
        true,
        Matrix4X4::IDENTITY,
        Color::default(),
        None,
        None,
        None,
    );

    let result = builder
        .build(&CancelToken::new())
        .expect("a refusal is a row, not an error");
    let rows: Vec<_> = result
        .group_results
        .iter()
        .map(|row| row.clone().expect("every group gets a row"))
        .collect();

    assert!(
        rows.iter().all(|row| !row.failed()),
        "every edge rounds: {:?}",
        rows.iter().map(|row| &row.failure).collect::<Vec<_>>()
    );
    let built = result.mesh.expect("the part publishes the rounds");
    assert!(built.is_closed_consistently_wound());
    assert_ne!(
        classify_boolean_operand(&built, Matrix4X4::IDENTITY, false),
        BooleanOperandVerdict::SelfIntersecting
    );

    // The crest: the front-most vertices, on the slab's middle plane z = 0, where the two rounds'
    // circles (centres SIZE in from the front, SIZE - 1 inside the top and bottom) cross:
    // y = -10 + SIZE - sqrt(SIZE^2 - (SIZE - THICKNESS / 2)^2).
    let reach = SIZE - THICKNESS / 2.0;
    let crest = -10.0 + SIZE - (SIZE * SIZE - reach * reach).sqrt();
    let front = built
        .vertices
        .iter()
        .map(|v| f64::from(v.y))
        .fold(f64::MAX, f64::min);
    assert!((front - crest).abs() < 1e-5, "front {front}, crest {crest}");
    assert!(built
        .vertices
        .iter()
        .filter(|v| (f64::from(v.y) - front).abs() < 1e-5)
        .all(|v| v.z.abs() < 1e-6));
}
