// Unit pins of the surface part builder's private bookkeeping, which no whole-build test reaches
// directly: the touching loops left alone where the band refused a touch, and
// put back once their partner went (`touching_loops`). The touch the band reads is the level
// line's to set, so these set it directly.

use std::rc::Rc;
use std::sync::Arc;

use crate::bevel::{BevelAdvisoryArg, BevelAdvisoryCode, BevelLoopFailure, BevelRebuildAdvisories};
use crate::topology::{BevelEdgeGroup, BevelEdgeGroupResolution, FeatureEdge, FeatureEdgeSet};
use indexmap::IndexMap;
use polygon_mesh::Mesh;
use vector_math::Color;
use vector_math::{Matrix4X4, Vector3, Vector3Float};

use super::{BevelSurfacePartBuilder, RimLoop};

/// One closed triangle loop through these vertices, its edges numbered from `first_edge`.
fn triangle_loop(vertices: [i32; 3], first_edge: i32) -> Vec<FeatureEdge> {
    (0..3)
        .map(|k| FeatureEdge {
            mesh_edge_index: first_edge + k as i32,
            start_vertex: vertices[k],
            end_vertex: vertices[(k + 1) % 3],
            ..FeatureEdge::default()
        })
        .collect()
}

/// Four groups, each one closed triangle loop: loops 0 and 1 share vertex 2; 2 and 3 touch nobody.
fn four_loop_builder() -> (BevelSurfacePartBuilder, Vec<Rc<RimLoop>>) {
    let mut mesh = Mesh::new();
    for index in 0..12 {
        mesh.vertices
            .push(Vector3Float::new(index as f64, 2.0 * index as f64, 0.5));
    }

    let features = Arc::new(FeatureEdgeSet::create(&mesh));
    let loops = [[0, 1, 2], [2, 3, 4], [5, 6, 7], [8, 9, 10]];
    let resolutions: Vec<BevelEdgeGroupResolution> = loops
        .iter()
        .enumerate()
        .map(|(group, vertices)| BevelEdgeGroupResolution {
            group: BevelEdgeGroup::default(),
            features: features.clone(),
            failure_reason: None,
            seed_edge_index: 0,
            edges: triangle_loop(*vertices, 10 * group as i32),
            chain: None,
            face_group: None,
        })
        .collect();
    let graph = mesh.get_mesh_edge_graph();

    let mut builder = BevelSurfacePartBuilder::new(
        vec![BevelEdgeGroup::default(); 4],
        resolutions,
        vec![true; 4],
        Rc::new(BevelRebuildAdvisories::new(None::<Vec<_>>)),
        Arc::new(mesh),
        graph,
        1.0,
        8,
        true,
        Matrix4X4::IDENTITY,
        Color::default(),
        None,
        None,
        None,
    );
    let rim = builder.the_rim_loops_of(&[0, 1, 2, 3]);

    (builder, rim)
}

#[test]
fn rust_only_a_touch_leaves_one_loop_of_the_pair_alone_and_puts_it_back_once_its_partner_went() {
    let (mut builder, rim) = four_loop_builder();
    assert_eq!(rim.len(), 4);

    // No touch read: nothing is named.
    assert!(builder.the_touching_loops_left_alone(&rim).is_none());

    // The band refused a touch, but the settling is off: the pass goes on down the older path.
    builder.the_band_refused_a_touch = true;
    builder.the_settling_is_off = true;
    assert!(builder.the_touching_loops_left_alone(&rim).is_none());
    assert!(builder.the_band_refused_a_touch);

    builder.the_settling_is_off = false;
    let condemned = builder
        .the_touching_loops_left_alone(&rim)
        .expect("one loop of the touching pair is named");
    // The touch is read once.
    assert!(!builder.the_band_refused_a_touch);

    assert_eq!(condemned.len(), 1);
    assert_eq!(builder.left_alone_for_touching().len(), 1);
    let (left, (partner, said)) = builder
        .left_alone_for_touching()
        .iter()
        .map(|(one, held)| (one.clone(), held.clone()))
        .next()
        .unwrap();
    assert!(
        (left == rim[0] && partner == rim[1]) || (left == rim[1] && partner == rim[0]),
        "only loops 0 and 1 touch"
    );
    assert_eq!(said.code(), BevelAdvisoryCode::LoopTouchesAnotherPickedLoop);
    // The sentence stands on the shared vertex, 2.
    assert_eq!(
        said.args(),
        &[BevelAdvisoryArg::Vector3(Vector3::new(2.0, 4.0, 0.5))]
    );
    assert_eq!(condemned[0].mesh_edges, left.mesh_edges);
    assert_eq!(condemned[0].reason, said);
    assert_eq!(condemned[0].settled, None);

    // The pass dropped the left-alone loop with its refusal; its partner still stands, so it
    // stays out.
    let mut refused_loops: IndexMap<i32, Vec<BevelLoopFailure>> = IndexMap::new();
    refused_loops.insert(
        left.group_index,
        vec![BevelLoopFailure {
            advisory: said.clone(),
            largest_size_that_fits: None,
            edge_count: 3,
        }],
    );
    let results = vec![None, None, None, None];
    let mut contributing: Vec<Rc<RimLoop>> =
        rim.iter().filter(|one| **one != left).cloned().collect();
    assert!(builder.put_back_the_loops_whose_partner_went(
        &rim,
        &mut contributing,
        &results,
        &mut refused_loops
    ));
    assert_eq!(contributing.len(), 3);
    assert_eq!(builder.left_alone_for_touching().len(), 1);

    // The partner is dropped in turn: the loop left alone goes back in, in the pick's order, and
    // loses its touch sentence.
    contributing.retain(|one| *one != partner);
    assert!(builder.put_back_the_loops_whose_partner_went(
        &rim,
        &mut contributing,
        &results,
        &mut refused_loops
    ));
    let expected: Vec<Rc<RimLoop>> = rim.iter().filter(|one| **one != partner).cloned().collect();
    assert_eq!(contributing, expected);
    assert!(builder.left_alone_for_touching().is_empty());
    assert!(refused_loops.is_empty());
    assert!(builder.put_back_once().contains(&left));

    // Once only: left alone again, it is not put back a second time.
    builder.the_band_refused_a_touch = true;
    let mut again: Vec<Rc<RimLoop>> = rim.clone();
    builder.the_touching_loops_left_alone(&again);
    again.retain(|one| *one != left && *one != partner);
    builder.put_back_the_loops_whose_partner_went(&rim, &mut again, &results, &mut IndexMap::new());
    assert!(!again.contains(&left));
}
