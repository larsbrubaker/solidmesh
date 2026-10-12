//! Port of `SurfaceChainOpensAtTheJunctionTests.cs`: the pass that cuts a CLOSED chain open where
//! another chain ends on it, read on a cube - a face rim taken whole with one vertical edge picked
//! beside it is three edges meeting at a corner, and the junction readings want three chain ENDS
//! there whichever way the three were picked.

use geometry_processing::bevel::bevel_chain_linker;
use geometry_processing::bevel::surface::surface_chain_opens_at_the_junction;
use geometry_processing::bevel::BevelSweptChain;
use geometry_processing::topology::{ChainEdge, FeatureEdge, FeatureEdgeSet};
use polygon_mesh::solids::platonic_solids;
use vector_math::Vector3;

const CUBE_SIZE: f64 = 20.0;

fn the_corner() -> Vector3 {
    Vector3::new(CUBE_SIZE / 2.0, -CUBE_SIZE / 2.0, CUBE_SIZE / 2.0)
}

fn the_opposite_corner() -> Vector3 {
    Vector3::new(-CUBE_SIZE / 2.0, CUBE_SIZE / 2.0, CUBE_SIZE / 2.0)
}

/// A rim nobody else stands on is left exactly as it arrived, closed and in one piece.
#[test]
fn a_rim_nobody_ends_on_stays_closed() {
    let features = the_cubes_edges();
    let chains = bevel_chain_linker::link(&the_top_rim(&features));

    let opened = surface_chain_opens_at_the_junction::open(chains.clone());

    assert_eq!(opened.len(), 1);
    assert!(opened[0].is_closed);
    // C# `IsSameReferenceAs`: a chain is a value here, so "the chain that arrived, not a copy" is
    // the chain handed back unchanged - every edge, in order, walked the same way.
    assert_eq!(
        opened[0], chains[0],
        "a chain with nothing to open is the chain that arrived, not a copy of it"
    );
}

/// One end standing on the rim opens it there: the same four edges, in the same order and each
/// walked the same way, rotated so the run starts and finishes at the junction.
#[test]
fn one_end_on_the_rim_opens_it_there() {
    let features = the_cubes_edges();
    let rim = bevel_chain_linker::link(&the_top_rim(&features));
    let arriving = bevel_chain_linker::link(&[edge_between(
        &features,
        the_corner(),
        Vector3::new(CUBE_SIZE / 2.0, -CUBE_SIZE / 2.0, -CUBE_SIZE / 2.0),
    )]);

    let opened = surface_chain_opens_at_the_junction::open(
        rim.iter().chain(arriving.iter()).cloned().collect(),
    );

    assert_eq!(opened.len(), 2);

    let run = single(&opened, |chain| chain.edges.len() == 4);
    let junction = vertex_at(&rim[0], the_corner());

    assert!(!run.is_closed);
    assert_eq!(run.edges[0].from_vertex(), junction);
    assert_eq!(run.edges[3].to_vertex(), junction);

    // The arriving chain is untouched, so all three of the edges at the corner are now chain ends.
    assert!(!single(&opened, |chain| chain.edges.len() == 1).is_closed);

    assert_it_is_the_same_walk_rotated(&rim[0], run);
}

/// Two ends on one rim cut it into two runs - the two stretches between the junctions - and
/// between them they are still the rim, once round, the way it was walked.
#[test]
fn two_ends_on_the_rim_cut_it_into_two_runs() {
    let features = the_cubes_edges();
    let rim = bevel_chain_linker::link(&the_top_rim(&features));

    let arriving = bevel_chain_linker::link(&[edge_between(
        &features,
        the_corner(),
        Vector3::new(CUBE_SIZE / 2.0, -CUBE_SIZE / 2.0, -CUBE_SIZE / 2.0),
    )]);

    let also_arriving = bevel_chain_linker::link(&[edge_between(
        &features,
        the_opposite_corner(),
        Vector3::new(-CUBE_SIZE / 2.0, CUBE_SIZE / 2.0, -CUBE_SIZE / 2.0),
    )]);

    let opened = surface_chain_opens_at_the_junction::open(
        rim.iter()
            .chain(arriving.iter())
            .chain(also_arriving.iter())
            .cloned()
            .collect(),
    );

    let runs: Vec<&BevelSweptChain> = opened
        .iter()
        .filter(|chain| chain.edges.len() == 2)
        .collect();

    assert_eq!(opened.len(), 4);
    assert_eq!(
        runs.len(),
        2,
        "the rim is cut at both junctions, into the two stretches between them"
    );

    let here = vertex_at(&rim[0], the_corner());
    let there = vertex_at(&rim[0], the_opposite_corner());

    for run in &runs {
        assert!(!run.is_closed);
        assert!(run.edges[0].from_vertex() == here || run.edges[0].from_vertex() == there);
        assert!(run.edges[1].to_vertex() == here || run.edges[1].to_vertex() == there);
        assert_ne!(run.edges[0].from_vertex(), run.edges[1].to_vertex());
    }

    // Laid end to end in the order they were published the two runs are the ring itself, rotated.
    let mut whole: Vec<ChainEdge> = runs[0].edges.clone();

    whole.extend(runs[1].edges.iter().cloned());

    assert_it_is_the_same_walk_rotated(&rim[0], &BevelSweptChain::new(whole, false));
}

/// Two rims that share an EDGE count four picked edges at each end of it - two from each rim -
/// which is no junction any patch reads, so both are left closed to be refused downstream by the
/// name that case has always had.
#[test]
fn two_rims_through_one_vertex_are_both_left_closed() {
    let features = the_cubes_edges();

    let mut rims = bevel_chain_linker::link(&the_top_rim(&features));
    rims.extend(bevel_chain_linker::link(&the_front_rim(&features)));

    assert_eq!(rims.len(), 2);

    let opened = surface_chain_opens_at_the_junction::open(rims);

    assert_eq!(opened.len(), 2);

    for chain in &opened {
        assert!(
            chain.is_closed,
            "four edges at one vertex is not the three ends a junction is read from"
        );
    }
}

/// The run really is the ring: the same edges, one for one, in the same order and each walked the
/// same way, started at some other point of the walk.
fn assert_it_is_the_same_walk_rotated(ring: &BevelSweptChain, run: &BevelSweptChain) {
    assert_eq!(run.edges.len(), ring.edges.len());

    let mut offset: i32 = -1;

    for (i, edge) in ring.edges.iter().enumerate() {
        if edge.edge_index == run.edges[0].edge_index
            && edge.is_reversed == run.edges[0].is_reversed
        {
            offset = i as i32;
        }
    }

    assert_ne!(
        offset, -1,
        "the run starts on an edge of the ring, walked the way the ring walked it"
    );

    for (i, edge) in run.edges.iter().enumerate() {
        let was = &ring.edges[(offset as usize + i) % ring.edges.len()];

        assert_eq!(edge.edge_index, was.edge_index);
        assert_eq!(
            edge.is_reversed, was.is_reversed,
            "rotating a ring may not turn any of its edges round"
        );
    }
}

/// LINQ `Single`: the one chain matching, failing on none or several.
fn single(
    chains: &[BevelSweptChain],
    matches: impl Fn(&BevelSweptChain) -> bool,
) -> &BevelSweptChain {
    let found: Vec<&BevelSweptChain> = chains.iter().filter(|chain| matches(chain)).collect();
    assert_eq!(found.len(), 1, "exactly one chain matches");
    found[0]
}

fn the_cubes_edges() -> FeatureEdgeSet {
    FeatureEdgeSet::create(&platonic_solids::create_cube(
        CUBE_SIZE, CUBE_SIZE, CUBE_SIZE,
    ))
}

/// The four edges of the cube's top face.
fn the_top_rim(features: &FeatureEdgeSet) -> Vec<FeatureEdge> {
    features
        .edges()
        .iter()
        .filter(|edge| {
            (edge.start.z - (CUBE_SIZE / 2.0)).abs() < 1e-6
                && (edge.end.z - (CUBE_SIZE / 2.0)).abs() < 1e-6
        })
        .copied()
        .collect()
}

/// The four edges of the face at `y = -10`, which shares one edge with the top rim.
fn the_front_rim(features: &FeatureEdgeSet) -> Vec<FeatureEdge> {
    features
        .edges()
        .iter()
        .filter(|edge| {
            (edge.start.y + (CUBE_SIZE / 2.0)).abs() < 1e-6
                && (edge.end.y + (CUBE_SIZE / 2.0)).abs() < 1e-6
        })
        .copied()
        .collect()
}

fn edge_between(features: &FeatureEdgeSet, from: Vector3, to: Vector3) -> FeatureEdge {
    let found: Vec<FeatureEdge> = features
        .edges()
        .iter()
        .filter(|edge| {
            ((edge.start - from).length() < 1e-6 && (edge.end - to).length() < 1e-6)
                || ((edge.start - to).length() < 1e-6 && (edge.end - from).length() < 1e-6)
        })
        .copied()
        .collect();
    assert_eq!(found.len(), 1);
    found[0]
}

/// The mesh's own index for a point, off a chain that stands on it.
fn vertex_at(chain: &BevelSweptChain, at: Vector3) -> i32 {
    for edge in &chain.edges {
        if (edge.from() - at).length() < 1e-6 {
            return edge.from_vertex();
        }

        if (edge.to() - at).length() < 1e-6 {
            return edge.to_vertex();
        }
    }

    panic!("no edge of the chain stands at {at:?}");
}
