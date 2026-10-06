//! Port of `BevelBeadClosesOverStandOffTests.cs`: a bead's cap stands off the vertex where the
//! capping face closes back over the run, stays on it where the face looks outward, and a hairpin
//! pair of beads mitres where a right-angled pair does not.

use geometry_processing::bevel::bevel_chain_end_cap::ResolveOptions;
use geometry_processing::bevel::{
    bevel_chain_linker, bevel_corner_seam, BevelChainEnds, BevelSweptChain,
};
use geometry_processing::topology::{FeatureEdge, FeatureEdgeSet};
use polygon_mesh::Mesh;
use vector_math::Vector3;

use crate::nose_in_notch_test_meshes as nose;
use crate::topology_test_meshes::create_corner_pocket_block;

const RADIUS: f64 = 2.0;
const ON_A_PLANE: f64 = 1e-9;

fn resolve(
    run: &BevelSweptChain,
    size: f64,
    features: &FeatureEdgeSet,
    selected: &[FeatureEdge],
) -> BevelChainEnds {
    let mut warnings = Vec::new();
    BevelChainEnds::resolve(
        &run.edges,
        run.is_closed,
        size,
        Some(features),
        Some(selected),
        Some(&mut warnings),
        ResolveOptions::default(),
    )
}

/// C# `ABeadCapStandsOffTheVertexWhereTheFaceClosesOverTheRun`.
#[test]
fn a_bead_cap_stands_off_the_vertex_where_the_face_closes_over_the_run() {
    let part = nose::create();
    let features = FeatureEdgeSet::create(&part);
    let (run, selected, partner) = the_right_creases(&part);
    let ends = resolve(&run, RADIUS, &features, &selected);

    let cap = ends.end;
    let position = nose::lower_crease_end();
    assert!(cap.is_capped(), "the bead ends on the left slope");
    assert!(
        cap.plane_normal.dot(-nose::left_slope_normal()) > 0.99,
        "{:?}",
        cap.plane_normal
    );
    let stand_off = cap.plane_distance - cap.plane_normal.dot(position);
    assert!((stand_off - bevel_corner_seam::stand_off(false, position)).abs() <= ON_A_PLANE);
    assert!(stand_off > 0.0);
    assert_eq!(partner.edges.len(), 1);
}

/// C# `ABeadCapStaysOnTheVertexWhereTheFaceLooksOutward`.
#[test]
fn a_bead_cap_stays_on_the_vertex_where_the_face_looks_outward() {
    let part = nose::create();
    let features = FeatureEdgeSet::create(&part);
    let crease =
        features.edges()[crease_index(&features, nose::notch_floor(), nose::lower_crease_end())];
    let run = bevel_chain_linker::link(&[crease]).remove(0);
    let ends = resolve(&run, RADIUS, &features, &[crease]);

    let position = nose::notch_floor();
    let at_the_floor = if (run.edges[0].from() - position).length() < ON_A_PLANE {
        ends.start
    } else {
        ends.end
    };
    assert!(at_the_floor.is_capped());
    assert!(at_the_floor.plane_normal.z < -0.99);
    assert!(
        (at_the_floor.plane_distance - at_the_floor.plane_normal.dot(position)).abs() <= ON_A_PLANE
    );
}

/// C# `AHairpinPairOfBeadsMitresAndARightAngledPairDoesNot`.
#[test]
fn a_hairpin_pair_of_beads_mitres_and_a_right_angled_pair_does_not() {
    let part = nose::create();
    let features = FeatureEdgeSet::create(&part);
    let (lower, selected, upper) = the_right_creases(&part);
    let lower_ends = resolve(&lower, RADIUS, &features, &selected);
    let upper_ends = resolve(&upper, RADIUS, &features, &selected);

    for (name, cap, sense) in [
        ("lower", &lower_ends.start, 1.0),
        ("upper", &upper_ends.start, -1.0),
    ] {
        assert!(cap.is_capped(), "{name}");
        assert!(
            cap.plane_normal.z * sense > 0.999,
            "{name}: {:?}",
            cap.plane_normal
        );
        assert!(
            (cap.plane_distance * sense - nose::RIDGE_Z).abs() <= 1e-5,
            "{name}"
        );
    }

    let block = create_corner_pocket_block(20.0, 10.0);
    let pocket_features = FeatureEdgeSet::create(&block);
    let corner = Vector3::new(10.0, 10.0, 10.0);
    let along_x = pocket_features.edges()
        [crease_index(&pocket_features, corner, Vector3::new(20.0, 10.0, 10.0))];
    let along_y = pocket_features.edges()
        [crease_index(&pocket_features, corner, Vector3::new(10.0, 20.0, 10.0))];
    let floor_run = bevel_chain_linker::link(&[along_x]).remove(0);
    let pocket_ends = resolve(&floor_run, 1.0, &pocket_features, &[along_x, along_y]);
    let at_the_corner = if (floor_run.edges[0].from() - corner).length() < ON_A_PLANE {
        pocket_ends.start
    } else {
        pocket_ends.end
    };
    assert!(
        !at_the_corner.is_capped(),
        "a right-angled pair does not mitre"
    );
}

fn the_right_creases(part: &Mesh) -> (BevelSweptChain, Vec<FeatureEdge>, BevelSweptChain) {
    let (creases, names) = nose::creases(part);
    let index = |name: &str| names.iter().position(|n| n == name).unwrap();
    let upper = creases[index("upper right")].clone();
    let lower = creases[index("lower right")].clone();
    let selected = vec![lower.edges[0].edge, upper.edges[0].edge];
    (lower, selected, upper)
}

fn crease_index(features: &FeatureEdgeSet, from: Vector3, to: Vector3) -> usize {
    features
        .edges()
        .iter()
        .position(|edge| {
            ((edge.start - from).length() < 1e-5 && (edge.end - to).length() < 1e-5)
                || ((edge.start - to).length() < 1e-5 && (edge.end - from).length() < 1e-5)
        })
        .unwrap_or_else(|| panic!("the fixture has no feature edge from {from:?} to {to:?}"))
}
