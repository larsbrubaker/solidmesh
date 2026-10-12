//! Port of `SurfaceRakedBoreBlendSeamTests.cs`, a part of the partial class `SurfaceBoreMouthTests`:
//! THE BLEND'S OWN SEAM at the raked mouth - no edge of the built mouth is used by two faces walking
//! it the SAME way round, which is a piece laid alongside its neighbour rather than sewn to it, a
//! different fault from a hole and counted apart from one. The raked fixture's constants are in
//! `surface_raked_bore_mouth_tests.rs`, its mouth reading in `surface_bore_mouth_tests.rs`.

use geometry_processing::topology::{
    bevel_edge_group_resolver, BevelSelectionKind, FaceGroup, FeatureEdgeSet,
};
use indexmap::IndexMap;
use polygon_mesh::{CancelToken, Mesh};

use crate::surface_bore_mouth_tests::the_upper_raked_mouth;
use crate::surface_raked_bore_mouth_tests::{
    the_raked_part, BEND_THRESHOLD_DEGREES, SEGMENTS, SIZE,
};
use crate::surface_test_build::{expect_built, try_bevel, TryBevelOptions};

/// No edge of the built mouth is used by two faces walking it the SAME way round.
#[test]
fn the_raked_mouth_lays_no_piece_alongside_its_neighbour() {
    let mesh = the_raked_mouth_as_built();

    let same_way: Vec<((i32, i32), i32)> = each_edges_uses(&mesh)
        .into_iter()
        .filter(|&(_, (uses, balance))| uses == 2 && balance != 0)
        .map(|(edge, (_, balance))| (edge, balance))
        .collect();

    let first = same_way.first().map_or(String::new(), |&((low, high), _)| {
        format!(
            "{:?} to {:?}",
            mesh.vertices[low as usize].as_vector3(),
            mesh.vertices[high as usize].as_vector3()
        )
    });

    assert_eq!(
        same_way.len(),
        0,
        "two faces on one edge are joined along it, not laid back to back - instead {} edges carry \
         two faces walking them the same way, the first {first}",
        same_way.len()
    );
}

/// The raked mouth built at the fixture's own size.
fn the_raked_mouth_as_built() -> Mesh {
    let part = the_raked_part();
    let features = FeatureEdgeSet::create_with_graph(
        &part,
        &part.get_mesh_edge_graph(),
        BEND_THRESHOLD_DEGREES,
    );
    let group = bevel_edge_group_resolver::create_from_edge(
        &features,
        the_upper_raked_mouth(&features),
        BevelSelectionKind::TangentChain,
        FaceGroup::DEFAULT_CURVE_TOLERANCE_DEGREES,
    )
    .expect("the mouth's edge is in the set");

    expect_built(
        try_bevel(
            &part,
            &[group],
            SIZE,
            SEGMENTS,
            &CancelToken::new(),
            TryBevelOptions::default(),
        ),
        "the raked mouth does not build",
    )
}

/// How many faces use each edge of a mesh and how they are wound on it: the balance is one per
/// face walking it low to high and minus one per face walking it the other way.
fn each_edges_uses(mesh: &Mesh) -> IndexMap<(i32, i32), (i32, i32)> {
    let mut uses = IndexMap::with_capacity(mesh.faces.len() * 3);

    for face in mesh.faces.iter() {
        for (from, to) in [(face.v0, face.v1), (face.v1, face.v2), (face.v2, face.v0)] {
            let entry = uses.entry((from.min(to), from.max(to))).or_insert((0, 0));

            entry.0 += 1;
            entry.1 += if from < to { 1 } else { -1 };
        }
    }

    uses
}
