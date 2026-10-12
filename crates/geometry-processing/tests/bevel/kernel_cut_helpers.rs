//! The private helpers several C# bevel test classes each carry a copy of (`Boolean`, `Cut`,
//! `EdgeIndex`, `Group` - BevelShadowClipTests, BevelShadowBodyTests, BevelSeamRingTests,
//! BevelThreeRunCornerTests), shared here once: the n-ary boolean at C#'s `Polygons` settings, the
//! part with a shadow-clipped cutter taken out of it, the feature edge between two known
//! positions, and the one-edge group picked there.

use polygon_mesh::csg::boolean_processing::do_array;
use polygon_mesh::csg::{BooleanOptions, CsgModes};
use polygon_mesh::{CancelToken, Mesh};
use vector_math::{Matrix4X4, Vector3};

use geometry_processing::topology::bevel_edge_group_resolver;
use geometry_processing::topology::{
    BevelEdgeGroup, BevelSelectionKind, FaceGroup, FeatureEdgeSet,
};

/// C# `Boolean`: `BooleanProcessing.DoArray` over `meshes` in polygon mode.
pub fn boolean(meshes: &[&Mesh], mode: CsgModes) -> Mesh {
    let items: Vec<(&Mesh, Matrix4X4)> = meshes.iter().map(|&m| (m, Matrix4X4::IDENTITY)).collect();

    do_array(
        &items,
        mode,
        &CancelToken::new(),
        &BooleanOptions::default(),
    )
    .unwrap_or_else(|error| panic!("the boolean was refused: {error:?}"))
}

/// C# `Cut`: the part with the clipped cutter taken out of it, which is what a rebuild publishes.
/// `shadows` are the columns (and bodies) the cutter must not reach into.
pub fn cut(part: &Mesh, cutter: &Mesh, shadows: &[Mesh]) -> Mesh {
    let clipped;
    let mut cutter = cutter;

    if !shadows.is_empty() {
        let mut operands = vec![cutter];
        operands.extend(shadows.iter());
        clipped = boolean(&operands, CsgModes::Subtract);
        cutter = &clipped;
    }

    boolean(&[part, cutter], CsgModes::Subtract)
}

/// C# `EdgeIndex`: the feature edge running between two known positions, whichever way round the
/// mesh stores it, matched `within` that distance at both ends.
pub fn edge_index(features: &FeatureEdgeSet, from: Vector3, to: Vector3, within: f64) -> i32 {
    for (i, edge) in features.edges().iter().enumerate() {
        if ((edge.start - from).length() < within && (edge.end - to).length() < within)
            || ((edge.start - to).length() < within && (edge.end - from).length() < within)
        {
            return i as i32;
        }
    }

    panic!("the fixture has no feature edge from {from:?} to {to:?}");
}

/// C# `Group`: the edge pick of the feature edge running between two known positions (matched
/// within 1e-5).
pub fn edge_group(features: &FeatureEdgeSet, from: Vector3, to: Vector3) -> BevelEdgeGroup {
    bevel_edge_group_resolver::create_from_edge(
        features,
        edge_index(features, from, to, 1e-5),
        BevelSelectionKind::Edge,
        FaceGroup::DEFAULT_CURVE_TOLERANCE_DEGREES,
    )
    .expect("an edge of the set")
}
