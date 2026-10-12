//! Port of the test fixture `Fixtures/TwoMouthsInOneFacetMesh.cs`: Many Holes' refused facet at a size
//! that can be worked on - a 48-facet tapered barrel with two small radial bores close enough above and
//! below z = 0 that their near mouths stand wholly inside ONE flat piece of the wall. The wall is picked
//! as [`crate::tapered_barrel_test_meshes::the_wall`] picks it.

#![allow(dead_code)]

use polygon_mesh::csg::boolean_processing::do_array;
use polygon_mesh::csg::{BooleanOptions, CsgModes};
use polygon_mesh::{CancelToken, Mesh};
use vector_math::{math_helper, Matrix4X4};

use geometry_processing::bevel::bevel_chain_linker::{self, BevelSweptChain};
use geometry_processing::bevel::bevel_working_mesh::{create_from, StampedVerdict};
use geometry_processing::topology::{
    bevel_edge_group_resolver, BevelEdgeGroupResolution, FaceGroup, FeatureEdgeSet,
};

use crate::{tapered_barrel_test_meshes, topology_test_meshes};

pub const FACETS: i32 = 48;
pub const TOP_RADIUS: f64 = 12.0;
pub const BOTTOM_RADIUS: f64 = 16.0;
pub const HEIGHT: f64 = 30.0;
/// Small against the facet, so a mouth never reaches a fold.
pub const BORE_RADIUS: f64 = 0.8;
pub const BORE_SIDES: i32 = 24;
/// The Many Holes row's own size, whose setback carries the boundary over the piece's rim.
pub const SIZE: f64 = 0.2;
pub const SEGMENTS: i32 = 8;
pub const BEND_THRESHOLD_DEGREES: f64 = 10.0;
/// How far above and below z = 0 the two bores stand, close enough to share one flat piece.
pub const BORE_SPACING: f64 = 3.0;

/// C# `ThePart`: the barrel with both bores through the same stretch of its wall.
pub fn the_part() -> Mesh {
    let barrel = topology_test_meshes::create_frustum(BOTTOM_RADIUS, TOP_RADIUS, HEIGHT, FACETS);
    let upper = a_radial_bore(BORE_SPACING);
    let lower = a_radial_bore(-BORE_SPACING);

    do_array(
        &[
            (&barrel, Matrix4X4::IDENTITY),
            (&upper, Matrix4X4::IDENTITY),
            (&lower, Matrix4X4::IDENTITY),
        ],
        CsgModes::Subtract,
        &CancelToken::new(),
        &BooleanOptions::default(),
    )
    .expect("the bores cut the barrel")
}

/// C# `TheWorkingPart`: the part as the operation sees it - the working mesh, weld and repair passes
/// and all.
pub fn the_working_part() -> Mesh {
    create_from(&the_part(), &mut StampedVerdict::default())
}

/// C# `TheWall`: the wall as a face pick resolves it, exactly as the tapered barrel's.
pub fn the_wall(part: &Mesh) -> BevelEdgeGroupResolution {
    let graph = part.get_mesh_edge_graph();
    let features = FeatureEdgeSet::create_with_graph(part, &graph, BEND_THRESHOLD_DEGREES);
    let face_group = FaceGroup::create_with_graph(
        part,
        &graph,
        tapered_barrel_test_meshes::a_wall_face(part),
        BEND_THRESHOLD_DEGREES,
    )
    .expect("the wall floods");
    let group = bevel_edge_group_resolver::create_from_face_group(
        &face_group,
        &face_group.boundary_edges()[0],
        features.flat_angle_degrees(),
    )
    .expect("the wall's rim bounds it");

    bevel_edge_group_resolver::resolve(&group, part)
}

/// C# `TheMouths`: the two mouths on the bores' NEAR side, lowest first. Each bore runs clean through
/// the barrel, so the wall's rim carries four mouths; the two at positive x are the pair that shares
/// one flat piece.
pub fn the_mouths(part: &Mesh, resolution: &BevelEdgeGroupResolution) -> Vec<BevelSweptChain> {
    let average = |chain: &BevelSweptChain, read: &dyn Fn(i32) -> f64| {
        chain
            .edges
            .iter()
            .map(|edge| read(edge.from_vertex()))
            .sum::<f64>()
            / chain.edges.len() as f64
    };
    let x = |v: i32| part.vertices[v as usize].x as f64;
    let z = |v: i32| part.vertices[v as usize].z as f64;

    let mut mouths: Vec<BevelSweptChain> = bevel_chain_linker::link(&resolution.edges)
        .into_iter()
        .filter(|chain| chain.is_closed && (chain.edges.len() as i32) < FACETS)
        .filter(|chain| average(chain, &x) > 0.0)
        .collect();

    // C# `OrderBy` is stable; so is `sort_by`.
    mouths.sort_by(|a, b| average(a, &z).total_cmp(&average(b, &z)));

    mouths
}

/// C# `ARadialBore`: one bore, laid on its side and lifted, long enough to pass clean through both
/// walls. It is turned HALF A FACET off zero because the frustum carries a crease at turn zero: bored
/// there, each mouth would straddle that crease and belong to two flat pieces.
fn a_radial_bore(at_height: f64) -> Mesh {
    let mut drill =
        topology_test_meshes::create_cylinder(BORE_RADIUS, 4.0 * BOTTOM_RADIUS, BORE_SIDES);

    drill.transform(
        Matrix4X4::create_rotation_y(math_helper::TAU / 4.0)
            * Matrix4X4::create_rotation_z(math_helper::TAU / FACETS as f64 / 2.0)
            * Matrix4X4::create_translation(0.0, 0.0, at_height),
    );

    drill
}
