//! Port of the test fixture `Fixtures/TaperedBarrelTestMeshes.cs`: a 48-facet tapered barrel (16
//! at the bottom to 12 at the top, 30 tall) with two radial bores through its wall at different
//! heights and a quarter turn apart, and the wall picked as the app picks it. Laid out with the
//! exact Pi only: the `FloatPi` layout (`ThePartAsFloatPiLaidItOut`) only the working-mesh
//! prologue's own test reads, and it lands with that test.

#![allow(dead_code)]

use polygon_mesh::csg::boolean_processing::do_array;
use polygon_mesh::csg::{BooleanOptions, CsgModes};
use polygon_mesh::{CancelToken, Mesh};
use vector_math::{math_helper, Matrix4X4, Vector2, Vector3};

use geometry_processing::bevel::bevel_working_mesh::{create_from, StampedVerdict};
use geometry_processing::topology::{
    bevel_edge_group_resolver, BevelEdgeGroupResolution, FaceGroup, FeatureEdgeSet,
};

use crate::topology_test_meshes;

pub const FACETS: i32 = 48;
pub const TOP_RADIUS: f64 = 12.0;
pub const BOTTOM_RADIUS: f64 = 16.0;
pub const HEIGHT: f64 = 30.0;
pub const BORE_RADIUS: f64 = 1.5;
pub const BORE_SIDES: i32 = 24;
pub const SIZE: f64 = 0.5;
pub const SEGMENTS: i32 = 8;
pub const BEND_THRESHOLD_DEGREES: f64 = 10.0;

/// C# `ThePart`: the barrel with both bores through it.
pub fn the_part() -> Mesh {
    let barrel = topology_test_meshes::create_frustum(BOTTOM_RADIUS, TOP_RADIUS, HEIGHT, FACETS);
    let upper = a_radial_bore(0.0, 8.0);
    let lower = a_radial_bore(90.0, -8.0);

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

/// C# `TheWorkingPart`: [`the_part`] through the working-mesh prologue, the only form any bevel
/// ever reads.
pub fn the_working_part() -> Mesh {
    create_from(&the_part(), &mut StampedVerdict::default())
}

/// C# `TheBlank`: the barrel with NO bores.
pub fn the_blank() -> Mesh {
    topology_test_meshes::create_frustum(BOTTOM_RADIUS, TOP_RADIUS, HEIGHT, FACETS)
}

/// C# `TheWall`: the wall as the app picks it - the smooth region flooded off one of its facets,
/// the group anchored on one of that region's boundary edges.
pub fn the_wall(part: &Mesh) -> BevelEdgeGroupResolution {
    let graph = part.get_mesh_edge_graph();
    let features = FeatureEdgeSet::create_with_graph(part, &graph, BEND_THRESHOLD_DEGREES);
    let face_group =
        FaceGroup::create_with_graph(part, &graph, a_wall_face(part), BEND_THRESHOLD_DEGREES)
            .expect("the wall floods");
    let group = bevel_edge_group_resolver::create_from_face_group(
        &face_group,
        &face_group.boundary_edges()[0],
        features.flat_angle_degrees(),
    )
    .expect("the wall's rim bounds it");

    bevel_edge_group_resolver::resolve(&group, part)
}

/// C# `AWallFace`: a face of the tapered WALL, read off the mesh - its normal leans off z by the
/// taper's own angle and points away from the axis.
pub fn a_wall_face(part: &Mesh) -> i32 {
    (0..part.faces.len())
        .find(|&i| {
            let normal = Vector3::from_vector3_float(part.faces[i].normal).get_normal();
            (normal.z - the_walls_normal_z()).abs() < 1e-3 && away_from_the_axis(part, i) > 0.9
        })
        .expect("the barrel has a wall face") as i32
}

/// C# `TheWallsNormalZ`: the z every facet of the tapered wall's outward normal carries.
pub fn the_walls_normal_z() -> f64 {
    Vector2::new(HEIGHT, BOTTOM_RADIUS - TOP_RADIUS)
        .get_normal()
        .y
}

/// How much a face's normal points away from the barrel's axis, at its own centre.
fn away_from_the_axis(part: &Mesh, face: usize) -> f64 {
    let f = part.faces[face];
    let at = |v: i32| Vector3::from_vector3_float(part.vertices[v as usize]);
    let centre = (at(f.v0) + at(f.v1) + at(f.v2)) / 3.0;
    let outward = Vector3::new(centre.x, centre.y, 0.0);

    if outward.length() < 1e-6 {
        0.0
    } else {
        outward
            .get_normal()
            .dot(Vector3::from_vector3_float(f.normal).get_normal())
    }
}

/// C# `ARadialBore`: a cylinder laid on its side, long enough to pass through both walls, turned
/// `turn_degrees` about the axis and lifted to `at_height`.
fn a_radial_bore(turn_degrees: f64, at_height: f64) -> Mesh {
    let mut drill =
        topology_test_meshes::create_cylinder(BORE_RADIUS, 4.0 * BOTTOM_RADIUS, BORE_SIDES);

    drill.transform(
        Matrix4X4::create_rotation_y(std::f64::consts::PI / 2.0)
            * Matrix4X4::create_rotation_z(math_helper::degrees_to_radians(turn_degrees))
            * Matrix4X4::create_translation(0.0, 0.0, at_height),
    );

    drill
}
