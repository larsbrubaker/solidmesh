//! Test fixture: a 48-facet tapered barrel (16 at the bottom to 12 at the top, 30 tall) with two
//! radial bores through its wall at different heights and a quarter turn apart, and the wall picked
//! as the app picks it. Laid out with the exact Pi, and once with Pi rounded to a float
//! (`the_part_as_float_pi_laid_it_out`), the raw boolean the working-mesh prologue's own test
//! reads.

#![allow(dead_code)]

use polygon_mesh::csg::boolean_processing::do_array;
use polygon_mesh::csg::{BooleanOptions, CsgModes};
use polygon_mesh::{CancelToken, Mesh};
use vector_math::{math_helper, Matrix4X4, Vector2, Vector3, Vector3Float};

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

/// The barrel with both bores through it.
pub fn the_part() -> Mesh {
    the_boolean(std::f64::consts::PI)
}

/// Pi rounded to a float (`3.1415927`), 8.7e-8 over: enough to leave a revolve's steps a hair long.
pub const FLOAT_PI: f64 = std::f32::consts::PI as f64;

/// [`the_part`] laid out with Pi as [`FLOAT_PI`] - every revolve step and the quarter turn that
/// lays each drill on its side a hair long, so the boolean cut the wall's folds twice a fraction of
/// a micron apart and left DOUBLED fold vertices and zero-area slivers: the raw output the
/// working-mesh prologue exists to clean up. Only the prologue's own test reads this.
pub fn the_part_as_float_pi_laid_it_out() -> Mesh {
    the_boolean(FLOAT_PI)
}

/// The barrel and both bores laid out with `pi`, the bores subtracted.
fn the_boolean(pi: f64) -> Mesh {
    let barrel = revolved_with(
        pi,
        topology_test_meshes::create_frustum(BOTTOM_RADIUS, TOP_RADIUS, HEIGHT, FACETS),
    );
    let upper = a_radial_bore(0.0, 8.0, pi);
    let lower = a_radial_bore(90.0, -8.0, pi);

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

/// A mesh revolved about z moved to where a revolve under `pi` would have put
/// it - every vertex's angle round the axis scaled by `pi / PI`, as each step of 2 pi / n was.
fn revolved_with(pi: f64, mut revolved: Mesh) -> Mesh {
    if pi == std::f64::consts::PI {
        return revolved;
    }

    for i in 0..revolved.vertices.len() {
        let at = Vector3::from_vector3_float(revolved.vertices[i]);
        let radius = (at.x * at.x + at.y * at.y).sqrt();
        let mut angle = at.y.atan2(at.x);

        // The revolve's steps run 0 to 2 pi; atan2 hands the upper half back negative.
        if angle < -1e-9 {
            angle += 2.0 * std::f64::consts::PI;
        }

        angle *= pi / std::f64::consts::PI;
        revolved.vertices[i] = Vector3Float::new(radius * angle.cos(), radius * angle.sin(), at.z);
    }

    revolved.mark_as_changed();
    revolved.calculate_normals();

    revolved
}

/// [`the_part`] through the working-mesh prologue, the only form any bevel
/// ever reads.
pub fn the_working_part() -> Mesh {
    create_from(&the_part(), &mut StampedVerdict::default())
}

/// The barrel with NO bores.
pub fn the_blank() -> Mesh {
    topology_test_meshes::create_frustum(BOTTOM_RADIUS, TOP_RADIUS, HEIGHT, FACETS)
}

/// The wall as the app picks it - the smooth region flooded off one of its facets,
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

/// A face of the tapered WALL, read off the mesh - its normal leans off z by the
/// taper's own angle and points away from the axis.
pub fn a_wall_face(part: &Mesh) -> i32 {
    (0..part.faces.len())
        .find(|&i| {
            let normal = Vector3::from_vector3_float(part.faces[i].normal).get_normal();
            (normal.z - the_walls_normal_z()).abs() < 1e-3 && away_from_the_axis(part, i) > 0.9
        })
        .expect("the barrel has a wall face") as i32
}

/// The z every facet of the tapered wall's outward normal carries.
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

/// A cylinder laid on its side, long enough to pass through both walls, turned
/// `turn_degrees` about the axis and lifted to `at_height`, laid out with `pi`.
fn a_radial_bore(turn_degrees: f64, at_height: f64, pi: f64) -> Mesh {
    let mut drill = revolved_with(
        pi,
        topology_test_meshes::create_cylinder(BORE_RADIUS, 4.0 * BOTTOM_RADIUS, BORE_SIDES),
    );

    // The quarter turn is in the layout's own Pi; the turn about the
    // axis is in degrees, converted with the exact Pi.
    drill.transform(
        Matrix4X4::create_rotation_y(pi / 2.0)
            * Matrix4X4::create_rotation_z(math_helper::degrees_to_radians(turn_degrees))
            * Matrix4X4::create_translation(0.0, 0.0, at_height),
    );

    drill
}
