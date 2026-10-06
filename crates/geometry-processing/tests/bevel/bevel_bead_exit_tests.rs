//! Port of `BevelBeadExitTests.cs`: a concave crease dying on a face it runs out through is
//! bounded by that face's plane; one dying at a corner that closes over it keeps its fill.

use polygon_mesh::csg::boolean_processing::do_array;
use polygon_mesh::csg::{BooleanOptions, CsgModes};
use polygon_mesh::solids::platonic_solids::create_cube;
use polygon_mesh::{CancelToken, Mesh};

use geometry_processing::bevel::bevel_chain_end_cap::ResolveOptions;
use geometry_processing::bevel::{bevel_bead_exit, bevel_chain_linker, BevelChainEnds};
use geometry_processing::topology::{EdgeConvexity, FeatureEdge, FeatureEdgeSet};
use vector_math::math_helper::degrees_to_radians;
use vector_math::{Matrix4X4, Vector3};

const SLAB_TOP: f64 = 10.0;
const WALL_FACE: f64 = 10.0;
const CREASE_ENDS_AT_X: f64 = 15.0;
const OBLIQUE_DEGREES: f64 = 70.0;
const SIZE: f64 = 20.0;
const RADIUS: f64 = 4.0;
const ON_A_PLANE: f64 = 1e-6;
const BOX_ON_THE_PLANE: f64 = 1e-3;

/// C# `ACreaseDyingOnAFaceItRunsOutThroughIsBoundedByIt`.
#[test]
fn a_crease_dying_on_a_face_it_runs_out_through_is_bounded_by_it() {
    let part = create_wall_on_slab(true);
    let (ends, at_the_oblique_end) = foot_crease_ends(&part);
    let exits = if at_the_oblique_end {
        ends.end_exit_planes
    } else {
        ends.start_exit_planes
    }
    .expect("the bead running out through the oblique face is bounded by it");

    assert_eq!(exits.len(), 1, "one face is run out through");
    let plane = &exits[0];
    let expected = oblique_normal();
    assert!(
        plane.plane_normal.dot(expected) > 1.0 - ON_A_PLANE,
        "{:?}",
        plane.plane_normal
    );
    assert!((plane.plane_distance - expected.dot(end_vertex())).abs() <= ON_A_PLANE);

    // And the half space kept is the one with the PART in it.
    let inside = bevel_bead_exit::inside(plane, &part.get_axis_aligned_bounding_box());
    let signed: Vec<f64> = inside
        .vertices
        .iter()
        .map(|&vertex| {
            plane.plane_normal.dot(Vector3::from_vector3_float(vertex)) - plane.plane_distance
        })
        .collect();
    let max = signed.iter().copied().fold(f64::NEG_INFINITY, f64::max);
    let min = signed.iter().copied().fold(f64::INFINITY, f64::min);
    assert!(
        max < BOX_ON_THE_PLANE,
        "the box stands {max} past the plane"
    );
    assert!(min < -2.0 * SIZE, "the box reaches only {}", -min);
}

/// C# `ACreaseDyingAtACornerThatClosesOverItKeepsItsFill`.
#[test]
fn a_crease_dying_at_a_corner_that_closes_over_it_keeps_its_fill() {
    let part = create_wall_on_slab(false);
    let (ends, at_the_oblique_end) = foot_crease_ends(&part);
    let exits = if at_the_oblique_end {
        ends.end_exit_planes
    } else {
        ends.start_exit_planes
    };
    assert!(exits.is_none(), "{exits:?}");
}

fn end_vertex() -> Vector3 {
    Vector3::new(CREASE_ENDS_AT_X, WALL_FACE, SLAB_TOP)
}

fn oblique_normal() -> Vector3 {
    Vector3::new(
        degrees_to_radians(OBLIQUE_DEGREES).cos(),
        degrees_to_radians(OBLIQUE_DEGREES).sin(),
        0.0,
    )
}

fn foot_crease_ends(part: &Mesh) -> (BevelChainEnds, bool) {
    let features = FeatureEdgeSet::create(part);
    let foot: Vec<FeatureEdge> = features
        .edges()
        .iter()
        .copied()
        .filter(|edge| edge.convexity == EdgeConvexity::Concave && on_the_crease(edge))
        .collect();
    assert!(
        !foot.is_empty(),
        "the wall's foot crease is not in this mesh"
    );

    let run = bevel_chain_linker::link(&foot).remove(0);
    let at_the_end = (run.edges[run.edges.len() - 1].to() - end_vertex()).length() < ON_A_PLANE;
    let mut warnings = Vec::new();
    let ends = BevelChainEnds::resolve(
        &run.edges,
        run.is_closed,
        RADIUS,
        Some(&features),
        Some(&foot),
        Some(&mut warnings),
        ResolveOptions::default(),
    );
    (ends, at_the_end)
}

fn on_the_crease(edge: &FeatureEdge) -> bool {
    let on_it =
        |point: Vector3| (point.y - WALL_FACE).abs() < 1e-4 && (point.z - SLAB_TOP).abs() < 1e-4;
    on_it(edge.start) && on_it(edge.end)
}

fn create_wall_on_slab(cut_away: bool) -> Mesh {
    let mut slab = create_cube(SIZE, SIZE, SLAB_TOP);
    slab.translate(Vector3::new(SIZE / 2.0, SIZE / 2.0, SLAB_TOP / 2.0));
    let mut wall = create_cube(SIZE, SIZE - WALL_FACE, SIZE - SLAB_TOP);
    wall.translate(Vector3::new(
        SIZE / 2.0,
        (SIZE + WALL_FACE) / 2.0,
        (SIZE + SLAB_TOP) / 2.0,
    ));

    // The oblique solid: a box turned in plan so its near face passes through the crease's end
    // vertex, leaning OBLIQUE_DEGREES off square to the run.
    let mut oblique = create_cube(200.0, 200.0, 400.0);
    let at = end_vertex() + (oblique_normal() * 100.0);
    oblique.transform(
        Matrix4X4::create_rotation_z(degrees_to_radians(OBLIQUE_DEGREES))
            * Matrix4X4::create_translation(at.x, at.y, at.z),
    );

    let cancel = CancelToken::new();
    let options = BooleanOptions::default();
    let block = do_array(
        &[(&slab, Matrix4X4::IDENTITY), (&wall, Matrix4X4::IDENTITY)],
        CsgModes::Union,
        &cancel,
        &options,
    )
    .expect("the slab and wall union");
    do_array(
        &[
            (&block, Matrix4X4::IDENTITY),
            (&oblique, Matrix4X4::IDENTITY),
        ],
        if cut_away {
            CsgModes::Subtract
        } else {
            CsgModes::Union
        },
        &cancel,
        &options,
    )
    .expect("the oblique cut")
}
