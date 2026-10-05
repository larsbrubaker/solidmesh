//! Ports `Tests/GeometryProcessingTests/BevelSourceHullTests.cs` 1:1: the part's padded hull
//! (a cube's is the cube pushed out by the pad, a tetrahedron's is the tetrahedron and not its box,
//! a concave L's swallows the notch) and the bound it puts on a bead clip (a bead whose void runs
//! past the end of the part is capped at it; one well inside is unchanged).

use geometry_processing::bevel::bevel_bead_clip::{self, Clippable};
use geometry_processing::bevel::bevel_source_hull::{self, PAD_FRACTION};
use geometry_processing::bevel::bevel_working_mesh::signed_volume;
use geometry_processing::bevel::BevelAdvisory;
use geometry_processing::bevel::BevelRebuildAdvisories;
use polygon_mesh::csg::boolean_processing::do_array;
use polygon_mesh::csg::{BooleanError, BooleanOptions, CsgModes};
use polygon_mesh::solids::platonic_solids::create_cube;
use polygon_mesh::{CancelToken, Mesh};
use vector_math::{Matrix4X4, Vector3, Vector3Float};

const CUBE_SIDE: f64 = 20.0;

fn pad() -> f64 {
    PAD_FRACTION * (CUBE_SIDE * 3f64.sqrt())
}

#[test]
fn a_cubes_hull_is_the_cube_itself_pushed_out_by_the_pad() {
    let cube = create_cube(CUBE_SIDE, CUBE_SIDE, CUBE_SIDE);
    let hull = bevel_source_hull::build(Some(&cube)).expect("a cube has a hull");

    assert!(
        hull.is_manifold(),
        "the hull is handed to an intersect as an operand, and an operand with no inside is not a bound at all"
    );

    let bounds = hull.get_axis_aligned_bounding_box();
    for (name, measured) in [
        ("min x", -bounds.min_xyz.x),
        ("min y", -bounds.min_xyz.y),
        ("min z", -bounds.min_xyz.z),
        ("max x", bounds.max_xyz.x),
        ("max y", bounds.max_xyz.y),
        ("max z", bounds.max_xyz.z),
    ] {
        assert!(
            measured > CUBE_SIDE / 2.0,
            "the hull's {name} is at {measured}, inside the cube's own {} - a bound that cuts the part cuts the beads with it",
            CUBE_SIDE / 2.0
        );
        assert!(
            measured <= (CUBE_SIDE / 2.0) + pad() + 1e-9,
            "the hull's {name} is at {measured}, further than {} outside the cube",
            pad()
        );
    }

    assert!(
        (signed_volume(&hull) - CUBE_SIDE * CUBE_SIDE * CUBE_SIDE).abs() <= 1.0,
        "a cube is its own hull, so the pad is the only volume between them"
    );
}

#[test]
fn a_tetrahedrons_hull_is_the_tetrahedron_and_not_its_box() {
    // The four alternate corners of a 20 mm cube: a regular tetrahedron of a third its volume.
    let corners = [
        Vector3::new(10.0, 10.0, 10.0),
        Vector3::new(10.0, -10.0, -10.0),
        Vector3::new(-10.0, 10.0, -10.0),
        Vector3::new(-10.0, -10.0, 10.0),
    ];

    let mut tetrahedron = Mesh::new();
    for corner in corners {
        tetrahedron
            .vertices
            .push(Vector3Float::from_vector3(corner));
    }
    for (a, b, c) in [(0, 1, 2), (0, 2, 3), (0, 3, 1), (1, 3, 2)] {
        let vertices = tetrahedron.vertices.clone();
        tetrahedron.faces.add(a, b, c, &vertices);
    }

    let hull = bevel_source_hull::build(Some(&tetrahedron)).expect("a tetrahedron has a hull");
    let measured = signed_volume(&hull);
    assert!(
        (measured - CUBE_SIDE * CUBE_SIDE * CUBE_SIDE / 3.0).abs() <= 2.0,
        "the hull of four alternate cube corners is a third of the cube, not the whole of it - measured {measured}"
    );
}

#[test]
fn a_concave_ls_hull_swallows_the_notch_and_still_contains_the_part() {
    let l = concave_l();
    let hull = bevel_source_hull::build(Some(&l)).expect("an L has a hull");

    let solid = signed_volume(&l);
    let swallowed = signed_volume(&hull);

    assert!(
        (solid - 6000.0).abs() <= 1.0,
        "the L measures 6000 - measured {solid}"
    );
    assert!(
        (swallowed - 7000.0).abs() <= 20.0,
        "the hull closes the notch with one plane across its mouth, which is half of the notch's 2000 - measured {swallowed}"
    );

    let bounds = hull.get_axis_aligned_bounding_box();
    for vertex in &l.vertices {
        let (x, y, z) = (
            f64::from(vertex.x),
            f64::from(vertex.y),
            f64::from(vertex.z),
        );
        let inside = x >= bounds.min_xyz.x
            && x <= bounds.max_xyz.x
            && y >= bounds.min_xyz.y
            && y <= bounds.max_xyz.y
            && z >= bounds.min_xyz.z
            && z <= bounds.max_xyz.z;
        assert!(
            inside,
            "({x}, {y}, {z}) is a vertex of the part and has to be inside its own hull"
        );
    }
}

#[test]
fn a_bead_whose_void_runs_past_the_end_of_the_part_is_capped_at_it() {
    let part = create_cube(CUBE_SIDE, CUBE_SIDE, CUBE_SIDE);

    // Half in the part and half in open air past its x = 10 face.
    let bead = a_box(5.0, -3.0, -3.0, 25.0, 3.0, 3.0);
    let wedge = a_box(4.0, -4.0, -4.0, 26.0, 4.0, 4.0);

    let clipped = clip(&part, bead, wedge);
    assert_eq!(
        clipped.len(),
        1,
        "the bead had a keep-set to be trimmed to, so it comes back trimmed rather than whole"
    );

    let furthest = clipped[0]
        .vertices
        .iter()
        .map(|v| f64::from(v.x))
        .fold(f64::NEG_INFINITY, f64::max);
    assert!(
        furthest <= (CUBE_SIDE / 2.0) + pad() + 1e-6,
        "the part ends at x = {} and the bead's own keep-set said it could go to 25. The furthest it goes is {furthest}",
        CUBE_SIDE / 2.0
    );
    assert!(
        furthest > (CUBE_SIDE / 2.0) - 0.01,
        "and it has to still REACH the face it dies on. The furthest it goes is {furthest}"
    );
}

#[test]
fn a_bead_well_inside_the_part_is_unchanged_by_the_bound() {
    let part = create_cube(CUBE_SIDE, CUBE_SIDE, CUBE_SIDE);

    let bead = a_box(-3.0, -3.0, -3.0, 3.0, 3.0, 3.0);
    let wedge = a_box(-4.0, -4.0, -4.0, 4.0, 4.0, 4.0);

    let clipped = clip(&part, bead, wedge);
    assert_eq!(clipped.len(), 1);

    let measured = signed_volume(&clipped[0]);
    assert!(
        (measured - 6.0 * 6.0 * 6.0).abs() <= 1e-3,
        "the bead lies wholly inside both its keep-set and the part's hull, so the clip hands back the whole of it. Measured {measured}"
    );
}

fn clip(part: &Mesh, bead: Mesh, wedge: Mesh) -> Vec<Mesh> {
    let combine = |meshes: &[(Mesh, Matrix4X4)], mode: CsgModes| -> Result<Mesh, BooleanError> {
        let operands: Vec<(&Mesh, Matrix4X4)> = meshes.iter().map(|(m, x)| (m, *x)).collect();
        do_array(
            &operands,
            mode,
            &CancelToken::new(),
            &BooleanOptions::default(),
        )
    };

    let mut advisories = BevelRebuildAdvisories::new(None::<Vec<BevelAdvisory>>);
    let mut whole = Vec::new();
    let (clipped, _) = bevel_bead_clip::apply_all(
        part,
        Matrix4X4::IDENTITY,
        &[Clippable::new(bead, wedge, None, None, None)],
        &mut whole,
        &mut advisories,
        &combine,
    )
    .expect("not cancelled");

    clipped
}

fn a_box(min_x: f64, min_y: f64, min_z: f64, max_x: f64, max_y: f64, max_z: f64) -> Mesh {
    let mut cube = create_cube(max_x - min_x, max_y - min_y, max_z - min_z);
    cube.transform(Matrix4X4::create_translation(
        (min_x + max_x) / 2.0,
        (min_y + max_y) / 2.0,
        (min_z + max_z) / 2.0,
    ));
    cube
}

fn concave_l() -> Mesh {
    let block = create_cube(CUBE_SIDE, CUBE_SIDE, CUBE_SIDE);
    let notch = a_box(
        0.0,
        0.0,
        -CUBE_SIDE,
        CUBE_SIDE / 2.0,
        CUBE_SIDE / 2.0,
        CUBE_SIDE,
    );

    do_array(
        &[(&block, Matrix4X4::IDENTITY), (&notch, Matrix4X4::IDENTITY)],
        CsgModes::Subtract,
        &CancelToken::new(),
        &BooleanOptions::default(),
    )
    .expect("the notch subtracts")
}
