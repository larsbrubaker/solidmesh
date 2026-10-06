//! Port of `BevelTVertexSliverRepairTests.cs`: the T-VERTEX SLIVER repair - a zero-area triangle
//! whose three corners are collinear, stitching a vertex that sits part way ALONG a longer edge
//! into the surface, which is what a boolean leaves where one solid's facet met another's edge.
//! Invisible to every measure of the part, but a face with no plane to face out of; the cure drops
//! the sliver and splits the face across its long edge at the middle vertex, on the working copy,
//! with an advisory.

use polygon_mesh::solids::platonic_solids;
use polygon_mesh::{Face, Mesh};
use vector_math::{Vector3, Vector3Float};

use geometry_processing::bevel::bevel_working_mesh::{
    create_from_with_advisories, seam_tolerance_for, signed_volume, StampedVerdict,
};
use geometry_processing::bevel::surface::surface_mesh_closure;
use geometry_processing::bevel::{BevelAdvisory, BevelAdvisoryCode};

/// How far the T-vertex stands off the line it is supposed to be on - the corpus's own slivers
/// carry about this much, because the boolean that made them stored its answer as float.
const OUT_OF_LINE_NOISE: f64 = 1.5e-6;

const CUBE_SIZE: f64 = 10.0;

fn create_from(source: &Mesh) -> (Mesh, Vec<BevelAdvisory>) {
    create_from_with_advisories(source, &mut StampedVerdict::default())
}

#[test]
fn a_collinear_triangle_stitching_a_t_vertex_is_dropped_and_the_face_across_is_split() {
    let source = a_cube_with_a_t_vertex_sliver_on_its_floor();

    let face_count = source.faces.len();
    let vertex_count = source.vertices.len();

    // Stated first: the defect is invisible to every measure of the part.
    assert_eq!(
        surface_mesh_closure::open_edge_count(&source).0,
        0,
        "the sliver covers the two halves of the split edge, so the source reads as closed"
    );
    assert!(
        the_thinnest_face_in(&source) < seam_tolerance_for(&source),
        "the sliver is a face with no plane to face out of, and that is the whole defect"
    );

    let (working, advisories) = create_from(&source);

    let (open, open_at) = surface_mesh_closure::open_edge_count(&working);
    assert_eq!(
        open, 0,
        "dropping a face that covered nothing may not open the part - instead open at {open_at:?}"
    );
    assert!(
        working.is_manifold(),
        "the split puts the middle vertex into the face across, so every edge has two faces"
    );
    assert!(
        the_thinnest_face_in(&working) > seam_tolerance_for(&working),
        "no face of the repaired copy is thinner than the distance two vertices merge at"
    );
    assert_eq!(
        working.vertices.len(),
        vertex_count,
        "the T-vertex is put INTO the face across, not taken out of the part"
    );
    assert_eq!(
        working.faces.len(),
        face_count,
        "one sliver goes and one face becomes two, so the count comes out where it started"
    );

    let middle = vertex_at(&working, Vector3::new(CUBE_SIZE / 2.0, 0.0, 0.0));
    assert_ne!(middle, -1, "the T-vertex is still in the mesh");

    let across = faces_using(
        &working,
        vertex_at(&working, Vector3::new(0.0, 0.0, 0.0)),
        vertex_at(&working, Vector3::new(CUBE_SIZE, 0.0, 0.0)),
    );
    assert!(
        across.is_empty(),
        "the long edge the sliver stitched is gone - it is two edges now, meeting at the T-vertex"
    );
    assert_eq!(
        faces_containing(&working, middle).len(),
        4,
        "the two wall facets that always had the T-vertex, plus the two halves of the face across"
    );
    assert!(
        (signed_volume(&working) - CUBE_SIZE * CUBE_SIZE * CUBE_SIZE).abs() <= 1e-3,
        "a zero-area face encloses nothing, so removing it may not move the solid"
    );
    assert_eq!(
        advisories.len(),
        1,
        "a face the file carried has been dropped, and the user is told so"
    );
    assert_eq!(
        advisories[0],
        BevelAdvisory::new(
            BevelAdvisoryCode::SourceMeshHadTVertexSlivers,
            vec![1.into()]
        ),
        "one sliver went, and the sentence counts them"
    );
}

/// NESTED T's: one long edge carrying two vertices the face across knows nothing about, stitched
/// by two slivers of which the first's own long edge is the second.
#[test]
fn two_slivers_nested_on_one_edge_both_land_in_the_face_across() {
    let source = a_cube_with_nested_t_vertex_slivers_on_its_floor();

    assert_eq!(
        surface_mesh_closure::open_edge_count(&source).0,
        0,
        "the two slivers cover between them every edge the wall's own subdivision made"
    );

    let face_count = source.faces.len();
    let vertex_count = source.vertices.len();

    let (working, advisories) = create_from(&source);

    assert_it_came_back_sound(&working, face_count, vertex_count, &advisories, 2);

    let first = vertex_at(&working, Vector3::new(4.0, 0.0, 0.0));
    let second = vertex_at(&working, Vector3::new(7.0, 0.0, 0.0));

    assert!(
        faces_using(
            &working,
            vertex_at(&working, Vector3::new(0.0, 0.0, 0.0)),
            vertex_at(&working, Vector3::new(CUBE_SIZE, 0.0, 0.0))
        )
        .is_empty(),
        "the long edge is three edges now, meeting at the two vertices that were on it"
    );
    assert_eq!(
        faces_containing(&working, first).len(),
        4,
        "the wall's two facets, and the two halves of the face across that now meet there"
    );
    assert_eq!(
        faces_containing(&working, second).len(),
        4,
        "and the same for the second, which is what taking both in order means"
    );
}

/// TWO SLIVERS FACING EACH OTHER across one long edge: the floor's own subdivision of the edge and
/// the wall's are different, and the only thing covering either is the other sliver.
#[test]
fn two_slivers_facing_each_other_across_one_edge_give_each_side_the_others_vertex() {
    let source = a_cube_whose_floor_and_wall_split_one_edge_differently();

    assert_eq!(
        surface_mesh_closure::open_edge_count(&source).0,
        0,
        "the two slivers cover each other, which is why nothing before now saw a defect"
    );

    let face_count = source.faces.len();
    let vertex_count = source.vertices.len();

    let (working, advisories) = create_from(&source);

    assert_it_came_back_sound(&working, face_count, vertex_count, &advisories, 2);

    let on_the_floor = vertex_at(&working, Vector3::new(4.0, 0.0, 0.0));
    let on_the_wall = vertex_at(&working, Vector3::new(7.0, 0.0, 0.0));

    assert_eq!(
        faces_containing(&working, on_the_floor).len(),
        4,
        "the floor's two facets, and the wall's facet split at the vertex it did not have"
    );
    assert_eq!(
        faces_containing(&working, on_the_wall).len(),
        4,
        "and the wall's two, with the floor's facet split at the vertex IT did not have"
    );
}

/// The readings every repaired copy has to give.
fn assert_it_came_back_sound(
    working: &Mesh,
    face_count: usize,
    vertex_count: usize,
    advisories: &[BevelAdvisory],
    slivers: i32,
) {
    let (open, open_at) = surface_mesh_closure::open_edge_count(working);
    assert_eq!(
        open, 0,
        "dropping faces that covered nothing may not open the part - instead open at {open_at:?}"
    );
    assert!(
        working.is_manifold(),
        "every edge of what is left has to be used by exactly the two faces that meet there"
    );
    assert!(
        the_thinnest_face_in(working) > seam_tolerance_for(working),
        "no face of the repaired copy is thinner than the distance two vertices merge at"
    );
    assert_eq!(
        working.vertices.len(),
        vertex_count,
        "the vertices are put INTO the faces across, not taken out of the part"
    );
    assert_eq!(
        working.faces.len(),
        face_count,
        "each sliver goes and each split adds one, so the count comes out where it started"
    );
    assert!(
        (signed_volume(working) - CUBE_SIZE * CUBE_SIZE * CUBE_SIZE).abs() <= 1e-3,
        "a zero-area face encloses nothing, so removing it may not move the solid"
    );
    assert_eq!(
        advisories.len(),
        1,
        "faces the file carried have been dropped, and the user is told so once"
    );
    assert_eq!(
        advisories[0],
        BevelAdvisory::new(
            BevelAdvisoryCode::SourceMeshHadTVertexSlivers,
            vec![slivers.into()]
        ),
        "and the sentence counts them"
    );
}

/// A part with nothing wrong with it comes through untouched, and says nothing.
#[test]
fn a_cube_with_no_sliver_is_untouched() {
    let cube = platonic_solids::create_cube(CUBE_SIZE, CUBE_SIZE, CUBE_SIZE);
    let face_count = cube.faces.len();

    let (working, advisories) = create_from(&cube);

    assert_eq!(
        working.faces.len(),
        face_count,
        "no face of a cube is collinear, so this repair has no opinion about it"
    );
    assert!(
        (signed_volume(&working) - CUBE_SIZE * CUBE_SIZE * CUBE_SIZE).abs() <= 1e-3,
        "nothing was done to it"
    );
    assert!(
        advisories.is_empty(),
        "nothing was dropped, so there is nothing to report"
    );
}

/// A 10 mm cube whose floor is tiled without the vertex the wall beside it carries, a zero-area
/// triangle stitching the two together.
fn a_cube_with_a_t_vertex_sliver_on_its_floor() -> Mesh {
    let mut mesh = a_cube_without_its_y_zero_wall();

    // 8: the T-vertex on the floor edge at y = 0, the boolean's float noise off its line.
    mesh.vertices
        .push(Vector3Float::new(CUBE_SIZE / 2.0, 0.0, OUT_OF_LINE_NOISE));

    // The floor, which does NOT know about it.
    add(&mut mesh, 0, 2, 1);
    add(&mut mesh, 0, 3, 2);

    // The wall at y = 0, which DOES.
    add(&mut mesh, 0, 8, 5);
    add(&mut mesh, 8, 1, 5);
    add(&mut mesh, 0, 5, 4);

    // And the sliver.
    add(&mut mesh, 0, 1, 8);

    mesh
}

/// TWO vertices on the floor edge the floor knows nothing about, stitched by nested slivers.
fn a_cube_with_nested_t_vertex_slivers_on_its_floor() -> Mesh {
    let mut mesh = a_cube_without_its_y_zero_wall();

    mesh.vertices
        .push(Vector3Float::new(4.0, 0.0, OUT_OF_LINE_NOISE));
    mesh.vertices
        .push(Vector3Float::new(7.0, 0.0, OUT_OF_LINE_NOISE * 0.7));

    add(&mut mesh, 0, 2, 1);
    add(&mut mesh, 0, 3, 2);

    add(&mut mesh, 0, 8, 5);
    add(&mut mesh, 8, 9, 5);
    add(&mut mesh, 9, 1, 5);
    add(&mut mesh, 0, 5, 4);

    // The slivers, the inner one FIRST.
    add(&mut mesh, 0, 9, 8);
    add(&mut mesh, 0, 1, 9);

    mesh
}

/// The floor and the wall each split the edge at y = 0 at a DIFFERENT point.
fn a_cube_whose_floor_and_wall_split_one_edge_differently() -> Mesh {
    let mut mesh = a_cube_without_its_y_zero_wall();

    // 8 is the floor's own extra vertex, 9 the wall's.
    mesh.vertices
        .push(Vector3Float::new(4.0, 0.0, OUT_OF_LINE_NOISE));
    mesh.vertices
        .push(Vector3Float::new(7.0, 0.0, OUT_OF_LINE_NOISE * 0.7));

    add(&mut mesh, 0, 2, 8);
    add(&mut mesh, 8, 2, 1);
    add(&mut mesh, 0, 3, 2);

    add(&mut mesh, 0, 9, 5);
    add(&mut mesh, 9, 1, 5);
    add(&mut mesh, 0, 5, 4);

    add(&mut mesh, 0, 8, 1);
    add(&mut mesh, 0, 1, 9);

    mesh
}

/// The cube's eight corners, its ceiling, and every wall but the one at y = 0.
///
/// `Vector3Float::new` rounds its doubles to float, as C#'s `(float)` casts did.
fn a_cube_without_its_y_zero_wall() -> Mesh {
    let mut mesh = Mesh::new();
    let size = CUBE_SIZE;

    for (x, y, z) in [
        (0.0, 0.0, 0.0),
        (size, 0.0, 0.0),
        (size, size, 0.0),
        (0.0, size, 0.0),
        (0.0, 0.0, size),
        (size, 0.0, size),
        (size, size, size),
        (0.0, size, size),
    ] {
        mesh.vertices.push(Vector3Float::new(x, y, z));
    }

    add(&mut mesh, 4, 5, 6);
    add(&mut mesh, 4, 6, 7);

    add(&mut mesh, 1, 2, 6);
    add(&mut mesh, 1, 6, 5);
    add(&mut mesh, 2, 3, 7);
    add(&mut mesh, 2, 7, 6);
    add(&mut mesh, 3, 0, 4);
    add(&mut mesh, 3, 4, 7);

    mesh
}

fn add(mesh: &mut Mesh, v0: i32, v1: i32, v2: i32) {
    let face = Face::new(v0, v1, v2, &mesh.vertices);
    mesh.faces.push(face);
}

/// The height of the flattest face in the mesh above its own longest edge.
fn the_thinnest_face_in(mesh: &Mesh) -> f64 {
    let mut thinnest = f64::MAX;
    let at = |v: i32| Vector3::from_vector3_float(mesh.vertices[v as usize]);

    for face in mesh.faces.iter() {
        let (p0, p1, p2) = (at(face.v0), at(face.v1), at(face.v2));
        let longest = (p1 - p0)
            .length()
            .max((p2 - p1).length().max((p0 - p2).length()));

        if longest > 0.0 {
            thinnest = thinnest.min((p1 - p0).cross(p2 - p0).length() / longest);
        }
    }

    thinnest
}

/// The index of the vertex at a position, or -1. Matched well inside the weld distance.
fn vertex_at(mesh: &Mesh, position: Vector3) -> i32 {
    mesh.vertices
        .iter()
        .position(|&v| (Vector3::from_vector3_float(v) - position).length() < 1e-4)
        .map_or(-1, |i| i as i32)
}

fn faces_using(mesh: &Mesh, vertex_a: i32, vertex_b: i32) -> Vec<usize> {
    (0..mesh.faces.len())
        .filter(|&f| uses(&mesh.faces[f], vertex_a) && uses(&mesh.faces[f], vertex_b))
        .collect()
}

fn faces_containing(mesh: &Mesh, vertex: i32) -> Vec<usize> {
    (0..mesh.faces.len())
        .filter(|&f| uses(&mesh.faces[f], vertex))
        .collect()
}

fn uses(face: &Face, vertex: i32) -> bool {
    face.v0 == vertex || face.v1 == vertex || face.v2 == vertex
}
