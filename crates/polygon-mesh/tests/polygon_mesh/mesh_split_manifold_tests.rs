//! Cutting a closed solid with planes (`Mesh::split`, `split_on_planes`, `split_face`) has to
//! leave it closed, and has to mark the mesh as changed. The two faces sharing an edge walk it in
//! opposite directions, so each must compute the plane crossing from the same end: from its own
//! end, in float the two differ by an ulp or two and `clean_and_merge` welds only bit-identical
//! positions.

use polygon_mesh::solids::platonic_solids;
use polygon_mesh::Mesh;
use vector_math::{AxisAlignedBoundingBox, Matrix4X4, Plane, Vector3};

/// `Mesh::split` with its usual arguments: no clip face, clean and merge, keep the faces on the
/// negative side.
fn split(mesh: &mut Mesh, plane: Plane, on_plane_distance: f64) -> bool {
    mesh.split(plane, on_plane_distance, None, true, false)
}

fn angled_normal() -> Vector3 {
    Vector3::new(0.3, 0.5, 0.81).get_normal()
}

#[test]
fn splitting_a_closed_solid_on_an_angled_plane_leaves_it_closed() {
    // An icosahedron has no axis aligned edges, so every crossing lands on an arbitrary
    // fraction of an edge - the case where the two ends disagree.
    let mut mesh = platonic_solids::create_icosahedron(10.0);

    assert!(
        mesh.is_manifold(),
        "the solid under test has to start closed"
    );

    split(&mut mesh, Plane::new(angled_normal(), 1.37), 0.001);

    assert_eq!(boundary_edge_count(&mesh), 0);
    assert!(mesh.is_manifold());
}

#[test]
fn splitting_on_many_serial_planes_leaves_the_solid_closed() {
    let mut mesh = platonic_solids::create_icosahedron(10.0);

    for i in 0..8 {
        split(
            &mut mesh,
            Plane::new(angled_normal(), -3.5 + f64::from(i) * 0.91),
            0.001,
        );
    }

    assert_eq!(boundary_edge_count(&mesh), 0);
    assert!(mesh.is_manifold());
}

/// The pattern the Curve tool needs: slice an elongated part into many strips along x.
#[test]
fn slicing_an_elongated_solid_into_strips_like_the_curve_tool_leaves_it_closed() {
    let mut mesh = elongated_tipped_icosahedron();

    let bounds = mesh.get_axis_aligned_bounding_box();
    const CUT_COUNT: i32 = 40;
    for cut in cuts_across_x(&bounds, CUT_COUNT) {
        split(
            &mut mesh,
            Plane::new(Vector3::UNIT_X, cut),
            bounds.x_size() / f64::from(CUT_COUNT) / 8.0,
        );
    }

    assert_eq!(boundary_edge_count(&mesh), 0);
    assert!(mesh.is_manifold());
}

/// The same slicing through the entry point the Curve tool actually calls.
#[test]
fn split_on_planes_leaves_the_solid_closed() {
    let mut mesh = elongated_tipped_icosahedron();

    let bounds = mesh.get_axis_aligned_bounding_box();
    const CUT_COUNT: i32 = 40;
    mesh.split_on_planes(
        Vector3::UNIT_X,
        &cuts_across_x(&bounds, CUT_COUNT),
        bounds.x_size() / f64::from(CUT_COUNT) / 8.0,
    );

    assert_eq!(boundary_edge_count(&mesh), 0);
    assert!(mesh.is_manifold());
}

/// A face crossed by k planes has to come out as 2k+1 triangles, not O(k^2).
#[test]
fn split_on_planes_triangulates_each_face_once() {
    // A tetrahedron, stretched along x so every one of its 4 faces spans all the cut planes.
    let mut mesh = platonic_solids::create_tetrahedron(10.0);
    mesh.transform(Matrix4X4::create_scale(20.0, 1.0, 1.0) * Matrix4X4::create_rotation_z(0.37));

    let bounds = mesh.get_axis_aligned_bounding_box();
    const CUT_COUNT: i32 = 40;
    let cuts = cuts_across_x(&bounds, CUT_COUNT);
    let source_face_count = mesh.faces.len();

    mesh.split_on_planes(
        Vector3::UNIT_X,
        &cuts,
        bounds.x_size() / f64::from(CUT_COUNT) / 8.0,
    );

    // Each source face is crossed by at most CutCount - 1 planes, so 2k+1 caps out here.
    let max_per_face = (2 * cuts.len()) + 2;
    assert!(mesh.faces.len() <= source_face_count * max_per_face);

    assert_eq!(boundary_edge_count(&mesh), 0);
    assert!(mesh.is_manifold());
}

/// SplitOnPlanes clamps an oversized tolerance to a fraction of the smallest plane gap.
#[test]
fn split_on_planes_clamps_an_oversized_tolerance_and_stays_closed() {
    let mut mesh = elongated_tipped_icosahedron();

    let bounds = mesh.get_axis_aligned_bounding_box();
    const CUT_COUNT: i32 = 40;
    let spacing = bounds.x_size() / f64::from(CUT_COUNT);

    // Twice the plane spacing - far past the point where the slicer used to open up.
    mesh.split_on_planes(
        Vector3::UNIT_X,
        &cuts_across_x(&bounds, CUT_COUNT),
        spacing * 2.0,
    );

    assert_eq!(boundary_edge_count(&mesh), 0);
    assert!(mesh.is_manifold());
}

/// The T junction case: a plane grazing one face's third corner while cleanly crossing the edge
/// it shares with its neighbour still cuts that edge on both faces.
#[test]
fn a_plane_grazing_a_vertex_still_cuts_the_shared_edge_on_both_faces() {
    let mut mesh = Mesh::new();
    // A square pyramid. The apex sits just barely past the cut plane, so the plane grazes it.
    const ON_PLANE_DISTANCE: f64 = 0.05;
    let cut = 4.0;
    let apex = Vector3::new(0.0, 0.0, 10.0);
    let corners = [
        Vector3::new(cut + 2.0, -5.0, 0.0),
        Vector3::new(cut + 2.0, 5.0, 0.0),
        Vector3::new(cut - 6.0, 5.0, 0.0),
        Vector3::new(cut - 6.0, -5.0, 0.0),
    ];

    for i in 0..4 {
        mesh.create_face(&[corners[i], corners[(i + 1) % 4], apex]);
    }

    mesh.create_face(&[corners[0], corners[2], corners[1]]);
    mesh.create_face(&[corners[0], corners[3], corners[2]]);
    mesh.clean_and_merge();

    assert!(
        mesh.is_manifold(),
        "the solid under test has to start closed"
    );

    // The plane sits a hair off the apex's x (which is 0), well inside onPlaneDistance.
    mesh.split_on_planes(Vector3::UNIT_X, &[0.02, cut], ON_PLANE_DISTANCE);

    assert_eq!(boundary_edge_count(&mesh), 0);
    assert!(mesh.is_manifold());
}

/// A mesh with no faces still needs the weld: dropping every face orphans the vertices.
#[test]
fn clean_and_merge_drops_vertices_left_over_when_every_face_is_clipped() {
    let mut mesh = platonic_solids::create_cube(10.0, 10.0, 10.0);

    // Clip against a plane the whole cube is behind - every face is discarded.
    mesh.split(Plane::new(Vector3::UNIT_X, 100.0), 0.001, None, false, true);

    assert_eq!(
        mesh.faces.len(),
        0,
        "the clip plane leaves nothing of the cube"
    );

    mesh.clean_and_merge();

    assert_eq!(
        mesh.vertices.len(),
        0,
        "a mesh with no faces has no vertices worth keeping"
    );
}

/// Split rebuilds Faces and appends to Vertices, so it has to mark the mesh as changed itself.
#[test]
fn split_marks_the_mesh_as_changed_even_when_not_merging() {
    let mut mesh = platonic_solids::create_icosahedron(10.0);

    // Prime the AABB cache so a missed invalidation is a stale bounds.
    let _ = mesh.get_axis_aligned_bounding_box();
    let changed_count_before_split = mesh.changed_count();
    let face_count_before_split = mesh.faces.len();

    mesh.split(Plane::new(Vector3::UNIT_X, 2.5), 0.001, None, false, false);

    assert_ne!(
        mesh.faces.len(),
        face_count_before_split,
        "the plane has to actually cut the solid for this to test anything"
    );

    assert!(
        mesh.changed_count() > changed_count_before_split,
        "Split rebuilt Faces and appended Vertices, so anything caching them is stale"
    );
}

/// The same cache invalidation contract for the single face entry point.
#[test]
fn split_face_marks_the_mesh_as_changed() {
    let mut mesh = platonic_solids::create_icosahedron(10.0);

    let _ = mesh.get_axis_aligned_bounding_box();
    let changed_count_before_split = mesh.changed_count();

    // Take the first face the plane crosses rather than hard coding an index.
    let plane = Plane::new(Vector3::UNIT_X, 2.5);
    let mut did_split = false;
    let mut i = 0;
    while i < mesh.faces.len() && !did_split {
        did_split = mesh.split_face(i, plane, 0.001);
        i += 1;
    }

    assert!(
        did_split,
        "the plane has to actually cross a face for this to test anything"
    );

    assert!(
        mesh.changed_count() > changed_count_before_split,
        "SplitFace rebuilt Faces and appended Vertices"
    );
}

/// A closed solid stretched along x and tipped, so the x cut planes meet its faces at
/// arbitrary angles.
fn elongated_tipped_icosahedron() -> Mesh {
    let mut mesh = platonic_solids::create_icosahedron(10.0);
    mesh.transform(
        Matrix4X4::create_scale(6.0, 1.0, 1.0)
            * Matrix4X4::create_rotation_z(0.37)
            * Matrix4X4::create_rotation_x(0.21),
    );
    mesh
}

fn cuts_across_x(bounds: &AxisAlignedBoundingBox, cut_count: i32) -> Vec<f64> {
    (1..cut_count)
        .map(|i| bounds.min_xyz.x + (bounds.x_size() * f64::from(i) / f64::from(cut_count)))
        .collect()
}

/// How many edges have other than two faces on them - the seam a failed weld leaves.
fn boundary_edge_count(mesh: &Mesh) -> i32 {
    let graph = mesh.get_mesh_edge_graph();
    let mut count = 0;
    for edge_index in 0..graph.edge_count() {
        if graph.get_face_count(edge_index) != 2 {
            count += 1;
        }
    }
    count
}
