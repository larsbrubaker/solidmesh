//! The automatic repair (`Mesh::auto_repaired`, the Repair operation's default path) over the
//! small open meshes a generated or downloaded mesh arrives as: a box missing a side (a hole), a
//! box whose sides each carry their own corners (vertices split at seams, as a file that keeps
//! per-side normals or texture coordinates writes them) and a box with one face flipped. Each one
//! comes back a closed, outward-wound solid of the same shape, which `Mesh::is_closed_consistently_wound`
//! tells apart from the open one. The real generated mesh this protects is checked through the
//! chat's own import in the app's `ai_chat` area.

use polygon_mesh::solids::platonic_solids;
use polygon_mesh::Mesh;
use vector_math::Vector3Float;

use super::solid_checks::assert_closed_outward;

/// The 10 mm box the cases start from, centred on the origin.
fn box_mesh() -> Mesh {
    platonic_solids::create_cube(10.0, 10.0, 10.0)
}

/// A mesh of `faces` over `vertices`.
fn mesh_of(vertices: Vec<Vector3Float>, faces: &[[i32; 3]]) -> Mesh {
    let mut mesh = Mesh::new();
    mesh.vertices = vertices;
    for face in faces {
        mesh.faces.add(face[0], face[1], face[2], &mesh.vertices);
    }
    mesh
}

/// The box's faces as index triples.
fn faces_of(mesh: &Mesh) -> Vec<[i32; 3]> {
    mesh.faces.iter().map(|f| [f.v0, f.v1, f.v2]).collect()
}

/// Asserts `repaired` is a closed outward solid with the shape of the 10 mm box: its volume and
/// bounds, and every corner at one of the box's corners (the repair moved nothing).
fn assert_box_shape(repaired: &Mesh, what: &str) {
    assert!(
        repaired.is_closed_consistently_wound(),
        "{what}: not closed"
    );
    assert_closed_outward(repaired, what);
    let volume = repaired.get_volume();
    assert!((volume - 1000.0).abs() < 1e-9, "{what}: volume {volume}");
    let bounds = repaired.get_axis_aligned_bounding_box();
    for (low, high) in [
        (bounds.min_xyz.x, bounds.max_xyz.x),
        (bounds.min_xyz.y, bounds.max_xyz.y),
        (bounds.min_xyz.z, bounds.max_xyz.z),
    ] {
        assert_eq!((low, high), (-5.0, 5.0), "{what}: bounds");
    }
    for v in &repaired.vertices {
        assert!(
            [v.x, v.y, v.z].iter().all(|c| c.abs() == 5.0),
            "{what}: a corner moved to {v:?}"
        );
    }
}

#[test]
fn a_closed_box_reads_as_closed_and_consistently_wound() {
    let mesh = box_mesh();
    assert!(mesh.is_closed_consistently_wound());
    assert!(!Mesh::new().is_closed_consistently_wound(), "no faces");
}

#[test]
fn a_box_missing_a_side_is_capped_into_the_box() {
    let full = box_mesh();
    // The two triangles of the top (z = 5) side gone: a square hole.
    let open: Vec<[i32; 3]> = faces_of(&full)
        .into_iter()
        .filter(|face| !face.iter().all(|&v| full.vertices[v as usize].z == 5.0))
        .collect();
    assert_eq!(open.len(), 10);
    let mesh = mesh_of(full.vertices.clone(), &open);
    assert!(!mesh.is_closed_consistently_wound());
    assert!(!mesh.is_manifold());

    let repaired = mesh.auto_repaired().expect("the repair runs");
    assert_box_shape(&repaired, "box missing its top");
}

#[test]
fn a_box_split_at_its_seams_is_welded_into_the_box() {
    let full = box_mesh();
    // Every face gets its own three corners: no edge is shared on vertex indices, though every
    // edge meets another at the same positions.
    let mut vertices = Vec::new();
    let mut faces = Vec::new();
    for face in faces_of(&full) {
        let first = vertices.len() as i32;
        for v in face {
            vertices.push(full.vertices[v as usize]);
        }
        faces.push([first, first + 1, first + 2]);
    }
    let mesh = mesh_of(vertices, &faces);
    assert!(!mesh.is_closed_consistently_wound());

    let repaired = mesh.auto_repaired().expect("the repair runs");
    assert_box_shape(&repaired, "box split at its seams");
    assert_eq!(
        repaired.vertices.len(),
        8,
        "welded back to the box's corners"
    );
}

#[test]
fn a_box_with_a_flipped_face_is_turned_back_outward() {
    let full = box_mesh();
    let mut faces = faces_of(&full);
    faces[0].swap(1, 2);
    let mesh = mesh_of(full.vertices.clone(), &faces);
    // Every edge still has two faces, so the face-count check alone can't see the flip.
    assert!(mesh.is_manifold());
    assert!(!mesh.is_closed_consistently_wound());

    let repaired = mesh.auto_repaired().expect("the repair runs");
    assert_box_shape(&repaired, "box with a flipped face");
}
