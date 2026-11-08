//! A boolean whose operands nearly touch comes back with no zero-area face once its result is
//! narrowed to the mesh's f32 positions when it asks for single precision
//! (`BooleanOptions::single_precision`): the kernel then never cuts a sliver too thin for f32 to
//! hold. Without it the kernel stays exact, slivers included.
//!
//! The fixture (`tests/fixtures/SinglePrecisionUnion`) is the helix and core of a three-sided,
//! one-turn Threads object used as a hole, whose roots sit within an f32 step of the core. Format:
//! "vertices faces", then one "x y z" per vertex (each an f32 value written as round-trip f64
//! text, so exact), then one "v0 v1 v2" per face.

use solidmesh_math::{Matrix4X4, Vector3, Vector3Float};
use solidmesh_mesh::csg::{boolean_processing, BooleanOptions, CsgModes};
use solidmesh_mesh::{CancelToken, Face, Mesh};

fn load_fixture(name: &str) -> Mesh {
    let path = format!(
        "{}/tests/fixtures/SinglePrecisionUnion/{name}.txt",
        env!("CARGO_MANIFEST_DIR")
    );
    let text = std::fs::read_to_string(&path).unwrap_or_else(|e| panic!("{path}: {e}"));
    let mut lines = text.lines();
    let header: Vec<usize> = lines
        .next()
        .expect("a header line")
        .split_whitespace()
        .map(|n| n.parse().expect("a count"))
        .collect();
    let mut mesh = Mesh::new();
    for line in lines.by_ref().take(header[0]) {
        let p: Vec<f64> = line
            .split_whitespace()
            .map(|n| n.parse().expect("a coordinate"))
            .collect();
        mesh.vertices
            .push(Vector3Float::from_vector3(Vector3::new(p[0], p[1], p[2])));
    }
    for line in lines.take(header[1]) {
        let v: Vec<i32> = line
            .split_whitespace()
            .map(|n| n.parse().expect("an index"))
            .collect();
        let face = Face::new(v[0], v[1], v[2], &mesh.vertices);
        mesh.faces.push(face);
    }
    mesh
}

/// Faces with a repeated corner or a cross product of exactly zero over their f32 corners.
fn zero_area_faces(mesh: &Mesh) -> usize {
    mesh.faces
        .iter()
        .filter(|face| {
            if face.v0 == face.v1 || face.v1 == face.v2 || face.v2 == face.v0 {
                return true;
            }
            let [a, b, c] = [face.v0, face.v1, face.v2].map(|i| mesh.vertices[i as usize]);
            let (ab, ac) = (b - a, c - a);
            let cross = (
                f64::from(ab.y) * f64::from(ac.z) - f64::from(ab.z) * f64::from(ac.y),
                f64::from(ab.z) * f64::from(ac.x) - f64::from(ab.x) * f64::from(ac.z),
                f64::from(ab.x) * f64::from(ac.y) - f64::from(ab.y) * f64::from(ac.x),
            );
            cross == (0.0, 0.0, 0.0)
        })
        .count()
}

fn union(single_precision: bool) -> Mesh {
    let thread = load_fixture("thread");
    let core = load_fixture("core");
    assert_eq!(zero_area_faces(&thread), 0, "the helix going in");
    assert_eq!(zero_area_faces(&core), 0, "the core going in");

    boolean_processing::do_array(
        &[(&thread, Matrix4X4::IDENTITY), (&core, Matrix4X4::IDENTITY)],
        CsgModes::Union,
        &CancelToken::new(),
        &BooleanOptions {
            single_precision,
            ..BooleanOptions::default()
        },
    )
    .expect("the union runs")
}

/// The kernel's exact union of the fixture measures this.
const EXACT_VOLUME: f64 = 49.446954794676;

#[test]
fn union_of_nearly_touching_operands_has_no_zero_area_face() {
    let union = union(true);
    assert_eq!(zero_area_faces(&union), 0, "zero-area faces in the union");
    // The same solid as the kernel's exact union: working to f32 precision merges edges a
    // hundred-millionth of a millimetre long, nothing a user sees.
    let volume = union.get_volume();
    assert!(
        (volume - EXACT_VOLUME).abs() < 1e-6,
        "union volume {volume}"
    );
}

#[test]
fn single_precision_is_opt_in_and_the_default_union_stays_exact() {
    assert!(
        !BooleanOptions::default().single_precision,
        "every boolean that does not ask for single precision keeps the kernel's exact f64"
    );
    let union = union(false);
    // Left exact, the kernel cuts the slivers this fixture was taken for, and they narrow to
    // zero-area faces: the reason the thread builders opt in.
    assert!(
        zero_area_faces(&union) > 0,
        "the exact union of these operands cuts slivers too thin for f32"
    );
    let volume = union.get_volume();
    assert!(
        (volume - EXACT_VOLUME).abs() < 1e-6,
        "union volume {volume}"
    );
}
