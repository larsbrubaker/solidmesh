//! Rust-only STL tests: round trips through the writer and reader, cancellation and the
//! error cases a reader must report instead of returning a mesh.

use solidmesh_mesh::processors::stl_processing::{self, StlError};
use solidmesh_mesh::processors::{stl_writer, MeshOutputSettings, OutputType};
use solidmesh_mesh::solids::platonic_solids;
use solidmesh_mesh::CancelToken;

fn round_trip(output_type: OutputType) {
    let cube = platonic_solids::create_cube(2.0, 3.0, 4.0);
    let settings = MeshOutputSettings::with_output_type(output_type, &[], None);
    let mut bytes = Vec::new();
    stl_writer::save(&cube, &mut bytes, &CancelToken::new(), &settings).unwrap();
    let loaded = stl_processing::parse_file_contents(&bytes, &CancelToken::new(), None).unwrap();
    assert_eq!(loaded.faces.len(), 12);
    assert_eq!(loaded.vertices.len(), 8);
    assert!((loaded.get_volume() - 24.0).abs() < 1e-9);
    assert!(loaded.is_manifold());
}

#[test]
fn rust_only_binary_stl_round_trips_a_cube() {
    round_trip(OutputType::Binary);
}

#[test]
fn rust_only_ascii_stl_round_trips_a_cube() {
    round_trip(OutputType::Ascii);
}

#[test]
fn rust_only_ascii_stl_text_keeps_its_layout() {
    let tetrahedron = platonic_solids::create_tetrahedron(1.0);
    let settings = MeshOutputSettings::with_output_type(OutputType::Ascii, &[], None);
    let mut bytes = Vec::new();
    stl_writer::save(&tetrahedron, &mut bytes, &CancelToken::new(), &settings).unwrap();
    let text = String::from_utf8(bytes).unwrap();
    let lines: Vec<&str> = text.lines().collect();
    assert_eq!(lines[0], "solid Default");
    assert!(lines[1].starts_with("  facet normal "));
    assert_eq!(lines[2], "    outer loop");
    assert_eq!(lines[3], "      vertex 1.000000 0.000000 -0.500000");
    assert_eq!(lines[6], "    endloop");
    assert_eq!(lines[7], "  endfacet");
    assert_eq!(*lines.last().unwrap(), "endsolid Default");
    assert_eq!(lines.len(), 2 + 7 * 4);
}

#[test]
fn rust_only_a_cancelled_load_returns_cancelled() {
    let cube = platonic_solids::create_cube(1.0, 1.0, 1.0);
    let mut bytes = Vec::new();
    stl_writer::save(
        &cube,
        &mut bytes,
        &CancelToken::new(),
        &MeshOutputSettings::new(),
    )
    .unwrap();
    let cancel = CancelToken::new();
    cancel.cancel();
    assert_eq!(
        stl_processing::parse_file_contents(&bytes, &cancel, None).unwrap_err(),
        StlError::Cancelled
    );
}

#[test]
fn rust_only_short_and_truncated_files_are_errors() {
    assert_eq!(
        stl_processing::parse_file_contents(&[0u8; 80], &CancelToken::new(), None).unwrap_err(),
        StlError::TooShort
    );
    let mut header = vec![0u8; 84];
    header[80] = 5; // claims five triangles, has none
    assert_eq!(
        stl_processing::parse_file_contents(&header, &CancelToken::new(), None).unwrap_err(),
        StlError::Truncated
    );
}
