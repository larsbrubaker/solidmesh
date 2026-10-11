// Ports agg-sharp `Tests/Agg.Tests/Other/InvariantCultureTests.cs`, its ASCII STL case: the
// writer puts '.' decimals in the text and the reader reads them back. C# runs it under de-DE;
// Rust formats and parses numbers with no culture at all, so the test is the same check without
// the culture scope. The vector parse is in vector-math's tests/other, the packed polygon string
// in agg-path's tests/other.

use polygon_mesh::processors::stl_processing;
use polygon_mesh::processors::{stl_writer, MeshOutputSettings, OutputType};
use polygon_mesh::{CancelToken, Mesh};
use vector_math::Vector3;

#[test]
fn ascii_stl_is_written_with_point_decimals() {
    let mut mesh = Mesh::new();
    mesh.create_face(&[
        Vector3::new(0.0, 0.0, 1.5),
        Vector3::new(10.0, 0.0, 1.5),
        Vector3::new(5.0, 5.0, 1.5),
    ]);

    let mut stream = Vec::new();
    let settings = MeshOutputSettings::with_output_type(OutputType::Ascii, &[], None);
    stl_writer::save(&mesh, &mut stream, &CancelToken::new(), &settings).expect("saves");
    let text = String::from_utf8(stream.clone()).expect("ASCII text");

    assert!(text.contains("vertex 0.000000 0.000000 1.500000"));

    let loaded = stl_processing::load(stream.as_slice(), &CancelToken::new(), None).expect("loads");
    assert_eq!(loaded.vertices[0].z, 1.5f32);
}
