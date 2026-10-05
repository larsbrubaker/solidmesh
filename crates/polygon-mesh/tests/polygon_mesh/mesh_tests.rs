//! Port of `Agg.PolygonMesh/MeshTests.cs`: the cases that need only the Mesh core and STL.
//! The rest (SDF, slicing/stitching, face splitting, BSP trees, dual contouring, T-junctions)
//! wait for their Phase 2 slices and are listed in `docs/PORTING_PLAN.md`.

use polygon_mesh::processors::stl_processing;
use polygon_mesh::CancelToken;

#[test]
fn ascii_stl_open_scad_format_parses_to_faces() {
    // Tetrahedron in the exact textual style produced by OpenSCAD: CRLF endings, two-space
    // facet indent, bare integer vertex coordinates.
    let lines = [
        "solid OpenSCAD_Model",
        "  facet normal 0 0 -1",
        "    outer loop",
        "      vertex 0 0 0",
        "      vertex 0 10 0",
        "      vertex 10 0 0",
        "    endloop",
        "  endfacet",
        "  facet normal -1 0 0",
        "    outer loop",
        "      vertex 0 0 0",
        "      vertex 0 0 10",
        "      vertex 0 10 0",
        "    endloop",
        "  endfacet",
        "  facet normal 0 -1 0",
        "    outer loop",
        "      vertex 0 0 0",
        "      vertex 10 0 0",
        "      vertex 0 0 10",
        "    endloop",
        "  endfacet",
        "  facet normal 1 1 1",
        "    outer loop",
        "      vertex 10 0 0",
        "      vertex 0 10 0",
        "      vertex 0 0 10",
        "    endloop",
        "  endfacet",
        "endsolid OpenSCAD_Model",
    ];
    let ascii_stl = lines.join("\r\n") + "\r\n";

    let mesh = stl_processing::load(ascii_stl.as_bytes(), &CancelToken::new(), None)
        .expect("the OpenSCAD STL parses");
    assert_eq!(mesh.faces.len(), 4);
}
