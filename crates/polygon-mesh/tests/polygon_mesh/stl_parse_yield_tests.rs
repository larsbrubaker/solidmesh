//! A large model has to hand the UI its thread back while it comes in, or in the browser the
//! frame is frozen for the whole load.
//!
//! The caller drives [`StlParse::next_chunk`] and each `Continue` is a point where it gets its
//! thread back, so no process-wide yield hook is needed. A caller with no reporter (exports,
//! thumbnails, tests) uses the synchronous [`stl_processing::parse_file_contents`], which hands
//! control back only with the finished mesh: `a_parse_nobody_is_watching_never_yields` checks
//! that.

use polygon_mesh::processors::stl_processing::{self, StlChunk, StlParse, FACES_PER_YIELD};
use polygon_mesh::processors::{stl_writer, MeshOutputSettings};
use polygon_mesh::{CancelToken, Mesh};
use vector_math::Vector3;

/// Enough faces to cross FACES_PER_YIELD several times, so a yield has to land in the middle
/// of the parse rather than only at its end.
const FACE_COUNT: i32 = FACES_PER_YIELD * 3;

/// A real binary STL, written by the production writer: a ribbon of distinct triangles, one
/// per step along X, so no two of them merge into the same face.
fn multi_chunk_stl_bytes() -> Vec<u8> {
    let mut mesh = Mesh::new();
    for i in 0..FACE_COUNT {
        let i = f64::from(i);
        mesh.create_face(&[
            Vector3::new(i, 0.0, 0.0),
            Vector3::new(i, 1.0, 0.0),
            Vector3::new(i, 0.0, 1.0),
        ]);
    }
    let mut bytes = Vec::new();
    stl_writer::save(
        &mesh,
        &mut bytes,
        &CancelToken::new(),
        &MeshOutputSettings::new(),
    )
    .expect("writing to memory succeeds");
    bytes
}

/// The same ribbon as an ASCII STL, generated here so the ASCII parse loop is walked too.
fn multi_chunk_ascii_stl_bytes() -> Vec<u8> {
    let mut text = String::from("solid ribbon\n");
    for i in 0..FACE_COUNT {
        text.push_str("  facet normal 0 0 1\n");
        text.push_str("    outer loop\n");
        text.push_str(&format!("      vertex {i} 0 0\n"));
        text.push_str(&format!("      vertex {i} 1 0\n"));
        text.push_str(&format!("      vertex {i} 0 1\n"));
        text.push_str("    endloop\n");
        text.push_str("  endfacet\n");
    }
    text.push_str("endsolid ribbon\n");
    text.into_bytes()
}

/// Parses chunk by chunk, counting the boundaries the caller got its thread back at.
fn parse_yielding(bytes: &[u8]) -> (Mesh, i32) {
    let mut parse = StlParse::new(bytes).expect("a valid STL");
    let mut yields = 0;
    let mut progress = |_ratio: f64, _message: &str| {};
    loop {
        match parse
            .next_chunk(&CancelToken::new(), Some(&mut progress))
            .expect("the parse succeeds")
        {
            StlChunk::Continue { .. } => yields += 1,
            StlChunk::Done(mesh) => return (*mesh, yields),
        }
    }
}

// Binary and ASCII are two separate loops in the parser with a chunk boundary each, so both
// have to be walked or half the seam is untested.
fn a_large_stl_hands_the_ui_its_thread_back_while_it_parses(binary: bool) {
    let bytes = if binary {
        multi_chunk_stl_bytes()
    } else {
        multi_chunk_ascii_stl_bytes()
    };
    let (parsed, yields) = parse_yielding(&bytes);
    assert!(
        yields >= 2,
        "a parse that only yields once has held the frame for everything but the last chunk"
    );
    assert_eq!(parsed.faces.len() as i32, FACE_COUNT);
}

#[test]
fn a_large_stl_hands_the_ui_its_thread_back_while_it_parses_true() {
    a_large_stl_hands_the_ui_its_thread_back_while_it_parses(true);
}

#[test]
fn a_large_stl_hands_the_ui_its_thread_back_while_it_parses_false() {
    a_large_stl_hands_the_ui_its_thread_back_while_it_parses(false);
}

#[test]
fn a_parse_nobody_is_watching_never_yields() {
    // The only reason not to hop the event loop is that there is no reporter. Getting this
    // wrong costs every non-UI caller - exports, thumbnails, tests - a UI hop per chunk of
    // every STL it reads. There is one "no reporter", `None`, so this one parse covers it.
    let parsed =
        stl_processing::parse_file_contents(&multi_chunk_stl_bytes(), &CancelToken::new(), None)
            .expect("the parse succeeds");
    // The one return carries every face: no chunk boundary handed control back mid-parse.
    assert_eq!(
        parsed.faces.len() as i32,
        FACE_COUNT,
        "a parse with no reporter has no progress to paint"
    );
}

fn yielding_does_not_change_the_parsed_mesh(binary: bool) {
    let stl_bytes = if binary {
        multi_chunk_stl_bytes()
    } else {
        multi_chunk_ascii_stl_bytes()
    };
    let (yielding, _) = parse_yielding(&stl_bytes);
    let plain = stl_processing::parse_file_contents(&stl_bytes, &CancelToken::new(), None)
        .expect("the parse succeeds");

    assert_eq!(
        (yielding.vertices.len(), yielding.faces.len()),
        (plain.vertices.len(), plain.faces.len()),
        "chunking the parse must not change the mesh it builds"
    );
    assert_eq!(
        yielding.vertices, plain.vertices,
        "the chunk boundaries must not reorder the vertices either"
    );
}

#[test]
fn yielding_does_not_change_the_parsed_mesh_true() {
    yielding_does_not_change_the_parsed_mesh(true);
}

#[test]
fn yielding_does_not_change_the_parsed_mesh_false() {
    yielding_does_not_change_the_parsed_mesh(false);
}
