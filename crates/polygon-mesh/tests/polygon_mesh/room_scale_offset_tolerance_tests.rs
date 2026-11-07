//! The two bevelled-extrude cases of room-scale offset tolerance: an extrude's bevel rings are
//! offsets cut by the size of what they offset, so a room-size frame gets no more of a mesh than
//! a printer-bed one, and a desk-size one is the mesh it always was. They need
//! `vertex_source_to_mesh::extrude_bevelled`, so they live with it here; the offset cases are in
//! agg-path's `tests/agg/room_scale_offset_tolerance_tests.rs`.

use agg_path::VertexStorage;
use clipper_library::{ClipperError, JoinType};
use polygon_mesh::processors::vertex_source_to_mesh::extrude_bevelled;
use polygon_mesh::Mesh;

/// Twenty feet, in mm.
const TWENTY_FEET: f64 = 20.0 * 304.8;

/// The printer bed `CurveTolerance.DeskSize` anchors on, in mm.
const BED: f64 = 300.0;

const DESK_BEVELLED_EXTRUDE_CHECKSUM: i64 = -600885690986330816;

/// An extrude's bevel rings: a 20 ft frame, bevelled in proportion, builds no more of a mesh
/// than the same frame 300 mm across.
#[test]
fn a_room_size_bevelled_extrude_is_no_finer_than_a_bed_size_one() -> Result<(), ClipperError> {
    let at_bed = bevelled_extrude(BED)?.vertices.len();
    let at_room = bevelled_extrude(TWENTY_FEET)?.vertices.len();

    println!("Bevelled frame 300 mm: {at_bed} vertices; 20 ft: {at_room}");

    assert!(
        at_room as f64 <= at_bed as f64 * 1.05,
        "a 20 ft bevelled frame built {at_room} vertices where a 300 mm one builds {at_bed}"
    );
    Ok(())
}

/// A 50 mm bevelled extrude is the mesh it always was.
#[test]
fn a_desk_size_bevelled_extrude_is_bit_identical() -> Result<(), ClipperError> {
    let mesh = bevelled_extrude(50.0)?;
    let checksum = checksum(&mesh);
    println!(
        "Bevelled frame 50 mm: {} vertices, checksum {checksum}",
        mesh.vertices.len()
    );

    assert_eq!(checksum, DESK_BEVELLED_EXTRUDE_CHECKSUM);
    Ok(())
}

/// A frame `size` mm across bevelled over a tenth of its size.
fn bevelled_extrude(size: f64) -> Result<Mesh, ClipperError> {
    let radius = size / 10.0;
    let height = size / 5.0;
    let bevel = [
        (height - radius, -radius * 0.1),
        (height - radius * 0.5, -radius * 0.5),
        (height, -radius),
    ];

    extrude_bevelled(
        &frame(size),
        height,
        Some(&bevel),
        JoinType::Round,
        None,
        JoinType::Round,
    )
}

/// A square `size` across with a square hole half its size.
fn frame(size: f64) -> VertexStorage {
    let outer = size / 2.0;
    let inner = size / 4.0;
    let mut frame = VertexStorage::new();
    frame.move_to(-outer, -outer);
    frame.line_to(outer, -outer);
    frame.line_to(outer, outer);
    frame.line_to(-outer, outer);
    frame.close_polygon();
    frame.move_to(-inner, -inner);
    frame.line_to(-inner, inner);
    frame.line_to(inner, inner);
    frame.line_to(inner, -inner);
    frame.close_polygon();

    frame
}

/// The exact bits of every vertex position, in sorted order, plus the face count (sorted because
/// the vertex ORDER is not stable; the positions are what a stored mesh keeps).
fn checksum(mesh: &Mesh) -> i64 {
    let mut vertices: Vec<[f64; 3]> = mesh
        .vertices
        .iter()
        .map(|v| [f64::from(v.x), f64::from(v.y), f64::from(v.z)])
        .collect();
    vertices.sort_by(|a, b| {
        a[0].total_cmp(&b[0])
            .then(a[1].total_cmp(&b[1]))
            .then(a[2].total_cmp(&b[2]))
    });

    let mut checksum = mesh.faces.len() as i64;
    for vertex in vertices {
        for coordinate in vertex {
            checksum = checksum
                .wrapping_mul(31)
                .wrapping_add(coordinate.to_bits() as i64);
        }
    }

    checksum
}
