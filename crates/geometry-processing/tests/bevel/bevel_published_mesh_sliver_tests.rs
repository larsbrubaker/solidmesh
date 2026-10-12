//! Port of `R/Tests/GeometryProcessingTests/BevelPublishedMeshSliverTests.cs`: the SLIVER half of
//! the publish-time cleanup (`bevel_published_mesh::clean` through `bevel_sliver_collapse`) -
//! triangles the boolean left lying in a flat face with all but no ALTITUDE, which the
//! weld-and-area pass never touched because their area is small only in proportion to how LONG
//! they are.
//!
//! Measured on "Inner Right Sides and Center.mcx" and "5mm Bevel Around Center.mcx": 24 triangles
//! lying in one corner face's own plane with edges up to 7.8 mm and areas around 7e-5 mm^2 - an
//! altitude of about 2e-5 mm - and two more spanning a whole 22.6 and 28.3 mm face diagonal.
//! Nothing about them is wrong in POSITION; every vertex is on the surface. What is wrong is that a
//! triangle that thin has no reliable normal, so the flat-shaded fill and the outline pass draw
//! them as lines and fans across a face that should read as one flat plane. C#'s "the same
//! instance comes back" is `clean` answering `None`.

use geometry_processing::bevel::{bevel_published_mesh, bevel_sliver_collapse, bevel_working_mesh};
use polygon_mesh::{CancelToken, Mesh};
use vector_math::{Vector3, Vector3Float};

/// The face index of the x = -1 side, whose normal one test corrupts and reads back.
const THE_LEFT_SIDE: usize = 8;

/// The closed 2 mm box, wound outwards, with its TOP left as the two triangles (4, 5, 6) and
/// (4, 6, 7) so that a caller can retriangulate that one face without touching anything else.
fn r#box() -> Mesh {
    let mut r#box = Mesh::new();

    r#box.vertices = vec![
        Vector3Float::new(-1.0, -1.0, -1.0),
        Vector3Float::new(1.0, -1.0, -1.0),
        Vector3Float::new(1.0, 1.0, -1.0),
        Vector3Float::new(-1.0, 1.0, -1.0),
        Vector3Float::new(-1.0, -1.0, 1.0),
        Vector3Float::new(1.0, -1.0, 1.0),
        Vector3Float::new(1.0, 1.0, 1.0),
        Vector3Float::new(-1.0, 1.0, 1.0),
    ];

    for (v0, v1, v2) in [
        (0, 2, 1),
        (0, 3, 2), // the bottom, z = -1
        (4, 5, 6),
        (4, 6, 7), // the top, z = 1
        (0, 1, 5),
        (0, 5, 4), // y = -1
        (2, 3, 7),
        (2, 7, 6), // y = 1
        (0, 4, 7),
        (0, 7, 3), // x = -1, THE_LEFT_SIDE
        (1, 2, 6),
        (1, 6, 5), // x = 1
    ] {
        r#box.faces.add(v0, v1, v2, &r#box.vertices);
    }

    r#box
}

fn signed_volume(mesh: &Mesh) -> f64 {
    let mut total = 0.0;

    for face in mesh.faces.iter() {
        let v0 = Vector3::from_vector3_float(mesh.vertices[face.v0 as usize]);
        let v1 = Vector3::from_vector3_float(mesh.vertices[face.v1 as usize]);
        let v2 = Vector3::from_vector3_float(mesh.vertices[face.v2 as usize]);

        total += v0.dot(v1.cross(v2));
    }

    total / 6.0
}

/// The smallest altitude any face of the mesh has, and where it is, for a failure message.
fn thinnest_face(mesh: &Mesh) -> (f64, String) {
    let mut thinnest = f64::INFINITY;
    let mut r#where = "(no faces)".to_string();

    for face in mesh.faces.iter() {
        let altitude = bevel_sliver_collapse::altitude_of(
            mesh.vertices[face.v0 as usize],
            mesh.vertices[face.v1 as usize],
            mesh.vertices[face.v2 as usize],
        );

        if altitude < thinnest {
            thinnest = altitude;
            r#where = format!(
                "{:?} {:?} {:?}",
                mesh.vertices[face.v0 as usize],
                mesh.vertices[face.v1 as usize],
                mesh.vertices[face.v2 as usize]
            );
        }
    }

    (thinnest, r#where)
}

/// How far the worst stored face normal is from the unit normal its own vertices give.
fn worst_stored_normal_error(mesh: &Mesh) -> f64 {
    let mut worst: f64 = 0.0;

    for face in mesh.faces.iter() {
        let v0 = Vector3::from_vector3_float(mesh.vertices[face.v0 as usize]);
        let geometric = (Vector3::from_vector3_float(mesh.vertices[face.v1 as usize]) - v0)
            .cross(Vector3::from_vector3_float(mesh.vertices[face.v2 as usize]) - v0);

        if geometric.length() == 0.0 {
            continue;
        }

        worst =
            worst.max((Vector3::from_vector3_float(face.normal) - geometric.get_normal()).length());
    }

    worst
}

/// A flat face triangulated with a sliver fan, which is what a boolean leaves where a bead's
/// tangent footprint lands on it: the top's second triangle split about a vertex sitting HALF THE
/// WELD TOLERANCE off its long diagonal, in the plane of the face, so the piece against that
/// diagonal has a full 2.83 mm edge and an altitude of 17 nanometres. A side face that has nothing
/// wrong with it is given a garbage stored normal too, so that the recompute at the end of the
/// cleanup is read as well as the collapse.
// C#'s garbage normal is the literal 0.7071f, not a constant.
#[allow(clippy::approx_constant)]
#[test]
fn a_sliver_in_a_flat_face_is_collapsed_and_every_normal_comes_back_from_the_vertices() {
    let mut shredded = r#box();

    let tolerance = bevel_working_mesh::seam_tolerance_for(&shredded);
    let offset = (tolerance / 2.0 / 2f64.sqrt()) as f32;

    // On the diagonal from corner 4 to corner 6, pushed off it towards corner 7 and staying in
    // z = 1, so the SHAPE is untouched and only the triangulation changes.
    let apex = shredded.vertices.len() as i32;

    shredded
        .vertices
        .push(Vector3Float::new_f32(-offset, offset, 1.0));

    // The top's second triangle (4, 6, 7) fanned about the new vertex.
    shredded.faces.remove(3);
    shredded.faces.add(4, 6, apex, &shredded.vertices);
    shredded.faces.add(6, 7, apex, &shredded.vertices);
    shredded.faces.add(7, 4, apex, &shredded.vertices);

    shredded.faces[THE_LEFT_SIDE].normal = Vector3Float::new_f32(0.5, -0.5, 0.7071);

    let box_volume = signed_volume(&r#box());

    let cleaned = bevel_published_mesh::clean(&shredded, &CancelToken::new())
        .expect("not cancelled")
        .expect(
            "there was a sliver to collapse, so the cleanup had work to do and has to hand back \
             the mesh it did that work on",
        );

    let (thinnest, r#where) = thinnest_face(&cleaned);

    assert!(
        thinnest >= tolerance,
        "no face may be left thinner than the distance at which two of its vertices would have \
         been welded into one. Thinnest is {thinnest:e} at {where}"
    );

    assert!(
        cleaned.is_manifold(),
        "collapsing a sliver splits its long edge in the face on the other side of it, which is \
         the operation that keeps the surface closed"
    );

    let moved = (signed_volume(&cleaned) - box_volume).abs() / box_volume.abs();
    assert!(
        moved < 1e-9,
        "every vertex involved lies in the plane z = 1, so retriangulating around them moves no \
         material at all: {moved}"
    );

    let worst = worst_stored_normal_error(&cleaned);
    assert!(
        worst < 1e-6,
        "the stored normals are what the flat shading and the outline pass read, and the cleanup \
         recomputes every one of them from the vertices it is publishing: {worst}"
    );
}

/// A face with a repeated vertex - no area, no normal, and one of its edges counted twice - is
/// dropped, and the surface it was hung on is still closed.
#[test]
fn a_degenerate_face_is_dropped() {
    let mut with_degenerate = r#box();

    with_degenerate
        .faces
        .add(0, 1, 0, &with_degenerate.vertices);

    let cleaned = bevel_published_mesh::clean(&with_degenerate, &CancelToken::new())
        .expect("not cancelled")
        .unwrap_or_else(|| with_degenerate.copy());

    assert_eq!(
        cleaned.faces.len(),
        r#box().faces.len(),
        "a triangle two of whose corners are the same vertex encloses nothing and can only \
         confuse whatever reads it"
    );

    assert!(
        cleaned.is_manifold(),
        "the box under it was closed and dropping the degenerate face leaves it closed"
    );
}

/// A capped 16 sided tube of radius 2 - the facets a fillet of that radius actually publishes. Its
/// thinnest triangle is three orders of magnitude above the weld tolerance, so the collapse must
/// not touch it, and the whole cleanup must hand the very mesh back.
#[test]
fn a_real_fillets_facets_are_not_touched() {
    let tube = tube(2.0, 10.0, 16);

    let tolerance = bevel_working_mesh::seam_tolerance_for(&tube);
    let (thinnest, _) = thinnest_face(&tube);

    assert!(
        thinnest > tolerance * 1000.0,
        "a fillet's own facets are nowhere near a sliver - this one's thinnest is {thinnest:.4} mm \
         against a weld tolerance of {tolerance:e} - and if that headroom ever closed the collapse \
         would be eating real geometry rather than boolean debris"
    );

    let cleaned = bevel_published_mesh::clean(&tube, &CancelToken::new()).expect("not cancelled");

    assert!(
        cleaned.is_none(),
        "there is nothing to weld, nothing to drop and nothing to collapse in a mesh a sweep \
         built, so the cleanup is a no-op and publishes the boolean's own object"
    );
}

/// A closed tube of the given radius and length along x, fanned flat at both ends.
fn tube(radius: f64, length: f64, segments: i32) -> Mesh {
    let mut tube = Mesh::new();

    for segment in 0..segments {
        let angle = segment as f64 * 2.0 * std::f64::consts::PI / segments as f64;
        let y = (radius * angle.cos()) as f32;
        let z = (radius * angle.sin()) as f32;

        tube.vertices.push(Vector3Float::new_f32(0.0, y, z));
        tube.vertices
            .push(Vector3Float::new_f32(length as f32, y, z));
    }

    let low_centre = tube.vertices.len() as i32;
    let high_centre = low_centre + 1;

    tube.vertices.push(Vector3Float::new(0.0, 0.0, 0.0));
    tube.vertices
        .push(Vector3Float::new_f32(length as f32, 0.0, 0.0));

    for segment in 0..segments {
        let a = segment * 2;
        let b = ((segment + 1) % segments) * 2;

        tube.faces.add(a, b, b + 1, &tube.vertices);
        tube.faces.add(a, b + 1, a + 1, &tube.vertices);
        tube.faces.add(low_centre, b, a, &tube.vertices);
        tube.faces.add(high_centre, a + 1, b + 1, &tube.vertices);
    }

    tube
}
