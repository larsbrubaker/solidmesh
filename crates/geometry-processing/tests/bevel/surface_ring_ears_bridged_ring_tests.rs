//! Ports `GeometryProcessingTests/SurfaceRingEarsBridgedRingTests.cs`: a ring that has been
//! BRIDGED - a flat piece whose outline was several rings, an outer one holding inner ones,
//! threaded onto one ring by running out to each inner ring and back along the same edge, a zero
//! width slit. The classical ear clipping of a polygon with holes, and what `SurfaceRingEars` has
//! to read without calling the slit a crossing.
//!
//! The ring is weakly simple rather than simple: the slit's two sides lie on top of one another.
//! They are the SAME mesh edge run in opposite directions - the same two pool indices either way
//! round - and that identity, not their positions, is what tells them apart from a ring that
//! genuinely crosses itself.

use std::collections::HashSet;

use geometry_processing::bevel::surface::{surface_ring_ears, SurfaceVertexPool};
use vector_math::{Matrix4X4, Vector3, Vector3Float};

fn pool_of(points: &[Vector3]) -> SurfaceVertexPool {
    let points: Vec<Vector3Float> = points
        .iter()
        .map(|at| Vector3Float::from_vector3(*at))
        .collect();
    SurfaceVertexPool::new(&points)
}

fn signed_area(pool: &SurfaceVertexPool, faces: &[(i32, i32, i32)], normal: Vector3) -> f64 {
    faces
        .iter()
        .map(|face| {
            0.5 * (pool.position_of(face.1) - pool.position_of(face.0))
                .cross(pool.position_of(face.2) - pool.position_of(face.0))
                .dot(normal)
        })
        .sum()
}

/// The bridged square-with-a-square-hole, read flat in z.
#[test]
fn a_square_bridged_to_its_hole_triangulates_to_the_area_between_them() {
    check_the_bridged_square(Matrix4X4::IDENTITY, 1e-12);
}

/// The same ring under a rigid motion, so nothing here is reading an axis.
#[test]
fn the_same_ring_under_a_rotation_reads_the_same() {
    let moved = Matrix4X4::create_rotation_x(0.7)
        * Matrix4X4::create_rotation_y(-0.4)
        * Matrix4X4::create_rotation_z(0.3)
        * Matrix4X4::create_translation(-13.0, 5.0, 41.0);

    // The pool keeps its points as floats, so a ring carried out to 41 mm reads its own area to
    // about a millionth - the ruler the whole surface bevel measures on, not slack in the
    // triangulation.
    check_the_bridged_square(moved, 1e-5);
}

/// A remainder every corner of which is a sliver: a long thin quad, far thinner than the smallest
/// triangle the ear test will cut off (`SurfaceFan.SmallestTriangle`), so the ear walk refuses
/// every corner of it. It is still two triangles.
#[test]
fn a_slit_thin_quad_is_still_two_triangles() {
    let pool = pool_of(&[
        Vector3::new(0.0, 0.0, 0.0),
        Vector3::new(1.0, 0.0, 0.0),
        Vector3::new(1.0, 1e-10, 0.0),
        Vector3::new(0.0, 1e-10, 0.0),
    ]);
    let normal = Vector3::new(0.0, 0.0, 1.0);
    let mut faces = Vec::new();

    assert!(surface_ring_ears::try_triangulate_the_ring(
        &[0, 1, 2, 3],
        &pool,
        &mut faces,
        normal,
        false
    ));

    assert_eq!(faces.len(), 2);

    let area = signed_area(&pool, &faces, normal);

    assert!((area - 1e-10).abs() <= 1e-16);

    // Every directed edge once - which is what says the two triangles cover the quad rather than
    // lying over one another.
    let mut used = HashSet::new();

    for face in &faces {
        for edge in [(face.0, face.1), (face.1, face.2), (face.2, face.0)] {
            assert!(used.insert(edge));
        }
    }

    for edge in [(0, 1), (1, 2), (2, 3), (3, 0)] {
        assert!(used.contains(&edge));
    }
}

/// The other side of that rule: four points collinear to the lattice are a LINE, and a line has
/// no triangulation to publish. Refused by name rather than fanned into noise.
#[test]
fn a_ring_collinear_to_the_lattice_is_still_refused() {
    let pool = pool_of(&[
        Vector3::new(0.0, 0.0, 0.0),
        Vector3::new(1.0, 0.0, 0.0),
        Vector3::new(2.0, 0.0, 0.0),
        Vector3::new(3.0, 0.0, 0.0),
    ]);
    let mut faces = Vec::new();

    assert!(!surface_ring_ears::try_triangulate_the_ring(
        &[0, 1, 2, 3],
        &pool,
        &mut faces,
        Vector3::new(0.0, 0.0, 1.0),
        false
    ));
}

/// A hole vertex standing EXACTLY on an outer corner's ear edge is no ear: cutting that corner off
/// lays a triangle straight over the hole.
///
/// The hole here is a triangle every vertex of which sits on the rim of the corner (4,0)'s own
/// triangle - two on the square's diagonal, one on its right hand side - so a veto read with any
/// nearness at all lets the corner through and the cover claims the hole's two square millimetres
/// as its own. Read with no ruler, on the rim counts as inside and the corner is refused.
#[test]
fn a_hole_vertex_standing_on_an_ears_edge_is_no_ear() {
    let pool = pool_of(&[
        Vector3::new(0.0, 0.0, 0.0),
        Vector3::new(4.0, 0.0, 0.0),
        Vector3::new(4.0, 4.0, 0.0),
        Vector3::new(0.0, 4.0, 0.0),
        Vector3::new(1.0, 1.0, 0.0),
        Vector3::new(4.0, 2.0, 0.0),
        Vector3::new(3.0, 3.0, 0.0),
    ]);
    let normal = Vector3::new(0.0, 0.0, 1.0);
    let mut faces = Vec::new();

    // Round the outer square, out along the slit to the hole's nearest corner, round the hole the
    // other way, and back along the same slit.
    let ring = [0, 1, 2, 3, 0, 4, 6, 5, 4];

    assert!(surface_ring_ears::try_triangulate_the_ring(
        &ring, &pool, &mut faces, normal, false
    ));

    // The square less the hole - which is what a triangle laid over the hole would break.
    let area = signed_area(&pool, &faces, normal);

    assert!((area - 14.0).abs() <= 1e-9);

    // C#'s `ring.Distinct()`, first occurrences in order.
    let mut distinct = Vec::new();
    for vertex in ring {
        if !distinct.contains(&vertex) {
            distinct.push(vertex);
        }
    }

    for face in &faces {
        let first = pool.position_of(face.0);
        let second = pool.position_of(face.1);
        let third = pool.position_of(face.2);

        // Every triangle lies the way the ring does.
        assert!((second - first).cross(third - first).dot(normal) >= 0.0);

        // And holds no vertex of the ring that is not one of its own.
        for &other in &distinct {
            if other == face.0 || other == face.1 || other == face.2 {
                continue;
            }

            let at = pool.position_of(other);
            let inside = [(first, second), (second, third), (third, first)]
                .iter()
                .all(|edge| (at - edge.0).cross(edge.1 - edge.0).dot(normal) > 1e-9);

            assert!(!inside);
        }
    }
}

fn check_the_bridged_square(motion: Matrix4X4, near_enough: f64) {
    let outer = [
        Vector3::new(0.0, 0.0, 0.0),
        Vector3::new(4.0, 0.0, 0.0),
        Vector3::new(4.0, 4.0, 0.0),
        Vector3::new(0.0, 4.0, 0.0),
    ];

    let hole = [
        Vector3::new(1.0, 1.0, 0.0),
        Vector3::new(3.0, 1.0, 0.0),
        Vector3::new(3.0, 3.0, 0.0),
        Vector3::new(1.0, 3.0, 0.0),
    ];

    let points: Vec<Vector3> = outer
        .iter()
        .chain(hole.iter())
        .map(|at| at.transform(motion))
        .collect();

    let pool = pool_of(&points);
    let normal = Vector3::new(0.0, 0.0, 1.0)
        .transform_normal(motion)
        .get_normal();

    // Round the outer ring, out to the hole at its nearest corner, round the hole the other way,
    // and back along the very same edge.
    let ring = [0, 1, 2, 3, 0, 4, 7, 6, 5, 4];

    let mut faces = Vec::new();

    assert!(surface_ring_ears::try_triangulate_the_ring(
        &ring, &pool, &mut faces, normal, false
    ));

    let area = signed_area(&pool, &faces, normal);

    // The outer square less the hole, every triangle lying the way the ring does.
    assert!((area - 12.0).abs() <= near_enough);

    let inverse = motion.inverted();

    for face in &faces {
        let middle =
            (pool.position_of(face.0) + pool.position_of(face.1) + pool.position_of(face.2)) / 3.0;
        let flat = middle.transform(inverse);

        // Nothing covers the hole.
        assert!(!(flat.x > 1.001 && flat.x < 2.999 && flat.y > 1.001 && flat.y < 2.999));
    }

    // A triangulation of one region uses each directed edge at most once - twice would be two
    // triangles laid over one another, which is how a folded cover reads.
    let mut used = HashSet::new();

    for face in &faces {
        for edge in [(face.0, face.1), (face.1, face.2), (face.2, face.0)] {
            assert!(used.insert(edge));
        }
    }

    // The slit's own pair is there, once each way: the bridge is a real edge of the cover.
    assert!(used.contains(&(0, 4)) && used.contains(&(4, 0)));
}

/// TWO SIDES FACING ONE ANOTHER ACROSS A RING THAT IS ONLY NEARLY FLAT do not cross, and the
/// simplicity check has to read them in the RING'S OWN PLANE to say so.
///
/// The long sides of this 5 by 1 quad run opposite ways, so the cross product of their two
/// directions - which is all the crossing test used to have to stand a plane on - is almost
/// nothing, and the hundredth of a millimetre by which the four corners stand off flat is then the
/// whole of it. Read across that plane, two sides five millimetres apart project onto one another
/// and straddle, and a plain rectangle is refused as self crossing.
///
/// This is the shape the barrel of "Many Holes" hands over: a flat piece of a tessellated wall
/// spliced to the bore mouths inside it, whose mouth vertices stand up to a twentieth of a
/// millimetre off the piece's plane.
#[test]
fn two_sides_facing_one_another_across_a_nearly_flat_ring_do_not_cross() {
    let pool = pool_of(&[
        Vector3::new(0.0, 0.0, -0.01),
        Vector3::new(0.0, 1.0, 0.01),
        Vector3::new(5.0, 1.0, 0.005),
        Vector3::new(5.0, 0.0, -0.005),
    ]);
    let normal = Vector3::new(0.0, 0.0, 1.0);
    let mut faces = Vec::new();

    assert!(
        surface_ring_ears::try_triangulate_the_ring(
            &[0, 1, 2, 3],
            &pool,
            &mut faces,
            normal,
            false
        ),
        "a rectangle is a rectangle however slightly its corners stand off flat"
    );

    let area = signed_area(&pool, &faces, normal);

    assert!((area - 5.0).abs() <= 1e-4);
}
