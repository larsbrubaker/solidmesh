//! Ports `GeometryProcessingTests/SurfaceFanTests.cs`: the one triangulator the surface bevel
//! uses, on the rings that are hard to fan. Everything else in the construction leans on this
//! doing two things: covering the ring exactly, and USING every point of it - a point the fan drops
//! is a point the walkback carefully shared with a neighbour and then abandoned, which is a T
//! junction on the very seam the sharing was for.
//!
//! The `[Arguments(0.001)]`/`[Arguments(1000.0)]` pair of
//! `AReflexRingIsFannedTheSameWayWhateverSizeItIs` runs both cases inside the one test.

use std::collections::HashSet;

use geometry_processing::bevel::surface::{surface_fan, SurfaceVertexPool};
use vector_math::Vector3;

/// The hard case, and the reason the apex is chosen rather than taken: a triangle with a point
/// clipped into each of its edges. Fanned from a corner, the triangle spanning the point on that
/// corner's OWN edge has no area, is dropped, and that point ends up in no triangle at all. Fanned
/// from a clipped point instead, every triangle is real and all six points are used.
#[test]
fn a_triangle_with_a_point_on_every_edge_uses_all_six_of_them() {
    let mut pool = SurfaceVertexPool::new(&[]);

    let corners = [
        Vector3::new(0.0, 0.0, 0.0),
        Vector3::new(12.0, 0.0, 0.0),
        Vector3::new(0.0, 9.0, 0.0),
    ];
    let mut ring = Vec::new();

    for corner in 0..corners.len() {
        ring.push(pool.add(corners[corner]));
        ring.push(pool.add((corners[corner] + corners[(corner + 1) % corners.len()]) / 2.0));
    }

    let mut faces = Vec::new();

    surface_fan::emit(&ring, &mut pool, &mut faces);

    // The fan covers the triangle exactly, no more and no less (C# `12 * 9 / 2` is integer 54).
    assert!((area_of(&pool, &faces) - 54.0).abs() <= 1e-9);

    let mut used = HashSet::new();

    for face in &faces {
        used.insert(face.0);
        used.insert(face.1);
        used.insert(face.2);
    }

    for vertex in &ring {
        assert!(
            used.contains(vertex),
            "{} is a point of the ring, so some triangle has to use it",
            pool.position_of(*vertex)
        );
    }

    assert_eq!(
        pool.vertices().len(),
        6,
        "a ring a vertex of its own can fan needs no new point in the middle of it"
    );
}

/// The fallback, reached directly. A ring every vertex of which sees a flat triangle has no apex
/// to fan from, and that takes a collinear triple at every vertex - which means the ring is a line
/// and covers nothing. It is emitted AS IT STANDS, from one of its two farthest apart corners: zero
/// area, but the ring's own sides used once each and no vertex standing in the middle of a shape
/// with no middle.
#[test]
fn a_ring_with_no_apex_to_fan_from_is_emitted_as_it_stands_with_nothing_minted() {
    let mut pool = SurfaceVertexPool::new(&[]);

    let ring = vec![
        pool.add(Vector3::new(0.0, 0.0, 0.0)),
        pool.add(Vector3::new(4.0, 0.0, 0.0)),
        pool.add(Vector3::new(9.0, 0.0, 0.0)),
    ];

    let mut faces = Vec::new();

    surface_fan::emit(&ring, &mut pool, &mut faces);

    assert_eq!(
        faces.len(),
        1,
        "three points of one straight line are the needle they came in as, whatever their area"
    );
    assert_eq!(
        faces[0],
        (ring[0], ring[1], ring[2]),
        "fanned from a farthest corner, which on three corners is the ring itself"
    );
    assert_eq!(
        pool.vertices().len(),
        3,
        "and the middle of nothing is not a vertex anybody needs"
    );
}

/// Winding to order. The strip's rings are built rail by rail out of lines and have no winding of
/// their own to follow, so every triangle is turned to face the way it is told - which is what
/// keeps a bevel's new surface facing out on a bead as much as on a cut.
#[test]
fn a_fan_told_which_way_to_face_turns_every_triangle_that_way() {
    let mut pool = SurfaceVertexPool::new(&[]);

    let ring = vec![
        pool.add(Vector3::new(0.0, 0.0, 0.0)),
        pool.add(Vector3::new(4.0, 0.0, 0.0)),
        pool.add(Vector3::new(4.0, 4.0, 0.0)),
        pool.add(Vector3::new(0.0, 4.0, 0.0)),
    ];

    for facing in [Vector3::new(0.0, 0.0, 1.0), Vector3::new(0.0, 0.0, -1.0)] {
        let mut faces = Vec::new();

        surface_fan::emit_facing(&ring, &mut pool, &mut faces, facing);

        assert!((area_of(&pool, &faces) - 16.0).abs() <= 1e-9);

        for face in &faces {
            let from = pool.position_of(face.0);
            let normal = (pool.position_of(face.1) - from).cross(pool.position_of(face.2) - from);

            assert!(
                normal.dot(facing) > 0.0,
                "every triangle of the fan faces {facing}"
            );
        }
    }
}

/// A ring with a REFLEX vertex, which is what a chamfer strip's bent end is. Only one vertex of
/// such a ring can see the whole of it, and the area test alone does not find it: every other apex
/// fans a triangle that is perfectly fat and lies outside the ring, folded back over its
/// neighbours. The fan has to cover 65 square millimetres and no more, which a folded one cannot.
///
/// The pentagon is A(0,0) B(10,0) C(10,10) D(5,3) E(0,10), wound counter clockwise, with D pushed
/// in far enough to hide C from A and A from C. Fanned from D every triangle lies the way the ring
/// does; fanned from any of the other four, one does not.
#[test]
fn a_ring_with_a_reflex_vertex_is_fanned_from_the_one_vertex_that_sees_it_all() {
    let mut pool = SurfaceVertexPool::new(&[]);

    let ring = vec![
        pool.add(Vector3::new(0.0, 0.0, 0.0)),
        pool.add(Vector3::new(10.0, 0.0, 0.0)),
        pool.add(Vector3::new(10.0, 10.0, 0.0)),
        pool.add(Vector3::new(5.0, 3.0, 0.0)),
        pool.add(Vector3::new(0.0, 10.0, 0.0)),
    ];

    let mut faces = Vec::new();

    surface_fan::emit(&ring, &mut pool, &mut faces);

    assert!(
        (area_of(&pool, &faces) - 65.0).abs() <= 1e-9,
        "the shoelace area of the pentagon is 65, and a fan that reaches outside it covers more"
    );

    for face in &faces {
        let from = pool.position_of(face.0);
        let normal = (pool.position_of(face.1) - from).cross(pool.position_of(face.2) - from);

        assert!(
            normal.z > 0.0,
            "the ring is wound counter clockwise, so every triangle of its fan is too - {}, {}, {} is not",
            pool.position_of(face.0),
            pool.position_of(face.1),
            pool.position_of(face.2)
        );
    }

    assert_eq!(
        pool.vertices().len(),
        5,
        "the reflex vertex can fan the ring, so no point is added in the middle of it"
    );
}

/// The same reflex pentagon a thousand times smaller and a thousand times larger. What it pins is
/// that the fold test is scale FREE: it asks whether a triangle agrees with the ring by more than
/// rounding, and rounding on an area is relative, so a floor written as a flat number would call
/// every triangle of the small ring folded or let every triangle of the large one through.
#[test]
fn a_reflex_ring_is_fanned_the_same_way_whatever_size_it_is() {
    for scale in [0.001, 1000.0] {
        let mut pool = SurfaceVertexPool::new(&[]);

        let corners = [
            Vector3::new(0.0, 0.0, 0.0),
            Vector3::new(10.0, 0.0, 0.0),
            Vector3::new(10.0, 10.0, 0.0),
            Vector3::new(5.0, 3.0, 0.0),
            Vector3::new(0.0, 10.0, 0.0),
        ];

        let mut ring = Vec::new();

        for corner in corners {
            ring.push(pool.add(corner * scale));
        }

        let mut faces = Vec::new();

        surface_fan::emit(&ring, &mut pool, &mut faces);

        assert!(
            (area_of(&pool, &faces) - 65.0 * scale * scale).abs() <= 65.0 * scale * scale * 1e-6,
            "the pentagon's area scales with the square, and a fan that reaches outside it covers more"
        );

        for face in &faces {
            let from = pool.position_of(face.0);
            let normal = (pool.position_of(face.1) - from).cross(pool.position_of(face.2) - from);

            assert!(
                normal.z > 0.0,
                "every triangle of the fan is wound the way the ring is, at any size"
            );
        }

        assert_eq!(
            pool.vertices().len(),
            5,
            "the reflex vertex can fan the ring at any size, so no point is added in the middle of it"
        );
    }
}

fn area_of(pool: &SurfaceVertexPool, faces: &[(i32, i32, i32)]) -> f64 {
    let mut area = 0.0;

    for face in faces {
        let from = pool.position_of(face.0);

        area += (pool.position_of(face.1) - from)
            .cross(pool.position_of(face.2) - from)
            .length()
            / 2.0;
    }

    area
}
