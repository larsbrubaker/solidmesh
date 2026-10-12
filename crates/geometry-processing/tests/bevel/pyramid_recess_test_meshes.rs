//! Port of `Fixtures/PyramidRecessTestMeshes.cs`: the part behind "UpperLeftInner should
//! bevel.mcx" - a 20 mm cube with a four sided PYRAMIDAL RECESS sunk into its front face, closing to
//! a horizontal ridge - with the fixture's own 0.0316 pinch, built either as a clean solid from its
//! planes or from the fixture's own fourteen vertices, optionally mirrored in z and optionally
//! moved to where the rebuild has it.
//!
//! **The shape.** The recess is the whole of the front face y = 0 at the mouth and closes to the
//! ridge at y = 10, z = 10.0316, x from 5 to 15. Its walls are the upper slope y + z = 20.0316, the
//! lower slope z = y + 0.0316, the left wall y = 2x and the right wall 2x + y = 40. The pinch puts
//! both slopes 0.0316 off the round numbers, and that shows at the mouth twice: the upper slope
//! leaves the cube through the TOP face at y = 0.0316, so the top face is a hexagon and the cube's
//! front-top-left corner (0, 0, 20) is a vertex where the top face, the left face and the left wall
//! meet; and the lower slope leaves through the FRONT face at z = 0.0316, leaving a strip of it
//! standing along the bottom. Fourteen vertices, 6333.343 mm^3.
//!
//! **Two builders.** [`create`] builds it from its planes, one polygon per face with the winding
//! checked against the outward normal; [`create_from_the_fixtures_own_triangles`] builds it from
//! the triangles the fixture's boolean left behind - the same fourteen vertices, but with the
//! walls' diagonals where that boolean put them. The second is what a reading about the rebuild has
//! to be taken on. **Where the rebuild has it:** the source item sits at [`fixture_offset`], a
//! different float grid, which is the whole point of offering the offset.

#![allow(dead_code)]

use polygon_mesh::Mesh;
use vector_math::Vector3;

/// The cube's edge.
pub const SIZE: f64 = 20.0;

/// How far the fixture's ridge sits above z = 10, and the mouth's top edge above y = 0 - the float
/// value the asset carries, so that the two builders agree to the bit.
pub const PINCH: f64 = 0.03160095214844;

/// The ridge's height.
pub const RIDGE_Z: f64 = 10.0 + PINCH;

/// Where the upper-left crease reaches the recess's mouth, on the cube's top face.
pub fn rim_end() -> Vector3 {
    Vector3::new(PINCH / 2.0, PINCH, SIZE)
}

/// Where the upper-left crease dies: the left end of the ridge, a three way concave corner.
pub fn apex_end() -> Vector3 {
    Vector3::new(5.0, 10.0, RIDGE_Z)
}

/// The right end of the ridge.
pub fn right_apex() -> Vector3 {
    Vector3::new(15.0, 10.0, RIDGE_Z)
}

/// Where the LOWER-left crease reaches the mouth, on the strip of front face the lower pinch leaves.
pub fn lower_rim_end() -> Vector3 {
    Vector3::new(0.0, 0.0, PINCH)
}

/// Where the fixture's own working mesh sits: the box frame translated by this, which is what the
/// rebuild actually hands the kernel.
pub fn fixture_offset() -> Vector3 {
    Vector3::new(-65.0, 95.0, -20.0)
}

/// The same point mirrored in z about the cube's middle.
pub fn mirror_z(point: Vector3) -> Vector3 {
    Vector3::new(point.x, point.y, SIZE - point.z)
}

/// The upper slope's outward normal, y + z = 20.0316 with material at y + z greater.
pub fn upper_slope_normal() -> Vector3 {
    Vector3::new(0.0, -1.0, -1.0).get_normal()
}

/// The lower slope's outward normal, z = y + 0.0316 with material below.
pub fn lower_slope_normal() -> Vector3 {
    Vector3::new(0.0, -1.0, 1.0).get_normal()
}

/// The left wall's outward normal, y = 2x with material at x less than y / 2.
pub fn left_wall_normal() -> Vector3 {
    Vector3::new(2.0, -1.0, 0.0).get_normal()
}

/// C# `Create`: the solid from its planes - ten polygons, each handed over with its outward normal
/// so the winding is checked rather than trusted. `Vector3::ZERO` offset builds it in the box frame.
pub fn create(mirror: bool, offset: Vector3) -> Mesh {
    let mut mesh = Mesh::new();

    let half = PINCH / 2.0;
    let top_left = Vector3::new(half, PINCH, SIZE);
    let top_right = Vector3::new(SIZE - half, PINCH, SIZE);
    let bottom_left = lower_rim_end();
    let bottom_right = Vector3::new(SIZE, 0.0, PINCH);
    let ridge_left = apex_end();
    let ridge_right = right_apex();

    let p = |point: Vector3| (if mirror { mirror_z(point) } else { point }) + offset;
    let n = |normal: Vector3| {
        if mirror {
            Vector3::new(normal.x, normal.y, -normal.z)
        } else {
            normal
        }
    };

    let mut face = |outward: Vector3, points: &[Vector3]| {
        let mut mapped: Vec<Vector3> = points.iter().map(|&point| p(point)).collect();

        // Newell's normal of the polygon as handed over; reverse it where that disagrees with the
        // outward normal asked for.
        let mut normal = Vector3::ZERO;

        for i in 0..mapped.len() {
            let a = mapped[i];
            let b = mapped[(i + 1) % mapped.len()];

            normal += Vector3::new(
                (a.y - b.y) * (a.z + b.z),
                (a.z - b.z) * (a.x + b.x),
                (a.x - b.x) * (a.y + b.y),
            );
        }

        if normal.dot(n(outward)) < 0.0 {
            mapped.reverse();
        }

        // Fanned from the first point: every face here is convex.
        for i in 1..mapped.len() - 1 {
            mesh.create_face(&[mapped[0], mapped[i], mapped[i + 1]]);
        }
    };

    let v = Vector3::new;

    // The recess's four walls.
    face(
        lower_slope_normal(),
        &[bottom_left, bottom_right, ridge_right, ridge_left],
    );
    face(
        upper_slope_normal(),
        &[top_left, ridge_left, ridge_right, top_right],
    );
    face(
        left_wall_normal(),
        &[bottom_left, ridge_left, top_left, v(0.0, 0.0, SIZE)],
    );
    face(
        v(-2.0, -1.0, 0.0).get_normal(),
        &[bottom_right, v(SIZE, 0.0, SIZE), top_right, ridge_right],
    );

    // What the two pinches leave of the cube's own faces: a hexagonal top, a strip of front, and
    // the strip's corners as vertices on the side faces' front edges.
    face(
        Vector3::UNIT_Z,
        &[
            v(0.0, 0.0, SIZE),
            top_left,
            top_right,
            v(SIZE, 0.0, SIZE),
            v(SIZE, SIZE, SIZE),
            v(0.0, SIZE, SIZE),
        ],
    );
    face(
        -Vector3::UNIT_Y,
        &[
            v(0.0, 0.0, 0.0),
            bottom_left,
            bottom_right,
            v(SIZE, 0.0, 0.0),
        ],
    );
    face(
        -Vector3::UNIT_Z,
        &[
            v(0.0, 0.0, 0.0),
            v(SIZE, 0.0, 0.0),
            v(SIZE, SIZE, 0.0),
            v(0.0, SIZE, 0.0),
        ],
    );
    face(
        -Vector3::UNIT_X,
        &[
            v(0.0, 0.0, 0.0),
            bottom_left,
            v(0.0, 0.0, SIZE),
            v(0.0, SIZE, SIZE),
            v(0.0, SIZE, 0.0),
        ],
    );
    face(
        Vector3::UNIT_X,
        &[
            v(SIZE, 0.0, 0.0),
            v(SIZE, SIZE, 0.0),
            v(SIZE, SIZE, SIZE),
            v(SIZE, 0.0, SIZE),
            bottom_right,
        ],
    );
    face(
        Vector3::UNIT_Y,
        &[
            v(0.0, SIZE, 0.0),
            v(0.0, SIZE, SIZE),
            v(SIZE, SIZE, SIZE),
            v(SIZE, SIZE, 0.0),
        ],
    );

    mesh.clean_and_merge();

    mesh
}

/// C# `CreateFromTheFixturesOwnTriangles`: the fixture's own source mesh - the 14 vertices and 24
/// triangles the boolean that cut the recess left behind - carried into the box frame, so that a
/// reading can be taken on exactly the topology the rebuild reads: its diagonals run from the apex
/// to the cube's top-left corner across the left wall and to the mouth's top RIGHT corner across
/// the upper slope. `offset` is where to put it ([`fixture_offset`] is where the rebuild has it);
/// `mirror` mirrors it about z = 10 first, as [`create`] does.
pub fn create_from_the_fixtures_own_triangles(offset: Vector3, mirror: bool) -> Mesh {
    // The asset's vertices, in the source item's local space: the recess is cut into y = -95 and
    // the ridge is at z = 9.9684. The rebuild turns that 180 degrees about x, which is what the map
    // below does - a rotation, so every triangle keeps its winding.
    let local = [
        Vector3::new(-65.0, -115.0, 0.0),
        Vector3::new(-65.0, -115.0, 20.0),
        Vector3::new(-64.98419952392578, -95.03160095214844, 0.0),
        Vector3::new(-65.0, -95.0, 0.0),
        Vector3::new(-60.0, -105.0, 9.968400955200195),
        Vector3::new(-65.0, -95.0, 19.968400955200195),
        Vector3::new(-65.0, -95.0, 20.0),
        Vector3::new(-45.0, -115.0, 0.0),
        Vector3::new(-45.0, -115.0, 20.0),
        Vector3::new(-50.0, -105.0, 9.968400955200195),
        Vector3::new(-45.01580047607422, -95.03160095214844, 0.0),
        Vector3::new(-45.0, -95.0, 0.0),
        Vector3::new(-45.0, -95.0, 19.968400955200195),
        Vector3::new(-45.0, -95.0, 20.0),
    ];

    let triangles: [(usize, usize, usize); 24] = [
        (0, 1, 6),
        (0, 8, 1),
        (0, 3, 2),
        (0, 5, 3),
        (0, 2, 10),
        (0, 6, 5),
        (1, 13, 6),
        (6, 13, 5),
        (0, 7, 8),
        (0, 10, 7),
        (7, 11, 8),
        (1, 8, 13),
        (7, 10, 11),
        (13, 12, 5),
        (8, 11, 12),
        (8, 12, 13),
        (4, 2, 3),
        (4, 3, 5),
        (4, 10, 2),
        (4, 5, 9),
        (4, 9, 10),
        (9, 12, 10),
        (11, 10, 12),
        (9, 5, 12),
    ];

    let mut mesh = Mesh::new();

    let in_the_box = |point: Vector3| {
        let in_the_box = Vector3::new(point.x + 65.0, -point.y - 95.0, 20.0 - point.z);

        (if mirror {
            mirror_z(in_the_box)
        } else {
            in_the_box
        }) + offset
    };

    for (a, b, c) in triangles {
        // A mirror turns every triangle inside out, so its winding is reversed to keep the solid
        // right way out.
        if mirror {
            mesh.create_face(&[
                in_the_box(local[a]),
                in_the_box(local[c]),
                in_the_box(local[b]),
            ]);
        } else {
            mesh.create_face(&[
                in_the_box(local[a]),
                in_the_box(local[b]),
                in_the_box(local[c]),
            ]);
        }
    }

    mesh.clean_and_merge();

    mesh
}
