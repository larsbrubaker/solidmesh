//! The plate with a round bore, the plate with a RAKED bore, and the split of one of its edges.
//! The pieces of the bore test meshes (`create_plate_with_a_round_bore`,
//! `create_plate_with_a_raked_bore`, `create_the_raked_fixture` and its `RAKED_*` dimensions,
//! `tile_the_ring_between`) and of their split-chord variant
//! (`create_plate_with_a_raked_bore_whose_mouth_chords_are_split`, `split_the_edge_between`) that
//! the test classes here read; the barrel port and the edge bore are their own modules.

use crate::path_solids::revolve;
use agg_path::vertex_storage::VertexStorage;
use polygon_mesh::{FaceList, Mesh};
use vector_math::{math_helper, Vector3, Vector3Float};

/// How many facets round the raked fixture's bore, which is also its mouth's
/// chord count.
pub const RAKED_SIDES: i32 = 40;

/// How far the raked fixture's drill leans off the plate's normal, in the
/// x z plane.
pub const RAKED_RAKE_DEGREES: f64 = 45.0;

/// The raked fixture's bore radius.
pub const RAKED_RADIUS: f64 = 1.5;

/// How thick that plate is, centred on z = 0.
pub const RAKED_THICKNESS: f64 = 3.0;

/// The raked fixture's drill direction, leaned [`RAKED_RAKE_DEGREES`] off the
/// plate's normal in the x z plane.
pub fn raked_drill_axis() -> Vector3 {
    let rake = math_helper::degrees_to_radians(RAKED_RAKE_DEGREES);
    Vector3::new(rake.sin(), 0.0, rake.cos())
}

/// The raked fixture at the dimensions every suite that reads it uses
/// (a land of 6).
pub fn create_the_raked_fixture() -> Mesh {
    create_plate_with_a_raked_bore(
        RAKED_SIDES,
        RAKED_RADIUS,
        RAKED_THICKNESS,
        RAKED_RAKE_DEGREES,
        6.0,
    )
}

/// A plate `thickness` thick, centred on z = 0, with a bore of
/// `radius` drawn with `sides` facets and `land` of plate outside it.
pub fn create_plate_with_a_round_bore(sides: i32, radius: f64, thickness: f64, land: f64) -> Mesh {
    let mut path = VertexStorage::new();
    path.move_to(radius, -thickness / 2.0);
    path.line_to(radius + land, -thickness / 2.0);
    path.line_to(radius + land, thickness / 2.0);
    path.line_to(radius, thickness / 2.0);

    // The closing segment back to (radius, -thickness / 2) is the BORE wall, which is why the
    // profile is left open here.
    revolve(&path, sides, 0.0, std::f64::consts::TAU, true).expect("the plate revolves")
}

/// The same plate and the same bore, drilled at a RAKE instead of
/// square - so each mouth is an ELLIPSE and the wedge along it runs from `90 - rake` at one end of
/// the loop to `90 + rake` at the other.
///
/// With the section taken square to the axis through the origin, the upper mouth comes out as the
/// ellipse `(r cos a / cos(rake) + (t / 2) tan(rake), r sin a)`, and the lower mouth the same the
/// other way. Both contain the origin as long as the plate is thinner than `2 r / sin(rake)`, which
/// is what lets each face be tiled by walking the mouth and the rim together in ANGLE about the
/// origin ([`tile_the_ring_between`]) with no boolean anywhere and so no seam of one.
pub fn create_plate_with_a_raked_bore(
    sides: i32,
    radius: f64,
    thickness: f64,
    rake_degrees: f64,
    land: f64,
) -> Mesh {
    let rake = math_helper::degrees_to_radians(rake_degrees);
    let axis = Vector3::new(rake.sin(), 0.0, rake.cos());
    let across = Vector3::new(rake.cos(), 0.0, -rake.sin());
    let along = Vector3::new(0.0, 1.0, 0.0);

    let count = sides as usize;
    let mut upper = Vec::with_capacity(count);
    let mut lower = Vec::with_capacity(count);
    let mut rim = Vec::with_capacity(count);

    let outer_radius = (radius / rake.cos()) + ((thickness / 2.0) * rake.tan()) + land;

    for i in 0..sides {
        let turn = 2.0 * std::f64::consts::PI * f64::from(i) / f64::from(sides);
        let on_the_section = (across * (radius * turn.cos())) + (along * (radius * turn.sin()));

        // Slid along the drill's own axis until it reaches each face, which is what makes the
        // mouth an ellipse and each wall facet an exactly planar parallelogram.
        upper.push(on_the_section + (axis * (((thickness / 2.0) - on_the_section.z) / axis.z)));
        lower.push(on_the_section + (axis * (((-thickness / 2.0) - on_the_section.z) / axis.z)));
        rim.push(Vector3::new(
            outer_radius * turn.cos(),
            outer_radius * turn.sin(),
            0.0,
        ));
    }

    let mut mesh = Mesh::new();

    tile_the_ring_between(&mut mesh, &upper, &rim, thickness / 2.0, true);
    tile_the_ring_between(&mut mesh, &lower, &rim, -thickness / 2.0, false);

    for i in 0..count {
        let next = (i + 1) % count;

        // The bore's wall, whose outward normal points IN toward the axis because the material of
        // a plate is outside its bore, and the plate's own outside.
        mesh.create_face(&[upper[i], upper[next], lower[next], lower[i]]);

        mesh.create_face(&[
            Vector3::new(rim[i].x, rim[i].y, thickness / 2.0),
            Vector3::new(rim[i].x, rim[i].y, -thickness / 2.0),
            Vector3::new(rim[next].x, rim[next].y, -thickness / 2.0),
            Vector3::new(rim[next].x, rim[next].y, thickness / 2.0),
        ]);
    }

    mesh.clean_and_merge();

    mesh
}

/// Tiles the flat ring between an inner loop and an outer one, both
/// wound the same way and both containing the origin, by walking them together in ANGLE about it:
/// at each step the vertex whose angle comes next is taken, and the triangle between the old spoke
/// and the new one is emitted. It cannot fold, because the two sequences are each sorted by angle
/// and so the spokes cannot cross.
///
/// `z` is the plane to lay the ring in (the outer loop is given at z = 0 and moved to it);
/// `facing_up` whether the ring's normal is +z.
fn tile_the_ring_between(
    mesh: &mut Mesh,
    mouth: &[Vector3],
    rim: &[Vector3],
    z: f64,
    facing_up: bool,
) {
    let sides = mouth.len();
    let rim_sides = rim.len();
    let start = mouth[0].y.atan2(mouth[0].x);
    let two_pi = 2.0 * std::f64::consts::PI;

    let ahead = |at: Vector3| {
        let turn = at.y.atan2(at.x) - start;

        if turn < 0.0 {
            turn + two_pi
        } else {
            turn
        }
    };

    let mut inner = vec![0.0; sides + 1];
    for k in 1..sides {
        inner[k] = ahead(mouth[k]);
    }

    inner[sides] = two_pi;

    // Which of the rim's vertices stands just past the mouth's first, so the walk starts on a
    // spoke that spans less than one step of either loop.
    let mut first = 0;
    let mut least = f64::MAX;
    for (k, &at) in rim.iter().enumerate() {
        let turn = ahead(at);

        if turn > 0.0 && turn < least {
            least = turn;
            first = k;
        }
    }

    let mut outer = vec![0.0; rim_sides + 1];
    outer[0] = least;
    for k in 1..=rim_sides {
        let turn = ahead(rim[(first + k) % rim_sides]);

        outer[k] = if turn <= outer[k - 1] {
            turn + two_pi
        } else {
            turn
        };
    }

    let mut emit = |from: Vector3, spoke: Vector3, to: Vector3| {
        let a = Vector3::new(from.x, from.y, z);
        let b = Vector3::new(spoke.x, spoke.y, z);
        let c = Vector3::new(to.x, to.y, z);

        if facing_up {
            mesh.create_face(&[a, b, c]);
        } else {
            mesh.create_face(&[a, c, b]);
        }
    };

    let mut walked = 0;
    let mut turned = 0;

    while walked < sides || turned < rim_sides {
        if walked < sides && (turned >= rim_sides || inner[walked + 1] <= outer[turned + 1]) {
            emit(
                mouth[walked],
                rim[(first + turned) % rim_sides],
                mouth[(walked + 1) % sides],
            );
            walked += 1;
        } else {
            emit(
                mouth[walked % sides],
                rim[(first + turned) % rim_sides],
                rim[(first + turned + 1) % rim_sides],
            );
            turned += 1;
        }
    }
}

/// The raked plate with chords of its
/// upper mouth split by an extra vertex standing part way along each - the thing a boolean leaves
/// behind and a hand built ring never has. `splits` names each chord by the index of the vertex it
/// leaves and how far along it the extra vertex stands. `standing_at` is where the bore's own
/// centre is put, moved BEFORE the splits so each split vertex is read off the float coordinates
/// the part's own boolean would have read it off (a zero `standing_at` leaves it in place).
pub fn create_plate_with_a_raked_bore_whose_mouth_chords_are_split(
    sides: i32,
    radius: f64,
    thickness: f64,
    rake_degrees: f64,
    splits: &[(i32, f64)],
    land: f64,
    standing_at: Vector3,
) -> Mesh {
    let mut mesh = create_plate_with_a_raked_bore(sides, radius, thickness, rake_degrees, land);

    if standing_at != Vector3::ZERO {
        for v in mesh.vertices.iter_mut() {
            *v = Vector3Float::from_vector3(Vector3::from_vector3_float(*v) + standing_at);
        }
        mesh.mark_as_changed();
    }

    let rake = math_helper::degrees_to_radians(rake_degrees);
    let axis = Vector3::new(rake.sin(), 0.0, rake.cos());
    let across = Vector3::new(rake.cos(), 0.0, -rake.sin());
    let along = Vector3::new(0.0, 1.0, 0.0);

    let mouth_vertex = |index: i32| {
        let turn =
            2.0 * std::f64::consts::PI * (((index % sides) + sides) % sides) as f64 / sides as f64;
        let on_the_section = (across * (radius * turn.cos())) + (along * (radius * turn.sin()));

        on_the_section + (axis * (((thickness / 2.0) - on_the_section.z) / axis.z)) + standing_at
    };

    for &(chord, at) in splits {
        split_the_edge_between(&mut mesh, mouth_vertex(chord), mouth_vertex(chord + 1), at);
    }

    mesh
}

/// Splits the mesh edge between the vertices nearest `from` and `to` at
/// `at` of the way along it, each face on it into two keeping its winding.
pub fn split_the_edge_between(mesh: &mut Mesh, from: Vector3, to: Vector3, at: f64) {
    let nearest = |mesh: &Mesh, point: Vector3| {
        let mut best = -1;
        let mut least = f64::MAX;
        for (i, &v) in mesh.vertices.iter().enumerate() {
            let away = (Vector3::from_vector3_float(v) - point).length_squared();
            if away < least {
                least = away;
                best = i as i32;
            }
        }
        best
    };

    let first = nearest(mesh, from);
    let second = nearest(mesh, to);
    let middle = mesh.vertices.len() as i32;

    let a = Vector3::from_vector3_float(mesh.vertices[first as usize]);
    let b = Vector3::from_vector3_float(mesh.vertices[second as usize]);
    mesh.vertices
        .push(Vector3Float::from_vector3(a + ((b - a) * at)));

    let mut replaced = FaceList::new();
    for face in mesh.faces.iter() {
        let corners = [face.v0, face.v1, face.v2];
        let carries = (0..3).find(|&k| {
            let next = (k + 1) % 3;
            (corners[k] == first && corners[next] == second)
                || (corners[k] == second && corners[next] == first)
        });

        let Some(carries) = carries else {
            // The face is kept itself, with the normal it was made with - before any stand-off
            // moved its vertices - because the builder reads that normal.
            replaced.add_with_normal(face.v0, face.v1, face.v2, face.normal);
            continue;
        };

        // The two halves of the face, each keeping its winding.
        let opposite = corners[(carries + 2) % 3];
        replaced.add(corners[carries], middle, opposite, &mesh.vertices);
        replaced.add(middle, corners[(carries + 1) % 3], opposite, &mesh.vertices);
    }

    mesh.faces = replaced;
    mesh.mark_as_changed();
}
