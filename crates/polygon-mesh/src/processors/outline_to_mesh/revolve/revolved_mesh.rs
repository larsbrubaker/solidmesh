//! The indexed mesh a revolve is built in (the entry point and the profile clean-up are in
//! `revolve.rs`). Every profile point is placed once per column of the sweep (once per pass where
//! the loops pass a point more than once: `processors/ring_vertices.rs`), a point on the axis once
//! for the whole solid (the pole every column shares), and every face - the strips between
//! columns and the end caps of a partial sweep - is made of those vertex indices. A full turn's
//! last strip closes onto the first column's own vertices, so the solid is closed by
//! construction, with no welding of equal positions afterwards.

use clipper2_rust::Paths64;
use vector_math::{Matrix4X4, Vector2, Vector3, Vector3Float};

use crate::mesh::Mesh;
use crate::processors::ring_vertices::RingVertices;

/// Clipper units per mm, the scale the cleaned profile's integer points are in.
const UNITS_PER_MM: f64 = clipper_library::DEFAULT_SCALING;

/// The solid swept by `loops` (loops prepared by `ring_loops`, x out from the axis, y along it,
/// outlines counter clockwise) through the column `angles`. A full turn (`wraps`) closes its last
/// strip onto the first column; a partial sweep caps its first and last columns flat.
pub(super) fn revolved_mesh(
    loops: &Paths64,
    angles: &[f64],
    wraps: bool,
    revolve_around_z: bool,
) -> Mesh {
    let mut mesh = Mesh::new();
    let on_axis = |l: usize, i: usize| loops[l][i].x == 0;

    // every loop point placed in every column: a point on the axis is the same vertex in every
    // column, and a point the loops pass more than once has a vertex per pass
    let mut columns: Vec<RingVertices> = Vec::with_capacity(angles.len());
    for &angle in angles {
        let first = columns.first();
        let column = RingVertices::place(loops, |l, i, _| {
            if let Some(first) = first.filter(|_| on_axis(l, i)) {
                return first.indices[l][i];
            }
            let point = loops[l][i];
            let position =
                Vector2::new(point.x as f64 / UNITS_PER_MM, point.y as f64 / UNITS_PER_MM);
            let position = revolved_position(position, angle, revolve_around_z);
            mesh.vertices.push(Vector3Float::from_vector3(position));
            mesh.vertices.len() as i32 - 1
        });
        columns.push(column);
    }

    let caps = !wraps && angles.len() > 1;
    let all_loops: Vec<usize> = (0..loops.len()).collect();
    // The strips walk the first column's loop edges backward and the last column's forward, so
    // the start cap walks them in loop order and the end cap backward, facing out of the solid
    // either way.
    if caps {
        let faces = columns[0].cap_faces(loops, &all_loops, true);
        add_faces(&mut mesh, &faces);
    }

    let strips = if wraps {
        angles.len()
    } else {
        angles.len() - 1
    };
    for column in 0..strips {
        let next = (column + 1) % angles.len();
        let (here, there) = (&columns[column].indices, &columns[next].indices);
        for (l, path) in loops.iter().enumerate() {
            for i in 0..path.len() {
                let j = (i + 1) % path.len();
                // a segment lying on the axis sweeps no area
                if path[i] == path[j] || (on_axis(l, i) && on_axis(l, j)) {
                    continue;
                }
                mesh.add_polygon(&[there[l][j], here[l][j], here[l][i], there[l][i]]);
            }
        }
    }

    if caps {
        let faces = columns[angles.len() - 1].cap_faces(loops, &all_loops, false);
        add_faces(&mut mesh, &faces);
    }
    mesh
}

fn add_faces(mesh: &mut Mesh, faces: &[[i32; 3]]) {
    for &[a, b, c] in faces {
        mesh.faces.add(a, b, c, &mesh.vertices);
    }
}

/// The single place a profile point becomes a point on the solid, at `angle` around the axis.
fn revolved_position(profile_point: Vector2, angle: f64, revolve_around_z: bool) -> Vector3 {
    if revolve_around_z {
        return Vector3::new(profile_point.x, 0.0, profile_point.y)
            .transform(Matrix4X4::create_rotation_z(angle));
    }

    Vector3::new(profile_point.x, profile_point.y, 0.0)
        .transform(Matrix4X4::create_rotation_y(angle))
}
