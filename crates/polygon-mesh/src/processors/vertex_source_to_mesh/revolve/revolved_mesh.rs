//! The indexed mesh a revolve is built in (the entry point and the profile clean-up are in
//! `revolve.rs`). Every profile point is placed once per column of the sweep, a point on the axis
//! once for the whole solid (the pole every column shares), and every face - the strips between
//! columns and the end caps of a partial sweep - is made of those vertex indices. A full turn's
//! last strip closes onto the first column's own vertices, so the solid is closed by
//! construction, with no welding of equal positions afterwards.

use std::collections::HashMap;

use clipper2_rust::{Path64, Paths64};
use vector_math::{Matrix4X4, Vector2, Vector3, Vector3Float};

use crate::mesh::Mesh;
use crate::processors::cap_fill::cap_faces;

/// Clipper units per mm, the scale the cleaned profile's integer points are in.
const UNITS_PER_MM: f64 = agg_path::data_converters_2d::DEFAULT_SCALING;

/// The profile's loops, each point named by its index among the distinct points.
struct Profile {
    /// The distinct points, in the order the loops first reach them.
    points: Vec<(i64, i64)>,
    /// Each loop as point indices.
    loops: Vec<Vec<usize>>,
    /// The loops themselves, for the cap fill.
    paths: Paths64,
    /// The index of each distinct point.
    by_position: HashMap<(i64, i64), usize>,
}

impl Profile {
    fn new(loops_in: &Paths64) -> Self {
        let mut by_position: HashMap<(i64, i64), usize> = HashMap::new();
        let mut points = Vec::new();
        let loops = loops_in
            .iter()
            .map(|path| {
                path.iter()
                    .map(|point| {
                        *by_position.entry((point.x, point.y)).or_insert_with(|| {
                            points.push((point.x, point.y));
                            points.len() - 1
                        })
                    })
                    .collect()
            })
            .collect();
        Profile {
            points,
            loops,
            paths: loops_in.clone(),
            by_position,
        }
    }

    fn on_axis(&self, point: usize) -> bool {
        self.points[point].0 == 0
    }
}

/// The solid swept by `loops` (cleaned clipper loops, x out from the axis, y along it, outlines
/// counter clockwise) through the column `angles`. A full turn (`wraps`) closes its last strip onto
/// the first column; a partial sweep caps its first and last columns flat.
pub(super) fn revolved_mesh(
    loops: &Paths64,
    angles: &[f64],
    wraps: bool,
    revolve_around_z: bool,
) -> Mesh {
    let profile = Profile::new(loops);
    let mut mesh = Mesh::new();

    // vertex[column][point]: a point on the axis is the same vertex in every column
    let mut vertex: Vec<Vec<i32>> = Vec::with_capacity(angles.len());
    for (column, &angle) in angles.iter().enumerate() {
        let row = (0..profile.points.len())
            .map(|point| {
                if column > 0 && profile.on_axis(point) {
                    return vertex[0][point];
                }
                let (x, y) = profile.points[point];
                let position = Vector2::new(x as f64 / UNITS_PER_MM, y as f64 / UNITS_PER_MM);
                let position = revolved_position(position, angle, revolve_around_z);
                mesh.vertices.push(Vector3Float::from_vector3(position));
                mesh.vertices.len() as i32 - 1
            })
            .collect();
        vertex.push(row);
    }

    let caps = !wraps && angles.len() > 1;
    if caps {
        add_cap(&mut mesh, &profile, &vertex[0], true);
    }

    let strips = if wraps {
        angles.len()
    } else {
        angles.len() - 1
    };
    for column in 0..strips {
        let next = (column + 1) % angles.len();
        for indices in &profile.loops {
            for (i, &a) in indices.iter().enumerate() {
                let b = indices[(i + 1) % indices.len()];
                // a segment lying on the axis sweeps no area
                if a == b || (profile.on_axis(a) && profile.on_axis(b)) {
                    continue;
                }
                mesh.add_polygon(&[
                    vertex[next][b],
                    vertex[column][b],
                    vertex[column][a],
                    vertex[next][a],
                ]);
            }
        }
    }

    if caps {
        add_cap(&mut mesh, &profile, &vertex[angles.len() - 1], false);
    }
    mesh
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

/// The flat face closing a partial sweep at one column (`row`, that column's vertices). The
/// strips walk the first column's loop edges backward and the last column's forward, so the start
/// cap walks them in loop order and the end cap backward, facing out of the solid either way. The
/// fill joins only the column's own vertices (see [`cap_faces`]).
fn add_cap(mesh: &mut Mesh, profile: &Profile, row: &[i32], start: bool) {
    let paths: Vec<&Path64> = profile.paths.iter().collect();
    let indices: Vec<Vec<i32>> = profile
        .loops
        .iter()
        .map(|points| points.iter().map(|&point| row[point]).collect())
        .collect();
    let indices: Vec<&[i32]> = indices.iter().map(Vec::as_slice).collect();
    let vertex_at = |x, y| profile.by_position.get(&(x, y)).map(|&point| row[point]);
    for [a, b, c] in cap_faces(&paths, &indices, vertex_at, start) {
        mesh.faces.add(a, b, c, &mesh.vertices);
    }
}
