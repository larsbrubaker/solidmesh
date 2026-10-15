//! The indexed mesh a revolve is built in (the entry point and the profile clean-up are in
//! `revolve.rs`). Every profile point is placed once per column of the sweep, a point on the axis
//! once for the whole solid (the pole every column shares), and every face - the strips between
//! columns and the end caps of a partial sweep - is made of those vertex indices. A full turn's
//! last strip closes onto the first column's own vertices, so the solid is closed by
//! construction, with no welding of equal positions afterwards.

use std::collections::HashMap;

use clipper2_rust::Paths64;
use tess2_rust::{ElementType, TessOption, Tessellator, WindingRule};
use vector_math::{Matrix4X4, Vector2, Vector3, Vector3Float};

use crate::mesh::Mesh;

/// Clipper units per mm, the scale the cleaned profile's integer points are in.
const UNITS_PER_MM: f64 = agg_path::data_converters_2d::DEFAULT_SCALING;

/// The profile's loops, each point named by its index among the distinct points.
struct Profile {
    /// The distinct points, in the order the loops first reach them.
    points: Vec<(i64, i64)>,
    /// Each loop as point indices.
    loops: Vec<Vec<usize>>,
}

impl Profile {
    fn new(loops: &Paths64) -> Self {
        let mut by_position: HashMap<(i64, i64), usize> = HashMap::new();
        let mut points = Vec::new();
        let loops = loops
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
        Profile { points, loops }
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
                add_polygon(
                    &mut mesh,
                    &[
                        vertex[next][b],
                        vertex[column][b],
                        vertex[column][a],
                        vertex[next][a],
                    ],
                );
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

/// Fans a polygon from its first corner, after dropping a corner that repeats the one before it
/// (the pole a strip meets the axis at is one vertex, so its quad is a triangle).
fn add_polygon(mesh: &mut Mesh, corners: &[i32]) {
    let mut distinct: Vec<i32> = Vec::with_capacity(corners.len());
    for &corner in corners {
        if !distinct.contains(&corner) {
            distinct.push(corner);
        }
    }
    for k in 1..distinct.len().saturating_sub(1) {
        let (a, b, c) = (distinct[0], distinct[k], distinct[k + 1]);
        mesh.faces.add(a, b, c, &mesh.vertices);
    }
}

/// The flat face closing a partial sweep at one column (`row`, that column's vertices). The
/// strips walk the first column's loop edges backward and the last column's forward, so the start
/// cap walks them in loop order and the end cap backward, facing out of the solid either way.
///
/// The fill only joins the profile's own points. Should it not close exactly the loops' edges,
/// each loop is fanned from its first point instead, which walks every edge once whatever the
/// loop's shape, so the solid stays closed.
fn add_cap(mesh: &mut Mesh, profile: &Profile, row: &[i32], start: bool) {
    let oriented = |[a, b, c]: [usize; 3]| if start { [a, b, c] } else { [c, b, a] };
    let filled = fill_triangles(profile).map(|triangles| {
        triangles
            .into_iter()
            .map(|triangle| oriented(triangle).map(|point| row[point]))
            .collect::<Vec<_>>()
    });
    let boundary: Vec<(i32, i32)> = profile
        .loops
        .iter()
        .flat_map(|indices| {
            (0..indices.len()).map(move |i| (indices[i], indices[(i + 1) % indices.len()]))
        })
        .map(|(a, b)| {
            if start {
                (row[a], row[b])
            } else {
                (row[b], row[a])
            }
        })
        .collect();

    let faces = match filled.filter(|faces| walks_exactly(faces, &boundary)) {
        Some(faces) => faces,
        None => profile
            .loops
            .iter()
            .flat_map(|indices| {
                (1..indices.len().saturating_sub(1))
                    .map(|k| oriented([indices[0], indices[k], indices[k + 1]]).map(|p| row[p]))
            })
            .collect(),
    };
    for [a, b, c] in faces {
        if a != b && b != c && c != a {
            mesh.faces.add(a, b, c, &mesh.vertices);
        }
    }
}

/// Whether `faces` is a surface whose open boundary is exactly `boundary`: each of those directed
/// edges walked once and never back, every other edge walked once each way, and no face with a
/// repeated corner.
fn walks_exactly(faces: &[[i32; 3]], boundary: &[(i32, i32)]) -> bool {
    let mut walks: HashMap<(i32, i32), u32> = HashMap::new();
    for &[a, b, c] in faces {
        if a == b || b == c || c == a {
            return false;
        }
        for edge in [(a, b), (b, c), (c, a)] {
            *walks.entry(edge).or_insert(0) += 1;
        }
    }
    // the strips beside the cap walk its boundary the other way
    for &(a, b) in boundary {
        *walks.entry((b, a)).or_insert(0) += 1;
    }
    walks
        .iter()
        .all(|(&(a, b), &count)| count == 1 && walks.get(&(b, a)) == Some(&1))
}

/// The constrained Delaunay fill of the profile under the odd rule, counter clockwise, each corner
/// one of the profile's points; `None` when the tessellator fails, adds a point of its own (only
/// loops that cross need one) or leaves a triangle with no area. Delaunay refinement keeps the
/// triangles off the zero-area slivers a plain sweep leaves along straight runs of points.
fn fill_triangles(profile: &Profile) -> Option<Vec<[usize; 3]>> {
    let mut tess = Tessellator::new();
    tess.set_option(TessOption::ConstrainedDelaunayTriangulation, true);
    // the tessellator numbers its input points in the order they are added
    let mut input_points = Vec::new();
    for indices in &profile.loops {
        if indices.len() >= 3 {
            let coordinates: Vec<f64> = indices
                .iter()
                .flat_map(|&point| {
                    let (x, y) = profile.points[point];
                    [x as f64, y as f64]
                })
                .collect();
            tess.add_contour(2, &coordinates);
            input_points.extend_from_slice(indices);
        }
    }
    if input_points.is_empty() {
        return Some(Vec::new());
    }
    // the +z normal makes every triangle counter clockwise in 2D
    if !tess.tessellate(
        WindingRule::Odd,
        ElementType::Polygons,
        3,
        2,
        Some([0.0, 0.0, 1.0]),
    ) {
        return None;
    }

    let original = tess.vertex_indices();
    let mut triangles = Vec::new();
    for triangle in tess.elements().as_chunks::<3>().0 {
        if triangle.contains(&tess2_rust::tess::TESS_UNDEF) {
            continue;
        }
        let mut corners = [0; 3];
        for (corner, &output) in corners.iter_mut().zip(triangle) {
            let input = original[output as usize];
            if input == tess2_rust::tess::TESS_UNDEF {
                return None;
            }
            *corner = *input_points.get(input as usize)?;
        }
        if doubled_area(profile, corners) == 0 {
            return None;
        }
        triangles.push(corners);
    }
    Some(triangles)
}

/// Twice the signed area of a triangle of profile points, exactly.
fn doubled_area(profile: &Profile, [a, b, c]: [usize; 3]) -> i128 {
    let [a, b, c] = [a, b, c].map(|point| profile.points[point]);
    let (abx, aby) = ((b.0 - a.0) as i128, (b.1 - a.1) as i128);
    let (acx, acy) = ((c.0 - a.0) as i128, (c.1 - a.1) as i128);
    abx * acy - aby * acx
}
