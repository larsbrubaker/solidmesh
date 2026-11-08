//! The level-line test geometry the band cut and refinement tests read: the analytic fields, the
//! hand-built meshes, `run_cut` and the mesh measures, with the distance to a surface; plus the
//! coincident-vertex reading the degenerate-triangle test uses.

#![allow(dead_code)]

use std::cell::RefCell;
use std::collections::HashMap;
use std::rc::Rc;

use solidmesh::bevel::level_line::band_cut::cut;
use solidmesh::bevel::level_line::band_refinement::{refine, RefineOptions};
use solidmesh::bevel::level_line::edge_root::find;
use solidmesh::bevel::level_line::{
    BandCutResult, LevelLineMesh, LevelLineTriangle, ScalarField, VertexFieldReading, WorkError,
};
use solidmesh_math::{ieee_math, Matrix4X4, Vector3};

/// The signed distance to a plane.
pub struct PlaneField {
    normal: Vector3,
    offset: f64,
}

impl PlaneField {
    pub fn new(normal: Vector3, offset: f64) -> Self {
        PlaneField {
            normal: normal.get_normal(),
            offset,
        }
    }
}

impl ScalarField for PlaneField {
    fn value_at(&self, point: Vector3) -> Result<f64, WorkError> {
        Ok(point.dot(self.normal) - self.offset)
    }
}

/// The signed distance to a sphere.
pub struct BallField {
    centre: Vector3,
    radius: f64,
}

impl BallField {
    pub fn new(centre: Vector3, radius: f64) -> Self {
        BallField { centre, radius }
    }
}

impl ScalarField for BallField {
    fn value_at(&self, point: Vector3) -> Result<f64, WorkError> {
        Ok((point - self.centre).length() - self.radius)
    }
}

/// The signed distance to a capsule round a segment.
pub struct CapsuleField {
    from: Vector3,
    to: Vector3,
    radius: f64,
}

impl CapsuleField {
    pub fn new(from: Vector3, to: Vector3, radius: f64) -> Self {
        CapsuleField { from, to, radius }
    }
}

impl ScalarField for CapsuleField {
    fn value_at(&self, point: Vector3) -> Result<f64, WorkError> {
        let along = self.to - self.from;
        let length_squared = along.length_squared();
        let t = if length_squared <= 0.0 {
            0.0
        } else {
            ((point - self.from).dot(along) / length_squared).clamp(0.0, 1.0)
        };
        Ok((point - (self.from + (along * t))).length() - self.radius)
    }
}

/// A field carried along by a rigid motion.
pub struct MovedField {
    field: Rc<dyn ScalarField>,
    inverse: Matrix4X4,
}

impl MovedField {
    pub fn new(field: Rc<dyn ScalarField>, motion: Matrix4X4) -> Self {
        MovedField {
            field,
            inverse: motion.inverted(),
        }
    }
}

impl ScalarField for MovedField {
    fn value_at(&self, point: Vector3) -> Result<f64, WorkError> {
        self.field.value_at(point.transform(self.inverse))
    }
}

fn mesh_of(positions: &[Vector3], indices: &[(i32, i32, i32)]) -> LevelLineMesh {
    LevelLineMesh::from_triangles(positions, indices, None).expect("a mesh")
}

pub fn two_triangle_square(size: f64) -> LevelLineMesh {
    mesh_of(
        &[
            Vector3::new(0.0, 0.0, 0.0),
            Vector3::new(size, 0.0, 0.0),
            Vector3::new(size, size, 0.0),
            Vector3::new(0.0, size, 0.0),
        ],
        &[(0, 1, 2), (0, 2, 3)],
    )
}

pub fn subdivided_plane(n: i32, size: f64) -> LevelLineMesh {
    let mut positions = Vec::new();
    for j in 0..=n {
        for i in 0..=n {
            positions.push(Vector3::new(
                size * i as f64 / n as f64,
                size * j as f64 / n as f64,
                0.0,
            ));
        }
    }

    let mut indices = Vec::new();
    for j in 0..n {
        for i in 0..n {
            let a = (j * (n + 1)) + i;
            let b = a + 1;
            let c = b + n + 1;
            let d = a + n + 1;
            indices.push((a, b, c));
            indices.push((a, c, d));
        }
    }

    mesh_of(&positions, &indices)
}

pub fn cube(half: f64) -> LevelLineMesh {
    let (positions, indices) = cube_solid(half);
    mesh_of(&positions, &indices)
}

pub fn cube_solid(half: f64) -> (Vec<Vector3>, Vec<(i32, i32, i32)>) {
    let p = vec![
        Vector3::new(-half, -half, -half),
        Vector3::new(half, -half, -half),
        Vector3::new(half, half, -half),
        Vector3::new(-half, half, -half),
        Vector3::new(-half, -half, half),
        Vector3::new(half, -half, half),
        Vector3::new(half, half, half),
        Vector3::new(-half, half, half),
    ];
    let indices = vec![
        (0, 2, 1),
        (0, 3, 2),
        (4, 5, 6),
        (4, 6, 7),
        (0, 1, 5),
        (0, 5, 4),
        (1, 2, 6),
        (1, 6, 5),
        (2, 3, 7),
        (2, 7, 6),
        (3, 0, 4),
        (3, 4, 7),
    ];
    (p, indices)
}

pub fn cylinder(sides: i32, radius: f64, height: f64) -> LevelLineMesh {
    let (positions, indices) = cylinder_solid(sides, radius, height);
    mesh_of(&positions, &indices)
}

pub fn cylinder_solid(
    sides: i32,
    radius: f64,
    height: f64,
) -> (Vec<Vector3>, Vec<(i32, i32, i32)>) {
    let mut positions = Vec::new();
    for i in 0..sides {
        let a = 2.0 * std::f64::consts::PI * i as f64 / sides as f64;
        positions.push(Vector3::new(
            radius * a.cos(),
            radius * a.sin(),
            -height / 2.0,
        ));
    }

    for i in 0..sides {
        let a = 2.0 * std::f64::consts::PI * i as f64 / sides as f64;
        positions.push(Vector3::new(
            radius * a.cos(),
            radius * a.sin(),
            height / 2.0,
        ));
    }

    let bottom_centre = positions.len() as i32;
    positions.push(Vector3::new(0.0, 0.0, -height / 2.0));
    let top_centre = positions.len() as i32;
    positions.push(Vector3::new(0.0, 0.0, height / 2.0));

    let mut indices = Vec::new();
    for i in 0..sides {
        let next = (i + 1) % sides;
        indices.push((i, next, sides + next));
        indices.push((i, sides + next, sides + i));
        indices.push((bottom_centre, next, i));
        indices.push((top_centre, sides + i, sides + next));
    }

    (positions, indices)
}

/// Refines every triangle, finds the roots and cuts. The mesh is the reading's.
pub fn run_cut(
    mesh: LevelLineMesh,
    field: Rc<dyn ScalarField>,
    snap: f64,
    chord_tolerance: f64,
) -> (VertexFieldReading, BandCutResult, bool) {
    let triangle_count = mesh.triangle_count();
    let reading =
        VertexFieldReading::new(Rc::new(RefCell::new(mesh)), field, snap).expect("a snap");
    let candidates: Vec<i32> = (0..triangle_count).collect();
    let (refined, _) = refine(
        &reading,
        &candidates,
        chord_tolerance,
        RefineOptions::default(),
    )
    .expect("the refinement");
    let roots = find(&reading.mesh.borrow(), &reading).expect("the roots");
    let result = cut(&reading, &roots).expect("the cut");
    (reading, result, refined)
}

pub fn is_watertight(mesh: &LevelLineMesh) -> bool {
    mesh.edges()
        .iter()
        .all(|e| mesh.triangles_on(e.0, e.1).len() == 2)
}

pub fn signed_volume(mesh: &LevelLineMesh) -> f64 {
    let mut total = 0.0;
    for t in 0..mesh.triangle_count() {
        if mesh.is_removed(t) {
            continue;
        }

        let tri = mesh.triangles()[t as usize];
        total += mesh
            .position(tri.a)
            .dot(mesh.position(tri.b).cross(mesh.position(tri.c)));
    }

    total / 6.0
}

/// Over EVERY triangle in the list, removed ones included, so a removal that leaves its
/// triangle's area behind shows.
pub fn surface_area(mesh: &LevelLineMesh) -> f64 {
    mesh.triangles()
        .iter()
        .map(|&t| triangle_area(mesh, t))
        .sum()
}

pub fn triangle_area(mesh: &LevelLineMesh, t: LevelLineTriangle) -> f64 {
    let a = mesh.position(t.a);
    (mesh.position(t.b) - a)
        .cross(mesh.position(t.c) - a)
        .length()
        / 2.0
}

/// The smallest area of any triangle; a NaN area wins, so a degenerate one is never hidden.
pub fn smallest_triangle_area(mesh: &LevelLineMesh) -> f64 {
    mesh.triangles()
        .iter()
        .map(|&t| triangle_area(mesh, t))
        .fold(f64::INFINITY, |a, b| {
            if b.is_nan() || a.is_nan() {
                f64::NAN
            } else {
                ieee_math::min_f64(a, b)
            }
        })
}

pub fn shortest_side(mesh: &LevelLineMesh) -> f64 {
    mesh.edges()
        .iter()
        .map(|e| mesh.length(e.0, e.1))
        .fold(f64::INFINITY, ieee_math::min_f64)
}

/// Every point the level line stands on - its vertices, and the midpoint of each
/// of its segments.
pub fn line_points(mesh: &LevelLineMesh, result: &BandCutResult) -> Vec<Vector3> {
    let mut points = Vec::new();
    for chain in &result.lines {
        let count = chain.vertices.len();
        for i in 0..count {
            points.push(mesh.position(chain.vertices[i]));
            if i + 1 < count || chain.closed {
                let next = chain.vertices[(i + 1) % count];
                points.push(0.5 * (mesh.position(chain.vertices[i]) + mesh.position(next)));
            }
        }
    }

    points
}

/// Every pair of vertices within `snap` of each other,
/// found by a bucket per snap cube.
pub fn coincident_vertices(mesh: &LevelLineMesh, snap: f64) -> Vec<(i32, i32, Vector3)> {
    let mut found = Vec::new();
    let mut buckets: HashMap<(i64, i64, i64), Vec<i32>> = HashMap::new();
    for v in 0..mesh.vertex_count() {
        let p = mesh.position(v);
        let key = (
            (p.x / snap).floor() as i64,
            (p.y / snap).floor() as i64,
            (p.z / snap).floor() as i64,
        );
        for dx in -1..=1 {
            for dy in -1..=1 {
                for dz in -1..=1 {
                    let near = (key.0 + dx, key.1 + dy, key.2 + dz);
                    if let Some(here) = buckets.get(&near) {
                        for &other in here {
                            if (mesh.position(other) - p).length() <= snap {
                                found.push((other, v, p));
                            }
                        }
                    }
                }
            }
        }

        buckets.entry(key).or_default().push(v);
    }

    found
}

/// Every edge of the mesh with only one triangle on it.
pub fn open_edges(mesh: &LevelLineMesh) -> Vec<(i32, i32)> {
    mesh.edges()
        .into_iter()
        .filter(|e| mesh.triangles_on(e.0, e.1).len() == 1)
        .collect()
}

/// The volume counting EVERY triangle the mesh holds, the
/// band's removed ones included - the reading a refinement test wants, where the claim is that the
/// cut only SPLIT. Never the reading for a fill test.
pub fn volume_of_every_triangle_even_removed(mesh: &LevelLineMesh) -> f64 {
    let mut total = 0.0;
    for t in mesh.triangles() {
        let a = mesh.position(t.a);
        let b = mesh.position(t.b);
        let c = mesh.position(t.c);
        total += a.dot(b.cross(c));
    }

    total / 6.0
}

/// The enclosed volume of bare positions and index triples.
pub fn signed_volume_of(positions: &[Vector3], indices: &[(i32, i32, i32)]) -> f64 {
    let mut total = 0.0;
    for &(a, b, c) in indices {
        total += positions[a as usize].dot(positions[b as usize].cross(positions[c as usize]));
    }

    total / 6.0
}

/// How many turns the triangles subtend at a point - one inside a closed solid,
/// none outside, a half on its surface. Needs nothing of the solid but its own triangles, so it
/// reads a box with a hole through it as readily as a cube.
pub fn winding_number(positions: &[Vector3], indices: &[(i32, i32, i32)], point: Vector3) -> f64 {
    let mut total = 0.0;
    for &(ia, ib, ic) in indices {
        let a = positions[ia as usize] - point;
        let b = positions[ib as usize] - point;
        let c = positions[ic as usize] - point;
        let la = a.length();
        let lb = b.length();
        let lc = c.length();
        if la == 0.0 || lb == 0.0 || lc == 0.0 {
            // The point is a corner of the solid; the solid angle there is not defined by this
            // formula.
            return 0.5;
        }

        // Van Oosterom and Strackee: the signed solid angle of a triangle seen from the origin.
        let numerator = a.dot(b.cross(c));
        let denominator = (la * lb * lc) + (a.dot(b) * lc) + (b.dot(c) * la) + (c.dot(a) * lb);
        total += 2.0 * numerator.atan2(denominator);
    }

    total / (4.0 * std::f64::consts::PI)
}

/// How far a point stands from the nearest of the given triangles.
pub fn distance_to_surface(
    positions: &[Vector3],
    indices: &[(i32, i32, i32)],
    point: Vector3,
) -> f64 {
    let mut least = f64::INFINITY;
    for &(ia, ib, ic) in indices {
        least = least.min(distance_to_triangle(
            positions[ia as usize],
            positions[ib as usize],
            positions[ic as usize],
            point,
        ));
    }

    least
}

fn distance_to_triangle(a: Vector3, b: Vector3, c: Vector3, point: Vector3) -> f64 {
    // The foot of the perpendicular in barycentric coordinates, then clamped onto the triangle by
    // taking the least distance to its three edges when that foot falls outside.
    let ab = b - a;
    let ac = c - a;
    let normal = ab.cross(ac);
    let area = normal.length();
    if area > 0.0 {
        let foot = point - (normal * (normal.dot(point - a) / (area * area)));
        let u = normal.dot((b - foot).cross(c - foot)) / (area * area);
        let v = normal.dot((c - foot).cross(a - foot)) / (area * area);
        let w = 1.0 - u - v;
        if u >= 0.0 && v >= 0.0 && w >= 0.0 {
            return (point - foot).length();
        }
    }

    f64::min(
        distance_to_segment(a, b, point),
        f64::min(
            distance_to_segment(b, c, point),
            distance_to_segment(c, a, point),
        ),
    )
}

fn distance_to_segment(a: Vector3, b: Vector3, point: Vector3) -> f64 {
    let along = b - a;
    let squared = along.length_squared();
    let t = if squared > 0.0 {
        (along.dot(point - a) / squared).clamp(0.0, 1.0)
    } else {
        0.0
    };
    (point - (a + (along * t))).length()
}
