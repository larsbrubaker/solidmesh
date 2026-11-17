//! Mesh distance: the true distance between two meshes' surfaces,
//! rather than between their bounding boxes, which overlap long before the parts do (a ball
//! resting near a tilted part is a box overlap and a real gap). It lives here rather than in
//! mattercad-model because it is built on the crate's `g3` pieces (the AABB tree's
//! nearest-pair search in `dmesh_aabb_tree3_pairs`, the triangle-triangle distance in
//! `dist_triangle3_triangle3`, and the fast winding number in `fast_winding`), and on the boolean
//! kernel (`csg`) for telling a touch from a crossing. The AI tool's `measure` reads it;
//! `point_distances` answers point-to-surface distances over the same tree, and
//! `point_triangle_distance_sqr` exposes the single point-triangle distance.
//!
//! `surface_deviation` measures how far one surface strays from another, both ways, on
//! `point_distances`: a caller that simplifies a mesh (`Mesh::reduced_to_triangle_count`)
//! refuses a result that drifted too far from the original. Each surface is sampled on a fixed
//! barycentric lattice per face, fine enough that no lattice cell is wider than the spacing the
//! sample budget allows, so every vertex and edge is sampled and the samples fall evenly over
//! the area. The two directions' samples are pooled, each weighted by the area it stands for,
//! giving the largest distance and an area-weighted percentile. Samples follow the faces'
//! order and the percentile's sort breaks ties by sample index, so the same two meshes give
//! the same numbers to the bit.

use solidmesh_math::{Matrix4X4, Vector3};

use crate::csg::{boolean_processing, BooleanError, BooleanOptions, CsgModes};
use crate::g3::dist_triangle3_triangle3::dist_triangle3_triangle3;
use crate::g3::dmesh3::{tri_distance_sqr, DMesh3};
use crate::g3::dmesh_aabb_tree3::DMeshAABBTree3;
use crate::g3::fast_winding::FastWinding;
use crate::{CancelToken, Mesh};

/// How far apart two meshes' surfaces are and whether the solids share
/// volume.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct MeshDistanceResult {
    /// The distance between the surfaces (0 when they meet).
    pub surface_distance: f64,
    /// The point of the first surface nearest the second.
    pub nearest_on_a: Vector3,
    /// The point of the second surface nearest the first.
    pub nearest_on_b: Vector3,
    /// The solids share volume: their surfaces cross, or a body of one lies inside the
    /// other. Touching is not overlap.
    pub overlap: bool,
    /// Every body of the first solid lies inside the second (surfaces apart).
    pub a_inside_b: bool,
    /// Every body of the second solid lies inside the first (surfaces apart).
    pub b_inside_a: bool,
    /// At least one body of the first solid lies inside the second (surfaces apart); true
    /// whenever `a_inside_b` is.
    pub a_partly_inside_b: bool,
    /// At least one body of the second solid lies inside the first (surfaces apart); true
    /// whenever `b_inside_a` is.
    pub b_partly_inside_a: bool,
}

/// Surfaces closer than this (mm) meet; the meshes store float vertices,
/// so coincident faces land within it.
pub const CONTACT_TOLERANCE: f64 = 1e-5;

/// A shared volume at or under this (mm³) is a touch, not an
/// overlap.
const OVERLAP_VOLUME_TOLERANCE: f64 = 1e-4;

/// Measures `a` against `b` with one AABB tree each (the trees' nearest
/// triangle pair search), so large meshes answer quickly. Containment is a winding-number test of
/// one corner of each body (each connected surface; with the surfaces apart, a body with one
/// corner inside the other solid lies wholly inside it), which assumes closed meshes; surfaces
/// that meet are told apart from surfaces that cross by the
/// volume of their boolean intersection. `None` when either mesh has no faces (there is no
/// surface to measure). A cancelled token is `Err(Cancelled)`.
pub fn between(
    a: &Mesh,
    b: &Mesh,
    cancel: &CancelToken,
) -> Result<Option<MeshDistanceResult>, BooleanError> {
    if a.faces.is_empty() || b.faces.is_empty() {
        return Ok(None);
    }
    let soup_a = triangle_soup(a);
    let soup_b = triangle_soup(b);
    let tree_a = DMeshAABBTree3::new(&soup_a);
    let tree_b = DMeshAABBTree3::new(&soup_b);

    let Some(((ta, tb), distance)) = tree_a.find_nearest_triangles(&tree_b) else {
        return Ok(None);
    };
    let nearest = dist_triangle3_triangle3(
        &soup_a.get_tri_vertices(ta as usize),
        &soup_b.get_tri_vertices(tb as usize),
    );
    let on_a = to_vector3(nearest.triangle0_closest);
    let on_b = to_vector3(nearest.triangle1_closest);

    if distance > CONTACT_TOLERANCE {
        // Separate surfaces: the solids share volume only where a body of one lies inside the
        // other, and then every corner of that body is inside, so one corner per body tells.
        let (a_inside_b, a_partly_inside_b) = bodies_inside(a, &FastWinding::new(&tree_b));
        let (b_inside_a, b_partly_inside_a) = bodies_inside(b, &FastWinding::new(&tree_a));
        return Ok(Some(MeshDistanceResult {
            surface_distance: distance,
            nearest_on_a: on_a,
            nearest_on_b: on_b,
            overlap: a_partly_inside_b || b_partly_inside_a,
            a_inside_b,
            b_inside_a,
            a_partly_inside_b,
            b_partly_inside_a,
        }));
    }

    Ok(Some(MeshDistanceResult {
        surface_distance: 0.0,
        nearest_on_a: on_a,
        nearest_on_b: on_b,
        overlap: surfaces_cross(a, b, cancel)?,
        a_inside_b: false,
        b_inside_a: false,
        a_partly_inside_b: false,
        b_partly_inside_a: false,
    }))
}

/// Whether every body of `mesh` lies inside the solid `other` winds around, and whether any does:
/// one corner of each body tested by its winding number.
fn bodies_inside(mesh: &Mesh, other: &FastWinding) -> (bool, bool) {
    let (mut all, mut any) = (true, false);
    for corner in body_corners(mesh) {
        let v = mesh.vertices[corner];
        let inside = other.fast_winding_number([v.x as f64, v.y as f64, v.z as f64]) > 0.5;
        all &= inside;
        any |= inside;
    }
    (all && any, any)
}

/// One corner index of each body of `mesh`, in index order: faces joined through shared corners,
/// a corner matched by position as well as by index so a body whose faces carry their own copies
/// of a corner is still one body.
fn body_corners(mesh: &Mesh) -> Vec<usize> {
    fn root(parent: &mut [usize], mut at: usize) -> usize {
        while parent[at] != at {
            parent[at] = parent[parent[at]];
            at = parent[at];
        }
        at
    }
    fn join(parent: &mut [usize], a: usize, b: usize) {
        let (a, b) = (root(parent, a), root(parent, b));
        if a != b {
            parent[a.max(b)] = a.min(b);
        }
    }

    let count = mesh.vertices.len();
    let mut parent: Vec<usize> = (0..count).collect();
    // Exact bits, sorted with the index as tie-break, so nothing depends on hash order;
    // `+ 0.0` makes -0.0 and 0.0 one position.
    let mut by_position: Vec<(u32, u32, u32, usize)> = mesh
        .vertices
        .iter()
        .enumerate()
        .map(|(index, v)| {
            let bits = |axis: f32| (axis + 0.0).to_bits();
            (bits(v.x), bits(v.y), bits(v.z), index)
        })
        .collect();
    by_position.sort_unstable();
    for pair in by_position.windows(2) {
        if pair[0].0 == pair[1].0 && pair[0].1 == pair[1].1 && pair[0].2 == pair[1].2 {
            join(&mut parent, pair[0].3, pair[1].3);
        }
    }
    let mut used = vec![false; count];
    for face in mesh.faces.iter() {
        let corners = [face.v0, face.v1, face.v2].map(|v| v as usize);
        join(&mut parent, corners[0], corners[1]);
        join(&mut parent, corners[1], corners[2]);
        for corner in corners {
            used[corner] = true;
        }
    }
    // The first corner a face uses in each body: a corner no face uses may share a body's
    // position, but it is no corner of the body.
    let mut seen = vec![false; count];
    let mut corners = Vec::new();
    for (corner, &used) in used.iter().enumerate() {
        if used {
            let body = root(&mut parent, corner);
            if !seen[body] {
                seen[body] = true;
                corners.push(corner);
            }
        }
    }
    corners
}

/// The distance from each of `points` to the nearest point of `mesh`'s surface, through one AABB
/// tree (the nearest triangle, then the exact distance to it, as the wall thickness checks
/// measure a hollowed part). `f64::MAX` for every point when the mesh has no
/// faces.
pub fn point_distances(mesh: &Mesh, points: &[Vector3]) -> Vec<f64> {
    if mesh.faces.is_empty() {
        return vec![f64::MAX; points.len()];
    }
    let soup = triangle_soup(mesh);
    let tree = DMeshAABBTree3::new(&soup);
    points
        .iter()
        .map(|p| {
            let query = [p.x, p.y, p.z];
            let tid = tree.find_nearest_triangle(query, f64::MAX);
            if tid < 0 {
                return f64::MAX;
            }
            tri_distance_sqr(&soup.get_tri_vertices(tid as usize), query).sqrt()
        })
        .collect()
}

/// The squared distance from `point` to the triangle `triangle` (Eberly's region walk). A
/// zero-area triangle answers the distance to the nearest of its edges.
pub fn point_triangle_distance_sqr(triangle: &[[f64; 3]; 3], point: [f64; 3]) -> f64 {
    tri_distance_sqr(triangle, point)
}

/// Surfaces that meet either just touch (a part resting on another) or
/// cross; only crossing leaves a shared volume. A mesh the boolean kernel refuses cannot be told
/// apart, so meeting counts as overlap.
fn surfaces_cross(a: &Mesh, b: &Mesh, cancel: &CancelToken) -> Result<bool, BooleanError> {
    // Polygon processing at the default resolution: the overlap only needs a volume.
    let shared = boolean_processing::do_array(
        &[(a, Matrix4X4::IDENTITY), (b, Matrix4X4::IDENTITY)],
        CsgModes::Intersect,
        cancel,
        &BooleanOptions::default(),
    );
    match shared {
        Ok(shared) => {
            Ok(!shared.faces.is_empty() && shared.get_volume().abs() > OVERLAP_VOLUME_TOLERANCE)
        }
        Err(BooleanError::Cancelled) => Err(BooleanError::Cancelled),
        Err(_) => Ok(true),
    }
}

/// Every triangle with its own three vertices. A `DMesh3` refuses triangles
/// that would make an edge non-manifold, and exported meshes can have those; a soup keeps every
/// triangle for the distance, and the winding number does not need connectivity.
fn triangle_soup(mesh: &Mesh) -> DMesh3 {
    let mut soup = DMesh3::default();
    let vertex = |index: i32| {
        let v = mesh.vertices[index as usize];
        [v.x as f64, v.y as f64, v.z as f64]
    };
    for face in mesh.faces.iter() {
        let v0 = soup.append_vertex(vertex(face.v0));
        let v1 = soup.append_vertex(vertex(face.v1));
        let v2 = soup.append_vertex(vertex(face.v2));
        soup.append_triangle(v0, v1, v2);
    }
    soup
}

fn to_vector3(v: [f64; 3]) -> Vector3 {
    Vector3::new(v[0], v[1], v[2])
}

/// Samples per surface `surface_deviation` callers use unless they need another budget.
pub const DEFAULT_SAMPLES_PER_SURFACE: usize = 200_000;

/// The two-way deviation between two surfaces.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct SurfaceDeviation {
    /// The largest distance from a sample of either surface to the other surface.
    pub max: f64,
    /// The area-weighted percentile of the pooled sample distances, at `percentile`.
    pub percentile_distance: f64,
    /// The percentile asked for, in `0.0..=1.0` (0.99 for the 99th).
    pub percentile: f64,
    /// The largest side of the first mesh's axis-aligned bounding box, the scale the fractions
    /// are taken against.
    pub scale: f64,
}

impl SurfaceDeviation {
    /// `max` as a fraction of the first mesh's largest bounding-box side (0 for a mesh with
    /// no extent).
    pub fn max_fraction(&self) -> f64 {
        if self.scale > 0.0 {
            self.max / self.scale
        } else {
            0.0
        }
    }

    /// `percentile_distance` as a fraction of the first mesh's largest bounding-box side.
    pub fn percentile_fraction(&self) -> f64 {
        if self.scale > 0.0 {
            self.percentile_distance / self.scale
        } else {
            0.0
        }
    }
}

/// The two-way deviation between `a` and `b`, sampling each surface with about
/// `samples_per_surface` points (more when it has more faces: every face's corners are
/// sampled) and reporting the area-weighted `percentile` (clamped to `0.0..=1.0`). A mesh
/// with no faces is infinitely far from one with faces; two empty meshes are zero apart.
pub fn surface_deviation(
    a: &Mesh,
    b: &Mesh,
    samples_per_surface: usize,
    percentile: f64,
) -> SurfaceDeviation {
    let percentile = percentile.clamp(0.0, 1.0);
    let scale = largest_side(a);
    if a.faces.is_empty() || b.faces.is_empty() {
        let d = if a.faces.is_empty() && b.faces.is_empty() {
            0.0
        } else {
            f64::INFINITY
        };
        return SurfaceDeviation {
            max: d,
            percentile_distance: d,
            percentile,
            scale,
        };
    }
    let (points_a, mut weights) = surface_samples(a, samples_per_surface);
    let (points_b, weights_b) = surface_samples(b, samples_per_surface);
    let mut distances = point_distances(b, &points_a);
    distances.extend(point_distances(a, &points_b));
    weights.extend(weights_b);

    let mut max = 0.0f64;
    let mut total_weight = 0.0f64;
    for (&d, &w) in distances.iter().zip(&weights) {
        if d > max {
            max = d;
        }
        total_weight += w;
    }
    let mut order: Vec<usize> = (0..distances.len()).collect();
    order.sort_by(|&i, &j| distances[i].total_cmp(&distances[j]).then(i.cmp(&j)));
    let wanted = percentile * total_weight;
    let mut reached = 0.0f64;
    let mut percentile_distance = max;
    for &i in &order {
        reached += weights[i];
        if reached >= wanted {
            percentile_distance = distances[i];
            break;
        }
    }
    SurfaceDeviation {
        max,
        percentile_distance,
        percentile,
        scale,
    }
}

/// The lattice samples of `mesh`'s faces and the area each stands for.
fn surface_samples(mesh: &Mesh, budget: usize) -> (Vec<Vector3>, Vec<f64>) {
    let corners = |face: &crate::Face| {
        [face.v0, face.v1, face.v2].map(|i| {
            let v = mesh.vertices[i as usize];
            [v.x as f64, v.y as f64, v.z as f64]
        })
    };
    let mut total_area = 0.0f64;
    for face in mesh.faces.iter() {
        total_area += triangle_area(&corners(face));
    }
    // The lattice spacing at which `budget` evenly spread samples cover the area.
    let spacing = if budget > 0 && total_area > 0.0 {
        (2.0 * total_area / budget as f64).sqrt()
    } else {
        f64::INFINITY
    };
    let mut points = Vec::new();
    let mut weights = Vec::new();
    for face in mesh.faces.iter() {
        let tri = corners(face);
        let longest = edge_length(tri[0], tri[1])
            .max(edge_length(tri[1], tri[2]))
            .max(edge_length(tri[2], tri[0]));
        let steps = if spacing.is_finite() {
            ((longest / spacing).ceil() as usize).clamp(1, 1024)
        } else {
            1
        };
        let weight = triangle_area(&tri) / ((steps + 1) * (steps + 2) / 2) as f64;
        for i in 0..=steps {
            for j in 0..=(steps - i) {
                let k = steps - i - j;
                let (u, v, w) = (
                    i as f64 / steps as f64,
                    j as f64 / steps as f64,
                    k as f64 / steps as f64,
                );
                points.push(Vector3::new(
                    u * tri[0][0] + v * tri[1][0] + w * tri[2][0],
                    u * tri[0][1] + v * tri[1][1] + w * tri[2][1],
                    u * tri[0][2] + v * tri[1][2] + w * tri[2][2],
                ));
                weights.push(weight);
            }
        }
    }
    (points, weights)
}

fn largest_side(mesh: &Mesh) -> f64 {
    let mut lo = [f64::INFINITY; 3];
    let mut hi = [f64::NEG_INFINITY; 3];
    for v in &mesh.vertices {
        for (axis, c) in [v.x as f64, v.y as f64, v.z as f64].into_iter().enumerate() {
            lo[axis] = lo[axis].min(c);
            hi[axis] = hi[axis].max(c);
        }
    }
    (0..3)
        .map(|axis| hi[axis] - lo[axis])
        .filter(|side| side.is_finite())
        .fold(0.0, f64::max)
}

fn edge_length(a: [f64; 3], b: [f64; 3]) -> f64 {
    let d = [a[0] - b[0], a[1] - b[1], a[2] - b[2]];
    (d[0] * d[0] + d[1] * d[1] + d[2] * d[2]).sqrt()
}

fn triangle_area(t: &[[f64; 3]; 3]) -> f64 {
    let e1 = [t[1][0] - t[0][0], t[1][1] - t[0][1], t[1][2] - t[0][2]];
    let e2 = [t[2][0] - t[0][0], t[2][1] - t[0][1], t[2][2] - t[0][2]];
    let c = [
        e1[1] * e2[2] - e1[2] * e2[1],
        e1[2] * e2[0] - e1[0] * e2[2],
        e1[0] * e2[1] - e1[1] * e2[0],
    ];
    0.5 * (c[0] * c[0] + c[1] * c[1] + c[2] * c[2]).sqrt()
}
