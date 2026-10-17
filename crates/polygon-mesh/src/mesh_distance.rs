//! Mesh distance: the true distance between two meshes' surfaces,
//! rather than between their bounding boxes, which overlap long before the parts do (a ball
//! resting near a tilted part is a box overlap and a real gap). It lives here rather than in
//! mattercad-model because it is built on the crate's `g3` pieces (the AABB tree's
//! nearest-pair search in `dmesh_aabb_tree3_pairs`, the triangle-triangle distance in
//! `dist_triangle3_triangle3`, and the fast winding number in `fast_winding`), and on the boolean
//! kernel (`csg`) for telling a touch from a crossing. The AI tool's `measure` reads it;
//! `point_distances` answers point-to-surface distances over the same tree, and
//! `point_triangle_distance_sqr` exposes the single point-triangle distance.

use vector_math::{Matrix4X4, Vector3};

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
    /// The solids share volume: their surfaces cross, or one lies wholly inside the
    /// other. Touching is not overlap.
    pub overlap: bool,
    /// The first solid lies wholly inside the second (surfaces apart).
    pub a_inside_b: bool,
    /// The second solid lies wholly inside the first (surfaces apart).
    pub b_inside_a: bool,
}

/// Surfaces closer than this (mm) meet; the meshes store float vertices,
/// so coincident faces land within it.
pub const CONTACT_TOLERANCE: f64 = 1e-5;

/// A shared volume at or under this (mm³) is a touch, not an
/// overlap.
const OVERLAP_VOLUME_TOLERANCE: f64 = 1e-4;

/// Measures `a` against `b` with one AABB tree each (the trees' nearest
/// triangle pair search), so large meshes answer quickly. Containment is a winding-number test,
/// which assumes closed meshes; surfaces that meet are told apart from surfaces that cross by the
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
        // Separate surfaces: the solids share volume only if one holds the other, and then every
        // vertex of the inner one is inside, so testing one is enough.
        let a_inside_b = FastWinding::new(&tree_b).fast_winding_number(soup_a.get_vertex(0)) > 0.5;
        let b_inside_a = !a_inside_b
            && FastWinding::new(&tree_a).fast_winding_number(soup_b.get_vertex(0)) > 0.5;
        return Ok(Some(MeshDistanceResult {
            surface_distance: distance,
            nearest_on_a: on_a,
            nearest_on_b: on_b,
            overlap: a_inside_b || b_inside_a,
            a_inside_b,
            b_inside_a,
        }));
    }

    Ok(Some(MeshDistanceResult {
        surface_distance: 0.0,
        nearest_on_a: on_a,
        nearest_on_b: on_b,
        overlap: surfaces_cross(a, b, cancel)?,
        a_inside_b: false,
        b_inside_a: false,
    }))
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
