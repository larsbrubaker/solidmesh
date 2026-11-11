//! UV charts of a textured operand: which of its pictured faces share one affine mapping from
//! surface to picture, so `manifold_uv_channel` may give them one face id.
//!
//! The kernel treats triangles with the same face id that are coplanar as one face: it
//! retriangulates the merged polygon in a boolean and collapses edges across it, carrying one
//! triangle's corner UVs onto its neighbours and interpolating between far ends of the face.
//! That is exact only when every triangle of the polygon maps the plane to the picture by the
//! same affine map. Two charts meeting on a flat side (an atlas seam), or one chart whose mapping
//! bends, must therefore keep separate ids, while the pieces a previous boolean cut one face into
//! (one map, rounded to f32) should share one so they merge back instead of growing the mesh at
//! every nested boolean.
//!
//! A chart grows from a seed face across shared edges. A face joins when it has the seed's
//! picture and colour, agrees bit for bit with the face it is reached from on the UVs at both
//! ends of their edge, and the seed's own map, extended over the seed's plane, gives each of its
//! corners' UVs, with each corner in that plane. Comparing every face with the seed rather than
//! with its neighbour is what keeps a mapping that bends a little at every edge from adding up
//! into one chart: every face of a chart lies within the tolerance of one map. Seeds are taken
//! largest face first (index order between equal areas), whose map extends furthest with the
//! least rounding, and each chart is named by its seed, so the grouping depends on no map order.

use std::collections::{HashMap, VecDeque};

use solidmesh_math::{Color, Vector3};

use crate::Mesh;

/// How far, in UV, a corner may lie from the seed's map and still be on it: four f32 steps at
/// the UV's size (UVs are stored as f32, and a face cut by a previous boolean has corners
/// rounded to f32 off the exact map). Every corner of a chart is within this of the seed's map,
/// so the kernel's interpolation across the chart moves a UV by at most about twice this.
const UV_STEPS: f64 = 4.0;

/// How far off the seed's plane, in f32 steps of the coordinates' size, a corner may lie and
/// still count as in it.
const PLANE_STEPS: f64 = 4.0;

/// For each face of `mesh`, the chart it belongs to (its seed's face index), or `None` for a
/// face with no picture. `uvs` gives each face's corner UVs as the kernel takes them, `picture`
/// its picture's identity and `colour_of` its colour.
pub(crate) fn uv_charts(
    mesh: &Mesh,
    uvs: impl Fn(usize) -> [[f64; 2]; 3],
    picture: impl Fn(usize) -> Option<usize>,
    colour_of: impl Fn(usize) -> Option<Color>,
) -> Vec<Option<usize>> {
    let face_count = mesh.faces.len();
    // Each face's data once: the growth below reads faces many times.
    let pictures: Vec<Option<usize>> = (0..face_count).map(picture).collect();
    let colours: Vec<Option<Color>> = (0..face_count).map(colour_of).collect();
    let face_uvs: Vec<[[f64; 2]; 3]> = (0..face_count)
        .map(|face| {
            if pictures[face].is_some() {
                uvs(face)
            } else {
                [[0.0; 2]; 3]
            }
        })
        .collect();
    let neighbours = edge_neighbours(mesh, &pictures);

    // Seeds largest first; the face index breaks ties, so the order is total.
    let areas: Vec<f64> = (0..face_count)
        .map(|face| {
            let p = corners(mesh, face);
            (p[1] - p[0]).cross(p[2] - p[0]).length()
        })
        .collect();
    let mut seeds: Vec<usize> = (0..face_count)
        .filter(|&face| pictures[face].is_some())
        .collect();
    seeds.sort_by(|&a, &b| areas[b].total_cmp(&areas[a]).then(a.cmp(&b)));

    let mut chart: Vec<Option<usize>> = vec![None; face_count];
    let mut queue = VecDeque::new();
    for seed in seeds {
        if chart[seed].is_some() {
            continue;
        }
        chart[seed] = Some(seed);
        // A sliver has no plane to extend: it is a chart of its own.
        let Some(map) = AffineMap::of(corners(mesh, seed), face_uvs[seed]) else {
            continue;
        };
        queue.clear();
        queue.push_back(seed);
        while let Some(face) = queue.pop_front() {
            for &(next, face_corner, next_corner) in neighbours[face].iter().flatten() {
                if chart[next].is_some()
                    || pictures[next] != pictures[seed]
                    || colours[next] != colours[seed]
                    || !same_edge_uvs(mesh, &face_uvs, face, face_corner, next, next_corner)
                {
                    continue;
                }
                let points = corners(mesh, next);
                let on_map = (0..3).all(|corner| map.gives(points[corner], face_uvs[next][corner]));
                if on_map {
                    chart[next] = Some(seed);
                    queue.push_back(next);
                }
            }
        }
    }
    chart
}

/// Per pictured face and corner, the pictured face across the edge that starts at that corner:
/// (that face, this face's corner, that face's corner the edge starts at). An edge used by more
/// than two pictured faces leads nowhere: which of them continues which is not known.
fn edge_neighbours(
    mesh: &Mesh,
    pictures: &[Option<usize>],
) -> Vec<[Option<(usize, usize, usize)>; 3]> {
    let face_count = mesh.faces.len();
    // Each undirected edge with the pictured faces using it: (face, corner the edge starts at)
    // for the first two, and whether more use it.
    let mut edges: HashMap<(i32, i32), EdgeUsers> = HashMap::with_capacity(face_count * 3 / 2);
    for (face, f) in mesh.faces.iter().enumerate() {
        if pictures[face].is_none() {
            continue;
        }
        let corners = [f.v0, f.v1, f.v2];
        for corner in 0..3 {
            let (a, b) = (corners[corner], corners[(corner + 1) % 3]);
            let user = (face, corner);
            edges
                .entry((a.min(b), a.max(b)))
                .and_modify(|users| {
                    if users.second.is_none() {
                        users.second = Some(user);
                    } else {
                        users.more = true;
                    }
                })
                .or_insert(EdgeUsers {
                    first: user,
                    second: None,
                    more: false,
                });
        }
    }
    // Filled per face slot, so the map's own order never reaches the result.
    let mut neighbours = vec![[None; 3]; face_count];
    for users in edges.values() {
        let (Some((second, second_corner)), false) = (users.second, users.more) else {
            continue;
        };
        let (first, first_corner) = users.first;
        neighbours[first][first_corner] = Some((second, first_corner, second_corner));
        neighbours[second][second_corner] = Some((first, second_corner, first_corner));
    }
    neighbours
}

/// The pictured faces using one edge.
struct EdgeUsers {
    first: (usize, usize),
    second: Option<(usize, usize)>,
    more: bool,
}

/// Whether faces `a` and `b`, sharing the edge that starts at corner `a_corner` of `a` and
/// `b_corner` of `b`, carry the same UV at both its ends, bit for bit (no seam between them).
fn same_edge_uvs(
    mesh: &Mesh,
    uvs: &[[[f64; 2]; 3]],
    a: usize,
    a_corner: usize,
    b: usize,
    b_corner: usize,
) -> bool {
    let a_vertices = vertex_indices(mesh, a);
    let b_vertices = vertex_indices(mesh, b);
    (0..2).all(|end| {
        let a_at = (a_corner + end) % 3;
        // The edge runs the other way round `b`, so its ends sit at `b_corner` + 1 and + 0.
        let b_at = (b_corner + 1 - end) % 3;
        let (left, right) = (uvs[a][a_at], uvs[b][b_at]);
        a_vertices[a_at] == b_vertices[b_at]
            && left[0].to_bits() == right[0].to_bits()
            && left[1].to_bits() == right[1].to_bits()
    })
}

/// A triangle's affine map from its plane to the picture, ready to extend to other points.
struct AffineMap {
    origin: Vector3,
    e1: Vector3,
    e2: Vector3,
    unit_normal: Vector3,
    d11: f64,
    d12: f64,
    d22: f64,
    det: f64,
    uvs: [[f64; 2]; 3],
    /// The largest coordinate of the triangle's corners, the size its plane tolerance scales by.
    scale: f64,
}

impl AffineMap {
    /// The map of the triangle `points` to `uvs`, or `None` for a sliver with no plane.
    fn of(points: [Vector3; 3], uvs: [[f64; 2]; 3]) -> Option<AffineMap> {
        let e1 = points[1] - points[0];
        let e2 = points[2] - points[0];
        let normal = e1.cross(e2);
        let normal_length = normal.length();
        if normal_length.partial_cmp(&0.0) != Some(std::cmp::Ordering::Greater) {
            return None;
        }
        let (d11, d12, d22) = (e1.dot(e1), e1.dot(e2), e2.dot(e2));
        let det = d11 * d22 - d12 * d12;
        if det.partial_cmp(&0.0) != Some(std::cmp::Ordering::Greater) {
            return None;
        }
        let scale = points.iter().fold(0.0f64, |scale, p| {
            scale.max(p.x.abs()).max(p.y.abs()).max(p.z.abs())
        });
        Some(AffineMap {
            origin: points[0],
            e1,
            e2,
            unit_normal: normal / normal_length,
            d11,
            d12,
            d22,
            det,
            uvs,
            scale,
        })
    }

    /// Whether `point` lies in the map's plane and the map gives `uv` there.
    fn gives(&self, point: Vector3, uv: [f64; 2]) -> bool {
        let f32_step = f64::from(f32::EPSILON);
        let q = point - self.origin;
        let scale = self
            .scale
            .max(point.x.abs())
            .max(point.y.abs())
            .max(point.z.abs());
        // A NaN corner fails this comparison and so never joins.
        let off_plane = q.dot(self.unit_normal).abs();
        if off_plane.is_nan() || off_plane > PLANE_STEPS * f32_step * scale {
            return false;
        }
        let (r1, r2) = (q.dot(self.e1), q.dot(self.e2));
        let s = (r1 * self.d22 - r2 * self.d12) / self.det;
        let t = (self.d11 * r2 - self.d12 * r1) / self.det;
        let uvs = &self.uvs;
        (0..2).all(|axis| {
            let predicted = uvs[0][axis]
                + s * (uvs[1][axis] - uvs[0][axis])
                + t * (uvs[2][axis] - uvs[0][axis]);
            let allowed = UV_STEPS * f32_step * predicted.abs().max(uv[axis].abs()).max(1.0);
            (predicted - uv[axis]).abs() <= allowed
        })
    }
}

fn corners(mesh: &Mesh, face: usize) -> [Vector3; 3] {
    vertex_indices(mesh, face).map(|v| {
        // Face indices name vertices of this mesh; the import refuses one that does not.
        usize::try_from(v)
            .ok()
            .and_then(|v| mesh.vertices.get(v))
            .map_or(Vector3::new(f64::NAN, f64::NAN, f64::NAN), |p| {
                p.as_vector3()
            })
    })
}

fn vertex_indices(mesh: &Mesh, face: usize) -> [i32; 3] {
    let f = &mesh.faces[face];
    [f.v0, f.v1, f.v2]
}
