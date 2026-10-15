//! The points a mesh's transformed bounds are measured over: every mesh vertex on or outside its
//! convex hull (`mesh_convex_hull`). `TransformedAabbCache` (`crate::transformed_aabb_cache`)
//! transforms these instead of every vertex, which is what keeps per-transform bounds fast.
//!
//! The hull is built with a distance tolerance, so it can leave out a vertex that lies just
//! outside one of its faces. Measuring bounds over the hull's own vertices would then miss that
//! vertex, and two meshes of the same solid with other vertices or another vertex order would get
//! bounds a few microns apart. Every vertex that is not strictly inside the hull (by a margin far
//! above rounding) is kept here, so the bounds of these points are the bounds of all the vertices:
//! a vertex dropped from this set is inside the hull by more than any transform's rounding, so it
//! can never be the extreme along any axis.

use vector_math::{Vector3, Vector3Float};

use crate::Mesh;

/// A region holding this many points or fewer is tested point by point.
const LEAF_POINTS: usize = 64;

/// The margin, relative to the mesh's largest coordinate, inside which a vertex counts as on the
/// hull. It is far above the rounding of a plane test or a transform (about 1e-15 relative) and
/// far below anything a user can see.
const RELATIVE_MARGIN: f64 = 1e-9;

/// One hull face's plane: `normal . p - offset` is positive outside, and a point counts as on or
/// outside when it is above `-slack` (the margin scaled by the unnormalized normal's length).
struct HullPlane {
    normal: Vector3,
    offset: f64,
    slack: f64,
}

impl HullPlane {
    fn reaches(&self, p: Vector3) -> bool {
        self.normal.dot(p) - self.offset > -self.slack
    }
}

/// Every vertex of `mesh` on or outside `hull` (within the margin), in the mesh's vertex order.
/// `hull` must be a closed hull of `mesh`'s vertices, as `Mesh::get_convex_hull` builds.
pub(crate) fn points_on_or_outside_hull(mesh: &Mesh, hull: &Mesh) -> Vec<Vector3Float> {
    let hull_vertices: Vec<Vector3> = hull
        .vertices
        .iter()
        .map(|&v| Vector3::from_vector3_float(v))
        .collect();
    if hull_vertices.is_empty() || hull.faces.is_empty() {
        return mesh.vertices.clone();
    }

    // A point strictly inside the hull, to orient each face's plane outward whatever the face's
    // winding.
    let mut inside = Vector3::ZERO;
    for &v in &hull_vertices {
        inside += v;
    }
    inside /= hull_vertices.len() as f64;

    let mut largest_coordinate: f64 = 0.0;
    for v in mesh.vertices.iter() {
        let biggest = (v.x.abs()).max(v.y.abs()).max(v.z.abs()) as f64;
        largest_coordinate = largest_coordinate.max(biggest);
    }
    let margin = RELATIVE_MARGIN * largest_coordinate.max(1.0);

    let mut planes = Vec::with_capacity(hull.faces.len());
    for face in hull.faces.iter() {
        let corners = [face.v0, face.v1, face.v2].map(|i| hull_vertices.get(i as usize).copied());
        let [Some(a), Some(b), Some(c)] = corners else {
            // a hull that does not index its own vertices cannot bound anything: keep every vertex
            return mesh.vertices.clone();
        };
        let mut normal = (b - a).cross(c - a);
        let length = normal.length();
        if length == 0.0 || !length.is_finite() {
            // a degenerate sliver bounds nothing; its neighbours carry the plane
            continue;
        }
        if normal.dot(inside) - normal.dot(a) > 0.0 {
            normal = -normal;
        }
        planes.push(HullPlane {
            normal,
            offset: normal.dot(a),
            slack: margin * length,
        });
    }
    if planes.is_empty() {
        return mesh.vertices.clone();
    }

    let points: Vec<Vector3> = mesh
        .vertices
        .iter()
        .map(|&v| Vector3::from_vector3_float(v))
        .collect();
    let mut keep = vec![false; points.len()];
    let mut indices: Vec<usize> = (0..points.len()).collect();
    let all_planes: Vec<usize> = (0..planes.len()).collect();
    mark_reaching(&points, &mut indices, &planes, &all_planes, &mut keep);

    mesh.vertices
        .iter()
        .zip(keep)
        .filter_map(|(&v, kept)| kept.then_some(v))
        .collect()
}

/// Marks in `keep` each point of `indices` that reaches one of `candidates`, splitting the
/// points' box in half along its longest side so each half tests only the planes its box can
/// reach: deep inside the hull no plane reaches the box and the whole region is skipped.
fn mark_reaching(
    points: &[Vector3],
    indices: &mut [usize],
    planes: &[HullPlane],
    candidates: &[usize],
    keep: &mut [bool],
) {
    let mut min = Vector3::new(f64::MAX, f64::MAX, f64::MAX);
    let mut max = Vector3::new(f64::MIN, f64::MIN, f64::MIN);
    for &i in indices.iter() {
        let p = points[i];
        min = Vector3::new(min.x.min(p.x), min.y.min(p.y), min.z.min(p.z));
        max = Vector3::new(max.x.max(p.x), max.y.max(p.y), max.z.max(p.z));
    }
    // a plane reaches the box when the box's corner furthest along the plane's normal does
    let reaching: Vec<usize> = candidates
        .iter()
        .copied()
        .filter(|&f| {
            let n = planes[f].normal;
            let corner = Vector3::new(
                if n.x > 0.0 { max.x } else { min.x },
                if n.y > 0.0 { max.y } else { min.y },
                if n.z > 0.0 { max.z } else { min.z },
            );
            planes[f].reaches(corner)
        })
        .collect();
    if reaching.is_empty() {
        return;
    }

    let size = max - min;
    if indices.len() <= LEAF_POINTS || !(size.x > 0.0 || size.y > 0.0 || size.z > 0.0) {
        for &i in indices.iter() {
            if reaching.iter().any(|&f| planes[f].reaches(points[i])) {
                keep[i] = true;
            }
        }
        return;
    }

    let axis = if size.x >= size.y && size.x >= size.z {
        0
    } else if size.y >= size.z {
        1
    } else {
        2
    };
    let coordinate = |p: Vector3| match axis {
        0 => p.x,
        1 => p.y,
        _ => p.z,
    };
    let middle = (coordinate(min) + coordinate(max)) * 0.5;
    // partition in place: points below the middle first
    let mut split = 0;
    for k in 0..indices.len() {
        if coordinate(points[indices[k]]) < middle {
            indices.swap(k, split);
            split += 1;
        }
    }
    if split == 0 || split == indices.len() {
        // every point on one side (they share the coordinate): test them directly
        for &i in indices.iter() {
            if reaching.iter().any(|&f| planes[f].reaches(points[i])) {
                keep[i] = true;
            }
        }
        return;
    }
    let (low, high) = indices.split_at_mut(split);
    mark_reaching(points, low, planes, &reaching, keep);
    mark_reaching(points, high, planes, &reaching, keep);
}
