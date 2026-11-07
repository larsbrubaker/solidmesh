//! The source part's convex hull, padded a hair outward - the bound every bead is kept inside so it
//! adds no material past the part's own extent: building it ([`build`]) and asking whether a solid
//! already lies inside it ([`lies_inside`]).

use polygon_mesh::processors::ConvexHull;
use polygon_mesh::Mesh;
use vector_math::{Vector3, Vector3Float};

use super::bevel_working_mesh;

/// How far the hull is padded outward, as a fraction of the source's diagonal.
pub const PAD_FRACTION: f64 = 1e-5;

/// True when every vertex of `solid` stands on the inner side of every face plane of the convex
/// `hull` (a degenerate face bounds nothing). No hull, no solid or an empty one answers true:
/// there is nothing to keep it out of.
pub fn lies_inside(hull: Option<&Mesh>, solid: Option<&Mesh>) -> bool {
    let (Some(hull), Some(solid)) = (hull, solid) else {
        return true;
    };
    if solid.vertices.is_empty() {
        return true;
    }

    let mut centre = Vector3::ZERO;
    for vertex in &hull.vertices {
        centre += Vector3::from_vector3_float(*vertex);
    }

    centre /= hull.vertices.len() as f64;

    for face in hull.faces.iter() {
        let a = Vector3::from_vector3_float(hull.vertices[face.v0 as usize]);
        let b = Vector3::from_vector3_float(hull.vertices[face.v1 as usize]);
        let c = Vector3::from_vector3_float(hull.vertices[face.v2 as usize]);

        let mut normal = (b - a).cross(c - a);
        let length = normal.length();
        if length <= 0.0 {
            // A degenerate triangle bounds nothing; the planes either side of it still do.
            continue;
        }

        normal /= length;
        if normal.dot(a - centre) < 0.0 {
            normal = -normal;
        }

        let distance = normal.dot(a);
        if solid
            .vertices
            .iter()
            .any(|vertex| normal.dot(Vector3::from_vector3_float(*vertex)) > distance)
        {
            return false;
        }
    }

    true
}

/// The source's convex hull, each vertex pushed out from the hull's centroid by
/// [`PAD_FRACTION`] of the source's diagonal, a closed operand on the hull's shared vertices; `None` where the
/// source has no hull of its own (flat, tiny or empty), no extent, or the padded hull is no usable
/// operand.
pub fn build(source: Option<&Mesh>) -> Option<Mesh> {
    let source = source?;
    let hull = match source.get_convex_hull(false)? {
        // The source handed back as its own hull: flat or empty, nothing to bound.
        ConvexHull::Mesh(_) => return None,
        ConvexHull::Hull(hull) => hull,
    };
    if hull.vertices.len() < 4 {
        return None;
    }

    let pad = PAD_FRACTION * source.get_axis_aligned_bounding_box().size().length();
    if pad.is_nan() || pad <= 0.0 {
        return None;
    }

    // The centre the padding pushes away from is the mean of every face's three corners, so a
    // corner many faces meet at weighs more. Any point inside the hull would do; this one keeps
    // the padded hull, and so every bevel bounded by it, the same as when the hull's faces each
    // carried their own copies of their corners.
    let mut centroid = Vector3::ZERO;
    for face in hull.faces.iter() {
        for corner in [face.v0, face.v1, face.v2] {
            centroid += Vector3::from_vector3_float(hull.vertices[corner as usize]);
        }
    }

    centroid /= (hull.faces.len() * 3) as f64;

    let mut padded = Mesh::new();
    for vertex in &hull.vertices {
        let position = Vector3::from_vector3_float(*vertex);
        let outward = position - centroid;
        let reach = outward.length();
        padded
            .vertices
            .push(Vector3Float::from_vector3(if reach > 0.0 {
                position + (outward * (pad / reach))
            } else {
                position
            }));
    }

    for face in hull.faces.iter() {
        padded
            .faces
            .add(face.v0, face.v1, face.v2, &padded.vertices);
    }

    // The hull shares its vertices between faces, so the padded copy is a closed solid as built;
    // the clean-up only acts should the padding round two corners onto one point.
    padded.clean_and_merge();

    bevel_working_mesh::is_usable_operand(Some(&padded), None).then_some(padded)
}
