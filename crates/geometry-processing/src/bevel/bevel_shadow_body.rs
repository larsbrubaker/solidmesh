//! The solid an obstruction stands as above a face - its SHADOW BODY - for the shadow clip to keep
//! a cutter out of: the faces standing on the obstruction's face reached without crossing a concave
//! fold, the convex region their planes bound inside the band, slabs along each spine, and the
//! checks that give the body up where it would not be the obstruction's.

use std::collections::{HashMap, HashSet};

use polygon_mesh::{Mesh, MeshEdgeGraph};
use vector_math::{cs_math, AxisAlignedBoundingBox, Vector3};

use super::bevel_shadow_clip::{self, Obstruction, IN_FRONT_OF_THE_FACE};
use super::bevel_working_mesh;

/// A plane as (unit normal, distance): the kept side is `normal . p >= distance`.
pub type BodyPlane = (Vector3, f64);

/// How far past the band the body may enclose before it is not believed.
pub const BIGGEST_SANE_SHARE_OF_THE_BAND: f64 = 1.01;

/// The body's clearance off the faces, as a fraction of the part's scale.
pub const CLEARANCE_OF_THE_PARTICULAR_PART: f64 = 1e-5;

/// The least clearance, whatever the part.
pub const SMALLEST_CLEARANCE: f64 = 1e-6;

/// The most of a cutter's volume a clip by the body may take before it is given up.
pub const MOST_OF_THE_CUTTER_A_CLIP_MAY_TAKE: f64 = 0.9;

/// The clearance the body keeps off the faces it is built from, read off the part's scale.
pub fn clearance(mesh: &Mesh) -> f64 {
    let bounds = mesh.get_axis_aligned_bounding_box();
    let scale = cs_math::max_f64(bounds.size().length(), furthest_from_the_origin(&bounds));
    if scale.is_nan() || scale <= 0.0 || scale.is_infinite() {
        return SMALLEST_CLEARANCE;
    }

    cs_math::max_f64(SMALLEST_CLEARANCE, scale * CLEARANCE_OF_THE_PARTICULAR_PART)
}

/// How far the furthest corner of a box is from the origin, on its furthest axis.
pub fn furthest_from_the_origin(bounds: &AxisAlignedBoundingBox) -> f64 {
    bevel_working_mesh::furthest_from_the_origin(bounds)
}

/// True when the cutter clipped by the body should be given up for the prisms alone: no clipped
/// cutter, an unusable one, one that lost most of the cutter or one that grew.
pub fn the_body_should_be_given_up(cutter: &Mesh, clipped: Option<&Mesh>) -> bool {
    let Some(clipped) = clipped.filter(|clipped| {
        !clipped.vertices.is_empty() && bevel_working_mesh::is_usable_operand(Some(clipped), None)
    }) else {
        // The prisms alone may well give a cutter the kernel WILL take, and giving the body up to
        // get it is strictly better than losing the whole clip and cutting the obstruction through.
        return true;
    };

    let was = enclosed_volume(cutter);
    let now = enclosed_volume(clipped);
    if was <= 0.0 {
        // Nothing to measure against - a question for whoever built the cutter, not for the clip.
        return false;
    }

    now <= was * (1.0 - MOST_OF_THE_CUTTER_A_CLIP_MAY_TAKE) || now > was * 1.001
}

/// The obstruction's shadow body, or `None` where it would not be the obstruction's: no faces
/// standing on it, fewer than two face planes (a half space cuts the cutter in two), faces that do
/// not close a convex body (an L-shaped boss's notch would be filled), or a region outside the
/// band's sane share. `clearance_multiplier` is normally 1.
#[allow(clippy::too_many_arguments)] // each argument is a separate input.
pub fn build(
    mesh: &Mesh,
    graph: &MeshEdgeGraph,
    edges_by_vertex_pair: &HashMap<(i32, i32), i32>,
    obstruction: &Obstruction,
    in_front: f64,
    stands_up_to: f64,
    across_at_edge: f64,
    along_at_start: f64,
    margin: f64,
    clearance_multiplier: f64,
) -> Option<Mesh> {
    let standing = standing_body(mesh, graph, edges_by_vertex_pair, obstruction, in_front);
    if standing.is_empty() {
        return None;
    }

    let clearance = clearance(mesh) * clearance_multiplier;

    // The same reach the column takes: as far behind the face as this cutter can reach, and no
    // higher than the obstruction actually stands.
    let below = (obstruction.setback * 2.0) + obstruction.planar_within;
    let above = stands_up_to + obstruction.planar_within;

    let mut planes: Vec<BodyPlane> = vec![
        (obstruction.across, across_at_edge - margin),
        (
            -obstruction.across,
            -(across_at_edge + obstruction.setback + margin),
        ),
        (obstruction.along, along_at_start - margin),
        (
            -obstruction.along,
            -(along_at_start + obstruction.along_length + margin),
        ),
        (obstruction.plane_normal, obstruction.plane_distance - below),
        (
            -obstruction.plane_normal,
            -(obstruction.plane_distance + above),
        ),
    ];
    let band_planes = planes.len();

    add_face_planes(
        mesh,
        &standing,
        obstruction,
        &mut planes,
        band_planes,
        clearance,
    );
    if planes.len() - band_planes < 2 {
        // One plane is a HALF SPACE: it cuts the cutter in two and hands back whichever half it
        // likes. A lone flat wall is left to its footprint's prism.
        return None;
    }

    if !the_reached_faces_close_a_convex_body(mesh, &standing, obstruction, &planes, band_planes) {
        // An intersection of half spaces is CONVEX, so where the obstruction is not, this region is
        // bigger than it - material the fillet is entitled to cut. Left to its prism.
        return None;
    }

    add_extent_slabs(
        mesh,
        &standing,
        obstruction,
        &mut planes,
        band_planes,
        clearance,
    );

    // Big enough to hold the band before the band's own planes cut it down, measured off the band
    // rather than off the part.
    let reach = obstruction.along_length + obstruction.setback + below + above + (margin * 4.0);
    let centre = obstruction.origin
        + (obstruction.along * (obstruction.along_length / 2.0))
        + (obstruction.across * (obstruction.setback / 2.0))
        + (obstruction.plane_normal * ((above - below) / 2.0));

    let body = convex_region(&planes, centre, reach * 4.0)?;

    // The band is the one bound every plane above cuts into, so the answer is sanity checked
    // against it.
    let band = (obstruction.along_length + (margin * 2.0))
        * (obstruction.setback + (margin * 2.0))
        * (below + above);
    let enclosed = enclosed_volume(&body);
    if enclosed <= 0.0 || enclosed > band * BIGGEST_SANE_SHARE_OF_THE_BAND {
        return None;
    }

    Some(body)
}

/// True when every face plane past the band's own has every standing face's corners on its kept
/// side, to the obstruction's planarity.
pub fn the_reached_faces_close_a_convex_body(
    mesh: &Mesh,
    standing: &[i32],
    obstruction: &Obstruction,
    planes: &[BodyPlane],
    band_planes: usize,
) -> bool {
    for &(normal, distance) in &planes[band_planes..] {
        for &face in standing {
            let mesh_face = mesh.faces[face as usize];
            for vertex in [mesh_face.v0, mesh_face.v1, mesh_face.v2] {
                if normal.dot(Vector3::from_vector3_float(mesh.vertices[vertex as usize]))
                    - distance
                    < -obstruction.planar_within
                {
                    return false;
                }
            }
        }
    }

    true
}

/// Adds each standing face's plane, turned to keep the side behind it and pushed `clearance` off,
/// merged with an equal plane already listed (within the planarity, the farther one kept).
pub fn add_face_planes(
    mesh: &Mesh,
    standing: &[i32],
    obstruction: &Obstruction,
    planes: &mut Vec<BodyPlane>,
    band_planes: usize,
    clearance: f64,
) {
    for &face in standing {
        let Some((normal, at)) = plane_of(mesh, face) else {
            continue;
        };

        let keep = (-normal, -(normal.dot(at) + clearance));
        let mut known = false;
        for plane in planes[band_planes..].iter_mut() {
            if (plane.0 - keep.0).length_squared() < 1e-12
                && (plane.1 - keep.1).abs() <= obstruction.planar_within
            {
                plane.1 = cs_math::max_f64(plane.1, keep.1);
                known = true;
                break;
            }
        }

        if !known {
            planes.push(keep);
        }
    }
}

/// Adds a slab across each spine (where two face planes meet, square to the band's run) bounding
/// the body to the standing faces' extent along it - carried on past a rounded obstruction's
/// continuation - so the convex region does not run on along the spine.
pub fn add_extent_slabs(
    mesh: &Mesh,
    standing: &[i32],
    obstruction: &Obstruction,
    planes: &mut Vec<BodyPlane>,
    band_planes: usize,
    clearance: f64,
) {
    let mut spines = Vec::new();
    for i in band_planes..planes.len() {
        for j in i + 1..planes.len() {
            let edge = planes[i].0.cross(planes[j].0);
            if edge.length_squared() > 1e-12 {
                if let Some(on) = try_meet(planes[i], planes[j]) {
                    spines.push((edge.get_normal(), on));
                }
            }
        }
    }

    for (direction, on) in spines {
        if direction.dot(obstruction.along).abs() > 1e-6 {
            continue;
        }

        let mut low = f64::MAX;
        let mut high = f64::MIN;
        for &face in standing {
            let mesh_face = mesh.faces[face as usize];
            for vertex in [mesh_face.v0, mesh_face.v1, mesh_face.v2] {
                let at = direction.dot(Vector3::from_vector3_float(mesh.vertices[vertex as usize]));
                low = cs_math::min_f64(low, at);
                high = cs_math::max_f64(high, at);
            }
        }

        if low > high {
            continue;
        }

        low = -reach_of_the_continuation(obstruction, on, -direction, -low);
        high = reach_of_the_continuation(obstruction, on, direction, high);

        planes.push((direction, low - obstruction.planar_within - clearance));
        planes.push((-direction, -(high + obstruction.planar_within + clearance)));
    }
}

/// How far along `direction` from `on` the obstruction's ROUND continues past `from` - the first
/// point beyond it where the line leaves the round's cylinder - or `from` itself for a sharp
/// obstruction or a line that never re-enters it.
pub fn reach_of_the_continuation(
    obstruction: &Obstruction,
    on: Vector3,
    direction: Vector3,
    from: f64,
) -> f64 {
    if obstruction.round_radius.is_nan() || obstruction.round_radius <= 0.0 {
        return from;
    }

    let across_at_edge = obstruction.across.dot(obstruction.origin);
    let across_rate = obstruction.across.dot(direction);
    let across_at = obstruction.across.dot(on) - across_at_edge - obstruction.setback;
    let depth_rate = -obstruction.plane_normal.dot(direction);
    let depth_at =
        obstruction.plane_distance - obstruction.plane_normal.dot(on) - obstruction.round_radius;

    let a = (across_rate * across_rate) + (depth_rate * depth_rate);
    let b = 2.0 * ((across_rate * across_at) + (depth_rate * depth_at));
    let c = (across_at * across_at) + (depth_at * depth_at)
        - (obstruction.round_radius * obstruction.round_radius);
    if a < 1e-12 {
        return from;
    }

    let at = from;
    if (a * at * at) + (b * at) + c <= 0.0 {
        return from;
    }

    let discriminant = (b * b) - (4.0 * a * c);
    if discriminant <= 0.0 {
        return from;
    }

    let nearest = (-b - discriminant.sqrt()) / (2.0 * a);
    if nearest > at {
        nearest
    } else {
        from
    }
}

/// The point of the line where two planes meet nearest the origin; `None` for (near) parallel
/// planes.
pub fn try_meet(first: BodyPlane, second: BodyPlane) -> Option<Vector3> {
    let between = first.0.dot(second.0);
    let spread = 1.0 - (between * between);
    if spread < 1e-9 {
        return None;
    }

    Some(
        (first.0 * ((first.1 - (between * second.1)) / spread))
            + (second.0 * ((second.1 - (between * first.1)) / spread)),
    )
}

/// The faces reached from the obstruction's seed face across two-face edges, each standing clear
/// of the obstruction's plane by `in_front` and not folding concavely from the face it was reached
/// from, in the order reached.
pub fn standing_body(
    mesh: &Mesh,
    graph: &MeshEdgeGraph,
    edges_by_vertex_pair: &HashMap<(i32, i32), i32>,
    obstruction: &Obstruction,
    in_front: f64,
) -> Vec<i32> {
    let mut reached = Vec::new();
    if obstruction.seed_face < 0 || obstruction.seed_face as usize >= mesh.faces.len() {
        return reached;
    }

    let mut seen = HashSet::from([obstruction.seed_face]);
    let mut to_visit = vec![obstruction.seed_face];
    reached.push(obstruction.seed_face);

    while let Some(face) = to_visit.pop() {
        for side in 0..3 {
            let mesh_face = mesh.faces[face as usize];
            let (vertex_a, vertex_b) = match side {
                0 => (mesh_face.v0, mesh_face.v1),
                1 => (mesh_face.v1, mesh_face.v2),
                _ => (mesh_face.v2, mesh_face.v0),
            };

            let Some(&edge_index) =
                edges_by_vertex_pair.get(&(vertex_a.min(vertex_b), vertex_a.max(vertex_b)))
            else {
                continue;
            };
            if graph.get_face_count(edge_index) != 2 {
                continue;
            }

            for i in 0..graph.get_face_count(edge_index) {
                let neighbour = graph.get_face(edge_index, i);
                if seen.contains(&neighbour)
                    || !bevel_shadow_clip::clear_of_the_plane(
                        mesh,
                        neighbour,
                        obstruction,
                        in_front,
                    )
                    || folds_concavely(mesh, face, neighbour, vertex_a, vertex_b)
                {
                    continue;
                }

                seen.insert(neighbour);
                reached.push(neighbour);
                to_visit.push(neighbour);
            }
        }
    }

    reached
}

/// True when `neighbour`'s far corner stands in front of `face`'s plane - the fold between them is
/// concave - by more than float noise at its magnitude.
pub fn folds_concavely(
    mesh: &Mesh,
    face: i32,
    neighbour: i32,
    vertex_a: i32,
    vertex_b: i32,
) -> bool {
    let Some((normal, at)) = plane_of(mesh, face) else {
        return false;
    };

    let other = mesh.faces[neighbour as usize];
    for vertex in [other.v0, other.v1, other.v2] {
        if vertex == vertex_a || vertex == vertex_b {
            continue;
        }

        let opposite = Vector3::from_vector3_float(mesh.vertices[vertex as usize]);
        return normal.dot(opposite - at) > (opposite.length() + 1.0) * IN_FRONT_OF_THE_FACE;
    }

    false
}

/// A face's unit normal and first corner; `None` for a face with no area.
pub fn plane_of(mesh: &Mesh, face: i32) -> Option<(Vector3, Vector3)> {
    let mesh_face = mesh.faces[face as usize];
    let at = Vector3::from_vector3_float(mesh.vertices[mesh_face.v0 as usize]);
    let normal = (Vector3::from_vector3_float(mesh.vertices[mesh_face.v1 as usize]) - at)
        .cross(Vector3::from_vector3_float(mesh.vertices[mesh_face.v2 as usize]) - at);
    if normal.length_squared() < 1e-18 {
        return None;
    }

    Some((normal.get_normal(), at))
}

/// The convex region the planes bound, cut down from a cube of side `span` about `centre`, as a
/// closed outward-wound mesh; `None` where a plane leaves nothing or fewer than four faces remain.
pub fn convex_region(planes: &[BodyPlane], centre: Vector3, span: f64) -> Option<Mesh> {
    let half = span / 2.0;
    let on_the_plane = span * 1e-9;
    let mut faces: Vec<Vec<Vector3>> = Vec::new();

    for axis in 0..3 {
        for side in 0..2 {
            let mut normal = Vector3::ZERO;
            normal[axis] = if side == 0 { 1.0 } else { -1.0 };
            let mut u = Vector3::ZERO;
            let mut v = Vector3::ZERO;
            u[(axis + 1) % 3] = 1.0;
            v[(axis + 2) % 3] = 1.0;
            if side == 1 {
                v = -v;
            }

            let at = centre + (normal * half);
            faces.push(vec![
                at - (u * half) - (v * half),
                at + (u * half) - (v * half),
                at + (u * half) + (v * half),
                at - (u * half) + (v * half),
            ]);
        }
    }

    for &(normal, distance) in planes {
        let mut kept = Vec::with_capacity(faces.len() + 1);
        let mut cut = Vec::new();
        for face in &faces {
            let clipped = bevel_shadow_clip::clip_to_half_space(face, normal, distance);
            if clipped.len() < 3 {
                continue;
            }

            for &point in &clipped {
                if (normal.dot(point) - distance).abs() <= on_the_plane {
                    cut.push(point);
                }
            }

            kept.push(clipped);
        }

        if kept.is_empty() {
            return None;
        }

        let cap = round_the_centroid(cut, normal, on_the_plane);
        if cap.len() >= 3 {
            kept.push(cap);
        }

        faces = kept;
    }

    let mut mesh = Mesh::new();
    let flip = six_times_enclosed_volume(&faces) < 0.0;
    for face in &faces {
        for i in 1..face.len() - 1 {
            if flip {
                mesh.create_face(&[face[0], face[i + 1], face[i]]);
            } else {
                mesh.create_face(&[face[0], face[i], face[i + 1]]);
            }
        }
    }

    (mesh.faces.len() >= 4).then_some(mesh)
}

/// Six times the volume a set of polygons encloses, signed by their winding.
pub fn six_times_enclosed_volume(faces: &[Vec<Vector3>]) -> f64 {
    let mut enclosed = 0.0;
    for face in faces {
        for i in 1..face.len().saturating_sub(1) {
            enclosed += face[0].dot(face[i].cross(face[i + 1]));
        }
    }

    enclosed
}

/// The volume a mesh's faces enclose, signed by their winding.
pub fn enclosed_volume(mesh: &Mesh) -> f64 {
    let mut enclosed = 0.0;
    for face in mesh.faces.iter() {
        enclosed += Vector3::from_vector3_float(mesh.vertices[face.v0 as usize]).dot(
            Vector3::from_vector3_float(mesh.vertices[face.v1 as usize])
                .cross(Vector3::from_vector3_float(mesh.vertices[face.v2 as usize])),
        );
    }

    enclosed / 6.0
}

/// The distinct points (within `same_spot`) of a planar cut, ordered round their centroid - the
/// cap polygon the cut leaves. Fewer than three come back as they are.
pub fn round_the_centroid(points: Vec<Vector3>, normal: Vector3, same_spot: f64) -> Vec<Vector3> {
    let mut distinct: Vec<Vector3> = Vec::with_capacity(points.len());
    for point in points {
        if !distinct
            .iter()
            .any(|already| (*already - point).length_squared() <= same_spot * same_spot)
        {
            distinct.push(point);
        }
    }

    if distinct.len() < 3 {
        return distinct;
    }

    let mut centroid = Vector3::ZERO;
    for &point in &distinct {
        centroid += point;
    }

    centroid /= distinct.len() as f64;

    let least = if normal.x.abs() <= normal.y.abs() && normal.x.abs() <= normal.z.abs() {
        Vector3::UNIT_X
    } else if normal.y.abs() <= normal.z.abs() {
        Vector3::UNIT_Y
    } else {
        Vector3::UNIT_Z
    };

    let across = normal.cross(least).get_normal();
    let up = normal.cross(across);
    let angle = |point: Vector3| {
        let offset = point - centroid;
        up.dot(offset).atan2(across.dot(offset))
    };

    // By descending angle, with an introsort whose tie order is fixed, so the result is
    // reproducible.
    vector_math::cs_sort::introspective_sort(&mut distinct, &mut |left, right| {
        let (a, b) = (angle(*right), angle(*left));
        match (a.is_nan(), b.is_nan()) {
            (true, true) => std::cmp::Ordering::Equal,
            (true, false) => std::cmp::Ordering::Less,
            (false, true) => std::cmp::Ordering::Greater,
            _ => a.partial_cmp(&b).unwrap_or(std::cmp::Ordering::Equal),
        }
    });

    distinct
}
