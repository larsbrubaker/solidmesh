//! Dropping the zero-area slivers a boolean leaves over a T-junction in the input, and putting the
//! vertex each stitched into the face whose edge it stands on: one face's split ([`Split`]) and the
//! repair's faces with where each came from ([`Rebuild`]), and the repair itself
//! ([`dropped_from`]).

use std::collections::{HashMap, HashSet};

use polygon_mesh::{Face, FaceList, Mesh};
use vector_math::{cs_sort, Vector3};

use super::bevel_working_mesh;

/// Which vertices a face has to take in, and along which of its own edges.
#[derive(Debug, Clone, PartialEq, Eq, Default)]
pub struct Split {
    /// The start of the face's edge being split, in the direction the face winds round it.
    pub x: i32,
    /// The end of that edge.
    pub y: i32,
    /// The vertices standing on it, in the order they will be met walking `x` to `y`.
    pub middles: Vec<i32>,
}

/// The faces of the repair, and where each of them came from in the mesh it is replacing.
#[derive(Debug, Clone, PartialEq, Default)]
pub struct Rebuild {
    /// The repaired faces.
    pub faces: FaceList,
    /// For each of [`Rebuild::faces`], the index of the source face it replaces.
    pub source_face_index: Vec<i32>,
}

/// The position of mesh vertex `v` in double precision.
fn at(mesh: &Mesh, v: i32) -> Vector3 {
    Vector3::from_vector3_float(mesh.vertices[v as usize])
}

/// Drops every face that is a T-vertex sliver (its apex standing on its own longest edge, within
/// the weld distance) and puts each such apex into the face across whose edge it stands; returns
/// how many faces went (0 where the plan conflicts or the repair would leave more open edges).
pub fn dropped_from(mesh: &mut Mesh) -> i32 {
    let weld_distance = bevel_working_mesh::seam_tolerance_for(mesh);
    if weld_distance.is_nan() || weld_distance <= 0.0 {
        // Nothing to scale a tolerance against, so there is no distance at which to call three
        // points collinear and nothing here to be sure enough about to drop.
        return 0;
    }

    let mut sliver_faces = HashSet::new();
    let mut sliver_corners = HashSet::new();
    let mut middles = Vec::new();
    let mut middle_already = HashSet::new();
    for (face_index, face) in mesh.faces.iter().enumerate() {
        if let Some(middle) = try_read_t_vertex(mesh, face, weld_distance) {
            sliver_faces.insert(face_index);
            sliver_corners.insert(face.v0);
            sliver_corners.insert(face.v1);
            sliver_corners.insert(face.v2);

            // Listed as well as set, because the fan lays the vertices of one edge in the order they
            // stand along it and a plan that varied with hash order would not be reproducible.
            if middle_already.insert(middle) {
                middles.push(middle);
            }
        }
    }

    if sliver_faces.is_empty() {
        return 0;
    }

    let Some(splits) = try_plan_the_splits(
        mesh,
        &sliver_faces,
        &sliver_corners,
        &middles,
        weld_distance,
    ) else {
        return 0;
    };

    let rebuilt = rebuilt(mesh, &sliver_faces, &splits);
    if would_leave_more_open_edges(&mesh.faces, &rebuilt) {
        return 0;
    }

    mesh.faces = rebuilt.faces;
    bevel_working_mesh::carry_face_data_onto(mesh, &rebuilt.source_face_index);
    mesh.mark_as_changed();
    sliver_faces.len() as i32
}

impl Split {
    /// The edge X-Y the middles go into, none yet.
    pub fn new(x: i32, y: i32) -> Self {
        Self {
            x,
            y,
            middles: Vec::new(),
        }
    }
}

/// For each face round the slivers that a middle stands on an edge of, the edge and the middles in
/// the order they stand along it; `None` where one face would be split along two different edges.
pub fn try_plan_the_splits(
    mesh: &Mesh,
    sliver_faces: &HashSet<usize>,
    sliver_corners: &HashSet<i32>,
    middles: &[i32],
    weld_distance: f64,
) -> Option<HashMap<usize, Split>> {
    let mut splits: HashMap<usize, Split> = HashMap::new();
    let candidates = faces_around(mesh, sliver_corners, sliver_faces);
    for &middle in middles {
        let point = at(mesh, middle);
        for &face_index in &candidates {
            let face = mesh.faces[face_index];
            let Some((x, y)) =
                try_find_the_edge_it_stands_on(mesh, &face, point, middle, weld_distance)
            else {
                continue;
            };

            let split = splits.entry(face_index).or_insert_with(|| Split::new(x, y));
            if split.x != x || split.y != y {
                return None;
            }

            split.middles.push(middle);
        }
    }

    for split in splits.values_mut() {
        // Met in the order they stand along the edge, so the fan lays one triangle per span and
        // none of them crosses another. Sorted with an introsort, whose tie order is fixed.
        let (x, y) = (split.x, split.y);
        cs_sort::introspective_sort(&mut split.middles, &mut |first, second| {
            let a = along_the_edge(mesh, x, y, *first);
            let b = along_the_edge(mesh, x, y, *second);
            match (a.is_nan(), b.is_nan()) {
                (true, true) => std::cmp::Ordering::Equal,
                (true, false) => std::cmp::Ordering::Less,
                (false, true) => std::cmp::Ordering::Greater,
                _ => a.partial_cmp(&b).unwrap_or(std::cmp::Ordering::Equal),
            }
        });
    }

    Some(splits)
}

/// The faces, in order, that touch any of `vertices` and are not themselves going. A face that is
/// going cannot take anything in - the vertex walks past the sliver across it to the face beyond.
pub fn faces_around(
    mesh: &Mesh,
    vertices: &HashSet<i32>,
    sliver_faces: &HashSet<usize>,
) -> Vec<usize> {
    (0..mesh.faces.len())
        .filter(|face_index| !sliver_faces.contains(face_index))
        .filter(|&face_index| {
            let face = mesh.faces[face_index];
            vertices.contains(&face.v0)
                || vertices.contains(&face.v1)
                || vertices.contains(&face.v2)
        })
        .collect()
}

/// How far along the split edge X-Y (0 at X, 1 at Y) a vertex stands.
pub fn along_the_edge(mesh: &Mesh, x: i32, y: i32, vertex: i32) -> f64 {
    let from = at(mesh, x);
    let along = at(mesh, y) - from;
    (at(mesh, vertex) - from).dot(along) / along.length_squared()
}

/// The edge of `face` that `point` (mesh vertex `vertex`) stands on inside, as (from, to); `None`
/// where it stands on none or is already a corner of it.
pub fn try_find_the_edge_it_stands_on(
    mesh: &Mesh,
    face: &Face,
    point: Vector3,
    vertex: i32,
    weld_distance: f64,
) -> Option<(i32, i32)> {
    for (from, to) in [(face.v0, face.v1), (face.v1, face.v2), (face.v2, face.v0)] {
        if from == vertex || to == vertex {
            // Already a corner of this face, so there is nothing to put into it.
            continue;
        }

        if stands_on(at(mesh, from), at(mesh, to), point, weld_distance) {
            return Some((from, to));
        }
    }

    None
}

/// True when `point` stands strictly inside segment from-to (longer than the weld distance),
/// within the weld distance of its line and more than that from either end.
pub fn stands_on(from: Vector3, to: Vector3, point: Vector3, weld_distance: f64) -> bool {
    let along = to - from;
    let length = along.length();
    if length.is_nan() || length <= weld_distance {
        return false;
    }

    let reach = (point - from).dot(along) / (length * length);
    reach > 0.0
        && reach < 1.0
        && along.cross(point - from).length() / length <= weld_distance
        && (point - from).length() > weld_distance
        && (point - to).length() > weld_distance
}

/// The vertex of a sliver standing on its own longest edge (the first of a tie, so the choice is
/// fixed), or `None` where the face is no T-vertex sliver.
pub fn try_read_t_vertex(mesh: &Mesh, face: &Face, weld_distance: f64) -> Option<i32> {
    let p0 = at(mesh, face.v0);
    let p1 = at(mesh, face.v1);
    let p2 = at(mesh, face.v2);

    let mut middle = face.v2;
    let (mut a, mut b, mut apex) = (p0, p1, p2);
    let mut longest = (p1 - p0).length();
    if (p2 - p1).length() > longest {
        middle = face.v0;
        a = p1;
        b = p2;
        apex = p0;
        longest = (p2 - p1).length();
    }

    if (p0 - p2).length() > longest {
        middle = face.v1;
        a = p2;
        b = p0;
        apex = p1;
    }

    stands_on(a, b, apex, weld_distance).then_some(middle)
}

impl Rebuild {
    /// The repair's faces and the source face each came from.
    pub fn new(faces: FaceList, source_face_index: Vec<i32>) -> Self {
        Self {
            faces,
            source_face_index,
        }
    }
}

/// The faces with the slivers dropped and each split face fanned from its apex through its
/// middles, with the face each came from.
pub fn rebuilt(
    mesh: &Mesh,
    sliver_faces: &HashSet<usize>,
    splits: &HashMap<usize, Split>,
) -> Rebuild {
    let mut kept = FaceList::new();
    let mut source_face_index = Vec::with_capacity(mesh.faces.len());
    for (face_index, face) in mesh.faces.iter().enumerate() {
        if sliver_faces.contains(&face_index) {
            continue;
        }

        let Some(split) = splits.get(&face_index) else {
            kept.add(face.v0, face.v1, face.v2, &mesh.vertices);
            source_face_index.push(face_index as i32);
            continue;
        };

        let apex = face.v0 + face.v1 + face.v2 - split.x - split.y;
        let mut from = split.x;
        for &middle in &split.middles {
            kept.add(apex, from, middle, &mesh.vertices);
            source_face_index.push(face_index as i32);
            from = middle;
        }

        kept.add(apex, from, split.y, &mesh.vertices);
        source_face_index.push(face_index as i32);
    }

    Rebuild::new(kept, source_face_index)
}

/// True when the rebuilt faces would leave more edges open than the mesh had.
pub fn would_leave_more_open_edges(before: &FaceList, after: &Rebuild) -> bool {
    bevel_working_mesh::open_edge_count(&after.faces) > bevel_working_mesh::open_edge_count(before)
}
