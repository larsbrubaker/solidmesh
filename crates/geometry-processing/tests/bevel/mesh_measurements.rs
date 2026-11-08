//! Whole-mesh measurements that tests use as shape assertions. They read the mesh the production
//! code actually produced rather than its tessellation, so they stay honest across geometry
//! changes; a face's altitude reads a sliver. (`sweep_mesh_measures` holds the same readings over a
//! level-line mesh's raw positions and indices.)

#![allow(dead_code)]

use std::collections::{HashMap, HashSet};

use geometry_processing::topology::FeatureEdge;
use polygon_mesh::Mesh;
use vector_math::{Vector3, Vector3Float};

/// The signed volume of a closed mesh from the divergence theorem. Positive
/// means the faces face out; zero or negative means the mesh is inside out or folds through itself.
pub fn signed_volume(mesh: &Mesh) -> f64 {
    let mut total = 0.0;

    for face in mesh.faces.iter() {
        let v0 = Vector3::from_vector3_float(mesh.vertices[face.v0 as usize]);
        let v1 = Vector3::from_vector3_float(mesh.vertices[face.v1 as usize]);
        let v2 = Vector3::from_vector3_float(mesh.vertices[face.v2 as usize]);

        total += v0.dot(v1.cross(v2));
    }

    total / 6.0
}

/// How many edges of a mesh `FeatureEdge::try_classify` can say NOTHING
/// about - a boundary or non-manifold edge, a zero length edge, or an edge with a face that has no
/// area to take a normal from - with where the first refused edge stands, for an assertion's
/// message.
///
/// This is the reading that says whether a result can be BEVELLED AGAIN. The graph is keyed by
/// vertex INDEX, so a seam welded by position but split by index reads as one-faced edges here even
/// though `SurfaceMeshClosure`, which welds by position, calls the same mesh closed; a count here is
/// a reading about SPLIT VERTICES first and degenerate triangles only second.
pub fn unclassifiable_edges(mesh: Option<&Mesh>) -> (i32, String) {
    let mut first_at = "nowhere".to_string();

    let Some(mesh) = mesh else {
        return (0, first_at);
    };

    let graph = mesh.get_mesh_edge_graph();
    let mut refused = 0;

    for edge_index in 0..graph.edge_count() {
        // The flat angle only decides Flat against Convex/Concave, never whether the edge
        // classifies at all, so the reading is the same whatever tolerance a caller would have used.
        if FeatureEdge::try_classify(
            mesh,
            &graph,
            edge_index,
            FeatureEdge::default_flat_angle_degrees(),
        )
        .is_some()
        {
            continue;
        }

        if refused == 0 {
            let start =
                Vector3::from_vector3_float(mesh.vertices[graph.get_vertex0(edge_index) as usize]);
            let end =
                Vector3::from_vector3_float(mesh.vertices[graph.get_vertex1(edge_index) as usize]);
            first_at = format!(
                "{start:?} to {end:?} ({} faces)",
                graph.get_face_count(edge_index)
            );
        }

        refused += 1;
    }

    (refused, first_at)
}

/// How many separate connected pieces a mesh is in. One is a single solid; more
/// means an operation cut the part apart.
///
/// Connectivity is judged by vertex POSITION rather than vertex index, because the boolean kernel
/// returns coincident-but-distinct vertices at seams on purpose. Two triangles that meet along such
/// a seam are one piece however they happen to be numbered.
pub fn shell_count(mesh: &Mesh) -> i32 {
    let mut canonical_of_position: HashMap<(u32, u32, u32), usize> = HashMap::new();
    let mut parent: Vec<usize> = Vec::new();

    // Keyed by the float values; `+ 0.0` folds -0 onto 0, so a point at -0 joins its +0 twin.
    let mut canonical = |position: Vector3Float, parent: &mut Vec<usize>| {
        let key = (
            (position.x + 0.0).to_bits(),
            (position.y + 0.0).to_bits(),
            (position.z + 0.0).to_bits(),
        );

        *canonical_of_position.entry(key).or_insert_with(|| {
            let index = parent.len();
            parent.push(index);
            index
        })
    };

    fn root(parent: &mut [usize], mut index: usize) -> usize {
        while parent[index] != index {
            // path halving: keeps the walk shallow without a second pass
            parent[index] = parent[parent[index]];
            index = parent[index];
        }

        index
    }

    fn union(parent: &mut [usize], a: usize, b: usize) {
        let root_a = root(parent, a);
        let root_b = root(parent, b);

        if root_a != root_b {
            parent[root_b] = root_a;
        }
    }

    let mut used: HashSet<usize> = HashSet::new();

    for face in mesh.faces.iter() {
        let v0 = canonical(mesh.vertices[face.v0 as usize], &mut parent);
        let v1 = canonical(mesh.vertices[face.v1 as usize], &mut parent);
        let v2 = canonical(mesh.vertices[face.v2 as usize], &mut parent);

        union(&mut parent, v0, v1);
        union(&mut parent, v1, v2);

        used.insert(v0);
        used.insert(v1);
        used.insert(v2);
    }

    let mut roots: HashSet<usize> = HashSet::new();

    for index in used {
        roots.insert(root(&mut parent, index));
    }

    roots.len() as i32
}

/// The height of the triangle standing on its own longest edge - twice its area over that edge -
/// or zero when it has no edge to stand on: how near to a straight line the three points are, in
/// the part's own units, which neither area nor edge length says on its own (a sliver's reading).
pub fn altitude_of(p0: Vector3Float, p1: Vector3Float, p2: Vector3Float) -> f64 {
    let v0 = Vector3::from_vector3_float(p0);
    let v1 = Vector3::from_vector3_float(p1);
    let v2 = Vector3::from_vector3_float(p2);

    let lengths = [(v1 - v0).length(), (v2 - v1).length(), (v0 - v2).length()];
    let mut longest_edge = 0;

    for edge in 1..3 {
        if lengths[edge] > lengths[longest_edge] {
            longest_edge = edge;
        }
    }

    let longest_length = lengths[longest_edge];

    #[allow(clippy::neg_cmp_op_on_partial_ord)]
    if !(longest_length > 0.0) {
        return 0.0;
    }

    let area = (v1 - v0).cross(v2 - v0).length() / 2.0;

    2.0 * area / longest_length
}
