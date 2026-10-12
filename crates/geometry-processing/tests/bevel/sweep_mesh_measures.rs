//! What the Thingi10K sweep measures a mesh by - the same questions asked of the input and of
//! every result. Ports `Tests/GeometryProcessingTests/LevelLine/SweepMeshMeasures.cs`, the shared
//! measures the sweep, `SwallowedSideTests`, `StripClipRibTests` and `GapFillTests` read; the
//! level-line mesh readings the older tests use are in `level_line_test_geometry.rs`.

#![allow(dead_code)]

use std::collections::{HashMap, HashSet};

use geometry_processing::bevel::level_line::LevelLineMesh;
use polygon_mesh::csg::boolean_processing::{classify_boolean_operand, BooleanOperandVerdict};
use polygon_mesh::Mesh;
use vector_math::{Matrix4X4, Vector3, Vector3Float};

/// A triangle's three directed edges, in winding order.
pub fn corners_of(t: (i32, i32, i32)) -> [(i32, i32); 3] {
    [(t.0, t.1), (t.1, t.2), (t.2, t.0)]
}

/// An undirected edge's key: its smaller vertex first.
pub fn key(a: i32, b: i32) -> (i32, i32) {
    if a < b {
        (a, b)
    } else {
        (b, a)
    }
}

/// Every undirected edge with the triangles standing on it, in triangle order.
pub fn standing(indices: &[(i32, i32, i32)]) -> HashMap<(i32, i32), Vec<i32>> {
    let mut standing: HashMap<(i32, i32), Vec<i32>> = HashMap::with_capacity(indices.len() * 2);
    for (i, t) in indices.iter().enumerate() {
        for (a, b) in corners_of(*t) {
            standing.entry(key(a, b)).or_default().push(i as i32);
        }
    }

    standing
}

/// Every edge whose two triangles fold by more than the threshold.
pub fn feature_edges(
    positions: &[Vector3],
    indices: &[(i32, i32, i32)],
    degrees: f64,
) -> HashSet<(i32, i32)> {
    let standing = standing(indices);
    let limit = (degrees * std::f64::consts::PI / 180.0).cos();
    let mut features = HashSet::new();

    let normal_of = |triangle: i32| {
        let t = indices[triangle as usize];
        let a = positions[t.0 as usize];
        (positions[t.1 as usize] - a)
            .cross(positions[t.2 as usize] - a)
            .get_normal()
    };

    for (edge, triangles) in &standing {
        if triangles.len() != 2 {
            // a boundary or a non-manifold edge is a wall either way.
            features.insert(*edge);
            continue;
        }

        if normal_of(triangles[0]).dot(normal_of(triangles[1])) < limit {
            features.insert(*edge);
        }
    }

    features
}

/// The mesh's triangles that are not removed, in order.
pub fn kept(mesh: &LevelLineMesh) -> Vec<(i32, i32, i32)> {
    let mut kept = Vec::with_capacity(mesh.triangle_count() as usize);
    for t in 0..mesh.triangle_count() {
        if !mesh.is_removed(t) {
            let triangle = mesh.triangles()[t as usize];
            kept.push((triangle.a, triangle.b, triangle.c));
        }
    }

    kept
}

/// Closed and consistently oriented in one statement: every directed edge once, its opposite
/// once.
pub fn is_closed_and_consistent(_positions: &[Vector3], indices: &[(i32, i32, i32)]) -> bool {
    let mut seen = HashSet::with_capacity(indices.len() * 3);
    for t in indices {
        for directed in corners_of(*t) {
            if !seen.insert(directed) {
                return false;
            }
        }
    }

    seen.iter()
        .all(|directed| seen.contains(&(directed.1, directed.0)))
}

/// How many separate shells (vertex-connected pieces) the triangles make.
pub fn shell_count(vertex_count: i32, indices: &[(i32, i32, i32)]) -> i32 {
    let mut parent: Vec<i32> = (0..vertex_count).collect();

    fn find(parent: &mut [i32], mut v: i32) -> i32 {
        while parent[v as usize] != v {
            parent[v as usize] = parent[parent[v as usize] as usize];
            v = parent[v as usize];
        }

        v
    }

    let mut used = vec![false; vertex_count as usize];
    for t in indices {
        used[t.0 as usize] = true;
        used[t.1 as usize] = true;
        used[t.2 as usize] = true;
        for (a, b) in corners_of(*t) {
            let (ra, rb) = (find(&mut parent, a), find(&mut parent, b));
            if ra != rb {
                parent[ra as usize] = rb;
            }
        }
    }

    let mut roots = HashSet::new();
    for v in 0..vertex_count {
        if used[v as usize] {
            roots.insert(find(&mut parent, v));
        }
    }

    roots.len() as i32
}

/// Whether a result intersects itself where its source did not, by the check the app's
/// level-line route makes before it keeps a build
/// (`LevelLineSelfIntersectionCheck.TheAnswerIntersectsItselfWhereTheSourceDidNot`): where it
/// does, the app quietly takes the existing construction instead, so a kernel test that asks only
/// closure passes a build the app never shows.
pub fn intersects_itself_where_the_source_did_not(
    positions: &[Vector3],
    indices: &[(i32, i32, i32)],
    source_positions: &[Vector3],
    source_indices: &[(i32, i32, i32)],
) -> bool {
    intersects_itself(positions, indices) && !intersects_itself(source_positions, source_indices)
}

/// Whether a triangle list intersects itself, by BooleanProcessing's operand check.
pub fn intersects_itself(positions: &[Vector3], indices: &[(i32, i32, i32)]) -> bool {
    let mut mesh = Mesh::new();
    let mut moved: HashMap<i32, i32> = HashMap::new();
    let mut place = |mesh: &mut Mesh, vertex: i32| -> i32 {
        *moved.entry(vertex).or_insert_with(|| {
            let at = mesh.vertices.len() as i32;
            mesh.vertices
                .push(Vector3Float::from_vector3(positions[vertex as usize]));
            at
        })
    };

    for &(a, b, c) in indices {
        let (pa, pb, pc) = (
            place(&mut mesh, a),
            place(&mut mesh, b),
            place(&mut mesh, c),
        );
        mesh.faces.add(pa, pb, pc, &mesh.vertices);
    }

    classify_boolean_operand(&mesh, Matrix4X4::IDENTITY, false)
        == BooleanOperandVerdict::SelfIntersecting
}

/// The smallest triangle's area.
pub fn smallest_area(positions: &[Vector3], indices: &[(i32, i32, i32)]) -> f64 {
    let mut smallest = f64::INFINITY;
    for t in indices {
        let a = positions[t.0 as usize];
        smallest = smallest.min(
            (positions[t.1 as usize] - a)
                .cross(positions[t.2 as usize] - a)
                .length()
                / 2.0,
        );
    }

    smallest
}

/// The volume the triangles enclose, signed by their winding.
pub fn signed_volume(positions: &[Vector3], indices: &[(i32, i32, i32)]) -> f64 {
    let mut total = 0.0;
    for t in indices {
        total +=
            positions[t.0 as usize].dot(positions[t.1 as usize].cross(positions[t.2 as usize]));
    }

    total / 6.0
}

/// The diagonal of the box around the named vertices.
pub fn diagonal(positions: &[Vector3], of: impl IntoIterator<Item = i32>) -> f64 {
    let mut low = Vector3::new(f64::MAX, f64::MAX, f64::MAX);
    let mut high = Vector3::new(f64::MIN, f64::MIN, f64::MIN);
    for v in of {
        low = Vector3::component_min(low, positions[v as usize]);
        high = Vector3::component_max(high, positions[v as usize]);
    }

    (high - low).length()
}

/// How far the pick runs: every chain's edge lengths added up, counting the closing edge only of
/// a chain that is closed. It is the one length an OPEN pick always has, which is why the sweep's
/// fallback size ladder is tied to it.
pub fn pick_length(positions: &[Vector3], chains: &[Vec<i32>], closed: &[bool]) -> f64 {
    let mut length = 0.0;
    for (c, chain) in chains.iter().enumerate() {
        let edges = if closed[c] {
            chain.len()
        } else {
            chain.len() - 1
        };
        for i in 0..edges {
            length += (positions[chain[(i + 1) % chain.len()] as usize]
                - positions[chain[i] as usize])
                .length();
        }
    }

    length
}
