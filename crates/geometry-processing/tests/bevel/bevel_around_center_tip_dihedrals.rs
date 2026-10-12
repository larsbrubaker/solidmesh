//! Port of the two shared measuring helpers of `BevelAroundCenterTipTests.cs`, `WorstDihedralDegrees`
//! and `Dihedrals`: the bend across every two-sided edge of a mesh in a region, read on a mesh
//! welded at 1e-4 so that a seam the construction left as two coincident vertices still reads as
//! one edge. Other classes (`SurfaceConcaveCornerTests`, ...) measure smoothness with them, so they
//! live here once rather than in each.

use std::collections::HashMap;

use polygon_mesh::Mesh;
use vector_math::Vector3;

/// C# `WorstDihedralDegrees`: the sharpest bend in the region, how many edges were measured, and
/// the six sharpest named for a failure message. Panics where the region selected no edges, as C#
/// throws: there is nothing to measure.
pub fn worst_dihedral_degrees(
    mesh: &Mesh,
    in_region: impl Fn(Vector3) -> bool,
) -> (f64, usize, String) {
    let bends = dihedrals(mesh, in_region);

    assert!(
        !bends.is_empty(),
        "the region selected no edges, so there is nothing to measure"
    );

    // C#'s OrderByDescending is a stable sort; so is sort_by.
    let mut sharpest = bends.clone();
    sharpest.sort_by(|a, b| b.0.total_cmp(&a.0));

    let offenders = sharpest
        .iter()
        .take(6)
        .map(|(angle, mid)| {
            format!(
                "{angle:.1} deg at ({:.3}, {:.3}, {:.3})",
                mid.x, mid.y, mid.z
            )
        })
        .collect::<Vec<_>>()
        .join(", ");

    (sharpest[0].0, bends.len(), offenders)
}

// C#'s `!(cross.LengthSquared > 0)`, which skips a NaN normal as well.
#[allow(clippy::neg_cmp_op_on_partial_ord)]
/// C# `Dihedrals`: every edge two faces share in the region, with the angle between their normals
/// in degrees and the edge's midpoint, in the order the edges were first met.
pub fn dihedrals(mesh: &Mesh, in_region: impl Fn(Vector3) -> bool) -> Vec<(f64, Vector3)> {
    const GRID: f64 = 1e-4;

    let mut cells: HashMap<(i64, i64, i64), usize> = HashMap::new();
    let mut welded: Vec<Vector3> = Vec::new();
    let mut identity = vec![0usize; mesh.vertices.len()];

    for (index, &stored) in mesh.vertices.iter().enumerate() {
        let vertex = Vector3::from_vector3_float(stored);
        // C#'s `(long)Math.Round(...)`: banker's rounding, then a truncating cast.
        let cell = (
            (vertex.x / GRID).round_ties_even() as i64,
            (vertex.y / GRID).round_ties_even() as i64,
            (vertex.z / GRID).round_ties_even() as i64,
        );
        let mut found = None;

        'search: for dx in -1..=1 {
            for dy in -1..=1 {
                for dz in -1..=1 {
                    if let Some(&neighbour) = cells.get(&(cell.0 + dx, cell.1 + dy, cell.2 + dz)) {
                        if (welded[neighbour] - vertex).length() <= GRID {
                            found = Some(neighbour);
                            break 'search;
                        }
                    }
                }
            }
        }

        let found = match found {
            Some(found) => found,
            None => {
                let found = welded.len();
                welded.push(vertex);
                cells.insert(cell, found);
                found
            }
        };

        identity[index] = found;
    }

    // C#'s Dictionary iterates in insertion order here (nothing is removed), so the edges are kept
    // in a list in the order they were first met, with a map into it.
    let mut edge_order: Vec<(usize, usize)> = Vec::new();
    let mut by_edge: HashMap<(usize, usize), Vec<Vector3>> = HashMap::new();

    for face in mesh.faces.iter() {
        let a = Vector3::from_vector3_float(mesh.vertices[face.v0 as usize]);
        let b = Vector3::from_vector3_float(mesh.vertices[face.v1 as usize]);
        let c = Vector3::from_vector3_float(mesh.vertices[face.v2 as usize]);
        let cross = (b - a).cross(c - a);

        if !(cross.length_squared() > 0.0) {
            continue;
        }

        let normal = cross.get_normal();
        let corners = [
            identity[face.v0 as usize],
            identity[face.v1 as usize],
            identity[face.v2 as usize],
        ];

        for side in 0..3 {
            let low = corners[side];
            let high = corners[(side + 1) % 3];

            if low == high {
                continue;
            }

            let key = if low < high { (low, high) } else { (high, low) };

            by_edge
                .entry(key)
                .or_insert_with(|| {
                    edge_order.push(key);
                    Vec::new()
                })
                .push(normal);
        }
    }

    let mut bends = Vec::new();

    for key in edge_order {
        let normals = &by_edge[&key];

        if normals.len() != 2 {
            continue;
        }

        let mid = (welded[key.0] + welded[key.1]) / 2.0;

        if !in_region(mid) {
            continue;
        }

        bends.push((
            normals[0].dot(normals[1]).clamp(-1.0, 1.0).acos() * 180.0 / std::f64::consts::PI,
            mid,
        ));
    }

    bends
}
