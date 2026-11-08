//! Point mode's candidates: the vertices a point bevel is
//! offered at. A vertex qualifies when its angle defect - 360 degrees less the sum of its incident
//! face angles - exceeds the bend threshold, so a cone's tip and a cube's corner qualify while a
//! vertex on a smooth tessellated surface or along a tessellated rim (a few degrees) does not.
//! Each candidate says whether a fillet is offered there as well as a chamfer, read off its
//! [`super::point_star::PointStar`]; the bevel itself is [`super::point_bevel`].

use std::collections::HashMap;

use solidmesh_math::Vector3;

use super::point_star;

/// A vertex point mode offers, and whether a fillet is offered there as well
/// as a chamfer.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct PointCandidate {
    /// The vertex index.
    pub vertex: i32,
    /// Its angle defect.
    pub defect_degrees: f64,
    /// False where a sharp (feature) edge meets it - see
    /// [`super::point_bevel::PointBevelRefusal::PointHasSharpEdges`].
    pub allows_fillet: bool,
}

/// The vertices whose angle defect exceeds `bend_threshold_degrees`, on
/// the welded mesh.
pub fn find(
    positions: &[Vector3],
    triangles: &[(i32, i32, i32)],
    bend_threshold_degrees: f64,
) -> Vec<i32> {
    let defects = angle_defects(positions, triangles);
    let mut candidates = Vec::new();
    for (v, &(degrees, referenced)) in defects.iter().enumerate() {
        // an unreferenced vertex has no faces, so its "defect" of 360 is not a corner
        if referenced && degrees > bend_threshold_degrees {
            candidates.push(v as i32);
        }
    }

    candidates
}

/// The candidates with what each allows: every candidate takes a
/// chamfer, a fillet only where no sharp edge meets it.
pub fn describe(
    positions: &[Vector3],
    triangles: &[(i32, i32, i32)],
    bend_threshold_degrees: f64,
) -> Vec<PointCandidate> {
    let defects = angle_defects(positions, triangles);
    let candidates = find(positions, triangles, bend_threshold_degrees);
    if candidates.is_empty() {
        return Vec::new();
    }

    // Indexed once, so each candidate reads only its own triangles.
    let around = point_star::triangles_around(positions.len(), triangles);
    let mut described = Vec::new();
    for v in candidates {
        let sharp = point_star::has_sharp_edges_at(
            positions,
            triangles,
            &around[v as usize],
            v,
            bend_threshold_degrees,
        );
        described.push(PointCandidate {
            vertex: v,
            defect_degrees: defects[v as usize].0,
            allows_fillet: !sharp,
        });
    }

    described
}

/// The angle defect at `vertex` in degrees. Positions are welded
/// first (exact coincidence), so a mesh whose faces do not share vertices still reads its corners.
pub fn angle_defect(positions: &[Vector3], triangles: &[(i32, i32, i32)], vertex: i32) -> f64 {
    angle_defects(positions, triangles)[vertex as usize].0
}

/// Each vertex's defect in degrees, and whether any triangle references it.
fn angle_defects(positions: &[Vector3], triangles: &[(i32, i32, i32)]) -> Vec<(f64, bool)> {
    let welded = weld(positions);
    // Only looked up by key, never iterated, so a `HashMap` cannot reach an output.
    let mut sums: HashMap<i32, f64> = HashMap::new();
    for &(a, b, c) in triangles {
        let corners = [a, b, c];
        for k in 0..3 {
            let at = positions[corners[k] as usize];
            let u = positions[corners[(k + 1) % 3] as usize] - at;
            let w = positions[corners[(k + 2) % 3] as usize] - at;
            let angle = libm::atan2(u.cross(w).length(), u.dot(w));
            let key = welded[corners[k] as usize];
            let sum = sums.entry(key).or_insert(0.0);
            // Starting from 0 is exact: 0 + angle == angle.
            *sum += angle;
        }
    }

    (0..positions.len())
        .map(|v| match sums.get(&welded[v]) {
            Some(&sum) => (360.0 - sum * 180.0 / std::f64::consts::PI, true),
            None => (360.0, false),
        })
        .collect()
}

/// Each vertex's representative, the first vertex at exactly the same
/// position.
pub fn weld(positions: &[Vector3]) -> Vec<i32> {
    let mut first: HashMap<[u64; 3], i32> = HashMap::new();
    let mut welded = Vec::with_capacity(positions.len());
    for (v, p) in positions.iter().enumerate() {
        welded.push(*first.entry(exact_key(*p)).or_insert(v as i32));
    }

    welded
}

/// The key positions are welded by: exact coordinates, with -0 equal to 0 as `==` on doubles has
/// it (adding +0.0 turns -0.0 into +0.0 and leaves every other value).
fn exact_key(p: Vector3) -> [u64; 3] {
    [
        (p.x + 0.0).to_bits(),
        (p.y + 0.0).to_bits(),
        (p.z + 0.0).to_bits(),
    ]
}
