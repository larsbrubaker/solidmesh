//! Point mode's candidates (`Bevel/LevelLine/PointCandidates.cs`): the vertices a point bevel is
//! offered at. A vertex qualifies when its angle defect - 360 degrees less the sum of its incident
//! face angles - exceeds the bend threshold, so a cone's tip and a cube's corner qualify while a
//! vertex on a smooth tessellated surface or along a tessellated rim (a few degrees) does not.
//! Each candidate says whether a fillet is offered there as well as a chamfer, read off its
//! [`PointStar`]; the bevel itself is [`super::point_bevel`].

use std::collections::HashMap;

use vector_math::Vector3;

use super::point_star::PointStar;

/// C# `PointCandidate`: a vertex point mode offers, and whether a fillet is offered there as well
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

/// C# `PointCandidates.Find`: the vertices whose angle defect exceeds `bend_threshold_degrees`, on
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

/// C# `PointCandidates.Describe`: the candidates with what each allows: every candidate takes a
/// chamfer, a fillet only where no sharp edge meets it.
pub fn describe(
    positions: &[Vector3],
    triangles: &[(i32, i32, i32)],
    bend_threshold_degrees: f64,
) -> Vec<PointCandidate> {
    let defects = angle_defects(positions, triangles);
    let mut described = Vec::new();
    for v in find(positions, triangles, bend_threshold_degrees) {
        // one star per candidate walks the whole mesh; fine for the kernel, index once if the panel
        // needs speed
        let sharp =
            PointStar::new(positions, triangles, v, bend_threshold_degrees).has_sharp_edges();
        described.push(PointCandidate {
            vertex: v,
            defect_degrees: defects[v as usize].0,
            allows_fillet: !sharp,
        });
    }

    described
}

/// C# `PointCandidates.AngleDefect`: the angle defect at `vertex` in degrees. Positions are welded
/// first (exact coincidence), so a mesh whose faces do not share vertices still reads its corners.
pub fn angle_defect(positions: &[Vector3], triangles: &[(i32, i32, i32)], vertex: i32) -> f64 {
    angle_defects(positions, triangles)[vertex as usize].0
}

/// C# `AngleDefects`: each vertex's defect in degrees, and whether any triangle references it.
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
            let angle = u.cross(w).length().atan2(u.dot(w));
            let key = welded[corners[k] as usize];
            let sum = sums.entry(key).or_insert(0.0);
            // C# stores `angle` on first sight and `sum + angle` after; 0 + angle == angle exactly.
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

/// C# `PointCandidates.Weld`: each vertex's representative, the first vertex at exactly the same
/// position.
pub fn weld(positions: &[Vector3]) -> Vec<i32> {
    let mut first: HashMap<[u64; 3], i32> = HashMap::new();
    let mut welded = Vec::with_capacity(positions.len());
    for (v, p) in positions.iter().enumerate() {
        welded.push(*first.entry(exact_key(*p)).or_insert(v as i32));
    }

    welded
}

/// The key C#'s `Dictionary<Vector3, int>` compares by: exact coordinates, with -0 equal to 0 as
/// C#'s `double` equality has it (adding +0.0 turns -0.0 into +0.0 and leaves every other value).
fn exact_key(p: Vector3) -> [u64; 3] {
    [
        (p.x + 0.0).to_bits(),
        (p.y + 0.0).to_bits(),
        (p.z + 0.0).to_bits(),
    ]
}
