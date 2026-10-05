//! The corner fill's loop reading. Ports the nested type of `Bevel/LevelLine/CornerFill.cs` (a C#
//! static partial class): one stretch of a group loop read by one chain ([`CornerFillPiece`]), the
//! position test the corner functions share ([`same`]) and the boundary connectors between open
//! runs ([`connectors`]). The fill's other functions land with later packages.

use std::collections::{HashMap, HashSet};

use vector_math::{cs_math, Vector3};

use super::{GapLoop, LevelLineMesh};

/// True when two positions are the same point, to a billionth of their magnitude (at least 1).
pub fn same(a: Vector3, b: Vector3) -> bool {
    let tolerance = 1e-9 * cs_math::max_f64(1.0, cs_math::max_f64(a.length(), b.length()));
    (a - b).length_squared() <= tolerance * tolerance
}

/// The boundary edges that join each open run's last vertex to the next run's first, walked along
/// the live mesh's one-sided edges; `None` where a walk branches, dead-ends or runs on past every
/// boundary vertex.
pub fn connectors(mesh: &LevelLineMesh, loops: &[GapLoop]) -> Option<Vec<(i32, i32)>> {
    let mut directed = HashSet::new();
    for t in 0..mesh.triangle_count() {
        if !mesh.is_removed(t) {
            let face = mesh.triangles()[t as usize];
            for corner in 0..3 {
                directed.insert((face.corner(corner), face.corner((corner + 1) % 3)));
            }
        }
    }

    // Only each list's LENGTH and, for a list of one, its one entry are read, so the set's own
    // order never reaches the answer.
    let mut out_of: HashMap<i32, Vec<i32>> = HashMap::new();
    for &(a, b) in &directed {
        if !directed.contains(&(b, a)) {
            out_of.entry(a).or_default().push(b);
        }
    }

    let run_starts: HashSet<i32> = loops
        .iter()
        .filter(|l| !l.is_closed)
        .map(|l| l.vertices[0])
        .collect();
    let mut connectors = Vec::new();
    for run in loops.iter().filter(|l| !l.is_closed) {
        let mut at = run.vertices[run.vertices.len() - 1];
        let mut steps = 0;
        while !run_starts.contains(&at) {
            let next = match out_of.get(&at) {
                Some(next) if steps <= out_of.len() && next.len() == 1 => next[0],
                _ => return None,
            };
            connectors.push((at, next));
            at = next;
            steps += 1;
        }
    }

    Some(connectors)
}

/// C# `CornerFill.Piece`: one stretch of a group loop read by one chain - its vertices in the
/// loop's walk order, each with the chain parameter it reads.
#[derive(Debug, Clone, PartialEq, Default)]
pub struct CornerFillPiece {
    /// The chain that owns this stretch.
    pub owner: i32,
    /// The stretch's vertices, in the loop's walk order.
    pub vertices: Vec<i32>,
    /// The owner's chain parameter at each vertex.
    pub parameters: Vec<f64>,
}
