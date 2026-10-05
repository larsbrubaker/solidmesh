//! The corner fill's loop reading. Ports the nested type of `Bevel/LevelLine/CornerFill.cs` (a C#
//! static partial class): one stretch of a group loop read by one chain ([`CornerFillPiece`]), the
//! position test the corner functions share ([`same`]) and the boundary connectors between open
//! runs ([`connectors`]), which cuts it takes ([`takes`], [`corner_points`]) and the tie rule
//! ([`fields_tie`], [`ties`]) and a closed chain's piece closed round its seam
//! ([`close_round_the_seam`]). The fill's other functions land with later packages.

use std::collections::{HashMap, HashSet};

use vector_math::{cs_math, Vector3};

use super::band_side_tagging;
use super::gap_fill::FILL_TAG;
use super::{
    BandGapResult, BandProfile, ChainField, GapLoop, LevelLineMesh, LevelLineMeshError, WorkError,
};

/// True when two positions are the same point, to a billionth of their magnitude (at least 1).
pub fn same(a: Vector3, b: Vector3) -> bool {
    let tolerance = 1e-9 * cs_math::max_f64(1.0, cs_math::max_f64(a.length(), b.length()));
    (a - b).length_squared() <= tolerance * tolerance
}

/// Lays one fill triangle `(a, b, c)` into `mesh`, tagged as fill, and records it in `added`; a
/// triangle that repeats a vertex is skipped.
pub fn lay(
    mesh: &mut LevelLineMesh,
    a: i32,
    b: i32,
    c: i32,
    added: &mut Vec<i32>,
) -> Result<(), LevelLineMeshError> {
    if a == b || b == c || c == a {
        return Ok(());
    }

    added.push(mesh.add_face(a, b, c, FILL_TAG)?);
    Ok(())
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

/// C# `CornerFill.Takes`: true for a built gap, chamfer or fillet, whose chains meet at a corner: a
/// point where at least two open chains end and at least three picked edges gather (an interior
/// vertex of another chain there counts twice).
pub fn takes(gap: Option<&BandGapResult>, profile: BandProfile) -> bool {
    gap.is_some_and(|gap| {
        gap.built()
            && (profile == BandProfile::Chamfer || profile == BandProfile::Fillet)
            && !corner_points(&gap.chains).is_empty()
    })
}

/// C# `CornerFill.CornerPoints`: the corners of the pick, as positions - see [`takes`]. Two ends
/// alone (two edges) are a chain cut at a setback step, not a corner, unless both-sided members
/// build, where one end with another chain's interior through it is one too.
pub fn corner_points(chains: &[std::rc::Rc<ChainField>]) -> Vec<Vector3> {
    let ends: Vec<Vector3> = chains
        .iter()
        .filter(|c| !c.is_closed)
        .flat_map(|c| [c.points[0], c.points[c.points.len() - 1]])
        .collect();
    let mut corners: Vec<Vector3> = Vec::new();
    for &end in &ends {
        if corners.iter().any(|&c| same(c, end)) {
            continue;
        }

        let ends_here = ends.iter().filter(|&&e| same(e, end)).count() as i32;
        let mut edges = ends_here;
        for chain in chains {
            let last = chain.points.len() - 1;
            for (i, &point) in chain.points.iter().enumerate() {
                let interior = chain.is_closed || (i > 0 && i < last);
                if interior && same(point, end) {
                    edges += 2;
                }
            }
        }

        if (ends_here >= 2 || (band_side_tagging::both_sided_members_build() && ends_here >= 1))
            && edges >= 2
        {
            corners.push(end);
        }
    }

    corners
}

/// C# `CornerFill.Ties`: true when the neighbouring piece's nearest vertex (its first, or its last)
/// reads the same field value for `chain` as for its own owner - a tie the walk broke by chain
/// index.
pub fn ties(
    mesh: &LevelLineMesh,
    chains: &[std::rc::Rc<ChainField>],
    chain: i32,
    neighbour: &CornerFillPiece,
    first: bool,
) -> Result<bool, WorkError> {
    let vertex = if first {
        neighbour.vertices[0]
    } else {
        neighbour.vertices[neighbour.vertices.len() - 1]
    };
    fields_tie(
        &chains[chain as usize],
        &chains[neighbour.owner as usize],
        mesh.position(vertex),
    )
}

/// C# `CornerFill.FieldsTie`: THE tie rule of a pick with corners - two chains' fields read the
/// same value at `p` up to rounding, scaled by the longer chain. The fill extends a rail over a tie
/// by it, and the band's corner-aware loop test counts a tie vertex as owned by every chain that
/// ties there by it - one rule, so the two halves never disagree about which vertex is whose.
pub fn fields_tie(a: &ChainField, b: &ChainField, p: Vector3) -> Result<bool, WorkError> {
    let tolerance = 1e-9 * cs_math::max_f64(1.0, cs_math::max_f64(a.length, b.length));
    Ok((a.evaluate(p)?.value - b.evaluate(p)?.value).abs() <= tolerance)
}

/// C# `CornerFill.CloseRoundTheSeam`: a one-owner piece of a CLOSED chain turned to start at its
/// vertex nearest the chain's seam, and closed by that vertex again at the end, so the rail is
/// monotone. The seam vertex is read at both ends: once as the reading gave it, once from across
/// the seam - on a mitre at the seam corner the two readings tie at p and L - p (its feet on the two
/// edges are equally far from the corner); a vertex off the mitre is carried on round by a whole
/// length instead. Which reading starts the rail is the loop's own sense, read as the sum of its
/// wrapped steps.
pub fn close_round_the_seam(mesh: &LevelLineMesh, chain: &ChainField, piece: &mut CornerFillPiece) {
    let n = piece.vertices.len();
    let length = chain.length;
    if n < 2 || length <= 0.0 {
        return;
    }

    // C#'s OrderBy(..).First(): the first nearest in piece order, NaN ordering below every number.
    let seam = chain.point_at(0.0);
    let distance = |i: usize| (mesh.position(piece.vertices[i]) - seam).length_squared();
    let mut first = 0;
    for i in 1..n {
        let (d, best) = (distance(i), distance(first));
        if !best.is_nan() && (d.is_nan() || d < best) {
            first = i;
        }
    }

    let mut vertices: Vec<i32> = (0..n).map(|i| piece.vertices[(first + i) % n]).collect();
    let mut parameters: Vec<f64> = (0..n).map(|i| piece.parameters[(first + i) % n]).collect();

    let wrapped = |step: f64| step - (length * (step / length).round_ties_even());
    let mut travel = 0.0;
    for i in 0..n {
        travel += wrapped(parameters[(i + 1) % n] - parameters[i]);
    }

    let p = parameters[0];
    let position = mesh.position(vertices[0]);
    let mut across = length - p;
    let tie = 1e-9 * cs_math::max_f64(1.0, length);
    if ((position - chain.point_at(p)).length() - (position - chain.point_at(across)).length())
        .abs()
        > tie
    {
        across = if p < length / 2.0 {
            p + length
        } else {
            p - length
        };
    }

    let low = cs_math::min_f64(p, across);
    let high = cs_math::max_f64(p, across);
    parameters[0] = if travel > 0.0 { low } else { high };
    vertices.push(vertices[0]);
    parameters.push(if travel > 0.0 { high } else { low });

    piece.vertices = vertices;
    piece.parameters = parameters;
}
