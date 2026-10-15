//! The corner fill's loop reading: one stretch of a group loop read by one chain
//! ([`CornerFillPiece`]), the
//! position test the corner functions share ([`same`]) and the boundary connectors between open
//! runs ([`connectors`]), which cuts it takes ([`takes`], [`corner_points`]) and the tie rule
//! ([`fields_tie`], [`ties`]) and a closed chain's piece closed round its seam
//! ([`close_round_the_seam`]), and a group loop split into its chains' pieces ([`pieces`]). The
//! fill's other functions land with later packages.

use std::collections::{HashMap, HashSet};

use vector_math::{ieee_math, Vector3};

use std::rc::Rc;

use super::band_side_tagging;
use super::gap_fill::FILL_TAG;
use super::{corner_fill_free_ends, corner_fill_pieces};
use super::{
    BandGapResult, BandProfile, ChainField, GapLoop, LevelLineMesh, LevelLineMeshError, WorkError,
};

/// True when two positions are the same point, to a billionth of their magnitude (at least 1).
pub fn same(a: Vector3, b: Vector3) -> bool {
    let tolerance = 1e-9 * ieee_math::max_f64(1.0, ieee_math::max_f64(a.length(), b.length()));
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

/// One stretch of a group loop read by one chain - its vertices in the
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

/// True for a built gap, chamfer or fillet, whose chains meet at a corner: a
/// point where at least two open chains end and at least three picked edges gather (an interior
/// vertex of another chain there counts twice).
pub fn takes(gap: Option<&BandGapResult>, profile: BandProfile) -> bool {
    gap.is_some_and(|gap| {
        gap.built()
            && (profile == BandProfile::Chamfer || profile == BandProfile::Fillet)
            && !corner_points(&gap.chains).is_empty()
    })
}

/// The corners of the pick, as positions - see [`takes`]. Two ends
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

/// True when the neighbouring piece's nearest vertex (its first, or its last)
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

/// THE tie rule of a pick with corners - two chains' fields read the
/// same value at `p` up to rounding, scaled by the longer chain. The fill extends a rail over a tie
/// by it, and the band's corner-aware loop test counts a tie vertex as owned by every chain that
/// ties there by it - one rule, so the two halves never disagree about which vertex is whose.
pub fn fields_tie(a: &ChainField, b: &ChainField, p: Vector3) -> Result<bool, WorkError> {
    let tolerance = 1e-9 * ieee_math::max_f64(1.0, ieee_math::max_f64(a.length, b.length));
    Ok((a.evaluate(p)?.value - b.evaluate(p)?.value).abs() <= tolerance)
}

/// A one-owner piece of a CLOSED chain turned to start at its
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

    // The first nearest in piece order, NaN ordering below every number, so ties and broken
    // readings pick the same vertex on every run.
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
    let tie = 1e-9 * ieee_math::max_f64(1.0, length);
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

    let low = ieee_math::min_f64(p, across);
    let high = ieee_math::max_f64(p, across);
    parameters[0] = if travel > 0.0 { low } else { high };
    vertices.push(vertices[0]);
    parameters.push(if travel > 0.0 { high } else { low });

    piece.vertices = vertices;
    piece.parameters = parameters;
}

/// A group loop split at every change of owner, starting at one: a loop of
/// one owner all the way round is one closed chain's side and not a corner's, so it is handed back
/// whole as one piece - except the one-owner loop of a closed chain that also owns a stretch of a
/// mixed loop, started at the vertex nearest a corner. A one-owner loop of an OPEN chain is split at
/// another chain's tie run, or round its free ends. Elsewhere each piece takes in a neighbour's
/// end vertex where the two chains' fields TIE there (it is as much this chain's rail end - on a
/// cube, the chamfer's mitre corner), pieces are split round free ends, and a chain whose whole
/// rail here is ties gets its one or two runs of tie vertices as its pieces. An empty list where
/// the loop has no vertex owners.
pub fn pieces(
    mesh: &LevelLineMesh,
    chains: &[Rc<ChainField>],
    gap_loop: &GapLoop,
) -> Result<Vec<CornerFillPiece>, WorkError> {
    let Some(owners) = gap_loop.vertex_owners.as_deref() else {
        return Ok(Vec::new());
    };
    let n = gap_loop.vertices.len();
    // An open run is split from its first vertex and has no neighbour past either end; a closed
    // loop is split from an owner change and wraps.
    let mut start = if gap_loop.is_closed {
        (0..n)
            .find(|&i| owners[i] != owners[(i + n - 1) % n])
            .unwrap_or(0)
    } else {
        0
    };
    if gap_loop.is_closed
        && owners.iter().all(|&o| o == owners[0])
        && band_side_tagging::both_sided_members_build()
    {
        let corners = corner_points(chains);
        if !corners.is_empty() {
            // The vertex nearest a corner, the first one on a tie.
            let nearest = |i: usize| {
                let p = mesh.position(gap_loop.vertices[i]);
                corners
                    .iter()
                    .map(|&c| (p - c).length_squared())
                    .fold(f64::INFINITY, |least, d| if d < least { d } else { least })
            };
            let mut best = 0;
            for i in 1..n {
                if nearest(i) < nearest(best) {
                    best = i;
                }
            }

            start = best;
        }
    }

    let mut pieces: Vec<CornerFillPiece> = Vec::new();
    for step in 0..n {
        let i = (start + step) % n;
        if pieces
            .last()
            .is_none_or(|current| owners[i] != current.owner)
        {
            pieces.push(CornerFillPiece {
                owner: owners[i],
                ..CornerFillPiece::default()
            });
        }

        let current = pieces.len() - 1;
        pieces[current].vertices.push(gap_loop.vertices[i]);
        pieces[current].parameters.push(gap_loop.parameters[i]);
    }

    if pieces.len() == 1 && gap_loop.is_closed {
        let field = chains[pieces[0].owner as usize].clone();
        if field.is_closed {
            close_round_the_seam(mesh, &field, &mut pieces[0]);
        } else {
            if let Some(split) =
                corner_fill_pieces::tie_split(mesh, chains, gap_loop, pieces[0].owner)?
            {
                return Ok(split.to_vec());
            }

            if corner_fill_free_ends::start_inside_an_end_arc(&field, &mut pieces[0]) {
                // An open chain's band as a closed loop of its own: both its free ends stand on its own
                // side group, so the loop runs down one rail, round one end, up the other and round
                // the other end. Started inside an end arc, it splits into its two rails.
                return Ok(corner_fill_free_ends::split_round_a_free_end(
                    chains, &pieces[0],
                ));
            }
        }
    }

    if pieces.len() < 2 {
        return Ok(pieces);
    }

    // A vertex where two chains' fields TIE belongs to both rails. A neighbour that does not tie
    // stays out, so no two strips ever lay the same stretch.
    let count = pieces.len();
    let mut extended = Vec::with_capacity(count);
    for p in 0..count {
        let piece = &pieces[p];
        let previous = (gap_loop.is_closed || p > 0).then(|| &pieces[(p + count - 1) % count]);
        let following = (gap_loop.is_closed || p < count - 1).then(|| &pieces[(p + 1) % count]);
        let before = match previous {
            Some(previous) if ties(mesh, chains, piece.owner, previous, false)? => {
                previous.vertices[previous.vertices.len() - 1]
            }
            _ => -1,
        };
        let after = match following {
            Some(following) if ties(mesh, chains, piece.owner, following, true)? => {
                following.vertices[0]
            }
            _ => -1,
        };
        extended.push((before, after));
    }

    for (piece, (before, after)) in pieces.iter_mut().zip(extended) {
        let field = &chains[piece.owner as usize];
        if before >= 0 {
            piece.vertices.insert(0, before);
            let at = field.evaluate(mesh.position(before))?.parameter;
            piece.parameters.insert(0, at);
        }

        if after >= 0 {
            piece.vertices.push(after);
            let at = field.evaluate(mesh.position(after))?.parameter;
            piece.parameters.push(at);
        }
    }

    let mut pieces: Vec<CornerFillPiece> = pieces
        .iter()
        .flat_map(|p| corner_fill_free_ends::split_round_a_free_end(chains, p))
        .collect();

    // A chain whose whole rail here is ties owns no vertex, so the split above gives it no piece
    // (every edge of a cube: each face's line is four ties). Its piece is its one run of tie
    // vertices - or two, the two rails of a both-sided member on one loop. An open run does not
    // wrap, so its tie run cannot either.
    if let Some(ties_at) = &gap_loop.vertex_ties {
        let mut tied: Vec<i32> = Vec::new();
        for &k in ties_at.iter().flatten() {
            if !tied.contains(&k) && pieces.iter().all(|p| p.owner != k) {
                tied.push(k);
            }
        }

        tied.sort();
        for k in tied {
            let starts: Vec<usize> = (0..n)
                .filter(|&i| {
                    ties_at[i].contains(&k)
                        && ((!gap_loop.is_closed && i == 0)
                            || !ties_at[(i + n - 1) % n].contains(&k))
                })
                .collect();
            if !(starts.len() == 1 || starts.len() == 2) {
                continue;
            }

            for run_start in starts {
                let mut piece = CornerFillPiece {
                    owner: k,
                    ..CornerFillPiece::default()
                };
                let mut i = run_start;
                while (gap_loop.is_closed || i < n)
                    && ties_at[i % n].contains(&k)
                    && piece.vertices.len() < n
                {
                    let vertex = gap_loop.vertices[i % n];
                    piece.vertices.push(vertex);
                    piece.parameters.push(
                        chains[k as usize]
                            .evaluate(mesh.position(vertex))?
                            .parameter,
                    );
                    i += 1;
                }

                pieces.push(piece);
            }
        }
    }

    Ok(pieces)
}
