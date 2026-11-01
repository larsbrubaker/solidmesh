//! Flow rung 2: a side's rail carried on VIRTUALLY past a neighbour that grows under the blend.
//! Where the band has run past an opposite-sense crease (a rib standing on the side), the cut
//! leaves the side's rail as two runs that stop on the crease, one each side of the neighbour. The
//! rail between them is laid in the side's own plane at the setback from the chain - for a fillet,
//! the tangent point of the row's circle on that plane - as points that were never on the mesh,
//! minted as new vertices so the fill's rails stay lists of vertex ids. The rows laid on them
//! overlap the neighbour until its plane clips the strip (the next rung). The joined loops
//! ([`joined_sides`]), the other side's run matched at the virtual points ([`matched_at`]), the
//! join itself ([`join`]), and the opposite-sense crease it is taken behind
//! ([`opposite_sense_crease_vertices`], [`chain_pair`]).
//!
//! The representation: a joined run is an ordinary [`GapLoop`] whose middle vertices are minted,
//! reported in the caller's list, and carry the chain parameter they were laid at - so nothing
//! downstream of the join tells a virtual point from a mesh one, and a pick that reaches no
//! opposite-sense crease never comes here. Assumes a STRAIGHT chain and a FLAT side across the
//! stretch (the away direction is read once, at the first run's end); a curved chain wants the
//! side's normal carried along, which no pick asks for yet.

use indexmap::{IndexMap, IndexSet};
use vector_math::ieee_math;

use super::{end_face_sense, triangle_edges};
use super::{
    BandGapResult, ChainField, GapLoop, LevelLineMesh, LevelLineMeshError, LevelLineTriangle,
    WorkError,
};

/// The gap's loops with each picked side's pair of runs joined where
/// a growing neighbour cut the rail in two; the loops themselves, untouched, wherever no side has
/// two runs that join. Every vertex minted on the way is appended to `minted`.
///
/// The gap's mesh is borrowed mutably while the runs are joined. A gap with no mesh (a refusal
/// raised before the working mesh existed - never handed here, since only a built gap is filled)
/// leaves its loops untouched.
pub fn joined_sides(
    gap: &BandGapResult,
    minted: &mut Vec<i32>,
) -> Result<Vec<GapLoop>, LevelLineMeshError> {
    if gap.chains.len() != 1 || gap.loops.iter().any(|l| l.is_closed) {
        return Ok(gap.loops.clone());
    }

    let Some(mesh) = gap.mesh.as_ref() else {
        return Ok(gap.loops.clone());
    };
    let mut mesh = mesh.borrow_mut();
    let chain = &*gap.chains[0];
    // Each side's rail is laid at its own setback where the rolling ball cut the sides apart.
    let rail_of = |side: i32| -> &ChainField {
        gap.side_chains
            .as_ref()
            .and_then(|sides| sides.first())
            .map_or(chain, |pair| &*pair[usize::from(side == 2)])
    };

    let mut loops = gap.loops.clone();
    for side in [1, 2] {
        // A stable sort, so ties keep their order; the loops are matched by place, so the runs
        // are held as indices into `loops`.
        let mut runs: Vec<usize> = (0..loops.len())
            .filter(|&i| loops[i].side == side)
            .collect();
        runs.sort_by(|&x, &y| {
            nan_first_compare(
                min_or_nan(&loops[x].parameters),
                min_or_nan(&loops[y].parameters),
            )
        });
        if runs.len() != 2 {
            continue;
        }

        let first = minted.len();
        let before_count = loops[runs[0]].vertices.len();
        let joined = join_at(
            &mut mesh,
            chain,
            rail_of(side),
            &loops[runs[0]],
            &loops[runs[1]],
            minted,
        )?;
        if let Some(joined) = joined {
            loops.remove(runs[0]);
            let after = if runs[1] > runs[0] {
                runs[1] - 1
            } else {
                runs[1]
            };

            // The row laid at a virtual point must be SQUARE to the chain, or it is not on the
            // round: a spoke to the other rail at another parameter lays a skewed arc standing off
            // the cylinder (the rib at 3.927: the front rail had 0, 12.15, 20, and the arc from the
            // virtual point at 10 stood up to 0.01 outside the round, and the tie on it with it).
            // So the other side's run gets a vertex at each virtual point's parameter, split into
            // its own rail edge.
            let parameters: Vec<f64> = joined
                .parameters
                .iter()
                .skip(before_count)
                .take(minted.len() - first)
                .copied()
                .collect();
            loops[after] = joined;
            let other: Vec<usize> = (0..loops.len())
                .filter(|&i| loops[i].side == 3 - side)
                .collect();
            if other.len() == 1 {
                loops[other[0]] =
                    matched_at(&mut mesh, chain, &loops[other[0]], &parameters, minted)?;
            }
        }
    }

    Ok(loops)
}

/// The run with a vertex at each of `parameters`, split into the rail
/// edge that spans it (a conforming split: the kept triangle on that edge is split with it). A
/// parameter at an existing vertex or outside the run leaves the run as it was there.
pub fn matched_at(
    mesh: &mut LevelLineMesh,
    chain: &ChainField,
    run: &GapLoop,
    parameters: &[f64],
    minted: &mut Vec<i32>,
) -> Result<GapLoop, LevelLineMeshError> {
    let mut vertices = run.vertices.clone();
    let mut at = run.parameters.clone();
    for &t in parameters {
        let mut i = 0;
        while i + 1 < vertices.len() {
            let (low, high) = (
                ieee_math::min_f64(at[i], at[i + 1]),
                ieee_math::max_f64(at[i], at[i + 1]),
            );
            if !(t > low && t < high) {
                i += 1;
                continue;
            }

            let a = mesh.position(vertices[i]);
            let b = mesh.position(vertices[i + 1]);
            let point = a + ((b - a) * ((t - at[i]) / (at[i + 1] - at[i])));
            let vertex = mesh.split_edge(vertices[i], vertices[i + 1], point, true)?;
            if vertex != vertices[i] && vertex != vertices[i + 1] {
                minted.push(vertex);
                vertices.insert(i + 1, vertex);
                at.insert(i + 1, chain.evaluate(mesh.position(vertex))?.parameter);
            }

            break;
        }
    }

    Ok(GapLoop::new(
        run.side,
        run.is_closed,
        vertices,
        at,
        run.chain,
        false,
        None,
    ))
}

/// Joins `before` (ending on the neighbour's crease) to `after` (starting
/// on it) through virtual points at the parameters of the neighbour's crease vertices between the
/// two ends. `None` when the two ends are not both on an opposite-sense crease of the side - the
/// path is taken only behind the sense test.
pub fn join(
    mesh: &mut LevelLineMesh,
    chain: &ChainField,
    before: &GapLoop,
    after: &GapLoop,
    minted: &mut Vec<i32>,
) -> Result<Option<GapLoop>, WorkError> {
    join_at(mesh, chain, chain, before, after, minted)
}

/// [`join`], each minted point laid at `rail`'s setback (the side's own field where the rolling
/// ball cuts the two sides at different distances); `chain` gives the parameters and points.
pub fn join_at(
    mesh: &mut LevelLineMesh,
    chain: &ChainField,
    rail: &ChainField,
    before: &GapLoop,
    after: &GapLoop,
    minted: &mut Vec<i32>,
) -> Result<Option<GapLoop>, WorkError> {
    let side = before.side;
    let crease_vertices = opposite_sense_crease_vertices(mesh, side);
    let a = before.vertices[before.vertices.len() - 1];
    let b = after.vertices[0];
    if !crease_vertices.contains(&a) || !crease_vertices.contains(&b) {
        return Ok(None);
    }

    let ta = before.parameters[before.parameters.len() - 1];
    let tb = after.parameters[0];
    let along = (chain.point_at(chain.length) - chain.point_at(0.0)).get_normal();
    let away = mesh.position(a) - chain.point_at(ta);
    let away = (away - (along * away.dot(along))).get_normal();

    let (low, high) = (ieee_math::min_f64(ta, tb), ieee_math::max_f64(ta, tb));
    // The parameters strictly between the ends, each value once (the first kept; NaN equals NaN),
    // in a stable sort.
    let mut between: Vec<f64> = Vec::new();
    for &v in &crease_vertices {
        let t = chain.evaluate(mesh.position(v))?.parameter;
        if t > low && t < high && !between.iter().any(|&u| nan_equal(u, t)) {
            between.push(t);
        }
    }

    between.sort_by(|&x, &y| {
        let key = |t: f64| if tb >= ta { t } else { -t };
        nan_first_compare(key(x), key(y))
    });

    let mut vertices = before.vertices.clone();
    let mut parameters = before.parameters.clone();
    for t in between {
        let vertex = mesh.add_vertex(chain.point_at(t) + (away * rail.setback_at(t)));
        minted.push(vertex);
        vertices.push(vertex);
        parameters.push(t);
    }

    vertices.extend_from_slice(&after.vertices);
    parameters.extend_from_slice(&after.parameters);
    Ok(Some(GapLoop::new(
        side,
        false,
        vertices,
        parameters,
        before.chain,
        false,
        None,
    )))
}

/// A total order on doubles in which NaN sorts below every number.
fn nan_first_compare(a: f64, b: f64) -> std::cmp::Ordering {
    match (a.is_nan(), b.is_nan()) {
        (true, true) => std::cmp::Ordering::Equal,
        (true, false) => std::cmp::Ordering::Less,
        (false, true) => std::cmp::Ordering::Greater,
        _ => a.partial_cmp(&b).unwrap_or(std::cmp::Ordering::Equal),
    }
}

/// The equality the parameter list is made distinct by: NaN equals NaN, and 0 equals -0.
fn nan_equal(a: f64, b: f64) -> bool {
    a == b || (a.is_nan() && b.is_nan())
}

/// The least of the values: NaN where any value is NaN.
fn min_or_nan(values: &[f64]) -> f64 {
    let mut least = values[0];
    for &v in values {
        if v.is_nan() {
            return v;
        }

        if v < least {
            least = v;
        }
    }

    least
}

/// A side-1 and a side-2 triangle meeting on an edge - the chain's fold, whose sense the crease is
/// read against - removed or not, since the cut takes both away, as (near, far, vertex). The least
/// such edge, so the read is fixed.
pub fn chain_pair(mesh: &LevelLineMesh) -> Option<(i32, i32, i32)> {
    let mut side_one: IndexMap<(i32, i32), i32> = IndexMap::new();
    for t in 0..mesh.triangle_count() {
        let triangle = mesh.triangles()[t as usize];
        if triangle.tag == 1 {
            for (p, q) in triangle_edges::level_line_triangle_corners(triangle) {
                side_one.insert(triangle_edges::key(p, q), t);
            }
        }
    }

    let mut best = None;
    let mut best_key = (i32::MAX, i32::MAX);
    for t in 0..mesh.triangle_count() {
        let triangle = mesh.triangles()[t as usize];
        if triangle.tag != 2 {
            continue;
        }

        for (p, q) in triangle_edges::level_line_triangle_corners(triangle) {
            let key = triangle_edges::key(p, q);
            if let Some(&near) = side_one.get(&key) {
                if key < best_key {
                    best_key = key;
                    best = Some((near, t, p));
                }
            }
        }
    }

    best
}

/// Every vertex on a crease between a removed triangle of `side`
/// and a kept untagged one that folds opposite to the chain. The side's triangles there are gone
/// after the cut, so the sense is read off the removed triangles themselves, and the chain's off
/// its two sides' triangles on the chain's fold ([`chain_pair`]), removed or not. Only looked up.
///
/// An `IndexSet`, so its order is the order the vertices were first met in.
pub fn opposite_sense_crease_vertices(mesh: &LevelLineMesh, side: i32) -> IndexSet<i32> {
    let corners = |t: LevelLineTriangle| (t.a, t.b, t.c);
    let mut found = IndexSet::new();

    let Some((near, far, vertex)) = chain_pair(mesh) else {
        return found;
    };

    for r in 0..mesh.triangle_count() {
        let removed = mesh.triangles[r as usize];

        if !mesh.is_removed(r) || removed.tag != side {
            continue;
        }

        for (p, q) in triangle_edges::corners(corners(removed)) {
            let standing = mesh.triangles_on(p, q);

            if standing.len() != 1 || mesh.triangles[standing[0] as usize].tag != 0 {
                continue;
            }

            if end_face_sense::is_opposite_sense_crease(
                mesh.vertices(),
                corners(mesh.triangles[near as usize]),
                corners(mesh.triangles[far as usize]),
                vertex,
                corners(removed),
                corners(mesh.triangles[standing[0] as usize]),
                p,
            ) {
                found.insert(p);
                found.insert(q);
            }
        }
    }

    found
}
