//! The gap fill's rail readings. Ports the nested type of `Bevel/LevelLine/GapFill.cs` (a C#
//! static class): a loop read both ways round ([`GapFillRailSenses`]), the class's constants and the
//! rail's monotone slack ([`monotone_slack`]) and the not-monotone refusal ([`not_monotone`]),
//! the rational-quadratic arc point ([`arc_point`]), an arc reversed ([`backwards`]) and a cycle's
//! wrap ([`largest_backward_step`]) and the chains' independence check
//! ([`no_two_chains_share_a_vertex`]), the rail senses' constructor and their pairing
//! ([`try_pair`]) and the excursion rule ([`within_excursion`]). The fill's
//! other functions land with later packages.

use std::collections::HashMap;
use std::rc::Rc;

use vector_math::{cs_math, Vector3};

use super::{
    ChainField, GapFillRefusal, GapFillResult, GapLoop, LevelLineMesh, LevelLineMeshError, ZipRail,
};
use std::cell::RefCell;

/// The tag the fill's own triangles carry, so the strip can be told from the kept surface. Stood
/// clear of every side-group id (a pick of several chains numbers its groups 1, 2, 3, ...).
pub const FILL_TAG: i32 = i32::MAX;

/// How close two rail parameters must be, relative to the chain's length, to count as the same
/// point of the chain in the zipper.
pub const PARAMETER_TIE_RELATIVE: f64 = 1e-9;

/// How far a rail's chain parameter may step BACKWARDS, as a fraction of the chain's least
/// setback, and still count as standing still rather than as a wrap.
///
/// Two readings measured on real parts say why this is not zero. Where the cut leaves two vertices
/// at the SAME position (77930's outer rail carries nine such pairs), their parameters are computed
/// by two different roots and differ in the last bits, so an exact comparison reads nine extra
/// wraps in a loop that is otherwise perfectly ordered. And where the rail curves round the far
/// side of a boss (65394's outer rail), the parameter genuinely wobbles - by 4e-4 on a chain 3.2
/// long, a thousandth of the setback. A backstep under this slack moves an arc's chain point by
/// less than a hundredth of the setback, below the chord tolerance the cut itself was traced to,
/// and the rail comes out monotone BY CONSTRUCTION, which is what the zipper's merge assumes.
pub const MONOTONE_SLACK_OF_SETBACK: f64 = 1e-2;

/// How deep an excursion below a rail's monotone envelope may run, as a multiple of the local
/// setback, before the rail is refused. On the INSIDE of a concave turn the minimising parameter
/// honestly swings BACK as the rail rounds the turn, by at most twice the setback over straight
/// legs; the fifth Thingi10K sweep's 62 non-monotone inputs split into 48 excursions at 2.19
/// setbacks or less and the rest from 7.39 up - correspondences that really do cross. Three is the
/// bound between those two populations.
pub const MONOTONE_EXCURSION_OF_SETBACK: f64 = 3.0;

/// How far back a rail's parameter may step on this chain and still count as standing still.
pub fn monotone_slack(chain: &ChainField) -> f64 {
    cs_math::max_f64(
        PARAMETER_TIE_RELATIVE * chain.length,
        MONOTONE_SLACK_OF_SETBACK * chain.min_setback,
    )
}

/// C# `GapFill.RailSenses`: a loop read both ways round, with what each reading cost in
/// flattening. Nothing picks a sense here, because only the PAIR of loops can say which sense is
/// which.
#[derive(Debug, Clone, PartialEq, Default)]
pub struct GapFillRailSenses {
    /// The loop read in its own walk's sense, or `None` when that reading is not monotone.
    pub forward: Option<Rc<ZipRail>>,
    /// How much parameter the forward reading had to flatten away.
    pub forward_cost: f64,
    /// The loop read against its walk, or `None` when that reading is not monotone.
    pub backward: Option<Rc<ZipRail>>,
    /// How much parameter the backward reading had to flatten away.
    pub backward_cost: f64,
    /// Where the deeper of the two refusals happened, for a refusal to name.
    pub offending: i32,
}

impl GapFillRailSenses {
    /// C#'s constructor.
    pub fn new(
        forward: Option<Rc<ZipRail>>,
        forward_cost: f64,
        backward: Option<Rc<ZipRail>>,
        backward_cost: f64,
        offending: i32,
    ) -> Self {
        GapFillRailSenses {
            forward,
            forward_cost,
            backward,
            backward_cost,
            offending,
        }
    }
}

/// C# `GapFill.TryPair`: picks one sense for each loop so that exactly one of the two rails
/// reverses its loop, taking the pairing that flattened less (the left forward on a tie); `None`
/// where neither pairing reads monotone. Where both loops read monotone BOTH ways - a short loop
/// whose whole parameter range is inside the slack reads either way round - this is the only thing
/// that decides the sense, and getting it wrong lays every spoke twice the same way and leaves the
/// shell open. (Only a NaN cost could pick a pairing with a missing sense; C# then answers true
/// with a null rail, this `None`.)
pub fn try_pair(
    left: &GapFillRailSenses,
    right: &GapFillRailSenses,
) -> Option<(Rc<ZipRail>, Rc<ZipRail>)> {
    let forward_backward = if left.forward.is_some() && right.backward.is_some() {
        left.forward_cost + right.backward_cost
    } else {
        f64::INFINITY
    };
    let backward_forward = if left.backward.is_some() && right.forward.is_some() {
        left.backward_cost + right.forward_cost
    } else {
        f64::INFINITY
    };

    if cs_math::min_f64(forward_backward, backward_forward) == f64::INFINITY {
        return None;
    }

    if forward_backward <= backward_forward {
        Some((left.forward.clone()?, right.backward.clone()?))
    } else {
        Some((left.backward.clone()?, right.forward.clone()?))
    }
}

/// The refusal for a rail whose chain parameter is not monotone, naming the offending vertex.
pub fn not_monotone(mesh: Rc<RefCell<LevelLineMesh>>, offending_vertex: i32) -> GapFillResult {
    GapFillResult {
        refusal: GapFillRefusal::RailIsNotMonotone,
        offending_vertex,
        mesh: Some(mesh),
        ..GapFillResult::default()
    }
}

/// C# `GapFill.ArcPoint`: the point at `t` on the rational quadratic from `p` to `q` with control
/// point `c` and middle weight cos(theta / 2), theta being the angle the arc subtends, which is the
/// supplement of the angle PCQ. Where |P - C| = |Q - C| that is exactly the circular arc tangent
/// to CP at P and to CQ at Q, centred on P + Q - C.
///
/// cos(theta / 2) with theta = pi - angle(PCQ) is sqrt((1 - cos angle(PCQ)) / 2), so the weight is
/// read straight off the dot product without ever taking an angle. Collinear P, C, Q give weight 1
/// (the straight line), a right angle at C gives sqrt(2) / 2 - the quarter circle.
pub fn arc_point(p: Vector3, c: Vector3, q: Vector3, t: f64) -> Vector3 {
    let to_p = p - c;
    let to_q = q - c;
    let lengths = to_p.length() * to_q.length();
    let cos_at_c = if lengths > 0.0 {
        (to_p.dot(to_q) / lengths).clamp(-1.0, 1.0)
    } else {
        1.0
    };
    let weight = cs_math::max_f64(0.0, (1.0 - cos_at_c) / 2.0).sqrt();

    let s = 1.0 - t;
    let wp = s * s;
    let wc = 2.0 * t * s * weight;
    let wq = t * t;
    ((p * wp) + (c * wc) + (q * wq)) / (wp + wc + wq)
}

/// Lays one fill triangle `(a, b, c)` into `mesh`, tagged as fill, and records it in `added`. A
/// triangle with exactly one repeated pair is skipped; one whose three corners are all the same
/// vertex still reaches [`LevelLineMesh::add_face`], which refuses it (as the C# does).
pub fn lay(
    mesh: &mut LevelLineMesh,
    a: i32,
    b: i32,
    c: i32,
    added: &mut Vec<i32>,
) -> Result<(), LevelLineMeshError> {
    let same = i32::from(a == b) + i32::from(b == c) + i32::from(c == a);
    if same == 1 {
        return Ok(());
    }

    added.push(mesh.add_face(a, b, c, FILL_TAG)?);
    Ok(())
}

/// C# `GapFill.Backwards`: the arc's vertices in the other order.
pub fn backwards(arc: &[i32]) -> Vec<i32> {
    arc.iter().rev().copied().collect()
}

/// C# `GapFill.LargestBackwardStep`: the index of the LARGEST backward step round the cycle, read
/// in the given sense: that step is the wrap, where the rail comes back round to the chain's start,
/// and the rail is cut open there. -1 where no step goes backward.
pub fn largest_backward_step(parameters: &[f64], reversed: bool) -> i32 {
    let count = parameters.len();
    let mut wrap = -1;
    let mut largest = 0.0;
    for i in 0..count {
        let index = if reversed { count - 1 - i } else { i };
        let here = parameters[index];
        let next = parameters[if reversed {
            count - 1 - ((i + 1) % count)
        } else {
            (i + 1) % count
        }];
        let drop = here - next;
        if drop > largest {
            largest = drop;
            wrap = i as i32;
        }
    }

    wrap
}

/// C# `GapFill.NoTwoChainsShareAVertex` (`[Conditional("DEBUG")]`): the independence the per-chain
/// fill rests on, asserted rather than assumed: no vertex of one chain's loops stands on
/// another's. Debug builds only (`debug_assert!`, as C#'s conditional method) - what it checks is
/// a consequence of the cut, so a release build paying for it every fill would learn nothing.
pub fn no_two_chains_share_a_vertex(loops: &[GapLoop]) {
    if !cfg!(debug_assertions) {
        return;
    }

    let mut owner: HashMap<i32, i32> = HashMap::new();
    for gap_loop in loops {
        for &vertex in &gap_loop.vertices {
            if let Some(&first) = owner.get(&vertex) {
                debug_assert!(
                    first == gap_loop.chain,
                    "vertex {vertex} stands on chain {first}'s loops and on chain {}'s: the pick's \
                     bands are not disjoint and the fills are not independent",
                    gap_loop.chain
                );
            }

            owner.insert(vertex, gap_loop.chain);
        }
    }
}

/// C# `GapFill.WithinExcursion`: the one rule that says whether a backward reading may be flattened
/// onto the envelope or the rail refused: an excursion a concave turn can explain, and no deeper
/// ([`MONOTONE_EXCURSION_OF_SETBACK`] setbacks past the slack). Open mode asks THIS one too -
/// flattening without a bound would crush a crossing rail monotone, still closed by construction,
/// and ship a self-intersecting strip.
pub fn within_excursion(envelope: f64, reading: f64, chain: &ChainField, slack: f64) -> bool {
    envelope - reading <= slack + (MONOTONE_EXCURSION_OF_SETBACK * chain.setback_at(envelope))
}
