//! The gap fill's rail readings: a loop read both ways round ([`GapFillRailSenses`]), the fill's
//! constants and the
//! rail's monotone slack ([`monotone_slack`]) and the not-monotone refusal ([`not_monotone`]),
//! the rational-quadratic arc point ([`arc_point`]), an arc reversed ([`backwards`]) and a cycle's
//! wrap ([`largest_backward_step`]) and the chains' independence check
//! ([`no_two_chains_share_a_vertex`]), the rail senses' constructor and their pairing
//! ([`try_pair`]), the excursion rule ([`within_excursion`]), a loop's two senses ([`senses`],
//! [`try_envelope`]) and the zipper's merge ([`merge`]), and the fill's entry points ([`fill`],
//! [`fill_with_budget`], [`fill_within`]); one chain's fill is `gap_fill_one_chain`.

use std::collections::HashMap;
use std::rc::Rc;

use vector_math::{ieee_math, Vector3};

use super::corner_fill_fill::CornerFillError;
use super::gap_fill_one_chain::fill_one_chain;
use super::{
    corner_fill, corner_fill_fill, dying_crease_fill, open_gap_fill, swallowed_side, BandGapResult,
    BandProfile, ChainField, GapFillRefusal, GapFillResult, GapLoop, LevelLineMesh,
    LevelLineMeshError, LevelLineWork, LevelLineWorkPhase, WorkBudget, WorkError, ZipRail,
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

/// Why the fill could not run. A budget that runs out is not one of these - the
/// fill refuses by name ([`GapFillRefusal::WorkBudgetExceeded`]) and hands back a result.
#[derive(Debug, Clone)]
pub enum GapFillError {
    /// The caller cancelled: it says nothing about the part,
    /// so it is never a refusal.
    Cancelled,
    /// The mesh refused an edit - or, inside [`fill_within`], the work budget's own exit, which
    /// [`fill_with_budget`] turns into a refusal or [`GapFillError::Cancelled`].
    Mesh(LevelLineMeshError),
    /// A caller's bug: an argument that is malformed or out of range.
    Argument(String),
}

impl From<LevelLineMeshError> for GapFillError {
    fn from(error: LevelLineMeshError) -> Self {
        GapFillError::Mesh(error)
    }
}

impl From<WorkError> for GapFillError {
    fn from(error: WorkError) -> Self {
        GapFillError::Mesh(error.into())
    }
}

impl From<CornerFillError> for GapFillError {
    fn from(error: CornerFillError) -> Self {
        match error {
            CornerFillError::Mesh(error) => GapFillError::Mesh(error),
            CornerFillError::FilletSegments(message) => GapFillError::Argument(message),
        }
    }
}

impl std::fmt::Display for GapFillError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            GapFillError::Cancelled => f.write_str("the gap fill was cancelled by its caller"),
            GapFillError::Mesh(error) => write!(f, "{error}"),
            GapFillError::Argument(message) => f.write_str(message),
        }
    }
}

impl std::error::Error for GapFillError {}

/// Lays the strip between a closed gap's two
/// loops, spending the default work for the mesh's size - [`fill_with_budget`] with no budget.
/// The default of four fillet segments is [`DEFAULT_FILLET_SEGMENTS`].
pub fn fill(
    gap: &BandGapResult,
    profile: BandProfile,
    fillet_segments: i32,
) -> Result<GapFillResult, GapFillError> {
    fill_with_budget(gap, profile, fillet_segments, None)
}

/// How many segments a fillet's arc gets when the caller names none.
pub const DEFAULT_FILLET_SEGMENTS: i32 = 4;

/// Fills the gap, spending no more than
/// the given work (`None` takes the default for the mesh's size). A fill that runs out refuses by
/// name and hands back nothing part-built; a cancel leaves as [`GapFillError::Cancelled`].
pub fn fill_with_budget(
    gap: &BandGapResult,
    profile: BandProfile,
    fillet_segments: i32,
    budget: Option<Rc<WorkBudget>>,
) -> Result<GapFillResult, GapFillError> {
    let budget = match budget {
        Some(budget) => budget,
        None => {
            let triangle_count = gap
                .mesh
                .as_ref()
                .map_or(0, |mesh| mesh.borrow().triangle_count());
            Rc::new(
                WorkBudget::new(WorkBudget::default_limit(triangle_count))
                    .expect("the default limit is a positive count of work"),
            )
        }
    };
    if budget.cancellation_token.is_cancelled() {
        return Err(GapFillError::Cancelled);
    }

    let _scope = LevelLineWork::begin(Some(Rc::clone(&budget)));
    LevelLineWork::enter(LevelLineWorkPhase::Fill);
    let filled = if corner_fill::takes(Some(gap), profile) {
        corner_fill_fill::fill(gap, profile, fillet_segments).map_err(GapFillError::from)
    } else {
        fill_within(gap, profile, fillet_segments)
    };
    match filled {
        // The caller asked for this, so it leaves as cancellation and never as a refusal: a
        // refusal is a statement about the part, and this one says nothing about the part at all.
        Err(GapFillError::Mesh(LevelLineMeshError::Work(WorkError::Cancelled))) => {
            Err(GapFillError::Cancelled)
        }
        Err(GapFillError::Mesh(LevelLineMeshError::Work(WorkError::BudgetExceeded(_)))) => {
            Ok(GapFillResult {
                refusal: GapFillRefusal::WorkBudgetExceeded,
                // The mesh travels with this refusal like it does with every other: the fill has
                // taken its part-laid strip back out, and a caller that cannot see the mesh
                // cannot check that.
                mesh: gap.mesh.clone(),
                phase: budget.exhausted_in.get(),
                work_spent: budget.spent.get(),
                work_limit: budget.limit,
                ..GapFillResult::default()
            })
        }
        other => other,
    }
}

/// The fill under the budget [`fill_with_budget`] put in force. Routes a
/// gap of swallowed runs to `swallowed_side`, an open one to `open_gap_fill`, and otherwise fills
/// each chain of the pick between its own two loops; one chain's refusal takes the WHOLE pick's
/// strip back out. The work budget's exit leaves as `Err` with every laid triangle removed.
pub fn fill_within(
    gap: &BandGapResult,
    profile: BandProfile,
    fillet_segments: i32,
) -> Result<GapFillResult, GapFillError> {
    if !gap.built() {
        return Err(GapFillError::Argument(
            "a refused gap has nothing to fill (Parameter 'gap')".to_string(),
        ));
    }

    if fillet_segments < 1 {
        return Err(GapFillError::Argument(
            "an arc is drawn with at least one segment (Parameter 'filletSegments')".to_string(),
        ));
    }

    if gap.swallowed_runs.is_some() {
        // Both sides swallowed whole: no run of the cut's, the strip is laid on runs made up off
        // the mesh.
        return Ok(swallowed_side::fill(gap, profile, fillet_segments)?);
    }

    if gap.loops.iter().any(|l| !l.is_closed)
        || dying_crease_fill::is_one_loop_round_an_open_chain(Some(gap))
    {
        // WHAT THE CUT LEFT tells the two open cases apart, never a guess from the chain's shape.
        // A crease that CARRIES ON past both ends leaves two open runs sharing their end vertices
        // and fades out (see open_gap_fill). A chain ending on an END FACE leaves its runs ending
        // APART, with the end face's surface between them; that needs an end flood and a cap.
        return Ok(open_gap_fill::try_open(gap, profile, fillet_segments)?);
    }

    let mesh = gap.mesh.as_ref().expect("a built gap carries its mesh");
    no_two_chains_share_a_vertex(&gap.loops);

    let mut added_triangles = Vec::new();
    let mut added_vertices = Vec::new();
    let mut triangles_of_chain = vec![0; gap.chains.len()];
    let mut vertices_of_chain = vec![0; gap.chains.len()];
    for k in 0..gap.chains.len() {
        let mine: Vec<GapLoop> = gap
            .loops
            .iter()
            .filter(|l| l.chain == k as i32)
            .cloned()
            .collect();
        let one = if mine.len() != 2
            || mine.iter().filter(|l| l.side == 1).count() != 1
            || mine.iter().filter(|l| l.side == 2).count() != 1
        {
            GapFillResult {
                refusal: GapFillRefusal::SideDoesNotLeaveOneLoop,
                mesh: Some(Rc::clone(mesh)),
                ..GapFillResult::default()
            }
        } else {
            match fill_one_chain(
                mesh,
                &gap.chains[k],
                &mine,
                profile,
                fillet_segments,
                &mut added_triangles,
                &mut HashMap::new(),
            ) {
                Ok(one) => one,
                Err(error @ LevelLineMeshError::Work(WorkError::BudgetExceeded(_))) => {
                    // The budget runs out in the middle of a strip, so the pick is left
                    // part-zipped; take every triangle laid so far back out before the refusal
                    // leaves. The chain's own fill appends to this very list as it goes, which is
                    // what makes the partial work visible here at all.
                    let mut edit = mesh.borrow_mut();
                    for &triangle in &added_triangles {
                        edit.remove_triangle(triangle);
                    }

                    return Err(error.into());
                }
                Err(error) => return Err(error.into()),
            }
        };

        if !one.filled() {
            // One chain's fill refusing takes the WHOLE pick's strip back out - every chain's, not
            // just this one's. A half-filled mesh is not a solid, and handing one back so that
            // something downstream could sew up the rest is the repair pass this construction
            // does not have.
            let mut edit = mesh.borrow_mut();
            for &triangle in &added_triangles {
                edit.remove_triangle(triangle);
            }

            return Ok(GapFillResult {
                refusal: one.refusal,
                offending_vertex: one.offending_vertex,
                open_edge: one.open_edge,
                offending_chain: k as i32,
                mesh: Some(Rc::clone(mesh)),
                ..GapFillResult::default()
            });
        }

        added_vertices.extend_from_slice(&one.added_vertices);
        triangles_of_chain[k] = one.added_triangles.len() as i32;
        vertices_of_chain[k] = one.added_vertices.len() as i32;
    }

    Ok(GapFillResult {
        refusal: GapFillRefusal::None,
        mesh: Some(Rc::clone(mesh)),
        added_triangles,
        added_vertices,
        triangles_of_chain,
        vertices_of_chain,
        ..GapFillResult::default()
    })
}

/// How far back a rail's parameter may step on this chain and still count as standing still.
pub fn monotone_slack(chain: &ChainField) -> f64 {
    ieee_math::max_f64(
        PARAMETER_TIE_RELATIVE * chain.length,
        MONOTONE_SLACK_OF_SETBACK * chain.min_setback,
    )
}

/// A loop read both ways round, with what each reading cost in
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
    /// A loop's two readings, forward and backward.
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

/// Picks one sense for each loop so that exactly one of the two rails
/// reverses its loop, taking the pairing that flattened less (the left forward on a tie); `None`
/// where neither pairing reads monotone. Where both loops read monotone BOTH ways - a short loop
/// whose whole parameter range is inside the slack reads either way round - this is the only thing
/// that decides the sense, and getting it wrong lays every spoke twice the same way and leaves the
/// shell open. (Only a NaN cost could pick a pairing with a missing sense; that answers `None`
/// rather than a pairing with no rail.)
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

    if ieee_math::min_f64(forward_backward, backward_forward) == f64::INFINITY {
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

/// The point at `t` on the rational quadratic from `p` to `q` with control
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
    let weight = ieee_math::max_f64(0.0, (1.0 - cos_at_c) / 2.0).sqrt();

    let s = 1.0 - t;
    let wp = s * s;
    let wc = 2.0 * t * s * weight;
    let wq = t * t;
    ((p * wp) + (c * wc) + (q * wq)) / (wp + wc + wq)
}

/// The cell between two arcs that share their far end - rows of quads,
/// collapsing to one triangle at the shared vertex. `u` starts at the cell's first boundary vertex
/// and `v` at its second; `base_run`, where it has more than two vertices, is the rail's stretch
/// from `v[0]` to `u[0]` that the base triangle fans over.
pub fn strip(
    mesh: &mut LevelLineMesh,
    u: &[i32],
    v: &[i32],
    added: &mut Vec<i32>,
    base_run: Option<&[i32]>,
) -> Result<(), LevelLineMeshError> {
    let last = u.len() as i32 - 1;
    let fans = base_run.filter(|run| run.len() > 2);
    for r in 0..(last - 1).max(0) as usize {
        if r == 0 {
            if let Some(run) = fans {
                for k in 0..run.len() - 1 {
                    lay(mesh, run[k + 1], run[k], v[1], added)?;
                }

                lay(mesh, u[r], v[r + 1], u[r + 1], added)?;
                continue;
            }
        }

        lay(mesh, u[r], v[r], v[r + 1], added)?;
        lay(mesh, u[r], v[r + 1], u[r + 1], added)?;
    }

    let last = last as usize;
    if last == 1 {
        if let Some(run) = fans {
            for k in 0..run.len() - 1 {
                lay(mesh, run[k + 1], run[k], u[last], added)?;
            }

            return Ok(());
        }
    }

    lay(mesh, u[last - 1], v[last - 1], u[last], added)
}

/// The cell between two arcs of one station count whose ends both differ, as a
/// row of quads wound as [`strip`] winds them.
pub fn quads(
    mesh: &mut LevelLineMesh,
    u: &[i32],
    v: &[i32],
    added: &mut Vec<i32>,
) -> Result<(), LevelLineMeshError> {
    for r in 0..u.len().saturating_sub(1) {
        lay(mesh, u[r], v[r], v[r + 1], added)?;
        lay(mesh, u[r], v[r + 1], u[r + 1], added)?;
    }

    Ok(())
}

/// Lays one fill triangle `(a, b, c)` into `mesh`, tagged as fill, and records it in `added`. A
/// triangle with exactly one repeated pair is skipped; one whose three corners are all the same
/// vertex still reaches [`LevelLineMesh::add_face`], which refuses it.
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

/// The arc's vertices in the other order.
pub fn backwards(arc: &[i32]) -> Vec<i32> {
    arc.iter().rev().copied().collect()
}

/// The index of the LARGEST backward step round the cycle, read
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

/// The independence the per-chain
/// fill rests on, asserted rather than assumed: no vertex of one chain's loops stands on
/// another's. Debug builds only (`debug_assert!`) - what it checks is
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

/// The one rule that says whether a backward reading may be flattened
/// onto the envelope or the rail refused: an excursion a concave turn can explain, and no deeper
/// ([`MONOTONE_EXCURSION_OF_SETBACK`] setbacks past the slack). Open mode asks THIS one too -
/// flattening without a bound would crush a crossing rail monotone, still closed by construction,
/// and ship a self-intersecting strip.
pub fn within_excursion(envelope: f64, reading: f64, chain: &ChainField, slack: f64) -> bool {
    envelope - reading <= slack + (MONOTONE_EXCURSION_OF_SETBACK * chain.setback_at(envelope))
}

/// Re-expresses a loop as a rail BOTH ways round: each sense is cut open at
/// its own largest backward step and takes its monotone envelope, and reports what that envelope
/// had to flatten. Nothing is sorted - a loop whose parameter wanders is a fact about the cut, not
/// something to tidy away - and nothing here picks a sense, because only the PAIR of loops can say
/// which sense is which.
pub fn senses(gap_loop: &GapLoop, chain: &ChainField, slack: f64) -> GapFillRailSenses {
    let mut vertices = gap_loop.vertices.clone();
    let mut parameters = gap_loop.parameters.clone();
    let (forward, forward_cost, forward_offender) = try_envelope(
        &vertices,
        &parameters,
        largest_backward_step(&gap_loop.parameters, false),
        false,
        chain,
        slack,
    );

    vertices.reverse();
    parameters.reverse();
    let (backward, backward_cost, backward_offender) = try_envelope(
        &vertices,
        &parameters,
        largest_backward_step(&gap_loop.parameters, true),
        true,
        chain,
        slack,
    );

    let offender = if backward_offender < 0 {
        forward_offender
    } else {
        backward_offender
    };
    GapFillRailSenses::new(
        forward.map(Rc::new),
        forward_cost,
        backward.map(Rc::new),
        backward_cost,
        gap_loop.vertices[0.max(offender) as usize],
    )
}

/// Rotates a cycle to begin just after its `wrap` and takes its
/// MONOTONE ENVELOPE - each point's parameter is the greatest reading so far - refusing when an
/// excursion below that envelope is deeper than a concave turn can explain
/// ([`within_excursion`]). Answers (the rail, or `None` on a refusal; the parameter flattened so
/// far; the offending index into the cycle as given, or -1).
pub fn try_envelope(
    vertices: &[i32],
    parameters: &[f64],
    wrap: i32,
    reversed: bool,
    chain: &ChainField,
    slack: f64,
) -> (Option<ZipRail>, f64, i32) {
    let mut flattened = 0.0;
    let count = vertices.len() as i32;
    let start = (wrap + 1) % count;
    let mut v = vec![0; count as usize];
    let mut p = vec![0.0; count as usize];
    for i in 0..count as usize {
        let index = ((start + i as i32) % count) as usize;
        v[i] = vertices[index];
        let mut reading = parameters[index];
        if i > 0 && reading < p[i - 1] {
            if !within_excursion(p[i - 1], reading, chain, slack) {
                return (None, flattened, index as i32);
            }

            flattened += p[i - 1] - reading;
            reading = p[i - 1];
        }

        p[i] = reading;
    }

    let rail = ZipRail {
        vertices: v,
        parameters: p,
        reversed,
    };
    (Some(rail), flattened, -1)
}

/// The zipper itself - for each state, which rail advances next. A state is
/// (left advances so far, right advances so far); the walk runs from (0, 0) to (n, m), which is
/// the very same spoke again. Across a JUMP the spoke the advance makes is the shorter in parameter
/// of the two it could make; elsewhere the usual advance gives way where its spoke would cut across
/// a turn of the chain.
pub fn merge(
    mesh: &LevelLineMesh,
    chain: &ChainField,
    left: &ZipRail,
    right: &ZipRail,
    length: f64,
    right_offset: f64,
) -> Result<Vec<(i32, i32, bool)>, WorkError> {
    let mut steps = Vec::new();
    let mut a = 0;
    let mut b = 0;

    // A corner's two rail vertices mean the same parameter but are computed by different roots, so
    // "the same" has to allow the last bits.
    let tie = PARAMETER_TIE_RELATIVE * length;
    while a < left.count() || b < right.count() {
        // The merge advances one rail per step, so this is bounded - charged all the same, because
        // the step that advances neither is exactly the runaway this catches.
        LevelLineWork::charge(1)?;
        let next_left = if a < left.count() {
            ZipRail::unwrapped(left, a + 1, length, 0.0)
        } else {
            f64::INFINITY
        };
        let next_right = if b < right.count() {
            ZipRail::unwrapped(right, b + 1, length, right_offset)
        } else {
            f64::INFINITY
        };
        let mut advances_left = next_left <= next_right + tie;
        if a < left.count()
            && b < right.count()
            && (ZipRail::is_a_jump(mesh, left, a, length)
                || ZipRail::is_a_jump(mesh, right, b, length))
        {
            let left_spoke = (next_left - ZipRail::unwrapped(right, b, length, right_offset)).abs();
            let right_spoke = (ZipRail::unwrapped(left, a, length, 0.0) - next_right).abs();
            advances_left = left_spoke <= right_spoke + tie;
        } else if a < left.count() && b < right.count() {
            advances_left ^= chain.spoke_gives_way_across_a_turn(
                ZipRail::unwrapped(left, a, length, 0.0),
                next_left,
                ZipRail::unwrapped(right, b, length, right_offset),
                next_right,
                advances_left,
            );
        }

        steps.push((a, b, advances_left));
        if advances_left {
            a += 1;
        } else {
            b += 1;
        }
    }

    Ok(steps)
}

/// A closed chain's zip's arcs, one per advance - the straight spoke at one
/// segment, otherwise the fillet's row carried along the chain between the two rails' unwrapped
/// readings ([`super::fillet_arc_rows::along_the_chain`]) - with the first repeated at the end to
/// close the loop.
#[allow(clippy::too_many_arguments)]
pub fn arcs(
    mesh: &mut LevelLineMesh,
    chain: &super::ChainField,
    advances: &[(i32, i32, bool)],
    left: &super::zip_rail::ZipRail,
    right: &super::zip_rail::ZipRail,
    left_span: &[(f64, f64)],
    right_span: &[(f64, f64)],
    length: f64,
    right_offset: f64,
    segments: i32,
    added_vertices: &mut Vec<i32>,
) -> Vec<Vec<i32>> {
    use super::zip_rail::ZipRail;

    let mut arcs: Vec<Vec<i32>> = Vec::new();
    let tie = PARAMETER_TIE_RELATIVE * length;
    for &(advance_left, advance_right, _) in advances {
        let (i, j) = (
            (advance_left % left.count()) as usize,
            (advance_right % right.count()) as usize,
        );
        if segments == 1 {
            arcs.push(vec![left.vertices[i], right.vertices[j]]);
            continue;
        }

        let left_at = ZipRail::unwrapped(left, advance_left, length, 0.0);
        let right_at = ZipRail::unwrapped(right, advance_right, length, right_offset);
        let from = (
            left.vertices[i],
            left_at,
            left_span[i],
            left_at - left.parameters[i],
        );
        let to = (
            right.vertices[j],
            right_at,
            right_span[j],
            right_at - right.parameters[j],
        );
        arcs.push(super::fillet_arc_rows::along_the_chain(
            mesh,
            chain,
            from,
            to,
            segments,
            tie,
            added_vertices,
        ));
    }

    arcs.push(arcs[0].clone());
    arcs
}
