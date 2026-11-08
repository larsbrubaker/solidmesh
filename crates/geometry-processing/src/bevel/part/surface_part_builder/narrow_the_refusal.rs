// Narrowing a refusal that names nobody: where a pass refuses and neither the kernel's stamp nor
// the place it names picks out a loop (a corner about equally near two or more picked edges), the
// one pass (`build`) asks this file for the smallest set of its loops that still refuses on its
// own, so only those are refused and the rest of the pick builds. The search itself is
// `the_smallest_refusing_set`, a plain function of a yes/no question so its order and its cap are
// pinned without a mesh; `the_loops_that_still_refuse` asks it of the real builder (`try_together`).

use std::rc::Rc;

use crate::bevel::{BevelAdvisory, BevelGroupResult, BevelLoopFailure};
use indexmap::IndexMap;
use solidmesh_mesh::CancelToken;

use super::build::check;
use super::{BevelSurfacePartBuilder, RimLoop, SurfacePartBuildError};

/// What a narrowed refusal comes to: the loops it belongs to and the reason they gave on their own.
pub struct NarrowedRefusal {
    /// The smallest set found that still refuses, in the pass's own order.
    pub loops: Vec<Rc<RimLoop>>,
    /// Why that set refuses on its own.
    pub reason: BevelAdvisory,
    /// Whether that reason is about the size (`bevel_group_failures::diagnose` reads it).
    pub about_the_size: bool,
}

/// The smallest subset of `items` found for which `refuses` still answers with a reason, starting
/// from the whole set (known to refuse with `first`), and that subset's reason, in `items`' order.
///
/// The loops that must stay for the refusal to stand are found one at a time: the shortest
/// leading run of what is left that, beside the loops already found, still refuses is found by
/// bisection, and its last loop is one that must stay. Once the loops found refuse on their own,
/// they are the answer. A corner that involves k loops among n costs about k * log2(n) attempts,
/// wherever in the pick's order those loops sit - halving the pick alone would refuse everybody
/// where the two loops of a ramp-meets-post corner fall in different halves. Every step is in the
/// set's own order, so the answer is the same on every run.
///
/// `attempts_allowed` caps how many questions are asked - a count rather than a clock, so the
/// answer does not depend on how loaded the machine is. Past it the set narrowed so far is the
/// answer, which is always a set that really refused.
pub fn the_smallest_refusing_set<T: Clone, R, E>(
    items: &[T],
    first: R,
    attempts_allowed: usize,
    mut refuses: impl FnMut(&[T]) -> Result<Option<R>, E>,
) -> Result<(Vec<T>, R), E> {
    // Positions into `items`: the loops found to be needed, and the leading run still in doubt.
    // Together they always make a set known to refuse, with `why` its reason.
    let mut needed: Vec<usize> = Vec::new();
    let mut in_doubt = items.len();
    let mut why = first;
    let mut asked = 0usize;

    let the_set = |needed: &[usize], run: usize| -> Vec<T> {
        let mut positions: Vec<usize> = needed.iter().copied().chain(0..run).collect();
        // Distinct positions, so the sort has no ties; it puts the set in the pick's order.
        positions.sort_unstable();
        positions.iter().map(|&k| items[k].clone()).collect()
    };

    'narrowing: while in_doubt > 0 {
        if !needed.is_empty() {
            if asked >= attempts_allowed {
                break;
            }
            asked += 1;

            if let Some(reason) = refuses(&the_set(&needed, 0))? {
                why = reason;
                in_doubt = 0;
                break;
            }
        }

        // The shortest leading run that still refuses beside `needed`: `low` is a run known not
        // to (the empty run beside `needed`, which was just asked or is the empty set), `high` one
        // known to.
        let mut low = 0usize;
        let mut high = in_doubt;

        while high - low > 1 {
            if asked >= attempts_allowed {
                in_doubt = high;
                break 'narrowing;
            }
            asked += 1;

            let middle = low + (high - low) / 2;

            if let Some(reason) = refuses(&the_set(&needed, middle))? {
                why = reason;
                high = middle;
            } else {
                low = middle;
            }
        }

        needed.push(high - 1);
        in_doubt = high - 1;
    }

    Ok((the_set(&needed, in_doubt), why))
}

impl BevelSurfacePartBuilder {
    /// How many attempts narrowing one refusal may make. Enough to pin the two or three loops a
    /// corner involves among the 56 of a large pick (about seven attempts a loop), and a bound on
    /// what a refusal that names nobody can cost: each attempt is one build of at most the loops
    /// the pass already built once.
    pub const NARROWING_ATTEMPTS_ALLOWED: usize = 24;

    /// The smallest set of `contributing` found that still refuses on its own, with its reason -
    /// or `None` where narrowing found nothing smaller than the whole pass, which is then the
    /// answer. `first` and `about_the_size` are the whole pass's refusal.
    ///
    /// Each question is one attempt (`try_these_groups_together`) at the operation's own size,
    /// with the settling off so every attempt answers down the path that always has an answer,
    /// written into scratch rows so nothing of a trial reaches the rows the user reads, and with
    /// the pass's touch bookkeeping put back afterwards.
    pub(super) fn the_loops_that_still_refuse(
        &mut self,
        contributing: &[Rc<RimLoop>],
        first: &BevelAdvisory,
        about_the_size: bool,
        refused_loops: &IndexMap<i32, Vec<BevelLoopFailure>>,
        passes_end: f64,
        cancellation_token: &CancelToken,
    ) -> Result<Option<NarrowedRefusal>, SurfacePartBuildError> {
        if contributing.len() < 2 {
            return Ok(None);
        }

        let settling_was_off = self.the_settling_is_off;
        let touch_was_refused = self.the_band_refused_a_touch;
        let left_alone = self.left_alone_for_touching.clone();
        self.the_settling_is_off = true;

        let mut scratch: Vec<Option<BevelGroupResult>> = vec![None; self.resolutions.len()];
        let size = self.size;

        let answer = the_smallest_refusing_set(
            contributing,
            (first.clone(), about_the_size),
            Self::NARROWING_ATTEMPTS_ALLOWED,
            |some: &[Rc<RimLoop>]| -> Result<Option<(BevelAdvisory, bool)>, SurfacePartBuildError> {
                check(cancellation_token)?;
                self.open_an_attempt(refused_loops, passes_end);

                let attempt =
                    self.try_these_groups_together(some, &mut scratch, size, cancellation_token)?;

                if attempt.mesh.is_some() {
                    return Ok(None);
                }

                // A refusal with no reason of its own still refuses; it is said with the whole
                // pass's reason, which is about these loops too.
                Ok(Some(attempt.reason.map_or_else(
                    || (first.clone(), about_the_size),
                    |reason| (reason, attempt.about_the_size),
                )))
            },
        );

        self.the_settling_is_off = settling_was_off;
        self.the_band_refused_a_touch = touch_was_refused;
        self.left_alone_for_touching = left_alone;

        let (loops, (reason, about_the_size)) = answer?;

        if loops.len() == contributing.len() {
            return Ok(None);
        }

        Ok(Some(NarrowedRefusal {
            loops,
            reason,
            about_the_size,
        }))
    }
}
