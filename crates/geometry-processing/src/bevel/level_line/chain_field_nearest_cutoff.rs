//! The point query's fast walk ([`ChainField::nearest_to`]): the reference walk in segment order
//! (`chain_field_nearest.rs`) with one more prune, a cutoff just above the least reading (or one
//! within rounding of it), which a nearest-first search of the box tree finds first; and the argument that the cutoff never
//! changes an answer.
//!
//! Why it is needed: the reference walk visits leaves in segment order, so its first leaves are
//! usually far from the point, its best stays poor for most of the walk and its prune seldom
//! fires - on a chain of a thousand segments it searched a fifth of them per query. The cutoff
//! drops, from the start, every subtree that cannot come near the least reading. Why the walk in
//! segment order still runs after the search: the least VALUE does not depend on the order it is
//! looked for in, but which of several near-tied readings the tie rule keeps does.

use solidmesh_math::Vector3;

use super::chain_field_nearest::ChainFieldNearest;
use super::{segment_box_tree, ChainField, SegmentNode};

/// The guard on the cutoff only works out while it stays a small fraction of the scale: past this
/// many readings (about a hundred million) the query walks without a cutoff.
const GUARD_FRACTION_LIMIT: f64 = 1e-3;

/// The magnitudes inside which the exactness argument's rounding bound holds: past the top a
/// squared length can overflow, under the bottom one can underflow by more than the argument's
/// `eps`. Outside them the query walks without a cutoff.
const SAFE_SCALE: std::ops::RangeInclusive<f64> = 1e-100..=1e100;

/// `|x| + |y| + |z|`.
fn magnitude(v: Vector3) -> f64 {
    v.x.abs() + v.y.abs() + v.z.abs()
}

impl ChainField {
    /// The nearest chain point to `point` - bit for bit the reference walk's answer
    /// ([`ChainField::visit_nearest_in_index_order`] from the root), reached by visiting only the
    /// subtrees that can come near the least reading. Charges nothing; its callers charge.
    ///
    /// **Why the answer is the reference walk's.** Both walks offer readings in one order (segment
    /// order, each segment's rays after it, `ChainField::offer_leaf`) and differ only in the
    /// subtrees they skip, so it is enough to follow the takes. Write `b` for the best so far,
    /// `cut(b)` for `b - TIE_RELATIVE * (1 + |b|)` as the tie rule computes it, `U` for the reading
    /// the nearest-first search found least (by step 1, at most `eps` above the least of all; the
    /// argument needs only that it is some reading's value), `S` for the magnitude sum in
    /// `ChainField::nearest_cutoff` and
    /// `N` for the root's `offers`. The cutoff is `U + G`, `G = 8 (N + 1) TIE_RELATIVE (1 + |U| +
    /// S)`, and is applied only while `b` is finite.
    ///
    /// 1. *Bounds hold within eps.* Every reading `v` under a node is at least its bound less
    ///    `eps = TIE_RELATIVE * S`. A reading is `|q - c| - s` for a chain point `c` in the node's
    ///    box (a segment point, or a ray point between its segment's end and the ray's far end,
    ///    both folded into the box) and a setback `s` no greater than the node's greatest (a ray
    ///    carries its vertex's setback). The few roundings in a reading and in `box_gap` move it
    ///    by tens of units in the last place of `S`, a few hundred times less than `eps`. That
    ///    holds while `S` stays in `SAFE_SCALE`, where no squared length overflows or underflows
    ///    by more than that; outside it the cutoff is off and the query is the reference walk. A ray read
    ///    off its side reads +infinity, which no bound exceeds.
    /// 2. *Identical until `b` is finite.* Until then the cutoff is off, so both walks decide alike.
    ///    `b` leaves +infinity at the first reading that is not +infinity, which both walks take;
    ///    if that reading is NaN, neither walk takes anything after it. Once `b` is finite a NaN
    ///    reading is never taken.
    /// 3. *A clear level.* Say a level `L` is blocked by a reading `v` when `L < v` and
    ///    `cut(v) <= L + eps`: the interval `[cut(v) - eps, v)`. For every `v` that can block a
    ///    level in `[U, U + G/2]`, `|v| <= |U| + G` and `G <= 1e-3 (1 + |U| + S)` (the readings
    ///    limit), so the interval is under `w = 4 TIE_RELATIVE (1 + |U| + S)` wide. `N` readings
    ///    block less than `N w` of `[U, U + G/2]`, which is `(N + 1) w` long, so some `L` in it is
    ///    blocked by none.
    /// 4. *Up to the first reading at or below `L`.* Let `k` be the first reading in segment order
    ///    with `v_k <= L`; `U` itself is a reading, so it exists. If `b` is still +infinity when `k`
    ///    comes, step 2 has both walks identical and both take it. Otherwise each walk's `b` is a
    ///    reading above `L`, blocking nothing, so `cut(b) > L + eps`. The subtrees holding `k` have
    ///    bounds at most `v_k + eps <= L + eps`: under `cut(b)`, so no current-best prune drops
    ///    them, and under `U + G`, so the cutoff does not. Both walks reach `k` and take it
    ///    (`v_k <= L < cut(b)`), which overwrites all four fields: from here they hold one state.
    /// 5. *After it.* Now `b <= L`, and by induction both walks hold the same `b` at every node. A
    ///    subtree the cutoff drops has a bound over `U + G`, so each reading in it is over
    ///    `U + G - eps > L >= b`: the reference walk would take none of them. Every other skip is
    ///    the reference walk's own prune on the same `b`.
    ///
    /// The roundings in `G` and `U + G` are far inside these margins (`G/2` is at least `4 eps`).
    /// So the only premise is step 1's (the cutoff checks `S` itself): the tree is built from this field's own points, setbacks
    /// and ray far ends with finite coordinates, as `from_polyline_bounded` builds it - the premise
    /// the reference walk's own prune already rests on.
    pub fn nearest_to(&self, point: Vector3) -> ChainFieldNearest {
        let mut found = ChainFieldNearest::start(self.points[0]);
        let (cutoff, searched) = self.nearest_cutoff(point);
        found.searched = searched;
        self.visit_nearest_below(0, point, cutoff, &mut found);
        found
    }

    /// The cutoff [`ChainField::nearest_to`] walks under, and how many segments finding it searched:
    /// the least reading a nearest-first search finds, plus the guard its exactness argument needs.
    /// +infinity - no cutoff - when that reading is not finite, the point's and the tree's scale is
    /// outside `SAFE_SCALE`, or the chain offers too many readings for the guard to stay small.
    fn nearest_cutoff(&self, point: Vector3) -> (f64, i32) {
        let root = self.nodes[0];
        let scale = magnitude(point)
            + magnitude(root.min)
            + magnitude(root.max)
            + root.max_setback.abs()
            + self.min_setback.abs();
        let guard_fraction = 8.0 * (f64::from(root.offers) + 1.0) * Self::TIE_RELATIVE;
        if !SAFE_SCALE.contains(&scale) || guard_fraction > GUARD_FRACTION_LIMIT {
            return (f64::INFINITY, 0);
        }

        let mut upper = f64::INFINITY;
        let mut searched = 0;
        let root_bound = self.lower_bound_of(&root, point);
        self.least_reading_under(0, root_bound, point, &mut upper, &mut searched);
        if !upper.is_finite() {
            return (f64::INFINITY, searched);
        }

        let guard = guard_fraction * (1.0 + upper.abs() + scale);
        (upper + guard, searched)
    }

    /// The least any reading under `node` can be: its box's gap from the point less its greatest
    /// setback - the bound both walks prune by.
    fn lower_bound_of(&self, node: &SegmentNode, point: Vector3) -> f64 {
        segment_box_tree::box_gap(point, point, node.min, node.max) - node.max_setback
    }

    /// Lowers `upper` to the least reading under `node` (whose bound is `bound`) where that is
    /// lower, nearer child first and pruning on `upper`, so the order is free: only the least
    /// value is read, never which reading gave it.
    fn least_reading_under(
        &self,
        node: usize,
        bound: f64,
        point: Vector3,
        upper: &mut f64,
        searched: &mut i32,
    ) {
        if bound >= *upper {
            return;
        }

        let here = self.nodes[node];
        if here.left < 0 {
            for i in here.start..here.start + here.count {
                let i = i as usize;
                *searched += 1;
                let (value, ..) = self.least_over_segment_from_point(i, point);
                if value < *upper {
                    *upper = value;
                }

                if let Some(rays) = self
                    .extensions
                    .as_ref()
                    .and_then(|extensions| extensions[i].as_ref())
                {
                    for ray in rays {
                        let (value, ..) = ray.value_at(point);
                        if value < *upper {
                            *upper = value;
                        }
                    }
                }
            }

            return;
        }

        let (left, right) = (here.left as usize, here.right as usize);
        let left_bound = self.lower_bound_of(&self.nodes[left], point);
        let right_bound = self.lower_bound_of(&self.nodes[right], point);
        if right_bound < left_bound {
            self.least_reading_under(right, right_bound, point, upper, searched);
            self.least_reading_under(left, left_bound, point, upper, searched);
        } else {
            self.least_reading_under(left, left_bound, point, upper, searched);
            self.least_reading_under(right, right_bound, point, upper, searched);
        }
    }

    /// The reference walk with the cutoff: while the best is finite, a subtree whose bound is over
    /// `cutoff` is skipped as well. With `cutoff` +infinity this is the reference walk.
    fn visit_nearest_below(
        &self,
        node: usize,
        point: Vector3,
        cutoff: f64,
        found: &mut ChainFieldNearest,
    ) {
        let here = self.nodes[node];

        let bound = segment_box_tree::box_gap(point, point, here.min, here.max) - here.max_setback;
        if found.best != f64::INFINITY
            && bound >= found.best - (Self::TIE_RELATIVE * (1.0 + found.best.abs()))
        {
            return;
        }

        if found.best.is_finite() && bound > cutoff {
            return;
        }

        if here.left < 0 {
            self.offer_leaf(&here, point, found);
            return;
        }

        self.visit_nearest_below(here.left as usize, point, cutoff, found);
        self.visit_nearest_below(here.right as usize, point, cutoff, found);
    }
}
