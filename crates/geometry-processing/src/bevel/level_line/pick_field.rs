//! One field over a GROUP of chains ([`PickField`]). Ports the type of
//! `Bevel/LevelLine/PickField.cs`; construction, the queries and the `ChainDistanceField` impl
//! land with their method packages.

use std::cell::OnceCell;
use std::rc::Rc;

use super::{ChainField, ChainFieldEndPlaneReader};

/// C# `PickField`: one field over a GROUP of chains - the least value any of the group's chains
/// reads, so a band that several picked loops share is cut once, in one pass, whatever order the
/// loops were picked in.
///
/// A face's rim is an outer loop plus one per hole, and a user picks several at once. Building them
/// one after another on the evolving mesh makes a later loop find its chain edge already gone,
/// makes the pick's room not one number, and makes the result depend on loop order. The cure is
/// that every rule downstream reads ONE field - this one - whose value at a point is `min over the
/// group's chains k of value_k(point)`.
///
/// **Everything the seam promises still holds.** Each member is exactly 1-Lipschitz (it is a
/// minimum of balls; see [`ChainField`]), and a minimum of 1-Lipschitz functions is 1-Lipschitz,
/// so the bounded greatest-value searches keep their bound unchanged. **The exact minima
/// commute:** `min_q min_k v_k(q) = min_k min_q v_k(q)`, so the pick's answer is still ATTAINED at
/// the point it reports, not merely bounded.
///
/// **The tie rule.** Within a chain ties go to the lower parameter; BETWEEN chains they go to the
/// LOWER CHAIN INDEX, by the same strictly-less-with-a-few-bits-of-room comparison, walking the
/// members in ascending index order. A point on the midline of two parallel chains must read the
/// same chain for every one of its neighbours, or the loops the cut leaves are not each owned by
/// one chain.
///
/// **A pick of ONE is today's field to the bit**: every query over a single chain forwards to that
/// chain and reports chain 0. **Work budget**: a point query charges ONE unit per member chain
/// actually evaluated - the members' own charges, with nothing added here. **Why the members are
/// all read**: on the Many Holes witness every later member read won against the running best
/// (4.68M of 4.68M), because members are walked in index order, not nearest first.
pub struct PickField {
    /// Every chain the pick knows, indexed by `FieldSample::chain`.
    pub chains: Vec<Rc<ChainField>>,
    /// The chains of THIS group, in ascending index order.
    pub members: Vec<i32>,
    /// The end-plane reader over the members, made once on first ask (C# `endPlaneReader ??=`).
    pub end_plane_reader: OnceCell<ChainFieldEndPlaneReader>,
    /// Whether any member is clipped by an end plane, found once on first ask (C# `bool?`).
    pub any_member_clipped: OnceCell<bool>,
    /// The smallest setback any point of any member chain asks for.
    pub min_setback: f64,
    /// The largest setback any point of any member chain asks for.
    pub max_setback: f64,
}

impl std::fmt::Debug for PickField {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        // The end-plane reader has no Debug; leave it and the caches out.
        f.debug_struct("PickField")
            .field("chains", &self.chains)
            .field("members", &self.members)
            .field("min_setback", &self.min_setback)
            .field("max_setback", &self.max_setback)
            .finish_non_exhaustive()
    }
}
