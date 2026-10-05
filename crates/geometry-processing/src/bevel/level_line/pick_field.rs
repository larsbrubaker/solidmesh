//! One field over a GROUP of chains ([`PickField`]). Ports the type of
//! `Bevel/LevelLine/PickField.cs` with its construction ([`PickField::new`], refusing bad
//! arguments with [`PickFieldArgumentError`] where C# throws), its member read-throughs and its
//! end-plane reader ([`PickField::end_plane_reader`]); the queries and the `ChainDistanceField`
//! impl land with their method packages.

use std::cell::OnceCell;
use std::rc::Rc;

use vector_math::cs_math;

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

impl PickField {
    /// True only when NO member chain reaches below `margin` over the box `min`..`max` - one chain
    /// that might is enough to make the caller pay for the exact search.
    pub fn nothing_reaches_below(
        &self,
        min: vector_math::Vector3,
        max: vector_math::Vector3,
        margin: f64,
    ) -> bool {
        self.members
            .iter()
            .all(|&k| self.chains[k as usize].nothing_reaches_below(min, max, margin))
    }
}

/// Where C# `PickField`'s constructor throws an `ArgumentException`: a caller's bug, reported with
/// C#'s message rather than a panic.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum PickFieldArgumentError {
    /// No member chains (C# "A pick field needs at least one chain.").
    NoMembers,
    /// A member index outside the chains (C# "A member names a chain the pick does not hold.").
    MemberOutOfRange,
    /// Members repeated or out of order (C# "A group's members must be distinct and in ascending
    /// order.").
    MembersNotAscending,
}

impl std::fmt::Display for PickFieldArgumentError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.write_str(match self {
            PickFieldArgumentError::NoMembers => "A pick field needs at least one chain.",
            PickFieldArgumentError::MemberOutOfRange => {
                "A member names a chain the pick does not hold."
            }
            PickFieldArgumentError::MembersNotAscending => {
                "A group's members must be distinct and in ascending order."
            }
        })
    }
}

impl std::error::Error for PickFieldArgumentError {}

impl PickField {
    /// A tie between two chains is two arithmetics answering the same number, so "the same" has
    /// to allow a few bits of drift - the same relative rule `ChainField` uses between its own
    /// segments.
    pub const TIE_RELATIVE: f64 = 1e-12;

    /// The field over `member_chains` (distinct, ascending indices into `all_chains`), its
    /// setbacks the least and greatest of the members'. C#'s null checks have no Rust case.
    pub fn new(
        all_chains: Vec<Rc<ChainField>>,
        member_chains: &[i32],
    ) -> Result<PickField, PickFieldArgumentError> {
        if member_chains.is_empty() {
            return Err(PickFieldArgumentError::NoMembers);
        }

        let mut members = Vec::with_capacity(member_chains.len());
        let mut min_setback = f64::INFINITY;
        let mut max_setback = f64::NEG_INFINITY;

        for (i, &k) in member_chains.iter().enumerate() {
            if k < 0 || k as usize >= all_chains.len() {
                return Err(PickFieldArgumentError::MemberOutOfRange);
            }

            if i > 0 && k <= members[i - 1] {
                return Err(PickFieldArgumentError::MembersNotAscending);
            }

            members.push(k);
            let chain = &all_chains[k as usize];
            min_setback = cs_math::min_f64(min_setback, chain.min_setback);
            max_setback = cs_math::max_f64(max_setback, chain.max_setback);
        }

        Ok(PickField {
            chains: all_chains,
            members,
            end_plane_reader: OnceCell::new(),
            any_member_clipped: OnceCell::new(),
            min_setback,
            max_setback,
        })
    }

    /// The field of ONE chain - today's field to the bit.
    pub fn of(chain: Rc<ChainField>) -> PickField {
        Self::new(vec![chain], &[0]).expect("one chain, member 0, is always a valid pick")
    }

    /// Every chain the pick knows.
    pub fn chains(&self) -> &[Rc<ChainField>] {
        &self.chains
    }

    /// The chains of this group, in ascending index order.
    pub fn members(&self) -> &[i32] {
        &self.members
    }
}

impl PickField {
    /// The members' end planes, read per chain - what the convex cell bound needs - or `None` when
    /// no member is clipped. Made once; it holds its own handles on the chains.
    pub fn end_plane_reader(&self) -> Option<ChainFieldEndPlaneReader> {
        let any_clipped = *self.any_member_clipped.get_or_init(|| {
            self.members
                .iter()
                .any(|&k| self.chains[k as usize].end_plane_reader().is_some())
        });
        if !any_clipped {
            return None;
        }

        Some(Rc::clone(self.end_plane_reader.get_or_init(|| {
            let chains = self.chains.clone();
            Rc::new(move |k: i32, point: vector_math::Vector3| {
                chains[k as usize].end_plane_at(point)
            })
        })))
    }
}
