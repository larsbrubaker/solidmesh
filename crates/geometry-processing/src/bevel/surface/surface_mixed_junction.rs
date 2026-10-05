//! The corner where THREE picked edges meet and their convexity is mixed
//! ([`SurfaceMixedJunction`]), which shape of junction it is ([`SurfaceJunctionKind`]), one cut's
//! side of it ([`SurfaceMixedJunctionArm`]) and a free chain end the gathering found
//! ([`SurfaceMixedJunctionChainEnd`]). Ports the records of
//! `Bevel/Surface/SurfaceMixedJunction.cs`; its reading and its read-throughs land with the
//! mixed-corner package's methods.

use std::rc::Rc;

use vector_math::Vector3;

use super::{SurfaceMixedCorner, SurfaceMixedCornerDualFrame};

/// C# `SurfaceJunctionKind`: which shape of junction a [`SurfaceMixedJunction`] is - which of the
/// three picked edges at the vertex are the creases.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, Default)]
pub enum SurfaceJunctionKind {
    /// Two cuts and ONE bead - the corner [`SurfaceMixedJunction`] was written about.
    #[default]
    OneCrease = 0,

    /// One cut and TWO beads - a ridge running into the two creases its own two faces make with the
    /// face they both stand against (`SurfaceTwoCreaseJunction`).
    TwoCreases = 1,
}

/// C# `SurfaceMixedJunction.ChainEnd` (a readonly struct): a free end of one chain, as the
/// gathering found it.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, Default)]
pub struct SurfaceMixedJunctionChainEnd {
    /// Which plan of the pass the chain is.
    pub plan: i32,
    /// Which edge of that chain the end is.
    pub edge: i32,
    /// Whether the end is the chain's START rather than its finish.
    pub at_the_start: bool,
}

/// C# `SurfaceMixedJunction.Arm` (a readonly struct): one cut's side of the junction, with
/// everything the plan and the patch need to quote.
#[derive(Debug, Clone)]
pub struct SurfaceMixedJunctionArm {
    /// Which plan of the pass this cut's chain is.
    pub plan: i32,
    /// Which edge of that chain the cut is - its end edge, unless a blend swallowed chords.
    pub cut_index: i32,
    /// Whether the corner stands at the START of that chain rather than its finish.
    pub corner_at_the_start: bool,
    /// The chords between the cut and the corner, which owe no strip of their own.
    pub swallowed: Vec<i32>,
    /// The mixed corner this cut makes with the bead.
    pub mixed: Rc<SurfaceMixedCorner>,
    /// The end of the BEAD's chain this arm reads - a two-crease junction has one per arm.
    ///
    /// Its `edge` is the edge that really CARRIES the blend's end ring, which on a crease chorded
    /// beside the corner is not the chain's own end edge (`SurfaceTwoCreaseJunction`); the chords
    /// between the two are [`SurfaceMixedJunctionArm::bead_swallowed`]. Its `at_the_start` is
    /// still which END of the chain the corner stands at, because that is what says which way the
    /// walk away from it runs.
    pub bead_end: SurfaceMixedJunctionChainEnd,
    /// The bead's chords between the corner and [`SurfaceMixedJunctionArm::bead_end`] - the ones
    /// the blend reaches over, which owe no strip of their own, exactly as
    /// [`SurfaceMixedJunctionArm::swallowed`] is for the cut. Empty at a one-crease junction, which
    /// does not read the bead this way (C# defaults a null argument to the empty array).
    pub bead_swallowed: Vec<i32>,
    /// The canal the blend really is at a TWO-crease junction - the bead's void ball rolling along
    /// the wall both creases stand against, pressed against the cut's tube - or `None` at a
    /// one-crease junction, which is the cap-riding torus [`SurfaceMixedJunctionArm::mixed`]
    /// already carries.
    pub dual: Option<Rc<SurfaceMixedCornerDualFrame>>,
}

/// C# `SurfaceMixedJunction` (a sealed class): the corner where THREE picked edges meet and their
/// convexity is mixed, read as the two mixed corners it really is rather than as a corner of a
/// third kind. With ONE crease among the three that is two cuts and a bead; with TWO it is one cut
/// and two beads, and the arms are read the same way but the canal between them is another shape
/// altogether (`SurfaceTwoCreaseJunction`, which this hands over to).
///
/// **Why it is not the octant.** The three-convex corner is closed by the ball that touches all
/// three faces at once (`SurfaceCornerPatch`). With a crease among the three there is no such
/// ball: the ball touching the two cuts' faces sits a radius INSIDE the material at their corner,
/// and a ball rolling on the bead stands `2r` from the bead's axis out in the void. On the L prism
/// at `r = 1` the two readings stand `2 root 2` apart and a patch fanned from the first cuts the
/// bead off a whole radius short (`SurfaceMixedThreeEdgeCornerTests`).
///
/// **What it is instead.** Each cut is tangent to the bead at ONE point, on the face that cut and
/// the bead SHARE - a mixed corner of exactly the shape [`SurfaceMixedCorner`] already reads, one
/// per cut. The two readings are of one bead, so they share an axis, an outer rail and a turn, and
/// the ball the first hands over at is the very ball the second starts from. So the blend between
/// the two cuts is ONE sweep of the rolling ball about the bead's axis, and it needs no runout:
/// the turn ends ON the second cut and the surface closes.
///
/// **What it refuses, and what it hands on.** Four or more picked edges, a CHAMFER at the corner,
/// or a cut that does not share the cap face with the other cut are refused by name; TWO creases is
/// a corner of its own (`SurfaceTwoCreaseJunction`); three creases has no cut at all. Each arm is
/// read like any other mixed corner, chorded side and all, so one arm of a junction can lean while
/// the other does not, which is the lobed prism's own seam.
#[derive(Debug, Clone)]
pub struct SurfaceMixedJunction {
    /// The vertex all three edges meet at.
    pub corner: Vector3,
    /// The mesh's own index for it, which is what gathered the three ends.
    pub vertex: i32,
    /// Which of the two shapes of junction this is - one crease among the three, or two.
    pub kind: SurfaceJunctionKind,
    /// The arm the blend's turn STARTS on - turn zero stands on this cut's centre line.
    pub first: SurfaceMixedJunctionArm,
    /// The arm it ends on, whose centre line the ball stands on at the far end of the turn.
    pub second: SurfaceMixedJunctionArm,
    /// What names the face BOTH cuts come down from - the cap the outer rail runs across - or -1
    /// where there is no such face. A TWO-crease junction has none: its one cut's two faces are
    /// each SHARED with a crease, so each arm's cap is the other arm's shared face.
    pub cap_region_key: i32,
}
