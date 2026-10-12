//! The corner where two ROUND convex chains overshoot into each other across an unselected crease
//! ([`super::BevelChainEnds::start_pair_corner`]): each cutter has its partner's rolling-ball tube
//! taken out of it before plan 5.3's union, which turns the union of two cuts into the cut a rolling
//! ball makes. Ports `Bevel/BevelPairCornerTrim.cs` (a C# `internal static class`, so a module of
//! free functions here). How far the tube reaches is [`super::bevel_pair_corner_reach`]'s; which
//! creases it is asked at is [`super::bevel_pair_corner_crease`]'s.
//!
//! **What is wrong with the union alone.** 5.3's answer to two chains meeting is to let both
//! overshoot and union their cutters before subtracting. On two CHAMFERS that is exactly right: the
//! cut is the intersection of two half spaces, and unioning the tools is how the kernel is asked for
//! it. On two FILLETS it is not. Take the L of two upright walls whose two inner top edges are
//! filleted at radius r, meeting over the unselected vertical crease between the walls. In the
//! r-by-r square of top face the two overshoots share, each cutter removes everything above its own
//! quarter cylinder, so their union removes everything above the LOWER of the two - a ridge along
//! the square's diagonal and a full r-deep notch at each far corner, where the surface should still
//! be flush with the top. What a ball of radius r rolling down one edge, round the corner and back
//! up the other actually leaves there is the HIGHER of the two: a valley along that diagonal,
//! continuing the crease upward and fading to nothing at the top face, with the top surviving out
//! to both tangent lines and the point where they cross.
//!
//! **The identity.** Write Z for a chain's cutter and B for the tube its own ball sweeps
//! (`BevelSweptSection.Ball`), which is the solid the cutter is the outside of: the two are
//! disjoint and meet along the fillet surface itself. The pipeline removes `Z_A ∪ Z_B`. A rolling
//! ball removes `(Z_A ∪ Z_B) − (B_A ∪ B_B)` - the same reach, minus everything either ball can still
//! touch - and distributing the subtraction over the union gives what this stage builds:
//! `(Z_A − B_B) ∪ (Z_B − B_A)`, since `Z_A − B_A` is already `Z_A`. Said from the other end, the
//! material this hands back to the part is `(B_A ∪ B_B) ∩ part`, so
//! `today's result ∪ ((B_A ∪ B_B) ∩ part)` IS the rolling-ball answer, and everything below is that
//! one sentence spent as booleans. Both readings check out on the L above: in the shared square a
//! point survives exactly when it is under either cylinder, which is the higher of the two.
//!
//! **Why only at a pair corner.** The identity is true everywhere, and applying it everywhere would
//! be both wasteful and wrong. Wasteful because `Z − B` is `Z` for every chain a tube does not
//! reach - which is every chain but the one at the far side of this corner - and each of those is a
//! boolean and a swept solid spent on nothing, on a size drag that rebuilds live. Wrong because the
//! tube is only the right thing to give back where the overshoot was the MECHANISM. Where a chain
//! end was capped, 5.2 already decided the tool stops on a plane and there is nothing past it to
//! hand back; where the two chains are of opposite convexity a blend owns the corner
//! (`BevelMixedBlends`) and a cut trimmed by a bead's ball would eat the fin the blend is built to
//! shape. So the trim reads exactly the ends [`super::BevelChainEnds`] marked as pair corners, which
//! is the one shape of corner it was written for, and one chain's tube reaches only the cutters that
//! meet it AT that vertex.
//!
//! **Cost.** One n-ary subtract per cutter that has a partner - two on the L above, and none at all
//! on a part with no such corner - carried in the same progress budget as every other boolean of
//! the rebuild. Nothing here is per-rebuild overhead for the parts that do not have this corner: no
//! tube is even swept for them (`BevelFeatureMeshBuilder` only asks for one where the ends say so).
//!
//! **How far the tube has to reach.** Everything of cutter A past the shared vertex lies within a
//! ball's reach of A's own arc axis, so B's tube covers the whole of that overreach exactly when it
//! runs far enough to pass A's AXIS. A CAPPED end slides its ring onto the capping plane and is not
//! a corner this stage is ever interested in; the end that matters is the UNCAPPED one, carrying
//! [`super::bevel_edge_tool_builder::overshoot`]'s `max(size, tangentDistance)` past the last
//! vertex. Where the two edges meet their faces at the same angle - every corner of a box, and the L
//! this was written for - that overshoot is the radius and lands the tube exactly on the partner's
//! axis, which is why this worked at all before anyone measured a corner that leans. Where the
//! dihedrals differ it does not: at a 75 degree wedge the partner's ball sits
//! `r / sin(37.5) = 1.64 r` up its own bisector, its axis stands `1.30 r` past the vertex along the
//! square run, and a tube carried by that run's own `r` left the last `0.3 r` of the overreach
//! untrimmed - a smaller copy of the ridge this stage exists to remove. So the TUBE, and only the
//! tube, is now swept to that axis explicitly ([`super::bevel_pair_corner_reach`]): the cutter keeps
//! its own overshoot, because lengthening a cutter removes material the user did not ask about
//! while lengthening a tube can only hand material back. What is left over is the partner's sagitta
//! over ONE chord, since its axis is read from its own last chord - an error that shrinks with the
//! tessellation rather than with the corner's angles.
//!
//! **When it cannot be had.** A tube that would not sweep, a subtract whose operands the kernel
//! refuses, or a result it will not vouch for, leaves that cutter exactly as plan 5.3 left it - the
//! ridge, which is the shape that shipped before this existed. The GEOMETRY of that fallback is
//! deliberate and safe: what is left is a corner that looks like the old one, never material
//! appearing where the user did not ask for it, so nothing downstream has to be unwound to survive
//! it.
//!
//! Safe is not the same as silent, though, and the difference took a while to see. A ridge where
//! the user asked for a rolled corner is a real difference in the part they will print, and it is
//! exactly the kind of difference a rebuild that reports success teaches them to stop looking for -
//! the same argument the bead's unclipped wedge lost. So [`plan`] hands back which cutters were left
//! untrimmed for want of a partner's tube, and the builder turns that into one advisory for the
//! operation (see `BevelRebuildAdvisories::corner_went_unrounded`). The two LATER failures - the
//! refused subtract and the result the kernel will not vouch for - are still quiet, because they
//! are caught inside [`apply`] where the plan has already been sized and reported, and because a
//! kernel that changes its mind mid-batch has nothing to add to the same sentence.
//!
//! Which takes one catch to actually mean, and it is worth saying why rather than reading as
//! defensive programming. A refused operand does not come back as a null mesh: `Subtract` is defined
//! by every one of its operands, so the kernel does not skip one and carry on the way a union does -
//! it fails, and with nothing to catch it that failure takes the WHOLE rebuild down over a corner
//! treatment. Which is the one outcome this stage must never cause: the corner it is polishing was,
//! until this file existed, a part that built.
//!
//! So every refusal is caught here - C#'s `InvalidOperationException`, the base of
//! `PartialBooleanException` and the kernel's `MeshImportRejectedException`, which in Rust is every
//! [`BooleanError`] but [`BooleanError::Cancelled`]. A cancellation still travels straight out to
//! the rebuild that asked for it. (`BevelBeadClip` carries the same note for the same reason - a
//! refused operand escaping a corner treatment was worth fixing in both places at once.)

use std::rc::Rc;

use indexmap::{IndexMap, IndexSet};
use polygon_mesh::csg::{BooleanError, CsgModes};
use polygon_mesh::Mesh;
use vector_math::Matrix4X4;

use super::bevel_bead_clip::CombineMeshes;
use super::bevel_working_mesh::is_usable_operand;
use super::BevelChainEnds;

/// C# `BevelPairCornerTrim.Candidate`: one cutter that meets another selected run over an
/// unselected crease, as the trim stage needs it - where it sits in the batch, the tube its own
/// ball leaves, and the corners it meets neighbours at.
#[derive(Debug, Clone)]
pub struct BevelPairCornerTrimCandidate {
    /// Where this cutter is in the builder's cutter list.
    pub cutter_index: i32,
    /// The tube this chain's rolling ball leaves, or `None` where it could not be swept - in which
    /// case this cutter can still BE trimmed, it just cannot trim anyone else. Shared (`Rc`): the
    /// same tube is handed to every partner, as C# hands the one object.
    pub ball_sweep: Option<Rc<Mesh>>,
    /// The vertices this chain overshoots into another selected run at; `None` is C#'s null.
    pub pair_corners: Option<Vec<i32>>,
}

impl BevelPairCornerTrimCandidate {
    /// C#'s constructor.
    pub fn new(
        cutter_index: i32,
        ball_sweep: Option<Rc<Mesh>>,
        pair_corners: Option<Vec<i32>>,
    ) -> Self {
        BevelPairCornerTrimCandidate {
            cutter_index,
            ball_sweep,
            pair_corners,
        }
    }
}

/// C# `BevelPairCornerTrim.Trim`: one cutter and the partners' tubes to take out of it.
#[derive(Debug, Clone)]
pub struct BevelPairCornerTrimTrim {
    /// Where this cutter is in the builder's cutter list.
    pub cutter_index: i32,
    /// The tubes of every chain that meets this one at a pair corner.
    pub partner_balls: Vec<Rc<Mesh>>,
}

impl BevelPairCornerTrimTrim {
    /// C#'s constructor.
    pub fn new(cutter_index: i32, partner_balls: Vec<Rc<Mesh>>) -> Self {
        BevelPairCornerTrimTrim {
            cutter_index,
            partner_balls,
        }
    }
}

/// C# `Plan`: which cutters have to be trimmed by which tubes, worked out before any boolean runs
/// so the caller can size its progress budget by it - and, second, C#'s `out cuttersLeftUntrimmed`.
///
/// A cutter is never trimmed by its OWN tube - `Z - B` is `Z`, so it would be a boolean that
/// changed nothing - and two runs pair only where [`super::BevelChainEnds`] said they meet, which is
/// why the corner vertex rather than proximity is what matches them up. Three or more runs at one
/// such vertex fall out of the same walk: each is trimmed by all the others' tubes at once.
///
/// Each partner is taken ONCE however many corners the two runs share, which is not a hypothetical
/// tidiness: two runs of two adjacent edges each round the top of a box meet at BOTH of the corners
/// between them, so the plain walk would hand the same tube to the kernel twice. The answer would be
/// right - subtracting a solid twice removes what subtracting it once does - but the second copy is
/// a coincident surface for the kernel to decide about and a second import to pay for, on a stage
/// that runs inside a live size drag.
///
/// `candidates` is every cutter that meets another selected run over an unselected crease (C#'s
/// null list is the empty slice). The second value is the batch index of each cutter that had at
/// least one partner with no tube, which is a corner going out as plan 5.3's ridge. "At least one"
/// because a cutter with one tubed partner and one tubeless one is both trimmed and reported - the
/// trim it did get says nothing about the corner it did not. Empty on every selection where every
/// pairing was met.
pub fn plan(
    candidates: &[BevelPairCornerTrimCandidate],
) -> (Vec<BevelPairCornerTrimTrim>, Vec<i32>) {
    let mut trims = Vec::new();
    let mut cutters_left_untrimmed = Vec::new();

    if candidates.len() < 2 {
        // One chain cannot pair with itself, and nothing is a pair corner without two runs at it.
        return (trims, cutters_left_untrimmed);
    }

    // Corner vertex to the candidates that end there. Built once rather than compared pairwise,
    // because a selection of many chains meeting round one part is exactly where the pairwise walk
    // would start costing something.
    let mut at_corner: IndexMap<i32, Vec<usize>> = IndexMap::new();

    for (i, candidate) in candidates.iter().enumerate() {
        let Some(corners) = &candidate.pair_corners else {
            continue;
        };

        for &corner in corners {
            at_corner
                .entry(corner)
                .or_insert_with(|| Vec::with_capacity(2))
                .push(i);
        }
    }

    for (i, candidate) in candidates.iter().enumerate() {
        let Some(corners) = &candidate.pair_corners else {
            continue;
        };

        let mut partner_balls: Option<Vec<Rc<Mesh>>> = None;
        let mut taken: IndexSet<usize> = IndexSet::new();
        let mut a_partner_had_no_tube = false;

        for corner in corners {
            let here = &at_corner[corner];

            for &p in here {
                let partner = &candidates[p];

                if p == i {
                    continue;
                }

                // By candidate index rather than by mesh: two runs meeting at two corners are one
                // partner met twice, and the tube they would hand over is the same object both
                // times. Counted before the tube is looked at, so a partner met twice is also one
                // MISSING tube rather than two.
                if !taken.insert(p) {
                    continue;
                }

                let Some(ball) = &partner.ball_sweep else {
                    // There was a real corner here and the trim cannot be made: this cutter keeps
                    // the ridge. Said out loud rather than skipped, because after this function
                    // returns the fact is gone - an untrimmed cutter looks exactly like one that
                    // never had a partner. What it MEANS is the caller's to decide: on a chamfer no
                    // tube is ever swept, so a missing one is the plan working as designed and not
                    // a fallback.
                    a_partner_had_no_tube = true;
                    continue;
                };

                partner_balls
                    .get_or_insert_with(|| Vec::with_capacity(2))
                    .push(Rc::clone(ball));
            }
        }

        if a_partner_had_no_tube {
            cutters_left_untrimmed.push(candidate.cutter_index);
        }

        if let Some(partner_balls) = partner_balls {
            trims.push(BevelPairCornerTrimTrim::new(
                candidate.cutter_index,
                partner_balls,
            ));
        }
    }

    (trims, cutters_left_untrimmed)
}

/// C# `CornersOf`: the vertices one run overshoots into another selected run at, or `None` where
/// it does neither end - which is every run but the few that meet a neighbour over an unselected
/// crease.
///
/// Lives here rather than with the builder that calls it because what it reads is this trim's own
/// input: a pair corner is only ever a pair corner to [`plan`].
pub fn corners_of(ends: Option<&BevelChainEnds>) -> Option<Vec<i32>> {
    let ends = ends?;

    if ends.start_pair_corner.is_none() && ends.end_pair_corner.is_none() {
        return None;
    }

    let mut corners = Vec::with_capacity(2);

    if let Some(corner) = ends.start_pair_corner {
        corners.push(corner);
    }

    if let Some(corner) = ends.end_pair_corner {
        corners.push(corner);
    }

    Some(corners)
}

/// C# `ApplyAsync`: runs the planned trims, replacing each cutter in place with itself less its
/// partners' tubes.
///
/// `cutters` is the cutter batch, in the source item's own space; trimmed entries are replaced.
/// `trims` is what [`plan`] worked out. `combine` is the caller's own boolean, so these steps share
/// the rebuild's progress budget. Every operand is handed over with the identity matrix, which is
/// what keeps the result in the source item's space alongside the cutters it goes back among: the
/// tools are all measured there, and it is the batch union afterwards that carries them into the
/// operation's space. Only a cancellation comes back as an error.
pub fn apply(
    cutters: &mut [Mesh],
    trims: &[BevelPairCornerTrimTrim],
    combine: &CombineMeshes<'_>,
) -> Result<(), BooleanError> {
    for trim in trims {
        let cutter_index = trim.cutter_index as usize;
        let mut operands: Vec<(Mesh, Matrix4X4)> = Vec::with_capacity(trim.partner_balls.len() + 1);
        operands.push((cutters[cutter_index].clone(), Matrix4X4::IDENTITY));

        for ball in &trim.partner_balls {
            operands.push(((**ball).clone(), Matrix4X4::IDENTITY));
        }

        let trimmed = match combine(&operands, CsgModes::Subtract) {
            Ok(trimmed) => trimmed,
            Err(BooleanError::Cancelled) => return Err(BooleanError::Cancelled),
            Err(_) => {
                // The kernel refused an operand. What it carries out is the subtract over the
                // operands it DID take - a third corner treatment, neither the trimmed one nor the
                // one that shipped, and nobody here asked for it - so it is dropped and the
                // untrimmed cutter stands. See the module docs: catching this is what makes "leaves
                // that cutter as plan 5.3 left it" true rather than "fails the whole rebuild".
                continue;
            }
        };

        if is_usable_operand(Some(&trimmed), None) {
            cutters[cutter_index] = trimmed;
        }

        // Else the cutter stays exactly as it was and this corner comes out as plan 5.3 left it -
        // see the module docs on why that is left quiet.
    }

    Ok(())
}
