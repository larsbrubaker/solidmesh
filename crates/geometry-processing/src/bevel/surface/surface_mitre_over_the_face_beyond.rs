//! A MITRE WHOSE TIP STANDS PAST THE FACE'S OWN RIM ([`SurfaceMitreOverTheFaceBeyond`],
//! `Bevel/Surface/SurfaceMitreOverTheFaceBeyond.cs`): the outermost rails of the two strips run off the
//! face the two beads share, out over the edge beyond it, and are cut on the face BEYOND that edge
//! instead of on the mitre plane - which the face beyond pays for by gaining the lens between the two
//! exit curves, standing outside its own old rim. The planes it reads are the part's hull
//! ([`SurfaceStripEndsOnTheHull::supporting_planes_of`]) and its corners are crossed by
//! [`surface_mitre_sector::try_three_planes`]; `SurfaceBevelBuilder`'s mitre asks it.
//!
//! **What makes a tip leave the part.** A mitre carries each rail by its own offset from the edge over
//! the cosine of half the chain's turn, so a corner that turns far enough throws the rails standing on
//! the shared face a long way out along that face's own surface - 1.5495 times the radius at the
//! 154.75 degree ridge ends of "5mm Bevel Around Center", where the face itself reaches only 7.0711.
//! Past that the point the mitre wants is off the solid altogether, and the two strips have nothing out
//! there to meet along.
//!
//! **The furthest cut back wins.** Per rail, the mitre plane is read as one cutting plane among the
//! part's own supporting planes, and the one that cuts the rail back FURTHEST from its end is the one
//! that really ends it - the same rule the hull reading uses, and the same reason: past that crossing
//! the rail is outside the part whatever the others say. The rails it takes are a contiguous run from
//! the shared face outward, because that is the side the mitre throws them out on; anything else is an
//! end this does not build and is named.
//!
//! **Where the end turns.** On the CHORD between the last rail the face beyond cut and the first the
//! mitre did, which is [`SurfaceStripEndsOnTheHull`]'s own rule - the band between two rails is a
//! plane, so the turn is the one point three planes meet at. The two strips' turn points both lie on
//! the line where the mitre plane meets the face beyond, and where the two beads set back by the same
//! amount they are the SAME point; that they meet is checked rather than assumed, within the pool's
//! ruler.
//!
//! **And the lens.** The two strips' run-off rails land on the face beyond along two exit curves, which
//! run from the rim out to the turn point and back. Between them the face beyond gains a lens that lies
//! OUTSIDE its old rim - the bead bulges a hair past the part where the exact cylinder would - and
//! swallows the piece of the rim between the two exits. That piece carries one vertex of the part,
//! where the mitre plane crosses the edge the shared face and the face beyond meet along, so the lens
//! is fanned from it and the rim's own two halves keep the triangles they had.
//!
//! **What it assumes.** That the face beyond is one plane of the part's own hull, and that the vertex
//! the lens is fanned from is really there - both refused by name rather than built around. And that
//! nothing else stands in the sector between the two exit curves, which is true wherever the two beads
//! swallowed the corner between them, as they must have to have thrown their tips out past it at all.
//!
//! [`SurfaceStripEndsOnTheHull::supporting_planes_of`]: super::SurfaceStripEndsOnTheHull::supporting_planes_of
//! [`SurfaceStripEndsOnTheHull`]: super::SurfaceStripEndsOnTheHull

use vector_math::Vector3;

use super::surface_mitre_joint_step::SideSeams;
use super::{surface_mitre_sector, surface_strip_ends};
use super::{
    SurfaceEdgePlan, SurfaceStrip, SurfaceStripEndsOnTheHullSupportingPlane, SurfaceVertexPool,
};
use crate::bevel::{BevelAdvisory, BevelAdvisoryCode};

/// C#'s `Func<IReadOnlyList<SurfaceStripEndsOnTheHull.SupportingPlane>>`: the part's supporting
/// planes, read only when they are wanted.
pub type SurfaceMitreHull<'a> = &'a dyn Fn() -> Vec<SurfaceStripEndsOnTheHullSupportingPlane>;

/// C# `SurfaceMitreOverTheFaceBeyond` (a sealed class): a mitre whose tip stands past the shared
/// face's rim, and what ends each of the rails it threw out there.
#[derive(Debug, Clone, PartialEq)]
pub struct SurfaceMitreOverTheFaceBeyond {
    leaving_at: Vec<f64>,
    arriving_at: Vec<f64>,
    /// How many rails out from the shared face are cut on the face beyond rather than mitred.
    pub run_off: i32,
    /// The plane of the part the run-off rails are cut on.
    pub beyond: SurfaceStripEndsOnTheHullSupportingPlane,
    /// Where the two ends turn from the face beyond back onto the mitre plane.
    pub turn: Vector3,
    /// The part's own vertex the lens is fanned from, on the rim between the two exits.
    pub apex: i32,
}

/// What [`SurfaceMitreOverTheFaceBeyond::try_run_off`] reads of one strip: how many rails ran off,
/// where each was cut, and the plane that cut them.
type RunOff = (i32, Vec<f64>, SurfaceStripEndsOnTheHullSupportingPlane);

impl SurfaceMitreOverTheFaceBeyond {
    /// C#'s private constructor.
    fn new(
        run_off: i32,
        leaving_at: Vec<f64>,
        arriving_at: Vec<f64>,
        beyond: SurfaceStripEndsOnTheHullSupportingPlane,
        turn: Vector3,
        apex: i32,
    ) -> Self {
        SurfaceMitreOverTheFaceBeyond {
            leaving_at,
            arriving_at,
            run_off,
            beyond,
            turn,
            apex,
        }
    }

    /// C# `LeavingAt`: where the LEAVING strip's rail meets the face beyond, along its own edge.
    pub fn leaving_at(&self, step: i32) -> f64 {
        self.leaving_at[step as usize]
    }

    /// C# `ArrivingAt`: where the ARRIVING strip's does.
    pub fn arriving_at(&self, step: i32) -> f64 {
        self.arriving_at[step as usize]
    }

    /// C# `TryRead`: reads whether this mitre's tip stands past the shared face's rim, and if it does,
    /// what ends each of the rails it threw out there. `Ok(None)` is C#'s `true` with no reading.
    ///
    /// `hull` is the part's supporting planes, read only when they are wanted: the cheap reading
    /// below - has the mitre's own outermost rail left the seam the walkback laid on the shared face? - is
    /// false at every ordinary mitre, and an ordinary mitre must not pay for a hull nothing asks of it.
    #[allow(clippy::too_many_arguments)]
    pub fn try_read(
        pool: &SurfaceVertexPool,
        strip: &SurfaceStrip,
        next: &SurfaceStrip,
        leaving: &SurfaceEdgePlan,
        arriving: &SurfaceEdgePlan,
        seams: &[SideSeams],
        edge_index: i32,
        next_index: i32,
        mine_on_the_face: i32,
        mine_step: i32,
        theirs_on_the_face: i32,
        theirs_step: i32,
        paired: i32,
        corner: Vector3,
        through: Vector3,
        normal: Vector3,
        hull: Option<SurfaceMitreHull<'_>>,
    ) -> Result<Option<SurfaceMitreOverTheFaceBeyond>, BevelAdvisory> {
        let Some(hull) = hull else {
            return Ok(None);
        };

        if paired < 2
            || Self::the_shared_rails_still_meet(
                pool,
                strip,
                next,
                seams,
                edge_index,
                next_index,
                mine_on_the_face,
                theirs_on_the_face,
                through,
                normal,
            )
        {
            // Either there is no room for a turn point between two rails, or the mitre's own
            // outermost rails land where the walkback left their seam - which is every mitre that
            // stands on the face it is cut against, and there is nothing here to read.
            return Ok(None);
        }

        let planes = hull();

        let mine_length = (leaving.to() - leaving.from()).dot(strip.along());
        let their_length = (arriving.to() - arriving.from()).dot(next.along());

        let crosses_the_corner = || {
            BevelAdvisory::new(
                BevelAdvisoryCode::SurfaceHullEndCrossesTheCorner,
                vec![corner.into()],
            )
        };

        let (
            Some((mine_run_off, mine_at, mine_plane)),
            Some((their_run_off, their_at, their_plane)),
        ) = (
            Self::try_the_run_off(
                strip,
                mine_on_the_face,
                mine_step,
                paired,
                false,
                mine_length,
                &planes,
                through,
                normal,
                pool.near_enough(),
            ),
            Self::try_the_run_off(
                next,
                theirs_on_the_face,
                theirs_step,
                paired,
                true,
                their_length,
                &planes,
                through,
                normal,
                pool.near_enough(),
            ),
        )
        else {
            return Err(crosses_the_corner());
        };

        if mine_run_off == 0 && their_run_off == 0 {
            // The tip stands past no plane of the part after all, so whatever the seam's complaint
            // is it is not this one - left to the mitre's own reading to name where it stands.
            return Ok(None);
        }

        if mine_run_off != their_run_off || mine_run_off >= paired {
            // The two strips ran off by different amounts, or one of them ran off entirely: neither
            // leaves the pair of exit curves this lens is bounded by.
            return Err(crosses_the_corner());
        }

        if mine_plane != their_plane {
            return Err(BevelAdvisory::new(
                BevelAdvisoryCode::SurfaceHullEndOnThreePlanes,
                vec![corner.into()],
            ));
        }

        let (Some(mine_turn), Some(their_turn)) = (
            Self::try_the_turn(
                strip,
                mine_on_the_face,
                mine_step,
                mine_run_off,
                mine_plane,
                through,
                normal,
                pool.near_enough(),
            ),
            Self::try_the_turn(
                next,
                theirs_on_the_face,
                theirs_step,
                their_run_off,
                their_plane,
                through,
                normal,
                pool.near_enough(),
            ),
        ) else {
            return Err(crosses_the_corner());
        };

        if (mine_turn - their_turn).length() > pool.near_enough() {
            // The two exit curves leave the mitre plane at two different points, so they bound no
            // lens and the two ends do not close on one another.
            return Err(BevelAdvisory::new(
                BevelAdvisoryCode::SurfaceStripSeamDidNotMeet,
                vec![((mine_turn + their_turn) / 2.0).into()],
            ));
        }

        // The rim the lens eats a piece of: the edge the shared face and the face beyond meet along,
        // cut by the mitre plane. The part has a vertex there - the walkback split the rim on that
        // plane when it took the two bands off either side of it - and the lens is fanned from it.
        let shared = leaving.edge.direction().cross(if mine_on_the_face == 0 {
            leaving.left_inward_at_the_edge
        } else {
            leaving.right_inward_at_the_edge
        });

        let on_the_rim = surface_mitre_sector::try_three_planes(
            corner,
            shared,
            mine_plane.at,
            mine_plane.normal,
            through,
            normal,
        );

        // C#'s `out onTheRim` is the zero vector where the three planes do not meet.
        let apex = on_the_rim
            .and_then(|on_the_rim| pool.try_find_near(on_the_rim, pool.snap_near(), None));

        let Some(apex) = apex else {
            return Err(BevelAdvisory::new(
                BevelAdvisoryCode::SurfaceStripEndNotSupported,
                vec![on_the_rim.unwrap_or(Vector3::ZERO).into()],
            ));
        };

        Ok(Some(SurfaceMitreOverTheFaceBeyond::new(
            mine_run_off,
            mine_at,
            their_at,
            mine_plane,
            (mine_turn + their_turn) / 2.0,
            apex,
        )))
    }

    /// C# `TryTheRails`: both strips' rails at one step out from the shared face, cut on the face
    /// beyond: each takes its OWN vertex there, which is the whole difference from a mitre - the two
    /// no longer meet.
    #[allow(clippy::too_many_arguments)]
    pub fn try_the_rails(
        &self,
        pool: &mut SurfaceVertexPool,
        strip: &SurfaceStrip,
        next: &SurfaceStrip,
        seams: &[SideSeams],
        edge_index: i32,
        next_index: i32,
        mine: i32,
        theirs: i32,
        step: i32,
        ring: &mut [i32],
        next_ring: &mut [i32],
        leaving_at_rail: &mut [f64],
        arriving_at_rail: &mut [f64],
    ) -> Result<(), BevelAdvisory> {
        leaving_at_rail[mine as usize] = self.leaving_at[step as usize];
        arriving_at_rail[theirs as usize] = self.arriving_at[step as usize];

        Self::try_one_rail(
            pool,
            strip,
            seams,
            edge_index,
            mine,
            self.leaving_at[step as usize],
            true,
            ring,
        )?;
        Self::try_one_rail(
            pool,
            next,
            seams,
            next_index,
            theirs,
            self.arriving_at[step as usize],
            false,
            next_ring,
        )
    }

    /// C# `TryGainTheLens`: the lens the face beyond gains - the two exit curves, the turn point they
    /// meet at, and the piece of rim between their two landings, fanned from the part's own vertex on
    /// that rim and wound to face the way the face beyond faces. Hands back the turn's vertex (C#'s
    /// `out turnVertex`; it never refuses).
    #[allow(clippy::too_many_arguments)]
    pub fn try_gain_the_lens(
        &self,
        pool: &mut SurfaceVertexPool,
        faces: &mut Vec<(i32, i32, i32)>,
        ring: &[i32],
        next_ring: &[i32],
        mine_on_the_face: i32,
        mine_step: i32,
        theirs_on_the_face: i32,
        theirs_step: i32,
    ) -> i32 {
        let turn_vertex = pool.add(self.turn);

        // Round the lens: out along the leaving strip's exit curve from the rim to the turn, back
        // along the arriving one, and home along the rim through the apex.
        let mut outline = Vec::with_capacity((2 * self.run_off.max(0) as usize) + 2);

        for step in 0..self.run_off {
            outline.push(ring[(mine_on_the_face + (mine_step * step)) as usize]);
        }

        outline.push(turn_vertex);

        for step in (0..self.run_off).rev() {
            outline.push(next_ring[(theirs_on_the_face + (theirs_step * step)) as usize]);
        }

        outline.push(self.apex);

        // Wound to face the way the part does there. The outline is walked one way round or the
        // other depending which of the two strips leaves and which arrives, so the winding is
        // measured rather than assumed.
        if Self::the_area_of(pool, &outline).dot(self.beyond.normal) < 0.0 {
            outline.reverse();
        }

        // C#'s `IndexOf`: the first place the apex stands; it is always in the outline.
        let from = outline
            .iter()
            .position(|&vertex| vertex == self.apex)
            .unwrap_or(0);

        for step in 1..outline.len().saturating_sub(1) {
            let a = outline[(from + step) % outline.len()];
            let b = outline[(from + step + 1) % outline.len()];

            if a == self.apex || b == self.apex || a == b {
                continue;
            }

            faces.push((self.apex, a, b));
        }

        turn_vertex
    }

    /// C# `TheAreaOf`: twice the polygon's area as a vector, which is its normal times its area.
    fn the_area_of(pool: &SurfaceVertexPool, outline: &[i32]) -> Vector3 {
        let mut area = Vector3::ZERO;
        let first = pool.position_of(outline[0]);

        for at in 1..outline.len().saturating_sub(1) {
            area += (pool.position_of(outline[at]) - first)
                .cross(pool.position_of(outline[at + 1]) - first);
        }

        area
    }

    /// C# `TheSharedRailsStillMeet`: whether both strips' outermost rails on the SHARED face land on
    /// the one vertex the walkback left there for them. True at every mitre that stands on the face it
    /// is cut against, which is what makes this the cheap reading the hull is only read past.
    #[allow(clippy::too_many_arguments)]
    fn the_shared_rails_still_meet(
        pool: &SurfaceVertexPool,
        strip: &SurfaceStrip,
        _next: &SurfaceStrip,
        seams: &[SideSeams],
        edge_index: i32,
        next_index: i32,
        mine_on_the_face: i32,
        theirs_on_the_face: i32,
        through: Vector3,
        normal: Vector3,
    ) -> bool {
        let Some(here) = strip.try_cross(mine_on_the_face, through, normal) else {
            return true;
        };

        let side = if mine_on_the_face == 0 { 0 } else { 1 };
        let their_side = if theirs_on_the_face == 0 { 0 } else { 1 };
        let mine = seams[edge_index as usize][side].as_deref();
        let theirs = seams[next_index as usize][their_side].as_deref();

        let (Some(mine), Some(theirs)) = (mine, theirs) else {
            return true;
        };

        if mine.is_empty() || theirs.is_empty() {
            return true;
        }

        mine[mine.len() - 1].0 == theirs[0].0
            && (pool.position_of(mine[mine.len() - 1].0) - strip.point_on(mine_on_the_face, here))
                .length()
                <= surface_strip_ends::meets_the_seam(pool)
    }

    /// C# `TryTheRunOff`: how many rails out from the shared face are cut by a plane of the part
    /// rather than by the mitre, and where. `None` where the two do not divide the strip into two
    /// contiguous runs, which is an end with no one turn point in it.
    #[allow(clippy::too_many_arguments, clippy::neg_cmp_op_on_partial_ord)]
    fn try_the_run_off(
        strip: &SurfaceStrip,
        on_the_face: i32,
        step: i32,
        paired: i32,
        at_the_start: bool,
        length: f64,
        planes: &[SurfaceStripEndsOnTheHullSupportingPlane],
        through: Vector3,
        normal: Vector3,
        near: f64,
    ) -> Option<RunOff> {
        let mut run_off = 0;
        let mut at = vec![0.0; paired.max(0) as usize];
        let mut beyond = SurfaceStripEndsOnTheHullSupportingPlane::default();

        let walk = if at_the_start {
            -strip.along()
        } else {
            strip.along()
        };
        let mut still_running_off = true;

        for outward in 0..paired {
            let rail = on_the_face + (step * outward);

            let mitre_at = strip.try_cross(rail, through, normal)?;

            let mut cut_back_by = if at_the_start {
                mitre_at
            } else {
                length - mitre_at
            };
            let mut found = SurfaceStripEndsOnTheHullSupportingPlane::default();

            for plane in planes {
                // C#'s `!(x > 1e-9)`, which passes a NaN over as well.
                if !(plane.normal.dot(walk) > 1e-9) {
                    continue;
                }

                let Some(param) = strip.try_cross(rail, plane.at, plane.normal) else {
                    continue;
                };

                let back = if at_the_start { param } else { length - param };

                if back > cut_back_by + near {
                    cut_back_by = back;
                    found = *plane;
                }
            }

            if !found.exists() {
                still_running_off = false;
                continue;
            }

            if !still_running_off {
                // A rail cut by the part after a rail the mitre cut: the end does not divide into one
                // run on the face beyond and one on the mitre plane, and there is no turn point in it.
                return None;
            }

            if run_off > 0 && found != beyond {
                return None;
            }

            beyond = found;
            at[outward as usize] = if at_the_start {
                cut_back_by
            } else {
                length - cut_back_by
            };
            run_off = outward + 1;
        }

        Some((run_off, at, beyond))
    }

    /// C# `TryTheTurn`: where the end turns from the face beyond onto the mitre plane - the two of them
    /// crossed with the plane of the chord between the last rail the one cut and the first the other
    /// did, which has to stand ON that chord.
    #[allow(clippy::too_many_arguments, clippy::neg_cmp_op_on_partial_ord)]
    fn try_the_turn(
        strip: &SurfaceStrip,
        on_the_face: i32,
        step: i32,
        run_off: i32,
        beyond: SurfaceStripEndsOnTheHullSupportingPlane,
        through: Vector3,
        normal: Vector3,
        near: f64,
    ) -> Option<Vector3> {
        let foot = strip.point_on(on_the_face + (step * (run_off - 1)), 0.0);
        let across = strip.point_on(on_the_face + (step * run_off), 0.0) - foot;
        let column = strip.along().cross(across);

        if !(column.length_squared() > 0.0) {
            return None;
        }

        let turn = surface_mitre_sector::try_three_planes(
            beyond.at,
            beyond.normal,
            through,
            normal,
            foot,
            column,
        )?;

        let sideways = (turn - foot).dot(across) / across.length_squared();

        (sideways >= -near && sideways <= 1.0 + near).then_some(turn)
    }

    /// C# `TryOneRail`: one rail cut on the face beyond. The strip's own outermost rail takes the
    /// vertex the walkback already left where its tangent line ran off that face's rim, as every outer
    /// rail does; the rails inside it are new points, standing on the face and on nothing else.
    #[allow(clippy::too_many_arguments)]
    fn try_one_rail(
        pool: &mut SurfaceVertexPool,
        strip: &SurfaceStrip,
        seams: &[SideSeams],
        edge_index: i32,
        rail: i32,
        param: f64,
        at_the_end: bool,
        ring: &mut [i32],
    ) -> Result<(), BevelAdvisory> {
        let at = strip.point_on(rail, param);

        if rail != 0 && rail != strip.rail_count() - 1 {
            ring[rail as usize] = pool.add(at);
            return Ok(());
        }

        let did_not_meet = || {
            BevelAdvisory::new(
                BevelAdvisoryCode::SurfaceStripSeamDidNotMeet,
                vec![at.into()],
            )
        };

        let side = if rail == 0 { 0 } else { 1 };

        // C# indexes the seam unguarded; a missing or empty one (which the builder never hands in)
        // is refused as the seam not meeting, which is what an unmatched one is.
        let Some(run) = seams[edge_index as usize][side]
            .as_deref()
            .filter(|run| !run.is_empty())
        else {
            return Err(did_not_meet());
        };

        let mut standing = if at_the_end {
            run[run.len() - 1].0
        } else {
            run[0].0
        };

        if (pool.position_of(standing) - at).length() > surface_strip_ends::meets_the_seam(pool) {
            standing = if at_the_end {
                run[0].0
            } else {
                run[run.len() - 1].0
            };
        }

        if (pool.position_of(standing) - at).length() > surface_strip_ends::meets_the_seam(pool) {
            return Err(did_not_meet());
        }

        ring[rail as usize] = standing;

        Ok(())
    }
}
