//! A LEANING one-crease mixed corner's canal, read WHOLE ([`SurfaceMixedCornerDualCanal`]).
//! Ports the record of `Bevel/Surface/SurfaceMixedCornerDualCanal.cs` with the turn it dies at, its
//! crease and its row accessors; the reading itself (`TryRead`) lands with a later package.

use std::rc::Rc;

use vector_math::Vector3;

use super::{SurfaceLine, SurfaceMixedCorner, SurfaceMixedCornerDualFrame};

/// C# `SurfaceMixedCornerDualCanal` (a sealed class): the dual's void ball rolling from the bead's
/// own end ring round until it dies on the crease the cap and the wall meet in - which is further
/// than the cut's own strip goes, and is the whole of why this corner needed a reading of its own.
///
/// **The canal does not stop where the cut does.** Measured against the Onshape control of
/// `NoseInNotchTestMeshes`, the blend KEEPS TURNING past the cut strip's end - 112.62 degrees to
/// 158.5036 at `r = 1` - and every vertex out there stands one radius off the SAME spine
/// (`BevelTopCenterFaceRegressionTests`). So the run-out is this canal turned further, and what
/// this adds to the frame is only WHERE it stops and WHAT bounds each row once the cut's strip has
/// ended.
///
/// **Where it stops.** The ball keeps circling until its contact on the WALL reaches an
/// UNSELECTED edge - [`SurfaceMixedCornerDualCanal::crease`] - at
/// [`SurfaceMixedCornerDualCanal::dies_at`], where the ball's centre stands exactly one radius from
/// the crease's LINE, so it touches the sharp edge at a point and the blend fades out instead of
/// cutting a step. The crease needs no finding: it is where the cap's plane and the wall's plane
/// meet, both through the corner.
///
/// **What bounds a row, and it changes half way.** While the cut's strip is still there a row ends
/// on that strip's tube; once it has ended the row ends where the ball's ring crosses the CAP's
/// plane. The two agree exactly at the cut strip's own end turn, so the boundary is continuous,
/// and the row's extent reaches zero exactly at the turn the blend dies at - the last column is a
/// POINT.
#[derive(Debug)]
pub struct SurfaceMixedCornerDualCanal {
    /// The corner this was read off, shared with every other reading of it.
    pub corner: Rc<SurfaceMixedCorner>,
    /// The canal itself - the frame the whole of this corner stands on.
    pub dual: Rc<SurfaceMixedCornerDualFrame>,
    /// The edge the cap and the wall meet in, from the corner outwards - the unselected edge the
    /// blend runs out onto and dies on.
    pub crease: SurfaceLine,
    /// The turn the blend dies at, where the ball's contact on the wall reaches the crease.
    pub dies_at: f64,
}

impl SurfaceMixedCornerDualCanal {
    /// How far past the cut strip's own sweep the turn may be scanned for where the canal dies,
    /// as a multiple of that sweep.
    pub const THE_FURTHEST_THE_TURN_MAY_RUN: f64 = 3.0;

    /// How many steps that scan takes before bisecting the one that brackets the death.
    pub const SCAN_STEPS: i32 = 512;

    /// C#'s private constructor.
    pub fn new(
        corner: Rc<SurfaceMixedCorner>,
        dual: Rc<SurfaceMixedCornerDualFrame>,
        crease: SurfaceLine,
        dies_at: f64,
    ) -> Self {
        SurfaceMixedCornerDualCanal {
            corner,
            dual,
            crease,
            dies_at,
        }
    }

    /// Where the canal dies on the crease: the wall rail at the turn it dies at.
    pub fn dies_on(&self) -> Vector3 {
        self.dual.spine.t_contact_at(self.dies_at)
    }

    /// How far along the crease the canal reaches before it dies.
    pub fn reach_along_the_crease(&self) -> f64 {
        (self.dies_on() - self.crease.point).dot(self.crease.direction)
    }

    /// The turn the CUT's own strip ends at - the dual's own sweep; past it the run-out carries.
    pub fn the_cut_strip_ends_at(&self) -> f64 {
        self.dual.turn_radians
    }

    /// The corner the canal is read of (C#'s `Corner` accessor).
    pub fn corner(&self) -> &Rc<SurfaceMixedCorner> {
        &self.corner
    }

    /// The way from the ball's centre towards the wall: into the wall it rolls on.
    pub fn towards_the_wall(&self) -> Vector3 {
        -self.dual.out_of_the_wall
    }

    /// The angle the row at `turn` runs to round its ring: the whole sweep to the tube while the
    /// cut's strip lasts, and past it the LARGER of the ring's two crossings of the cap plane (the
    /// one that meets the tube contact at the changeover), or zero where the ring misses the cap.
    pub fn row_runs_to(&self, turn: f64) -> f64 {
        let sweep = self.sweep_at(turn);

        if turn <= self.dual.turn_radians {
            return sweep;
        }

        match self.try_the_cap_crossing(turn) {
            // C#'s Math.Min, which passes a NaN through.
            Some(crossing) if crossing.is_nan() || sweep.is_nan() => f64::NAN,
            Some(crossing) => crossing.min(sweep),
            None => 0.0,
        }
    }

    /// Where the row at `turn` ends: on its ring at the angle it runs to.
    pub fn row_ends_at(&self, turn: f64) -> Vector3 {
        self.on_the_ring(turn, self.row_runs_to(turn))
    }

    /// The point `angle` round the ball's ring at `turn`, from the way towards the wall.
    pub fn on_the_ring(&self, turn: f64, angle: f64) -> Vector3 {
        let along = self.towards_the_wall();
        let across = self.across_at(turn);

        self.dual.spine.centre_at(turn)
            + (along * (self.corner.size * angle.cos()))
            + (across * (self.corner.size * angle.sin()))
    }

    /// The crease the dual canal runs out along, from the corner outwards: the cap's plane met
    /// with the wall's, signed by where the ball's wall contact stands at the end of the cut's
    /// sweep. `None` where a cap and a wall in one plane meet in no line.
    #[allow(clippy::neg_cmp_op_on_partial_ord)] // C#'s `!(x > 0)`, which refuses a NaN as well.
    pub fn try_the_crease(
        corner: &SurfaceMixedCorner,
        dual: &SurfaceMixedCornerDualFrame,
    ) -> Option<SurfaceLine> {
        let along = corner.out_of_the_cap_face.cross(corner.out_of_the_wall);

        if !(along.length_squared() > 0.0) {
            return None;
        }

        let along = along.get_normal();

        let outwards = dual.spine.t_contact_at(dual.turn_radians) - corner.corner;

        Some(SurfaceLine::new(
            corner.corner,
            if outwards.dot(along) >= 0.0 {
                along
            } else {
                -along
            },
        ))
    }

    /// The turn past the cut's sweep at which the wall rail crosses the cap's plane - where the
    /// canal dies - found by a scan of [`Self::SCAN_STEPS`] up to
    /// [`Self::THE_FURTHEST_THE_TURN_MAY_RUN`] times the sweep and a bisection of the bracketing
    /// step; `None` where it never crosses in that range.
    #[allow(clippy::neg_cmp_op_on_partial_ord)] // C#'s `!(x > 0)`, which refuses a NaN as well.
    pub fn try_the_turn_it_dies_at(&self) -> Option<f64> {
        let off_the_cap = |turn: f64| {
            (self.dual.spine.t_contact_at(turn) - self.corner.corner)
                .dot(self.corner.out_of_the_cap_face)
        };

        let mut from = self.dual.turn_radians;
        let furthest = Self::THE_FURTHEST_THE_TURN_MAY_RUN * self.dual.turn_radians;
        let sign = off_the_cap(from) > 0.0;
        let step = (furthest - from) / f64::from(Self::SCAN_STEPS);

        if !(step > 0.0) {
            return None;
        }

        let mut to = from + step;
        while to <= furthest {
            if (off_the_cap(to) > 0.0) == sign {
                from = to;
                to += step;
                continue;
            }

            return Some(Self::bisect(&off_the_cap, from, to, sign));
        }

        None
    }

    /// The angle round the ring at `turn` where it crosses the cap's plane - the larger of its two
    /// crossings - or `None` where the ring stands wholly on one side of that plane.
    #[allow(clippy::neg_cmp_op_on_partial_ord)] // C#'s `!(x > 0)`, which refuses a NaN as well.
    pub fn try_the_cap_crossing(&self, turn: f64) -> Option<f64> {
        let cap = self.corner.out_of_the_cap_face;
        let height = (self.dual.spine.centre_at(turn) - self.corner.corner).dot(cap);
        let a = self.corner.size * self.towards_the_wall().dot(cap);
        let b = self.corner.size * self.across_at(turn).dot(cap);
        let reach = ((a * a) + (b * b)).sqrt();

        if !(reach > 0.0) || height.abs() > reach {
            // The ring stands wholly on one side of the cap's plane, so nothing of the cap bounds
            // it.
            return None;
        }

        Some(f64::atan2(b, a) + (-height / reach).clamp(-1.0, 1.0).acos())
    }

    /// The ring's in-plane direction across at `turn`: towards the cut's tube, squared up to the
    /// way towards the wall.
    pub fn across_at(&self, turn: f64) -> Vector3 {
        let towards_the_tube = -self.dual.spine.out_of_the_axis(turn);
        let along = self.towards_the_wall();

        (towards_the_tube - (along * towards_the_tube.dot(along))).get_normal()
    }

    /// The angle from the way towards the wall round to the cut's tube at `turn`.
    pub fn sweep_at(&self, turn: f64) -> f64 {
        let towards_the_tube = -self.dual.spine.out_of_the_axis(turn);
        let along = self.towards_the_wall();
        let height = towards_the_tube.dot(along);

        f64::atan2((towards_the_tube - (along * height)).length(), height)
    }

    /// Two hundred halvings of `from`..`to`, keeping the end whose sign of `of` is `sign`.
    pub fn bisect(of: &dyn Fn(f64) -> f64, from: f64, to: f64, sign: bool) -> f64 {
        let (mut from, mut to) = (from, to);

        for _ in 0..200 {
            let middle = (from + to) / 2.0;

            if (of(middle) > 0.0) == sign {
                from = middle;
            } else {
                to = middle;
            }
        }

        (from + to) / 2.0
    }
}
