//! The mixed-corner patch's rows laid ALONG THE SPINE ([`SurfaceMixedCornerRows`]). Ports
//! `Bevel/Surface/SurfaceMixedCornerRows.cs`: the record, its two readings and its rail accessor.
//! C#'s overloads are named apart here: `TryRead(SurfaceMixedCorner, ..)` is
//! [`SurfaceMixedCornerRows::try_read_corner`], and the per-turn `AcrossAt`/`SweepAt` are
//! `across_at_turn`/`sweep_at_turn` beside the static `across_at`/`sweep_at`.

use std::rc::Rc;

use vector_math::Vector3;

use super::{SurfaceMixedCorner, SurfaceMixedCornerSpine};
use crate::bevel::{BevelAdvisory, BevelAdvisoryCode};

/// How far the row at a turn RUNS, as an angle at the ball's centre (C# `Func<double, double>`).
pub type SurfaceMixedCornerRowRunsTo = Rc<dyn Fn(f64) -> f64>;

/// C# `SurfaceMixedCornerRows` (a sealed class): the patch's rows, laid ALONG THE SPINE - the ring
/// at station zero carried from station to station on the rolling ball rather than turned rigidly
/// about an axis.
///
/// **Which canal, though - the CALLER's to say.** Told a corner, these rows are laid on the
/// CAP-RIDING canal: exact for a square corner, and a sub-band of an oblique one, which the build's
/// own gate (`BevelAdvisoryCode::SurfaceObliqueMixedCornerRowsNotBuilt`) still holds back. Told a
/// SPINE instead, the same arithmetic lays the dual's rows - which is what a two-crease junction's
/// patch is made of (`SurfaceTwoCreaseJunctionRings`).
///
/// **What a station's ring is.** A canal of CONSTANT radius meets each of its balls in the circle
/// of radius `r` lying in the plane NORMAL TO THE SPINE'S TANGENT. The two contacts the ball is
/// defined by - `TContactAt` at `+u` and `BeadContactAt` at `-e(phi)` - are the two ENDS of the
/// station's own ring, and the ring is the arc between them.
///
/// **And the arc SHRINKS along the spine, which is the whole of what leaning costs.** On the
/// oblique fixture 112.62 degrees at turn zero closing to 66.91 at the end of the turn. A rigid
/// turn keeps the first all the way round, which lifts its own outer rail off the cap and off the
/// bead's cylinder alike. So each rail is carried at its FRACTION of the sweep: rail `p` sits at
/// `psi_p` of the ring at turn zero and at `psi_p * sweep(phi) / sweep(0)` at every station after.
///
/// **A square cap is the rigid turn, rail for rail** - Rodrigues written out - for a ring of ANY
/// shape, so nothing that builds today moves off the arithmetic's last bits.
pub struct SurfaceMixedCornerRows {
    /// The spine the rows are carried along.
    pub spine: SurfaceMixedCornerSpine,
    /// The way from the ball's centre to the face it rides.
    pub towards_the_plate: Vector3,
    /// The ring's sweep at station zero, which every later station's fraction is taken of.
    pub sweep_at_turn_zero: f64,
    /// How far the row at a turn RUNS, or `None` where that is simply the ball's own sweep.
    pub runs_to: Option<SurfaceMixedCornerRowRunsTo>,
    /// Each rail as its angle round the station's ring, its radius there, and how far it stands
    /// off that ring's own plane.
    pub rails: Vec<(f64, f64, f64)>,
}

impl SurfaceMixedCornerRows {
    /// C#'s private constructor.
    fn new(
        spine: SurfaceMixedCornerSpine,
        towards_the_plate: Vector3,
        sweep_at_turn_zero: f64,
        runs_to: Option<SurfaceMixedCornerRowRunsTo>,
        rails: Vec<(f64, f64, f64)>,
    ) -> Self {
        SurfaceMixedCornerRows {
            spine,
            towards_the_plate,
            sweep_at_turn_zero,
            runs_to,
            rails,
        }
    }

    /// The rows of a cap-riding corner: its own spine, against its cap face, from the cut's ring.
    pub fn try_read_corner(
        corner: &SurfaceMixedCorner,
        the_cuts_ring: &[Vector3],
    ) -> Result<SurfaceMixedCornerRows, BevelAdvisory> {
        Self::try_read(
            corner.spine.clone(),
            corner.out_of_the_cap_face,
            the_cuts_ring,
            corner.corner,
            None,
        )
    }

    /// Reads station zero's rails (`station_zero`) in the frame of `spine` at turn zero - each
    /// rail's angle, radius and offset off the profile plane about the ball's centre, the plate
    /// being the plane `towards_the_plate` faces out of - so every later station can lay the same
    /// rails at its own turn. `runs_to` is the sweep each turn's row runs to (C#'s optional
    /// argument; `None` reads it off the spine). Refused (at `corner`) for a ring of one rail.
    #[allow(clippy::neg_cmp_op_on_partial_ord)] // C#'s `!(x > 0)`, which refuses a NaN as well.
    pub fn try_read(
        spine: SurfaceMixedCornerSpine,
        towards_the_plate: Vector3,
        station_zero: &[Vector3],
        corner: Vector3,
        runs_to: Option<SurfaceMixedCornerRowRunsTo>,
    ) -> Result<SurfaceMixedCornerRows, BevelAdvisory> {
        let sweep = Self::sweep_at(towards_the_plate, spine.out_of_the_axis(0.0));

        if station_zero.len() < 2 || !(sweep > 0.0) {
            // A ring of one rail is no ring. The sweep is the belt on the proof - it cannot be
            // zero at a corner the reading accepted - and is asked at turn zero because it is the
            // one the fractions are taken against.
            return Err(BevelAdvisory::new(
                BevelAdvisoryCode::SurfaceCornerNotSupported,
                vec![corner.into()],
            ));
        }

        let centre = spine.centre_at(0.0);
        let along = towards_the_plate;
        let across = Self::across_at(towards_the_plate, spine.out_of_the_axis(0.0));
        let off_the_plane = along.cross(across);

        let read = station_zero
            .iter()
            .map(|&rail| {
                let from = rail - centre;
                let height = from.dot(along);
                let width = from.dot(across);

                (
                    f64::atan2(width, height),
                    ((height * height) + (width * width)).sqrt(),
                    from.dot(off_the_plane),
                )
            })
            .collect();

        let sweep_at_turn_zero = match &runs_to {
            Some(runs_to) => runs_to(0.0),
            None => sweep,
        };

        Ok(SurfaceMixedCornerRows::new(
            spine,
            towards_the_plate,
            sweep_at_turn_zero,
            runs_to,
            read,
        ))
    }

    /// Rail `rail` of the row at `turn`: station zero's rail laid about that turn's ball centre,
    /// its angle scaled to the sweep the row runs to there.
    pub fn at(&self, rail: usize, turn: f64) -> Vector3 {
        let along = self.towards_the_plate;
        let across = self.across_at_turn(turn);
        let (angle, radius, off_the_plane) = self.rails[rail];
        let at = angle * self.runs_to_at(turn) / self.sweep_at_turn_zero;

        self.spine.centre_at(turn)
            + (along * (radius * at.cos()))
            + (across * (radius * at.sin()))
            + (along.cross(across) * off_the_plane)
    }

    /// The row's in-plane direction across the profile at `turn` (C#'s private `AcrossAt(double)`).
    pub fn across_at_turn(&self, turn: f64) -> Vector3 {
        Self::across_at(self.towards_the_plate, self.spine.out_of_the_axis(turn))
    }

    /// The way from the ball's centre towards the bead, squared up to the plate and normalised
    /// outright, with no zero case to fall back to: a fallback here would collapse the whole row
    /// onto one point rather than say anything.
    pub fn across_at(towards_the_plate: Vector3, out_of_the_axis: Vector3) -> Vector3 {
        let towards_the_bead = -out_of_the_axis;
        let across =
            towards_the_bead - (towards_the_plate * towards_the_bead.dot(towards_the_plate));

        across.get_normal()
    }

    /// The sweep from the plate round to the bead at `turn` (C#'s private `SweepAt(double)`).
    pub fn sweep_at_turn(&self, turn: f64) -> f64 {
        Self::sweep_at(self.towards_the_plate, self.spine.out_of_the_axis(turn))
    }

    /// What the row at `turn` runs to: the caller's `runs_to` where there is one, and the sweep
    /// otherwise (C#'s private `RunsTo`).
    pub fn runs_to_at(&self, turn: f64) -> f64 {
        match &self.runs_to {
            Some(runs_to) => runs_to(turn),
            None => self.sweep_at_turn(turn),
        }
    }

    /// The angle from `towards_the_plate` round to the way towards the bead.
    pub fn sweep_at(towards_the_plate: Vector3, out_of_the_axis: Vector3) -> f64 {
        let towards_the_bead = -out_of_the_axis;
        let along = towards_the_bead.dot(towards_the_plate);
        let across = towards_the_bead - (towards_the_plate * along);

        f64::atan2(across.length(), along)
    }
}
