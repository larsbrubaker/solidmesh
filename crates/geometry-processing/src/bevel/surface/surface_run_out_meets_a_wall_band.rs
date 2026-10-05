//! Where a mixed corner's RUN-OUT meets a band standing on the WALL it turns against
//! ([`SurfaceRunOutMeetsAWallBand`]). Ports the record of
//! `Bevel/Surface/SurfaceRunOutMeetsAWallBand.cs` with its readings of the band's profile (where a
//! rail stands, the near and far sides the ray meets, the cap exit) and the reading itself
//! ([`SurfaceRunOutMeetsAWallBand::try_read`]).

use std::rc::Rc;

use vector_math::{cs_math, Vector3};

use super::super::{BevelAdvisory, BevelAdvisoryCode};
use super::{surface_mixed_corner_run_out, SurfaceMixedCorner, SurfaceStrip};

/// C# `SurfaceRunOutMeetsAWallBand` (a sealed class): where a mixed corner's run-out meets a band
/// standing on the wall it turns against, read as the radius out of the bead's axis at which that
/// band ENDS each column of the run-out.
///
/// **The band ends the column, exactly as the wall does.** A band laid on the wall takes the wall
/// away for its own setback and puts its strip in its place, so from the turn the two first meet
/// it is the BAND that stands nearest the axis and the column ends on it. One `max` of two radii is
/// the whole difference: the rows fall away in the same order and the column stays a prefix of the
/// one before it.
///
/// **The dual of a band on the cap.** A band the run-out crosses on the CAP ends the rows, each at
/// its own turn (`SurfaceRunOutMeetsTheBand`); a band on the WALL ends the columns, each at its own
/// radius. Measured on the lobed prism at `r = 3.756`: `o..o x..x w..w` - standing, then taken by
/// the band, then taken by the wall.
///
/// **One point per rail, in closed form.** A band whose edge runs along the bead's own axis has
/// rails parallel to it, each at a fixed radius `rho_i`, meeting the torus at one height
/// `z = r - sqrt(r^2 - (2r - rho_i)^2)`. The run-out is stationed AT those turns, so each crossing
/// is one vertex both surfaces are drawn to rather than two a rounding apart. And read against the
/// chords, not the arc: where a column ends is asked of the POLYLINE the rails make in the cap's
/// own plane, which is exactly on the strip's own quads.
#[derive(Debug)]
pub struct SurfaceRunOutMeetsAWallBand {
    /// The corner whose run-out this is.
    pub corner: Rc<SurfaceMixedCorner>,
    /// The band's rails, flattened into the cap's own plane.
    pub flat_rails: Vec<Vector3>,
    /// The turn about the bead's axis each of those rails stands at.
    pub rail_turns: Vec<f64>,
    /// The turns past the bead at which the run-out's own columns end ON a rail of the band, in
    /// order. These are the stations the run-out has to be drawn at: a column ending anywhere else
    /// ends part way along a quad of the strip, which is a vertex only one of the two surfaces has.
    pub rail_turns_past_the_bead: Vec<f64>,
}

impl SurfaceRunOutMeetsAWallBand {
    /// How near the band's own direction has to run to the bead's axis, as a cosine's distance
    /// from one.
    pub const ALONG_THE_AXIS: f64 = 1e-6;

    /// How near a rail's own turn has to be to a crossing's to be that rail. An angle, so fixed.
    pub const AT_THE_RAIL: f64 = 1e-9;

    /// Whether `rail` is the TURN-BACK of this band's profile, where its turn round the axis peaks.
    pub fn it_is_the_turn_back(&self, rail: i32) -> bool {
        Self::it_is_the_turn_back_of(&self.rail_turns, rail)
    }

    /// Whether `rail` is where `turns` peaks: further round than both of its neighbours (a missing
    /// neighbour reads as no turn at all).
    pub fn it_is_the_turn_back_of(turns: &[f64], rail: i32) -> bool {
        if rail < 0 || rail as usize >= turns.len() {
            return false;
        }

        let rail = rail as usize;
        let before = if rail > 0 {
            turns[rail - 1]
        } else {
            f64::NEG_INFINITY
        };
        let after = if rail + 1 < turns.len() {
            turns[rail + 1]
        } else {
            f64::NEG_INFINITY
        };

        turns[rail] > before && turns[rail] > after
    }

    /// The rail whose own turn stands `past` the bead's, or -1 where none does. (`_flat` is C#'s
    /// unused argument, kept so the reading's call reads as the C# one.)
    pub fn the_rail_at(
        _flat: &[Vector3],
        turns: &[f64],
        corner: &SurfaceMixedCorner,
        past: f64,
    ) -> i32 {
        for (rail, turn) in turns.iter().enumerate() {
            if (turn - corner.turn_radians() - past).abs() <= Self::AT_THE_RAIL {
                return rail as i32;
            }
        }

        -1
    }

    /// The rail of this band standing at `past` the bead's own turn, or -1 where none does.
    pub fn the_rail_standing_at(&self, past: f64) -> i32 {
        Self::the_rail_at(&self.flat_rails, &self.rail_turns, &self.corner, past)
    }

    /// How far out of the axis the band's profile stands at the NEAR side of the ray at `past`
    /// the bead's turn, as `(stands_off, the point on the run-out there)`; `None` where the ray
    /// misses the profile.
    pub fn try_stands_off_at(&self, past: f64) -> Option<(f64, Vector3)> {
        // A rail of the band is a VERTEX of that profile, and at the turn it stands at the ray
        // passes exactly through it. Its own radius is exact and is what the column ends at, so it
        // is answered from the rail rather than searched for - which also answers the TURN-BACK,
        // where the ray touches the profile without crossing it.
        for (rail, turn) in self.rail_turns.iter().enumerate() {
            if (turn - self.corner.turn_radians() - past).abs() <= Self::AT_THE_RAIL {
                let stands_off = (self.flat_rails[rail] - self.corner.outer_rail_centre).length();

                return Some((stands_off, self.point_at(past, stands_off)));
            }
        }

        let centre = self.corner.outer_rail_centre;
        let out_of_the_axis = self
            .corner
            .out_of_the_axis(self.corner.turn_radians() + past);
        let across = out_of_the_axis.cross(self.corner.out_of_the_cap_face);

        let mut nearest = f64::INFINITY;

        for rail in 0..self.flat_rails.len().saturating_sub(1) {
            let from = self.flat_rails[rail] - centre;
            let to = self.flat_rails[rail + 1] - centre;

            let here = from.dot(across);
            let there = to.dot(across);

            if (here > 0.0) == (there > 0.0) && here != 0.0 && there != 0.0 {
                // Both ends of this chord stand the same side of the ray, so the ray does not
                // cross it.
                continue;
            }

            let span = there - here;

            if span == 0.0 {
                continue;
            }

            let along = from + ((to - from) * (-here / span));
            let radius = along.dot(out_of_the_axis);

            if radius > 0.0 && radius < nearest {
                nearest = radius;
            }
        }

        if nearest == f64::INFINITY {
            return None;
        }

        Some((nearest, self.point_at(past, nearest)))
    }

    /// How far out of the axis the band's profile stands at the FAR side of the ray at `past`;
    /// `None` where the ray crosses no chord of it. At the TURN-BACK the two sides meet at that
    /// one vertex, so both are that rail's own radius.
    pub fn try_the_far_side_at(&self, past: f64) -> Option<f64> {
        for (rail, turn) in self.rail_turns.iter().enumerate() {
            if (turn - self.corner.turn_radians() - past).abs() <= Self::AT_THE_RAIL
                && self.it_is_the_turn_back(rail as i32)
            {
                return Some((self.flat_rails[rail] - self.corner.outer_rail_centre).length());
            }
        }

        let centre = self.corner.outer_rail_centre;
        let out_of_the_axis = self
            .corner
            .out_of_the_axis(self.corner.turn_radians() + past);
        let across = out_of_the_axis.cross(self.corner.out_of_the_cap_face);

        let mut furthest = f64::NEG_INFINITY;

        for rail in 0..self.flat_rails.len().saturating_sub(1) {
            let from = self.flat_rails[rail] - centre;
            let to = self.flat_rails[rail + 1] - centre;

            let here = from.dot(across);
            let there = to.dot(across);

            if (here > 0.0) == (there > 0.0) {
                continue;
            }

            let along = from + ((to - from) * (-here / (there - here)));
            let radius = along.dot(out_of_the_axis);

            if radius > furthest {
                furthest = radius;
            }
        }

        if furthest == f64::NEG_INFINITY {
            return None;
        }

        Some(furthest)
    }

    /// The point on the run-out's torus at `past` the bead's turn and `radius` out of the axis:
    /// in the cap where it is the touch row (`2r`), sinking under the cap toward the bead.
    pub fn point_at(&self, past: f64, radius: f64) -> Vector3 {
        let size = self.corner.size;
        let under_the_cap =
            size - cs_math::max_f64(0.0, (size * size) - ((2.0 * size) - radius).powi(2)).sqrt();

        self.corner.outer_rail_centre
            + (self
                .corner
                .out_of_the_axis(self.corner.turn_radians() + past)
                * radius)
            - (self.corner.out_of_the_cap_face * under_the_cap)
    }

    /// Where the crossing curve leaves the cap - the far branch of the band's profile out from the
    /// turn-back meeting the touch row - as `(past the bead's turn, the point)`; `None` where the
    /// far side is still outside the touch row at the hairpin, or the branch never reaches it.
    ///
    /// Walked out along the FAR branch from the turn-back, rail by rail, rather than halved for
    /// over the whole sweep: the ray misses the band entirely below the branch's own turns, so a
    /// halving that brackets the whole sweep settles on the turn the band first appears at. The
    /// crossing is the one chord of the branch whose ends straddle the touch row.
    pub fn try_the_cap_exit(&self) -> Option<(f64, Vector3)> {
        let furthest = *self.rail_turns_past_the_bead.last()?;
        let touch = 2.0 * self.corner.size;
        let the_turn_back =
            Self::the_rail_at(&self.flat_rails, &self.rail_turns, &self.corner, furthest);

        if the_turn_back < 0 {
            return None;
        }
        let at_the_hairpin = self.try_the_far_side_at(furthest)?;
        if at_the_hairpin >= touch {
            // The far side is still outside the touch row where the two sides meet, so the curve
            // reaches the cap on its NEAR side instead and there is no hairpin to collapse.
            return None;
        }

        let outward = self.outward_of_the_turn_back(the_turn_back);

        if outward == 0 {
            return None;
        }

        let centre = self.corner.outer_rail_centre;
        let count = self.flat_rails.len() as i32;

        let mut rail = the_turn_back;
        while rail + outward >= 0 && rail + outward < count {
            let from = self.flat_rails[rail as usize] - centre;
            let to = self.flat_rails[(rail + outward) as usize] - centre;

            if to.length() < touch {
                rail += outward;
                continue;
            }

            let away = Self::try_across(from, to, touch)?;

            let past = f64::atan2(
                away.dot(self.corner.towards_the_wall_square()),
                away.dot(self.corner.towards_the_shared_face()),
            ) - self.corner.turn_radians();

            return Some((past, self.point_at(past, touch)));
        }

        // The branch runs off the end of the band without ever reaching the touch row, so the
        // curve never dies on the cap and there is nothing to cut either surface back to.
        None
    }

    /// Which way along the rails the profile's radius rises away from the turn-back: -1, 1, or 0
    /// where it rises neither way.
    pub fn outward_of_the_turn_back(&self, the_turn_back: i32) -> i32 {
        let centre = self.corner.outer_rail_centre;
        let at = the_turn_back as usize;
        let here = (self.flat_rails[at] - centre).length();
        let before = if at > 0 {
            (self.flat_rails[at - 1] - centre).length()
        } else {
            f64::NEG_INFINITY
        };
        let after = if at + 1 < self.flat_rails.len() {
            (self.flat_rails[at + 1] - centre).length()
        } else {
            f64::NEG_INFINITY
        };

        if before > here && before >= after {
            return -1;
        }

        if after > here {
            1
        } else {
            0
        }
    }

    /// Where the chord `from`..`to` crosses the circle of `radius` about the origin - the root
    /// further along the chord first - or `None` where it does not within the chord.
    #[allow(clippy::manual_range_contains)] // C#'s comparisons, which let a NaN through.
    pub fn try_across(from: Vector3, to: Vector3, radius: f64) -> Option<Vector3> {
        let along = to - from;
        let a = along.length_squared();
        let b = 2.0 * from.dot(along);
        let c = from.length_squared() - (radius * radius);
        let discriminant = (b * b) - (4.0 * a * c);

        if a == 0.0 || discriminant < 0.0 {
            return None;
        }

        let root = discriminant.sqrt();
        let mut how_far = (-b + root) / (2.0 * a);

        if how_far < 0.0 || how_far > 1.0 {
            how_far = (-b - root) / (2.0 * a);
        }

        if how_far < 0.0 || how_far > 1.0 {
            return None;
        }

        Some(from + (along * how_far))
    }
}

impl SurfaceRunOutMeetsAWallBand {
    /// C#'s private constructor.
    pub fn new(
        corner: Rc<SurfaceMixedCorner>,
        flat_rails: Vec<Vector3>,
        rail_turns: Vec<f64>,
        crosses_at: Vec<f64>,
    ) -> Self {
        SurfaceRunOutMeetsAWallBand {
            corner,
            flat_rails,
            rail_turns,
            rail_turns_past_the_bead: crosses_at,
        }
    }

    /// The run-out's `stations` short of the band's first rail crossing (by `hair`), then each rail
    /// crossing a `hair` past the last station kept.
    pub fn stationed(&self, stations: &[f64], hair: f64) -> Vec<f64> {
        let mut stationed =
            Vec::with_capacity(stations.len() + self.rail_turns_past_the_bead.len() + 1);
        let first = self.rail_turns_past_the_bead[0];

        for &station in stations {
            if station < first - hair {
                stationed.push(station);
            }
        }

        for &rail in &self.rail_turns_past_the_bead {
            if stationed.last().is_none_or(|&last| rail - last > hair) {
                stationed.push(rail);
            }
        }

        stationed
    }
}

impl SurfaceRunOutMeetsAWallBand {
    /// C# `TryRead`: where the mixed corner's run-out meets `band`, a band standing on the wall it
    /// turns against - each rail flattened into the cap's plane, its turn about the run-out's centre,
    /// and the turns past the bead of the rails that cross the run-out's own quarter of the torus.
    /// `Ok(None)` (C#'s false with no failure) where there is nothing to sew; `Err` names a crossing
    /// of another shape. `_hands_over_at_a_mitre` is read by no line of the C#.
    pub fn try_read(
        corner: Option<&Rc<SurfaceMixedCorner>>,
        band: Option<&SurfaceStrip>,
        _hands_over_at_a_mitre: bool,
    ) -> Result<Option<SurfaceRunOutMeetsAWallBand>, BevelAdvisory> {
        let (Some(corner), Some(band)) = (corner, band) else {
            return Ok(None);
        };
        #[allow(clippy::neg_cmp_op_on_partial_ord)] // NaN declines, as the C#'s `!(x > 0)` does
        let no_size = !(corner.size > 0.0);
        if band.rail_count() < 2 || no_size {
            return Ok(None);
        }

        if band.along().dot(corner.axis().direction).abs() < 1.0 - Self::ALONG_THE_AXIS {
            return Err(BevelAdvisory::new(
                BevelAdvisoryCode::SurfaceCrossingBandNotAlongTheAxis,
                vec![corner.corner.into()],
            ));
        }

        let centre = corner.outer_rail_centre;
        let out_of_the_cap = corner.out_of_the_cap_face;
        let rails = band.rail_count() as usize;
        let mut flat = vec![Vector3::ZERO; rails];
        let mut turns = vec![0.0; rails];
        let mut crossings = Vec::with_capacity(rails);

        for rail in 0..rails {
            let at = band.point_on(rail as i32, 0.0);

            // In the cap's own plane, where every way out of the axis lies.
            flat[rail] = at - (out_of_the_cap * (at - centre).dot(out_of_the_cap));
            let away = flat[rail] - centre;
            turns[rail] = away
                .dot(corner.towards_the_wall_square())
                .atan2(away.dot(corner.towards_the_shared_face()));
            let radius = away.length();

            // Only a rail between the ball's own radius and the circle its touch point traces meets
            // the patch at all.
            if radius > corner.size
                && radius < 2.0 * corner.size
                && turns[rail] > corner.turn_radians()
                && turns[rail]
                    <= corner.turn_radians() + surface_mixed_corner_run_out::clears_the_wall()
            {
                crossings.push(turns[rail] - corner.turn_radians());
            }
        }

        if crossings.is_empty() {
            // The band stands clear of the run-out's own quarter of the torus: nothing to sew.
            return Ok(None);
        }

        if crossings.len() < 2 {
            // ONE rail of the band inside the run-out, and it the hairpin.
            return Err(BevelAdvisory::new(
                BevelAdvisoryCode::SurfaceCrossingAtTheRunOutsDeath,
                vec![corner.corner.into()],
            ));
        }

        // C#'s `List<double>.Sort` (no NaN reaches here; equal values are indistinguishable).
        crossings.sort_by(|a: &f64, b: &f64| a.partial_cmp(b).expect("finite crossings"));

        let last = crossings[crossings.len() - 1];
        if !Self::it_is_the_turn_back_of(&turns, Self::the_rail_at(&flat, &turns, corner, last)) {
            // The turn the profile folds back at stands outside the run-out's own quarter.
            return Err(BevelAdvisory::new(
                BevelAdvisoryCode::SurfaceCrossingTurnsBackBeyondTheRunOut,
                vec![corner.corner.into()],
            ));
        }

        Ok(Some(SurfaceRunOutMeetsAWallBand::new(
            corner.clone(),
            flat,
            turns,
            crossings,
        )))
    }
}
