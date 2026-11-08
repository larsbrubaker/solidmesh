//! Where a mixed corner's RUN-OUT and another chain's BAND land on one another
//! ([`SurfaceRunOutMeetsTheBand`]), read as the turns the run-out's own rows and its foot leave the
//! band's material at: its fields, its constructor and its questions (where a row crosses, whether
//! a row is taken, whether a point stands in the section, which rail a row is sewn to, and the turn
//! its foot enters the section at), and the reading itself
//! ([`SurfaceRunOutMeetsTheBand::try_read`]).

use std::rc::Rc;

use solidmesh_math::{Vector2, Vector3};

use super::surface_mixed_corner_run_out;
use super::{surface_mixed_corner_wall_ends, SurfaceBandEnd};
use super::{SurfaceMixedCorner, SurfaceMixedCornerWall, SurfaceStrip};
use crate::bevel::{BevelAdvisory, BevelAdvisoryCode};

/// A run-out and a band crossing, read per row of the run-out.
///
/// **One arc, swept two ways.** The band's strip is the r-arc swept along a straight line and the
/// run-out's rings are the same r-arc turned about the bead's axis, so at every height the two
/// carry the same offset from their own tangent line: row j stands `d_j` in from the touch point,
/// which puts it `rho_j = 2r - d_j` out of the axis, and rail i stands `d_i` in from the band's
/// foot. The two surfaces cross where those offsets meet, which per ROW is simply the turn at which
/// that row's circle reaches the vertical plane its own rail stands in.
///
/// **Rows leave from the cap upward**, a PREFIX of the column, which is what lets the columns stay
/// prefixes of each other. **And the foot last**: past the turn the run-out's trim point enters the
/// section the band's end cut takes, it is standing on face the band has already taken, so that
/// turn is where it dies. **Paired by where they stand, never by index**: rows and rails are
/// matched by height and offset, and a pair that does not match to the part's own ruler is refused
/// by name rather than sewn to the nearest rail.
#[derive(Debug, Clone, PartialEq)]
pub struct SurfaceRunOutMeetsTheBand {
    /// Per row of the run-out's seed, the turn past the bead's own last at which that row leaves
    /// the band's material; infinity where that row's circle never reaches the band at all.
    pub crosses_at: Vec<f64>,
    /// Per row of the seed, whether the band already has it at turn zero.
    pub the_band_has_it_at_the_seed: Vec<bool>,
    /// The band's strip, whose rails the crossings stand on.
    pub band: Rc<SurfaceStrip>,
    /// The band's profile, rail by rail, read at the strip's own origin.
    pub profile: Vec<Vector3>,
    /// The band's own sharp corner - the edge it was taken off - in that same section.
    pub sharp_corner: Vector3,
    /// One of the two axes the band's section is read in, square to the way its rails run.
    pub across: Vector3,
    /// The other axis of the band's section; see [`SurfaceRunOutMeetsTheBand::across`].
    pub up: Vector3,
}

impl SurfaceRunOutMeetsTheBand {
    /// How many steps the reading takes across the run-out's sweep.
    pub const STEPS_ACROSS_THE_SWEEP: i32 = 240;

    /// Builds the reading from its parts.
    #[allow(clippy::too_many_arguments)]
    pub fn new(
        crosses_at: Vec<f64>,
        the_band_has_it_at_the_seed: Vec<bool>,
        band: Rc<SurfaceStrip>,
        profile: Vec<Vector3>,
        sharp_corner: Vector3,
        across: Vector3,
        up: Vector3,
    ) -> Self {
        SurfaceRunOutMeetsTheBand {
            crosses_at,
            the_band_has_it_at_the_seed,
            band,
            profile,
            sharp_corner,
            across,
            up,
        }
    }

    /// The turn at which `row` leaves the band's material, or infinity where it never does (or
    /// there is no such row).
    pub fn where_the_row_crosses(&self, row: i32) -> f64 {
        if row >= 0 && (row as usize) < self.crosses_at.len() {
            self.crosses_at[row as usize]
        } else {
            f64::INFINITY
        }
    }

    /// Whether the band already has any row at the run-out's seed.
    pub fn it_has_taken_a_row_already(&self) -> bool {
        self.the_band_has_it_at_the_seed.iter().any(|&taken| taken)
    }

    /// Whether `at` stands inside the band's section - the polygon of its profile closed at the
    /// sharp corner - read in the (across, up) plane by the even-odd rule.
    pub fn it_stands_in_the_section(&self, at: Vector3) -> bool {
        let read = |p: Vector3| Vector2::new(p.dot(self.across), p.dot(self.up));
        let corner = read(self.sharp_corner);
        let here = read(at);

        let mut inside = false;
        let mut previous = corner;

        for rail in 0..=self.profile.len() {
            let next = if rail < self.profile.len() {
                read(self.profile[rail])
            } else {
                corner
            };

            if (previous.y > here.y) != (next.y > here.y) {
                let crosses = previous.x
                    + ((here.y - previous.y) / (next.y - previous.y) * (next.x - previous.x));

                if crosses > here.x {
                    inside = !inside;
                }
            }

            previous = next;
        }

        inside
    }

    /// The one free rail of `profile` a run-out `row` is sewn to: the nearest untaken rail within
    /// `near_enough` of the row's height (along `up`, about the corner's outer rail centre), and
    /// only where it is the ONLY rail within the ruler, taken or not - two rails under one row is a
    /// chording the two do not share.
    pub fn try_the_rail_for(
        corner: &SurfaceMixedCorner,
        profile: &[Vector3],
        row: Vector3,
        up: Vector3,
        near_enough: f64,
        taken: &[bool],
    ) -> Option<usize> {
        let height = (row - corner.outer_rail_centre).dot(up);
        let mut nearest = f64::MAX;
        let mut others = 0;
        let mut rail = None;

        for (candidate, &point) in profile.iter().enumerate() {
            let off = ((point - corner.outer_rail_centre).dot(up) - height).abs();

            if off > near_enough {
                continue;
            }

            others += 1;

            if !taken[candidate] && off < nearest {
                nearest = off;
                rail = Some(candidate);
            }
        }

        rail.filter(|_| others == 1)
    }
}

impl SurfaceRunOutMeetsTheBand {
    /// The turn at which the run-out's own foot ENTERS the section the band's end cut takes out of
    /// the wall, or `None` where it never does (a run-out that dies on the wall as it always did) -
    /// or where it is inside already at the bead's end, which is a band under the blend itself and
    /// a corner of another shape.
    ///
    /// Where it enters need not be where it stays: a section is a bounded polygon the foot can pass
    /// right through, so the sweep is walked in [`Self::STEPS_ACROSS_THE_SWEEP`] steps to bracket
    /// the FIRST entry and halved sixty times from there.
    pub fn try_where_the_foot_crosses(
        &self,
        corner: &SurfaceMixedCorner,
        wall: &SurfaceMixedCornerWall,
        clears: f64,
    ) -> Option<f64> {
        let inside = surface_mixed_corner_run_out::try_scallop_point(corner, wall, 0.0)?;

        if self.it_stands_in_the_section(inside) {
            return None;
        }

        let mut low = 0.0;
        let mut high = clears;
        let mut entered = false;

        for step in 1..=Self::STEPS_ACROSS_THE_SWEEP {
            // Multiplied then divided, left to right, so every station lands on the same value.
            let at = clears * step as f64 / Self::STEPS_ACROSS_THE_SWEEP as f64;
            let here = surface_mixed_corner_run_out::try_scallop_point(corner, wall, at)?;

            if self.it_stands_in_the_section(here) {
                high = at;
                entered = true;
                break;
            }

            low = at;
        }

        if !entered {
            return None;
        }

        for _ in 0..60 {
            let middle = (low + high) / 2.0;
            let at = surface_mixed_corner_run_out::try_scallop_point(corner, wall, middle)?;

            if self.it_stands_in_the_section(at) {
                high = middle;
            } else {
                low = middle;
            }
        }

        Some((low + high) / 2.0)
    }
}

impl SurfaceRunOutMeetsTheBand {
    /// Where the mixed corner's run-out rows (`rows`, one per rail of the band's section) cross
    /// another chain's `band` - each row sewn to its own rail, read against one plane standing in
    /// that rail's line facing the band's material: where it crosses and whether the band already
    /// has it at the seed. `Ok(None)` where there is nothing to read; `Err` where the two profiles
    /// do not match.
    pub fn try_read(
        corner: Option<&SurfaceMixedCorner>,
        band: Option<&Rc<SurfaceStrip>>,
        the_bands_edge: Vector3,
        rows: Option<&[Vector3]>,
        near_enough: f64,
    ) -> Result<Option<SurfaceRunOutMeetsTheBand>, BevelAdvisory> {
        let (Some(corner), Some(band), Some(rows)) = (corner, band, rows) else {
            return Ok(None);
        };
        if rows.len() < 2 || band.rail_count() < 2 {
            return Ok(None);
        }

        // The band's own section, square to the way its rails run.
        let profile: Vec<Vector3> = (0..band.rail_count())
            .map(|rail| band.point_on(rail, 0.0))
            .collect();

        let up = -corner.out_of_the_cap_face;
        let across = band.along().cross(up);
        #[allow(clippy::neg_cmp_op_on_partial_ord)] // `!(x > 0)` on purpose: NaN declines
        let square = !(across.length_squared() > 0.0);
        if square {
            // The band runs square to the cap it is crossed on.
            return Ok(None);
        }

        let mut across = across.get_normal();

        // Pointing the way the material the band KEPT lies.
        if across.dot(corner.corner - profile[0]) < 0.0 {
            across = -across;
        }

        let do_not_match = || {
            BevelAdvisory::new(
                BevelAdvisoryCode::SurfaceCrossingProfilesDoNotMatch,
                vec![corner.corner.into()],
            )
        };

        if rows.len() != profile.len() {
            // Two samplings of one arc that do not even have the same number of samples.
            return Err(do_not_match());
        }

        let mut crosses_at = vec![0.0; rows.len()];
        let mut already = vec![false; rows.len()];
        let mut taken = vec![false; profile.len()];

        for (row, &at) in rows.iter().enumerate() {
            let Some(rail) = Self::try_the_rail_for(corner, &profile, at, up, near_enough, &taken)
            else {
                return Err(do_not_match());
            };

            taken[rail] = true;

            // One plane per row, standing in its own rail's line and facing the band's material.
            let meets_there = SurfaceBandEnd::new(profile[rail], across);

            already[row] = !surface_mixed_corner_wall_ends::stands_in_the_material(meets_there, at);
            crosses_at[row] =
                surface_mixed_corner_wall_ends::where_it_crosses(corner, at, meets_there);
        }

        Ok(Some(SurfaceRunOutMeetsTheBand::new(
            crosses_at,
            already,
            band.clone(),
            profile,
            the_bands_edge,
            across,
            up,
        )))
    }
}
