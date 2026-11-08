//! Where a mixed corner's RUN-OUT skirt and another chain's BAND land on one another over the same
//! region, and the two footprints are sewn together rather than refused ([`SurfaceRoomCrossing`]).
//! Its record, its derived readings, the test that finds one ([`SurfaceRoomCrossing::try_cross`])
//! and the gathering of every crossing of a pass ([`SurfaceRoomCrossing::gather`]).

use solidmesh_mesh::CancelToken;

use super::{SurfaceRoomCut, SurfaceRoomFootprint};
use crate::bevel::level_line::WorkError;
use solidmesh_math::ieee_math;

/// A run-out skirt and a band crossing over one region.
///
/// Every other pair `surface_room::largest_fitting` solves is two cuts facing each other across a
/// region, each the other's far rim, and the two of them share the gap:
/// `size * (k_a + k_b) <= gap`. A skirt over a band is not that. Both footprints are REMOVALS from
/// one face, so where they overlap the face is simply taken twice and what is left is the lower
/// envelope of the two - the band's strip and the run-out's torus trimmed at the curve where they
/// cross and sewn along that crease. Nothing is shrunk to avoid it, because there is nothing to
/// avoid: a chain edge that builds alone builds in company, which is what "Every Chamfer Works"
/// means.
///
/// Two things have to hold before it can be sewn. The run-out's touch row must really REACH the
/// strip - past the band's foot line and before the face itself ends the run-out - and the crossing
/// must stand inside the band's own WINDOW: the stretch of the band's edge the skirt lies along,
/// widened by the mitre overhang the band's material really carries past each end of its edge
/// ([`SurfaceRoomCut::overhang_before_from`]). This says only that the two meet and how far into
/// each other they reach; the curve itself, and the trimming and sewing along it, is the builder's
/// work.
#[derive(Debug, Clone, Copy, PartialEq, Default)]
pub struct SurfaceRoomCrossing {
    /// The region both footprints land on (`SurfaceBevelPlan::key_of`).
    pub region_key: i32,
    /// The band being crossed: its plan, its edge's span and the side it takes off that edge.
    pub band: SurfaceRoomCut,
    /// The run-out's skirt doing the crossing, from the corner it runs out of.
    pub skirt: SurfaceRoomCut,
    /// How far in front of the band's edge line the corner the skirt runs from stands.
    pub gap: f64,
    /// How far the two reach into each other at this size: `size * (k_band + k_skirt) - gap`, which
    /// is exactly the amount the old pair rule would have refused the size over.
    pub overlap: f64,
    /// Whether the band's own END NOTCH stands under this skirt as well (`false` unless set). One
    /// band's removal is its strip on the face it stands on AND the section its end takes out of
    /// the face beyond (`SurfaceRoomEndCut`), so a run-out crossing that band walks off its strip
    /// and onto that section rather than meeting a second footprint there - and the two are sewn as
    /// ONE crossing.
    pub also_takes_the_cap_notch: bool,
}

impl SurfaceRoomCrossing {
    /// How opposed the band's and the skirt's inward directions have to stand to face each
    /// other, as a cosine.
    pub const OPPOSING_COSINE: f64 = -0.999;

    /// The share of a length (floored at one unit) a gap has to clear to count.
    pub const RELATIVE_TOLERANCE: f64 = 1e-7;

    /// Which plan of the pass the skirt belongs to.
    pub fn skirt_plan_index(&self) -> i32 {
        self.skirt.plan_index
    }

    /// Which mixed corner of that plan laid the skirt.
    pub fn skirt_corner_key(&self) -> i32 {
        self.skirt.corner_key
    }

    /// Which plan of the pass the band belongs to.
    pub fn band_plan_index(&self) -> i32 {
        self.band.plan_index
    }

    /// Which edge of that plan the band was taken off.
    pub fn band_edge_index(&self) -> i32 {
        self.band.edge_index
    }

    /// Which side of that edge the band takes.
    pub fn band_side(&self) -> i32 {
        self.band.side
    }

    /// Whether the skirt is the bite the run-out takes out of the WALL it turns against.
    pub fn stands_on_the_wall(&self) -> bool {
        self.skirt.the_wall_bite
    }

    /// Whether any of `crossings` (`None` has none) is a wall bite of corner `corner_key` of plan
    /// `plan`.
    pub fn any_stands_on_the_wall(
        crossings: Option<&[SurfaceRoomCrossing]>,
        plan: i32,
        corner_key: i32,
    ) -> bool {
        crossings.is_some_and(|crossings| {
            crossings.iter().any(|crossing| {
                crossing.skirt_plan_index() == plan
                    && crossing.skirt_corner_key() == corner_key
                    && crossing.stands_on_the_wall()
            })
        })
    }

    /// Every crossing among `cuts` (`None` has none) at `size`, each pair asked once in order; a
    /// crossing whose band's own end notch is among the cuts also takes the cap notch. Polls
    /// `cancellation_token` before each pair, leaving with [`WorkError::Cancelled`] once a cancel
    /// is asked for.
    pub fn gather(
        cuts: Option<&[SurfaceRoomCut]>,
        size: f64,
        cancellation_token: &CancelToken,
    ) -> Result<Vec<SurfaceRoomCrossing>, WorkError> {
        let mut crossings = Vec::new();

        let Some(cuts) = cuts else {
            return Ok(crossings);
        };

        for here in 0..cuts.len() {
            for other in (here + 1)..cuts.len() {
                if cancellation_token.is_cancelled() {
                    return Err(WorkError::Cancelled);
                }

                if let Some(crossing) = Self::try_cross(&cuts[here], &cuts[other], size) {
                    crossings.push(crossing);
                }
            }
        }

        for crossing in &mut crossings {
            if cuts
                .iter()
                .any(|cut| Self::it_is_the_bands_notch(crossing, cut))
            {
                crossing.also_takes_the_cap_notch = true;
            }
        }

        Ok(crossings)
    }

    /// Whether the pair `here`/`other` (a run-out skirt and a band, either way round) is one of
    /// `crossings` - sewn rather than refused: the same skirt crossing that band on its region,
    /// or that band's own end notch.
    pub fn it_is_sewn(
        crossings: Option<&[SurfaceRoomCrossing]>,
        here: &SurfaceRoomCut,
        other: &SurfaceRoomCut,
    ) -> bool {
        let Some(crossings) = crossings else {
            return false;
        };

        let skirt_is_here = here.shape == SurfaceRoomFootprint::RunOutSkirt;
        let (skirt, band) = if skirt_is_here {
            (here, other)
        } else {
            (other, here)
        };

        if skirt.shape != SurfaceRoomFootprint::RunOutSkirt {
            return false;
        }

        crossings.iter().any(|crossing| {
            Self::it_is_the_same_skirt(crossing, skirt)
                && ((band.shape == SurfaceRoomFootprint::Band
                    && skirt.region_key == crossing.region_key
                    && crossing.region_key == band.region_key
                    && crossing.band_plan_index() == band.plan_index
                    && crossing.band_edge_index() == band.edge_index
                    && crossing.band_side() == band.side)
                    || Self::it_is_the_bands_notch(crossing, band))
        })
    }

    /// Whether `cut` is the end notch of the crossing's band at the end the crossing runs off:
    /// past either end of the band's edge, the skirt stands at that end.
    pub fn it_is_the_bands_notch(crossing: &SurfaceRoomCrossing, cut: &SurfaceRoomCut) -> bool {
        if cut.shape != SurfaceRoomFootprint::EndNotch
            || cut.plan_index != crossing.band_plan_index()
            || cut.edge_index != crossing.band_edge_index()
        {
            return false;
        }

        let band = &crossing.band;

        // Where the skirt stands along the band's own edge, clipped to nothing: past either end
        // it is that end, which is what a crossing running off the end of the strip is.
        let along = (crossing.skirt.from - band.from).dot(band.along);

        let (it_runs_off, the_other) = if along * 2.0 < band.length {
            (band.from, band.to)
        } else {
            (band.to, band.from)
        };

        (cut.from - it_runs_off).length_squared() < (cut.from - the_other).length_squared()
    }

    /// Whether `skirt` is the crossing's own skirt: the same plan and the same corner.
    pub fn it_is_the_same_skirt(crossing: &SurfaceRoomCrossing, skirt: &SurfaceRoomCut) -> bool {
        skirt.plan_index == crossing.skirt_plan_index()
            && skirt.corner_key == crossing.skirt_corner_key()
    }

    /// The crossing of a run-out skirt and a band (either way round) facing each other across one
    /// region at `size`, where the run-out's touch row really reaches the strip, the band leaves
    /// the run-out ground to stand on, and the two do not both fit in the gap. `None` for any
    /// other pair, or where they meet nowhere or fit.
    #[allow(clippy::neg_cmp_op_on_partial_ord)] // `!(x > y)` on purpose: it refuses a NaN as well.
    pub fn try_cross(
        here: &SurfaceRoomCut,
        other: &SurfaceRoomCut,
        size: f64,
    ) -> Option<SurfaceRoomCrossing> {
        if !(size > 0.0) || size.is_infinite() || here.region_key != other.region_key {
            return None;
        }

        let skirt_is_here = here.shape == SurfaceRoomFootprint::RunOutSkirt;
        let (skirt, band) = if skirt_is_here {
            (here, other)
        } else {
            (other, here)
        };

        // One of each, and nothing else.
        if skirt.shape != SurfaceRoomFootprint::RunOutSkirt
            || band.shape != SurfaceRoomFootprint::Band
            || !(band.length > 0.0)
            || !(skirt.length > 0.0)
            || band.inward.dot(skirt.inward) > Self::OPPOSING_COSINE
        {
            return None;
        }
        let gap = Self::try_gap_across_the_window(band, skirt, size)?;

        // Whether the run-out's TOUCH ROW really reaches the strip, read in the band's own frame
        // and cut off where the FACE ends: what decides the matter is the strip's near rail, the
        // band's foot line.
        let across_to_the_band = -skirt.inward.dot(band.inward);
        let reach = skirt.setback_per_size * size * across_to_the_band;
        let usable_reach = ieee_math::min_f64(reach, gap);
        let the_bands_foot_line = gap - (band.setback_per_size * size);

        if !(usable_reach
            > the_bands_foot_line + (ieee_math::max_f64(gap, 1.0) * Self::RELATIVE_TOLERANCE))
        {
            // The row that gets furthest dies before the strip begins, so the two footprints
            // never meet on this region and there is nothing here to sew.
            return None;
        }

        // A band that sets back past the corner the skirt runs OUT of has swallowed the place the
        // run-out stands on: nothing to sew there.
        if !(band.setback_per_size * size < gap) {
            return None;
        }

        let overlap = (size * (band.setback_per_size + skirt.setback_per_size)) - gap;

        if !(overlap > 0.0) {
            // They both fit in the gap at this size: the pair rule reads it as the room it is.
            return None;
        }

        Some(SurfaceRoomCrossing {
            region_key: band.region_key,
            band: *band,
            skirt: *skirt,
            gap,
            overlap,
            also_takes_the_cap_notch: false,
        })
    }

    /// The gap across the region from the band's edge to the skirt's corner line, where the
    /// skirt stands within the band's MATERIAL along its edge (its span widened by the mitre
    /// overhangs at `size`); `None` where it does not, or the gap is no wider than the tolerance.
    /// The nearer end of the skirt is taken, so any lean is charged to the crossing.
    pub fn try_gap_across_the_window(
        band: &SurfaceRoomCut,
        skirt: &SurfaceRoomCut,
        size: f64,
    ) -> Option<f64> {
        let tolerance = ieee_math::max_f64(band.length, 1.0) * Self::RELATIVE_TOLERANCE;

        let opens = -(band.overhang_before_from * size);
        let closes = band.length + (band.overhang_past_to * size);

        let from_the_band = skirt.from - band.from;
        let to_the_band = skirt.to - band.from;

        let along_from = from_the_band.dot(band.along);
        let along_to = to_the_band.dot(band.along);

        if ieee_math::min_f64(along_from, along_to) < opens - tolerance
            || ieee_math::max_f64(along_from, along_to) > closes + tolerance
        {
            return None;
        }

        let near = from_the_band.dot(band.inward);
        let far = to_the_band.dot(band.inward);
        let gap = ieee_math::min_f64(near, far);

        (gap > tolerance).then_some(gap)
    }
}
