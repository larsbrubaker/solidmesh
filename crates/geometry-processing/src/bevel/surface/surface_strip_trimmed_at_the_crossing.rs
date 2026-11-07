//! A band's strip cut short at the curve a mixed corner's RUN-OUT crosses it in
//! ([`SurfaceStripTrimmedAtTheCrossing`]): its fields, its constructor and its rail questions, and
//! the reading itself ([`SurfaceStripTrimmedAtTheCrossing::try_read`]).

use std::cell::{Cell, RefCell};
use std::rc::Rc;

use super::{SurfaceCrossingAtTheMitre, SurfaceEdgePlan, SurfaceStrip, SurfaceVertexPool};
use crate::bevel::{BevelAdvisory, BevelAdvisoryCode};

/// Which rails the crossing stands on, how far along the
/// edge each of them is cut, and the one point on the profile between two rails where the run-out's
/// own foot dies.
///
/// The curve is the run-out's own section (`surface_mixed_corner_run_out::try_columns`): a corner for
/// every row of the torus that left the band's material, in row order from the cap face inward, and
/// then the foot. Each row corner stands ON the rail its row was paired with - the two carry the
/// same offset at the same height, which is what makes the crossing one shared vertex rather than
/// two points a tolerance apart - and the foot stands between two rails, on the chord of the
/// profile the band's own end cut left in the wall.
///
/// So the strip's end ring becomes a STAIRCASE: the crossed rails cut back to their own crossings,
/// the foot carried between the last of them and the rail below it, and every rail under that
/// standing where the end cut always put it. Which rail a crossing belongs to is asked of WHERE IT
/// STANDS and never of its number.
///
/// The builder and the strip ends settle its `set` properties after it is read, through the handles
/// the crossing's holders share, so those are `Cell`/`RefCell`. The `-1` default of
/// [`SurfaceStripTrimmedAtTheCrossing::the_die_vertex`] means "no die vertex yet", which is why
/// `Default` is written out.
#[derive(Debug)]
pub struct SurfaceStripTrimmedAtTheCrossing {
    /// The corners the curve carries BETWEEN the crossings on rails `k` and `k + 1`, entry `k`, in
    /// the chord's own order (`the_betweens_on_the_chord_from`). A run-out stationed by the OTHER
    /// band of a mitred pair ends columns at turns that are not this band's rails', and the points
    /// it leaves there stand on the chord between two rail crossings rather than on a rail of their
    /// own; both surfaces walk that chord, so the staircase carries them as its between points.
    pub between_before: Vec<Vec<i32>>,
    /// Which strip is cut short, under the build's own numbering of the edges.
    pub strip: i32,
    /// Which end of it the crossing stands at.
    pub at_the_start: bool,
    /// The rail each crossing corner stands on, in the order the curve runs.
    pub rail_of: Vec<i32>,
    /// How far along the edge each of those corners stands.
    pub param_of: Vec<f64>,
    /// The crossing corners themselves, as vertices the run-out already made.
    pub at: Vec<i32>,
    /// The run-out's own foot, which stands between two rails rather than on one.
    pub foot: i32,
    /// How far along the edge the foot stands.
    pub foot_param: f64,
    /// Whether the end this crossing stands at was really CLOSED on it. An end that mitres into the
    /// next edge of a closed rim, or that meets a junction, has no end cut to close on the curve -
    /// its ring belongs to the corner beside it - and a crossing left unclosed is refused by name
    /// rather than having its seam cut back under a ring that still reaches the corner
    /// (`SurfaceCrossingMeetsAMitredBand`).
    pub closed_the_end: Cell<bool>,
    /// The hand-over this end mitres on, where it does: the one point the two bands of a closed rim
    /// and the run-out that crosses both of them all share. `Some` says the end is closed by the
    /// MITRE rather than by an end cut, and the staircase is built against the mitre's own ring
    /// ([`SurfaceCrossingAtTheMitre`]).
    pub at_the_mitre: RefCell<Option<Rc<SurfaceCrossingAtTheMitre>>>,
    /// Whether the FOOT stands on the rim of the section this end gives up rather than inside it,
    /// and so has to be a corner of that rim.
    ///
    /// It does where the band was crossed on the WALL the corner turns against: the face beyond
    /// that band's end is then the CAP the run-out travels across, the curve reaches it exactly at
    /// the end plane, and the section's rim runs through the foot rather than past it. A rim that
    /// does not carry the point mints its own where the two cross instead - a pair of them a
    /// hundredth either side of the rail beside it, which nothing else in the part ever visits.
    pub the_foot_is_on_the_rim: Cell<bool>,
    /// The point the run-out would have DIED at had nothing crossed it - on the sharp edge between
    /// the two faces - or -1. The sector that replaced the run-out closes its own rim there, so the
    /// section this end gives up has to carry the same point on its wall-tangent edge: the two rims
    /// are collinear from there to the corner, and a rim that did not share the vertex would be
    /// clipped against a point a chord away from its own.
    pub the_die_vertex: Cell<i32>,
}

impl Default for SurfaceStripTrimmedAtTheCrossing {
    fn default() -> Self {
        SurfaceStripTrimmedAtTheCrossing {
            between_before: Vec::new(),
            strip: 0,
            at_the_start: false,
            rail_of: Vec::new(),
            param_of: Vec::new(),
            at: Vec::new(),
            foot: 0,
            foot_param: 0.0,
            closed_the_end: Cell::new(false),
            at_the_mitre: RefCell::new(None),
            the_foot_is_on_the_rim: Cell::new(false),
            the_die_vertex: Cell::new(-1),
        }
    }
}

/// Per strip, the trim at its start (`[0]`) and at its end (`[1]`), `None` where that end is not
/// crossed. A build with no crossings at all passes `None` for the whole table at the call sites.
pub type SurfaceStripTrims = Vec<[Option<Rc<SurfaceStripTrimmedAtTheCrossing>>; 2]>;

/// The trim's constructor and its rail questions (`the_betweens_on_the_chord_from`,
/// `the_band_is_already_crossed`, `standing_at`, `crosses`, `vertex_on`, `param_on`,
/// `lowest_rail_crossed`, `highest_rail_crossed`, `stands_on_the_chord`, `try_the_rail_under`).
impl SurfaceStripTrimmedAtTheCrossing {
    /// A trim from its readings, starting not closed, with no mitre, the foot not on the rim and no
    /// die vertex: the builder and the strip ends settle those later.
    #[allow(clippy::too_many_arguments)]
    pub fn new(
        strip: i32,
        at_the_start: bool,
        rail_of: Vec<i32>,
        param_of: Vec<f64>,
        at: Vec<i32>,
        between_before: Vec<Vec<i32>>,
        foot: i32,
        foot_param: f64,
    ) -> Self {
        Self {
            between_before,
            strip,
            at_the_start,
            rail_of,
            param_of,
            at,
            foot,
            foot_param,
            ..Self::default()
        }
    }

    /// The corners the curve carries BETWEEN the crossings on rails
    /// `rail` and `rail + 1`, in the chord's own order, or empty where it carries none.
    ///
    /// A run-out stationed by the OTHER band of a mitred pair ends columns at turns that are not this
    /// band's rails', and the points it leaves there stand on the chord between two rail crossings
    /// rather than on a rail of their own. They belong to the strip's staircase as much as to the
    /// run-out - both surfaces walk that chord - so the staircase carries them as the between points
    /// of that chord and the two meet along one line.
    pub fn the_betweens_on_the_chord_from(&self, rail: i32) -> Vec<i32> {
        for corner in 1..self.rail_of.len() {
            if self.between_before[corner].is_empty()
                || self.rail_of[corner - 1].min(self.rail_of[corner]) != rail
            {
                continue;
            }

            if self.rail_of[corner - 1] < self.rail_of[corner] {
                return self.between_before[corner].clone();
            }

            // Read the other way round: the curve ran from the higher rail to the lower, and the chord
            // the staircase walks always runs from rail to rail + 1.
            let mut backwards = self.between_before[corner].clone();

            backwards.reverse();

            return backwards;
        }

        Vec::new()
    }

    /// Whether `already` has claimed the same END of `strip` before
    /// this crossing did. One end of a band cut back at two curves at once is two corners' work and
    /// one staircase's, which is a construction of its own rather than the later crossing quietly
    /// replacing the earlier - but the two ENDS of one band are two staircases and are each other's
    /// business no more than the two ends of any strip are.
    pub fn the_band_is_already_crossed(
        already: Option<&SurfaceStripTrims>,
        strip: i32,
        at_the_start: bool,
    ) -> bool {
        Self::standing_at(already, strip, at_the_start).is_some()
    }

    /// The trim standing at one end of one strip, or `None` where that end is not
    /// crossed.
    pub fn standing_at(
        trims: Option<&SurfaceStripTrims>,
        strip: i32,
        at_the_start: bool,
    ) -> Option<Rc<SurfaceStripTrimmedAtTheCrossing>> {
        let trims = trims?;

        if strip < 0 || strip as usize >= trims.len() {
            return None;
        }

        trims[strip as usize][if at_the_start { 0 } else { 1 }].clone()
    }

    /// Whether `rail` is cut back by the crossing rather than by the end.
    pub fn crosses(&self, rail: i32) -> bool {
        self.rail_of.contains(&rail)
    }

    /// The crossing corner standing on `rail`, or `None` for a rail the crossing does
    /// not reach (callers ask [`Self::crosses`] first).
    pub fn vertex_on(&self, rail: i32) -> Option<i32> {
        self.index_of(rail).map(|index| self.at[index])
    }

    /// How far along the edge the crossing corner on `rail` stands, or `None` as for
    /// [`Self::vertex_on`].
    pub fn param_on(&self, rail: i32) -> Option<f64> {
        self.index_of(rail).map(|index| self.param_of[index])
    }

    /// The lowest rail the crossing reaches - the one the FOOT is carried just
    /// under - so that the rails beneath it close as they always did.
    pub fn lowest_rail_crossed(&self) -> i32 {
        self.rail_of.iter().copied().fold(self.rail_of[0], i32::min)
    }

    /// The highest rail the crossing reaches.
    pub fn highest_rail_crossed(&self) -> i32 {
        self.rail_of.iter().copied().fold(self.rail_of[0], i32::max)
    }

    /// The first index of `rail` in `rail_of`, `None` where it is not there.
    fn index_of(&self, rail: i32) -> Option<usize> {
        self.rail_of.iter().position(|&on| on == rail)
    }

    /// Whether `carried` stands on the segment from `from` to `to`, inside
    /// its span and to the part's own ruler.
    #[allow(clippy::neg_cmp_op_on_partial_ord)]
    pub fn stands_on_the_chord(pool: &SurfaceVertexPool, from: i32, to: i32, carried: i32) -> bool {
        let start = pool.position_of(from);
        let along = pool.position_of(to) - start;
        let reach = along.length_squared();

        if !(reach > 0.0) {
            return false;
        }

        let how_far = (pool.position_of(carried) - start).dot(along) / reach;

        how_far > 0.0
            && how_far < 1.0
            && (start + (along * how_far) - pool.position_of(carried)).length()
                <= pool.near_enough()
    }

    /// Which rail of the strip `vertex` stands on, and how far along the edge -
    /// `None` where it stands on none of them, which is a corner of the curve the strip cannot carry.
    pub fn try_the_rail_under(
        pool: &SurfaceVertexPool,
        strip: &SurfaceStrip,
        vertex: i32,
    ) -> Option<(i32, f64)> {
        let mut rail = -1;

        let at = pool.position_of(vertex);

        let param = strip.param_of(at);

        let mut nearest = f64::MAX;

        for candidate in 0..strip.rail_count() {
            let off = (strip.point_on(candidate, param) - at).length();

            if off < nearest {
                nearest = off;
                rail = candidate;
            }
        }

        (nearest <= pool.near_enough()).then_some((rail, param))
    }
}

impl SurfaceStripTrimmedAtTheCrossing {
    /// The strip `strip_index` cut short where the run-out's `crossing` (its rows,
    /// then its foot) crosses it - each row on the rail it stands on (a row on none is carried onto
    /// the chord between the two rails either side), at the end of the band the crossing stands at
    /// (or at `at_the_mitre`). `Ok(None)` where there is nothing to read; `Err` where the profiles
    /// do not match, or the crossing leaves the band.
    #[allow(clippy::too_many_arguments)]
    pub fn try_read(
        pool: Option<&SurfaceVertexPool>,
        strip: Option<&SurfaceStrip>,
        strip_index: i32,
        edge: &SurfaceEdgePlan,
        crossing: Option<&[i32]>,
        at_the_mitre: Option<Rc<SurfaceCrossingAtTheMitre>>,
    ) -> Result<Option<SurfaceStripTrimmedAtTheCrossing>, BevelAdvisory> {
        let (Some(pool), Some(strip), Some(crossing)) = (pool, strip, crossing) else {
            return Ok(None);
        };
        if crossing.len() < 2 {
            return Ok(None);
        }

        let rows = crossing.len() - 1;
        let mut rail_of = Vec::with_capacity(rows);
        let mut param_of = Vec::with_capacity(rows);
        let mut at = Vec::with_capacity(rows);
        let mut between: Vec<Vec<i32>> = Vec::with_capacity(rows);
        let mut carried = Vec::new();

        for &row in &crossing[..rows] {
            let Some((rail, param)) = Self::try_the_rail_under(pool, strip, row) else {
                carried.push(row);
                continue;
            };

            rail_of.push(rail);
            param_of.push(param);
            at.push(row);
            between.push(std::mem::take(&mut carried));
        }

        if rail_of.is_empty() || !carried.is_empty() || !between[0].is_empty() {
            let at = if carried.is_empty() {
                crossing[0]
            } else {
                carried[0]
            };
            return Err(BevelAdvisory::new(
                BevelAdvisoryCode::SurfaceCrossingProfilesDoNotMatch,
                vec![pool.position_of(at).into()],
            ));
        }

        for corner in 1..rail_of.len() {
            for &carried_on in &between[corner] {
                if (rail_of[corner] - rail_of[corner - 1]).abs() != 1
                    || !Self::stands_on_the_chord(pool, at[corner - 1], at[corner], carried_on)
                {
                    return Err(BevelAdvisory::new(
                        BevelAdvisoryCode::SurfaceCrossingProfilesDoNotMatch,
                        vec![pool.position_of(carried_on).into()],
                    ));
                }
            }
        }

        let foot = crossing[crossing.len() - 1];
        let foot_param = strip.param_of(pool.position_of(foot));

        let length = (edge.to() - edge.from()).dot(strip.along());
        let mut reach = foot_param;
        for &param in &param_of {
            reach = vector_math::ieee_math::max_f64(reach, param);
        }

        let mut nearest = foot_param;
        for &param in &param_of {
            nearest = vector_math::ieee_math::min_f64(nearest, param);
        }

        let at_the_start = nearest < length - reach;
        let near = vector_math::ieee_math::max_f64(pool.near_enough(), 0.0);
        let stands_at_the_end = at_the_mitre.is_some()
            || if at_the_start {
                nearest <= near
            } else {
                reach >= length - near
            };

        if !stands_at_the_end {
            return Err(BevelAdvisory::new(
                BevelAdvisoryCode::SurfaceCrossingLeavesTheBand,
                vec![pool.position_of(foot).into()],
            ));
        }

        let trimmed = SurfaceStripTrimmedAtTheCrossing::new(
            strip_index,
            at_the_start,
            rail_of,
            param_of,
            at,
            between,
            foot,
            foot_param,
        );
        *trimmed.at_the_mitre.borrow_mut() = at_the_mitre;

        Ok(Some(trimmed))
    }
}
