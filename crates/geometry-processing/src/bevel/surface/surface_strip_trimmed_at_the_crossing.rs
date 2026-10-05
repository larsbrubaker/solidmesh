//! A band's strip cut short at the curve a mixed corner's RUN-OUT crosses it in
//! ([`SurfaceStripTrimmedAtTheCrossing`]). Ports
//! `Bevel/Surface/SurfaceStripTrimmedAtTheCrossing.cs`: its fields, its constructor and its rail
//! questions; the reading itself (`TryRead`) lands with a later package as a further `impl` block.

use std::cell::{Cell, RefCell};
use std::rc::Rc;

use super::{SurfaceCrossingAtTheMitre, SurfaceStrip, SurfaceVertexPool};

/// C# `SurfaceStripTrimmedAtTheCrossing`: which rails the crossing stands on, how far along the
/// edge each of them is cut, and the one point on the profile between two rails where the run-out's
/// own foot dies.
///
/// The curve is the run-out's own section (`SurfaceMixedCornerRunOut.TryColumns`): a corner for
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
/// [`SurfaceStripTrimmedAtTheCrossing::the_die_vertex`] is C#'s initializer, which is why `Default`
/// is written out.
#[derive(Debug)]
pub struct SurfaceStripTrimmedAtTheCrossing {
    /// The corners the curve carries BETWEEN the crossings on rails `k` and `k + 1`, entry `k`, in
    /// the chord's own order (C# `TheBetweensOnTheChordFrom`). A run-out stationed by the OTHER
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

/// C#'s `SurfaceStripTrimmedAtTheCrossing[,]`: per strip, the trim at its start (`[0]`) and at its
/// end (`[1]`), `None` where that end is not crossed. C#'s null table is `None` at the call sites.
pub type SurfaceStripTrims = Vec<[Option<Rc<SurfaceStripTrimmedAtTheCrossing>>; 2]>;

/// The trim's constructor and its rail questions (C# `<init>`, the private constructor,
/// `TheBetweensOnTheChordFrom`, `TheBandIsAlreadyCrossed`, `StandingAt`, `Crosses`, `VertexOn`,
/// `ParamOn`, `LowestRailCrossed`, `HighestRailCrossed`, `StandsOnTheChord`, `TryTheRailUnder`).
impl SurfaceStripTrimmedAtTheCrossing {
    /// C#'s private constructor, with the field initializers (`<init>`: not closed, no mitre, the
    /// foot not on the rim, no die vertex).
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

    /// C# `TheBetweensOnTheChordFrom`: the corners the curve carries BETWEEN the crossings on rails
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

    /// C# `TheBandIsAlreadyCrossed`: whether `already` has claimed the same END of `strip` before
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

    /// C# `StandingAt`: the trim standing at one end of one strip, or `None` where that end is not
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

    /// C# `Crosses`: whether `rail` is cut back by the crossing rather than by the end.
    pub fn crosses(&self, rail: i32) -> bool {
        self.rail_of.contains(&rail)
    }

    /// C# `VertexOn`: the crossing corner standing on `rail`, or `None` for a rail the crossing does
    /// not reach (where C#'s `At[Array.IndexOf(..)]` would throw; callers ask [`Self::crosses`]
    /// first).
    pub fn vertex_on(&self, rail: i32) -> Option<i32> {
        self.index_of(rail).map(|index| self.at[index])
    }

    /// C# `ParamOn`: how far along the edge the crossing corner on `rail` stands, or `None` as for
    /// [`Self::vertex_on`].
    pub fn param_on(&self, rail: i32) -> Option<f64> {
        self.index_of(rail).map(|index| self.param_of[index])
    }

    /// C# `LowestRailCrossed`: the lowest rail the crossing reaches - the one the FOOT is carried just
    /// under - so that the rails beneath it close as they always did.
    pub fn lowest_rail_crossed(&self) -> i32 {
        self.rail_of.iter().copied().fold(self.rail_of[0], i32::min)
    }

    /// C# `HighestRailCrossed`: the highest rail the crossing reaches.
    pub fn highest_rail_crossed(&self) -> i32 {
        self.rail_of.iter().copied().fold(self.rail_of[0], i32::max)
    }

    /// C# `Array.IndexOf(RailOf, rail)`, `None` for C#'s -1.
    fn index_of(&self, rail: i32) -> Option<usize> {
        self.rail_of.iter().position(|&on| on == rail)
    }

    /// C# `StandsOnTheChord`: whether `carried` stands on the segment from `from` to `to`, inside
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

    /// C# `TryTheRailUnder`: which rail of the strip `vertex` stands on, and how far along the edge -
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
