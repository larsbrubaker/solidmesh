//! A band's strip cut short at the curve a mixed corner's RUN-OUT crosses it in
//! ([`SurfaceStripTrimmedAtTheCrossing`]). Ports the fields of
//! `Bevel/Surface/SurfaceStripTrimmedAtTheCrossing.cs`; the reading and its rail questions land
//! with the run-out crossing package as further `impl` blocks.

use std::cell::{Cell, RefCell};
use std::rc::Rc;

use super::SurfaceCrossingAtTheMitre;

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
