//! Where a run-out that crosses BOTH bands of one mitred corner hands over from the one to the
//! other ([`SurfaceCrossingAtTheMitre`]): the single point standing on all three surfaces at once.
//! Ports the fields of `Bevel/Surface/SurfaceCrossingAtTheMitre.cs`; the reading off the mitre ring
//! and the staircases it hands the two trims land with the run-out crossing package.

use std::cell::{Cell, RefCell};

use vector_math::Vector3;

use super::SurfaceMixedCorner;

/// C# `SurfaceCrossingAtTheMitre`: where a run-out that crosses BOTH bands of one mitred corner
/// hands over from the one to the other.
///
/// A closed rim picked as a face puts two bands at every corner of it, and a run-out reaching
/// across the cap can lie over both - the band on the cap it travels across and the band on the
/// wall it turns against. Each of those cuts the run-out's columns from its own end
/// ([`super::SurfaceRunOutColumnWindow`]), and the turn the two meet at is where the window closes
/// and nothing of the column is left.
///
/// **That meeting point is on the mitre ring.** The point the window closes at lies on the cap
/// band's surface and on the wall band's surface at once - and the one curve both of those surfaces
/// hold is their MITRE, the ring `SurfaceStripMitre.TryMitre` already cuts rail by rail and checks
/// the two strips agree on to a weld. So the meeting point is not a new point to be arrived at
/// twice and welded: it is a point ON a chord the two strips already share, found once, pooled
/// once, and handed to both. Asking it of the ring rather than of a chord of the crossing curve is
/// what makes the answer exact rather than a chording error's worth off (measured: 0.31 mm out of
/// 3.76 on the lobed prism, which is a quarter of a chord and nothing a weld would close).
///
/// **What "under the run-out" means.** NOT "inside the swept ball". The ball sits in the AIR,
/// touching the two faces from outside; what the fillet takes away is the material between the ball
/// and the sharp corner, which is the region OUTSIDE it. In the meridian plane at one turn, with
/// `rho` the radius out of the bead's axis and `z` the height above the cap face, the run-out
/// removes everything from the touch row at `(2r, 0)` round to the bead at `(r, r)` that lies BELOW
/// the arc `z = r - sqrt(r^2 - (rho - 2r)^2)`. Reading the ball's BODY instead inverts the answer
/// exactly where it matters, and invents a second crossing out beyond the last meridian.
///
/// **Which end is which is not assumed.** A strip numbers its rails from whichever side of its own
/// edge the plan walked, so ONE end of the ring stands clear of the run-out and the other is under
/// it, and the answer carries which rails are given up rather than a direction to walk in. A ring
/// with both ends clear, or one that dips under and comes back, is
/// `SurfaceCrossingMissesTheMitre`; a ring under the run-out at BOTH ends is
/// `SurfaceCrossingSwallowsTheMitre`.
///
/// Built once and handed to BOTH trims (`SurfaceStripTrimmedAtTheCrossing::at_the_mitre`), and its
/// settable parts are filled in afterwards through those shared handles by the builder and the
/// run-out - hence `Rc` at its holders and `Cell`/`RefCell` on the C# `set` properties. The `-1`
/// defaults are C#'s initializers, which is why `Default` is written out.
#[derive(Debug)]
pub struct SurfaceCrossingAtTheMitre {
    /// Per rail of the ring, whether the run-out has given it up (it stands under the run-out).
    pub given_up: Vec<bool>,
    /// The mitre ring itself, rail by rail, as both strips cut it.
    pub ring: Vec<Vector3>,
    /// How far along the cap band's own edge each rail of the ring stands.
    pub param_on_the_cap: Vec<f64>,
    /// How far along the wall band's own edge each rail of it stands.
    pub param_on_the_wall: Vec<f64>,
    /// The mitre plane's normal, which is the two strips' walk directions summed.
    pub mitre_normal: Vector3,
    /// Which strip is the band on the CAP the run-out travels across, under the build's own
    /// numbering, or -1. The two bands are told apart by this and never by the order the mitre loop
    /// reaches them in: which of a corner's two edges arrives and which leaves is the chain's
    /// business, and the two are not interchangeable here - the cap band is the UPPER cut and the
    /// wall band the lower, and each has its own parameters along its own edge.
    pub the_cap_band: Cell<i32>,
    /// And which is the band standing on the WALL it turns against, or -1.
    pub the_wall_band: Cell<i32>,
    /// Which chord of the ring the shared point splits - the one running from rail `chord` to rail
    /// `chord + 1`. Which SIDE of it is given up depends on which way the strip numbered its rails,
    /// so ask [`SurfaceCrossingAtTheMitre::given_up`] rather than comparing with this.
    pub chord: i32,
    /// How far along that chord it stands, from zero at `chord` to one at the next.
    pub how_far_along_the_chord: f64,
    /// The shared point itself.
    pub at: Vector3,
    /// Its own turn past the bead, which is the window's SWITCH TURN: past it the two cuts have met
    /// and there is nothing of any column left to draw.
    pub turn_past: f64,
    /// The shared point in the pool, set once by the builder and handed to both trims; -1 before.
    pub vertex: Cell<i32>,
    /// Whatever the run-out's own column tops left standing ALONG the chord from the last row the
    /// upper cut took to the hand-over, in the order the turn runs, or `None` before the run-out
    /// sets it. Past that row the column no longer tops on a row of the profile: the two cuts are
    /// closing on each other, and its top stands on the upper cut's curve - which is one straight
    /// chord across the band's planar quad. Those points are the band's rim as much as the
    /// run-out's, so they are carried between the two rails that chord runs between
    /// ([`super::SurfaceStripStation::between`]).
    pub along_the_chord: RefCell<Option<Vec<i32>>>,
    /// The vertex `along_the_chord`'s points were read from - the last row the upper cut took, on
    /// the band it took it from - or -1 where no top was needed. It has to be the band's own
    /// crossing on the rail at the given-up end of `chord`, which is what makes the tops points OF
    /// that chord rather than points near it.
    pub the_chord_starts_at: Cell<i32>,
}

impl Default for SurfaceCrossingAtTheMitre {
    fn default() -> Self {
        SurfaceCrossingAtTheMitre {
            given_up: Vec::new(),
            ring: Vec::new(),
            param_on_the_cap: Vec::new(),
            param_on_the_wall: Vec::new(),
            mitre_normal: Vector3::default(),
            the_cap_band: Cell::new(-1),
            the_wall_band: Cell::new(-1),
            chord: 0,
            how_far_along_the_chord: 0.0,
            at: Vector3::default(),
            turn_past: 0.0,
            vertex: Cell::new(-1),
            along_the_chord: RefCell::new(None),
            the_chord_starts_at: Cell::new(-1),
        }
    }
}

impl SurfaceCrossingAtTheMitre {
    /// How far `at` stands above the arc the cap-riding blend draws across the corner wedge at its
    /// turn, measured square to the cap face - negative under it, `f64::INFINITY` where the point
    /// is not over the corner wedge at all (turned short of the bead's own turn or past `clears`,
    /// out past the touch row, or inside the bead's own barrel). Standing clear is reported as
    /// infinity rather than as a distance, because the callers ask only for the SIGN and a signed
    /// distance to an arc the point is nowhere near would read as a near miss and invite a
    /// tolerance.
    pub fn above_the_arc(corner: &SurfaceMixedCorner, at: Vector3, clears: f64) -> f64 {
        let turn = Self::turn_of(corner, at);

        if turn < corner.turn_radians() || turn > clears {
            return f64::INFINITY;
        }

        let out_of_the_axis = corner.axis().distance_to(at);
        let from_the_touch_row = out_of_the_axis - (2.0 * corner.size);

        if from_the_touch_row > 0.0 || out_of_the_axis < corner.size {
            return f64::INFINITY;
        }

        // Height above the cap face, whose plane passes through the bead's own axis centre: the
        // touch row stands on it by construction, so this is zero there and a radius at the bead.
        let above = (at - corner.outer_rail_centre).dot(-corner.out_of_the_cap_face);

        above
            - (corner.size
                - f64::max(
                    0.0,
                    (corner.size * corner.size) - (from_the_touch_row * from_the_touch_row),
                )
                .sqrt())
    }

    /// A point's own turn about the bead's axis, measured the way the corner measures turns.
    pub fn turn_of(corner: &SurfaceMixedCorner, at: Vector3) -> f64 {
        let out_of = at - corner.axis().closest_point_to(at);

        f64::atan2(
            out_of.dot(corner.out_of_the_axis(std::f64::consts::PI / 2.0)),
            out_of.dot(corner.out_of_the_axis(0.0)),
        )
    }

    /// The same, as a turn PAST the bead's own.
    pub fn turn_past_of(corner: &SurfaceMixedCorner, at: Vector3) -> f64 {
        Self::turn_of(corner, at) - corner.turn_radians()
    }
}
