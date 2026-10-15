//! Where a run-out that crosses BOTH bands of one mitred corner hands over from the one to the
//! other ([`SurfaceCrossingAtTheMitre`]): the single point standing on all three surfaces at once.
//! Here are its fields, its turn readings and the
//! staircases it hands the two trims ([`SurfaceCrossingAtTheMitre::try_the_staircases`]), and the
//! reading off the mitre ring ([`SurfaceCrossingAtTheMitre::try_read`]).

use std::cell::{Cell, RefCell};

use vector_math::{ieee_math, Vector3};

use super::surface_mixed_corner_run_out;
use super::{
    SurfaceMixedCorner, SurfaceMixedCornerWall, SurfaceStrip, SurfaceStripStation,
    SurfaceStripTrimmedAtTheCrossing, SurfaceVertexPool,
};
use crate::bevel::{BevelAdvisory, BevelAdvisoryCode};

/// Where a run-out that crosses BOTH bands of one mitred corner
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
/// hold is their MITRE, the ring `surface_strip_mitre::try_mitre` already cuts rail by rail and
/// checks the two strips agree on to a weld. So the meeting point is not a new point to be arrived
/// at twice and welded: it is a point ON a chord the two strips already share, found once, pooled
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
/// run-out - hence `Rc` at its holders and `Cell`/`RefCell` on the parts filled in later. Those
/// parts start at `-1` (not yet known), which is why `Default` is written out.
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

/// The hand-over's staircases: the first rail under the crossing, and the staircase each trim
/// takes.
impl SurfaceCrossingAtTheMitre {
    /// The first rail of the ring standing under the run-out, walking from
    /// the end that is clear.
    pub fn the_first_rail_under(&self) -> i32 {
        if self.given_up[0] {
            self.chord
        } else {
            self.chord + 1
        }
    }

    /// The two end stations the mitre becomes - each band's own staircase, cut back to its
    /// crossings on the rails the run-out took and standing on the shared ring everywhere else,
    /// with the hand-over carried between the two rails of the chord it splits. `leaving` is the
    /// trim at the end of the edge the chain ARRIVES on, `arriving` the trim at the start of the
    /// one it leaves on (either may be `None`); `Ok` holds their staircases in the same order, and
    /// both trims are marked closed.
    ///
    /// A mitre is ordinarily ONE ring both strips stop on. Here the run-out has taken the rails below
    /// the hand-over out of both, so along those rails the two strips do NOT meet: each stops at its
    /// own crossing with the run-out, and the run-out's own surface stands between them. Above the
    /// hand-over nothing has changed and the ring is the ring, welded into the pool here and shared by
    /// index. The hand-over goes in as the BETWEEN of the one chord it splits, on both strips, rather
    /// than as a tenth rail.
    pub fn try_the_staircases(
        &self,
        pool: &mut SurfaceVertexPool,
        leaving: Option<&SurfaceStripTrimmedAtTheCrossing>,
        arriving: Option<&SurfaceStripTrimmedAtTheCrossing>,
    ) -> Result<(SurfaceStripStation, SurfaceStripStation), BevelAdvisory> {
        let misses = || {
            BevelAdvisory::new(
                BevelAdvisoryCode::SurfaceCrossingMissesTheMitre,
                vec![self.at.into()],
            )
        };

        let (Some(leaving), Some(arriving)) = (leaving, arriving) else {
            return Err(misses());
        };

        let mut on_the_cap = leaving;
        let mut on_the_wall = arriving;

        // Told apart by which strip each trim is ON, so that the mitre loop may hand them over in
        // whichever order that corner's two edges run in.
        if on_the_cap.strip == self.the_wall_band.get()
            && on_the_wall.strip == self.the_cap_band.get()
        {
            (on_the_cap, on_the_wall) = (on_the_wall, on_the_cap);
        }

        if on_the_cap.strip != self.the_cap_band.get()
            || on_the_wall.strip != self.the_wall_band.get()
        {
            return Err(misses());
        }

        // The ring's own vertices FIRST and once, so that the rails both strips still meet on are one
        // vertex and not two a weld apart. The rails the run-out took never reach the ring at all.
        let on_the_ring: Vec<i32> = (0..self.ring.len())
            .map(|rail| {
                if self.given_up[rail] {
                    -1
                } else {
                    pool.add(self.ring[rail])
                }
            })
            .collect();

        // BOTH staircases have to step down to the hand-over on the chord it really splits: the
        // highest cut rail has to be the one just below the hand-over, or the tops interpolated from
        // that rail's vertex are carried along a chord that starts somewhere else - an orphan vertex
        // and a crack the length of a rail.
        let first_under = self.the_first_rail_under();

        for trim in [on_the_cap, on_the_wall] {
            let reached = if self.given_up[0] {
                trim.highest_rail_crossed()
            } else {
                trim.lowest_rail_crossed()
            };

            if !trim.crosses(first_under) || reached != first_under {
                return Err(misses());
            }
        }

        // And the run-out's own chord has to START on that rail's vertex, which is the one the band's
        // staircase steps down from. Asked by IDENTITY rather than by distance.
        let chord_starts_at = self.the_chord_starts_at.get();

        if chord_starts_at >= 0 && Some(chord_starts_at) != on_the_cap.vertex_on(first_under) {
            return Err(misses());
        }

        // The column tops belong to the CAP band's chord and to no other: they stand on the UPPER
        // cut's own curve, which is the cap band's. The wall band's chord carries the hand-over and
        // nothing else.
        let along_the_chord = self.along_the_chord.borrow();
        let cap_station = self.the_staircase(
            on_the_cap,
            &on_the_ring,
            &self.param_on_the_cap,
            along_the_chord.as_deref(),
        );
        let wall_station =
            self.the_staircase(on_the_wall, &on_the_ring, &self.param_on_the_wall, None);

        for station in [&cap_station, &wall_station] {
            if station.rail.iter().any(|&rail| rail < 0) {
                // A rail the run-out took that its own curve did not cut: the strip has no vertex to
                // stop on there, so the staircase has a step with nothing under it.
                return Err(misses());
            }
        }

        leaving.closed_the_end.set(true);
        arriving.closed_the_end.set(true);

        // Handed back in the order they came in, which is the order the mitre loop assigns them in.
        if std::ptr::eq(on_the_cap, leaving) {
            Ok((cap_station, wall_station))
        } else {
            Ok((wall_station, cap_station))
        }
    }

    /// One band's staircase - its own crossing on every rail the run-out took, the
    /// shared ring on the rest, and the hand-over between the two rails of the chord it splits.
    fn the_staircase(
        &self,
        trim: &SurfaceStripTrimmedAtTheCrossing,
        on_the_ring: &[i32],
        on_the_rings_params: &[f64],
        along_the_chord: Option<&[i32]>,
    ) -> SurfaceStripStation {
        let mut rail = vec![0i32; on_the_ring.len()];
        let mut param = vec![0.0; on_the_ring.len()];

        for at in 0..rail.len() {
            if self.given_up[at] {
                rail[at] = trim.vertex_on(at as i32).unwrap_or(-1);
                param[at] = trim.param_on(at as i32).unwrap_or(on_the_rings_params[at]);
                continue;
            }

            rail[at] = on_the_ring[at];
            param[at] = on_the_rings_params[at];
        }

        let mut between = vec![Vec::new(); rail.len().saturating_sub(1)];

        for (chord, slot) in between.iter_mut().enumerate() {
            if chord as i32 != self.chord {
                // Whatever the curve carried part way along THIS chord: a run-out stationed by the
                // other band of the pair ends columns between this band's rails, and those points are
                // the staircase's as much as the run-out's (SurfaceStripTrimmedAtTheCrossing).
                *slot = trim.the_betweens_on_the_chord_from(chord as i32);
                continue;
            }

            // In the order the CHORD runs - from rail Chord to rail Chord + 1 - which is not the order
            // the column tops were read in. Listed the other way round where the run-out took the rails
            // at the HIGH end of the profile, the stitch walks the chord's boundary back over itself:
            // two faces walking one edge the same way and a shell with a hole beside them.
            let mut carrying = Vec::new();
            let vertex = self.vertex.get();

            if self.given_up[0] {
                if let Some(tops) = along_the_chord {
                    carrying.extend_from_slice(tops);
                }

                carrying.push(vertex);
            } else {
                carrying.push(vertex);

                if let Some(tops) = along_the_chord {
                    carrying.extend(tops.iter().rev());
                }
            }

            *slot = carrying;
        }

        SurfaceStripStation::new(rail, param, between)
    }
}

impl SurfaceCrossingAtTheMitre {
    /// A crossing with its later-filled parts not yet known (`-1` for the
    /// bands, the vertex and the chord's start; no run along the chord yet).
    #[allow(clippy::too_many_arguments)]
    pub fn new(
        ring: Vec<Vector3>,
        on_the_cap: Vec<f64>,
        on_the_wall: Vec<f64>,
        given_up: Vec<bool>,
        mitre_normal: Vector3,
        chord: i32,
        along_the_chord: f64,
        at: Vector3,
        turn_past: f64,
    ) -> Self {
        SurfaceCrossingAtTheMitre {
            given_up,
            ring,
            param_on_the_cap: on_the_cap,
            param_on_the_wall: on_the_wall,
            mitre_normal,
            chord,
            how_far_along_the_chord: along_the_chord,
            at,
            turn_past,
            ..Default::default()
        }
    }

    /// Whether the run-out has given rail `rail` of the ring up (it stands under the run-out).
    pub fn is_given_up(&self, rail: usize) -> bool {
        self.given_up[rail]
    }

    /// The rail of the ring the run-out leaves standing clear: the last where it gives up from rail
    /// zero, and rail zero otherwise.
    pub fn the_rail_that_stands_clear(&self) -> usize {
        if self.given_up[0] {
            self.given_up.len() - 1
        } else {
            0
        }
    }
}

impl SurfaceCrossingAtTheMitre {
    /// Reads the hand-over of one run-out across the two bands of one mitred corner -
    /// `corner` the run-out comes out of, `wall` its reading of the wall it turns against,
    /// `on_the_cap` / `on_the_wall` the strips of the band on the cap it travels across and on the
    /// wall it turns against (`None` where there is none), `the_mitre_corner` the part's own corner
    /// the two bands mitre at, `near_enough` the part's ruler.
    #[allow(clippy::neg_cmp_op_on_partial_ord)] // `!(x > 0)` refuses a NaN as well.
    pub fn try_read(
        corner: &SurfaceMixedCorner,
        wall: &SurfaceMixedCornerWall,
        on_the_cap: Option<&SurfaceStrip>,
        on_the_wall: Option<&SurfaceStrip>,
        the_mitre_corner: Vector3,
        near_enough: f64,
    ) -> Result<Self, BevelAdvisory> {
        let misses = |at: Vector3| {
            BevelAdvisory::new(
                BevelAdvisoryCode::SurfaceCrossingMissesTheMitre,
                vec![at.into()],
            )
        };

        let (Some(on_the_cap), Some(on_the_wall)) = (on_the_cap, on_the_wall) else {
            return Err(misses(the_mitre_corner));
        };

        if on_the_cap.rail_count() != on_the_wall.rail_count() || on_the_cap.rail_count() < 2 {
            return Err(misses(the_mitre_corner));
        }

        // The mitre plane. Its NORMAL is named exactly as `surface_strip_mitre::try_mitre` names
        // it, so the ring read here and the ring built there are one curve. Its POINT is the
        // CORNER, and it stays the corner even though an ordinary chain mitre's plane has moved to
        // the crossing of the two beads' tangent lines (`SurfaceMitreSeam`): this is a MIXED
        // corner, where that crossing is a different point and the wrong one - MEASURED: reading it
        // through the seam fails five of the run-out tests.
        let mut mitre_normal = on_the_cap.along() + on_the_wall.along();

        if !(mitre_normal.length_squared() > 0.0) {
            return Err(misses(the_mitre_corner));
        }

        mitre_normal = mitre_normal.get_normal();

        let rails = on_the_cap.rail_count() as usize;
        let mut ring = vec![Vector3::ZERO; rails];
        let mut cap_params = vec![0.0; rails];
        let mut wall_params = vec![0.0; rails];

        for rail in 0..rails {
            let (Some(on_cap), Some(on_wall)) = (
                on_the_cap.try_cross(rail as i32, the_mitre_corner, mitre_normal),
                on_the_wall.try_cross(rail as i32, the_mitre_corner, mitre_normal),
            ) else {
                return Err(misses(the_mitre_corner));
            };
            cap_params[rail] = on_cap;
            wall_params[rail] = on_wall;

            ring[rail] = on_the_cap.point_on(rail as i32, on_cap);

            if (ring[rail] - on_the_wall.point_on(rail as i32, on_wall)).length() > near_enough {
                // The two strips do not meet in one ring, so there is no shared chord to put the
                // hand-over on and nothing said here would be shared by both.
                return Err(misses(ring[rail]));
            }
        }

        // How far the run-out really turns, which bounds the sweep: past this the ball has left the
        // cap altogether and nothing it would reach is the run-out's.
        let Some(meets_the_wall) = wall.try_where_the_rail_meets_the_wall() else {
            return Err(misses(the_mitre_corner));
        };

        let clears = ieee_math::min_f64(
            meets_the_wall,
            corner.turn_radians() + surface_mixed_corner_run_out::clears_the_wall(),
        );

        let given_up: Vec<bool> = ring
            .iter()
            .map(|&at| Self::above_the_arc(corner, at, clears) <= 0.0)
            .collect();

        if given_up[0] && given_up[rails - 1] {
            // Nothing of the mitre stands clear of the run-out, so there is no ring left to hand
            // over on and both bands would be drawn entirely inside material the run-out took.
            return Err(BevelAdvisory::new(
                BevelAdvisoryCode::SurfaceCrossingSwallowsTheMitre,
                vec![ring[0].into()],
            ));
        }

        let mut straddles: Option<usize> = None;

        for chord in 0..rails - 1 {
            if given_up[chord] == given_up[chord + 1] {
                continue;
            }

            if straddles.is_some() {
                // The ring dips under the run-out and comes back, which no single hand-over
                // describes: the two staircases would each have to end twice. Named rather than
                // closed on the first of them.
                return Err(misses(ring[chord]));
            }

            straddles = Some(chord);
        }

        let Some(straddles) = straddles else {
            // Both ends clear and nothing between: the run-out never reaches the mitre at all, and
            // this is not the crossing the room reading said it was.
            return Err(misses(the_mitre_corner));
        };

        // Bisected from the CLEAR end of the straddling chord, so the parameter reported runs from
        // rail `straddles` to `straddles + 1` whichever of the two the ring was under.
        let (mut low, mut high) = if given_up[straddles] {
            (1.0, 0.0)
        } else {
            (0.0, 1.0)
        };
        let chord_from = ring[straddles];
        let chord_step = ring[straddles + 1] - ring[straddles];

        for _ in 0..60 {
            let middle = (low + high) / 2.0;

            if Self::above_the_arc(corner, chord_from + (chord_step * middle), clears) > 0.0 {
                low = middle;
            } else {
                high = middle;
            }
        }

        let along = (low + high) / 2.0;
        let stands_at = chord_from + (chord_step * along);

        // AND IT HAS TO BE ON THE RUN-OUT. A sign change is not only the arc: a chord whose clear
        // end is clear because its own TURN is past the last meridian straddles just as readily,
        // and the point it converges on is then the meridian. What the arc really says is that the
        // point stands a ball's radius off the sweep, so that is what is asked, to the part's ruler.
        let turn_past = Self::turn_past_of(corner, stands_at);

        if ((stands_at - corner.ball_centre_at(corner.turn_radians() + turn_past)).length()
            - corner.size)
            .abs()
            > near_enough
        {
            return Err(misses(stands_at));
        }

        Ok(Self::new(
            ring,
            cap_params,
            wall_params,
            given_up,
            mitre_normal,
            straddles as i32,
            along,
            stands_at,
            turn_past,
        ))
    }
}
