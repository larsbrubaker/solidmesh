//! What is left of a run-out's column at each of its turns ([`SurfaceRunOutColumnWindow`]): the
//! WINDOW between the cut that eats the rows from the cap inward and the cut that eats them from
//! the wall outward: its fields, its constructor and readings, the carry of a band's staircase onto
//! the column ends, and the window's own reading ([`SurfaceRunOutColumnWindow::try_read`]).

use solidmesh_math::Vector3;

use super::{
    surface_mixed_corner_bites, surface_mixed_corner_run_out, surface_mixed_corner_wall_ends,
    surface_run_out_column_ends,
};
use super::{
    SurfaceBandEnd, SurfaceCrossingAtTheMitre, SurfaceMixedCorner, SurfaceMixedCornerWall,
    SurfaceRunOutCarry, SurfaceRunOutMeetsAWallBand, SurfaceRunOutMeetsTheBand, SurfaceVertexPool,
};
use crate::bevel::{BevelAdvisory, BevelAdvisoryCode};

/// The window each run-out column is drawn in.
///
/// A column of the run-out is the rows of the patch's profile still standing in the material at one
/// turn about the bead's axis. Two things can take rows off it, and they take them from opposite
/// ends. The UPPER cut - the plane across a wall that runs out under the blend, or another chain's
/// band lying on the cap the run-out travels over - reaches the OUTER rail first, because that is
/// the row furthest out of the axis, and works inward: what it names is, per row, the turn that row
/// leaves the material at. The LOWER cut - the wall the run-out turns against, or another chain's
/// band standing on that wall - swallows the rows NEAREST the axis first: what it names is, per
/// turn, how far out of the axis it stands there, and every row inside that is gone. So a column is
/// the interval `[first row the upper cut still leaves, last row the lower cut still leaves]`, and
/// the two cuts never need to know about each other except at one number: the SWITCH TURN
/// ([`SurfaceRunOutColumnWindow::the_switch_turn`]), past which the two have met and every column
/// is empty.
///
/// Zero, one or two cuts. With none the window is the whole profile and the switch turn is
/// infinite. With one - which is every case built before the mitred pair - the other end is the
/// wall's own, and the switch turn is where the single cut's FOOT leaves the material.
#[derive(Debug, Clone, PartialEq, Default)]
pub struct SurfaceRunOutColumnWindow {
    /// Per row, the turn past the bead that row leaves the material at under the upper cut.
    pub crosses_at: Vec<f64>,
    /// Per row, whether the WALL had already taken that row by the turn it crosses at: such a row
    /// leaves material the part does not carry there, so its crossing is not a place the plane
    /// starts cutting the surface.
    pub the_wall_has_it: Vec<bool>,
    /// Per station, how far out of the bead's axis the lower cut stands there.
    pub stands_off_at: Vec<f64>,
    /// Per station, where the column ends on the lower cut.
    pub scallops: Vec<Vector3>,
    /// Per station, whether it is a BAND that ends the column there.
    pub the_band_ends_them: Vec<bool>,
    /// Per station, how far out of the bead's axis the band's FAR side stands inside the LENS - the
    /// stretch of turn between the cap exit and the band's own hairpin.
    pub lens_radii: Vec<f64>,
    /// The turns past the bead the columns are drawn at, in order.
    pub stations: Vec<f64>,
    /// The turn past which the two cuts have met and every column is empty, or infinity where
    /// nothing closes them. With one cut this is that cut's FOOT - the turn its own trim point
    /// leaves the material at - which is what closed the columns before there were ever two.
    pub the_switch_turn: f64,
    /// The point the run-out would have died at on the sharp edge had nothing crossed it, in the
    /// pool, or -1 where nothing needs it. The cap between the crossing and there is under the
    /// torus either way, and it is the SECTOR that takes it, so its rim closes on this rather than
    /// on a corner of the band's section a chord away.
    pub the_die_point: i32,
    /// Where the lower cut's far side reaches the touch row and dies on the cap, in the pool, or
    /// -1.
    pub the_cap_exit: i32,
    /// The turn past the bead the cap exit stands at, or infinity where there is none. Past it the
    /// touch row runs over cap the BAND has already taken, so the sector the cap gives up ends
    /// there (`surface_mixed_corner_bites::try_bite_the_cap_face`) however far the stations run on.
    pub the_cap_exits_turn: f64,
}

impl SurfaceRunOutColumnWindow {
    /// The nearest station from `from` stepping by `step` (+1 or -1) that the band ends AND a rail
    /// of the band stands at, or -1 where none does.
    pub fn the_rail_station_before(
        stations: &[f64],
        the_band_ends_them: &[bool],
        meets_a_wall_band: &SurfaceRunOutMeetsAWallBand,
        from: i32,
        step: i32,
    ) -> i32 {
        let mut station = from + step;
        while station >= 0 && (station as usize) < stations.len() {
            if the_band_ends_them[station as usize]
                && meets_a_wall_band.the_rail_standing_at(stations[station as usize]) >= 0
            {
                return station;
            }

            station += step;
        }

        -1
    }

    /// Moves every column end the band ends at, but that stands between two of the band's rails,
    /// onto the CHORD between the rail stations either side of it (`scallops`), and re-reads its
    /// stand-off (`stands_off_at`) with it: the strip is ruled by those chords, so that is where
    /// the band's own end really is. The chord stands outside the curve it subtends, so the
    /// re-read can only ever take rows OFF a column, never add one.
    pub fn onto_the_bands_staircase(
        corner: &SurfaceMixedCorner,
        stations: &[f64],
        the_band_ends_them: &[bool],
        meets_a_wall_band: &SurfaceRunOutMeetsAWallBand,
        scallops: &mut [Vector3],
        stands_off_at: &mut [f64],
    ) {
        for station in 0..stations.len() {
            if !the_band_ends_them[station]
                || meets_a_wall_band.the_rail_standing_at(stations[station]) >= 0
            {
                continue;
            }

            let before = Self::the_rail_station_before(
                stations,
                the_band_ends_them,
                meets_a_wall_band,
                station as i32,
                -1,
            );
            let after = Self::the_rail_station_before(
                stations,
                the_band_ends_them,
                meets_a_wall_band,
                station as i32,
                1,
            );

            if before < 0 || after < 0 || stations[after as usize] <= stations[before as usize] {
                // Off the end of the band's own rails, with no chord between two of them to stand
                // on: left where the band really ends the column, and refused by name at the STRIP
                // if the staircase cannot carry it there. The strip is the authority on where its
                // own vertices may stand, so nothing is named here.
                continue;
            }

            let (before, after) = (before as usize, after as usize);
            let how_far =
                (stations[station] - stations[before]) / (stations[after] - stations[before]);

            scallops[station] = scallops[before] + ((scallops[after] - scallops[before]) * how_far);

            // And the stand-off with it. The end has MOVED, and what admits rows to the column is
            // this number: left on the curve it would admit a row the column's own end already
            // stands outside of - a fin between the two surfaces.
            stands_off_at[station] = corner.axis().distance_to(scallops[station]);
        }
    }
}

impl SurfaceRunOutColumnWindow {
    /// Builds the window from its parts.
    #[allow(clippy::too_many_arguments)]
    pub fn new(
        stations: Vec<f64>,
        crosses_at: Vec<f64>,
        the_wall_has_it: Vec<bool>,
        stands_off_at: Vec<f64>,
        scallops: Vec<Vector3>,
        the_band_ends_them: Vec<bool>,
        lens_radii: Vec<f64>,
        the_switch_turn: f64,
        the_die_point: i32,
        the_cap_exit: i32,
        the_cap_exits_turn: f64,
    ) -> Self {
        SurfaceRunOutColumnWindow {
            crosses_at,
            the_wall_has_it,
            stands_off_at,
            scallops,
            the_band_ends_them,
            lens_radii,
            stations,
            the_switch_turn,
            the_die_point,
            the_cap_exit,
            the_cap_exits_turn,
        }
    }

    /// A turn that needs no settling: `turn` itself.
    pub fn settled(turn: f64) -> Option<f64> {
        Some(turn)
    }

    /// The turn at which `row` leaves the band.
    pub fn where_the_row_crosses(&self, row: usize) -> f64 {
        self.crosses_at[row]
    }

    /// How far out of the axis the column at `station` ends.
    pub fn stands_off_at(&self, station: usize) -> f64 {
        self.stands_off_at[station]
    }

    /// Where the column at `station` ends.
    pub fn scallop_at(&self, station: usize) -> Vector3 {
        self.scallops[station]
    }

    /// Every column's end.
    pub fn scallops(&self) -> &[Vector3] {
        &self.scallops
    }

    /// Whether the band ends the column at `station`.
    pub fn the_band_ends_them(&self, station: usize) -> bool {
        self.the_band_ends_them[station]
    }

    /// The lens's top at `station` - how far out of the axis it stands - where the column has one.
    pub fn try_the_lens_top(&self, station: usize) -> Option<f64> {
        let stands_off = self.lens_radii[station];

        (stands_off > 0.0).then_some(stands_off)
    }
}

impl SurfaceRunOutColumnWindow {
    /// Reads both cuts and the turns to draw them at. The three alternatives are as
    /// `surface_mixed_corner_run_out::try_columns` has them: `ends_on` the plane across a wall that
    /// runs out, `meets_the_band` a band on the cap, `meets_a_wall_band` a band on the wall - and,
    /// where BOTH of the last two stand at one mitred corner, `at_the_mitre`, the turn they meet
    /// at.
    #[allow(clippy::too_many_arguments)]
    pub fn try_read(
        pool: &mut SurfaceVertexPool,
        corner: &SurfaceMixedCorner,
        wall: &SurfaceMixedCornerWall,
        last_column: &[i32],
        step: f64,
        ends_on: Option<SurfaceBandEnd>,
        meets_the_band: Option<&SurfaceRunOutMeetsTheBand>,
        meets_a_wall_band: Option<&SurfaceRunOutMeetsAWallBand>,
        at_the_mitre: Option<&SurfaceCrossingAtTheMitre>,
        carry: &SurfaceRunOutCarry,
    ) -> Result<SurfaceRunOutColumnWindow, BevelAdvisory> {
        let refused =
            |code: BevelAdvisoryCode| BevelAdvisory::new(code, vec![corner.corner.into()]);
        let mut the_die_point = -1;

        // The SNAP rather than the plain ruler: whether two stations are one place is the reading
        // the cut itself makes when it decides whether a crossing is a new vertex. And with the
        // patch's own touch row named: it is the point this turn CARRIES round the cap.
        let touches_the_cap_at = last_column.first().map(|&v| pool.position_of(v));

        let mut stations = surface_mixed_corner_run_out::try_turns_past_the_bead(
            corner,
            wall,
            step,
            pool.snap_near(),
            touches_the_cap_at,
            Some(carry),
        )?;

        // Unless the plane across the wall's END cuts the trim curve PAST that: then that wall is
        // read to the RAIL's end as it always was, and the touch row's hair outside it is owed its
        // own reading.
        if let (Some(ends), None, Some(_), Some(&last)) =
            (ends_on, meets_the_band, touches_the_cap_at, stations.last())
        {
            let would_cross = wall
                .try_where_the_rail_meets_the_wall()
                .and_then(|meets_the_rail| {
                    surface_mixed_corner_wall_ends::try_where_the_foot_crosses(
                        corner,
                        wall,
                        ends,
                        meets_the_rail - corner.turn_radians(),
                    )
                });

            if would_cross.is_some_and(|the_foot_would_cross| the_foot_would_cross > last) {
                stations = surface_mixed_corner_run_out::try_turns_past_the_bead(
                    corner,
                    wall,
                    step,
                    pool.snap_near(),
                    None,
                    Some(carry),
                )?;
            }
        }

        // Where a band stands on the WALL, the run-out's columns end on IT from the turn the two
        // first meet: the stations are that band's own rails from there on.
        let mut the_cap_exit = -1;
        let mut the_cap_exits_turn = f64::INFINITY;

        if let Some(wall_band) = meets_a_wall_band.filter(|_| !stations.is_empty()) {
            match at_the_mitre {
                None => {
                    // The curve never reaches the touch row, so it does not close on the cap and
                    // there is nothing to cut either surface back to.
                    let Some((exits_at, exit)) = wall_band.try_the_cap_exit() else {
                        return Err(refused(BevelAdvisoryCode::SurfaceCrossingLeavesTheBand));
                    };

                    the_cap_exit = pool.add(exit);
                    the_cap_exits_turn = exits_at;

                    // And where the run-out would have died had nothing crossed it - unless the
                    // FACE ends before that reach does, and then it dies on the face's own far
                    // edge.
                    if let Some(mut dies_on_the_edge) =
                        surface_mixed_corner_run_out::try_scallop_point(
                            corner,
                            wall,
                            stations[stations.len() - 1],
                        )
                    {
                        if let Some(ends) = ends_on {
                            if !surface_mixed_corner_wall_ends::stands_in_the_material(
                                ends,
                                dies_on_the_edge,
                            ) {
                                let Some(leaves_at) =
                                    surface_mixed_corner_bites::try_where_the_touch_row_leaves_the_face(
                                        corner, ends, exit,
                                    )
                                else {
                                    return Err(refused(
                                        BevelAdvisoryCode::SurfaceSectorLeavesTheFace,
                                    ));
                                };

                                dies_on_the_edge = leaves_at;
                            }
                        }

                        the_die_point = pool.add(dies_on_the_edge);
                    }
                }
                Some(at_the_mitre) => {
                    // With a band on the CAP as well the two cuts meet first, at the hand-over on
                    // their own mitre, and that is the one corner both curves close on.
                    the_cap_exit = at_the_mitre.vertex.get();
                }
            }

            stations = wall_band.stationed(&stations, pool.snap_near());
        }

        // And where the wall RUNS OUT under the blend, each row's own crossing of the plane across
        // it and the turn the foot on the wall leaves at, stationed on. Never where a band on the
        // WALL ends the columns ALONE (its stations are its own rails).
        let rows = last_column.len();
        let mut crosses_at = vec![0.0; rows];
        let mut the_wall_has_it = vec![false; rows];
        let mut foot_crosses_at = f64::INFINITY;
        let mut no_crossings = true;

        if (ends_on.is_some() || meets_the_band.is_some())
            && (meets_a_wall_band.is_none() || at_the_mitre.is_some())
            && !stations.is_empty()
        {
            let mut past_it_at_the_seed =
                meets_the_band.is_some_and(|band| band.it_has_taken_a_row_already());

            for row in 0..rows {
                if let Some(band) = meets_the_band {
                    crosses_at[row] = band.where_the_row_crosses(row as i32);
                } else if let Some(ends) = ends_on {
                    let seat = pool.position_of(last_column[row]);
                    past_it_at_the_seed |=
                        !surface_mixed_corner_wall_ends::stands_in_the_material(ends, seat);

                    crosses_at[row] =
                        surface_mixed_corner_wall_ends::where_it_crosses(corner, seat, ends);
                }

                // Whether the WALL had already taken that row by the turn it crosses at, read at
                // the row's OWN crossing turn.
                the_wall_has_it[row] = crosses_at[row] == f64::INFINITY
                    || match wall.try_out_of_the_axis(corner.turn_radians() + crosses_at[row]) {
                        None => true,
                        Some((stands_off_there, _)) => {
                            carry.out_of_the_axis_at(crosses_at[row])[row]
                                < stands_off_there + pool.near_enough()
                        }
                    };
            }

            // The SWITCH TURN: with one cut that cut's own foot; with two the turn they MEET at.
            let last = stations[stations.len() - 1];
            let the_foot_leaves = if let Some(at_the_mitre) = at_the_mitre {
                Self::settled(at_the_mitre.turn_past)
            } else if let Some(band) = meets_the_band {
                band.try_where_the_foot_crosses(corner, wall, last)
            } else {
                ends_on.and_then(|ends| {
                    surface_mixed_corner_wall_ends::try_where_the_foot_crosses(
                        corner, wall, ends, last,
                    )
                })
            };

            if let Some(at_the_foot) = the_foot_leaves {
                if past_it_at_the_seed {
                    // The patch itself has already drawn a row past the plane, so the surface
                    // stands over material the part does not carry before this piece begins.
                    return Err(refused(BevelAdvisoryCode::SurfaceCornerNotSupported));
                }

                foot_crosses_at = at_the_foot;
                stations = surface_run_out_column_ends::with_the_crossings_stationed(
                    &stations,
                    &crosses_at,
                    &the_wall_has_it,
                    foot_crosses_at,
                    meets_a_wall_band.map(|band| band.rail_turns_past_the_bead.as_slice()),
                );
                no_crossings = false;
            }
        }

        if no_crossings {
            // The foot never leaves the material (or nothing cuts from above), so every row stays.
            crosses_at.fill(f64::INFINITY);
        }

        // Every station's trim point first: the TRIM CHORD a row is drawn to is named by the
        // station after it as much as by its own.
        let count = stations.len();
        let mut scallops = vec![Vector3::ZERO; count];
        let mut stands_off_at = vec![0.0; count];
        let mut the_band_ends_them = vec![false; count];
        let mut lens_radii = vec![0.0; count];

        for station in 0..count {
            let (Some((stands_off, _)), Some(scallop)) = (
                wall.try_out_of_the_axis(corner.turn_radians() + stations[station]),
                surface_mixed_corner_run_out::try_scallop_point(corner, wall, stations[station]),
            ) else {
                return Err(refused(BevelAdvisoryCode::SurfaceCornerNotSupported));
            };
            stands_off_at[station] = stands_off;
            scallops[station] = scallop;

            // A band on the wall stands in front of the wall from the turn the two first meet, read
            // to the part's own ruler so the meeting turn ends on that rail's own point.
            if let Some(wall_band) = meets_a_wall_band {
                if let Some((the_band_stands_off, on_the_band)) =
                    wall_band.try_stands_off_at(stations[station])
                {
                    if the_band_stands_off + pool.near_enough() >= stands_off_at[station] {
                        stands_off_at[station] = the_band_stands_off;
                        scallops[station] = on_the_band;
                        the_band_ends_them[station] = true;
                    }
                }

                // Past the CAP EXIT the band's far side has come inside the touch row, and that far
                // crossing tops the column.
                if the_band_ends_them[station] && stations[station] > the_cap_exits_turn {
                    if let Some(the_far_side) = wall_band.try_the_far_side_at(stations[station]) {
                        if the_far_side > stands_off_at[station] + pool.near_enough() {
                            lens_radii[station] = the_far_side;
                        }
                    }
                }
            }
        }

        // The ends the band mints part way along one of its own quads are moved onto that quad's
        // chord, so both surfaces carry the same vertex there.
        if let (Some(wall_band), Some(_)) = (meets_a_wall_band, at_the_mitre) {
            Self::onto_the_bands_staircase(
                corner,
                &stations,
                &the_band_ends_them,
                wall_band,
                &mut scallops,
                &mut stands_off_at,
            );
        }

        // The LAST station's is the point the curve dies at - the ball's own touch point on the
        // wall, wherever the ring the turn carries leans - so the cap's sector and the wall's
        // scallop have ONE corner to die at.
        if !last_column.is_empty() && count > 0 {
            let last = count - 1;
            let dies = carry.touches_the_cap_at(stations[last]);

            if let Some((stands_on, facing)) =
                wall.try_plane_at(corner.turn_radians() + stations[last])
            {
                if (dies - stands_on).dot(facing).abs() <= pool.near_enough() {
                    scallops[last] = dies;
                }
            }
        }

        // And where the plane across the wall's END is what the curve dies against, the part's own
        // CORNER there is the same place as the foot whenever the two stand within a SNAP.
        if let (Some(ends), None, None) = (ends_on, meets_the_band, meets_a_wall_band) {
            if count > 0 && foot_crosses_at != f64::INFINITY {
                if let Some(the_face_ends_at_the_corner) =
                    surface_mixed_corner_wall_ends::try_where_the_three_meet(
                        corner.outer_rail_centre,
                        corner.out_of_the_cap_face,
                        corner.corner,
                        corner.out_of_the_wall,
                        ends.point,
                        -ends.inward,
                    )
                {
                    if (the_face_ends_at_the_corner - scallops[count - 1]).length()
                        <= pool.snap_near()
                    {
                        scallops[count - 1] = the_face_ends_at_the_corner;
                    }
                }
            }
        }

        Ok(Self::new(
            stations,
            crosses_at,
            the_wall_has_it,
            stands_off_at,
            scallops,
            the_band_ends_them,
            lens_radii,
            foot_crosses_at,
            the_die_point,
            the_cap_exit,
            the_cap_exits_turn,
        ))
    }
}
