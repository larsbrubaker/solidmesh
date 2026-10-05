//! What is left of a run-out's column at each of its turns ([`SurfaceRunOutColumnWindow`]): the
//! WINDOW between the cut that eats the rows from the cap inward and the cut that eats them from
//! the wall outward. Ports the fields of `Bevel/Surface/SurfaceRunOutColumnWindow.cs` and the
//! carry of a band's staircase onto the column ends; its other readings and the builds of the
//! window land with later packages as further `impl` blocks.

use vector_math::Vector3;

use super::{SurfaceMixedCorner, SurfaceRunOutMeetsAWallBand};

/// C# `SurfaceRunOutColumnWindow`: the window each run-out column is drawn in.
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
    /// Per row, the turn past the bead that row leaves the material at under the upper cut
    /// (C# `WhereTheRowCrosses`).
    pub crosses_at: Vec<f64>,
    /// Per row, whether the WALL had already taken that row by the turn it crosses at: such a row
    /// leaves material the part does not carry there, so its crossing is not a place the plane
    /// starts cutting the surface.
    pub the_wall_has_it: Vec<bool>,
    /// Per station, how far out of the bead's axis the lower cut stands there (C# `StandsOffAt`).
    pub stands_off_at: Vec<f64>,
    /// Per station, where the column ends on the lower cut (C# `ScallopAt`).
    pub scallops: Vec<Vector3>,
    /// Per station, whether it is a BAND that ends the column there (C# `TheBandEndsThem`).
    pub the_band_ends_them: Vec<bool>,
    /// Per station, how far out of the bead's axis the band's FAR side stands inside the LENS - the
    /// stretch of turn between the cap exit and the band's own hairpin (C# `TryTheLensTop`).
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
    /// there (`SurfaceMixedCornerBites.TryBiteTheCapFace`) however far the stations run on.
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
