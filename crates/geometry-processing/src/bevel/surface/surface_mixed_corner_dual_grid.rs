//! The grid a LEANING one-crease mixed corner's patch is sewn over
//! ([`SurfaceMixedCornerDualGrid`]). Its record, its reading from the two rings, its grid
//! accessors, and the reading from the two strips
//! ([`SurfaceMixedCornerDualGrid::try_read_strips`]).

use std::rc::Rc;

use solidmesh_math::Vector3;

use super::surface_mixed_corner_patch;
use super::{
    SurfaceMixedCorner, SurfaceMixedCornerDualCanal, SurfaceMixedCornerRowRunsTo,
    SurfaceMixedCornerRows, SurfaceStrip,
};
use crate::bevel::{BevelAdvisory, BevelAdvisoryCode};

/// The grid a LEANING one-crease mixed corner's patch is sewn over - read, not emitted. Two end
/// rings and a canal that runs past both of them: the BEAD's own square section at station zero,
/// the CUT's end ring along the far rail while that strip lasts, and then the run-out, which owns
/// every rail of its own and closes on a single point.
///
/// **The same transpose a two-crease junction's grid is.** The canal turns about the CUT's centre
/// line, so the stations run along the cut's ring and each station's row runs from the ball's
/// contact on the wall out to its contact on the cut's tube (`SurfaceTwoCreaseJunctionRings` lays
/// the two-crease case the same way). What is different is only the far END: here it carries on
/// past the cut altogether until the ball dies on the crease the cap and the wall meet in.
///
/// **So the grid is in two halves and the join is exact.** Up to the cut strip's own end the
/// stations are the cut ring's own rails, read rather than solved, and the last rail of each row
/// is that rail itself. Past it the rows are the canal's own, sampled at the step the cut's ring
/// was already chorded at, because the blend a strip feeds into is not entitled to be finer than
/// the strip. The closing column is ONE point, kept as a station so the patch has a row to fan its
/// last band to.
pub struct SurfaceMixedCornerDualGrid {
    /// The rows the grid's stations are laid with, which it built and owns.
    pub rows: SurfaceMixedCornerRows,
    /// The canal the whole patch stands on.
    pub canal: Rc<SurfaceMixedCornerDualCanal>,
    /// The CUT's end ring, cut on the canal's own leaning plane, running from its rail on the face
    /// the two edges SHARE round to its rail on the cap.
    pub cut_ring: Vec<Vector3>,
    /// The turn each station of the grid stands at, from zero to the turn the blend dies at.
    pub stations: Vec<f64>,
    /// The BEAD's square end ring, from its contact on the wall out to its contact on the cut's
    /// tube.
    pub bead_ring: Vec<Vector3>,
}

impl SurfaceMixedCornerDualGrid {
    /// The CUT's ring read the way the grid's STATIONS run - from its rail on the face the two
    /// edges share round to its rail on the cap - whichever way the strip's own rails go.
    ///
    /// Rail zero of a strip lies on the LEFT of its walk, so which end of the ring stands on the
    /// shared face is the plan's own left and right. Read that way rather than taken from the order
    /// the stations came out in, because the ring and its stations are turned over TOGETHER: a ring
    /// reversed against its stations would sew the patch to the cut backwards.
    pub fn from_the_shared_face<T: Clone>(corner: &SurfaceMixedCorner, ring: &[T]) -> Vec<T> {
        if corner.cut.left_region_key() == corner.shared_region_key {
            ring.to_vec()
        } else {
            Self::reversed(ring)
        }
    }

    /// The BEAD's end ring read the way every ROW runs - the wall's contact first, the cut's tube
    /// last.
    pub fn from_the_wall<T: Clone>(corner: &SurfaceMixedCorner, ring: &[T]) -> Vec<T> {
        if corner.bead.left_region_key() == corner.shared_region_key {
            Self::reversed(ring)
        } else {
            ring.to_vec()
        }
    }

    /// `of` back to front, as a new array.
    pub fn reversed<T: Clone>(of: &[T]) -> Vec<T> {
        of.iter().rev().cloned().collect()
    }
}

impl SurfaceMixedCornerDualGrid {
    /// How near a read station has to come to where it belongs - an angle, so fixed.
    pub const ONE_STATION: f64 = 1e-5;

    /// Builds the grid from its parts; the readings make one.
    fn new(
        canal: Rc<SurfaceMixedCornerDualCanal>,
        cut_ring: Vec<Vector3>,
        stations: Vec<f64>,
        bead_ring: Vec<Vector3>,
        rows: SurfaceMixedCornerRows,
    ) -> Self {
        SurfaceMixedCornerDualGrid {
            rows,
            canal,
            cut_ring,
            stations,
            bead_ring,
        }
    }

    /// How many rails each row carries - the bead's own end ring's.
    pub fn rails_per_row(&self) -> usize {
        self.bead_ring.len()
    }

    /// How many stations the grid has.
    pub fn station_count(&self) -> usize {
        self.stations.len()
    }

    /// How many of the stations the CUT's own ring carries.
    pub fn stations_the_cut_carries(&self) -> usize {
        self.cut_ring.len()
    }

    /// Whether `station` is the last one, where the canal dies to a point.
    pub fn is_the_point(&self, station: usize) -> bool {
        station == self.station_count() - 1
    }

    /// Whether `station` is one the cut's own ring carries.
    pub fn the_cut_carries(&self, station: usize) -> bool {
        station < self.stations_the_cut_carries()
    }

    /// Rail `rail` of the row at `station`.
    pub fn at(&self, rail: usize, station: usize) -> Vector3 {
        self.rows.at(rail, self.stations[station])
    }

    /// Reads the grid from the two strips themselves, cutting each one's end ring where this corner
    /// ends it - the CUT's on the canal's own LEANING plane, because a cut running into this blend
    /// has no square stop of its own out there, and the BEAD's on its own square section, because
    /// the canal's first ball is a ball of the bead and the two meet G1 with nothing between them.
    pub fn try_read_strips(
        canal: Rc<SurfaceMixedCornerDualCanal>,
        cut: &SurfaceStrip,
        bead: &SurfaceStrip,
    ) -> Result<SurfaceMixedCornerDualGrid, BevelAdvisory> {
        let corner = Rc::clone(&canal.corner);
        let (_, cut_ring) = surface_mixed_corner_patch::try_cut_the_ring(
            &corner,
            cut,
            true,
            Some(canal.dual.cut_ring_plane()),
        )?;
        let (_, bead_ring) =
            surface_mixed_corner_patch::try_cut_the_ring(&corner, bead, false, None)?;

        Self::try_read(
            canal,
            Self::from_the_shared_face(&corner, &cut_ring),
            Self::from_the_wall(&corner, &bead_ring),
        )
    }

    /// The grid over `canal` from the cut's ring (read from the shared face) and the bead's ring
    /// (read from the wall). The rows ride the WALL, so the way from the ball's centre to the face
    /// it rides is INTO that wall - a void ball stands outside the material it is cutting - and
    /// how far each row runs is the canal's own business rather than the ball's whole sweep.
    pub fn try_read(
        canal: Rc<SurfaceMixedCornerDualCanal>,
        cut_ring: Vec<Vector3>,
        bead_ring: Vec<Vector3>,
    ) -> Result<SurfaceMixedCornerDualGrid, BevelAdvisory> {
        let stations = Self::try_the_stations_of(&canal, &cut_ring)?;

        let runs_to: SurfaceMixedCornerRowRunsTo = {
            let canal = canal.clone();
            Rc::new(move |turn| canal.row_runs_to(turn))
        };

        let rows = SurfaceMixedCornerRows::try_read(
            canal.dual.spine.clone(),
            canal.towards_the_wall(),
            &bead_ring,
            canal.corner.corner,
            Some(runs_to),
        )?;

        Ok(SurfaceMixedCornerDualGrid::new(
            canal, cut_ring, stations, bead_ring, rows,
        ))
    }

    /// The turn every station stands at: the cut ring's own rails while that strip lasts, and
    /// then as many more as the run-out needs at the step the ring was already chorded at.
    ///
    /// The cut's rails are READ and not solved - each lies on the cut's own tube, so the angle it
    /// makes round the cut's centre line in the canal's frame IS its station. The run-out's are
    /// spread evenly over what is left, at the ring's own MEAN step, so the last station lands
    /// exactly on the turn the blend dies at.
    #[allow(clippy::neg_cmp_op_on_partial_ord)] // `!(x > y)` on purpose: it refuses a NaN as well.
    pub fn try_the_stations_of(
        canal: &SurfaceMixedCornerDualCanal,
        cut_ring: &[Vector3],
    ) -> Result<Vec<f64>, BevelAdvisory> {
        let not_supported = || {
            BevelAdvisory::new(
                BevelAdvisoryCode::SurfaceCornerNotSupported,
                vec![canal.corner.corner.into()],
            )
        };

        if cut_ring.len() < 2 {
            return Err(not_supported());
        }

        let axis = canal.dual.axis;
        let spine = &canal.dual.spine;
        let read: Vec<f64> = cut_ring
            .iter()
            .map(|&rail| {
                let out_of_the_axis = rail - axis.closest_point_to(rail);

                f64::atan2(
                    out_of_the_axis.dot(spine.towards_the_wall_square()),
                    out_of_the_axis.dot(spine.towards_the_shared_face()),
                )
            })
            .collect();

        for rail in 1..read.len() {
            if !(read[rail] > read[rail - 1]) {
                // A ring that doubled back would fold the grid over itself.
                return Err(not_supported());
            }
        }

        if read[0].abs() > Self::ONE_STATION
            || (read[read.len() - 1] - canal.the_cut_strip_ends_at()).abs() > Self::ONE_STATION
        {
            // The ring found the wrong ends: it must span the CUT's own sweep exactly, its two
            // rails standing on the shared face's tangency and on the cap's.
            return Err(not_supported());
        }

        let left = canal.dies_at - canal.the_cut_strip_ends_at();
        let step = canal.the_cut_strip_ends_at() / (cut_ring.len() - 1) as f64;
        // Rounded up to a whole count; `as` saturates on a value too large for an `i32`.
        let more = i32::max(1, (left / step).ceil() as i32);

        let mut all = read;
        for extra in 1..=more {
            all.push(canal.the_cut_strip_ends_at() + (f64::from(extra) * left / f64::from(more)));
        }

        Ok(all)
    }
}
