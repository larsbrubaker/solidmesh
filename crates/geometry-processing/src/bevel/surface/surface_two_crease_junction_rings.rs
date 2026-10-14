//! The three end rings a TWO-crease junction's patch is sewn to, and the grid between them
//! ([`SurfaceTwoCreaseJunctionRings`]): the record, its reading from the three rings and its grid
//! accessors, and the reading from the three strips
//! ([`SurfaceTwoCreaseJunctionRings::try_read_strips`]).

use std::rc::Rc;

use vector_math::{cs_math, Vector3};

use crate::bevel::{BevelAdvisory, BevelAdvisoryCode};

use super::{
    surface_mixed_corner_patch, SurfaceJunctionKind, SurfaceMixedCornerDualFrame,
    SurfaceMixedCornerRows, SurfaceMixedJunction, SurfaceMixedJunctionArm, SurfaceStrip,
    SurfaceVertexPool,
};

/// The three end rings a two-crease
/// junction's patch is sewn to, and the grid between them - read, not emitted. What the patch has
/// left to do with this is win its vertices from the pool and wind its quads
/// (`SurfaceTwoCreaseJunction` reads the canal itself).
///
/// **The grid is TRANSPOSED against a one-crease junction's.** Here the canal turns about the
/// CUT's centre line, so the stations run along the cut's ring and each station's row runs from
/// the wall's contact to the cut's tube. The two ends of the turn are the two BEADS' own square end
/// rings, which is what makes this patch close where a two-edge mixed corner's runs out.
///
/// **Everything is REUSED and nothing is re-derived.** Station `k` stands at the turn the cut's
/// ring rail `k` already names - the rail lies on the cut's tube, so the angle it makes round the
/// cut's centre line IS that station's own turn, to 2e-15 on the nose fixture. Column zero is the
/// upper bead's square end ring and the last column is the lower bead's, checked against where the
/// canal puts them rather than trusted.
///
/// **And the two beads must be sampled alike.** The rows pair by index, so two end rings of
/// different lengths have no pairing at all and are refused by name
/// (`BevelAdvisoryCode::SurfaceJunctionBeadsSampledDifferently`).
pub struct SurfaceTwoCreaseJunctionRings {
    /// The rows the grid's stations are laid with, which it built and owns.
    pub rows: SurfaceMixedCornerRows,
    /// The canal the whole patch stands on - the FIRST arm's dual, which is the whole of it.
    pub canal: Rc<SurfaceMixedCornerDualFrame>,
    /// The CUT's end ring, cut on the canal's own leaning plane rather than on a square section,
    /// running from the first arm's tangency round to the second's.
    pub cut_ring: Vec<Vector3>,
    /// The turn each rail of that ring stands at - one station of the grid each.
    pub stations: Vec<f64>,
    /// The FIRST arm's bead's square end ring, from its wall contact round to its tube contact.
    pub first_bead_ring: Vec<Vector3>,
    /// The SECOND arm's, the same way about - the column the grid closes on.
    pub last_bead_ring: Vec<Vector3>,
}

impl SurfaceTwoCreaseJunctionRings {
    /// The cut's ring read from the FIRST arm's tangency round to the second's, whichever way the
    /// strip's own rails go. Rail zero of a strip lies on the LEFT of its walk, so which end of the
    /// ring stands on which arm's shared face is the plan's own left and right - read that way
    /// because the ring and its stations have to be turned over TOGETHER.
    pub fn from_the_first_arms_tangency<T: Clone>(
        junction: &SurfaceMixedJunction,
        ring: &[T],
    ) -> Vec<T> {
        if junction.first.mixed.cut.left_region_key() == junction.first.mixed.shared_region_key {
            ring.to_vec()
        } else {
            Self::reversed(ring)
        }
    }

    /// One bead's end ring read the way every ROW of the grid runs - the wall's contact first, the
    /// cut's tube last. The mirror of `SurfaceMixedCornerPatch`'s `from_the_handover`, which reads
    /// the other way about because a one-crease patch's rows run from the handover to the wall.
    pub fn from_the_wall<T: Clone>(arm: &SurfaceMixedJunctionArm, ring: &[T]) -> Vec<T> {
        if arm.mixed.bead.left_region_key() == arm.mixed.shared_region_key {
            Self::reversed(ring)
        } else {
            ring.to_vec()
        }
    }

    /// How far off the canal the CLOSING column may stand: the shared faces' own chord sagitta,
    /// the same slack a one-crease junction's closing ring is allowed, read for both arms and the
    /// wider taken. The closing column is a ring the BEAD's strip already minted, and that strip
    /// stands on whatever chording the faces it was rolled along have.
    pub fn closing_slack(junction: &SurfaceMixedJunction, pool: &SurfaceVertexPool) -> f64 {
        cs_math::max_f64(
            surface_mixed_corner_patch::the_shared_faces_own_sagitta(&junction.first.mixed, pool),
            surface_mixed_corner_patch::the_shared_faces_own_sagitta(&junction.second.mixed, pool),
        )
    }

    /// `of` back to front, as a new array.
    pub fn reversed<T: Clone>(of: &[T]) -> Vec<T> {
        of.iter().rev().cloned().collect()
    }
}

impl SurfaceTwoCreaseJunctionRings {
    /// How near a read station has to come to the end of the turn it belongs at - an angle, so
    /// fixed.
    pub const ONE_STATION: f64 = 1e-5;

    /// The rings and grid from their readings, as the two readings below make them.
    fn new(
        canal: Rc<SurfaceMixedCornerDualFrame>,
        cut_ring: Vec<Vector3>,
        stations: Vec<f64>,
        first_bead_ring: Vec<Vector3>,
        last_bead_ring: Vec<Vector3>,
        rows: SurfaceMixedCornerRows,
    ) -> Self {
        SurfaceTwoCreaseJunctionRings {
            rows,
            canal,
            cut_ring,
            stations,
            first_bead_ring,
            last_bead_ring,
        }
    }

    /// How many rails each row carries - the first bead's end ring's.
    pub fn rails_per_row(&self) -> usize {
        self.first_bead_ring.len()
    }

    /// How many stations the grid has - one per rail of the cut's ring.
    pub fn station_count(&self) -> usize {
        self.cut_ring.len()
    }

    /// Rail `rail` of the row at `station`.
    pub fn at(&self, rail: usize, station: usize) -> Vector3 {
        self.rows.at(rail, self.stations[station])
    }

    /// Reads the rings and the grid from the one cut's strip and the two arms' beads' strips. The
    /// CUT's ring is
    /// cut on the canal's own plane - both of its ends stand on a tangency, one per arm, because both
    /// of the cut's faces are shared here - and each bead's on its own square section.
    pub fn try_read_strips(
        junction: &SurfaceMixedJunction,
        cut: &SurfaceStrip,
        first_bead: &SurfaceStrip,
        last_bead: &SurfaceStrip,
    ) -> Result<SurfaceTwoCreaseJunctionRings, BevelAdvisory> {
        let not_supported = || {
            BevelAdvisory::new(
                BevelAdvisoryCode::SurfaceCornerNotSupported,
                vec![junction.corner.into()],
            )
        };

        if junction.kind != SurfaceJunctionKind::TwoCreases {
            // The grid turns about the CUT's centre line, which is only the corner's own canal where
            // there are two beads to end it on.
            return Err(not_supported());
        }

        // As in `try_read`: a two-crease junction's arms always carry their dual.
        let Some(canal) = junction.first.dual.as_ref() else {
            return Err(not_supported());
        };

        let (_, cut_ring) = surface_mixed_corner_patch::try_cut_the_ring(
            &junction.first.mixed,
            cut,
            true,
            Some(canal.cut_ring_plane()),
        )?;
        let (_, first_ring) = surface_mixed_corner_patch::try_cut_the_ring(
            &junction.first.mixed,
            first_bead,
            false,
            None,
        )?;
        let (_, last_ring) = surface_mixed_corner_patch::try_cut_the_ring(
            &junction.second.mixed,
            last_bead,
            false,
            None,
        )?;

        Self::try_read(
            junction,
            Self::from_the_first_arms_tangency(junction, &cut_ring),
            Self::from_the_wall(&junction.first, &first_ring),
            Self::from_the_wall(&junction.second, &last_ring),
        )
    }

    /// The grid of a TWO-crease `junction` from the cut's ring (read from the first arm's
    /// tangency) and the two beads' end rings (read from the wall). The rows ride the WALL, so the
    /// way from the ball's centre to the face it rides is INTO that wall - a void ball stands
    /// outside the material it is cutting.
    pub fn try_read(
        junction: &SurfaceMixedJunction,
        cut_ring: Vec<Vector3>,
        first_bead_ring: Vec<Vector3>,
        last_bead_ring: Vec<Vector3>,
    ) -> Result<SurfaceTwoCreaseJunctionRings, BevelAdvisory> {
        let not_supported = || {
            BevelAdvisory::new(
                BevelAdvisoryCode::SurfaceCornerNotSupported,
                vec![junction.corner.into()],
            )
        };

        if junction.kind != SurfaceJunctionKind::TwoCreases {
            return Err(not_supported());
        }

        // A two-crease junction's arms always carry their dual; an arm without one is no two-crease
        // junction, refused in the same words rather than a panic.
        let Some(canal) = junction.first.dual.clone() else {
            return Err(not_supported());
        };

        let stations = Self::try_the_stations_of(junction, &canal, &cut_ring)?;

        if first_bead_ring.len() != last_bead_ring.len() {
            return Err(BevelAdvisory::new(
                BevelAdvisoryCode::SurfaceJunctionBeadsSampledDifferently,
                vec![junction.corner.into()],
            ));
        }

        let rows = SurfaceMixedCornerRows::try_read(
            canal.spine.clone(),
            -canal.out_of_the_wall,
            &first_bead_ring,
            junction.corner,
            None,
        )?;

        Ok(SurfaceTwoCreaseJunctionRings::new(
            canal,
            cut_ring,
            stations,
            first_bead_ring,
            last_bead_ring,
            rows,
        ))
    }

    /// Whether the CLOSING column - the last bead's own ring - really stands where the canal puts
    /// the last station, rail for rail, within `slack`. Returns the verdict, the rail that stands
    /// off furthest (the default point where none does) and by how much.
    pub fn the_closing_column_stands(&self, slack: f64) -> (bool, Vector3, f64) {
        let mut stands_off = Vector3::default();
        let mut by = 0.0;

        for rail in 0..self.rails_per_row() {
            let apart =
                (self.last_bead_ring[rail] - self.at(rail, self.station_count() - 1)).length();

            if apart > by {
                by = apart;
                stands_off = self.last_bead_ring[rail];
            }
        }

        (by <= slack, stands_off, by)
    }

    /// The turn every station stands at: each rail of the cut's ring read round the canal's axis.
    /// They have to march ONE way and span the WHOLE turn: a ring that doubled back would fold the
    /// grid over itself, and one that stopped short would leave the patch's own edge standing off
    /// the bead it hands over to.
    #[allow(clippy::neg_cmp_op_on_partial_ord)] // `!(x > y)`, which refuses a NaN as well.
    pub fn try_the_stations_of(
        junction: &SurfaceMixedJunction,
        canal: &SurfaceMixedCornerDualFrame,
        cut_ring: &[Vector3],
    ) -> Result<Vec<f64>, BevelAdvisory> {
        let not_supported = || {
            BevelAdvisory::new(
                BevelAdvisoryCode::SurfaceCornerNotSupported,
                vec![junction.corner.into()],
            )
        };

        let axis = canal.axis;
        let spine = &canal.spine;
        let stations: Vec<f64> = cut_ring
            .iter()
            .map(|&rail| {
                let out_of_the_axis = rail - axis.closest_point_to(rail);

                f64::atan2(
                    out_of_the_axis.dot(spine.towards_the_wall_square()),
                    out_of_the_axis.dot(spine.towards_the_shared_face()),
                )
            })
            .collect();

        for rail in 1..stations.len() {
            if !(stations[rail] > stations[rail - 1]) {
                return Err(not_supported());
            }
        }

        // An empty ring reads as a ring that found no ends, rather than a panic.
        let (Some(&first), Some(&last)) = (stations.first(), stations.last()) else {
            return Err(not_supported());
        };

        if first.abs() > Self::ONE_STATION || (last - canal.turn_radians).abs() > Self::ONE_STATION
        {
            return Err(not_supported());
        }

        Ok(stations)
    }
}
