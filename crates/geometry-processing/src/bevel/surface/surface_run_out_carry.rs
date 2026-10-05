//! How the run-out carries the patch's own last column PAST the end of the bead's rim
//! ([`SurfaceRunOutCarry`]). Ports the record of `Bevel/Surface/SurfaceRunOutCarry.cs` with its
//! constructor, counts, its reading off the patch's last column ([`SurfaceRunOutCarry::of`]) and
//! the carry itself ([`SurfaceRunOutCarry::at`] and the readings taken through it).

use std::borrow::Cow;
use std::rc::Rc;

use vector_math::Vector3;

use super::{SurfaceMixedCorner, SurfaceMixedCornerRows, SurfaceVertexPool};

/// C# `SurfaceRunOutCarry` (a sealed class): how the run-out carries the patch's own last column
/// past the end of the bead's rim - along the SPINE where the corner leans and rides the cap, and
/// rigidly about the bead's axis where it does not.
///
/// **One question asked in four places.** The run-out is more stations of the same turn, so what
/// it draws at each of them is the patch's last column carried there - where a row STANDS, and how
/// far out of the axis it stands there, which is what decides whether the wall has swallowed it.
/// Carried rigidly those radii are the same at every station; carried on the spine the ring
/// SHRINKS along the turn ([`SurfaceMixedCornerRows`]) and every station has its own.
///
/// **And the rigid carry is kept exactly as it was.** Told no rows - every corner that does not
/// lean, and the MITRE handover, whose grid is scheduled rather than laid on the spine - it is the
/// corner's own rigid turn and the one array of radii, bit for bit as before.
pub struct SurfaceRunOutCarry {
    /// The corner the run-out belongs to.
    pub corner: Rc<SurfaceMixedCorner>,
    /// The rows the patch laid on the spine, or `None` where the column is turned rigidly. Shared
    /// with the patch that laid them.
    pub rows: Option<Rc<SurfaceMixedCornerRows>>,
    /// Where each row of the patch's last column stands - the seat the rigid turn carries.
    pub seats: Vec<Vector3>,
    /// Those rows' radii out of the axis, which a rigid turn does not change.
    pub where_they_stand: Vec<f64>,
}

impl SurfaceRunOutCarry {
    /// C#'s private constructor.
    pub fn new(
        corner: Rc<SurfaceMixedCorner>,
        rows: Option<Rc<SurfaceMixedCornerRows>>,
        seats: Vec<Vector3>,
        where_they_stand: Vec<f64>,
    ) -> Self {
        Self {
            corner,
            rows,
            seats,
            where_they_stand,
        }
    }

    /// How many rows the carry lays: one per seat.
    pub fn rows(&self) -> i32 {
        self.seats.len() as i32
    }

    /// True when the carry runs along the corner's spine (it has the spine's rows to follow).
    pub fn on_the_spine(&self) -> bool {
        self.rows.is_some()
    }

    /// C# `Of`: reads the carry off the patch's own last column. `rows` are the rows the patch's
    /// grid was laid with, or `None` where it was scheduled instead - and dropped as well wherever
    /// the corner's bead ring does not lean, so that nothing that builds without a lean is carried
    /// by anything but the rigid turn.
    pub fn of(
        corner: Rc<SurfaceMixedCorner>,
        rows: Option<Rc<SurfaceMixedCornerRows>>,
        pool: &SurfaceVertexPool,
        last_column: &[i32],
    ) -> Self {
        let mut seats = Vec::with_capacity(last_column.len());
        let mut where_they_stand = Vec::with_capacity(last_column.len());

        for &vertex in last_column {
            let seat = pool.position_of(vertex);
            seats.push(seat);
            where_they_stand.push(corner.axis().distance_to(seat));
        }

        let rows = if corner.the_beads_ring_leans() {
            rows
        } else {
            None
        };

        Self::new(corner, rows, seats, where_they_stand)
    }

    /// C# `At`: where row `row` stands at `past` turns past the bead.
    pub fn at(&self, row: usize, past: f64) -> Vector3 {
        match &self.rows {
            Some(rows) => rows.at(row, self.corner.turn_radians() + past),
            None => self.corner.turned_about(self.seats[row], past),
        }
    }

    /// C# `TouchesTheCapAt`: where the ball TOUCHES THE CAP there - row zero, which is the curve the
    /// cap's own sector is drawn to and the one that has to die on the wall.
    pub fn touches_the_cap_at(&self, past: f64) -> Vector3 {
        if self.seats.is_empty() {
            self.corner
                .outer_rail_point(self.corner.turn_radians() + past)
        } else {
            self.at(0, past)
        }
    }

    /// C# `OutOfTheAxisAt`: how far out of the bead's axis every row stands at `past` - the reading
    /// that says which of them the wall has swallowed. The one array over again (borrowed) wherever
    /// the carry is rigid, so that reading costs nothing it did not cost before.
    pub fn out_of_the_axis_at(&self, past: f64) -> Cow<'_, [f64]> {
        if self.rows.is_none() {
            return Cow::Borrowed(&self.where_they_stand);
        }

        let axis = self.corner.axis();

        Cow::Owned(
            (0..self.seats.len())
                .map(|row| axis.distance_to(self.at(row, past)))
                .collect(),
        )
    }

    /// C# `TheTouchRowStandsOffAt`: how far out of the axis the row the ball TOUCHES THE CAP with
    /// stands there - what the wall has to reach for the column to have died. The ideal rail's own
    /// `2r` where nothing leans, which is what that reading was written as.
    pub fn the_touch_row_stands_off_at(&self, past: f64) -> f64 {
        if self.rows.is_some() && !self.seats.is_empty() {
            self.corner.axis().distance_to(self.at(0, past))
        } else {
            self.corner.outer_rail_radius()
        }
    }
}
