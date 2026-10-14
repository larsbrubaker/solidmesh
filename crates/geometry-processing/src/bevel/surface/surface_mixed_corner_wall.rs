//! The wall nobody picked, as a MIXED CORNER sees it ([`SurfaceMixedCornerWall`], with its
//! [`SurfaceMixedCornerWallFacet`]s): the records and their turn bookkeeping (the constructor, each
//! facet's stretch as a turn about the bead's axis, and where a ray, a circle or the touch curve
//! meets the wall), its reading off a corner ([`SurfaceMixedCornerWall::try_read`]), where the rail
//! meets it, and the strip of it under the blend.

use std::f64::consts::PI;
use std::rc::Rc;

use vector_math::{cs_math, Vector3};

use super::{SurfaceMixedCorner, SurfaceWallSection};
use crate::bevel::{BevelAdvisory, BevelAdvisoryCode};

/// One facet of the wall - where it cuts the cap's plane, and the plane it stands in.
#[derive(Debug, Clone, Copy, PartialEq, Default)]
pub struct SurfaceMixedCornerWallFacet {
    /// The end nearer the picked edge.
    pub from: Vector3,
    /// The far end, which is where the next facet starts.
    pub to: Vector3,
    /// The facet's outward normal.
    pub normal: Vector3,
}

/// The wall nobody picked, as a mixed corner sees it: the run of facets it is made of, each one a
/// straight segment in the cap face's own plane with the plane it stands in. Held through `Rc`,
/// because the rolled face and the unpicked face are often the one wall
/// ([`SurfaceMixedCornerFaces`](super::SurfaceMixedCornerFaces)).
///
/// **Why the cap's plane.** Everything the corner draws turns about the bead's axis, and that axis
/// is square to the cap. So the whole construction is flat: the outer rail is an arc about the
/// point the axis meets the cap, the ball centres are a radius under it, and the wall - wherever it
/// runs along the axis, which is checked here - shows in that plane as the polyline its facets
/// cut. Read once, every question the footprint asks is a ray or a segment in one plane.
///
/// **A flat wall is one facet.** Where nothing was rolled the plan carries no section, and this
/// reads the wall as the single segment from the corner vertex to the corner's cap corner in the
/// wall's own plane - which is the very square the flat construction always took, so an L prism
/// gets the same points from the general statement as it did from the special one.
#[derive(Debug)]
pub struct SurfaceMixedCornerWall {
    /// The corner this wall was read for.
    pub corner: Rc<SurfaceMixedCorner>,
    /// The facets of the wall, from the picked edge outward.
    pub facets: Vec<SurfaceMixedCornerWallFacet>,
    /// The bead's own foot on THIS face, in the cap's plane - the far end of the strip under the
    /// blend.
    pub foot: Vector3,
    /// Where each facet's own stretch of the wall begins, as a TURN about the axis.
    pub entry_turn: Vec<f64>,
    /// Where each facet's own stretch of the wall ends, as a TURN about the axis. A wall read as
    /// ONE PLANE has no far end, so its last facet's exit is `+inf`: the plane itself goes on to
    /// trim the whole runout.
    pub exit_turn: Vec<f64>,
    /// Which facet the bead's own profile is tangent to - `k`, counted from the picked edge.
    pub touched_facet: i32,
}

impl SurfaceMixedCornerWall {
    /// How far from square a facet may stand to the bead's axis and still be read as running
    /// along it.
    pub const SQUARE_ENOUGH: f64 = 1e-6;

    /// How near a turn has to come to a facet's own end to be read as standing on it. An ANGLE,
    /// and so a fixed epsilon: it forgives the rounding in one arc tangent of points that have
    /// been through float storage, and does not grow with the part.
    pub const TURN_SLACK: f64 = 1e-9;

    /// Builds the wall from its parts: the wall's `facets` outward from the picked edge, the one
    /// the bead's profile is tangent to (`touched`), whether the last of them runs on as a plane
    /// (`the_last_runs_on`), and the bead's own foot on this face in the cap's plane.
    pub fn new(
        corner: Rc<SurfaceMixedCorner>,
        facets: Vec<SurfaceMixedCornerWallFacet>,
        touched: i32,
        the_last_runs_on: bool,
        foot: Vector3,
    ) -> Self {
        let mut entry_turn = vec![0.0; facets.len()];
        let mut exit_turn = vec![0.0; facets.len()];

        for (facet, each) in facets.iter().enumerate() {
            entry_turn[facet] = Self::turn_about(&corner, each.from);
            exit_turn[facet] = Self::turn_about(&corner, each.to);
        }

        if the_last_runs_on && !facets.is_empty() {
            // A wall read as ONE PLANE has no far end: the segment named for it is only as long as
            // the strip under the blend, and the plane itself goes on to trim the whole runout. A
            // wall read from a section walk does end - the walk stops at the region's own rim -
            // and past that there is nothing left to trim against.
            exit_turn[facets.len() - 1] = f64::INFINITY;
        }

        SurfaceMixedCornerWall {
            corner,
            facets,
            foot,
            entry_turn,
            exit_turn,
            touched_facet: touched,
        }
    }

    /// How many facets of the wall the corner knows about, which is as far as the section walked.
    pub fn count(&self) -> usize {
        self.facets.len()
    }

    /// The outward normal of one facet's plane.
    pub fn normal_of(&self, facet: usize) -> Vector3 {
        self.facets[facet].normal
    }

    /// Where one facet's section begins, in the cap face's plane - the end nearer the picked edge.
    pub fn entry_of(&self, facet: usize) -> Vector3 {
        self.facets[facet].from
    }

    /// And where it ends, which is the next facet's beginning.
    pub fn exit_of(&self, facet: usize) -> Vector3 {
        self.facets[facet].to
    }

    /// The first facet the walks outward start from: the touched one, or the first where none is.
    fn first_facet(&self) -> usize {
        self.touched_facet.max(0) as usize
    }

    /// How far out of the axis the wall stands in the meridian plane at `turn`, and which facet
    /// answers for it, as `(distance, facet)`. The ray from the axis is turned in the CAP's plane
    /// and met with each facet's own line in it, walking outward from the facet the bead touches.
    /// On a flat wall this is exactly `r / cos(phi)` past the turn the bead ends at. `None` past
    /// the last facet, where the wall has RUN OUT - a corner of another shape.
    pub fn try_out_of_the_axis(&self, turn: f64) -> Option<(f64, usize)> {
        for j in self.first_facet()..self.facets.len() {
            if turn > self.exit_turn[j] + Self::TURN_SLACK {
                continue;
            }

            let distance = self.try_distance_on(j, turn)?;

            return Some((distance, j));
        }

        None
    }

    /// The plane one facet stands in, named by the turn the blend meets it at: the facet the ray
    /// at `turn` lands on, as `(a point of it, its outward normal)`.
    pub fn try_plane_at(&self, turn: f64) -> Option<(Vector3, Vector3)> {
        let (_, facet) = self.try_out_of_the_axis(turn)?;

        Some((self.facets[facet].from, self.facets[facet].normal))
    }

    /// The turn at which the curve the ball's touch point really traces on the cap - an ELLIPSE
    /// about where the axis meets it wherever the cap leans, the circle of radius `2r` where it
    /// does not - meets the wall. The ellipse is `M + C cos + S sin`, so met with a facet's plane
    /// it is `A cos + B sin = K` and the crossing is an arc cosine rather than a search. The
    /// facet's own stretch is read at the CROSSING POINT's angle round the axis and not at the
    /// curve's parameter, because what says which facet a point stands on is where the point is.
    pub fn try_where_the_contact_meets_the_wall(&self) -> Option<f64> {
        let arms = self.corner.spine.the_contact_arms();
        let (middle, cosine, sine) = (arms.middle, arms.cosine, arms.sine);

        for facet in self.first_facet()..self.facets.len() {
            let normal = self.facets[facet].normal;

            let Some(at) = Self::try_the_first_root(
                cosine.dot(normal),
                sine.dot(normal),
                (self.facets[facet].from - middle).dot(normal),
                self.corner.turn_radians(),
            ) else {
                continue;
            };

            let stands = self.turn_of(middle + (cosine * at.cos()) + (sine * at.sin()));

            if stands < self.entry_turn[facet] - Self::TURN_SLACK
                || stands > self.exit_turn[facet] + Self::TURN_SLACK
            {
                continue;
            }

            return Some(at);
        }

        None
    }

    /// The first root of `A cos t + B sin t = K` at or past `from`, or `None` where the two never
    /// meet - a curve that never stands that far across the plane at all.
    #[allow(clippy::neg_cmp_op_on_partial_ord)] // `!(x > y)` on purpose: it refuses a NaN as well.
    pub fn try_the_first_root(a: f64, b: f64, k: f64, from: f64) -> Option<f64> {
        let reach = ((a * a) + (b * b)).sqrt();

        if !(reach > 1e-12) {
            return None;
        }

        // A TANGENCY is a double root and not a miss, and it is the case this is asked at first:
        // the station at the patch's own last turn touches the wall, so its ring meets that plane
        // exactly once and the discriminant is zero up to the rounding in a dot product. The same
        // floor the closed form puts under its own square root, said as a ratio because this one
        // is dimensionless.
        let ratio = k / reach;

        if ratio.abs() > 1.0 + 1e-9 {
            return None;
        }

        let middle = f64::atan2(b, a);
        let spread = ratio.clamp(-1.0, 1.0).acos();
        let mut root: Option<f64> = None;

        // Two roots, one either side of the curve's own furthest reach across the plane; the one
        // the turn REACHES is the first of them past where it started.
        for each in [middle + spread, middle - spread] {
            let mut at = each;

            while at < from - Self::TURN_SLACK {
                at += 2.0 * PI;
            }

            if root.is_none_or(|found| at < found) {
                root = Some(at);
            }
        }

        root
    }

    /// How much further than its own turn a point of the ring (`touch`, where the ball touches
    /// the cap at the patch's last turn) has to be carried round the axis before it meets the
    /// wall: the same question asked of the IDEAL rail, asked of a point that stands elsewhere.
    /// Where the ring LEANS its touch row stands a degree or so past the corner's turn and a few
    /// thousandths off the rail's radius, and that row is the curve that has to die on the wall.
    #[allow(clippy::neg_cmp_op_on_partial_ord)] // `!(x > 0)` on purpose: it refuses a NaN as well.
    pub fn try_where_the_touch_point_meets_the_wall(&self, touch: Vector3) -> Option<f64> {
        let out0 = touch - self.corner.outer_rail_centre;
        let in_the_plane =
            out0 - (self.corner.out_of_the_cap_face * out0.dot(self.corner.out_of_the_cap_face));
        let radius = in_the_plane.length();
        let from = self.turn_of(touch);

        if !(radius > 0.0) {
            return None;
        }

        let at = self.try_where_a_circle_meets_the_wall(radius, from)?;

        Some(at - from)
    }

    /// Where a circle about the axis of `radius` first meets the wall past `from`, as a turn: the
    /// facet it lands on is the first one whose own stretch carries the crossing.
    pub fn try_where_a_circle_meets_the_wall(&self, radius: f64, from: f64) -> Option<f64> {
        for facet in self.first_facet()..self.facets.len() {
            let Some(at) = self.try_meets_the_circle_on(facet, radius, from) else {
                continue;
            };

            if at < self.entry_turn[facet] - Self::TURN_SLACK
                || at > self.exit_turn[facet] + Self::TURN_SLACK
            {
                // The circle clears this facet's own stretch without ever standing that far out of
                // the axis over it, so the facet past it is what answers.
                continue;
            }

            return Some(at);
        }

        None
    }

    /// The turns between `from` and `to` where the ray crosses from one facet to the next - the
    /// creases the trim curve runs over. A station is put on each of them, so that every chord of
    /// the trim curve has both its ends on ONE facet and the wall gives up a planar piece under
    /// each.
    pub fn creases_between(&self, from: f64, to: f64) -> Vec<f64> {
        let mut crossings = Vec::new();

        let mut facet = self.first_facet();
        while facet + 1 < self.facets.len() {
            if self.facets[facet].normal.dot(self.facets[facet + 1].normal)
                > 1.0 - Self::SQUARE_ENOUGH
            {
                // The two stand in one PLANE - a wall's quad is two triangles, and the walk crosses
                // the diagonal between them like any other edge - so there is no crease here.
                facet += 1;
                continue;
            }

            let at = self.exit_turn[facet];

            if at > from + Self::TURN_SLACK && at < to - Self::TURN_SLACK {
                crossings.push(at);
            }

            facet += 1;
        }

        crossings
    }

    /// Where `at` stands as a turn about the axis, read against the cap plane's own frame: zero
    /// towards the shared face, a quarter turn towards the wall, in `[0, 2 pi)`.
    pub fn turn_of(&self, at: Vector3) -> f64 {
        Self::turn_about(&self.corner, at)
    }

    /// [`SurfaceMixedCornerWall::turn_of`] for a corner, so the constructor can ask it before the
    /// wall exists.
    fn turn_about(corner: &SurfaceMixedCorner, at: Vector3) -> f64 {
        let out0 = at - corner.outer_rail_centre;
        let turn = f64::atan2(
            out0.dot(corner.towards_the_wall_square()),
            out0.dot(corner.towards_the_shared_face()),
        );

        if turn < 0.0 {
            turn + (2.0 * PI)
        } else {
            turn
        }
    }

    /// How far out of the axis one facet's own plane stands at `turn`; `None` where the ray runs
    /// along the facet (it never meets that plane, and the facet past it answers) or meets it
    /// behind the axis.
    #[allow(clippy::neg_cmp_op_on_partial_ord)] // `!(x > y)` on purpose: it refuses a NaN as well.
    pub fn try_distance_on(&self, facet: usize, turn: f64) -> Option<f64> {
        let normal = self.facets[facet].normal;
        let along_the_ray = self.corner.out_of_the_axis(turn).dot(normal);

        if !(along_the_ray.abs() > 1e-9) {
            return None;
        }

        let distance =
            (self.facets[facet].from - self.corner.outer_rail_centre).dot(normal) / along_the_ray;

        (distance > 0.0).then_some(distance)
    }

    /// Where a circle of `radius` about the axis crosses one facet's own line, as a turn past
    /// `from`. The facet's normal is a unit vector in the cap's plane, so the ray's own projection
    /// on it is `cos(turn - psi)` and the crossing is one arc cosine rather than a search.
    #[allow(clippy::neg_cmp_op_on_partial_ord)] // `!(x > y)` on purpose: it refuses a NaN as well.
    pub fn try_meets_the_circle_on(&self, facet: usize, radius: f64, from: f64) -> Option<f64> {
        let normal = self.facets[facet].normal;
        let zero = self.corner.towards_the_shared_face().dot(normal);
        let quarter = self.corner.towards_the_wall_square().dot(normal);
        let stand_off = (self.facets[facet].from - self.corner.outer_rail_centre).dot(normal);
        let wanted = stand_off / radius;

        // The length of the `(zero, quarter)` vector.
        if wanted.abs() > 1.0 || !(((zero * zero) + (quarter * quarter)).sqrt() > 1e-9) {
            return None;
        }

        let psi = f64::atan2(quarter, zero);
        let spread = wanted.clamp(-1.0, 1.0).acos();
        let mut turn = 0.0;

        // Two roots, one either side of the facet's own perpendicular; the one the turn REACHES is
        // the one past where it started, so the smaller of them above that turn is taken. (Zero is
        // the "none yet" sentinel, so a root of exactly zero reads as none.)
        for root in [psi + spread, psi - spread] {
            let mut at = root;

            while at < from - Self::TURN_SLACK {
                at += 2.0 * PI;
            }

            if turn == 0.0 || at < turn {
                turn = at;
            }
        }

        (turn > 0.0).then_some(turn)
    }
}

/// The wall's reading off its corner, where the rail meets it, and the strip under the blend.
impl SurfaceMixedCornerWall {
    /// The turn at which the ball's touch point on the cap - `2r` out of the axis - meets the wall,
    /// which is where the runout dies into the sharp edge the part keeps. On a flat wall it is
    /// sixty degrees past the bead's own end whatever the wedge; on a chorded one it is where the
    /// outer rail crosses the facet segment it reaches. Where the bead's ring leans it is the touch
    /// curve's own crossing instead.
    pub fn try_where_the_rail_meets_the_wall(&self) -> Option<f64> {
        if self.corner.the_beads_ring_leans() {
            self.try_where_the_contact_meets_the_wall()
        } else {
            self.try_where_a_circle_meets_the_wall(
                self.corner.outer_rail_radius(),
                self.corner.turn_radians(),
            )
        }
    }

    /// The strip of this wall the blend stands over between the bead's stop plane and the cap,
    /// facet by facet as `(rim, normal)` - each facet's piece its section segment carried down the
    /// wall to the plane the bead's band stops on ([`SurfaceMixedCorner::down_to_the_beads_end`]),
    /// the last ending at the bead's foot.
    #[allow(clippy::neg_cmp_op_on_partial_ord)] // `!(x > tolerance)` on purpose: it skips a NaN too.
    pub fn strip_under_the_blend(&self) -> Vec<([Vector3; 4], Vector3)> {
        let mut pieces = Vec::new();
        let last = (self.touched_facet as i64).min(self.facets.len() as i64 - 1);

        for facet in 0..=last {
            let each = self.facets[facet as usize];
            let from = each.from;
            let to = if facet == last { self.foot } else { each.to };

            // Measured against the FACET's own length rather than against zero: where the bead
            // rests on a crease its foot IS that crease's vertex, but the two are arrived at
            // through different arithmetic and agree to the last bit or two rather than exactly.
            // Read as a piece, that is a rim with a side of no length, which no bite can name a
            // half plane for - and one piece fewer, not a refusal.
            if !((to - from).length()
                > 1e-9 * cs_math::max_f64(1.0, (each.to - each.from).length()))
            {
                continue;
            }

            pieces.push((
                [
                    from,
                    to,
                    self.corner.down_to_the_beads_end(to),
                    self.corner.down_to_the_beads_end(from),
                ],
                each.normal,
            ));
        }

        pieces
    }

    /// Reads the wall NOBODY PICKED - the face the runout is trimmed against - as one plane or as
    /// the walk the bead took across it, its foot the corner's cap corner.
    pub fn try_read(corner: &Rc<SurfaceMixedCorner>) -> Result<Self, BevelAdvisory> {
        Self::try_read_section(
            corner,
            corner.unpicked_section().as_deref(),
            corner.cap_corner(),
        )
    }

    /// Reads one face of the corner - `section` the walk across it, or `None` where it is one
    /// plane - with `foot` the bead's own foot on it in the cap's plane. A facet that does not run
    /// along the bead's axis shows in the cap's plane as something other than the segment this is
    /// written in, so it is refused by name rather than answered wrongly.
    pub fn try_read_section(
        corner: &Rc<SurfaceMixedCorner>,
        section: Option<&SurfaceWallSection>,
        foot: Vector3,
    ) -> Result<Self, BevelAdvisory> {
        let not_supported = || {
            BevelAdvisory::new(
                BevelAdvisoryCode::SurfaceCornerNotSupported,
                vec![corner.corner.into()],
            )
        };

        let Some(section) = section else {
            // Nothing was rolled, so the wall is one plane and the corner's own two points name it.
            return Ok(Self::new(
                Rc::clone(corner),
                vec![SurfaceMixedCornerWallFacet::new(
                    corner.corner,
                    foot,
                    corner.out_of_the_wall,
                )],
                0,
                true,
                foot,
            ));
        };

        let along = corner.axis().direction;
        let mut read = Vec::with_capacity(section.facets.len());

        for step in &section.facets {
            if step.normal.get_normal().dot(along).abs() > Self::SQUARE_ENOUGH {
                // A facet leaning off the axis cuts the cap's plane in a line that is not where the
                // section says it is, and the whole reading below is drawn in that plane.
                return Err(not_supported());
            }

            read.push(SurfaceMixedCornerWallFacet::new(
                Self::in_the_cap(corner, step.from),
                Self::in_the_cap(corner, step.to),
                step.normal.get_normal(),
            ));
        }

        if read.is_empty() || section.touched_facet as i64 >= read.len() as i64 {
            return Err(not_supported());
        }

        Ok(Self::new(
            Rc::clone(corner),
            read,
            section.touched_facet,
            false,
            foot,
        ))
    }

    /// One point of the section, carried into the cap face's own plane along a line the facet
    /// holds - the bead's axis, which every facet of this wall runs along.
    pub fn in_the_cap(corner: &SurfaceMixedCorner, at: Vector3) -> Vector3 {
        corner.up_into_the_cap(at)
    }
}

impl SurfaceMixedCornerWallFacet {
    /// A facet from `from` to `to` facing `normal`.
    pub fn new(from: Vector3, to: Vector3, normal: Vector3) -> Self {
        SurfaceMixedCornerWallFacet { from, to, normal }
    }
}
