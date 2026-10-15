//! A band's strip cut back along the FACE BEYOND the region it landed on
//! ([`SurfaceStripTrimmedByTheFaceBeyond`]): its fields, its constructor, the corner
//! beyond (three planes met) and its ring, rim and profile questions, and the reading itself
//! ([`SurfaceStripTrimmedByTheFaceBeyond::try_read`]).

use vector_math::Vector3;

use super::{SurfaceEdgePlan, SurfaceStrip, SurfaceVertexPool};
use crate::bevel::{BevelAdvisory, BevelAdvisoryCode};

/// Which rails the face beyond's plane crosses inside the
/// edge's span, where each of them is cut, and the one point the rails it never crosses all
/// collapse to.
///
/// The shape this is for is a band that takes the WHOLE of its landing face
/// (`SurfaceBandRunsOffTheFaceBeyond`): the tangent rail then stands off the part over the edge's
/// own length, and what bounds the strip on that side is the plane the face runs off onto. Every
/// rail meets that plane at its own parameter along the edge - a staircase - and the curve those
/// crossings draw is the ellipse the plane cuts the profile's cylinder in. **Closed form, per
/// rail**: a rail is a line, so where it meets a plane is one division, which puts the crossings
/// exactly on the face beyond.
///
/// **The rails that never cross inside the span collapse to ONE point**, put where the surface
/// really ends: on the chord between the tangent rail and the first rail that IS crossed, clipped
/// at the plane. That point is minted ONCE and is the strip's, the section's on the face beyond,
/// and the end face's at the other end of the edge - three rims through one vertex. **The corner
/// beyond** the section turns down at is a vertex the part already has, looked up rather than
/// minted, and carried at the head of the profile as the section's DIE POINT
/// (`surface_end_cut::try_cut`).
#[derive(Debug, Clone, PartialEq, Default)]
pub struct SurfaceStripTrimmedByTheFaceBeyond {
    /// Which side of the strip ran off - 0 for rail zero's, 1 for the last rail's.
    pub side: i32,
    /// Which end of the edge the rails are cut back FROM.
    pub at_the_start: bool,
    /// The parameter of the end the trim does NOT stand at, which every rail reaches.
    pub far_param: f64,
    /// Where each rail meets the plane beyond, as a parameter along the edge.
    pub trimmed_at: Vec<f64>,
    /// The first rail, counted in from the run-off side, whose crossing stands inside the span. The
    /// rails before it are air over the whole edge and collapse to
    /// [`SurfaceStripTrimmedByTheFaceBeyond::shared`].
    pub first_kept: i32,
    /// The one point the collapsed rails stand at, minted once and shared by three rims.
    pub shared: i32,
    /// The part's own corner where the plane beyond, the end plane and the landing face meet.
    pub the_corner_beyond: i32,
    /// A point on the plane beyond.
    pub plane_point: Vector3,
    /// The outward normal of the plane beyond.
    pub plane_normal: Vector3,
    /// The seam the strip sews to on the run-off side, as `(vertex, param)`: the ONE shared vertex,
    /// standing at the end the trim does not cut back. A band that landed on its face has a run of
    /// tangent vertices there; a band that ran off it has a point, which is all the stitch needs -
    /// it reads the seam's first and last vertex, and here they are the same one.
    pub seam: Vec<(i32, f64)>,
}

impl SurfaceStripTrimmedByTheFaceBeyond {
    /// The part's own vertex where the plane beyond (`plane_point`, `plane_normal`), the plane the
    /// band ENDS on and the landing face on `side` (0 left, 1 right) all meet - the corner the
    /// section turns down at. Looked up in the pool rather than minted: it is a vertex of three
    /// faces the build never touches, and a second one a rounding away is a crack.
    pub fn try_the_corner_beyond(
        pool: &SurfaceVertexPool,
        edge: &SurfaceEdgePlan,
        side: i32,
        at_the_start: bool,
        plane_point: Vector3,
        plane_normal: Vector3,
    ) -> Option<i32> {
        let along = edge.edge.direction();
        let ends_at = if at_the_start { edge.to() } else { edge.from() };
        let landing = if side == 0 {
            edge.edge.left_normal()
        } else {
            edge.edge.right_normal()
        };

        let at = Self::try_three_planes(
            plane_point,
            plane_normal,
            ends_at,
            along,
            edge.from(),
            landing,
        )?;

        pool.try_find_near(at, pool.near_enough(), None)
    }

    /// Where three planes meet, by Cramer's rule - `None` where they do not meet at a point.
    #[allow(clippy::neg_cmp_op_on_partial_ord)] // `!(x > y)`, which refuses a NaN as well.
    pub fn try_three_planes(
        point_a: Vector3,
        normal_a: Vector3,
        point_b: Vector3,
        normal_b: Vector3,
        point_c: Vector3,
        normal_c: Vector3,
    ) -> Option<Vector3> {
        let volume = normal_a.dot(normal_b.cross(normal_c));

        if !(volume.abs() > 1e-9) {
            return None;
        }

        Some(
            ((normal_b.cross(normal_c) * point_a.dot(normal_a))
                + (normal_c.cross(normal_a) * point_b.dot(normal_b))
                + (normal_a.cross(normal_b) * point_c.dot(normal_c)))
                / volume,
        )
    }
}

/// The trim's constructor and its ring, rim and profile questions (`rail_at`, `ring_at`,
/// `walk_of_the_rim`, `profile_of`).
impl SurfaceStripTrimmedByTheFaceBeyond {
    /// A trim from its readings. The seam starts empty; the reading sets it once the trim is made.
    #[allow(clippy::too_many_arguments)]
    pub fn new(
        side: i32,
        at_the_start: bool,
        far_param: f64,
        trimmed_at: Vec<f64>,
        first_kept: i32,
        shared: i32,
        the_corner_beyond: i32,
        plane_point: Vector3,
        plane_normal: Vector3,
    ) -> Self {
        Self {
            side,
            at_the_start,
            far_param,
            trimmed_at,
            first_kept,
            shared,
            the_corner_beyond,
            plane_point,
            plane_normal,
            seam: Vec::new(),
        }
    }

    /// Which rail stands `steps_in` in from the side that ran off.
    pub fn rail_at(side: i32, rail_count: i32, steps_in: i32) -> i32 {
        if side == 0 {
            steps_in
        } else {
            rail_count - 1 - steps_in
        }
    }

    /// The ring at one end of the strip, with the collapsed rails standing on the
    /// shared point - the STAIRCASE at the end the trim cuts back, and the square end's own ring at
    /// the other - and where each rail meets it. `on_the_seam` is the vertex the
    /// rail on the OTHER side of the strip stands at, the one the walkback left on that side's
    /// tangent line, which the ring has to be built from rather than minting its own a rounding away.
    pub fn ring_at(
        &self,
        pool: &mut SurfaceVertexPool,
        strip: &SurfaceStrip,
        at_the_start: bool,
        on_the_seam: i32,
    ) -> (Vec<i32>, Vec<f64>) {
        let rail_count = strip.rail_count();
        let mut ring = vec![0i32; rail_count as usize];
        let mut meets_at = vec![0.0; rail_count as usize];

        for i in 0..rail_count {
            let rail = Self::rail_at(self.side, rail_count, i) as usize;

            if i < self.first_kept {
                ring[rail] = self.shared;
                meets_at[rail] = self.far_param;
                continue;
            }

            meets_at[rail] = if at_the_start == self.at_the_start {
                self.trimmed_at[rail]
            } else {
                self.far_param
            };

            ring[rail] = if i == rail_count - 1 {
                on_the_seam
            } else {
                pool.add(strip.point_on(rail as i32, meets_at[rail]))
            };
        }

        (ring, meets_at)
    }

    /// Which rails the section's rim is walked over, in profile order - from the
    /// run-off side in, starting one step OUTSIDE the first kept rail so that the collapsed point the
    /// profile turns down at is on the walk. Read off here rather than inside the cap, so that a cap
    /// standing at a LANDING end can be the same construction (`SurfaceEndCapAroundAFeature`).
    pub fn walk_of_the_rim(&self, rail_count: i32) -> Vec<i32> {
        let skipped = 0.max(self.first_kept - 1);

        (0..rail_count - skipped)
            .map(|at| Self::rail_at(self.side, rail_count, at + skipped))
            .collect()
    }

    /// The ring read as the PROFILE a section is bounded by - the part's corner
    /// beyond, then the ring from the run-off side in, with the collapsed rails' repeated point said
    /// once.
    pub fn profile_of(&self, ring: &[i32]) -> Vec<i32> {
        let mut profile = vec![self.the_corner_beyond];
        let count = ring.len() as i32;

        for i in self.first_kept - 1..count {
            let vertex = ring[Self::rail_at(self.side, count, i.max(0)) as usize];

            if profile.last() != Some(&vertex) {
                profile.push(vertex);
            }
        }

        profile
    }
}

impl SurfaceStripTrimmedByTheFaceBeyond {
    /// The trim standing on side `side` of strip `strip` in `trims` (per strip, per side), or
    /// `None` where there is none or no such strip.
    pub fn standing_on(
        trims: Option<&[[Option<std::rc::Rc<SurfaceStripTrimmedByTheFaceBeyond>>; 2]]>,
        strip: i32,
        side: usize,
    ) -> Option<std::rc::Rc<SurfaceStripTrimmedByTheFaceBeyond>> {
        let trims = trims?;
        if strip < 0 || strip as usize >= trims.len() {
            return None;
        }

        trims[strip as usize][side].clone()
    }
}

impl SurfaceStripTrimmedByTheFaceBeyond {
    /// The strip cut back along the face beyond (the plane `plane_point`,
    /// `plane_normal`) where its band ran off at `side` - each rail's crossing, clamped into the span
    /// only for the rails the run-off collapsed; the point the surface ends on at the far end, minted
    /// once; and the part's corner the three planes meet at. `Err` names where the run-off reading and
    /// this one disagree, or where no vertex stands at the corner beyond.
    #[allow(clippy::too_many_arguments, clippy::neg_cmp_op_on_partial_ord)]
    pub fn try_read(
        pool: &mut SurfaceVertexPool,
        strip: &SurfaceStrip,
        edge: &SurfaceEdgePlan,
        side: i32,
        plane_point: Vector3,
        plane_normal: Vector3,
        at_the_start: bool,
    ) -> Result<SurfaceStripTrimmedByTheFaceBeyond, BevelAdvisory> {
        let disagrees = |at: Vector3| {
            BevelAdvisory::new(
                BevelAdvisoryCode::SurfaceRunOffReadingDisagrees,
                vec![at.into()],
            )
        };
        let length = (edge.to() - edge.from()).dot(strip.along());
        let far_param = if at_the_start { length } else { 0.0 };
        let rail_count = strip.rail_count();
        let mut trimmed_at = vec![0.0; rail_count.max(0) as usize];
        let mut first_kept = -1;

        for i in 0..rail_count {
            let rail = Self::rail_at(side, rail_count, i);
            let r = rail as usize;

            let Some(crossing) = strip.try_cross(rail, plane_point, plane_normal) else {
                // A rail running ALONG the plane beyond: the two readings have come apart.
                return Err(disagrees(strip.point_on(rail, 0.0)));
            };
            trimmed_at[r] = crossing;

            let in_the_span =
                trimmed_at[r] > -pool.near_enough() && trimmed_at[r] < length + pool.near_enough();

            if first_kept < 0 {
                if in_the_span {
                    first_kept = i;
                }
            } else if !in_the_span {
                // A rail further in than the first one crossed, standing outside the span again.
                return Err(disagrees(strip.point_on(rail, trimmed_at[r])));
            }

            // Only the COLLAPSED rails are ever clamped.
            trimmed_at[r] = vector_math::ieee_math::min_f64(
                vector_math::ieee_math::max_f64(trimmed_at[r], 0.0),
                length,
            );
        }

        if first_kept <= 0 {
            // Nothing was cut off, or the tangent rail still crosses inside the span.
            return Err(disagrees((edge.from() + edge.to()) / 2.0));
        }

        // Where the surface really ends on the run-off side: minted HERE, before anything is drawn.
        let in_the_air = strip.point_on(Self::rail_at(side, rail_count, first_kept - 1), far_param);
        let on_the_part = strip.point_on(Self::rail_at(side, rail_count, first_kept), far_param);
        let across = (in_the_air - on_the_part).dot(plane_normal);

        if !(across.abs() > 0.0) {
            // The chord the surface ends on runs ALONG the plane beyond.
            return Err(disagrees(on_the_part));
        }

        let shared = pool.add(
            on_the_part
                + ((in_the_air - on_the_part)
                    * ((plane_point - on_the_part).dot(plane_normal) / across)),
        );

        let Some(corner_beyond) =
            Self::try_the_corner_beyond(pool, edge, side, at_the_start, plane_point, plane_normal)
        else {
            // The three planes do not meet at a vertex the part already has: named.
            return Err(BevelAdvisory::new(
                BevelAdvisoryCode::SurfaceStripEndNotSupported,
                vec![(if at_the_start { edge.to() } else { edge.from() }).into()],
            ));
        };

        let mut trimmed = SurfaceStripTrimmedByTheFaceBeyond::new(
            side,
            at_the_start,
            far_param,
            trimmed_at,
            first_kept,
            shared,
            corner_beyond,
            plane_point,
            plane_normal,
        );
        trimmed.seam = vec![(shared, far_param)];

        Ok(trimmed)
    }
}
