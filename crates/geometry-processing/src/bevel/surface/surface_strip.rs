//! The bevel surface over ONE chain edge, as the lines it is ruled by ([`SurfaceStrip`]): its
//! fields and its construction from a plan ([`SurfaceStrip::try_create`]), and its rail readings
//! ([`SurfaceStrip::point_on`], [`SurfaceStrip::param_of`], [`SurfaceStrip::try_cross`]).

use vector_math::{math_helper, Vector3};

use super::{SurfaceBevelPlan, SurfaceEdgePlan};
use crate::bevel::{bevel_arc_chording, BevelAdvisory, BevelAdvisoryArg, BevelAdvisoryCode};
use crate::topology::EdgeConvexity;

/// The bevel surface over ONE chain edge, as the lines it is ruled by - one rail per profile
/// sample, each a line parallel to the edge, the first lying on the left tangent line and the last
/// on the right. A chamfer has two rails and a round has one per arc sample.
///
/// **Rails rather than points.** The surface over a straight edge between two planar faces is
/// ruled: every cross section is the same profile, carried along. Saying so in lines rather than in
/// sampled points is what lets the ends be cut by a PLANE - the mitre with the next edge, or the
/// face beyond the corner - exactly, at whatever parameter each rail meets it, instead of only
/// where a cross section happened to be sampled. `SurfaceBevelBuilder` is what turns rails into
/// triangles.
///
/// **No convexity branch.** The arc is swung about the center line the plan found, from the left
/// foot to the right foot the short way round, and that alone puts it where it belongs: the center
/// stands in the wedge the profile is drawn in - the material wedge on a convex edge, the void
/// wedge on a concave one - so the arc always bulges TOWARD the original corner. On a convex edge
/// that leaves the circular segment under each chord as material, which is why a tessellated cut
/// takes a little more than the closed form; on a concave edge the same construction leaves it as
/// void, and the bead adds a little more. One formula, two readings, and
/// `SurfaceEdgePlan::convexity` read by nothing here.
///
/// **Which way is out.** The bisector of the two INWARD directions the plan used, read outward
/// ([`SurfaceStrip::outward_reference`]). Every sample's own outward normal lies in the sector
/// between the two tangencies, whose half angle is `(180 - wedge) / 2` and so always under 90
/// degrees, so a triangle wound to face that bisector is wound to face out - on a bead as much as
/// on a cut, because a bead's two faces are the ones that bound the void it fills. Taken from the
/// plan's inward directions rather than from the two FACE normals, which are the same bisector
/// wherever nothing was rolled and not where something was: a profile rolled along a curved wall is
/// tangent to facet k rather than to the chain's own face, so the arc sweeps round by the roll's
/// turn as well, and on a narrow wedge - 40 degrees rolled three 9 degree facets - the last samples
/// leave the hemisphere the face normals' bisector names and the strip is wound inside out.
///
/// Shared by everything that reads a strip's rails (the corner patches, the run-out crossings), so
/// those hold it as `Rc<SurfaceStrip>`.
#[derive(Debug, Clone, PartialEq, Default)]
pub struct SurfaceStrip {
    /// The point the edge's parameter is measured from: parameter 0 along every rail.
    pub origin: Vector3,
    /// The unit direction of the edge, which every rail runs along.
    pub along: Vector3,
    /// One point of each rail, rail by rail, at the strip's own origin: the profile sample the rail
    /// carries along the edge.
    pub rail_points: Vec<Vector3>,
    /// The angle between one rail and the next, in degrees - the strip's own sweep divided by the
    /// chord count `bevel_arc_chording::segments_for` gave it, and zero for a chamfer, whose one
    /// chord sweeps no arc at all.
    ///
    /// Read by the MITRE, which pairs the two strips' rails by their ANGLE from the face they
    /// share: two beads of the same radius tangent to one face carry their rail at angle theta in
    /// the same plane parallel to it, so rail theta of one crosses rail theta of the other on the
    /// mitre plane whatever the two wedges are. Where both arcs took one step that pairing is rail
    /// k against rail k; where they did not, it is the UNION of the two sets of angles
    /// (`SurfaceMitreChordedApart`), and this is the number that says which of the two.
    pub rail_step_degrees: f64,
    /// The direction any triangle of this strip must face, to within a right angle: the bisector of
    /// the two face normals.
    pub outward_reference: Vector3,
}

impl SurfaceStrip {
    /// Builds the strip from its parts; [`Self::try_create`] makes one.
    fn new(
        origin: Vector3,
        along: Vector3,
        rail_points: Vec<Vector3>,
        outward_reference: Vector3,
        rail_step_degrees: f64,
    ) -> Self {
        SurfaceStrip {
            origin,
            along,
            rail_points,
            rail_step_degrees,
            outward_reference,
        }
    }

    /// Draws the strip's rails across `edge`'s profile: the two feet for a chamfer, and one rail
    /// per arc sample from the left tangent line round the centre line to the right for a round,
    /// chorded at `segments_per_quarter` per quarter turn of arc ([`bevel_arc_chording`]) so that
    /// an edge's chording is decided by how much arc it actually sweeps. Fails on a wedge too sharp
    /// or too flat for the plan's one floor ([`SurfaceBevelPlan::MINIMUM_WEDGE_ANGLE_DEGREES`]), or
    /// a profile with no radius.
    // The negated comparisons are `!(x > y)` on purpose: they refuse a NaN as well.
    #[allow(clippy::neg_cmp_op_on_partial_ord)]
    pub fn try_create(
        edge: &SurfaceEdgePlan,
        segments_per_quarter: i32,
    ) -> Result<SurfaceStrip, BevelAdvisory> {
        let origin = edge.from();
        let along = edge.edge.direction();
        let left_foot = edge.left_tangent_line.closest_point_to(origin);
        let right_foot = edge.right_tangent_line.closest_point_to(origin);
        let outward = edge.edge.left_normal() + edge.edge.right_normal();

        // The two normals are (180 - wedge) apart, so their sum is 2 sin(wedge / 2) long: a knife
        // edge has no outside to face, and a wedge below the plan's own floor has one that no
        // float can tell from a knife edge's. One angle, cited rather than restated - see
        // SurfaceBevelPlan::MINIMUM_WEDGE_ANGLE_DEGREES.
        let knife = 2.0
            * (math_helper::degrees_to_radians(SurfaceBevelPlan::MINIMUM_WEDGE_ANGLE_DEGREES)
                / 2.0)
                .sin();
        if !(outward.length() > knife) {
            return Err(BevelAdvisory::new(
                BevelAdvisoryCode::FaceAngleTooSharp,
                vec![BevelAdvisoryArg::Double(0.0)],
            ));
        }

        // The profile's own wedge, read outward: the two inward directions bisect it, pointing
        // into the material on a convex edge and into the void on a concave one, so one sign turns
        // either into the way the strip faces. It follows the roll, which the face normals do not.
        // A bisector of no length is a wedge at the plan's flat end, where the sweep guard below
        // has the better complaint, so the normals' answer stands in until it fires.
        let to_the_wedge = edge.left_inward + edge.right_inward;
        let mut facing = if edge.convexity() == EdgeConvexity::Concave {
            to_the_wedge
        } else {
            -to_the_wedge
        };

        if !(facing.length_squared() > 0.0) {
            facing = outward;
        }

        if !edge.has_center_line {
            return Ok(SurfaceStrip::new(
                origin,
                along,
                vec![left_foot, right_foot],
                facing.get_normal(),
                0.0,
            ));
        }

        let center = edge.center_line.closest_point_to(origin);
        let from_center = left_foot - center;
        let to_center = right_foot - center;
        let radius = from_center.length();

        if !(radius > 0.0) || !(to_center.length_squared() > 0.0) {
            return Err(BevelAdvisory::new(
                BevelAdvisoryCode::BevelDoesNotFitCorner,
                Vec::new(),
            ));
        }

        // The sweep is the turn from one foot to the other, which is 180 less the wedge; taken
        // from the vectors themselves rather than from the wedge so that the last sample lands ON
        // the right tangent line to the bit, which is what the strip has to sew to.
        let cosine = from_center.dot(to_center) / (from_center.length() * to_center.length());
        // Clamped to [-1, 1] in a way that passes a NaN through.
        let sweep = (if cosine.is_nan() {
            cosine
        } else {
            cosine.clamp(-1.0, 1.0)
        })
        .acos();

        // And it is bounded from below by the same one angle: a wedge at the plan's flat end
        // leaves an arc of nothing to swing, and an arc of nothing between two feet a rounding
        // apart is not a profile.
        if !(math_helper::radians_to_degrees(sweep)
            > 180.0 - SurfaceBevelPlan::MAXIMUM_WEDGE_ANGLE_DEGREES)
        {
            return Err(BevelAdvisory::new(
                BevelAdvisoryCode::FaceAngleTooFlat,
                vec![BevelAdvisoryArg::Double(
                    180.0 - math_helper::radians_to_degrees(sweep),
                )],
            ));
        }
        let turning = if from_center.cross(to_center).dot(along) < 0.0 {
            -1.0
        } else {
            1.0
        };
        let samples = i32::max(
            1,
            bevel_arc_chording::segments_for(
                math_helper::radians_to_degrees(sweep),
                segments_per_quarter,
            ),
        );

        let mut rails = Vec::with_capacity(samples as usize + 1);

        for i in 0..=samples {
            rails.push(
                center
                    + Self::turn(
                        from_center,
                        along,
                        turning * sweep * f64::from(i) / f64::from(samples),
                    ),
            );
        }

        // The ends are the feet themselves rather than the last turn of the arc: the walkback's
        // own vertices stand on those two lines, and a rail that misses one by a rounding is a
        // crack.
        rails[0] = left_foot;
        rails[samples as usize] = right_foot;

        Ok(SurfaceStrip::new(
            origin,
            along,
            rails,
            facing.get_normal(),
            math_helper::radians_to_degrees(sweep) / f64::from(samples),
        ))
    }

    /// `vector` turned `radians` about `axis` - Rodrigues, written out rather than built into a
    /// matrix because it is called once per arc sample.
    pub fn turn(vector: Vector3, axis: Vector3, radians: f64) -> Vector3 {
        let cos = radians.cos();
        let sin = radians.sin();

        (vector * cos) + (axis.cross(vector) * sin) + (axis * axis.dot(vector) * (1.0 - cos))
    }
}

/// The strip's readings.
impl SurfaceStrip {
    /// How many rails the strip is ruled by - one more than the number of quad columns.
    pub fn rail_count(&self) -> i32 {
        self.rail_points.len() as i32
    }

    /// The unit direction the rails run in, which is the chain's own walk direction.
    pub fn along(&self) -> Vector3 {
        self.along
    }

    /// The point on one rail at `param` along the edge from its start.
    pub fn point_on(&self, rail: i32, param: f64) -> Vector3 {
        self.rail_points[rail as usize] + (self.along * param)
    }

    /// How far along the edge `point` stands, measured from the edge's start.
    pub fn param_of(&self, point: Vector3) -> f64 {
        (point - self.origin).dot(self.along)
    }

    /// Where one rail crosses the plane through `plane_point` with `plane_normal`, as a parameter
    /// along the edge. `None` on a plane the rails run along, which is a mitre or an end face the
    /// strip cannot be cut at.
    #[allow(clippy::neg_cmp_op_on_partial_ord)]
    pub fn try_cross(&self, rail: i32, plane_point: Vector3, plane_normal: Vector3) -> Option<f64> {
        let slope = plane_normal.dot(self.along);

        if !(slope.abs() > 1e-9) {
            return None;
        }

        let param = plane_normal.dot(plane_point - self.rail_points[rail as usize]) / slope;

        param.is_finite().then_some(param)
    }
}
