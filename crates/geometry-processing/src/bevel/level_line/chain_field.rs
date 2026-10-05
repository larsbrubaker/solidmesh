//! The chain a bevel is built round, as a distance field ([`ChainField`]), and the scope that
//! makes fields built inside it read given joints as smooth ([`ChainFieldSmoothJointsScope`]).
//! Ports the type of `Bevel/LevelLine/ChainField.cs` with the fields of every partial
//! (`ChainField.EndClip.cs`, `.EndRay.cs`, `.Mitre.cs`, `.Queries.cs`, `.SmoothJointSpans.cs`,
//! `.Swallow.cs`); the nested records live in the Rust files of the partials that declare them.
//! Here too: the closest-point closed forms on a segment and a triangle, the smooth-joint test and
//! the sharp-joint finder. Construction, the queries and the `ChainDistanceField` impl land with
//! their method packages.

use std::cell::OnceCell;
use std::collections::HashSet;
use std::rc::Rc;

use vector_math::Vector3;

use super::{ChainFieldEndClip, ChainFieldExtension, EndRayBound, SegmentNode};

/// The greatest end plane's reading at a point, as the convex cell bound takes it (C#
/// `Func<int, Vector3, double>`: chain index, point).
pub type ChainFieldEndPlaneReader = Rc<dyn Fn(i32, Vector3) -> f64>;

/// C# `ChainField`: the chain a bevel is built round, as a distance field - a polyline carrying a
/// setback at each of its points, able to say for any point in space how far inside or outside
/// the setback boundary it lies, and which point of the chain put it there.
///
/// This is the whole of step (a) of the level-line construction. Nothing here knows about the
/// mesh: band vertices are handed in one at a time and the answer is exact, so no point is ever
/// classified onto a face, a fold or a vertex. That is the property the rest of the construction
/// is built on.
///
/// **The field is the union of balls.** `Value(q) = min over ALL chain points c of
/// (|q - c| - setback(c))` - the signed distance to the union of the balls of radius `setback(c)`
/// centred on the chain. Each term is 1-Lipschitz in q, so their minimum is EXACTLY 1-Lipschitz:
/// every bound is "a value read at a sample, plus the distance to it", with no setback-spread term
/// to carry. The older reading (the distance to the NEAREST chain point less THAT point's setback)
/// jumps where the nearest point jumps, so it was not Lipschitz and its bounded searches could
/// not close. With ONE setback along the chain the two readings are identical.
///
/// **The tie rule.** A point in the outer quadrant of a corner is equidistant from two segments.
/// Ties go to the LOWER parameter, which falls out of walking the segments in parameter order and
/// only ever taking a strictly smaller value. It matters that this is a rule and not a tolerance:
/// the whole fan of band vertices round a corner must read the same chain parameter, or the fill's
/// two boundary loops do not pair up.
pub struct ChainField {
    /// The chain's points in order (a closed chain does not repeat its first point).
    pub points: Vec<Vector3>,
    /// The setback asked at each point.
    pub setbacks: Vec<f64>,
    /// One parameter per point, plus one more for the closing point of a closed chain - so both
    /// ends of every segment read out of the same array and no caller special cases the wrap.
    pub parameter_at_point: Vec<f64>,
    /// The segments in an index-ordered binary box tree, so a query visits only the segments that
    /// could beat what it has already found. Built once; see [`SegmentNode`] for why the order
    /// matters.
    pub nodes: Vec<SegmentNode>,
    /// This field's chain point indices that are never mitred, whatever their turn; `None` for
    /// none.
    pub smooth_joints: Option<HashSet<i32>>,
    /// The turn, in degrees, past which a chain vertex is mitred rather than rounded (see
    /// `chain_field_mitre`); +infinity when no vertex is.
    pub sharp_turn_degrees: f64,
    /// Which ends of an open chain carry an END RAY (see `chain_field_end_ray`), as (first, last):
    /// the chain's line run on past that end, read with a parameter beyond it. Never set on a
    /// closed chain.
    pub end_rays: (bool, bool),
    /// How far each end's ray runs, and the end face's plane it runs to (see [`EndRayBound`]), as
    /// (first, last); `None` at an end whose ray is unbounded - the box diagonal plus four
    /// setbacks - or that has none.
    pub end_ray_bounds: (Option<EndRayBound>, Option<EndRayBound>),
    /// True when a segment runs from the chain's last point back to its first.
    pub is_closed: bool,
    /// Total arc length of the chain; the parameter runs 0..this.
    pub length: f64,
    /// The smallest setback any point of the chain asks for.
    pub min_setback: f64,
    /// The largest setback any point of the chain asks for - at worst, the room a build needs.
    pub max_setback: f64,
    /// Per segment, the rays it also answers for (C# `extensions`); `None` when the chain has no
    /// sharp turn, and `None` for a segment with none.
    pub extensions: Option<Vec<Option<Vec<ChainFieldExtension>>>>,
    /// The end planes the field is clipped by; `None` when no end's ray is bounded.
    pub end_clips: Option<Vec<ChainFieldEndClip>>,
    /// The end-plane reader handed to the convex cell bound, made once on first ask (C#
    /// `endPlaneReader ??=`).
    pub end_plane_reader: OnceCell<ChainFieldEndPlaneReader>,
    /// The chain's joints sharper than [`ChainField::sharp_turn_degrees`], each as its parameter
    /// and the setback asked there, in parameter order; found once on first ask (C# `sharpJoints
    /// ??=`).
    pub sharp_joints: OnceCell<Vec<(f64, f64)>>,
}

impl std::fmt::Debug for ChainField {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        // The end-plane reader has no Debug; leave it and the caches out.
        f.debug_struct("ChainField")
            .field("points", &self.points)
            .field("setbacks", &self.setbacks)
            .field("is_closed", &self.is_closed)
            .field("length", &self.length)
            .field("sharp_turn_degrees", &self.sharp_turn_degrees)
            .field("end_rays", &self.end_rays)
            .field("end_ray_bounds", &self.end_ray_bounds)
            .finish_non_exhaustive()
    }
}

/// C# `ChainField.SmoothJointsScope`: holds the smooth-joint positions that were in scope before
/// `ChainField::smooth_joints_while`, and puts them back when the scope ends.
///
/// The scope exists because rounds first's pass B (`BandGap.BuildRoundsFirst`) joins crease runs
/// over pass A's round as that round's section, a sampled smooth curve, and near the ellipse's tip
/// its samples turn past the pick's sharp-turn threshold - read as mitres, the strip kept repeated
/// stations there and the clip refused the self-loop rail edges. A scope rather than a parameter,
/// because the field is built in a dozen places under the build, all of which must read the same
/// joints. The set holds each position as its component bits with -0.0 read as 0.0 (C#
/// `Vector3` equality on the crease's own exact vertex positions).
#[derive(Debug, Clone, PartialEq, Eq, Default)]
pub struct ChainFieldSmoothJointsScope {
    /// The joint positions in scope before this one began; `None` for none.
    pub previous: Option<HashSet<[u64; 3]>>,
}

impl ChainField {
    /// Whether chain point `k` is a smooth joint (built inside the smooth-joints scope).
    pub fn is_smooth_joint(&self, k: i32) -> bool {
        self.smooth_joints
            .as_ref()
            .is_some_and(|joints| joints.contains(&k))
    }

    /// The chain's points in order (a closed chain does not repeat its first point).
    pub fn points(&self) -> &[Vector3] {
        &self.points
    }

    /// The closest point of a segment to a point.
    pub fn closest_on_segment(point: Vector3, a: Vector3, b: Vector3) -> Vector3 {
        let along = b - a;
        let length_squared = along.length_squared();
        if length_squared <= 0.0 {
            return a;
        }

        let mut t = (point - a).dot(along) / length_squared;
        t = t.clamp(0.0, 1.0);
        a + (along * t)
    }

    /// The closest point of a triangle - interior included - to a point, by the usual barycentric
    /// region test.
    pub fn closest_on_triangle(point: Vector3, a: Vector3, b: Vector3, c: Vector3) -> Vector3 {
        let ab = b - a;
        let ac = c - a;
        let ap = point - a;
        let d1 = ab.dot(ap);
        let d2 = ac.dot(ap);
        if d1 <= 0.0 && d2 <= 0.0 {
            return a;
        }

        let bp = point - b;
        let d3 = ab.dot(bp);
        let d4 = ac.dot(bp);
        if d3 >= 0.0 && d4 <= d3 {
            return b;
        }

        let vc = (d1 * d4) - (d3 * d2);
        if vc <= 0.0 && d1 >= 0.0 && d3 <= 0.0 {
            return a + (ab * (d1 / (d1 - d3)));
        }

        let cp = point - c;
        let d5 = ab.dot(cp);
        let d6 = ac.dot(cp);
        if d6 >= 0.0 && d5 <= d6 {
            return c;
        }

        let vb = (d5 * d2) - (d1 * d6);
        if vb <= 0.0 && d2 >= 0.0 && d6 <= 0.0 {
            return a + (ac * (d2 / (d2 - d6)));
        }

        let va = (d3 * d6) - (d5 * d4);
        if va <= 0.0 && (d4 - d3) >= 0.0 && (d5 - d6) >= 0.0 {
            return b + ((c - b) * ((d4 - d3) / ((d4 - d3) + (d5 - d6))));
        }

        let denom = 1.0 / (va + vb + vc);
        a + (ab * (vb * denom)) + (ac * (vc * denom))
    }

    /// How many segments the chain has: one fewer than its parameters.
    pub fn segment_count(&self) -> i32 {
        self.parameter_at_point.len() as i32 - 1
    }

    /// The chain's joints sharper than [`ChainField::sharp_turn_degrees`] (its segments'
    /// directions dotting below the threshold's cosine), each as its parameter and the setback
    /// asked there, in parameter order; none under an infinite threshold. A closed chain's first
    /// point is a joint at 0. The finder behind the `sharp_joints` cache.
    pub fn find_sharp_joints(&self) -> Vec<(f64, f64)> {
        let mut joints = Vec::new();
        if !self.sharp_turn_degrees.is_finite() {
            return joints;
        }

        let sharp = (self.sharp_turn_degrees * std::f64::consts::PI / 180.0).cos();
        let n = self.points.len() as i32;
        let count = self.segment_count();
        let direction =
            |k: i32| (self.points[((k + 1) % n) as usize] - self.points[k as usize]).get_normal();
        let (first, end) = if self.is_closed { (0, n) } else { (1, n - 1) };
        for joint in first..end {
            let before = (joint - 1 + count) % count;
            if direction(before).dot(direction(joint)) < sharp && !self.is_smooth_joint(joint) {
                joints.push((
                    self.parameter_at_point[joint as usize],
                    self.setbacks[joint as usize],
                ));
            }
        }

        joints
    }
}
