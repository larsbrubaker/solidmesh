//! THE READING of a MIXED corner - two chain edges sharing one smooth side but meeting it at
//! different wedges, convex on one side of the vertex and concave on the other - and the measured
//! reason it is NOT the mitre [`section_corner`] draws. It reads each canal's axis with
//! [`section_corner::the_axis_at`]; nothing is built here yet - what
//! is here is what the next reader needs so as not to measure it twice.
//!
//! **The corner read.** The handover corner of "Every Chamfer Works" at `(-10.7723, 28, 0)`, at the
//! file's own size `r = 3.7561607778139794`: the chain ARRIVES on the bottom cap's rim chord (floor
//! against barrel, a square 90 degree CONVEX fold) and LEAVES straight up the seam (the cube's
//! `y = 28` wall against the SAME barrel facet, 76.5 degrees CONCAVE). Shared side: the barrel. The
//! other two sides - the floor and the wall - are square to each other and fold along the cube's
//! own bottom edge, which runs `(-1, 0, 0)` out of the vertex. Every number below is asserted in
//! `SectionHandoverCornerTests.TheTwoCanalsAxesMeetAndTheMitreIsNowhereNearTheCorner`.
//!
//! **The two canals are read by ONE rule, not two.** `SectionCorner.TheAxisAt`'s
//! `C = V - r (nL + nR) / (1 + c)` serves the concave station as it serves the convex one - there
//! is no sign to flip, which is what `SectionBuilder`'s own remark on the sign of the centre
//! already says. Measured: the convex canal's axis stands at `(-7.1199, 28.8769, 3.7562)`, the
//! concave one's at `(-7.8112, 31.7562, 0)`, and BOTH stand exactly `-r` off the shared barrel
//! facet's plane (-3.7561607778139816 and -3.756160777813981) and exactly `-r` off their own other
//! side. So the ball of each really is tangent to the three faces it should be; the concave one's
//! centre is simply on the far side of its crease.
//!
//! **THE AXES MEET - the intersection IS planar.** Both axes lie in the one plane standing a radius
//! inside the shared facet, so they cannot be skew, and they are not: `|skew| = 4.4e-16`, crossing
//! at 90 degrees at `(-7.8112, 31.7562, 3.7562)`. Two equal-radius cylinders whose axes meet cut
//! each other in TWO plane ellipses, one in each bisector plane of the axes, so the mixed corner's
//! two canals do meet in a plane curve - exactly as `SectionCorner.TryTheMitrePlane` assumes for
//! unequal wedges on a flat shared side.
//!
//! **And that plane is nowhere near the corner, which is why this is not a mitre.** The meet
//! stands 6.0816 mm from the vertex - 1.62 radii - and the mitre plane through it, normal
//! `(-0.1651, 0.6876, 0.7071)`, leaves the VERTEX 4.7498 mm off it, 1.26 radii. Slid onto that plane
//! the arriving edge's profile travels 6.7173 mm along `d0`, and the arriving edge is 1.12115 mm
//! long: the mitre would consume six chords of the cap's rim before it landed. The leaving profile
//! travels the same 6.7173 mm up a 20 mm seam. A section that far from its own station is not a
//! corner section - it is the far-off place where a carving fillet and a filling one happen to
//! cross - so the plane is real geometry and useless as a corner, and `SectionCorner` goes on
//! refusing unequal wedges.
//!
//! **What DOES meet at one point is the shared side.** The two canals' tangency lines on the
//! barrel meet exactly, at `(-11.4636, 30.8793, 3.7562)` - a radius up the seam and 2.9611 round the
//! rim from the vertex - and that point lies ON the mitre plane to 0. So the shared side has a
//! genuine corner point; it just is not the vertex.
//!
//! **The far ends do NOT meet, and that is the missing patch.** Each canal ends its other-side
//! contact on the cube's bottom fold, and at DIFFERENT points: the convex canal's floor tangency
//! line crosses the fold 3.8629 mm from the vertex (its setback is the full `r`, the fold leaning
//! against it), the concave canal's wall tangency line at 2.9611 mm (`r / tan(wedge / 2)` for a
//! 76.5 degree wedge). They stand 0.9018 mm apart - 0.24 of a radius, 240 times what
//! `SectionCorner.ProfilesMeetToWithinThisMuchOfTheRadius` allows a shared curve to wander - so
//! there is no one point to end both strips at. It is the same debt `SectionCorner`'s remarks name
//! at the pierced face (0.0278 mm there), one order of magnitude larger: a PATCH of the floor and
//! the wall between two tangencies, which nothing in the sections draws.
//!
//! **How legacy answers instead, READ OFF ITS OWN BUILD - and it is not the dual canal.** An
//! earlier reading here guessed `SurfaceMixedCornerDualCanal`, which turns ONE void ball past the
//! end of the cut's strip until its wall contact reaches a crease. That canal IS readable at this
//! corner - cut strip ending at 90 degrees, dying at 150 on `(2.7769, 28, 0)` - and legacy REFUSES
//! it: `SurfaceMixedCornerWhichCanal.TheCornerTakesTheDual` is false, because `TheCapLeans` is false
//! and the part carries no edge the way the dual would travel. None of the dual's rails is in the
//! published mesh (0.27 to 3.81 mm off the nearest vertex). **The dual is a LEANING corner's
//! answer; this corner does not lean.**
//!
//! **What legacy DOES draw: the cap-riding corner, and past it a run-out of 60 degrees.** The
//! corner itself is the ordinary cap-riding torus, its CUT not the chord touching the vertex but
//! the one two back (`HandsOverAtAMitre`), its BEAD the concave seam, turn 67.5 degrees. Past the
//! bead's last turn `SurfaceMixedCornerRunOut` adds MORE STATIONS OF THE SAME TURN on the same centre
//! circle - nothing new is constructed - each trimmed where its arc crosses the unpicked wall's
//! plane. The axis stands a radius off that wall, so at `phi` past the last turn the wall stands
//! `r / cos(phi)` out of it against the arc's `2r`, and the column closes at `phi = 60` degrees
//! exactly, whatever the wedge, on one point `r sqrt 3 = 6.5059` mm along the fold -
//! `(-17.2782, 28, 0)` - past which the fold is left sharp. The fold it dies on is the one
//! [`try_where_each_ends_on_the_fold`] measures, running `(-1, 0, 0)` for 10.2277 mm; the run-out
//! reaches well past BOTH tangency crossings (3.8629 and 2.9611), so the 0.9018 mm gap is not what
//! legacy fills there - it simply keeps turning over it.
//!
//! **So the shape a section owes here is a CORNER'S turn continued, not a run's stations
//! continued.** A run-out section is not a station marched along either edge past the vertex: it is
//! the corner's own canal turned further about the bead's axis and cut by an unpicked wall. It
//! cannot be built before a corner section for unequal wedges exists to continue - which is why
//! nothing is built here yet.

// The negated comparisons (`!(room < size)`, `!(x > 0)`) are written that way on purpose: they
// are what decides which way a NaN reading falls.
#![allow(clippy::neg_cmp_op_on_partial_ord)]

use solidmesh_math::{math_helper, Vector3};

use super::section_corner;
use crate::topology::{ChainEdge, FaceGroup};

/// How nearly two face normals must agree to be read as one smooth surface - the same tolerance
/// [`section_corner`] reads a shared side at, and deliberately not a second angle.
fn smooth_surface_dot() -> f64 {
    libm::cos(math_helper::degrees_to_radians(
        FaceGroup::DEFAULT_CURVE_TOLERANCE_DEGREES,
    ))
}

/// One mixed corner's sides: each edge's normal on the side the two share, and on its own other
/// one.
#[derive(Debug, Clone, Copy, PartialEq, Default)]
pub struct MixedCornerSides {
    pub shared_of_arriving: Vector3,
    pub shared_of_leaving: Vector3,
    pub other_of_arriving: Vector3,
    pub other_of_leaving: Vector3,
}

/// Whether the two edges share one smooth side - the thing that makes a
/// turn a MIXED corner rather than something with no section at all - handing back each edge's
/// normal on that side and on its own other one.
pub fn try_read_the_shared_side(
    arriving: &ChainEdge,
    leaving: &ChainEdge,
) -> Option<MixedCornerSides> {
    let left0 = arriving.left_normal().get_normal();
    let right0 = arriving.right_normal().get_normal();
    let left1 = leaving.left_normal().get_normal();
    let right1 = leaving.right_normal().get_normal();
    let smooth = smooth_surface_dot();

    if right0.dot(right1) >= smooth {
        return Some(MixedCornerSides {
            shared_of_arriving: right0,
            shared_of_leaving: right1,
            other_of_arriving: left0,
            other_of_leaving: left1,
        });
    }

    if left0.dot(left1) >= smooth {
        return Some(MixedCornerSides {
            shared_of_arriving: left0,
            shared_of_leaving: left1,
            other_of_arriving: right0,
            other_of_leaving: right1,
        });
    }

    None
}

/// How far the two canals' axes stand out of one another's plane at the
/// vertex - zero when they meet, which is what makes their intersection a PLANE curve. Both axes
/// stand a radius inside the shared side, so on a genuinely shared side this is always zero to
/// float noise; it is measured rather than assumed because the assumption is the whole reason the
/// mitre plane exists.
pub fn how_skew_the_axes_are(arriving: &ChainEdge, leaving: &ChainEdge, radius: f64) -> f64 {
    let station = leaving.from();
    let along0 = arriving.direction().get_normal();
    let along1 = leaving.direction().get_normal();
    let across = along0.cross(along1);

    if !(across.length_squared() > 0.0) {
        // Parallel axes are either the same line or never meet, and neither is a corner.
        return f64::INFINITY;
    }

    let from0 = section_corner::the_axis_at(arriving, station, radius);
    let from1 = section_corner::the_axis_at(leaving, station, radius);

    (from1 - from0).dot(across.get_normal()).abs()
}

/// Where the two canals' contact lines on the SHARED side
/// meet - the one point of that side both strips end at - and how far apart the two lines pass
/// there. `None` where the two lines do not actually meet, which cannot happen while
/// [`how_skew_the_axes_are`] reads zero.
pub fn try_where_the_shared_side_boundaries_meet(
    arriving: &ChainEdge,
    leaving: &ChainEdge,
    radius: f64,
) -> Option<(Vector3, f64)> {
    let sides = try_read_the_shared_side(arriving, leaving)?;

    let station = leaving.from();
    let contact0 = section_corner::the_axis_at(arriving, station, radius)
        + (sides.shared_of_arriving * radius);
    let contact1 =
        section_corner::the_axis_at(leaving, station, radius) + (sides.shared_of_leaving * radius);

    let (on0, on1) = try_the_closest_approach(
        contact0,
        arriving.direction().get_normal(),
        contact1,
        leaving.direction().get_normal(),
    )?;

    Some(((on0 + on1) / 2.0, (on1 - on0).length()))
}

/// How far along the FOLD - the crease where each edge's own other
/// side meets the other's, running out of the vertex - each canal ends its contact on that side,
/// as `(fold, arriving_reach, leaving_reach)`. The two answers differ whenever the wedges do, and
/// the difference is the patch of wall a mixed corner still owes (0.9018 mm on the handover corner,
/// against a radius of 3.7562).
pub fn try_where_each_ends_on_the_fold(
    arriving: &ChainEdge,
    leaving: &ChainEdge,
    radius: f64,
) -> Option<(Vector3, f64, f64)> {
    let sides = try_read_the_shared_side(arriving, leaving)?;
    let other0 = sides.other_of_arriving;
    let other1 = sides.other_of_leaving;

    let station = leaving.from();
    let along = other0.cross(other1);

    if !(along.length_squared() > 0.0) {
        // The two other sides lie in one plane, so they fold nowhere and there is no crease to end
        // on.
        return None;
    }

    let fold = along.get_normal();

    let contact0 = section_corner::the_axis_at(arriving, station, radius) + (other0 * radius);
    let contact1 = section_corner::the_axis_at(leaving, station, radius) + (other1 * radius);

    let (_, meet0) =
        try_the_closest_approach(contact0, arriving.direction().get_normal(), station, fold)?;
    let (_, meet1) =
        try_the_closest_approach(contact1, leaving.direction().get_normal(), station, fold)?;

    Some((
        fold,
        (meet0 - station).dot(fold),
        (meet1 - station).dot(fold),
    ))
}

/// The two points, one on each line, where two non-parallel lines come closest.
fn try_the_closest_approach(
    from0: Vector3,
    along0: Vector3,
    from1: Vector3,
    along1: Vector3,
) -> Option<(Vector3, Vector3)> {
    let lean = along0.dot(along1);
    let denominator = 1.0 - (lean * lean);

    if !(denominator > 1e-9) {
        return None;
    }

    let between = from1 - from0;
    let step0 = (between.dot(along0) - (lean * between.dot(along1))) / denominator;
    let step1 = ((lean * between.dot(along0)) - between.dot(along1)) / denominator;

    Some((from0 + (along0 * step0), from1 + (along1 * step1)))
}
