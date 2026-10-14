//! The cross section one edge's bevel is cut with, and the frame it is drawn in: on a convex edge
//! the profile of the material that has to go, on a concave edge the mirror of the same
//! construction, which is the bead that is unioned on. `BevelChainSweep` is what turns these into a
//! solid. Its `try_*` functions return `Result<_, BevelAdvisory>`: the built section, or the reason
//! there is none.
//!
//! The size is dimensioned the way CAD dimensions it - a chamfer's setback is measured along each
//! face, a fillet's radius is the rolling ball - and the round is the true tangent arc (tangent
//! points at `r / tan(theta / 2)`, centre at `r / sin(theta / 2)` along the bisector), so a fillet
//! really is the surface a ball of that radius would leave.
//!
//! The cross section is drawn in a 2D frame taken across the edge, where +y is the outward
//! bisector of the two [`try_get_profile_normals`] and the origin is the edge itself. The material
//! near the edge lives below both face lines, so the tool straddles the edge: out past one face
//! along its normal, over the point where the two cleared faces meet, back down past the other
//! face, and home along the profile. That turn over the top is load bearing on a sharp edge and is
//! derived where it is built.
//!
//! There is no straight extrude here: it ran the profile through Clipper's 0.001 mm integer grid,
//! so its cut plane sat up to a micron off the analytic one while the sweep's is exact
//! (`BevelToolPlaneAccuracyTests`).

use vector_math::math_helper::{degrees_to_radians, radians_to_degrees};
use vector_math::{Vector2, Vector3};

use super::{BevelAdvisory, BevelAdvisoryArg, BevelAdvisoryCode};
use crate::topology::{EdgeConvexity, FeatureEdge};

/// How square to the edge the two faces have to be before the frame means anything. Below this
/// the faces have folded back on each other or lie flat, and there is no corner to cut.
///
/// Shared rather than private because it is the same question wherever a profile is inscribed in
/// a wedge - `BevelMixedBlends` draws one per station in the wedge its own miter plane presents
/// and is refused on these bounds, so a corner this builder would turn down cannot be reached
/// round the back by a blend.
pub const MINIMUM_INTERIOR_ANGLE_DEGREES: f64 = 1.0;

/// See [`MINIMUM_INTERIOR_ANGLE_DEGREES`].
pub const MAXIMUM_INTERIOR_ANGLE_DEGREES: f64 = 179.0;

/// How far past the bead's own tangent points the void wedge reaches, as a proportion of them -
/// just enough that the arc lies strictly inside the wedge rather than on its boundary. Derived,
/// and argued for being this small rather than larger, on [`try_build_void_wedge_cross_section`].
/// Shared with `BevelVoidWedgeSectionTests`, so the bound moves with the margin.
pub const CONTAINMENT_MARGIN: f64 = 1.05;

/// How far INSIDE the material a swept void wedge's two lateral walls are buried, as a fraction
/// of the wedge's own reach, so that neither wall lies in the crease face it runs along. Why, and
/// why a thousandth rather than a float hair, is on [`try_build_void_wedge_cross_section`]; the
/// same fraction, against the same hazard, sets the corner void's apex back in
/// `BevelCornerPatchMesher`.
pub const WALLS_CLEAR_THE_FACES_BY: f64 = 1e-3;

/// How far the rolling-ball TUBE's section stands outside the ball itself, as a fraction of one
/// chord's sagitta - the margin that keeps a tube's facet planes off the partner cutter's own arc
/// facets, which are the same planes when the two runs meet at a right angle. Argued on
/// [`try_build_ball_cross_section`].
pub const TUBE_STAND_OFF_FRACTION: f64 = 0.25;

/// How far past each end of an edge the tool runs, so the cut does not stop short of the
/// corners. The chain sweep leaves an open chain's two free ends exactly this far over too (plan
/// 5.2's overshoot).
///
/// It is a function of the SIZE and of nothing else - the run it is swept along does not appear -
/// and that is deliberate at a corner the tool has to reach past. It is also why `BevelChainEnds`
/// bounds it at an uncapped end shorter than itself.
pub fn overshoot(size: f64, tangent_distance: f64) -> f64 {
    // NaN propagates, which f64::max would not do: a NaN size must not read as a real overshoot.
    if size.is_nan() || tangent_distance.is_nan() {
        return f64::NAN;
    }
    size.max(tangent_distance)
}

/// How far back along each face the bevel reaches, without building the cross section - the
/// closed form of the tangent distance [`try_build_cross_section`] hands back.
///
/// `size` is the fillet radius or the chamfer setback, `wedge_angle_degrees` the wedge the profile
/// is drawn in ([`profile_wedge_angle_degrees`]), `round_profile` true for a fillet's tangent arc.
/// A chamfer's setback IS the size. A fillet's is `radius / tan(theta / 2)`, the same line
/// `TryMeasureCorner` computes from the angle it measures out of the frame; the two agree because
/// they are the same angle read two ways (`BevelVoidWedgeSectionTests` asserts it).
pub fn tangent_distance(size: f64, wedge_angle_degrees: f64, round_profile: bool) -> f64 {
    if !round_profile {
        return size;
    }

    let tangent_of_half_angle = (degrees_to_radians(wedge_angle_degrees) / 2.0).tan();

    // A wedge folded shut has no corner to reach back along. Infinity is the honest answer and it
    // stays usable where it matters: BevelChainEnds only ever COMPARES this against half a
    // segment, and infinity compares the right way round. try_build_cross_section refuses the
    // corner outright in any case, so no tool is ever swept on it.
    if tangent_of_half_angle > 0.0 {
        size / tangent_of_half_angle
    } else {
        f64::INFINITY
    }
}

/// Refuses a size that would remove nothing.
// `!(x > 0)` is true for NaN too, which the rewritten comparison would not be.
#[allow(clippy::neg_cmp_op_on_partial_ord)]
pub fn try_validate_size(size: f64) -> Result<(), BevelAdvisory> {
    if !(size > 0.0) {
        return Err(BevelAdvisory::new(
            BevelAdvisoryCode::SizeRemovesNothing,
            vec![BevelAdvisoryArg::Double(size)],
        ));
    }

    Ok(())
}

/// The angle of the wedge the profile is drawn in, in degrees: the MATERIAL wedge on a convex
/// edge (the dihedral) and the VOID wedge on a concave one (360 less the dihedral). It is
/// `180 - |turn|` on both, which is why every formula below is written once.
pub fn profile_wedge_angle_degrees(edge: &FeatureEdge) -> f64 {
    180.0 - edge.turn_angle_degrees.abs()
}

/// How far the cross section reaches from the edge at its furthest point, without building it -
/// the bound `BevelChainEnds` sizes a capping plane's ring slide against.
///
/// Read straight off [`try_build_cross_section`], where every point of the section is one of
/// three things. The cleared apex sits `clearance / sin(halfAngle)` up the bisector, and the
/// clearance IS the size. Each cleared corner sits at `hypot(tangentDistance, clearance)`, which
/// for a fillet is that same `size / sin(halfAngle)` and for a chamfer is `sqrt(2) size`. And the
/// profile return lies inside the triangle those corners span. So the larger of the two closed
/// forms bounds both profiles.
// `!(x > 0)` is true for NaN too, which the rewritten comparison would not be.
#[allow(clippy::neg_cmp_op_on_partial_ord)]
pub fn cross_section_reach(size: f64, wedge_angle_degrees: f64) -> f64 {
    let sine_of_half_angle = (degrees_to_radians(wedge_angle_degrees) / 2.0).sin();

    if !(sine_of_half_angle > 0.0) {
        // A wedge that has folded shut has no section to measure; try_build_cross_section refuses
        // it outright, and every caller here reads an unbounded reach as "trust no plane".
        return f64::INFINITY;
    }

    size * 2.0_f64.sqrt().max(1.0 / sine_of_half_angle)
}

/// The two normals the cross section is drawn from: the faces' own outward normals on a convex
/// edge, and both of them NEGATED on a concave one. That single sign flip is the whole of the
/// plan's mirrored concave profile (5.1).
///
/// **Why one flip is all it takes.** Everything below is drawn in the frame [`try_build_frame`]
/// builds, whose +y is the bisector of the two normals it is handed and whose origin is the edge.
/// Negating both normals turns that frame 180 degrees about the edge - x and y both flip, z does
/// not, so it stays right handed and every winding, triangulation and miter below is untouched -
/// and +y now points along the bisector of the VOID wedge a concave edge opens instead of the
/// material wedge a convex one closes. In that turned frame the construction reads term for term
/// as it did, but about the void: the arc centre lands where a ball rolling INSIDE the corner
/// sits, and the "clearance" walls run in past each face into solid material - a closed bead whose
/// curved face is the arc and whose back faces are buried inside the part. Only which boolean
/// takes it differs (`BevelFeatureMeshBuilder` subtracts a cut and unions a bead).
///
/// A flat edge is refused: there is no wedge either way, so there is nothing to draw a profile in.
///
/// **Why this and not Minkowski.** A ball swept along the chain hands back a TUBE, which is only a
/// mask; the bead is still the region between the part and the swept ball, so it needs either
/// this construction anyway or a morphological closing of the whole part - seconds per boss
/// against arithmetic plus one boolean, 12% over the analytic bead, and every other concave edge
/// rounded too. Minkowski keeps its place as the select-all oracle and nothing more.
pub fn try_get_profile_normals(
    convexity: EdgeConvexity,
    face_normal0: Vector3,
    face_normal1: Vector3,
) -> Result<(Vector3, Vector3), BevelAdvisory> {
    match convexity {
        EdgeConvexity::Convex => Ok((face_normal0, face_normal1)),
        EdgeConvexity::Concave => Ok((-face_normal0, -face_normal1)),
        _ => Err(BevelAdvisory::new(BevelAdvisoryCode::FacesCoplanar, vec![])),
    }
}

/// The frame the cross section is drawn in, as `(x_axis, y_axis, z_axis)`: z runs along the edge,
/// y points out along the bisector of the two profile normals, x runs across. y is rebuilt from x
/// and z so the frame stays orthogonal even when the bisector is not exactly square to the edge.
///
/// The chain sweep builds one of these per segment from that segment's OWN face pair, which is
/// what lets a tool follow a tessellated wall where the two surfaces turn from one edge to the
/// next.
pub fn try_build_frame(
    normal0: Vector3,
    normal1: Vector3,
    direction: Vector3,
) -> Result<(Vector3, Vector3, Vector3), BevelAdvisory> {
    let z_axis = direction.get_normal();

    let bevel_direction = (normal0 + normal1).get_normal();
    let x_axis = bevel_direction.cross(z_axis);

    if x_axis.length() < 1e-9 {
        return Err(BevelAdvisory::new(BevelAdvisoryCode::FacesFoldBack, vec![]));
    }

    let x_axis = x_axis.get_normal();
    let y_axis = z_axis.cross(x_axis).get_normal();

    Ok((x_axis, y_axis, z_axis))
}

/// The closed cross section of one edge's bevel, in that edge's 2D frame, and the tangent
/// distance (how far back along each face the cut reaches, which is also how far the tool has to
/// overshoot an open end). Walked in the order the polygon closes: the tangent point on face 0,
/// out past face 0, over the cleared apex, back down past face 1, the tangent point on face 1, and
/// then the profile itself back to the start.
///
/// On a concave edge the same walk runs in the mirrored frame - see [`try_get_profile_normals`] -
/// so "out past face 0" is INTO the material and the cleared apex is the buried back of a bead.
///
/// `back_depth` is how far past each face the cleared walls may run, when that has to be less than
/// the size; zero means the size itself.
#[allow(clippy::too_many_arguments)]
pub fn try_build_cross_section(
    normal0: Vector3,
    normal1: Vector3,
    x_axis: Vector3,
    y_axis: Vector3,
    z_axis: Vector3,
    size: f64,
    segments: i32,
    round_profile: bool,
    back_depth: f64,
) -> Result<(Vec<Vector2>, f64), BevelAdvisory> {
    let (corner, tangent_distance) = try_measure_corner(
        normal0,
        normal1,
        x_axis,
        y_axis,
        z_axis,
        size,
        round_profile,
    )?;

    let half_angle = corner.half_angle;
    let radius = if round_profile { size } else { 0.0 };

    let arc_segments = segments.max(1);

    let cut_start = corner.face0_direction * tangent_distance;
    let cut_end = corner.face1_direction * tangent_distance;

    // How far the tool reaches out past each surface. It only has to clear the part; the walls run
    // along the face normals, so they leave the material the moment they leave the cut.
    //
    // One size, unless the caller asked for a shallower back. It does that only when the chain
    // this section is swept along turns tightly enough that a back a whole size deep would fold
    // through itself on the inside of the turn - see BevelChainSweep.TryCreate, which measures the
    // fold and retries rather than refusing the tool. Never DEEPER than the size.
    let clearance = if back_depth > 0.0 {
        // NaN cannot reach here (back_depth > 0 is false for NaN), so f64::min is safe.
        back_depth.min(size)
    } else {
        size
    };

    // Where the two cleared faces meet, and where the far side of the tool turns rather than
    // cutting straight across. +y is exactly the outward bisector of the two faces, so each normal
    // makes an angle of half_angle with it, and a plane one clearance out from either face crosses
    // the bisector here.
    //
    // Turning AROUND this point rather than running a chord between the two cleared corners is
    // what keeps the tool covering the edge itself. A chord sits clearance * sin(halfAngle) -
    // tangentDistance * cos(halfAngle) above the edge, which goes NEGATIVE as the corner sharpens:
    // the chord then runs through solid material, the tool becomes a band that misses the tip
    // entirely, and the subtraction severs the tip off as a loose sliver with nothing reported
    // (FilletChamferObject3DTests' prism fixtures pin both profiles).
    //
    // The turning point is above both cleared corners at every angle - the gap is
    // clearance * cos^2(halfAngle) / sin(halfAngle) + tangentDistance * cos(halfAngle), both terms
    // positive for half_angle in (0, 90) - which keeps the profile a simple polygon.
    let cleared_apex = Vector2::new(0.0, clearance / half_angle.sin());

    let mut points = Vec::with_capacity(5 + arc_segments as usize);
    points.push(cut_start);
    points.push(cut_start + (corner.face0_out * clearance));
    points.push(cleared_apex);
    points.push(cut_end + (corner.face1_out * clearance));
    points.push(cut_end);

    add_profile_return(
        &mut points,
        cut_start,
        cut_end,
        radius,
        half_angle,
        arc_segments,
        round_profile,
    );

    Ok((points, tangent_distance))
}

/// The triangle that covers the WEDGE the profile is drawn in, in the same 2D frame and walked in
/// the same sense as [`try_build_cross_section`], so one sweep can carry either; with the tangent
/// distance.
///
/// This exists for the concave case, where the wedge is the VOID between the two selected faces,
/// and it is what lets the bead's buried back be trimmed to the wall it is buried in: swept along
/// the same chain with the same frames, miters and caps, `BevelFeatureMeshBuilder` keeps only
/// `bead ∩ (part ∪ wedge)`, so on a wall thinner than the size the back does not union on as a
/// slab.
///
/// It is the triangle the two tangent points span, scaled out from the edge by
/// [`CONTAINMENT_MARGIN`]: everything of the bead in the void is bounded by the ARC, which is
/// tangent to both faces at those points and bows back toward the edge, so the whole of it sits
/// inside their triangle; the margin makes the containment strict.
///
/// **Why it is no bigger.** An over-large wedge fails SILENTLY: `BevelChainSweep`'s fold test
/// refuses it where it is too wide for the curvature, and a refused wedge is a bead added
/// unclipped. Covering the bead's cleared apex (which is never in the void) made it about three
/// times the bead's section; a fourth point pushed out by `1 / cos(halfAngle)` ran away without
/// bound as the wedge flattened. What is left is bounded by the bead's own reach at every angle.
///
/// **Why the two walls are BURIED.** Drawn as the bare triangle each lateral wall lies exactly on
/// a face of the crease, so the clip's `part ∪ wedge` is handed coincident sheets; at the
/// coordinates rebuilds work in, floats put the wall nanometres off the face and the clipped bead
/// carried coincident triangle pairs and zero-area slivers ("UpperLeftInner should bevel.mcx").
/// Insetting each wall by [`WALLS_CLEAR_THE_FACES_BY`] of the reach along its profile normal (into
/// the MATERIAL on a concave edge) makes each wall CROSS its face; the extra sliver is inside
/// `part`, so the kept point set is unchanged. The edge corner moves to where both inset walls
/// meet, `(out0 + out1) * s / (1 + out0 . out1)`, left unclamped.
///
/// **Why a thousandth.** Swept across the kernel suite: at `1e-9` the wall is still coincident
/// (`BevelRecessApexRunOutTests` fails); `1e-6` through `1e-3` is green; a thousandth clears the
/// floor by three decades and matches `BevelCornerPatchMesher.TryBuildVoidCone`'s setback.
///
/// **And why ONE wedge is still drawn flush** (`clear_the_faces` false). Where the bead squares off
/// against a 5.3 corner the keep-set is `wedge ∪ corner void`, whose walls lie ON these same faces,
/// so an inset makes a near-parallel pair rather than removing a coincidence
/// (`BevelTwoBeadCornerClipTests`). That squared wedge alone is swept flush
/// (`BevelSweptSection.VoidWedgeOnTheFaces`); the section tests also measure the bare triangle.
/// `clear_the_faces` is normally true.
// `!(x > 0)` is true for NaN too, which the rewritten comparison would not be.
#[allow(clippy::too_many_arguments, clippy::neg_cmp_op_on_partial_ord)]
pub fn try_build_void_wedge_cross_section(
    normal0: Vector3,
    normal1: Vector3,
    x_axis: Vector3,
    y_axis: Vector3,
    z_axis: Vector3,
    size: f64,
    round_profile: bool,
    clear_the_faces: bool,
) -> Result<(Vec<Vector2>, f64), BevelAdvisory> {
    let (corner, tangent_distance) = try_measure_corner(
        normal0,
        normal1,
        x_axis,
        y_axis,
        z_axis,
        size,
        round_profile,
    )?;

    if !(tangent_distance > 0.0) {
        // The size is validated positive and the angle is gated well inside (0, 180) degrees, so
        // this cannot happen; refusing rather than sweeping it keeps a future change to either
        // bound from producing a wedge that is quietly the wrong shape.
        return Err(BevelAdvisory::new(
            BevelAdvisoryCode::NoWedgeToClipAgainst,
            vec![],
        ));
    }

    let reach = CONTAINMENT_MARGIN * tangent_distance;

    // Both walls buried in the material by a thousandth of the reach - see above. The corner is
    // where the two inset wall lines cross; the denominator only vanishes as the wedge closes to
    // a knife, which try_measure_corner's angle bounds have already refused.
    let wall_stand_off = if clear_the_faces {
        WALLS_CLEAR_THE_FACES_BY * reach
    } else {
        0.0
    };
    let outward = corner.face0_out + corner.face1_out;
    let closing = 1.0 + corner.face0_out.dot(corner.face1_out);
    let edge_corner = if closing > 0.0 {
        outward * (wall_stand_off / closing)
    } else {
        Vector2::ZERO
    };

    let points = vec![
        edge_corner,
        (corner.face0_direction * reach) + (corner.face0_out * wall_stand_off),
        (corner.face1_direction * reach) + (corner.face1_out * wall_stand_off),
    ];

    Ok((points, tangent_distance))
}

/// The BALL the profile is drawn around, in the same 2D frame and walked in the same sense as
/// [`try_build_cross_section`]: a closed polygon of `4 * segments` sides around the arc centre, of
/// the rolling ball's own radius (stood off a hair), with the tangent distance.
///
/// Swept along a chain this is the tube a ball of that radius leaves as it rolls down the edge,
/// for the corner where two convex fillet chains overshoot into each other
/// (`BevelPairCornerTrim`): each cutter has its partner's tube taken out of it before the union.
///
/// **The same centre.** `size / sin(halfAngle)` up the bisector, read from the same
/// `try_measure_corner` the tool reads, so the tube's surface is the tool's own arc surface rather
/// than a second one a rounding off it.
///
/// **4 * segments sides, phased on the arc.** At a right angle the tool's arc step is
/// `90 / segments` degrees, so a full circle of that step is `4 * segments` sides; starting at the
/// tool's own arc start (the tangent point on face 1) puts a ball vertex on every arc vertex.
///
/// **A hair OUTSIDE the ball.** Drawn on the circle itself, the tube's facets lay exactly ON the
/// partner's fillet facets, a coincident face that made the union of the two trimmed cutters
/// self-intersecting at every unequal-dihedral corner. Standing off by
/// [`TUBE_STAND_OFF_FRACTION`] of one chord's sagitta lifts every facet plane clear; a quarter was
/// bracketed against the Onshape control of `CornerShouldBeSmoothRegressionTests` (a whole
/// sagitta stood 0.049 mm proud; a half-step rotation cut 0.098 mm below).
///
/// A chamfer has no ball, so a square profile is refused.
#[allow(clippy::too_many_arguments)]
pub fn try_build_ball_cross_section(
    normal0: Vector3,
    normal1: Vector3,
    x_axis: Vector3,
    y_axis: Vector3,
    z_axis: Vector3,
    size: f64,
    segments: i32,
    round_profile: bool,
) -> Result<(Vec<Vector2>, f64), BevelAdvisory> {
    if !round_profile {
        return Err(BevelAdvisory::new(
            BevelAdvisoryCode::ChamferHasNoBall,
            vec![],
        ));
    }

    let (corner, tangent_distance) = try_measure_corner(
        normal0,
        normal1,
        x_axis,
        y_axis,
        z_axis,
        size,
        round_profile,
    )?;

    let cut_start = corner.face0_direction * tangent_distance;
    let cut_end = corner.face1_direction * tangent_distance;
    let bisector = cut_start + cut_end;

    if bisector.length() < 1e-12 {
        // The two faces have opened to a straight line, which try_measure_corner's angle bounds
        // already refuse; kept so a future change to those bounds cannot produce a centre at the
        // origin.
        return Err(BevelAdvisory::new(
            BevelAdvisoryCode::FacesDoNotCloseCorner,
            vec![],
        ));
    }

    let arc_centre = bisector.get_normal() * (size / corner.half_angle.sin());
    let sides = 4 * segments.max(1);
    let start_angle = (cut_end.y - arc_centre.y).atan2(cut_end.x - arc_centre.x);

    // A quarter of the chord's own sagitta proud of the ball, which is what keeps this surface
    // OFF the partner cutter's arc rather than in it - see above.
    let stand_off = size
        + (TUBE_STAND_OFF_FRACTION
            * size
            * (1.0 - (std::f64::consts::PI / f64::from(sides)).cos()));

    let mut points = Vec::with_capacity(sides as usize);

    for i in 0..sides {
        let angle = start_angle + (2.0 * std::f64::consts::PI * f64::from(i) / f64::from(sides));

        points.push(Vector2::new(
            arc_centre.x + (stand_off * angle.cos()),
            arc_centre.y + (stand_off * angle.sin()),
        ));
    }

    Ok((points, tangent_distance))
}

/// What `try_measure_corner` reads off one edge's frame, for whatever is being drawn in it.
struct CrossSectionCorner {
    /// The way face 0 runs away from the edge, into the wedge the profile is drawn in.
    face0_direction: Vector2,
    /// The way face 1 runs away from the edge.
    face1_direction: Vector2,
    /// The way face 0 faces - its profile normal in this frame, which is the way the tool's
    /// clearance wall runs out past it.
    face0_out: Vector2,
    /// The way face 1 faces.
    face1_out: Vector2,
    /// Half the angle of the wedge between the two faces, in radians.
    half_angle: f64,
}

/// The corner as anything drawn in this frame reads it: which way each face runs away from the
/// edge and which way it faces, half the angle of the wedge between them, and how far back along
/// each face the bevel reaches. Shared so the tool and the wedge that clips it cannot drift apart
/// in what they think the corner is.
fn try_measure_corner(
    normal0: Vector3,
    normal1: Vector3,
    x_axis: Vector3,
    y_axis: Vector3,
    z_axis: Vector3,
    size: f64,
    round_profile: bool,
) -> Result<(CrossSectionCorner, f64), BevelAdvisory> {
    // Where each face runs away from the edge, and which way it faces, in that 2D frame
    let face0_direction = cross_section_face_direction(normal0, z_axis, x_axis, y_axis);
    let face1_direction = cross_section_face_direction(normal1, z_axis, x_axis, y_axis);
    let face0_out = Vector2::new(normal0.dot(x_axis), normal0.dot(y_axis));
    let face1_out = Vector2::new(normal1.dot(x_axis), normal1.dot(y_axis));

    // The interior angle of the corner, measured through the wedge the profile is drawn in - the
    // material on a convex edge, the void on a concave one. (A NaN dot stays NaN through clamp and
    // acos alike and passes both angle bounds.)
    let interior_angle = face0_direction.dot(face1_direction).clamp(-1.0, 1.0).acos();
    let interior_angle_degrees = radians_to_degrees(interior_angle);

    if interior_angle_degrees < MINIMUM_INTERIOR_ANGLE_DEGREES
        || interior_angle_degrees > MAXIMUM_INTERIOR_ANGLE_DEGREES
    {
        let code = if interior_angle_degrees < MINIMUM_INTERIOR_ANGLE_DEGREES {
            BevelAdvisoryCode::FaceAngleTooSharp
        } else {
            BevelAdvisoryCode::FaceAngleTooFlat
        };
        return Err(BevelAdvisory::new(
            code,
            vec![BevelAdvisoryArg::Double(interior_angle_degrees)],
        ));
    }

    let half_angle = interior_angle / 2.0;

    // A fillet is dimensioned by its radius and a chamfer by its setback, and on anything but a
    // right angle those are different numbers: tangentDistance = radius / tan(theta / 2).
    let tangent_distance = if round_profile {
        size / half_angle.tan()
    } else {
        size
    };

    if tangent_distance.is_nan() || tangent_distance.is_infinite() {
        return Err(BevelAdvisory::new(
            BevelAdvisoryCode::BevelDoesNotFitCorner,
            vec![],
        ));
    }

    Ok((
        CrossSectionCorner {
            face0_direction,
            face1_direction,
            face0_out,
            face1_out,
            half_angle,
        },
        tangent_distance,
    ))
}

/// Walks the profile back from the far tangent point to the near one, which is the part of the
/// cut the user actually sees. A single segment is the straight chord of a chamfer; more segments
/// sample the arc a rolling ball of `radius` would leave, tangent to both faces.
fn add_profile_return(
    path: &mut Vec<Vector2>,
    cut_start: Vector2,
    cut_end: Vector2,
    radius: f64,
    half_angle: f64,
    segments: i32,
    round_profile: bool,
) {
    if !round_profile || segments < 2 {
        // Closing the polygon draws the chord from cut_end back to cut_start on its own
        return;
    }

    // The standard inscribed fillet: the centre sits r / sin(theta / 2) along the bisector, which
    // is the hypotenuse of the right triangle whose far side is the radius.
    let bisector = cut_start + cut_end;
    if bisector.length() < 1e-12 {
        return;
    }

    let arc_center = bisector.get_normal() * (radius / half_angle.sin());

    let start_angle = (cut_end.y - arc_center.y).atan2(cut_end.x - arc_center.x);
    let end_angle = (cut_start.y - arc_center.y).atan2(cut_start.x - arc_center.x);

    // the two tangent points are reachable both ways around the centre; the short way is the one
    // that stays inside the corner
    let mut sweep = end_angle - start_angle;

    while sweep > std::f64::consts::PI {
        sweep -= 2.0 * std::f64::consts::PI;
    }

    while sweep < -std::f64::consts::PI {
        sweep += 2.0 * std::f64::consts::PI;
    }

    // The two ends are already in the path (cut_end was just written, cut_start is where the
    // polygon closes back to), so only the points between them are added.
    for i in 1..segments {
        let angle = start_angle + (sweep * f64::from(i) / f64::from(segments));

        path.push(Vector2::new(
            arc_center.x + (radius * angle.cos()),
            arc_center.y + (radius * angle.sin()),
        ));
    }
}

/// The direction a face runs away from the edge, in the 2D cross section frame. It is always the
/// direction that heads into the wedge the profile is drawn in (negative y), never back out over
/// the corner.
fn cross_section_face_direction(
    face_normal: Vector3,
    edge_direction: Vector3,
    x_axis: Vector3,
    y_axis: Vector3,
) -> Vector2 {
    let along_face = face_normal.cross(edge_direction).get_normal();
    let cross_section = Vector2::new(along_face.dot(x_axis), along_face.dot(y_axis));

    if cross_section.y > 0.0 {
        -cross_section
    } else {
        cross_section
    }
}
