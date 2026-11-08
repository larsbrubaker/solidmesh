//! The cross section one edge's bevel is cut with, and the frame it is drawn in: on a convex edge
//! the profile of the material that has to go, on a concave edge the mirror of the same
//! construction. The chain sweep's sections (`bevel_chain_sweep_sections`) draw them along a run,
//! and the surface construction reads the closed forms (`tangent_distance`,
//! `profile_wedge_angle_degrees`). Its `try_*` functions return `Result<_, BevelAdvisory>`: the built section, or the reason
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

use solidmesh_math::math_helper::{degrees_to_radians, radians_to_degrees};
use solidmesh_math::{Vector2, Vector3};

use super::{BevelAdvisory, BevelAdvisoryArg, BevelAdvisoryCode};
use crate::topology::{EdgeConvexity, FeatureEdge};

/// How square to the edge the two faces have to be before the frame means anything. Below this
/// the faces have folded back on each other or lie flat, and there is no corner to cut.
pub const MINIMUM_INTERIOR_ANGLE_DEGREES: f64 = 1.0;

/// See [`MINIMUM_INTERIOR_ANGLE_DEGREES`].
pub const MAXIMUM_INTERIOR_ANGLE_DEGREES: f64 = 179.0;

/// How far back along each face the bevel reaches, without building the cross section - the
/// closed form of the tangent distance [`try_build_cross_section`] hands back.
///
/// `size` is the fillet radius or the chamfer setback, `wedge_angle_degrees` the wedge the profile
/// is drawn in ([`profile_wedge_angle_degrees`]), `round_profile` true for a fillet's tangent arc.
/// A chamfer's setback IS the size. A fillet's is `radius / tan(theta / 2)`, the same line
/// `TryMeasureCorner` computes from the angle it measures out of the frame; the two agree because
/// they are the same angle read two ways.
pub fn tangent_distance(size: f64, wedge_angle_degrees: f64, round_profile: bool) -> f64 {
    if !round_profile {
        return size;
    }

    let tangent_of_half_angle = libm::tan(degrees_to_radians(wedge_angle_degrees) / 2.0);

    // A wedge folded shut has no corner to reach back along. Infinity is the honest answer and it
    // stays usable where it matters: a caller only ever COMPARES this against a length, and
    // infinity compares the right way round. try_build_cross_section refuses the corner outright
    // in any case, so no section is ever swept on it.
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
/// curved face is the arc and whose back faces are buried inside the part.
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
    // through itself on the inside of the turn - see the chain linker's mitre split
    // (`bevel_chain_linker`), which measures the back reach and retries at the depth the turn
    // allows (`bevel_sweep_back_depth`). Never DEEPER than the size.
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
    let cleared_apex = Vector2::new(0.0, clearance / libm::sin(half_angle));

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
    let interior_angle = libm::acos(face0_direction.dot(face1_direction).clamp(-1.0, 1.0));
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
        size / libm::tan(half_angle)
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

    let arc_center = bisector.get_normal() * (radius / libm::sin(half_angle));

    let start_angle = libm::atan2(cut_end.y - arc_center.y, cut_end.x - arc_center.x);
    let end_angle = libm::atan2(cut_start.y - arc_center.y, cut_start.x - arc_center.x);

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
            arc_center.x + (radius * libm::cos(angle)),
            arc_center.y + (radius * libm::sin(angle)),
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
