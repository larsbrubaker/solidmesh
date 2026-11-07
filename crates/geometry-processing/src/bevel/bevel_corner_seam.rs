//! How a run's cutter meets the surface it stops on - the face that bounds its end, or the corner
//! patch that owns the vertex.
//!
//! Both answers are about the same hazard, which is why they live together: two solids that share
//! a plane EXACTLY are the worst thing a boolean kernel can be handed, and each of the two seams
//! deals with that a different way. Its sibling [`super::bevel_blend_seam`] reads the plane a
//! blend hands over on; both produce a [`BevelEndCap`].

use std::collections::HashSet;

use vector_math::Vector3;

use super::BevelEndCap;

/// Four float ulps as a fraction of a coordinate's own magnitude, and the floor a part modelled AT
/// the origin still gets - see [`stand_off`] for what a cutter's cap has to clear.
const CAP_STAND_OFF_OF_THE_COORDINATE: f64 = 4.0 * 1.192092896e-7;

const MINIMUM_CAP_STAND_OFF: f64 = 1e-6;

/// How much of the tool's outer clearance is taken back at the cap of a drive-through, as a
/// fraction.
///
/// The walls it moves stand a whole clearance OUTSIDE the face they belong to, so the space
/// between them and that face is void: pulling them in by a quarter shrinks the removed set only
/// where there was nothing to remove. What it buys is that two runs driving through the same
/// corner no longer present walls that are coplanar AND overlapping on the shared face plane - each
/// now meets that plane along a line instead of over an area, and Manifold's n-ary union stops
/// calling the result self-intersecting. A quarter rather than a hair for the same reason the
/// overshoot is not a hair: near-parallel is worse to hand a kernel than plainly crossing.
pub const CAP_TAPER: f64 = 0.25;

/// How far past the bounding face a cap is pushed, so that a face which is not flat to float
/// precision is still cut clean through. A cutter's always, and a bead's only where the face
/// closes over the run.
///
/// The plane is read through the run's own END VERTEX, and a mesh face is only as flat as the
/// float coordinates bounding it. "Encroaching Side.mcx" is the case that made this: its part comes
/// out of a boolean union, so its top face carries vertices at z = 20 and vertices at
/// z = 20.000001907, and the run's end vertex is one of the LOW ones. The cap stopped 1.9e-6 under
/// the highest of them and the subtraction left a wafer of the unbevelled corner, two microns
/// thick (`BevelCapStandOffTests`).
///
/// Outward is the safe side to err on. A cap that stops SHORT leaves a coincident double sheet, the
/// worst thing a boolean can be handed; a cap that goes PAST cuts a hair deeper, and past an
/// outward-facing bounding face there is usually no material at all. Where there IS material
/// beyond, this takes microns off it as a clean open cut rather than a coincidence.
///
/// Float noise is RELATIVE, so the size is four ulps at the end vertex's own distance from the
/// origin: 5.7e-5 for the fixture above, thirty times the error it must clear.
///
/// Cutters, and beads in one case. A bead's cap is normally where material stops being ADDED, so
/// pushing it out leaves a bump proud of the surface it ran out on - which is why `is_bead`
/// answers zero. The exception is a bead whose capping face CLOSES OVER the run, where everything
/// past that plane is the part's own material, so the extra micron is buried rather than proud;
/// the caller passes `is_bead && !closes_over` (`BevelChainEnds`, at its cap).
pub fn stand_off(is_bead: bool, position: Vector3) -> f64 {
    if is_bead {
        0.0
    } else {
        MINIMUM_CAP_STAND_OFF.max(position.length() * CAP_STAND_OFF_OF_THE_COORDINATE)
    }
}

/// The cap for a run that may drive straight through its corner rather than stop on the patch's
/// wall, or `None` where this corner is not one of those.
///
/// `seam_points` are the points the patch has put on the ball-centre plane, which this end's ring
/// carries in place of its own arc (`BevelSeamRing`). Driving through does not move that ring -
/// only what is added PAST it - so the seam is the same either way, and it is carried on the cap
/// because that is what the sweep reads.
///
/// Only at an ALL-CONVEX corner of exactly three runs, which the caller names in
/// `runs_may_drive_through`. Two solids sharing a wall exactly is what a boolean kernel handles
/// worst, and on a 45 degree knife tip - where this plane stands R/tan(22.5) = 8.2 mm out rather
/// than a cube corner's 3.4 - it resolved that wall into a zero thickness flap and left the
/// corner's own sharp edge standing between the two, 1.4 mm from the control.
///
/// Driving through removes the coincidence rather than nudging it, and here it costs nothing. The
/// ball's centre is R from all three faces, so it lies on all three cylinder AXES: the sphere is
/// inscribed in every one of them, so inside the corner's cell "outside cylinder" is contained in
/// "outside sphere" - which is what the patch already removes. What moves is only the seam: the
/// patch's wall becomes an interior face of the union, and at the shared circle the sphere meets
/// the cylinder tangentially as it does on a cube corner. An epsilon cannot do this: a cap a few
/// ulps past the plane leaves its arc within about d^2/2R of the sphere over a whole band, a near
/// tangency in picometres, which is worse to hand a kernel than the coincidence.
///
/// Only all-convex, only three runs, only a cutter. A concave end at the vertex means a bead is
/// putting material back where this would drive through; another edge count is a different cell
/// whose containment argument has not been made; and a bead overshooting ADDS material rather
/// than removing what something else already removed.
pub fn try_drive_through(
    is_bead: bool,
    vertex: i32,
    outward: Vector3,
    position: Vector3,
    ball_centre: Vector3,
    runs_may_drive_through: Option<&HashSet<i32>>,
    seam_points: Option<&[Vector3]>,
) -> Option<BevelEndCap> {
    if is_bead || !runs_may_drive_through.is_some_and(|runs| runs.contains(&vertex)) {
        return None;
    }

    // The run still stops exactly on the ball centre's plane, as it always did - that ring keeps
    // its full clearance and the wall behind it is unchanged, so it goes on touching a
    // neighbouring run's wall at a point rather than over an area. What is added is ONE ring
    // beyond it, a stand-off short of the corner VERTEX: the vertex is on all three faces, so the
    // plane square to the run through it is the last plane still inside the cell, and nothing of
    // the tool reaches the void past the corner.
    //
    // Driving a couple of radii PAST the vertex instead put the cap out in free space, which fixed
    // the seam and then left the two runs' outer walls overlapping in AREA on each shared face
    // plane - coplanar, same orientation, two operands - which the kernel's n-ary union answers as
    // self-intersecting.
    let centre = outward.dot(ball_centre);
    let far = outward.dot(position) - stand_off(false, position);

    // How far the overshoot actually runs is the setback the ball centre sits back by, and on a
    // corner whose faces open out towards flat that setback tends to zero. Below the stand-off it
    // would put the extra ring level with, or BEHIND, the run's own last ring - a strip wound
    // backwards, which is a self-intersecting operand rather than a shorter overshoot. Such a
    // corner keeps the plain cap it had before any of this.
    if far <= centre {
        return None;
    }

    Some(BevelEndCap::with_options(
        outward,
        centre,
        true,
        far,
        seam_points.map(<[Vector3]>::to_vec),
    ))
}
