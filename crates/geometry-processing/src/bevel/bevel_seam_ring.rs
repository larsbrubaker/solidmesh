//! The ring of points a corner patch and the run that ends on it MEET at: the arc of the ball's own
//! surface lying in the plane through the ball's centre square to that run. Computed here once, by
//! one route, and carried literally by both solids.
//!
//! The two used to be computed twice, by two different routes: the corner patch mesher slerps
//! between the two face normals on the ball's sphere, the run builds its arc in a 2D profile frame
//! and sweeps it onto the plane. Those are the same circle as mathematics and about one FLOAT ULP
//! apart as arithmetic - 1.9e-6 at a coordinate of 20. Manifold's tolerance for a part that size
//! is around 3.5e-6, so at eight segments the near-miss snapped shut; at twelve one vertex missed
//! by 7.6e-6 and the n-ary union of six clean operands came back `SelfIntersecting`.
//!
//! An epsilon cannot settle that: whatever tolerance the kernel snaps at, a seam computed twice
//! will eventually miss by more than it. What settles it is having ONE set of points. The patch is
//! built first, so it computes the seam and publishes it, and the run REPLACES the arc points of
//! its own ring on that plane with the published ones ([`snap`]) rather than recomputing or
//! re-projecting anything. Only the ring ON the plane is shared; everything the run does past it
//! follows the run's own profile, because none of that is a seam with anything.

use std::collections::HashSet;

use vector_math::Vector3;

/// How far a run's own arc point may sit from the published seam point it stands for, as a
/// fraction of the radius, and still be recognised as that point.
///
/// Both sides of the seam are doubles here and the two routes agree to about 1e-13, so what this
/// has to clear is a float ulp (1.9e-6 at a coordinate of 20) in case anything upstream has been
/// through one. At `r = 3.4` a hundred-thousandth of the radius is 3.4e-5: an order clear of that
/// rounding and three orders under the closest two seam points can be (a whole chord, 0.67 mm at
/// eight segments round that radius). It was 1e-3 first, which was loose enough to accept a ring
/// that belonged somewhere else and move it, silently, onto the seam. The point of the match is to
/// recognise the SAME point computed twice, not to find the nearest one.
const MATCH_FRACTION: f64 = 1e-5;

/// The seam over one run: `segments + 1` points at `radius` from `centre`, stepped at equal angles
/// from where the ball touches one face (`normal0`) to where it touches the other (`normal1`).
///
/// The two ENDS are the corner directions themselves rather than a slerp at fraction 0 and 1.
/// Those come out a rounding apart from the direction they stand for, and each end is shared with
/// the NEIGHBOURING run's seam, whose arc spans a different angle - so a slerped end would put two
/// different points where the two runs and the patch all have to meet at one.
pub fn points(
    centre: Vector3,
    radius: f64,
    normal0: Vector3,
    normal1: Vector3,
    segments: i32,
) -> Vec<Vector3> {
    let count = segments.max(1);
    let mut points = Vec::with_capacity(count as usize + 1);

    for step in 0..=count {
        let direction = if step == 0 {
            normal0
        } else if step == count {
            normal1
        } else {
            slerp(normal0, normal1, f64::from(step) / f64::from(count))
        };

        points.push(centre + (direction * radius));
    }

    points
}

/// The point `fraction` of the way along the great circle from one unit vector to another - equal
/// angles, which is how the edge tool builder samples a fillet's arc and how the corner patch
/// mesher samples the ball.
pub fn slerp(from: Vector3, to: Vector3, fraction: f64) -> Vector3 {
    let angle = from.dot(to).clamp(-1.0, 1.0).acos();

    if angle < 1e-9 {
        return from;
    }

    ((from * ((1.0 - fraction) * angle).sin()) + (to * (fraction * angle).sin())) / angle.sin()
}

/// Puts the published seam points into a swept run's own ring, in place, and says whether the
/// whole ring of them was placed. The ring is not moved at all unless every seam point found
/// exactly one point of it to stand for.
///
/// `ring` is the run's ring on the ball-centre plane, `seam` the patch's published points for this
/// end in arc order, and `radius` the ball's radius, the scale [`MATCH_FRACTION`] is read in.
///
/// The correspondence is found by nearness rather than by index, deliberately: which indices of
/// the profile carry the arc depends on where the cross section starts, which way the sweep wound
/// it and which face the seam was walked from. Nearness needs none of that tracked.
///
/// ALL or NOTHING. A partly snapped ring would carry some of the patch's points and some of its
/// own - the same two-polylines-on-one-circle the exercise exists to remove - silently. So a ring
/// that does not present the whole seam is left exactly as the sweep made it.
pub fn snap(ring: &mut [Vector3], seam: &[Vector3], radius: f64) -> bool {
    if seam.is_empty() || ring.len() < seam.len() {
        return false;
    }

    let tolerance = radius.abs() * MATCH_FRACTION;
    let mut at = vec![0usize; seam.len()];
    let mut taken = HashSet::new();

    for (i, seam_point) in seam.iter().enumerate() {
        let mut best: Option<usize> = None;
        let mut closest = tolerance;

        for (point, ring_point) in ring.iter().enumerate() {
            let distance = (*ring_point - *seam_point).length();

            if distance <= closest {
                best = Some(point);
                closest = distance;
            }
        }

        match best {
            Some(best) if taken.insert(best) => at[i] = best,
            _ => return false,
        }
    }

    for (i, seam_point) in seam.iter().enumerate() {
        ring[at[i]] = *seam_point;
    }

    true
}
