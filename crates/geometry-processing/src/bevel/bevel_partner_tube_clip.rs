//! A partnered bead's station profile clipped out of its partners' tubes: where a partner run's
//! ball tube swallows a stretch of the profile, those points are laid along the seam the tube draws
//! in the station's plane instead, so the profile keeps its point count and its winding.

use vector_math::{Vector2, Vector3};

use super::BevelPartnerTube;

/// How far inside a tube a point has to be to count as swallowed.
pub const INTRUDES_BY: f64 = 1e-7;

/// How many bisections pin a crossing down.
pub const HALVINGS: i32 = 40;

const ALONG_THE_PLANE: &str =
    "the partner run's tube runs along this station's plane, so it draws no seam in it";

/// `profile` (in the station's `x_axis`/`y_axis` plane through `origin`, `z_axis` its normal)
/// clipped out of every partner tube in turn, or the refusal that stopped it.
pub fn try_clip(
    profile: &[Vector2],
    origin: Vector3,
    x_axis: Vector3,
    y_axis: Vector3,
    z_axis: Vector3,
    partners: &[BevelPartnerTube],
) -> Result<Vec<Vector2>, String> {
    let mut clipped = profile.to_vec();

    for partner in partners {
        clipped = try_clip_one(&clipped, origin, x_axis, y_axis, z_axis, partner)?;
    }

    Ok(clipped)
}

/// `profile` clipped out of one partner's tube. Untouched where nothing intrudes; refused where the
/// tube swallows it all or crosses it other than twice.
#[allow(clippy::neg_cmp_op_on_partial_ord)]
pub fn try_clip_one(
    profile: &[Vector2],
    origin: Vector3,
    x_axis: Vector3,
    y_axis: Vector3,
    z_axis: Vector3,
    partner: &BevelPartnerTube,
) -> Result<Vec<Vector2>, String> {
    let mut clipped = profile.to_vec();
    let in_part = |point: Vector2| origin + (x_axis * point.x) + (y_axis * point.y);
    let count = profile.len();

    let depths: Vec<f64> = profile.iter().map(|&p| partner.depth(in_part(p))).collect();
    let intruding = depths.iter().filter(|&&d| intrudes(d)).count();

    if intruding == 0 {
        // The overwhelmingly common case, and the one that has to cost nothing and change nothing.
        return Ok(clipped);
    }

    if intruding == count {
        return Err("the partner run's tube covers this station's whole profile".to_string());
    }

    let crossings = (0..count)
        .filter(|&i| intrudes(depths[i]) != intrudes(depths[(i + 1) % count]))
        .count();

    if crossings != 2 {
        return Err(format!(
            "the partner run's tube crosses this station's profile {crossings} times, so what it leaves is not one closed region"
        ));
    }

    // Two crossings means the swallowed points are one run round the ring.
    let first = (0..count)
        .find(|&i| intrudes(depths[i]) && !intrudes(depths[(i + count - 1) % count]))
        .unwrap_or(0);

    let last = (first + intruding - 1) % count;
    let before = (first + count - 1) % count;
    let after = (last + 1) % count;

    let entered = crossing(
        profile[before],
        profile[first],
        depths[before],
        partner,
        &in_part,
    );
    let left = crossing(
        profile[after],
        profile[last],
        depths[after],
        partner,
        &in_part,
    );

    // Where each spent slot sits along the seam: the fraction of the way it already was along the
    // piece of profile the tube swallowed, walked from one crossing to the other.
    let mut walked = vec![0.0; intruding + 1];
    let mut at = entered;

    for i in 0..intruding {
        let next = profile[(first + i) % count];
        walked[i] = (next - at).length() + if i > 0 { walked[i - 1] } else { 0.0 };
        at = next;
    }

    walked[intruding] = (left - at).length() + walked[intruding - 1];

    if !(walked[intruding] > 0.0) {
        return Err("the piece of this station's profile inside the partner run's tube has no length, so there is nothing to lay along the seam".to_string());
    }

    let mut parameters = vec![0.0; intruding];

    if intruding == 1 {
        // One slot for two crossings: it takes the seam's midpoint.
        parameters[0] = 0.5;
    } else {
        // Stretched so the FIRST spent slot lands on the crossing the profile entered by and the
        // LAST on the one it left by, keeping the profile's two corners at the bite.
        let span = walked[intruding - 1] - walked[0];

        for (i, parameter) in parameters.iter_mut().enumerate() {
            *parameter = if span > 0.0 {
                (walked[i] - walked[0]) / span
            } else {
                i as f64 / (intruding - 1) as f64
            };
        }
    }

    let seam = try_draw_seam(
        entered,
        left,
        &parameters,
        profile,
        origin,
        x_axis,
        y_axis,
        z_axis,
        partner,
    )?;

    for (i, point) in seam.into_iter().enumerate().take(intruding) {
        clipped[(first + i) % count] = point;
    }

    Ok(clipped)
}

/// A depth far enough inside the tube to count as swallowed.
pub fn intrudes(depth: f64) -> bool {
    depth < -INTRUDES_BY
}

/// Where the segment `from`..`to` crosses the tube's wall, by [`HALVINGS`] bisections from `from`'s
/// side.
pub fn crossing(
    from: Vector2,
    to: Vector2,
    from_depth: f64,
    partner: &BevelPartnerTube,
    in_part: &dyn Fn(Vector2) -> Vector3,
) -> Vector2 {
    let mut low = 0.0;
    let mut high = 1.0;
    let low_is_outside = !intrudes(from_depth);

    for _ in 0..HALVINGS {
        let middle = (low + high) / 2.0;
        let outside = !intrudes(partner.depth(in_part(from + ((to - from) * middle))));

        if outside == low_is_outside {
            low = middle;
        } else {
            high = middle;
        }
    }

    from + ((to - from) * ((low + high) / 2.0))
}

/// The tube's seam in the station's plane from `from` to `to`, sampled at `parameters`, taken the
/// way round whose halfway point the profile encloses.
#[allow(clippy::too_many_arguments)]
pub fn try_draw_seam(
    from: Vector2,
    to: Vector2,
    parameters: &[f64],
    profile: &[Vector2],
    origin: Vector3,
    x_axis: Vector3,
    y_axis: Vector3,
    z_axis: Vector3,
    partner: &BevelPartnerTube,
) -> Result<Vec<Vector2>, String> {
    let in_part = |point: Vector2| origin + (x_axis * point.x) + (y_axis * point.y);

    let from_angle = partner.angle_of(in_part(from));
    let to_angle = partner.angle_of(in_part(to));

    for forwards in [true, false] {
        let mut sweep = to_angle - from_angle;

        while forwards && sweep <= 0.0 {
            sweep += std::f64::consts::PI * 2.0;
        }

        while !forwards && sweep >= 0.0 {
            sweep -= std::f64::consts::PI * 2.0;
        }

        // The halfway point decides which way round, whatever the count is.
        let Some(halfway) = try_at(
            from_angle + (sweep / 2.0),
            origin,
            x_axis,
            y_axis,
            z_axis,
            partner,
        ) else {
            return Err(ALONG_THE_PLANE.to_string());
        };

        if !encloses(profile, halfway) {
            continue;
        }

        let mut seam = Vec::with_capacity(parameters.len());

        for &along in parameters {
            let Some(point) = try_at(
                from_angle + (sweep * along),
                origin,
                x_axis,
                y_axis,
                z_axis,
                partner,
            ) else {
                return Err(ALONG_THE_PLANE.to_string());
            };

            seam.push(point);
        }

        return Ok(seam);
    }

    Err(
        "neither way round the partner run's tube draws a seam across this station's profile"
            .to_string(),
    )
}

/// The tube's ruling at `angle` crossed with the station's plane, in the plane's own coordinates.
pub fn try_at(
    angle: f64,
    origin: Vector3,
    x_axis: Vector3,
    y_axis: Vector3,
    z_axis: Vector3,
    partner: &BevelPartnerTube,
) -> Option<Vector2> {
    let crossing = partner.try_cross_plane(angle, origin, z_axis)?;
    let offset = crossing - origin;

    Some(Vector2::new(offset.dot(x_axis), offset.dot(y_axis)))
}

/// The even-odd test of `point` against the closed `profile`.
pub fn encloses(profile: &[Vector2], point: Vector2) -> bool {
    let mut inside = false;
    let count = profile.len();
    let mut j = count.wrapping_sub(1);

    for i in 0..count {
        if (profile[i].y > point.y) != (profile[j].y > point.y)
            && point.x
                < (((profile[j].x - profile[i].x) * (point.y - profile[i].y)
                    / (profile[j].y - profile[i].y))
                    + profile[i].x)
        {
            inside = !inside;
        }

        j = i;
    }

    inside
}
