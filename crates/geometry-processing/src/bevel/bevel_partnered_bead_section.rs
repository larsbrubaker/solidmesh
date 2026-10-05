//! One station's section of a partnered bead (`Bevel/BevelPartneredBeadSection.cs`, a file of the
//! C# `static partial class BevelPartneredBead`): how the arc is closed - on the crease line, past
//! the end against a partner, a wall or a plane - and the bead sewn from the sections.

use polygon_mesh::Mesh;
use vector_math::{Vector2, Vector3, Vector3Float};

use super::bevel_partner_tube_clip::try_clip;
use super::bevel_partnered_bead::{
    format_three_places, BevelPartneredBeadEnd, BevelPartneredBeadRun, BevelPartneredBeadStation,
};
use super::bevel_partnered_bead_tolerances::{
    covered_by, reaches_the_face, CUT_BY, DEEPEST_BURIAL,
};
use super::{bevel_profile_polygon, BevelPartnerTube};

/// C# `Along`: where the `i`-th of `beyond` spent slots sits along a walk (`walked` its running
/// lengths), stretched so the first lands at 0 and the last at 1; one slot takes the middle.
pub fn along(walked: &[f64], beyond: usize, i: usize) -> f64 {
    if beyond == 1 {
        return 0.5;
    }

    let span = walked[beyond - 1] - walked[0];

    if span > 0.0 {
        (walked[i] - walked[0]) / span
    } else {
        i as f64 / (beyond - 1) as f64
    }
}

/// C# `OnTheLine`: where the segment from `outside` to `inside` crosses `depth`'s zero, by its
/// linear interpolation (`outside` itself where the two depths agree).
#[allow(clippy::neg_cmp_op_on_partial_ord)]
pub fn on_the_line(outside: Vector2, inside: Vector2, depth: &dyn Fn(Vector2) -> f64) -> Vector2 {
    let here = depth(outside);
    let there = depth(inside);
    let across = here - there;

    if !(across.abs() > 0.0) {
        outside
    } else {
        outside + ((inside - outside) * (here / across))
    }
}

/// C# `Close`: the arc closed into a polygon - its far end buried into the far side, the closing
/// point buried into the corner, and its near end buried into the riding face, each by `burial`
/// clearances.
pub fn close(
    run: &BevelPartneredBeadRun,
    closing: Vector2,
    into_the_corner: Vector2,
    into_the_far_side: Vector2,
    burial: f64,
) -> Vec<Vector2> {
    let mut polygon = Vec::with_capacity(run.point_count.max(0) as usize);

    polygon.extend_from_slice(&run.arc);
    polygon.push(run.arc[run.arc.len() - 1] + (into_the_far_side * burial));
    polygon.push(closing + (into_the_corner * burial));
    polygon.push(run.arc[0] + (run.into_the_riding_face * burial));

    polygon
}

/// C# `Burial`: how many clearances a station past a face end may bury its flat sides - fewer as
/// the face's own end comes near, so the burial never pokes through it. `ends` is the run's
/// `[start, end]` pair the station's `past_end` indexes.
#[allow(clippy::neg_cmp_op_on_partial_ord)]
pub fn burial(
    run: &BevelPartneredBeadRun,
    ends: &[BevelPartneredBeadEnd; 2],
    station: &BevelPartneredBeadStation,
    origin: Vector3,
) -> f64 {
    let Some(end) = station.past_end.map(|index| &ends[index]) else {
        return 1.0;
    };

    if !end.has_face_end || !(run.clearance > 0.0) {
        return 1.0;
    }

    let line = Vector2::new(
        run.x_axis.dot(end.face_end_normal),
        run.y_axis.dot(end.face_end_normal),
    );

    if !(line.length() > 1e-9) {
        return 1.0;
    }

    let reach = ((run.riding_tangent_point.dot(line)
        + (origin - end.face_end_point).dot(end.face_end_normal))
        / line.length())
        / run.clearance;

    reach.clamp(0.02, 1.0)
}

/// C# `Sew`: the bead from its stations' rings, strip by strip, capped at both ends by the first
/// and last sections triangulated. `None` where there are too few rings or a cap will not
/// triangulate.
pub fn sew(
    rings: &[Vec<Vector3>],
    first_section: &[Vector2],
    last_section: &[Vector2],
    point_count: i32,
) -> Option<Mesh> {
    if rings.len() < 2 {
        return None;
    }

    let start_cap = bevel_profile_polygon::try_triangulate(first_section)?;
    let end_cap = bevel_profile_polygon::try_triangulate(last_section)?;

    let mut mesh = Mesh::new();

    for ring in rings {
        for &position in ring {
            mesh.vertices.push(Vector3Float::from_vector3(position));
        }
    }

    for strip in 0..rings.len() as i32 - 1 {
        let back = strip * point_count;
        let front = (strip + 1) * point_count;

        for point in 0..point_count {
            let next = (point + 1) % point_count;

            mesh.faces
                .add(back + point, back + next, front + next, &mesh.vertices);
            mesh.faces
                .add(back + point, front + next, front + point, &mesh.vertices);
        }
    }

    let last_ring = (rings.len() as i32 - 1) * point_count;

    for cap in start_cap.chunks(3) {
        mesh.faces.add(cap[0], cap[2], cap[1], &mesh.vertices);
    }

    for cap in end_cap.chunks(3) {
        mesh.faces.add(
            last_ring + cap[0],
            last_ring + cap[1],
            last_ring + cap[2],
            &mesh.vertices,
        );
    }

    Some(mesh)
}

/// C# `SignedArea`: the polygon's signed area, positive anticlockwise.
pub fn signed_area(polygon: &[Vector2]) -> f64 {
    let mut twice = 0.0;
    let count = polygon.len();
    let mut j = count.wrapping_sub(1);

    for i in 0..count {
        twice += (polygon[j].x * polygon[i].y) - (polygon[i].x * polygon[j].y);
        j = i;
    }

    twice / 2.0
}

/// C# `TryStation`: one station's section - the arc closed on the crease, or past the end against
/// its partners, wall or planes - clipped out of the partners' tubes and cut at any face end or
/// across plane. `ends` is the run's `[start, end]` pair.
pub fn try_station(
    run: &BevelPartneredBeadRun,
    ends: &[BevelPartneredBeadEnd; 2],
    station: &BevelPartneredBeadStation,
) -> Result<Vec<Vector2>, String> {
    let origin = run.origin(station.along);
    let mut closing = run.crease;
    let mut into_the_corner = run.into_the_corner;
    let mut bounding = Vec::new();

    if let Some(index) = station.past_end {
        (closing, into_the_corner, bounding) =
            try_close_past_the_end(run, &ends[index], station, origin)?;
    }

    let into_the_far_side = if station.past_end.is_none() {
        run.into_the_second_face
    } else {
        into_the_corner
    };

    let mut polygon = close(
        run,
        closing,
        into_the_corner,
        into_the_far_side,
        burial(run, ends, station, origin),
    );

    if run.reverse {
        polygon.reverse();
    }

    let partners = bounding;

    let mut section = try_clip(
        &polygon,
        origin,
        run.x_axis,
        run.y_axis,
        run.direction,
        &partners,
    )?;

    try_cut_at_the_face_end(run, ends, station, origin, &mut section)?;
    try_cut_on_the_across_plane(run, ends, station, origin, &mut section)?;

    for partner in &partners {
        for &point in &section {
            if partner.depth(run.in_part(station.along, point)) < -1e-6 {
                return Err(format!(
                    "a partner's tube still covers part of the section at {} along the run after it was clipped out of it",
                    format_three_places(station.along)
                ));
            }
        }
    }

    Ok(section)
}

/// C# `TryCutAtTheFaceEnd`: the section cut at the plane the riding face ends at, past an end
/// that has one.
pub fn try_cut_at_the_face_end(
    run: &BevelPartneredBeadRun,
    ends: &[BevelPartneredBeadEnd; 2],
    station: &BevelPartneredBeadStation,
    origin: Vector3,
    section: &mut [Vector2],
) -> Result<(), String> {
    let Some(end) = station.past_end.map(|index| &ends[index]) else {
        return Ok(());
    };

    if !end.has_face_end {
        return Ok(());
    }

    try_cut_on_a_plane(
        run,
        origin,
        section,
        end.face_end_point,
        end.face_end_normal,
        "the face this run rides has ended across",
        "the end of the face this run rides",
    )
}

/// C# `TryCutOnTheAcrossPlane`: the section cut at the across plane, past an end that stops on it.
pub fn try_cut_on_the_across_plane(
    run: &BevelPartneredBeadRun,
    ends: &[BevelPartneredBeadEnd; 2],
    station: &BevelPartneredBeadStation,
    origin: Vector3,
    section: &mut [Vector2],
) -> Result<(), String> {
    let Some(end) = station.past_end.map(|index| &ends[index]) else {
        return Ok(());
    };

    if !end.ends_on_the_across_plane {
        return Ok(());
    }

    try_cut_on_a_plane(
        run,
        origin,
        section,
        end.across_point,
        end.across_normal,
        "the face across the edge this run's second face ended at stands over",
        "the plane of the face across the edge this run's second face ended at",
    )
}

/// C# `TryCutOnAPlane`: the points of `section` beyond the plane laid along the line it cuts the
/// section on, keeping the point count; `covers` and `meets` word the refusals.
#[allow(clippy::neg_cmp_op_on_partial_ord)]
pub fn try_cut_on_a_plane(
    run: &BevelPartneredBeadRun,
    origin: Vector3,
    section: &mut [Vector2],
    plane_point: Vector3,
    plane_normal: Vector3,
    covers: &str,
    meets: &str,
) -> Result<(), String> {
    let mut line = Vector2::new(run.x_axis.dot(plane_normal), run.y_axis.dot(plane_normal));
    let mut offset = (origin - plane_point).dot(plane_normal);

    if !(line.length() > 1e-9) {
        return Ok(());
    }

    let scale = 1.0 / line.length();
    line *= scale;
    offset *= scale;

    let depth = |point: Vector2| point.dot(line) + offset;
    let count = section.len();

    let beyond = section.iter().filter(|&&p| depth(p) < -CUT_BY).count();

    if beyond == 0 {
        return Ok(());
    }

    if beyond == count {
        return Err(format!(
            "{covers} the whole of this station's section, so there is nothing of the bead left here to build"
        ));
    }

    let crossings = (0..count)
        .filter(|&i| (depth(section[i]) < -CUT_BY) != (depth(section[(i + 1) % count]) < -CUT_BY))
        .count();

    if crossings != 2 {
        return Err(format!(
            "{meets} crosses this station's section {crossings} times, so what it leaves is not one closed region"
        ));
    }

    let first = (0..count)
        .find(|&i| {
            depth(section[i]) < -CUT_BY && !(depth(section[(i + count - 1) % count]) < -CUT_BY)
        })
        .unwrap_or(0);

    let before = (first + count - 1) % count;
    let last = (first + beyond - 1) % count;
    let after = (last + 1) % count;

    let entered = on_the_line(section[before], section[first], &depth);
    let left = on_the_line(section[after], section[last], &depth);

    if !((left - entered).length() > 0.0) {
        return Err(format!(
            "{meets} touches this station's section at one point rather than crossing it, so there is nothing left of the bead here to draw"
        ));
    }

    let mut walked = vec![0.0; beyond];
    let mut at = entered;

    for i in 0..beyond {
        let next = section[(first + i) % count];
        walked[i] = (next - at).length() + if i > 0 { walked[i - 1] } else { 0.0 };
        at = next;
    }

    for i in 0..beyond {
        let along = along(&walked, beyond, i);
        section[(first + i) % count] = entered + ((left - entered) * along);
    }

    Ok(())
}

/// C# `TryClosePastTheEnd`: where a station past the end closes - on the crease, on a wall, or on
/// the foot of the partner that covers it - as `(closing, into the corner, the partners reaching
/// this station)`.
#[allow(clippy::neg_cmp_op_on_partial_ord)]
pub fn try_close_past_the_end(
    run: &BevelPartneredBeadRun,
    end: &BevelPartneredBeadEnd,
    station: &BevelPartneredBeadStation,
    origin: Vector3,
) -> Result<(Vector2, Vector2, Vec<BevelPartnerTube>), String> {
    let mut closing = run.crease;
    let mut into_the_corner = run.into_the_corner;
    let mut reaching = Vec::with_capacity(end.partners.len());
    let mut on_a_partner = false;

    let mut face_direction = run.crease - run.riding_tangent_point;

    if !(face_direction.length() > 0.0) {
        return Err("this run's crease and its riding tangent point are the same point, so the section has no direction to close along".to_string());
    }

    face_direction = face_direction.get_normal();

    let mut furthest = (run.crease - run.riding_tangent_point).dot(face_direction);

    if end.ends_on_the_across_plane {
        into_the_corner = run.into_the_riding_face;
    }

    if end.has_wall {
        if let Some((on_the_wall, into_the_wall)) = try_wall_here(run, end, origin) {
            into_the_corner = into_the_wall;

            let wall_reach = (on_the_wall - run.riding_tangent_point).dot(face_direction);

            if wall_reach > furthest {
                furthest = wall_reach;
                closing = on_the_wall;
            }
        }
    }

    let mut feet = Vec::with_capacity(end.partners.len());

    for partner in &end.partners {
        let along = partner.axis_direction.dot(run.direction);

        if along.abs() < 1e-6 {
            continue;
        }

        let axis_here = partner.axis_point
            + (partner.axis_direction * ((origin - partner.axis_point).dot(run.direction) / along));

        let offset = axis_here - origin;
        let flat = Vector2::new(offset.dot(run.x_axis), offset.dot(run.y_axis));

        if (flat - run.riding_tangent_point)
            .dot(run.riding_normal2)
            .abs()
            > run.size + reaches_the_face(run)
        {
            continue;
        }

        reaching.push(*partner);
        feet.push(
            flat - (run.riding_normal2 * (flat - run.riding_tangent_point).dot(run.riding_normal2)),
        );
    }

    for &foot in &feet {
        let reach = (foot - run.riding_tangent_point).dot(face_direction);

        if station.past_the_crease
            && (!on_a_partner || reach < furthest)
            && covered(run, &reaching, origin, foot)
        {
            furthest = reach;
            closing = foot;
            into_the_corner = run.into_the_riding_face;
            on_a_partner = true;
        }
    }

    if on_a_partner && !covered(run, &reaching, origin, run.arc[run.arc.len() - 1]) {
        return Err("no partner's tube reaches the far end of this section's arc, so the seam between the two beads runs off the end of it".to_string());
    }

    Ok((closing, into_the_corner, reaching))
}

/// C# `TryWallHere`: where the wall an end ran out onto crosses this station - `(on the wall,
/// into the corner)`, the corner point buried a clearance inside both the riding face and the
/// wall, at most [`DEEPEST_BURIAL`] clearances deep.
#[allow(clippy::neg_cmp_op_on_partial_ord)]
pub fn try_wall_here(
    run: &BevelPartneredBeadRun,
    end: &BevelPartneredBeadEnd,
    origin: Vector3,
) -> Option<(Vector2, Vector2)> {
    let along = end.wall_direction.dot(run.direction);

    if along.abs() < 1e-6 {
        return None;
    }

    let here = end.wall_point
        + (end.wall_direction * ((origin - end.wall_point).dot(run.direction) / along));
    let offset = here - origin;

    let on_the_wall = Vector2::new(offset.dot(run.x_axis), offset.dot(run.y_axis));

    let riding_here = Vector2::new(
        run.x_axis.dot(run.riding_normal),
        run.y_axis.dot(run.riding_normal),
    );
    let wall_here = Vector2::new(
        run.x_axis.dot(end.wall_normal),
        run.y_axis.dot(end.wall_normal),
    );
    let determinant = (riding_here.x * wall_here.y) - (riding_here.y * wall_here.x);

    if !(determinant.abs() > 1e-9) {
        return Some((on_the_wall, run.into_the_riding_face));
    }

    let mut buried = Vector2::new(
        (-run.clearance * wall_here.y) + (run.clearance * riding_here.y),
        (-run.clearance * riding_here.x) + (run.clearance * wall_here.x),
    ) / determinant;

    if buried.length() > DEEPEST_BURIAL * run.clearance {
        buried = buried * (DEEPEST_BURIAL * run.clearance / buried.length());
    }

    Some((on_the_wall, buried))
}

/// C# `Covered`: whether `point` lies inside any of `partners` by [`covered_by`].
pub fn covered(
    run: &BevelPartneredBeadRun,
    partners: &[BevelPartnerTube],
    origin: Vector3,
    point: Vector2,
) -> bool {
    let in_part = origin + (run.x_axis * point.x) + (run.y_axis * point.y);

    partners
        .iter()
        .any(|partner| partner.depth(in_part) < covered_by(run))
}
