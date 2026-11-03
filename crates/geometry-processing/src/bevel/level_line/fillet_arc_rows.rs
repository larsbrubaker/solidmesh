//! A fillet's arc across the band, drawn into the level-line mesh: the plain and skew rows
//! ([`build`], [`build_skew`]), a STOPPED end's row in its plane ([`build_in_a_stopped_plane`]), a closed chain's zip spoke carried along or round the chain
//! ([`along_the_chain`]), and the rows on a sharp joint's seam ([`in_the_seam`],
//! [`on_a_rim_joints_seam`]). The geometry they read (the carry, the seam row, the leg tests) is in
//! `fillet_arc`.

use vector_math::{ieee_math, Vector3};

use super::end_face_section_row::{section_row, EndRowAsked};
use super::fillet_arc::{
    carried, carry_weight, on_a_whole_leg, on_the_partners_leg, seam_row, seam_tolerance,
    tangent_at, RowEnd,
};
use super::gap_fill::arc_point;
use super::open_gap_fill::chain_direction_at;
use super::{ChainField, LevelLineMesh};

/// The new interior vertices of a row: `points[1..segments]` added to the mesh, recorded in
/// `added_vertices`, between the row's two ends.
fn row_of(
    mesh: &mut LevelLineMesh,
    from: i32,
    to: i32,
    segments: i32,
    added_vertices: &mut Vec<i32>,
    point: impl Fn(i32) -> Vector3,
) -> Vec<i32> {
    let mut arc = vec![0; segments as usize + 1];
    arc[0] = from;
    arc[segments as usize] = to;
    for s in 1..segments {
        let added = mesh.add_vertex(point(s));
        added_vertices.push(added);
        arc[s as usize] = added;
    }

    arc
}

/// The arc's vertex indices from `from` to
/// `to` about `chain_point`, `segments + 1` of them; every interior point is a new vertex, recorded
/// in `added_vertices`. One segment is the chamfer's straight row and adds nothing.
pub fn build(
    mesh: &mut LevelLineMesh,
    from: i32,
    to: i32,
    chain_point: Vector3,
    segments: i32,
    added_vertices: &mut Vec<i32>,
) -> Vec<i32> {
    if segments == 1 {
        return vec![from, to];
    }

    let p = mesh.position(from);
    let q = mesh.position(to);
    row_of(mesh, from, to, segments, added_vertices, |s| {
        arc_point(p, chain_point, q, s as f64 / segments as f64)
    })
}

/// The row from `from` to `to` where BOTH stand in one of the chain's STOPPED end planes
/// (`stopped_end_clip.rs`): the round's own section in that plane (`end_face_section_row`), the
/// square section slid along the chain onto the plane, so the row lies in the plane and the flat
/// cap beside it closes there. A skew row's points leave the plane between its ends wherever the
/// plane leans off square to the chain (its shift is shared out by the arc's parameter, not by
/// where each point stands). `None` where the two do not stand in one stopped plane, or the plane
/// is square to the chain (the plain rows are already the section there).
pub fn build_in_a_stopped_plane(
    mesh: &mut LevelLineMesh,
    chain: &ChainField,
    from: i32,
    to: i32,
    segments: i32,
    added_vertices: &mut Vec<i32>,
) -> Option<Vec<i32>> {
    if segments == 1 {
        return None;
    }

    let (p, q) = (mesh.position(from), mesh.position(to));
    let (origin, _, _) = chain.stopped_end_plane_at(p)?;
    let (other, _, _) = chain.stopped_end_plane_at(q)?;
    if origin != other {
        return None;
    }

    let points = chain.points();
    let at_the_far_end = origin == points[points.len() - 1] && origin != points[0];
    let end = if at_the_far_end { chain.length } else { 0.0 };
    let asked = EndRowAsked {
        at_the_far_end,
        tangent: chain_direction_at(chain, end),
    };
    let section = section_row(chain, p, q, asked, None)?;
    Some(row_of(mesh, from, to, segments, added_vertices, |s| {
        section(s as f64 / segments as f64)
    }))
}

/// A SKEW
/// spoke's arc, whose two rail vertices cross the chain at different points. Both ends are carried
/// along the chain to the section at `chain_point`, the arc is drawn there, and each point is
/// carried back by its share of the two offsets - so on a straight chain every point lies on the
/// fillet's cylinder.
#[allow(clippy::too_many_arguments)]
pub fn build_skew(
    mesh: &mut LevelLineMesh,
    from: i32,
    to: i32,
    from_chain_point: Vector3,
    chain_point: Vector3,
    to_chain_point: Vector3,
    segments: i32,
    added_vertices: &mut Vec<i32>,
) -> Vec<i32> {
    if segments == 1 {
        return vec![from, to];
    }

    let from_shift = chain_point - from_chain_point;
    let to_shift = chain_point - to_chain_point;
    let p = mesh.position(from) + from_shift;
    let q = mesh.position(to) + to_shift;
    row_of(mesh, from, to, segments, added_vertices, |s| {
        let t = s as f64 / segments as f64;
        arc_point(p, chain_point, q, t) - (from_shift * (1.0 - t)) - (to_shift * t)
    })
}

/// A parameter round the chain's loop into `[0, length)`.
fn wrapped(length: f64, parameter: f64) -> f64 {
    let parameter = parameter % length;
    if parameter < 0.0 {
        parameter + length
    } else {
        parameter
    }
}

/// A spoke of a closed chain's zip, each end given as its rail
/// vertex, unwrapped parameter, tie span and lap. Where both readings lie on one straight leg the
/// row is carried along the chain (the skew row); across a gentle turn it is carried round the
/// chain with the chain's direction, fading to the plain row near a sharp joint; at an infinite
/// bend threshold, or across a sharp turn, it is drawn about the chain point at the mean.
#[allow(clippy::too_many_arguments)]
pub fn along_the_chain(
    mesh: &mut LevelLineMesh,
    chain: &ChainField,
    from: RowEnd,
    to: RowEnd,
    segments: i32,
    tie: f64,
    added_vertices: &mut Vec<i32>,
) -> Vec<i32> {
    let length = chain.length;
    let from = on_the_partners_leg(mesh, chain, from, to);
    let to = on_the_partners_leg(mesh, chain, to, from);
    if let Some(seam) = in_the_seam(mesh, chain, from, to, segments, added_vertices) {
        return seam;
    }

    let from_reads = (to.1 - from.3).clamp(from.2 .0, from.2 .1) + from.3;
    let to_reads = (from.1 - to.3).clamp(to.2 .0, to.2 .1) + to.3;
    let from_point = chain.point_at(wrapped(length, from_reads));
    let to_point = chain.point_at(wrapped(length, to_reads));
    if ((to_point - from_point).length() - (to_reads - from_reads).abs()).abs() <= tie * 1e3
        && on_a_whole_leg(chain, wrapped(length, (from_reads + to_reads) / 2.0))
    {
        let mean = chain.point_at(wrapped(length, (from_reads + to_reads) / 2.0));
        return build_skew(
            mesh,
            from.0,
            to.0,
            from_point,
            mean,
            to_point,
            segments,
            added_vertices,
        );
    }

    // An infinite threshold says only that no vertex is mitred, not which turns are gentle, so it
    // carries nothing.
    if !chain.sharp_turn_degrees.is_finite() {
        let mean = chain.point_at(wrapped(length, (from.1 + to.1) / 2.0));
        return build(mesh, from.0, to.0, mean, segments, added_vertices);
    }

    // The carry fades rather than switches: full well inside a gentle run, nothing within a
    // setback of a sharp joint or as the row's own turn nears the bend threshold.
    let mean_reads = wrapped(length, (from_reads + to_reads) / 2.0);
    let from_tangent = tangent_at(chain, wrapped(length, from_reads));
    let mean_tangent = tangent_at(chain, mean_reads);
    let to_tangent = tangent_at(chain, wrapped(length, to_reads));
    let low = ieee_math::min_f64(
        ieee_math::min_f64(from_reads, to_reads),
        ieee_math::min_f64(from.2 .0 + from.3, to.2 .0 + to.3),
    );
    let high = ieee_math::max_f64(
        ieee_math::max_f64(from_reads, to_reads),
        ieee_math::max_f64(from.2 .1 + from.3, to.2 .1 + to.3),
    );
    let carry = carry_weight(chain, low, high, from_tangent, to_tangent);
    let mean_point = chain.point_at(wrapped(length, (from.1 + to.1) / 2.0));
    if carry <= 0.0 || segments == 1 {
        return build(mesh, from.0, to.0, mean_point, segments, added_vertices);
    }

    let p = mesh.position(from.0);
    let q = mesh.position(to.0);
    let carried_points = carried(
        p,
        q,
        (from_point, from_tangent),
        (chain.point_at(mean_reads), mean_tangent),
        (to_point, to_tangent),
        segments,
    );
    row_of(mesh, from.0, to.0, segments, added_vertices, |s| {
        let drawn = arc_point(p, mean_point, q, s as f64 / segments as f64);
        (drawn * (1.0 - carry)) + (carried_points[s as usize] * carry)
    })
}

/// A closed chain's row whose two ends both stand on a sharp joint's
/// seam plane, one of them on the MITRE line out of the joint, drawn on the seam
/// ([`seam_row`]); `None` for any other row.
pub fn in_the_seam(
    mesh: &mut LevelLineMesh,
    chain: &ChainField,
    from: RowEnd,
    to: RowEnd,
    segments: i32,
    added_vertices: &mut Vec<i32>,
) -> Option<Vec<i32>> {
    if segments == 1 {
        return None;
    }

    let length = chain.length;
    let p = mesh.position(from.0);
    let q = mesh.position(to.0);
    for &(joint_at, joint_setback) in chain.sharp_joints() {
        let offset = |at: f64| {
            let offset = at - joint_at;
            offset - ((offset / length).round_ties_even() * length)
        };

        if offset(from.1).abs() > 2.0 * joint_setback || offset(to.1).abs() > 2.0 * joint_setback {
            continue;
        }

        let apex = chain.point_at(joint_at);
        let tolerance = seam_tolerance(apex, joint_setback);
        let step = 1e-3 * joint_setback;
        let into = (apex - chain.point_at(wrapped(length, joint_at - step))).get_normal();
        let out_of = (chain.point_at(wrapped(length, joint_at + step)) - apex).get_normal();
        let normal = (into + out_of).get_normal();
        let slide = into.dot(normal);
        if (p - apex).dot(normal).abs() > tolerance
            || (q - apex).dot(normal).abs() > tolerance
            || slide.abs() < 1e-6
        {
            continue;
        }

        // Only the MITRE's row: one end on the outer bisector out of the joint, where the two
        // fillets carried on meet; an end elsewhere on the plane is held by another crease.
        let mitre = (into - out_of).get_normal();
        let on_the_mitre = |x: Vector3| {
            (x - apex).dot(mitre) > tolerance && (x - apex).cross(mitre).length() <= tolerance
        };
        if !on_the_mitre(p) && !on_the_mitre(q) {
            continue;
        }

        // Each end's offset from the incoming leg's axis: its point in that leg's section.
        let section = |x: Vector3| (x - apex) - (into * (x - apex).dot(into));
        if section(p).length() <= tolerance || section(q).length() <= tolerance {
            continue;
        }

        let points = seam_row(apex, into, out_of, p, q, segments);
        return Some(row_of(mesh, from.0, to.0, segments, added_vertices, |s| {
            points[s as usize]
        }));
    }

    None
}

/// An OPEN chain's row across a sharp joint's seam - both ends on
/// the seam plane at the setback from both legs, one inside the joint and one outside - drawn on
/// the seam ([`seam_row`]); `None` for any other row.
pub fn on_a_rim_joints_seam(
    mesh: &mut LevelLineMesh,
    chain: &ChainField,
    from: i32,
    to: i32,
    segments: i32,
    added_vertices: &mut Vec<i32>,
) -> Option<Vec<i32>> {
    if segments == 1 {
        return None;
    }

    let p = mesh.position(from);
    let q = mesh.position(to);
    for &(joint_at, joint_setback) in chain.sharp_joints() {
        let apex = chain.point_at(joint_at);
        let tolerance = seam_tolerance(apex, joint_setback);
        let step = 1e-3 * joint_setback;
        let into = (apex - chain.point_at(ieee_math::max_f64(0.0, joint_at - step))).get_normal();
        let out_of =
            (chain.point_at(ieee_math::min_f64(chain.length, joint_at + step)) - apex).get_normal();
        let normal = (into + out_of).get_normal();
        if (p - apex).dot(normal).abs() > tolerance || (q - apex).dot(normal).abs() > tolerance {
            continue;
        }

        let to_line =
            |x: Vector3, along: Vector3| ((x - apex) - (along * (x - apex).dot(along))).length();
        let at_the_setback = |x: Vector3| {
            (to_line(x, into) - joint_setback).abs() <= tolerance
                && (to_line(x, out_of) - joint_setback).abs() <= tolerance
        };
        let inside = (out_of - into).get_normal();
        let is_inside = |x: Vector3| (x - apex).dot(inside) > tolerance;
        if !at_the_setback(p) || !at_the_setback(q) || is_inside(p) == is_inside(q) {
            continue;
        }

        let points = seam_row(apex, into, out_of, p, q, segments);
        return Some(row_of(mesh, from, to, segments, added_vertices, |k| {
            points[k as usize]
        }));
    }

    None
}
