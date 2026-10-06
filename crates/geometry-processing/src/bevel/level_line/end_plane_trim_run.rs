//! The trim of a chain's two rails to its end plane and the section laid in that plane between
//! them. Ports `EndPlaneTrim.TrimEnd` and `EndPlaneTrim.Run` from
//! `Bevel/LevelLine/EndPlaneTrim.cs`; the geometry they read (the end plane, the corner, the
//! farthest reach, the section, the cut) is in `end_plane_trim`. Each refusal notes why on the open
//! gap fill's ends site, as C# does.

use vector_math::number_format::format_f64_max_decimals;
use vector_math::{cs_math, Vector3};

use super::end_plane_trim::{
    at_ray_parameter, cut_at, end_plane, farthest_out, section, to_the_corner,
};
use super::open_gap_fill::append_ends_site;
use super::{BandProfile, ChainField, GapLoop, LevelLineMesh, LevelLineMeshError};

/// C#'s `{0:0.###}`.
fn three(value: f64) -> String {
    format_f64_max_decimals(value, 3)
}

/// C#'s bool interpolation.
fn cs_bool(value: bool) -> &'static str {
    if value {
        "True"
    } else {
        "False"
    }
}

/// C#'s `{0:G3}`: three significant digits, scientific (`E+00`) where the exponent is below -5 or
/// at least 3, trailing zeros dropped.
fn g3(value: f64) -> String {
    if !value.is_finite() {
        return if value.is_nan() {
            "NaN".to_string()
        } else if value > 0.0 {
            "∞".to_string()
        } else {
            "-∞".to_string()
        };
    }

    if value == 0.0 {
        return "0".to_string();
    }

    let formatted = format!("{value:.2e}");
    let (mantissa, exponent) = formatted.split_once('e').unwrap_or((&formatted, "0"));
    let exponent: i32 = exponent.parse().unwrap_or(0);
    let trim = |text: String| {
        if text.contains('.') {
            text.trim_end_matches('0').trim_end_matches('.').to_string()
        } else {
            text
        }
    };
    if !(-5..3).contains(&exponent) {
        let sign = if exponent < 0 { '-' } else { '+' };
        return format!("{}E{sign}{:02}", trim(mantissa.to_string()), exponent.abs());
    }

    let decimals = (2 - exponent).max(0) as usize;
    let rounded: f64 = format!("{mantissa}e{exponent}").parse().unwrap_or(value);
    trim(format!("{rounded:.decimals$}"))
}

/// LINQ `Min` / `Max` over a run's parameters (NaN first for `Min`, NaN only if all for `Max`).
fn least(values: &[f64]) -> f64 {
    values[1..].iter().fold(values[0], |m, &v| {
        if m.is_nan() || v.is_nan() {
            f64::NAN
        } else {
            cs_math::min_f64(m, v)
        }
    })
}

fn greatest(values: &[f64]) -> f64 {
    values
        .iter()
        .fold(f64::NAN, |m, &v| if m.is_nan() || v > m { v } else { m })
}

/// The chain's end point at `last` and its outward unit direction there.
fn end_and_outward(chain: &ChainField, last: bool) -> (Vector3, Vector3) {
    let points = chain.points();
    let n = points.len();
    let end = if last { points[n - 1] } else { points[0] };
    let outward = if last {
        points[n - 1] - points[n - 2]
    } else {
        points[0] - points[1]
    }
    .get_normal();
    (end, outward)
}

/// C# `EndPlaneTrim.TrimEnd`: trims the two rails (`left`, `right`) to the chain's end plane at
/// its first or `last` end - to the corners where a bounded end's rails meet the plane, then cut
/// where either rail end pokes out past the section's reach - and lays the section between them in
/// the end face ([`run`]). Answers the run round the end: the first rail's cut-off stretch out to
/// its end, the section, the other's stretch back; `None` with the reason on the ends site where
/// it cannot. The rails are updated in place; where a rail has no corner it is left as it was
/// (C# leaves the null it read).
pub fn trim_end(
    mesh: &mut LevelLineMesh,
    chain: &ChainField,
    left: &mut GapLoop,
    right: &mut GapLoop,
    last: bool,
    profile: BandProfile,
    fillet_segments: i32,
) -> Result<Option<Vec<i32>>, LevelLineMeshError> {
    let which = if last { 1 } else { 0 };
    let first = |run: &GapLoop| run.vertices[0];
    let final_ = |run: &GapLoop| run.vertices[run.vertices.len() - 1];

    // End 0 runs from the right run's last vertex to the left run's first, end 1 from the left's
    // last to the right's first; the rail ends P are those vertices.
    let mut pl = if last { final_(left) } else { first(left) };
    let mut pr = if last { first(right) } else { final_(right) };
    let (end, outward) = end_and_outward(chain, last);
    let (plane, _) = end_plane(mesh, end, outward, pl, pr, 0);
    let Some(normal) = plane else {
        append_ends_site(&format!(" / trim {which}: no end plane"));
        return Ok(None);
    };

    let square = |p: Vector3| p - (outward * (p - end).dot(outward));
    let mut reach = farthest_out(
        square(mesh.position(pl)),
        square(mesh.position(pr)),
        end,
        normal,
        profile,
    );
    let end_parameter = if last { chain.length } else { 0.0 };
    let sign = if last { 1.0 } else { -1.0 };
    let mut t_star = end_parameter - (sign * reach / outward.dot(normal));
    let bounded = if last {
        chain.end_ray_bounds.1.is_some()
    } else {
        chain.end_ray_bounds.0.is_some()
    };
    if bounded {
        // A bounded end's field is clipped by the end plane, and a rail point on the plane reads
        // the END's parameter; the runs end at the corners where the rails meet the plane.
        let setback = chain.setback_at(end_parameter);
        let (new_left, left_corner_inside) =
            to_the_corner(mesh, left, last, end, outward, normal, setback);
        let (new_right, right_corner_inside) =
            to_the_corner(mesh, right, !last, end, outward, normal, setback);
        let (Some(new_left), Some(new_right)) = (new_left.clone(), new_right.clone()) else {
            append_ends_site(&format!(
                " / trim {which}: no corner (left {}, right {}, setback {})",
                cs_bool(new_left.is_some()),
                cs_bool(new_right.is_some()),
                three(setback)
            ));
            return Ok(None);
        };
        *left = new_left;
        *right = new_right;

        pl = if last { final_(left) } else { first(left) };
        pr = if last { first(right) } else { final_(right) };
        if left_corner_inside || right_corner_inside {
            // A corner kept inside the setback is where the rail really meets the plane, so the
            // section reaches out from it.
            reach = farthest_out(
                square(mesh.position(pl)),
                square(mesh.position(pr)),
                end,
                normal,
                profile,
            );
            t_star = end_parameter - (sign * reach / outward.dot(normal));
        }

        let left_index = if last { left.vertices.len() - 1 } else { 0 };
        *left = at_ray_parameter(mesh, left, left_index, end, outward, end_parameter, last);
        let right_index = if last { 0 } else { right.vertices.len() - 1 };
        *right = at_ray_parameter(mesh, right, right_index, end, outward, end_parameter, last);
    }

    // The rails are cut where EITHER P pokes out past t*, not only the left's.
    let t_left = if last {
        left.parameters[left.parameters.len() - 1]
    } else {
        left.parameters[0]
    };
    let t_right = if last {
        right.parameters[0]
    } else {
        right.parameters[right.parameters.len() - 1]
    };
    let t_p = if last {
        cs_math::max_f64(t_left, t_right)
    } else {
        cs_math::min_f64(t_left, t_right)
    };
    let mut stretch_left = Vec::new();
    let mut stretch_right = Vec::new();
    if if last { t_star < t_p } else { t_star > t_p } {
        let cut_left = cut_at(mesh, left, t_star, !last)?;
        let cut_right = cut_at(mesh, right, t_star, last)?;
        let (Some((cut_left, sl)), Some((cut_right, sr))) = (cut_left.clone(), cut_right.clone())
        else {
            append_ends_site(&format!(
                " / trim {which}: no cut at {} (left {} [{}, {}], right {} [{}, {}], right P - t* {}, left P - t* {}, tP {}, reach {})",
                three(t_star),
                cs_bool(cut_left.is_some()),
                three(least(&left.parameters)),
                three(greatest(&left.parameters)),
                cs_bool(cut_right.is_some()),
                three(least(&right.parameters)),
                three(greatest(&right.parameters)),
                g3(t_right - t_star),
                g3(t_left - t_star),
                three(t_p),
                three(reach)
            ));
            return Ok(None);
        };

        *left = cut_left;
        *right = cut_right;
        stretch_left = sl;
        stretch_right = sr;
    }

    let from = if last { pl } else { pr };
    let to = if last { pr } else { pl };
    let Some(the_section) = run(mesh, chain, from, to, last, profile, fillet_segments, 0)? else {
        append_ends_site(&format!(" / trim {which}: no section"));
        return Ok(None);
    };

    // The run: the first run's stretch out to its P, the section, the other's stretch back.
    let (head, tail) = if last {
        (stretch_left, stretch_right)
    } else {
        (stretch_right, stretch_left)
    };
    let mut result = Vec::new();
    if !head.is_empty() {
        result.extend_from_slice(&head);
        result.extend_from_slice(&the_section[1..]);
    } else {
        result.extend_from_slice(&the_section);
    }
    if !tail.is_empty() {
        result.extend(tail.iter().rev().skip(1));
    }

    Ok(Some(result))
}

/// C# `EndPlaneTrim.Run`: the section from rail end `from` to rail end `to` inserted into the end
/// face (`end_face_tag`'s triangles in the chain's end plane, to the float input's noise) as a
/// chain of edges, and the end face's part it encloses round the end vertex taken out. `None`
/// with the reason on the ends site where there is no end plane or the section does not insert
/// end to end.
#[allow(clippy::too_many_arguments)]
pub fn run(
    mesh: &mut LevelLineMesh,
    chain: &ChainField,
    from: i32,
    to: i32,
    last: bool,
    profile: BandProfile,
    fillet_segments: i32,
    end_face_tag: i32,
) -> Result<Option<Vec<i32>>, LevelLineMeshError> {
    let (end, outward) = end_and_outward(chain, last);
    let (plane, end_vertex) = end_plane(mesh, end, outward, from, to, end_face_tag);
    let Some(normal) = plane else {
        append_ends_site(" / section: no end plane");
        return Ok(None);
    };

    let in_face = |mesh: &LevelLineMesh, t: i32| {
        let tri = mesh.triangles()[t as usize];
        if tri.tag != end_face_tag {
            return false;
        }

        let a = mesh.position(tri.a);
        let n = (mesh.position(tri.b) - a)
            .cross(mesh.position(tri.c) - a)
            .get_normal();
        // The input is float: a vertex made on the side-to-end crease stands a few 1e-6 off the
        // plane read through the end vertex, and must still be in the face.
        1.0 - n.dot(normal).abs() <= 1e-9
            && (a - end).dot(normal).abs() <= 1e-5 * cs_math::max_f64(1.0, (a - end).length())
    };

    let points = section(
        mesh.position(from),
        mesh.position(to),
        end,
        outward,
        normal,
        profile,
        fillet_segments,
    );
    // The region test reads the mesh as it stands at each ask, as C#'s closure does.
    let inserted = mesh.insert_polyline(&points, normal, &in_face)?;
    let inserted = match inserted {
        Some(chain_of_edges)
            if chain_of_edges[0] == from && chain_of_edges[chain_of_edges.len() - 1] == to =>
        {
            chain_of_edges
        }
        other => {
            append_ends_site(&format!(
                " / section not inserted ({}, from {} to {} off the plane)",
                if other.is_none() { "none" } else { "ends off" },
                format_f64_max_decimals((mesh.position(from) - end).dot(normal), 6),
                format_f64_max_decimals((mesh.position(to) - end).dot(normal), 6)
            ));
            return Ok(None);
        }
    };

    // The removal changes no live triangle's tag or plane, so the face is read once before it.
    let triangles: Vec<bool> = (0..mesh.triangle_count())
        .map(|t| in_face(mesh, t))
        .collect();
    mesh.remove_enclosed(end_vertex, &inserted, &|t| triangles[t as usize]);
    Ok(Some(inserted))
}
