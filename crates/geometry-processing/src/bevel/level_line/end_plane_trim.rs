//! The trim of a rail to a chain's end plane: a run trimmed to the end-face corner
//! ([`to_the_corner`]), a run vertex read at
//! its end-ray parameter ([`at_ray_parameter`]) and the end face's plane at the chain's end
//! ([`end_plane`]), the row's reach past a plane ([`farthest_out`]) and the round's section in the
//! end plane ([`section`]), and a run cut at an end parameter ([`cut_at`]); the trim itself is in
//! `end_plane_trim_run`.

use solidmesh_math::{ieee_math, Vector3};

use super::super::float_grid;
use super::{BandProfile, GapLoop, LevelLineMesh};

/// The run with its vertices inside the end's setback (measured square to `outward`) trimmed off
/// its tail (or head), stopping at the end-face corner - the last vertex on the end plane before
/// the rail leaves it - with whether the vertex it stopped at is inside; `None` when fewer than two
/// vertices are left.
#[allow(clippy::too_many_arguments)] // Every input the corner needs; `corner_inside` returned.
pub fn to_the_corner(
    mesh: &LevelLineMesh,
    run: &GapLoop,
    at_tail: bool,
    end: Vector3,
    outward: Vector3,
    normal: Vector3,
    setback: f64,
) -> (Option<GapLoop>, bool) {
    let mut vertices = run.vertices.clone();
    let mut parameters = run.parameters.clone();
    let inside = |v: i32| {
        let w = mesh.position(v) - end;
        (w - (outward * w.dot(outward))).length() < setback - 1e-6
    };

    // The last vertex on the end plane before the rail leaves it IS the corner, inside the setback
    // or not: where the end face meets the side obliquely the rail leaves the wall 1.3 from the
    // end ray, and read by the setback alone the whole rail was eaten 5.2 along the 12.2 chain,
    // leaving end 0 no rail to trim (RoundsFirstTests.TheJoinedCreaseBuildsOneShell).
    // On the plane to two float steps where the points stand: the part came from float storage, so
    // a vertex on the end face is off the plane read through the end by about a step at its own
    // magnitude (7.6e-6 at 100 mm), whatever the setback
    // (EndPlaneTrimCornerTests.FarFromTheOriginTheCornerIsReadOnThePlaneToAFloatStep).
    let on_the_plane = |v: i32| {
        (mesh.position(v) - end).dot(normal).abs()
            <= 2.0
                * ieee_math::max_f64(
                    float_grid::step_at_point(end),
                    float_grid::step_at_point(mesh.position(v)),
                )
    };
    let is_the_corner = |vertices: &[i32], index: usize, next: usize| {
        on_the_plane(vertices[index]) && !on_the_plane(vertices[next])
    };

    while vertices.len() > 2 {
        let count = vertices.len();
        let at = if at_tail { count - 1 } else { 0 };
        let corner = if at_tail {
            is_the_corner(&vertices, count - 1, count - 2)
        } else {
            is_the_corner(&vertices, 0, 1)
        };
        if !inside(vertices[at]) || corner {
            break;
        }

        vertices.remove(at);
        parameters.remove(at);
    }

    let corner_inside = vertices.len() >= 2
        && inside(if at_tail {
            vertices[vertices.len() - 1]
        } else {
            vertices[0]
        });
    if vertices.len() < 2 {
        return (None, corner_inside);
    }

    (
        Some(GapLoop::new(
            run.side,
            run.is_closed,
            vertices,
            parameters,
            run.chain,
            false,
            None,
        )),
        corner_inside,
    )
}

/// The run with vertex `index`'s parameter read along the end ray: the end's parameter plus how
/// far past the end it stands (outward at the last end, back at the first).
pub fn at_ray_parameter(
    mesh: &LevelLineMesh,
    run: &GapLoop,
    index: usize,
    end: Vector3,
    outward: Vector3,
    end_parameter: f64,
    last: bool,
) -> GapLoop {
    let mut parameters = run.parameters.clone();
    let along = (mesh.position(run.vertices[index]) - end).dot(outward);
    parameters[index] = end_parameter + ((if last { 1.0 } else { -1.0 }) * along);
    GapLoop::new(
        run.side,
        run.is_closed,
        run.vertices.clone(),
        parameters,
        run.chain,
        false,
        None,
    )
}

/// The unit normal, turned outward, of the largest live end-face triangle (`end_face_tag`) with a
/// corner AT `end` and not square to `outward`, with that corner; a triangle cornered on the rail
/// (`from` or `to`) only where no other has answered. `None`, -1 where there is none.
pub fn end_plane(
    mesh: &LevelLineMesh,
    end: Vector3,
    outward: Vector3,
    from: i32,
    to: i32,
    end_face_tag: i32,
) -> (Option<Vector3>, i32) {
    let mut end_vertex = -1;
    let mut plane: Option<Vector3> = None;
    let mut largest = 0.0;
    for t in 0..mesh.triangle_count() {
        let tri = mesh.triangles()[t as usize];
        if mesh.is_removed(t) || tri.tag != end_face_tag {
            continue;
        }

        let rail_cornered = tri.a == from
            || tri.b == from
            || tri.c == from
            || tri.a == to
            || tri.b == to
            || tri.c == to;
        for c in [tri.a, tri.b, tri.c] {
            if mesh.position(c) == end && (!rail_cornered || plane.is_none()) {
                let cross = (mesh.position(tri.b) - mesh.position(tri.a))
                    .cross(mesh.position(tri.c) - mesh.position(tri.a));
                let n = cross.get_normal();
                if n.dot(outward).abs() > 1e-6 && cross.length() > largest {
                    end_vertex = c;
                    largest = cross.length();
                    plane = Some(if n.dot(outward) > 0.0 { n } else { n * -1.0 });
                }
            }
        }
    }

    (plane, end_vertex)
}

/// The greatest of `(q - end) . normal` over the row at the end
/// vertex: the fillet's arc tangent to both sides at the square rail points (its greatest is at an
/// end or at its point along the normal's direction in the section), a chamfer's chord (at an
/// end).
pub fn farthest_out(
    rail_left: Vector3,
    rail_right: Vector3,
    end: Vector3,
    normal: Vector3,
    profile: BandProfile,
) -> f64 {
    let out = |q: Vector3| (q - end).dot(normal);
    let mut best = ieee_math::max_f64(out(rail_left), out(rail_right));
    if profile != BandProfile::Fillet {
        return best;
    }

    let sa = rail_left - end;
    let sb = rail_right - end;
    let half = sa.get_normal().dot(sb.get_normal()).clamp(-1.0, 1.0).acos() / 2.0;
    let bisector = (sa.get_normal() + sb.get_normal()).get_normal();
    let centre = end + (bisector * (sa.length() / half.cos()));
    let radius = sa.length() * half.tan();
    let in_section = normal - (sa.cross(sb).get_normal() * normal.dot(sa.cross(sb).get_normal()));
    if in_section.length_squared() == 0.0 {
        return best;
    }

    let top = centre + (in_section.get_normal() * radius);
    let chord_mid = (rail_left + rail_right) / 2.0;
    if (top - chord_mid).dot(end - chord_mid) >= 0.0 {
        best = ieee_math::max_f64(best, out(top));
    }

    best
}

/// The round's section in the end plane from `a` to `b` (both on the
/// plane): the row at the end vertex - the arc tangent to both sides at the rail points - sampled
/// at equal angles and each point carried along the chain onto the plane. The ends are `a` and `b`
/// themselves, to the bit.
pub fn section(
    a: Vector3,
    b: Vector3,
    end: Vector3,
    outward: Vector3,
    normal: Vector3,
    profile: BandProfile,
    fillet_segments: i32,
) -> Vec<Vector3> {
    let square = |p: Vector3| p - (outward * (p - end).dot(outward));
    let onto_the_plane = |p: Vector3| p + (outward * ((end - p).dot(normal) / outward.dot(normal)));

    let mut result = vec![a];
    if profile == BandProfile::Fillet {
        let sa = square(a) - end;
        let sb = square(b) - end;
        let half = sa.get_normal().dot(sb.get_normal()).clamp(-1.0, 1.0).acos() / 2.0;
        let bisector = (sa.get_normal() + sb.get_normal()).get_normal();
        let centre = end + (bisector * (sa.length() / half.cos()));
        let from_centre = end + sa - centre;
        let to_centre = end + sb - centre;
        let sweep = from_centre
            .get_normal()
            .dot(to_centre.get_normal())
            .clamp(-1.0, 1.0)
            .acos();
        let axis = from_centre.cross(to_centre).get_normal();
        let segments = 1.max(fillet_segments);
        for i in 1..segments {
            let angle = sweep * i as f64 / segments as f64;
            let rotated = (from_centre * angle.cos())
                + (axis.cross(from_centre) * angle.sin())
                + (axis * axis.dot(from_centre) * (1.0 - angle.cos()));
            result.push(onto_the_plane(centre + rotated));
        }
    }

    result.push(b);
    result
}

/// The run cut at chain parameter `t`, keeping the part away from its
/// head end (`at_head`) or its tail - the cut a vertex ON the end plane (an exact split, never a
/// neighbour a float step off it, or a rail vertex within placement noise of `t`) - with the cut
/// off stretch from the cut back to the end. `None` where `t` falls on no edge of the run, or on a
/// pair of rail vertices the mesh does not join: refused, never thrown.
pub fn cut_at(
    mesh: &mut LevelLineMesh,
    run: &GapLoop,
    t: f64,
    at_head: bool,
) -> Result<Option<(GapLoop, Vec<i32>)>, super::LevelLineMeshError> {
    let mut stretch = Vec::new();
    let mut vertices = run.vertices.clone();
    let mut parameters = run.parameters.clone();
    if !at_head {
        vertices.reverse();
        parameters.reverse();
    }

    // A cut within placement noise of a rail vertex IS that vertex: a split there minted a second
    // vertex 1e-7 from the first, which the end zip read as the run turning backwards.
    const SAME_STATION: f64 = 1e-5;

    // Now the end being cut is at index 0.
    for i in 0..vertices.len().saturating_sub(1) {
        let (low, high) = (
            ieee_math::min_f64(parameters[i], parameters[i + 1]),
            ieee_math::max_f64(parameters[i], parameters[i + 1]),
        );
        let along = (t - parameters[i]) / (parameters[i + 1] - parameters[i]);

        // The rail whose own P sets t is cut at P itself.
        let at_the_runs_end = i == 0 && along.abs() < SAME_STATION;
        #[allow(clippy::neg_cmp_op_on_partial_ord)]
        if !(t > low && t <= high) && !at_the_runs_end {
            continue;
        }

        let cut = if t == parameters[i + 1] || along > 1.0 - SAME_STATION {
            vertices[i + 1]
        } else if along < SAME_STATION {
            vertices[i]
        } else if mesh.triangles_on(vertices[i], vertices[i + 1]).is_empty() {
            // Two rail vertices the mesh does not join: there is no edge to cut.
            return Ok(None);
        } else {
            let a = mesh.position(vertices[i]);
            let b = mesh.position(vertices[i + 1]);
            // Exact only: the trim's cut is a vertex ON the end plane.
            mesh.split_edge(
                vertices[i],
                vertices[i + 1],
                a + ((b - a) * ((t - parameters[i]) / (parameters[i + 1] - parameters[i]))),
                false,
            )?
        };

        stretch.push(cut);
        for k in (0..=i).rev() {
            if vertices[k] != cut {
                stretch.push(vertices[k]);
            }
        }

        let mut kept_vertices = vec![cut];
        let mut kept_parameters = vec![t];
        for k in (i + 1)..vertices.len() {
            if vertices[k] != cut {
                kept_vertices.push(vertices[k]);
                kept_parameters.push(parameters[k]);
            }
        }

        if !at_head {
            kept_vertices.reverse();
            kept_parameters.reverse();
        }

        return Ok(Some((
            GapLoop::new(
                run.side,
                false,
                kept_vertices,
                kept_parameters,
                run.chain,
                false,
                None,
            ),
            stretch,
        )));
    }

    Ok(None)
}
