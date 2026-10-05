//! The trim of a rail to a chain's end plane. Ports `Bevel/LevelLine/EndPlaneTrim.cs` (a C# static
//! class): so far a run trimmed to the end-face corner ([`to_the_corner`]), a run vertex read at
//! its end-ray parameter ([`at_ray_parameter`]) and the end face's plane at the chain's end
//! ([`end_plane`]); the trim itself lands with later packages.

use vector_math::{cs_math, Vector3};

use super::super::float_grid;
use super::{GapLoop, LevelLineMesh};

/// The run with its vertices inside the end's setback (measured square to `outward`) trimmed off
/// its tail (or head), stopping at the end-face corner - the last vertex on the end plane before
/// the rail leaves it - with whether the vertex it stopped at is inside; `None` when fewer than two
/// vertices are left.
#[allow(clippy::too_many_arguments)] // C#'s signature, `out cornerInside` returned.
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
                * cs_math::max_f64(
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
