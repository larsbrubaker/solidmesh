//! The corner strip's end at a bounded free end whose END PLANE E is the knife neighbour's own
//! plane: one rail reaches E inside the chain while the other runs on past the end along the end
//! ray to E, and the round past the first rail's end is laid as rows clipped by E, so the end hole
//! left is a flat piece of E. Ports the nested type of `Bevel/LevelLine/KnifeEndOverrun.cs` (a C#
//! static class): the end read ([`KnifeEndOverrunEnd`]) and the on-the-plane test
//! ([`on_the_plane`]), and the overrun read at one end ([`read`]); laying the rows lands with a
//! later package.

use vector_math::{cs_math, Vector3};

use super::{ChainField, CornerFillPiece, LevelLineMesh, WorkError};

/// C# `KnifeEndOverrun.End`: the end read - which end, its plane, and the direction square to the
/// chain, in the stopped rail's face, the rows are drawn toward. Its init-only properties are
/// public fields here.
#[derive(Debug, Clone, Copy, PartialEq, Default)]
pub struct KnifeEndOverrunEnd {
    /// True at the chain's last end, false at its first.
    pub last: bool,
    /// Where the end vertex stands.
    pub vertex: Vector3,
    /// The unit normal of the end plane E.
    pub normal: Vector3,
    /// The direction square to the chain, in the stopped rail's face, the rows are drawn toward.
    pub across: Vector3,
}

/// True when `p` stands on the plane through `vertex` of normal `normal`, to a millionth of its
/// distance from the vertex (at least 1).
pub fn on_the_plane(p: Vector3, vertex: Vector3, normal: Vector3) -> bool {
    (p - vertex).dot(normal).abs() <= 1e-6 * cs_math::max_f64(1.0, (p - vertex).length())
}

/// C# `KnifeEndOverrun.Read`: the overrun at one end of the chain (`last`, else its first), or
/// `None`. `climbing` is the rail whose walk climbs the parameter, `falling` the other. Where it
/// applies, the climbing rail's vertices on E at that end are cut back to the innermost one - the
/// stretch along E is the end hole's - and that one is given its own parameter along the chain
/// (the clip reads it at the end), so the strip stops where the rail leaves E.
///
/// A falling rail whose end already stands ON E (read past the end only by its own parameter) does
/// not run on to E: Inner Right Sides and Center's two beads end on the notch wall that way, and
/// laid as an overrun the whole pick crossed itself.
pub fn read(
    mesh: &LevelLineMesh,
    chain: &ChainField,
    climbing: &mut CornerFillPiece,
    falling: &CornerFillPiece,
    tie: f64,
    last: bool,
) -> Result<Option<KnifeEndOverrunEnd>, WorkError> {
    let bound = if last {
        chain.end_ray_bounds.1
    } else {
        chain.end_ray_bounds.0
    };
    let rays = if last {
        chain.end_rays.1
    } else {
        chain.end_rays.0
    };
    let count = climbing.parameters.len();
    let Some(bound) = bound else {
        return Ok(None);
    };
    if !rays || count < 2 || falling.parameters.is_empty() {
        return Ok(None);
    }

    let points = &chain.points;
    let n = points.len();
    let vertex = if last { points[n - 1] } else { points[0] };
    let end_parameter = if last { chain.length } else { 0.0 };
    let normal = bound.plane_normal.get_normal();
    let on_e = |v: i32| on_the_plane(mesh.position(v), vertex, normal);

    // The climbing rail meets the end at its tail at the last end and at its head at the first;
    // the falling rail runs on past it at its other end.
    let from_the_end = |k: usize| if last { count - 1 - k } else { k };
    let mut runs_on = if last {
        falling.parameters[0] > chain.length + tie
    } else {
        falling.parameters[falling.parameters.len() - 1] < -tie
    };
    let falling_outer = if last {
        falling.vertices[0]
    } else {
        falling.vertices[falling.vertices.len() - 1]
    };
    runs_on = runs_on && !on_e(falling_outer);
    if (climbing.parameters[from_the_end(0)] - end_parameter).abs() > tie
        || !on_e(climbing.vertices[from_the_end(0)])
        || !runs_on
    {
        return Ok(None);
    }

    let mut on_e_count = 1;
    while on_e_count < count - 1 && on_e(climbing.vertices[from_the_end(on_e_count)]) {
        on_e_count += 1;
    }

    let stop = from_the_end(on_e_count - 1);
    let inner = from_the_end(on_e_count);
    let own = chain.unclipped_parameter_at(mesh.position(climbing.vertices[stop]))?;
    let stopped = &climbing.parameters;
    let inside = if last {
        own < end_parameter - tie && own > stopped[inner] + tie
    } else {
        own > end_parameter + tie && own < stopped[inner] - tie
    };
    if !inside {
        return Ok(None);
    }

    let tangent = if last {
        (points[n - 1] - points[n - 2]).get_normal()
    } else {
        (points[1] - points[0]).get_normal()
    };
    let (_, foot) = chain.point_on_the_ray(stopped[inner]);
    let mut across = mesh.position(climbing.vertices[inner]) - foot;
    across = across - (tangent * across.dot(tangent));
    if across.length() <= tie {
        return Ok(None);
    }

    climbing.parameters[stop] = own;
    let outer = on_e_count - 1;
    let first = if last { stop + 1 } else { 0 };
    climbing.vertices.drain(first..first + outer);
    climbing.parameters.drain(first..first + outer);
    Ok(Some(KnifeEndOverrunEnd {
        last,
        vertex,
        normal,
        across: across.get_normal(),
    }))
}
