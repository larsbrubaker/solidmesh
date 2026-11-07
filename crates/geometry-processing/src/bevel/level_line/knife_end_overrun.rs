//! The corner strip's end at a bounded free end whose END PLANE E is the knife neighbour's own
//! plane: one rail reaches E inside the chain while the other runs on past the end along the end
//! ray to E, and the round past the first rail's end is laid as rows clipped by E, so the end hole
//! left is a flat piece of E. The end read ([`KnifeEndOverrunEnd`]) and the on-the-plane test
//! ([`on_the_plane`]), the overrun read at one end ([`read`]) and the rows laid past the end
//! spoke ([`lay`]).

use vector_math::{ieee_math, Vector3};

use super::{ChainField, CornerFillPiece, LevelLineMesh, WorkError};

/// The end read - which end, its plane, and the direction square to the
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
    (p - vertex).dot(normal).abs() <= 1e-6 * ieee_math::max_f64(1.0, (p - vertex).length())
}

/// The overrun at one end of the chain (`last`, else its first), or
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

/// The open-edge neighbour of `outermost` (not `inward`) that
/// stands on the end's plane, or `-1`.
pub fn meets_the_plane(
    mesh: &super::LevelLineMesh,
    outermost: i32,
    inward: i32,
    end: &KnifeEndOverrunEnd,
) -> i32 {
    for (low, high) in mesh.edges() {
        let other = if low == outermost {
            high
        } else if high == outermost {
            low
        } else {
            -1
        };

        if other < 0
            || other == inward
            || mesh
                .triangles_on(low, high)
                .iter()
                .filter(|&&t| !mesh.is_removed(t))
                .count()
                != 1
        {
            continue;
        }

        if on_the_plane(mesh.position(other), end.vertex, end.normal) {
            return other;
        }
    }

    -1
}

/// Lays the rows past the strip's end spoke `end_spoke` (A to B): one at
/// every station of the running-on rail `bv` beyond index `from`, each the fillet section there
/// clipped by E, then a fan to the vertex where that rail meets E.
#[allow(clippy::too_many_arguments)]
pub fn lay(
    mesh: &mut LevelLineMesh,
    chain: &ChainField,
    end: &KnifeEndOverrunEnd,
    end_spoke: &[i32],
    bv: &[i32],
    bp: &[f64],
    from: i32,
    segments: i32,
    added: &mut Vec<i32>,
    added_vertices: &mut Vec<i32>,
) -> Result<(), super::LevelLineMeshError> {
    use super::gap_fill::{arc_point, quads};

    let off = |p: Vector3| (p - end.vertex).dot(end.normal);

    // A cell is laid with its higher-parameter row first, as the strip lays its quads.
    let cell = |mesh: &mut LevelLineMesh,
                beyond: &[i32],
                previous: &[i32],
                added: &mut Vec<i32>|
     -> Result<(), super::LevelLineMeshError> {
        if end.last {
            quads(mesh, beyond, previous, added)
        } else {
            quads(mesh, previous, beyond, added)
        }
    };

    let mut previous = end_spoke.to_vec();
    let step: i32 = if end.last { 1 } else { -1 };
    let mut k = from + step;
    while k >= 0 && (k as usize) < bv.len() {
        let p = mesh.position(bv[k as usize]);
        if off(p) > 0.0 {
            break;
        }

        let (_, c) = chain.point_on_the_ray(bp[k as usize]);
        let q = c + (end.across * (p - c).length());
        let at = |s: f64| arc_point(p, c, q, s);
        let mut cut = 1.0;
        if off(at(1.0)) > 0.0 {
            let (mut inside, mut outside) = (0.0, 1.0);
            for _ in 0..60 {
                let mid = (inside + outside) / 2.0;
                if off(at(mid)) > 0.0 {
                    outside = mid;
                } else {
                    inside = mid;
                }
            }

            cut = (inside + outside) / 2.0;
        }

        // A to B, as the strip's spokes run: the end on E first, the rail station last.
        let mut row = vec![0; segments as usize + 1];
        row[segments as usize] = bv[k as usize];
        for i in 0..segments {
            row[i as usize] = mesh.add_vertex(at(cut * (segments - i) as f64 / segments as f64));
            added_vertices.push(row[i as usize]);
        }

        cell(mesh, &row, &previous, added)?;
        previous = row;
        k += step;
    }

    // The rail's outermost station is not on E: the fan to the vertex where the kept boundary
    // meets E closes the round there.
    let outermost = if end.last { bv.len() as i32 - 1 } else { 0 };
    let inward = if bv.len() > 1 {
        bv[(outermost - step) as usize]
    } else {
        -1
    };
    let on_e = meets_the_plane(mesh, bv[outermost as usize], inward, end);
    if on_e >= 0 {
        cell(mesh, &vec![on_e; segments as usize + 1], &previous, added)?;
    }

    Ok(())
}
