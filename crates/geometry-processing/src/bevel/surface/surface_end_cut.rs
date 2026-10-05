//! Cutting a chain end's profile out of the face beyond its corner. Ports
//! `Bevel/Surface/SurfaceEndCut.cs` (C# `internal static class`; a module of free functions here).
//! What is here are the two readings the end caps share with the cut: the vertices standing along a
//! straight run ([`standing_between`]) and the vertices a face list uses ([`vertices_of`]). The cut
//! itself (`TryCut`) and its profile readings land with later packages.

use indexmap::IndexSet;
use vector_math::Vector3;

use super::SurfaceVertexPool;

/// C# `StandingBetween`: the vertices standing part way along the straight run from `from` to
/// `to`, in order along it - what the walkback's own cut left on one side of the section, gathered
/// so the cap can sew to it. `standing` is C#'s `IReadOnlyCollection<int>`, read in its own order.
///
/// Read over the mesh's own vertices rather than off a named few, because the points wanted here
/// were made by the WALKBACK - in whichever region's facets its end cut crossed - and this step has
/// no list of those.
///
/// What it takes is therefore ANY vertex of the mesh within [`SurfaceVertexPool::near_enough`] of
/// the 3D segment, not only the ones that cut made: a vertex elsewhere in the part that happens to
/// fall on this few-millimetre run between the chain's corner and its tangent point would be sewn
/// in too. That false positive fails safe rather than silently - a cap fanned through a point that
/// is not on its boundary leaves an open edge, and the build is refused at the closure gate
/// (`BevelSurfacePartBuilder`, [`super::surface_mesh_closure`]) instead of published.
///
/// C# orders the finds with `List.Sort`, which is unstable; this sort is stable, so two finds at
/// the same place along the run keep `standing`'s order. Two vertices at one place along one line
/// within the ruler are one weld, which the pool does not hand out twice, so the tie does not occur.
#[allow(clippy::neg_cmp_op_on_partial_ord)]
pub fn standing_between<'a>(
    pool: &SurfaceVertexPool,
    standing: impl IntoIterator<Item = &'a i32>,
    from: i32,
    to: i32,
) -> Vec<i32> {
    let start = pool.position_of(from);
    let run = pool.position_of(to) - start;
    let mut found: Vec<(f64, i32)> = Vec::new();

    if !(run.length_squared() > 0.0) {
        return Vec::new();
    }

    // The run's own box, opened by the same nearness the test below uses: a vertex outside it cannot
    // pass, and most of the part's vertices are outside it, so this is the whole of the saving.
    let end = start + run;
    let near = pool.near_enough();
    let low = Vector3::component_min(start, end) - Vector3::new(near, near, near);
    let high = Vector3::component_max(start, end) + Vector3::new(near, near, near);

    for &vertex in standing {
        if vertex == from || vertex == to {
            continue;
        }

        let position = pool.position_of(vertex);

        if position.x < low.x
            || position.x > high.x
            || position.y < low.y
            || position.y > high.y
            || position.z < low.z
            || position.z > high.z
        {
            continue;
        }

        let at = position - start;
        let where_ = at.dot(run) / run.length_squared();

        if where_ > 0.0 && where_ < 1.0 && (at - (run * where_)).length() < pool.near_enough() {
            found.push((where_, vertex));
        }
    }

    // C# `first.Where.CompareTo(second.Where)`: the finds are strictly inside (0, 1), so no NaN.
    found.sort_by(|first, second| first.0.total_cmp(&second.0));

    found.into_iter().map(|(_, vertex)| vertex).collect()
}

/// C# `VerticesOf`: every vertex the faces use, once each. An [`IndexSet`] keeps C#'s `HashSet<int>`
/// enumeration order (insertion order, nothing being removed), which reaches
/// [`standing_between`]'s tie order.
pub fn vertices_of(faces: &[(i32, i32, i32)]) -> IndexSet<i32> {
    let mut used = IndexSet::new();

    for face in faces {
        used.insert(face.0);
        used.insert(face.1);
        used.insert(face.2);
    }

    used
}
