//! Whether a rewritten surface still closes: every edge of it used by exactly two triangles, one
//! running each way along it. Ports `Bevel/Surface/SurfaceMeshClosure.cs`.
//!
//! It is the one thing the surface construction promises that cannot be read off any single step
//! of it, so it is read off the answer, in production, before anything is published.
//!
//! By vertex INDEX rather than by position, which is the whole point of a construction that shares
//! its seam vertices as it makes them ([`super::SurfaceVertexPool`]): two faces that meet along a
//! seam use the same two indices there, and an edge used once is a crack no matter how near the
//! two rims happen to lie. An edge used three times or more is a fold, which is the other way a
//! stitch goes wrong, and both read as "not closed" here.
//!
//! And by DIRECTION, because the boolean kernel the result is handed to next reads it that way and
//! nothing else we take would notice. A triangle wound the other way round leaves each of its three
//! edges used twice - so a count alone calls it closed, as do `Mesh.IsManifold` and the shell
//! count. manifold pairs halfedges `v0 -> v1` with `v1 -> v0`; three edges with no partner running
//! the other way drop it onto the triangle-soup path, where the import fails with `NotClosed`. A
//! Fillet is normally followed by a Combine, so a surface published with a face facing the wrong way
//! is a boolean that refuses the part later rather than a bevel that refuses itself now.
//!
//! The bevel tests took this reading of the built mesh long before the builder did. It lives here
//! so that the check the tests make and the check the application makes are the same code, and so
//! that a group the construction cannot close is refused in words
//! ([`crate::bevel::BevelAdvisoryCode::SurfaceResultNotClosed`]) rather than published with a hole.
//!
//! C# reports the FIRST offending edge in `Dictionary` insertion order, so the uses are counted in
//! an [`IndexMap`]. C#'s null mesh / face list (answering 0) is not representable here.

use indexmap::IndexMap;
use polygon_mesh::Mesh;
use vector_math::Vector3;

/// How many times an undirected edge is used, and the balance of its directed uses: +1 for each
/// use running low to high, -1 for each running high to low.
type EdgeUses = IndexMap<(i32, i32), (i32, i32)>;

fn count_uses<I: IntoIterator<Item = (i32, i32, i32)>>(faces: I, capacity: usize) -> EdgeUses {
    let mut uses: EdgeUses = IndexMap::with_capacity(capacity);

    for (v0, v1, v2) in faces {
        for (first, second) in [(v0, v1), (v1, v2), (v2, v0)] {
            let edge = (first.min(second), first.max(second));
            let already = uses.get(&edge).copied().unwrap_or((0, 0));

            // The balance is what says the two faces run OPPOSITE ways along the edge, which is the
            // pairing the kernel wants: one use each way cancels to zero, and two the same way does
            // not. Counted the same way the kernel counts it, so the two readings cannot drift.
            uses.insert(
                edge,
                (
                    already.0 + 1,
                    already.1 + if first < second { 1 } else { -1 },
                ),
            );
        }
    }

    uses
}

/// Whether every edge of `mesh` is used by exactly two faces running opposite ways along it, and
/// the middle of one edge that is not, when one is not (zero when closed).
pub fn is_closed(mesh: &Mesh) -> (bool, Vector3) {
    let (open, open_at) = open_edge_count(mesh);
    (open == 0, open_at)
}

/// How many edges of a face LIST are FOLDED - used by two faces walking them the same way, or by
/// three faces or more - and the first of them, as the two vertices it runs between (low, high),
/// or `(0, 0)` when there is none.
///
/// The half of [`open_edge_count`] that can be asked of a surface still being built. An edge with
/// ONE face on it is ordinary part way through: the gap the walkback opens has a rim of them until
/// the strips are sewn into it, so a stage that checked for closure would refuse every part there
/// is. A fold is not ordinary anywhere - nothing downstream takes a face away - so the stage that
/// MADE one can be named by asking this of its own output before the next stage builds on it.
///
/// The same directed arithmetic [`open_edge_count`] counts by, on purpose: the gate a stage passes
/// and the reading the published mesh is refused by have to be one reading, or a stage can be held
/// innocent of the very fold that comes back at the end.
pub fn folded_edge_count(faces: &[(i32, i32, i32)]) -> (i32, (i32, i32)) {
    let uses = count_uses(faces.iter().copied(), faces.len() * 3);

    let mut folds = 0;
    let mut folded = (0, 0);

    for (&edge, &(count, balance)) in &uses {
        if count < 2 || (count == 2 && balance == 0) {
            continue;
        }

        if folds == 0 {
            folded = edge;
        }

        folds += 1;
    }

    (folds, folded)
}

/// How many edges of `mesh` are used by some number of faces other than two, or by two faces
/// running the same way along them, and where the first of them stands (its middle), or zero when
/// there is none.
pub fn open_edge_count(mesh: &Mesh) -> (i32, Vector3) {
    let uses = count_uses(
        mesh.faces.iter().map(|face| (face.v0, face.v1, face.v2)),
        mesh.faces.len() * 3,
    );

    let mut open = 0;
    let mut open_at = Vector3::ZERO;

    for (&(low, high), &(count, balance)) in &uses {
        if count == 2 && balance == 0 {
            continue;
        }

        if open == 0 {
            open_at = (Vector3::from_vector3_float(mesh.vertices[low as usize])
                + Vector3::from_vector3_float(mesh.vertices[high as usize]))
                / 2.0;
        }

        open += 1;
    }

    (open, open_at)
}
