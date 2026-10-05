//! A CONVEX bite taken out of one flat face of the part, bounded by a ring of points lying in that
//! face rather than by a single tangent line. Ports `Bevel/Surface/SurfaceConvexBite.cs` (C#
//! `internal static class`; a module of free functions here). What is here is the reading that
//! puts the vertices standing on a piece's sides into it ([`with_the_vertices_standing_on_it`]),
//! which reads the pieces across its sides off [`SurfaceBiteSharedSides`]; the bite itself
//! (`TryTake`) and its other readings land with later packages.
//!
//! The walkback's own cut is a half space through a straight tangent line, which is all a strip
//! along one edge ever needs (`SurfaceFacetCut`). A mixed corner's blend needs more: its footprint
//! on the cap face is bounded by the arc the rolling ball traces, and on the wall by the square the
//! bead's own stop plane leaves standing. Both are convex rings, so both are the intersection of the
//! half planes their own sides name.

use indexmap::IndexSet;

use super::{SurfaceBiteSharedSides, SurfaceVertexPool};

/// C# `WithTheVerticesStandingOnIt`: one piece with the vertices that STAND ON its sides put into
/// it - any vertex the faces this bite cuts already carry, lying within the ruler of one of the
/// piece's sides and strictly between that side's ends, becomes a corner of the ring at that place.
/// C#'s optional `shared` (null) and `ofPiece` (-1) are `None` and `-1` here; `carried` is an
/// [`IndexSet`] so it is walked in the order C#'s `HashSet<int>` enumerates.
///
/// Which is the T junction this class would otherwise leave. A cut line is drawn where the bite's
/// own rim says, and nothing stops it running through a vertex the part already had: the face on
/// the far side of that line keeps the vertex as its own corner, while the piece on this side spans
/// it with one straight side, and the two then meet along an edge one of them has cut in two.
/// Measured on "Every Chamfer Works": the cap's own vertex at (19.5913, -0.5759, 20) stands 2e-5
/// off the sector's first side, and the six edges that leaves open are all of what the rim still
/// had open.
///
/// The candidates are the vertices of the faces this bite is IN THE WAY OF, and not the whole pool:
/// a vertex that stands on a side of a piece cut out of a face is a vertex of that face or of one
/// beside it in the same plane, and all of those are among the faces the bite is already walking.
///
/// C# orders each side's finds with `List.Sort`, which is unstable; this sort is stable, so finds
/// at one place keep `carried`'s order (two of them would be one weld, so the tie does not arise).
#[allow(clippy::neg_cmp_op_on_partial_ord)]
pub fn with_the_vertices_standing_on_it(
    pool: &SurfaceVertexPool,
    piece: Vec<i32>,
    carried: &IndexSet<i32>,
    shared: Option<&SurfaceBiteSharedSides>,
    of_piece: i32,
) -> Vec<i32> {
    if piece.len() < 2 || (carried.is_empty() && shared.is_none()) {
        return piece;
    }

    let mut grown = Vec::with_capacity(piece.len() + 2);

    // What the ring already carries, which is every corner of it and every vertex put in so far.
    // Two adjoining sides both carry a vertex standing near the corner they meet at - it is within
    // the ruler of each and strictly between the ends of each - and put in twice the ring walks out
    // to it and back along a side of no length. Only looked up.
    let mut already: std::collections::HashSet<i32> = piece.iter().copied().collect();

    for corner in 0..piece.len() {
        let from = pool.position_of(piece[corner]);
        let to = pool.position_of(piece[(corner + 1) % piece.len()]);
        let along = to - from;

        grown.push(piece[corner]);

        if !(along.length_squared() > 0.0) {
            continue;
        }

        // Where the piece across this side is one of this bite's own, WHICH of ITS corners this side
        // carries is read off the neighbour rather than measured - the only way the two of them are
        // sure to carry the same sequence (SurfaceBiteSharedSides). It only ever takes candidates
        // AWAY: what the neighbour has along the side that this piece has not is an ordinary T
        // junction, and the sewing is what puts those back.
        let read = shared.and_then(|shared| {
            shared.try_the_chain_across(
                pool,
                of_piece,
                piece[corner],
                piece[(corner + 1) % piece.len()],
            )
        });

        let mut standing: Vec<(f64, i32)> = Vec::new();

        for &vertex in carried {
            if already.contains(&vertex) {
                // Already in this ring, wherever it stands.
                continue;
            }

            if let Some((across, neighbour)) = &read {
                if named_by(neighbour, vertex) && !named_by(across, vertex) {
                    // The neighbour has said which of its OWN corners this side carries. One it did not
                    // name is a corner of some other side of the neighbour, standing within the ruler
                    // of this one because the two are one line to within that ruler - and put in here
                    // it claims an edge the neighbour has not got. A vertex of a face the bite never
                    // cut is named by nobody, and that is still a T junction this side has to take in.
                    continue;
                }
            }

            let at = pool.position_of(vertex);
            let how_far = (at - from).dot(along) / along.length_squared();

            if !(how_far > 0.0) || !(how_far < 1.0) {
                continue;
            }

            if (at - (from + (along * how_far))).length() > pool.near_enough() {
                continue;
            }

            standing.push((how_far, vertex));
        }

        // Every find is strictly inside (0, 1), so no NaN reaches the comparison.
        standing.sort_by(|first, second| first.0.total_cmp(&second.0));

        for (_, vertex) in standing {
            grown.push(vertex);
            already.insert(vertex);
        }
    }

    grown
}

/// C# `NamedBy`: whether a piece names `vertex` as one of its own corners.
fn named_by(piece: &[i32], vertex: i32) -> bool {
    piece.contains(&vertex)
}
