//! The pieces ONE bite leaves, read as neighbours of each other ([`SurfaceBiteSharedSides`]): what
//! a piece's side looks like from the piece on the other side of it. Ports the fields of
//! `Bevel/Surface/SurfaceBiteSharedSides.cs` and its chain-across reading
//! ([`SurfaceBiteSharedSides::try_the_chain_across`]); its constructor lands with a later package
//! as a further `impl` block.

use std::collections::HashMap;

use super::{surface_split_sewing, SurfaceVertexPool};

/// C# `SurfaceBiteSharedSides`: the pieces one bite leaves, read as neighbours of each other.
///
/// A side two pieces share has to leave the bite with the same vertex sequence on both of them, or
/// the two walk one directed edge the same way and that is a fold - reported by
/// `SurfaceMixedCornerPatch.NothingCoversNothing` hundreds of faces later, at a place the bite had
/// nothing to do with. Read PIECE BY PIECE that cannot be guaranteed: whether a vertex stands on a
/// side is a measurement against the part's own ruler, and on a sliver whose two sides are one line
/// to within that ruler a vertex the neighbour carries as a corner of some OTHER side measures as
/// standing on this one. Put in here and not there, it claims an edge the neighbour does not have.
///
/// So where the piece across a side is one of this bite's own, the side's sequence is READ OFF IT
/// rather than measured: two pieces wound the same way walk their shared boundary opposite ways, so
/// the chain the neighbour walks from this side's far end back to its near end IS this side,
/// exactly and with no tolerance in it (`SurfaceConvexBite.WithTheVerticesStandingOnIt`). Where no
/// piece of this bite names both of the side's ends - the face across it was never cut - there is
/// nothing to read and the measurement is still the answer.
#[derive(Debug, Clone, PartialEq, Default)]
pub struct SurfaceBiteSharedSides {
    /// The bite's pieces, each a ring of pool vertices.
    pub pieces: Vec<Vec<i32>>,
    /// Which pieces carry each vertex, in piece order, each piece once. Only ever looked up by
    /// vertex, never iterated, so a `HashMap`'s order cannot reach an output.
    pub carried_by: HashMap<i32, Vec<i32>>,
}

/// The chain-across reading (C# `TryTheChainAcross`, `TryTheChainIn`, `TryTheWalk`).
impl SurfaceBiteSharedSides {
    /// C# `TryTheChainAcross`: the vertices standing along the side from `from` to `to` of piece
    /// `of_piece`, as the piece on the other side of it has them - in this side's own order - with
    /// the piece they were read off (C#'s `across`), or `None` where no piece of this bite names
    /// both of its ends.
    ///
    /// The chain is taken only while every vertex of it really stands part way along the side, on
    /// the same reading the sewing uses ([`surface_split_sewing::stands_on_the_side`]). A neighbour
    /// that happens to carry both ends without sharing the side between them - two pieces of one
    /// face meeting at two separate corners - walks some other part of its own rim from one to the
    /// other, and that is not this side.
    ///
    /// What the piece across does NOT name is as much of the answer as what it does: a corner of
    /// ITS that this side measures as standing on it, and that the chain left out, is a corner of
    /// some other side of that same piece - and taken in here it claims an edge the neighbour has
    /// not got (`SurfaceConvexBite.WithTheVerticesStandingOnIt`).
    pub fn try_the_chain_across(
        &self,
        pool: &SurfaceVertexPool,
        of_piece: i32,
        from: i32,
        to: i32,
    ) -> Option<(Vec<i32>, &[i32])> {
        let carriers = self.carried_by.get(&from)?;

        for &piece in carriers {
            if piece == of_piece {
                continue;
            }

            let neighbour = &self.pieces[piece as usize];

            if let Some(chain) = Self::try_the_chain_in(pool, neighbour, from, to) {
                return Some((chain, neighbour));
            }
        }

        None
    }

    /// C# `TryTheChainIn`: one neighbour's own walk between the side's two ends, in this side's
    /// order, where that walk is the side itself.
    ///
    /// Walked BOTH ways round the neighbour. Two pieces wound the same way share their boundary
    /// walking it opposite ways, which is the first reading - but a piece the clip left with no area
    /// to speak of comes back in whatever order the clip walked it, and the fan is what puts its
    /// facing right (`SurfaceConvexBite`). So a neighbour that walks this side the same way round as
    /// we do is read too, and where both ways stay on the side the LONGER one is the answer: a
    /// vertex the neighbour carries between the side's two ends is a T junction this side has to
    /// take in, and the empty walk round the other way is the same two ends with the corner between
    /// them left out.
    fn try_the_chain_in(
        pool: &SurfaceVertexPool,
        neighbour: &[i32],
        from: i32,
        to: i32,
    ) -> Option<Vec<i32>> {
        let backwards = Self::try_the_walk(pool, neighbour, to, from).map(|mut backwards| {
            backwards.reverse();
            backwards
        });
        let forwards = Self::try_the_walk(pool, neighbour, from, to);

        match (backwards, forwards) {
            (Some(backwards), Some(forwards)) if backwards.len() >= forwards.len() => {
                Some(backwards)
            }
            (Some(backwards), None) => Some(backwards),
            (_, Some(forwards)) => Some(forwards),
            (None, None) => None,
        }
    }

    /// C# `TryTheWalk`: the neighbour's walk from `starting` round to `ending`, where every vertex
    /// it passes stands part way along the side between them.
    fn try_the_walk(
        pool: &SurfaceVertexPool,
        neighbour: &[i32],
        starting: i32,
        ending: i32,
    ) -> Option<Vec<i32>> {
        let mut walked = Vec::new();

        let starts = neighbour.iter().position(|&corner| corner == starting)?;

        for step in 1..=neighbour.len() {
            let at = neighbour[(starts + step) % neighbour.len()];

            if at == ending {
                return Some(walked);
            }

            if !surface_split_sewing::stands_on_the_side(
                pool,
                pool.position_of(starting),
                pool.position_of(ending),
                pool.position_of(at),
            )
            .0
            {
                // Round the neighbour's rim somewhere else, not along this side.
                return None;
            }

            walked.push(at);
        }

        None
    }
}
