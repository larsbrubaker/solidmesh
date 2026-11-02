//! A one-chain band's side tags read from the triangles the rolling ball rolled on
//! ([`BallSides`], which `rolling_ball_setbacks::with_the_sides` hands out with the ball's
//! setbacks), for the pass whose own side floods join its two sides into one region.
//!
//! **Why.** The build's tagging floods each side from the triangles beside the chain over every
//! edge that is no feature edge and no chain edge (`band_side_tagging_ends::tag_sides`). Where a
//! crease runs on unpicked past a free end, or the caller's feature edges let one side run round
//! into the other (the T Handle's pocket wall and sphere side round its free tip), the two floods
//! meet and the band is one group, which one field cuts: no side's own setback can stand for both
//! where the ball reads them apart. The ball already knows each side: the caller's face regions
//! where it gives them, its own flood otherwise (`rolling_ball_sides`), and it takes a pass only
//! where its two sides are apart. So the band takes the ball's sides: a triangle of side A is tag 1,
//! of side B tag 2, as `chain_on_the_mesh::sides_of` names left and right; every other triangle is
//! untagged, and each open end's end region is flooded from the untagged triangles round its end
//! vertex exactly as the floods' own tagging floods it (`band_side_tagging_ends::flood_ends`, the
//! end face read with the band's reach). From then on everything reads these tags off the working
//! mesh: the cut, the band's delete, the room per side (`one_chain_room`,
//! `foreign_edge_room_per_side`), the consumed rims, the loop walk and the fill.
//!
//! **Only where the floods join.** Where the floods divide the sides the band keeps their tagging:
//! without face regions the ball's sides are the same flood over the same edges, and with them the
//! floods' tagging is what the narrow face flow (`narrow_face_flow`) reads and retags, so passes
//! that built before build the same. The narrow face flow does not run on the ball's tags: it
//! retags from the floods, which here join the sides again, and its side reach (a side flowed onto
//! its gentle neighbour past a feature edge) would grow a side past the surface the ball measured
//! that side's setbacks on; a pass whose floods join never ran it before either.
//!
//! **Bounded by the contacts.** A side region can run on past where the ball's contact on it ever
//! goes, round past the chain into the other side's sweep (a step's tread in the top's region,
//! under the riser the ball rests on). The build then takes out of each side what its contacts
//! never sweep (`rolling_ball_sweep`, right after these tags), so the band is cut only where the
//! ball rolls.
//!
//! With the switch off, or where the ball does not take the pass, nothing here runs.

use std::collections::HashSet;

use vector_math::Vector3;

use super::band_side_tagging_ends::{self, EndReach, SideTaggingError};
use super::triangle_edges;
use super::ChainField;

/// One chain's two sides as the ball rolled on them, by triangle index: side A (the left, tag 1)
/// and side B (the right, tag 2). The two never share a triangle: where the floods make them one
/// surface, it is divided where they meet (`rolling_ball_sides`).
#[derive(Clone, Debug, PartialEq)]
pub struct BallSides {
    /// Whether each triangle is in side A.
    pub in_a: Vec<bool>,
    /// Whether each triangle is in side B.
    pub in_b: Vec<bool>,
}

/// The band's tags for the one chain whose edges are `chain_edges`, from the ball's `sides`: 1 on
/// side A, 2 on side B, an open chain's end regions flooded as the floods' own tagging floods them
/// (3 at the first end, 4 at the last, over every edge that is no feature edge), 0 elsewhere. A
/// closed chain has no end to flood.
#[allow(clippy::too_many_arguments)]
pub fn tags_of(
    positions: &[Vector3],
    indices: &[(i32, i32, i32)],
    chain_edges: &[(i32, i32)],
    closed: bool,
    is_feature_edge: &dyn Fn(i32, i32) -> bool,
    sides: &BallSides,
    end_rays: (bool, bool),
    end_face_reach: &ChainField,
) -> Result<Vec<i32>, SideTaggingError> {
    let mut tags: Vec<i32> = sides
        .in_a
        .iter()
        .zip(&sides.in_b)
        .map(|(&a, &b)| match (a, b) {
            (true, _) => 1,
            (false, true) => 2,
            (false, false) => 0,
        })
        .collect();
    if closed {
        return Ok(tags);
    }
    let standing = triangle_edges::standing_map(indices);
    let chain_keys: HashSet<(i32, i32)> = chain_edges
        .iter()
        .map(|&(f, t)| triangle_edges::key(f, t))
        .collect();
    band_side_tagging_ends::flood_ends(
        positions,
        indices,
        &standing,
        &chain_keys,
        is_feature_edge,
        chain_edges,
        &mut tags,
        None,
        end_rays,
        EndReach {
            end_reach: None,
            end_face_reach: Some(end_face_reach),
        },
        false,
    )?;
    Ok(tags)
}
