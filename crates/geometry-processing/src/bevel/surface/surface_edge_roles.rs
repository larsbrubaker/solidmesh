//! What part each edge of one chain plays in the surface build: it has a STRIP of its own, or the
//! blend at a mixed corner SWALLOWS it and it has none.
//!
//! Every corner but one is read off the two edges that meet at it, and then every edge of the chain
//! has a strip and every corner joins two of them. The exception is the corner over a chorded
//! SHARED face - the cylinder's own wall picked as a face - where the convex side is a tangent
//! chain of small chords and the bead's ball is tangent to the chord standing on the facet it
//! ROLLED to rather than to the chord arriving at the corner. The chords between lie inside the
//! blend: a strip on one of them would stand inside the surface that replaces it.
//!
//! So an edge may have no strip, and this is where the build asks - once per stage - rather than
//! each stage keeping a null and remembering what it means. It also carries which edge is a
//! corner's CUT, and which corner bounds an edge at each of its two ends. On a chain that swallows
//! nothing both answers are the neighbour they always were.

use std::collections::HashMap;
use std::sync::LazyLock;

/// The roles of one chain's edges. Its maps are only looked up, never iterated, so a `HashMap`'s
/// order cannot leak out.
#[derive(Debug, Clone, Default, PartialEq, Eq)]
pub struct SurfaceEdgeRoles {
    swallowed_by: HashMap<i32, i32>,
    cut_of_the_corner: HashMap<i32, i32>,
    // The same for a junction's CONCAVE side, where the blend swallows chords of the BEAD rather
    // than of the cut - which is what a crease chorded beside the corner leaves. Kept apart from
    // `cut_of_the_corner` so that nothing asking for the cut is handed a bead.
    bead_of_the_corner: HashMap<i32, i32>,
    bounds_where_the_chain_arrives: HashMap<i32, i32>,
    bounds_where_the_chain_leaves: HashMap<i32, i32>,
}

static EVERY_EDGE_HAS_A_STRIP: LazyLock<SurfaceEdgeRoles> =
    LazyLock::new(SurfaceEdgeRoles::default);

impl SurfaceEdgeRoles {
    /// What a chain with no mixed corner at all asks for: every edge has a strip (one shared
    /// instance, built once).
    pub fn every_edge_has_a_strip() -> &'static SurfaceEdgeRoles {
        &EVERY_EDGE_HAS_A_STRIP
    }

    /// How many edges of the chain the blend swallows, which is none on nearly every chain.
    pub fn swallowed_count(&self) -> i32 {
        self.swallowed_by.len() as i32
    }

    /// Records one mixed corner's reading.
    ///
    /// - `corner`: the corner, named by the edge the chain ARRIVES on as everywhere else.
    /// - `cut_edge`: which edge of the chain the corner's cut is.
    /// - `cut_is_arriving`: whether the convex side of the corner is the side the chain arrives
    ///   from - so the cut lies before the corner and the corner stands at the cut's LEAVING end -
    ///   rather than the side it leaves on.
    /// - `bead_edge`: which edge the corner's bead is, which is always one of the corner's own two.
    /// - `swallowed`: the edges between the cut and the corner, which owe no strip.
    pub fn read(
        &mut self,
        corner: i32,
        cut_edge: i32,
        cut_is_arriving: bool,
        bead_edge: i32,
        swallowed: &[i32],
    ) {
        self.cut_of_the_corner.insert(corner, cut_edge);

        // The cut reaches the corner at whichever of its ends faces it, and the bead at the other.
        if cut_is_arriving {
            self.bounds_where_the_chain_leaves.insert(cut_edge, corner);
            self.bounds_where_the_chain_arrives
                .insert(bead_edge, corner);
        } else {
            self.bounds_where_the_chain_arrives.insert(cut_edge, corner);
            self.bounds_where_the_chain_leaves.insert(bead_edge, corner);
        }

        for &edge in swallowed {
            self.swallowed_by.insert(edge, corner);
        }
    }

    /// Records one end of this chain standing at a JUNCTION, where the corner's other edges are on
    /// chains of their own and this chain has only its own side of it to record.
    ///
    /// - `corner`: the sentinel naming that end (the plan's junction key).
    /// - `edge_index`: this chain's own edge at the corner.
    /// - `at_the_start`: whether that end is the chain's first edge rather than its last.
    /// - `is_the_cut`: whether this chain's edge is the corner's CUT rather than its bead.
    /// - `swallowed`: the chords between that edge and the corner, which owe no strip.
    ///
    /// False where that end of that edge is already some other corner's, which is a refusal.
    pub fn read_junction(
        &mut self,
        corner: i32,
        edge_index: i32,
        at_the_start: bool,
        is_the_cut: bool,
        swallowed: &[i32],
    ) -> bool {
        let bounds = if at_the_start {
            &self.bounds_where_the_chain_arrives
        } else {
            &self.bounds_where_the_chain_leaves
        };

        // On the chorded side the junction's cut can be an edge two or three chords into the chain,
        // and that edge may already have a mixed corner of its own at the very end the junction
        // wants. Two corners on one end of one edge is a band bounded by whichever was written
        // last, so it is refused rather than overwritten.
        if bounds.contains_key(&edge_index) {
            return false;
        }

        if is_the_cut {
            self.cut_of_the_corner.insert(corner, edge_index);
        } else {
            self.bead_of_the_corner.insert(corner, edge_index);
        }

        let bounds = if at_the_start {
            &mut self.bounds_where_the_chain_arrives
        } else {
            &mut self.bounds_where_the_chain_leaves
        };
        bounds.insert(edge_index, corner);

        for &edge in swallowed {
            self.swallowed_by.insert(edge, corner);
        }

        true
    }

    /// Whether the blend at some corner swallows the edge at `edge_index`.
    pub fn is_swallowed(&self, edge_index: i32) -> bool {
        self.swallowed_by.contains_key(&edge_index)
    }

    /// Which edge of the chain is the cut of the corner named by `corner`.
    pub fn try_cut_of(&self, corner: i32) -> Option<i32> {
        self.cut_of_the_corner.get(&corner).copied()
    }

    /// Which edge of the chain really CARRIES the corner's end ring, whichever side of the corner
    /// this chain is - the cut on the convex side, the bead on the concave one.
    ///
    /// What a caller standing at a chain END wants is not "the cut" but "the edge that still has a
    /// band there", because either side of a junction can have its own chords swallowed: the CUT's
    /// where a chorded shared face carries the tangency further along, the BEAD's where a boolean
    /// split the crease and the canal's ring stands past the first piece. Asking for the cut alone
    /// left that second case closing nothing, so the junction's three ends came up two.
    pub fn try_the_edge_carrying_the_ring_at(&self, corner: i32) -> Option<i32> {
        self.cut_of_the_corner
            .get(&corner)
            .or_else(|| self.bead_of_the_corner.get(&corner))
            .copied()
    }

    /// The mixed corner standing at one end of an edge, where one does: the end the chain ARRIVES
    /// at this edge from, or (`where_the_chain_leaves`) the end it leaves by. Only the corner's own
    /// cut and bead have one - a swallowed edge has no end of its own to bound, and every other
    /// edge meets a mitre there.
    pub fn try_corner_at(&self, edge_index: i32, where_the_chain_leaves: bool) -> Option<i32> {
        if where_the_chain_leaves {
            &self.bounds_where_the_chain_leaves
        } else {
            &self.bounds_where_the_chain_arrives
        }
        .get(&edge_index)
        .copied()
    }
}
