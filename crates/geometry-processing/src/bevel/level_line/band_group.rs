//! One SIDE GROUP of a level-line pick: the triangles one topological region of the input holds,
//! the chains that seed it, and which side of each of those chains it is. Ports
//! `Bevel/LevelLine/BandGroup.cs` (its field builder, `BandGroupFields`, lands with the cyclic
//! core).

use indexmap::IndexMap;

/// C# `BandGroup`, whose init-only properties are public fields here. The tag a group's triangles
/// carry is its `tag`, and chain 0's left is always 1 and its right always 2, so a pick of one
/// chain tags exactly what the two-sided build always tagged.
///
/// The maps are [`IndexMap`]s because C# iterates its `Dictionary`s in insertion order and the
/// level-line build reads `carried_ends`' keys in that order.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct BandGroup {
    /// The tag this group's triangles carry.
    pub tag: i32,

    /// The chains that seed this group, in ascending order - the members of its field.
    pub members: Vec<i32>,

    /// For each member chain, which of ITS two sides (1 left, 2 right) this group is.
    pub side_of_chain: IndexMap<i32, i32>,

    /// Member chains BOTH of whose sides lie in this group although no path on the part joins
    /// them: the two sides are separate regions, made one group only through ANOTHER chain's side
    /// touching both (an open edge starting at a corner of a picked face rim). Such a chain still
    /// divides its sides; the group's gap boundary runs down one of its rails and back up the
    /// other, and the corner it starts at is one hole. `side_of_chain` still names ONE side for it,
    /// so every reader that does not know this list reads the group as before; which side a loop
    /// of such a chain is on is read from the geometry.
    pub both_sided: Vec<i32>,

    /// On a pick of several chains, the chain whose FREE END's end region this group is, or -1 for
    /// a side group. Its run is part of the band's boundary round that end, not a rail of the
    /// chain, so the fill never zips it.
    pub end_of: i32,

    /// On a SIDE group, the chains that are members only through a FREE END standing on the
    /// group's region: the end face was already another chain's side, so no end region could be
    /// carved there (edges 0-1 and 2-6 of a cube: 0-1's end at 1 stands on face 1-2-6-5, a side of
    /// 2-6). The chain joins the group's field so its band cuts round that end on this face as it
    /// would on an end region of its own; its piece here is that cut - kept boundary round the
    /// end, closed by the cap, never a rail.
    pub end_members: Vec<i32>,

    /// On a SIDE group, the members CARRIED onto it past a rim step corner, each with how many of
    /// its ends were carried here. Such a member is not one of the face's own bounding chains: it
    /// reaches onto the face only round those ends, so the group's loop does not ring the face for
    /// it, and it owns one stretch of the loop per carried end.
    pub carried_ends: IndexMap<i32, i32>,

    /// On a SIDE group, the end members whose end is a CONCAVE chain's oblique end standing on the
    /// group's face: a bead grows over the face it ends on rather than cutting it, so such a
    /// member owns the pieces the cut leaves round its end - for the loop test and the fill -
    /// while its field takes no part in cutting the face.
    pub grow_over_members: Vec<i32>,
}

impl Default for BandGroup {
    /// C#'s property initialisers: `end_of` is -1 and every list and map starts empty (C# leaves
    /// `Tag` 0 and `Members`/`SideOfChain` null until an initialiser sets them; empty stands for
    /// unset here).
    fn default() -> Self {
        BandGroup {
            tag: 0,
            members: Vec::new(),
            side_of_chain: IndexMap::new(),
            both_sided: Vec::new(),
            end_of: -1,
            end_members: Vec::new(),
            carried_ends: IndexMap::new(),
            grow_over_members: Vec::new(),
        }
    }
}
