//! How much room a chain's sections have on the part (`Bevel/Sections/SectionRoom.cs`, a C#
//! `internal static class`, so a module here): where one side's boundary goes per unit of radius
//! ([`Boundary`]), a region of the surface as one flood read it ([`Region`]), and everything a room
//! reading knows about the PART rather than about a chain ([`SectionRoomCache`]). The readings
//! land with the core's section packages as free functions and `impl` blocks in this module.

use indexmap::IndexMap;
use vector_math::Vector3;

use super::section_fan_walk::EdgeAtPair;

/// C# `SectionRoom.Boundary`: where one side's boundary goes per unit of radius,
/// `V + r * reach * along`, with [`Boundary::largest_that_fits`] the radius at which it runs out
/// of the part it is standing on.
#[derive(Debug, Clone, Copy, PartialEq, Default)]
pub struct Boundary {
    /// The unit direction the boundary leaves the station along.
    pub along: Vector3,
    /// How far it goes that way per unit of radius.
    pub reach: f64,
    /// The radius at which it reaches the end of its fold or of its fan.
    pub largest_that_fits: f64,
    /// Whether the boundary steps SQUARE off the chain into a flat fan - no fold leaves the
    /// station this side - which is what `SectionSide::Offset` means to the builder.
    pub is_offset: bool,
}

/// C# `SectionRoom.Region`: a region of the surface as one flood read it.
#[derive(Debug, Clone, PartialEq, Eq, Default)]
pub struct Region {
    /// The edges the flood stopped at.
    pub rim: Vec<i32>,
    /// Whether every face of the region lies in the seed face's plane - the only case in which a
    /// straight ray drawn in that plane is a path along the surface.
    pub is_one_plane: bool,
}

/// C# `SectionRoomCache`: everything a room reading knows about the PART rather than about a
/// chain, so a pass measuring several chains reads it once - the edge table the fan walk is
/// handed, the part's lattice, and the flat regions' rims as they are flooded.
///
/// The C# `Dictionary`s are [`IndexMap`]s here, filled in the C# order, except the pair table,
/// which is only ever looked up and is the fan walk's own [`EdgeAtPair`].
#[derive(Debug, Clone, PartialEq, Default)]
pub struct SectionRoomCache {
    /// The mesh edge filed under each vertex pair.
    pub edge_at_pair: EdgeAtPair,
    /// The margin every reach limit is pulled back by.
    pub lattice: f64,
    /// How far a reach is ever asked to travel - the part's own bounding diagonal. Nothing on the
    /// part can stand further away, so a walk that spends it whole has met no limit at all.
    pub span: f64,
    /// The lattice a split vertex is read on - `BevelSplitChords.LatticeFor`.
    pub chord_lattice: f64,
    /// The mesh edges touching each vertex, in edge order - what a slide's walk along a crease
    /// steps by, and so a table of the part rather than of a chain.
    pub edges_at_vertex: IndexMap<i32, Vec<i32>>,
    /// Each face's SMOOTH region as flooded with the operation's feature reading - everything
    /// reachable without crossing a feature - filled as the regions are walked. Filed by face
    /// alone, so a cache is one pass's and one feature reading's.
    pub feature_regions_by_face: IndexMap<i32, Region>,
    /// Each face's FLAT region as flooded by coplanarity - the reading a caller with no operation
    /// in hand has to take. Held apart from the feature reading so one pass's two answers cannot
    /// be confused.
    pub coplanar_regions_by_face: IndexMap<i32, Region>,
}
