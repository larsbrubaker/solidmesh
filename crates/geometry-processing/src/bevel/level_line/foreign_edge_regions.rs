//! The pick's candidate foreign edges, filed by the pair of region tags they stand between
//! ([`ForeignEdgeRegions`]), with one open chain's growing creases on a pick of several
//! ([`ForeignEdgeRegionsChainGrowth`]). Ports the types of `Bevel/LevelLine/ForeignEdgeRegions.cs`;
//! the filing and the per-chain queries land with the room-reading method package.

use std::cell::RefCell;
use std::collections::HashSet;
use std::rc::Rc;

use indexmap::IndexMap;

use super::{ForeignEdgeIndex, GrowingCreaseRuns, LevelLineMesh};

/// C# `ForeignEdgeRegions`: the pick's candidate foreign edges, read off the mesh ONCE and filed by
/// the pair of region tags they stand between, so a reading of many chains builds the geometry
/// once and each chain only says WHICH of the files are foreign to it.
///
/// The membership test is unchanged and still per chain: an edge is foreign to chain k when k
/// seeds the region on ONE side of it and not the other. That test reads nothing about the edge
/// but the two TAGS across it, so every edge carrying the same unordered tag pair is foreign to
/// exactly the same chains - which is what makes one index per pair the exact filing of the
/// per-chain sets and not an approximation of them. An edge with the SAME tag either side can
/// never be foreign to anybody and is not filed at all. Not filed either: an edge that is any
/// chain's own, and an edge not standing on exactly two triangles.
#[derive(Debug)]
pub struct ForeignEdgeRegions {
    /// The working mesh the edges are on.
    pub mesh: Rc<LevelLineMesh>,
    /// The filed edges, by (low, high) tag pair, in filing order.
    pub by_tag_pair: IndexMap<(i32, i32), Vec<(i32, i32)>>,
    /// The box tree over each tag pair's edges, built on first ask.
    pub indexed: RefCell<IndexMap<(i32, i32), Rc<ForeignEdgeIndex>>>,
    /// Per chain, on a pick of SEVERAL open chains only (`None` otherwise; `None` for a closed
    /// chain): what that chain's own ends change, as its one-chain reading has them - the edges not
    /// foreign to it though their tags say they are (its side-to-end folds, its growing ends'
    /// crease stretches, the growing creases its neighbours grow over) and the edges that bound its
    /// room instead (its end regions' rims, what those growing creases grow into).
    pub per_chain: Option<Vec<Option<ForeignEdgeRegionsChainGrowth>>>,
    /// The growing creases met while filing the edges, for the profile-pierce room; `None` where
    /// none.
    pub growing: Option<Rc<GrowingCreaseRuns>>,
}

/// C# `ForeignEdgeRegions.ChainGrowth`: one open chain's growing creases on a pick of several -
/// what it sets aside and reads instead.
#[derive(Debug)]
pub struct ForeignEdgeRegionsChainGrowth {
    /// The chain's end vertices.
    pub ends: HashSet<i32>,
    /// The chain's growing ends' crease stretches, as (low, high) edges.
    pub stretch: HashSet<(i32, i32)>,
    /// Whether the first end keeps the run-out exemption - false where it has an end region.
    pub first_runs_out: bool,
    /// Whether the last end keeps the run-out exemption - false where it has an end region.
    pub last_runs_out: bool,
    /// The growing creases this chain's neighbours grow over, for its profile-pierce room.
    pub runs: Rc<GrowingCreaseRuns>,
    /// The edges not foreign to this chain though their tags say they are.
    pub set_aside: HashSet<(i32, i32)>,
    /// The edges that bound this chain's room instead, in the order they were read.
    pub bounding: Vec<(i32, i32)>,
    /// The box tree over [`ForeignEdgeRegionsChainGrowth::bounding`], built on first ask.
    pub index: RefCell<Option<Rc<ForeignEdgeIndex>>>,
}
