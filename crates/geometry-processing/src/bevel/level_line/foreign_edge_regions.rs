//! The pick's candidate foreign edges, filed by the pair of region tags they stand between
//! ([`ForeignEdgeRegions`]), with one open chain's growing creases on a pick of several
//! ([`ForeignEdgeRegionsChainGrowth`]). Ports the types of `Bevel/LevelLine/ForeignEdgeRegions.cs`;
//! here too the distance to a chain ([`distance_to_chain`]), the constructors, the per-chain
//! growth readings ([`ForeignEdgeRegions::growing_for`], [`ForeignEdgeRegions::set_aside_for`],
//! [`ForeignEdgeRegions::sets_aside_any`], [`ForeignEdgeRegions::runs_out`]), the pick's open
//! chain ([`open_chain_of`]) and a chain's ends ([`chain_ends`]); the filing and the per-chain index
//! queries land with the room-reading method package.

use std::cell::RefCell;
use std::collections::{HashMap, HashSet};
use std::rc::Rc;

use indexmap::IndexMap;

use vector_math::Vector3;

use super::{ChainField, ForeignEdgeIndex, GrowingCreaseRuns, LevelLineMesh};

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
    pub mesh: Rc<RefCell<LevelLineMesh>>,
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

/// The distance from `point` to the nearest of the chain's edges (given by their keys). A NaN
/// distance wins, as C#'s `Enumerable.Min` lets it; panics on no edges, where C# throws.
pub fn distance_to_chain(
    mesh: &LevelLineMesh,
    chain_keys: &HashSet<(i32, i32)>,
    point: Vector3,
) -> f64 {
    let mut distances = chain_keys.iter().map(|&(a, b)| {
        (point - ChainField::closest_on_segment(point, mesh.position(a), mesh.position(b))).length()
    });
    let first = distances.next().expect("a chain has at least one edge");
    distances.fold(first, |least, d| {
        if least.is_nan() || d.is_nan() {
            f64::NAN
        } else if d < least {
            d
        } else {
            least
        }
    })
}

impl ForeignEdgeRegions {
    /// C#'s private constructor (with the field initializer: no index built yet, and no growing
    /// creases until the filing sets them).
    pub fn new(
        mesh: Rc<RefCell<LevelLineMesh>>,
        by_tag_pair: IndexMap<(i32, i32), Vec<(i32, i32)>>,
        per_chain: Option<Vec<Option<ForeignEdgeRegionsChainGrowth>>>,
    ) -> Self {
        ForeignEdgeRegions {
            mesh,
            by_tag_pair,
            indexed: RefCell::new(IndexMap::new()),
            per_chain,
            growing: None,
        }
    }

    /// Chain `chain`'s growth on a pick of several open chains (C#'s `perChain?[chain]`); `None`
    /// on any other pick, for a closed chain, or past the chains (where C#'s indexer throws).
    fn growth(&self, chain: i32) -> Option<&ForeignEdgeRegionsChainGrowth> {
        let per_chain = self.per_chain.as_ref()?;
        per_chain.get(usize::try_from(chain).ok()?)?.as_ref()
    }

    /// C# `ForeignEdgeRegions.GrowingFor`: the growing creases chain `chain`'s neighbours grow over,
    /// for its profile-pierce room - the pick's own [`ForeignEdgeRegions::growing`] unless the pick
    /// holds several open chains.
    pub fn growing_for(&self, chain: i32) -> Option<Rc<GrowingCreaseRuns>> {
        if self.per_chain.is_some() {
            self.growth(chain).map(|growth| growth.runs.clone())
        } else {
            self.growing.clone()
        }
    }

    /// C# `ForeignEdgeRegions.SetAsideFor`: true when `edge`, filed as foreign to chain `chain` by
    /// its tags, is not foreign to it: a growing crease of its own. Never true but on a pick of
    /// several open chains.
    pub fn set_aside_for(&self, chain: i32, edge: (i32, i32)) -> bool {
        self.growth(chain)
            .is_some_and(|growth| growth.set_aside.contains(&edge))
    }

    /// C# `ForeignEdgeRegions.SetsAsideAny`: true when chain `chain` sets any filed edge aside.
    pub fn sets_aside_any(&self, chain: i32) -> bool {
        self.growth(chain)
            .is_some_and(|growth| !growth.set_aside.is_empty())
    }

    /// C# `ForeignEdgeRegions.RunsOut`: whether chain `chain`'s ends keep the run-out exemption
    /// (`ForeignEdgeRoom.MeetsTheChainAtAnEnd`): both do but where a pick of several open chains
    /// reads an end region at an end, which, as in the one-chain reading, gives it up.
    pub fn runs_out(&self, chain: i32) -> (bool, bool) {
        match self.growth(chain) {
            None => (true, true),
            Some(growth) => (growth.first_runs_out, growth.last_runs_out),
        }
    }
}

impl ForeignEdgeRegionsChainGrowth {
    /// C#'s constructor (with the field initializers: nothing set aside or bounding yet, no
    /// index).
    pub fn new(
        runs: Rc<GrowingCreaseRuns>,
        ends: HashSet<i32>,
        stretch: HashSet<(i32, i32)>,
        first_runs_out: bool,
        last_runs_out: bool,
    ) -> Self {
        ForeignEdgeRegionsChainGrowth {
            ends,
            stretch,
            first_runs_out,
            last_runs_out,
            runs,
            set_aside: HashSet::new(),
            bounding: Vec::new(),
            index: RefCell::new(None),
        }
    }
}

/// C# `ForeignEdgeRegions.OpenChainOf`: the pick's one open chain, in order, or `None` when the
/// caller gave no chains or none is open.
pub fn open_chain_of<'a>(
    chain_edges: Option<&'a [Vec<(i32, i32)>]>,
    closed: Option<&[bool]>,
) -> Option<&'a Vec<(i32, i32)>> {
    let (chain_edges, closed) = (chain_edges?, closed?);
    chain_edges
        .iter()
        .enumerate()
        .find(|(k, edges)| !closed[*k] && !edges.is_empty())
        .map(|(_, edges)| edges)
}

/// C# `ForeignEdgeRegions.ChainEnds`: the vertices only one of the chain's edges (given by their
/// keys) touches.
pub fn chain_ends(chain_keys: &HashSet<(i32, i32)>) -> HashSet<i32> {
    let mut count: HashMap<i32, i32> = HashMap::new();
    for &(a, b) in chain_keys {
        *count.entry(a).or_insert(0) += 1;
        *count.entry(b).or_insert(0) += 1;
    }

    count
        .into_iter()
        .filter(|&(_, c)| c == 1)
        .map(|(v, _)| v)
        .collect()
}
