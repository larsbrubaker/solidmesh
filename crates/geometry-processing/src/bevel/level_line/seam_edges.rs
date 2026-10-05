//! A crease DOUBLED by a ribbon microns wide is the same crease, not a wall beside one
//! ([`SeamEdges`]). Ports the type of `Bevel/LevelLine/SeamEdges.cs`; its constructor and readings
//! land with a later package.

use std::cell::RefCell;
use std::collections::{HashMap, HashSet};

use vector_math::Vector3;

/// C# `SeamEdges`: says which feature edges are a doubling ribbon's, so the whole construction can
/// stop reading them as features.
///
/// Measured on `Issue with even a single edge.mcx` (2026-09-21): 149 of its 179 size refusals at
/// radius 0.1 are a second 90-degree feature edge running parallel to the picked one 2.5 to 6
/// microns beside it, and 14 more are a chain end whose vertex is trebled 0.008 apart. Both are real
/// topology the file carries - so nothing is repaired or deleted, the reading simply stops calling
/// them walls.
///
/// **Where the test is applied, and why it is not only the room's.** The flood is what must cross
/// the seam. Were the room reading alone to step over the ribbon, `BandSideTagging.TagSides` would
/// still stop at the ribbon's far edge, the side's whole region would BE the three-micron ribbon,
/// and a size of 0.1 would swallow it. So this is folded into `isFeatureEdge` once, at the kernel's
/// entry, and flood, foreign reading and chain validation all inherit it.
///
/// **Distance to the POLYLINE, not to its vertices.** A ribbon's far edge runs parallel to the
/// picked edge along its whole length, so both its ends stand off every chain vertex by most of a
/// facet while standing microns from the chain itself.
///
/// **The chain's own edges are never a seam**, though they lie at distance zero: they are the
/// chain, and several readers ask `isFeatureEdge` about them. They are excluded by identity, not by
/// distance.
///
/// **The epsilon is a LENGTH the caller passes, never a fraction of the size asked for.** An
/// epsilon proportional to the size makes the room a function of the size and breaks the
/// construction's own promise that a pick which builds at a size builds at every smaller one.
/// Zero - the default everywhere - hands the predicate back untouched.
#[derive(Debug, Clone, PartialEq, Default)]
pub struct SeamEdges {
    /// The mesh's vertex positions.
    pub positions: Vec<Vector3>,
    /// Each chain's polyline, as positions.
    pub polylines: Vec<Vec<Vector3>>,
    /// The chains' own edges, as (low, high) vertex keys - never a seam.
    pub chain_keys: HashSet<(i32, i32)>,
    /// The OPEN chains' end vertices, the one place a cluster can sit and not be a seam.
    pub ends: Vec<Vector3>,
    /// The seam epsilon: a length, never a fraction of the size.
    pub seam_epsilon: f64,
    /// Lazily per vertex: whether it stands within the epsilon of some chain's polyline. Filled
    /// while the edges are read through a shared handle, hence the `RefCell`; only looked up, never
    /// iterated, so a `HashMap`.
    pub beside: RefCell<HashMap<i32, bool>>,
}
