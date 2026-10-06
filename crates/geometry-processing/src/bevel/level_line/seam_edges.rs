//! A crease DOUBLED by a ribbon microns wide is the same crease, not a wall beside one
//! ([`SeamEdges`]). Ports the type of `Bevel/LevelLine/SeamEdges.cs`, its constructor and the
//! readings of a point beside the chain, the seam test ([`SeamEdges::is_seam`]) and the wrapped
//! feature-edge test ([`SeamEdges::not_the_chains_seam`]).

use std::cell::RefCell;
use std::collections::{HashMap, HashSet};
use std::rc::Rc;

use vector_math::{cs_math, Vector3};

use super::{chain_on_the_mesh, triangle_edges};

/// A feature-edge test over two vertex indices (C#'s `Func<int, int, bool>`).
pub type FeatureEdgeTest = Rc<dyn Fn(i32, i32) -> bool>;

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

impl SeamEdges {
    /// C#'s private constructor (with its field initializer: nothing yet read beside).
    pub fn new(
        positions: Vec<Vector3>,
        polylines: Vec<Vec<Vector3>>,
        chain_keys: HashSet<(i32, i32)>,
        ends: Vec<Vector3>,
        seam_epsilon: f64,
    ) -> Self {
        SeamEdges {
            positions,
            polylines,
            chain_keys,
            ends,
            seam_epsilon,
            beside: RefCell::new(HashMap::new()),
        }
    }

    /// Whether input `vertex` stands within the seam epsilon of the chain, remembered per vertex.
    /// A vertex the cut has added (outside the input positions) stands on no input seam.
    pub fn beside(&self, vertex: i32) -> bool {
        if vertex < 0 || vertex as usize >= self.positions.len() {
            return false;
        }

        if let Some(&known) = self.beside.borrow().get(&vertex) {
            return known;
        }

        let within = self.point_is_beside(self.positions[vertex as usize]);
        self.beside.borrow_mut().insert(vertex, within);
        within
    }

    /// Whether `point` stands within the seam epsilon of any segment of the chain's polylines.
    pub fn point_is_beside(&self, point: Vector3) -> bool {
        self.polylines.iter().any(|polyline| {
            polyline.windows(2).any(|pair| {
                let t = Self::parameter_on_segment(point, pair[0], pair[1]);
                let nearest = pair[0] + ((pair[1] - pair[0]) * t);

                (point - nearest).length() <= self.seam_epsilon
            })
        })
    }

    /// Where `point` projects onto the segment `from`..`to`, clamped to it (zero for a segment of
    /// no length).
    #[allow(clippy::neg_cmp_op_on_partial_ord)] // C#'s `!(x > 0)`, which refuses a NaN as well.
    pub fn parameter_on_segment(point: Vector3, from: Vector3, to: Vector3) -> f64 {
        let along = to - from;
        let length_squared = along.length_squared();
        if !(length_squared > 0.0) {
            return 0.0;
        }

        let offset = point - from;
        let projected = (offset.x * along.x) + (offset.y * along.y) + (offset.z * along.z);
        cs_math::min_f64(1.0, cs_math::max_f64(0.0, projected / length_squared))
    }
}

impl SeamEdges {
    /// C# `NotTheChainsSeam`: the caller's feature test with the pick's own seams taken out of it -
    /// or the caller's test itself, unwrapped, where there is no epsilon to apply (or nothing to
    /// read it against) and the answer must not move by so much as a bit.
    #[allow(clippy::neg_cmp_op_on_partial_ord)] // C#'s `!(x > 0)`, which refuses a NaN as well.
    pub fn not_the_chains_seam(
        positions: Option<&[Vector3]>,
        chains: Option<&[Vec<i32>]>,
        closed: Option<&[bool]>,
        is_feature_edge: Option<FeatureEdgeTest>,
        seam_epsilon: f64,
    ) -> Option<FeatureEdgeTest> {
        let (Some(test), Some(positions), Some(chains)) =
            (is_feature_edge.clone(), positions, chains)
        else {
            return is_feature_edge;
        };

        if !(seam_epsilon > 0.0) {
            return Some(test);
        }

        let mut polylines = Vec::new();
        let mut ends = Vec::new();
        let mut chain_keys = HashSet::new();

        for (k, chain) in chains.iter().enumerate() {
            if chain.len() < 2 {
                continue;
            }

            let mut points: Vec<Vector3> = chain.iter().map(|&v| positions[v as usize]).collect();

            let is_closed = closed.is_some_and(|closed| k < closed.len() && closed[k]);

            if is_closed {
                points.push(points[0]);
            } else {
                ends.push(points[0]);
                ends.push(points[points.len() - 1]);
            }

            polylines.push(points);

            for (from, to) in chain_on_the_mesh::chain_edges(chain, is_closed) {
                chain_keys.insert(triangle_edges::key(from, to));
            }
        }

        if polylines.is_empty() {
            return Some(test);
        }

        let seams = SeamEdges::new(
            positions.to_vec(),
            polylines,
            chain_keys,
            ends,
            seam_epsilon,
        );

        Some(Rc::new(move |a, b| test(a, b) && !seams.is_seam(a, b)))
    }

    /// C# `IsSeam`: a doubling edge runs BESIDE the chain and is the chain's seam; a cluster of
    /// near-coincident vertices sits AT one of its ends and is not (crossing one floods round the
    /// chain's end, so it is left a feature here and taken instead by
    /// `ForeignEdgeRoom.MeetsTheChainAtAnEnd`). The cluster's shape is an edge lying wholly within
    /// the epsilon of ONE end vertex - the SAME end, since a ribbon beside a one-edge chain has a
    /// vertex within the epsilon of EACH end.
    pub fn is_seam(&self, a: i32, b: i32) -> bool {
        if self.chain_keys.contains(&triangle_edges::key(a, b)) {
            return false;
        }

        if !self.beside(a) || !self.beside(b) {
            return false;
        }

        // Both ends beside the pick is not yet BESIDE it: an edge from one chain's end to ANOTHER
        // chain's end runs across the gap between them (an end face's edge along a stair's tread),
        // so its midpoint has to stand within the epsilon too.
        let low = self.positions[a as usize];
        let high = self.positions[b as usize];

        if !self.point_is_beside((low + high) / 2.0) {
            return false;
        }

        for &end in &self.ends {
            if (low - end).length() <= self.seam_epsilon
                && (high - end).length() <= self.seam_epsilon
            {
                return false;
            }
        }

        true
    }
}
