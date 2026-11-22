//! Linking a group's selected edges into the runs a single swept tool is built along: one such run
//! ([`BevelSweptChain`]) and the linking - walking the selected edges into runs ([`link`],
//! [`link_on_the_mesh`], [`link_resolution`]), and splitting a run where its convexity turns over
//! ([`split_by_convexity`]) or where one mitred sweep cannot carry it
//! ([`split_where_the_mitre_cannot_fit`]).
//!
//! **Through a junction.** A picked crease is the boundary between two face regions. Where three
//! or more picked edges meet at a vertex, [`link_on_the_mesh`] carries a run on along the one edge
//! that folds the same way as the edge it arrived on and bounds the same face region on the same
//! side of the walk - one region meaning faces joined round the vertex across edges too shallow to
//! be features (`topology::vertex_fan_regions`), so the facets of a curved wall are one region. A
//! crease the run does not carry on along truly ends there, on the run. A tube joined to a body
//! meets it in one closed loop even where the body's own crease runs into the tube: the loop folds
//! one way all round and keeps the tube's wall on one side, while the body's crease folds the other
//! way and so ends on the loop. Where no edge or more than one carries the run on - a box corner,
//! whose three convex edges each share a face with the arriving one - the run stops, as it does at
//! every junction under plain [`link`], which has no mesh to read regions from.
//!
//! **Only where the run closes, simply.** A run carried through a junction is kept only where it
//! closes and enters each of its vertices once; any other is linked again stopping at every
//! junction. The closing is what the recorded parts showed rather than a fact of the geometry: a
//! closed loop with a crease ending part way along it builds (the handle join of Tea Scoop Outside
//! Fillet), but carrying open runs through a junction moved four recorded parts - on three the
//! level line refused an open run with a crease ending part way along it and the part fell to the
//! existing construction, and on one a loop whose convexity turns over at the junction was carried
//! onto the crease ending there instead. The once is geometry: where two rim loops touch at a
//! vertex, as on the barrel of "Many Holes.mcx", the arriving edge can match one edge of the OTHER
//! loop, and the carry closes as one run through that vertex twice, which the build refuses as a
//! run touching itself. Still unguarded: a wrong carry that closes a simple loop.

use std::collections::{HashMap, HashSet};

use solidmesh_math::ieee_math;

use solidmesh_mesh::{Mesh, MeshEdgeGraph};

use super::bevel_chain_sweep_sections::try_build_segments;
use super::{bevel_mitre_fit, bevel_sweep_back_depth};
use crate::topology::vertex_fan_regions;
use crate::topology::{BevelEdgeGroupResolution, ChainEdge, FeatureEdge, FeatureEdgeSet};

/// One run of edges a single swept tool is built along - the edges head to tail, and whether the
/// run closes on itself.
#[derive(Debug, Clone, PartialEq, Default)]
pub struct BevelSweptChain {
    /// The edges in sweep order, each oriented along the sweep so consecutive edges meet head to
    /// tail.
    pub edges: Vec<ChainEdge>,
    /// Whether the last edge runs back into the first, in which case the sweep welds rather than
    /// caps.
    pub is_closed: bool,
}

impl BevelSweptChain {
    /// A run of `edges`, closed into a loop or not.
    pub fn new(edges: Vec<ChainEdge>, is_closed: bool) -> Self {
        BevelSweptChain { edges, is_closed }
    }
}

/// How near the two side readings of a joint have to come to be read as a tie.
pub const SIDE_EVIDENCE_TIE: f64 = 1e-9;

/// The runs a resolved group sweeps along: its one chain where the resolver found one, and the
/// runs its edges link into otherwise.
pub fn link_resolution(resolution: &BevelEdgeGroupResolution) -> Vec<BevelSweptChain> {
    if let Some(chain) = &resolution.chain {
        return vec![BevelSweptChain::new(
            chain.edges().to_vec(),
            chain.is_closed(),
        )];
    }

    link(&resolution.edges)
}

/// `chain` split into runs of one convexity each - started, on a closed chain, where its
/// convexity turns over - or `chain` itself where it has one convexity throughout.
pub fn split_by_convexity(chain: BevelSweptChain) -> Vec<BevelSweptChain> {
    let edges = &chain.edges;
    let first = edges[0].edge.convexity;

    if edges.iter().all(|e| e.edge.convexity == first) {
        return vec![chain];
    }

    let mut start = 0;

    if chain.is_closed {
        let count = edges.len();
        if let Some(i) = (0..count)
            .find(|&i| edges[i].edge.convexity != edges[(i + count - 1) % count].edge.convexity)
        {
            start = i;
        }
    }

    let mut runs = Vec::new();
    let mut current: Vec<ChainEdge> = Vec::new();

    for step in 0..edges.len() {
        let edge = edges[(start + step) % edges.len()];

        if !current.is_empty() && edge.edge.convexity != current[0].edge.convexity {
            runs.push(BevelSweptChain::new(std::mem::take(&mut current), false));
        }

        current.push(edge);
    }

    runs.push(BevelSweptChain::new(current, false));

    runs
}

/// Whether two faces standing at a vertex are one face region round it: `(vertex, face, face)`.
pub type SameRegionAt<'a> = &'a dyn Fn(i32, i32, i32) -> bool;

/// The selected `edges` walked into runs: each unclaimed edge seeds a run that grows forward and
/// then backward through vertices with exactly two selected edges whose sides agree; a run that
/// comes back to its seed (either way round, or end to start) is closed.
pub fn link(edges: &[FeatureEdge]) -> Vec<BevelSweptChain> {
    link_with(edges, None)
}

/// [`link`] carried on through junctions as well where that closes a loop, on the mesh the edges
/// were read from: at a vertex with three or more selected edges a run goes on along the one edge
/// that folds the same way and bounds the same face region on the same side (the module header),
/// read off `mesh`, its `graph` and the `features` the edges were picked among.
pub fn link_on_the_mesh(
    edges: &[FeatureEdge],
    mesh: &Mesh,
    graph: &MeshEdgeGraph,
    features: &FeatureEdgeSet,
) -> Vec<BevelSweptChain> {
    let same_region = |vertex: i32, a: i32, b: i32| {
        vertex_fan_regions::share_a_region_at(mesh, graph, features, vertex, a, b)
    };
    link_with(edges, Some(&same_region))
}

fn link_with(edges: &[FeatureEdge], same_region: Option<SameRegionAt>) -> Vec<BevelSweptChain> {
    let mut chains = Vec::new();

    if edges.is_empty() {
        return chains;
    }

    let mut at_vertex: HashMap<i32, Vec<i32>> = HashMap::new();

    for (i, edge) in edges.iter().enumerate() {
        add_incidence(&mut at_vertex, edge.start_vertex, i as i32);
        add_incidence(&mut at_vertex, edge.end_vertex, i as i32);
    }

    let mut used = vec![false; edges.len()];

    for seed in 0..edges.len() {
        if used[seed] {
            continue;
        }

        let (mut walk, mut closes) = walk_from(edges, &at_vertex, &mut used, seed, same_region);
        if same_region.is_some() && !(closes && passes_each_vertex_once(&walk)) {
            // Carried through a junction only to close a simple loop (the module header says
            // why): any other run is walked again stopping at every junction.
            for edge in &walk {
                used[edge.edge_index as usize] = false;
            }
            (walk, closes) = walk_from(edges, &at_vertex, &mut used, seed, None);
        }
        chains.push(BevelSweptChain::new(walk, closes));
    }

    chains
}

/// The run seeded at edge `seed`, grown forward and then backward and claimed in `used`, and
/// whether it closes.
fn walk_from(
    edges: &[FeatureEdge],
    at_vertex: &HashMap<i32, Vec<i32>>,
    used: &mut [bool],
    seed: usize,
    same_region: Option<SameRegionAt>,
) -> (Vec<ChainEdge>, bool) {
    used[seed] = true;

    let mut walk = vec![ChainEdge::new(seed as i32, edges[seed], false)];

    let mut closed = grow_with(edges, at_vertex, used, &mut walk, seed as i32, same_region);

    if !closed {
        // The other direction is the same walk over the run turned around, so it goes
        // through the same code and is flipped back afterwards. Its answer counts as much as
        // the forward one's: a closure missed here becomes an OPEN run whose two capped ends
        // sit on top of each other.
        let mut backward = vec![walk[0].reversed()];

        closed = grow_with(
            edges,
            at_vertex,
            used,
            &mut backward,
            seed as i32,
            same_region,
        );

        backward.remove(0);
        backward.reverse();

        let mut joined: Vec<ChainEdge> = backward.iter().map(|e| e.reversed()).collect();
        joined.extend(walk);
        walk = joined;
    }

    let closes = closed || closes_on_itself(&walk);
    (walk, closes)
}

/// Whether a walk enters each of its vertices once: a closed run through one vertex twice is two
/// loops touching there, not one.
fn passes_each_vertex_once(walk: &[ChainEdge]) -> bool {
    let mut entered = HashSet::with_capacity(walk.len());
    walk.iter().all(|edge| entered.insert(edge.from_vertex()))
}

/// Whether an open walk ends where it began, with its sides agreeing across that joint.
pub fn closes_on_itself(walk: &[ChainEdge]) -> bool {
    let first = walk[0];
    let last = walk[walk.len() - 1];

    walk.len() > 1 && first.from_vertex() == last.to_vertex() && sides_agree(&last, &first)
}

/// Grows `walk` forward through vertices with exactly two selected edges whose sides agree,
/// claiming each edge it takes; true where it comes back to the `seed`.
pub fn grow(
    edges: &[FeatureEdge],
    at_vertex: &HashMap<i32, Vec<i32>>,
    used: &mut [bool],
    walk: &mut Vec<ChainEdge>,
    seed: i32,
) -> bool {
    grow_with(edges, at_vertex, used, walk, seed, None)
}

/// [`grow`], carried on through a junction where `same_region` reads the faces round it.
fn grow_with(
    edges: &[FeatureEdge],
    at_vertex: &HashMap<i32, Vec<i32>>,
    used: &mut [bool],
    walk: &mut Vec<ChainEdge>,
    seed: i32,
    same_region: Option<SameRegionAt>,
) -> bool {
    loop {
        let current = walk[walk.len() - 1];
        let vertex = current.to_vertex();

        let Some(incident) = at_vertex.get(&vertex) else {
            return false;
        };

        let next = match (incident.len(), same_region) {
            (2, _) => {
                let next_index = if incident[0] == current.edge_index {
                    incident[1]
                } else {
                    incident[0]
                };
                let next_edge = edges[next_index as usize];
                ChainEdge::new(next_index, next_edge, next_edge.end_vertex == vertex)
            }
            (3.., Some(same_region)) => {
                match carried_on_through(edges, incident, &current, vertex, same_region) {
                    Some(next) => next,
                    None => return false,
                }
            }
            // A vertex with one selected edge is where the run ends; three or more, with no mesh
            // to read regions from, is a junction the sweep would have to guess its way through.
            _ => return false,
        };
        let next_index = next.edge_index;

        if !sides_agree(&current, &next) {
            return false;
        }

        if next_index == seed {
            return true;
        }

        if used[next_index as usize] {
            // Already claimed by another run: it cannot be walked twice, and it is not the
            // closure of this one either.
            return false;
        }

        used[next_index as usize] = true;
        walk.push(next);
    }
}

/// The one selected edge at junction `vertex` that carries `current` on: it folds the same way and
/// bounds the same face region on the same side of the walk. `None` where no edge or more than one
/// does.
fn carried_on_through(
    edges: &[FeatureEdge],
    incident: &[i32],
    current: &ChainEdge,
    vertex: i32,
    same_region: SameRegionAt,
) -> Option<ChainEdge> {
    let mut carrying: Option<ChainEdge> = None;
    for &index in incident {
        if index == current.edge_index {
            continue;
        }
        let edge = edges[index as usize];
        let leaving = ChainEdge::new(index, edge, edge.end_vertex == vertex);
        let carries = edge.convexity == current.edge.convexity
            && (same_region(vertex, current.left_face(), leaving.left_face())
                || same_region(vertex, current.right_face(), leaving.right_face()));
        if carries {
            if carrying.is_some() {
                return None;
            }
            carrying = Some(leaving);
        }
    }
    carrying
}

/// Whether `next` keeps `current`'s sides - its left face on the left - read off shared faces
/// first, and off the side normals where the two share none: the pairing whose normals agree
/// best, with a near tie settled by the sums.
pub fn sides_agree(current: &ChainEdge, next: &ChainEdge) -> bool {
    if current.left_face() == next.left_face() || current.right_face() == next.right_face() {
        return true;
    }

    if current.left_face() == next.right_face() || current.right_face() == next.left_face() {
        return false;
    }

    let agreeing_left = current.left_normal().dot(next.left_normal());
    let agreeing_right = current.right_normal().dot(next.right_normal());
    let swapped_left = current.left_normal().dot(next.right_normal());
    let swapped_right = current.right_normal().dot(next.left_normal());

    let agreeing_best = ieee_math::max_f64(agreeing_left, agreeing_right);
    let swapped_best = ieee_math::max_f64(swapped_left, swapped_right);

    if (agreeing_best - swapped_best).abs() > SIDE_EVIDENCE_TIE {
        return agreeing_best > swapped_best;
    }

    agreeing_left + agreeing_right >= swapped_left + swapped_right
}

/// Records `edge_index` as standing at `vertex`.
pub fn add_incidence(at_vertex: &mut HashMap<i32, Vec<i32>>, vertex: i32, edge_index: i32) {
    at_vertex
        .entry(vertex)
        .or_insert_with(|| Vec::with_capacity(2))
        .push(edge_index);
}

/// `chain` split where a single mitred sweep cannot carry it - or kept whole at a shallower back
/// depth where that alone lets every joint fit - and that back depth (zero where none was needed).
/// The short form is [`split_where_the_mitre_cannot_fit`].
pub fn split_where_the_mitre_cannot_fit_with_depth(
    chain: BevelSweptChain,
    size: f64,
    segments: i32,
    round_profile: bool,
) -> (Vec<BevelSweptChain>, f64) {
    let Some(cannot_fit) = bevel_mitre_fit::joints_the_mitre_cannot_fit(
        &chain.edges,
        chain.is_closed,
        size,
        segments,
        round_profile,
        0.0,
    ) else {
        return (vec![chain], 0.0);
    };

    if let Ok(swept) = try_build_segments(&chain.edges, size, segments, round_profile, 0.0) {
        let shallower = bevel_sweep_back_depth::retry_depth(
            &chain.edges,
            chain.is_closed,
            size,
            bevel_sweep_back_depth::measure_back_reach(&swept),
            f64::INFINITY,
        );

        if shallower > 0.0
            && bevel_mitre_fit::joints_the_mitre_cannot_fit(
                &chain.edges,
                chain.is_closed,
                size,
                segments,
                round_profile,
                shallower,
            )
            .is_none()
        {
            return (vec![chain], shallower);
        }
    }

    let edges = &chain.edges;
    let mut start = 0;

    if chain.is_closed {
        if let Some(joint) = cannot_fit.iter().position(|&c| c) {
            start = joint;
        }
    }

    let mut runs = Vec::new();
    let mut current: Vec<ChainEdge> = Vec::new();

    for step in 0..edges.len() {
        let index = (start + step) % edges.len();

        if !current.is_empty() && cannot_fit[index] {
            runs.push(BevelSweptChain::new(std::mem::take(&mut current), false));
        }

        current.push(edges[index]);
    }

    runs.push(BevelSweptChain::new(current, false));

    (runs, 0.0)
}

/// [`split_where_the_mitre_cannot_fit_with_depth`] without the back depth.
pub fn split_where_the_mitre_cannot_fit(
    chain: BevelSweptChain,
    size: f64,
    segments: i32,
    round_profile: bool,
) -> Vec<BevelSweptChain> {
    split_where_the_mitre_cannot_fit_with_depth(chain, size, segments, round_profile).0
}
