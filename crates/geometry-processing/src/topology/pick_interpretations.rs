//! The candidate readings of one pick, for the user to cycle through before committing, as free
//! functions. The reading type itself is [`super::pick_interpretation`], and
//! [`super::bevel_edge_readings`] reaches the same
//! list from a stored group.
//!
//! An edge is only an edge relative to a flat tolerance, and the same click legitimately means a
//! single edge, the loop it chains into, or a different loop read at another tolerance - a
//! tessellated cylinder rim is one edge at half a degree and the whole circle at ten. The plan's
//! resolved decision is that there is no right answer, so the pick offers all of them and the
//! user chooses; whichever they choose travels onto the group with the tolerance that produced it.
//!
//! Ordering is fixed - the single edge, then the chains by increasing tolerance - so cycling with
//! a gesture lands on the same reading every time for the same click.

use std::sync::Arc;

use polygon_mesh::{Mesh, MeshEdgeGraph};

use super::bevel_edge_group::BevelSelectionKind;
use super::bevel_edge_group_resolver;
use super::error::TopologyError;
use super::face_group::FaceGroup;
use super::face_region_cache::FaceRegionCache;
use super::feature_edge::FeatureEdge;
use super::feature_edge_chain::{ChainEnd, ChainStopReason, FeatureEdgeChain};
use super::feature_edge_set::FeatureEdgeSet;
use super::pick_interpretation::{PickInterpretation, PickInterpretationKind};

/// The ladder rung that reads a tessellated wall as a run of separate facets: the tolerance
/// feature-edge detection uses by default. At this rung a cylinder rim vertex has three feature
/// edges and so is a corner, and the rim chains to one edge.
///
/// Not what a 3D pick runs at. A bevel's pick set is built at the operation's own bend threshold
/// (the operation's `bend_threshold_degrees`, twenty degrees by default), and the ladder
/// starts there - so on a wall whose facets turn less than that this rung is not offered at all,
/// and a click cannot land on a facet join the user has said is not an edge. It stays the
/// DETECTION default, and the rung a caller with no threshold of its own - construction plane
/// face picking - asks for. (A function because [`FeatureEdge::default_flat_angle_degrees`] is.)
pub fn faceted_flat_angle_degrees() -> f64 {
    FeatureEdge::default_flat_angle_degrees()
}

/// The ladder rung that reads a tessellated wall as one curved surface, so a rim walks the whole
/// way round. Ten degrees is the plan's own worked example of the other reading of a cylinder
/// rim, and it clears a 64 sided wall's 5.625 degree facets with room to spare while still
/// stopping at anything a person would call a corner.
pub const SMOOTH_WALL_FLAT_ANGLE_DEGREES: f64 = 10.0;

/// The loosest rung: the resolved curve tolerance (Blender's auto-smooth default), which reads
/// even a coarsely tessellated surface as one. Anything looser starts swallowing real corners.
pub const COARSE_FLAT_ANGLE_DEGREES: f64 = FaceGroup::DEFAULT_CURVE_TOLERANCE_DEGREES;

/// The flat tolerances the readings are generated at, in the order they are offered.
pub fn default_flat_angle_ladder_degrees() -> [f64; 3] {
    [
        faceted_flat_angle_degrees(),
        SMOOTH_WALL_FLAT_ANGLE_DEGREES,
        COARSE_FLAT_ANGLE_DEGREES,
    ]
}

/// Two ladder rungs closer together than this are the same rung. Only wide enough to absorb
/// arithmetic on the tolerances themselves.
const SAME_RUNG_DEGREES: f64 = 1e-9;

/// Builds the readings of a pick at the default tolerance ladder and the default tangent
/// tolerance.
pub fn build_default(
    mesh: &Arc<Mesh>,
    pick_features: &Arc<FeatureEdgeSet>,
    seed_edge_index: i32,
) -> Result<Vec<PickInterpretation>, TopologyError> {
    build_at_angles(
        mesh,
        None,
        pick_features,
        seed_edge_index,
        FeatureEdgeChain::DEFAULT_TANGENT_ANGLE_DEGREES,
        &default_flat_angle_ladder_degrees(),
    )
}

/// Builds the readings of a pick: the collinear run the picked edge lies in, the tangent chain
/// it lies in at each rung of a ladder of flat tolerances, and the rim of each face it divides -
/// with any of them that come out to the same edges collapsed into the first that produced them.
///
/// `graph` is the mesh's edge graph if the caller has one (built here when `None`); it must be
/// this mesh's own. `pick_features` is the set the pick ran against, whose tolerance the
/// collinear reading keeps, and `seed_edge_index` indexes it. `flat_angle_ladder_degrees` is
/// sorted here, so the caller need not.
///
/// Each rung needs its own feature edge detection, and the picked edge has to be found again in
/// each new set, whose indices have nothing to do with the previous one's. It is found by
/// [`FeatureEdge::mesh_edge_index`]: every set here is built over one mesh and one
/// [`MeshEdgeGraph`], so the mesh edge index IS the edge's identity, and matching on it is exact
/// and cannot pick a neighbour. A rung coarser than the edge's own turn angle simply has no
/// reading: the picked edge is flat there and is not in that set at all, so the rung is skipped
/// rather than answered with some other edge's chain.
pub fn build_at_angles(
    mesh: &Arc<Mesh>,
    graph: Option<&MeshEdgeGraph>,
    pick_features: &Arc<FeatureEdgeSet>,
    seed_edge_index: i32,
    curve_tolerance_degrees: f64,
    flat_angle_ladder_degrees: &[f64],
) -> Result<Vec<PickInterpretation>, TopologyError> {
    check_seed(pick_features, seed_edge_index)?;

    let built_graph;
    let graph = match graph {
        Some(graph) => graph,
        None => {
            built_graph = mesh.get_mesh_edge_graph();
            &built_graph
        }
    };

    let mut rung_features = Vec::with_capacity(flat_angle_ladder_degrees.len());

    for flat_angle_degrees in ascending_angles(flat_angle_ladder_degrees) {
        // The set the pick was made in is reused when a rung asks for the same tolerance, both
        // to save the detection and so that the reading really is the one the user is looking at.
        rung_features.push(
            if (flat_angle_degrees - pick_features.flat_angle_degrees()).abs() <= SAME_RUNG_DEGREES
            {
                pick_features.clone()
            } else {
                Arc::new(FeatureEdgeSet::create_with_graph(
                    mesh,
                    graph,
                    flat_angle_degrees,
                ))
            },
        );
    }

    build(
        mesh,
        Some(graph),
        pick_features,
        seed_edge_index,
        curve_tolerance_degrees,
        &rung_features,
        None,
    )
}

/// The same readings, built from feature edge sets the caller already has - one per rung of the
/// ladder, in any order. Every set must have been built over the same mesh and edge graph as
/// `pick_features`, because the readings are matched and deduplicated by mesh edge index across
/// them. `regions` is the face region cache to flood the face rims in; `None` makes one at the
/// pick's tolerance.
///
/// This exists for the viewport, where detecting the whole ladder again on every pick is the
/// expensive part and the mesh has not changed between picks. It also makes the ladder explicit:
/// a caller can offer readings at whatever tolerances it likes without this having to know how
/// to produce them. A rung whose set has its tangent chains prepared
/// ([`FeatureEdgeSet::prepare_tangent_chains`]) at `curve_tolerance_degrees` reads the chain off
/// them rather than walking it, with the same answer.
pub fn build(
    mesh: &Arc<Mesh>,
    graph: Option<&MeshEdgeGraph>,
    pick_features: &Arc<FeatureEdgeSet>,
    seed_edge_index: i32,
    curve_tolerance_degrees: f64,
    rung_features: &[Arc<FeatureEdgeSet>],
    regions: Option<&FaceRegionCache>,
) -> Result<Vec<PickInterpretation>, TopologyError> {
    check_seed(pick_features, seed_edge_index)?;

    let mut interpretations = Vec::new();
    let mut edge_sets_already_offered: Vec<Vec<i32>> = Vec::new();

    let picked = pick_features.edges()[seed_edge_index as usize];
    let rungs = ascending_sets(rung_features);

    // The default reading, and the one the stored BevelSelectionKind::Edge group re-derives on
    // rebuild: the whole straight crease, in the set the pick was made in.
    add_collinear_run(
        pick_features,
        seed_edge_index,
        curve_tolerance_degrees,
        &mut interpretations,
        &mut edge_sets_already_offered,
    )?;

    // And the same run read in each of the other rungs. A run is walked in ONE feature edge set,
    // and which set that is can change what it comes to - an edge whose fold is a hair under a
    // rung's tolerance is missing from that rung, so a run that continues through it at a finer
    // reading stops there at a coarser one. Emitting the run per rung is what keeps this list
    // able to contain what a stored group resolved to whatever tolerance that group was picked
    // at: BevelEdgeReadings seeds these readings from the FINEST rung holding the edge, while the
    // resolver walks the group's OWN set, and a list that could not contain the group's own
    // answer would leave its row saying it is on no reading at all. Where the two rungs agree -
    // which is nearly always, because a collinear continuation has the same two faces as the seed
    // and so survives exactly the same tolerances - the dedup drops the repeat.
    for rung in &rungs {
        let rung_seed = index_of_mesh_edge(rung, picked.mesh_edge_index);

        if rung_seed >= 0 {
            add_collinear_run(
                rung,
                rung_seed,
                curve_tolerance_degrees,
                &mut interpretations,
                &mut edge_sets_already_offered,
            )?;
        }
    }

    for rung in &rungs {
        let rung_seed_index = index_of_mesh_edge(rung, picked.mesh_edge_index);
        if rung_seed_index < 0 {
            continue;
        }

        // One tolerance for both of the chain's tests, matching what a group stores: it persists
        // a single "how far may the surface turn" number and re-propagates from that on rebuild.
        // Read off the set's walked chains where it has them, which is the same chain without
        // walking a rim of thousands of edges on every hover.
        let (chain, signature) =
            match rung.tangent_chain_through(rung_seed_index, curve_tolerance_degrees) {
                Some((chain, signature)) => (chain, signature.to_vec()),
                None => {
                    let chain = FeatureEdgeChain::propagate_with(
                        rung,
                        rung_seed_index,
                        curve_tolerance_degrees,
                        curve_tolerance_degrees,
                    )?;
                    let signature = mesh_edge_signature_of_chain(&chain);
                    (chain, signature)
                }
            };

        if edge_sets_already_offered.contains(&signature) {
            // The same edges as a lower rung, so it is the same reading told twice - including
            // the collinear run, which a chain that stops where the crease stops comes out as.
            continue;
        }

        let chain_edges: Vec<FeatureEdge> = chain.edges().iter().map(|e| e.edge).collect();

        let stops_where_the_shape_does = chain.is_closed()
            || (stop_is_a_corner_of_the_shape(rung, chain.start())
                && stop_is_a_corner_of_the_shape(rung, chain.end()));

        let group = bevel_edge_group_resolver::create_from_edge(
            rung,
            rung_seed_index,
            BevelSelectionKind::TangentChain,
            curve_tolerance_degrees,
        )?;

        interpretations.push(
            PickInterpretation::new(
                PickInterpretationKind::TangentChain,
                rung.clone(),
                rung_seed_index,
                chain_edges,
                Some(chain),
                curve_tolerance_degrees,
                group,
            )
            .with_stops_where_the_shape_does(stops_where_the_shape_does),
        );
        edge_sets_already_offered.push(signature);
    }

    // Last, the two faces the picked edge divides - one reading each. They come after the chains
    // because they are the furthest thing from what was clicked: the user aimed at an edge, and a
    // face rim is a different question about the same click.
    let own_regions;
    let regions = match regions {
        Some(regions) => regions,
        None => {
            own_regions = FaceRegionCache::new(
                mesh.clone(),
                graph.cloned(),
                pick_features.flat_angle_degrees(),
            );
            &own_regions
        }
    };

    for face in [picked.face0, picked.face1] {
        add_face_rim(
            regions,
            pick_features,
            seed_edge_index,
            face,
            curve_tolerance_degrees,
            &mut interpretations,
            &mut edge_sets_already_offered,
        )?;
    }

    Ok(interpretations)
}

/// Which reading a pick in chain mode starts on: the first tangent chain that ends where the
/// SHAPE ends, or - when no reading does - the first tangent chain there is. -1 when the pick
/// offers no chain reading at all, which is every pick on a part with no curved surface in it.
///
/// The ladder is ordered by increasing flat tolerance, so the first chain smooths only as much
/// faceting as this pick requires, and that is still what is wanted: a rim that walks the whole
/// way round at ten degrees must not be read at thirty, where it might swallow a corner. What is
/// skipped is a rung that walks PART of a tessellated surface and stops inside it
/// ([`PickInterpretation::stops_where_the_shape_does`]) - not a coarser reading of the loop but
/// an artefact of where that rung's tolerance happens to fall.
pub fn index_of_preferred_chain(readings: &[PickInterpretation]) -> i32 {
    let mut first = -1;

    for (i, reading) in readings.iter().enumerate() {
        if reading.kind() != PickInterpretationKind::TangentChain {
            continue;
        }

        if reading.stops_where_the_shape_does() {
            return i as i32;
        }

        if first < 0 {
            first = i as i32;
        }
    }

    first
}

/// Whether a walk that stopped here stopped at a corner of the SHAPE. Any stop that is not a
/// junction is one; a junction is one only while it is still a junction at the ladder's coarsest
/// rung.
pub fn stop_is_a_corner_of_the_shape(features: &FeatureEdgeSet, end: ChainEnd) -> bool {
    end.reason != ChainStopReason::Corner
        || is_a_junction_at_the_coarsest_rung(features, end.vertex)
}

/// Whether three or more of the feature edges meeting at a vertex would still be feature edges at
/// the ladder's coarsest rung - the same test `FeatureEdgeChain`'s walk stops on, asked of the
/// coarser reading.
pub fn is_a_junction_at_the_coarsest_rung(features: &FeatureEdgeSet, vertex: i32) -> bool {
    let survivors = features
        .edges_at_vertex(vertex)
        .iter()
        .filter(|&&i| survives_the_coarsest_rung(&features.edges()[i as usize]))
        .count();

    survivors > 2
}

/// Whether an edge is still a feature edge when the part is read at the ladder's coarsest rung.
/// The turn angle is signed - positive convex, negative concave - and a flat tolerance is
/// compared against its magnitude, which is how [`FeatureEdge::try_classify`] decides the same
/// thing.
pub fn survives_the_coarsest_rung(edge: &FeatureEdge) -> bool {
    edge.turn_angle_degrees.abs() > COARSE_FLAT_ANGLE_DEGREES
}

/// The one reading a click in the MIDDLE of a face means: the rim of the smooth region that face
/// belongs to. Empty when that region has no bevelable rim - a closed shell, or one bounded only
/// by open or non-manifold edges.
///
/// Its own entry point because a pick that is not near any edge has no seed edge for the other
/// readings to be built from. There is exactly one thing such a click can mean, so there is one
/// reading and nothing to cycle through.
///
/// Also hands back the region that was flooded, whether or not it had a rim to offer - what the
/// viewport shades to show which face is being read. Handed
/// back rather than asked for again, because the flood is the expensive part and the highlight
/// and the reading have to be describing the same region. `None` only where the flood itself
/// could not run.
pub fn build_for_face(
    mesh: &Arc<Mesh>,
    graph: Option<&MeshEdgeGraph>,
    pick_features: &Arc<FeatureEdgeSet>,
    face_index: i32,
    curve_tolerance_degrees: f64,
    regions: Option<&FaceRegionCache>,
) -> Result<(Vec<PickInterpretation>, Option<Arc<FaceGroup>>), TopologyError> {
    if face_index < 0 || face_index as usize >= mesh.faces.len() {
        return Err(TopologyError::FaceIndexOutOfRange {
            parameter: "faceIndex",
            index: face_index,
            face_count: mesh.faces.len(),
        });
    }

    let mut interpretations = Vec::new();

    let own_regions;
    let regions = match regions {
        Some(regions) => regions,
        None => {
            own_regions = FaceRegionCache::new(
                mesh.clone(),
                graph.cloned(),
                pick_features.flat_angle_degrees(),
            );
            &own_regions
        }
    };

    let region = regions.region(face_index);

    add_face_rim(
        regions,
        pick_features,
        -1,
        face_index,
        curve_tolerance_degrees,
        &mut interpretations,
        &mut Vec::new(),
    )?;

    Ok((interpretations, region))
}

/// The picked edge's index in another set built over the same mesh, or -1 if that set does not
/// hold it - which means the edge is flat at that set's tolerance. Shared with
/// [`super::bevel_edge_readings`], which asks the same question of the same ladder, so the two
/// cannot come to differ.
pub fn index_of_mesh_edge(features: &FeatureEdgeSet, mesh_edge_index: i32) -> i32 {
    features.index_of_mesh_edge(mesh_edge_index)
}

/// Whether two readings select the same thing - the same mesh edges, whatever their order.
///
/// The viewport's click needs it to tell "the reading already stored" from "another reading of
/// an edge that reading holds": the first is the toggle off, the second replaces the group.
/// Comparing the edges rather than the kind and the tolerance is what makes a loop the same loop
/// wherever on it the user clicked.
pub fn select_the_same_edges(left: &[FeatureEdge], right: &[FeatureEdge]) -> bool {
    mesh_edge_signature(left) == mesh_edge_signature(right)
}

/// Adds the rim of the region `seed_face_index` belongs to, unless it selects what an earlier
/// reading already does. `anchor_edge_index` is the picked edge to anchor the group on, or -1 to
/// anchor on whichever rim edge comes first - which is what a click in the middle of a face has
/// to do. Any rim edge anchors the region equally well: it is the anchor's ORIENTATION, not
/// where it sits, that names which side the region is on.
///
/// The flood runs at the PICK's flat tolerance (the cache's) rather than at
/// `curve_tolerance_degrees`: the curve tolerance is a chain's number - thirty degrees, loose
/// enough to walk round a tessellated rim - and flooding a chorded cylinder with it swallows the
/// whole barrel from a click on one facet.
fn add_face_rim(
    regions: &FaceRegionCache,
    pick_features: &Arc<FeatureEdgeSet>,
    anchor_edge_index: i32,
    seed_face_index: i32,
    curve_tolerance_degrees: f64,
    interpretations: &mut Vec<PickInterpretation>,
    edge_sets_already_offered: &mut Vec<Vec<i32>>,
) -> Result<(), TopologyError> {
    let Some(face_group) = regions.region(seed_face_index) else {
        return Ok(());
    };

    if face_group.boundary_edges().is_empty() {
        // Nothing to bevel: a closed region, or one bounded only by edges no bevel can be built on.
        return Ok(());
    }

    let signature = mesh_edge_signature(face_group.boundary_edges());

    if edge_sets_already_offered.contains(&signature) {
        // The same edges as a reading already offered - a box top's rim IS the chain round it -
        // so this is one reading told a second way, and the first of them keeps the place.
        return Ok(());
    }

    let seed_edge_index = if anchor_edge_index >= 0 {
        anchor_edge_index
    } else {
        index_of_mesh_edge(
            pick_features,
            face_group.boundary_edges()[0].mesh_edge_index,
        )
    };

    if seed_edge_index < 0 {
        // The rim is there but no edge of it is in the set the pick ran in, so there is nothing
        // to anchor a group on that could be matched again later. Nothing honest to offer.
        return Ok(());
    }

    let group = bevel_edge_group_resolver::create_from_face_group(
        &face_group,
        &pick_features.edges()[seed_edge_index as usize],
        pick_features.flat_angle_degrees(),
    )?;

    interpretations.push(PickInterpretation::new(
        PickInterpretationKind::FaceGroup,
        pick_features.clone(),
        seed_edge_index,
        face_group.boundary_edges().to_vec(),
        None,
        curve_tolerance_degrees,
        group,
    ));

    edge_sets_already_offered.push(signature);

    Ok(())
}

/// Adds the collinear run through an edge as read in one feature edge set, unless it selects
/// what a reading already offered.
fn add_collinear_run(
    features: &Arc<FeatureEdgeSet>,
    seed_edge_index: i32,
    curve_tolerance_degrees: f64,
    interpretations: &mut Vec<PickInterpretation>,
    edge_sets_already_offered: &mut Vec<Vec<i32>>,
) -> Result<(), TopologyError> {
    let run = FeatureEdgeChain::propagate_collinear(features, seed_edge_index)?;
    let run_edges: Vec<FeatureEdge> = run.edges().iter().map(|e| e.edge).collect();

    let signature = mesh_edge_signature(&run_edges);

    if edge_sets_already_offered.contains(&signature) {
        return Ok(());
    }

    let group = bevel_edge_group_resolver::create_from_edge(
        features,
        seed_edge_index,
        BevelSelectionKind::Edge,
        curve_tolerance_degrees,
    )?;

    interpretations.push(PickInterpretation::new(
        PickInterpretationKind::CollinearRun,
        features.clone(),
        seed_edge_index,
        run_edges,
        None,
        curve_tolerance_degrees,
        group,
    ));

    edge_sets_already_offered.push(signature);

    Ok(())
}

/// Refuses a seed edge index that is not an edge of the set.
fn check_seed(features: &FeatureEdgeSet, seed_edge_index: i32) -> Result<(), TopologyError> {
    if seed_edge_index < 0 || seed_edge_index as usize >= features.count() {
        return Err(TopologyError::EdgeIndexOutOfRange {
            index: seed_edge_index,
            edge_count: features.count(),
        });
    }

    Ok(())
}

/// The ladder in increasing order with repeats dropped, which is what makes the offered order
/// deterministic and makes "keep the lower tolerance" the rule dedup can follow.
fn ascending_angles(flat_angle_ladder_degrees: &[f64]) -> Vec<f64> {
    let mut sorted = flat_angle_ladder_degrees.to_vec();
    sorted.sort_by(f64::total_cmp);

    let mut rungs: Vec<f64> = Vec::with_capacity(sorted.len());
    for (i, &angle) in sorted.iter().enumerate() {
        if i > 0 && angle - sorted[i - 1] <= SAME_RUNG_DEGREES {
            continue;
        }

        rungs.push(angle);
    }

    rungs
}

/// The same ordering over sets that have already been detected: increasing tolerance, one set
/// per rung. Two sets at the same tolerance are the same rung whichever instances they are, so
/// the first of them is the one offered. The sort is stable, so "the first" is the caller's
/// first, and which instance is offered never depends on the sort's algorithm.
fn ascending_sets(rung_features: &[Arc<FeatureEdgeSet>]) -> Vec<Arc<FeatureEdgeSet>> {
    let mut sorted = rung_features.to_vec();
    sorted.sort_by(|a, b| a.flat_angle_degrees().total_cmp(&b.flat_angle_degrees()));

    let mut rungs: Vec<Arc<FeatureEdgeSet>> = Vec::with_capacity(sorted.len());
    for (i, set) in sorted.iter().enumerate() {
        if i > 0
            && set.flat_angle_degrees() - sorted[i - 1].flat_angle_degrees() <= SAME_RUNG_DEGREES
        {
            continue;
        }

        rungs.push(set.clone());
    }

    rungs
}

/// [`mesh_edge_signature`] of a chain's edges, without copying them out first.
fn mesh_edge_signature_of_chain(chain: &FeatureEdgeChain) -> Vec<i32> {
    let mut mesh_edges: Vec<i32> = chain
        .edges()
        .iter()
        .map(|edge| edge.edge.mesh_edge_index)
        .collect();
    mesh_edges.sort_unstable();
    mesh_edges
}

/// What makes two readings the same reading: the mesh edges they select, in a canonical order.
/// Mesh edge indices are comparable across the ladder's sets because every one of them is built
/// over the same mesh and graph.
fn mesh_edge_signature(edges: &[FeatureEdge]) -> Vec<i32> {
    let mut mesh_edges: Vec<i32> = edges.iter().map(|edge| edge.mesh_edge_index).collect();
    mesh_edges.sort_unstable();
    mesh_edges
}
