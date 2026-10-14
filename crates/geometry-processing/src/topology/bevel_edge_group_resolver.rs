//! Turning a stored [`BevelEdgeGroup`] back into edges of a rebuilt mesh, and making one from a
//! pick ([`BevelEdgeGroupResolution`]).
//!
//! An anchor is matched by gates (midpoint drift within a fraction of the feature extent,
//! sign-insensitive direction, both face normals under either pairing) and the survivors scored
//! with each term divided by its own gate. A group that no longer names one edge - nothing
//! passes, or two pass about equally well - is dropped with a [`BevelAdvisory`] rather than
//! guessed at.

use std::collections::HashMap;
use std::sync::Arc;

use polygon_mesh::{Mesh, MeshEdgeGraph};
use vector_math::{math_helper, AxisAlignedBoundingBox, Vector3};

use crate::bevel::{BevelAdvisory, BevelAdvisoryCode};

use super::bevel_edge_group::{BevelEdgeGroup, BevelSelectionKind};
use super::error::TopologyError;
use super::face_group::FaceGroup;
use super::feature_edge::FeatureEdge;
use super::feature_edge_chain::FeatureEdgeChain;
use super::feature_edge_set::FeatureEdgeSet;

/// How far a candidate's direction may turn from the anchor's and still match, in degrees.
pub const DEFAULT_DIRECTION_TOLERANCE_DEGREES: f64 = 20.0;
/// How far each face normal may turn from the anchor's and still match, in degrees.
pub const DEFAULT_NORMAL_TOLERANCE_DEGREES: f64 = 20.0;
/// How far the midpoint may drift, as a fraction of the feature edges' bounding diagonal.
pub const DEFAULT_DRIFT_FRACTION_OF_FEATURE_EXTENT: f64 = 0.25;
/// A best score at or above this fraction of the runner-up's is two matches, not one.
pub const DEFAULT_AMBIGUITY_RATIO: f64 = 0.8;

/// The matching tolerances, with the defaults a caller gets without asking.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct MatchTolerances {
    /// See [`DEFAULT_DIRECTION_TOLERANCE_DEGREES`].
    pub direction_tolerance_degrees: f64,
    /// See [`DEFAULT_NORMAL_TOLERANCE_DEGREES`].
    pub normal_tolerance_degrees: f64,
    /// See [`DEFAULT_DRIFT_FRACTION_OF_FEATURE_EXTENT`].
    pub drift_fraction_of_feature_extent: f64,
    /// See [`DEFAULT_AMBIGUITY_RATIO`].
    pub ambiguity_ratio: f64,
}

impl Default for MatchTolerances {
    fn default() -> Self {
        MatchTolerances {
            direction_tolerance_degrees: DEFAULT_DIRECTION_TOLERANCE_DEGREES,
            normal_tolerance_degrees: DEFAULT_NORMAL_TOLERANCE_DEGREES,
            drift_fraction_of_feature_extent: DEFAULT_DRIFT_FRACTION_OF_FEATURE_EXTENT,
            ambiguity_ratio: DEFAULT_AMBIGUITY_RATIO,
        }
    }
}

/// What one group resolved to on one mesh, or why it did not.
#[derive(Debug, Clone)]
pub struct BevelEdgeGroupResolution {
    /// The group as it was resolved (read at the feature set's tolerance where that applies).
    pub group: BevelEdgeGroup,
    /// The feature edges it was matched among.
    pub features: Arc<FeatureEdgeSet>,
    /// Why it did not resolve; `None` when it did.
    pub failure_reason: Option<BevelAdvisory>,
    /// The matched feature edge's index in `features`, or -1.
    pub seed_edge_index: i32,
    /// The edges the group selects; empty when it failed.
    pub edges: Vec<FeatureEdge>,
    /// The tangent chain walked, for a [`BevelSelectionKind::TangentChain`] (or crease) group.
    pub chain: Option<FeatureEdgeChain>,
    /// The flooded region, for a [`BevelSelectionKind::FaceGroup`] group.
    pub face_group: Option<FaceGroup>,
}

impl BevelEdgeGroupResolution {
    /// Whether the group named edges of this mesh.
    pub fn is_resolved(&self) -> bool {
        self.failure_reason.is_none()
    }

    fn failed(group: BevelEdgeGroup, features: Arc<FeatureEdgeSet>, reason: BevelAdvisory) -> Self {
        BevelEdgeGroupResolution {
            group,
            features,
            failure_reason: Some(reason),
            seed_edge_index: -1,
            edges: Vec::new(),
            chain: None,
            face_group: None,
        }
    }

    pub(crate) fn succeeded(
        group: BevelEdgeGroup,
        features: Arc<FeatureEdgeSet>,
        seed_edge_index: i32,
        edges: Vec<FeatureEdge>,
        chain: Option<FeatureEdgeChain>,
        face_group: Option<FaceGroup>,
    ) -> Self {
        BevelEdgeGroupResolution {
            group,
            features,
            failure_reason: None,
            seed_edge_index,
            edges,
            chain,
            face_group,
        }
    }
}

/// A group anchored on one feature edge of a set, read at that set's flat tolerance.
pub fn create_from_edge(
    features: &FeatureEdgeSet,
    seed_edge_index: i32,
    selection_kind: BevelSelectionKind,
    curve_tolerance_degrees: f64,
) -> Result<BevelEdgeGroup, TopologyError> {
    if seed_edge_index < 0 || seed_edge_index as usize >= features.count() {
        return Err(TopologyError::EdgeIndexOutOfRange {
            index: seed_edge_index,
            edge_count: features.count(),
        });
    }

    Ok(create(
        &features.edges()[seed_edge_index as usize],
        selection_kind,
        curve_tolerance_degrees,
        features.flat_angle_degrees(),
        false,
    ))
}

/// The optional seed filter [`create_groups_for_all_edges`] takes: `false` skips that feature edge.
pub type IncludeEdge<'a> = &'a dyn Fn(&FeatureEdgeSet, i32) -> bool;

/// "Bevel everything": one [`BevelSelectionKind::Edge`] group per straight CREASE of `mesh` at
/// `flat_angle_degrees`, not one per mesh edge. A crease a boolean cut into several mesh edges
/// would otherwise be swept once per piece, from rows of the panel that select the same edges.
/// Each group is anchored on the first edge of its collinear run, and every feature edge lands in
/// exactly one run. `include`, where given, skips a seed it answers `false` for.
///
/// It lives here rather than with the bevel operation because it is pure topology; the
/// operation's node form calls it.
pub fn create_groups_for_all_edges(
    mesh: &Mesh,
    flat_angle_degrees: f64,
    include: Option<IncludeEdge<'_>>,
) -> Result<Vec<BevelEdgeGroup>, TopologyError> {
    let features = FeatureEdgeSet::create_with_angle(mesh, flat_angle_degrees);
    let mut groups = Vec::with_capacity(features.count());
    let mut already_in_a_run = vec![false; features.count()];

    for i in 0..features.count() {
        let seed = i as i32;
        if already_in_a_run[i] || include.is_some_and(|include| !include(&features, seed)) {
            continue;
        }

        // Walked before the group is made rather than after it resolves, because it is the same
        // walk the resolver will do and this is the cheapest place to know what the group takes.
        for run_edge in FeatureEdgeChain::propagate_collinear(&features, seed)?.edges() {
            already_in_a_run[run_edge.edge_index as usize] = true;
        }

        groups.push(create_from_edge(
            &features,
            seed,
            BevelSelectionKind::Edge,
            FaceGroup::DEFAULT_CURVE_TOLERANCE_DEGREES,
        )?);
    }

    Ok(groups)
}

/// A face-group selection anchored on one of the region's rim edges, with the region's face as
/// the anchor's first face (so the flood re-runs on the same side after a rebuild).
pub fn create_from_face_group(
    face_group: &FaceGroup,
    boundary_edge: &FeatureEdge,
    flat_angle_degrees: f64,
) -> Result<BevelEdgeGroup, TopologyError> {
    let face0_is_in_group = face_group.faces().contains(&boundary_edge.face0);
    let face1_is_in_group = face_group.faces().contains(&boundary_edge.face1);

    if !face0_is_in_group && !face1_is_in_group {
        return Err(TopologyError::EdgeDoesNotBoundFaceGroup {
            message: format!(
                "Mesh edge {} does not bound this face group - neither of its faces ({} and {}) is in the region flooded from face {}.",
                boundary_edge.mesh_edge_index,
                boundary_edge.face0,
                boundary_edge.face1,
                face_group.seed_face()
            ),
        });
    }

    Ok(create(
        boundary_edge,
        BevelSelectionKind::FaceGroup,
        face_group.curve_tolerance_degrees(),
        flat_angle_degrees,
        !face0_is_in_group,
    ))
}

/// Resolves one group at the default tolerances, at the group's own flat tolerance.
pub fn resolve(group: &BevelEdgeGroup, mesh: &Mesh) -> BevelEdgeGroupResolution {
    resolve_with(group, mesh, MatchTolerances::default())
}

/// Resolves one group at the given tolerances, at the group's own flat tolerance.
pub fn resolve_with(
    group: &BevelEdgeGroup,
    mesh: &Mesh,
    tolerances: MatchTolerances,
) -> BevelEdgeGroupResolution {
    let (mut resolved, _) = resolve_all_with(std::slice::from_ref(group), mesh, tolerances);
    resolved.remove(0)
}

/// Resolves every group, building one feature set per distinct flat tolerance and one edge
/// graph, and only when a group asks for it: no groups builds nothing, so the graph handed back
/// is `None` there.
pub fn resolve_all(
    groups: &[BevelEdgeGroup],
    mesh: &Mesh,
) -> (Vec<BevelEdgeGroupResolution>, Option<MeshEdgeGraph>) {
    resolve_all_with(groups, mesh, MatchTolerances::default())
}

/// [`resolve_all`] at the given tolerances.
pub fn resolve_all_with(
    groups: &[BevelEdgeGroup],
    mesh: &Mesh,
    tolerances: MatchTolerances,
) -> (Vec<BevelEdgeGroupResolution>, Option<MeshEdgeGraph>) {
    let mut graph: Option<MeshEdgeGraph> = None;
    let mut resolutions = Vec::with_capacity(groups.len());
    // keyed by the tolerance's value, so groups at one tolerance share one set of candidates
    let mut by_flat_angle: HashMap<u64, MatchCandidates> = HashMap::new();

    for group in groups {
        let key = flat_angle_key(group.flat_angle_degrees);
        let graph_ref = graph.get_or_insert_with(|| mesh.get_mesh_edge_graph());
        let candidates = by_flat_angle.entry(key).or_insert_with(|| {
            MatchCandidates::new(Arc::new(FeatureEdgeSet::create_with_graph(
                mesh,
                graph_ref,
                group.flat_angle_degrees,
            )))
        });

        resolutions.push(resolve_candidates(
            group.clone(),
            mesh,
            Some(graph_ref),
            candidates,
            tolerances,
        ));
    }

    (resolutions, graph)
}

/// Resolves one group among a feature set the caller already has, reading the group at that
/// set's tolerance first (the operation's bend threshold is the only answer to "is this an
/// edge"). `graph` must be `mesh`'s own; `None` builds one only if a face group needs it.
pub fn resolve_in(
    group: &BevelEdgeGroup,
    mesh: &Mesh,
    graph: Option<&MeshEdgeGraph>,
    features: Arc<FeatureEdgeSet>,
    tolerances: MatchTolerances,
) -> BevelEdgeGroupResolution {
    let group = group.read_at(features.flat_angle_degrees());
    resolve_candidates(
        group,
        mesh,
        graph,
        &MatchCandidates::new(features),
        tolerances,
    )
}

/// Resolves a group as a CREASE: the anchor matched as usual, then walked with
/// [`FeatureEdgeChain::propagate_crease`] at the given tolerances.
pub fn resolve_crease(
    group: &BevelEdgeGroup,
    features: Arc<FeatureEdgeSet>,
    crease_direction_tolerance_degrees: f64,
    crease_face_tolerance_degrees: f64,
    tolerances: MatchTolerances,
) -> BevelEdgeGroupResolution {
    let group = group.read_at(features.flat_angle_degrees());
    let candidates = MatchCandidates::new(features);

    let (seed_edge_index, _) = match try_match_anchor(&group, &candidates, tolerances) {
        Ok(matched) => matched,
        Err(reason) => return BevelEdgeGroupResolution::failed(group, candidates.features, reason),
    };

    let chain = FeatureEdgeChain::propagate_crease(
        &candidates.features,
        seed_edge_index,
        crease_direction_tolerance_degrees,
        crease_face_tolerance_degrees,
    )
    .expect("the matched seed is an index into the feature set");
    let edges = chain_edges(&chain);

    BevelEdgeGroupResolution::succeeded(
        group,
        candidates.features,
        seed_edge_index,
        edges,
        Some(chain),
        None,
    )
}

fn resolve_candidates(
    group: BevelEdgeGroup,
    mesh: &Mesh,
    graph: Option<&MeshEdgeGraph>,
    candidates: &MatchCandidates,
    tolerances: MatchTolerances,
) -> BevelEdgeGroupResolution {
    let features = Arc::clone(&candidates.features);

    let (seed_edge_index, faces_are_swapped) =
        match try_match_anchor(&group, candidates, tolerances) {
            Ok(matched) => matched,
            Err(reason) => return BevelEdgeGroupResolution::failed(group, features, reason),
        };

    let seed = features.edges()[seed_edge_index as usize];

    match group.selection_kind {
        BevelSelectionKind::TangentChain => {
            // One tolerance for both tests: the group stores a single "how far may the surface
            // turn" number.
            let chain = FeatureEdgeChain::propagate_with(
                &features,
                seed_edge_index,
                group.curve_tolerance_degrees,
                group.curve_tolerance_degrees,
            )
            .expect("the matched seed is an index into the feature set");
            let edges = chain_edges(&chain);
            BevelEdgeGroupResolution::succeeded(
                group,
                features,
                seed_edge_index,
                edges,
                Some(chain),
                None,
            )
        }
        BevelSelectionKind::FaceGroup => {
            // AnchorNormal0 names the side the region is on, so the flood starts from whichever
            // face matched it - Face0 and Face1 change places freely across a rebuild.
            let seed_face = if faces_are_swapped {
                seed.face1
            } else {
                seed.face0
            };
            let built_graph;
            let graph = match graph {
                Some(graph) => graph,
                None => {
                    built_graph = mesh.get_mesh_edge_graph();
                    &built_graph
                }
            };
            let face_group =
                FaceGroup::create_with_graph(mesh, graph, seed_face, group.curve_tolerance_degrees)
                    .expect("a feature edge's faces are faces of the mesh it was read from");

            if face_group.boundary_edges().is_empty() {
                let reason = describe_empty_boundary(&group, &face_group);
                return BevelEdgeGroupResolution::failed(group, features, reason);
            }

            let edges = face_group.boundary_edges().to_vec();
            BevelEdgeGroupResolution::succeeded(
                group,
                features,
                seed_edge_index,
                edges,
                None,
                Some(face_group),
            )
        }
        BevelSelectionKind::Edge | BevelSelectionKind::Point => {
            // An edge pick means the whole straight crease, walked at the collinear tolerance
            // (only about what straight means). Not handed back as `chain`: that means "was
            // walked as a tangent chain" to everything downstream.
            let run = FeatureEdgeChain::propagate_collinear(&features, seed_edge_index)
                .expect("the matched seed is an index into the feature set");
            let edges = chain_edges(&run);
            BevelEdgeGroupResolution::succeeded(group, features, seed_edge_index, edges, None, None)
        }
    }
}

fn chain_edges(chain: &FeatureEdgeChain) -> Vec<FeatureEdge> {
    chain
        .edges()
        .iter()
        .map(|chain_edge| chain_edge.edge)
        .collect()
}

fn create(
    seed: &FeatureEdge,
    selection_kind: BevelSelectionKind,
    curve_tolerance_degrees: f64,
    flat_angle_degrees: f64,
    swap_faces: bool,
) -> BevelEdgeGroup {
    let midpoint = midpoint(seed);

    // Swapping the two faces also reverses the edge direction - the sign convention FeatureEdge
    // documents - so a reader can rebuild the profile frame from the anchor alone.
    let normal0 = if swap_faces {
        seed.normal1
    } else {
        seed.normal0
    };
    let normal1 = if swap_faces {
        seed.normal0
    } else {
        seed.normal1
    };

    BevelEdgeGroup {
        selection_kind,
        anchor_midpoint: midpoint,
        anchor_direction: if swap_faces {
            -seed.direction
        } else {
            seed.direction
        },
        anchor_normal0: normal0,
        anchor_plane_distance0: normal0.dot(midpoint),
        anchor_normal1: normal1,
        anchor_plane_distance1: normal1.dot(midpoint),
        source_item_id: None,
        curve_tolerance_degrees,
        flat_angle_degrees,
    }
}

/// The matched seed edge index and whether its faces pair with the anchor's swapped, or why
/// nothing matched.
fn try_match_anchor(
    group: &BevelEdgeGroup,
    candidates: &MatchCandidates,
    tolerances: MatchTolerances,
) -> Result<(i32, bool), BevelAdvisory> {
    let features = &candidates.features;

    if features.count() == 0 {
        return Err(BevelAdvisory::new(
            BevelAdvisoryCode::NoFeatureEdgesAtTolerance,
            vec![group.flat_angle_degrees.into()],
        ));
    }

    if group.anchor_direction.length_squared() <= 0.0 {
        return Err(BevelAdvisory::new(
            BevelAdvisoryCode::GroupHasNoAnchorDirection,
            Vec::new(),
        ));
    }

    let anchor_direction = group.anchor_direction.get_normal();
    let maximum_drift = candidates.feature_extent * tolerances.drift_fraction_of_feature_extent;
    let minimum_direction_agreement =
        math_helper::degrees_to_radians(tolerances.direction_tolerance_degrees).cos();
    let minimum_normal_agreement =
        math_helper::degrees_to_radians(tolerances.normal_tolerance_degrees).cos();

    // Each score term is divided by the most it can be at its own gate, so each runs 0 (perfect)
    // to 1 (only just allowed through) and the equal weights really are equal.
    let worst_direction_term = 1.0 - minimum_direction_agreement;
    let worst_normal_term = 1.0 - minimum_normal_agreement;

    let mut best_score = f64::MAX;
    let mut runner_up_score = f64::MAX;
    let mut seed_edge_index = -1;
    let mut faces_are_swapped = false;

    for (i, candidate) in features.edges().iter().enumerate() {
        let drift = (midpoint(candidate) - group.anchor_midpoint).length();
        if drift > maximum_drift {
            continue;
        }

        // sign-insensitive: a rebuilt mesh may wind the same edge the other way round
        let direction_agreement = candidate.direction.dot(anchor_direction).abs();
        if direction_agreement < minimum_direction_agreement {
            continue;
        }

        // Both face pairings are tried and the better one taken; the worse normal within a
        // pairing is what stops one kept face carrying an edge whose other face is new.
        let straight_pairing = candidate
            .normal0
            .dot(group.anchor_normal0)
            .min(candidate.normal1.dot(group.anchor_normal1));
        let swapped_pairing = candidate
            .normal1
            .dot(group.anchor_normal0)
            .min(candidate.normal0.dot(group.anchor_normal1));
        let swapped = swapped_pairing > straight_pairing;
        let normal_agreement = if swapped {
            swapped_pairing
        } else {
            straight_pairing
        };
        if normal_agreement < minimum_normal_agreement {
            continue;
        }

        let score = term(drift, maximum_drift)
            + term(1.0 - direction_agreement, worst_direction_term)
            + term(1.0 - normal_agreement, worst_normal_term);

        if score < best_score {
            runner_up_score = best_score;
            best_score = score;
            seed_edge_index = i as i32;
            faces_are_swapped = swapped;
        } else if score < runner_up_score {
            runner_up_score = score;
        }
    }

    if seed_edge_index < 0 {
        return Err(BevelAdvisory::new(
            BevelAdvisoryCode::NoFeatureEdgeMatchesAnchor,
            vec![
                group.anchor_midpoint.into(),
                (features.count() as i32).into(),
                maximum_drift.into(),
                tolerances.direction_tolerance_degrees.into(),
                tolerances.normal_tolerance_degrees.into(),
            ],
        ));
    }

    // Repeated geometry defeats every per-candidate gate: two matches this close together mean
    // the anchor no longer names one edge.
    if runner_up_score < f64::MAX && best_score >= runner_up_score * tolerances.ambiguity_ratio {
        return Err(BevelAdvisory::new(
            BevelAdvisoryCode::AnchorMatchesMoreThanOneEdge,
            vec![
                group.anchor_midpoint.into(),
                best_score.into(),
                runner_up_score.into(),
            ],
        ));
    }

    Ok((seed_edge_index, faces_are_swapped))
}

fn term(value: f64, worst_allowed: f64) -> f64 {
    if worst_allowed > 0.0 {
        value / worst_allowed
    } else {
        0.0
    }
}

fn describe_empty_boundary(group: &BevelEdgeGroup, face_group: &FaceGroup) -> BevelAdvisory {
    if !face_group.open_boundary_mesh_edges().is_empty() {
        return BevelAdvisory::new(
            BevelAdvisoryCode::FaceRegionBoundedOnlyByOpenEdges,
            vec![
                group.anchor_midpoint.into(),
                (face_group.open_boundary_mesh_edges().len() as i32).into(),
            ],
        );
    }

    BevelAdvisory::new(
        BevelAdvisoryCode::FaceRegionHasNoBevelableBoundary,
        vec![
            group.anchor_midpoint.into(),
            group.curve_tolerance_degrees.into(),
        ],
    )
}

fn midpoint(edge: &FeatureEdge) -> Vector3 {
    (edge.start + edge.end) / 2.0
}

/// A map key for a tolerance that compares as the value does, so 0.0 and -0.0 are one key.
fn flat_angle_key(value: f64) -> u64 {
    if value == 0.0 {
        0
    } else {
        value.to_bits()
    }
}

/// A feature set with the extent the drift gate is a fraction of.
struct MatchCandidates {
    features: Arc<FeatureEdgeSet>,
    feature_extent: f64,
}

impl MatchCandidates {
    fn new(features: Arc<FeatureEdgeSet>) -> Self {
        let feature_extent = measure_extent(&features);
        MatchCandidates {
            features,
            feature_extent,
        }
    }
}

fn measure_extent(features: &FeatureEdgeSet) -> f64 {
    let mut bounds = AxisAlignedBoundingBox::empty();
    for edge in features.edges() {
        bounds.expand_to_include_point(edge.start);
        bounds.expand_to_include_point(edge.end);
    }

    if features.count() > 0 {
        bounds.size().length()
    } else {
        0.0
    }
}
