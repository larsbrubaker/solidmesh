//! The readings of a group that is already stored: the same list
//! [`super::pick_interpretations`] offers a hover, reached from the group rather than from the
//! cursor. Ports `Topology/BevelEdgeReadings.cs`.
//!
//! Cycling readings is a viewport gesture - hover an edge, press Tab - which means a selection
//! can only be re-read while the mouse is on it. The property panel shows a row for a group
//! nobody is hovering, and the one thing a user wants to say to that row ("no, I meant the whole
//! rim") needs this: a stored group turned back into the readings it came from.
//!
//! The list has to be the SAME list Tab walks, or there are two selection languages to learn.
//! Two things make it the same. The readings are built by [`super::pick_interpretations`] itself,
//! so the order and the dedup rule are not restated; and the set they are seeded from is the
//! tightest rung of the ladder that still holds the edge, not the set the group happens to have
//! been stored at. A group stored at ten degrees resolves in a ten degree set, where a
//! tessellated wall is one surface and the faceted reading of its rim does not exist; seeding
//! from that set would quietly drop the reading the user most wants back. An edge that is a
//! feature edge at a coarse tolerance is a feature edge at every finer one, so the tightest rung
//! always holds it and always offers the fullest list.

use std::sync::Arc;

use polygon_mesh::{Mesh, MeshEdgeGraph};

use super::bevel_edge_group::{BevelEdgeGroup, BevelSelectionKind};
use super::bevel_edge_group_resolver::{self, MatchTolerances};
use super::bevel_reading_label::BevelReadingLabel;
use super::error::TopologyError;
use super::feature_edge::FeatureEdge;
use super::feature_edge_set::FeatureEdgeSet;
use super::pick_interpretation::PickInterpretation;
use super::pick_interpretations;

/// The readings of a stored group against one mesh, seeded from the tightest ladder rung that
/// holds the group's anchored edge. Empty when the anchor does not resolve on this mesh - a group
/// pointing at geometry that is gone has no readings, and the readings of whichever edge came
/// nearest would be a different selection wearing this one's name.
///
/// `mesh` is the bevel's working copy, not the raw source; `graph` its edge graph if the caller
/// has one (built where needed when `None`). Every set of `ladder` must have been built over
/// `mesh` and that graph, because readings are matched across rungs by mesh edge index. A
/// [`BevelSelectionKind::FaceGroup`] group is re-read like any other: a pick offers the rim of
/// each face its edge divides, so the list contains the region the group is on.
///
/// Errors where C# throws `ArgumentException` for a ladder with no rungs. (C#'s null-rung check
/// has no Rust counterpart: a rung cannot be null.)
pub fn for_group(
    mesh: &Arc<Mesh>,
    graph: Option<&MeshEdgeGraph>,
    ladder: &[Arc<FeatureEdgeSet>],
    group: &BevelEdgeGroup,
) -> Result<Vec<PickInterpretation>, TopologyError> {
    // The anchor is matched in the ladder's FINEST rung, which for a bevel's snapshot is the
    // operation's own bend threshold - never in a set built at whatever angle the group carries.
    // The angle is the operation's, so a group read at its own would be answering about edges
    // the build does not have; and a tolerance that was not a rung would send this down a full
    // feature scan of the mesh, on the UI thread, every time the panel drew a row.
    let anchor_features = finest_rung(ladder).ok_or(TopologyError::EmptyLadder)?;

    let resolution = bevel_edge_group_resolver::resolve_in(
        group,
        mesh,
        graph,
        anchor_features,
        MatchTolerances::default(),
    );

    if !resolution.is_resolved() {
        return Ok(Vec::new());
    }

    let seed_mesh_edge_index =
        resolution.features.edges()[resolution.seed_edge_index as usize].mesh_edge_index;

    // The rung a fresh hover would have picked in: the finest reading of this edge that exists.
    // The group's own set is the fallback rather than the answer, for the reason in the header.
    let mut pick_features = resolution.features.clone();
    let mut seed_edge_index = resolution.seed_edge_index;

    for rung in ladder {
        if rung.flat_angle_degrees() >= pick_features.flat_angle_degrees() {
            continue;
        }

        let rung_seed_index = pick_interpretations::index_of_mesh_edge(rung, seed_mesh_edge_index);

        if rung_seed_index >= 0 {
            pick_features = rung.clone();
            seed_edge_index = rung_seed_index;
        }
    }

    // The group's own curve tolerance rather than the pick default, so re-reading a selection
    // keeps how far the walk may turn - the one number of the pick the stored group still
    // carries.
    pick_interpretations::build(
        mesh,
        graph,
        &pick_features,
        seed_edge_index,
        group.curve_tolerance_degrees,
        ladder,
        None,
    )
}

/// The same readings where the caller has only the mesh (C#'s two-argument `ForGroup`). This
/// detects the whole default tolerance ladder - the expensive part of picking, a full feature
/// scan per rung - so it is for tests and for callers with nothing better; anything on the UI
/// thread should be handing in the pick data's own snapshot.
pub fn for_group_on_mesh(
    mesh: &Arc<Mesh>,
    group: &BevelEdgeGroup,
) -> Result<Vec<PickInterpretation>, TopologyError> {
    let graph = mesh.get_mesh_edge_graph();

    let ladder: Vec<Arc<FeatureEdgeSet>> =
        pick_interpretations::default_flat_angle_ladder_degrees()
            .iter()
            .map(|&flat_angle_degrees| {
                Arc::new(FeatureEdgeSet::create_with_graph(
                    mesh,
                    &graph,
                    flat_angle_degrees,
                ))
            })
            .collect();

    for_group(mesh, Some(&graph), &ladder, group)
}

/// Which reading a group is on now, given what it resolved to, or -1 when none of them selects
/// that.
///
/// `resolved_edges` and `readings` must come from ONE pick data snapshot: the comparison is by
/// [`FeatureEdge::mesh_edge_index`], which numbers the edges of the mesh they were measured
/// against. Compared by the edges selected rather than by the group's kind and tolerance, which
/// is the same rule the viewport's click follows: two picks on one rim make two groups with
/// different anchors and one selection. A group that resolved to something no reading offers
/// says so with -1 instead of naming a reading it is not on.
pub fn index_of_current_reading(
    readings: &[PickInterpretation],
    resolved_edges: &[FeatureEdge],
) -> i32 {
    if resolved_edges.is_empty() {
        return -1;
    }

    readings
        .iter()
        .position(|reading| {
            pick_interpretations::select_the_same_edges(reading.edges(), resolved_edges)
        })
        .map_or(-1, |i| i as i32)
}

/// What one reading of a list calls itself in a chooser: "1 edge", "64 edges, chain", "64 edges,
/// chain at 10°".
///
/// The list decides whether the tolerance is shown, rather than the caller saying so. Asked for
/// as a bare flag it was got wrong the obvious way - `readings.len() > 1` is true of the everyday
/// [single edge, one chain] list. Here the question is asked of the readings themselves: more
/// than one CHAIN, which is the only thing the number tells apart.
///
/// Errors where C# throws `ArgumentOutOfRangeException` for an `index` not into `readings`.
pub fn label(
    readings: &[PickInterpretation],
    index: i32,
) -> Result<BevelReadingLabel, TopologyError> {
    if index < 0 || index as usize >= readings.len() {
        return Err(TopologyError::ReadingIndexOutOfRange {
            index,
            reading_count: readings.len(),
        });
    }

    let chains = readings
        .iter()
        .filter(|reading| reading.group().selection_kind == BevelSelectionKind::TangentChain)
        .count();

    Ok(label_reading(&readings[index as usize], chains > 1))
}

/// What a reading calls itself in a chooser, told whether its list holds more than one chain -
/// as the facts of the label, which the application puts into words.
///
/// Named by what the reading would STORE - [`BevelEdgeGroup::selection_kind`] - rather than by
/// [`super::PickInterpretationKind`], because that is the axis that already knows about face
/// rims. C# `internal` (the panel calls [`label`], which derives the flag); kept as its own
/// function because tests pin both labels of the chain reading directly.
pub fn label_reading(
    reading: &PickInterpretation,
    ladder_offers_more_than_one_chain: bool,
) -> BevelReadingLabel {
    let kind = reading.group().selection_kind;

    // The tolerance is a fact about chains only: a face rim or a single edge has no other
    // reading of itself to be told apart from, whatever else is on the list.
    BevelReadingLabel::new(
        kind,
        // a reading's edge count is a mesh's edge count, far inside i32
        reading.count() as i32,
        reading.flat_angle_degrees(),
        kind == BevelSelectionKind::TangentChain && ladder_offers_more_than_one_chain,
    )
}

/// The tightest rung of a ladder - the one a pick runs in, and the only angle a bevel's snapshot
/// carries. The first of equal rungs, as C#'s strict `<` scan keeps it. `None` for no rungs.
fn finest_rung(ladder: &[Arc<FeatureEdgeSet>]) -> Option<Arc<FeatureEdgeSet>> {
    let mut finest = ladder.first()?;

    for rung in ladder {
        if rung.flat_angle_degrees() < finest.flat_angle_degrees() {
            finest = rung;
        }
    }

    Some(finest.clone())
}
