//! Port of `R/Tests/GeometryProcessingTests/BevelSecondPassMergeTests.cs`: what the second pass
//! (`bevel_second_pass`) makes of the creases it re-reads - which of them are the SAME crease
//! ([`merge`]), which members a run's swept reading answers for
//! ([`longest_reading_and_what_it_covers`]), which of them are the crease the pick actually named,
//! and what it does when the replacement will not sweep - on the nose-in-notch part with its ridge
//! rounded (`nose_in_notch_cut`).

use geometry_processing::bevel::bevel_crease_merge::merge;
use geometry_processing::bevel::bevel_second_pass::{
    self, face_tolerance_degrees, names_the_same_crease, BevelSecondPassRebead,
    ANCHOR_FRACTION_OF_THE_DIAGONAL,
};
use geometry_processing::bevel::bevel_second_pass_run_plan::longest_reading_and_what_it_covers;
use geometry_processing::topology::bevel_edge_group_resolver;
use geometry_processing::topology::{
    BevelEdgeGroup, FeatureEdgeChain, FeatureEdgeSet, MatchTolerances,
};
use polygon_mesh::{CancelToken, Mesh};
use vector_math::Vector3;

use super::nose_in_notch_cut::cut_ridge;
use super::nose_in_notch_test_meshes;

const RADIUS: f64 = 2.0;

const SEGMENTS: i32 = 16;

fn readings(lists: &[Option<&[i32]>]) -> Vec<Option<Vec<i32>>> {
    lists
        .iter()
        .map(|list| list.map(|edges| edges.to_vec()))
        .collect()
}

fn sorted(mut list: Vec<usize>) -> Vec<usize> {
    list.sort();
    list
}

#[test]
fn a_crease_shared_through_a_middle_one_is_all_one_run() {
    let merged = merge(&readings(&[
        Some(&[10, 11]),
        Some(&[30, 31]),
        Some(&[11, 12, 30]),
    ]));

    // The third crease shares an edge with each of the other two, so all three are one crease.
    assert_eq!(merged.len(), 1);
    assert_eq!(merged[0], vec![0, 1, 2]);
}

#[test]
fn creases_that_share_nothing_stay_apart_and_keep_their_order() {
    let merged = merge(&readings(&[Some(&[5]), None, Some(&[6]), Some(&[5, 7])]));

    assert_eq!(merged.len(), 2);
    // The last shares an edge with the first, and the run is named by the earliest in it.
    assert_eq!(merged[0], vec![0, 3]);
    // And the one that shares nothing is its own run - the null taking no part at all.
    assert_eq!(merged[1], vec![2]);
}

#[test]
fn no_member_of_a_merged_run_loses_its_bead_to_a_reading_that_does_not_cover_it() {
    let readings_ = readings(&[Some(&[10, 11]), Some(&[11, 12, 30]), Some(&[30, 31])]);

    let merged = merge(&readings_);

    // A shares an edge with B and B with C, so the three of them are one run.
    assert_eq!(merged.len(), 1);

    let (longest, covered, not_covered) =
        longest_reading_and_what_it_covers(&merged[0], &readings_);

    // B reads the most of the crease, so B's reading is the one swept.
    assert_eq!(longest, 1);
    // Only the member the swept chain really carries may have its first-pass bead withdrawn.
    assert_eq!(covered, vec![1]);
    // A and C each read an edge B never touches, so both keep the bead the first pass built.
    assert_eq!(sorted(not_covered.clone()), vec![0, 2]);
    // Every member is accounted for one way or the other - a member in neither list is a bead
    // deleted in silence.
    let mut all = covered.clone();
    all.extend(not_covered);
    assert_eq!(sorted(all), vec![0, 1, 2]);

    let inside = readings(&[Some(&[11, 12]), Some(&[11, 12, 30]), Some(&[30, 31])]);

    let (_, also_covered, still_kept) =
        longest_reading_and_what_it_covers(&merge(&inside)[0], &inside);

    // The first reading is wholly inside the one being swept, so that bead really is replaced.
    assert_eq!(sorted(also_covered), vec![0, 1]);
    assert_eq!(still_kept, vec![2]);
}

/// Two picks whose creases the round joined come back as ONE run, even though they were picked at
/// different flat tolerances. Each distinct flat angle in the group list gets its own
/// `FeatureEdgeSet`, and an edge's index in one of those sets is not its index in another - so a
/// merge keyed on `ChainEdge::edge_index` reads two picks of the same crease as two different
/// creases and beads it twice, which is a duplicate solid the union cannot fold.
#[test]
fn one_crease_is_named_by_its_mesh_edges_whichever_tolerance_read_it() {
    let cut = cut_ridge(RADIUS, SEGMENTS);
    let upper = cut.upper.clone();
    let mut lower = cut.lower.clone();

    // Looser than the default on one of them, which is what a smooth-wall reading leaves behind: it
    // drops the two flattest joins of the cut part from that group's feature set, and so numbers
    // every edge after them differently from the way the other group's set does.
    lower.flat_angle_degrees = 5.0;

    let groups = vec![upper.clone(), lower.clone()];

    let (upper_feature_indices, upper_mesh_edges) = read(&cut.cut, &upper);
    let (lower_feature_indices, lower_mesh_edges) = read(&cut.cut, &lower);

    assert_eq!(
        upper_mesh_edges, lower_mesh_edges,
        "both picks read the very same run of the cut part, which is what makes them one crease"
    );

    assert_ne!(
        upper_feature_indices, lower_feature_indices,
        "and the two tolerances number that run differently, which is why the feature set index \
         cannot be what identifies it"
    );

    let rebead = rebead(&cut.cut, &groups, RADIUS);

    assert_eq!(
        rebead.runs.len(),
        1,
        "both picks name the one crease the round made of them, so there is one bead to roll"
    );

    assert_eq!(
        sorted(rebead.runs[0].groups.clone()),
        vec![0, 1],
        "and both rows are told they were built as that one run"
    );

    assert_eq!(
        sorted(rebead.rebuilt.iter().copied().collect()),
        vec![0, 1],
        "so both rows' first-pass beads come off the batch"
    );
}

/// The gate is opened deliberately (see `bevel_second_pass`), which means the resolver is allowed
/// to settle a tie rather than refuse it - and a tie settled the wrong way would replace a bead the
/// first pass built correctly with one rolled along a crease somewhere else on the part.
#[test]
fn a_crease_is_only_taken_over_when_it_is_the_one_the_pick_names() {
    let cut = cut_ridge(RADIUS, SEGMENTS);
    let upper = cut.upper.clone();
    let lower = cut.lower.clone();

    let features = std::sync::Arc::new(FeatureEdgeSet::create_with_graph(
        &cut.cut,
        &cut.cut.get_mesh_edge_graph(),
        upper.flat_angle_degrees,
    ));
    let face_tolerance = face_tolerance_degrees(SEGMENTS);

    let crease = bevel_edge_group_resolver::resolve_crease(
        &upper,
        features,
        FeatureEdgeChain::DEFAULT_TANGENT_ANGLE_DEGREES,
        face_tolerance,
        gate_open(),
    );

    assert!(
        crease.is_resolved(),
        "the fixture means nothing unless the upper pick reads a crease off the cut part"
    );

    let chain = crease
        .chain
        .as_ref()
        .expect("a resolved crease has a chain");
    let tolerance =
        ANCHOR_FRACTION_OF_THE_DIAGONAL * cut.cut.get_axis_aligned_bounding_box().size().length();
    let (on_the_uncut_part, _) = bevel_edge_group_resolver::resolve_all(
        &[upper.clone(), lower],
        &nose_in_notch_test_meshes::create(),
    );

    assert!(
        names_the_same_crease(chain, &upper, &on_the_uncut_part[0], tolerance),
        "this IS the crease that pick names - it runs through the pick's own midpoint"
    );

    // The same crease, offered a pick from the far side of the part: same shape of anchor, nowhere
    // near this run and sharing none of its edges.
    let mut elsewhere: BevelEdgeGroup = upper.clone();
    elsewhere.anchor_midpoint = upper.anchor_midpoint + Vector3::new(0.0, 0.0, 100.0);

    assert!(
        !names_the_same_crease(chain, &elsewhere, &on_the_uncut_part[1], tolerance),
        "a crease that neither passes the pick nor carries any edge the first pass resolved is a \
         different crease, and that row keeps the bead it has"
    );
}

/// A crease whose replacement will not sweep KEEPS its first-pass bead. Marking it rebuilt would
/// take that bead off the batch with nothing to put in its place, which answers a corner the first
/// pass filled correctly with no fillet at all.
#[test]
fn a_crease_whose_replacement_will_not_sweep_keeps_the_bead_it_has() {
    let cut = cut_ridge(RADIUS, SEGMENTS);
    let groups = vec![cut.upper.clone(), cut.lower.clone()];

    // Far larger than the part, so the crease still reads and merges but nothing can be swept along
    // it - which is the shape of every second-pass sweep refusal.
    let rebead = rebead(&cut.cut, &groups, 200.0);

    assert_eq!(
        rebead.runs.len(),
        0,
        "the fixture means nothing unless the sweep really does refuse this size"
    );

    assert_eq!(
        rebead.rebuilt.len(),
        0,
        "nothing was built, so nothing of the first pass may be taken away"
    );

    // Every kept row carries its reason by construction (a `BevelAdvisory`, never null), which is
    // C#'s "a kept bead with no reason beside it is a silent fallback".
    assert_eq!(
        sorted(
            rebead
                .kept_first_pass
                .iter()
                .map(|(group, _)| *group)
                .collect()
        ),
        vec![0, 1],
        "both rows are told why their fillet is still the one built before the cut"
    );
}

/// C#'s `ambiguityRatio: 1`: the resolver's gate open, every other tolerance at its default.
fn gate_open() -> MatchTolerances {
    MatchTolerances {
        ambiguity_ratio: 1.0,
        ..MatchTolerances::default()
    }
}

/// C# `Read`: the crease this group reads off the cut part, by its feature set index and by the
/// mesh edge each of those stands for - the two candidate identities.
fn read(cut: &Mesh, group: &BevelEdgeGroup) -> (Vec<i32>, Vec<i32>) {
    let features = std::sync::Arc::new(FeatureEdgeSet::create_with_graph(
        cut,
        &cut.get_mesh_edge_graph(),
        group.flat_angle_degrees,
    ));

    let crease = bevel_edge_group_resolver::resolve_crease(
        group,
        features,
        FeatureEdgeChain::DEFAULT_TANGENT_ANGLE_DEGREES,
        face_tolerance_degrees(SEGMENTS),
        gate_open(),
    );
    let chain = crease.chain.as_ref().expect("the pick reads a crease");

    let mut feature_indices: Vec<i32> = chain.edges().iter().map(|edge| edge.edge_index).collect();
    let mut mesh_edges: Vec<i32> = chain
        .edges()
        .iter()
        .map(|edge| edge.edge.mesh_edge_index)
        .collect();
    feature_indices.sort();
    mesh_edges.sort();

    (feature_indices, mesh_edges)
}

/// C# `Rebead`: the second pass over the cut part, against the first pass's readings taken on the
/// part as it was BEFORE the ridge was rounded - which is where the rebuild takes them and what the
/// guard measures a re-read crease against.
fn rebead(
    cut: &std::sync::Arc<Mesh>,
    groups: &[BevelEdgeGroup],
    size: f64,
) -> BevelSecondPassRebead {
    let (resolutions, _) =
        bevel_edge_group_resolver::resolve_all(groups, &nose_in_notch_test_meshes::create());
    let eligible = vec![true; groups.len()];

    bevel_second_pass::rebead(
        Some(cut),
        groups,
        &resolutions,
        &eligible,
        true,
        size,
        SEGMENTS,
        &CancelToken::new(),
    )
    .expect("not cancelled")
}
