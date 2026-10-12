//! Port of `LevelLine/CubeEdgeUnionTests.cs`: the plan's goal on the simplest part there is - on a
//! cube, every union of up to two face rims and up to two single edges builds pick-wide at a tenth
//! of the side, closed, one shell and consistently oriented. The union is linked the way the app
//! links a pick (each mesh edge once, by the production `BevelChainLinker`), so a rim and an edge
//! that overlap or continue each other are one edge set, not two picks. C# `[Arguments]` cases run
//! as one loop per test.

use std::collections::{BTreeMap, BTreeSet, HashMap, HashSet};
use std::rc::Rc;

use geometry_processing::bevel::bevel_chain_linker;
use geometry_processing::bevel::level_line::band_gap_entry_points::{self, BandGapBuildOptions};
use geometry_processing::bevel::level_line::band_gap_in_turn::{
    build_open_chains_in_turn, takes_open_chains_in_turn,
};
use geometry_processing::bevel::level_line::seam_edges::FeatureEdgeTest;
use geometry_processing::bevel::level_line::{
    gap_fill, gap_loop_walk, BandGapResult, BandProfile, GapFillResult, LevelLineMesh, WorkBudget,
};
use geometry_processing::topology::FeatureEdgeSet;
use polygon_mesh::Mesh;
use vector_math::Vector3;

use crate::level_line_feature_edges::feature_edges_over;
use crate::level_line_test_geometry::{cube_solid, signed_volume};

const FOLD_DEGREES: f64 = 30.0;

/// Half 1: the side is 2 and a tenth of it is 0.2.
const SIZE: f64 = 0.2;

/// C# `FaceRims`.
pub const FACE_RIMS: [[i32; 4]; 6] = [
    [0, 1, 2, 3],
    [4, 5, 6, 7],
    [0, 1, 5, 4],
    [1, 2, 6, 5],
    [2, 3, 7, 6],
    [3, 0, 4, 7],
];

type EdgeSet = BTreeSet<(i32, i32)>;

/// The cube, its feature test and C#'s `FoldDegrees` reading.
fn cube() -> (Vec<Vector3>, Vec<(i32, i32, i32)>, FeatureEdgeTest) {
    let (positions, indices) = cube_solid(1.0);
    let feature = feature_edges_over(&positions, &indices, FOLD_DEGREES);
    (positions, indices, feature)
}

/// C# `UnionsOfTwoRimsAndTwoEdgesBuildClosedNoFewerThanTheFloor`: run as a RATCHET - the counts
/// that build (pick-wide, and on the app's route) may only go up. Two open edges sharing no vertex
/// never build pick-wide; the app builds them in turn, so the app's route makes the full claim of
/// 509 for both profiles. A build counts only with the right SHAPE ([`shape`]).
#[test]
fn unions_of_two_rims_and_two_edges_build_closed_no_fewer_than_the_floor() {
    for (profile, pick_wide_floor, app_route_floor) in [
        (BandProfile::Chamfer, 407, 509),
        (BandProfile::Fillet, 407, 509),
    ] {
        let (positions, indices, feature) = cube();

        let mut pick_wide_failures = 0;
        let mut failures = Vec::new();
        let unions = every_union();
        for edges in &unions {
            let (chains, closed) = link(&positions, &indices, edges);
            let (pick_wide, gap) =
                try_pick_wide(&positions, &indices, &feature, &chains, &closed, profile);
            if let Some(said) = &pick_wide {
                pick_wide_failures += 1;
                if said.starts_with("shape:") {
                    println!("  pick-wide {}: {said}", describe(edges));
                }
            }

            // The app's route (BevelSurfacePartBuilder.TheLevelLineBuild): a combination the
            // pick-wide call cannot take, or one of open chains that share no loop, is built a chain
            // at a time.
            if pick_wide.is_some() || takes_open_chains_in_turn(gap.as_ref(), &closed) {
                let in_turn = if chains.len() > 1 {
                    try_in_turn(
                        &positions,
                        &indices,
                        &feature,
                        &chains,
                        &closed,
                        profile,
                        gap.as_ref(),
                    )
                } else {
                    Some("one chain".to_string())
                };
                if let Some(in_turn) = in_turn {
                    failures.push(format!(
                        "{}: {}; in turn: {in_turn}",
                        describe(edges),
                        pick_wide.as_deref().unwrap_or("taken in turn")
                    ));
                }
            }
        }

        println!(
            "{profile:?}: {} of {} unions build pick-wide, {} on the app's route",
            unions.len() - pick_wide_failures,
            unions.len(),
            unions.len() - failures.len()
        );
        for failure in &failures {
            println!("  {failure}");
        }

        assert!(
            unions.len() - pick_wide_failures >= pick_wide_floor,
            "{profile:?}"
        );
        assert!(
            unions.len() - failures.len() >= app_route_floor,
            "{profile:?}"
        );
    }
}

/// C# `ARimWithAnEdgeOffItsCornerBuildsPickWide`: a face's rim with an edge leaving one of its
/// corners, the edge's far end at its chain's start.
#[test]
fn a_rim_with_an_edge_off_its_corner_builds_pick_wide() {
    for profile in [BandProfile::Chamfer, BandProfile::Fillet] {
        let (positions, indices, feature) = cube();
        for edge in [(0, 1), (6, 7)] {
            let mut edges: EdgeSet = rim_edges(&FACE_RIMS[5]).collect();
            edges.insert(edge);
            let (chains, closed) = link(&positions, &indices, &edges);
            let (said, _) =
                try_pick_wide(&positions, &indices, &feature, &chains, &closed, profile);
            assert_eq!(said, None, "rim 3-0-4-7 with {edge:?}");
        }
    }
}

/// C# `ARimWithAPathOffItsCornerEndingOnTheRimsSideBuildsPickWide`: rim 0-1-2-3 with the path
/// 0-4-5, whose free end stands on the rim's own outer side - ONE stretch of the side's closed loop.
#[test]
fn a_rim_with_a_path_off_its_corner_ending_on_the_rims_side_builds_pick_wide() {
    for profile in [BandProfile::Chamfer, BandProfile::Fillet] {
        let (positions, indices, feature) = cube();
        let mut edges: EdgeSet = rim_edges(&FACE_RIMS[0]).collect();
        edges.insert((0, 4));
        edges.insert((4, 5));
        let (chains, closed) = link(&positions, &indices, &edges);
        let (said, _) = try_pick_wide(&positions, &indices, &feature, &chains, &closed, profile);
        assert_eq!(said, None, "{}", describe(&edges));
    }
}

/// C# `TwoEdgesEndingOnEachOthersSidesCutPickWideAndBuildInTurn`: edges 0-1 and 2-6 share no
/// vertex and each ends on a side of the other; the pick-wide cut stands, and the app builds it a
/// chain at a time.
#[test]
fn two_edges_ending_on_each_others_sides_cut_pick_wide_and_build_in_turn() {
    for profile in [BandProfile::Chamfer, BandProfile::Fillet] {
        let (positions, indices, feature) = cube();
        let edges: EdgeSet = [(0, 1), (2, 6)].into_iter().collect();
        let (chains, closed) = link(&positions, &indices, &edges);
        let (_, gap) = try_pick_wide(&positions, &indices, &feature, &chains, &closed, profile);
        let gap = gap.expect("the cut ran");
        assert!(gap.built(), "{:?}", gap.refusal);
        assert!(takes_open_chains_in_turn(Some(&gap), &closed));
        assert_eq!(
            try_in_turn(
                &positions,
                &indices,
                &feature,
                &chains,
                &closed,
                profile,
                Some(&gap)
            ),
            None
        );
    }
}

/// C# `ARimWithAnEdgeEndingOnAnotherEdgesSideBuildsPickWide`: rim 0-1-2-3 with edge 0-4 off its
/// corner and edge 5-6 touching neither.
#[test]
fn a_rim_with_an_edge_ending_on_another_edges_side_builds_pick_wide() {
    for profile in [BandProfile::Chamfer, BandProfile::Fillet] {
        let (positions, indices, feature) = cube();
        let mut edges: EdgeSet = rim_edges(&FACE_RIMS[0]).collect();
        edges.insert((0, 4));
        edges.insert((5, 6));
        let (chains, closed) = link(&positions, &indices, &edges);
        let (said, _) = try_pick_wide(&positions, &indices, &feature, &chains, &closed, profile);
        assert_eq!(said, None, "{}", describe(&edges));
    }
}

/// C# `AFillNeverSaysFilledOverADoubledFlap`: rim 3-0-4-7 and edge 3-2 - whatever the fill makes
/// of the pick, it never says Filled over a mesh that is not closed.
#[test]
fn a_fill_never_says_filled_over_a_doubled_flap() {
    for profile in [BandProfile::Chamfer, BandProfile::Fillet] {
        let (positions, indices, feature) = cube();
        let mut edges: EdgeSet = rim_edges(&FACE_RIMS[5]).collect();
        edges.insert((2, 3));
        let (chains, closed) = link(&positions, &indices, &edges);
        let gap = build(
            &positions, &indices, &feature, &chains, &closed, SIZE, profile,
        )
        .expect("the pick builds or refuses by name");
        if !gap.built() {
            continue;
        }

        let fill = gap_fill::fill(&gap, profile, 8).expect("the fill");
        if fill.filled() {
            let mesh = fill.mesh.as_ref().unwrap().borrow();
            assert!(
                is_closed_and_consistent(&mesh),
                "Filled{}",
                bad_edges(&mesh)
            );
        }
    }
}

/// C# `EdgesEndingFreeOnAnEndFaceBuildPickWide`: open edges ending FREE on an end face in a pick of
/// several chains - the end face is the chain's END REGION, cut and bounded as the chain alone cuts
/// it.
#[test]
fn edges_ending_free_on_an_end_face_build_pick_wide() {
    for profile in [BandProfile::Chamfer, BandProfile::Fillet] {
        let (positions, indices, feature) = cube();
        let picks = [
            (5, vec![(2, 3)]),
            (0, vec![(0, 4), (1, 5)]),
            (0, vec![(1, 5), (2, 6)]),
        ];
        for (rim, extra) in picks {
            let mut edges: EdgeSet = rim_edges(&FACE_RIMS[rim]).collect();
            edges.extend(extra);
            let (chains, closed) = link(&positions, &indices, &edges);
            let (said, _) =
                try_pick_wide(&positions, &indices, &feature, &chains, &closed, profile);
            assert_eq!(said, None, "{}", describe(&edges));
        }
    }
}

/// C# `ThreeEdgesOfACornerChamferToTheirMitrePoint`: the three edges of one cube corner, each
/// running out free onto its far end face, chamfered pick-wide - the cap is the mitre, a fan to the
/// point the three strip planes share.
#[test]
fn three_edges_of_a_corner_chamfer_to_their_mitre_point() {
    const HALF: f64 = 10.0;
    const D: f64 = 3.0;
    let (positions, indices) = cube_solid(HALF);
    let feature = feature_edges_over(&positions, &indices, FOLD_DEGREES);
    let edges: EdgeSet = [(0, 1), (0, 3), (0, 4)].into_iter().collect();
    let (chains, closed) = link(&positions, &indices, &edges);
    let gap = build(
        &positions,
        &indices,
        &feature,
        &chains,
        &closed,
        D,
        BandProfile::Chamfer,
    )
    .expect("the pick builds or refuses by name");
    assert!(gap.built(), "{:?}", gap.refusal);
    let fill = gap_fill::fill(&gap, BandProfile::Chamfer, 8).expect("the fill");
    assert_eq!(verdict(&fill), None);

    let side = 2.0 * HALF;
    let expected =
        (side * side * side) - (3.0 * (side - D) * D * D / 2.0) - ((D * D * D) - (D * D * D / 4.0));
    let volume = signed_volume(&fill.mesh.as_ref().unwrap().borrow());
    assert!(
        (volume - expected).abs() <= 1e-9 * expected,
        "{volume} against {expected}"
    );
}

/// C# `AnEdgeWhoseRailIsAllTiesLaysItsOwnStrip`: an edge whose rail on some side is nothing but
/// corner ties still lays its own strip when that side is an OPEN run, or when both its rails are
/// runs of ONE loop.
#[test]
fn an_edge_whose_rail_is_all_ties_lays_its_own_strip() {
    for profile in [BandProfile::Chamfer, BandProfile::Fillet] {
        let (positions, indices, feature) = cube();
        let mut first: EdgeSet = rim_edges(&FACE_RIMS[1]).collect();
        first.extend([(0, 4), (1, 5)]);
        let mut second: EdgeSet = rim_edges(&FACE_RIMS[2])
            .chain(rim_edges(&FACE_RIMS[4]))
            .collect();
        second.insert((4, 7));
        for edges in [first, second] {
            let (chains, closed) = link(&positions, &indices, &edges);
            let (said, _) =
                try_pick_wide(&positions, &indices, &feature, &chains, &closed, profile);
            assert_eq!(said, None, "{}", describe(&edges));
        }
    }
}

/// C# `AnEdgeWhoseBandIsALoopOfItsOwnBuildsPickWide`: rims 0-1-2-3 and 3-0-4-7 with edge 5-6, whose
/// band is a closed loop all its own - split into its two rails and zipped.
#[test]
fn an_edge_whose_band_is_a_loop_of_its_own_builds_pick_wide() {
    for profile in [BandProfile::Chamfer, BandProfile::Fillet] {
        let (positions, indices, feature) = cube();
        let mut edges: EdgeSet = rim_edges(&FACE_RIMS[0])
            .chain(rim_edges(&FACE_RIMS[5]))
            .collect();
        edges.insert((5, 6));
        let (chains, closed) = link(&positions, &indices, &edges);
        let (said, _) = try_pick_wide(&positions, &indices, &feature, &chains, &closed, profile);
        assert_eq!(said, None, "{}", describe(&edges));
    }
}

/// C# `TwoRimsJoinedByAnEdgeFilletInsideTheCube`: the corner rings must stand in or on the cube,
/// since every picked edge is convex and a bevel only removes material.
#[test]
fn two_rims_joined_by_an_edge_fillet_inside_the_cube() {
    let (positions, indices, feature) = cube();
    let edges: EdgeSet = [
        (0, 1),
        (0, 3),
        (0, 4),
        (1, 2),
        (1, 5),
        (2, 3),
        (2, 6),
        (3, 7),
        (4, 5),
    ]
    .into_iter()
    .collect();
    let (chains, closed) = link(&positions, &indices, &edges);
    let (said, _) = try_pick_wide(
        &positions,
        &indices,
        &feature,
        &chains,
        &closed,
        BandProfile::Fillet,
    );
    assert_eq!(said, None, "{}", describe(&edges));
}

/// C# `ABothSidedLoopsSideDoesNotDependOnWhereTheWalkStarted`.
#[test]
fn a_both_sided_loops_side_does_not_depend_on_where_the_walk_started() {
    let round_the_edge: Vec<i32> = std::iter::repeat_n(1, 7)
        .chain(std::iter::repeat_n(2, 7))
        .collect();
    let down_one_rail: Vec<i32> = std::iter::once(1)
        .chain(std::iter::repeat_n(2, 10))
        .chain(std::iter::once(1))
        .collect();
    for start in 0..round_the_edge.len() {
        let rotated = rotated(&round_the_edge, start);
        assert_eq!(gap_loop_walk::side_of_a_both_sided_loop(&rotated, 1), 1);
        assert_eq!(gap_loop_walk::side_of_a_both_sided_loop(&rotated, 2), 2);
    }

    for start in 0..down_one_rail.len() {
        let rotated = rotated(&down_one_rail, start);
        assert_eq!(gap_loop_walk::side_of_a_both_sided_loop(&rotated, 1), 2);
    }
}

fn rotated(list: &[i32], start: usize) -> Vec<i32> {
    list[start..]
        .iter()
        .chain(&list[..start])
        .copied()
        .collect()
}

/// C#'s `BandGap.Build(..., Size, profile, 0, 1e-9, null, new WorkBudget(...), sharpTurnDegrees:
/// FoldDegrees, filletSegments: 8)`.
fn build(
    positions: &[Vector3],
    indices: &[(i32, i32, i32)],
    feature: &FeatureEdgeTest,
    chains: &[Vec<i32>],
    closed: &[bool],
    size: f64,
    profile: BandProfile,
) -> Result<BandGapResult, String> {
    let options = BandGapBuildOptions {
        budget: Some(Rc::new(
            WorkBudget::new(WorkBudget::default_limit(indices.len() as i32))
                .expect("a positive limit"),
        )),
        sharp_turn_degrees: FOLD_DEGREES,
        fillet_segments: 8,
        ..BandGapBuildOptions::default()
    };
    band_gap_entry_points::build_pick(
        positions,
        indices,
        chains,
        closed,
        Some(Rc::clone(feature)),
        size,
        profile,
        &options,
    )
    .map_err(|e| format!("threw {e:?}"))
}

/// C# `TryPickWide`: `None` where the pick builds pick-wide closed, one shell and consistently
/// oriented; why not otherwise. C#'s caught exception is the kernel's error here.
fn try_pick_wide(
    positions: &[Vector3],
    indices: &[(i32, i32, i32)],
    feature: &FeatureEdgeTest,
    chains: &[Vec<i32>],
    closed: &[bool],
    profile: BandProfile,
) -> (Option<String>, Option<BandGapResult>) {
    let gap = match build(positions, indices, feature, chains, closed, SIZE, profile) {
        Ok(gap) => gap,
        Err(threw) => return (Some(threw), None),
    };
    if !gap.built() {
        return (Some(format!("band {:?}", gap.refusal)), Some(gap));
    }

    let said = match gap_fill::fill(&gap, profile, 8) {
        Ok(fill) => verdict(&fill).or_else(|| shape(&fill, chains, closed, profile)),
        Err(e) => Some(format!("threw {e:?}")),
    };
    (said, Some(gap))
}

fn try_in_turn(
    positions: &[Vector3],
    indices: &[(i32, i32, i32)],
    feature: &FeatureEdgeTest,
    chains: &[Vec<i32>],
    closed: &[bool],
    profile: BandProfile,
    gap: Option<&BandGapResult>,
) -> Option<String> {
    let room = gap.filter(|gap| gap.built()).map(|gap| gap.room);
    match build_open_chains_in_turn(
        positions,
        indices,
        chains,
        Some(Rc::clone(feature)),
        SIZE,
        profile,
        8,
        room,
        1e-9,
        0.0,
        FOLD_DEGREES,
        None,
        Some(closed),
    ) {
        Ok(fill) => verdict(&fill).or_else(|| shape(&fill, chains, closed, profile)),
        Err(e) => Some(format!("threw {e:?}")),
    }
}

fn verdict(fill: &GapFillResult) -> Option<String> {
    if !fill.filled() {
        return Some(format!(
            "fill {:?} (chain {})",
            fill.refusal, fill.offending_chain
        ));
    }

    let mesh = fill.mesh.as_ref().unwrap().borrow();
    if !is_closed_and_consistent(&mesh) {
        return Some(format!("not closed and consistent{}", bad_edges(&mesh)));
    }

    if shell_count(&mesh) != 1 {
        return Some("not one shell".to_string());
    }

    None
}

/// C# `Shape`: `None` where the built part has the shape a convex-edge bevel on the cube must
/// have; what is wrong otherwise. Every picked cube edge is convex, so a bevel only removes
/// material, only near its picks, and no more than its prisms: (1) every vertex lies in or on the
/// cube; (2) a vertex on no cube face plane is within r*sqrt(2) of the nearest picked edge segment;
/// (3) volume strictly decreases; (4) volume loses no more than the full PRISMS of its own section
/// (a chamfer's r^2/2, a fillet's chorded 8-row section).
fn shape(
    fill: &GapFillResult,
    chains: &[Vec<i32>],
    closed: &[bool],
    profile: BandProfile,
) -> Option<String> {
    const HALF: f64 = 1.0;
    const TOLERANCE: f64 = 1e-9;
    let (positions, _) = cube_solid(HALF);
    let mut segments = Vec::new();
    for (c, chain) in chains.iter().enumerate() {
        let count = if closed[c] {
            chain.len()
        } else {
            chain.len() - 1
        };
        for i in 0..count {
            segments.push((
                positions[chain[i] as usize],
                positions[chain[(i + 1) % chain.len()] as usize],
            ));
        }
    }

    let mesh = fill.mesh.as_ref().unwrap().borrow();
    let mut used = HashSet::new();
    for t in 0..mesh.triangle_count() {
        if !mesh.is_removed(t) {
            let tri = mesh.triangles()[t as usize];
            used.insert(tri.a);
            used.insert(tri.b);
            used.insert(tri.c);
        }
    }

    let reach = SIZE * 2f64.sqrt() * (1.0 + TOLERANCE);
    for &v in &used {
        let p = mesh.position(v);
        if p.x.abs() > HALF + TOLERANCE
            || p.y.abs() > HALF + TOLERANCE
            || p.z.abs() > HALF + TOLERANCE
        {
            return Some(format!(
                "shape: vertex {p:?} outside the cube by {:.2e}",
                p.x.abs().max(p.y.abs().max(p.z.abs())) - HALF
            ));
        }

        let on_face_plane = (p.x.abs() - HALF).abs() <= TOLERANCE
            || (p.y.abs() - HALF).abs() <= TOLERANCE
            || (p.z.abs() - HALF).abs() <= TOLERANCE;
        if !on_face_plane
            && segments
                .iter()
                .all(|&(a, b)| distance_to_segment(p, a, b) > reach)
        {
            return Some(format!(
                "shape: vertex {p:?} off every face and out of every pick's reach"
            ));
        }
    }

    let cube = 8.0 * HALF * HALF * HALF;
    let volume = signed_volume(&mesh);
    // C#'s `!(volume < cube - Tolerance)`: a NaN volume fails too.
    if volume.partial_cmp(&(cube - TOLERANCE)) != Some(std::cmp::Ordering::Less) {
        return Some(format!(
            "shape: volume {volume} does not drop below the cube's {cube}"
        ));
    }

    let section = if profile == BandProfile::Chamfer {
        SIZE * SIZE / 2.0
    } else {
        chorded_fillet_section(SIZE, 8)
    };
    let most_loss = segments.iter().map(|&(a, b)| (b - a).length()).sum::<f64>() * section;
    if cube - volume > most_loss * (1.0 + TOLERANCE) {
        return Some(format!(
            "shape: loses {}, more than the {most_loss} of its full chamfer prisms",
            cube - volume
        ));
    }

    None
}

/// C# `LevelLineTestGeometry.ChordedFilletSection`: the section a fillet of `segments` rows removes
/// from a right-angled edge - r^2 less the sum of r^2 sin(step) / 2, MORE than the true round's
/// r^2 (1 - pi / 4).
pub fn chorded_fillet_section(r: f64, segments: i32) -> f64 {
    let p = Vector3::new(r, 0.0, 0.0);
    let q = Vector3::new(0.0, r, 0.0);
    let centre = Vector3::new(r, r, 0.0);
    let mut kept = 0.0;
    for s in 0..segments {
        let a = gap_fill::arc_point(p, Vector3::ZERO, q, s as f64 / segments as f64) - centre;
        let b = gap_fill::arc_point(p, Vector3::ZERO, q, (s + 1) as f64 / segments as f64) - centre;
        kept += a.cross(b).length() / 2.0;
    }

    (r * r) - kept
}

fn distance_to_segment(p: Vector3, a: Vector3, b: Vector3) -> f64 {
    let ab = b - a;
    let t = ((p - a).dot(ab) / ab.length_squared()).clamp(0.0, 1.0);
    (p - (a + (ab * t))).length()
}

/// C# `EveryUnion`: every distinct non-empty union of at most two face rims and at most two cube
/// edges.
fn every_union() -> Vec<EdgeSet> {
    let all_edges: Vec<(i32, i32)> = FACE_RIMS
        .iter()
        .flat_map(rim_edges)
        .collect::<BTreeSet<_>>()
        .into_iter()
        .collect();
    let mut rim_choices: Vec<Vec<usize>> = vec![Vec::new()];
    for a in 0..FACE_RIMS.len() {
        rim_choices.push(vec![a]);
        for b in (a + 1)..FACE_RIMS.len() {
            rim_choices.push(vec![a, b]);
        }
    }

    let mut edge_choices: Vec<Vec<usize>> = vec![Vec::new()];
    for a in 0..all_edges.len() {
        edge_choices.push(vec![a]);
        for b in (a + 1)..all_edges.len() {
            edge_choices.push(vec![a, b]);
        }
    }

    let mut seen = HashSet::new();
    let mut unions = Vec::new();
    for rims in &rim_choices {
        for picked in &edge_choices {
            let mut union: EdgeSet = rims
                .iter()
                .flat_map(|&r| rim_edges(&FACE_RIMS[r]))
                .collect();
            union.extend(picked.iter().map(|&i| all_edges[i]));
            if !union.is_empty() && seen.insert(describe(&union)) {
                unions.push(union);
            }
        }
    }

    unions
}

/// C# `RimEdges`.
pub fn rim_edges(rim: &[i32; 4]) -> impl Iterator<Item = (i32, i32)> + '_ {
    rim.iter()
        .enumerate()
        .map(|(i, &v)| key(v, rim[(i + 1) % rim.len()]))
}

fn key(a: i32, b: i32) -> (i32, i32) {
    if a < b {
        (a, b)
    } else {
        (b, a)
    }
}

/// C# `Describe`: the edges in order.
fn describe(edges: &EdgeSet) -> String {
    edges
        .iter()
        .map(|e| format!("{}-{}", e.0, e.1))
        .collect::<Vec<_>>()
        .join(" ")
}

/// C# `Link`: the edge set as chains, as the app links a pick (BevelSurfacePartBuilder.TheChainsOf)
/// - each mesh edge once, linked by the production `BevelChainLinker`.
pub fn link(
    positions: &[Vector3],
    indices: &[(i32, i32, i32)],
    edges: &EdgeSet,
) -> (Vec<Vec<i32>>, Vec<bool>) {
    let flat: Vec<i32> = indices.iter().flat_map(|t| [t.0, t.1, t.2]).collect();
    let mesh = Mesh::from_vector3_list_and_indices(positions, &flat);
    let features = FeatureEdgeSet::create_with_angle(&mesh, FOLD_DEGREES);
    let picked: Vec<_> = features
        .edges()
        .iter()
        .filter(|e| edges.contains(&key(e.start_vertex, e.end_vertex)))
        .cloned()
        .collect();
    assert_eq!(
        picked.len(),
        edges.len(),
        "{} of {} picked edges are features",
        picked.len(),
        edges.len()
    );

    let mut chains = Vec::new();
    let mut closed = Vec::new();
    for chain in bevel_chain_linker::link(&picked) {
        let mut vertices: Vec<i32> = chain.edges.iter().map(|edge| edge.from_vertex()).collect();
        if !chain.is_closed {
            vertices.push(chain.edges[chain.edges.len() - 1].to_vertex());
        }

        chains.push(vertices);
        closed.push(chain.is_closed);
    }

    (chains, closed)
}

/// Every live triangle's directed edges with their counts, in triangle order.
fn directed_counts(mesh: &LevelLineMesh) -> Vec<((i32, i32), i32)> {
    let mut order = Vec::new();
    let mut seen: HashMap<(i32, i32), i32> = HashMap::new();
    for t in 0..mesh.triangle_count() {
        if mesh.is_removed(t) {
            continue;
        }

        let tri = mesh.triangles()[t as usize];
        for directed in [(tri.a, tri.b), (tri.b, tri.c), (tri.c, tri.a)] {
            let n = seen.entry(directed).or_insert(0);
            if *n == 0 {
                order.push(directed);
            }
            *n += 1;
        }
    }

    order.into_iter().map(|e| (e, seen[&e])).collect()
}

fn bad_edges(mesh: &LevelLineMesh) -> String {
    let counts = directed_counts(mesh);
    let seen: HashMap<(i32, i32), i32> = counts.iter().copied().collect();
    let bad: Vec<String> = counts
        .iter()
        .filter(|(e, n)| *n != 1 || seen.get(&(e.1, e.0)) != Some(&1))
        .take(6)
        .map(|(e, n)| {
            format!(
                "{e:?}x{n}@{:?}->{:?}",
                mesh.position(e.0),
                mesh.position(e.1)
            )
        })
        .collect();
    format!(" [{}]", bad.join("; "))
}

fn is_closed_and_consistent(mesh: &LevelLineMesh) -> bool {
    let counts = directed_counts(mesh);
    let seen: HashMap<(i32, i32), i32> = counts.iter().copied().collect();
    counts
        .iter()
        .all(|(e, n)| *n == 1 && seen.get(&(e.1, e.0)) == Some(&1))
}

fn shell_count(mesh: &LevelLineMesh) -> i32 {
    let mut neighbours: BTreeMap<i32, Vec<i32>> = BTreeMap::new();
    for t in 0..mesh.triangle_count() {
        if !mesh.is_removed(t) {
            neighbours.insert(t, Vec::new());
        }
    }

    for (low, high) in mesh.edges() {
        let standing = mesh.triangles_on(low, high).to_vec();
        for i in 0..standing.len() {
            for j in (i + 1)..standing.len() {
                neighbours
                    .get_mut(&standing[i])
                    .expect("a live triangle")
                    .push(standing[j]);
                neighbours
                    .get_mut(&standing[j])
                    .expect("a live triangle")
                    .push(standing[i]);
            }
        }
    }

    let mut visited = HashSet::new();
    let mut shells = 0;
    for &start in neighbours.keys() {
        if !visited.insert(start) {
            continue;
        }

        shells += 1;
        let mut stack = vec![start];
        while let Some(at) = stack.pop() {
            for &next in &neighbours[&at] {
                if visited.insert(next) {
                    stack.push(next);
                }
            }
        }
    }

    shells
}
