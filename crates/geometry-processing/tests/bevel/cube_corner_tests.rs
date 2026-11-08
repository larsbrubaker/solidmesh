//! The first corner case of the level line - two adjacent cube faces picked as the app links them
//! (the shared top-front edge 4-5, the rest of the top rim 5-6-7-4 and the rest of the front rim
//! 5-1-0-4), three open chains meeting at two corners. Read as corners, not refused as chains that
//! touch; the pair room is not read round the corner; the cut leaves each side group one loop whose
//! owner changes only at the corners.

use std::rc::Rc;

use solidmesh::bevel::level_line::band_gap_entry_points::{self, BandGapBuildOptions};
use solidmesh::bevel::level_line::band_gap_reading::read_room_of_pick;
use solidmesh::bevel::level_line::band_side_groups::group_sides;
use solidmesh::bevel::level_line::{
    chain_on_the_mesh, gap_loop_walk, pick_validation, BandProfile, RoomReadingKind,
};

use crate::hole_fill_corner_tests::two_faces;
use crate::level_line_feature_edges::feature_edges_over;
use crate::level_line_test_geometry::cube_solid;
use crate::several_chains_fixtures::FOLD_DEGREES;

const ALL_OPEN: [bool; 3] = [false, false, false];

/// Two adjacent faces are two corners with room and one loop per group.
#[test]
fn two_adjacent_faces_are_two_corners_with_room_and_one_loop_per_group() {
    // Half 1, so the edge is 2 and a tenth of it is 0.2.
    let (positions, indices) = cube_solid(1.0);
    let feature = feature_edges_over(&positions, &indices, FOLD_DEGREES);

    let corners = pick_validation::corners_of(&two_faces(), &ALL_OPEN, false);
    let mut vertices: Vec<i32> = corners.iter().map(|c| c.vertex).collect();
    vertices.sort();
    assert_eq!(vertices, vec![4, 5]);
    for corner in &corners {
        let mut chains = corner.chains.clone();
        chains.sort();
        assert_eq!(chains, vec![0, 1, 2]);
    }

    let reading = read_room_of_pick(
        &positions,
        &indices,
        &two_faces(),
        &ALL_OPEN,
        Some(Rc::clone(&feature)),
        BandProfile::Chamfer,
        1e-9,
        None,
        100.0,
        0.0,
        f64::INFINITY,
    )
    .expect("a valid pick");
    assert!(reading.chains_are_apart);
    assert!(reading.room >= 0.2);

    // The pair room is read only where two chains approach AWAY from the corners they share: across
    // the top face, 4-5 against the far edge 6-7, 2 apart, which two chamfers close from both sides
    // at 1. Nothing reads the corner itself as a collision.
    assert!((reading.room - 1.0).abs() <= 1e-6, "room {}", reading.room);
    let pair_rows: Vec<_> = reading
        .candidates
        .iter()
        .filter(|c| c.source.kind == RoomReadingKind::ChainToChain)
        .collect();
    assert!(!pair_rows.is_empty());
    let least = pair_rows
        .iter()
        .map(|c| c.room)
        .fold(f64::INFINITY, f64::min);
    assert!(least >= reading.room);

    let options = BandGapBuildOptions {
        known_room: Some(reading.room),
        ..BandGapBuildOptions::default()
    };
    let built = band_gap_entry_points::build_pick(
        &positions,
        &indices,
        &two_faces(),
        &ALL_OPEN,
        Some(Rc::clone(&feature)),
        0.2,
        BandProfile::Chamfer,
        &options,
    )
    .expect("the pick builds or refuses by name");
    assert_eq!(built.loops.len(), 3);

    let chain_edges: Vec<Vec<(i32, i32)>> = two_faces()
        .iter()
        .enumerate()
        .map(|(k, c)| chain_on_the_mesh::chain_edges(c, ALL_OPEN[k]))
        .collect();
    let (_, groups, _) = group_sides(&indices, &chain_edges, &*feature, true, None, None, None)
        .expect("the sides group");
    assert!(gap_loop_walk::every_chain_side_leaves_one_owned_loop(
        &groups,
        &ALL_OPEN,
        &built.loops,
        Some(&two_faces()),
    ));
}

/// The kernel's own validation takes the two faces as a good
/// pick with their two corners rather than naming a fault the route would fall through on.
#[test]
fn a_corner_pick_routes_to_the_level_line() {
    let (positions, indices) = cube_solid(1.0);
    let feature = feature_edges_over(&positions, &indices, FOLD_DEGREES);

    let (fault, corners) = pick_validation::validate_on_the_input(
        &positions,
        &indices,
        &two_faces(),
        &ALL_OPEN,
        Some(&*feature),
    );

    assert!(fault.is_none());
    assert_eq!(corners.len(), 2);
}
