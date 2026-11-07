//! A pick is a LIST of chains - the list of one is the single-chain call to the bit, every chain is
//! read on the input mesh before anything is cut, two chains that touch refuse by name, and a pick
//! of several valid chains is cut in one pass, its room read across the chains. The fixtures are
//! `several_chains_fixtures.rs`. Each parameter row is one pass of a loop in the test of the same
//! name.

use std::collections::HashMap;
use std::rc::Rc;

use geometry_processing::bevel::level_line::band_gap_entry_points::{self, BandGapBuildOptions};
use geometry_processing::bevel::level_line::band_gap_reading;
use geometry_processing::bevel::level_line::pick_validation;
use geometry_processing::bevel::level_line::{
    BandGapRefusal, BandProfile, GapLoop, LevelLineMesh, LevelLineWork, RoomReadingKind,
    RoomSource, WorkBudget,
};
use vector_math::Vector3;

use crate::level_line_room_geometry::box_with_square_hole;
use crate::level_line_test_geometry::{
    cube_solid, open_edges, signed_volume_of, volume_of_every_triangle_even_removed,
};
use crate::several_chains_fixtures::{motion, named};
use crate::several_chains_support::{
    build, build_with, feature, never, pick_named, point_set, read, read_room, room_margin,
    room_of, shared_groups, to_chain,
};

/// How far a needle repair may move this fixture's volume - the one thing in
/// the build that is not a split.
const NEEDLE_MOVEMENT: f64 = 1e-3;

#[test]
fn the_list_of_one_answers_bit_for_bit_what_the_chain_answers() {
    for pick in ["cube top rim", "cylinder rim"] {
        let (positions, indices, chain) = pick_named(pick);

        let one = band_gap_entry_points::build(
            &positions,
            &indices,
            &chain,
            true,
            feature(&positions, &indices),
            0.2,
            BandProfile::Chamfer,
            &BandGapBuildOptions::default(),
        )
        .expect("the chain builds");
        let as_list = build(
            &positions,
            &indices,
            std::slice::from_ref(&chain),
            &[true],
            feature(&positions, &indices),
            0.2,
        );

        assert_eq!(as_list.refusal, one.refusal);
        assert_eq!(as_list.room, one.room);
        let one_mesh = one.mesh.as_ref().expect("a mesh").borrow();
        let list_mesh = as_list.mesh.as_ref().expect("a mesh").borrow();
        assert_eq!(list_mesh.vertices.len(), one_mesh.vertices.len());
        assert_eq!(list_mesh.triangles.len(), one_mesh.triangles.len());
        assert_eq!(as_list.removed_triangles, one.removed_triangles);
        assert_eq!(as_list.loops.len(), one.loops.len());
        for v in 0..one_mesh.vertices.len() {
            assert_eq!(
                list_mesh.vertices[v], one_mesh.vertices[v],
                "the list of one takes exactly the path the single chain took"
            );
        }

        for l in 0..one.loops.len() {
            assert_eq!(as_list.loops[l].vertices, one.loops[l].vertices);
            assert_eq!(as_list.loops[l].parameters, one.loops[l].parameters);
        }

        // And the pick's chains are the list of the one chain's field.
        assert_eq!(as_list.chains.len(), 1);
        assert!(Rc::ptr_eq(
            &as_list.chains[0],
            &as_list.chain().expect("a chain")
        ));
    }
}

#[test]
fn the_list_of_one_reads_the_same_room() {
    let (positions, indices, rim) = box_with_square_hole(2.0, 0.5, 1.0);

    let one = band_gap_reading::room_of(
        &positions,
        &indices,
        &rim,
        true,
        feature(&positions, &indices),
        BandProfile::Chamfer,
        1e-9,
    )
    .expect("the chain reads");
    let as_list = band_gap_reading::room_of_pick(
        &positions,
        &indices,
        std::slice::from_ref(&rim),
        &[true],
        feature(&positions, &indices),
        BandProfile::Chamfer,
        1e-9,
    )
    .expect("the pick reads");

    assert_eq!(as_list, one);
}

#[test]
fn the_fixtures_are_closed_consistently_wound_solids() {
    for fixture in ["A", "B", "C"] {
        let (positions, indices, _) = named(fixture);

        let mut directed: HashMap<(i32, i32), i32> = HashMap::new();
        for &(a, b, c) in &indices {
            for step in [(a, b), (b, c), (c, a)] {
                *directed.entry(step).or_insert(0) += 1;
            }
        }

        for (step, count) in &directed {
            assert_eq!(
                *count, 1,
                "a consistently wound solid traverses {step:?} exactly once"
            );
            assert!(
                directed.contains_key(&(step.1, step.0)),
                "the edge {step:?} must stand on a second triangle, wound the other way"
            );
        }

        assert!(
            signed_volume_of(&positions, &indices) > 0.0,
            "the fixture is wound outward"
        );
    }
}

#[test]
fn every_fixture_chain_validates_on_its_own_mesh() {
    for fixture in ["A", "B", "C"] {
        let (positions, indices, chains) = named(fixture);

        for chain in &chains {
            let reading = read_room(
                &positions,
                &indices,
                std::slice::from_ref(chain),
                &[true],
                feature(&positions, &indices),
                None,
            );
            assert!(reading.chain_edge_stands_on_two_triangles);
            assert!(reading.read);
        }
    }
}

/// The top rim and the side edge 4-0 meet at a CORNER,
/// so the pick is not refused as chains that touch; with `BandSideTagging.BothSidedMembersBuild`
/// off (carried on the build's budget here) it refuses by its old name.
#[test]
fn chains_that_share_a_vertex_at_a_corner_build() {
    let (positions, indices) = cube_solid(1.0);
    let chains = vec![vec![4, 5, 6, 7], vec![4, 0]];

    let corners = pick_validation::corners_of(&chains, &[true, false], false);
    assert_eq!(corners.len(), 1);
    assert_eq!(corners[0].vertex, 4);

    let built = build(
        &positions,
        &indices,
        &chains,
        &[true, false],
        feature(&positions, &indices),
        0.1,
    );
    assert_eq!(built.refusal, BandGapRefusal::None);

    let budget = Rc::new(
        WorkBudget::new(WorkBudget::default_limit(indices.len() as i32)).expect("a positive limit"),
    );
    let _work = LevelLineWork::begin(Some(Rc::clone(&budget)));
    budget.both_sided_members_off.set(true);
    let refused = build_with(
        &positions,
        &indices,
        &chains,
        &[true, false],
        feature(&positions, &indices),
        0.1,
        &BandGapBuildOptions {
            budget: Some(Rc::clone(&budget)),
            ..BandGapBuildOptions::default()
        },
    );
    assert_eq!(refused.refusal, BandGapRefusal::ChainDoesNotDivideItsSides);
    assert_eq!(refused.room, 0.0);

    let reading = read_room(
        &positions,
        &indices,
        &chains,
        &[true, false],
        feature(&positions, &indices),
        Some(budget),
    );
    assert!(reading.chains_are_apart);
    assert!(!reading.chain_divides_sides);
}

#[test]
fn chains_meeting_end_to_end_refuse_by_name_and_name_both() {
    let (positions, indices) = cube_solid(1.0);
    let chains = vec![vec![4, 5, 6], vec![6, 7]];

    assert_eq!(
        pick_validation::corners_of(&chains, &[false, false], false).len(),
        0
    );

    let built = build(
        &positions,
        &indices,
        &chains,
        &[false, false],
        feature(&positions, &indices),
        0.1,
    );

    assert_eq!(built.refusal, BandGapRefusal::ChainsTouch);
    assert_eq!(built.offending_chain, 0);
    assert_eq!(built.other_chain, 1);
    assert_eq!(built.offending_vertex, 6);
    assert_eq!(built.room, 0.0);

    let reading = read_room(
        &positions,
        &indices,
        &chains,
        &[false, false],
        feature(&positions, &indices),
        None,
    );
    assert!(!reading.chains_are_apart);
    assert_eq!(reading.offending_vertex, 6);
}

#[test]
fn a_chain_edge_that_is_not_in_the_mesh_names_its_chain() {
    let (positions, indices) = cube_solid(1.0);

    // 1-3 is a diagonal of the bottom face, not an edge of it, and shares no vertex with the top rim.
    let chains = vec![vec![4, 5, 6, 7], vec![1, 3]];

    let built = build(
        &positions,
        &indices,
        &chains,
        &[true, false],
        feature(&positions, &indices),
        0.1,
    );

    assert_eq!(built.refusal, BandGapRefusal::ChainEdgeNotOnTwoTriangles);
    assert_eq!(
        built.offending_chain, 1,
        "the edge check runs chain by chain in the order given, before the chains are compared"
    );
    assert_eq!(built.offending_edge, (1, 3));
    assert_eq!(built.room, 0.0);
}

#[test]
fn a_single_chains_bad_edge_names_chain_zero() {
    let (positions, indices) = cube_solid(1.0);

    let built = band_gap_entry_points::build(
        &positions,
        &indices,
        &[1, 3],
        false,
        never(),
        0.1,
        BandProfile::Chamfer,
        &BandGapBuildOptions::default(),
    )
    .expect("the chain refuses by name");

    assert_eq!(built.refusal, BandGapRefusal::ChainEdgeNotOnTwoTriangles);
    assert_eq!(built.offending_chain, 0);
}

/// A pick of several chains is CUT, in one pass.
/// [`NEEDLE_MOVEMENT`] bounds the one volume change that is not a split.
#[test]
fn every_picked_side_leaves_its_own_one_closed_loop() {
    for (fixture, size) in [("A", 0.3), ("B", 0.5)] {
        let (positions, indices, chains) = named(fixture);
        let closed: Vec<bool> = chains.iter().map(|_| true).collect();

        let built = build(
            &positions,
            &indices,
            &chains,
            &closed,
            feature(&positions, &indices),
            size,
        );

        assert_eq!(built.refusal, BandGapRefusal::None);
        assert_eq!(built.loops.len(), 4);
        assert!(built.loops.iter().all(|l| l.is_closed));
        assert!(built.loops.iter().all(|l| !l.has_mixed_owners));
        for k in [0, 1] {
            let mine: Vec<&GapLoop> = built.loops.iter().filter(|l| l.chain == k).collect();
            assert_eq!(mine.len(), 2);
            let mut sides: Vec<i32> = mine.iter().map(|l| l.side).collect();
            sides.sort();
            assert_eq!(sides, vec![1, 2]);
        }

        let mesh = built.mesh.as_ref().expect("a mesh").borrow();

        // Every loop vertex stands at its OWN chain's setback from that chain - a chamfer's setback
        // is the size itself - within the chord tolerance the cut was allowed.
        for gap_loop in &built.loops {
            for &v in &gap_loop.vertices {
                let off = to_chain(
                    mesh.position(v),
                    &positions,
                    &chains[gap_loop.chain as usize],
                );
                assert!((off - size).abs() < size / 10.0);
            }
        }

        // The cut only SPLIT the solid - EXCEPT that a needle is re-tiled through its apex - so the
        // kept mesh and the band together are still what went in, to within the movement one repair
        // can charge.
        let went_in = volume_of_every_triangle_even_removed(
            &LevelLineMesh::from_triangles(&positions, &indices, Some(&vec![0; indices.len()]))
                .expect("the input is a mesh"),
        );
        let came = volume_of_every_triangle_even_removed(&mesh);
        println!(
            "fixture {fixture} at {size}: volume {came:?} against {went_in:?}, moved {:.4e}",
            (came - went_in).abs()
        );
        assert!((came - went_in).abs() <= NEEDLE_MOVEMENT);

        // Nothing is open but the four loops.
        let open = open_edges(&mesh);
        assert_eq!(
            open.len(),
            built.loops.iter().map(|l| l.vertices.len()).sum::<usize>()
        );
    }
}

#[test]
fn the_sides_that_share_a_face_are_one_group() {
    let (positions, indices, chains) = named("A");

    let built = build(
        &positions,
        &indices,
        &chains,
        &[true, true],
        feature(&positions, &indices),
        0.3,
    );

    assert_eq!(built.refusal, BandGapRefusal::None);
    let mesh = built.mesh.as_ref().expect("a mesh").borrow();

    // A group is ONE tag, so the loops the shared face keeps carry the same tag under them.
    let shared = shared_groups(&mesh, &built.loops);
    assert_eq!(shared.len(), 1);
    let top = &shared[0];
    assert_eq!(top.len(), 2);
    let mut owners: Vec<i32> = top.iter().map(|l| l.chain).collect();
    owners.sort();
    assert_eq!(owners, vec![0, 1]);

    // And that face is the plate's TOP: every vertex of both loops stands at the highest Z.
    let highest = positions
        .iter()
        .map(|v| v.z)
        .fold(f64::NEG_INFINITY, f64::max);
    assert!(top.iter().all(|l| l
        .vertices
        .iter()
        .all(|&v| (mesh.position(v).z - highest).abs() < 1e-9)));
}

#[test]
fn two_bosses_share_the_plates_face_and_travel_with_the_part() {
    let (positions, indices, chains) = named("B");
    let closed = [true, true];

    let still = build(
        &positions,
        &indices,
        &chains,
        &closed,
        feature(&positions, &indices),
        0.5,
    );
    assert_eq!(still.refusal, BandGapRefusal::None);
    let still_mesh = still.mesh.as_ref().expect("a mesh").borrow();

    let shared = shared_groups(&still_mesh, &still.loops);
    assert_eq!(shared.len(), 1);
    let mut owners: Vec<i32> = shared[0].iter().map(|l| l.chain).collect();
    owners.sort();
    assert_eq!(owners, vec![0, 1]);

    let moved: Vec<Vector3> = positions.iter().map(|&v| v.transform(motion())).collect();
    let travelled = build(
        &moved,
        &indices,
        &chains,
        &closed,
        feature(&moved, &indices),
        0.5,
    );
    let travelled_mesh = travelled.mesh.as_ref().expect("a mesh").borrow();

    assert_eq!(travelled.refusal, still.refusal);
    assert_eq!(travelled.loops.len(), still.loops.len());
    assert_eq!(travelled_mesh.vertices.len(), still_mesh.vertices.len());
    assert_eq!(travelled_mesh.triangles.len(), still_mesh.triangles.len());
    for i in 0..still.loops.len() {
        assert_eq!(travelled.loops[i].chain, still.loops[i].chain);
        assert_eq!(travelled.loops[i].side, still.loops[i].side);
        assert_eq!(
            travelled.loops[i].vertices.len(),
            still.loops[i].vertices.len()
        );
        for j in 0..still.loops[i].vertices.len() {
            let here = still_mesh
                .position(still.loops[i].vertices[j])
                .transform(motion());
            let there = travelled_mesh.position(travelled.loops[i].vertices[j]);
            assert!((here - there).length() < 1e-9);
        }
    }
}

#[test]
fn permuting_the_chains_permutes_nothing_but_their_indices() {
    let (positions, indices, chains) = named("A");

    let as_picked = build(
        &positions,
        &indices,
        &chains,
        &[true, true],
        feature(&positions, &indices),
        0.3,
    );
    let swapped = build(
        &positions,
        &indices,
        &[chains[1].clone(), chains[0].clone()],
        &[true, true],
        feature(&positions, &indices),
        0.3,
    );

    assert_eq!(swapped.refusal, as_picked.refusal);
    assert_eq!(swapped.loops.len(), as_picked.loops.len());
    assert_eq!(point_set(&swapped), point_set(&as_picked));
}

#[test]
fn the_refusals_are_the_same_wherever_the_part_stands() {
    let (positions, indices, chains) = named("A");
    let moved: Vec<Vector3> = positions.iter().map(|&p| p.transform(motion())).collect();
    let closed = [true, true];

    let still = build(
        &positions,
        &indices,
        &chains,
        &closed,
        feature(&positions, &indices),
        0.2,
    );
    let travelled = build(
        &moved,
        &indices,
        &chains,
        &closed,
        feature(&moved, &indices),
        0.2,
    );
    assert_eq!(travelled.refusal, still.refusal);

    let touching = vec![chains[0].clone(), vec![chains[0][0], chains[0][1]]];
    let still_touch = build(
        &positions,
        &indices,
        &touching,
        &[true, false],
        never(),
        0.2,
    );
    let moved_touch = build(&moved, &indices, &touching, &[true, false], never(), 0.2);
    assert_eq!(still_touch.refusal, BandGapRefusal::ChainsTouch);
    assert_eq!(moved_touch.refusal, still_touch.refusal);
    assert_eq!(moved_touch.offending_vertex, still_touch.offending_vertex);
    assert_eq!(moved_touch.offending_chain, still_touch.offending_chain);
    assert_eq!(moved_touch.other_chain, still_touch.other_chain);
}

/// Fixture C's two rims close on EACH OTHER.
#[test]
fn two_rims_close_on_each_other_and_that_sets_the_room() {
    let (positions, indices, chains) = named("C");
    let reading = read(&positions, &indices, &chains);

    assert!(reading.read);
    assert!((reading.room - ((0.3 / 2.0) - room_margin())).abs() <= 1e-9);
    assert_eq!(reading.room_set_by, RoomSource::chain_to_chain(0, 1));
    let least = reading
        .candidates
        .iter()
        .map(|c| c.room)
        .fold(f64::INFINITY, f64::min);
    assert_eq!(least, reading.room);
}

#[test]
fn the_room_as_offered_builds_and_a_hair_above_the_room_refuses_by_size() {
    let (positions, indices, chains) = named("C");
    let closed = [true, true];
    let room = room_of(&positions, &indices, &chains, &closed);

    let at = build(
        &positions,
        &indices,
        &chains,
        &closed,
        feature(&positions, &indices),
        room * 0.99,
    );
    assert_eq!(at.refusal, BandGapRefusal::None);
    assert_eq!(at.room, room);
    assert_eq!(at.loops.len(), 4);
    for gap_loop in &at.loops {
        assert!(gap_loop.is_closed);
        assert!(!gap_loop.has_mixed_owners);
    }

    let mut owners: Vec<i32> = at.loops.iter().map(|l| l.chain).collect();
    owners.sort();
    owners.dedup();
    assert_eq!(owners.len(), 2);

    let above = build(
        &positions,
        &indices,
        &chains,
        &closed,
        feature(&positions, &indices),
        room * (1.0 + 1e-6),
    );
    assert_eq!(above.refusal, BandGapRefusal::Size);
    assert_eq!(above.room, room);
    assert_eq!(above.room_set_by, RoomSource::chain_to_chain(0, 1));
}

#[test]
fn the_reading_and_the_build_name_the_same_room() {
    for fixture in ["A", "B", "C"] {
        let (positions, indices, chains) = named(fixture);
        let closed = [true, true];

        let room = room_of(&positions, &indices, &chains, &closed);
        let built = build(
            &positions,
            &indices,
            &chains,
            &closed,
            feature(&positions, &indices),
            room,
        );

        if built.pinch_probes > 0 {
            // The ladder names a size BELOW the reading, and that size must itself build.
            assert!(built.room < room);
            let at_the_ladder = build(
                &positions,
                &indices,
                &chains,
                &closed,
                feature(&positions, &indices),
                built.room,
            );
            assert_eq!(
                at_the_ladder.refusal,
                BandGapRefusal::None,
                "the ladder names a size it built at, so asking exactly that size builds"
            );
            continue;
        }

        assert_eq!(built.room, room);
    }
}

#[test]
fn the_room_does_not_depend_on_the_order_of_the_pick() {
    for fixture in ["A", "B", "C"] {
        let (positions, indices, chains) = named(fixture);
        let closed = [true, true];

        let room = room_of(&positions, &indices, &chains, &closed);
        let swapped = room_of(
            &positions,
            &indices,
            &[chains[1].clone(), chains[0].clone()],
            &closed,
        );
        let mut walked_back = chains[0].clone();
        walked_back.reverse();
        let backwards = vec![walked_back, chains[1].clone()];
        let reversed = room_of(&positions, &indices, &backwards, &closed);

        assert_eq!(swapped, room);
        assert_eq!(reversed, room);
    }
}

#[test]
fn the_room_is_the_same_wherever_the_part_stands() {
    for fixture in ["A", "B", "C"] {
        let (positions, indices, chains) = named(fixture);
        let moved: Vec<Vector3> = positions.iter().map(|&p| p.transform(motion())).collect();

        let still = read(&positions, &indices, &chains);
        let travelled = read(&moved, &indices, &chains);

        assert!((travelled.room - still.room).abs() <= 1e-9);
        assert_eq!(travelled.room_set_by, still.room_set_by);
    }
}

#[test]
fn two_bosses_are_bounded_by_the_gap_between_them() {
    let (positions, indices, chains) = named("B");
    let reading = read(&positions, &indices, &chains);

    assert!((reading.room - ((3.0 / 2.0) - room_margin())).abs() <= 1e-9);
    assert_eq!(reading.room_set_by.kind, RoomReadingKind::ChainToChain);
}

#[test]
fn the_strip_between_the_rims_sets_fixture_as_room() {
    let (positions, indices, chains) = named("A");
    let reading = read(&positions, &indices, &chains);

    assert!((reading.room - ((1.0 / 2.0) - room_margin())).abs() <= 1e-9);
    assert_eq!(reading.room_set_by, RoomSource::chain_to_chain(0, 1));

    let foreign: Vec<_> = reading
        .candidates
        .iter()
        .filter(|c| c.source.kind == RoomReadingKind::Foreign)
        .collect();
    assert_eq!(foreign.len(), 2);
    for candidate in foreign {
        assert!((candidate.room - (2.0 - room_margin())).abs() <= 1e-9);
    }
}

#[test]
fn a_chains_foreign_room_is_the_same_picked_alone_or_together() {
    for chain in [0, 1] {
        let (positions, indices, chains) = named("B");

        let alone = band_gap_reading::room_of(
            &positions,
            &indices,
            &chains[chain as usize],
            true,
            feature(&positions, &indices),
            BandProfile::Chamfer,
            1e-9,
        )
        .expect("the chain reads");
        let reading = read(&positions, &indices, &chains);
        let together: Vec<_> = reading
            .candidates
            .iter()
            .filter(|c| c.source == RoomSource::foreign(chain))
            .collect();
        assert_eq!(together.len(), 1, "exactly one");

        assert_eq!(together[0].room, alone);
    }
}
