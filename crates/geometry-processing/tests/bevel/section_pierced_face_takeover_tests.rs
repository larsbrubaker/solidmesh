//! One part of the pierced face tests: THE RIM TAKES OVER AT THE CREASE - what the smooth side's
//! boundary does where a junction's wall end was CARRIED onto a facet that stands past a feature
//! crease from the station beside it. The class's helpers are
//! [`crate::section_pierced_face_readings`].

use std::collections::{BTreeSet, HashSet};
use std::fmt::Write;

use geometry_processing::bevel::sections::section_smooth_side::{
    self, SmoothSideCut, SmoothSidePlace,
};
use geometry_processing::bevel::sections::{
    section_fan_walk, section_surface_walk_join, SectionPool,
};
use geometry_processing::topology::FeatureEdgeSet;
use polygon_mesh::MeshEdgeGraph;

use crate::inner_face_test_mesh;
use crate::section_pierced_face_junction_run_tests::the_run;
use crate::section_pierced_face_readings::RADIUS;

/// THE JOINS EITHER SIDE OF BOTH JUNCTIONS LAY. Each junction's group is read out of the cut and
/// every join that touches it is tried here, so a refusal is named at the JOIN it stands on and not
/// only at the one point the cut reports.
///
/// MEASURED at the operation's own size, 2 * Radius. Each junction carries ONE
/// wall end onto a facet past the bore's mouth - station 1's P2 onto face 37, station 41's P1 onto
/// face 70 - and the boundary takes that crease over at a minted point 0.1159 from the carried end.
/// The groups are then `[P1, F1, P2, C]` where the crease is crossed LEAVING and `[C, P1, F1, P2]`
/// where it is crossed ARRIVING, which is the whole reason the takeover is the junction's point and
/// not its neighbour's: the ordinary joins either side keep their own stations' readings.
#[test]
fn the_boundary_joins_round_both_junctions() {
    let part = inner_face_test_mesh::the_part();
    let graph = part.get_mesh_edge_graph();
    let features = FeatureEdgeSet::create_with_graph(
        &part,
        &graph,
        inner_face_test_mesh::BEND_THRESHOLD_DEGREES,
    );
    let edge_at_pair = section_fan_walk::edges_by_vertex_pair(&graph);
    let feature_edges: HashSet<i32> = features
        .edges()
        .iter()
        .map(|feature| feature.mesh_edge_index)
        .collect();
    let is_a_feature_edge = |edge: i32| feature_edges.contains(&edge);
    let (run, mut pool) = the_run(&part, 2.0 * RADIUS, inner_face_test_mesh::SEGMENTS);

    let smooth_side_is_left = !run
        .junctions
        .values()
        .next()
        .expect("two junctions")
        .flat_on_the_left;

    let (cut, _) = section_smooth_side::try_cut_the_smooth_side(
        &part,
        &graph,
        &edge_at_pair,
        &mut pool,
        &run,
        smooth_side_is_left,
        Some(&is_a_feature_edge),
    );

    let mut said = String::new();
    let mut around_a_junction: BTreeSet<usize> = BTreeSet::new();
    let boundary_count = cut.station_boundary.len() as i32;

    for (station, junction) in &run.junctions {
        let group = &cut.station_points[*station as usize];

        let _ = writeln!(
            said,
            "junction {station}: [{}]",
            group
                .iter()
                .map(|&point| format!("{point} {}", where_(&cut, &pool, point)))
                .collect::<Vec<_>>()
                .join(", ")
        );

        // The carried end is the FAR one, so exactly one of the two is carried and the group gains
        // exactly one takeover point beside the rim's own three.
        assert_eq!(group.len(), 4, "{said}");

        let arriving_carried = junction.carried_onto[junction.arriving_wall_end as usize] >= 0;
        let carried = if arriving_carried {
            junction.samples[junction.arriving_wall_end as usize]
        } else {
            junction.samples[junction.leaving_wall_end as usize]
        };

        // The takeover stands at the group's own end, on the side the carry crossed the crease
        // from: FIRST where the arriving end was carried, LAST where the leaving one was.
        let takeover = if arriving_carried {
            group[0]
        } else {
            group[group.len() - 1]
        };

        assert!(
            cut.places[&takeover].edge >= 0,
            "the takeover is placed ON the crease it was minted on - {said}"
        );

        assert!(
            feature_edges.contains(&cut.places[&takeover].edge),
            "and that crease is a FEATURE edge - {said}"
        );

        assert!(
            ((pool.position_of(takeover) - pool.position_of(carried)).length() - 0.1159).abs()
                <= 1e-3,
            "it stands where the carried end's own wall meets the bore - {said}"
        );

        // -1 where the group's first point is not on the boundary at all.
        let first = cut
            .station_boundary
            .iter()
            .position(|&point| point == group[0])
            .map_or(-1, |at| at as i32);

        for join in (first - 1)..(first + group.len() as i32) {
            around_a_junction.insert(((join + boundary_count) % boundary_count) as usize);
        }
    }

    let mut refused = 0;

    for &i in &around_a_junction {
        let next = (i + 1) % cut.station_boundary.len();
        let from = cut.station_boundary[i];
        let to = cut.station_boundary[next];
        let mut landed = false;

        for from_face in the_faces_of_the_place(&graph, &cut.places[&from]) {
            for &to_face in &the_faces_of_the_place(&graph, &cut.places[&to]) {
                if section_surface_walk_join::try_join(
                    &part,
                    &graph,
                    &edge_at_pair,
                    pool.position_of(from),
                    from_face,
                    pool.position_of(to),
                    to_face,
                    Some(&is_a_feature_edge),
                )
                .is_ok()
                {
                    landed = true;
                }
            }
        }

        if !landed {
            refused += 1;
            let _ = writeln!(
                said,
                "join {i} -> {next}: {from} {} to {to} {} REFUSED",
                where_(&cut, &pool, from),
                where_(&cut, &pool, to)
            );
        }
    }

    assert_eq!(refused, 0, "{said}");
}

/// A boundary point's position and the place it was written with, for a reading's
/// message.
fn where_(cut: &SmoothSideCut, pool: &SectionPool, point: i32) -> String {
    let place = &cut.places[&point];

    format!(
        "at {:?} face {} edge {}",
        pool.position_of(point),
        place.face,
        place.edge
    )
}

/// The faces a place may be read in - one for a face place, both of an
/// edge's for an edge.
fn the_faces_of_the_place(graph: &MeshEdgeGraph, place: &SmoothSidePlace) -> Vec<i32> {
    if place.face >= 0 {
        return vec![place.face];
    }

    (0..graph.get_face_count(place.edge))
        .map(|i| graph.get_face(place.edge, i))
        .collect()
}
