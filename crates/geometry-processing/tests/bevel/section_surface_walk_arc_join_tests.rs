//! EVERY JOIN OF THE OPEN MOUTH ARC'S BORE-WALL
//! BOUNDARY LANDS, and lands the same wherever the part stands ([`crate::rigid_motion_meshes`]). The
//! joins are the straight surface lines between one station's wall-side landing and the next, and
//! each is laid ONE WAY, from the earlier station to the later - there is no second attempt
//! backwards. Each parameter row is one pass of a loop in the test of the same name.
//!
//! This is the rung `SectionSurfaceWalk.TryJoin`'s converging strip was written for: the strip is
//! re-read from the faces the walk actually crossed until the line and its own unfolding agree,
//! which no guess enters.

use std::collections::HashSet;

use solidmesh_math::Vector3;
use solidmesh_mesh::{Mesh, MeshEdgeGraph};

use solidmesh::bevel::sections::section_builder_run::try_build_run;
use solidmesh::bevel::sections::section_fan_walk::edges_by_vertex_pair;
use solidmesh::bevel::sections::section_smooth_side::{try_cut_the_smooth_side, SmoothSidePlace};
use solidmesh::bevel::sections::section_surface_walk::{SurfaceWalkCrossing, SurfaceWalkRefusal};
use solidmesh::bevel::sections::section_surface_walk_join::{try_join, try_join_counting};
use solidmesh::bevel::sections::SectionPool;
use solidmesh::topology::FeatureEdgeSet;

use crate::inner_face_test_mesh;
use crate::mesh_surface_distance;
use crate::open_mouth_arc_walk;
use crate::rigid_motion_meshes;

/// The bore's tessellated wall is the arc's left side; see SectionOpenMouthArcCutTests.
const THE_BORE_WALL_IS_THE_LEFT: bool = true;

/// Every interior join of the arc's bore-wall boundary lands, under every rigid motion, and every
/// point it mints stands on the part.
#[test]
fn every_join_of_the_arc_lands_wherever_the_part_stands() {
    for radius in [2.0, 0.5] {
        let mut said = String::new();
        let mut refused = 0;
        let mut worst_off_the_wall = 0.0f64;
        let mut worst_unfoldings = 0;
        let mut not_built = String::new();
        let mut read = 0;

        // The pick, read once where the fixture was made: a rigid motion moves no index.
        let walk = open_mouth_arc_walk::the_top_mouth_arc_walk();

        for (name, motion) in rigid_motion_meshes::the_motions() {
            let mut part = inner_face_test_mesh::the_part();

            rigid_motion_meshes::move_mesh(&mut part, motion);

            let graph = part.get_mesh_edge_graph();
            let features = FeatureEdgeSet::create_with_graph(
                &part,
                &graph,
                inner_face_test_mesh::BEND_THRESHOLD_DEGREES,
            );
            let edge_at_pair = edges_by_vertex_pair(&graph);
            let feature_edges: HashSet<i32> = features
                .edges()
                .iter()
                .map(|feature| feature.mesh_edge_index)
                .collect();

            let arc = open_mouth_arc_walk::the_same_arc_on(&features, &walk);
            let mut pool = SectionPool::new(&part.vertices);

            let run = match try_build_run(
                &arc,
                false,
                &part,
                &graph,
                &mut pool,
                radius,
                inner_face_test_mesh::SEGMENTS,
                true,
                Some(&features),
            ) {
                Ok(run) => run,
                Err(why_not) => {
                    // The run's OWN construction refuses under this motion - a different wall from
                    // the join's (it is the walked side's strip end), and one this reading is not
                    // about. It is named so the count below is read for what it is, and never
                    // silently skipped.
                    not_built.push_str(&format!(
                        "{name}: the open run does not build: {why_not:?}\n"
                    ));

                    continue;
                }
            };

            read += 1;

            // The places are all laid BEFORE any join is tried, so they are read off the cut
            // whether or not the cut itself goes on to refuse.
            let is_a_feature = |edge: i32| feature_edges.contains(&edge);
            let (cut, _) = try_cut_the_smooth_side(
                &part,
                &graph,
                &edge_at_pair,
                &mut pool,
                &run,
                THE_BORE_WALL_IS_THE_LEFT,
                Some(&is_a_feature),
            );

            assert_eq!(cut.station_points.len(), run.sections.len());

            // The two END stations are left out: their landing lies on a CREASE, and which of that
            // crease's faces the caller offers a join is read from the smooth side's own flood,
            // which is not reachable from here. Every join between them - the whole tessellated
            // wall - is read.
            let mut station = 1;

            while station + 2 < cut.station_boundary.len() {
                let at = pool.position_of(cut.station_boundary[station]);
                let on_to = pool.position_of(cut.station_boundary[station + 1]);
                let mut landed_on = false;
                let mut crossings: Vec<SurfaceWalkCrossing> = Vec::new();
                let mut refusal = SurfaceWalkRefusal::None;
                let mut unfoldings = 0;

                // A station's point may be read in either of the faces its own fold joins, exactly
                // as the caller reads it - but only FORWARDS, from this station to the next.
                'from: for from_face in
                    the_faces_of(&graph, &cut.places[&cut.station_boundary[station]])
                {
                    for to_face in
                        the_faces_of(&graph, &cut.places[&cut.station_boundary[station + 1]])
                    {
                        let (done, unfolded) = try_join_counting(
                            &part,
                            &graph,
                            &edge_at_pair,
                            at,
                            from_face,
                            on_to,
                            to_face,
                            Some(&is_a_feature),
                        );
                        unfoldings = unfolded;

                        match done {
                            Ok(found) => {
                                crossings = found;
                                refusal = SurfaceWalkRefusal::None;
                                landed_on = true;

                                break 'from;
                            }
                            Err(why) => {
                                crossings = Vec::new();
                                refusal = why;
                            }
                        }
                    }
                }

                if !landed_on {
                    refused += 1;
                    said.push_str(&format!(
                        "{name} join {station}: {refusal:?} after {unfoldings} unfoldings, {at:?} to {on_to:?}\n"
                    ));

                    station += 1;
                    continue;
                }

                worst_unfoldings = worst_unfoldings.max(unfoldings);

                if unfoldings > 1 {
                    said.push_str(&format!(
                        "{name} join {station}: landed on the {unfoldings}th strip\n"
                    ));
                }

                // Every point the join mints stands ON the part: each is a crossing of a mesh edge,
                // and the ends are the stations' own landings. (The MIDPOINTS of the pieces are not
                // measured: a line through a mesh vertex crosses no edge there, so one piece may
                // span several facets and bow off a curved wall by the sagitta.)
                for crossing in &crossings {
                    worst_off_the_wall = worst_off_the_wall.max(off_the_part(&part, crossing.at));
                }

                station += 1;
            }
        }

        println!(
            "radius {radius}: {read} placements read, {refused} refused, up to {worst_unfoldings} \
             unfoldings, pieces up to {worst_off_the_wall:.9} off the part"
        );
        println!("{said}");
        println!("{not_built}");

        assert!(
            read > 0,
            "at least one placement's run builds to read (radius {radius})"
        );

        assert_eq!(
            refused, 0,
            "a join is a straight line on the surface and lands wherever the part stands \
             (radius {radius}): {said}"
        );

        assert!(
            worst_off_the_wall < 1e-9,
            "every piece of a join lies in the plane of the facet it runs over (radius {radius})"
        );
    }
}

/// THE TWO END JOINS LAND TOO. An End station's boundary point sits ON a crease - the mouth's
/// continuation onto the wall the bore is raked into - and the join arriving there is a straight
/// surface line that ENDS on a feature edge. The faces that point may be read in are the cut's own
/// reading (`SmoothSideCut.StationFaces`), so this lays the two joins the assembly lays.
#[test]
fn the_two_end_joins_of_the_arc_land_wherever_the_part_stands() {
    for radius in [2.0, 0.5] {
        let mut said = String::new();
        let mut refused = 0;
        let mut read = 0;
        let walk = open_mouth_arc_walk::the_top_mouth_arc_walk();

        for (name, motion) in rigid_motion_meshes::the_motions() {
            let mut part = inner_face_test_mesh::the_part();

            rigid_motion_meshes::move_mesh(&mut part, motion);

            let graph = part.get_mesh_edge_graph();
            let features = FeatureEdgeSet::create_with_graph(
                &part,
                &graph,
                inner_face_test_mesh::BEND_THRESHOLD_DEGREES,
            );
            let edge_at_pair = edges_by_vertex_pair(&graph);
            let feature_edges: HashSet<i32> = features
                .edges()
                .iter()
                .map(|feature| feature.mesh_edge_index)
                .collect();

            let arc = open_mouth_arc_walk::the_same_arc_on(&features, &walk);
            let mut pool = SectionPool::new(&part.vertices);

            let Ok(run) = try_build_run(
                &arc,
                false,
                &part,
                &graph,
                &mut pool,
                radius,
                inner_face_test_mesh::SEGMENTS,
                true,
                Some(&features),
            ) else {
                continue;
            };

            read += 1;

            let is_a_feature = |edge: i32| feature_edges.contains(&edge);
            let (cut, _) = try_cut_the_smooth_side(
                &part,
                &graph,
                &edge_at_pair,
                &mut pool,
                &run,
                THE_BORE_WALL_IS_THE_LEFT,
                Some(&is_a_feature),
            );

            // The first join of the boundary and the last: each has an END station at one of its
            // ends.
            for station in [0, cut.station_boundary.len() - 2] {
                let at = pool.position_of(cut.station_boundary[station]);
                let on_to = pool.position_of(cut.station_boundary[station + 1]);
                let mut landed_on = false;
                let mut refusal = SurfaceWalkRefusal::None;

                'from: for &from_face in &cut.station_faces[station] {
                    for &to_face in &cut.station_faces[station + 1] {
                        match try_join(
                            &part,
                            &graph,
                            &edge_at_pair,
                            at,
                            from_face,
                            on_to,
                            to_face,
                            Some(&is_a_feature),
                        ) {
                            Ok(_) => {
                                refusal = SurfaceWalkRefusal::None;
                                landed_on = true;

                                break 'from;
                            }
                            Err(why) => refusal = why,
                        }
                    }
                }

                if !landed_on {
                    refused += 1;

                    let end_place = &cut.places[&cut.station_boundary[station + 1]];
                    let on_edge = if end_place.edge < 0 {
                        "in a face".to_string()
                    } else {
                        format!(
                            "edge {} at {:.4} from {:?} to {:?}, the point {:.6} off it",
                            end_place.edge,
                            end_place.parameter,
                            vertex(&part, graph.get_vertex0(end_place.edge)),
                            vertex(&part, graph.get_vertex1(end_place.edge)),
                            off_the_segment(&part, &graph, end_place.edge, on_to)
                        )
                    };

                    let mut nearest_feature = -1;
                    let mut nearest_off = f64::MAX;

                    for &feature in &feature_edges {
                        let off = off_the_segment(&part, &graph, feature, on_to);

                        if off < nearest_off {
                            nearest_off = off;
                            nearest_feature = feature;
                        }
                    }

                    said.push_str(&format!(
                        "{name} end join {station}: {refusal:?}, {at:?} to {on_to:?}; the end is \
                         {on_edge}; the nearest feature edge of all is {nearest_feature}, \
                         {nearest_off:.6} off, from {:?} to {:?}; the end stands {:.6} off the part\n",
                        vertex(&part, graph.get_vertex0(nearest_feature)),
                        vertex(&part, graph.get_vertex1(nearest_feature)),
                        off_the_part(&part, on_to)
                    ));
                }
            }
        }

        println!("radius {radius}: {read} placements read, {refused} end joins refused");
        println!("{said}");

        assert!(read > 0, "radius {radius}");

        assert_eq!(
            refused, 0,
            "a join ending on a crease arrives at it (radius {radius}): {said}"
        );
    }
}

/// The faces a boundary point may be read as lying in - the one face a walked
/// landing ended in, or, for a point that slid onto a fold, both of the faces that fold joins.
fn the_faces_of(graph: &MeshEdgeGraph, place: &SmoothSidePlace) -> Vec<i32> {
    if place.face >= 0 {
        return vec![place.face];
    }

    (0..graph.get_face_count(place.edge))
        .map(|i| graph.get_face(place.edge, i))
        .collect()
}

fn vertex(part: &Mesh, index: i32) -> Vector3 {
    Vector3::from_vector3_float(part.vertices[index as usize])
}

/// How far a point stands from the SEGMENT of one mesh edge - not from its
/// infinite line.
fn off_the_segment(part: &Mesh, graph: &MeshEdgeGraph, edge: i32, point: Vector3) -> f64 {
    let a = vertex(part, graph.get_vertex0(edge));
    let along = vertex(part, graph.get_vertex1(edge)) - a;
    let t = if along.length_squared() < 1e-300 {
        0.0
    } else {
        ((point - a).dot(along) / along.length_squared()).clamp(0.0, 1.0)
    };

    (a + (along * t) - point).length()
}

/// How far a point stands from the nearest of the part's triangles.
fn off_the_part(part: &Mesh, point: Vector3) -> f64 {
    let mut nearest = f64::MAX;

    for drawn in part.faces.iter() {
        nearest = nearest.min(
            (mesh_surface_distance::closest_point_on_triangle(
                point,
                vertex(part, drawn.v0),
                vertex(part, drawn.v1),
                vertex(part, drawn.v2),
            ) - point)
                .length(),
        );

        if nearest < 1e-12 {
            break;
        }
    }

    nearest
}
