//! Port of `SectionSurfaceWalkTests.cs`, its BARREL and CORNER half (the roof half is
//! `section_surface_walk_tests`). The tapered barrel's high bore mouth (32 stations) had its
//! wall-side 0.5 offset taken in ONE plane, the worst point standing 0.0446 mm off the faceted
//! wall; walked along the surface from the same station for the same setback, the worst is under
//! 1e-9, and joining consecutive walked points lays the whole boundary on the wall. All four bore
//! mouths read the same.
//!
//! WHAT STOPS A WALK is the OPERATION's own feature reading - the `FeatureEdgeSet` built for this
//! pick - and never the half-degree coplanarity test, which would call every 7.5 degree fold of the
//! barrel's smooth wall a crease: exactly the folds the walk exists to CROSS. The pyramid tests pin
//! that a walk running out at a corner records the corner's VERTEX in every placement.

use std::collections::HashSet;

use polygon_mesh::{Mesh, MeshEdgeGraph};
use vector_math::Vector3;

use geometry_processing::bevel::bevel_chain_linker::{link, BevelSweptChain};
use geometry_processing::bevel::sections::section_builder_run::try_build_loop;
use geometry_processing::bevel::sections::section_fan_walk::{self, EdgeAtPair};
use geometry_processing::bevel::sections::section_surface_walk_faces::try_face_holding;
use geometry_processing::bevel::sections::section_surface_walk_join::try_join;
use geometry_processing::bevel::sections::section_surface_walk_try::try_walk;
use geometry_processing::bevel::sections::{
    SectionPool, SectionSide, SurfaceWalkRefusal, SurfaceWalkResult,
};
use geometry_processing::topology::BevelEdgeGroupResolution;

use crate::mesh_surface_distance::closest_point_on_triangle;
use crate::rigid_motion_meshes;
use crate::section_surface_walk_roof::{corners, off_the_face, triangle};
use crate::tapered_barrel_test_meshes;

const RADIUS: f64 = 0.5;

const SEGMENTS: i32 = 8;

/// THE NUMBER THE READING REFUSED ON: every wall-side offset of the barrel's high mouth, walked
/// along the wall instead of taken in one plane, lands ON the wall - 0.0446 mm becomes under 1e-9.
#[test]
fn the_mouths_wall_side_offset_walked_along_the_wall_is_on_the_wall() {
    let read = the_high_mouth();

    assert_eq!(
        read.stations.len(),
        32,
        "the high bore's mouth is the 32-station loop (39 under float Pi's drill tilt)"
    );

    assert!(read.offsets > 0);
    assert_eq!(
        read.no_fan_face, 0,
        "every station's offset direction is held by one of the wall facets in its own fan"
    );

    println!(
        "high mouth: {} of {} walked, crossings {}",
        read.walked,
        read.offsets,
        crossings_histogram(&read.crossing_counts)
    );

    for stop in &read.walk_stops {
        println!("high mouth stop: {stop}");
    }

    assert_eq!(
        read.walked,
        read.offsets,
        "EVERY wall-side station walks its whole setback once the stop is the operation's own \
         feature set: the wall's 7.5 degree folds are not features of this pick, and crossing them \
         is the walk's whole purpose - instead {}",
        read.walk_stops.join("; ")
    );

    assert!(
        read.crossing_counts.iter().filter(|&&c| c > 0).count() >= 16,
        "most of these reaches leave the facet they set out in - that is exactly why the one-plane \
         offset stood off the part"
    );

    assert!(
        read.worst_as_the_run_drew_it < 1e-9,
        "the boundary the RUN hands out is itself laid along the wall now, so this reading sets out \
         from a point already on the part - the 0.0446 mm is what the one-plane offset it replaced \
         stood off ({})",
        read.worst_as_the_run_drew_it
    );

    assert!(
        read.worst_walked < 1e-9,
        "laid along the surface, the boundary point is on the faceted wall ({})",
        read.worst_walked
    );

    assert!(
        read.worst_off_its_own_end_face < 1e-9,
        "and it is inside the facet the walk says it is in, which is what the rebuild needs ({})",
        read.worst_off_its_own_end_face
    );
}

/// AND BETWEEN THE STATIONS: joining consecutive walked boundary points lays the whole boundary on
/// the wall - every piece is the near-flat geodesic between its ends, so it neither wanders nor
/// doubles back.
#[test]
fn joining_the_walked_points_lays_the_whole_boundary_on_the_wall() {
    let read = the_high_mouth();

    assert_eq!(
        read.joins, read.joins_asked,
        "EVERY consecutive pair round the mouth joins - instead {:?}",
        read.join_refusals
    );

    assert!(
        read.worst_join_off_the_wall < 1e-9,
        "every piece of the joined polyline lies in the plane of the facet it runs over ({})",
        read.worst_join_off_the_wall
    );

    assert!(
        read.worst_join_stretch < 1.003,
        "a join over 7.5 degree folds is barely longer than its chord, so the polyline goes round \
         the mouth in order rather than wandering off and back ({})",
        read.worst_join_stretch
    );
}

/// AND IT IS NOT THE ONE MOUTH: all four bore mouths walk every wall-side station and join every
/// pair round themselves, on the wall throughout.
#[test]
fn every_mouth_walks_and_joins_all_the_way_round() {
    let mouths = the_mouths();

    assert_eq!(
        mouths.len(),
        4,
        "the barrel's two through bores make four mouths"
    );

    for mouth in &mouths {
        println!(
            "mouth of {}: {} of {} walked, {} of {} joined, crossings {}",
            mouth.stations.len(),
            mouth.walked,
            mouth.offsets,
            mouth.joins,
            mouth.joins_asked,
            crossings_histogram(&mouth.crossing_counts)
        );

        assert_eq!(
            mouth.walked,
            mouth.offsets,
            "instead {}",
            mouth.walk_stops.join("; ")
        );

        assert_eq!(
            mouth.joins, mouth.joins_asked,
            "instead {:?}",
            mouth.join_refusals
        );

        assert!(mouth.worst_walked < 1e-9);
        assert!(mouth.worst_join_off_the_wall < 1e-9);
    }
}

/// A WALK THAT ENDS ON A MESH VERTEX RECORDS THE VERTEX. The pyramid's apex is a corner where four
/// folds meet, so an end standing there stands on all four alike. Read in all three ways a reach
/// can run out at a corner - exactly at it, a micron short and a micron past - the vertex is the
/// same fact of the part in every placement, and the end is put exactly ON it.
fn a_walk_running_out_at_a_corner_records_that_vertex_in_every_placement(past: f64) {
    for (name, motion) in rigid_motion_meshes::the_motions() {
        let mut pyramid = a_pyramid();

        rigid_motion_meshes::move_mesh(&mut pyramid, motion);

        let graph = pyramid.get_mesh_edge_graph();
        let edge_at_pair = section_fan_walk::edges_by_vertex_pair(&graph);
        let apex = the_apex(&pyramid);
        let apex_at = Vector3::from_vector3_float(pyramid.vertices[apex as usize]);
        let face = a_face_at(&pyramid, apex);
        let (first, second) = the_arms_at(&pyramid, face, apex);

        // Inside the wedge, aimed exactly at the corner: the exit IS the apex.
        let start = apex_at + (first * 0.4) + (second * 0.4);
        let reach = (apex_at - start).length() + past;

        let walk = try_walk(
            &pyramid,
            &graph,
            &edge_at_pair,
            start,
            face,
            apex_at - start,
            reach,
            None,
        )
        .unwrap_or_else(|walk| panic!("{name}: {:?}", walk.refusal));

        assert_eq!(walk.end_vertex, apex, "{name}");
        assert!((walk.end - apex_at).length() <= 1e-12, "{name}");
        assert!(
            graph.get_vertex0(walk.end_edge) == apex || graph.get_vertex1(walk.end_edge) == apex,
            "{name}: the edge named must at least be one of the corner's"
        );
    }
}

#[test]
fn a_walk_running_out_at_a_corner_records_that_vertex_in_every_placement_0() {
    a_walk_running_out_at_a_corner_records_that_vertex_in_every_placement(0.0);
}

#[test]
fn a_walk_running_out_at_a_corner_records_that_vertex_in_every_placement_minus_1e_6() {
    a_walk_running_out_at_a_corner_records_that_vertex_in_every_placement(-1e-6);
}

#[test]
fn a_walk_running_out_at_a_corner_records_that_vertex_in_every_placement_1e_6() {
    a_walk_running_out_at_a_corner_records_that_vertex_in_every_placement(1e-6);
}

/// THE SAME, FOR A REACH THAT NEVER GETS TO AN EXIT. A line running nearly ALONG one of the apex's
/// folds ends microns from the apex with its exit the whole length of the face away; read by
/// perpendicular distance alone the end stands on TWO of the corner's folds equally, so the corner
/// itself is what it stands on.
#[test]
fn a_walk_running_out_alongside_a_corners_fold_records_the_corner() {
    for (name, motion) in rigid_motion_meshes::the_motions() {
        let mut pyramid = a_pyramid();

        rigid_motion_meshes::move_mesh(&mut pyramid, motion);

        let graph = pyramid.get_mesh_edge_graph();
        let edge_at_pair = section_fan_walk::edges_by_vertex_pair(&graph);
        let apex = the_apex(&pyramid);
        let apex_at = Vector3::from_vector3_float(pyramid.vertices[apex as usize]);
        let face = a_face_at(&pyramid, apex);
        let (first, second) = the_arms_at(&pyramid, face, apex);

        // Three microns into the wedge, running parallel to one of its folds: the exit is five
        // millimetres off down the base, and a micron of reach leaves the end on the corner.
        let start = apex_at + (first * 3e-6) + (second * 3e-6);

        let walk = try_walk(
            &pyramid,
            &graph,
            &edge_at_pair,
            start,
            face,
            first,
            1e-6,
            None,
        )
        .unwrap_or_else(|walk| panic!("{name}: {:?}", walk.refusal));

        assert_eq!(walk.end_vertex, apex, "{name}");
        assert!((walk.end - apex_at).length() <= 1e-12, "{name}");
    }
}

/// C# `HighMouthReading`.
#[derive(Default)]
struct MouthReading {
    stations: Vec<Vector3>,
    offsets: i32,
    walked: i32,
    joins: i32,
    joins_asked: i32,
    /// How many folds each landed walk crossed, in station order.
    crossing_counts: Vec<usize>,
    /// What each refused walk ran into, named.
    walk_stops: Vec<String>,
    worst_as_the_run_drew_it: f64,
    worst_walked: f64,
    worst_off_its_own_end_face: f64,
    worst_join_off_the_wall: f64,
    worst_join_stretch: f64,
    join_refusals: HashSet<SurfaceWalkRefusal>,
    walk_refusals: HashSet<SurfaceWalkRefusal>,
    no_fan_face: i32,
}

/// C#'s `GroupBy(c => c).OrderBy(g => g.Key)` of the crossing counts, as `NxM` pairs.
fn crossings_histogram(counts: &[usize]) -> String {
    let mut keys: Vec<usize> = counts.to_vec();
    keys.sort_unstable();
    keys.dedup();

    keys.iter()
        .map(|key| format!("{key}x{}", counts.iter().filter(|&c| c == key).count()))
        .collect::<Vec<_>>()
        .join(", ")
}

fn the_high_mouth() -> MouthReading {
    the_mouths()
        .into_iter()
        .find(|mouth| mouth.stations.len() == 32)
        .expect("the high mouth is the 32-station loop")
}

/// All four bore mouths of the barrel, each read the same way: build the mouth's run, walk every
/// wall-side offset along the wall, and join the landed points round the mouth.
fn the_mouths() -> Vec<MouthReading> {
    let part = tapered_barrel_test_meshes::the_working_part();
    let resolution = tapered_barrel_test_meshes::the_wall(&part);
    let graph = part.get_mesh_edge_graph();
    let edge_at_pair = section_fan_walk::edges_by_vertex_pair(&graph);
    let wall_faces: Vec<i32> = (0..part.faces.len() as i32)
        .filter(|&f| is_wall(&part, f))
        .collect();

    // THE STOP IS THE OPERATION'S OWN FEATURE READING, built once: see `stops_the_walk`.
    let feature_edges: HashSet<i32> = resolution
        .features
        .edges()
        .iter()
        .map(|feature| feature.mesh_edge_index)
        .collect();

    link(&resolution.edges)
        .iter()
        .filter(|chain| chain.is_closed && chain.edges.len() != 48)
        .map(|one| {
            read_mouth(
                &part,
                &resolution,
                &graph,
                &edge_at_pair,
                &wall_faces,
                &feature_edges,
                one,
            )
        })
        .collect()
}

fn read_mouth(
    part: &Mesh,
    resolution: &BevelEdgeGroupResolution,
    graph: &MeshEdgeGraph,
    edge_at_pair: &EdgeAtPair,
    wall_faces: &[i32],
    feature_edges: &HashSet<i32>,
    one: &BevelSweptChain,
) -> MouthReading {
    let ring: Vec<i32> = one.edges.iter().map(|edge| edge.from_vertex()).collect();
    let mut pool = SectionPool::new(&part.vertices);

    let run = try_build_loop(
        &ring,
        Some(&resolution.features),
        part,
        graph,
        &mut pool,
        RADIUS,
        SEGMENTS,
        true,
    )
    .unwrap_or_else(|_| panic!("every mouth builds its run on the conditioned part"));

    let mut read = MouthReading {
        stations: run.sections.iter().map(|s| s.station).collect(),
        ..MouthReading::default()
    };
    let mut landed: Vec<(Vector3, i32)> = Vec::new();
    let stops = |edge: i32| stops_the_walk(feature_edges, edge);

    for section in &run.sections {
        if section.left_kind != SectionSide::Offset {
            continue;
        }

        read.offsets += 1;

        let at = pool.position_of(section.left_boundary);
        read.worst_as_the_run_drew_it = read
            .worst_as_the_run_drew_it
            .max(off_the_wall(part, wall_faces, at));

        let fan: Vec<i32> = wall_faces
            .iter()
            .copied()
            .filter(|&f| corners(part, f as usize).contains(&section.chain_vertex))
            .collect();
        let direction = at - section.station;

        let Ok((face, in_face)) = try_face_holding(part, section.chain_vertex, direction, &fan)
        else {
            read.no_fan_face += 1;

            continue;
        };

        let walk = match try_walk(
            part,
            graph,
            edge_at_pair,
            section.station,
            face,
            in_face,
            section.setback,
            Some(&stops),
        ) {
            Ok(walk) => walk,
            Err(walk) => {
                read.walk_refusals.insert(walk.refusal);
                read.walk_stops
                    .push(what_it_met(part, graph, feature_edges, &walk));

                continue;
            }
        };

        read.walked += 1;
        read.crossing_counts.push(walk.crossings.len());
        read.worst_walked = read
            .worst_walked
            .max(off_the_wall(part, wall_faces, walk.end));
        read.worst_off_its_own_end_face =
            read.worst_off_its_own_end_face
                .max(off_the_face(part, walk.end_face, walk.end));

        landed.push((walk.end, walk.end_face));
    }

    measure_the_joins(
        part,
        graph,
        edge_at_pair,
        wall_faces,
        feature_edges,
        &landed,
        &mut read,
    );

    read
}

/// WHAT STOPS A WALK, and the whole of it: an edge the OPERATION calls a feature at its own bend
/// threshold - the very set the section builder is given for this pick. A mesh boundary stops it
/// too, which `try_walk` names for itself.
fn stops_the_walk(feature_edges: &HashSet<i32>, edge: i32) -> bool {
    feature_edges.contains(&edge)
}

/// The feature edge a stopped walk ran into, named by where it is, for the refusal to say.
fn what_it_met(
    part: &Mesh,
    graph: &MeshEdgeGraph,
    feature_edges: &HashSet<i32>,
    walk: &SurfaceWalkResult,
) -> String {
    let vertex = |i: i32| Vector3::from_vector3_float(part.vertices[i as usize]);
    let mut best = -1;
    let mut nearest = f64::MAX;

    // Ascending, so a tie names the same edge every run (C#'s HashSet order is insertion order).
    let mut edges: Vec<i32> = feature_edges.iter().copied().collect();
    edges.sort_unstable();

    for edge in edges {
        let a = vertex(graph.get_vertex0(edge));
        let b = vertex(graph.get_vertex1(edge));
        let along = b - a;
        let t = if along.length_squared() < 1e-300 {
            0.0
        } else {
            ((walk.end - a).dot(along) / along.length_squared()).clamp(0.0, 1.0)
        };
        let off = (a + (along * t) - walk.end).length();

        if off < nearest {
            nearest = off;
            best = edge;
        }
    }

    if best < 0 {
        format!(
            "{:?} with no feature edge anywhere near {:?}",
            walk.refusal, walk.end
        )
    } else {
        format!(
            "{:?} at {:?} after {:.4}, on feature edge {best} ({:?} - {:?}), {nearest:.3e} off it",
            walk.refusal,
            walk.end,
            walk.travelled,
            vertex(graph.get_vertex0(best)),
            vertex(graph.get_vertex1(best))
        )
    }
}

fn measure_the_joins(
    part: &Mesh,
    graph: &MeshEdgeGraph,
    edge_at_pair: &EdgeAtPair,
    wall_faces: &[i32],
    feature_edges: &HashSet<i32>,
    landed: &[(Vector3, i32)],
    read: &mut MouthReading,
) {
    let stops = |edge: i32| stops_the_walk(feature_edges, edge);

    for i in 0..landed.len() {
        let from = landed[i];
        let to = landed[(i + 1) % landed.len()];

        read.joins_asked += 1;

        let crossings = match try_join(
            part,
            graph,
            edge_at_pair,
            from.0,
            from.1,
            to.0,
            to.1,
            Some(&stops),
        ) {
            Ok(crossings) => crossings,
            Err(refusal) => {
                read.join_refusals.insert(refusal);

                continue;
            }
        };

        read.joins += 1;

        let mut points = vec![from.0];
        points.extend(crossings.iter().map(|c| c.at));
        points.push(to.0);

        let mut length = 0.0;

        for piece in points.windows(2) {
            length += (piece[1] - piece[0]).length();

            let middle = (piece[0] + piece[1]) / 2.0;
            read.worst_join_off_the_wall = read
                .worst_join_off_the_wall
                .max(off_the_wall(part, wall_faces, middle));
        }

        let chord = (to.0 - from.0).length();

        if chord > 1e-9 {
            read.worst_join_stretch = read.worst_join_stretch.max(length / chord);
        }
    }
}

/// How far a point stands from the faceted wall, measured within each facet's own footprint.
fn off_the_wall(part: &Mesh, wall_faces: &[i32], at: Vector3) -> f64 {
    let mut nearest = f64::MAX;

    for &face in wall_faces {
        let (a, b, c) = triangle(part, face);
        nearest = nearest.min((closest_point_on_triangle(at, a, b, c) - at).length());
    }

    nearest
}

fn is_wall(mesh: &Mesh, face: i32) -> bool {
    (Vector3::from_vector3_float(mesh.faces[face as usize].normal)
        .get_normal()
        .z
        - tapered_barrel_test_meshes::the_walls_normal_z())
    .abs()
        < 2e-3
}

/// A shallow four-sided pyramid: an apex where four folds of 22 degrees meet, over a square base.
/// No two of its folds are coplanar, so naming one of them instead of the vertex is a reading that
/// can be wrong.
fn a_pyramid() -> Mesh {
    let mut mesh = Mesh::new();
    let drop = 1.0;
    let apex = Vector3::new(0.0, 0.0, 0.0);
    let north = Vector3::new(0.0, 5.0, -drop);
    let east = Vector3::new(5.0, 0.0, -drop);
    let south = Vector3::new(0.0, -5.0, -drop);
    let west = Vector3::new(-5.0, 0.0, -drop);

    mesh.create_face(&[apex, north, east]);
    mesh.create_face(&[apex, east, south]);
    mesh.create_face(&[apex, south, west]);
    mesh.create_face(&[apex, west, north]);

    mesh.clean_and_merge();

    mesh
}

/// The pyramid's apex - the one vertex four faces meet at, read topologically.
fn the_apex(mesh: &Mesh) -> i32 {
    // C#'s Dictionary in first-seen order, then OrderByDescending (stable) and First.
    let mut fans: Vec<(i32, i32)> = Vec::new();

    for face in mesh.faces.iter() {
        for corner in [face.v0, face.v1, face.v2] {
            match fans.iter_mut().find(|(vertex, _)| *vertex == corner) {
                Some((_, count)) => *count += 1,
                None => fans.push((corner, 1)),
            }
        }
    }

    let most = fans.iter().map(|&(_, count)| count).max().unwrap_or(0);

    fans.iter()
        .find(|&&(_, count)| count == most)
        .map(|&(vertex, _)| vertex)
        .unwrap_or(-1)
}

/// Any face of a vertex's fan.
fn a_face_at(mesh: &Mesh, vertex: i32) -> i32 {
    (0..mesh.faces.len())
        .find(|&i| corners(mesh, i).contains(&vertex))
        .map(|i| i as i32)
        .unwrap_or(-1)
}

/// The two unit arms of a face's wedge at one of its corners.
fn the_arms_at(mesh: &Mesh, face: i32, vertex: i32) -> (Vector3, Vector3) {
    let at = Vector3::from_vector3_float(mesh.vertices[vertex as usize]);
    let arms: Vec<Vector3> = corners(mesh, face as usize)
        .into_iter()
        .filter(|&corner| corner != vertex)
        .map(|corner| {
            (Vector3::from_vector3_float(mesh.vertices[corner as usize]) - at).get_normal()
        })
        .collect();

    (arms[0], arms[1])
}
