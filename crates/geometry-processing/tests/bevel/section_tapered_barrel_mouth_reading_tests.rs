//! THE WHOLE RIM of the tapered barrel
//! ([`crate::tapered_barrel_test_meshes`]) at 0.5 - four bore mouths and two end rims - read
//! through the loop entry `section_builder_run::try_build_loop` and then through
//! `section_assembly::try_assemble`, on the part as the APP would hand it over: through the
//! working-mesh prologue (`the_working_part`).
//!
//! **The conditioning is the whole of the earlier refusal.** Raw boolean output laid out with agg's
//! old float Pi carries DOUBLED fold vertices a thousandth of a millimetre apart; the prologue every
//! bevel runs first removes exactly that, silently. Every one of the six loops then builds its run,
//! lays every wall-side offset ALONG the surface, and assembles closed, manifold, one shell with no
//! directed edge run twice.
//!
//! The rim as the conditioned part carries it: six closed loops, the wall on the LEFT of all six -
//! the four bore mouths (30, 30, 32, 32 stations), each touching the wall's fold at exactly two
//! stations, and the two 48-station end rims, which slide on the wall and offset into the flat cap
//! at every station.

use std::collections::HashMap;

use solidmesh_math::{math_helper, Vector3};
use solidmesh_mesh::{Mesh, MeshEdgeGraph};

use solidmesh::bevel::bevel_chain_linker::{link, BevelSweptChain};
use solidmesh::bevel::bevel_working_mesh::{create_from_with_advisories, StampedVerdict};
use solidmesh::bevel::sections::section_builder::try_creases_in_the_fan;
use solidmesh::bevel::sections::section_builder_run::try_build_loop;
use solidmesh::bevel::sections::section_fan_walk::{edges_by_vertex_pair, EdgeAtPair};
use solidmesh::bevel::sections::{section_assembly, SectionPool, SectionRun};
use solidmesh::bevel::BevelAdvisory;
use solidmesh::topology::{BevelEdgeGroupResolution, ChainEdge};

use crate::{mesh_measurements, mesh_surface_distance, tapered_barrel_test_meshes};

const RADIUS: f64 = 0.5;

const SEGMENTS: i32 = 8;

/// WHAT CONDITIONING DID: the weld the app runs first takes the boolean's doubled fold vertices and
/// its zero-area slivers out, silently. Read on the part as float Pi laid it out, because only that
/// layout HAS doubled fold vertices.
#[test]
fn the_apps_prologue_takes_the_booleans_doubled_fold_vertices_out() {
    let raw = tapered_barrel_test_meshes::the_part_as_float_pi_laid_it_out();
    let (working, advisories) = create_from_with_advisories(&raw, &mut StampedVerdict::default());

    assert_eq!(raw.vertices.len(), 247);
    assert_eq!(raw.faces.len(), 498);
    assert_eq!(working.vertices.len(), 240);
    assert_eq!(working.faces.len(), 484);

    assert_eq!(
        advisories.len(),
        0,
        "the weld handles it, so nothing here is worth telling the user about"
    );

    assert!(
        smallest_edge(&raw) < 1e-6,
        "the boolean left two copies of each cut fold vertex on top of each other"
    );

    assert!(smallest_edge(&working) > 1e-3);
    assert!(thinnest_triangle(&raw) < 1e-7);
    assert!(thinnest_triangle(&working) > 1e-3);
}

#[test]
fn every_loop_of_the_rim_builds_its_run_on_the_conditioned_part() {
    let read = read();

    assert_eq!(
        sorted(read.iter().map(|l| l.ring.len() as i32).collect()),
        vec![30, 30, 32, 32, 48, 48],
        "four bore mouths and the two end rims (36, 36, 39, 39 under float Pi's drill tilt)"
    );

    for one in &read {
        assert!(one.wall_is_on_the_left);

        assert_eq!(
            one.doubled_stations, 0,
            "the weld already merged every doubled fold vertex the cut left"
        );

        assert_eq!(one.wall_creases.iter().filter(|&&c| c > 1).count(), 0);
        assert_eq!(
            one.bore_creases.iter().filter(|&&c| c > 1).count(),
            0,
            "the two and three crease fans the old reading refused were the weld's to remove"
        );

        assert!(one.built, "{:?}", one.why);
    }

    let mouths: Vec<&LoopReading> = read.iter().filter(|l| l.ring.len() != 48).collect();

    assert_eq!(
        sorted(
            mouths
                .iter()
                .map(|l| l.wall_creases.iter().filter(|&&c| c == 1).count() as i32)
                .collect()
        ),
        vec![2, 2, 2, 2],
        "each mouth still touches the wall's fold at exactly two stations"
    );

    for rim in read.iter().filter(|l| l.ring.len() == 48) {
        assert_eq!(rim.wall_creases.iter().filter(|&&c| c == 1).count(), 48);
        assert_eq!(
            rim.bore_creases.iter().filter(|&&c| c == 0).count(),
            48,
            "an end rim slides on the wall and offsets into the flat cap at every station"
        );
    }
}

#[test]
fn the_wall_side_offset_is_walked_onto_its_facet_and_every_mouth_assembles_closed() {
    let read = read();
    let part_volume = signed_volume(&tapered_barrel_test_meshes::the_working_part());

    for mouth in read.iter().filter(|l| l.ring.len() != 48) {
        assert_eq!(
            mouth.boundary_points_off_the_wall, 0,
            "the 0.5 offset is laid ALONG the wall now, so it ends in a real facet however many \
             folds the mouth's tangent carries it over - it used to stand 0.0446 off"
        );

        assert!(mouth.assembled, "{:?}", mouth.assembly_why);

        assert_eq!(mouth.open_edges, 0);
        assert_eq!(mouth.non_manifold_edges, 0);
        assert_eq!(mouth.shells, 1);

        assert_eq!(
            mouth.directed_edges_not_run_once, 0,
            "the loft's last row and the kept rings of the facets the cut left standing run the \
             SAME boundary polyline, by pool index, opposite ways round"
        );

        assert!(
            mouth.volume < part_volume,
            "a fillet on a bore mouth takes material away"
        );
    }

    // Every loop's boundary is on the wall now - so the one number left is the lattice. The
    // MOUTHS are on it to picometres; the two 48-edge END RIMS are the loose ones at up to 9.2e-9,
    // because their side reads as an offset and is WALKED onto the surface.
    let worst = read
        .iter()
        .map(|l| l.worst_boundary_off_the_wall)
        .fold(f64::MIN, f64::max);

    let worst_mouth = read
        .iter()
        .filter(|l| l.ring.len() != 48)
        .map(|l| l.worst_boundary_off_the_wall)
        .fold(f64::MIN, f64::max);

    assert!(
        worst_mouth < 1e-9,
        "no mouth stands off the part at all - {worst_mouth}"
    );

    assert!(
        worst < 1e-8,
        "and the walked end rims stand off it by a hundredth of a micron, worst {worst:.3e}"
    );

    for rim in read.iter().filter(|l| l.ring.len() == 48) {
        assert!(
            rim.worst_boundary_off_the_wall < 1e-6,
            "an end rim's boundary slides along a mesh edge, so it never leaves the surface"
        );

        assert!(rim.assembled, "{:?}", rim.assembly_why);

        assert_eq!(rim.open_edges, 0);
        assert_eq!(rim.non_manifold_edges, 0);
        assert_eq!(rim.shells, 1);

        assert!(
            rim.worst_wall_near_the_loop < 1e-6,
            "the rebuilt wall beside the rim is the original wall, untouched"
        );
    }
}

#[test]
fn every_six_of_the_barrels_loops_assembles() {
    let read = read();

    assert_eq!(read.len(), 6);

    for one in &read {
        assert!(
            one.assembled,
            "{} stations refused {} at mesh edge {}",
            one.ring.len(),
            one.assembly_why
                .as_ref()
                .map_or("with no code".to_string(), |w| format!("{:?}", w.code())),
            one.assembly_why
                .as_ref()
                .map_or("none".to_string(), |w| w.mesh_edge_index().to_string())
        );

        assert_eq!(one.open_edges, 0);
        assert_eq!(one.non_manifold_edges, 0);

        assert_eq!(
            one.unclassifiable_edges, 0,
            "a second pass reads {} as a torn face",
            one.unclassifiable_at
        );
        assert_eq!(one.shells, 1);
        assert_eq!(one.directed_edges_not_run_once, 0);
    }

    let worst = read
        .iter()
        .map(|l| l.worst_boundary_off_the_wall)
        .fold(f64::MIN, f64::max);

    assert!(
        worst < 1e-8,
        "absorbing a split moves no boundary point - the 9.2e-9 is the WALKED end rim's own \
         landing tolerance, not a split (see the wall-side row), worst {worst:.3e}"
    );
}

#[test]
fn the_walls_fold_leaves_the_mouth_square_where_it_touches_it() {
    let read = read();
    let off_the_walk: Vec<f64> = read
        .iter()
        .filter(|l| l.ring.len() != 48)
        .flat_map(|l| {
            l.crease_angle_to_the_walk
                .iter()
                .copied()
                .filter(|a| !a.is_nan())
        })
        .collect();

    assert_eq!(
        off_the_walk.len(),
        8,
        "two wall slides on each of the four mouths"
    );

    let least = off_the_walk.iter().copied().fold(f64::MAX, f64::min);
    let most = off_the_walk.iter().copied().fold(f64::MIN, f64::max);

    assert!(
        least > 70.0,
        "the fold runs up and down the wall while the mouth's tangent runs across it, so the one \
         place the fold touches the mouth is the one place it is most nearly square to it - {least}"
    );

    assert!(most < 90.5, "{most}");
}

#[derive(Default)]
struct LoopReading {
    ring: Vec<i32>,
    stations: Vec<Vector3>,
    wall_is_on_the_left: bool,
    wall_creases: Vec<i32>,
    bore_creases: Vec<i32>,
    crease_angle_to_the_walk: Vec<f64>,
    doubled_stations: i32,
    built: bool,
    why: Option<BevelAdvisory>,
    boundary_points_off_the_wall: i32,
    worst_boundary_off_the_wall: f64,
    assembled: bool,
    assembly_why: Option<BevelAdvisory>,
    open_edges: i32,
    non_manifold_edges: i32,
    unclassifiable_edges: i32,
    unclassifiable_at: String,
    shells: i32,
    worst_wall_near_the_loop: f64,
    volume: f64,
    directed_edges_not_run_once: i32,
}

fn sorted(mut values: Vec<i32>) -> Vec<i32> {
    values.sort_unstable();
    values
}

fn signed_volume(mesh: &Mesh) -> f64 {
    let mut six = 0.0;

    for face in 0..mesh.faces.len() {
        let (a, b, c) = corners(mesh, face);
        six += a.dot(b.cross(c));
    }

    six / 6.0
}

fn read() -> Vec<LoopReading> {
    let part = tapered_barrel_test_meshes::the_working_part();
    let resolution = tapered_barrel_test_meshes::the_wall(&part);
    let graph = part.get_mesh_edge_graph();
    let edge_at_pair = edges_by_vertex_pair(&graph);
    let wall_faces = wall_faces(&part);

    let mut loops: Vec<BevelSweptChain> = link(&resolution.edges)
        .into_iter()
        .filter(|chain| chain.is_closed)
        .collect();
    // A stable sort, so loops of equal length keep their walked order.
    loops.sort_by_key(|chain| chain.edges.len());

    loops
        .iter()
        .map(|one| read_one_loop(&part, &graph, &edge_at_pair, &resolution, &wall_faces, one))
        .collect()
}

fn read_one_loop(
    part: &Mesh,
    graph: &MeshEdgeGraph,
    edge_at_pair: &EdgeAtPair,
    resolution: &BevelEdgeGroupResolution,
    wall_faces: &[usize],
    one: &BevelSweptChain,
) -> LoopReading {
    let mut read = LoopReading {
        ring: one.edges.iter().map(|edge| edge.from_vertex()).collect(),
        stations: one.edges.iter().map(|edge| edge.from()).collect(),
        wall_is_on_the_left: one.edges.iter().all(|edge| is_wall(edge.left_normal())),
        ..Default::default()
    };

    let count = one.edges.len();

    for i in 0..count {
        let leaving = &one.edges[i];
        let arriving = &one.edges[(i + count - 1) % count];
        let vertex = leaving.from_vertex();

        let wall =
            try_creases_in_the_fan(vertex, arriving, leaving, true, part, graph, edge_at_pair).ok();
        let bore =
            try_creases_in_the_fan(vertex, arriving, leaving, false, part, graph, edge_at_pair)
                .ok();

        read.wall_creases
            .push(wall.as_ref().map_or(-1, |w| w.len() as i32));
        read.bore_creases
            .push(bore.as_ref().map_or(-1, |b| b.len() as i32));
        read.crease_angle_to_the_walk.push(match &wall {
            Some(wall) if wall.len() == 1 => {
                angle_to_the_walk(part, graph, wall[0], vertex, arriving, leaving)
            }
            _ => f64::NAN,
        });

        if (read.stations[i] - read.stations[(i + 1) % count]).length() < 1e-3 {
            read.doubled_stations += 1;
        }
    }

    let mut pool = SectionPool::new(&part.vertices);

    let run = match try_build_loop(
        &read.ring,
        Some(&resolution.features),
        part,
        graph,
        &mut pool,
        RADIUS,
        SEGMENTS,
        true,
    ) {
        Ok(run) => run,
        Err(why) => {
            read.why = why.advisory().cloned();

            return read;
        }
    };

    read.built = true;

    measure_the_wall_side_boundary(part, wall_faces, &run, &pool, &mut read);

    match section_assembly::try_assemble(part, graph, &run, &mut pool) {
        Ok(result) => {
            read.assembled = true;
            measure_the_result(part, &result, &mut read);
        }
        Err(failure) => read.assembly_why = failure.advisory().cloned(),
    }

    read
}

fn measure_the_wall_side_boundary(
    part: &Mesh,
    wall_faces: &[usize],
    run: &SectionRun,
    pool: &SectionPool,
    read: &mut LoopReading,
) {
    for section in &run.sections {
        let at = pool.position_of(if read.wall_is_on_the_left {
            section.left_boundary
        } else {
            section.right_boundary
        });
        let mut nearest = f64::MAX;

        for &face in wall_faces {
            let (a, b, c) = corners(part, face);
            nearest = nearest
                .min((mesh_surface_distance::closest_point_on_triangle(at, a, b, c) - at).length());
        }

        read.worst_boundary_off_the_wall = read.worst_boundary_off_the_wall.max(nearest);

        if nearest > 1e-6 {
            read.boundary_points_off_the_wall += 1;
        }
    }
}

fn measure_the_result(part: &Mesh, result: &Mesh, read: &mut LoopReading) {
    let graph = result.get_mesh_edge_graph();

    for edge in 0..graph.edge_count() {
        if graph.get_face_count(edge) == 1 {
            read.open_edges += 1;
        } else if graph.get_face_count(edge) != 2 {
            read.non_manifold_edges += 1;
        }
    }

    let (unreadable, unreadable_at) = mesh_measurements::unclassifiable_edges(Some(result));
    read.unclassifiable_edges = unreadable;
    read.unclassifiable_at = unreadable_at;

    read.shells = mesh_measurements::shell_count(result);
    read.volume = signed_volume(result);

    let mut directed: HashMap<(i32, i32), i32> = HashMap::new();

    for drawn in result.faces.iter() {
        for edge in [
            (drawn.v0, drawn.v1),
            (drawn.v1, drawn.v2),
            (drawn.v2, drawn.v0),
        ] {
            *directed.entry(edge).or_insert(0) += 1;
        }
    }

    read.directed_edges_not_run_once = directed.values().filter(|&&run| run != 1).count() as i32;

    let centre = read
        .stations
        .iter()
        .fold(Vector3::ZERO, |sum, &station| sum + station)
        / read.stations.len() as f64;

    for face in wall_faces(result) {
        let (a, b, c) = corners(result, face);
        let at = (a + b + c) / 3.0;

        // Only the wall NEAR this loop is the loop's own doing; the rest of the barrel is untouched
        // either way, so measuring it would only dilute the number.
        if (at - centre).length() <= 3.0 {
            read.worst_wall_near_the_loop = read
                .worst_wall_near_the_loop
                .max(mesh_surface_distance::from(part, at));
        }
    }
}

fn angle_to_the_walk(
    mesh: &Mesh,
    graph: &MeshEdgeGraph,
    crease: i32,
    vertex: i32,
    arriving: &ChainEdge,
    leaving: &ChainEdge,
) -> f64 {
    let other = if graph.get_vertex0(crease) == vertex {
        graph.get_vertex1(crease)
    } else {
        graph.get_vertex0(crease)
    };
    let along = (Vector3::from_vector3_float(mesh.vertices[other as usize])
        - Vector3::from_vector3_float(mesh.vertices[vertex as usize]))
    .get_normal();
    let walk = (arriving.direction() + leaving.direction()).get_normal();

    math_helper::radians_to_degrees(along.dot(walk).abs().clamp(0.0, 1.0).acos())
}

fn wall_faces(mesh: &Mesh) -> Vec<usize> {
    (0..mesh.faces.len())
        .filter(|&face| is_wall(Vector3::from_vector3_float(mesh.faces[face].normal)))
        .collect()
}

fn corners(mesh: &Mesh, face: usize) -> (Vector3, Vector3, Vector3) {
    let drawn = mesh.faces[face];
    let at = |v: i32| Vector3::from_vector3_float(mesh.vertices[v as usize]);

    (at(drawn.v0), at(drawn.v1), at(drawn.v2))
}

fn smallest_edge(mesh: &Mesh) -> f64 {
    let mut shortest = f64::MAX;

    for face in 0..mesh.faces.len() {
        let (a, b, c) = corners(mesh, face);
        shortest = shortest.min((a - b).length().min((b - c).length().min((c - a).length())));
    }

    shortest
}

fn thinnest_triangle(mesh: &Mesh) -> f64 {
    let mut thinnest = f64::MAX;

    for face in 0..mesh.faces.len() {
        let (a, b, c) = corners(mesh, face);
        let area = (b - a).cross(c - a).length() / 2.0;
        let longest = (a - b).length().max((b - c).length().max((c - a).length()));
        thinnest = thinnest.min(if longest < 1e-12 {
            0.0
        } else {
            2.0 * area / longest
        });
    }

    thinnest
}

fn is_wall(normal: Vector3) -> bool {
    (normal.get_normal().z - tapered_barrel_test_meshes::the_walls_normal_z()).abs() < 2e-3
}
