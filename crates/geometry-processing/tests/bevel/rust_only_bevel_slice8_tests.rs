//! Rust-only tests for the rest of bevel slice 8: the walk of a side's fan at a chain vertex
//! (`sections::section_fan_walk`). The section construction exercises it in context
//! (`SectionSurfaceWalkTests`, `SectionCornerTests`, ...); these hold it to its own contract.

use solidmesh::bevel::sections::section_fan_walk::{self, pair_key};
use solidmesh::bevel::BevelAdvisoryCode;
use solidmesh::topology::{ChainEdge, FeatureEdge};
use solidmesh_math::Vector3;
use solidmesh_mesh::Mesh;

/// A flat hexagonal fan round vertex 0, its rim 1..=6, every face wound up (+z).
fn hexagon(open: bool) -> Mesh {
    let mut mesh = Mesh::new();
    let rim: Vec<Vector3> = (0..6)
        .map(|i| {
            let angle = std::f64::consts::TAU * i as f64 / 6.0;
            Vector3::new(angle.cos(), angle.sin(), 0.0)
        })
        .collect();

    let faces = if open { 5 } else { 6 };
    for i in 0..faces {
        mesh.create_face(&[Vector3::ZERO, rim[i], rim[(i + 1) % 6]]);
    }

    mesh.clean_and_merge();

    mesh
}

/// The vertex nearest a point, and where it stands.
fn vertex_near(mesh: &Mesh, point: Vector3) -> (i32, Vector3) {
    let (index, position) = mesh
        .vertices
        .iter()
        .enumerate()
        .map(|(i, v)| (i as i32, Vector3::from_vector3_float(*v)))
        .min_by(|a, b| (a.1 - point).length().total_cmp(&(b.1 - point).length()))
        .unwrap();

    (index, position)
}

fn edge_between(mesh: &Mesh, one: Vector3, other: Vector3) -> FeatureEdge {
    let graph = mesh.get_mesh_edge_graph();

    (0..graph.edge_count())
        .filter_map(|edge| FeatureEdge::try_classify(mesh, &graph, edge, 0.5))
        .find(|edge| {
            (edge.start == one && edge.end == other) || (edge.start == other && edge.end == one)
        })
        .expect("the two points share an interior edge")
}

/// The chain edge running from `from` to `to` over that edge.
fn chain_edge(mesh: &Mesh, from: Vector3, to: Vector3) -> ChainEdge {
    let edge = edge_between(mesh, from, to);

    ChainEdge::new(0, edge, edge.start != from)
}

#[test]
fn rust_only_the_fan_walk_turns_each_side_from_the_arriving_edge_to_the_leaving_one() {
    let mesh = hexagon(false);
    let graph = mesh.get_mesh_edge_graph();
    let edge_at_pair = section_fan_walk::edges_by_vertex_pair(&graph);
    let (centre, middle) = vertex_near(&mesh, Vector3::ZERO);
    let (_, east) = vertex_near(&mesh, Vector3::new(1.0, 0.0, 0.0));
    let (_, west) = vertex_near(&mesh, Vector3::new(-1.0, 0.0, 0.0));
    let arriving = chain_edge(&mesh, west, middle);
    let leaving = chain_edge(&mesh, middle, east);
    assert_eq!(arriving.to_vertex(), centre);
    assert_eq!(leaving.from_vertex(), centre);

    let mut seen = Vec::new();
    for on_the_left in [true, false] {
        let fan = section_fan_walk::try_walk(
            &mesh,
            &graph,
            &edge_at_pair,
            centre,
            &arriving,
            &leaving,
            on_the_left,
            Vector3::ZERO,
        )
        .expect("a closed fan walks");

        assert_eq!(fan.faces.len(), 3);
        assert_eq!(fan.crossed.len(), fan.faces.len() - 1);
        assert_eq!(
            fan.faces[0],
            if on_the_left {
                arriving.left_face()
            } else {
                arriving.right_face()
            }
        );

        // Every crossed edge leaves the centre and joins the two faces it was crossed between.
        for (j, &crossed) in fan.crossed.iter().enumerate() {
            assert!(graph.get_vertex0(crossed) == centre || graph.get_vertex1(crossed) == centre);
            let faces = [graph.get_face(crossed, 0), graph.get_face(crossed, 1)];
            assert!(faces.contains(&fan.faces[j]) && faces.contains(&fan.faces[j + 1]));
            assert!(!section_fan_walk::is_a_crease(&mesh, &graph, crossed));
        }

        seen.extend(fan.faces);
    }

    // The two sides between them are the whole fan, each face once.
    seen.sort();
    assert_eq!(seen, vec![0, 1, 2, 3, 4, 5]);
}

#[test]
fn rust_only_the_fan_walk_refuses_a_fan_that_runs_off_the_mesh() {
    let mesh = hexagon(true);
    let graph = mesh.get_mesh_edge_graph();
    let edge_at_pair = section_fan_walk::edges_by_vertex_pair(&graph);
    let (centre, _) = vertex_near(&mesh, Vector3::ZERO);

    // From the face north of the centre, turning away from one of its edges at the centre, toward
    // an edge the walk can never reach: whichever way it turns it runs out at the open rim.
    let station = Vector3::new(0.25, 0.5, 0.0);
    let north = mesh
        .faces
        .iter()
        .position(|face| {
            let corners = [face.v0, face.v1, face.v2];
            corners.contains(&centre)
                && corners
                    .iter()
                    .all(|&v| v == centre || mesh.vertices[v as usize].y > 0.5)
        })
        .expect("the fan has a face due north") as i32;
    let drawn = &mesh.faces[north as usize];
    let spoke = [drawn.v0, drawn.v1, drawn.v2]
        .into_iter()
        .find(|&v| v != centre)
        .unwrap();
    let start_edge = edge_at_pair[&pair_key(centre, spoke)];
    let refusal = section_fan_walk::try_walk_to(
        &mesh,
        &graph,
        &edge_at_pair,
        centre,
        north,
        start_edge,
        -5,
        station,
    )
    .unwrap_err();

    assert_eq!(refusal.code(), BevelAdvisoryCode::SurfaceCornerNotSupported);
    assert_eq!(refusal.args(), &[station.into()]);

    // The open rim edge is a "crease" too: nothing may cross it unnoticed.
    let (east, _) = vertex_near(&mesh, Vector3::new(1.0, 0.0, 0.0));
    let rim_edge = edge_at_pair[&pair_key(centre, east)];
    assert_eq!(graph.get_face_count(rim_edge), 1);
    assert!(section_fan_walk::is_a_crease(&mesh, &graph, rim_edge));
}

#[test]
fn rust_only_a_vertex_pair_key_is_the_same_either_way_round() {
    assert_eq!(pair_key(3, 9), pair_key(9, 3));
    assert_ne!(pair_key(3, 9), pair_key(3, 10));
    assert_eq!(pair_key(1, 2), (1i64 << 32) | 2);
}
