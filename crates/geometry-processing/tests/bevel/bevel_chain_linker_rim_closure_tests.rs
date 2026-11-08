//! The rim of one face of a box has to link into ONE closed run whichever of its edges the walk
//! starts from, even where the face's normal misses its axis by a ten-millionth - which breaks the
//! square-corner tie any normal comparison is deciding.

use solidmesh::bevel::bevel_chain_linker;
use solidmesh::topology::{bevel_edge_group_resolver, FaceGroup, FeatureEdgeSet};
use solidmesh_math::Vector3Float;
use solidmesh_mesh::solids::platonic_solids;
use solidmesh_mesh::{Mesh, MeshEdgeGraph};

const CUBE_SIZE: f64 = 20.0;

/// How far the skewed corner of the x-min face is pulled out along x: over the 20 mm face that
/// leans its normal by 1e-7 radians.
const SKEW: f64 = 2e-6;

/// The bug, at every tilt that could have broken the tie the other way.
#[test]
fn a_rim_whose_face_normal_misses_its_axis_still_closes_from_every_seed() {
    for (skew, toward_z) in [(SKEW, false), (-SKEW, false), (SKEW, true), (-SKEW, true)] {
        assert_the_rim_closes_from_every_seed(
            &cube(skew, toward_z),
            &format!("skew {skew:e} toward {}", if toward_z { "z" } else { "y" }),
        );
    }
}

/// And the guard: the exact case has always closed and has to go on closing.
#[test]
fn an_unskewed_rim_closes_from_every_seed() {
    assert_the_rim_closes_from_every_seed(&cube(0.0, false), "no skew");
}

/// Links the x-min face's rim once per boundary edge used as the face group's anchor, and once
/// per rotation of the resolved edge bag, so each of the four corners gets its turn as the joint
/// the walk has to close on.
fn assert_the_rim_closes_from_every_seed(mesh: &Mesh, fixture: &str) {
    let features = FeatureEdgeSet::create(mesh);
    let graph = MeshEdgeGraph::create(mesh);
    let face_group = FaceGroup::create_with_graph(
        mesh,
        &graph,
        x_min_face(mesh),
        FaceGroup::DEFAULT_CURVE_TOLERANCE_DEGREES,
    )
    .expect("the x-min face floods");

    assert_eq!(
        face_group.boundary_edges().len(),
        4,
        "one face of a box is bounded by four edges - {fixture}"
    );

    for anchor in face_group.boundary_edges() {
        let group = bevel_edge_group_resolver::create_from_face_group(
            &face_group,
            anchor,
            features.flat_angle_degrees(),
        )
        .expect("the anchor bounds the face");
        let resolution = bevel_edge_group_resolver::resolve(&group, mesh);

        assert_eq!(
            resolution.edges.len(),
            4,
            "the rim resolves to its four edges - {fixture}, anchor {}",
            anchor.mesh_edge_index
        );

        let count = resolution.edges.len();
        for rotation in 0..count {
            let seeded: Vec<_> = (0..count)
                .map(|i| resolution.edges[(rotation + i) % count])
                .collect();

            let seed = format!(
                "{fixture}, anchor {}, seed {}",
                anchor.mesh_edge_index, seeded[0].mesh_edge_index
            );
            let chains = bevel_chain_linker::link(&seeded);

            assert_eq!(chains.len(), 1, "the rim is one run - {seed}");
            assert_eq!(
                chains[0].edges.len(),
                4,
                "and all four of its edges are in it - {seed}"
            );
            assert!(
                chains[0].is_closed,
                "and it comes back to where it started, which is what stops the sweep capping both \
                 of its ends in the same place - {seed}"
            );
        }
    }
}

/// A cube whose x-min face is tilted by pulling the two corners on its +y (or +z) side out along
/// x, so the face stays planar and only its normal leaves the axis.
fn cube(skew: f64, toward_z: bool) -> Mesh {
    let mut mesh = platonic_solids::create_cube(CUBE_SIZE, CUBE_SIZE, CUBE_SIZE);

    if skew != 0.0 {
        for i in 0..mesh.vertices.len() {
            let vertex = mesh.vertices[i];

            if vertex.x < 0.0
                && (if toward_z {
                    vertex.z > 0.0
                } else {
                    vertex.y > 0.0
                })
            {
                mesh.vertices[i] =
                    Vector3Float::new_f32(vertex.x - skew as f32, vertex.y, vertex.z);
            }
        }

        mesh.calculate_normals();
        mesh.mark_as_changed();
    }

    mesh
}

/// The index of a triangle of the x-min face, which is the face the rim is taken from.
fn x_min_face(mesh: &Mesh) -> i32 {
    for i in 0..mesh.faces.len() {
        let face = mesh.faces[i];

        if [face.v0, face.v1, face.v2].iter().all(|&vertex| {
            f64::from(mesh.vertices[vertex as usize].x) < -((CUBE_SIZE / 2.0) - 1e-3)
        }) {
            return i as i32;
        }
    }

    panic!("the cube has no face at its minimum x");
}
