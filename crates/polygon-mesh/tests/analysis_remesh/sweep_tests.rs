//! Ordinary parts in ordinary poses must come back: a regression sweep of the shapes CAD users
//! make (booleans, holes, thin plates, pins, rounded edges) over rotations and an offset from the
//! origin, each remeshed at the extent-derived target and bound a caller would pick. A refusal
//! here means the default contract turned away a part it should accept.

use std::f64::consts::TAU;

use solidmesh_math::{Matrix4X4, Vector2};
use solidmesh_mesh::csg::csg_operations::{intersect, subtract, union};
use solidmesh_mesh::processors::outline_to_mesh::extrude_contours;
use solidmesh_mesh::solids::platonic_solids::create_cube;
use solidmesh_mesh::{
    remesh_for_analysis, AnalysisRemeshError, AnalysisRemeshOptions, CancelToken, Mesh,
};

use super::curved_parts::{icosphere, mesh_of, P};
use super::parts::bracket_holes;

/// Parts x rotations x positions covered by the sweep: 7 x 3 x 2.
const SWEEP_CASES: usize = 42;

fn translated(mut mesh: Mesh, x: f64, y: f64, z: f64) -> Mesh {
    mesh.transform(Matrix4X4::create_translation(x, y, z));
    mesh
}

fn cylinder(radius: f64, height: f64, sides: usize) -> Mesh {
    let outline: Vec<Vector2> = (0..sides)
        .map(|i| {
            let a = TAU * i as f64 / sides as f64;
            Vector2::new(radius * a.cos(), radius * a.sin())
        })
        .collect();
    extrude_contours(&[outline], height).expect("extruded a cylinder")
}

fn sphere(radius: f64, level: u32) -> Mesh {
    let (points, faces) = icosphere(radius, level);
    mesh_of(&points, &faces)
}

fn parts() -> Vec<(&'static str, Mesh)> {
    let cancel = CancelToken::new();
    vec![
        (
            "sphere cut by a box",
            intersect(&sphere(10.0, 5), &create_cube(16.0, 16.0, 16.0), &cancel)
                .expect("cut the sphere"),
        ),
        (
            "plate with a boss",
            union(
                &translated(create_cube(60.0, 60.0, 4.0), 0.0, 0.0, 2.0),
                &translated(cylinder(6.0, 10.0, 48), 0.0, 0.0, 3.0),
                &cancel,
            )
            .expect("united the boss"),
        ),
        (
            "thin rib on a plate",
            union(
                &translated(create_cube(100.0, 100.0, 5.0), 0.0, 0.0, 2.5),
                &translated(create_cube(30.0, 0.4, 0.3), 0.0, 10.0, 5.15),
                &cancel,
            )
            .expect("united the rib"),
        ),
        ("thin plate", create_cube(100.0, 60.0, 0.5)),
        ("bracket with holes", bracket_holes()),
        ("cylinder", cylinder(10.0, 20.0, 96)),
        (
            "hollow sphere",
            subtract(&sphere(10.0, 5), &sphere(9.0, 5), &cancel).expect("hollowed the sphere"),
        ),
    ]
}

fn extent(mesh: &Mesh) -> f64 {
    let (mut low, mut high) = ([f64::MAX; 3], [f64::MIN; 3]);
    for v in mesh.vertices.iter() {
        for (k, c) in [v.x as f64, v.y as f64, v.z as f64].into_iter().enumerate() {
            low[k] = low[k].min(c);
            high[k] = high[k].max(c);
        }
    }
    (0..3).map(|k| high[k] - low[k]).fold(0.0, f64::max)
}

#[test]
fn ordinary_parts_in_ordinary_poses_are_never_refused() {
    let rotations: [[f64; 3]; 3] = [[0.0, 0.0, 0.0], [0.41, 0.13, 0.27], [2.2, 1.9, 3.1]];
    let mut covered = 0;
    let mut refused = Vec::new();
    for (name, part) in parts() {
        for (index, rotation) in rotations.iter().enumerate() {
            for offset in [0.0, 300.0] {
                let mut posed = part.clone();
                posed.transform(Matrix4X4::create_rotation_z(rotation[2]));
                posed.transform(Matrix4X4::create_rotation_x(rotation[0]));
                posed.transform(Matrix4X4::create_rotation_y(rotation[1]));
                if offset != 0.0 {
                    posed.transform(Matrix4X4::create_translation(
                        offset,
                        offset * 0.7,
                        -offset * 0.3,
                    ));
                }
                let size = extent(&posed);
                let target = size / 20.0;
                let scale = move |_: P| target;
                let options = AnalysisRemeshOptions::new(&scale, size / 400.0);
                covered += 1;
                if let Err(error) = remesh_for_analysis(&posed, &options) {
                    refused.push(format!("{name} rot{index} off{offset}: {error}"));
                }
            }
        }
    }
    assert_eq!(covered, SWEEP_CASES);
    assert!(
        refused.is_empty(),
        "{} of {covered} ordinary parts refused:\n{}",
        refused.len(),
        refused.join("\n")
    );
}

fn posed(part: &Mesh, rotation: [f64; 3]) -> Mesh {
    let mut posed = part.clone();
    posed.transform(Matrix4X4::create_rotation_z(rotation[2]));
    posed.transform(Matrix4X4::create_rotation_x(rotation[0]));
    posed.transform(Matrix4X4::create_rotation_y(rotation[1]));
    posed
}

/// A pin much smaller than the target is one of the few inputs where the coarse surface folds a
/// free edge over itself: a tetrahedral mesher cannot start from that, so the caller is told by
/// name. The refused case is the tilted pose the sweep found it in.
#[test]
fn a_tiny_pin_that_coarsening_folds_over_is_refused_by_name() {
    let cancel = CancelToken::new();
    let part = union(
        &translated(create_cube(100.0, 100.0, 5.0), 0.0, 0.0, 2.5),
        &translated(cylinder(0.25, 0.3, 16), 20.0, 5.0, 4.9),
        &cancel,
    )
    .expect("united the pin");
    let tilted = posed(&part, [0.41, 0.13, 0.27]);
    let size = extent(&tilted);
    let target = size / 20.0;
    let scale = move |_: P| target;
    let options = AnalysisRemeshOptions::new(&scale, size / 400.0);
    match remesh_for_analysis(&tilted, &options) {
        Err(AnalysisRemeshError::FoldedSurface { edges }) => assert_eq!(edges, 1),
        other => panic!("expected the named fold refusal, got {other:?}"),
    }
}
