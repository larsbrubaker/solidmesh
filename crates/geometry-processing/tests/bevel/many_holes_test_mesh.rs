//! Port of the test fixture `Fixtures/ManyHolesTestMesh.cs`: the working mesh "Many Holes.mcx"
//! hands the bevel construction, in the space the size is quoted in, with the one pick the app
//! resolved on it - read from `fixtures/ManyHolesWorkingMesh.txt` (copied from C#'s
//! `Fixtures/ManyHolesWorkingMesh.txt`) instead of loading the `.mcx` on every turn. Read by
//! `section_many_holes_tests` and `section_many_holes_neighbour_tests` through
//! [`crate::many_holes_rim_reading`].

#![allow(dead_code)]

use polygon_mesh::Mesh;
use vector_math::Vector3;

use geometry_processing::topology::{BevelEdgeGroup, BevelSelectionKind};

pub const SIZE: f64 = 0.5;

pub const SEGMENTS: i32 = 8;

pub const BEND_THRESHOLD_DEGREES: f64 = 10.0;

pub const RESOLVED_EDGE_COUNT: i32 = 3301;

/// C# `ThePart`: a fresh copy of the working mesh each time - the sections build rewrites what it
/// is handed.
pub fn the_part() -> Mesh {
    read().0
}

/// C# `ThePickedEdges`: the edges of the one resolved pick, as vertex pairs.
pub fn the_picked_edges() -> Vec<(i32, i32)> {
    read().1
}

/// C# `TheOuterWall`: the face-group pick of the barrel's outer wall, as the app saved it.
pub fn the_outer_wall() -> BevelEdgeGroup {
    BevelEdgeGroup {
        selection_kind: BevelSelectionKind::FaceGroup,
        curve_tolerance_degrees: 10.0,
        flat_angle_degrees: BEND_THRESHOLD_DEGREES,
        anchor_midpoint: Vector3::new(-124.17251654739624, 101.16693496704102, -95.02843152730627),
        anchor_direction: Vector3::new(
            0.6319409519673161,
            -0.7750165373891315,
            -5.5246034819183266E-08,
        ),
        anchor_normal0: Vector3::new(-0.7701122598077506, -0.6279420597002263, 0.1123204209092772),
        anchor_plane_distance0: 21.42617038007257,
        anchor_normal1: Vector3::new(-8.742278000372475E-08, 0.0, -0.9999999999999962),
        anchor_plane_distance1: 95.02844238281251,
        ..BevelEdgeGroup::default()
    }
}

/// C# `Read`: a count line names what follows - `v` vertices, `f` faces, `e` the edges of one
/// resolved pick - and `#` lines are comments.
fn read() -> (Mesh, Vec<(i32, i32)>) {
    let path = format!(
        "{}/tests/bevel/fixtures/ManyHolesWorkingMesh.txt",
        env!("CARGO_MANIFEST_DIR")
    );
    let text = std::fs::read_to_string(&path).unwrap_or_else(|e| panic!("{path}: {e}"));

    let mut coordinates = Vec::new();
    let mut triangles = Vec::new();
    let mut edges = Vec::new();
    let mut lines = text.lines();

    while let Some(line) = lines.next() {
        if line.is_empty() || line.starts_with('#') {
            continue;
        }

        let kind = line.as_bytes()[0];
        let count: usize = line[2..].trim().parse().expect("a count line");

        for _ in 0..count {
            let parts: Vec<&str> = lines.next().expect("the counted line").split(' ').collect();

            match kind {
                b'v' => {
                    for part in &parts[..3] {
                        coordinates.push(part.parse::<f64>().expect("a coordinate"));
                    }
                }
                b'f' => {
                    for part in &parts[..3] {
                        triangles.push(part.parse::<i32>().expect("a vertex index"));
                    }
                }
                _ => edges.push((
                    parts[0].parse::<i32>().expect("an edge end"),
                    parts[1].parse::<i32>().expect("an edge end"),
                )),
            }
        }
    }

    (Mesh::from_arrays(&coordinates, &triangles), edges)
}
