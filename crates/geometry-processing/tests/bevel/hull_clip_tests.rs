//! Ports `Tests/GeometryProcessingTests/LevelLine/HullClipTests.cs` 1:1: the hull mode of the strip
//! clip (`hull_clip`) - a strip run on past the part is cut back by the part's supporting planes,
//! the part as UpperLeftInner's rim end meets it, out through a front plane and a top plane that
//! meet at a corner. C# `[Arguments]` rows are a loop over the same rows.

use std::collections::HashSet;

use geometry_processing::bevel::level_line::hull_clip::{clip, crossed_supporting_planes};
use geometry_processing::bevel::level_line::LevelLineMesh;
use vector_math::Vector3;

use super::level_line_test_geometry::cube_solid;

/// A square strip in the plane x = c across a box [-1, 1]^3 reaching 1 past its front (y = 1) and
/// top (z = 1): only those two planes are crossed, and what is left is the quarter inside, area 1,
/// every vertex inside both - read the same with the whole scene moved far from the origin.
#[test]
fn a_strip_past_a_front_and_a_top_is_cut_back_to_the_box() {
    for offset in [0.0, 1000.0] {
        let shift = Vector3::new(offset, -offset, offset * 0.5);
        let (cube, cube_indices) = cube_solid(1.0);
        let mut positions: Vec<Vector3> = cube.iter().map(|&p| p + shift).collect();
        let s = positions.len() as i32;
        positions.extend(
            [
                Vector3::new(0.0, 0.0, 0.0),
                Vector3::new(0.0, 2.0, 0.0),
                Vector3::new(0.0, 2.0, 2.0),
                Vector3::new(0.0, 0.0, 2.0),
            ]
            .iter()
            .map(|&p| p + shift),
        );
        let mut indices = cube_indices.clone();
        indices.push((s, s + 1, s + 2));
        indices.push((s, s + 2, s + 3));
        let mut mesh = LevelLineMesh::from_triangles(&positions, &indices, None).expect("a mesh");
        let count = indices.len() as i32;
        let mut strip: HashSet<i32> = [count - 2, count - 1].into_iter().collect();

        let mut planes = crossed_supporting_planes(&mesh, &strip);
        assert_eq!(planes.len(), 2, "offset {offset}");
        assert!(planes.iter().any(|p| p.normal.y > 0.999), "offset {offset}");
        assert!(planes.iter().any(|p| p.normal.z > 0.999), "offset {offset}");

        assert!(
            clip(&mut mesh, &mut strip, &mut planes).expect("clips"),
            "offset {offset}"
        );

        let mut area = 0.0;
        let mut worst: f64 = 0.0;
        for &t in &strip {
            let face = mesh.triangles()[t as usize];
            let (a, b, c) = (
                mesh.position(face.a) - shift,
                mesh.position(face.b) - shift,
                mesh.position(face.c) - shift,
            );
            area += (b - a).cross(c - a).length() / 2.0;
            // C# `new[] { a, b, c }.Max(p => Math.Max(Math.Max(p.Y, p.Z) - 1, worst))`.
            worst = [a, b, c]
                .iter()
                .map(|p| f64::max(f64::max(p.y, p.z) - 1.0, worst))
                .fold(f64::MIN, f64::max);
        }

        assert!((area - 1.0).abs() <= 1e-9, "offset {offset}: area {area}");
        assert!(worst <= 1e-9, "offset {offset}: worst {worst}");
        assert!(
            strip.iter().all(|&t| !mesh.is_removed(t)),
            "offset {offset}"
        );
    }
}
