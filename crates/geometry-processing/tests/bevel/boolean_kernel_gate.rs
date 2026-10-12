//! Port of the test helper `LevelLine/BooleanKernelGate.cs`: the app's own gate on a bevel answer,
//! the Boolean kernel's reading of it as the app publishes it, in single precision.
//! `round_mouth_fillet_crossings::crossing_pairs` counts only strict interior piercings, so a strip
//! of zero-width triangles passes it while the kernel reads the same answer SelfIntersecting and
//! the app refuses it; a kernel test that stops at the crossing count can be green on an answer
//! the app throws away.

#![allow(dead_code)]

use geometry_processing::bevel::level_line::LevelLineMesh;
use polygon_mesh::csg::boolean_processing::{classify_boolean_operand, BooleanOperandVerdict};
use polygon_mesh::Mesh;
use vector_math::{Matrix4X4, Vector3Float};

use crate::sweep_mesh_measures;

/// C# `Verdict`: the kernel's verdict on the kept triangles of `mesh`, taken in single precision.
pub fn verdict(mesh: &LevelLineMesh) -> BooleanOperandVerdict {
    let mut answer = Mesh::new();
    answer.vertices.extend(
        mesh.vertices()
            .iter()
            .map(|&p| Vector3Float::from_vector3(p)),
    );
    for (a, b, c) in sweep_mesh_measures::kept(mesh) {
        answer.faces.add(a, b, c, &answer.vertices);
    }

    classify_boolean_operand(&answer, Matrix4X4::IDENTITY, false)
}
