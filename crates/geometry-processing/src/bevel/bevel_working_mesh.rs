//! The copy of a source mesh that a fillet or chamfer works against - welded if it needed
//! welding, turned right way out, freed of any face folded back over itself, and never the
//! source's own object (`Bevel/BevelWorkingMesh.cs`, a C# `internal static class`, so a module
//! here): the remembered verdict on whether a source mesh is usable as it stands
//! ([`StampedVerdict`]) and carrying the working mesh into the source's placement
//! ([`carry_into`]). Building the working mesh lands with the core's packages as free functions in
//! this module.

use polygon_mesh::Mesh;
use vector_math::Matrix4X4;

/// C# `BevelWorkingMesh.StampedVerdict`: whether a source mesh can be used without the weld
/// repair, stamped with the mesh's `changed_count` when the verdict was taken, so a mesh edited
/// in place answers afresh rather than from a verdict about what it used to be.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub struct StampedVerdict {
    /// The source mesh's `changed_count` when the verdict was taken, or `None` on a fresh entry.
    /// Optional because 0 is the very count a never-edited mesh carries: a plain zero read as a
    /// verdict already taken, and its default `is_usable` of false sent a good operand down the
    /// weld-repair path.
    pub changed_count: Option<i32>,
    /// Whether the mesh is usable as it stands.
    pub is_usable: bool,
}

/// Carries the working mesh by `matrix` (nothing for the identity), turning it right way out again
/// when the matrix mirrors.
pub fn carry_into(working_mesh: &mut Mesh, matrix: Matrix4X4) {
    if matrix == Matrix4X4::IDENTITY {
        return;
    }

    working_mesh.transform(matrix);

    if matrix.determinant() < 0.0 {
        turn_right_way_out(working_mesh);
    }
}

/// Reverses every face's winding. (C# first rewrites each face with its own corners and normal,
/// an identity copy of a value type, so only the reversal does anything.)
pub fn turn_right_way_out(working_mesh: &mut Mesh) {
    working_mesh.reverse_faces();
}
