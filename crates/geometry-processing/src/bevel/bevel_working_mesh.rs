//! The copy of a source mesh that a fillet or chamfer works against - welded if it needed
//! welding, turned right way out, freed of any face folded back over itself, and never the
//! source's own object (`Bevel/BevelWorkingMesh.cs`, a C# `internal static class`, so a module
//! here): the remembered verdict on whether a source mesh is usable as it stands
//! ([`StampedVerdict`]). Building the working mesh lands with the core's packages as free
//! functions in this module.

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
