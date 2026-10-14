//! What every selected set of a bevel is, whichever leg of the operation made it: a
//! rebuild-stable SELECTION and nothing else.
//!
//! Two kinds implement it: [`super::BevelEdgeGroup`] for a mesh source and
//! [`super::BevelCornerGroup`] for a path one. They share no data. Size and segment count
//! deliberately do NOT live on a group: one Fillet or Chamfer has ONE radius and ONE segment
//! count, on the operation itself. A file written before the sizes moved holds a `Size` and a
//! `Segments` on every group; unknown members are ignored on load, so such a file still opens.

/// A bevel selection that can copy itself field for field.
///
/// On the group rather than in the edit helper because only the concrete kind knows its own
/// fields; see [`super::bevel_group_edits`] for why every edit copies rather than writes through.
pub trait BevelGroup {
    /// A field-for-field copy of this group.
    fn copy(&self) -> Self
    where
        Self: Sized;
}
