//! The beads a rebuild has gathered so far, each remembered with WHOSE it is, so the second pass
//! can take back the ones whose crease it re-read (`Bevel/BevelBeadBatch.cs`). The adding and
//! withdrawing land with the core's packages as an `impl` block.
//!
//! Two lists rather than one, because a bead that brought a void wedge with it is trimmed to that
//! wedge before it joins the batch ([`super::bevel_bead_clip`]) and a bead that did not goes on
//! whole. A chain bead belongs to the one group whose crease it was swept along; a corner FILL
//! belongs to a vertex, so it names the groups whose beads adjoin it instead - which is what makes
//! it withdrawable when either of those beads is rebuilt.

use polygon_mesh::Mesh;

use super::bevel_bead_clip::Clippable;

/// C# `BevelBeadBatch`. `Default` is C#'s freshly constructed batch: every list empty.
#[derive(Debug, Clone, Default)]
pub struct BevelBeadBatch {
    /// The beads that go on with no trim of their own, fill patches last.
    pub whole: Vec<Mesh>,
    /// The beads that brought a void wedge, each to be trimmed to what it may occupy.
    pub clippable: Vec<Clippable>,
    /// Whose each entry of [`BevelBeadBatch::whole`] is, in step with it.
    pub whole_owners: Vec<Owner>,
    /// Whose each entry of [`BevelBeadBatch::clippable`] is, in step with it.
    pub clippable_owners: Vec<Owner>,
    /// How many of the trailing entries of [`BevelBeadBatch::whole`] are corner fills - the split
    /// the batch union's middle fallback rung wants, recovered from a count rather than
    /// remembered as an index because withdrawals renumber the list.
    pub fill_patches_in_whole: i32,
}

/// C# `BevelBeadBatch.Owner`: whose one bead is - a group's, or a corner's.
#[derive(Debug, Clone, PartialEq, Eq, Default)]
pub struct Owner {
    /// The group whose crease a chain bead was swept along; negative for a corner fill.
    pub group: i32,
    /// The groups whose beads meet at a fill's corner (`BevelCornerPatch::groups`), or `None` on
    /// a chain bead.
    pub adjoining_groups: Option<Vec<i32>>,
}
