//! What a pick must be before a level-line build will take it. Ports the type of
//! `Bevel/LevelLine/PickValidation.cs` that is not the C# static class `PickValidation`: a pick
//! corner ([`PickCorner`]). The validation's functions land with later packages.

/// C# `PickCorner`: a vertex where picked chains turn into each other - see
/// `PickValidation.CornersOf`.
#[derive(Debug, Clone, PartialEq, Eq, Default)]
pub struct PickCorner {
    /// The input vertex the chains meet at.
    pub vertex: i32,
    /// Every chain of the pick holding the vertex, ascending.
    pub chains: Vec<i32>,
}
