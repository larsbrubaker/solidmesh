//! Linking a group's selected edges into the runs a single swept tool is built along
//! (`Bevel/BevelChainLinker.cs`): one such run ([`BevelSweptChain`]). The linking itself lands
//! with the core's packages.

use crate::topology::ChainEdge;

/// C# `BevelSweptChain`: one run of edges a single swept tool is built along - the edges head to
/// tail, and whether the run closes on itself.
#[derive(Debug, Clone, PartialEq, Default)]
pub struct BevelSweptChain {
    /// The edges in sweep order, each oriented along the sweep so consecutive edges meet head to
    /// tail.
    pub edges: Vec<ChainEdge>,
    /// Whether the last edge runs back into the first, in which case the sweep welds rather than
    /// caps.
    pub is_closed: bool,
}
