//! What the cut left ONE picked side of ONE chain - the reading a multi-loop refusal is made on,
//! said per chain and per side rather than as the single yes or no the build decides by.

/// The loops the cut left one side of one chain.
#[derive(Debug, Clone, PartialEq, Eq, Hash)]
pub struct ChainSideLoops {
    /// The chain, as an index into the gap result's chains.
    pub chain: i32,
    /// Which of that chain's two sides this row is: 1 left, 2 right.
    pub side: i32,
    /// The side group's tag - rows sharing a tag are the same region of the input.
    pub group_tag: i32,
    /// How many loops OWNED by this chain the cut left on this side; one is what it owes.
    pub loops: i32,
    /// How many of those are closed.
    pub closed_loops: i32,
    /// How many loops this chain minimises somewhere on but does not own all the way round - the
    /// fronts of two chains met along that stretch of boundary, so it is nobody's to fill against.
    pub mixed_loops: i32,
    /// Every OTHER chain that minimises somewhere on a loop counted in this row, ascending - who
    /// this side ran into. Empty where nothing of the sort happened.
    pub met_chains: Vec<i32>,
    /// True where this side left exactly what it owes: one owned loop, closed if the chain is, and
    /// no mixed loop touching it. A build of several chains refuses unless every row says so.
    pub owed: bool,
}
