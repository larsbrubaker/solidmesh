//! One boundary of a level-line gap: where the kept surface ends, on one side of the chain. Ports
//! `Bevel/LevelLine/GapLoop.cs`.

/// C# `GapLoop`. The get-only properties are public fields here; C#'s internal init-only ones
/// (`VertexOwners`, `GroupTag`, `VertexTies`) are too, set after [`GapLoop::new`].
#[derive(Debug, Clone, PartialEq)]
pub struct GapLoop {
    /// The chain of the pick this loop is OWNED by: the chain that is the field's minimiser at
    /// every one of the loop's vertices, as an index into the gap result's chains. A pick of one
    /// chain leaves every loop owned by chain 0. Where the minimiser is not the same chain all the
    /// way round the loop is not owned at all - see `has_mixed_owners` - and this is the chain read
    /// at its first vertex.
    pub chain: i32,

    /// True when the loop's vertices do not all read the same minimising chain: the boundary
    /// crosses from one chain's territory into another's, so its `parameters` are not one chain's
    /// parameters and no single chain can fill against it.
    pub has_mixed_owners: bool,

    /// EVERY chain that minimises the field somewhere on this loop, ascending - the one owner where
    /// the loop is owned, and the several a mixed loop crosses between. Kept because "which chains'
    /// fronts met" is the whole question a multi-loop refusal is about, and `chain` alone cannot
    /// answer it.
    pub owners: Vec<i32>,

    /// 1 for the chain's left side, 2 for its right.
    pub side: i32,

    /// True when the walk comes back to where it started; then the first vertex is not repeated at
    /// the end.
    pub is_closed: bool,

    /// The mesh vertices of the boundary in order, walked so that the KEPT surface lies to the LEFT
    /// of the walk: every step runs along its kept triangle's own winding.
    pub vertices: Vec<i32>,

    /// The chain parameter of each of those vertices, in the same order.
    pub parameters: Vec<f64>,

    /// The minimising chain at each of `vertices`, in the same order, where the walk read it
    /// (`None` otherwise). What the corner-aware loop test reads: at a corner a loop's owner
    /// changes, and WHERE and in what order it changes is the question.
    pub vertex_owners: Option<Vec<i32>>,

    /// The tag of the side group whose walk found this loop. The loop tests read each group's own
    /// loops: a both-sided member has a side in a group it does not name in `side_of_chain`, so
    /// the side alone no longer says which group a loop came from.
    pub group_tag: i32,

    /// Every chain of the side group that owns each of `vertices`: its minimiser, and with it every
    /// other member whose field TIES there, ascending - the walk hands a tie to the lower chain
    /// index, but the vertex is as much the other chain's rail end. Read only by the corner-aware
    /// loop test, and taken only for a pick with corners (`None` otherwise).
    pub vertex_ties: Option<Vec<Vec<i32>>>,
}

impl GapLoop {
    /// C#'s constructor. C#'s optional arguments (`chain = 0`, `hasMixedOwners = false`,
    /// `owners = null`) are explicit here; `owners: None` reads as `[chain]`, as in C#.
    pub fn new(
        side: i32,
        is_closed: bool,
        vertices: Vec<i32>,
        parameters: Vec<f64>,
        chain: i32,
        has_mixed_owners: bool,
        owners: Option<Vec<i32>>,
    ) -> Self {
        GapLoop {
            chain,
            has_mixed_owners,
            owners: owners.unwrap_or_else(|| vec![chain]),
            side,
            is_closed,
            vertices,
            parameters,
            vertex_owners: None,
            group_tag: 0,
            vertex_ties: None,
        }
    }
}
