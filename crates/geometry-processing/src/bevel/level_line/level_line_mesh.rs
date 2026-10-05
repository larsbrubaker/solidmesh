//! The level-line build's own working triangle mesh ([`LevelLineMesh`]) and its triangle
//! ([`LevelLineTriangle`]). Ports the types of `Bevel/LevelLine/LevelLineMesh.cs` with the fields
//! of every partial (`LevelLineMesh.InputStrips.cs`, `.InsertPolyline.cs`, `.Needles.cs`), with
//! the triangle's corner indexer and the mesh's read accessors (triangles, edges, positions,
//! lengths, the same-place backstop, a triangle's apex, the needles left standing); the splits,
//! needle repair and polyline insertion land with their method packages. Its holders keep it as
//! `Rc<RefCell<LevelLineMesh>>` (one `RefCell` round the whole mesh), since the build mutates it.

use std::collections::{HashMap, HashSet};
use std::rc::Rc;

use vector_math::Vector3;

use super::FieldCounters;

/// Says of a vertex pair whether it is an edge of the pick (C# `Func<int, int, bool>`).
pub type IsChainEdge = Rc<dyn Fn(i32, i32) -> bool>;

/// C# `LevelLineTriangle`: one triangle of the working mesh - three vertex indices in winding
/// order, and the tag it carries.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, Default)]
pub struct LevelLineTriangle {
    /// The first corner's vertex index.
    pub a: i32,
    /// The second corner's vertex index.
    pub b: i32,
    /// The third corner's vertex index.
    pub c: i32,
    /// Whatever the caller wants carried through the cut - a region id, a feature flag. Children
    /// of a split inherit it unchanged; nothing in this module reads it.
    pub tag: i32,
}

/// C# `LevelLineMesh`: this module's own working triangle mesh - positions, index triples, and
/// edge adjacency, with exactly one mutation, the conforming edge split.
///
/// Kept deliberately small and separate from the kernel's `Mesh`. The level line construction
/// needs only two things of a mesh - to ask which triangles sit on an edge, and to split an edge
/// conformingly - and every other capability is weight the cut would have to reason about. The
/// fill adds two more, both additive (adding a vertex and adding a face).
///
/// A triangle index stays meaningful for the life of the mesh: a split replaces its parent in
/// place with the first child and appends the second, so the index names one of the children, and
/// a triangle that LEAVES - the band the removal takes out, and the needle the repair re-tiles
/// away - keeps its slot and its corners and is marked removed instead. Its edges are detached, so
/// it is invisible to the edge queries, but NOT to a reader walking the triangle list: every such
/// walk has to ask whether it is removed for itself.
///
/// The needle half (`LevelLineMesh.Needles.cs`) finds triangles with no height, mends the ones
/// that are really T-vertices, and keeps the counters that say what a build met. The input-strip
/// half (`LevelLineMesh.InputStrips.cs`) takes a strip of needles the tessellator handed in out
/// ONCE, before the build reads the mesh: on a CAD mesh whose flat faces carry a row of nearly
/// collinear vertices the per-needle repair can cascade (the Many Holes witness ran from 62 MB to
/// 2.2 GB that way), so the strip together with the ring across its edges is re-tiled by an ear
/// cut, each strip visited once and no vertex minted.
#[derive(Default)]
pub struct LevelLineMesh {
    /// The vertex positions.
    pub vertices: Vec<Vector3>,
    /// The triangles, removed ones included (see [`LevelLineMesh::removed`]).
    pub triangles: Vec<LevelLineTriangle>,
    /// The triangles standing on each edge, keyed (low, high). Never iterated into an output: the
    /// edge enumeration sorts the keys, because detaching and re-attaching edges changes the map's
    /// own order mid-build.
    pub edge_triangles: HashMap<(i32, i32), Vec<i32>>,
    /// Per triangle, whether it has left the mesh.
    pub removed: Vec<bool>,
    /// Which INPUT edge each vertex descends from, as (low, high), or (-1, -1). Carried because a
    /// reader downstream has to tell a piece of an input edge from a new edge that merely stands
    /// near it, and after a few splits no geometry says which is which.
    pub descends_from: Vec<(i32, i32)>,
    /// How many triangles have been removed - the gap's band.
    pub removed_count: i32,
    /// How near an edge's own end a point has to stand for the edge split to answer with that end
    /// instead of minting a vertex, as a length; zero means exact coincidence only. Set by whoever
    /// builds the mesh for a cut, to the SNAP that cut reads by, and left small on purpose: it is a
    /// backstop under the refinement's routing, and a backstop must never be WIDER than what a
    /// legitimate caller can need. Never negative or NaN (C#'s setter refuses both).
    pub same_place: f64,
    /// True when the input vertices came from float storage: the vertex reading then reads an
    /// input vertex within its own float step of the line as ON it. Off (exact) by default,
    /// because on a double-precision mesh built at exactly the room a vertex that close must keep
    /// its side.
    pub input_is_float_stored: bool,
    /// The memo of edges a refinement rule has already found clean, keyed (low, high); `None`
    /// until a refinement starts one. One refinement runs against ONE field and ONE reading, so a
    /// memo may never outlive the call that filled it.
    pub answered_clean: Option<HashSet<(i32, i32)>>,
    /// How many input needle strips the strip removal re-tiled away.
    pub input_strips_removed: i32,
    /// How many input needle strips it found and left for the per-needle repair: the region round
    /// the strip was not flat, not one tag, not one simple rim, crossed the pick, or had no
    /// needle-free tiling.
    pub input_strips_left: i32,
    /// The work units the strip removal charged - counted beside each charge, not read off the
    /// budget, so it still says where the work went when the budget runs out inside the removal.
    pub input_strip_work_spent: i64,
    /// The longest rim, in vertices, the ear cut was asked to tile, whether or not it tiled it.
    pub longest_rim_cut: i32,
    /// The needles this mesh has looked at and left standing, by triangle id. A SET and not a
    /// tally because one needle is examined many times - every repair beside it puts it back on
    /// the worklist - and a tally of examinations reads as a mesh full of needles when it holds
    /// one.
    pub left_alone: HashSet<i32>,
    /// The height under which a triangle is a NEEDLE this mesh will repair - the refinement's own
    /// no-height floor, set by whoever builds the mesh for a cut. Zero, the default, repairs
    /// nothing. Carried on the mesh because the splits that can MAKE a needle happen deep inside
    /// the refinement, and a needle has to be repaired the moment it appears.
    pub needle_floor: f64,
    /// Says of any vertex pair whether it is an edge of the PICK - a chain edge, or a fold the
    /// tagging flooded to. `None`, the default, means none is. A repair re-tiles the neighbour
    /// across the needle's long edge, which makes that edge stop existing; where it is the user's
    /// own picked crease the needle is left standing.
    pub is_chain_edge: Option<IsChainEdge>,
    /// How many needles the repair has mended on this mesh.
    pub needles_repaired: i32,
    /// How many of the needles left alone were left because their long edge is a chain edge -
    /// the one reason that is about the PICK rather than about the mesh.
    pub needles_left_on_a_chain_edge: i32,
    /// True where a repair pass stopped at its own bound instead of reaching a fixpoint - a mesh so
    /// tangled that repairing one needle keeps making another. Never seen; the flag is so it
    /// cannot be silent.
    pub repair_hit_its_bound: bool,
}

impl std::fmt::Debug for LevelLineMesh {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        // The chain-edge predicate has no Debug; give the sizes and counters a reader wants.
        f.debug_struct("LevelLineMesh")
            .field("vertex_count", &self.vertices.len())
            .field("triangle_count", &self.triangles.len())
            .field("removed_count", &self.removed_count)
            .field("same_place", &self.same_place)
            .field("needle_floor", &self.needle_floor)
            .field("needles_repaired", &self.needles_repaired)
            .field("has_chain_edge_test", &self.is_chain_edge.is_some())
            .finish_non_exhaustive()
    }
}

impl LevelLineTriangle {
    /// The vertex index at corner 0, 1 or 2 (C# indexer `this[int]`; any other corner reads the
    /// third, as C#'s does).
    pub fn corner(&self, corner: i32) -> i32 {
        if corner == 0 {
            self.a
        } else if corner == 1 {
            self.b
        } else {
            self.c
        }
    }
}

/// The refusal [`LevelLineMesh::set_same_place`] gives a negative or NaN length (C# throws
/// `ArgumentOutOfRangeException`).
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct SamePlaceNotALength(pub f64);

impl std::fmt::Display for SamePlaceNotALength {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        write!(
            f,
            "SamePlace is a length: zero means exact coincidence only. (Actual value was {}.)",
            self.0
        )
    }
}

impl std::error::Error for SamePlaceNotALength {}

impl LevelLineMesh {
    /// The triangles, removed ones included: every walk of this list has to ask
    /// [`LevelLineMesh::is_removed`] for itself.
    pub fn triangles(&self) -> &[LevelLineTriangle] {
        &self.triangles
    }

    /// How many triangles the mesh holds, removed ones included.
    pub fn triangle_count(&self) -> i32 {
        self.triangles.len() as i32
    }

    /// How near an edge's own end a point has to stand for the edge split to answer with that end
    /// instead of minting a vertex, as a length. Zero - the default - means exact coincidence only.
    pub fn same_place(&self) -> f64 {
        self.same_place
    }

    /// Sets [`LevelLineMesh::same_place`]. A negative or NaN backstop is not a narrower test, it
    /// is a silent one - every comparison it takes part in answers false - so it is refused where
    /// it is set and not diagnosed in the cut.
    pub fn set_same_place(&mut self, value: f64) -> Result<(), SamePlaceNotALength> {
        if value.is_nan() || value < 0.0 {
            return Err(SamePlaceNotALength(value));
        }

        self.same_place = value;
        Ok(())
    }

    /// Every edge of the mesh, each once, as its low and high vertex index, in ascending
    /// (low, high) order.
    ///
    /// The order is CANONICAL - a property of the edge set alone - and not the order the edges
    /// were made in. Replacing a triangle detaches and re-attaches its edges, so an edge left
    /// standing on nothing leaves the map and a later split puts it back: the map's own key order
    /// therefore CHANGES mid-build. A caller that roots and splits in that order splits into
    /// different vertex indices depending on where in the build it asked, which is exactly the
    /// fragility this construction exists to remove. Sorting here costs one pass per enumeration
    /// - a handful per build - and buys the whole cut being a pure function of the input.
    pub fn edges(&self) -> Vec<(i32, i32)> {
        let mut keys: Vec<(i32, i32)> = self.edge_triangles.keys().copied().collect();
        // Keys are distinct, so the unstable sort has no ties to order.
        keys.sort_unstable();
        keys
    }

    /// True when this triangle has been removed. A removed triangle keeps its place in the
    /// triangle list - so the caller can still weigh the band that left against the solid it came
    /// from - but it stands on no edge any more, so the edge queries read the mesh that is left.
    pub fn is_removed(&self, triangle: i32) -> bool {
        self.removed[triangle as usize]
    }

    /// The position of a vertex.
    pub fn position(&self, vertex: i32) -> Vector3 {
        self.vertices[vertex as usize]
    }

    /// The length of an edge, or of any vertex pair.
    pub fn length(&self, a: i32, b: i32) -> f64 {
        (self.vertices[a as usize] - self.vertices[b as usize]).length()
    }
}

impl LevelLineMesh {
    /// The corner opposite a triangle's longest side - the first of a tie (C#
    /// `LevelLineMesh.InputStrips.ApexOf`).
    pub fn apex_of(&self, t: LevelLineTriangle) -> i32 {
        let mut corner = 0;
        let mut longest = -1.0;
        for k in 0..3 {
            let length = self.length(t.corner(k), t.corner((k + 1) % 3));
            if length > longest {
                longest = length;
                corner = k;
            }
        }

        t.corner((corner + 2) % 3)
    }

    /// Records a needle the repair leaves standing, counted once however often it is looked at
    /// (C# `LevelLineMesh.Needles.LeaveTheNeedle`).
    pub fn leave_the_needle(&mut self, needle: i32) {
        if self.left_alone.insert(needle) {
            FieldCounters::note_needle(false);
        }
    }

    /// Records a needle left standing because its long edge is a chain edge (C#
    /// `LevelLineMesh.Needles.LeaveTheNeedleOnItsChain`).
    pub fn leave_the_needle_on_its_chain(&mut self, needle: i32) {
        if !self.left_alone.insert(needle) {
            return;
        }

        self.needles_left_on_a_chain_edge += 1;
        FieldCounters::note_needle_on_a_chain_edge();
    }
}
