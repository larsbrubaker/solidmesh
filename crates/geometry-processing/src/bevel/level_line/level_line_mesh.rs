//! The level-line build's own working triangle mesh ([`LevelLineMesh`]) and its triangle
//! ([`LevelLineTriangle`]): the types with the fields every half of the mesh uses (input strips,
//! polyline insertion, needles), the triangle's corner indexer and the mesh's read accessors
//! (triangles, edges, positions, lengths, the same-place backstop, a triangle's apex, the needles
//! left standing); the edits, needle repair and polyline insertion are in
//! `level_line_mesh_edits.rs`, `level_line_mesh_needles.rs` and
//! `level_line_mesh_insert_polyline.rs`. Its holders keep it as
//! `Rc<RefCell<LevelLineMesh>>` (one `RefCell` round the whole mesh), since the build mutates it.

use std::collections::{HashMap, HashSet};
use std::rc::Rc;

use vector_math::Vector3;

use super::FieldCounters;

/// Says of a vertex pair whether it is an edge of the pick.
pub type IsChainEdge = Rc<dyn Fn(i32, i32) -> bool>;

/// One triangle of the working mesh - three vertex indices in winding
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

/// This module's own working triangle mesh - positions, index triples, and
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
/// The needle half (`level_line_mesh_needles.rs`) finds triangles with no height, mends the ones
/// that are really T-vertices, and keeps the counters that say what a build met. The input-strip
/// half (`level_line_mesh_input_strips.rs`) takes a strip of needles the tessellator handed in out
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
    /// Per triangle, the corners of the triangle it descends from as it was first added: an input
    /// triangle's own corners, which every piece a split cuts from it inherits
    /// ([`LevelLineMesh::input_triangle_behind`]). Carried because a piece lies in its input
    /// triangle's plane, so how far rounding the input can tilt the piece is that triangle's, never
    /// what the piece's own shape would say (a sliver's would be far wider). A triangle added whole
    /// (the input's, a re-tiled strip's, a fill's) is its own.
    pub triangle_descends_from: Vec<[i32; 3]>,
    /// How many triangles have been removed - the gap's band.
    pub removed_count: i32,
    /// How near an edge's own end a point has to stand for the edge split to answer with that end
    /// instead of minting a vertex, as a length; zero means exact coincidence only. Set by whoever
    /// builds the mesh for a cut, to the SNAP that cut reads by, and left small on purpose: it is a
    /// backstop under the refinement's routing, and a backstop must never be WIDER than what a
    /// legitimate caller can need. Never negative or NaN (the setter refuses both).
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
    /// True where the band's side tags are the rolling ball's sides
    /// (`rolling_ball_band_tags`), whose two sides are the caller's face regions and so may meet
    /// again away from the chain (another chain's crease, or the chain's own crease running on
    /// past a free end): the room reads such a meeting as foreign (`foreign_edge_room`). False,
    /// the default, for every other tagging, whose sides meet only on the chain or across a
    /// feature edge the room has always left to the cut.
    pub sides_from_the_ball: bool,
    /// Where the band's sides are the ball's (`sides_from_the_ball`), whether each end of its one
    /// open chain (first, last) is a free end, as the ball read it
    /// (`rolling_ball_setbacks::SideSetbacks::free_ends`); false otherwise.
    pub ball_free_ends: [bool; 2],
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
    /// The vertex index at corner 0, 1 or 2 (any other corner reads the third).
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

/// The refusal [`LevelLineMesh::set_same_place`] gives a negative or NaN length.
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
    /// The corner opposite a triangle's longest side - the first of a tie.
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

    /// Records a needle the repair leaves standing, counted once however often it is looked at.
    pub fn leave_the_needle(&mut self, needle: i32) {
        if self.left_alone.insert(needle) {
            FieldCounters::note_needle(false);
        }
    }

    /// Records a needle left standing because its long edge is a chain edge.
    pub fn leave_the_needle_on_its_chain(&mut self, needle: i32) {
        if !self.left_alone.insert(needle) {
            return;
        }

        self.needles_left_on_a_chain_edge += 1;
        FieldCounters::note_needle_on_a_chain_edge();
    }
}

impl LevelLineTriangle {
    /// A triangle from three corners in winding order and the tag.
    pub fn new(a: i32, b: i32, c: i32, tag: i32) -> Self {
        Self { a, b, c, tag }
    }
}

impl LevelLineMesh {
    /// An empty mesh (the builders fill it).
    pub fn new() -> Self {
        Self::default()
    }

    /// The vertex positions.
    pub fn vertices(&self) -> &[Vector3] {
        &self.vertices
    }

    /// How many vertices the mesh holds.
    pub fn vertex_count(&self) -> i32 {
        self.vertices.len() as i32
    }

    /// How many triangles have been removed - the gap's band.
    pub fn removed_count(&self) -> i32 {
        self.removed_count
    }

    /// The INPUT edge a vertex descends from, as the two input vertex indices behind it, or
    /// (-1, -1) where it descends from no single input edge. An input vertex answers (itself,
    /// itself), since it stands on every input edge that names it. A split-edge vertex descends
    /// from whatever its parent ENDS descend from, merged - so a piece of an input edge answers
    /// that edge however many times it has been split; two vertices are on one input edge exactly
    /// when their two answers name the same two input vertices between them, which is structure and
    /// not a tolerance.
    pub fn input_edge_behind(&self, vertex: i32) -> (i32, i32) {
        self.descends_from[vertex as usize]
    }

    /// The corners of the triangle `triangle` descends from (`triangle_descends_from`): its input
    /// triangle's, however many times it has been split. A triangle pushed onto `triangles` without
    /// [`LevelLineMesh::add_triangle`] carries no record and answers its own corners.
    pub fn input_triangle_behind(&self, triangle: i32) -> [i32; 3] {
        self.triangle_descends_from
            .get(triangle as usize)
            .copied()
            .unwrap_or_else(|| {
                let t = self.triangles[triangle as usize];
                [t.a, t.b, t.c]
            })
    }

    /// The input edge two ancestries have in common: the pair of input vertices they name between
    /// them, or (-1, -1) where they name none, one, or more than two - none of which is a single
    /// input edge.
    pub fn merge(left: (i32, i32), right: (i32, i32)) -> (i32, i32) {
        let mut low = -1;
        let mut high = -1;
        for vertex in [left.0, left.1, right.0, right.1] {
            if vertex < 0 {
                return (-1, -1);
            }

            if vertex == low || vertex == high {
                continue;
            }

            if low < 0 {
                low = vertex;
            } else if high < 0 {
                high = vertex;
            } else {
                return (-1, -1);
            }
        }

        if high < 0 {
            (low, low)
        } else {
            (low.min(high), low.max(high))
        }
    }

    /// Takes a triangle out of the mesh: it leaves every edge it stood on, and an edge left
    /// standing on nothing leaves the mesh with it. Removing a triangle twice is a no-op.
    pub fn remove_triangle(&mut self, triangle: i32) {
        if self.removed[triangle as usize] {
            return;
        }

        self.detach(triangle, self.triangles[triangle as usize]);
        self.removed[triangle as usize] = true;
        self.removed_count += 1;
    }

    /// The triangles standing on an edge: one on a boundary, two inside a closed mesh.
    pub fn triangles_on(&self, a: i32, b: i32) -> &[i32] {
        self.edge_triangles
            .get(&Self::key(a, b))
            .map_or(&[], |found| found.as_slice())
    }

    /// True when the edge exists in the mesh.
    pub fn has_edge(&self, a: i32, b: i32) -> bool {
        self.edge_triangles.contains_key(&Self::key(a, b))
    }

    /// A triangle's centroid.
    pub fn centroid(&self, triangle: i32) -> Vector3 {
        let t = self.triangles[triangle as usize];
        (self.vertices[t.a as usize] + self.vertices[t.b as usize] + self.vertices[t.c as usize])
            / 3.0
    }

    /// The corner of a triangle that is not either end of one of its own edges.
    pub fn opposite_corner(t: LevelLineTriangle, a: i32, b: i32) -> i32 {
        if t.a != a && t.a != b {
            t.a
        } else if t.b != a && t.b != b {
            t.b
        } else {
            t.c
        }
    }

    /// The (low, high) key of an edge.
    pub fn key(a: i32, b: i32) -> (i32, i32) {
        if a < b {
            (a, b)
        } else {
            (b, a)
        }
    }

    /// Starts a fresh memo of edges a refinement rule has already found clean, discarding whatever
    /// a previous refinement left. One refinement runs against ONE field and ONE reading, and an
    /// answer about an edge is an answer about that field, so a memo may never outlive the call
    /// that filled it.
    pub fn forget_clean_edges(&mut self) {
        self.answered_clean = Some(HashSet::new());
    }

    /// Whether a rule has already found this edge clean and it has not been split since - the mesh
    /// keeps the mark because the mesh is what knows when an edge stops existing.
    pub fn is_answered_clean(&self, a: i32, b: i32) -> bool {
        self.answered_clean
            .as_ref()
            .is_some_and(|clean| clean.contains(&Self::key(a, b)))
    }

    /// Marks an edge clean until it is split (nothing while no memo is running).
    pub fn answer_clean(&mut self, a: i32, b: i32) {
        if let Some(clean) = &mut self.answered_clean {
            clean.insert(Self::key(a, b));
        }
    }

    /// Stands triangle `index` on its three edges.
    pub fn attach(&mut self, index: i32, t: LevelLineTriangle) {
        self.attach_edge(index, t.a, t.b);
        self.attach_edge(index, t.b, t.c);
        self.attach_edge(index, t.c, t.a);
    }

    /// Stands triangle `index` on edge `a b`.
    pub fn attach_edge(&mut self, index: i32, a: i32, b: i32) {
        self.edge_triangles
            .entry(Self::key(a, b))
            .or_insert_with(|| Vec::with_capacity(2))
            .push(index);
    }

    /// Takes triangle `index` off its three edges.
    pub fn detach(&mut self, index: i32, t: LevelLineTriangle) {
        self.detach_edge(index, t.a, t.b);
        self.detach_edge(index, t.b, t.c);
        self.detach_edge(index, t.c, t.a);
    }

    /// Takes triangle `index` off edge `a b` (its first entry only); an edge left
    /// standing on nothing leaves the mesh.
    pub fn detach_edge(&mut self, index: i32, a: i32, b: i32) {
        let key = Self::key(a, b);
        if let Some(list) = self.edge_triangles.get_mut(&key) {
            if let Some(at) = list.iter().position(|&t| t == index) {
                list.remove(at);
            }

            if list.is_empty() {
                self.edge_triangles.remove(&key);
            }
        }
    }
}
