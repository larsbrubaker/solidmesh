//! The void each 5.3 corner opens, read from the PART and remembered for the whole rebuild: the
//! solid a bead is allowed to keep material in past its own last vertex, in place of the wedge that
//! used to run out there with it. [`super::bevel_bead_clip`] takes the cones this hands out as its
//! corner pieces; the cone itself is [`super::bevel_corner_patch_mesher::try_build_void_cone`] over
//! the ring [`super::bevel_corner_face_ring::try_order_at_vertex`] reads.
//!
//! **What went wrong without it.** A bead and its void wedge are swept along one chain, off one
//! set of frames, through one set of mitres and onto one set of ENDS, and that shared construction
//! is the whole argument for clipping each bead to its own wedge alone: every millimetre the bead
//! adds lies inside its own wedge by construction (`BevelBeadClip`). At a 5.3 corner the shared
//! ends turn that identity inside out. Both solids run `max(size, tangentDistance)` past the
//! vertex - deliberately, since that overshoot is what makes two beads MEET - but out there the
//! wedge has stopped describing void the run opened and started describing void the run WOULD have
//! opened had the surface gone on. On a wall thinner than the overshoot the surface has not gone
//! on, and `bead within (part + wedge)` hands the bead the far side of the wall: two of the three
//! creases of a 24 mm block with 4 mm walls came back with bounds at `(0, -4, -4)`.
//!
//! **Why the wedge and not the overshoot.** Shortening the overshoot only shortens the spike - the
//! direction still leaves a thin wall, and the two beads stop meeting, which is the corner the
//! overshoot exists for. Unioning the PARTNER's wedge into the clip goes the wrong way outright: it
//! widens the keep-set, and the leak is inside this run's OWN wedge past the vertex. So the bead
//! keeps its overshoot untouched and the wedge is capped square at the vertex
//! (`BevelChainEnds.SquaredAt`), which leaves exactly the corner region to account for - and the
//! corner's own void is what accounts for it.
//!
//! **The corner's own void.** The same `K_v` a fill patch travels with (`BevelCornerPatches`): the
//! intersection of the outward half spaces of the faces meeting at the vertex, bounded by a ball.
//! Where the void at that corner is locally that cone, everything the bead has past the vertex is
//! either part or `K_v`, so the pair of them together clip nothing the corner is owed; where the
//! wall runs out first, the air past it is in neither and is trimmed.
//!
//! **Convexity is not read, and the invariant that makes that safe.** The ring is walked over
//! every crease at the vertex whether it folds in or out, so a MIXED vertex - two concave creases
//! and the unselected convex corner between them, which is exactly a square boss's foot and exactly
//! the commonest 5.3 shape there is - builds a cone like any other. It is still a subset of the
//! void: `K_v` is an intersection of OUTWARD half spaces, and a point on the outward side of every
//! face at a vertex is outside the solid there whichever way each pair of those faces folds. So a
//! mixed corner can only ever leave the bead LESS than the void it opens, never more - the failure
//! direction is a notch, not a leak - and on an ordinary boss it takes nothing at all, because the
//! whole of each bead's overshoot lies in the quadrant the two walls and the plate cut out
//! (`BevelBeadEndCapTests` measures both bosses).
//!
//! **Why the PART's feature set and not the group's.** Every group carries its own
//! `FeatureEdgeSet`, built at whatever flat angle that group's reading was resolved with
//! (`BevelEdgeGroupResolver` keeps one per distinct angle), and a coarse one is the wrong instrument
//! for this question twice over. It MISSES creases: pick a 48 sided boss's foot at a 20 degree wall
//! tolerance and the 7.5 degree facet step between the two walls is not an edge at all, so the
//! shared vertex looks like a plain two-edge corner, the ring has nothing to close over, and an
//! ordinary boss fillet that leaks nothing would be refused and apologised for on every rebuild. And
//! it makes the answer depend on the PICK: two runs meeting at one vertex through groups read at
//! different tolerances would disagree about what the corner is. The void a corner opens is a fact
//! about the part, so it is read at `FeatureEdge::default_flat_angle_degrees` off the working mesh,
//! once, and the memo below is keyed on the vertex and the reach alone because there is nothing
//! else left for it to depend on.
//!
//! **How far the ball reaches.** The bead reaches at most the overshoot ALONG the run and its cross
//! section's reach ACROSS it, so their sum bounds everything it has out past the vertex, and the
//! containment margin puts the ball strictly outside that rather than touching it. Erring long is
//! free - `K_v` is a cone of the corner's own half spaces, so a longer one admits only more of the
//! same void, and the bead is not out there to keep it - while erring short would cut into the
//! corner the two beads meet in. The reach is part of the key for that reason, and the memo is a
//! COST saving rather than a promise of sharing: two runs meeting at one vertex whose wedges differ
//! want different balls and get them, and two whose reaches agree only to within a rounding will
//! build one cone each. Nothing depends on which of those happens - two cones of nearly equal
//! length clip identically - so the key is left exact rather than fuzzed to underwrite a claim the
//! geometry never needed.
//!
//! **Refusals.** A corner whose faces do not close a ring, or whose half spaces bound no convex
//! cone (a saddle), or whose wedge has folded shut so that neither the overshoot nor the reach is a
//! number, comes back `None` - and that is not a failure. The end keeps the wedge it always had,
//! swept along the overshoot, which is exactly what shipped before this existed, and the bead is
//! never dropped.
//!
//! **How this cone is asked for, and why that took a plan item of its own.** The clip these feed is
//! `bead within (part + squared wedge + K_v)`, and the keep-set cannot be had by unioning those
//! three: this cone's lateral walls ARE the part's own faces at that vertex, exactly and by
//! construction (`BevelCornerPatchMesher.TryBuildVoidCone`), the wedge's two walls are two of the
//! same faces, and three solids touching over surfaces of no thickness came back self-intersecting
//! on every boss and post measured. `BevelBeadClip` takes it in overlapping PIECES instead, one of
//! them the corner's, and takes that one ONCE for both beads meeting here - which is the second
//! reason this type memoizes, beyond the cost: a cone handed to two beads separately would cut both
//! of them with one surface and duplicate it. The cone is handed out as an `Rc` for that reason:
//! the bead clip groups the beads by the cone's identity.
//!
//! A refusal is still possible, and the answer to one is still silent. The chains fall back to the
//! clip that shipped before this existed - the wedge swept with its overshoot, no cone
//! (`BevelTool.FallbackWedge`) - which is a deliberate trade rather than an oversight: the corner
//! void changes nothing on a post or a boss in the first place, because everything the bead has
//! past such a vertex is either material or the corner's own void, so announcing it there would put
//! "could not be trimmed" on every rebuild of an ordinary boss fillet and teach the user to ignore
//! the one sentence that matters on a thin wall. What it costs is the case where a thin wall and a
//! kernel refusal meet at the same corner, which nothing in the corpus reaches: there the leak would
//! come back unannounced.
//!
//! One instance per rebuild, built beside the working mesh and handed down to every group. It is
//! not thread safe and does not need to be: the rebuild resolves and sweeps its groups in order on
//! one thread, and the booleans that do run in a batch are all downstream of every tool being
//! built. The memo and the lazily read feature set are interior (`RefCell`/`OnceCell`), so every
//! group can ask through a shared reference.

use std::cell::{OnceCell, RefCell};
use std::rc::Rc;
use std::sync::Arc;

use indexmap::IndexMap;
use polygon_mesh::{Mesh, MeshEdgeGraph};
use vector_math::Vector3;

use super::bevel_corner_face_ring::try_order_at_vertex;
use super::bevel_corner_patch_mesher::try_build_void_cone;
use super::bevel_edge_tool_builder as builder;
use crate::topology::{ChainEdge, FeatureEdge, FeatureEdgeSet};

/// The cones built so far by (vertex, reach), the reach keyed by its bits under value equality
/// ([`reach_key`]); a refusal is remembered as `None`.
type BuiltCones = IndexMap<(i32, u64), Option<Rc<Mesh>>>;

/// The corners of one part, read on demand and remembered.
#[derive(Debug)]
pub struct BevelCornerVoids {
    part: Arc<Mesh>,
    graph: Option<Rc<MeshEdgeGraph>>,
    /// The cones read so far, by vertex and reach.
    built: RefCell<BuiltCones>,
    features: OnceCell<FeatureEdgeSet>,
}

impl BevelCornerVoids {
    /// Reads the corners of one part.
    ///
    /// `part` is the working mesh every tool of this rebuild is measured on. `graph` is that
    /// mesh's own edge graph where the caller already has one, which is the expensive half of the
    /// scan. `None` builds one on first use - and only on first use, since a rebuild with no 5.3
    /// corner in it never asks a question at all.
    pub fn new(part: Arc<Mesh>, graph: Option<Rc<MeshEdgeGraph>>) -> Self {
        BevelCornerVoids {
            part,
            graph,
            built: RefCell::new(IndexMap::new()),
            features: OnceCell::new(),
        }
    }

    /// The part's creases at the flat angle this reading is defined at, built once and only if
    /// asked.
    fn features(&self) -> &FeatureEdgeSet {
        self.features.get_or_init(|| match &self.graph {
            Some(graph) => FeatureEdgeSet::create_with_graph(
                &self.part,
                graph,
                FeatureEdge::default_flat_angle_degrees(),
            ),
            None => FeatureEdgeSet::create_with_angle(
                &self.part,
                FeatureEdge::default_flat_angle_degrees(),
            ),
        })
    }

    /// The void one 5.3 corner opens, or `None` where that corner cannot be read.
    ///
    /// `corner` is the vertex this end overshoots into another selected run at
    /// (`BevelChainEnds.StartOvershootCorner`), or `None` where the end is not that kind of end and
    /// there is nothing to read. Numbered in the MESH's vertices, which is what makes it readable
    /// here at all: a group's feature set and this one are two classifications of the one edge
    /// graph. `segment` is the end's own segment, which sizes how far past the vertex the bead can
    /// be; `position` that end's vertex, in the part's own space; `size` the size every tool of the
    /// operation is swept at; `segments` the segment count the cone's cap is fanned at;
    /// `round_profile` false for a chamfer, whose overshoot is its own setback.
    pub fn try_read(
        &self,
        corner: Option<i32>,
        segment: &ChainEdge,
        position: Vector3,
        size: f64,
        segments: i32,
        round_profile: bool,
    ) -> Option<Rc<Mesh>> {
        let corner = corner?;

        let wedge_angle = builder::profile_wedge_angle_degrees(&segment.edge);
        let overshoot = builder::overshoot(
            size,
            builder::tangent_distance(size, wedge_angle, round_profile),
        );
        let reach = builder::cross_section_reach(size, wedge_angle);

        if overshoot.is_infinite() || reach.is_infinite() {
            // A wedge folded shut has no reach anyone can put a number on, so there is no ball to
            // bound the cone with. BevelChainSweep refuses such a corner outright in any case.
            return None;
        }

        let length = builder::CONTAINMENT_MARGIN * (overshoot + reach);
        let key = (corner, reach_key(length));

        if let Some(already) = self.built.borrow().get(&key) {
            return already.clone();
        }

        let cone = try_order_at_vertex(Some(self.features()), corner).and_then(|outward_normals| {
            try_build_void_cone(position, &outward_normals, length, segments, false)
        });
        let cone = cone.map(Rc::new);

        // Remembered even when it is None: a corner that cannot be read cannot be read by the
        // other run meeting there either, and the walk that says so is not free.
        self.built.borrow_mut().insert(key, cone.clone());

        cone
    }
}

/// A dictionary key by value: 0.0 and -0.0 alike, every NaN alike.
fn reach_key(value: f64) -> u64 {
    if value == 0.0 {
        0
    } else if value.is_nan() {
        f64::NAN.to_bits()
    } else {
        value.to_bits()
    }
}
