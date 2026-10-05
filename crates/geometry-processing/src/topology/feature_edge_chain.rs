//! Runs of feature edges: the tangent run, the collinear run and the crease run a picked edge
//! extends into. Ports `Topology/FeatureEdgeChain.cs` (`ChainStopReason`, `ChainEnd`,
//! `ChainEdge`, `FeatureEdgeChain`). Walks a [`FeatureEdgeSet`]; the bevel sweeps the result.

use std::collections::HashSet;

use vector_math::{math_helper, Vector3};

use super::error::TopologyError;
use super::feature_edge::FeatureEdge;
use super::feature_edge_set::FeatureEdgeSet;

/// Why a chain stopped growing at one of its ends.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, Default)]
pub enum ChainStopReason {
    /// The end vertex offers no single way on. For a tangent walk that is three or more feature
    /// edges meeting, however smoothly any one of them continues; for a collinear run it is a
    /// vertex that carries other feature edges but nothing that continues the crease, or one where
    /// two things continue it and neither can be told to be the continuation. Either way the
    /// vertex becomes a corner-treatment site.
    #[default]
    Corner,
    /// The one edge that continues from the end vertex turns or twists too far to be part of the
    /// same tangent run, or there is no edge there at all.
    NoCandidate,
    /// The chain came back to where it started; it is a closed loop and has no ends.
    Loop,
}

/// Where a chain stops, and why.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub struct ChainEnd {
    /// Why the walk stopped here.
    pub reason: ChainStopReason,
    /// The mesh vertex the walk stopped at. On a closed chain this is the vertex the loop was
    /// entered at, which is not a stopping point at all.
    pub vertex: i32,
}

impl ChainEnd {
    pub fn new(reason: ChainStopReason, vertex: i32) -> Self {
        ChainEnd { reason, vertex }
    }
}

/// One feature edge as it is traversed by a chain, oriented along the walk.
///
/// A [`FeatureEdge`] is stored in the orientation its own Face0 winds it, which has nothing to do
/// with the direction a chain happens to walk it. This wrapper flips that where needed so a chain
/// reads as one continuous run: `from_vertex` of each edge is the `to_vertex` of the one before
/// it, and `direction` always points the way the chain is going. `left_face` is the face lying to
/// the left of that walk direction seen from outside the solid, and it stays the same physical
/// side of the chain from edge to edge - that is what lets the sweep carry a profile frame along
/// without it flipping over mid-chain.
#[derive(Debug, Clone, Copy, PartialEq, Default)]
pub struct ChainEdge {
    /// The index of this edge in the [`FeatureEdgeSet`] the chain was walked over.
    pub edge_index: i32,
    /// The edge itself, in its own stored orientation.
    pub edge: FeatureEdge,
    /// Whether the chain walks this edge against its stored direction.
    pub is_reversed: bool,
}

impl ChainEdge {
    pub fn new(edge_index: i32, edge: FeatureEdge, is_reversed: bool) -> Self {
        ChainEdge {
            edge_index,
            edge,
            is_reversed,
        }
    }

    /// The vertex the chain enters this edge at.
    pub fn from_vertex(&self) -> i32 {
        if self.is_reversed {
            self.edge.end_vertex
        } else {
            self.edge.start_vertex
        }
    }

    /// The vertex the chain leaves this edge at.
    pub fn to_vertex(&self) -> i32 {
        if self.is_reversed {
            self.edge.start_vertex
        } else {
            self.edge.end_vertex
        }
    }

    /// The position of `from_vertex`.
    pub fn from(&self) -> Vector3 {
        if self.is_reversed {
            self.edge.end
        } else {
            self.edge.start
        }
    }

    /// The position of `to_vertex`.
    pub fn to(&self) -> Vector3 {
        if self.is_reversed {
            self.edge.start
        } else {
            self.edge.end
        }
    }

    /// The unit direction the chain travels along this edge.
    pub fn direction(&self) -> Vector3 {
        if self.is_reversed {
            -self.edge.direction
        } else {
            self.edge.direction
        }
    }

    /// The face on the left of the walk direction (seen from outside the solid), which is the
    /// face whose winding runs this edge the same way the chain does.
    pub fn left_face(&self) -> i32 {
        if self.is_reversed {
            self.edge.face1
        } else {
            self.edge.face0
        }
    }

    /// The face on the right of the walk direction.
    pub fn right_face(&self) -> i32 {
        if self.is_reversed {
            self.edge.face0
        } else {
            self.edge.face1
        }
    }

    /// The outward normal of `left_face`.
    pub fn left_normal(&self) -> Vector3 {
        if self.is_reversed {
            self.edge.normal1
        } else {
            self.edge.normal0
        }
    }

    /// The outward normal of `right_face`.
    pub fn right_normal(&self) -> Vector3 {
        if self.is_reversed {
            self.edge.normal0
        } else {
            self.edge.normal1
        }
    }

    /// The same edge walked the other way, which swaps its ends and its left and right faces.
    pub fn reversed(&self) -> ChainEdge {
        ChainEdge::new(self.edge_index, self.edge, !self.is_reversed)
    }
}

/// What a walk asks of the next edge at a vertex.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum WalkRule {
    /// The tangent run: only ever on through a joint of exactly two feature edges, and only while
    /// the surface keeps turning the same way as the edge just walked.
    Tangent,
    /// The collinear run: on through however many feature edges meet, as long as exactly one of
    /// them carries the seed's own crease straight on.
    Collinear,
    /// The crease run: the tangent walk's question asked at a vertex that may carry any number of
    /// other feature edges, and answered by the best of whatever continues rather than refused
    /// for being crowded. See [`FeatureEdgeChain::propagate_crease`].
    Crease,
}

/// A run of feature edges: the "edge loop" a CAD package extends a picked edge into, walked from
/// a seed edge in both directions for as long as the surface keeps going the same way.
///
/// Every edge in `edges` is oriented along the walk, so consecutive edges meet head to tail. The
/// sweep depends on that: it carries a cross-section frame from segment to segment and miters
/// the joints, and an edge stored the other way round would flip the frame and turn the tool
/// inside out at that joint.
///
/// The walk only sees the edges in the [`FeatureEdgeSet`] it is given, so the set's flat
/// tolerance decides what counts as a corner. On a tessellated cylinder built at the default
/// half-degree tolerance every wall facet edge is a feature edge, so each rim vertex has three of
/// them and a rim chain stops immediately; build the set at a tolerance that reads the wall as one
/// surface (10 degrees, say) and the same seed walks the whole rim.
#[derive(Debug, Clone, PartialEq)]
pub struct FeatureEdgeChain {
    edges: Vec<ChainEdge>,
    is_closed: bool,
    start: ChainEnd,
    end: ChainEnd,
}

impl FeatureEdgeChain {
    /// The default tolerance for both tests, in degrees. It is the plan's resolved curve
    /// tolerance (Blender's auto-smooth default): loose enough to walk a tessellated rim, tight
    /// enough to stop at a corner.
    pub const DEFAULT_TANGENT_ANGLE_DEGREES: f64 = 30.0;

    /// How far a collinear run may bend, in degrees: the same angle feature detection uses to
    /// decide an edge is not an edge at all ([`FeatureEdge::default_flat_angle_degrees`], about
    /// 0.512). One number rather than the tolerance the selection was picked at: a pick's flat
    /// tolerance is deliberately loosened to read a tessellated wall as one surface, and
    /// measuring collinearity by it would let a run swallow a facet or two of a rim. Straight is
    /// straight whatever else the part is being read loosely for.
    pub fn default_collinear_angle_degrees() -> f64 {
        FeatureEdge::default_flat_angle_degrees()
    }

    /// The edges of the chain in walk order, each oriented along the walk.
    pub fn edges(&self) -> &[ChainEdge] {
        &self.edges
    }

    /// Whether the chain closes on itself. A closed chain sweeps all the way around and welds;
    /// an open one needs end treatment.
    pub fn is_closed(&self) -> bool {
        self.is_closed
    }

    /// Where and why the chain stops at its beginning.
    pub fn start(&self) -> ChainEnd {
        self.start
    }

    /// Where and why the chain stops at its end.
    pub fn end(&self) -> ChainEnd {
        self.end
    }

    /// The number of edges in the chain.
    pub fn count(&self) -> usize {
        self.edges.len()
    }

    /// Extends a seed feature edge into a tangent chain at the default tolerance.
    pub fn propagate(
        features: &FeatureEdgeSet,
        seed_edge_index: i32,
    ) -> Result<FeatureEdgeChain, TopologyError> {
        Self::propagate_with(
            features,
            seed_edge_index,
            Self::DEFAULT_TANGENT_ANGLE_DEGREES,
            Self::DEFAULT_TANGENT_ANGLE_DEGREES,
        )
    }

    /// Extends a seed feature edge into a tangent chain, walking both ways from the seed.
    ///
    /// `direction_tolerance_degrees` is how far the edge direction may turn from one edge to the
    /// next and still be the same run (a 64 sided cylinder rim steps 5.625 degrees at a time).
    /// `face_tolerance_degrees` is how far each of the two surfaces along the chain may turn from
    /// one edge to the next; the faces are matched by which side of the walk they are on, so a
    /// chain never continues onto an edge that has the same two surfaces swapped over.
    pub fn propagate_with(
        features: &FeatureEdgeSet,
        seed_edge_index: i32,
        direction_tolerance_degrees: f64,
        face_tolerance_degrees: f64,
    ) -> Result<FeatureEdgeChain, TopologyError> {
        walk(
            features,
            seed_edge_index,
            direction_tolerance_degrees,
            face_tolerance_degrees,
            WalkRule::Tangent,
        )
    }

    /// Extends a seed feature edge into the COLLINEAR run it lies in - the whole of one straight
    /// crease - at [`Self::default_collinear_angle_degrees`].
    pub fn propagate_collinear(
        features: &FeatureEdgeSet,
        seed_edge_index: i32,
    ) -> Result<FeatureEdgeChain, TopologyError> {
        Self::propagate_collinear_with(
            features,
            seed_edge_index,
            Self::default_collinear_angle_degrees(),
        )
    }

    /// Extends a seed feature edge into the collinear run it lies in: every mesh edge that
    /// carries the same straight crease on through a vertex, walked both ways from the seed.
    ///
    /// This is what a plain edge pick means in a CAD package. A mesh splits one crease into as
    /// many edges as the triangulation needed - a boolean drops a vertex wherever another solid
    /// touched, and leaves that solid's feature edges standing at it - so unlike
    /// [`Self::propagate_with`], extra feature edges at a vertex do NOT stop the run; what
    /// matters is whether exactly one of them carries the crease on.
    ///
    /// Every candidate is measured against the SEED rather than against the edge the walk
    /// arrived on, so a long run cannot curve away a fraction of a degree at a time - which also
    /// means the run can never close on itself.
    ///
    /// It needs a WELDED mesh: edges meet here by shared vertex INDEX, so a crease whose halves
    /// carry their own copies of the vertex between them is two creases to this walk.
    ///
    /// `angle_tolerance_degrees` is how far a candidate may differ from the seed - in direction,
    /// and in each of the two surfaces along the run - and still be the same straight crease.
    pub fn propagate_collinear_with(
        features: &FeatureEdgeSet,
        seed_edge_index: i32,
        angle_tolerance_degrees: f64,
    ) -> Result<FeatureEdgeChain, TopologyError> {
        walk(
            features,
            seed_edge_index,
            angle_tolerance_degrees,
            angle_tolerance_degrees,
            WalkRule::Collinear,
        )
    }

    /// Extends a seed feature edge into the CREASE it lies in, across a rounded ridge: the
    /// tangent walk's test applied at vertices that carry other feature edges as well as this
    /// crease's own.
    ///
    /// This is what a concave crease becomes once the convex edges around it have been rounded:
    /// it runs onto the round's facets, bends by one facet step at each, and comes down the far
    /// side as what used to be a second crease. Every vertex along the round carries the
    /// cylinder's own facet edge too, which a tangent walk reads as a corner; so this rule reads a
    /// crowded vertex the way the collinear walk does and answers it with the tangent walk's own
    /// test against the edge just walked. A candidate must fold the same way, and where more than
    /// one continues the straightest is taken rather than the run refused: on a round there is no
    /// ambiguity to protect against, only tessellation.
    ///
    /// `direction_tolerance_degrees` is how far the crease may turn from one edge to the next
    /// (the looser of the two); `face_tolerance_degrees` how far either surface along it may turn,
    /// the test that actually says where the crease ends.
    pub fn propagate_crease(
        features: &FeatureEdgeSet,
        seed_edge_index: i32,
        direction_tolerance_degrees: f64,
        face_tolerance_degrees: f64,
    ) -> Result<FeatureEdgeChain, TopologyError> {
        walk(
            features,
            seed_edge_index,
            direction_tolerance_degrees,
            face_tolerance_degrees,
            WalkRule::Crease,
        )
    }
}

fn walk(
    features: &FeatureEdgeSet,
    seed_edge_index: i32,
    direction_tolerance_degrees: f64,
    face_tolerance_degrees: f64,
    rule: WalkRule,
) -> Result<FeatureEdgeChain, TopologyError> {
    if seed_edge_index < 0 || seed_edge_index as usize >= features.count() {
        return Err(TopologyError::EdgeIndexOutOfRange {
            index: seed_edge_index,
            edge_count: features.count(),
        });
    }

    let mut walk = vec![ChainEdge::new(
        seed_edge_index,
        features.edges()[seed_edge_index as usize],
        false,
    )];

    let mut used = HashSet::from([seed_edge_index]);

    let seed = walk[0];
    let (end_stop, closed) = grow(
        features,
        &mut walk,
        &mut used,
        direction_tolerance_degrees,
        face_tolerance_degrees,
        rule,
        seed,
    );

    if closed {
        // A loop has no ends. Both are reported against the vertex the seed was entered at,
        // which is just where the walk happened to start going round.
        let entry_vertex = walk[0].from_vertex();
        return Ok(FeatureEdgeChain {
            edges: walk,
            is_closed: true,
            start: ChainEnd::new(ChainStopReason::Loop, entry_vertex),
            end: ChainEnd::new(ChainStopReason::Loop, entry_vertex),
        });
    }

    // The other direction is the same walk over a chain that has been turned around, so it runs
    // through the same code and is flipped back afterwards.
    //
    // The backward walk's own closed flag is discarded under two of the three rules. A tangent
    // walk explores a simple path or a cycle and both tolerance tests are symmetric under
    // reversal, so the backward walk retraces the forward one's refusal; a collinear run cannot
    // close at all. A CREASE walk has neither guarantee - it takes the straightest candidate
    // against the edge just walked, so traversed the other way it can come round onto the run -
    // and that is honoured below so a closed crease is reported closed whichever edge was seeded.
    let backward_seed = walk[0].reversed();
    let mut backward = vec![backward_seed];
    let (start_stop, closed_backward) = grow(
        features,
        &mut backward,
        &mut used,
        direction_tolerance_degrees,
        face_tolerance_degrees,
        rule,
        backward_seed,
    );

    // drop the seed, which is already in the forward walk, and put the rest on the front
    backward.remove(0);
    backward.reverse();
    for edge in backward.iter_mut() {
        *edge = edge.reversed();
    }

    walk.splice(0..0, backward);

    if closed_backward
        && rule == WalkRule::Crease
        && walk[walk.len() - 1].to_vertex() == walk[0].from_vertex()
    {
        // A true cycle and not a lasso. Grow stops on ANY edge it has already used, which at a
        // crowded vertex may be one in the middle of the run - and a run that comes back onto
        // its own middle is not a loop, so the ends test is what tells the two apart.
        let meeting_vertex = walk[0].from_vertex();

        return Ok(FeatureEdgeChain {
            edges: walk,
            is_closed: true,
            start: ChainEnd::new(ChainStopReason::Loop, meeting_vertex),
            end: ChainEnd::new(ChainStopReason::Loop, meeting_vertex),
        });
    }

    Ok(FeatureEdgeChain {
        edges: walk,
        is_closed: false,
        start: start_stop,
        end: end_stop,
    })
}

/// Walks forward from the end of a partial chain until something stops it, appending as it
/// goes. Returns where it stopped and whether it closed onto an edge already in the chain.
fn grow(
    features: &FeatureEdgeSet,
    walk: &mut Vec<ChainEdge>,
    used: &mut HashSet<i32>,
    direction_tolerance_degrees: f64,
    face_tolerance_degrees: f64,
    rule: WalkRule,
    seed: ChainEdge,
) -> (ChainEnd, bool) {
    loop {
        let current = walk[walk.len() - 1];
        let vertex = current.to_vertex();
        let reference = if rule == WalkRule::Collinear {
            seed
        } else {
            current
        };

        let next = match try_find_next(
            features,
            current,
            reference,
            vertex,
            direction_tolerance_degrees,
            face_tolerance_degrees,
            rule,
        ) {
            Ok(next) => next,
            Err(reason) => return (ChainEnd::new(reason, vertex), false),
        };

        if used.contains(&next.edge_index) {
            // walked all the way round onto an edge already in the chain
            return (ChainEnd::new(ChainStopReason::Loop, vertex), true);
        }

        walk.push(next);
        used.insert(next.edge_index);
    }
}

/// The one feature edge that carries the chain on through a vertex, or why there is none.
/// `current` is the edge the walk arrived on; `reference` is what a candidate has to match: the
/// arriving edge for a tangent or crease walk, the seed for a collinear run.
fn try_find_next(
    features: &FeatureEdgeSet,
    current: ChainEdge,
    reference: ChainEdge,
    vertex: i32,
    direction_tolerance_degrees: f64,
    face_tolerance_degrees: f64,
    rule: WalkRule,
) -> Result<ChainEdge, ChainStopReason> {
    let incident = features.edges_at_vertex(vertex);

    // For a tangent walk, three or more feature edges meeting is a corner however smoothly any
    // one of them continues: there is no single way on, and the sweep has to be capped here
    // rather than guess.
    if rule == WalkRule::Tangent && incident.len() > 2 {
        return Err(ChainStopReason::Corner);
    }

    let mut next: Option<ChainEdge> = None;

    // The crease rule keeps the best candidate rather than the first.
    let mut best_turn = f64::MAX;
    for &candidate_index in incident {
        if candidate_index == current.edge_index {
            continue;
        }

        let candidate = features.edges()[candidate_index as usize];

        // orient the candidate so the chain leaves the shared vertex along it
        let oriented = ChainEdge::new(candidate_index, candidate, candidate.end_vertex == vertex);

        if rule == WalkRule::Tangent {
            // The only other edge at a two-edge joint; it is measured below.
            next = Some(oriented);
            break;
        }

        if rule == WalkRule::Crease {
            // A facet edge of the round crosses the crease at every vertex along it and folds the
            // other way; nothing that folds the other way is this crease.
            if candidate.convexity != current.edge.convexity
                || !continues(
                    reference,
                    oriented,
                    direction_tolerance_degrees,
                    face_tolerance_degrees,
                )
            {
                continue;
            }

            let turn = angle_between_degrees(reference.direction(), oriented.direction());

            if turn < best_turn {
                best_turn = turn;
                next = Some(oriented);
            }

            continue;
        }

        if !continues(
            reference,
            oriented,
            direction_tolerance_degrees,
            face_tolerance_degrees,
        ) {
            continue;
        }

        if next.is_some() {
            // Two edges both carry the crease straight on. Which of them the pick meant cannot be
            // told from the geometry, and running onto either would be a guess.
            return Err(ChainStopReason::Corner);
        }

        next = Some(oriented);
    }

    let Some(next) = next else {
        // Nothing continues. A vertex with other feature edges on it is still a corner to cap
        // against; one with none is simply where the crease runs out.
        return Err(if incident.len() > 2 {
            ChainStopReason::Corner
        } else {
            ChainStopReason::NoCandidate
        });
    };

    // Only the tangent leg has anything left to test: a collinear candidate got here by passing.
    if rule == WalkRule::Tangent
        && !continues(
            reference,
            next,
            direction_tolerance_degrees,
            face_tolerance_degrees,
        )
    {
        return Err(ChainStopReason::NoCandidate);
    }

    Ok(next)
}

/// Whether one edge carries a run on from another: the direction holds, and so does each of the
/// two surfaces, matched by which side of the walk it is on.
fn continues(
    from: ChainEdge,
    to: ChainEdge,
    direction_tolerance_degrees: f64,
    face_tolerance_degrees: f64,
) -> bool {
    angle_between_degrees(from.direction(), to.direction()) <= direction_tolerance_degrees
        && angle_between_degrees(from.left_normal(), to.left_normal()) <= face_tolerance_degrees
        && angle_between_degrees(from.right_normal(), to.right_normal()) <= face_tolerance_degrees
}

/// The angle between two unit vectors in degrees, clamped against the rounding that can push a
/// dot product just outside acos's domain.
fn angle_between_degrees(a: Vector3, b: Vector3) -> f64 {
    // C# Math.Clamp: a NaN dot stays NaN, as f64::clamp does.
    let dot = a.dot(b).clamp(-1.0, 1.0);

    math_helper::radians_to_degrees(dot.acos())
}
