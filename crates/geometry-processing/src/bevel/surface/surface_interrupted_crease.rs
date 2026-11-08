//! Whether the selection is two pieces of ONE crease that something standing on it interrupts,
//! rather than two creases that each end where they stop. A PLAN-stage reading over every chain of
//! the pass at once, asked after [`super::surface_chain_opens_at_the_junction::open`]; its readings
//! are the boolean leg's own ([`crate::bevel::bevel_interrupted_crease`]), shared rather than
//! copied.
//!
//! **The shape.** "Center Flows Through.mcx" is the nose-in-notch part with the notch's concave
//! centre crease picked both sides of the knife-edge nose welded over it - the piece below the nose
//! and the piece above. The two slopes the crease is made of run the whole height of the part and go
//! on making the same crease behind the nose; only the mesh's edge stops. The Onshape control
//! answers with ONE cylinder of the fillet's radius the whole height, flat capped on the floor and
//! the ceiling alone, with the nose kept where it stands proud of it.
//!
//! **Why a pair the feature SPANS is not named, and builds.** What the span reading proves is that
//! the two strips DO NOT OVERLAP: every rail of each piece's profile meets its own end face before
//! the other piece's, so the two are two ordinary ends each dying on one plane, with no part of
//! either drawn where the other already is. Each piece read on its own already ends that way - the
//! end plane through the corner and the two seam ends IS the nose's face, the rails run past the
//! chain end to it and the face is trimmed to the ring (`SurfaceEndGivesUpTheSection`) - so the pair
//! needs no construction it does not have, and naming it would refuse a shape that builds to within
//! a chord of its control.
//!
//! **What is left under the name: a bead that would pass UNDER the feature.** That answer holds
//! only while the feature SPANS the strip - while, for every point of the profile, the rail through
//! it meets the NEAR piece's end face before the far piece's. Where some rail meets the far face
//! first the two strips would overlap inside the feature, which is one joined run with a notch in
//! the middle of it and an exit where its ridge crosses the band: constructions this does not have.
//! On the fixture that starts between 7 and 7.5 mm, the feet standing further from the crease than
//! the nose's own root does (the ridge ends are 7.071 away), and the refusal says so.
//!
//! **The span is read in closed form, not sampled.** A rail's crossing parameter with a plane is
//! AFFINE in the profile point, so the difference of the two is affine over the arc and its minimum
//! is at a foot or at the one arc point facing down the difference's own gradient. Three points are
//! measured, never a sweep (see `the_feature_spans_the_strip`).
//!
//! **The readings.** Something here CLOSES OVER the run's own continuation and nothing FALLS AWAY
//! from it (`Leaning::closes_over_the_run`, fed every face at the vertex that is not one of the
//! run's own two) - which holds off a crease ending at the foot of something STANDING on the part;
//! both of the end's own two faces carry on past the vertex, matched by NORMAL
//! (`both_own_faces_carry_on`); no unselected edge carries the crease's own line on past the vertex
//! (`crease_carries_on`); and the crease RESUMES in another open concave run of this pass
//! (`crease_resumes_beyond`) that stands on THE SAME TWO SURFACES (`the_same_two_regions`), which
//! is what makes this one crease rather than two that happen to be collinear. The region test
//! answers two blocks on a plate with ordinary plate between them however much plate there is,
//! which a bound on the distance would not.
//!
//! **The edge of the rule, which is deliberate.** A feature whose footprint spans one of the
//! landing surfaces EDGE TO EDGE cuts that surface in two, and then the crease beyond it rides a
//! region this one never touches: the pair goes silent and builds as two capped strips. And there
//! is no bound on the distance, though the boolean leg has one: that leg RUNS ON and overshoots into
//! the interruption, so how far it reaches is a real limit; this one JOINS, and a nose a hundred
//! millimetres long standing on a crease that resumes beyond it is still one interrupted crease.

use std::collections::HashSet;

use solidmesh_math::{ieee_math, math_helper, Vector3};

use super::SurfaceBevelPlan;
use crate::bevel::bevel_edge_tool_builder;
use crate::bevel::bevel_interrupted_crease::{self, Leaning};
use crate::bevel::{BevelAdvisory, BevelAdvisoryCode, BevelSweptChain};
use crate::topology::{ChainEdge, EdgeConvexity, FaceRegionCache, FeatureEdgeSet};

/// The refusal for a selection whose two pieces of one
/// crease would have to run UNDER what interrupts it, or `None` where every end of it is an end the
/// strip can die on - which is every end a feature spanning the strip closes, and every end that is
/// really an end.
///
/// Named at the FIRST such end in the planned order - chains in the order the plans are read in,
/// and within a chain the end it starts at before the end it stops at. One refusal for the whole
/// pass and one place on it. `chains` is every chain of the pass, joined and opened; `features` the
/// mesh's feature edges at the pick's own flat tolerance; `regions` the mesh read as face regions at
/// the operation's own bend threshold; `size` the fillet radius or chamfer setback, which decides
/// as much as the shape; `round_profile` whether the profile is an arc or the chamfer's chord.
pub fn try_read(
    chains: &[BevelSweptChain],
    features: &FeatureEdgeSet,
    regions: &FaceRegionCache,
    size: f64,
    round_profile: bool,
) -> Option<BevelAdvisory> {
    if chains.len() < 2 {
        // One run cannot resume itself, so a pass of one chain is never this. Said here rather
        // than left to the resumption reading, because it is the whole answer for every selection
        // that is a single pick.
        return None;
    }

    let mut selected: HashSet<i32> = HashSet::new();

    for chain in chains {
        for edge in &chain.edges {
            selected.insert(edge.edge.mesh_edge_index);
        }
    }

    // ONE rule for both sides of the reading: a run is a crease's if every edge of it is a crease.
    // The list is read both ways - a chain is looked at for ends of its OWN that were interrupted,
    // and looked in for the crease coming back - and the two have to be the same set or the pass
    // could accuse a run it would not accept as the answer. A chain that changes convexity along
    // itself is neither, which errs toward silence: this reading's answer is a refusal.
    let mut creases: Vec<BevelSweptChain> = Vec::new();

    for chain in chains {
        // A closed run has no end to be interrupted at; one passing through a junction was cut
        // open before this (SurfaceChainOpensAtTheJunction) and arrives here as open pieces.
        if !chain.is_closed && is_a_crease_throughout(chain) {
            creases.push(chain.clone());
        }
    }

    for chain in &creases {
        let first = &chain.edges[0];
        let last = &chain.edges[chain.edges.len() - 1];

        // The two ends, each described exactly as BevelChainEndCap describes them to the boolean
        // leg's own reading: the chain-oriented faces and normals, and the direction the run
        // leaves the vertex in.
        for (edge, vertex, position, outward) in [
            (first, first.from_vertex(), first.from(), -first.direction()),
            (last, last.to_vertex(), last.to(), last.direction()),
        ] {
            if !is_interrupted(
                edge,
                vertex,
                position,
                outward,
                features,
                regions,
                &selected,
                &creases,
                size,
                round_profile,
            ) {
                continue;
            }

            return Some(BevelAdvisory::new(
                BevelAdvisoryCode::SurfaceCreaseInterruptedByAFeature,
                vec![position.into()],
            ));
        }
    }

    None
}

/// Whether every edge of a run is a crease, which is what makes it a run
/// a crease can resume in.
fn is_a_crease_throughout(chain: &BevelSweptChain) -> bool {
    chain
        .edges
        .iter()
        .all(|edge| edge.edge.convexity == EdgeConvexity::Concave)
}

/// Whether one end of one chain is where something INTERRUPTED the crease
/// rather than where the crease ended - the readings the module docs list, in the order that turns
/// the most ends down soonest.
#[allow(clippy::too_many_arguments)]
#[allow(clippy::neg_cmp_op_on_partial_ord)] // `!(x > y)` is true for NaN too.
fn is_interrupted(
    edge: &ChainEdge,
    vertex: i32,
    position: Vector3,
    outward: Vector3,
    features: &FeatureEdgeSet,
    regions: &FaceRegionCache,
    selected: &HashSet<i32>,
    creases: &[BevelSweptChain],
    size: f64,
    round_profile: bool,
) -> bool {
    let incident = features.edges_at_vertex(vertex);
    let own_edge = edge.edge.mesh_edge_index;
    let here = scan_the_end(edge, incident, features, outward, selected);

    if here.another_pick_departs_here {
        // Another picked edge departs here, so this is a corner or a junction, and the corner
        // readings own it. Nothing was interrupted: the selection goes on round.
        return false;
    }

    if !here.leaning.closes_over_the_run(here.saw_candidate)
        || !bevel_interrupted_crease::both_own_faces_carry_on(
            incident,
            features,
            own_edge,
            edge.left_normal(),
            edge.right_normal(),
        )
        || bevel_interrupted_crease::crease_carries_on(incident, features, own_edge, outward)
    {
        return false;
    }

    // The crease coming back, asked only of the ends that ride THIS crease's own two surfaces. Each
    // candidate end is handed over as a run of one edge oriented to leave the vertex it ends at, so
    // the ray reading answers about that end and no other point: the edge's far end then faces back
    // along the ray and its own direction test throws it out.
    let mut resuming: Vec<BevelSweptChain> = Vec::new();
    let mut ends: Vec<ChainEdge> = Vec::new();

    for run in creases {
        for end in [run.edges[0], run.edges[run.edges.len() - 1].reversed()] {
            if the_same_two_regions(edge, &end, regions) {
                resuming.push(BevelSweptChain::new(vec![end], false));
                ends.push(end);
            }
        }
    }

    let (gap, which) =
        bevel_interrupted_crease::crease_resumes_beyond(Some(&resuming), position, outward);

    if !(gap > 0.0) {
        return false;
    }

    let wedge_angle_degrees = bevel_edge_tool_builder::profile_wedge_angle_degrees(&edge.edge);

    if !(wedge_angle_degrees > SurfaceBevelPlan::MINIMUM_WEDGE_ANGLE_DEGREES)
        || !(wedge_angle_degrees < SurfaceBevelPlan::MAXIMUM_WEDGE_ANGLE_DEGREES)
    {
        // A wedge the PLAN will not draw a section in at all, and this reading's own answer is
        // drawn from that section. Left alone, so that the edge is refused by what is actually
        // wrong with it (FaceAngleTooFlat, FaceAngleTooSharp) rather than by a name about a feature
        // standing on its crease.
        return false;
    }

    // WHICH end the crease came back at - named by the reading that measured the gap rather than
    // found again by comparing positions to it, since the candidate ends were handed over one to a
    // run in this order. Either end of that one edge may be the one the ray reached, so it is turned
    // to leave whichever of the two stands NEARER along the ray, which is the end the gap was read
    // to. Its own end face is the far side of what stands between them.
    let mut far = ends[which as usize];

    if (far.to() - position).dot(outward) < (far.from() - position).dot(outward) {
        far = far.reversed();
    }

    // The far end read the same way, from ITS run's point of view: the direction that run leaves
    // its own end vertex in is back towards this one. Only the closing face is wanted, so whether
    // another pick departs there is the far chain's own business and not asked about here.
    let there = scan_the_end(
        &far,
        features.edges_at_vertex(far.from_vertex()),
        features,
        -far.direction(),
        selected,
    );

    if !here.one_face_closes_over || !there.one_face_closes_over {
        // More than one distinct plane closes over one of the two runs, or the far end reads none
        // at all. Which single face each strip would die on is then not decided, so there is no
        // pair of planes to read the span between and the pair is named rather than guessed at.
        return true;
    }

    !the_feature_spans_the_strip(
        edge,
        position,
        outward,
        here.closing_normal,
        far.from(),
        there.closing_normal,
        size,
        round_profile,
    )
}

/// What one end's incidence says - which way the faces bounding it lean against the
/// run, whether another pick departs there, and the ONE face plane that closes over the run.
#[derive(Debug, Clone, Copy, Default)]
struct EndScan {
    /// The lean reading, accumulated exactly as the boolean leg's cap plane scan accumulates it.
    leaning: Leaning,
    /// Whether any face at all bounded the corner - an end with none has nothing to read.
    saw_candidate: bool,
    /// Whether a second edge of the selection leaves this vertex, which makes it a corner.
    another_pick_departs_here: bool,
    /// Whether exactly ONE plane closes over the run here. Two distinct ones - a corner of the
    /// feature standing on the end vertex itself - leave no single face for the strip to die on.
    one_face_closes_over: bool,
    /// That plane's outward normal. The plane itself is this and the end VERTEX: every face read
    /// here bounds an edge that ends at the vertex, so the vertex lies on all of them, which is why
    /// two closing faces can be told apart by their normals alone.
    closing_normal: Vector3,
}

/// How far two closing faces' normals may differ and still be one plane: the same one degree
/// `BevelInterruptedCrease` reads two pieces of one surface at.
fn the_same_plane_agreement() -> f64 {
    libm::cos(1.0 * std::f64::consts::PI / 180.0)
}

/// One end's incidence read once: every face of an edge here that is not one of the run's own
/// two, with the run's own edge passed over.
fn scan_the_end(
    edge: &ChainEdge,
    incident: &[i32],
    features: &FeatureEdgeSet,
    outward: Vector3,
    selected: &HashSet<i32>,
) -> EndScan {
    let mut scan = EndScan::default();
    let own_edge = edge.edge.mesh_edge_index;
    let mut closing = 0;

    for &index in incident {
        let at_the_end = features.edges()[index as usize];

        if selected.contains(&at_the_end.mesh_edge_index) {
            if at_the_end.mesh_edge_index != own_edge {
                scan.another_pick_departs_here = true;
            }

            continue;
        }

        for side in 0..2 {
            let face = if side == 0 {
                at_the_end.face0
            } else {
                at_the_end.face1
            };

            if face == edge.left_face() || face == edge.right_face() {
                // The two surfaces the bevel is tangent to bound the cut ACROSS the run, not along
                // it, and say nothing about what is past its end.
                continue;
            }

            scan.saw_candidate = true;

            let normal = if side == 0 {
                at_the_end.normal0
            } else {
                at_the_end.normal1
            };
            let along_outward = normal.dot(outward);

            scan.leaning.saw(along_outward);

            if along_outward >= 0.0 || !bevel_interrupted_crease::crosses_the_run(-along_outward) {
                continue;
            }

            if closing == 0 {
                scan.closing_normal = normal;
                closing = 1;
            } else if scan.closing_normal.dot(normal) < the_same_plane_agreement() {
                closing = 2;
            }
        }
    }

    scan.one_face_closes_over = closing == 1;

    scan
}

/// Whether what stands between the two pieces SPANS the strip -
/// whether, for every point of the profile, the rail through that point meets the NEAR piece's end
/// face before the far piece's.
///
/// Closed form, and exact. A rail is the line through a profile point along the run, so its
/// crossing parameter with a plane is `(n.a - n.P) / (n.outward)` - AFFINE in the profile point P.
/// The difference of the two crossings is therefore affine over the profile as well, and an affine
/// function on an arc takes its minimum either at an end of the arc (a foot) or at the single arc
/// point facing down its own gradient. The profile is drawn at the END VERTEX's own station, which
/// is the same section moved along the run; a chamfer's profile is the chord between the feet, so
/// the feet alone answer it. A plane the run lies ALONG cannot be crossed by any rail and is left to
/// the refusal, as is anything that comes out non finite. Both ends are read as the FLAT two plane
/// section they are planned as; a strip that would instead ROLL onto the face it lands on is not
/// seen here.
#[allow(clippy::too_many_arguments)]
#[allow(clippy::neg_cmp_op_on_partial_ord)] // `!(x > 0)` is true for NaN too.
fn the_feature_spans_the_strip(
    edge: &ChainEdge,
    position: Vector3,
    outward: Vector3,
    near_normal: Vector3,
    far_position: Vector3,
    far_normal: Vector3,
    size: f64,
    round_profile: bool,
) -> bool {
    let near_along = near_normal.dot(outward);
    let far_along = far_normal.dot(outward);

    if !bevel_interrupted_crease::crosses_the_run(near_along.abs())
        || !bevel_interrupted_crease::crosses_the_run(far_along.abs())
    {
        return false;
    }

    // The section, drawn as SurfaceBevelPlan draws it, at the end vertex's own station.
    let along = edge.direction();
    let wedge_angle_degrees = bevel_edge_tool_builder::profile_wedge_angle_degrees(&edge.edge);
    let setback =
        bevel_edge_tool_builder::tangent_distance(size, wedge_angle_degrees, round_profile);
    let left_inward = edge.left_normal().cross(along);
    let right_inward = along.cross(edge.right_normal());
    let left_foot = position + (left_inward * setback);
    let right_foot = position + (right_inward * setback);

    let further = |profile_point: Vector3| {
        how_much_further_the_far_face_is(
            profile_point,
            position,
            near_normal,
            near_along,
            far_position,
            far_normal,
            far_along,
        )
    };

    let mut worst = ieee_math::min_f64(further(left_foot), further(right_foot));

    if round_profile {
        let bisector = left_inward + right_inward;

        if !(bisector.length_squared() > 0.0) {
            return false;
        }

        let centre = position
            + (bisector.get_normal()
                * (size / libm::sin(math_helper::degrees_to_radians(wedge_angle_degrees) / 2.0)));

        // The arc's own interior candidate: the point facing down the difference's gradient, taken
        // only where it lies in the sector the profile actually sweeps. The sector is named by its
        // mid direction - the centre back towards the edge - and its half angle by the feet, taken
        // as the SMALLER of the two cosines, which is the WIDER sector. A candidate wrongly let in
        // can only pull the worst DOWN and so errs towards refusing the pair by name, while one
        // wrongly kept out errs towards building a pair whose strips overlap; the wider sector is
        // the side to err on, because a refusal says so and a bad build does not.
        let gradient = (near_normal / near_along) - (far_normal / far_along);
        let across = gradient - (along * gradient.dot(along));

        if across.length_squared() > 0.0 {
            let facing = -across.get_normal();
            let mid = (position - centre).get_normal();
            let half_sweep = ieee_math::min_f64(
                (left_foot - centre).get_normal().dot(mid),
                (right_foot - centre).get_normal().dot(mid),
            );

            if facing.dot(mid) >= half_sweep {
                worst = ieee_math::min_f64(worst, further(centre + (facing * size)));
            }
        }
    }

    // Anything non finite fails this as well, since a comparison against NaN is false.
    worst > 0.0
}

/// How much further along the run the FAR face stands than the
/// near one, on the rail through one profile point. Positive is the near strip dying on its own
/// face with the far one still ahead of it; zero or less is the two strips overlapping inside the
/// feature.
fn how_much_further_the_far_face_is(
    profile_point: Vector3,
    position: Vector3,
    near_normal: Vector3,
    near_along: f64,
    far_position: Vector3,
    far_normal: Vector3,
    far_along: f64,
) -> f64 {
    let to_the_near = near_normal.dot(position - profile_point) / near_along;
    let to_the_far = far_normal.dot(far_position - profile_point) / far_along;

    to_the_far - to_the_near
}

/// Whether two crease ends ride the same two surfaces - the reading that
/// tells one crease something interrupts from two creases that merely lie on one line.
///
/// A region is flooded over CONNECTED faces at the operation's own bend threshold: two pockets'
/// floors are the same plane but two regions, while a slope the nose bites into is still ONE region
/// above the nose and below it. Unordered, because which surface a run calls left is which way it
/// happens to be walked. Named by [`SurfaceBevelPlan::key_of`]. A face the cache holds no region
/// for matches nothing, a guard rather than a panic; a chain edge's two faces are always mesh
/// faces, so the builder never asks that.
fn the_same_two_regions(run: &ChainEdge, other: &ChainEdge, regions: &FaceRegionCache) -> bool {
    let key = |face: i32| {
        regions
            .region(face)
            .map(|region| SurfaceBevelPlan::key_of(&region))
    };

    let (Some(left), Some(right), Some(their_left), Some(their_right)) = (
        key(run.left_face()),
        key(run.right_face()),
        key(other.left_face()),
        key(other.right_face()),
    ) else {
        return false;
    };

    (left == their_left && right == their_right) || (left == their_right && right == their_left)
}
