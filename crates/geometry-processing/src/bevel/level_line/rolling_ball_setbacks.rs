//! A level-line pass's setbacks read from the rolling ball (`bevel::rolling_ball`), where its
//! switch is on (`rolling_ball::rolling_ball_routing`): the band's field at the size asked takes,
//! at each chain point, how far the ball's contacts stand from the chain there, in place of the
//! setback worked out from the angle at the edge (`chain_on_the_mesh::unit_setbacks`).
//!
//! A contact is measured to the chain's own edges either side of its chain point, the nearer of
//! the two, because that is what the field's setback is: the distance from the chain at which the
//! rail runs. Along a straight run that is the contact's distance from the chain point (the
//! contact lies in the plane square to the chain there). At a turn of the chain it is not: the
//! ball at a cube rim's corner, rolled square to the corner's blended direction, touches the top
//! r from both edges but r sqrt 2 from the corner vertex, and a setback of r sqrt 2 there would
//! cut the band far wider at every corner than the round is.
//!
//! **A setback per side.** Each side's contacts give that side's own setbacks ([`SideSetbacks`],
//! left and right as `chain_on_the_mesh::sides_of` names a chain edge's two triangles: the left
//! one runs the edge in the chain's direction, and the ball's side A is the left). Where a side
//! curves away from the edge (a groove wall that bends, the T Handle's wall against its sphere)
//! the two differ, and each side's band is cut at its own distance (`band_group_fields::of` hands
//! each side group its own side's field through `band_group_fields::of_sides`).
//!
//! **Whole pass, never half.** The rolling ball decides the pass only where every chain in it
//! walks: every station solves, both contacts' distances are finite numbers, and each end finishes
//! on an end face or as a plain last station. Anything else keeps the level line's own setbacks
//! for every chain, and the route says why: a chamfer, a pass with no feature-edge test, chains of
//! the pass meeting at a corner (the ball's three-face rule for a corner is not built, and a corner
//! is no end face), a chain whose edges are not all convex or all concave (one ball sits on one
//! side of the part), a walk refused, a run-out end, a distance that is not a finite number.
//!
//! **An open chain's sides per side.** Where an open chain's two sides differ by more than
//! [`SIDES_AGREE_SHARE`] of the size, each side's rail is cut at its own distance and each end face
//! the round cuts (an end region) is cut by its own cap field ([`CapShift`]), which meets each
//! side's rail exactly where that side's band ends. Where they differ the level line is still kept
//! whole, with the route naming the sides' distances, at:
//! - a free end (`LastStation`): the two rails end apart on the rim, and nothing cuts the end;
//! - an end face whose normal does not run along the chain's end edge, or whose edges with the
//!   walls do not carry both rails' ends (a face narrower than a rail's distance, or one the band
//!   flowed over): the cap's circle is laid in the plane square to the chain through the rails'
//!   ends there ([`SQUARE_END_SLACK`]), which only on such a square end are the rails' ends;
//! - an open chain in a pass of several chains (its end regions are groups of the whole pick, not
//!   tied to one chain end, so no cap is handed them);
//! - an open chain with an end ray ([`unless_a_rayed_end_differs`]: the rail carried past such an
//!   end and its end-plane trim carry one setback for both sides).
//!
//! **Only the outer pass.** The ball is walked, and the route written, only by the outermost
//! `build_within` on the thread and only where it is not a topology-only probe ([`PassScope`]).
//! The probes a build runs inside itself - the pinch ladder's, which call `build_within` again at
//! other sizes, and every topology-only build - keep the level line's setbacks and never write
//! the route, so the route a build reports is its own pass's. The room search (`band_room`,
//! `band_room_pair_room_pick`) asks its fields at other sizes and keeps the level line's setbacks
//! too: a contact's distance is not linear in the size on a curved side, so a setback per unit of
//! size read at one size says nothing at another.
//!
//! **Where the sides come from.** The kernel reads each side for itself: from the triangles the
//! chain's edges stand between, flooded over every edge that is neither a feature edge (the pass's
//! own `is_feature_edge`) nor an edge of a chain in the pass. That is the region the pick shows
//! the user on each side of the edge (the model floods its face regions to the same creases), and
//! reading it here rather than handing triangle lists in through `BandGapBuildOptions` keeps the
//! sides matched to the chains the pass actually builds: a pick is split at setback steps and
//! probed one chain at a time on its way here, so lists made for the caller's chains would not
//! line up with these. Its cost is one flood per side per pass, only with the switch on.
//!
//! The whole part is the obstacle the ball may not reach past. A chain's end that lies on a face
//! of neither side ends on that face (its plane through the end vertex, normal the area-weighted
//! sum of those faces round the vertex); an end with none is free and the ball rolls on to it.

use std::cell::Cell;
use std::collections::HashSet;

use vector_math::Vector3;

use super::triangle_edges::{self, StandingMap};
use super::{chain_on_the_mesh, pick_validation};
use super::{BandProfile, ChainField};
use crate::bevel::rolling_ball::rolling_ball_routing::{
    set_last_route, use_rolling_ball, ROLLING_BALL_ROUTE,
};
use crate::bevel::rolling_ball::{ChainPoint, EndFace, RollingBall, RollingBallSide, WalkEnd};

/// One chain's setbacks per unit of size from the rolling ball, one per chain vertex on each side:
/// how far side A's (left) and side B's (right) contacts stand from the chain there.
#[derive(Clone, Debug, PartialEq)]
pub struct SideSetbacks {
    /// The left side's (tag 1's) setbacks.
    pub left: Vec<f64>,
    /// The right side's (tag 2's) setbacks.
    pub right: Vec<f64>,
    /// How each end's cap is cut, as (first end, last end), where that end's two sides differ by
    /// more than [`SIDES_AGREE_SHARE`] of the size (only on a square end face, the pass refused
    /// at every other end where they differ); `None` where they agree and on a closed chain, whose
    /// cap, if any, reads the chain's own field (the larger side) as the level line does.
    pub caps: [Option<CapShift>; 2],
}

/// An open chain's end cap cut to meet each side's rail where that side's band ends, per unit of
/// size: the cap reads the chain's own field moved by `offset` and every setback grown by `grow`.
///
/// **Why a moved circle.** In the end plane the left side's rail meets the cap at A', `l` from the
/// end vertex e along the left wall, and the right side's at B', `r` along the right. The cap's
/// cut must pass through both, or a notch opens along the wall whose rail ends short of the cut.
/// The cut taken is the circle through A' and B' whose centre is the point of their bisector
/// nearest e: it is smooth (one arc between the walls), it is the level line's own circle about e
/// where `l` = `r` (the offset is then nothing), and it keeps the cap a chain field - a union of
/// balls, so every bounded search, sample and chain parameter the cap's group reads works as
/// before (a translation moves no parameter). A setback blended by angle round e would meet both
/// walls too, but it is no union of balls: not 1-Lipschitz near e, which the bounded searches rely
/// on. The offset runs along B' - A', which lies in the end face, so an end ray's bound plane
/// through the moved end vertex is the same plane.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct CapShift {
    /// How far the cap's chain is moved, per unit of size.
    pub offset: Vector3,
    /// How much every setback of the chain's own field grows for the cap, per unit of size: the
    /// circle's radius less the larger side's setback at the end.
    pub grow: f64,
}

impl SideSetbacks {
    /// The larger of the two sides at each chain vertex: how far the band reaches from the chain
    /// on either side, which is what a reading of the whole band (not one side's rail) asks.
    pub fn larger(&self) -> Vec<f64> {
        self.left
            .iter()
            .zip(&self.right)
            .map(|(&l, &r)| vector_math::ieee_math::max_f64(l, r))
            .collect()
    }
}

/// How far apart an open chain's two sides' contact distances may stand at one chain point, as a
/// share of the size, for its end caps to read the chain's own field as the level line does (a
/// cap of its own past that, [`CapShift`]). A flat corner's two distances are equal to the solve's
/// precision; a side that curves away from the edge sets them apart.
pub const SIDES_AGREE_SHARE: f64 = 1e-3;

thread_local! {
    // How many `build_within` passes are running on this thread, the outer one included.
    static PASS_DEPTH: Cell<usize> = const { Cell::new(0) };
}

/// One `build_within` pass running on this thread, held for the whole pass so the builds it runs
/// inside itself (the pinch ladder's probes) know they are not the outer pass.
#[must_use = "the pass ends as soon as the scope is dropped"]
#[derive(Debug)]
pub struct PassScope {
    outer: bool,
    // The depth is this thread's; a scope dropped on another thread would count wrongly.
    _not_send: std::marker::PhantomData<*const ()>,
}

impl PassScope {
    /// Enters a pass on this thread.
    pub fn enter() -> PassScope {
        let depth = PASS_DEPTH.with(|d| d.replace(d.get() + 1));
        PassScope {
            outer: depth == 0,
            _not_send: std::marker::PhantomData,
        }
    }

    /// Whether no other pass was running on this thread when this one began.
    pub fn is_outer(&self) -> bool {
        self.outer
    }
}

impl Drop for PassScope {
    fn drop(&mut self) {
        PASS_DEPTH.with(|d| d.set(d.get().saturating_sub(1)));
    }
}

/// The pass's setbacks per unit of size from the rolling ball, one [`SideSetbacks`] per chain - or `None` where the switch is off, the pass is a `probe` (not the outer pass, or
/// topology only), or the ball cannot take the whole pass. With the switch on, an outer pass that
/// is no probe writes the route it took; a probe writes nothing (see the file header).
#[allow(clippy::too_many_arguments)]
pub fn for_the_pass(
    positions: &[Vector3],
    indices: &[(i32, i32, i32)],
    chains: &[Vec<i32>],
    closed: &[bool],
    is_feature_edge: Option<&dyn Fn(i32, i32) -> bool>,
    size: f64,
    profile: BandProfile,
    probe: bool,
) -> Option<Vec<SideSetbacks>> {
    if probe || !use_rolling_ball() {
        return None;
    }

    match walked_setbacks(
        positions,
        indices,
        chains,
        closed,
        is_feature_edge,
        size,
        profile,
    ) {
        Ok(setbacks) => {
            set_last_route(Some(ROLLING_BALL_ROUTE.to_string()));
            Some(setbacks)
        }
        Err(why) => {
            set_last_route(Some(format!("level-line: {why}")));
            None
        }
    }
}

/// The ball's setbacks kept, or `None` with the route naming why, where an open chain whose two
/// sides differ anywhere by more than [`SIDES_AGREE_SHARE`] of the size carries an END RAY at
/// either end (`end_rays`, as `end_face_sense::oblique_ends` reads them): an oblique end face is
/// not cut by the chain's distance, and the rails carried past such an end and its end-plane trim
/// (`virtual_rail_past_the_end`, `end_face_fill_try`) carry one setback for both sides.
pub fn unless_a_rayed_end_differs(
    ball: Option<Vec<SideSetbacks>>,
    closed: &[bool],
    end_rays: &[(bool, bool)],
    size: f64,
) -> Option<Vec<SideSetbacks>> {
    let setbacks = ball?;
    for (k, chain) in setbacks.iter().enumerate() {
        let rayed = end_rays.get(k).is_some_and(|&(first, last)| first || last);
        let differ = chain
            .left
            .iter()
            .zip(&chain.right)
            .position(|(l, r)| (l - r).abs() > SIDES_AGREE_SHARE);
        if let (false, true, Some(i)) = (closed[k], rayed, differ) {
            set_last_route(Some(format!(
                "level-line: chain {k} point {i}: the sides' contacts stand {} and {} from the \
                 chain, and an oblique end's rail carried past the end is not cut per side yet",
                chain.left[i] * size,
                chain.right[i] * size
            )));
            return None;
        }
    }
    Some(setbacks)
}

/// Every chain's walked setbacks, or why the pass keeps the level line's.
#[allow(clippy::neg_cmp_op_on_partial_ord)] // `!(x <= tolerance)` on purpose: a NaN refuses
fn walked_setbacks(
    positions: &[Vector3],
    indices: &[(i32, i32, i32)],
    chains: &[Vec<i32>],
    closed: &[bool],
    is_feature_edge: Option<&dyn Fn(i32, i32) -> bool>,
    size: f64,
    profile: BandProfile,
) -> Result<Vec<SideSetbacks>, String> {
    if profile == BandProfile::Chamfer {
        return Err("a chamfer is not rolled by a ball".to_string());
    }
    let Some(is_feature_edge) = is_feature_edge else {
        return Err("the pass has no feature edges to bound its sides".to_string());
    };

    if let Some(corner) = pick_validation::corners_of(chains, closed, true).first() {
        return Err(format!(
            "the pass's chains meet at a corner at vertex {}, which the ball does not close yet",
            corner.vertex
        ));
    }

    let standing = triangle_edges::standing_map(indices);
    let chain_edges: Vec<Vec<(i32, i32)>> = chains
        .iter()
        .zip(closed)
        .map(|(chain, &is_closed)| chain_on_the_mesh::chain_edges(chain, is_closed))
        .collect();
    let on_a_chain: HashSet<(i32, i32)> = chain_edges
        .iter()
        .flatten()
        .map(|&(a, b)| triangle_edges::key(a, b))
        .collect();
    let every_triangle: Vec<[i32; 3]> = indices.iter().map(|&(a, b, c)| [a, b, c]).collect();
    let part = RollingBallSide::new(positions, &every_triangle);

    let mut setbacks = Vec::with_capacity(chains.len());
    for (k, chain) in chains.iter().enumerate() {
        let edges = &chain_edges[k];
        let mut lefts = Vec::with_capacity(edges.len());
        let mut rights = Vec::with_capacity(edges.len());
        for &(from, to) in edges {
            let (left, right) = chain_on_the_mesh::sides_of(indices, &standing, from, to)
                .map_err(|error| format!("chain {k}: {}", error.message))?;
            lefts.push(left);
            rights.push(right);
        }

        let convex_at = |e: usize| {
            let left_normal = triangle_edges::normal(positions, indices[lefts[e] as usize]);
            is_convex(positions, indices, edges[e], rights[e], left_normal)
        };
        let convex = convex_at(0);
        if let Some(e) = (1..edges.len()).find(|&e| convex_at(e) != convex) {
            let kind = |c: bool| if c { "convex" } else { "concave" };
            return Err(format!(
                "chain {k}: edge {e} is {} where its first edge is {}",
                kind(!convex),
                kind(convex)
            ));
        }
        let crosses = |a: i32, b: i32| {
            !on_a_chain.contains(&triangle_edges::key(a, b)) && !is_feature_edge(a, b)
        };
        let in_a = flood(indices, &standing, &lefts, &crosses);
        let in_b = flood(indices, &standing, &rights, &crosses);
        if in_a.iter().zip(&in_b).any(|(&a, &b)| a && b) {
            return Err(format!("chain {k}: its two sides are one surface"));
        }

        let triangles_of = |inside: &[bool]| -> Vec<[i32; 3]> {
            (0..indices.len())
                .filter(|&t| inside[t])
                .map(|t| every_triangle[t])
                .collect()
        };
        let side_a = RollingBallSide::new(positions, &triangles_of(&in_a));
        let side_b = RollingBallSide::new(positions, &triangles_of(&in_b));

        let points = chain_points(positions, indices, chain, closed[k], edges, &lefts, &rights);
        let ends = if closed[k] {
            [None, None]
        } else {
            [
                end_face(positions, indices, chain[0], &in_a, &in_b),
                end_face(positions, indices, chain[chain.len() - 1], &in_a, &in_b),
            ]
        };

        let ball = RollingBall {
            side_a: &side_a,
            side_b: &side_b,
            part: Some(&part),
            radius: size,
            convex,
        };
        let walk = ball
            .walk_ending_on(&points, ends)
            .map_err(|error| format!("chain {k}: the ball has no walk: {error:?}"))?;
        for (end, finished) in walk.ends.iter().enumerate() {
            if !matches!(finished, WalkEnd::EndFace | WalkEnd::LastStation) {
                return Err(format!("chain {k}: end {end} finished {finished:?}"));
            }
        }
        if walk.stations.len() != points.len() {
            return Err(format!(
                "chain {k}: {} stations for {} chain points",
                walk.stations.len(),
                points.len()
            ));
        }

        let mut left = Vec::with_capacity(points.len());
        let mut right = Vec::with_capacity(points.len());
        for (i, station) in walk.stations.iter().enumerate() {
            let to_a = from_the_chain(positions, edges, closed[k], i, station.contact_a.point);
            let to_b = from_the_chain(positions, edges, closed[k], i, station.contact_b.point);
            if !to_a.is_finite() || !to_b.is_finite() {
                return Err(format!(
                    "chain {k} point {i}: a contact's distance is not a number ({to_a}, {to_b})"
                ));
            }
            if !closed[k] && chains.len() > 1 && !((to_a - to_b).abs() <= SIDES_AGREE_SHARE * size)
            {
                return Err(format!(
                    "chain {k} point {i}: the sides' contacts stand {to_a} and {to_b} from the \
                     chain, and an open chain's end regions in a pass of several chains are not \
                     cut per side yet"
                ));
            }
            left.push(to_a / size);
            right.push(to_b / size);
        }
        let mut caps = [None, None];
        if !closed[k] {
            for (end, finished) in walk.ends.iter().enumerate() {
                let i = if end == 0 { 0 } else { points.len() - 1 };
                let (l, r) = (left[i] * size, right[i] * size);
                if (l - r).abs() <= SIDES_AGREE_SHARE * size {
                    continue;
                }
                let differ = format!("chain {k} end {end}: the sides' contacts stand {l} and {r}");
                let (Some(face), WalkEnd::EndFace) = (ends[end].as_ref(), *finished) else {
                    return Err(format!(
                        "{differ} from the chain at a free end, whose rails end apart on the \
                         rim, and a free end is not cut per side yet"
                    ));
                };
                let edge = edges[if end == 0 { 0 } else { edges.len() - 1 }];
                let e = points[i].point;
                let along = (positions[edge.1 as usize] - positions[edge.0 as usize]).get_normal();
                let station = &walk.stations[i];
                let rail_at = |contact: Vector3, distance: f64| {
                    let foot = ChainField::closest_on_segment(
                        contact,
                        positions[edge.0 as usize],
                        positions[edge.1 as usize],
                    );
                    e + ((contact - foot).get_normal() * distance)
                };
                let (a, b) = (
                    rail_at(station.contact_a.point, l),
                    rail_at(station.contact_b.point, r),
                );
                let wall_edge = |rail: Vector3, distance: f64, side: &[bool], other: &[bool]| {
                    on_a_wall_edge(positions, indices, &standing, (side, other), rail, distance)
                };
                if face.normal.get_normal().dot(along).abs() < 1.0 - SQUARE_END_SLACK
                    || !wall_edge(a, l, &in_a, &in_b)
                    || !wall_edge(b, r, &in_b, &in_a)
                {
                    return Err(format!(
                        "{differ} from the chain, and an end face that is not square to the \
                         chain or does not reach both rails along its own wall edges is not cut \
                         per side yet"
                    ));
                }
                let shift = cap_shift(e, a, b, vector_math::ieee_math::max_f64(l, r))
                    .ok_or_else(|| format!("chain {k} end {end}: its cap has no circle"))?;
                caps[end] = Some(CapShift {
                    offset: shift.offset * (1.0 / size),
                    grow: shift.grow / size,
                });
            }
        }
        setbacks.push(SideSetbacks { left, right, caps });
    }

    Ok(setbacks)
}

/// How far from exact a square end may stand for its cap to be cut per side: the end face's unit
/// normal's cosine with the chain's end edge within this of 1, and a rail's end point within this
/// share of its distance (at least 1) of the end face's edge with its side. Past it the cap's circle, laid in
/// the plane square to the chain, would not meet the rails on the end face, so the pass keeps the
/// level line.
const SQUARE_END_SLACK: f64 = 1e-9;

/// Whether `rail` lies on the end face's edge with `side`: an edge of a triangle of `side` whose
/// other triangle is of neither side (`sides`: that side's and the other's triangles), within
/// [`SQUARE_END_SLACK`] of `distance` (at least 1). An end face narrower than the rail's distance
/// has no such edge there.
fn on_a_wall_edge(
    positions: &[Vector3],
    indices: &[(i32, i32, i32)],
    standing: &StandingMap,
    (side, other): (&[bool], &[bool]),
    rail: Vector3,
    distance: f64,
) -> bool {
    let near = SQUARE_END_SLACK * vector_math::ieee_math::max_f64(1.0, distance);
    indices.iter().enumerate().any(|(t, &corners)| {
        side[t]
            && triangle_edges::corners(corners).into_iter().any(|(a, b)| {
                let on_the_end_face = standing
                    .get(&triangle_edges::key(a, b))
                    .map_or(&[][..], Vec::as_slice)
                    .iter()
                    .any(|&u| !side[u as usize] && !other[u as usize]);
                let (from, to) = (positions[a as usize], positions[b as usize]);
                on_the_end_face
                    && (rail - ChainField::closest_on_segment(rail, from, to)).length() <= near
            })
    })
}

/// The cap's circle at an end vertex `e` through the left rail's end `a` and the right's `b`
/// (see [`CapShift`]), at the size: the moved centre's offset from `e`, and its radius less
/// `larger`. `None` where `a` and `b` are one point or the radius is not a finite number.
fn cap_shift(e: Vector3, a: Vector3, b: Vector3, larger: f64) -> Option<CapShift> {
    let across = b - a;
    let across_squared = across.length_squared();
    if across_squared.is_nan() || across_squared <= 0.0 {
        return None;
    }
    // The bisector of a and b is every p with (p - e) . across = (|b - e|^2 - |a - e|^2) / 2.
    let along = ((b - e).length_squared() - (a - e).length_squared()) / (2.0 * across_squared);
    let offset = across * along;
    let radius = (a - (e + offset)).length();
    let grow = radius - larger;
    (radius.is_finite() && grow.is_finite()).then_some(CapShift { offset, grow })
}

/// How far `contact` stands from the chain's edges either side of chain vertex `v` (the one edge
/// at an open chain's end): the nearer.
fn from_the_chain(
    positions: &[Vector3],
    edges: &[(i32, i32)],
    is_closed: bool,
    v: usize,
    contact: Vector3,
) -> f64 {
    let before = if v > 0 {
        Some(v - 1)
    } else if is_closed {
        Some(edges.len() - 1)
    } else {
        None
    };
    let after = (v < edges.len()).then_some(v);
    let mut nearest = f64::INFINITY;
    for e in [before, after].into_iter().flatten() {
        let (from, to) = edges[e];
        let on = ChainField::closest_on_segment(
            contact,
            positions[from as usize],
            positions[to as usize],
        );
        let distance = (contact - on).length();
        if distance < nearest {
            nearest = distance;
        }
    }

    nearest
}

/// Which triangles a walk from `seeds` reaches over edges `crosses` allows, by triangle index.
fn flood(
    indices: &[(i32, i32, i32)],
    standing: &StandingMap,
    seeds: &[i32],
    crosses: &dyn Fn(i32, i32) -> bool,
) -> Vec<bool> {
    let mut inside = vec![false; indices.len()];
    let mut stack: Vec<i32> = Vec::new();
    for &seed in seeds {
        if !inside[seed as usize] {
            inside[seed as usize] = true;
            stack.push(seed);
        }
    }
    while let Some(t) = stack.pop() {
        for (a, b) in triangle_edges::corners(indices[t as usize]) {
            if !crosses(a, b) {
                continue;
            }
            for &next in standing
                .get(&triangle_edges::key(a, b))
                .map_or(&[][..], Vec::as_slice)
            {
                if !inside[next as usize] {
                    inside[next as usize] = true;
                    stack.push(next);
                }
            }
        }
    }

    inside
}

/// The chain's points as the ball reads them: each vertex with the chain's direction there and
/// each side's normal beside it, both blended over the edges either side of the vertex (the one
/// edge at an open chain's end).
fn chain_points(
    positions: &[Vector3],
    indices: &[(i32, i32, i32)],
    chain: &[i32],
    is_closed: bool,
    edges: &[(i32, i32)],
    lefts: &[i32],
    rights: &[i32],
) -> Vec<ChainPoint> {
    let count = edges.len();
    let direction =
        |e: usize| (positions[edges[e].1 as usize] - positions[edges[e].0 as usize]).get_normal();
    let left_normal = |e: usize| triangle_edges::normal(positions, indices[lefts[e] as usize]);
    let right_normal = |e: usize| triangle_edges::normal(positions, indices[rights[e] as usize]);
    let blend = |of: &dyn Fn(usize) -> Vector3, before: Option<usize>, after: Option<usize>| match (
        before, after,
    ) {
        (Some(b), Some(a)) => (of(b) + of(a)).get_normal(),
        (Some(only), None) | (None, Some(only)) => of(only),
        (None, None) => Vector3::ZERO,
    };

    (0..chain.len())
        .map(|v| {
            // Vertex v stands between edge v - 1 and edge v.
            let before = if v > 0 {
                Some(v - 1)
            } else if is_closed {
                Some(count - 1)
            } else {
                None
            };
            let after = (v < count).then_some(v);
            ChainPoint {
                point: positions[chain[v] as usize],
                tangent: blend(&direction, before, after),
                normal_a: blend(&left_normal, before, after),
                normal_b: blend(&right_normal, before, after),
            }
        })
        .collect()
}

/// Whether the edge `from -> to` is convex: the right triangle folds away behind the left one's
/// outward normal.
fn is_convex(
    positions: &[Vector3],
    indices: &[(i32, i32, i32)],
    (from, to): (i32, i32),
    right: i32,
    left_normal: Vector3,
) -> bool {
    let (a, b, c) = indices[right as usize];
    let far = [a, b, c]
        .into_iter()
        .find(|&v| v != from && v != to)
        .unwrap_or(a);
    (positions[far as usize] - positions[from as usize]).dot(left_normal) < 0.0
}

/// The face of neither side `vertex` lies on, as its plane there, or `None` where every triangle
/// round the vertex is one side's or the other's.
fn end_face(
    positions: &[Vector3],
    indices: &[(i32, i32, i32)],
    vertex: i32,
    in_a: &[bool],
    in_b: &[bool],
) -> Option<EndFace> {
    let mut normal = Vector3::ZERO;
    for (t, &(a, b, c)) in indices.iter().enumerate() {
        if (a == vertex || b == vertex || c == vertex) && !in_a[t] && !in_b[t] {
            let (pa, pb, pc) = (
                positions[a as usize],
                positions[b as usize],
                positions[c as usize],
            );
            normal += (pb - pa).cross(pc - pa);
        }
    }

    (normal.length() > 0.0).then(|| EndFace {
        point: positions[vertex as usize],
        normal,
    })
}
