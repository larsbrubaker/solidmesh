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
//! cut the band far wider at every corner than the round is. At an open chain's end on an end
//! face, the end station touches each side where the walk extends it past the face, which beyond a
//! face leaning back over the end vertex lies past that vertex: there the end edge is read as its
//! line carried on past the end vertex, and clamped toward the chain's interior
//! ([`from_the_end_edge`]), as the band's own field reads the chain past its end.
//!
//! **A setback per side.** Each side's contacts give that side's own setbacks ([`SideSetbacks`],
//! left and right as `chain_on_the_mesh::sides_of` names a chain edge's two triangles: the left
//! one runs the edge in the chain's direction, and the ball's side A is the left). Where a side
//! curves away from the edge (a groove wall that bends, the T Handle's wall against its sphere)
//! the two differ, and each side's band is cut at its own distance (`band_group_fields::of` hands
//! each side group its own side's field through `band_group_fields::of_sides`) - where the band's
//! tagging divides the two sides into a group each; where it does not, see below.
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
//! side's rail exactly where that side's band ends. At every free end (`LastStation` with no end
//! face: the edge runs on as an unpicked crease), whether or not the sides differ there, both
//! sides' setbacks taper to nothing at the end vertex (`rolling_ball_free_ends`), so the round
//! closes onto that vertex. Where they differ the level
//! line is still kept whole, with the route naming the sides' distances, at:
//! - an end face the walk never reached (its last station short of the face's plane): nothing
//!   says where on that face the rails end;
//! - an end face where the end station's contact on a side rests on a rim of that side as the
//!   walk extends it past the face (a gap in the extension, `rolling_ball::side_extension`): its
//!   distance is no setback, so the pass keeps the level line rather than build short;
//! - an end face whose edges with the walls end before a rail does (narrower than the rail's
//!   distance, or one the band flows over): no point of its edge carries that rail's end
//!   (`rolling_ball_end_caps::rail_end`);
//! - an open chain in a pass of several chains (its end regions are groups of the whole pick, not
//!   tied to one chain end, so no cap is handed them);
//! - an open chain with an end ray ([`unless_a_rayed_end_differs`]: the rail carried past such an
//!   end and its end-plane trim carry one setback for both sides);
//! - a pass of one open chain whose band tagging does not divide its two sides (the side floods
//!   meet, as on the T Handle round its free tip): its band is one group cut by one field
//!   ([`unless_an_undivided_chain_differs`]). Where the sides agree there, that group reads the
//!   chain's own field.
//!
//! **Only the outer pass.** The ball is walked, and the route written, only by the outermost
//! `build_within` on the thread and only where it is not a topology-only probe ([`PassScope`]).
//! The probes a build runs inside itself - the pinch ladder's, which call `build_within` again at
//! other sizes, and every topology-only build - keep the level line's setbacks and never write
//! the route, so the route a build reports is its own pass's. The room of a pass of one chain the
//! ball takes is read with these setbacks, per unit of the size asked (`one_chain_room`): exact as
//! a yes or no at that size, an estimate below it, since a contact's distance is not linear in the
//! size on a curved side. The room of a pass of several chains (`band_room_pair_room_pick`) keeps
//! the level line's.
//!
//! **Where the sides come from** (`rolling_ball_sides.rs`). From the caller's face region of every
//! triangle where it gives them (`band_gap_entry_points::build_pick_with_side_regions`, the
//! regions the pick shows the user): each side is every region holding a triangle beside the
//! chain's edges. Where it gives none, the kernel floods each side for itself from those
//! triangles over every edge that is neither a feature edge (the pass's own `is_feature_edge`)
//! nor an edge of a chain in the pass. That file's header says why the caller hands a region per
//! triangle and not a side list per chain. The flood costs one walk per side per pass, only with
//! the switch on.
//!
//! The whole part is the obstacle the ball may not reach past. A chain's end that lies on a face
//! of neither side ends on that face (its plane through the end vertex, normal the area-weighted
//! sum of those faces round the vertex); an end with none is free and the ball rolls on to it.

use std::cell::Cell;
use std::collections::HashSet;

use vector_math::Vector3;

use super::band_gap_setback_fields::PassSetbacks;
use super::band_side_tagging_ends::{self, SideReach, SideTaggingError};
use super::rolling_ball_end_caps::{self, CapSide};
use super::rolling_ball_free_ends;
use super::triangle_edges;
use super::{chain_on_the_mesh, pick_validation, rolling_ball_sides};
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
    /// more than [`SIDES_AGREE_SHARE`] of the size (only on an end face, square or leaning, that
    /// carries both rails' ends; a free end's sides taper to the end vertex instead, and the pass
    /// is refused at every other end where they differ); `None` where they agree, at a free end
    /// and on a closed chain, whose cap, if any, reads the chain's own field (the larger side) as
    /// the level line does.
    pub caps: [Option<CapShift>; 2],
}

/// An open chain's end cap cut to meet each side's rail where that side's band ends, per unit of
/// size: the cap reads the chain's own field moved by `offset` and every setback grown by `grow`.
///
/// Each rail's end is where that side's field meets the side's edge with the end face, and the cap
/// is moved and grown so its cut passes through both; `rolling_ball_end_caps` says why that cut is
/// smooth and still a chain field. The offset runs along the line between the rails' ends, which
/// lies in the end face, so an end ray's bound plane through the moved end vertex is the same
/// plane.
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
        larger_of(&self.left, &self.right)
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

    /// Whether a pass entered now on this thread would be the outer one: no pass is running.
    pub fn would_be_outer() -> bool {
        PASS_DEPTH.with(|d| d.get() == 0)
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
    side_regions: Option<&[u32]>,
    size: f64,
    profile: BandProfile,
    band: &PassSetbacks,
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
        side_regions,
        size,
        profile,
        band,
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

/// The ball's setbacks for a pass of ONE OPEN chain whose band tagging does not divide its two
/// sides (the side floods meet: a crease that dies out, or sides joined round a free end), as
/// (setbacks, whether the one band group reads the chain's own field). Such a band is one group cut
/// by one field, so no side's own setback can stand for both: where the two sides differ anywhere
/// by more than [`SIDES_AGREE_SHARE`] of the size, the whole pass keeps the level line, the route
/// naming why (the larger field would overshoot on the nearer side, either side's own would cut
/// the other at the wrong distance); where they agree the group reads the chain's own field. Every
/// other pass comes back as it was, with `false`. `divides` is read by the same side floods the
/// build's tagging runs (`band_side_tagging_ends::tag_sides`), which no field changes.
pub fn unless_an_undivided_chain_differs(
    ball: Option<Vec<SideSetbacks>>,
    indices: &[(i32, i32, i32)],
    chain_edges: &[Vec<(i32, i32)>],
    closed: &[bool],
    is_feature_edge: &dyn Fn(i32, i32) -> bool,
    size: f64,
) -> Result<(Option<Vec<SideSetbacks>>, bool), SideTaggingError> {
    let Some(setbacks) = ball else {
        return Ok((None, false));
    };
    if setbacks.len() != 1 || closed[0] {
        return Ok((Some(setbacks), false));
    }
    let (_, divides) = band_side_tagging_ends::tag_sides(
        indices,
        &chain_edges[0],
        is_feature_edge,
        false,
        None,
        (false, false),
        SideReach::default(),
    )?;
    if divides {
        return Ok((Some(setbacks), false));
    }
    let chain = &setbacks[0];
    let differ = chain
        .left
        .iter()
        .zip(&chain.right)
        .position(|(l, r)| (l - r).abs() > SIDES_AGREE_SHARE);
    if let Some(i) = differ {
        set_last_route(Some(format!(
            "level-line: chain 0 point {i}: the sides' contacts stand {} and {} from the chain, \
             and the band's tagging joins its two sides into one region, which is not cut per \
             side yet",
            chain.left[i] * size,
            chain.right[i] * size
        )));
        return Ok((None, false));
    }
    Ok((Some(setbacks), true))
}

/// Every chain's walked setbacks, or why the pass keeps the level line's.
#[allow(clippy::neg_cmp_op_on_partial_ord)] // `!(x <= tolerance)` on purpose: a NaN refuses
#[allow(clippy::too_many_arguments)]
fn walked_setbacks(
    positions: &[Vector3],
    indices: &[(i32, i32, i32)],
    chains: &[Vec<i32>],
    closed: &[bool],
    is_feature_edge: Option<&dyn Fn(i32, i32) -> bool>,
    side_regions: Option<&[u32]>,
    size: f64,
    profile: BandProfile,
    band: &PassSetbacks,
) -> Result<Vec<SideSetbacks>, String> {
    if profile == BandProfile::Chamfer {
        return Err("a chamfer is not rolled by a ball".to_string());
    }
    // The caller's face regions bound each side on their own; only the kernel's flood needs the
    // feature edges.
    if is_feature_edge.is_none() && side_regions.is_none() {
        return Err("the pass has no feature edges to bound its sides".to_string());
    }

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
            !on_a_chain.contains(&triangle_edges::key(a, b))
                && !is_feature_edge.is_some_and(|feature| feature(a, b))
        };
        let (in_a, in_b) = rolling_ball_sides::of_the_chain(
            indices,
            &standing,
            k,
            (&lefts, &rights),
            side_regions,
            &crosses,
        )?;

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
            let on_an_end_face = !closed[k]
                && ((i == 0 && walk.ends[0] == WalkEnd::EndFace)
                    || (i + 1 == walk.stations.len() && walk.ends[1] == WalkEnd::EndFace));
            // A contact on a rim of a side read as extended past the end face lies where the
            // extension has a hole (the ones `rolling_ball::side_extension` lists): no surface
            // the round rests on, so its distance is no setback. Fail safe.
            if on_an_end_face {
                let end = usize::from(i != 0);
                for (contact, name) in [(&station.contact_a, "A"), (&station.contact_b, "B")] {
                    if contact.feature.is_rim() {
                        return Err(format!(
                            "chain {k} end {end}: side {name}'s contact at the end face rests on a \
                             rim of the side extended past the face, so its setback there is not \
                             the round's"
                        ));
                    }
                }
            }
            let measure = |contact: Vector3| {
                from_the_chain(positions, edges, closed[k], i, on_an_end_face, contact)
            };
            let (to_a, to_b) = (
                measure(station.contact_a.point),
                measure(station.contact_b.point),
            );
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
            // Every free end closes the round onto its end vertex, whether or not the sides
            // agree there: both sides' setbacks taper to nothing at it (`rolling_ball_free_ends`).
            let free_ends =
                [0, 1].map(|end| ends[end].is_none() && walk.ends[end] == WalkEnd::LastStation);
            for (end, &free) in free_ends.iter().enumerate() {
                if free {
                    for side in [&mut left, &mut right] {
                        rolling_ball_free_ends::close_onto_the_end_vertex(
                            side,
                            &band.chain_points[k],
                            end == 1,
                            size,
                            band.sharp_turn_degrees(),
                            // The other end's end ray, if it has one.
                            if end == 1 {
                                band.end_rays[k].0
                            } else {
                                band.end_rays[k].1
                            },
                        );
                    }
                }
            }
            for (end, finished) in walk.ends.iter().enumerate() {
                let i = if end == 0 { 0 } else { points.len() - 1 };
                let (l, r) = (left[i] * size, right[i] * size);
                if free_ends[end] || (l - r).abs() <= SIDES_AGREE_SHARE * size {
                    continue;
                }
                let differ = format!("chain {k} end {end}: the sides' contacts stand {l} and {r}");
                let (Some(_), WalkEnd::EndFace) = (ends[end].as_ref(), *finished) else {
                    return Err(format!(
                        "{differ} from the chain at an end face the ball's walk never reached, \
                         and only an end it reaches is cut per side"
                    ));
                };
                // The band's own fields, so each side's rail ends where that side's band does.
                let field = |units: &[f64]| {
                    band.field_from(k, units, size)
                        .map_err(|error| format!("{differ}: a side's field: {error:?}"))
                };
                let (own, left_field, right_field) = (
                    field(&larger_of(&left, &right))?,
                    field(&left)?,
                    field(&right)?,
                );
                let side_a = CapSide {
                    side: &in_a,
                    field: &left_field,
                };
                let side_b = CapSide {
                    side: &in_b,
                    field: &right_field,
                };
                let chain_end = chain[i];
                // The end face's own triangles: across the caller's regions where it gives them,
                // else across every edge that is no feature edge.
                let joins = |t: usize, u: usize, a: i32, b: i32| match side_regions {
                    Some(regions) => regions.get(t) == regions.get(u),
                    None => !is_feature_edge.is_some_and(|feature| feature(a, b)),
                };
                let end_face = rolling_ball_end_caps::end_face_triangles(
                    indices,
                    &standing,
                    chain_end,
                    (&in_a, &in_b),
                    &joins,
                );
                let rail = |side: &CapSide<'_>| {
                    rolling_ball_end_caps::rail_end(
                        positions, indices, &standing, chain_end, &end_face, side,
                    )
                    .map_err(|why| format!("{differ} from the chain, and {why}"))
                };
                let (a, b) = (rail(&side_a)?, rail(&side_b)?);
                let shift = rolling_ball_end_caps::cap_through(&own, a, b)
                    .map_err(|why| format!("chain {k} end {end}: {why}"))?;
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

/// The larger of two sides' setbacks at each chain point.
fn larger_of(left: &[f64], right: &[f64]) -> Vec<f64> {
    left.iter()
        .zip(right)
        .map(|(&l, &r)| vector_math::ieee_math::max_f64(l, r))
        .collect()
}

/// How far `contact` stands from the chain's edges either side of chain vertex `v`: the nearer.
/// At an open chain's end on an end face (`on_an_end_face`), the one edge there is carried on
/// past the end vertex as its line ([`from_the_end_edge`]): the end station there touches each
/// side where the side runs on past the face (`rolling_ball::walk`), which past a face leaning
/// back over the end vertex lies beyond it, and the rail there stands that far from the chain
/// carried on past its end, as the band's own field reads it past the end. At a square end the
/// contact lies in the plane square to the chain at the end vertex, where the two agree.
fn from_the_chain(
    positions: &[Vector3],
    edges: &[(i32, i32)],
    is_closed: bool,
    v: usize,
    on_an_end_face: bool,
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
        let (start, stop) = (positions[from as usize], positions[to as usize]);
        let on = if on_an_end_face {
            from_the_end_edge(start, stop, v == 0, contact)
        } else {
            ChainField::closest_on_segment(contact, start, stop)
        };
        let distance = (contact - on).length();
        if distance < nearest {
            nearest = distance;
        }
    }

    nearest
}

/// The point of an open chain's end edge `start -> stop` nearest `contact`, the edge carried on as
/// its line past the chain's end vertex only (`start` at the first end, `stop` at the last) and
/// clamped at its other vertex, toward the chain's interior, where the next edge in takes over.
#[allow(clippy::neg_cmp_op_on_partial_ord)] // `!(x > 0)` on purpose: a NaN reads the vertex
fn from_the_end_edge(start: Vector3, stop: Vector3, first_end: bool, contact: Vector3) -> Vector3 {
    let along = stop - start;
    let length_squared = along.length_squared();
    if !(length_squared > 0.0) {
        return start;
    }
    let t = (contact - start).dot(along) / length_squared;
    let t = if first_end {
        vector_math::ieee_math::min_f64(t, 1.0)
    } else {
        vector_math::ieee_math::max_f64(t, 0.0)
    };
    start + (along * t)
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

#[cfg(test)]
mod tests {
    use super::*;

    /// A chain turning a right angle, (0, 0, 0) to (1, 0, 0) to (1, 1, 0): each end edge is 1 long.
    fn turning() -> (Vec<Vector3>, Vec<(i32, i32)>) {
        let positions = vec![
            Vector3::new(0.0, 0.0, 0.0),
            Vector3::new(1.0, 0.0, 0.0),
            Vector3::new(1.0, 1.0, 0.0),
        ];
        (positions, vec![(0, 1), (1, 2)])
    }

    /// At an end face the end edge runs on as its line past the end vertex only: a contact past
    /// the end stands its distance from that line, and one whose offset along the edge reaches
    /// past the edge's inner vertex stands its distance from that vertex, never from the line.
    #[test]
    fn the_end_edge_runs_on_past_the_end_vertex_only() {
        let (positions, edges) = turning();
        let measure = |v: usize, contact: Vector3| {
            from_the_chain(&positions, &edges, false, v, true, contact)
        };
        // First end: past (0, 0, 0) along -x the line, 1.5 off it.
        assert_eq!(measure(0, Vector3::new(-2.0, 1.5, 0.0)), 1.5);
        // First end, 3 along the edge where it is 1 long: from (1, 0, 0), not the line's 2.
        let inner = measure(0, Vector3::new(3.0, 2.0, 0.0));
        assert!((inner - 8f64.sqrt()).abs() < 1e-15, "{inner}");
        // Last end: past (1, 1, 0) along +y the line, 0.75 off it.
        assert_eq!(measure(2, Vector3::new(0.25, 2.5, 0.0)), 0.75);
        // Last end, 3 back along the edge: from (1, 0, 0).
        let inner = measure(2, Vector3::new(1.5, -3.0, 0.0));
        assert!((inner - 0.5f64.hypot(3.0)).abs() < 1e-15, "{inner}");
    }
}
