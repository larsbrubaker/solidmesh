//! A level-line pass's setbacks read from the rolling ball (`bevel::rolling_ball`), where its
//! switch is on (`rolling_ball::rolling_ball_routing`): the band's field at the size asked takes,
//! at each chain point, how far the ball's contacts stand from the chain there, in place of the
//! setback worked out from the angle at the edge (`chain_on_the_mesh::unit_setbacks`).
//!
//! A contact is measured to the chain's own edges either side of its chain point, the nearer of
//! the two, because that is what the field's setback is: the distance from the chain at which the
//! rail runs. Along a straight run that is the contact's distance from the chain point (the
//! contact lies in the plane square to the chain there). At a turn of the chain it is not: the
//! ball at a gentle turn, rolled square to the vertex's blended direction, touches each side
//! further from the vertex than from either edge. At a turn sharper than the pass's bend threshold
//! where one side's face changes (a cube rim's corner, a pocket floor's), the chain is walked in
//! legs, each ending on the other leg's face, and the turn's setback is the mean of the legs'
//! (`rolling_ball_legs`, whose header says why). At an open chain's end on an end face, the end
//! station touches each side where the walk extends it past the face, which beyond a face leaning
//! back over the end vertex lies past that vertex: there the end edge is read as its line carried
//! on past the end vertex, and clamped toward the chain's interior
//! (`rolling_ball_legs::from_the_end_edge`), as the band's own field reads the chain past its end.
//!
//! **A setback per side.** Each side's contacts give that side's own setbacks ([`SideSetbacks`],
//! left and right as `chain_on_the_mesh::sides_of` names a chain edge's two triangles: the left
//! one runs the edge in the chain's direction, and the ball's side A is the left). Where a side
//! curves away from the edge (a groove wall that bends, the T Handle's wall against its sphere)
//! the two differ, and each side's band is cut at its own distance (`band_group_fields::of` hands
//! each side group its own side's field through `band_group_fields::of_sides`), the band's two
//! side groups tagged from the triangles the ball rolled on wherever the build's own side floods
//! join the two sides into one region ([`BallSides`], `rolling_ball_band_tags`).
//!
//! **Whole pass, never half.** The rolling ball decides the pass only where every chain in it
//! walks: every station solves, both contacts' distances are finite numbers, and each end finishes
//! on an end face or as a plain last station. Anything else refuses the whole pass
//! (`BandGapRefusal::RollingBall`, strict: never built on the level line's own setbacks), and the
//! route says why: a pass with no feature-edge test, a chain whose edges are not all convex or all
//! concave (one ball sits on one side of the part), a walk refused, a run-out end, a distance that
//! is not a finite number.
//!
//! **Corners where picked chains meet** (`pick_validation::corners_of`, setback steps included).
//! The ball supplies each chain's setbacks up to the corner vertex, and the level line's corner
//! fill closes the corner as it does with the switch off (`corner_cap_hole_fill`, whose patch is
//! snapped onto the ball touching all three faces where every row shares one centre). On flat
//! faces, where one flat face of neither side stands round the corner, the ball's end station
//! there touches each side as far out as the angle's setback, r tan(phi / 2): that third face is
//! the chain's end face, so the station's centre is kept on the chain's side of its plane, a radius
//! from both sides, and the two routes agree to the solve's precision. Where several faces of
//! neither side stand round the corner, the end face is one plane through the vertex whose normal
//! is the area-weighted sum of theirs (`end_face`), which is none of them, and the setbacks there
//! need not be the angle's. A corner end is no free end, even with no face of neither side round
//! it (a setback step), so its setbacks are not tapered to the end vertex, and it is never cut by a
//! cap of the chain's own. Where an open chain's sides differ in such a pass, the rule for a pass
//! of several chains below refuses it: a corner whose contacts differ needs the corner's own rows
//! about the ball, not built yet.
//!
//! **Chamfers are outside the ball.** A chamfer is a flat cut at a distance along each face; no
//! ball defines it. A chamfer's pass is never offered to the ball (as with the switch off: no walk,
//! no route, no refusal) and keeps the level line's own construction, a different tool and not a
//! fallback.
//!
//! **An open chain's sides per side.** Where an open chain's two sides differ by more than
//! [`SIDES_AGREE_SHARE`] of the size, each side's rail is cut at its own distance and each end face
//! the round cuts (an end region) is cut by its own cap field ([`CapShift`]), which meets each
//! side's rail exactly where that side's band ends. At every free end (`LastStation` with no end
//! face: the edge runs on as an unpicked crease), whether or not the sides differ there, both
//! sides' setbacks taper to nothing at the end vertex (`rolling_ball_free_ends`), so the round
//! closes onto that vertex. Where they differ the pass
//! is refused, with the route naming the sides' distances, at:
//! - an end face the walk never reached (its last station short of the face's plane): nothing
//!   says where on that face the rails end;
//! - an end face where the end station's contact on a side rests on a hole of that side as the
//!   walk extends it past the face (`rolling_ball::side_extension`; a contact on the side's own
//!   rim there, its far corner in the end face, is the round's): its distance is no setback, so
//!   the pass is refused rather than built short;
//! - an end face whose edges with the walls end before a rail does (narrower than the rail's
//!   distance, or one the band flows over): no point of its edge carries that rail's end
//!   (`rolling_ball_end_caps::rail_end`);
//! - an open chain in a pass of several chains (its end regions are groups of the whole pick, not
//!   tied to one chain end, so no cap is handed them).
//!
//! An open chain whose sides differ and which ends on an oblique end face (an end ray) is the
//! ball's: past such an end each side's rail runs on along that side's face, at that side's own
//! distance from the chain's line, to the end plane. The fill reads each side's own field there
//! (`BandGapResult::side_chains`): the rail carried past the end
//! (`virtual_rail_past_the_end`) and the end-plane trim (`end_plane_trim_run::trim_end`) each
//! carry and cut a side's rail at its own setback.
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
//! The whole part is what the ball's round may not reach past, read where the round can meet it:
//! the ball's segment beyond the chord through its contacts (`rolling_ball::ball_reach`). A
//! chain's end that lies on a face of neither side ends on that face (its plane through the end
//! vertex, normal the area-weighted sum of those faces round the vertex); an end with none is
//! free and the ball rolls on to it.

use std::cell::Cell;
use std::collections::HashSet;

use vector_math::Vector3;

use super::band_gap_setback_fields::PassSetbacks;
use super::rolling_ball_band_tags::BallSides;
use super::rolling_ball_end_caps::{self, CapSide};
use super::rolling_ball_free_ends;
use super::rolling_ball_legs::{self, ChainToWalk};
use super::rolling_ball_refusal::BallRefusal;
use super::triangle_edges;
use super::BandProfile;
use super::{chain_on_the_mesh, pick_validation, rolling_ball_sides};
use crate::bevel::rolling_ball::rolling_ball_routing::{
    record_refusal, set_last_route, use_rolling_ball, ROLLING_BALL_ROUTE,
};
use crate::bevel::rolling_ball::{ContactFeature, EndFace, RollingBallSide, WalkEnd};

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
    /// The ball's centre less its chain point at each chain vertex, per unit of size: where the
    /// round's axis runs, which the growing crease runs' pierce room reads
    /// (`ball_pierce_room`).
    pub centres: Vec<Vector3>,
    /// Side A's (left) and side B's (right) contact less its chain point at each chain vertex, per
    /// unit of size, as the walk found them (a free end's taper does not move them): where each
    /// side's band stops behind the other's contact (`rolling_ball_sweep`).
    pub contacts: [Vec<Vector3>; 2],
    /// What side A's and side B's contact lies on at each chain vertex, in the part's own vertex
    /// numbers (a vertex the walk added past an end face numbered from the part's count up), or
    /// `None` at a station on an end face, where the side is read as extended past the face and
    /// the feature names the extension, not the part: where a side's contacts rest on a rim, that
    /// rim is the side's rail (`rolling_ball_rested_rims`).
    pub features: [Vec<Option<ContactFeature>>; 2],
    /// Whether each end (first, last) is a free end, whose round closes onto its end vertex: no
    /// end face round it, no picked chain meeting it, and the walk's last station there
    /// (`rolling_ball_free_ends`). The end-face fill reads such an end as the round's nose
    /// (`end_face_fill_try`), never as a growing end face.
    pub free_ends: [bool; 2],
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
/// is no probe writes the route it took; a probe writes nothing (see the file header). The build
/// itself asks [`with_the_sides`], which tells a refusal apart from a pass never asked.
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
    let pass = with_the_sides(
        positions,
        indices,
        chains,
        closed,
        is_feature_edge,
        side_regions,
        size,
        profile,
        band,
        probe,
    );
    pass.ok().flatten().map(|(setbacks, _)| setbacks)
}

/// A pass the ball took: one [`SideSetbacks`] and one [`BallSides`] per chain.
pub type BallPass = (Vec<SideSetbacks>, Vec<BallSides>);

/// [`for_the_pass`] with the triangles the ball rolled on, one [`BallSides`] per chain: what the
/// build tags a one-chain band's sides by where its own floods join them
/// (`rolling_ball_band_tags`). `Ok(None)` where the ball was not asked (the switch off, or a
/// probe); `Err` with why and its kind (`rolling_ball_refusal`), recorded as the route, where it
/// was asked and
/// cannot take the whole pass, which the build refuses (`BandGapRefusal::RollingBall`).
#[allow(clippy::too_many_arguments)]
pub fn with_the_sides(
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
) -> Result<Option<BallPass>, BallRefusal> {
    if !asks_the_ball(probe, profile) {
        return Ok(None);
    }

    match walked_setbacks(
        positions,
        indices,
        chains,
        closed,
        is_feature_edge,
        side_regions,
        size,
        band,
    ) {
        Ok(setbacks) => {
            set_last_route(Some(ROLLING_BALL_ROUTE.to_string()));
            Ok(Some(setbacks))
        }
        Err(refusal) => {
            record_refusal(&refusal.why);
            Err(refusal)
        }
    }
}

/// Whether a pass is the ball's to take: the switch is on, it is no topology-only or inner probe
/// (`probe`), and it is a fillet. A chamfer is no ball's to decide (see the file header).
pub fn asks_the_ball(probe: bool, profile: BandProfile) -> bool {
    !probe && profile != BandProfile::Chamfer && use_rolling_ball()
}

/// Every chain's walked setbacks, or why the ball cannot take the pass.
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
    band: &PassSetbacks,
) -> Result<(Vec<SideSetbacks>, Vec<BallSides>), BallRefusal> {
    // The caller's face regions bound each side on their own; only the kernel's flood needs the
    // feature edges.
    if is_feature_edge.is_none() && side_regions.is_none() {
        return Err(BallRefusal::other(
            "the pass has no feature edges to bound its sides".to_string(),
        ));
    }

    // Where picked chains meet, each chain's end there is no end of the round: the corner's own
    // fill closes it (see the file header), so the ends below that would close an open end skip
    // these vertices.
    let corner_vertices: HashSet<i32> = pick_validation::corners_of(chains, closed, true)
        .iter()
        .map(|corner| corner.vertex)
        .collect();

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
    let mut sides = Vec::with_capacity(chains.len());
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
            return Err(BallRefusal::other(format!(
                "chain {k}: edge {e} is {} where its first edge is {}",
                kind(!convex),
                kind(convex)
            )));
        }
        let crosses = |a: i32, b: i32| {
            !on_a_chain.contains(&triangle_edges::key(a, b))
                && !is_feature_edge.is_some_and(|feature| feature(a, b))
        };
        let (in_a, in_b) = rolling_ball_sides::of_the_chain(
            positions,
            indices,
            &standing,
            k,
            (&lefts, &rights),
            side_regions,
            &crosses,
        )?;

        let ends = if closed[k] {
            [None, None]
        } else {
            [
                end_face(positions, indices, chain[0], &in_a, &in_b),
                end_face(positions, indices, chain[chain.len() - 1], &in_a, &in_b),
            ]
        };
        // The walk, leg by leg where the chain turns sharply (`rolling_ball_legs`).
        let walk = rolling_ball_legs::walked(&ChainToWalk {
            positions,
            indices,
            standing: &standing,
            k,
            chain,
            closed: closed[k],
            edges,
            lefts: &lefts,
            rights: &rights,
            convex,
            in_a: &in_a,
            in_b: &in_b,
            ends,
            side_regions,
            crosses: &crosses,
            part: &part,
            size,
            sharp_turn_degrees: band.sharp_turn_degrees(),
        })?;
        let count = walk.readings.len();

        let mut left = Vec::with_capacity(count);
        let mut right = Vec::with_capacity(count);
        let mut centres = Vec::with_capacity(count);
        let mut contacts = [Vec::with_capacity(count), Vec::with_capacity(count)];
        let mut features = [Vec::with_capacity(count), Vec::with_capacity(count)];
        for (i, reading) in walk.readings.iter().enumerate() {
            let (to_a, to_b) = (reading.to_a, reading.to_b);
            if !to_a.is_finite() || !to_b.is_finite() {
                return Err(BallRefusal::other(format!(
                    "chain {k} point {i}: a contact's distance is not a number ({to_a}, {to_b})"
                )));
            }
            if !closed[k] && chains.len() > 1 && !((to_a - to_b).abs() <= SIDES_AGREE_SHARE * size)
            {
                return Err(BallRefusal::other(format!(
                    "chain {k} point {i}: the sides' contacts stand {to_a} and {to_b} from the \
                     chain, and an open chain's end regions in a pass of several chains are not \
                     cut per side yet"
                )));
            }
            left.push(to_a / size);
            right.push(to_b / size);
            centres.push(reading.centre * (1.0 / size));
            for side in 0..2 {
                contacts[side].push(reading.contacts[side] * (1.0 / size));
                features[side].push(reading.features[side]);
            }
        }
        let mut caps = [None, None];
        let mut free = [false, false];
        if !closed[k] {
            // Every free end closes the round onto its end vertex, whether or not the sides
            // agree there: both sides' setbacks taper to nothing at it (`rolling_ball_free_ends`).
            // An end where picked chains meet is no free end, even with no face of neither side
            // round it (two chains meeting in a setback step): the round runs on into the next
            // chain's, so its setback stays the ball's to the corner.
            let at_a_corner =
                [chain[0], chain[chain.len() - 1]].map(|v| corner_vertices.contains(&v));
            let free_ends = [0, 1].map(|end| {
                !at_a_corner[end] && ends[end].is_none() && walk.ends[end] == WalkEnd::LastStation
            });
            free = free_ends;
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
                let i = if end == 0 { 0 } else { count - 1 };
                let (l, r) = (left[i] * size, right[i] * size);
                // A corner end is cut by the corner's fill, never by a cap of the chain's own;
                // its sides differing in a pass of several chains is refused above.
                if free_ends[end] || at_a_corner[end] || (l - r).abs() <= SIDES_AGREE_SHARE * size {
                    continue;
                }
                let differ = format!("chain {k} end {end}: the sides' contacts stand {l} and {r}");
                let (Some(_), WalkEnd::EndFace) = (ends[end].as_ref(), *finished) else {
                    return Err(BallRefusal::other(format!(
                        "{differ} from the chain at an end face the ball's walk never reached, \
                         and only an end it reaches is cut per side"
                    )));
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
        setbacks.push(SideSetbacks {
            left,
            right,
            caps,
            centres,
            contacts,
            features,
            free_ends: free,
        });
        sides.push(BallSides { in_a, in_b });
    }

    Ok((setbacks, sides))
}

/// The larger of two sides' setbacks at each chain point.
fn larger_of(left: &[f64], right: &[f64]) -> Vec<f64> {
    left.iter()
        .zip(right)
        .map(|(&l, &r)| vector_math::ieee_math::max_f64(l, r))
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
