//! The STOPPED end: an open chain's end where the pick ends but the crease carries on (the next
//! facet of a faceted curve, picked in part), whose round stops at the pick's own end instead of
//! fading over the neighbours. The per-flow gate ([`stopped_ends_on`], [`turn_stopped_ends_on`] and
//! its scope [`StoppedEndClipScope`], kept on the build's [`WorkBudget`](super::WorkBudget) so it
//! travels with the build, off by default) and the clips read off plain positions
//! ([`stopped_end_clips`]); a pick's ends take them in `pick_end_bounds.rs`.
//!
//! Unstopped, such an end keeps the plain field: its end region is the ball of radius setback about
//! the end vertex, which on a short facet chord covers several of the unpicked neighbours, and a
//! pair's two fields can meet only through that ball. Stopped, the end is clipped (as an
//! [`EndRayBound`] of no reach marked `stopped`, read by `ChainField::find_end_clips`) on the MITRE
//! plane through the end vertex: the plane bisecting the chain's direction out of the end and the
//! leaving feature crease's direction, so the band stops where the facet seam between the picked
//! chord and its neighbour would put it. The clip cuts the whole half-space past the plane, so an
//! end whose plane any point of the chain itself stands past is not stopped.
//!
//! The band build and the room readings take their ends from one place (`pick_end_bounds.rs`), so
//! the pick reading follows the clipped bands. Because the clip does not scale with the size, the
//! foreign-edge room (`foreign_edge_room_readings.rs`) reads a stopped field by bisection on the
//! field itself, from the bare field's reading as a lower bound. The pair reading
//! (`band_room_pair_room_pick.rs`) reads a stopped chain's edges as balls of its setback, which its
//! clip cuts back, so where the other chain's band is its balls (no ray, no clip, no shared corner,
//! no faced end) it reads that pair only the other way round, the stopped chain's clipped field
//! against the other's edges; elsewhere the pair is read both ways as before, which can read low
//! but never high. A stopped end also has a room of its own (`stopped_end_room.rs`): the size at
//! which the other members sharing a side cover its stretch of its plane there, leaving it no
//! rail on that side.
//!
//! In the band cut the plane is not a known cut as a knife end's is: it crosses faces the mesh
//! carries on with past the end (the leaving crease leaves the end vertex across it), so the line
//! runs through a vertex in it and the vertex reads on the line (`vertex_side.rs`); the refinement
//! splits each edge leaving such a band corner with neither end inside at its least point
//! (`band_refinement_asking.rs`, rule (i)), which puts the band's side of the corner - the chain's
//! own edge, between two end vertices on their planes - inside the band.
//!
//! The fill cuts a stopped end flat (`stopped_end_fill.rs`): the round keeps its whole profile up
//! to the plane, and the plane shows its section. A point within
//! `ChainField::IN_A_STOPPED_PLANE_STEPS` float steps of the plane reads exactly on it
//! (`ChainField::clip_at`), and the corner where the plane meets each side's rail is put on the
//! line (`end_clip_corners.rs`), so the band's boundary follows the plane to the rail on both
//! sides. Nothing turns the gate on but a caller that asks; the fillet join wires the route to it.
//! The clip's `same_sense` is false (the default): the fill's same-sense trim (`EndPlaneTrim`)
//! trims a face the mesh already has, which a stopped end does not.

use std::cell::Cell;

use vector_math::Vector3;

use super::super::float_grid;
use super::end_face_sense::{
    face_normal, is_convex, Corners, COPLANAR_TOLERANCE, SEPARATE_END_FACE_DEGREES,
};
use super::{EndRayBound, LevelLineWork};

/// Whether the build in force reads stopped ends (off unless a caller turns them on with
/// [`turn_stopped_ends_on`]; off wherever no budget is).
pub fn stopped_ends_on() -> bool {
    LevelLineWork::current().is_some_and(|budget| budget.stopped_ends_on.get())
}

thread_local! {
    /// The flow's own setting, for a scope opened before the build that will read it exists: a
    /// build started on this thread captures it when its budget is created
    /// ([`flow_stopped_ends_on`]).
    static FLOW_STOPPED_ENDS_ON: Cell<bool> = const { Cell::new(false) };
}

/// What a new build's budget starts its stopped-end gate at: on while a [`turn_stopped_ends_on`]
/// scope is open on this flow.
pub fn flow_stopped_ends_on() -> bool {
    FLOW_STOPPED_ENDS_ON.with(Cell::get)
}

/// Turns stopped ends on for the build in force, and for every build started on this flow, until
/// the returned scope is dropped.
pub fn turn_stopped_ends_on() -> StoppedEndClipScope {
    StoppedEndClipScope::new()
}

/// Turns stopped ends on until it is dropped, which puts back what was there before - on the
/// build in force, and on the flow so a build started inside the scope starts with them on.
#[derive(Debug)]
#[must_use = "stopped ends stay on only while the scope is alive"]
pub struct StoppedEndClipScope {
    /// Whether the build in force read stopped ends when the scope was opened.
    pub was: bool,
    /// The flow's own setting when the scope was opened.
    was_in_flow: bool,
}

impl StoppedEndClipScope {
    /// Remembers the setting and turns stopped ends on.
    pub fn new() -> Self {
        let was =
            LevelLineWork::current().is_some_and(|budget| budget.stopped_ends_on.replace(true));
        let was_in_flow = FLOW_STOPPED_ENDS_ON.with(|flow| flow.replace(true));
        StoppedEndClipScope { was, was_in_flow }
    }
}

impl Default for StoppedEndClipScope {
    fn default() -> Self {
        Self::new()
    }
}

impl Drop for StoppedEndClipScope {
    /// Puts back what was there before.
    fn drop(&mut self) {
        if let Some(budget) = LevelLineWork::current() {
            budget.stopped_ends_on.set(self.was);
        }
        FLOW_STOPPED_ENDS_ON.with(|flow| flow.set(self.was_in_flow));
    }
}

fn has(t: Corners, v: i32) -> bool {
    t.0 == v || t.1 == v || t.2 == v
}

/// The stopped-end clip at each UNRAYED end of an open chain whose crease carries on past it, or
/// `None` at an end that is not stopped: a rayed end, a closed chain, an end no feature crease
/// leaves within [`SEPARATE_END_FACE_DEGREES`] of the chain's line, or an end whose plane some
/// point of the chain itself stands past (an open chain over half a faceted circle: its plane would
/// cut the chain's own band, so that end keeps fading). Reads nothing unless [`stopped_ends_on`].
pub fn stopped_end_clips(
    positions: &[Vector3],
    indices: &[Corners],
    chain: &[i32],
    closed: bool,
    rays: (bool, bool),
    is_feature_edge: &dyn Fn(i32, i32) -> bool,
) -> (Option<EndRayBound>, Option<EndRayBound>) {
    if !stopped_ends_on() || closed || chain.len() < 2 {
        return (None, None);
    }

    let n = chain.len();
    let first = if rays.0 {
        None
    } else {
        stop_at(
            positions,
            indices,
            chain,
            chain[0],
            chain[1],
            is_feature_edge,
        )
    };
    let last = if rays.1 {
        None
    } else {
        stop_at(
            positions,
            indices,
            chain,
            chain[n - 1],
            chain[n - 2],
            is_feature_edge,
        )
    };
    (first, last)
}

/// How far past its own end plane a chain point may stand and still read as on it: float noise at
/// the point's magnitude.
const ON_ITS_PLANE_RELATIVE: f64 = 1e-9;

/// The clip at chain end `end`, whose next chain vertex is `next`: the mitre plane between the
/// chain and the feature crease leaving `end` most nearly along the chain's line, when that crease
/// folds the chain's way and turns off the line by less than [`SEPARATE_END_FACE_DEGREES`] (the
/// faceted curve carrying on, read as the oblique-end reader reads a facet), and no point of the
/// chain stands past the plane. `None` otherwise.
fn stop_at(
    positions: &[Vector3],
    indices: &[Corners],
    chain: &[i32],
    end: i32,
    next: i32,
    is_feature_edge: &dyn Fn(i32, i32) -> bool,
) -> Option<EndRayBound> {
    let v = positions[end as usize];
    let outward = (v - positions[next as usize]).get_normal();
    let sides: Vec<Corners> = indices
        .iter()
        .copied()
        .filter(|&t| has(t, end) && has(t, next))
        .collect();
    if sides.len() != 2 {
        return None;
    }

    let chain_convex = is_convex(positions, sides[0], sides[1], end);
    let at_end: Vec<Corners> = indices.iter().copied().filter(|&t| has(t, end)).collect();
    let least_lean = SEPARATE_END_FACE_DEGREES.to_radians().cos();
    // The leaving crease: the most leaning on; a tie goes to the lower vertex index, so the answer
    // does not depend on the triangles' order. Only a feature edge is a crease: a soft
    // triangulation diagonal is not the curve carrying on.
    let mut leaving: Option<(f64, i32)> = None;
    for &t in &at_end {
        for w in [t.0, t.1, t.2] {
            if w == end || w == next || !is_feature_edge(end, w) {
                continue;
            }

            let on_edge: Vec<Corners> = at_end.iter().copied().filter(|&o| has(o, w)).collect();
            if on_edge.len() != 2 {
                continue;
            }

            let fold = face_normal(positions, on_edge[0]).dot(face_normal(positions, on_edge[1]));
            if fold >= 1.0 - COPLANAR_TOLERANCE
                || is_convex(positions, on_edge[0], on_edge[1], end) != chain_convex
            {
                continue;
            }

            let lean = (positions[w as usize] - v).get_normal().dot(outward);
            if lean.is_nan() || lean < least_lean {
                continue;
            }

            let better = match leaving {
                None => true,
                Some((best, at)) => lean > best || (lean == best && w < at),
            };
            if better {
                leaving = Some((lean, w));
            }
        }
    }

    let (_, w) = leaving?;
    let along = (positions[w as usize] - v).get_normal();
    let bisector = outward + along;
    let normal = bisector.get_normal();
    let step = float_grid::step_at_point(v)
        .max(float_grid::step_at_point(positions[next as usize]))
        .max(float_grid::step_at_point(positions[w as usize]));
    let lean = |far: Vector3| 3.0_f64.sqrt() * step / (far - v).length();
    let plane_tilt =
        (lean(positions[next as usize]) + lean(positions[w as usize])) / bisector.length();
    // The plane clips the whole half-space past it: a chain point standing there (an arc over half
    // a circle reaches round behind its own end) would have its own band cut.
    for &c in chain {
        let p = positions[c as usize];
        let past = (p - v).dot(normal);
        if past > ON_ITS_PLANE_RELATIVE * (1.0 + p.length()) {
            return None;
        }
    }

    Some(EndRayBound {
        reach_per_setback: 0.0,
        plane_normal: normal,
        stopped: true,
        plane_tilt,
        ..EndRayBound::default()
    })
}
