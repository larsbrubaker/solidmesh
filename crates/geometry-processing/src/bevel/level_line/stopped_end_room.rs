//! The room a STOPPED end (`stopped_end_clip.rs`) leaves its chain on a side it shares with other
//! picked chains ([`stopped_end_room`]), one of the pick's room readings
//! (`band_room_pair_room_pick.rs`).
//!
//! A stopped end's band ends on its plane, and on each side the chain keeps a piece of the kept
//! boundary there: the stretch of the plane inside the chain's own band. Another picked chain's
//! band grows over that stretch as the size grows; once the other chains' bands cover all of it,
//! the chain's rail on that side has gone into theirs and the side leaves the chain no loop of its
//! own, so the band does not build. The room is the largest size at which some point of the
//! stretch stands outside every other member's band. A member through the end vertex itself meets
//! the chain at a corner there and carries its line on, so it is not read.
//!
//! The stretch is the plane's cut through the side's triangles, read at fixed samples: a sample is
//! in it where the chain's own band holds it apart from this end's clip (its raw field below zero,
//! and past none of the chain's other end clips). The rail the chain keeps there ends on the plane
//! where the other members' bands meet it, and the band's loop test gives a vertex to every member
//! whose field TIES its owner's there (`corner_fill::fields_tie`, the pick's one tie rule); a
//! stretch narrower than that tie leaves the chain no vertex of its own, so a sample counts as kept
//! only where every other member's field stands above zero and does not tie the chain's own there.
//! A missed uncovered sliver reads as covered, so the samples can read the room low but never high.
//!
//! A side whose plane cut holds no sample of the chain's band at a size keeps nothing on the plane
//! to lose there, and counts as keeping. What is kept need not be monotone in the size (a chain
//! whose setback grows faster than the others' can outgrow their cover again), so the size is
//! first scanned upward from zero in [`SIZE_SCAN_STEPS`] equal steps to the room already read, and
//! the first scanned size that keeps nothing is bisected against the step below it. A covered
//! window narrower than one scan step, between two kept sizes, is not seen.

use solidmesh_math::{ieee_math, Vector3};

use super::super::float_grid;
use super::{corner_fill, ChainField, LevelLineWork, ScalarField, WorkError};

/// Samples along each piece of the plane's cut through a side triangle, ends included.
const SAMPLES_PER_PIECE: i32 = 32;

/// Equal steps the size is scanned in, from zero to the room already read, before bisecting.
pub const SIZE_SCAN_STEPS: i32 = 16;

/// Bisection steps on the size, inside one scan step.
const SIZE_BISECTION_STEPS: i32 = 50;

/// One stopped end: its plane (through `vertex`, facing `normal` past the end), the chain it ends
/// and that end's vertex index.
#[derive(Debug, Clone, Copy)]
pub struct StoppedEnd {
    /// The chain the end belongs to.
    pub chain: usize,
    /// The end vertex's mesh index.
    pub end_vertex: i32,
    /// The end vertex's position, which the plane passes through.
    pub vertex: Vector3,
    /// The plane's unit normal, faced on past the end.
    pub normal: Vector3,
}

/// The pieces of the end's plane inside the triangles tagged `tag`: each a segment, a triangle
/// with an edge in the plane giving that edge.
fn plane_pieces(
    positions: &[Vector3],
    indices: &[(i32, i32, i32)],
    tags: &[i32],
    tag: i32,
    end: &StoppedEnd,
    reach: f64,
) -> Result<Vec<(Vector3, Vector3)>, WorkError> {
    let mut pieces = Vec::new();
    for (t, &(a, b, c)) in indices.iter().enumerate() {
        LevelLineWork::charge(1)?;
        if tags[t] != tag {
            continue;
        }

        let corners = [a, b, c].map(|v| positions[v as usize]);
        // Nothing past the reach of the chain's widest band can be in it.
        if corners.iter().all(|&p| (p - end.vertex).length() > reach) {
            continue;
        }

        let across = corners.map(|p| {
            let d = (p - end.vertex).dot(end.normal);
            if d.abs() <= 4.0 * float_grid::step_at_point(p) {
                0.0
            } else {
                d
            }
        });
        let mut points: Vec<Vector3> = Vec::with_capacity(3);
        for k in 0..3 {
            if across[k] == 0.0 {
                points.push(corners[k]);
            }
        }

        for k in 0..3 {
            let (p, q) = (corners[k], corners[(k + 1) % 3]);
            let (dp, dq) = (across[k], across[(k + 1) % 3]);
            if dp != 0.0 && dq != 0.0 && (dp < 0.0) != (dq < 0.0) {
                points.push(p + ((q - p) * (dp / (dp - dq))));
            }
        }

        if points.len() == 2 {
            pieces.push((points[0], points[1]));
        }
    }

    Ok(pieces)
}

/// Whether `p` is in the chain's own band apart from the stopped `end`'s clip: its raw field below
/// zero and past none of the chain's other end clips (the clip at the chain's far end, which the
/// raw field does not read).
fn in_own_band(own: &ChainField, end: &StoppedEnd, p: Vector3) -> Result<bool, WorkError> {
    if own.unclipped_value_at(p)? >= 0.0 {
        return Ok(false);
    }

    for clip in own.end_clips.iter().flatten() {
        if clip.vertex == end.vertex {
            continue;
        }

        if (p - clip.vertex).dot(clip.normal) > 0.0 {
            return Ok(false);
        }
    }

    Ok(true)
}

/// The first of `others` (by position) whose band covers `p` for the chain `own`: its field at or
/// below zero there, or tying the chain's own by the pick's tie rule; `None` where none does.
fn covering_member(
    own: &ChainField,
    others: &[ChainField],
    p: Vector3,
) -> Result<Option<usize>, WorkError> {
    for (i, other) in others.iter().enumerate() {
        // A NaN reads as covering, so a broken reading lowers the room rather than raising it.
        let value = other.value_at(p)?;
        if value.is_nan() || value <= 0.0 || corner_fill::fields_tie(own, other, p)? {
            return Ok(Some(i));
        }
    }

    Ok(None)
}

/// What one size leaves the chain on the plane.
enum PlaneAtSize {
    /// No sample of the cut is in the chain's band: nothing on the plane to lose.
    NoStretch,
    /// This sample of the stretch stands outside every other member's band.
    Keeps(Vector3),
    /// Every sample of the stretch is covered.
    Covered,
}

/// The samples along the pieces, ends included, in piece order.
fn samples(pieces: &[(Vector3, Vector3)]) -> impl Iterator<Item = Vector3> + '_ {
    pieces.iter().flat_map(|&(a, b)| {
        (0..=SAMPLES_PER_PIECE)
            .map(move |i| Vector3::lerp(a, b, f64::from(i) / f64::from(SAMPLES_PER_PIECE)))
    })
}

/// What the plane's pieces keep the chain against the others.
fn plane_at_size(
    pieces: &[(Vector3, Vector3)],
    end: &StoppedEnd,
    own: &ChainField,
    others: &[ChainField],
) -> Result<PlaneAtSize, WorkError> {
    let mut any_in_stretch = false;
    for p in samples(pieces) {
        if !in_own_band(own, end, p)? {
            continue;
        }

        any_in_stretch = true;
        if covering_member(own, others, p)?.is_none() {
            return Ok(PlaneAtSize::Keeps(p));
        }
    }

    Ok(if any_in_stretch {
        PlaneAtSize::Covered
    } else {
        PlaneAtSize::NoStretch
    })
}

/// A stopped end's room on one side, and the member whose band covers the last of its stretch.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct StoppedEndRoom {
    /// The largest size read at which the chain still keeps a piece of its stretch.
    pub room: f64,
    /// The member (an index into the chains) whose band covers, just past the room, the last point
    /// of the stretch kept at the room.
    pub covered_by: usize,
}

/// The room the stopped `end` leaves on the side tagged `tag` against the `others` sharing it, read
/// below `under` (the least room already read, or the size every candidate is named under): `None`
/// where the chain still keeps a piece at every size scanned up to `under`, or where no other
/// member is read. `field_at(k, s)` is chain k's field at size s, as the band is cut on it;
/// `widest_unit` the chain's largest unit setback, which bounds where its band can reach at
/// `under`.
#[allow(clippy::too_many_arguments)]
pub fn stopped_end_room(
    positions: &[Vector3],
    indices: &[(i32, i32, i32)],
    tags: &[i32],
    tag: i32,
    end: &StoppedEnd,
    others: &[usize],
    field_at: &dyn Fn(usize, f64) -> ChainField,
    widest_unit: f64,
    under: f64,
) -> Result<Option<StoppedEndRoom>, WorkError> {
    if others.is_empty() || !under.is_finite() || under <= 0.0 {
        return Ok(None);
    }

    let pieces = plane_pieces(
        positions,
        indices,
        tags,
        tag,
        end,
        ieee_math::max_f64(0.0, widest_unit) * under,
    )?;
    if pieces.is_empty() {
        return Ok(None);
    }

    let fields_at = |s: f64| -> (ChainField, Vec<ChainField>) {
        (
            field_at(end.chain, s),
            others.iter().map(|&k| field_at(k, s)).collect(),
        )
    };
    let keeps_at = |s: f64| -> Result<Option<Vector3>, WorkError> {
        let (own, other_fields) = fields_at(s);
        Ok(match plane_at_size(&pieces, end, &own, &other_fields)? {
            PlaneAtSize::NoStretch => Some(end.vertex),
            PlaneAtSize::Keeps(p) => Some(p),
            PlaneAtSize::Covered => None,
        })
    };

    // The first scanned size that keeps nothing, and the kept size and point a step below it.
    let mut lo = 0.0;
    let mut kept_point = end.vertex;
    let mut hi = None;
    for i in 1..=SIZE_SCAN_STEPS {
        let s = under * (f64::from(i) / f64::from(SIZE_SCAN_STEPS));
        match keeps_at(s)? {
            Some(p) => {
                lo = s;
                kept_point = p;
            }
            None => {
                hi = Some(s);
                break;
            }
        }
    }

    let Some(mut hi) = hi else {
        return Ok(None);
    };

    for _ in 0..SIZE_BISECTION_STEPS {
        let mid = 0.5 * (lo + hi);
        match keeps_at(mid)? {
            Some(p) => {
                lo = mid;
                kept_point = p;
            }
            None => hi = mid,
        }
    }

    // Who covers, at the first covered size, the last point the chain kept.
    let (own, other_fields) = fields_at(hi);
    let covered_by =
        covering_member(&own, &other_fields, kept_point)?.map_or(others[0], |i| others[i]);
    Ok(Some(StoppedEndRoom {
        room: lo,
        covered_by,
    }))
}
