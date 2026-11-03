//! The open fill at STOPPED ends (`stopped_end_clip.rs`): the round keeps its full profile up to
//! each end plane and is cut flat there. The band of a stopped chain stops on its end planes, so
//! each side's run walks from the end vertex along the plane to the rail, along the rail, and back
//! along the other plane to the other end vertex; read as a fade, the strip's rows would taper
//! into the end vertices. Here each run's stretches in the planes are moved off the rails onto the
//! end runs instead (end 0: the right run's stretch on the first plane, through the end vertex, then
//! the left run's; end 1 the same way round), and the strip is laid by the open fill with those end
//! runs (`open_gap_fill_fill::fill`). An end run lying in its plane is a flat end face to that
//! fill, so its last row is the profile's section in the plane and the hole between that row and
//! the end run is closed flat in the plane (`corner_cap_flat_ends::closes_flat`). Every corner of
//! the cap is a vertex the cut or the strip already has, shared by index.
//!
//! Each end is cut by [`cut_flat_at`], shared with the end-face fill (`end_face_fill_try.rs`),
//! which cuts a stopped end flat the same way where the chain's other end is on its end face.
//! [`try_stopped_ends`] is taken only where BOTH ends of the one chain are stopped and each run
//! starts and ends on the end planes; anything else is read as before (`open_gap_fill::try_fade`).
//! A chain stopped at ONE end only with its other end at a corner where two creases leave is such
//! a fade: its band still stops on the stopped end's plane, so the neighbours past it stay sharp,
//! but the strip's rows taper into that end vertex instead of showing the section there.

use std::cell::RefCell;
use std::rc::Rc;

use vector_math::Vector3;

use super::open_gap_fill_fill::{self, OpenFillEnds};
use super::{
    BandGapResult, BandProfile, ChainField, GapFillResult, GapLoop, LevelLineMesh,
    LevelLineMeshError,
};

/// The loop's stretch `from..=to`, its readings cut to match.
fn stretch(gap_loop: &GapLoop, from: usize, to: usize) -> GapLoop {
    let mut part = gap_loop.clone();
    part.vertices = gap_loop.vertices[from..=to].to_vec();
    part.parameters = gap_loop.parameters[from..=to].to_vec();
    part.vertex_owners = gap_loop
        .vertex_owners
        .as_ref()
        .map(|owners| owners[from..=to].to_vec());
    part.vertex_ties = gap_loop
        .vertex_ties
        .as_ref()
        .map(|ties| ties[from..=to].to_vec());
    part
}

/// How many of the run's vertices from its start stand in the plane, read as the band cut reads a
/// stopped plane ([`ChainField::on_a_stopped_plane`]).
fn leading_in_plane(
    mesh: &LevelLineMesh,
    run: &[i32],
    (origin, normal, tilt): (Vector3, Vector3, f64),
) -> usize {
    run.iter()
        .take_while(|&&v| ChainField::on_a_stopped_plane(origin, normal, tilt, mesh.position(v)))
        .count()
}

/// The stopped plane at the chain's first end (`last` false) or last end, as (end point, unit
/// normal, tilt); `None` where that end is not stopped.
pub fn stopped_plane(chain: &ChainField, last: bool) -> Option<(Vector3, Vector3, f64)> {
    let bound = if last {
        chain.end_ray_bounds.1
    } else {
        chain.end_ray_bounds.0
    }?;
    if !bound.stopped {
        return None;
    }

    let points = chain.points();
    let point = if last {
        points[points.len() - 1]
    } else {
        points[0]
    };
    Some((point, bound.plane_normal.get_normal(), bound.plane_tilt))
}

/// One stopped end cut flat: the runs `left` (walking end 0 to end 1) and `right` (end 1 to end
/// 0), which meet at that end's vertex, with their stretches in the end's `plane` taken off them,
/// and the end run those stretches make (end 0: the right run's stretch, through the end vertex,
/// then the left run's; end 1 the same way round). `None` where the runs do not meet there, or a
/// run does not reach the plane at a vertex of its own other than the end vertex (two or more in
/// the plane) with rail left between, so the caller reads that end as before.
pub fn cut_flat_at(
    mesh: &LevelLineMesh,
    left: &GapLoop,
    right: &GapLoop,
    plane: (Vector3, Vector3, f64),
    last: bool,
) -> Option<(GapLoop, GapLoop, Vec<i32>)> {
    let (l, r) = (&left.vertices, &right.vertices);
    if l.len() < 2 || r.len() < 2 {
        return None;
    }

    let reversed = |run: &[i32]| run.iter().rev().copied().collect::<Vec<i32>>();
    if last {
        if l[l.len() - 1] != r[0] {
            return None;
        }

        // The first index of the left run's stretch on the plane, and the last of the right's.
        let left_to = l
            .len()
            .checked_sub(leading_in_plane(mesh, &reversed(l), plane))?;
        let right_from = leading_in_plane(mesh, r, plane).checked_sub(1)?;
        if left_to == 0 || left_to >= l.len() - 1 || right_from == 0 || right_from >= r.len() - 1 {
            return None;
        }

        let end: Vec<i32> = l[left_to..]
            .iter()
            .chain(r[1..=right_from].iter())
            .copied()
            .collect();
        Some((
            stretch(left, 0, left_to),
            stretch(right, right_from, r.len() - 1),
            end,
        ))
    } else {
        if r[r.len() - 1] != l[0] {
            return None;
        }

        // The last index of the left run's stretch on the plane, and the first of the right's.
        let left_from = leading_in_plane(mesh, l, plane).checked_sub(1)?;
        let right_to = r
            .len()
            .checked_sub(leading_in_plane(mesh, &reversed(r), plane))?;
        if left_from == 0 || left_from >= l.len() - 1 || right_to == 0 || right_to >= r.len() - 1 {
            return None;
        }

        let end: Vec<i32> = r[right_to..]
            .iter()
            .chain(l[1..=left_from].iter())
            .copied()
            .collect();
        Some((
            stretch(left, left_from, l.len() - 1),
            stretch(right, 0, right_to),
            end,
        ))
    }
}

/// The fill of a one-chain gap whose two ends are both stopped, its runs `left` (walking end 0 to
/// end 1) and `right` (end 1 to end 0) meeting at the end vertices, each end cut flat
/// ([`cut_flat_at`]); `None` where the chain is not stopped at both ends or either end cannot be
/// cut flat, so the caller fills it as a fade.
pub fn try_stopped_ends(
    gap: &BandGapResult,
    mesh_handle: &Rc<RefCell<LevelLineMesh>>,
    left: &GapLoop,
    right: &GapLoop,
    profile: BandProfile,
    fillet_segments: i32,
) -> Result<Option<GapFillResult>, LevelLineMeshError> {
    let Some(chain) = gap.chains.first() else {
        return Ok(None);
    };
    let (Some(first_plane), Some(last_plane)) =
        (stopped_plane(chain, false), stopped_plane(chain, true))
    else {
        return Ok(None);
    };

    let cut = {
        let mesh = mesh_handle.borrow();
        cut_flat_at(&mesh, left, right, first_plane, false).and_then(|(left, right, end_0)| {
            cut_flat_at(&mesh, &left, &right, last_plane, true)
                .map(|(left, right, end_1)| (left, right, [end_0, end_1]))
        })
    };
    let Some((left_rail, right_rail, end_runs)) = cut else {
        return Ok(None);
    };

    Ok(Some(open_gap_fill_fill::fill(
        mesh_handle,
        chain,
        &left_rail,
        &right_rail,
        profile,
        fillet_segments,
        &mut Vec::new(),
        OpenFillEnds {
            end_runs: Some(&end_runs),
            flowed: gap.unflowed_room.is_some(),
            ..OpenFillEnds::default()
        },
    )?))
}
