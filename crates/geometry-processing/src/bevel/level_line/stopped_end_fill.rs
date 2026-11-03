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
//! Taken only where BOTH ends of the one chain are stopped and each run starts and ends on the end
//! planes; anything else is read as before (`open_gap_fill::try_fade`).

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

/// The fill of a one-chain gap whose two ends are both stopped, its runs `left` (walking end 0 to
/// end 1) and `right` (end 1 to end 0) meeting at the end vertices; `None` where the chain is not
/// stopped at both ends or a run does not run along both planes to its rail, so the caller fills
/// it as a fade.
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
    let (Some(first), Some(last)) = (chain.end_ray_bounds.0, chain.end_ray_bounds.1) else {
        return Ok(None);
    };
    if !first.stopped || !last.stopped {
        return Ok(None);
    }

    let points = chain.points();
    let planes = [
        (points[0], first.plane_normal.get_normal(), first.plane_tilt),
        (
            points[points.len() - 1],
            last.plane_normal.get_normal(),
            last.plane_tilt,
        ),
    ];
    let (l, r) = (&left.vertices, &right.vertices);
    if l.len() < 2 || r.len() < 2 || l[0] != r[r.len() - 1] || r[0] != l[l.len() - 1] {
        return Ok(None);
    }

    let (left_rail, right_rail, end_runs) = {
        let mesh = mesh_handle.borrow();
        let reversed = |run: &[i32]| run.iter().rev().copied().collect::<Vec<i32>>();
        // How many vertices from each run's start stand in the plane it starts on, and from its
        // end in the plane it ends on. Each run must leave its start plane and reach its end plane
        // at a vertex of its own other than the end vertex (two or more in each plane), with rail
        // between; a run whose end vertex is not read in its plane (a refinement vertex just
        // outside the zone) is no stopped end this fill can lay.
        let counts = [
            leading_in_plane(&mesh, l, planes[0]),
            leading_in_plane(&mesh, &reversed(l), planes[1]),
            leading_in_plane(&mesh, r, planes[1]),
            leading_in_plane(&mesh, &reversed(r), planes[0]),
        ];
        if counts.iter().any(|&count| count < 2) {
            return Ok(None);
        }
        // The last index of each run's stretch on the plane it starts on, and the first of the
        // stretch on the plane it ends on.
        let left_from = counts[0] - 1;
        let left_to = l.len() - counts[1];
        let right_from = counts[2] - 1;
        let right_to = r.len() - counts[3];
        if left_from >= left_to || right_from >= right_to {
            return Ok(None);
        }

        let end_0: Vec<i32> = r[right_to..]
            .iter()
            .chain(l[1..=left_from].iter())
            .copied()
            .collect();
        let end_1: Vec<i32> = l[left_to..]
            .iter()
            .chain(r[1..=right_from].iter())
            .copied()
            .collect();
        (
            stretch(left, left_from, left_to),
            stretch(right, right_from, right_to),
            [end_0, end_1],
        )
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
