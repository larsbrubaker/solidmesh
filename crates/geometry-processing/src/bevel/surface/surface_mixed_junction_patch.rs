//! Closes a JUNCTION - two cuts and a bead at one vertex - with the single torus patch between the
//! two cuts' end rings, and the sector it takes out of the cap face
//! (`Bevel/Surface/SurfaceMixedJunctionPatch.cs`, a static class). Its nested record lands
//! here, one strip standing at the junction ([`SurfaceMixedJunctionPatchArrival`]), with the
//! closing ring's check ([`the_closing_rail_stands`]), the patch's schedule ([`try_schedule`]) and
//! the grid's quads ([`emit`]).

use std::rc::Rc;

use vector_math::Vector3;

use super::{surface_fan, surface_mixed_corner_ring_schedule};
use super::{SurfaceMixedCorner, SurfaceStrip, SurfaceVertexPool};
use crate::bevel::{BevelAdvisory, BevelAdvisoryCode};

/// C# `SurfaceMixedJunctionPatch.Arrival` (a readonly struct): one strip standing at the junction,
/// as the strip ends hand it over.
#[derive(Debug, Clone)]
pub struct SurfaceMixedJunctionPatchArrival {
    /// The strip itself.
    pub strip: Rc<SurfaceStrip>,
    /// Which edge of the whole build it belongs to, which is how its seams are found.
    pub edge_index: i32,
    /// Whether the corner is that strip's FINISH rather than its start.
    pub ends_here: bool,
}

/// Whether the rail the CLOSING ring is reused for (`closing`) really stands where the turn puts
/// it: the `leading` rail carried the row's whole `sweep`, compared in the two cylindrical terms
/// the sweep did not fix - the radius out of the bead's axis and the height along it - against
/// `slack`, the shared faces' own chord sagitta. Returns the verdict and where the turn puts the
/// leading rail, which is where a refusal is named.
///
/// The angle between the two is zero by construction, because `sweep` IS the difference of their
/// angles, so measuring it would only read the same number back.
pub fn the_closing_rail_stands(
    corner: &SurfaceMixedCorner,
    leading: Vector3,
    closing: Vector3,
    sweep: f64,
    slack: f64,
) -> (bool, Vector3) {
    let axis = corner.axis();
    let turned_to = corner.turned_about(leading, sweep);

    let mine_out_of_the_axis = (turned_to - axis.closest_point_to(turned_to)).length();
    let theirs_out_of_the_axis = (closing - axis.closest_point_to(closing)).length();
    let apart_along_the_axis = (closing - turned_to).dot(axis.direction);
    let outwards = theirs_out_of_the_axis - mine_out_of_the_axis;

    let stands =
        ((outwards * outwards) + (apart_along_the_axis * apart_along_the_axis)).sqrt() <= slack;

    (stands, turned_to)
}

/// How near two angles have to be to be one station - an angle, so fixed.
pub const ANGLE_RULER: f64 = 1e-5;

/// Whether two strips really end on ONE vertex, which two rings that meet there have to: near
/// enough is not enough, because two indices a weld apart leave an edge nothing pairs with.
pub fn the_same_vertex(
    pool: &super::SurfaceVertexPool,
    mine: i32,
    theirs: i32,
) -> Result<(), crate::bevel::BevelAdvisory> {
    if mine == theirs {
        return Ok(());
    }

    Err(crate::bevel::BevelAdvisory::new(
        crate::bevel::BevelAdvisoryCode::SurfaceStripSeamDidNotMeet,
        vec![pool.position_of(mine).into()],
    ))
}

impl SurfaceMixedJunctionPatchArrival {
    /// C#'s constructor.
    pub fn new(strip: Rc<SurfaceStrip>, edge_index: i32, ends_here: bool) -> Self {
        SurfaceMixedJunctionPatchArrival {
            strip,
            edge_index,
            ends_here,
        }
    }
}

/// C# `Emit`: the patch's grid of quads, each fanned facing the corner's outward sum.
pub fn emit(
    pool: &mut SurfaceVertexPool,
    faces: &mut Vec<(i32, i32, i32)>,
    corner: &SurfaceMixedCorner,
    grid: &[Vec<i32>],
) {
    let outward = corner.outward_sum();
    for p in 0..grid.len().saturating_sub(1) {
        for j in 0..grid[p].len().saturating_sub(1) {
            let quad = [
                grid[p][j],
                grid[p + 1][j],
                grid[p + 1][j + 1],
                grid[p][j + 1],
            ];
            surface_fan::emit_facing(&quad, pool, faces, outward);
        }
    }
}

/// C# `TrySchedule`: the patch's grid, profile row by row: each row swept from its `profile`
/// vertex round `lead`'s axis to its `closing` vertex, with a station at every one of the shared
/// face's `turns` strictly inside its sweep - minted on the other rows, the turn's own vertex on the
/// shared face (within `slack`). Refused where a row does not sweep forward, a ring's end is not the
/// turn's own vertex (unless that corner hands over at a mitre), or a closing rail does not stand.
#[allow(clippy::too_many_arguments, clippy::neg_cmp_op_on_partial_ord)]
pub fn try_schedule(
    pool: &mut SurfaceVertexPool,
    lead: &SurfaceMixedCorner,
    closes: &SurfaceMixedCorner,
    profile: &[i32],
    closing: &[i32],
    turns: &[i32],
    slack: f64,
) -> Result<Vec<Vec<i32>>, BevelAdvisory> {
    let not_supported = || {
        BevelAdvisory::new(
            BevelAdvisoryCode::SurfaceCornerNotSupported,
            vec![lead.corner.into()],
        )
    };

    if turns.len() < 2 || profile.len() < 2 || closing.len() != profile.len() {
        return Err(not_supported());
    }

    let angle_of = |pool: &SurfaceVertexPool, v: i32| {
        surface_mixed_corner_ring_schedule::angle_of(lead, pool.position_of(v))
    };
    let on_the_shared_face = profile.len() - 1;
    let mut start_angle = vec![0.0; profile.len()];
    let mut sweep = vec![0.0; profile.len()];

    for p in 0..profile.len() {
        start_angle[p] = angle_of(pool, profile[p]);
        sweep[p] = angle_of(pool, closing[p]) - start_angle[p];

        if !(sweep[p] > 0.0) {
            return Err(not_supported());
        }
    }

    let shared_start = start_angle[on_the_shared_face];
    let shared_sweep = sweep[on_the_shared_face];
    let stations: Vec<usize> = (0..turns.len())
        .filter(|&j| {
            let angle = angle_of(pool, turns[j]);
            angle > shared_start + ANGLE_RULER && angle < shared_start + shared_sweep - ANGLE_RULER
        })
        .collect();

    if !lead.hands_over_at_a_mitre {
        the_same_vertex(pool, profile[on_the_shared_face], turns[0])?;
    }
    if !closes.hands_over_at_a_mitre {
        the_same_vertex(pool, closing[on_the_shared_face], turns[turns.len() - 1])?;
    }

    let mut grid = Vec::with_capacity(profile.len());

    for p in 0..profile.len() {
        let mut row = vec![0; stations.len() + 2];
        row[0] = profile[p];

        let (stands, turned_to) = the_closing_rail_stands(
            lead,
            pool.position_of(profile[p]),
            pool.position_of(closing[p]),
            sweep[p],
            slack,
        );
        if !stands {
            return Err(BevelAdvisory::new(
                BevelAdvisoryCode::SurfaceStripSeamDidNotMeet,
                vec![turned_to.into()],
            ));
        }

        row[stations.len() + 1] = closing[p];

        for (station, &turn) in stations.iter().enumerate() {
            let rail = turns[turn];
            let how_far = (angle_of(pool, rail) - shared_start) / shared_sweep;
            let at = lead.turned_about(pool.position_of(profile[p]), how_far * sweep[p]);

            if p != on_the_shared_face {
                row[station + 1] = pool.add(at);
                continue;
            }

            if (pool.position_of(rail) - at).length() > slack {
                return Err(BevelAdvisory::new(
                    BevelAdvisoryCode::SurfaceStripSeamDidNotMeet,
                    vec![at.into()],
                ));
            }

            row[station + 1] = rail;
        }

        grid.push(row);
    }

    Ok(grid)
}
