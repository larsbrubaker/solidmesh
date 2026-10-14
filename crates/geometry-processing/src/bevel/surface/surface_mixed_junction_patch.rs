//! Closes a JUNCTION - two cuts and a bead at one vertex - with the single torus patch between the
//! two cuts' end rings, and the sector it takes out of the cap face (the module's free functions).
//! Its record lands here, one strip standing at the junction
//! ([`SurfaceMixedJunctionPatchArrival`]), with the patch's driver ([`try_close`]), the walls under
//! the blend ([`try_bite_the_strips_under_the_blend`]), the closing ring's check
//! ([`the_closing_rail_stands`]), the patch's schedule ([`try_schedule`]), the cap's sector
//! ([`try_bite_the_cap_face`]) and the grid's quads ([`emit`]).

use std::rc::Rc;

use vector_math::Vector3;

use super::surface_mitre_joint_step::SideSeams;
use super::{
    surface_convex_bite, surface_fan, surface_mixed_corner_patch,
    surface_mixed_corner_ring_schedule, surface_split_sewing, surface_two_crease_junction_patch,
};
use super::{
    SurfaceJunctionKind, SurfaceMixedCorner, SurfaceMixedCornerWall, SurfaceMixedJunction,
    SurfaceStrip, SurfaceStripStation, SurfaceVertexPool,
};
use crate::bevel::{BevelAdvisory, BevelAdvisoryCode};

/// One strip standing at the junction, as the strip ends hand it over.
#[derive(Debug, Clone)]
pub struct SurfaceMixedJunctionPatchArrival {
    /// The strip itself.
    pub strip: Rc<SurfaceStrip>,
    /// Which edge of the whole build it belongs to, which is how its seams are found.
    pub edge_index: i32,
    /// Whether the corner is that strip's FINISH rather than its start.
    pub ends_here: bool,
}

/// Closes the junction - the three end rings, the grid between them and the cap's sector.
/// `arriving` is the three strips at the corner: the first cut, the second cut and the bead (for a
/// TWO-crease junction, the cut and then both beads - that corner is a patch of its own,
/// [`surface_two_crease_junction_patch::try_close`]). Returns one end ring per strip, in the order
/// they were handed in. Fewer than three arrivals are refused by name.
pub fn try_close(
    pool: &mut SurfaceVertexPool,
    faces: &mut Vec<(i32, i32, i32)>,
    junction: &SurfaceMixedJunction,
    arriving: &[SurfaceMixedJunctionPatchArrival],
    seams: &[SideSeams],
) -> Result<Vec<SurfaceStripStation>, BevelAdvisory> {
    if junction.kind == SurfaceJunctionKind::TwoCreases {
        // One cut and two beads: the grid below is written about ONE bead's axis, and the canal
        // this corner really is turns about the CUT's - so it is a patch of its own rather than a
        // case of this one, and the arrivals it is handed are the cut once and both beads.
        return surface_two_crease_junction_patch::try_close(
            pool, faces, junction, arriving, seams,
        );
    }

    let not_supported = || {
        BevelAdvisory::new(
            BevelAdvisoryCode::SurfaceCornerNotSupported,
            vec![junction.corner.into()],
        )
    };

    // Which arm LEADS: the one whose ring does NOT lean - First where neither does, since then the
    // two choices are the same grid. The lead is the frame everything below is read in, and every
    // interior vertex of the patch is a vertex of the LEADING ring carried round the axis, so the
    // lean is left where it can only be REUSED - the closing column. Only one of the two may lean
    // at all: two leaning rings is a corner of its own.
    let first_leans = junction.first.mixed.hands_over_at_a_mitre;
    let second_leans = junction.second.mixed.hands_over_at_a_mitre;
    let lead_is_first = !first_leans || second_leans;

    if first_leans && second_leans {
        return Err(not_supported());
    }

    // Unreachable from real input: the only caller (`surface_strip_ends::try_close_the_junction`)
    // builds `arriving` from `the_arriving_ends()`, a fixed three, and refuses before calling if
    // any of their strips is missing - so the three reads below never see fewer.
    let [first_arrival, second_arrival, bead_arrival, ..] = arriving else {
        return Err(not_supported());
    };

    let (lead, closes) = if lead_is_first {
        (&junction.first.mixed, &junction.second.mixed)
    } else {
        (&junction.second.mixed, &junction.first.mixed)
    };
    let (lead_arrival, close_arrival) = if lead_is_first {
        (first_arrival, second_arrival)
    } else {
        (second_arrival, first_arrival)
    };

    // A negative edge index finds no seam and is refused as the seam not meeting.
    let edge = |arrival: &SurfaceMixedJunctionPatchArrival| {
        usize::try_from(arrival.edge_index).unwrap_or(usize::MAX)
    };

    // Everything added here is new, and whatever of it lands part way along an edge of a face
    // nobody cut is a T junction - so the sew at the end is told where the corner's own vertices
    // start.
    let split_from = pool.vertices().len() as i32;

    let (lead_ring, lead_meets_at) = surface_mixed_corner_patch::try_ring(
        pool,
        lead,
        &lead_arrival.strip,
        true,
        seams,
        edge(lead_arrival),
        lead_arrival.ends_here,
        None,
    )?;
    let (close_ring, close_meets_at) = surface_mixed_corner_patch::try_ring(
        pool,
        closes,
        &close_arrival.strip,
        true,
        seams,
        edge(close_arrival),
        close_arrival.ends_here,
        None,
    )?;
    let (bead_ring, bead_meets_at) = surface_mixed_corner_patch::try_ring(
        pool,
        lead,
        &bead_arrival.strip,
        false,
        seams,
        edge(bead_arrival),
        bead_arrival.ends_here,
        None,
    )?;

    // All three read from the same end, whichever way each strip's own rails run: both profiles
    // from the cap face round to the face that cut shares with the bead, and the bead's from the
    // leading cut's tangency round to the closing one's.
    let profile = surface_mixed_corner_patch::from_the_cap_face(lead, &lead_ring);
    let closing = surface_mixed_corner_patch::from_the_cap_face(closes, &close_ring);
    let turns = surface_mixed_corner_patch::from_the_handover(lead, &bead_ring);

    // How far a reused rail may stand from where the turn puts it: the widest chord sagitta either
    // shared face carries, which is what the mesh itself already stands off the true surface there.
    let slack = surface_mixed_corner_patch::the_shared_faces_own_sagitta(lead, pool).max(
        surface_mixed_corner_patch::the_shared_faces_own_sagitta(closes, pool),
    );

    let grid = try_schedule(pool, lead, closes, &profile, &closing, &turns, slack)?;
    try_bite_the_cap_face(pool, faces, lead, closes, &grid, split_from)?;
    try_bite_the_strips_under_the_blend(pool, faces, junction, split_from)?;

    emit(pool, faces, lead, &grid);

    // ONCE, and after everything: every crossing the bites made, sewn into whatever face now
    // carries it part way along an edge. The sewing's verdict is not needed here.
    surface_split_sewing::sew_in_the_splits_since_unchecked(pool, faces, split_from);

    surface_mixed_corner_patch::nothing_covers_nothing(pool, faces)?;

    let lead_station = SurfaceStripStation::clean(lead_ring, lead_meets_at);
    let close_station = SurfaceStripStation::clean(close_ring, close_meets_at);
    let bead_station = SurfaceStripStation::clean(bead_ring, bead_meets_at);

    Ok(if lead_is_first {
        vec![lead_station, close_station, bead_station]
    } else {
        vec![close_station, lead_station, bead_station]
    })
}

/// The wall each arm stands over, where it stands over any - the stretch of the face it shares with
/// the bead between the bead's own stop plane and the cap, facet by facet
/// ([`SurfaceMixedCornerWall::strip_under_the_blend`]).
///
/// Owed exactly where the arm hands over on a MITRE ring, which is where the bead's band stops at
/// its own height on that face rather than running out to the cap rim - the same reading the plan's
/// band ends make. Where both arms hand over on their own tangency nothing is owed at all.
pub fn try_bite_the_strips_under_the_blend(
    pool: &mut SurfaceVertexPool,
    faces: &mut Vec<(i32, i32, i32)>,
    junction: &SurfaceMixedJunction,
    split_from: i32,
) -> Result<(), BevelAdvisory> {
    for arm in [&junction.first.mixed, &junction.second.mixed] {
        if !arm.both_bands_stop_on_the_shared_face() {
            continue;
        }

        let rolled = if arm.rolled_is_the_shared_face() {
            arm.rolled_section()
        } else {
            None
        };
        let wall = SurfaceMixedCornerWall::try_read_section(
            arm,
            rolled.as_deref(),
            arm.shared_face_foot(),
        )?;

        for (piece, normal) in wall.strip_under_the_blend() {
            let rim = piece.to_vec();
            let snap_to: Vec<i32> = piece.iter().map(|&at| pool.add(at)).collect();

            // Each piece in its OWN facet's plane, named by a point of that piece: on a rolled bead
            // the facets past the first do not pass through the corner at all.
            surface_convex_bite::try_take(
                pool, faces, &rim, normal, piece[0], &snap_to, split_from,
            )?;
        }
    }

    Ok(())
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
    /// One strip standing at the junction, on edge `edge_index`, ending here or starting here.
    pub fn new(strip: Rc<SurfaceStrip>, edge_index: i32, ends_here: bool) -> Self {
        SurfaceMixedJunctionPatchArrival {
            strip,
            edge_index,
            ends_here,
        }
    }
}

/// The patch's grid of quads, each fanned facing the corner's outward sum.
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

/// The patch's grid, profile row by row: each row swept from its `profile` vertex round `lead`'s
/// axis to its `closing` vertex, with a station at every one of the shared face's `turns` strictly
/// inside its sweep - minted on the other rows, the turn's own vertex on the shared face (within
/// `slack`). Refused where a row does not sweep forward, a ring's end is not the turn's own vertex
/// (unless that corner hands over at a mitre), or a closing rail does not stand.
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

/// The cap face bitten out from the sector's apex (between `corner` and the corner it `closes` on)
/// round the grid's outer rail.
pub fn try_bite_the_cap_face(
    pool: &mut SurfaceVertexPool,
    faces: &mut Vec<(i32, i32, i32)>,
    corner: &SurfaceMixedCorner,
    closes: &SurfaceMixedCorner,
    grid: &[Vec<i32>],
    split_from: i32,
) -> Result<(), BevelAdvisory> {
    let outer_rail = &grid[0];
    let apex = super::surface_mixed_corner_sector::try_apex_between(
        corner,
        closes,
        pool.position_of(outer_rail[0]),
        pool.position_of(outer_rail[outer_rail.len() - 1]),
    )?;

    let mut rim = Vec::with_capacity(outer_rail.len() + 1);
    rim.push(apex);
    let mut snap_to = Vec::with_capacity(outer_rail.len() + 1);
    snap_to.push(pool.add(apex));
    for &vertex in outer_rail {
        rim.push(pool.position_of(vertex));
        snap_to.push(vertex);
    }

    super::surface_convex_bite::try_take(
        pool,
        faces,
        &rim,
        corner.out_of_the_cap_face,
        corner.corner,
        &snap_to,
        split_from,
    )
}
