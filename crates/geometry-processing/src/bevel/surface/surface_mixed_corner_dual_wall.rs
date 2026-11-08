//! What the WALL gives up and gains under a leaning one-crease corner's patch - the face rail zero
//! runs across, made to end exactly where that rail runs: its record, where rail zero leaves one
//! crease ([`SurfaceMixedCornerDualWallCrossing`]), how it is found ([`try_the_crossing`]), the
//! bead's foot on its crease ([`try_the_foot`]), a rim's positions ([`rim_of`]) and the root's fan
//! ([`try_fill_the_root`], [`try_fan`]). `surface_mixed_corner_dual_patch` puts these together with
//! the cap's side and the grid.

use solidmesh_math::Vector3;

use super::{surface_fan, SurfaceLine, SurfaceMixedCorner, SurfaceVertexPool};
use crate::bevel::{BevelAdvisory, BevelAdvisoryArg, BevelAdvisoryCode};

/// Where rail zero leaves one crease - the two stations it happens between, and the point.
#[derive(Debug, Clone, Copy, PartialEq, Default)]
pub struct SurfaceMixedCornerDualWallCrossing {
    /// The last station standing on the side the walk started from.
    pub before: i32,
    /// The first standing past it - the neighbour of
    /// [`SurfaceMixedCornerDualWallCrossing::before`].
    pub after: i32,
    /// Where between the two rail zero meets the crease's own line.
    pub at: Vector3,
}

/// The bead's own FOOT on its crease: where the plane that bead's band stops on crosses the crease
/// line. A point the walkback already left there, so it is added to the pool, which welds it onto
/// that vertex rather than beside it.
pub fn try_the_foot(corner: &SurfaceMixedCorner) -> Result<Vector3, BevelAdvisory> {
    let away = SurfaceMixedCorner::away_from_the_corner(&corner.bead, corner.corner);
    let stops_at = corner.bead_stops_at;
    let leaning = away.dot(stops_at.inward);

    if leaning.abs() < 1e-9 {
        // DEFENSIVE: a band that stops on a plane its own edge runs along stops nowhere.
        return Err(BevelAdvisory::new(
            BevelAdvisoryCode::SurfaceCornerNotSupported,
            vec![BevelAdvisoryArg::Vector3(corner.corner)],
        ));
    }

    Ok(corner.corner + (away * ((stops_at.point - corner.corner).dot(stops_at.inward) / leaning)))
}

/// The nose's ROOT between the two crossings, fanned from the CORNER over rail zero - the wall the
/// ball scooped the feature's own tip away from, which nobody has drawn. `grid` is the patch's
/// vertex grid, station by station, rail zero first.
///
/// From the corner because both creases end there, so the fan's two straight sides are the
/// creases themselves and its arc is rail zero: every side of it is a curve the part or the patch
/// already owns. The sense is MEASURED and not assumed - a fan that doubles back has two triangles
/// turning opposite ways, and that is a refusal rather than a sliver wound to look right.
#[allow(clippy::too_many_arguments)]
pub fn try_fill_the_root(
    pool: &mut SurfaceVertexPool,
    faces: &mut Vec<(i32, i32, i32)>,
    corner: Vector3,
    out_of_the_wall: Vector3,
    grid: &[Vec<i32>],
    at_the_bead: SurfaceMixedCornerDualWallCrossing,
    bead_at: i32,
    at_the_edge: SurfaceMixedCornerDualWallCrossing,
    edge_at: i32,
) -> Result<(), BevelAdvisory> {
    let apex = pool.add(corner);
    let mut rails = vec![bead_at];

    let mut station = at_the_bead.after;
    while station <= at_the_edge.before {
        rails.push(grid[station as usize][0]);
        station += 1;
    }

    rails.push(edge_at);

    try_fan(pool, faces, apex, corner, out_of_the_wall, &rails)
}

/// A fan from `apex` over `rails`, refused where it doubles back. What it emitted before the
/// refusal stays in `faces`; the caller refuses the whole corner.
pub fn try_fan(
    pool: &SurfaceVertexPool,
    faces: &mut Vec<(i32, i32, i32)>,
    apex: i32,
    corner: Vector3,
    out_of_the_wall: Vector3,
    rails: &[i32],
) -> Result<(), BevelAdvisory> {
    let mut sweeping = 0.0;

    let mut rail = 0;
    while rail + 1 < rails.len() {
        let from = pool.position_of(rails[rail]);
        let to = pool.position_of(rails[rail + 1]);
        let turning = (from - corner).cross(to - corner).dot(out_of_the_wall);

        if turning == 0.0 || (sweeping != 0.0 && (turning > 0.0) != (sweeping > 0.0)) {
            return Err(BevelAdvisory::new(
                BevelAdvisoryCode::SurfaceCornerNotSupported,
                vec![from.into()],
            ));
        }

        sweeping = turning;

        surface_fan::emit_triangle_facing(
            pool,
            faces,
            apex,
            rails[rail],
            rails[rail + 1],
            out_of_the_wall,
        );

        rail += 1;
    }

    Ok(())
}

impl SurfaceMixedCornerDualWallCrossing {
    /// Rail zero leaves the crease between stations `before` and `after`, at `at`.
    pub fn new(before: i32, after: i32, at: Vector3) -> Self {
        SurfaceMixedCornerDualWallCrossing { before, after, at }
    }
}

/// Where rail zero (`on_the_wall`, walked on from station `from`) crosses `crease` in the wall
/// facing `out_of_the_wall`, as the two stations it falls between and the point between them.
/// Refused where it never does: a wall with no mouth, which is not this corner drawn differently.
#[allow(clippy::neg_cmp_op_on_partial_ord)] // `!(x < 0)` on purpose: a NaN passes.
pub fn try_the_crossing(
    crease: SurfaceLine,
    out_of_the_wall: Vector3,
    on_the_wall: &[Vector3],
    from: usize,
) -> Result<SurfaceMixedCornerDualWallCrossing, BevelAdvisory> {
    let across = crease.direction.cross(out_of_the_wall);
    let mut started = (on_the_wall[from] - crease.point).dot(across);

    for station in (from + 1)..on_the_wall.len() {
        let here = (on_the_wall[station] - crease.point).dot(across);

        if !(started * here < 0.0) {
            started = here;
            continue;
        }

        let previous = on_the_wall[station - 1];
        let was = (previous - crease.point).dot(across);

        return Ok(SurfaceMixedCornerDualWallCrossing::new(
            station as i32 - 1,
            station as i32,
            previous + ((on_the_wall[station] - previous) * (was / (was - here))),
        ));
    }

    Err(BevelAdvisory::new(
        BevelAdvisoryCode::SurfaceCornerNotSupported,
        vec![BevelAdvisoryArg::Vector3(crease.point)],
    ))
}

/// The positions of the rim's vertices `snap_to`, in order.
pub fn rim_of(pool: &SurfaceVertexPool, snap_to: &[i32]) -> Vec<Vector3> {
    snap_to
        .iter()
        .map(|&vertex| pool.position_of(vertex))
        .collect()
}

/// Rail zero of the dual grid crossed with the bead's crease and then the canal's, and the wall
/// given up in three parts - the bead's bite, the root fanned from the corner between the two
/// crossings, and the lune past the canal's. A corner whose wall is an unpicked section is refused.
pub fn try_give_up_the_wall(
    pool: &mut SurfaceVertexPool,
    faces: &mut Vec<(i32, i32, i32)>,
    read: &super::SurfaceMixedCornerDualGrid,
    grid: &[Vec<i32>],
    split_from: i32,
) -> Result<(), BevelAdvisory> {
    let canal = &read.canal;
    let corner = &canal.corner;
    let out_of_the_wall = corner.out_of_the_wall;
    if corner.unpicked_section().is_some() {
        return Err(BevelAdvisory::new(
            BevelAdvisoryCode::SurfaceCornerNotSupported,
            vec![corner.corner.into()],
        ));
    }

    let on_the_wall: Vec<Vector3> = grid
        .iter()
        .map(|rails| pool.position_of(rails[0]))
        .collect();
    let beads_crease = SurfaceLine::new(
        corner.corner,
        SurfaceMixedCorner::away_from_the_corner(&corner.bead, corner.corner),
    );
    let at_the_bead = try_the_crossing(beads_crease, out_of_the_wall, &on_the_wall, 0)?;
    let at_the_edge = try_the_crossing(
        canal.crease,
        out_of_the_wall,
        &on_the_wall,
        at_the_bead.after as usize,
    )?;

    let bead_at = pool.add(at_the_bead.at);
    let edge_at = pool.add(at_the_edge.at);
    try_the_beads_bite(
        pool,
        faces,
        corner,
        out_of_the_wall,
        grid,
        at_the_bead,
        bead_at,
        split_from,
    )?;
    try_fill_the_root(
        pool,
        faces,
        corner.corner,
        out_of_the_wall,
        grid,
        at_the_bead,
        bead_at,
        at_the_edge,
        edge_at,
    )?;
    try_the_lune(
        pool,
        faces,
        read,
        out_of_the_wall,
        grid,
        at_the_edge,
        edge_at,
        split_from,
    )
}

/// The wall bitten from rail zero's start, round the bead's foot on its crease and the crossing,
/// back down the rail to its start.
#[allow(clippy::too_many_arguments)]
pub fn try_the_beads_bite(
    pool: &mut SurfaceVertexPool,
    faces: &mut Vec<(i32, i32, i32)>,
    corner: &SurfaceMixedCorner,
    out_of_the_wall: Vector3,
    grid: &[Vec<i32>],
    crossing: SurfaceMixedCornerDualWallCrossing,
    crossing_at: i32,
    split_from: i32,
) -> Result<(), BevelAdvisory> {
    let foot = try_the_foot(corner)?;
    let mut snap_to = vec![grid[0][0], pool.add(foot), crossing_at];
    let mut station = crossing.before;
    while station > 0 {
        snap_to.push(grid[station as usize][0]);
        station -= 1;
    }

    let rim = rim_of(pool, &snap_to);
    super::surface_convex_bite::try_take(
        pool,
        faces,
        &rim,
        out_of_the_wall,
        corner.corner,
        &snap_to,
        split_from,
    )
}

/// The wall bitten from the canal's crossing along rail zero to the grid's end - nothing where
/// fewer than three distinct points are left.
#[allow(clippy::too_many_arguments)]
pub fn try_the_lune(
    pool: &mut SurfaceVertexPool,
    faces: &mut Vec<(i32, i32, i32)>,
    read: &super::SurfaceMixedCornerDualGrid,
    out_of_the_wall: Vector3,
    grid: &[Vec<i32>],
    crossing: SurfaceMixedCornerDualWallCrossing,
    crossing_at: i32,
    split_from: i32,
) -> Result<(), BevelAdvisory> {
    let mut snap_to = vec![crossing_at];
    for rails in grid.iter().skip(crossing.after as usize) {
        if snap_to[snap_to.len() - 1] != rails[0] {
            snap_to.push(rails[0]);
        }
    }

    if snap_to.len() < 3 {
        return Ok(());
    }

    let rim = rim_of(pool, &snap_to);
    super::surface_convex_bite::try_take(
        pool,
        faces,
        &rim,
        out_of_the_wall,
        read.canal.crease.point,
        &snap_to,
        split_from,
    )
}
