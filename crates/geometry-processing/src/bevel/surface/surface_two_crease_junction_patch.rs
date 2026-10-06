//! Closes a TWO-crease junction - one cut and two beads - with the one sweep of the canal between
//! the two beads' own end rings (`Bevel/Surface/SurfaceTwoCreaseJunctionPatch.cs`, a static class;
//! `SurfaceTwoCreaseJunction` reads it, `SurfaceTwoCreaseJunctionRings` lays its grid). ONE sweep,
//! not two half patches: its columns run from the upper bead's end ring round to the lower's, the
//! cut's ring is the last row, and every seam of it is a curve somebody else already minted. Only
//! its nested record (with its constructor), the ring positions ([`positions_of`]) and the three
//! end rings won from the pool ([`try_the_end_rings`]) land here, one strip's end ring being a
//! [`SurfaceTwoCreaseJunctionPatchEndRing`], with the patch's driver ([`try_close`]), its grid
//! ([`try_the_grid`]) and its quads ([`emit`]).

use super::surface_mitre_joint_step::SideSeams;
use super::{
    surface_fan, surface_mixed_corner_patch, surface_split_sewing, surface_two_crease_junction_wall,
};
use super::{
    SurfaceMixedJunction, SurfaceMixedJunctionPatchArrival, SurfaceStripStation,
    SurfaceTwoCreaseJunctionRings, SurfaceVertexPool,
};
use crate::bevel::{BevelAdvisory, BevelAdvisoryCode};

/// C# `SurfaceTwoCreaseJunctionPatch.EndRing` (a private readonly struct): one strip's end ring at
/// the corner, in BOTH of the orders it is wanted in.
///
/// The two are kept apart because they are answers to different questions and the answer is not
/// always the same. The GRID wants every ring running the way it runs - the stations from the first
/// arm's tangency round, the rows from the wall out - and turns a strip's rails over to get it. The
/// STRIP wants its own end ring back in ITS rail order, because that is the order its seams are in
/// and the stitch checks the two rail for rail. Handing the grid's order back to the strip sews it
/// to the seams reversed, which is a strip whose wall side is stitched to its shared face's tangent
/// run - and it only shows where the turning-over actually turns something, which is why a crease
/// chorded beside the corner found it and the whole crease did not.
#[derive(Debug, Clone, PartialEq, Default)]
pub struct SurfaceTwoCreaseJunctionPatchEndRing {
    /// The ring's rails in the STRIP's own order, as vertices of the build's own pool.
    pub ring: Vec<i32>,
    /// Where along its own edge each rail of it meets the plane it was cut on, in that same order.
    pub meets_at: Vec<f64>,
    /// The same rails in the GRID's order.
    pub as_the_grid_wants_it: Vec<i32>,
}

impl SurfaceTwoCreaseJunctionPatchEndRing {
    /// C#'s constructor.
    pub fn new(ring: Vec<i32>, meets_at: Vec<f64>, as_the_grid_wants_it: Vec<i32>) -> Self {
        Self {
            ring,
            meets_at,
            as_the_grid_wants_it,
        }
    }
}

/// C# `TryClose`: closes the junction - the three end rings, the grid between them and what the
/// wall gives up under it. `arriving` is the three strips in the order this patch wants them: the
/// CUT, then the first arm's bead, then the second's (`SurfaceMixedJunction::the_arriving_ends`).
/// Returns one end ring per strip, in the order they were handed in.
pub fn try_close(
    pool: &mut SurfaceVertexPool,
    faces: &mut Vec<(i32, i32, i32)>,
    junction: &SurfaceMixedJunction,
    arriving: &[SurfaceMixedJunctionPatchArrival],
    seams: &[SideSeams],
) -> Result<Vec<SurfaceStripStation>, BevelAdvisory> {
    // Everything added here is new, and whatever of it lands part way along an edge of a face
    // nobody cut is a T junction - so the sew at the end is told where the corner's own vertices
    // start.
    let split_from = pool.vertices().len() as i32;

    let (cut, first, last) = try_the_end_rings(pool, junction, arriving, seams)?;

    let rings = SurfaceTwoCreaseJunctionRings::try_read(
        junction,
        positions_of(pool, &cut.as_the_grid_wants_it),
        positions_of(pool, &first.as_the_grid_wants_it),
        positions_of(pool, &last.as_the_grid_wants_it),
    )?;

    let grid = try_the_grid(
        pool,
        junction,
        &rings,
        &cut.as_the_grid_wants_it,
        &first.as_the_grid_wants_it,
        &last.as_the_grid_wants_it,
    )?;

    surface_two_crease_junction_wall::try_give_up_the_notch(
        pool, faces, junction, &grid, split_from,
    )?;

    emit(pool, faces, junction, &grid);

    // ONCE, and after everything: every crossing this made, sewn into whatever face now carries it
    // part way along an edge (`surface_mixed_corner_patch` says why it is not carried in the
    // rings). C# calls the overload that drops the verdict.
    surface_split_sewing::sew_in_the_splits_since_unchecked(pool, faces, split_from);

    surface_mixed_corner_patch::nothing_covers_nothing(pool, faces)?;

    Ok(vec![
        SurfaceStripStation::clean(cut.ring, cut.meets_at),
        SurfaceStripStation::clean(first.ring, first.meets_at),
        SurfaceStripStation::clean(last.ring, last.meets_at),
    ])
}

/// The pool positions of a ring of vertex indices.
pub fn positions_of(pool: &super::SurfaceVertexPool, ring: &[i32]) -> Vec<vector_math::Vector3> {
    ring.iter().map(|&index| pool.position_of(index)).collect()
}

/// C# `TryTheEndRings`: the three end rings - the cut's, the first arm's bead's and the second's -
/// WON FROM THE POOL: every rail a vertex the build already has, and the two outer rails of each the
/// very vertices the walkback's own tangent runs end at. As `(cut, first, last)`.
///
/// The CUT's ring is cut on the canal's leaning plane rather than on a band's stop plane, because a
/// two-crease junction's cut is bounded on NEITHER face. Each BEAD's is its own square section. All
/// three are then turned the way the grid runs, so that nothing below has to ask which way round a
/// strip's own rails went. `arriving` is the cut's strip then the two beads' (C# reads the three
/// unguarded, and the arm's dual as well; fewer, or no dual, is no two-crease junction and is
/// refused by name).
pub fn try_the_end_rings(
    pool: &mut SurfaceVertexPool,
    junction: &SurfaceMixedJunction,
    arriving: &[SurfaceMixedJunctionPatchArrival],
    seams: &[SideSeams],
) -> Result<
    (
        SurfaceTwoCreaseJunctionPatchEndRing,
        SurfaceTwoCreaseJunctionPatchEndRing,
        SurfaceTwoCreaseJunctionPatchEndRing,
    ),
    BevelAdvisory,
> {
    let (Some(canal), [cut, first, last, ..]) = (junction.first.dual.as_ref(), arriving) else {
        return Err(BevelAdvisory::new(
            BevelAdvisoryCode::SurfaceCornerNotSupported,
            vec![junction.corner.into()],
        ));
    };

    // A negative edge index finds no seam and is refused as the seam not meeting.
    let edge = |arrival: &SurfaceMixedJunctionPatchArrival| {
        usize::try_from(arrival.edge_index).unwrap_or(usize::MAX)
    };

    let (cut_ring, cut_meets_at) = surface_mixed_corner_patch::try_ring(
        pool,
        &junction.first.mixed,
        &cut.strip,
        true,
        seams,
        edge(cut),
        cut.ends_here,
        Some(canal.cut_ring_plane()),
    )?;
    let (first_ring, first_meets_at) = surface_mixed_corner_patch::try_ring(
        pool,
        &junction.first.mixed,
        &first.strip,
        false,
        seams,
        edge(first),
        first.ends_here,
        None,
    )?;
    let (last_ring, last_meets_at) = surface_mixed_corner_patch::try_ring(
        pool,
        &junction.second.mixed,
        &last.strip,
        false,
        seams,
        edge(last),
        last.ends_here,
        None,
    )?;

    let cut_grid = SurfaceTwoCreaseJunctionRings::from_the_first_arms_tangency(junction, &cut_ring);
    let first_grid = SurfaceTwoCreaseJunctionRings::from_the_wall(&junction.first, &first_ring);
    let last_grid = SurfaceTwoCreaseJunctionRings::from_the_wall(&junction.second, &last_ring);

    Ok((
        SurfaceTwoCreaseJunctionPatchEndRing::new(cut_ring, cut_meets_at, cut_grid),
        SurfaceTwoCreaseJunctionPatchEndRing::new(first_ring, first_meets_at, first_grid),
        SurfaceTwoCreaseJunctionPatchEndRing::new(last_ring, last_meets_at, last_grid),
    ))
}

/// C# `TryTheGrid`: the patch's grid, station by station and rail by rail - the first and last
/// rows the two beads' end rings, the last rail the cut's ring, and every other vertex minted at the
/// rings' own position - refused where the closing column does not stand, or a bead's tube contact
/// is not the cut ring's own end rail.
pub fn try_the_grid(
    pool: &mut SurfaceVertexPool,
    junction: &SurfaceMixedJunction,
    rings: &SurfaceTwoCreaseJunctionRings,
    cut_ring: &[i32],
    first_ring: &[i32],
    last_ring: &[i32],
) -> Result<Vec<Vec<i32>>, BevelAdvisory> {
    let (stands, stands_off, _) = rings
        .the_closing_column_stands(SurfaceTwoCreaseJunctionRings::closing_slack(junction, pool));
    if !stands {
        return Err(BevelAdvisory::new(
            BevelAdvisoryCode::SurfaceStripSeamDidNotMeet,
            vec![stands_off.into()],
        ));
    }

    let rails = rings.rails_per_row();
    let stations = rings.station_count();

    // The two corners the rings SHARE are one vertex.
    for (bead, station) in [(first_ring, 0), (last_ring, stations - 1)] {
        if bead[rails - 1] != cut_ring[station] {
            return Err(BevelAdvisory::new(
                BevelAdvisoryCode::SurfaceStripSeamDidNotMeet,
                vec![pool.position_of(cut_ring[station]).into()],
            ));
        }
    }

    let mut grid = Vec::with_capacity(stations);
    #[allow(clippy::needless_range_loop)] // C#'s station-by-rail grid, read by index
    for station in 0..stations {
        let mut row = Vec::with_capacity(rails);
        for rail in 0..rails {
            let vertex = if station == 0 {
                first_ring[rail]
            } else if station == stations - 1 {
                last_ring[rail]
            } else if rail == rails - 1 {
                cut_ring[station]
            } else {
                pool.add(rings.at(rail, station))
            };
            row.push(vertex);
        }
        grid.push(row);
    }

    Ok(grid)
}

/// C# `Emit`: the grid's quads, each fanned facing the junction's outward sum.
pub fn emit(
    pool: &mut SurfaceVertexPool,
    faces: &mut Vec<(i32, i32, i32)>,
    junction: &SurfaceMixedJunction,
    grid: &[Vec<i32>],
) {
    let outward = junction.first.mixed.outward_sum();
    for station in 0..grid.len().saturating_sub(1) {
        for rail in 0..grid[station].len().saturating_sub(1) {
            let quad = [
                grid[station][rail],
                grid[station + 1][rail],
                grid[station + 1][rail + 1],
                grid[station][rail + 1],
            ];
            surface_fan::emit_facing(&quad, pool, faces, outward);
        }
    }
}
