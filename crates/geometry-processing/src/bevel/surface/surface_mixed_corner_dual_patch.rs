//! Closes a LEANING one-crease mixed corner on its DUAL canal. Ports
//! `Bevel/Surface/SurfaceMixedCornerDualPatch.cs`: one grid from the bead's own end ring round to
//! the point the blend dies on the crease the cap and the wall meet in, with what the wall
//! (`surface_mixed_corner_dual_wall`) and the cap (`surface_mixed_corner_dual_cap`) give up under
//! it. The grid itself is laid by [`SurfaceMixedCornerDualGrid`] and its reach is
//! [`SurfaceMixedCornerDualCanal`]'s; what is left here is winning the two end rings from the pool
//! so that the seams the walkback already minted are REUSED rather than made again, winding the
//! quads, and fanning the last band to the one point the blend dies on.
//!
//! The wall gives up three pieces along rail zero - the convex bite nearest the bead, the root
//! fanned from the corner between the two crease crossings, and the lune past the second - and the
//! cap gives up what the row's far rail bounds once the cut's strip has ended.

use std::rc::Rc;

use vector_math::Vector3;

use super::surface_fan;
use super::surface_mitre_joint_step::SideSeams;
use super::surface_mixed_corner_dual_cap::try_give_up_the_cap;
use super::surface_mixed_corner_dual_wall::try_give_up_the_wall;
use super::surface_mixed_corner_patch::{nothing_covers_nothing, try_ring};
use super::surface_split_sewing::sew_in_the_splits_since_unchecked;
use super::{
    SurfaceMixedCorner, SurfaceMixedCornerDualCanal, SurfaceMixedCornerDualGrid, SurfaceStrip,
    SurfaceStripStation, SurfaceVertexPool,
};
use crate::bevel::{BevelAdvisory, BevelAdvisoryCode};

/// C# `SurfaceMixedCornerDualPatch.TryClose`: closes the corner, or says why it could not be
/// closed. `seams` are every edge's two tangent runs, by edge index and side; `cut_edge` and
/// `bead_edge` which edges of the pass the cut and the bead are; `cut_ends_here`/`bead_ends_here`
/// whether each one's own edge ENDS at this corner rather than starting. As
/// `(cut_station, bead_station)`: the rings the cut and the bead are left standing on.
#[allow(clippy::too_many_arguments)]
pub fn try_close(
    pool: &mut SurfaceVertexPool,
    faces: &mut Vec<(i32, i32, i32)>,
    canal: &Rc<SurfaceMixedCornerDualCanal>,
    cut: &SurfaceStrip,
    bead: &SurfaceStrip,
    seams: &[SideSeams],
    cut_edge: usize,
    bead_edge: usize,
    cut_ends_here: bool,
    bead_ends_here: bool,
) -> Result<(SurfaceStripStation, SurfaceStripStation), BevelAdvisory> {
    // Everything added here is new, and whatever of it lands part way along an edge of a face
    // nobody cut is a T junction - so the sew at the end is told where this corner's own vertices
    // start.
    let split_from = pool.vertices().len() as i32;
    let corner = Rc::clone(&canal.corner);

    let (cut_ring, cut_meets_at) = try_ring(
        pool,
        &corner,
        cut,
        true,
        seams,
        cut_edge,
        cut_ends_here,
        Some(canal.dual.cut_ring_plane()),
    )?;
    let (bead_ring, bead_meets_at) = try_ring(
        pool,
        &corner,
        bead,
        false,
        seams,
        bead_edge,
        bead_ends_here,
        None,
    )?;

    let cut_as_the_grid_wants_it =
        SurfaceMixedCornerDualGrid::from_the_shared_face(&corner, &cut_ring);
    let bead_as_the_grid_wants_it = SurfaceMixedCornerDualGrid::from_the_wall(&corner, &bead_ring);

    let read = SurfaceMixedCornerDualGrid::try_read(
        Rc::clone(canal),
        positions_of(pool, &cut_as_the_grid_wants_it),
        positions_of(pool, &bead_as_the_grid_wants_it),
    )?;
    let grid = try_the_grid(
        pool,
        &read,
        &cut_as_the_grid_wants_it,
        &bead_as_the_grid_wants_it,
    )?;

    try_give_up_the_wall(pool, faces, &read, &grid, split_from)?;
    try_give_up_the_cap(pool, faces, &read, &grid)?;

    emit(pool, faces, &corner, &grid);

    // ONCE, and after everything: every crossing this made, sewn into whatever face now carries
    // it part way along an edge.
    sew_in_the_splits_since_unchecked(pool, faces, split_from);

    nothing_covers_nothing(pool, faces)?;

    Ok((
        SurfaceStripStation::clean(cut_ring, cut_meets_at),
        SurfaceStripStation::clean(bead_ring, bead_meets_at),
    ))
}

/// C# `TryTheGrid`: the grid's vertices - its interior minted, and every point of its two REUSED
/// sides taken from the ring that already owns it: station zero the bead's ring, and the far rail
/// of every station the cut carries that ring's own rail.
///
/// The closing station is ONE vertex repeated across the row, which is what makes the last band a
/// fan rather than a row of slivers: the row's own extent has closed to nothing there, so every
/// rail of it really is the same point and pooling it once is what says so.
fn try_the_grid(
    pool: &mut SurfaceVertexPool,
    read: &SurfaceMixedCornerDualGrid,
    cut_ring: &[i32],
    bead_ring: &[i32],
) -> Result<Vec<Vec<i32>>, BevelAdvisory> {
    // The corner the two rings SHARE - the bead's own tube contact is the cut ring's first rail -
    // is one vertex or the patch has two surfaces at a tangency rather than one. (C# would throw
    // indexing an empty ring; an empty one is refused the same way here.)
    let (Some(&bead_last), Some(&cut_first)) = (bead_ring.last(), cut_ring.first()) else {
        return Err(BevelAdvisory::new(
            BevelAdvisoryCode::SurfaceStripSeamDidNotMeet,
            vec![canal_corner(read).into()],
        ));
    };

    if bead_last != cut_first {
        return Err(BevelAdvisory::new(
            BevelAdvisoryCode::SurfaceStripSeamDidNotMeet,
            vec![pool.position_of(cut_first).into()],
        ));
    }

    let mut grid = Vec::with_capacity(read.station_count());

    let dies = pool.add(read.canal.dies_on());

    // `station` reads the grid, the cut ring and the read grid alike, as C#'s loop does.
    #[allow(clippy::needless_range_loop)]
    for station in 0..read.station_count() {
        let mut row = vec![0; read.rails_per_row()];

        for (rail, slot) in row.iter_mut().enumerate() {
            *slot = if read.is_the_point(station) {
                dies
            } else if station == 0 {
                bead_ring[rail]
            } else if rail == read.rails_per_row() - 1 && read.the_cut_carries(station) {
                cut_ring[station]
            } else {
                pool.add(read.at(rail, station))
            };
        }

        grid.push(row);
    }

    Ok(grid)
}

/// Where an empty ring is named: the corner itself, there being no rail to name.
fn canal_corner(read: &SurfaceMixedCornerDualGrid) -> Vector3 {
    read.canal.corner.corner
}

/// C# `Emit`: the patch's quads, wound to face the sum of the corner's outward normals rather than
/// to follow its grid, which is built from carried points and has no winding of its own. The last
/// band is a FAN, because its far station is one point.
fn emit(
    pool: &mut SurfaceVertexPool,
    faces: &mut Vec<(i32, i32, i32)>,
    corner: &SurfaceMixedCorner,
    grid: &[Vec<i32>],
) {
    let outward = corner.outward_sum();

    for station in 0..grid.len().saturating_sub(1) {
        for rail in 0..grid[station].len().saturating_sub(1) {
            let here = grid[station][rail];
            let next = grid[station][rail + 1];
            let over = grid[station + 1][rail];
            let across = grid[station + 1][rail + 1];

            if over == across {
                // The far station is the one point the blend dies on, so the quad is a triangle.
                surface_fan::emit_triangle_facing(pool, faces, here, next, over, outward);
                continue;
            }

            surface_fan::emit_facing(&[here, over, across, next], pool, faces, outward);
        }
    }
}

/// C# `PositionsOf`: where each of a ring's pooled rails stands.
fn positions_of(pool: &SurfaceVertexPool, ring: &[i32]) -> Vec<Vector3> {
    ring.iter().map(|&rail| pool.position_of(rail)).collect()
}
