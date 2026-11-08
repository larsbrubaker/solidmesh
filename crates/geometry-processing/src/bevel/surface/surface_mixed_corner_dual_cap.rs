//! What the CAP gives up under a leaning one-crease corner's run-out - the face the row's far rail
//! lies on once the cut's own strip has ended: [`try_give_up_the_cap`] and the cap's own corner and
//! faces it is read from.
//!
//! **Only the run-out takes any of it.** While the cut's strip is still there the row ends on that
//! strip's tube and the cap is nobody's business: whatever of it the cut wanted, the cut's own band
//! already took. Past the changeover the row ends ON the cap's plane
//! (`SurfaceMixedCornerDualCanal::row_runs_to`), so the curve the far rail traces from there round
//! to the point the blend dies on is where the cap now ends, and what stands between that curve and
//! the cap's own corner is what it gives up.
//!
//! **The section hangs off ONE corner, and it is not the part's.** The cut's band on this face was
//! walked back to the canal's own leaning plane (`SurfaceMixedCorner::band_end_for`), so what the
//! cap has left at this end is bounded by that plane's trace and by the crease - and the corner
//! between those two, where the trace crosses the crease, is what the run-out's section hangs off.
//! The part's own corner is not it: the leaning plane does not pass through that vertex, and on the
//! nose fixture it stands 0.1738 mm clear of it.
//!
//! **And the section is NOT convex**, which is why this is a bounded subtraction and not a bite.
//! The far rail's curve caves IN towards that corner - measured on the fixture the chord from the
//! changeover to the dying point stands 0.13 mm outside the curve at mid span - so a convex bite
//! refuses it by name, and what takes it is `surface_end_gives_up_the_section`, whose whole
//! business is the polygon of a corner and an arc caving in towards it, taken as a triangle less
//! the bulge.
//!
//! **Flat only**, as the wall's is: a cap rolled into facets would take a piece per facet and none
//! is drawn by any fixture.

use indexmap::IndexSet;
use solidmesh_math::Vector3;

use super::surface_end_gives_up_the_section::try_give_it_up;
use super::SurfaceVertexPool;
use super::{SurfaceMixedCorner, SurfaceMixedCornerDualCanal, SurfaceMixedCornerDualGrid};
use crate::bevel::{BevelAdvisory, BevelAdvisoryCode};

/// Takes the run-out's section out of the cap, or says why it cannot be taken. `read` is the
/// corner's grid, read, and `grid` its vertices, by station and then rail.
pub fn try_give_up_the_cap(
    pool: &mut SurfaceVertexPool,
    faces: &mut Vec<(i32, i32, i32)>,
    read: &SurfaceMixedCornerDualGrid,
    grid: &[Vec<i32>],
) -> Result<(), BevelAdvisory> {
    let canal = &read.canal;
    let corner = &canal.corner;
    // `rails_per_row` is at least one on any grid the dual reads (its bead ring has rails).
    let last = read.rails_per_row() - 1;
    let mut profile: Vec<i32> = Vec::new();

    // From the changeover - the cut ring's own last rail, which stands on the cut's tube and on the
    // cap's plane at once - round to the dying point. The cut ring always carries its changeover
    // station; `saturating_sub` only keeps an empty cut ring from wrapping to a station before the
    // first, a guard: the grid refuses a cut ring of fewer than two points before one reaches here.
    let from = read.stations_the_cut_carries().saturating_sub(1);
    for row in &grid[from..read.station_count()] {
        if profile.last() != Some(&row[last]) {
            profile.push(row[last]);
        }
    }

    if profile.len() < 2 {
        // A run-out of one station takes no area of the cap at all.
        return Ok(());
    }

    let Some(hangs_off) = try_the_caps_own_corner(canal) else {
        // DEFENSIVE: the band's own stop plane running ALONG the crease would cross it nowhere, and
        // a corner whose cut lies in the wall has already been refused by the dual's own reading.
        return Err(BevelAdvisory::new(
            BevelAdvisoryCode::SurfaceCornerNotSupported,
            vec![corner.corner.into()],
        ));
    };

    let in_the_cap = in_the_cap(pool, faces, corner);

    try_give_it_up(
        pool,
        faces,
        &profile,
        hangs_off,
        corner.out_of_the_cap_face,
        &in_the_cap,
    )
    .map(|_| ())
}

/// The corner of the cap the section hangs off - where the plane the cut's band was stopped on
/// crosses the crease the cap and the wall meet in.
#[allow(clippy::neg_cmp_op_on_partial_ord)]
fn try_the_caps_own_corner(canal: &SurfaceMixedCornerDualCanal) -> Option<Vector3> {
    let stop = canal.dual.cut_ring_plane();
    let along = canal.crease.direction.dot(stop.inward);

    if !(along.abs() > 1e-9) {
        return None;
    }

    Some(
        canal.crease.point
            + (canal.crease.direction
                * ((stop.point - canal.crease.point).dot(stop.inward) / along)),
    )
}

/// Which of the faces stand IN the cap's plane - the ones the subtraction is entitled to rewrite.
///
/// Read by the plane and not by a flood from the corner, because the section hangs off a corner
/// that is not a vertex of the mesh at all: it is a point on the crease the cut's own band left
/// bare, and a flood seeded there would find nothing. The subtraction is BOUNDED, so a coplanar
/// face somewhere else on the part is offered and simply kept whole.
fn in_the_cap(
    pool: &SurfaceVertexPool,
    faces: &[(i32, i32, i32)],
    corner: &SurfaceMixedCorner,
) -> IndexSet<i32> {
    let mut in_the_cap = IndexSet::new();

    for (face, &(v0, v1, v2)) in faces.iter().enumerate() {
        if stands_in_the_cap(pool, corner, v0)
            && stands_in_the_cap(pool, corner, v1)
            && stands_in_the_cap(pool, corner, v2)
        {
            in_the_cap.insert(face as i32);
        }
    }

    in_the_cap
}

/// Whether `vertex` stands in the cap face's plane.
fn stands_in_the_cap(pool: &SurfaceVertexPool, corner: &SurfaceMixedCorner, vertex: i32) -> bool {
    (pool.position_of(vertex) - corner.corner)
        .dot(corner.out_of_the_cap_face)
        .abs()
        <= pool.near_enough()
}
