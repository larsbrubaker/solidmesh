//! The room foreign edges leave a chain read per side, where the rolling ball cuts the two sides at
//! different distances (`one_chain_room` hands the sides; `band_room::room_of_cut_mesh` reads them
//! here): each foreign edge is checked against the band of the side it lies on, with that side's
//! own setbacks, and the room is the least over the sides ([`room_for_by_side`]); where the room is
//! set is named from the side that set it ([`where_the_room_is_set_by_side`]).
//!
//! **Which side an edge lies on.** A foreign edge stands between a tagged triangle and an untagged
//! one; the side is read from the band tags (above 0) of the triangles on it ([`side_of_edge`]):
//! only tag 1's, the left side (`SideSetbacks::left`); only tag 2's, the right. Every other edge -
//! one touching both sides, an end region (whose cap is cut from the chain's own field) or no band
//! group at all - is read against the chain's own field, the larger side, as without sides. So the
//! reading only ever trades the larger side's reach for a side's own where the band that can reach
//! the edge is that side's alone.
//!
//! **Why three readings and not one.** The edges are filed into three lists (left, right, either)
//! and each list is read by [`room_for`] with its own field family, unchanged: its bracket, its
//! pruning and its fixed bisection hold for one field family each, and the least of three rooms is
//! the room. A single reading with a field chosen per edge would bisect once rather than up to
//! three times, but every bracket and prune would have to learn per-edge setbacks. The cost: each
//! edge is still read in one list only, but a list whose setbacks are not one number bisects with
//! its own fixed steps, each building that family's field at the trial size, so up to three times
//! the field builds of one reading.
//!
//! **Growing crease runs** (`GrowingCreaseRuns::least_with`, in `band_room::room_of_cut_mesh`) are
//! read against the ball's own centres where the ball takes the pass (`ball_pierce_room`: one
//! centre a radius from both sides, so the side a run lies on needs no setback of its own), and
//! against the chain's own setbacks, the larger side's, otherwise.

use vector_math::{ieee_math, Vector3};

use super::foreign_edge_room_readings::{room_for, where_the_room_is_set, FieldAt, ForeignEdge};
use super::{LevelLineMesh, WorkError};

/// Each side's field family, as the room reads it: index 0 the left side (tag 1), 1 the right
/// (tag 2).
pub struct SideRoomFields<'a> {
    /// Each side's setbacks per unit of size, one per chain point.
    pub units: [&'a [f64]; 2],
    /// Each side's field at a trial size.
    pub field_at: [FieldAt<'a>; 2],
    /// Each side's field at a trial size without the convex end faces' rays
    /// (`band_room::room_of_cut_mesh` says why such an edge is read without them).
    pub without_convex_rays: [FieldAt<'a>; 2],
}

/// Which side the foreign edge `low`-`high` lies on: `Some(0)` where only the left side's (tag 1)
/// triangles stand on it among the band's tags, `Some(1)` where only the right's (tag 2), `None`
/// otherwise (both sides, an end region, or no band tag).
pub fn side_of_edge(mesh: &LevelLineMesh, low: i32, high: i32) -> Option<usize> {
    let (mut left, mut right, mut other) = (false, false, false);
    for &t in mesh.triangles_on(low, high) {
        match mesh.triangles()[t as usize].tag {
            0 => {}
            1 => left = true,
            2 => right = true,
            _ => other = true,
        }
    }
    match (left, right, other) {
        (true, false, false) => Some(0),
        (false, true, false) => Some(1),
        _ => None,
    }
}

/// `edges` filed as (left, right, either), each in the order given.
fn filed(mesh: &LevelLineMesh, edges: &[ForeignEdge]) -> [Vec<ForeignEdge>; 3] {
    let mut lists: [Vec<ForeignEdge>; 3] = [Vec::new(), Vec::new(), Vec::new()];
    for &edge in edges {
        let at = side_of_edge(mesh, edge.0, edge.1).unwrap_or(2);
        lists[at].push(edge);
    }
    lists
}

/// Each list's field family: the side's own for the left and right lists, the chain's own
/// (`field_at`, `units`) for the third. `without_rays` picks the family without convex rays.
fn families<'a>(
    sides: &'a SideRoomFields<'a>,
    field_at: FieldAt<'a>,
    units: &'a [f64],
    without_rays: bool,
) -> [(FieldAt<'a>, &'a [f64]); 3] {
    let side = |k: usize| {
        let at = if without_rays {
            sides.without_convex_rays[k]
        } else {
            sides.field_at[k]
        };
        (at, sides.units[k])
    };
    [side(0), side(1), (field_at, units)]
}

/// The room `edges` leave the chain: with no `sides`, [`room_for`] over the chain's own field
/// family exactly; with them, the least of [`room_for`] over each side's edges with that side's
/// family and over the rest with the chain's own. `without_rays` reads the families without the
/// convex end faces' rays (the caller's `field_at` is then already that family).
pub fn room_for_by_side(
    mesh: &LevelLineMesh,
    edges: &[ForeignEdge],
    field_at: FieldAt,
    units: &[f64],
    sides: Option<&SideRoomFields>,
    without_rays: bool,
) -> Result<f64, WorkError> {
    let Some(sides) = sides else {
        return room_for(
            mesh,
            edges,
            field_at,
            units,
            super::band_room::REFERENCE_SIZE,
        );
    };
    let lists = filed(mesh, edges);
    let mut room = f64::INFINITY;
    for (list, (at, of)) in lists
        .iter()
        .zip(families(sides, field_at, units, without_rays))
    {
        let read = room_for(mesh, list, at, of, super::band_room::REFERENCE_SIZE)?;
        room = ieee_math::min_f64(room, read);
    }
    Ok(room)
}

/// Where the edge that sets `room` comes nearest the chain: with no `sides`,
/// [`where_the_room_is_set`] exactly; with them, read in the first list (left, right, either)
/// whose own room is `room`, with that list's field family.
pub fn where_the_room_is_set_by_side(
    mesh: &LevelLineMesh,
    edges: &[ForeignEdge],
    field_at: FieldAt,
    units: &[f64],
    sides: Option<&SideRoomFields>,
    room: f64,
) -> Result<Vector3, WorkError> {
    let Some(sides) = sides else {
        return where_the_room_is_set(mesh, edges, field_at, units, room);
    };
    let lists = filed(mesh, edges);
    for (list, (at, of)) in lists.iter().zip(families(sides, field_at, units, false)) {
        if list.is_empty() {
            continue;
        }
        if room_for(mesh, list, at, of, super::band_room::REFERENCE_SIZE)? == room {
            return where_the_room_is_set(mesh, list, at, of, room);
        }
    }
    where_the_room_is_set(mesh, edges, field_at, units, room)
}
