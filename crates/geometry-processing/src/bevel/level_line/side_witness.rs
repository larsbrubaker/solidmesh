//! The last point of a side to survive the cut, made a VERTEX of the mesh so that the side's
//! survival is decided exactly instead of inside a search tolerance: its constants, its per-thread
//! tally ([`SideWitnessTally`], [`reset`]), the side's live triangles, the reading's own last
//! surviving point ([`from_the_reading`]), the triangle holding a point, the decision search
//! ([`search`]) and the witness's insertion, and giving a side its witness
//! ([`give_the_side_a_witness`]).
//!
//! The refinement's rules find the level line by bounded search, and every such search answers
//! only to within its own accuracy. Where the last surviving scrap of a side is a small island
//! INTERIOR to a triangle, whether the island is found is decided inside that accuracy: the side
//! then holds no Outside vertex at all, the cut leaves it no loop, and a size the room promised
//! builds refuses. That is the state this rule, and only this rule, repairs. It is LAZY: a side
//! that already holds one Outside vertex after refinement is left exactly as it stands.

use std::cell::Cell;
use std::collections::{HashMap, VecDeque};
use std::rc::Rc;

use solidmesh_math::{ieee_math, Vector3};

use super::band_refinement::{refine, RefineError, RefineOptions};
use super::band_room::REFERENCE_SIZE;
use super::self_room;
use super::{ChainDistanceField, ChainField, LevelLineMesh, RoomField, WorkError};

/// How close to the snap the witness search decides, as a fraction of the size. Fifty times finer
/// than the refinement's own `size / 20`, which is what the search has to beat to be worth
/// running, and affordable because it is only ever run on a side that is otherwise about to be
/// lost.
pub const WITNESS_ACCURACY: f64 = 1.0 / 1000.0;

/// How far off a triangle a point may stand and still count as standing ON it, as a fraction of
/// the triangle's own longest side - the same millionth `RefinementSplits.WhereThePointStands`
/// calls a point on an EDGE.
pub const ON_THE_TRIANGLE: f64 = 1e-6;

/// The side witness's counters: one thread's tally, counted per thread for the
/// reason the field counters give.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub struct SideWitnessTally {
    /// How many builds have entered the search - zero on every build whose sides survive on their
    /// own.
    pub entered: i64,
    /// How many of those searches found a witness and put it into the mesh.
    pub inserted: i64,
    /// How many witnesses came from the READING's own last surviving point - the first line.
    pub by_the_readings_point: i64,
    /// How many came from the decision search instead - the second line.
    pub by_the_decision_search: i64,
}

thread_local! {
    static TALLY: Cell<SideWitnessTally> = const {
        Cell::new(SideWitnessTally {
            entered: 0,
            inserted: 0,
            by_the_readings_point: 0,
            by_the_decision_search: 0,
        })
    };
}

/// This thread's tally.
pub fn tally() -> SideWitnessTally {
    TALLY.with(Cell::get)
}

/// Changes this thread's tally.
pub fn update_tally(f: impl FnOnce(&mut SideWitnessTally)) {
    TALLY.with(|cell| {
        let mut tally = cell.get();
        f(&mut tally);
        cell.set(tally);
    });
}

/// Zeroes this thread's tally.
pub fn reset() {
    TALLY.with(|cell| cell.set(SideWitnessTally::default()));
}

/// Gives a side that refinement has left with no Outside
/// vertex its last surviving point as a vertex, and refines round it, so that the cut sees the
/// loop the room promised.
///
/// The mesh is the reading's own (as in [`super::band_refinement::refine`]); it is borrowed only
/// between reads. The inner result is `Ok(())` when the side survives (or is left as it stands),
/// `Err(triangle)` only when the second refinement meets a triangle it cannot settle. The work
/// budget's exits and the refinement's errors are the outer `Err`.
#[allow(clippy::neg_cmp_op_on_partial_ord)]
pub fn give_the_side_a_witness(
    reading: &super::VertexFieldReading,
    field: &dyn ChainDistanceField,
    side: i32,
    size: f64,
    chord_tolerance: f64,
    topology_only: bool,
) -> Result<Result<(), i32>, RefineError> {
    let triangles = live_triangles_of_side(&reading.mesh.borrow(), side);
    if triangles.is_empty() || holds_an_outside_vertex(&reading.mesh.borrow(), reading, &triangles)?
    {
        return Ok(Ok(()));
    }

    update_tally(|tally| tally.entered += 1);
    let snap = reading.snap;
    let tolerance = ieee_math::max_f64(size * WITNESS_ACCURACY, snap);
    if !(tolerance > 0.0) {
        // A search needs a positive accuracy; with no size and no snap to take one from there is
        // nothing to decide either.
        return Ok(Ok(()));
    }

    let mut witness =
        from_the_reading(&reading.mesh.borrow(), field, &triangles, side, size, snap)?;
    if witness.is_some() {
        update_tally(|tally| tally.by_the_readings_point += 1);
    } else {
        witness = search(&reading.mesh.borrow(), field, &triangles, tolerance, snap)?;
        if witness.is_some() {
            update_tally(|tally| tally.by_the_decision_search += 1);
        }
    }

    let Some((triangle, point)) = witness else {
        // The side really is swallowed at this size - every point of it stands at or below the
        // line. Today's path then refuses, which is the right answer.
        return Ok(Ok(()));
    };

    update_tally(|tally| tally.inserted += 1);
    let mut pending = VecDeque::new();
    let mut depth = HashMap::new();
    insert(
        &mut reading.mesh.borrow_mut(),
        triangle,
        point,
        snap,
        &mut pending,
        &mut depth,
    )?;
    let candidates: Vec<i32> = pending.into_iter().collect();
    let (settled, offending_triangle) = refine(
        reading,
        &candidates,
        chord_tolerance,
        RefineOptions {
            topology_only,
            ..RefineOptions::default()
        },
    )?;
    Ok(if settled {
        Ok(())
    } else {
        Err(offending_triangle)
    })
}

/// The side's triangles still in the mesh, in triangle order.
pub fn live_triangles_of_side(mesh: &LevelLineMesh, side: i32) -> Vec<i32> {
    (0..mesh.triangle_count())
        .filter(|&triangle| {
            mesh.triangles()[triangle as usize].tag == side && !mesh.is_removed(triangle)
        })
        .collect()
}

/// The reading's own last surviving point for this side, read at
/// the size being built, with the side's triangle holding it - the first line. `None` when the
/// reading has no such point for this side, when the field is not one a room can be read from (a
/// chain's or a pick's), when the point does NOT stand above the snap (a side genuinely swallowed
/// at this size), or when no triangle of the side holds it.
///
/// The reading is taken on the field at [`REFERENCE_SIZE`], where the room itself was read;
/// setbacks are linear in the size, so scaling the build's field back to the reference is the
/// same field the reading saw. The ceiling is left off (infinite): only the point is wanted.
#[allow(clippy::neg_cmp_op_on_partial_ord)]
pub fn from_the_reading(
    mesh: &LevelLineMesh,
    field: &dyn ChainDistanceField,
    triangles: &[i32],
    side: i32,
    size: f64,
    snap: f64,
) -> Result<Option<(i32, Vector3)>, WorkError> {
    let reference = if let Some(chain) = field.as_chain_field() {
        RoomField::from_chain(Rc::new(chain.with_setbacks_scaled(REFERENCE_SIZE / size)?))
    } else if let Some(pick) = field.as_pick_field() {
        RoomField::from_pick(Rc::new(pick.with_setbacks_scaled(REFERENCE_SIZE / size)?))
    } else {
        return Ok(None);
    };

    let point = self_room::last_surviving_point_of_side(
        mesh,
        &reference,
        side,
        REFERENCE_SIZE,
        f64::INFINITY,
    )?;
    let point = match point {
        Some(point) if field.value_at(point)? > snap => point,
        _ => return Ok(None),
    };

    let holding = triangle_holding(mesh, triangles, point);
    Ok(if holding < 0 {
        None
    } else {
        Some((holding, point))
    })
}

/// The triangle of `triangles` nearest `point` - the first of a tie - when the point stands ON it
/// (within [`ON_THE_TRIANGLE`] of its longest side), or -1.
pub fn triangle_holding(mesh: &LevelLineMesh, triangles: &[i32], point: Vector3) -> i32 {
    let mut best = -1;
    let mut nearest = f64::INFINITY;
    let mut best_size = 0.0;
    for &triangle in triangles {
        let t = mesh.triangles()[triangle as usize];
        let a = mesh.position(t.a);
        let b = mesh.position(t.b);
        let c = mesh.position(t.c);
        let here = (ChainField::closest_on_triangle(point, a, b, c) - point).length();
        if here < nearest {
            nearest = here;
            best = triangle;
            best_size = ieee_math::max_f64(
                (b - a).length(),
                ieee_math::max_f64((c - b).length(), (a - c).length()),
            );
        }
    }

    if nearest <= ON_THE_TRIANGLE * best_size {
        best
    } else {
        -1
    }
}

/// The first point of the side found standing above the snap, and the
/// triangle holding it - the decision search, the second line. `None` when no triangle of the side
/// holds one.
///
/// The threshold is the snap and it never rises: `at_least` prunes every piece that cannot reach
/// it, `stop_above` ends the search on the first value that clears it. An answer that did not beat
/// the threshold stands at a meaningless point, so it is compared strictly and then dropped.
pub fn search(
    mesh: &LevelLineMesh,
    field: &dyn ChainDistanceField,
    triangles: &[i32],
    tolerance: f64,
    snap: f64,
) -> Result<Option<(i32, Vector3)>, WorkError> {
    for &triangle in triangles {
        let t = mesh.triangles()[triangle as usize];
        let a = mesh.position(t.a);
        let b = mesh.position(t.b);
        let c = mesh.position(t.c);

        let greatest = field.max_over_triangle_bounded(a, b, c, tolerance, snap, snap)?;
        if greatest.value > snap {
            return Ok(Some((triangle, greatest.point)));
        }
    }

    Ok(None)
}

/// Whether anything of the side is already known to survive. Stops at the first Outside vertex,
/// reading each triangle's corners in order and stopping at the first that answers.
pub fn holds_an_outside_vertex(
    mesh: &LevelLineMesh,
    reading: &super::VertexFieldReading,
    triangles: &[i32],
) -> Result<bool, super::WorkError> {
    for &triangle in triangles {
        let t = mesh.triangles()[triangle as usize];
        for corner in [t.a, t.b, t.c] {
            if reading.side_of(corner)? == super::VertexSide::Outside {
                return Ok(true);
            }
        }
    }

    Ok(false)
}

/// Puts the witness into the mesh the way the Island rule puts its peak
/// in - on an edge when it stands on one, inside the triangle otherwise, nothing at an existing
/// corner. "At a corner" is asked at the reading's SNAP, not the triangle's millionth: the last
/// surviving scrap of a side is about two snaps wide, and a millionth would swallow it.
pub fn insert(
    mesh: &mut LevelLineMesh,
    triangle: i32,
    point: Vector3,
    snap: f64,
    pending: &mut VecDeque<i32>,
    depth: &mut HashMap<i32, i32>,
) -> Result<(), super::LevelLineMeshError> {
    super::refinement_splits_enqueue::insert_where_it_stands(
        mesh,
        triangle,
        point,
        pending,
        depth,
        Some(snap),
    )?;
    Ok(())
}
