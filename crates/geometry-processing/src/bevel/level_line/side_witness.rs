//! The last point of a side to survive the cut, made a VERTEX of the mesh so that the side's
//! survival is decided exactly instead of inside a search tolerance. Ports
//! `Bevel/LevelLine/SideWitness.cs` (a C# static class): its constants, its per-thread tally
//! ([`SideWitnessTally`], [`reset`]) and the side's live triangles and the triangle holding a
//! point; the witness search and its insertion land with later packages.
//!
//! The refinement's rules find the level line by bounded search, and every such search answers
//! only to within its own accuracy. Where the last surviving scrap of a side is a small island
//! INTERIOR to a triangle, whether the island is found is decided inside that accuracy: the side
//! then holds no Outside vertex at all, the cut leaves it no loop, and a size the room promised
//! builds refuses. That is the state this rule, and only this rule, repairs. It is LAZY: a side
//! that already holds one Outside vertex after refinement is left exactly as it stands.

use std::cell::Cell;

use vector_math::{cs_math, Vector3};

use super::{ChainField, LevelLineMesh};

/// How close to the snap the witness search decides, as a fraction of the size. Fifty times finer
/// than the refinement's own `size / 20`, which is what the search has to beat to be worth
/// running, and affordable because it is only ever run on a side that is otherwise about to be
/// lost.
pub const WITNESS_ACCURACY: f64 = 1.0 / 1000.0;

/// How far off a triangle a point may stand and still count as standing ON it, as a fraction of
/// the triangle's own longest side - the same millionth `RefinementSplits.WhereThePointStands`
/// calls a point on an EDGE.
pub const ON_THE_TRIANGLE: f64 = 1e-6;

/// C# `SideWitness`'s `[ThreadStatic]` counters: one thread's tally, counted per thread for the
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

/// The side's triangles still in the mesh, in triangle order.
pub fn live_triangles_of_side(mesh: &LevelLineMesh, side: i32) -> Vec<i32> {
    (0..mesh.triangle_count())
        .filter(|&triangle| {
            mesh.triangles()[triangle as usize].tag == side && !mesh.is_removed(triangle)
        })
        .collect()
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
            best_size = cs_math::max_f64(
                (b - a).length(),
                cs_math::max_f64((c - b).length(), (a - c).length()),
            );
        }
    }

    if nearest <= ON_THE_TRIANGLE * best_size {
        best
    } else {
        -1
    }
}
