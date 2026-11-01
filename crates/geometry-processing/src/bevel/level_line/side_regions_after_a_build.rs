//! The caller's face regions carried onto the mesh a build left, so a pick built a chain at a time
//! (`band_gap_in_turn.rs`, the only build that carries them) can hand the rolling ball its sides
//! on every chain's pass, not only the first (`rolling_ball_sides.rs` reads them; it needs one
//! region per triangle of the mesh the pass builds on).
//!
//! **Not rounds first.** A rounds-first pick's second pass reads creases that run on over the
//! first pass's rounds, so a round can lie on both sides of such a crease and the crease would read
//! its two sides as one face region; carrying needs a rule for which side a round is first, so
//! `band_gap_rounds_first.rs` hands its passes no regions.
//!
//! **Only where the ball reads them** ([`the_ball_reads_them`]): the rolling ball's switch on, a
//! fillet (a chamfer is outside the ball) and a build that is the outer pass on its thread (a
//! probe inside another pass keeps the level line's setbacks and reads no sides). Anywhere else the
//! passes are handed none, exactly as a build given none, and nothing is carried.
//!
//! **Which triangles are the round.** The fill's own record says so: every triangle a fill laid
//! is in its `GapFillResult::added_triangles` (by the working mesh's triangle index; the rows, the
//! caps, the fades and the corner patches all append to it). Each piece of the round is a region
//! of its own, numbered above every region before in the order of its first triangle, so an
//! earlier round is never a side's, and a later crease with a round on each side reads two
//! regions. A piece is the fill triangles joined edge to edge: the chain's round (a pass of one
//! chain lays one). One exception: a fill triangle lying in the
//! plane of the triangle before it lies on (an end face's cap laid flat in the end face) is a
//! piece of that face and keeps its region. "Lying in" is a centroid within [`ON_THE_SURFACE`] of
//! the part's diagonal (rounding of positions the build interpolated, which scales with the
//! coordinates) and normals parallel within `COPLANAR_AFTER_A_BUILD`, facing either way (a cap can
//! be laid wound against the face it lies in); a round's rows tilt off the face they leave, so
//! neither test is close for them.
//!
//! **Every other triangle descends from the mesh before.** A build never moves a vertex: it
//! appends them, splits triangles on their own edges or inside them, and re-tiles needles and
//! flat strips from vertices already there. A triangle whose three corners are a triangle before
//! is that triangle (most of the part) and keeps its region. Any other is a piece of a triangle
//! the build took apart: its region is that of the taken-apart triangle nearest its centroid, with
//! no tolerance, since a re-tiled needle may stand off its parents by up to the needle floor (a
//! share of the size); the lowest index wins a tie (a centroid on the edge between two coplanar
//! triangles, which a flood at any bend threshold puts in one region). The search runs over a box
//! tree of the taken-apart triangles alone (`RollingBallSide`), polling the build's cancellation.
//!
//! **Why carried by geometry and the fill's record, not a record kept through the build.** The
//! working mesh's triangle tag already carries the band's sides through the cut, and the strip and
//! needle re-tilings rebuild triangles from their corners; a provenance record would have to be
//! threaded through every one of those edits.

use std::collections::{HashMap, HashSet};

use vector_math::Vector3;

use super::band_gap::BandGapBuildError;
use super::band_gap_in_turn::COPLANAR_AFTER_A_BUILD;
use super::band_gap_rounds_first::kept;
use super::rolling_ball_setbacks::PassScope;
use super::triangle_edges;
use super::{BandProfile, LevelLineMesh, LevelLineWork, WorkBudget};
use crate::bevel::rolling_ball::rolling_ball_routing::use_rolling_ball;
use crate::bevel::rolling_ball::RollingBallSide;

/// How far, as a share of the part's diagonal, a fill triangle's centroid may stand off the
/// triangle before it lies on and still lie in that face.
pub const ON_THE_SURFACE: f64 = 1e-9;

/// How many triangles are read between two polls of the build's cancellation.
const POLL_EVERY: usize = 256;

/// True where the rolling ball reads the side regions a build of `profile` is handed: its switch
/// on, a fillet, and no pass already running on this thread (see the file header).
pub fn the_ball_reads_them(profile: BandProfile) -> bool {
    use_rolling_ball() && profile == BandProfile::Fillet && PassScope::would_be_outer()
}

/// The corners of `t` in rising order: the same triangle whatever its winding or first corner.
fn sorted((a, b, c): (i32, i32, i32)) -> (i32, i32, i32) {
    let mut corners = [a, b, c];
    corners.sort_unstable();
    (corners[0], corners[1], corners[2])
}

/// The unit normal of `t` over `at`.
fn normal(at: &[Vector3], (a, b, c): (i32, i32, i32)) -> Vector3 {
    let p = at[a as usize];
    (at[b as usize] - p).cross(at[c as usize] - p).get_normal()
}

/// `labels` (one region per triangle of `before`) carried onto the live triangles of `after`
/// (in `band_gap_rounds_first::kept`'s order), the mesh a build on `before` left, whose fill laid
/// the triangles `fill` (by `after`'s triangle index), as the file header says. `budget` is the
/// build's, polled for cancellation (the thread's budget in force where `None`).
///
/// `Ok(None)` where the labels cannot be carried: `labels` does not label every triangle of
/// `before`, a triangle that is no fill has no triangle before to descend from, or no region
/// number is left above every one of `labels` for the round's pieces. The pass after it is then handed
/// none.
pub fn carried(
    before: (&[Vector3], &[(i32, i32, i32)]),
    labels: &[u32],
    after: &LevelLineMesh,
    fill: &[i32],
    budget: Option<&WorkBudget>,
) -> Result<Option<Vec<u32>>, BandGapBuildError> {
    let (before_at, before_triangles) = before;
    if labels.len() != before_triangles.len() {
        return Ok(None);
    }
    let poll = || -> Result<(), BandGapBuildError> {
        let cancelled = match budget {
            Some(budget) => budget.cancellation_token.is_cancelled(),
            None => LevelLineWork::poll_cancel().is_err(),
        };
        if cancelled {
            Err(BandGapBuildError::Cancelled)
        } else {
            Ok(())
        }
    };

    let after_at = &after.vertices;
    let live: Vec<i32> = (0..after.triangle_count())
        .filter(|&t| !after.is_removed(t))
        .collect();
    let after_triangles = kept(after);
    let is_fill: HashSet<i32> = fill.iter().copied().collect();

    // Looked up only, never iterated.
    let mut by_corners: HashMap<(i32, i32, i32), usize> = HashMap::new();
    for (t, &triangle) in before_triangles.iter().enumerate() {
        by_corners.entry(sorted(triangle)).or_insert(t);
    }
    let still_there: HashSet<(i32, i32, i32)> =
        after_triangles.iter().map(|&t| sorted(t)).collect();
    // The triangles before that the build took apart, in index order.
    let taken_apart: Vec<usize> = (0..before_triangles.len())
        .filter(|&t| !still_there.contains(&sorted(before_triangles[t])))
        .collect();
    let corners: Vec<[i32; 3]> = taken_apart
        .iter()
        .map(|&t| {
            let (a, b, c) = before_triangles[t];
            [a, b, c]
        })
        .collect();
    let tree = RollingBallSide::new(before_at, &corners);

    let (mut low, mut high) = (
        Vector3::new(f64::INFINITY, f64::INFINITY, f64::INFINITY),
        Vector3::new(f64::NEG_INFINITY, f64::NEG_INFINITY, f64::NEG_INFINITY),
    );
    for p in before_at {
        low = Vector3::new(low.x.min(p.x), low.y.min(p.y), low.z.min(p.z));
        high = Vector3::new(high.x.max(p.x), high.y.max(p.y), high.z.max(p.z));
    }
    let near = if before_at.is_empty() {
        0.0
    } else {
        ON_THE_SURFACE * (high - low).length()
    };

    // Each live triangle's region, or None for a triangle of the round, numbered below.
    let mut carried: Vec<Option<u32>> = Vec::with_capacity(after_triangles.len());
    for (i, &triangle) in after_triangles.iter().enumerate() {
        if i % POLL_EVERY == 0 {
            poll()?;
        }
        let laid = is_fill.contains(&live[i]);
        if !laid {
            if let Some(&t) = by_corners.get(&sorted(triangle)) {
                carried.push(Some(labels[t]));
                continue;
            }
        }

        let (a, b, c) = triangle;
        let centroid = (after_at[a as usize] + after_at[b as usize] + after_at[c as usize]) / 3.0;
        let parent = tree
            .closest(centroid)
            .map(|contact| (taken_apart[contact.triangle as usize], contact.distance));
        let region = match (parent, laid) {
            (Some((t, _)), false) => Some(labels[t]),
            (None, false) => return Ok(None),
            (Some((t, distance)), true) => {
                let flat = distance <= near
                    && 1.0
                        - normal(after_at, triangle)
                            .dot(normal(before_at, before_triangles[t]))
                            .abs()
                        <= COPLANAR_AFTER_A_BUILD;
                flat.then_some(labels[t])
            }
            (None, true) => None,
        };
        carried.push(region);
    }

    // The round's pieces: its triangles joined edge to edge, each numbered in the order of its
    // first triangle.
    let Some(first) = labels.iter().max().map_or(Some(0), |m| m.checked_add(1)) else {
        return Ok(None);
    };
    let standing = triangle_edges::standing_map(&after_triangles);
    let mut next = first;
    let mut out: Vec<u32> = vec![0; carried.len()];
    let mut numbered = vec![false; carried.len()];
    for start in 0..carried.len() {
        if let Some(region) = carried[start] {
            out[start] = region;
            continue;
        }
        if numbered[start] {
            continue;
        }
        let piece = next;
        let Some(after_this) = next.checked_add(1) else {
            return Ok(None);
        };
        next = after_this;
        numbered[start] = true;
        let mut stack = vec![start];
        while let Some(t) = stack.pop() {
            out[t] = piece;
            for (a, b) in triangle_edges::corners(after_triangles[t]) {
                for &other in standing
                    .get(&triangle_edges::key(a, b))
                    .map_or(&[][..], Vec::as_slice)
                {
                    let other = other as usize;
                    if carried[other].is_none() && !numbered[other] {
                        numbered[other] = true;
                        stack.push(other);
                    }
                }
            }
        }
    }

    Ok(Some(out))
}
