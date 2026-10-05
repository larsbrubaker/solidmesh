//! The end-face clip at the unrayed end of an open convex chain that stops on an unpicked concave
//! crease against one flat end face square to it. Ports `Bevel/LevelLine/SquareCreaseEndClip.cs`
//! (a C# static class): the per-flow gate ([`square_crease_clips_on`],
//! [`turn_square_crease_clips_off`] and its scope [`SquareCreaseEndClipSquareCreaseClipsOffScope`],
//! kept on the build's [`WorkBudget`](super::WorkBudget) as C#'s `AsyncLocal`) and the clips read
//! off plain positions ([`square_crease_end_clips`]); the clips over a pick land with later
//! packages. Reads the end-face sense (`end_face_sense.rs`).

use vector_math::{cs_math, Vector3};

use super::end_face_sense::{face_normal, is_convex, shared_corners, Corners, COPLANAR_TOLERANCE};
use super::{EndRayBound, LevelLineWork};

/// C# `SquareCreaseEndClip.SquareCreaseClipsOn`: whether [`square_crease_end_clips`] reads any
/// clip: on unless a test turns it off with [`turn_square_crease_clips_off`] to read the unclipped
/// ball end. Per build, as `BandGroupFields.RunOutsOn` (on wherever no budget is in force).
pub fn square_crease_clips_on() -> bool {
    LevelLineWork::current().is_none_or(|budget| !budget.square_crease_clips_off.get())
}

/// C# `SquareCreaseEndClip.TurnSquareCreaseClipsOff`: turns the clips off for the build in force
/// until the returned scope is dropped.
pub fn turn_square_crease_clips_off() -> SquareCreaseEndClipSquareCreaseClipsOffScope {
    SquareCreaseEndClipSquareCreaseClipsOffScope::new()
}

/// C# `SquareCreaseEndClip.SquareCreaseClipsOffScope`: turns the square-crease clips off for the
/// build in force until it is dropped, which puts back what was there before - so a test can read
/// the unclipped ball end. With no budget in force there is no build to switch and the scope does
/// nothing.
#[derive(Debug)]
#[must_use = "the clips stay off only while the scope is alive"]
pub struct SquareCreaseEndClipSquareCreaseClipsOffScope {
    /// Whether the clips were off when the scope was opened - restored when it closes.
    pub was: bool,
}

impl SquareCreaseEndClipSquareCreaseClipsOffScope {
    /// C#'s `SquareCreaseClipsOffScope()`: remembers the setting and turns the clips off.
    pub fn new() -> Self {
        let was = LevelLineWork::current()
            .is_some_and(|budget| budget.square_crease_clips_off.replace(true));
        SquareCreaseEndClipSquareCreaseClipsOffScope { was }
    }
}

impl Default for SquareCreaseEndClipSquareCreaseClipsOffScope {
    fn default() -> Self {
        Self::new()
    }
}

impl Drop for SquareCreaseEndClipSquareCreaseClipsOffScope {
    /// C#'s `Dispose`: puts back what was there before.
    fn drop(&mut self) {
        if let Some(budget) = LevelLineWork::current() {
            budget.square_crease_clips_off.set(self.was);
        }
    }
}

fn has(t: Corners, v: i32) -> bool {
    t.0 == v || t.1 == v || t.2 == v
}

/// Square to well under a degree: an oblique end carries a ray and its own bound.
const SQUARE_COSINE: f64 = 1.0 - 1e-6;

/// C# `SquareCreaseEndClip.SquareCreaseEndClips`: the end-face clip (`ChainField.EndClip`, no ray)
/// at each UNRAYED end of an open CONVEX chain that stops on an unpicked concave crease against one
/// flat end face square to it, where a side's flat face runs on past the end across a flat edge
/// lying in the end face's plane. `None` at every other end.
///
/// Bevel to Center Feature's left-slope top edge ends on the notch's crease against the right
/// slope, and the top face runs on past it: the top face's own flat edge v18 - v7 is collinear with
/// the right slope's top edge, so the right slope's plane carries on across the top. Unclipped, the
/// square end reads the end vertex's ball there, and the round curled on round the crease vertex
/// into the top face's far triangles and stopped a radius out, standing up to 3.08 past the plane
/// with ~79 degree creases. The Onshape control ends the round flat in that plane; the clip is that
/// plane (`BevelToCenterBlendCreaseTests`).
///
/// Read SAME sense although the end face folds against the chain at the crease: the end face lies
/// IN the clip plane, so the clipped field reads exactly zero over it and the level line wandered
/// along the right slope's top edge; the fill refused FillWouldNotClose at most sizes on the same
/// edge picked alone. Same sense puts the corners where the rails meet the plane on the plane's
/// edges (`EndClipCorners`) and caps the end in that plane.
pub fn square_crease_end_clips(
    positions: &[Vector3],
    indices: &[Corners],
    chain: &[i32],
    closed: bool,
    rays: (bool, bool),
) -> (Option<EndRayBound>, Option<EndRayBound>) {
    if !square_crease_clips_on() || closed || chain.len() < 2 {
        return (None, None);
    }

    let first = if rays.0 {
        None
    } else {
        clip_at(positions, indices, chain[0], chain[1])
    };
    let last = if rays.1 {
        None
    } else {
        clip_at(
            positions,
            indices,
            chain[chain.len() - 1],
            chain[chain.len() - 2],
        )
    };
    (first, last)
}

/// C#'s local `ClipAt` of `SquareCreaseEndClips`.
#[allow(clippy::neg_cmp_op_on_partial_ord)]
fn clip_at(positions: &[Vector3], indices: &[Corners], end: i32, next: i32) -> Option<EndRayBound> {
    let v = positions[end as usize];
    let outward = (v - positions[next as usize]).get_normal();
    let sides: Vec<Corners> = indices
        .iter()
        .copied()
        .filter(|&t| has(t, end) && has(t, next))
        .collect();
    if sides.len() != 2 || !is_convex(positions, sides[0], sides[1], end) {
        return None;
    }

    let side_normals: Vec<Vector3> = sides.iter().map(|&t| face_normal(positions, t)).collect();
    let in_a_side = |normal: Vector3| {
        side_normals
            .iter()
            .any(|s| s.dot(normal) >= 1.0 - COPLANAR_TOLERANCE)
    };
    let end_triangles: Vec<Corners> = indices
        .iter()
        .copied()
        .filter(|&t| has(t, end) && !has(t, next) && !in_a_side(face_normal(positions, t)))
        .collect();
    if end_triangles.is_empty() {
        return None;
    }

    let mut n = face_normal(positions, end_triangles[0]);
    let bent = end_triangles
        .iter()
        .any(|&t| face_normal(positions, t).dot(n) < 1.0 - COPLANAR_TOLERANCE);
    if bent || !(n.dot(outward).abs() >= SQUARE_COSINE) {
        return None;
    }

    n = if n.dot(outward) > 0.0 { n } else { -n };

    // On an unpicked CONCAVE crease: some side folds into the end face against the chain's own
    // sense. A fanned side meets the end face on its own triangle at the end, not on the chain
    // edge's.
    let side_faces: Vec<Corners> = indices
        .iter()
        .copied()
        .filter(|&t| has(t, end) && in_a_side(face_normal(positions, t)))
        .collect();
    // The crease's length from the end vertex bounds the room: the round's section in the end plane
    // meets the crease a setback down it, so past its far vertex the section's foot leaves the
    // crease (onto the nose on Bevel to Center Feature, 6.667 down) and the fill's arc sinks into
    // the solid.
    let mut crease_length = f64::INFINITY;
    for &side in &side_faces {
        for &other in &end_triangles {
            let shared = shared_corners(side, other);
            if shared.len() == 2 && shared.contains(&end) && !is_convex(positions, side, other, end)
            {
                if let Some(&far) = shared.iter().find(|&&c| c != end) {
                    crease_length =
                        cs_math::min_f64(crease_length, (positions[far as usize] - v).length());
                }
            }
        }
    }

    if crease_length == f64::INFINITY {
        return None;
    }

    // A flat edge of a side's face, out of the end vertex, lying in the end face's plane: both its
    // triangles in that side's plane, so the side runs on past the end and the plane runs on
    // across it.
    let mut carries_the_plane = false;
    for &t in &side_faces {
        for w in [t.0, t.1, t.2] {
            if w == end || w == next {
                continue;
            }

            let along = positions[w as usize] - v;
            if along.dot(n).abs() > 1e-6 * along.length() {
                continue;
            }

            let standing: Vec<Corners> = indices
                .iter()
                .copied()
                .filter(|&o| has(o, end) && has(o, w))
                .collect();
            carries_the_plane |= standing.len() == 2
                && face_normal(positions, standing[0]).dot(face_normal(positions, standing[1]))
                    >= 1.0 - COPLANAR_TOLERANCE;
        }
    }

    if carries_the_plane {
        Some(EndRayBound {
            reach_per_setback: 0.0,
            plane_normal: n,
            same_sense: true,
            crease_room: crease_length,
        })
    } else {
        None
    }
}
