//! The end-face clip at the unrayed end of an open convex chain that stops on an unpicked concave
//! crease against one flat end face square to it: the per-flow gate ([`square_crease_clips_on`],
//! [`turn_square_crease_clips_off`] and its scope [`SquareCreaseEndClipSquareCreaseClipsOffScope`],
//! kept on the build's [`WorkBudget`](super::WorkBudget) so it travels with the build) and the
//! clips read
//! off plain positions ([`square_crease_end_clips`]), and the clips over a pick with the room they
//! leave ([`square_crease_end_clips_of_pick`], [`square_crease_end_room`], [`cap_room`]); the room
//! is the reach of the end's concave crease, followed from chord to chord where it is a faceted
//! curve. Reads the end-face sense (`end_face_sense.rs`).
//! The end face may lean off square by ANY amount ([`leaning_clips_on`]) where its plane leaves
//! the whole chain on the band's side and no end ray takes the end: an end face that holds the
//! round's section stops it flush whatever its lean (a faceted curve's end wall leans by half a
//! facet's turn; a steep wall at 35 degrees is pinned in `square_crease_faceted_room_tests`), and a
//! side may
//! carry on past the end across a flat edge in the plane or under it across a convex crease; a
//! pick with a leaning clip ([`a_clip_leans`]) is first built with square clips only, and where
//! that build flows over a narrow end face that answer stands (`band_gap_entry_points`).

use std::collections::HashSet;

use vector_math::{ieee_math, Vector3};

use super::{end_face_sense, pick_validation, RoomSource};

use super::end_face_sense::{
    carries_on, face_normal, is_convex, shared_corners, Corners, COPLANAR_TOLERANCE,
};
use super::{EndRayBound, LevelLineWork, WorkBudget};

/// Whether [`square_crease_end_clips`] reads any
/// clip: on unless a test turns it off with [`turn_square_crease_clips_off`] to read the unclipped
/// ball end. Per build, as the run-outs switch is (on wherever no budget is in force).
pub fn square_crease_clips_on() -> bool {
    LevelLineWork::current().is_none_or(|budget| !budget.square_crease_clips_off.get())
}

/// Whether a square crease end clip may lean off square in the build in force: on unless that
/// build turned leaning clips off ([`WorkBudget::leaning_crease_clips_off`](super::WorkBudget)).
pub fn leaning_clips_on() -> bool {
    LevelLineWork::current().is_none_or(|budget| !budget.leaning_crease_clips_off.get())
}

/// Whether some chain end of the pick takes a square crease clip only because it may lean: the
/// pick's clips, read exactly as the build reads them (each chain's end rays at
/// `sharp_turn_degrees`, a pick corner's end taking none: [`square_crease_end_clips_of_pick`]),
/// differ with leaning clips allowed and not. A rayed or corner end never takes a clip, so it
/// never asks for the second build. Read outside any build, so leaning is on.
pub fn a_clip_leans(
    positions: &[Vector3],
    indices: &[Corners],
    chains: &[Vec<i32>],
    closed: &[bool],
    sharp_turn_degrees: f64,
) -> bool {
    let rays: Vec<(bool, bool)> = chains
        .iter()
        .enumerate()
        .map(|(k, chain)| {
            end_face_sense::oblique_ends(positions, indices, chain, closed[k], sharp_turn_degrees)
        })
        .collect();
    let leaning = square_crease_end_clips_of_pick(positions, indices, chains, closed, &rays);
    let Ok(off) = WorkBudget::new(1) else {
        return false;
    };
    off.leaning_crease_clips_off.set(true);
    let square = {
        let _work = LevelLineWork::begin(Some(std::rc::Rc::new(off)));
        square_crease_end_clips_of_pick(positions, indices, chains, closed, &rays)
    };
    leaning != square
}

/// Turns the clips off for the build in force
/// until the returned scope is dropped.
pub fn turn_square_crease_clips_off() -> SquareCreaseEndClipSquareCreaseClipsOffScope {
    SquareCreaseEndClipSquareCreaseClipsOffScope::new()
}

/// Turns the square-crease clips off for the
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
    /// Remembers the setting and turns the clips off.
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
    /// Puts back what was there before.
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

/// The end-face clip (`ChainField.EndClip`, no ray)
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
        clip_at(positions, indices, chain, chain[0], chain[1])
    };
    let last = if rays.1 {
        None
    } else {
        clip_at(
            positions,
            indices,
            chain,
            chain[chain.len() - 1],
            chain[chain.len() - 2],
        )
    };
    (first, last)
}

/// The clip at chain end `end`, whose next chain vertex is `next`, or `None` where it does not
/// clip.
#[allow(clippy::neg_cmp_op_on_partial_ord)]
fn clip_at(
    positions: &[Vector3],
    indices: &[Corners],
    chain: &[i32],
    end: i32,
    next: i32,
) -> Option<EndRayBound> {
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

    let end_normal = face_normal(positions, end_triangles[0]);
    let mut n = end_normal;
    let bent = end_triangles
        .iter()
        .any(|&t| face_normal(positions, t).dot(n) < 1.0 - COPLANAR_TOLERANCE);
    if bent {
        return None;
    }

    // An end face leaning off square clips too, at its own plane, whatever its lean, so long as the
    // plane leaves the whole chain on the band's side ([`leans_clear_of_its_chain`]) and no ray
    // takes the end: an end face that holds the round's section stops it flush as a square one
    // does (a faceted curve's end wall leans by half a facet's turn, a steep wall further). Only a
    // SEPARATE end face: one that carries either side on (`end_face_sense::carries_on`, the one
    // reading of the faceted curve carrying on, a 12-gon's 30 degrees included) is that side's
    // next facet, where the round runs out instead. Separate is exactly not carrying on, so the
    // limit falls the same way here as everywhere else.
    n = if n.dot(outward) > 0.0 { n } else { -n };
    let square = n.dot(outward) >= SQUARE_COSINE;
    let separate = !side_normals
        .iter()
        .any(|side| carries_on(side.dot(n).abs()));
    if !square
        && (!leaning_clips_on() || !separate || !leans_clear_of_its_chain(positions, chain, v, n))
    {
        return None;
    }

    // On an unpicked CONCAVE crease: some side folds into the end face against the chain's own
    // sense. A fanned side meets the end face on its own triangle at the end, not on the chain
    // edge's.
    let side_faces: Vec<Corners> = indices
        .iter()
        .copied()
        .filter(|&t| has(t, end) && in_a_side(face_normal(positions, t)))
        .collect();
    // The crease's reach from the end vertex bounds the room: the round's section in the end plane
    // meets the crease a setback from the end vertex, so past the crease's last vertex the
    // section's foot leaves the crease (onto the nose on Bevel to Center Feature, 6.667 down) and
    // the fill's arc sinks into the solid. A faceted crease carries the foot on from chord to
    // chord ([`crease_reach`]).
    let mut crease_length = f64::INFINITY;
    for &side in &side_faces {
        for &other in &end_triangles {
            let shared = shared_corners(side, other);
            if shared.len() == 2 && shared.contains(&end) && !is_convex(positions, side, other, end)
            {
                if let Some(&far) = shared.iter().find(|&&c| c != end) {
                    let reach = crease_reach(positions, indices, end, far, side, end_normal);
                    crease_length = ieee_math::min_f64(crease_length, reach);
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
                && (face_normal(positions, standing[0]).dot(face_normal(positions, standing[1]))
                    >= 1.0 - COPLANAR_TOLERANCE
                    || (!square && carries_on_under(positions, &standing, end, &in_a_side)));
        }
    }

    if carries_the_plane {
        Some(EndRayBound {
            reach_per_setback: 0.0,
            plane_normal: n,
            same_sense: true,
            crease_room: crease_length,
            stopped: false,
            plane_tilt: 0.0,
        })
    } else {
        None
    }
}

/// How far from the end vertex `end` the concave crease leaving it along the chord `end` - `far`
/// reaches, `side` the face beside that chord and `end_normal` the end face's normal. The round's
/// section lies in the end face's plane, and its foot stands on the crease where the crease is a
/// setback from the end vertex; on a FACETED crease (an end wall meeting a faceted curve) the foot
/// rides on from chord to chord. The crease runs on from a vertex along the one next chord that is
/// again a concave crease between the end face and a face, where that face carries the last one on
/// (`end_face_sense::carries_on`, the faceted curve carrying on, read as the oblique end reader
/// reads a facet), and only while each vertex stands farther from the end vertex than
/// the last: where the crease turns back, or meets a real corner, the foot leaves it there.
fn crease_reach(
    positions: &[Vector3],
    indices: &[Corners],
    end: i32,
    far: i32,
    side: Corners,
    end_normal: Vector3,
) -> f64 {
    let origin = positions[end as usize];
    let in_the_end_face =
        |t: Corners| face_normal(positions, t).dot(end_normal) >= 1.0 - COPLANAR_TOLERANCE;
    let (mut previous, mut current, mut face) = (end, far, side);
    let mut reach = (positions[far as usize] - origin).length();
    loop {
        // Each next chord out of `current`: its two triangles, one in the end face and one beside
        // it, folding concavely, the face beside it carrying the last one's surface on.
        let mut next: Option<(i32, Corners)> = None;
        let mut several = false;
        for &t in indices.iter().filter(|&&t| has(t, current)) {
            if in_the_end_face(t) {
                continue;
            }

            for w in [t.0, t.1, t.2] {
                if w == current || w == previous {
                    continue;
                }

                let standing: Vec<Corners> = indices
                    .iter()
                    .copied()
                    .filter(|&o| has(o, current) && has(o, w))
                    .collect();
                if standing.len() != 2 {
                    continue;
                }

                let Some(&wall) = standing.iter().find(|&&o| in_the_end_face(o)) else {
                    continue;
                };
                if is_convex(positions, t, wall, current)
                    || !carries_on(face_normal(positions, t).dot(face_normal(positions, face)))
                {
                    continue;
                }

                match next {
                    Some((at, _)) if at == w => {}
                    Some(_) => several = true,
                    None => next = Some((w, t)),
                }
            }
        }

        // No chord on, or more than one (the crease forks): the crease ends here.
        let Some((w, t)) = next.filter(|_| !several) else {
            return reach;
        };
        let farther = (positions[w as usize] - origin).length();
        // A NaN distance (a degenerate vertex) ends the crease as a nearer vertex does.
        if farther.partial_cmp(&reach) != Some(std::cmp::Ordering::Greater) {
            return reach;
        }

        reach = farther;
        (previous, current, face) = (current, w, t);
    }
}

/// How far past a leaning end face's plane a chain point may stand and still read as on it: float
/// noise at the point's magnitude, as a stopped end's plane reads its chain
/// (`stopped_end_clip.rs`).
const ON_ITS_PLANE_RELATIVE: f64 = 1e-9;

/// Whether an end face's plane through the end vertex `v`, unit `normal` faced on past the end,
/// leaves the whole chain on the band's side. The clip cuts the whole half-space past the plane,
/// so a plane that leans until some point of the chain stands past it would cut the chain's own
/// band; short of that, the plane meets the chain only at its end and the round stops on it.
fn leans_clear_of_its_chain(
    positions: &[Vector3],
    chain: &[i32],
    v: Vector3,
    normal: Vector3,
) -> bool {
    chain.iter().all(|&c| {
        let p = positions[c as usize];
        (p - v).dot(normal) <= ON_ITS_PLANE_RELATIVE * (1.0 + p.length())
    })
}

/// Whether the two triangles standing on an edge out of the end vertex are a side's face and the
/// end face meeting in a CONVEX crease: the side carries on past the end, under the end face's
/// plane, as a cylinder's wall carries on below a floor cut into it. The round must stop on the
/// end face's plane there just as where the side runs on across a flat edge in it.
fn carries_on_under(
    positions: &[Vector3],
    standing: &[Corners],
    end: i32,
    in_a_side: &dyn Fn(Vector3) -> bool,
) -> bool {
    let (a, b) = (standing[0], standing[1]);
    let (a_side, b_side) = (
        in_a_side(face_normal(positions, a)),
        in_a_side(face_normal(positions, b)),
    );
    a_side != b_side && is_convex(positions, a, b, end)
}

/// [`square_crease_end_clips`] over a pick, as the build reads
/// them - an end at a CORNER of the pick is the corner's business (as for the end rays), and an end
/// already carrying a ray's bound keeps it. Each entry is `(None, None)` where no end clips.
pub fn square_crease_end_clips_of_pick(
    positions: &[Vector3],
    indices: &[Corners],
    chains: &[Vec<i32>],
    closed: &[bool],
    end_rays: &[(bool, bool)],
) -> Vec<(Option<EndRayBound>, Option<EndRayBound>)> {
    let corner_ends: HashSet<i32> = if chains.len() > 1 {
        pick_validation::corners_of(chains, closed, true)
            .iter()
            .map(|corner| corner.vertex)
            .collect()
    } else {
        HashSet::new()
    };

    chains
        .iter()
        .enumerate()
        .map(|(k, chain)| {
            let (first, last) =
                square_crease_end_clips(positions, indices, chain, closed[k], end_rays[k]);
            let at_a_corner = |end: Option<&i32>| end.is_some_and(|v| corner_ends.contains(v));

            (
                if at_a_corner(chain.first()) {
                    None
                } else {
                    first
                },
                if at_a_corner(chain.last()) {
                    None
                } else {
                    last
                },
            )
        })
        .collect()
}

/// The least [`EndRayBound::crease_room`] over a pick's square crease
/// end clips - infinity where none clips - and the chain it is read at (-1 with none), as
/// `(room, chain)`. The build takes it, so a size past the crease is refused by size.
pub fn square_crease_end_room(
    positions: &[Vector3],
    indices: &[Corners],
    chains: &[Vec<i32>],
    closed: &[bool],
    sharp_turn_degrees: f64,
) -> (f64, i32) {
    let rays: Vec<(bool, bool)> = chains
        .iter()
        .enumerate()
        .map(|(k, chain)| {
            end_face_sense::oblique_ends(positions, indices, chain, closed[k], sharp_turn_degrees)
        })
        .collect();
    let clips = square_crease_end_clips_of_pick(positions, indices, chains, closed, &rays);
    let mut room = f64::INFINITY;
    let mut chain = -1;

    for (k, (first, last)) in clips.iter().enumerate() {
        for clip in [first, last].into_iter().flatten() {
            if clip.crease_room < room {
                room = clip.crease_room;
                chain = k as i32;
            }
        }
    }

    (room, chain)
}

/// Lowers `room` to [`square_crease_end_room`] where that is less, naming the chain
/// it is read at in `room_set_by`. A square crease end's section stays on its crease only so far
/// down it - past the crease's far vertex the arc's foot leaves the crease and no cap exists (the
/// fill refused `ArcSinksIntoTheSolid` at 7 on 15mm Bevel Top Edge) - and the build's foreign
/// reading plateaus at zero on the clip plane, so it does not say so itself.
pub fn cap_room(
    positions: &[Vector3],
    indices: &[Corners],
    chains: &[Vec<i32>],
    closed: &[bool],
    sharp_turn_degrees: f64,
    room: &mut f64,
    room_set_by: &mut RoomSource,
) {
    let (crease_room, crease_chain) =
        square_crease_end_room(positions, indices, chains, closed, sharp_turn_degrees);

    if crease_room < *room {
        *room = crease_room;
        *room_set_by = RoomSource::foreign(crease_chain);
    }
}
