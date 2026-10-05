//! The section a band's run-out end gives up, SPLIT by a feature standing in it
//! (`Bevel/Surface/SurfaceEndCapAroundAFeature.cs`, a static class): the upper piece the corner
//! still sees, the sliver beyond the feature's far trace, and nothing at all between them - the
//! extended feature is material there, so that stretch of the end plane is interior and carries no
//! face. Here are its nested record, where one piece meets the feature
//! ([`SurfaceEndCapAroundAFeatureMeeting`]), and the walk round the section's rim that splits it
//! ([`try_walk_around`]); the capping itself (`TryCap`) lands with a later package.

use vector_math::Vector3;

use super::{SurfaceStripNotchedByAFeature, SurfaceVertexPool};
use crate::bevel::{BevelAdvisory, BevelAdvisoryCode};

/// C# `SurfaceEndCapAroundAFeature.Meeting` (a private readonly struct): where one of the two
/// pieces meets the feature - its Q on the strip, and the crease vertex standing in the end plane.
///
/// Every vertex of both pieces is one the notch or the trim already minted - the two Qs, the two
/// crease ends, the strip's own end ring - so the pieces, the extension and the notched strip close
/// on shared indices rather than on points a rounding apart.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, Default)]
pub struct SurfaceEndCapAroundAFeatureMeeting {
    /// The feature's Q on the strip's end ring.
    pub q: i32,
    /// The crease vertex standing in the end plane.
    pub crease: i32,
}

impl SurfaceEndCapAroundAFeatureMeeting {
    /// C#'s `Meeting` constructor: the piece's Q on the strip, and the crease vertex standing in the
    /// end plane.
    pub fn new(q: i32, crease: i32) -> Self {
        Self { q, crease }
    }
}

/// What [`try_walk_around`] reads off the section's rim: the run of ring vertices before the notch
/// (`head`), the walk step each of them came from (`head_step`, -1 for the die point), the run after
/// it (`tail`), and where each of the two runs meets the feature.
pub type SurfaceEndCapWalkAround = (
    Vec<i32>,
    Vec<i32>,
    Vec<i32>,
    SurfaceEndCapAroundAFeatureMeeting,
    SurfaceEndCapAroundAFeatureMeeting,
);

/// C# `TryWalkAround`: the section's rim read from the run-off side in, broken at the feature - the
/// run of ring vertices before the notch, the run after it, and which Q and crease vertex each of
/// them ends on. `Err` is C#'s `failure`. A one-faced notch has no tail, and its tail meeting is
/// the default (C#'s `default(Meeting)`).
///
/// Walked in the order the profile is read in, so the head starts where the section's rim starts -
/// the part's corner beyond at a run-off end, the first rail at a landing one - and the tail ends on
/// the strip's tangent foot, whichever side of the strip ran off and whichever way the chain was
/// linked.
pub fn try_walk_around(
    pool: &SurfaceVertexPool,
    walk: &[i32],
    the_die_point: i32,
    notch: &SurfaceStripNotchedByAFeature,
    ring: &[i32],
    corner: Vector3,
) -> Result<SurfaceEndCapWalkAround, BevelAdvisory> {
    let does_not_extend = || {
        BevelAdvisory::new(
            BevelAdvisoryCode::SurfaceStandingFeatureDoesNotExtend,
            vec![corner.into()],
        )
    };

    let mut head = if the_die_point >= 0 {
        vec![the_die_point]
    } else {
        Vec::new()
    };
    let mut head_step = if the_die_point >= 0 {
        vec![-1]
    } else {
        Vec::new()
    };
    let mut tail = Vec::new();

    let steps = walk.len();
    let mut at = 0;

    while at < steps && !notch.notches(walk[at]) {
        if carry(&mut head, ring[walk[at] as usize]) {
            head_step.push(at as i32);
        }

        at += 1;
    }

    if at == 0 || at >= steps {
        // The notch takes the very first rail of the walk, or none of the rails the walk reaches:
        // either way there is no chord between the section's rim and the feature for a Q to stand on.
        return Err(does_not_extend());
    }

    let into_the_notch = the_chord(walk, at);

    while at < steps && notch.notches(walk[at]) {
        at += 1;
    }

    if at >= steps {
        if notch.is_one_faced() {
            // Expected, and the whole shape of a one-faced notch: the trace ends on the strip's own
            // SEAM rail, which is the last step of the walk, so the section is cut in TWO and not in
            // three. NAMED where its one Q does not stand on the chord the walk meets it at, and not
            // handed back as a refusal with nothing to say.
            let at_the_head =
                try_the_meeting(pool, notch, into_the_notch).ok_or_else(does_not_extend)?;

            return Ok((
                head,
                head_step,
                tail,
                at_the_head,
                SurfaceEndCapAroundAFeatureMeeting::default(),
            ));
        }

        // The notch runs off the far edge of the profile rather than back onto it, so the section has
        // no sliver beyond it and the trace leaves through the strip's own seam.
        return Err(does_not_extend());
    }

    let out_of_the_notch = the_chord(walk, at);

    while at < steps {
        let rail = walk[at];

        if notch.notches(rail) {
            // A second run of notched rails further round. One notch is one hole; two would want two
            // slivers and two more pieces, which this split does not draw.
            return Err(does_not_extend());
        }

        carry(&mut tail, ring[rail as usize]);
        at += 1;
    }

    if tail.is_empty() {
        return Err(does_not_extend());
    }

    let at_the_head = try_the_meeting(pool, notch, into_the_notch).ok_or_else(does_not_extend)?;
    let at_the_tail = try_the_meeting(pool, notch, out_of_the_notch).ok_or_else(does_not_extend)?;

    Ok((head, head_step, tail, at_the_head, at_the_tail))
}

/// C# `TheChord`: the chord of the strip the walk crosses between step `at` and the one before it.
fn the_chord(walk: &[i32], at: usize) -> i32 {
    walk[at - 1].min(walk[at])
}

/// C# `TryTheMeeting`: the Q standing on one chord, and the vertex the face that drew it lost its
/// footing at in the end plane - which is the end of that face's crease the feature's own RIDGE does
/// not carry.
pub fn try_the_meeting(
    pool: &SurfaceVertexPool,
    notch: &SurfaceStripNotchedByAFeature,
    chord: i32,
) -> Option<SurfaceEndCapAroundAFeatureMeeting> {
    for q in [&notch.at_the_low_rail, &notch.at_the_high_rail] {
        if !q.stands() || q.below != chord {
            continue;
        }

        let face = q.face.as_ref()?;
        let (_, on_the_end) = notch
            .feature
            .try_the_ends_of_the_crease(pool, face, notch.reach)?;

        return Some(SurfaceEndCapAroundAFeatureMeeting::new(
            q.vertex, on_the_end,
        ));
    }

    None
}

/// C# `Carry`: the same vertex twice over is one corner of the rim, not two - the collapsed rails
/// all stand on one point. True where the corner was a new one.
fn carry(walk: &mut Vec<i32>, vertex: i32) -> bool {
    if walk.last() != Some(&vertex) {
        walk.push(vertex);
        return true;
    }

    false
}
