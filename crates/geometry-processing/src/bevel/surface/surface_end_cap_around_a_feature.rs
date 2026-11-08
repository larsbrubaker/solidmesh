//! The section a band's run-out end gives up, SPLIT by a feature standing in it:
//! the upper piece the corner
//! still sees, the sliver beyond the feature's far trace, and nothing at all between them - the
//! extended feature is material there, so that stretch of the end plane is interior and carries no
//! face. Here are its nested record, where one piece meets the feature
//! ([`SurfaceEndCapAroundAFeatureMeeting`]), the walk round the section's rim that splits it
//! ([`try_walk_around`]), the stitches read along the profile ([`try_the_betweens`]) and the ring
//! tests ([`the_area_of`], [`is_simple`]), and the capping itself ([`try_cap`]).

use solidmesh_math::Vector3;

use super::{
    surface_end_cut, surface_end_notch, surface_fan, SurfaceStripNotchedByAFeature,
    SurfaceVertexPool,
};
use crate::bevel::{BevelAdvisory, BevelAdvisoryCode};

/// Where one of the two
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
    /// Where a piece meets the feature: the piece's Q on the strip, and the crease vertex standing
    /// in the end plane.
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

/// The section's rim read from the run-off side in, broken at the feature - the
/// run of ring vertices before the notch, the run after it, and which Q and crease vertex each of
/// them ends on. `Err` is why it would not split. A one-faced notch has no tail, and its tail
/// meeting is the default.
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

/// The chord of the strip the walk crosses between step `at` and the one before it.
fn the_chord(walk: &[i32], at: usize) -> i32 {
    walk[at - 1].min(walk[at])
}

/// The Q standing on one chord, and the vertex the face that drew it lost its
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

/// The same vertex twice over is one corner of the rim, not two - the collapsed rails
/// all stand on one point. True where the corner was a new one.
fn carry(walk: &mut Vec<i32>, vertex: i32) -> bool {
    if walk.last() != Some(&vertex) {
        walk.push(vertex);
        return true;
    }

    false
}

/// The stitch each chord of the end's ring carries, by the LOWER rail of the pair it spans:
/// `along_the_profile[chord]` (the vertices found standing on chord `chord` of `profile`, `None`
/// or empty where none) mapped onto the rail pair `walk` gives the two `head_step` ends, read from
/// the lower rail. `rails` is the strip's rail count. Refused, at the first such vertex, where a
/// chord carries vertices but the ring has no neighbouring rail pair for it - the stitch has
/// nothing to sew it to, so it is named here rather than dropped.
pub fn try_the_betweens(
    pool: &SurfaceVertexPool,
    walk: &[i32],
    head_step: &[i32],
    profile: &[i32],
    along_the_profile: &[Option<Vec<i32>>],
    rails: i32,
) -> Result<Vec<Vec<i32>>, BevelAdvisory> {
    let mut between = vec![Vec::new(); i32::max(0, rails - 1) as usize];

    let mut chord = 0;
    while chord + 1 < profile.len() && chord < along_the_profile.len() {
        let Some(found) = along_the_profile[chord]
            .as_ref()
            .filter(|found| !found.is_empty())
        else {
            chord += 1;
            continue;
        };

        let from = head_step.get(chord).copied().unwrap_or(-1);
        let to = head_step.get(chord + 1).copied().unwrap_or(-1);

        if from < 0 || to < 0 || (walk[from as usize] - walk[to as usize]).abs() != 1 {
            // Standing on a chord the ring has no neighbouring rail pair for - the chord into the
            // feature's Q, the one into the crease vertex, or one spanning rails a trim collapsed
            // onto a single point.
            return Err(BevelAdvisory::new(
                BevelAdvisoryCode::SurfaceStripSeamDidNotMeet,
                vec![pool.position_of(found[0]).into()],
            ));
        }

        let (from, to) = (walk[from as usize], walk[to as usize]);
        let mut carried = found.clone();

        if from > to {
            // A walk running DOWN the rails reads each chord from its higher rail, and a
            // station's between is read from the lower one.
            carried.reverse();
        }

        between[from.min(to) as usize] = carried;
        chord += 1;
    }

    Ok(between)
}

/// The signed area of the polygon `ring` (a fan from its first vertex), seen along `facing`.
pub fn the_area_of(ring: &[i32], pool: &SurfaceVertexPool, facing: Vector3) -> f64 {
    let mut twice = Vector3::ZERO;
    let first = pool.position_of(ring[0]);

    for at in 1..ring.len().saturating_sub(1) {
        twice += (pool.position_of(ring[at]) - first).cross(pool.position_of(ring[at + 1]) - first);
    }

    twice.dot(facing) / 2.0
}

/// Whether `ring` is a simple polygon by index: at least three vertices, none repeated.
pub fn is_simple(ring: &[i32]) -> bool {
    if ring.len() < 3 {
        return false;
    }

    for at in 0..ring.len() {
        for other in (at + 1)..ring.len() {
            if ring[at] == ring[other] {
                return false;
            }
        }
    }

    true
}

/// The section of a run-out end (facing `runs_out_facing`) split by the feature
/// standing in it - the head piece the corner still sees, capped as the section is, and the sliver
/// past the feature's far trace with whatever the walkback left on its straight side sewn in.
/// Returns the stitch each chord of the end's ring carries, or why the end
/// cannot be capped round the feature.
#[allow(clippy::too_many_arguments)]
pub fn try_cap(
    pool: &mut SurfaceVertexPool,
    faces: &mut Vec<(i32, i32, i32)>,
    walk: &[i32],
    the_die_point: i32,
    notch: &SurfaceStripNotchedByAFeature,
    ring: &[i32],
    corner: Vector3,
    runs_out_facing: Vector3,
) -> Result<Vec<Vec<i32>>, BevelAdvisory> {
    let does_not_extend = |at: Vector3| {
        BevelAdvisory::new(
            BevelAdvisoryCode::SurfaceStandingFeatureDoesNotExtend,
            vec![at.into()],
        )
    };

    #[allow(clippy::neg_cmp_op_on_partial_ord)] // NaN refuses, as `!(x > 0)` reads it
    let no_wall = !(runs_out_facing.length_squared() > 0.0);
    if no_wall {
        // The end has a face BEYOND it rather than a wall of its own: named rather than capped.
        return Err(does_not_extend(corner));
    }

    let facing = runs_out_facing.get_normal();

    let (head, head_step, tail, at_the_head, at_the_tail) =
        try_walk_around(pool, walk, the_die_point, notch, ring, corner)?;

    let mut profile = head;
    profile.push(at_the_head.q);
    profile.push(at_the_head.crease);

    if !is_simple(&profile) {
        return Err(does_not_extend(pool.position_of(at_the_head.q)));
    }

    let along_the_profile =
        surface_end_notch::try_cap_with_the_section(pool, faces, &profile, corner, facing)?;
    let along: Vec<Option<Vec<i32>>> = along_the_profile.into_iter().map(Some).collect();
    let between = try_the_betweens(pool, walk, &head_step, &profile, &along, ring.len() as i32)?;

    if tail.is_empty() {
        // A one-faced notch: no sliver beyond it to close.
        return Ok(between);
    }

    // The sliver's own straight side, between the foot and the crease vertex, with whatever the
    // walkback left standing along it sewn in.
    let foot = tail[tail.len() - 1];
    let mut sliver = Vec::with_capacity(tail.len() + 3);
    sliver.push(at_the_tail.q);
    sliver.extend(&tail);
    sliver.extend(surface_end_cut::standing_between(
        pool,
        &surface_end_cut::vertices_of(faces),
        foot,
        at_the_tail.crease,
    ));
    sliver.push(at_the_tail.crease);

    if !is_simple(&sliver) {
        return Err(does_not_extend(pool.position_of(at_the_tail.q)));
    }

    #[allow(clippy::neg_cmp_op_on_partial_ord)] // NaN refuses, as `!(x > 0)` reads it
    let covers_nothing = !(the_area_of(&sliver, pool, facing) > 0.0);
    if covers_nothing {
        // A sliver of no area, or wound the other way about, covers nothing.
        return Err(does_not_extend(pool.position_of(at_the_tail.q)));
    }

    // The sliver carries no betweens of its own: it is a ring of points already standing.
    surface_fan::emit_facing(&sliver, pool, faces, facing);

    Ok(between)
}
