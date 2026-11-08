//! THE MITRE OF TWO ARCS THAT WERE NOT CHORDED THE SAME WAY (free functions): the ring each strip
//! meets the mitre plane on is still one vertex per rail, and the OTHER strip's crossings are
//! carried as points BETWEEN two of its rails, so the two ends sew to one polyline - the union of
//! both strips' crossings, ordered by their angle from the face the two beads share.
//! `surface_strip_mitre::try_mitre` asks it.
//!
//! **Why the two rings normally are one ring.** Two beads of one radius tangent to the face they
//! share carry their rail at angle theta in the same plane `r (1 - cos theta)` above that face, so
//! rail theta of one crosses rail theta of the other ON the mitre plane whatever the two wedges are.
//! The mitre therefore pairs rails by counting steps out from the shared face - and step `k` is the
//! same theta in both only where both arcs took ONE angular step. `BevelArcChording::segments_for`
//! rounds the chord count per arc, so two sweeps whose ratio does not survive that rounding - 90
//! degrees in 15 chords of exactly 6 against 66.907 in 11 of 6.0824 - put step `k` at two different
//! angles and the two rings apart.
//!
//! **What is built instead.** Nothing of one ring per rail is given up. Each strip keeps its own
//! rail crossings as its ring; every crossing of the OTHER strip that falls strictly between two of
//! its rails by angle is put on that chord as a between-station, in order. That is the same carry an
//! end cut's own breaks already travel by, and the stitch already lays a quad as a RING rather than
//! as two triangles so that whatever is between gets used. Both ends then read the same sequence of
//! pool vertices, so every edge on the mitre plane is used exactly twice and the shell closes.
//!
//! **What it costs, and why that is the price already being paid.** A foreign crossing is put on a
//! chord it does not lie exactly on. How far off it can be is the chord's own SAG, `r (1 -
//! cos(step / 2))` - 0.0014 mm at radius 1 and a 6 degree step - because the point stands on the
//! true arc between the chord's two ends. That is the tolerance chording the arc at that step
//! already spends: the polyline is no further from the bead than the bead's own facets are.
//!
//! **Where it does not reach.** Only the rails the two strips PAIR are read here. Past that the
//! wider bead has rails the narrower has nothing to meet, and those go on landing on the narrower
//! edge's far face exactly as they did (`surface_strip_mitre`'s rails with no partner,
//! [`super::surface_mitre_sector`]).

// The negated comparisons are deliberate: the comparison inside each is false for NaN, so a NaN
// takes the negated branch.
#![allow(clippy::neg_cmp_op_on_partial_ord)]

use solidmesh_math::Vector3;

use super::surface_mitre_joint_step::SideSeams;
use super::{surface_strip_ends, surface_strip_mitre_rails};
use super::{SurfaceMitreSliver, SurfaceStrip, SurfaceVertexPool};
use crate::bevel::{BevelAdvisory, BevelAdvisoryCode};

/// How near two rails' angles have to read to be one angle, counted in STEPS. A millionth of a
/// step: far above the last bits of two sweeps read off two different sets of feet, and far below
/// the whole step a rail that is really a different rail stands away by. At a 6 degree step and
/// radius 1 it is 1e-7 mm on the bead, which is a hundredth of the pool's own weld.
const ANGLE_RULER: f64 = 1e-6;

/// One strip's side of a chorded-apart mitre: the strip, its edge index, the rail standing on the
/// shared face and the direction its rails count out from there.
#[derive(Clone, Copy)]
pub struct ChordedApartSide<'a> {
    /// The strip.
    pub strip: &'a SurfaceStrip,
    /// Its edge index.
    pub edge: i32,
    /// The strip's rail standing on the face the two share.
    pub on_the_face: i32,
    /// +1 where the rails count up from that face, -1 where they count down.
    pub step: i32,
}

/// What [`try_the_rails`] hands back besides the rings it fills.
#[derive(Debug, Clone, Default)]
pub struct ChordedApartRails {
    /// What the leaving strip's end ring carries on each of its chords.
    pub between_leaving: Vec<Vec<i32>>,
    /// The same for the arriving strip's start ring.
    pub between_arriving: Vec<Vec<i32>>,
    /// How many of the leaving strip's rails the mitre stopped - counted by ANGLE rather than by
    /// rail, because the two arcs step differently: a rail past the other bead's own sweep has
    /// nothing over it to be mitred against and lands on the face beyond instead.
    pub paired_on_mine: i32,
    /// The same for the arriving strip.
    pub paired_on_theirs: i32,
    /// Where the leaving strip leaves the mitre plane for the face beyond, when that is no rail of
    /// its own: the OTHER bead's outermost rail crossing, which stands on the mitre plane and on that
    /// face at once. -1 where this strip is the narrower of the two and has no such crossing.
    pub transition_on_mine: i32,
    /// The same for the arriving strip.
    pub transition_on_theirs: i32,
}

/// Both strips' rails over the angle the two beads SHARE, where the two arcs were
/// chorded at different angular steps. Every pair that does land at one angle is left to the
/// ordinary pair reading, so a corner whose steps agree on some rails is built there exactly as it
/// always was. `through` is a point the mitre plane passes through, `normal` its normal; `ring`,
/// `next_ring`, `leaving_at` and `arriving_at` are filled rail by rail.
#[allow(clippy::too_many_arguments)]
pub fn try_the_rails(
    pool: &mut SurfaceVertexPool,
    mine_side: ChordedApartSide<'_>,
    theirs_side: ChordedApartSide<'_>,
    seams: &[SideSeams],
    corner: Vector3,
    through: Vector3,
    normal: Vector3,
    sliver: Option<&SurfaceMitreSliver>,
    ring: &mut [i32],
    next_ring: &mut [i32],
    leaving_at: &mut [f64],
    arriving_at: &mut [f64],
) -> Result<ChordedApartRails, BevelAdvisory> {
    let (strip, edge_index, mine_on_the_face, mine_step) = (
        mine_side.strip,
        mine_side.edge,
        mine_side.on_the_face,
        mine_side.step,
    );
    let (next, next_index, theirs_on_the_face, theirs_step) = (
        theirs_side.strip,
        theirs_side.edge,
        theirs_side.on_the_face,
        theirs_side.step,
    );

    // The angle the two beads share is the narrower one's whole sweep: past it nothing of that bead
    // stands over the other, and the other's rails land on its far face rather than on the mitre.
    let overlap = sweep_of(strip).min(sweep_of(next));

    let paired_on_mine = steps_within(strip, overlap);
    let paired_on_theirs = steps_within(next, overlap);

    // What each strip has to carry of the other's, as (angle from the shared face, pool vertex).
    let mut onto_mine: Vec<(f64, i32)> = Vec::new();
    let mut onto_theirs: Vec<(f64, i32)> = Vec::new();

    for step in 0..paired_on_mine {
        let mine = mine_on_the_face + (mine_step * step);
        let angle = step as f64 * strip.rail_step_degrees;

        if let Some(sliver) = sliver.filter(|sliver| sliver.claims_the_foot(edge_index, mine)) {
            // A foot the patch claimed stops at the rim its walkback ended at, and the crossing it
            // steps out to is carried by the OTHER strip's ring like any other foreign point - that
            // step is one of the patch's own sides (SurfaceMitreSliver).
            sliver.take_the_foot(edge_index, ring, leaving_at);
            onto_theirs.push((angle, sliver.the_crossing_of(edge_index)));

            continue;
        }

        // The other strip's rail at this very angle, where it has one: those two do meet at a point
        // and share a vertex, which step 0 always does - both feet stand on the face the beads share.
        let their_step = the_step_at(next, angle, paired_on_theirs);

        if their_step >= 0 {
            let theirs = theirs_on_the_face + (theirs_step * their_step);

            // By the two strips' OWN rail indices, which are not one number: the rail standing on
            // the face they share is index 0 in one and the last in the other wherever the walk
            // turns that way, so the pair has to be named twice.
            let (shared, here, there) = surface_strip_mitre_rails::try_one_pair_of_the_mitre(
                pool, strip, next, seams, edge_index, next_index, mine, theirs, corner, through,
                normal,
            )?;
            leaving_at[mine as usize] = here;
            arriving_at[theirs as usize] = there;

            ring[mine as usize] = shared;
            next_ring[theirs as usize] = shared;

            continue;
        }

        let (vertex, at) = try_one_rail(
            pool, strip, seams, edge_index, mine, through, normal, corner,
        )?;
        leaving_at[mine as usize] = at;

        ring[mine as usize] = vertex;
        onto_theirs.push((angle, vertex));
    }

    for step in 0..paired_on_theirs {
        let theirs = theirs_on_the_face + (theirs_step * step);
        let angle = step as f64 * next.rail_step_degrees;

        if let Some(sliver) = sliver.filter(|sliver| sliver.claims_the_foot(next_index, theirs)) {
            // The same claim from the other side: which of the two strips the narrower run is, is
            // which way round the corner the chain walked.
            sliver.take_the_foot(next_index, next_ring, arriving_at);
            onto_mine.push((angle, sliver.the_crossing_of(next_index)));

            continue;
        }

        let mine_at = the_step_at(strip, angle, paired_on_mine);

        if mine_at >= 0
            && !sliver.is_some_and(|sliver| {
                sliver.claims_the_foot(edge_index, mine_on_the_face + (mine_step * mine_at))
            })
        {
            // Already taken above, as the shared vertex of a pair that does stand at one angle -
            // unless the patch claimed that rail, in which case the two no longer share a vertex and
            // this one still wants its own.
            continue;
        }

        let (vertex, at) = try_one_rail(
            pool, next, seams, next_index, theirs, through, normal, corner,
        )?;
        arriving_at[theirs as usize] = at;

        next_ring[theirs as usize] = vertex;
        onto_mine.push((angle, vertex));
    }

    let (between_leaving, transition_on_mine) = spread(
        strip,
        mine_on_the_face,
        mine_step,
        paired_on_mine,
        &onto_mine,
    );
    let (between_arriving, transition_on_theirs) = spread(
        next,
        theirs_on_the_face,
        theirs_step,
        paired_on_theirs,
        &onto_theirs,
    );

    Ok(ChordedApartRails {
        between_leaving,
        between_arriving,
        paired_on_mine,
        paired_on_theirs,
        transition_on_mine,
        transition_on_theirs,
    })
}

/// How far a strip's arc turns in all - its step times the chords it was cut into.
pub fn sweep_of(strip: &SurfaceStrip) -> f64 {
    (strip.rail_count() - 1) as f64 * strip.rail_step_degrees
}

/// How many of a strip's rails stand within `overlap` of the shared face - the
/// ones the other bead has surface over and the mitre therefore stops. Never more rails than it has.
fn steps_within(strip: &SurfaceStrip, overlap: f64) -> i32 {
    if !(strip.rail_step_degrees > 0.0) {
        return strip.rail_count();
    }

    // A floor of a finite, bounded ratio, so the cast never saturates.
    strip
        .rail_count()
        .min(((overlap / strip.rail_step_degrees) + ANGLE_RULER).floor() as i32 + 1)
}

/// Which of a strip's first `paired` steps stands at `angle`, or -1 where none
/// does. Read by dividing rather than by walking, because the rails are evenly spaced and the answer
/// has to be the same whichever of the two strips asks it.
fn the_step_at(strip: &SurfaceStrip, angle: f64, paired: i32) -> i32 {
    if !(strip.rail_step_degrees > 0.0) {
        return if angle < ANGLE_RULER { 0 } else { -1 };
    }

    let steps = angle / strip.rail_step_degrees;
    // Banker's rounding.
    let nearest = steps.round_ties_even() as i32;

    if nearest >= 0 && nearest < paired && (steps - nearest as f64).abs() < ANGLE_RULER {
        nearest
    } else {
        -1
    }
}

/// Whether the ordinary pairing - rail `k` of one against rail `k` of
/// the other, counted out from the face they share - leaves any pair standing apart. Read rather
/// than assumed from the two arcs' steps: two pieces of ONE crease have sweeps that agree to their
/// last bits and not always to every bit, and a corner whose rails do meet must go on being built
/// exactly where it was, so the question asked is the one the mitre actually needs answered.
/// Nothing is minted and nothing is written - the crossings are read off the strips and thrown away.
#[allow(clippy::too_many_arguments)]
pub fn the_pairing_by_index_misses(
    pool: &SurfaceVertexPool,
    strip: &SurfaceStrip,
    next: &SurfaceStrip,
    mine_on_the_face: i32,
    mine_step: i32,
    theirs_on_the_face: i32,
    theirs_step: i32,
    paired: i32,
    through: Vector3,
    normal: Vector3,
) -> bool {
    for step in 0..paired {
        let mine = mine_on_the_face + (mine_step * step);
        let theirs = theirs_on_the_face + (theirs_step * step);

        let (Some(here), Some(there)) = (
            strip.try_cross(mine, through, normal),
            next.try_cross(theirs, through, normal),
        ) else {
            // A rail the mitre plane runs along, which is a refusal either way round: left to the
            // ordinary reading, which has the better complaint for it.
            return false;
        };

        if (strip.point_on(mine, here) - next.point_on(theirs, there)).length()
            > surface_strip_ends::meets_the_seam(pool)
        {
            return true;
        }
    }

    false
}

/// One rail's crossing of the mitre plane, as a vertex and a parameter along the
/// edge.
#[allow(clippy::too_many_arguments)]
fn try_one_rail(
    pool: &mut SurfaceVertexPool,
    strip: &SurfaceStrip,
    seams: &[SideSeams],
    edge_index: i32,
    rail: i32,
    through: Vector3,
    normal: Vector3,
    corner: Vector3,
) -> Result<(i32, f64), BevelAdvisory> {
    let Some(at) = strip.try_cross(rail, through, normal) else {
        return Err(BevelAdvisory::new(
            BevelAdvisoryCode::SurfaceStripsDoNotMitre,
            vec![corner.into()],
        ));
    };

    let vertex = try_the_vertex(
        pool,
        strip,
        seams,
        edge_index,
        rail,
        strip.point_on(rail, at),
    )?;

    Ok((vertex, at))
}

/// The pool vertex one rail's crossing stands at. An OUTERMOST rail stands on a
/// tangent line the walkback already left a vertex on, and takes that one rather than minting a
/// second a rounding away from it - which would split the seam down the strip's whole length. Every
/// other rail is a point of the mitre and nothing else's, so it is minted here.
fn try_the_vertex(
    pool: &mut SurfaceVertexPool,
    strip: &SurfaceStrip,
    seams: &[SideSeams],
    edge_index: i32,
    rail: i32,
    at: Vector3,
) -> Result<i32, BevelAdvisory> {
    let did_not_meet = || {
        BevelAdvisory::new(
            BevelAdvisoryCode::SurfaceStripSeamDidNotMeet,
            vec![at.into()],
        )
    };

    if rail != 0 && rail != strip.rail_count() - 1 {
        return Ok(pool.add(at));
    }

    // Which end of that seam's run the mitre is at is which way the strip was walked, so both are
    // tried - the same two-sided reading the rails with no partner take. A missing or empty seam
    // (which the builder never hands in) is refused as the seam not meeting, which is what an
    // unmatched one is, rather than panic.
    let side = if rail == 0 { 0 } else { 1 };
    let Some(run) = seams
        .get(edge_index as usize)
        .and_then(|seam| seam[side].as_deref())
        .filter(|run| !run.is_empty())
    else {
        return Err(did_not_meet());
    };

    let mut standing = run[run.len() - 1].0;

    if (pool.position_of(standing) - at).length() > surface_strip_ends::meets_the_seam(pool) {
        standing = run[0].0;
    }

    if (pool.position_of(standing) - at).length() > surface_strip_ends::meets_the_seam(pool) {
        return Err(did_not_meet());
    }

    Ok(standing)
}

/// One strip's chords with the other's crossings laid along them, and the transition.
/// Each foreign point goes on the chord whose two rails bracket it by ANGLE, and the points on one
/// chord are ordered the way the stitch reads them - from the lower rail INDEX to the higher, which
/// is out from the shared face where the rails were counted up and back toward it where they were
/// counted down.
///
/// The transition is the one foreign point that falls PAST this strip's last mitred rail, if there
/// is one (else -1) - which there is only for the WIDER of the two beads, and is exactly where the
/// narrower one's surface ends: its own outermost rail, standing on the mitre plane and on its far
/// landing face at once. That is the point the wider strip leaves the mitre for the face beyond at,
/// so it is handed back rather than laid down here, for the mitre sector to begin its section from.
fn spread(
    strip: &SurfaceStrip,
    on_the_face: i32,
    step: i32,
    paired: i32,
    foreign: &[(f64, i32)],
) -> (Vec<Vec<i32>>, i32) {
    let mut transition = -1;

    let mut along: Vec<Option<Vec<(f64, i32)>>> =
        vec![None; (strip.rail_count() - 1).max(0) as usize];

    for &point in foreign {
        // Which of this strip's steps the angle falls after, floored.
        let mut below = ((point.0 / strip.rail_step_degrees) + ANGLE_RULER).floor() as i32;

        if below > paired - 2 {
            transition = point.1;

            continue;
        }

        below = below.max(0);

        let chord = (on_the_face + (step * below)).min(on_the_face + (step * (below + 1)));

        if chord < 0 || chord >= along.len() as i32 {
            continue;
        }

        along[chord as usize]
            .get_or_insert_with(Vec::new)
            .push(point);
    }

    let between = along
        .into_iter()
        .map(|points| {
            let Some(mut points) = points else {
                return Vec::new();
            };

            // A stable sort keeps two foreign points of one angle in the order they were laid.
            points.sort_by(|left, right| left.0.total_cmp(&right.0));

            // Counted DOWN from the shared face, the rail index runs against the angle, so the
            // chord is read back to front.
            let count = points.len();
            (0..count)
                .map(|i| points[if step > 0 { i } else { count - 1 - i }].1)
                .collect()
        })
        .collect();

    (between, transition)
}
