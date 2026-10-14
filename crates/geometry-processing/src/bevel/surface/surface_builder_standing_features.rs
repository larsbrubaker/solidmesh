//! What a feature STANDING inside a band's footprint takes out of the strip that band ruled
//! (free functions): a closed
//! hole where the feature stands part way along it ([`SurfaceStripBittenByAFeature`]), an open notch
//! where it stands at an end ([`SurfaceStripNotchedByAFeature`]), and the feature's own faces carried
//! out over the corner the band took in the notched case ([`surface_standing_feature_faces`]).
//!
//! Split out of `SurfaceBevelBuilder`, which was over the file-length limit. This is one step of the
//! pass - read after the seams are picked and the run-off trims stand, and before the ends are
//! closed - and it is read ONCE PER REACH rather than once per feature, because a feature two bands of two
//! different chains undercut is held back as one record with two reaches. Nothing here is new.

use std::rc::Rc;

use super::surface_mitre_joint_step::SideSeams;
use super::surface_standing_feature_faces;
use super::{
    SurfaceBevelPlan, SurfaceEdgePlan, SurfaceMixedJunction, SurfaceStrip,
    SurfaceStripBittenByAFeature, SurfaceStripNotchedByAFeature, SurfaceStripNotches,
    SurfaceStripTrimmedByTheFaceBeyond, SurfaceVertexPool, SurfaceWalkbackResult,
};
use crate::bevel::{BevelAdvisory, BevelAdvisoryCode};

/// What [`try_read`] reads: per strip and per end, the open notch a feature takes there, and per
/// strip, the one closed hole a feature takes out of its middle.
pub type SurfaceStandingFeatureReading = (
    SurfaceStripNotches,
    Vec<Option<Rc<SurfaceStripBittenByAFeature>>>,
);

/// Every withheld feature read against the strip its band reached, into the bite or the
/// notch it takes there, or refused by name where the two readings cannot both stand.
///
/// The tables are the ones the builder hands its other steps: `strips` one per build edge (`None`
/// where a blend swallowed it), `seams` the `[edge][side]` runs (a bitten strip's seam on the
/// bitten side is replaced here), and `ran_off` the `[edge][side]` run-off trims.
#[allow(clippy::too_many_arguments)]
pub fn try_read(
    pool: &mut SurfaceVertexPool,
    faces: &mut Vec<(i32, i32, i32)>,
    walked: &SurfaceWalkbackResult,
    plans: &[Rc<SurfaceBevelPlan>],
    junctions: Option<&[Rc<SurfaceMixedJunction>]>,
    edges: &[SurfaceEdgePlan],
    first_of_plan: &[i32],
    strips: &[Option<Rc<SurfaceStrip>>],
    seams: &mut [SideSeams],
    ran_off: &[[Option<Rc<SurfaceStripTrimmedByTheFaceBeyond>>; 2]],
    count: i32,
) -> Result<SurfaceStandingFeatureReading, BevelAdvisory> {
    // And where a feature STANDING inside a band's footprint takes a notch out of the strip: the
    // walkback held that feature's faces back - whole coplanar regions - and each is carried out
    // over the corner the band took until it meets the fillet (SurfaceStripNotchedByAFeature). Read
    // here, with the seams picked and the run-off trims standing, because the notch has to know
    // which rails the run-off already collapsed - and before the ends are closed, which is what the
    // notch changes.
    let mut notched: SurfaceStripNotches = vec![[None, None]; count.max(0) as usize];

    // And where the feature stands part way ALONG the band instead, with no end of the strip to run
    // off through: what it takes is a closed hole in the middle of the strip rather than a notch open
    // at an end (SurfaceStripBittenByAFeature). Read FIRST, and it hands an end case straight back -
    // the two are told apart by the one question of whether a crease of the feature ends in one of
    // the band's end planes, which the notch reader owns and this one asks of it. The hole itself is
    // not laid until the ends are closed, because its rows stand between the strip's own two rings.
    let mut bitten: Vec<Option<Rc<SurfaceStripBittenByAFeature>>> =
        vec![None; count.max(0) as usize];

    // ONCE PER REACH, not once per feature. A feature two bands of two different chains undercut is
    // held back as ONE record with two reaches (SurfaceFaceWalkback), and each reach takes its own
    // hole out of its own strip - so the reading below is asked of each in turn, against that band's
    // own run of crease.
    for feature in &walked.standing_features {
        let mut reach = 0;

        while reach < feature.reaches.borrow().len() {
            let (strip, on_the_left) = feature.reaches.borrow()[reach];

            let Some(strip_ruled) = (strip >= 0 && strip < count)
                .then(|| strips.get(strip as usize).and_then(|ruled| ruled.as_ref()))
                .flatten()
            else {
                reach += 1;
                continue;
            };

            let s = strip as usize;
            let runs_on_its_side = walked.tangent_runs.get(&(strip, on_the_left));
            let ran_off_here = [ran_off[s][0].as_deref(), ran_off[s][1].as_deref()];

            let bite = SurfaceStripBittenByAFeature::try_read(
                pool,
                strip_ruled,
                &edges[s],
                feature,
                reach as i32,
                &ran_off_here,
                runs_on_its_side.map(|runs| runs.as_slice()),
            )?;

            if let Some(bite) = bite {
                if bitten[s].is_some() {
                    // Two features biting the SAME strip. Each wants the strip stopped and resumed at
                    // its own hole, and the two sections cannot be four; named rather than built on
                    // whichever was read first.
                    return Err(BevelAdvisory::new(
                        BevelAdvisoryCode::SurfaceBiteDoesNotRead,
                        vec![pool.position_of(bite.tip.vertex).into()],
                    ));
                }

                // The seam with the feature's break taken into it: the walkback left the tangent line
                // on that side in two runs, one either side of the hole, and the pick above took
                // whichever of them overlapped the edge most. Joined here so that nothing downstream
                // sees half a seam - the two stitches are each handed only the part inside their own
                // span.
                //
                // Overwritten mid loop, which makes a strip that is BOTH bitten and notched depend on
                // the order the walkback lists its features in: a notch read first would be read
                // against the picked half of this seam and a notch read second against the joined
                // whole. Two BITES on one strip are named above; one of each is not, because nothing
                // reaches it - "Bevel to Center Feature" is the fixture to watch, and the reading that
                // lets two extensions share one strip is where this has to be settled rather than
                // patched here.
                seams[s][if on_the_left { 0 } else { 1 }] = Some(bite.seam.clone());
                bitten[s] = Some(Rc::new(bite));

                reach += 1;
                continue;
            }

            let notch = SurfaceStripNotchedByAFeature::try_read(
                pool,
                strip_ruled,
                &edges[s],
                strip,
                feature,
                reach as i32,
                &ran_off_here,
                Some(seams),
            )?;

            let Some(notch) = notch else {
                reach += 1;
                continue;
            };

            if feature.reaches.borrow().len() > 1 {
                // A feature TWO bands reached, one of whose reaches runs off through an END of its
                // strip. A bite is a closed hole and each reach can take its own out of its own strip
                // without the two ever meeting; a NOTCH is open at the end, and the extension it draws
                // runs out to the strip's own end ring - so two of them on one feature would each carry
                // the same faces out to a different rim, and the second would replace the side of an
                // outline the first had already replaced. Named rather than built on whichever was read
                // first, in the same words the stretch reader uses for two bands of one region.
                let from = feature.faces[0].creases.borrow()[reach].0;

                return Err(BevelAdvisory::new(
                    BevelAdvisoryCode::SurfaceStandingFeatureUnderTwoCuts,
                    vec![pool.position_of(from).into()],
                ));
            }

            if !the_notched_end_can_be_closed(
                plans,
                junctions,
                edges,
                first_of_plan,
                strip,
                notch.at_the_start,
            ) {
                // The end the extension runs off through is a MITRE, a junction or a closed chain's
                // own turn - none of which knows to give the extension room, and all of which would
                // close on a ring drawn as though the strip still reached its corner. Named HERE,
                // before the feature's faces go in: emitted first and refused afterwards, the pass
                // would leave the extension standing in a mesh nothing else was cut for.
                return Err(BevelAdvisory::new(
                    BevelAdvisoryCode::SurfaceStandingFeatureDoesNotExtend,
                    vec![if notch.at_the_start {
                        edges[s].from()
                    } else {
                        edges[s].to()
                    }
                    .into()],
                ));
            }

            let notch = Rc::new(notch);

            notched[s][if notch.at_the_start { 0 } else { 1 }] = Some(notch.clone());

            // The feature itself, carried out over the corner the band took until it meets the
            // strip. Emitted here, with the notch just read, so that the end cap below closes on the
            // very vertices the extension is bounded by (SurfaceStandingFeatureFaces).
            surface_standing_feature_faces::try_emit(pool, faces, strip_ruled, &notch)?;

            reach += 1;
        }
    }

    Ok((notched, bitten))
}

/// Whether the end a notch runs off through is one the pass closes
/// ALONE - a free end of an open chain, with no junction standing at it and no other chain ending
/// there - which is the only end the split cap and the tail are drawn for.
fn the_notched_end_can_be_closed(
    plans: &[Rc<SurfaceBevelPlan>],
    junctions: Option<&[Rc<SurfaceMixedJunction>]>,
    edges: &[SurfaceEdgePlan],
    first_of_plan: &[i32],
    strip: i32,
    at_the_start: bool,
) -> bool {
    for plan in 0..plans.len() {
        let first = first_of_plan[plan];
        let last = first + plans[plan].edges.len() as i32 - 1;

        if strip < first || strip > last {
            continue;
        }

        if plans[plan].is_closed || strip != if at_the_start { first } else { last } {
            return false;
        }

        let corner = if at_the_start {
            edges[first as usize].edge.from_vertex()
        } else {
            edges[last as usize].edge.to_vertex()
        };

        for junction in junctions.unwrap_or(&[]) {
            if junction.vertex == corner {
                return false;
            }
        }

        // And ALONE means alone. Three chain ends gathering at one vertex are closed as a corner
        // patch (SurfaceCornerPatch), which draws the whole ball's octant over all three of them and
        // knows nothing of a notch - so the extension would go in and the patch would cap straight
        // over it. Counted the same way the closing pass gathers, by the mesh's own vertex index,
        // which is exact on a welded mesh.
        if ends_gathering_at(plans, edges, first_of_plan, corner) != 1 {
            return false;
        }

        return true;
    }

    false
}

/// How many free ends of open chains stand at one vertex of the part - one for
/// an end the pass closes on its own, three for a corner.
fn ends_gathering_at(
    plans: &[Rc<SurfaceBevelPlan>],
    edges: &[SurfaceEdgePlan],
    first_of_plan: &[i32],
    vertex: i32,
) -> i32 {
    let mut gathered = 0;

    for plan in 0..plans.len() {
        if plans[plan].is_closed {
            continue;
        }

        let first = first_of_plan[plan];
        let last = first + plans[plan].edges.len() as i32 - 1;

        if edges[first as usize].edge.from_vertex() == vertex {
            gathered += 1;
        }

        if edges[last as usize].edge.to_vertex() == vertex {
            gathered += 1;
        }
    }

    gathered
}
