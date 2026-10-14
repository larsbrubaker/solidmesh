//! The end of a band that RAN OFF its landing face, closed over the face beyond: a module of free
//! functions. What is here is the end ring at a NOTCHED run-off end
//! ([`try_the_notched_ring`]), which side ran off ([`try_the_run_off_on`]) and the close itself
//! ([`try_close_over_the_face_beyond`]). The trim it reads is
//! [`SurfaceStripTrimmedByTheFaceBeyond`], the cross section comes from
//! [`super::surface_strip_tail_around_a_feature`].

use vector_math::Vector3;

use super::surface_mitre_joint_step::SideSeams;
use super::{
    surface_end_cap_around_a_feature, surface_end_cut, surface_split_sewing, surface_strip_ends,
    SurfaceEdgePlan, SurfaceStripStation,
};
use super::{
    surface_strip_tail_around_a_feature, SurfaceStrip, SurfaceStripNotchedByAFeature,
    SurfaceStripTrimmedAtTheCrossing, SurfaceStripTrimmedByTheFaceBeyond, SurfaceVertexPool,
};
use crate::bevel::{BevelAdvisory, BevelAdvisoryCode};
use crate::topology::EdgeConvexity;

/// The strip's end ring at a NOTCHED end - every rail the feature took
/// pulled back to a cross section standing CLEAR of the notch, and every other rail standing where it
/// always did, on the end plane - with where each rail meets it.
///
/// The stitch ends a strip on a ring of CHORDS, and the notch's own rim is not one: between the two
/// rails the ridge leaves through, the rim dips back to the exit, which is deeper than either rail's
/// crossing. So the strip stops short of the notch entirely, on a clean cross section standing clear
/// of it, and everything past it is sewn band by band (`SurfaceStripTailAroundAFeature`).
///
/// Only the notched rails move. An un-notched rail still reaches the end plane, which keeps the two
/// rails at the edges of the strip standing on their SEAMS' own last vertices - a point minted part
/// way along a seam would be a T junction with the walkback's face across it.
#[allow(clippy::too_many_arguments)]
pub fn try_the_notched_ring(
    pool: &mut SurfaceVertexPool,
    strip: &SurfaceStrip,
    off_the_face: &SurfaceStripTrimmedByTheFaceBeyond,
    notch: &SurfaceStripNotchedByAFeature,
    crossed_at_the_other_end: Option<&SurfaceStripTrimmedAtTheCrossing>,
    notched_at_the_other_end: Option<&SurfaceStripNotchedByAFeature>,
    on_the_seam: i32,
) -> Result<(Vec<i32>, Vec<f64>), BevelAdvisory> {
    let rails = strip.rail_count();

    let mut ring = vec![0i32; rails as usize];
    let mut meets_at = vec![0.0; rails as usize];

    let section = surface_strip_tail_around_a_feature::try_the_section_clear_of_the_notch(
        pool,
        strip,
        notch,
        notched_at_the_other_end,
        crossed_at_the_other_end,
        &off_the_face.trimmed_at,
    )?;

    for at in 0..rails {
        let rail =
            SurfaceStripTrimmedByTheFaceBeyond::rail_at(off_the_face.side, rails, at) as usize;

        if at < off_the_face.first_kept {
            // Air over the whole edge: the run-off already collapsed this rail onto the shared point,
            // and it stands there at the far parameter whatever the notch does.
            ring[rail] = off_the_face.shared;
            meets_at[rail] = off_the_face.far_param;
            continue;
        }

        meets_at[rail] = if notch.notches(rail as i32) {
            section
        } else {
            off_the_face.far_param
        };

        ring[rail] = if at == rails - 1 {
            on_the_seam
        } else {
            pool.add(strip.point_on(rail as i32, meets_at[rail]))
        };
    }

    Ok((ring, meets_at))
}

/// The one side of edge `edge_index` that ran off its landing face, where either did; refused (at
/// `corner`) where both did.
pub fn try_the_run_off_on(
    ran_off: Option<&[[Option<std::rc::Rc<SurfaceStripTrimmedByTheFaceBeyond>>; 2]]>,
    edge_index: i32,
    corner: vector_math::Vector3,
) -> Result<Option<std::rc::Rc<SurfaceStripTrimmedByTheFaceBeyond>>, crate::bevel::BevelAdvisory> {
    let left = SurfaceStripTrimmedByTheFaceBeyond::standing_on(ran_off, edge_index, 0);
    let right = SurfaceStripTrimmedByTheFaceBeyond::standing_on(ran_off, edge_index, 1);

    if left.is_some() && right.is_some() {
        return Err(crate::bevel::BevelAdvisory::new(
            crate::bevel::BevelAdvisoryCode::SurfaceStripRunsOffBothSides,
            vec![corner.into()],
        ));
    }

    Ok(left.or(right))
}

/// A band that RAN OFF its landing face closed over the face
/// beyond (`off_the_face`) - the end ring (cut back clear of a standing feature's `notch`, where one
/// stands at this end, and capped and sewn around it), the run-off reading checked against the
/// edge's own end, the section cut out of the face beyond ([`surface_end_cut::try_cut`], with the
/// corner beyond as its die point), and the shared point sewn into whatever stands on it. Returns
/// the station the stitch sews to. `seams` is the `[edge, side]` table of seam runs indexed by
/// `edge_index`; a missing or empty run there is refused as an end not supported rather than a
/// panic.
#[allow(clippy::too_many_arguments)]
pub fn try_close_over_the_face_beyond(
    pool: &mut SurfaceVertexPool,
    faces: &mut Vec<(i32, i32, i32)>,
    strip: &SurfaceStrip,
    edge: &SurfaceEdgePlan,
    seams: &[SideSeams],
    edge_index: i32,
    corner: Vector3,
    at_the_start: bool,
    runs_out_facing: Vector3,
    off_the_face: &SurfaceStripTrimmedByTheFaceBeyond,
    notch: Option<&SurfaceStripNotchedByAFeature>,
    crossed_at_the_other_end: Option<&SurfaceStripTrimmedAtTheCrossing>,
    notched_at_the_other_end: Option<&SurfaceStripNotchedByAFeature>,
) -> Result<SurfaceStripStation, BevelAdvisory> {
    let not_supported = || {
        BevelAdvisory::new(
            BevelAdvisoryCode::SurfaceStripEndNotSupported,
            vec![corner.into()],
        )
    };
    let other_side = if off_the_face.side == 0 { 1 } else { 0 };
    let on_the_seam = seams
        .get(edge_index as usize)
        .and_then(|sides| sides[other_side].as_ref())
        .and_then(|run| {
            if at_the_start {
                run.first()
            } else {
                run.last()
            }
        })
        .map(|&(vertex, _)| vertex);
    let Some(on_the_seam) = on_the_seam else {
        return Err(not_supported());
    };

    if notch.is_some() && at_the_start == off_the_face.at_the_start {
        return Err(BevelAdvisory::new(
            BevelAdvisoryCode::SurfaceStandingFeatureDoesNotExtend,
            vec![corner.into()],
        ));
    }

    let (ring, meets_at) = match notch {
        Some(notch) => try_the_notched_ring(
            pool,
            strip,
            off_the_face,
            notch,
            crossed_at_the_other_end,
            notched_at_the_other_end,
            on_the_seam,
        )?,
        None => off_the_face.ring_at(pool, strip, at_the_start, on_the_seam),
    };

    let profile = off_the_face.profile_of(&ring);
    if profile.len() < 3 {
        return Err(not_supported());
    }

    let into_the_chain = if at_the_start {
        edge.edge.direction()
    } else {
        -edge.edge.direction()
    };
    let ends_on = if off_the_face.at_the_start {
        edge.to()
    } else {
        edge.from()
    };
    let along_the_edge = edge.edge.direction();

    for on_the_side in [off_the_face.the_corner_beyond, off_the_face.shared] {
        if (pool.position_of(on_the_side) - ends_on)
            .dot(along_the_edge)
            .abs()
            > surface_strip_ends::meets_the_seam(pool)
        {
            return Err(BevelAdvisory::new(
                BevelAdvisoryCode::SurfaceRunOffReadingDisagrees,
                vec![pool.position_of(on_the_side).into()],
            ));
        }
    }

    if let Some(notch) = notch {
        let between_the_rails = surface_end_cap_around_a_feature::try_cap(
            pool,
            faces,
            &off_the_face.walk_of_the_rim(strip.rail_count()),
            off_the_face.the_corner_beyond,
            notch,
            &ring,
            corner,
            runs_out_facing,
        )?;
        surface_strip_tail_around_a_feature::try_sew(pool, faces, strip, notch, &ring)?;

        surface_split_sewing::sew_in_the_splits_unchecked(pool, faces, &[off_the_face.shared]);

        return Ok(SurfaceStripStation::new(ring, meets_at, between_the_rails));
    }

    let along_the_profile = surface_end_cut::try_cut(
        pool,
        faces,
        &profile,
        corner,
        edge.convexity() == EdgeConvexity::Concave,
        into_the_chain,
        runs_out_facing,
        -1,
        off_the_face.the_corner_beyond,
    )?;

    if let Some(&first) = along_the_profile[0].first() {
        return Err(BevelAdvisory::new(
            BevelAdvisoryCode::SurfaceStripSeamDidNotMeet,
            vec![pool.position_of(first).into()],
        ));
    }

    let rails = strip.rail_count();
    let mut between = Vec::with_capacity((rails - 1).max(0) as usize);
    for gap in 0..(rails - 1).max(0) {
        let chord = if off_the_face.side == 0 {
            gap + 2 - off_the_face.first_kept
        } else {
            rails - 2 - gap + 2 - off_the_face.first_kept
        };

        let mut found = if chord >= 1 && (chord as usize) < along_the_profile.len() {
            along_the_profile[chord as usize].clone()
        } else {
            Vec::new()
        };

        if off_the_face.side != 0 && found.len() > 1 {
            found.reverse();
        }

        between.push(found);
    }

    surface_split_sewing::sew_in_the_splits_unchecked(pool, faces, &[off_the_face.shared]);

    Ok(SurfaceStripStation::new(ring, meets_at, between))
}
