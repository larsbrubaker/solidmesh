//! The last pass of the build, where the strips are sewn
//! (`Bevel/Surface/SurfaceBuilderBittenStrips.cs`, C# `internal static class`; free functions
//! here): the rows of every hole a standing feature bit out of a strip
//! ([`SurfaceStripBiteRows`]), then the features those holes were taken out of
//! ([`surface_bitten_feature_faces`]), then the stitches ([`surface_bevel_stitch`]) - three passes
//! where there used to be one, because a feature is emitted once and a feature can be bitten by two
//! strips.
//!
//! Split out of the builder ([`super::surface_bevel_builder`]), which was over the file-length
//! limit. The order is the whole point: every hole's rows have to be standing before any feature
//! runs out to them, and the stitch is told nothing about the holes at all - a bitten strip is
//! handed to it twice instead. Nothing here is new.

use std::rc::Rc;

use super::surface_mitre_joint_step::SideSeams;
use super::surface_strip_bite_rows::SurfaceStripBiteRows;
use super::{surface_bevel_builder, surface_bevel_stitch, surface_bitten_feature_faces};
use super::{
    SurfaceBevelPlan, SurfaceStrip, SurfaceStripBittenByAFeature, SurfaceStripStation,
    SurfaceVertexPool,
};
use crate::bevel::{BevelAdvisory, BevelAdvisoryCode};

/// One strip's start or end ring. Every built strip has both by the time the stitch runs - the
/// mitres, the corners and the ends close every one - so a missing ring is a contradiction; C#
/// would pass the null on and throw inside the stitch. Refused here as the end ring that does not
/// land on the seam (divergence 5070).
fn ring_of<'a>(
    rings: &'a [Option<SurfaceStripStation>],
    strip: &SurfaceStrip,
    index: usize,
) -> Result<&'a SurfaceStripStation, BevelAdvisory> {
    rings
        .get(index)
        .and_then(|ring| ring.as_ref())
        .ok_or_else(|| {
            BevelAdvisory::new(
                BevelAdvisoryCode::SurfaceStripSeamDidNotMeet,
                vec![strip.point_on(0, 0.0).into()],
            )
        })
}

/// C# `TryLayThemAndStitch`: the rows, then the features, then the stitches, over every strip the
/// pass built.
#[allow(clippy::too_many_arguments)]
pub fn try_lay_them_and_stitch(
    pool: &mut SurfaceVertexPool,
    faces: &mut Vec<(i32, i32, i32)>,
    plans: &[Rc<SurfaceBevelPlan>],
    first_of_plan: &[i32],
    strips: &[Option<Rc<SurfaceStrip>>],
    bitten: &[Option<Rc<SurfaceStripBittenByAFeature>>],
    seams: &[SideSeams],
    start_rings: &[Option<SurfaceStripStation>],
    end_rings: &[Option<SurfaceStripStation>],
    later_splits: &[i32],
    count: i32,
) -> Result<(), BevelAdvisory> {
    let count = count.max(0) as usize;

    // THE ROWS FIRST, THEN THE FEATURES, THEN THE STITCHES - three passes where there used to be
    // one, because a FEATURE is emitted once and a feature can be bitten by two strips. Its faces
    // run out to every hole taken out of it, so every one of those holes' rows has to be standing
    // before any of them is drawn (SurfaceBittenFeatureFaces).
    let mut rows_of: Vec<Option<SurfaceStripBiteRows>> = vec![None; count];

    for i in 0..count {
        if surface_bevel_builder::is_swallowed(plans, first_of_plan, i as i32) {
            continue;
        }

        let (Some(bite), Some(strip)) = (&bitten[i], &strips[i]) else {
            continue;
        };

        let start_ring = ring_of(start_rings, strip, i)?;
        let end_ring = ring_of(end_rings, strip, i)?;

        rows_of[i] = Some(SurfaceStripBiteRows::try_build(
            pool, faces, strip, bite, start_ring, end_ring,
        )?);
    }

    // C#'s `HashSet<SurfaceStandingFeatureWithheld>` of reference identities.
    let mut already_emitted = Vec::new();

    for i in 0..count {
        let (Some(_), Some(bite)) = (&rows_of[i], &bitten[i]) else {
            continue;
        };

        if already_emitted
            .iter()
            .any(|emitted| Rc::ptr_eq(emitted, &bite.feature))
        {
            continue;
        }

        already_emitted.push(bite.feature.clone());

        // Every hole this ONE feature was bitten by, with the rows each of them laid.
        let mut its_bites = Vec::new();

        for other in i..count {
            if let (Some(rows), Some(other_bite)) = (&rows_of[other], &bitten[other]) {
                if Rc::ptr_eq(&other_bite.feature, &bite.feature) {
                    its_bites.push((other_bite.as_ref(), rows));
                }
            }
        }

        surface_bitten_feature_faces::try_emit(pool, faces, &bite.feature, &its_bites)?;
    }

    for i in 0..count {
        if surface_bevel_builder::is_swallowed(plans, first_of_plan, i as i32) {
            continue;
        }

        let Some(strip) = &strips[i] else {
            continue;
        };

        let start_ring = ring_of(start_rings, strip, i)?;
        let end_ring = ring_of(end_rings, strip, i)?;

        if let (Some(bite), Some(rows)) = (&bitten[i], &rows_of[i]) {
            try_the_bitten_strip(
                pool,
                faces,
                later_splits,
                strip,
                bite,
                rows,
                seams,
                i as i32,
                start_ring,
                end_ring,
            )?;

            continue;
        }

        surface_bevel_stitch::try_stitch(
            pool,
            faces,
            later_splits,
            strip,
            seams,
            i as i32,
            start_ring,
            end_ring,
        )?;
    }

    Ok(())
}

/// C# `TryTheBittenStrip`: one strip a feature took a bite out of, once its rows are laid and its
/// feature emitted - the strip either side of the hole stitched as two.
///
/// The stitch is told nothing about the hole. It sews between two rings, so the strip is handed to
/// it twice - start to the section before the bite, and the section after it to the end - each with
/// only the part of the two seams inside its own span ([`SurfaceStripBiteRows::try_the_seams`]).
#[allow(clippy::too_many_arguments)]
fn try_the_bitten_strip(
    pool: &mut SurfaceVertexPool,
    faces: &mut Vec<(i32, i32, i32)>,
    later_splits: &[i32],
    strip: &SurfaceStrip,
    bite: &SurfaceStripBittenByAFeature,
    rows: &SurfaceStripBiteRows,
    seams: &[SideSeams],
    edge_index: i32,
    start_ring: &SurfaceStripStation,
    end_ring: &SurfaceStripStation,
) -> Result<(), BevelAdvisory> {
    let up_to_the_bite = SurfaceStripBiteRows::try_the_seams(
        pool,
        seams,
        edge_index,
        bite,
        strip.rail_count(),
        &rows.before,
        true,
    )?;
    let past_it = SurfaceStripBiteRows::try_the_seams(
        pool,
        seams,
        edge_index,
        bite,
        strip.rail_count(),
        &rows.after,
        false,
    )?;

    surface_bevel_stitch::try_stitch(
        pool,
        faces,
        later_splits,
        strip,
        &up_to_the_bite,
        edge_index,
        start_ring,
        &rows.before,
    )?;
    surface_bevel_stitch::try_stitch(
        pool,
        faces,
        later_splits,
        strip,
        &past_it,
        edge_index,
        &rows.after,
        end_ring,
    )
}
