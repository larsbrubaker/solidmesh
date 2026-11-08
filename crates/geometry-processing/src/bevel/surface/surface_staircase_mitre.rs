//! The two rings a CONSUMED chord and its two neighbours meet on. The chord is read by
//! [`SurfaceMitreConsumesTheRail`]; each ring here is a staircase: the rails the mitre did not eat
//! end on the real corner as any mitre's do
//! ([`surface_strip_mitre_rails::try_one_rail_of_the_mitre`]), and the rails it ate end on the
//! VIRTUAL corner the two neighbours' lines meet at, which both rings share - so the consumed side
//! of the chord's own strip stands still between its two ends and collapses to nothing, and the
//! step between the last consumed rail and the first that is not closes the surface with a
//! triangle.
//!
//! Nothing is emitted here. The rings are handed to [`super::surface_bevel_stitch`] exactly as an
//! ordinary mitre's are, and the stitch already draws what a station standing still asks for: a
//! quad column whose two ends are the same two vertices is a ring with no area and is dropped, and
//! the column across the step has three corners rather than four. The step's triangle stands in for
//! the true triple point - the rail where the two mitres and the virtual one meet at once, which is
//! BETWEEN two rails and would be a sample of the profile the strip does not have.
//!
//! Every vertex is minted once and shared by index: the virtual corner's rail 0 (or last) is the
//! walkback's OWN vertex, taken out of the neighbours' seams, and the consumed rails between are
//! added to the pool here and read by all four strip ends. The builder
//! ([`super::surface_bevel_builder`]) calls [`try_build`] once per consumed chord.

use std::rc::Rc;

use solidmesh_math::Vector3;

use super::surface_mitre_joint_step::SideSeams;
use super::surface_strip_ends;
use super::surface_strip_mitre_rails;
use super::{SurfaceMitreConsumesTheRail, SurfaceStrip, SurfaceStripStation, SurfaceVertexPool};
use crate::bevel::{BevelAdvisory, BevelAdvisoryCode};

/// The strip at `index`, refused as the mitre that does not meet where it is missing: a guard,
/// since the consumed-chord survey reads only chords whose two neighbours have strips.
fn strip_at<'a>(
    strips: &'a [Option<Rc<SurfaceStrip>>],
    index: i32,
    reading: &SurfaceMitreConsumesTheRail,
) -> Result<&'a SurfaceStrip, BevelAdvisory> {
    strips
        .get(index as usize)
        .and_then(|strip| strip.as_deref())
        .ok_or_else(|| {
            BevelAdvisory::new(
                BevelAdvisoryCode::SurfaceStripsDoNotMitre,
                vec![reading.virtual_corner.into()],
            )
        })
}

/// The four strip ends around one consumed chord - the previous strip's end, both of the consumed
/// strip's, and the next strip's start.
pub fn try_build(
    pool: &mut SurfaceVertexPool,
    strips: &[Option<Rc<SurfaceStrip>>],
    seams: &[SideSeams],
    reading: &SurfaceMitreConsumesTheRail,
    start_rings: &mut [Option<SurfaceStripStation>],
    end_rings: &mut [Option<SurfaceStripStation>],
) -> Result<(), BevelAdvisory> {
    let previous = strip_at(strips, reading.previous_index, reading)?;
    let here = strip_at(strips, reading.edge_index, reading)?;
    let next = strip_at(strips, reading.next_index, reading)?;

    let rails = here.rail_count();
    let count = rails.max(0) as usize;

    let mut before = vec![0; count];
    let mut after = vec![0; count];
    let mut previous_ends = vec![0.0; count];
    let mut here_starts = vec![0.0; count];
    let mut here_ends = vec![0.0; count];
    let mut next_starts = vec![0.0; count];

    let before_plane = (previous.along() + here.along()).get_normal();
    let after_plane = (here.along() + next.along()).get_normal();

    for rail in 0..rails {
        let r = rail as usize;

        if reading.consumes_rail(rail, rails) {
            let (vertex, previous_end, next_start) =
                try_the_virtual_rail(pool, strips, seams, reading, rail)?;
            before[r] = vertex;
            previous_ends[r] = previous_end;
            next_starts[r] = next_start;

            after[r] = before[r];

            // The consumed strip meets this vertex nowhere on its own rail, so it is asked only
            // where the vertex stands ALONG its edge, and told the same at both of its ends: that
            // is what makes the column between two consumed rails stand still and cover nothing.
            here_starts[r] = here.param_of(pool.position_of(before[r]));
            here_ends[r] = here_starts[r];

            continue;
        }

        // The plane point is the corner itself at both of these, which is what a staircase's two
        // real mitres have always cut on. Every chord a mitre can consume stands between two edges
        // of one rim turning on one face at one wedge, so the two setbacks there agree and the
        // crossing of the tangent lines IS the corner
        // (surface_strip_mitre_rails::try_the_seam_point). Told apart here only if a consumed chord
        // is ever found between two edges of different wedges.
        let (vertex, previous_end, here_start) =
            surface_strip_mitre_rails::try_one_rail_of_the_mitre(
                pool,
                previous,
                here,
                seams,
                reading.previous_index,
                reading.edge_index,
                rail,
                reading.corner_before,
                reading.corner_before,
                before_plane,
            )?;
        before[r] = vertex;
        previous_ends[r] = previous_end;
        here_starts[r] = here_start;

        let (vertex, here_end, next_start) = surface_strip_mitre_rails::try_one_rail_of_the_mitre(
            pool,
            here,
            next,
            seams,
            reading.edge_index,
            reading.next_index,
            rail,
            reading.corner_after,
            reading.corner_after,
            after_plane,
        )?;
        after[r] = vertex;
        here_ends[r] = here_end;
        next_starts[r] = next_start;
    }

    end_rings[reading.previous_index as usize] =
        Some(SurfaceStripStation::clean(before.clone(), previous_ends));
    start_rings[reading.edge_index as usize] =
        Some(SurfaceStripStation::clean(before, here_starts));
    end_rings[reading.edge_index as usize] =
        Some(SurfaceStripStation::clean(after.clone(), here_ends));
    start_rings[reading.next_index as usize] = Some(SurfaceStripStation::clean(after, next_starts));

    Ok(())
}

/// One consumed rail - where the two NEIGHBOURS' rails cross the mitre plane through the virtual
/// corner, as the one vertex both of them end on and the parameter each meets it at (`(vertex,
/// previous_ends, next_starts)`). The two are checked to arrive at one point, because a staircase
/// that stepped between two points a rounding apart would be a crack along the whole of the
/// consumed side.
fn try_the_virtual_rail(
    pool: &mut SurfaceVertexPool,
    strips: &[Option<Rc<SurfaceStrip>>],
    seams: &[SideSeams],
    reading: &SurfaceMitreConsumesTheRail,
    rail: i32,
) -> Result<(i32, f64, f64), BevelAdvisory> {
    let previous = strip_at(strips, reading.previous_index, reading)?;
    let next = strip_at(strips, reading.next_index, reading)?;

    let does_not_mitre = |at: Vector3| {
        BevelAdvisory::new(BevelAdvisoryCode::SurfaceStripsDoNotMitre, vec![at.into()])
    };

    let (Some(previous_ends), Some(next_starts)) = (
        previous.try_cross(rail, reading.virtual_corner, reading.virtual_normal),
        next.try_cross(rail, reading.virtual_corner, reading.virtual_normal),
    ) else {
        return Err(does_not_mitre(reading.virtual_corner));
    };

    let at = previous.point_on(rail, previous_ends);

    if (at - next.point_on(rail, next_starts)).length() > surface_strip_ends::meets_the_seam(pool) {
        return Err(does_not_mitre(at));
    }

    if rail == 0 || rail == previous.rail_count() - 1 {
        // The rail ON the seam is the walkback's own vertex - the corner its offset boundary
        // already turned at when it skipped this chord's tangent line - so it is taken by index,
        // never made.
        let vertex =
            SurfaceMitreConsumesTheRail::try_the_shared_corner(pool, strips, seams, reading)?;

        return Ok((vertex, previous_ends, next_starts));
    }

    let vertex = pool.add(at);
    Ok((vertex, previous_ends, next_starts))
}
