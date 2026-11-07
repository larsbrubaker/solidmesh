//! The rows of strip that stand ACROSS a bite ([`SurfaceStripBiteRows`]): the two cross sections
//! the strip stops and resumes at,
//! and the surface between them, which is the whole width of the strip less the hole itself. Built
//! from a [`SurfaceStripBittenByAFeature`] reading; the builder stitches the strip in two sections
//! between the rows this hands back.
//!
//! **Why the strip stops at all.** `SurfaceBevelStitch` sews a strip as a run of cross sections,
//! and every one of them is a ring of chords right across the profile. A bite is a hole in the
//! middle of that, so no chord over it says where the surface goes - the same reason a NOTCH has the
//! strip stop short and its tail sewn band by band (`SurfaceStripTailAroundAFeature`). Here the strip
//! stops twice and resumes twice, and what stands between is laid out rail pair by rail pair.
//!
//! **Where it stops is the hole's own widest place, and nothing is chosen.** The bite is widest at
//! the seam it opens onto and narrows inward to its tip, so the seam rail's two crossings ARE the
//! extreme parameters of the whole hole: the section before is the one through the near crossing and
//! the section after the one through the far. That buys two things at once - the two rows need no
//! clearance picked out of the air, and the seam rail's own row points are the walkback's two
//! vertices rather than a pair minted beside them, so the strip meets the landing face on the very
//! points it is already sewn to.
//!
//! **Three shapes between the rows.** A rail pair the hole does not reach is one ordinary column
//! ring across. A pair it passes clean through is two rings, one either side of it. And the ONE pair
//! its tip stands in is a single ring with the tip as an extra corner - the hole notched into its
//! side - which is the pentagon the notch's tail draws, one corner longer.

use super::surface_mitre_joint_step::SideSeams;
use super::{
    surface_bevel_stitch, surface_fan, surface_ring, surface_split_sewing, SurfaceStrip,
    SurfaceStripBittenByAFeature, SurfaceStripStation, SurfaceVertexPool,
};
use crate::bevel::{BevelAdvisory, BevelAdvisoryCode};

/// The rows across one bite: the two sections the strip stops and resumes at.
#[derive(Debug, Clone)]
pub struct SurfaceStripBiteRows {
    /// The cross section the strip stops at, which is the section the stitch ends on.
    pub before: SurfaceStripStation,
    /// The one it resumes at, which is the section the second stitch starts from.
    pub after: SurfaceStripStation,
    /// Where the hole starts on each rail, as a vertex, or -1 on a rail it does not reach. The seam
    /// rail's is the walkback's own vertex and so is shared with the landing face by identity.
    pub near_at: Vec<i32>,
    /// Where it ends again on each rail, the same way.
    pub far_at: Vec<i32>,
}

/// The advisory every refusal here is named with.
fn does_not_read(at: vector_math::Vector3) -> BevelAdvisory {
    BevelAdvisory::new(BevelAdvisoryCode::SurfaceBiteDoesNotRead, vec![at.into()])
}

impl SurfaceStripBiteRows {
    /// Lays the rows across one bite and hands back the two sections the strip is
    /// stitched between. `start_ring` is the ring the strip starts on, which the section before has
    /// to stand clear of; `end_ring` the ring it ends on, which the section after has to stand clear
    /// of.
    pub fn try_build(
        pool: &mut SurfaceVertexPool,
        faces: &mut Vec<(i32, i32, i32)>,
        strip: &SurfaceStrip,
        bite: &SurfaceStripBittenByAFeature,
        start_ring: &SurfaceStripStation,
        end_ring: &SurfaceStripStation,
    ) -> Result<SurfaceStripBiteRows, BevelAdvisory> {
        let rails = strip.rail_count();
        let near = pool.near_enough();

        // The two rows stand where the WALKBACK broke the tangent line, not where the feature's
        // planes say it should have. The reading has already checked the two agree, but it allows
        // them the pool's snap - ten welds - and everything from here on measures in welds: the rows
        // put the walkback's own vertices on the seam rail, and the slices below cut at one weld.
        // Taken from the planes, a break three welds short would read fine and then refuse at the
        // slice, the row's vertex and the row's parameter naming two different places.
        let before_param = bite.near_seam_param;
        let after_param = bite.far_seam_param;

        let rail_slots = rails.max(0) as usize;
        let mut near_at = vec![-1; rail_slots];
        let mut far_at = vec![-1; rail_slots];

        for step in 0..bite.rail_of.len() {
            if bite.near_param_of[step] < before_param - near
                || bite.far_param_of[step] > after_param + near
            {
                // The hole reaches further along the edge on some inner rail than it does at the
                // seam, so the seam's own crossings are not its widest place and the two rows would
                // cut through it. Named rather than sewn through the middle of the hole.
                return Err(does_not_read(
                    strip.point_on(bite.rail_of[step], bite.near_param_of[step]),
                ));
            }
        }

        for rail in 0..rails {
            if start_ring.param[rail as usize] > before_param - near
                || end_ring.param[rail as usize] < after_param + near
            {
                // One of the strip's own ends reaches into the hole, so there is no strip left either
                // side of it to stitch and the end cut and the bite would both claim the same rail.
                return Err(does_not_read(strip.point_on(rail, before_param)));
            }
        }

        let seam_rail = bite.seam_rail();
        let mut before_rail = vec![0; rail_slots];
        let mut after_rail = vec![0; rail_slots];
        let mut before_on = vec![0.0; rail_slots];
        let mut after_on = vec![0.0; rail_slots];

        for rail in 0..rails {
            let r = rail as usize;
            before_on[r] = before_param;
            after_on[r] = after_param;

            before_rail[r] = if rail == seam_rail {
                bite.near_seam_vertex
            } else {
                pool.add(strip.point_on(rail, before_param))
            };
            after_rail[r] = if rail == seam_rail {
                bite.far_seam_vertex
            } else {
                pool.add(strip.point_on(rail, after_param))
            };
        }

        // The hole's own rim, rail by rail. The seam rail's two points are the row's own - the hole
        // is widest there, so its rim meets the two sections exactly at the seam - and every other
        // covered rail carries a point of its own between them.
        for step in 0..bite.rail_of.len() {
            let rail = bite.rail_of[step];
            let r = rail as usize;

            near_at[r] = if rail == seam_rail {
                before_rail[r]
            } else {
                pool.add(strip.point_on(rail, bite.near_param_of[step]))
            };

            far_at[r] = if rail == seam_rail {
                after_rail[r]
            } else {
                pool.add(strip.point_on(rail, bite.far_param_of[step]))
            };
        }

        let before = SurfaceStripStation::clean(before_rail.clone(), before_on);
        let after = SurfaceStripStation::clean(after_rail.clone(), after_on);

        // The OTHER seam's rail is broken here too - the strip stops and resumes right across its
        // width - and the landing face on that side knows nothing about it: the walkback left its
        // tangent line whole there, because nothing stands on that face. So the two points the rows
        // put on that rail are sewn into whatever already spans them, which is the face's own rim
        // edge. Without it the strip meets that face at two T junctions and the shell does not close.
        //
        // BEFORE the rows go in, and that order is the point: this conditions what is already
        // standing so that what is emitted next meets it, which is how SurfaceEndCut uses the same
        // sewing. Run afterwards it would be asked to rewrite the very rings it just laid.
        //
        // It rewrites the WHOLE face list, so it can reach faces put there by strips stitched earlier
        // in the builder's loop as well as by the walkback. What keeps that safe is the box: two
        // points wide, on one rail of one strip, so nothing outside a sliver of the part is even
        // looked at. The one thing in it that is not construction is its lobe pass, which may drop a
        // zero-area lobe off a face it splits; tolerated here only because of that box, and it would
        // not be if the sewing were ever handed a wider set of points.
        //
        // The unchecked form: the bite carries no refusal of its own for the sewing to report, and
        // the box above is what makes that safe.
        let far_seam_rail = if seam_rail == 0 { rails - 1 } else { 0 } as usize;

        surface_split_sewing::sew_in_the_splits_unchecked(
            pool,
            faces,
            &[before_rail[far_seam_rail], after_rail[far_seam_rail]],
        );

        for ring in Self::the_rings(
            rails,
            &before_rail,
            &after_rail,
            &near_at,
            &far_at,
            bite.tip.vertex,
        ) {
            // Faced the way the rest of the strip is faced, and by the same statement: every one of
            // these is a piece of the strip's own surface, not a cap standing in a plane of its own.
            //
            // WithoutRepeats is load bearing rather than tidy: on the SEAM rail the hole's rim point
            // and the row's point are one and the same vertex - the hole is widest there - so the two
            // rings of that rail pair each arrive with a corner visiting it twice, and what is meant
            // is a triangle. Skipped, the fan would lay a needle along one of its own sides.
            surface_fan::emit_facing(
                &surface_ring::without_repeats(&ring),
                pool,
                faces,
                strip.outward_reference,
            );
        }

        Ok(SurfaceStripBiteRows {
            before,
            after,
            near_at,
            far_at,
        })
    }

    /// The seams for ONE of the two sections the strip is stitched in - each
    /// side's own run cut at the section, with the section's own row point standing at its end.
    /// `Ok` is the sliced table: every edge's slot empty except `edge_index`'s.
    ///
    /// The slice is what keeps `SurfaceBevelStitch` ignorant of the hole. It sews between two rings
    /// and clamps any seam vertex outside their window onto whichever ring it is past - so handed the
    /// whole seam, the section before the bite would clamp the hole's far corner onto its own end
    /// ring and sew the strip across the hole. Handed only the part inside its own span, it sews what
    /// it is for and nothing else.
    ///
    /// On the feature's own side the cut falls exactly on the walkback's break, so the slice ends on
    /// the very vertex the row already stands on. On the other side there is no break, and the row's
    /// own minted point stands in for one.
    ///
    /// `station` is the section to cut at, which is [`Self::before`] or [`Self::after`]; `up_to`
    /// whether to take the run UP TO the section, which is the part before the bite.
    pub fn try_the_seams(
        pool: &SurfaceVertexPool,
        seams: &[SideSeams],
        edge_index: i32,
        bite: &SurfaceStripBittenByAFeature,
        rail_count: i32,
        station: &SurfaceStripStation,
        up_to: bool,
    ) -> Result<Vec<SideSeams>, BevelAdvisory> {
        let mut sliced: Vec<SideSeams> = vec![[None, None]; seams.len()];

        // One WELD, and it may not be widened. Every seam vertex inside [opens - apart, opens +
        // apart] is DROPPED from the slice on the assumption that the row's own point stands for it,
        // and that is only true at this distance: the row points are minted through pool.add, which
        // welds to whatever is already within exactly this much. A wider band would drop a vertex the
        // pool kept as its own and leave the landing face sewn to a point the strip no longer stands
        // on.
        let apart = surface_bevel_stitch::one_station(pool);
        let opens = bite.near_seam_param;
        let closes = bite.far_seam_param;

        for side in 0..2 {
            let rail = if side == 0 { 0 } else { rail_count - 1 } as usize;

            let Some(seam) = seams[edge_index as usize][side].as_ref() else {
                // No run of the tangent line on this side at all. The whole strip is sewn to its two
                // seams, so there is nothing to stitch either section to - named here rather than
                // handed back as a bare refusal with nothing to say.
                return Err(does_not_read(pool.position_of(station.rail[rail])));
            };

            let mut run: Vec<(i32, f64)> = Vec::with_capacity(seam.len() + 1);

            if !up_to {
                run.push((station.rail[rail], station.param[rail]));
            }

            for &one in seam {
                if one.1 > opens + apart && one.1 < closes - apart {
                    // A vertex of this seam standing INSIDE the window the bite lays its rows across.
                    // Neither section reaches it and the rows are drawn rail pair by rail pair rather
                    // than station by station, so nothing would sew to it - which is a T junction.
                    return Err(does_not_read(pool.position_of(one.0)));
                }

                let keep = if up_to {
                    one.1 < station.param[rail] - apart
                } else {
                    one.1 > station.param[rail] + apart
                };

                if keep {
                    run.push(one);
                }
            }

            if up_to {
                run.push((station.rail[rail], station.param[rail]));
            }

            if run.len() < 2 {
                // A section with one point on a seam has no run to sew to at all.
                return Err(does_not_read(pool.position_of(run[0].0)));
            }

            sliced[edge_index as usize][side] = Some(run);
        }

        Ok(sliced)
    }

    /// The rings between the two sections, one or two per pair of neighbouring rails.
    /// Pure, so what is laid over the bite can be read off its answer rather than off the mesh
    /// afterwards.
    ///
    /// Each is walked in boundary order, out along the lower rail and back down the higher one, with
    /// the hole's rim standing in for whatever stretch of a rail it takes. Which shape a pair gets is
    /// decided by how many of its two rails the hole reaches, and the ONE pair it reaches by half is
    /// the pair its tip stands in - there is exactly one of those, because the run of covered rails
    /// is contiguous from the seam and closes on the tip.
    fn the_rings(
        rails: i32,
        before: &[i32],
        after: &[i32],
        near_at: &[i32],
        far_at: &[i32],
        tip: i32,
    ) -> Vec<Vec<i32>> {
        let mut rings = Vec::new();
        let mut lower = 0usize;

        while (lower as i32) + 1 < rails {
            let higher = lower + 1;
            let lower_covered = near_at[lower] >= 0;
            let higher_covered = near_at[higher] >= 0;

            if !lower_covered && !higher_covered {
                rings.push(vec![
                    before[lower],
                    before[higher],
                    after[higher],
                    after[lower],
                ]);
            } else if lower_covered && higher_covered {
                rings.push(vec![
                    before[lower],
                    before[higher],
                    near_at[higher],
                    near_at[lower],
                ]);
                rings.push(vec![
                    far_at[lower],
                    far_at[higher],
                    after[higher],
                    after[lower],
                ]);
            } else if higher_covered {
                rings.push(vec![
                    before[lower],
                    before[higher],
                    near_at[higher],
                    tip,
                    far_at[higher],
                    after[higher],
                    after[lower],
                ]);
            } else {
                rings.push(vec![
                    before[lower],
                    before[higher],
                    after[higher],
                    after[lower],
                    far_at[lower],
                    tip,
                    near_at[lower],
                ]);
            }

            lower += 1;
        }

        rings
    }
}
