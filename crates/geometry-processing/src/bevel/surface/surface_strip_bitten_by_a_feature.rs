//! The BITE a feature standing part way ALONG a band takes out of the middle of its strip
//! ([`SurfaceStripBittenByAFeature`]). Ports `Bevel/Surface/SurfaceStripBittenByAFeature.cs`: its
//! fields, its reading ([`SurfaceStripBittenByAFeature::try_read`]) and the tip and seam readings
//! that reading is made of. Its sibling, the notch at a band's END, is
//! `SurfaceStripNotchedByAFeature`, whose readings off the strip this one shares.
//!
//! This is the notch's sibling and the case that one refuses. A wedge standing under a band's END
//! has an end of the strip to run off through, and what it takes out is a NOTCH open at that end;
//! a wedge standing part way along has no end to leave by, and what it takes out is a closed hole -
//! Lars's ruling and the Onshape control's answer, the feature's faces carried on past the landing
//! face they used to die on and trimmed against the round, the round carrying the matching hole.
//! "Encroaching Side.mcx" is that shape.
//!
//! **Seam to tip, and only one tip.** The feature stands ON one of the band's two landing faces, so
//! its material necessarily enters the strip through THAT face's seam: the bite is widest at the
//! seam and narrows inward to a single point where the feature's ridge crosses the strip. A hole
//! hanging free in the middle of a strip, with a tip at each end, is a different shape and nothing
//! reaches it yet. The two corners at the seam are the walkback's OWN vertices - the tangent line
//! it left standing is broken in two where the feature's creases cross it, and the break's two ends
//! are those corners - taken from it rather than minted beside it, for the same reason the notch's
//! one-faced reading takes its inner end off the seam: a point read a second time from two planes
//! lands a rounding off the one the mesh already stands on, and that rounding is a crack. What the
//! planes say is used to CHECK the seam's own break, not to replace it.
//!
//! **The far half of the wedge is not the feature.** A wedge is unbounded along its ridge, so it
//! crosses the round a second time on the far side of the axis - on the fixture's 3.4 band the
//! rails at both ends of the profile read a pair, with three uncovered rails between them. Only the
//! run reaching the landing face the feature's creases lie in is the feature: the other run is the
//! same two planes passing through ground nothing stands on, and biting the strip there would open
//! a hole into thin air. So the run is walked from that seam's own rail inward, and contiguity is
//! asked of THAT run only.
//!
//! **Read on the CHORDS.** Every crossing here is one division against a rail, and the tip is read
//! against the plane of the quad the two rails it stands between rule
//! (`SurfaceStripNotchedByAFeature::try_the_ridge_on_the_chord`) - never against the ideal
//! cylinder, which stands off the mesh by the chording. On the fixture that puts the tip at
//! (15.520, 5, 10) where the true cylinder would put it at (15.474, 5, 10): 0.045, which is the
//! sagitta of a 16.875 degree step at radius 3.4 and not an error.

use std::rc::Rc;

use vector_math::Vector3;

use super::{
    SurfaceEdgePlan, SurfaceStandingFeatureWithheld, SurfaceStandingFeatureWithheldFace,
    SurfaceStrip, SurfaceStripNotchedByAFeature, SurfaceStripNotchedByAFeatureBetween,
    SurfaceStripTrimmedByTheFaceBeyond, SurfaceVertexPool,
};
use crate::bevel::{BevelAdvisory, BevelAdvisoryCode};

type Face = SurfaceStandingFeatureWithheldFace;

/// C# `SurfaceStripBittenByAFeature`: which rails the feature's wedge passes clean through, where
/// it enters and leaves each of them, the one TIP the bite closes on, and the two points on the
/// band's own seam the hole is bounded by at the other end.
#[derive(Debug, Clone, PartialEq)]
pub struct SurfaceStripBittenByAFeature {
    /// Whether the bite opens onto the strip's LEFT seam - which is the side the walkback says the
    /// feature stands on ([`SurfaceStandingFeatureWithheld::reaches`]), and so which of rail zero
    /// and the last rail the run starts at.
    pub at_the_left_seam: bool,
    /// The feature whose extension takes the hole.
    pub feature: Rc<SurfaceStandingFeatureWithheld>,
    /// WHICH of that feature's reaches this bite is - which of the bands that undercut it takes
    /// this hole, as an index into [`SurfaceStandingFeatureWithheld::reaches`]. One feature can be
    /// bitten by two bands of two different chains, and each bite is read against its own band's
    /// run of crease.
    pub reach: i32,
    /// The rails the wedge passes clean through, in the order they are walked: the seam's own rail
    /// first and the tip's neighbour last.
    pub rail_of: Vec<i32>,
    /// Where the hole starts on each of those rails - the crossing at the SMALLER parameter.
    pub near_param_of: Vec<f64>,
    /// Where it ends again on each of them - the crossing at the larger parameter.
    pub far_param_of: Vec<f64>,
    /// The one face of the feature that draws the hole's near side, the same on every covered rail.
    /// A side drawn by more than one plane would be two holes rather than one and is refused.
    pub near_face: Rc<Face>,
    /// The one face that draws its far side.
    pub far_face: Rc<Face>,
    /// The point the bite closes on: where the feature's ridge crosses the chord between the last
    /// covered rail and the first uncovered one. It stands where BOTH faces meet, so it carries no
    /// one face of its own and reads false to `SurfaceStripNotchedByAFeatureBetween::stands`, which
    /// is a question about the notch's Qs rather than about this.
    pub tip: SurfaceStripNotchedByAFeatureBetween,
    /// The band's seam on the feature's side as the bite leaves it: the walkback's two runs, in
    /// order along the edge, joined into the one ordered run every reader downstream expects, with
    /// the hole standing in the gap between [`Self::near_seam_vertex`] and
    /// [`Self::far_seam_vertex`].
    ///
    /// Joined here rather than at the seam pick, which takes whichever single run overlaps the edge
    /// most (`SurfaceSeamOverTheEdge`) and on a strip broken exactly in half would take one of two
    /// equal halves by the order they happened to arrive in. Nothing downstream is told about the
    /// gap, and nothing needs to be: the bite lays its own rows across it, and the two stitched
    /// sections are each given only the part of this run inside their own span.
    pub seam: Vec<(i32, f64)>,
    /// The seam vertex the hole's near side ends on - the last of the walkback's first run.
    pub near_seam_vertex: i32,
    /// How far along the edge that vertex stands, which is where the strip stops.
    ///
    /// The WALKBACK's own reading of the hole's near edge and not the planes': the two agree to
    /// within the pool's snap or this was refused, but snap is ten welds and everything downstream
    /// measures in welds, so the number the construction uses has to be the one its vertex is at.
    pub near_seam_param: f64,
    /// The seam vertex its far side starts from - the first of the walkback's second run.
    pub far_seam_vertex: i32,
    /// How far along the edge THAT one stands, which is where the strip resumes.
    pub far_seam_param: f64,
}

/// The seam a bite leaves and its two corners (C# `TryTheSeam`'s outs).
struct TheSeam {
    seam: Vec<(i32, f64)>,
    near_seam: (i32, f64),
    far_seam: (i32, f64),
}

impl SurfaceStripBittenByAFeature {
    /// C# `SeamRail`: the rail the bite opens onto the seam at.
    pub fn seam_rail(&self) -> i32 {
        self.rail_of[0]
    }

    /// C# `TryRead`: the bite one standing feature takes out of the middle of one strip; `Ok(None)`
    /// where this feature takes none, which is an END case the notch reader owns
    /// (`SurfaceStripNotchedByAFeature::try_read`).
    ///
    /// `ran_off` is the run-off trim on either side of this strip, whose collapsed rails are air
    /// (`None` where a side did not run off). `runs` are the walkback's own runs of the tangent line
    /// on the side the feature stands on, which the hole's two corners at the seam are taken from
    /// (`None` for C#'s null).
    #[allow(clippy::too_many_arguments, clippy::neg_cmp_op_on_partial_ord)]
    pub fn try_read(
        pool: &mut SurfaceVertexPool,
        strip: &SurfaceStrip,
        edge: &SurfaceEdgePlan,
        feature: &Rc<SurfaceStandingFeatureWithheld>,
        reach: i32,
        ran_off: &[Option<&SurfaceStripTrimmedByTheFaceBeyond>],
        runs: Option<&[Vec<i32>]>,
    ) -> Result<Option<SurfaceStripBittenByAFeature>, BevelAdvisory> {
        let snap = SurfaceVertexPool::near_enough_for(pool.vertices());
        let length = (edge.to() - edge.from()).length();

        if SurfaceStripNotchedByAFeature::try_the_end_it_leaves_through(
            pool, strip, feature, reach, length, snap,
        )
        .is_some()
        {
            // An END of one of this feature's creases stands in one of the band's end planes, so
            // the feature has an end of the strip to run off through and what it takes is a notch
            // open at that end. Not this reading's, and said as "no bite" rather than as a refusal
            // so that the caller goes on to the reader that owns it.
            return Ok(None);
        }

        let at_the_left_seam = feature.reaches.borrow()[reach as usize].1;
        let seam_rail = if at_the_left_seam {
            0
        } else {
            strip.rail_count() - 1
        };
        let inward = if at_the_left_seam { 1 } else { -1 };

        let mut rail_of: Vec<i32> = Vec::new();
        let mut near_param_of: Vec<f64> = Vec::new();
        let mut far_param_of: Vec<f64> = Vec::new();
        let mut found: Vec<(Rc<Face>, f64)> = Vec::new();

        let mut faces_of_the_sides: Option<(Rc<Face>, Rc<Face>)> = None;

        let mut rail = seam_rail;
        while rail >= 0 && rail < strip.rail_count() {
            if SurfaceStripNotchedByAFeature::is_collapsed(ran_off, strip.rail_count(), rail) {
                // Air over the whole edge: the run-off already took this rail, and a hole read on
                // it would open a surface that is not there.
                break;
            }

            feature.crossings_on_the_wedge(strip, rail, snap, &mut found);

            if found.len() != 2 {
                // One crossing is a rail the wedge cuts BACK rather than passes through, and none
                // is a rail it misses. Either way the run of covered rails ends here.
                break;
            }

            let (near, far) = if found[0].1 <= found[1].1 {
                (&found[0], &found[1])
            } else {
                (&found[1], &found[0])
            };

            if !(near.1 > snap && far.1 < length - snap) {
                // The hole would open into the band's own end plane, and the end cut draws that end
                // from the strip's rails rather than from anything here.
                return Err(refused(strip, rail, near.1));
            }

            match &faces_of_the_sides {
                None => faces_of_the_sides = Some((near.0.clone(), far.0.clone())),
                Some((near_face, far_face)) => {
                    if !Rc::ptr_eq(&near.0, near_face) || !Rc::ptr_eq(&far.0, far_face) {
                        // A side of the hole changing plane part way along is two holes meeting
                        // rather than one, which is not a shape this closes.
                        return Err(refused(strip, rail, near.1));
                    }
                }
            }

            rail_of.push(rail);
            near_param_of.push(near.1);
            far_param_of.push(far.1);

            rail += inward;
        }

        let Some((near_face, far_face)) = faces_of_the_sides else {
            // Nothing of the feature reaches the strip at the seam of the very face it stands on.
            // Named at the foot, which is the thing standing in the band and the place a reader
            // looks.
            return Err(BevelAdvisory::new(
                BevelAdvisoryCode::SurfaceBiteDoesNotRead,
                vec![SurfaceStripNotchedByAFeature::the_foot_of(
                    pool,
                    edge,
                    feature,
                    reach as usize,
                )
                .into()],
            ));
        };

        let last = rail_of[rail_of.len() - 1];
        let beyond = last + inward;

        if beyond < 0 || beyond >= strip.rail_count() {
            // The run crossed the whole profile and reached the strip's OTHER seam, so the wedge
            // takes the strip in two rather than biting a hole in it.
            return Err(refused(strip, last, near_param_of[near_param_of.len() - 1]));
        }

        let tip = Self::try_the_tip(pool, strip, edge, feature, last, beyond, snap).map_err(
            |tip_at| {
                BevelAdvisory::new(
                    BevelAdvisoryCode::SurfaceBiteDoesNotRead,
                    vec![tip_at.into()],
                )
            },
        )?;

        let the_seam = Self::try_the_seam(
            pool,
            strip,
            runs,
            seam_rail,
            near_param_of[0],
            far_param_of[0],
            snap,
        )
        .map_err(|refused_at| {
            BevelAdvisory::new(
                BevelAdvisoryCode::SurfaceBiteDoesNotRead,
                vec![refused_at.into()],
            )
        })?;

        Ok(Some(SurfaceStripBittenByAFeature {
            at_the_left_seam,
            feature: feature.clone(),
            reach,
            rail_of,
            near_param_of,
            far_param_of,
            near_face,
            far_face,
            tip,
            seam: the_seam.seam,
            near_seam_vertex: the_seam.near_seam.0,
            near_seam_param: the_seam.near_seam.1,
            far_seam_vertex: the_seam.far_seam.0,
            far_seam_param: the_seam.far_seam.1,
        }))
    }

    /// C# `TryTheTip`: the TIP the bite closes on - the feature's ridge read against the plane of
    /// the quad between the last covered rail and the first uncovered one, and standing BETWEEN the
    /// two of them. `Err` carries where to name a refusal: the point that would not do, or (as C#
    /// leaves its `out at`) the origin where the ridge runs along the quad.
    #[allow(clippy::too_many_arguments, clippy::neg_cmp_op_on_partial_ord)]
    fn try_the_tip(
        pool: &mut SurfaceVertexPool,
        strip: &SurfaceStrip,
        edge: &SurfaceEdgePlan,
        feature: &SurfaceStandingFeatureWithheld,
        last: i32,
        beyond: i32,
        snap: f64,
    ) -> Result<SurfaceStripNotchedByAFeatureBetween, Vector3> {
        let Some((from, along)) = feature.try_the_ridge() else {
            // No two planes meeting along a line is a feature with no ridge for the bite to close
            // on.
            return Err(strip.point_on(last, 0.0));
        };

        let below = last.min(beyond);

        let Some(at) = SurfaceStripNotchedByAFeature::try_the_ridge_on_the_chord(
            strip, edge, from, along, below,
        ) else {
            return Err(Vector3::ZERO);
        };

        // Across the chord rather than along the edge: the quad is ruled, so the point it found is
        // the rail at 'last' carried a fraction of the way to 'beyond' and then some distance
        // along, and only the fraction says whether the ridge really leaves the strip between these
        // two rails.
        let across = strip.point_on(beyond, 0.0) - strip.point_on(last, 0.0);
        let how_far = (at - strip.point_on(last, 0.0)).dot(across) / across.length_squared();

        if !(how_far > 0.0 && how_far < 1.0) {
            return Err(at);
        }

        Ok(SurfaceStripNotchedByAFeatureBetween::new(
            SurfaceStripNotchedByAFeature::share(pool, at, snap),
            below,
            strip.param_of(at),
            None,
        ))
    }

    /// C# `TryTheSeam`: the seam the bite leaves behind - the walkback's two runs of the tangent
    /// line joined in order along the edge, and the two vertices of the break between them, which
    /// are the hole's own corners. `Err` carries where to name the refusal.
    ///
    /// The planes' crossings of the seam's rail are what the break is CHECKED against and not what
    /// is used: both readings are of one thing, and taking the planes' answer would put the hole's
    /// corners a rounding off the vertices the landing face is already sewn to.
    #[allow(clippy::too_many_arguments)]
    fn try_the_seam(
        pool: &SurfaceVertexPool,
        strip: &SurfaceStrip,
        runs: Option<&[Vec<i32>]>,
        seam_rail: i32,
        near_param: f64,
        far_param: f64,
        snap: f64,
    ) -> Result<TheSeam, Vector3> {
        let refused_at = strip.point_on(seam_rail, near_param);

        let Some(runs) = runs.filter(|runs| runs.len() == 2) else {
            // The feature breaks the tangent line in exactly two, one run either side of it. More
            // breaks than that is something else standing on the face as well, which is not one
            // bite.
            return Err(refused_at);
        };

        let mut read: [Vec<(i32, f64)>; 2] = [Vec::new(), Vec::new()];

        for which in 0..2 {
            if runs[which].len() < 2 {
                return Err(refused_at);
            }

            read[which] = runs[which]
                .iter()
                .map(|&vertex| (vertex, strip.param_of(pool.position_of(vertex))))
                .collect();

            if read[which][0].1 > read[which][read[which].len() - 1].1 {
                // A run is a stretch of the line walked one way; one walked the other is a reading
                // this cannot join to its neighbour without deciding which of the two orders is the
                // seam's.
                return Err(pool.position_of(read[which][0].0));
            }
        }

        let [first, second] = read;
        let (before, after) = if first[0].1 <= second[0].1 {
            (first, second)
        } else {
            (second, first)
        };

        let near_seam = before[before.len() - 1];
        let far_seam = after[0];

        if (near_seam.1 - near_param).abs() > snap || (far_seam.1 - far_param).abs() > snap {
            // Two readings of one thing disagreeing: the walkback says the tangent line is broken
            // here and the feature's own planes say the hole reaches the seam somewhere else. Named
            // rather than sewn to whichever was asked for first.
            return Err(pool.position_of(near_seam.0));
        }

        let mut seam = Vec::with_capacity(before.len() + after.len());
        seam.extend(before);
        seam.extend(after);

        Ok(TheSeam {
            seam,
            near_seam,
            far_seam,
        })
    }
}

/// C# `Refused`: one refusal, named at the point on the strip the reading gave up at.
fn refused(strip: &SurfaceStrip, rail: i32, param: f64) -> BevelAdvisory {
    BevelAdvisory::new(
        BevelAdvisoryCode::SurfaceBiteDoesNotRead,
        vec![strip.point_on(rail, param).into()],
    )
}
