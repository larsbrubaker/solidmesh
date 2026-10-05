//! A band's strip notched back by a feature STANDING inside its footprint
//! ([`SurfaceStripNotchedByAFeature`]) and the points between rails it is bounded by
//! ([`SurfaceStripNotchedByAFeatureBetween`]). Ports the fields of
//! `Bevel/Surface/SurfaceStripNotchedByAFeature.cs`, the feature's foot as the band sees it, and
//! its constructor, rail questions and readings off the strip; the reading that puts them together
//! (`TryRead`) lands with a later package as a further `impl` block.

use std::rc::Rc;

use vector_math::Vector3;

use super::surface_mitre_joint_step::SideSeams;
use super::{
    surface_strip_ends, SurfaceEdgePlan, SurfaceStandingFeatureWithheld,
    SurfaceStandingFeatureWithheldFace, SurfaceStrip, SurfaceStripTrimmedByTheFaceBeyond,
    SurfaceVertexPool,
};
use crate::bevel::{BevelAdvisory, BevelAdvisoryCode};

/// C# `SurfaceStripNotchedByAFeature`: which rails the feature's extended planes cut, how far along
/// the edge each of them is cut, and the three points between rails where the two planes and the
/// band's end meet - the ridge's EXIT and one Q per plane.
///
/// The shape this is for is a wedge standing on the face the band lands on, whose root the band has
/// run under ([`SurfaceStandingFeatureWithheld`]). Lars's ruling is that such a wedge is carried
/// out over the removed corner until it meets the fillet, which is what the Onshape control does -
/// so the STRIP has to give way where the wedge now stands, and the notch is exactly that hole.
///
/// **Closed form, per rail.** A rail is a line and each of the feature's faces is a plane, so where
/// they meet is one division (`SurfaceStrip.TryCross`). Which of the faces a rail is cut by is
/// asked of WHERE THE CROSSING LANDS and never of the rail's number: the crossing that is really on
/// the feature is the one standing inside every OTHER face's half space, and a chain walked the
/// other way round puts rail zero on the far side of the profile.
///
/// **Read on the CHORDS.** The exit and the two Qs are taken against the polygons the strip really
/// has rather than against the ideal cylinder: a point read off the ideal surface stands off the
/// mesh by the chording, which is a crack.
///
/// **What is refused.** Everything that would have the trace leave the strip somewhere this end cut
/// does not own is named rather than built (`SurfaceStandingFeatureDoesNotExtend`), and a feature
/// standing part way ALONG the strip is `SurfaceStandingFeatureInTheBand` - built now by
/// `SurfaceStripBittenByAFeature`, which the builder asks first.
#[derive(Debug, Clone, PartialEq)]
pub struct SurfaceStripNotchedByAFeature {
    /// Which strip is notched, under the build's own numbering of the edges.
    pub strip: i32,
    /// Which end of it the notch runs off through.
    pub at_the_start: bool,
    /// The feature whose extension takes the hole.
    pub feature: Rc<SurfaceStandingFeatureWithheld>,
    /// WHICH of that feature's reaches this notch is - which of the bands that undercut it took
    /// this hole, as an index into [`SurfaceStandingFeatureWithheld::reaches`]. A feature two bands
    /// reached lost its footing on one run of crease per band, and everything this reading does
    /// with a crease means that band's run and no other.
    pub reach: i32,
    /// The rails the trace cuts, as one contiguous run in ascending rail order.
    pub rail_of: Vec<i32>,
    /// How far along the edge each of those rails is cut back to.
    pub param_of: Vec<f64>,
    /// Which of the feature's faces cut each of those rails.
    pub face_of: Vec<Rc<SurfaceStandingFeatureWithheldFace>>,
    /// The parameter of the band END the notch runs off through.
    pub end_param: f64,
    /// Where the feature's RIDGE leaves the strip: on the chord between the two rails the two faces
    /// change over at, which is the one point both extensions reach.
    pub exit: SurfaceStripNotchedByAFeatureBetween,
    /// Where the trace of the face cutting the run's LOWEST rail meets the band's end ring, on the
    /// chord just below that rail. Low and high are rail numbers and nothing else - which of them
    /// is the upper face of the part depends on which way the chain was walked.
    pub at_the_low_rail: SurfaceStripNotchedByAFeatureBetween,
    /// The same for the face cutting the run's HIGHEST rail, on the chord just above it.
    pub at_the_high_rail: SurfaceStripNotchedByAFeatureBetween,
    /// The rail the feature's CREASE crosses the strip on - which is the strip's own seam rail, the
    /// crease lying in the very face the band walks back from - or -1 where the notch is two-faced
    /// and its inner end is the ridge's exit between two rails instead.
    pub the_crease_rail: i32,
}

/// C# `SurfaceStripNotchedByAFeature.Between` (a readonly struct): one point standing BETWEEN two
/// rails rather than on either of them. It stands only where it has a face, a vertex and a rail
/// below (C# `Stands`); `Between.None` is `(-1, -1, 0, None)`.
#[derive(Debug, Clone, PartialEq, Default)]
pub struct SurfaceStripNotchedByAFeatureBetween {
    /// The vertex itself, found in the pool where the part already has it.
    pub vertex: i32,
    /// The lower of the two rails it stands between.
    pub below: i32,
    /// How far along the edge it stands.
    pub param: f64,
    /// The face whose plane drew it, or `None` where nothing did.
    pub face: Option<Rc<SurfaceStandingFeatureWithheldFace>>,
}

impl SurfaceStripNotchedByAFeature {
    /// The feature's FOOT as the band sees it: whichever end of its creases (the crease each face
    /// holds at `reach`) stands nearest the picked edge's LINE, which is the point the band's reach
    /// has to cover for the feature to be in its way at all. Read off the crease ENDS rather than
    /// off the feature's facets, because those ends are the part's own vertices; measured to the
    /// edge's line and not to its segment, because a foot past either end of it is the end cut's
    /// business.
    pub fn the_foot_of(
        pool: &SurfaceVertexPool,
        edge: &SurfaceEdgePlan,
        feature: &SurfaceStandingFeatureWithheld,
        reach: usize,
    ) -> Vector3 {
        let along = (edge.to() - edge.from()).get_normal();
        let mut foot = edge.from();
        let mut nearest = f64::MAX;

        for face in &feature.faces {
            let crease = face.creases.borrow()[reach];

            for end in [crease.0, crease.1] {
                let at = pool.position_of(end);
                let from_the_edge = at - edge.from();
                let across = (from_the_edge - (along * from_the_edge.dot(along))).length();

                if across < nearest {
                    nearest = across;
                    foot = at;
                }
            }
        }

        foot
    }
}

/// The notch's rail questions (C# `IsOneFaced`, `Notches`).
impl SurfaceStripNotchedByAFeature {
    /// C# `IsOneFaced`: whether the band undercut ONE face of the feature only. Then the notch has no
    /// ridge exit: it is bounded by that one plane's crossings on one side and by the crease's own
    /// crossing of the strip on the other, and it has one Q rather than two.
    pub fn is_one_faced(&self) -> bool {
        self.the_crease_rail >= 0
    }

    /// C# `Notches`: whether `rail` is cut back by the feature rather than by the end.
    pub fn notches(&self, rail: i32) -> bool {
        self.rail_of.contains(&rail)
    }
}

impl SurfaceStripNotchedByAFeatureBetween {
    /// C# `Between.Stands`: whether this is a point standing BETWEEN two rails at all - which is what
    /// the notch's low and high Qs are. Not to be asked of a one-faced notch's exit, which stands ON
    /// a rail and carries `below` of -1 by design.
    pub fn stands(&self) -> bool {
        self.face.is_some() && self.vertex >= 0 && self.below >= 0
    }
}

/// C#'s `SurfaceStripNotchedByAFeature[,]`: per strip, the notch at its start (`[0]`) and at its end
/// (`[1]`), `None` where that end is not notched. C#'s null table is `None` at the call sites.
pub type SurfaceStripNotches = Vec<[Option<Rc<SurfaceStripNotchedByAFeature>>; 2]>;

/// The notch's constructor and its small readings (C# constructor, `ParamOn`, `OnOneFaceOnly`,
/// `Share`, `IsCollapsed`, `StandingAt`).
impl SurfaceStripNotchedByAFeature {
    /// C#'s private constructor.
    #[allow(clippy::too_many_arguments)]
    pub fn new(
        strip: i32,
        at_the_start: bool,
        feature: Rc<SurfaceStandingFeatureWithheld>,
        reach: i32,
        rail_of: Vec<i32>,
        param_of: Vec<f64>,
        face_of: Vec<Rc<SurfaceStandingFeatureWithheldFace>>,
        end_param: f64,
        exit: SurfaceStripNotchedByAFeatureBetween,
        lower: SurfaceStripNotchedByAFeatureBetween,
        upper: SurfaceStripNotchedByAFeatureBetween,
        the_crease_rail: i32,
    ) -> Self {
        Self {
            strip,
            at_the_start,
            feature,
            reach,
            rail_of,
            param_of,
            face_of,
            end_param,
            exit,
            at_the_low_rail: lower,
            at_the_high_rail: upper,
            the_crease_rail,
        }
    }

    /// C# `ParamOn`: how far along the edge `rail` is cut back to, or `None` for a rail the notch
    /// does not take (where C#'s `ParamOf[Array.IndexOf(..)]` would throw; callers ask
    /// [`Self::notches`] first).
    pub fn param_on(&self, rail: i32) -> Option<f64> {
        self.rail_of
            .iter()
            .position(|&on| on == rail)
            .map(|index| self.param_of[index])
    }

    /// C# `OnOneFaceOnly`: whether one face of the feature cut every rail the trace took.
    pub fn on_one_face_only(face_of: &[Rc<SurfaceStandingFeatureWithheldFace>]) -> bool {
        face_of.iter().all(|face| Rc::ptr_eq(face, &face_of[0]))
    }

    /// C# `Share`: the vertex at one point - the part's OWN where it already has one within `snap`,
    /// and a new one only where it has not. A crossing landing on a crease the walkback left standing
    /// is that crease's vertex and not a second beside it.
    pub fn share(pool: &mut SurfaceVertexPool, at: Vector3, snap: f64) -> i32 {
        match pool.try_find_near(at, snap, None) {
            Some(found) => found,
            None => pool.add(at),
        }
    }

    /// C# `IsCollapsed`: whether the run-off trim has already collapsed one rail to a point. `ran_off`
    /// is C#'s array of trims, `None` where a side did not run off.
    pub fn is_collapsed(
        ran_off: &[Option<&SurfaceStripTrimmedByTheFaceBeyond>],
        rail_count: i32,
        rail: i32,
    ) -> bool {
        ran_off.iter().flatten().any(|trim| {
            (if trim.side == 0 {
                rail
            } else {
                rail_count - 1 - rail
            }) < trim.first_kept
        })
    }

    /// C# `StandingAt`: the notch standing at one end of one strip, or `None` where that end is not
    /// notched.
    pub fn standing_at(
        notches: Option<&SurfaceStripNotches>,
        strip: i32,
        at_the_start: bool,
    ) -> Option<Rc<SurfaceStripNotchedByAFeature>> {
        let notches = notches?;

        if strip < 0 || strip as usize >= notches.len() {
            return None;
        }

        notches[strip as usize][if at_the_start { 0 } else { 1 }].clone()
    }
}

impl SurfaceStripNotchedByAFeatureBetween {
    /// C#'s `Between` constructor: the vertex, the lower of the two rails it stands between, how far
    /// along the edge it stands, and the face whose plane drew it.
    pub fn new(
        vertex: i32,
        below: i32,
        param: f64,
        face: Option<Rc<SurfaceStandingFeatureWithheldFace>>,
    ) -> Self {
        Self {
            vertex,
            below,
            param,
            face,
        }
    }

    /// C# `Between.None`: no point at all - the Q a ONE-FACED notch does not have. Said with a vertex
    /// and a chord of -1 rather than left as the default, because that default reads as vertex zero
    /// standing on chord zero - a real point of the pool on a real chord of the strip.
    pub fn none() -> Self {
        Self::new(-1, -1, 0.0, None)
    }
}

/// The notch's readings off the strip and the feature (C# `TryTheOneFacedNotch`,
/// `TryTheCreaseCrossing`, `TryTheEndItLeavesThrough`, `TryTheFaceThatCuts`, `TryTheExit`,
/// `TryTheRidgeOnTheChord`, `TryTheQ`). `Try*` methods with C#'s `out BevelAdvisory failure` return
/// `Result`; the others return `Option`.
impl SurfaceStripNotchedByAFeature {
    /// C# `TryTheOneFacedNotch`: the notch a band that undercut ONE face of the feature takes - the
    /// trace runs from where the feature's CREASE leaves the strip, out through that face's crossings,
    /// to a single Q on the band's end ring. `rail_of`, `param_of` and `face_of` are the plane's own
    /// crossings, which this completes in place (as C# does to its lists) with the crease rail.
    ///
    /// The crossing is the walkback's own point: the seam is the run of the tangent line the walkback
    /// left standing on the landing face, and it ENDS where that face ends - on the crease. So it is
    /// taken from the seam rather than crossed a second time, which would land a rounding off the one
    /// the mesh already stands on and crack the whole length of the seam.
    #[allow(clippy::too_many_arguments)]
    pub fn try_the_one_faced_notch(
        pool: &mut SurfaceVertexPool,
        strip: &SurfaceStrip,
        strip_index: i32,
        seams: Option<&[SideSeams]>,
        at_the_start: bool,
        feature: &Rc<SurfaceStandingFeatureWithheld>,
        reach: i32,
        rail_of: &mut Vec<i32>,
        param_of: &mut Vec<f64>,
        face_of: &mut Vec<Rc<SurfaceStandingFeatureWithheldFace>>,
        end_param: f64,
        snap: f64,
    ) -> Result<Self, BevelAdvisory> {
        let does_not_extend = |at: Vector3| {
            BevelAdvisory::new(
                BevelAdvisoryCode::SurfaceStandingFeatureDoesNotExtend,
                vec![at.into()],
            )
        };

        let Some((crease_rail, crossing)) = Self::try_the_crease_crossing(
            pool,
            seams,
            strip_index,
            strip.rail_count(),
            at_the_start,
            &face_of[0],
            reach,
            snap,
        ) else {
            // The crease does not run out on either seam of this strip, so the band did not undercut it
            // where this reading says it did and there is no inner end for the trace to start at.
            let crease_from = face_of[0].creases.borrow()[reach as usize].0;
            return Err(does_not_extend(pool.position_of(crease_from)));
        };

        let first = rail_of[0];
        let last = rail_of[rail_of.len() - 1];

        if first != crease_rail && last != crease_rail {
            if first - 1 != crease_rail && last + 1 != crease_rail {
                // The rails the plane cuts and the rail the crease runs out on are not one run. The
                // notch is then two holes with strip between them, which is not a hole this end closes.
                return Err(does_not_extend(pool.position_of(crossing.vertex)));
            }

            // The plane's own crossing of the SEAM rail is the point the ridge stands on - both of the
            // feature's faces meet the rail there and neither of them can be said to cut it - so the run
            // reads one rail short. The crease says what that rail is cut back to.
            let before = first - 1 == crease_rail;
            let first_face = face_of[0].clone();

            if before {
                rail_of.insert(0, crease_rail);
                param_of.insert(0, crossing.param);
                face_of.insert(0, first_face);
            } else {
                rail_of.push(crease_rail);
                param_of.push(crossing.param);
                face_of.push(first_face);
            }
        } else {
            let index = rail_of
                .iter()
                .position(|&rail| rail == crease_rail)
                .unwrap_or_default();

            if (param_of[index] - crossing.param).abs() > surface_strip_ends::meets_the_seam(pool) {
                // Two readings of one thing disagreeing: the plane says this rail is cut back to one
                // place and the seam the walkback left says another. Named rather than sewn to whichever
                // was asked for first.
                return Err(does_not_extend(pool.position_of(crossing.vertex)));
            }

            // The seam's own point, and not the one the plane crossing would mint beside it.
            param_of[index] = crossing.param;
        }

        // The run's far end from the crease, and the rail just outside it - which way that is said by
        // the SEAM rail's own end of the profile and not by the run's ordering, since a run of one rail
        // is both of its own ends at once.
        let runs_up = crease_rail == 0;
        let far = if runs_up {
            rail_of[rail_of.len() - 1]
        } else {
            rail_of[0]
        };
        let outside = if runs_up { far + 1 } else { far - 1 };

        let q = Self::try_the_q(pool, strip, far, outside, &face_of[0], end_param, snap)?;

        let (lower, upper) = if runs_up {
            (SurfaceStripNotchedByAFeatureBetween::none(), q)
        } else {
            (q, SurfaceStripNotchedByAFeatureBetween::none())
        };

        Ok(Self::new(
            strip_index,
            at_the_start,
            feature.clone(),
            reach,
            rail_of.clone(),
            param_of.clone(),
            face_of.clone(),
            end_param,
            crossing,
            lower,
            upper,
            crease_rail,
        ))
    }

    /// C# `TryTheCreaseCrossing`: where the feature's crease leaves the strip - the vertex the
    /// walkback's own SEAM ends on, where that vertex stands on the crease's line, and which of the
    /// strip's two seam rails it is on - as `(crease_rail, crossing)`. A seam table row that is not
    /// there reads as no seam, as a null run does in C#.
    #[allow(clippy::too_many_arguments, clippy::neg_cmp_op_on_partial_ord)]
    pub fn try_the_crease_crossing(
        pool: &SurfaceVertexPool,
        seams: Option<&[SideSeams]>,
        strip_index: i32,
        rail_count: i32,
        at_the_start: bool,
        face: &Rc<SurfaceStandingFeatureWithheldFace>,
        reach: i32,
        snap: f64,
    ) -> Option<(i32, SurfaceStripNotchedByAFeatureBetween)> {
        let (crease_from, crease_to) = face.creases.borrow()[reach as usize];
        let from = pool.position_of(crease_from);
        let to = pool.position_of(crease_to);
        let along = to - from;

        if !(along.length_squared() > 0.0) {
            return None;
        }

        // The ruler, in the crease's OWN parameter: snap is a length, and how far along the crease a
        // point stands is a fraction of it, so the two are only the same question once the length is
        // divided out.
        let ends_within = snap / along.length();

        let mut crease_rail = -1;
        let mut crossing = SurfaceStripNotchedByAFeatureBetween::none();

        for side in 0..2 {
            let Some(seam) = seams
                .and_then(|seams| seams.get(strip_index as usize))
                .and_then(|row| row[side].as_ref())
                .filter(|seam| !seam.is_empty())
            else {
                continue;
            };

            // The end of the seam this notch's own end stands at: the run is ordered along the edge,
            // and a notch at the start of the band is closed by the start of the seam.
            let mut at = seam[0];

            for &one in seam {
                if if at_the_start {
                    one.1 < at.1
                } else {
                    one.1 > at.1
                } {
                    at = one;
                }
            }

            let stands = pool.position_of(at.0);
            let how_far = (stands - from).dot(along) / along.length_squared();

            if how_far < -ends_within
                || how_far > 1.0 + ends_within
                || (stands - (from + (along * how_far))).length() > snap
            {
                continue;
            }

            if crease_rail >= 0 {
                // BOTH seams end on this crease. The crease lies in ONE of the two landing faces, so only
                // one of the strip's seam rails can be the one it runs out on - two of them saying so is
                // two readings of one thing that cannot both be true.
                return None;
            }

            crease_rail = if side == 0 { 0 } else { rail_count - 1 };
            crossing =
                SurfaceStripNotchedByAFeatureBetween::new(at.0, -1, at.1, Some(face.clone()));
        }

        (crease_rail >= 0).then_some((crease_rail, crossing))
    }

    /// C# `TryTheEndItLeavesThrough`: which of the band's two ends the extension runs off through -
    /// the one whose plane a crease of the feature ends in, since that is the line the feature's own
    /// faces already die on - as `(at_the_start, end_param)`.
    pub fn try_the_end_it_leaves_through(
        pool: &SurfaceVertexPool,
        strip: &SurfaceStrip,
        feature: &SurfaceStandingFeatureWithheld,
        reach: i32,
        length: f64,
        snap: f64,
    ) -> Option<(bool, f64)> {
        let mut nearest_to_the_end = f64::MAX;
        let mut nearest_to_the_start = f64::MAX;

        for face in &feature.faces {
            let (crease_from, crease_to) = face.creases.borrow()[reach as usize];

            for at in [crease_from, crease_to] {
                let param = strip.param_of(pool.position_of(at));

                nearest_to_the_end = nearest_to_the_end.min((param - length).abs());
                nearest_to_the_start = nearest_to_the_start.min(param.abs());
            }
        }

        if nearest_to_the_end.min(nearest_to_the_start) > snap {
            return None;
        }

        let at_the_start = nearest_to_the_start < nearest_to_the_end;

        Some((at_the_start, if at_the_start { 0.0 } else { length }))
    }

    /// C# `TryTheFaceThatCuts`: which of the feature's faces cuts one rail, and where - the single
    /// crossing of the wedge this end cut is drawn from. TWO crossings is the ridge itself passing
    /// through the rail, which has no one plane to cut by, and reads as none: a rail the wedge passes
    /// clean through is bitten rather than notched (`SurfaceStripBittenByAFeature`).
    pub fn try_the_face_that_cuts(
        strip: &SurfaceStrip,
        feature: &SurfaceStandingFeatureWithheld,
        rail: i32,
        snap: f64,
    ) -> Option<(Rc<SurfaceStandingFeatureWithheldFace>, f64)> {
        let mut found = Vec::new();

        feature.crossings_on_the_wedge(strip, rail, snap, &mut found);

        if found.len() == 1 {
            found.pop()
        } else {
            None
        }
    }

    /// C# `TryTheExit`: the ridge's EXIT - where the feature's own ridge line leaves the strip, read
    /// against the plane of the quad between the two rails the faces change over at rather than
    /// against the ideal cylinder.
    #[allow(clippy::too_many_arguments)]
    pub fn try_the_exit(
        pool: &mut SurfaceVertexPool,
        strip: &SurfaceStrip,
        edge: &SurfaceEdgePlan,
        feature: &SurfaceStandingFeatureWithheld,
        rail_of: &[i32],
        face_of: &[Rc<SurfaceStandingFeatureWithheldFace>],
        snap: f64,
    ) -> Result<SurfaceStripNotchedByAFeatureBetween, BevelAdvisory> {
        let does_not_extend = |at: Vector3| {
            BevelAdvisory::new(
                BevelAdvisoryCode::SurfaceStandingFeatureDoesNotExtend,
                vec![at.into()],
            )
        };

        let mut changed: i32 = -1;

        for step in 0..face_of.len().saturating_sub(1) {
            if Rc::ptr_eq(&face_of[step], &face_of[step + 1]) {
                continue;
            }

            if changed >= 0 {
                changed = -2;
                break;
            }

            changed = step as i32;
        }

        let ridge = if changed < 0 {
            None
        } else {
            feature.try_the_ridge()
        };

        let Some((from, along)) = ridge else {
            // No changeover is a feature standing on ONE plane across the whole notch, and more than one
            // is a ridge crossing the strip twice; neither is the single exit this cap closes on. And a
            // feature whose two planes do not meet along a line has no ridge to leave the strip on.
            return Err(does_not_extend(strip.point_on(rail_of[0], 0.0)));
        };

        let below = rail_of[changed as usize];

        let Some(at) = Self::try_the_ridge_on_the_chord(strip, edge, from, along, below) else {
            return Err(does_not_extend(from));
        };

        Ok(SurfaceStripNotchedByAFeatureBetween::new(
            Self::share(pool, at, snap),
            below,
            strip.param_of(at),
            Some(face_of[changed as usize].clone()),
        ))
    }

    /// C# `TryTheRidgeOnTheChord`: where the feature's RIDGE line meets the strip between rail `below`
    /// and the one above it - read against the PLANE OF THE QUAD those two rails rule rather than
    /// against the ideal cylinder, so the point stands on the mesh the strip really has. `None` where
    /// the ridge runs along that quad. Whether the point stands BETWEEN the two rails is the caller's
    /// question.
    #[allow(clippy::neg_cmp_op_on_partial_ord)]
    pub fn try_the_ridge_on_the_chord(
        strip: &SurfaceStrip,
        edge: &SurfaceEdgePlan,
        from: Vector3,
        along: Vector3,
        below: i32,
    ) -> Option<Vector3> {
        let quad = (strip.point_on(below + 1, 0.0) - strip.point_on(below, 0.0))
            .cross(edge.edge.direction())
            .get_normal();
        let slope = quad.dot(along);

        if !(slope.abs() > 1e-9) {
            return None;
        }

        Some(from + (along * (quad.dot(strip.point_on(below, 0.0) - from) / slope)))
    }

    /// C# `TryTheQ`: one Q - where a face's plane meets the CHORD the strip's end ring draws between
    /// the last rail of the notched run (`in_the_run`) and the rail just `outside` it. C#'s unused
    /// `feature` parameter is dropped.
    #[allow(clippy::too_many_arguments, clippy::neg_cmp_op_on_partial_ord)]
    pub fn try_the_q(
        pool: &mut SurfaceVertexPool,
        strip: &SurfaceStrip,
        in_the_run: i32,
        outside: i32,
        face: &Rc<SurfaceStandingFeatureWithheldFace>,
        end_param: f64,
        snap: f64,
    ) -> Result<SurfaceStripNotchedByAFeatureBetween, BevelAdvisory> {
        let inside = strip.point_on(in_the_run, end_param);
        let does_not_extend = |at: Vector3| {
            BevelAdvisory::new(
                BevelAdvisoryCode::SurfaceStandingFeatureDoesNotExtend,
                vec![at.into()],
            )
        };

        if outside < 0 || outside >= strip.rail_count() {
            // The run reaches the edge of the profile, so the trace leaves through the strip's own SEAM
            // rather than through the band's end - a face beyond this end cut does not own.
            return Err(does_not_extend(inside));
        }

        let beyond = strip.point_on(outside, end_param);
        let here = face.plane_normal.dot(inside - face.plane_point);
        let there = face.plane_normal.dot(beyond - face.plane_point);

        if !(here * there < 0.0) {
            return Err(does_not_extend(inside));
        }

        let at = inside + ((beyond - inside) * (here / (here - there)));

        Ok(SurfaceStripNotchedByAFeatureBetween::new(
            Self::share(pool, at, snap),
            in_the_run.min(outside),
            end_param,
            Some(face.clone()),
        ))
    }
}
