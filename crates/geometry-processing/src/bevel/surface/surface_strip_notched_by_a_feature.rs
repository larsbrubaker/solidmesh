//! A band's strip notched back by a feature STANDING inside its footprint
//! ([`SurfaceStripNotchedByAFeature`]) and the points between rails it is bounded by
//! ([`SurfaceStripNotchedByAFeatureBetween`]). Ports the fields of
//! `Bevel/Surface/SurfaceStripNotchedByAFeature.cs`, the feature's foot as the band sees it, and
//! its constructor and rail questions; the reading itself lands with a later package as a further
//! `impl` block.

use std::rc::Rc;

use vector_math::Vector3;

use super::{
    SurfaceEdgePlan, SurfaceStandingFeatureWithheld, SurfaceStandingFeatureWithheldFace,
    SurfaceStripTrimmedByTheFaceBeyond, SurfaceVertexPool,
};

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
