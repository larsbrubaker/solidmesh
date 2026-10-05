//! A band's strip notched back by a feature STANDING inside its footprint
//! ([`SurfaceStripNotchedByAFeature`]) and the points between rails it is bounded by
//! ([`SurfaceStripNotchedByAFeatureBetween`]). Ports the fields of
//! `Bevel/Surface/SurfaceStripNotchedByAFeature.cs` and the feature's foot as the band sees it;
//! the reading and its rail questions land with another package as further `impl` blocks.

use std::rc::Rc;

use vector_math::Vector3;

use super::{
    SurfaceEdgePlan, SurfaceStandingFeatureWithheld, SurfaceStandingFeatureWithheldFace,
    SurfaceVertexPool,
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
