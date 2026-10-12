//! One tool built for one run of edges, and which way the boolean has to take it ([`BevelTool`]).
//! Ports `Bevel/BevelTool.cs`. The tool builders (the chain sweep, the partnered bead) make these;
//! the cutter and bead clips ([`super::bevel_bead_clip`], [`super::bevel_cap_clip`]) read them.

use std::rc::Rc;

use polygon_mesh::Mesh;

use super::bevel_cap_clip::ClippedCap;
use super::BevelEndCap;

/// C# `BevelTool`: one tool built for one run of edges, and which way the boolean has to take it:
/// a convex run makes a cutter to SUBTRACT, a concave run makes a bead to UNION (the plan's
/// section 3 table, both rows).
#[derive(Debug, Clone)]
pub struct BevelTool {
    /// The tool solid, in the source item's own local space.
    pub mesh: Mesh,
    /// Whether this is a bead to union on rather than a cutter to subtract.
    pub is_bead: bool,
    /// Whether this bead was already bounded where it was built, so that arriving with no void
    /// wedge is what it MEANT rather than a wedge that would not sweep.
    ///
    /// True only of a `BevelPartneredBead`, which is drawn station by station already lying where
    /// the finished part's material lies and has no clearance walls for a wedge to trim. Without
    /// this the builder reads its missing wedge as a failure and warns the user that a bead went on
    /// unclipped, which for this one is not true and not useful.
    pub bounded_at_sweep: bool,
    /// The void this bead is allowed to fill - the wedge between its two faces, swept along the
    /// same chain (`BevelSweptSection.VoidWedge`). `None` on a cutter, and on a bead whose wedge
    /// could not be swept, which leaves that bead unioned on whole as it was before the clip
    /// existed.
    pub void_wedge: Option<Mesh>,
    /// The tube the rolling ball leaves along this chain (`BevelSweptSection.Ball`) - the solid
    /// whose surface is this fillet - or `None` where nothing needs it. Only a CUTTER at a pair
    /// corner carries one, because that is the only place anything asks: see `BevelPairCornerTrim`.
    pub ball_sweep: Option<Mesh>,
    /// The vertices where this run overshoots into another selected run of its own convexity,
    /// from `BevelChainEnds.StartPairCorner`. `None` or empty on every run that does not - which is
    /// most of them - and it is what pairs two cutters up for the trim.
    pub pair_corners: Option<Vec<i32>>,
    /// The end caps this run's sweep could not carry, which the boolean has to take off the
    /// finished cutter instead ([`super::bevel_cap_clip`]). `None` on every tool whose caps were
    /// swept, which is all but the few whose two caps lean into each other over a short run.
    pub cap_clips: Option<Vec<ClippedCap>>,
    /// The void the CORNERS this run overshoots into open, one solid per 5.3 vertex
    /// (`BevelChainEnds.StartOvershootCorner`), which the clip admits alongside
    /// [`BevelTool::void_wedge`]. `None` on every run that ends at no such corner, which is most of
    /// them.
    ///
    /// It travels with the tool rather than with the wedge because it is a different KIND of
    /// claim. The wedge is what this run's own sweep opened and is bounded by the run; this is what
    /// the corner opens and is bounded by the ball round the vertex, shared with whichever other run
    /// met this one there (hence `Rc`: the corner pieces group beads by that identity). The two
    /// together are what the bead may keep past its own last vertex - see `BevelChainEnds.SquaredAt`
    /// for why the wedge alone was too generous and [`super::bevel_bead_clip`] for where the pair is
    /// used.
    pub corner_voids: Option<Vec<Rc<Mesh>>>,
    /// The same void wedge swept along the OVERSHOOT rather than squared off at this run's 5.3
    /// corners - the clip that shipped before [`BevelTool::corner_voids`] existed - or `None` on
    /// every run whose wedge was not squared in the first place.
    ///
    /// It exists because the keep-set the squared wedge belongs to is one the boolean kernel will
    /// sometimes not build, and the answer there has to be a clip rather than no clip: see
    /// `BevelCornerVoids` for the corners where that happens and why falling back to it is silent,
    /// and [`super::bevel_bead_clip`] for where the choice is made.
    pub fallback_wedge: Option<Mesh>,
    /// The faces this bead's run leaves the part through at an end it had to square off, as planes
    /// the clip holds it inside ([`super::bevel_bead_exit`]). `None` on a cutter and on every bead
    /// whose ends were capped or overshot, which is most of them.
    pub exit_planes: Option<Vec<BevelEndCap>>,
}

impl BevelTool {
    /// C#'s constructor with only its two required arguments; every optional one takes C#'s
    /// default (`None`, `false`). Set the public fields for the rest, as C#'s named arguments do.
    pub fn new(mesh: Mesh, is_bead: bool) -> Self {
        BevelTool {
            mesh,
            is_bead,
            bounded_at_sweep: false,
            void_wedge: None,
            ball_sweep: None,
            pair_corners: None,
            cap_clips: None,
            corner_voids: None,
            fallback_wedge: None,
            exit_planes: None,
        }
    }
}
