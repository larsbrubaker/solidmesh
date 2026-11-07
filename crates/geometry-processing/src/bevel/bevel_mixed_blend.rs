//! One mixed corner's blend tool and the groups whose selections meet at it ([`BevelMixedBlend`]),
//! and every blend a selection asked for with the two cap decisions the chains ending at one of
//! them have to be told about before they are swept ([`BevelMixedBlendSet`]).
//!
//! The set's tables are keyed by (mesh vertex, mesh EDGE); they are [`IndexMap`]s because
//! `BevelMixedBlends` fills them in a fixed order that reaches the output - which is also what
//! [`super::bevel_blend_seam`] reads them as.

use indexmap::IndexMap;
use polygon_mesh::Mesh;
use vector_math::Vector3;

use super::{BevelAdvisory, BevelEndCap};

/// One mixed corner's blend tool and the groups whose selections meet at it. Like
/// `BevelCornerPatch` this is a solid to SUBTRACT, but it is subtracted after the beads have been
/// unioned on rather than with the other cutters - see the remarks on `BevelMixedBlends`.
#[derive(Debug, Clone)]
pub struct BevelMixedBlend {
    mesh: Mesh,
    groups: Vec<i32>,
    cap_face_reach: f64,
    void_face_reach: f64,
    runout_reach: f64,
}

impl BevelMixedBlend {
    /// A blend tool with its groups and its three reaches.
    pub fn new(
        mesh: Mesh,
        groups: Vec<i32>,
        cap_face_reach: f64,
        void_face_reach: f64,
        runout_reach: f64,
    ) -> Self {
        Self {
            mesh,
            groups,
            cap_face_reach,
            void_face_reach,
            runout_reach,
        }
    }

    /// The blend solid to SUBTRACT, in the source item's own local space.
    pub fn mesh(&self) -> &Mesh {
        &self.mesh
    }

    /// The groups whose selections meet at this corner - the group the cut was picked into and
    /// the group the bead was, which are usually two different rows of the property panel.
    pub fn groups(&self) -> &[i32] {
        &self.groups
    }

    /// How far across the CAP FACE this tool reaches, measured from the bead's own wall: the
    /// widest of its ARC stations' tangent distances. Square to the face that is exactly the
    /// radius, which is what the convex chain's own setback already reserved there; off square it
    /// grows, and `BevelMixedBlends`' remarks are where that is measured against what
    /// `BevelFaceFit` checks.
    ///
    /// The arc's stations only; the runout's are deliberately left out. This number exists to be
    /// set against what the CONVEX chain's setback reserved across the cap face, and that
    /// reservation is walked from the convex edge - which the arc's footprint lies across the face
    /// from, and the runout's does not. Past the arc the footprint has swung right round onto the
    /// OTHER wall's rim, where the reaches that describe it are [`Self::void_face_reach`] and
    /// [`Self::runout_reach`] instead.
    pub fn cap_face_reach(&self) -> f64 {
        self.cap_face_reach
    }

    /// How far past the VOID face - the other wall's own inner face, the one the bead is tangent
    /// to where its arc ends - this tool reaches, into that wall's material.
    ///
    /// The tool has always reached there: where the bead's rim meets the other wall, the blend's
    /// last arc ring stands from the axis by the wall's own radius plus the ball's, and the axis is
    /// a radius from that face by construction, so the ring pokes about `r` through it. That is the
    /// mouth of the cut, and it is correct - it is where the rolling ball is. What it is NOT is
    /// reserved: nothing walks that wall, because it is not a selected edge's face. Published so
    /// the fit check can learn it; see `BevelFaceFit`'s known limit.
    pub fn void_face_reach(&self) -> f64 {
        self.void_face_reach
    }

    /// How far ALONG the other wall's unselected rim, from the end of the bead's arc, the runout
    /// carries - zero at a three-edge corner, where a second selected cut takes the far end
    /// instead.
    ///
    /// `sqrt(reach^2 - r^2)` for the tool's own reach from the axis, which square to the cap face
    /// is a hair over `r sqrt 3`. See `BevelMixedBlendTool` for where the runout comes from and
    /// why it ends where it does.
    pub fn runout_reach(&self) -> f64 {
        self.runout_reach
    }
}

/// Every mixed corner blend a selection asked for, and the two cap decisions the chains ending at
/// one of them have to be told about before they are swept.
#[derive(Debug, Clone)]
pub struct BevelMixedBlendSet {
    blends: Vec<BevelMixedBlend>,
    convex_handovers: IndexMap<(i32, i32), BevelEndCap>,
    concave_cap_normals: IndexMap<(i32, i32), Vector3>,
    warnings: Vec<Option<Vec<BevelAdvisory>>>,
}

impl BevelMixedBlendSet {
    /// A set of blends with the handovers and cap normals they publish and the warnings per
    /// group.
    pub fn new(
        blends: Vec<BevelMixedBlend>,
        convex_handovers: IndexMap<(i32, i32), BevelEndCap>,
        concave_cap_normals: IndexMap<(i32, i32), Vector3>,
        warnings: Vec<Option<Vec<BevelAdvisory>>>,
    ) -> Self {
        Self {
            blends,
            convex_handovers,
            concave_cap_normals,
            warnings,
        }
    }

    /// The blend solids to SUBTRACT, one per blended vertex.
    pub fn blends(&self) -> &[BevelMixedBlend] {
        &self.blends
    }

    /// By (mesh vertex, mesh EDGE), the PLANE the blend's own ring at that edge's end of the rim
    /// is drawn in - the plane through the concave edge's rolling-ball axis spanned by that axis
    /// and the direction that station leaves it by. The CONVEX chain ending there stops on exactly
    /// this plane, so the two solids meet with coplanar faces at any lean.
    ///
    /// Keyed by the EDGE as well as the vertex because a three-edge mixed corner has two convex
    /// runs ending at one vertex and they stop on different planes - the two ends of the one rim
    /// the blend follows.
    ///
    /// A corner patch hands over a POINT instead (`BevelCornerPatchSet.BallCentres`) and the chain
    /// squares itself to its own last edge through it, because a patch's wall really is square to
    /// every chain that reaches it. A blend's first ring is not: it is laid in the miter plane the
    /// bead's first chord presents, which is square to the run only where the concave edge runs
    /// square to the cap face. So the blend publishes the plane it actually used rather than a
    /// point for the chain to guess it from. The normal is unoriented; `BevelChainEnds` flips it to
    /// point the way its own sweep leaves the end.
    pub fn convex_handovers(&self) -> &IndexMap<(i32, i32), BevelEndCap> {
        &self.convex_handovers
    }

    /// By (mesh vertex, mesh EDGE), the outward normal of the face the CONCAVE chain ending there
    /// has to be capped on - the face the convex fillet dies on, which is where the blend reads the
    /// bead's rim from. The surface rules would nearly always pick that face anyway; saying it out
    /// loud is what makes the blend's geometry provable rather than lucky.
    ///
    /// Keyed by the edge for the mirror of [`Self::convex_handovers`]' reason: a three-edge mixed
    /// corner of one cut and two beads has two concave runs ending at one vertex, and each of them
    /// dies on its own face.
    pub fn concave_cap_normals(&self) -> &IndexMap<(i32, i32), Vector3> {
        &self.concave_cap_normals
    }

    /// The warnings per group, in the resolutions' own order, or `None` for a group with no corner
    /// to complain about.
    pub fn warnings(&self) -> &[Option<Vec<BevelAdvisory>>] {
        &self.warnings
    }
}
