//! One tangent line read as the half space a face region is kept in ([`SurfaceFacetCut`]). Ports
//! the fields of `Bevel/Surface/SurfaceFacetCut.cs`; its constructors and its clipping and
//! removal readings land with the walkback package as further `impl` blocks.

use std::collections::HashSet;
use std::rc::Rc;

use vector_math::Vector3;

use super::{SurfaceBandEnd, SurfaceLine};

/// C# `SurfaceFacetCut` (a readonly struct): one tangent line read as the half space a face region
/// is kept in - the plane that stands square to the region through the tangent line, with the
/// material side named.
///
/// A PLANE rather than a 2D line on a projected polygon, and that is the phase's one real decision.
/// The plan says a face is walked back TO a line lying in it; cutting the face's own triangles by
/// the plane through that line gives the same answer where the region is planar - the plane meets
/// the face plane in exactly the tangent line - and needs no frame, no projection and no mapping
/// back, so the vertices it makes are on the tangent line to the last bit rather than to the
/// accuracy of a round trip. Where a "planar" region is slightly warped (the fixture
/// `WarpedWallTestMeshes`) it degrades the way one would want: each facet is cut in its own plane
/// and the cut stays ON the surface, where a mean-plane projection would land the line off the
/// facets. What it does NOT do is bend round a curved region, which is phase 5's business.
///
/// The half space is infinite, which is what makes an open chain's tangent line run "from rim to
/// rim": one picked edge of a cube top walks the WHOLE top face back by the setback, not just the
/// part over the edge. What bounds it is not the plane but the MATERIAL: the walkback applies a cut
/// only to the facets its own edge's band actually reaches across the region
/// ([`SurfaceFacetCut::seed_face`]), so a notch bitten out of the region's side stops the cut dead
/// rather than letting it take a second band off the far side of the notch - a band no strip is
/// going to fill, and so a hole.
#[derive(Debug, Clone, PartialEq, Default)]
pub struct SurfaceFacetCut {
    /// The planes that bound the band along the edge, none of them on a chain that runs out to the
    /// region's own rim at both ends ([`SurfaceBandEnd`]).
    pub ends: Vec<SurfaceBandEnd>,
    /// Which PLAN this cut was made for, as its place in the plans handed to the walkback, and -1
    /// for a cut with no chain behind it (`SurfaceEndCut`).
    ///
    /// One region holds the cuts of every plan that reaches it, and the standing-feature guard's
    /// chain exemption is a thing one plan may claim only of its OWN chain
    /// (`SurfaceWalkbackBands.TryCheckNothingIsStandingInTheBand`): a boss's foot being bevelled
    /// excuses the boss's own band, not a rim edge's band that happens to run into the same foot.
    /// [`SurfaceFacetCut::edge_index`] cannot answer it, since that numbers the plans' edges end to
    /// end and says nothing about where one plan stops.
    pub plan_index: i32,
    /// Which edge of the plan this cut belongs to, as an index into its edges.
    pub edge_index: i32,
    /// Whether it is the left face of the walk that is being cut.
    pub left: bool,
    /// The line in the face the walkback cuts to.
    pub tangent_line: SurfaceLine,
    /// The unit direction, lying in the face and square to the chain edge, that points away from
    /// the edge into the face - the side of the cut the region is KEPT on.
    pub inward: Vector3,
    /// The region's own facet that the chain edge runs along - where the band this cut takes off
    /// starts. The walkback spreads the cut out from here through the material it can reach and no
    /// further, which is what keeps one edge's band from reappearing across a notch.
    pub seed_face: i32,
    /// The only facets of the region this cut may take material off, or `None` to let the band
    /// spread through whatever material it can reach - which is what every cut between two planes
    /// does. Shared with the edge plan's band it was read from, so held as `Rc`.
    ///
    /// A cut is written as a half space and nothing else, and that is right wherever the region is
    /// flat: the band is then bounded by the MATERIAL it can reach. On a wall rolled along, it is
    /// not - the half space through the foot with the touched facet's inward direction also
    /// swallows the facets coming back round the far side of a wall that turns more than half a
    /// circle, and those are joined to the band by material all the way round. So the roll names
    /// the facets its own sections paced out, and the band's flood is kept inside them
    /// (`SurfaceWalkbackBands.LimitEachCutToTheBandItsOwnEdgeReaches`).
    pub may_touch: Option<Rc<HashSet<i32>>>,
    /// The two vertices the chord this cut is drawn along runs between, where it was drawn along
    /// one the construction had already stood, or `None`. A ring corner that IS one of them stands
    /// on the cut by identity (`SurfaceCutClip.ClipBy`).
    pub the_cuts_own_ends: Option<Vec<i32>>,
    /// Where the chain edge this cut belongs to begins, in the mesh's own space.
    pub chain_from: Vector3,
    /// Where it ends. The same point as [`SurfaceFacetCut::chain_from`] for a cut that has no chain
    /// edge behind it, which is the end cut's own (`SurfaceEndCut`).
    pub chain_to: Vector3,
}
