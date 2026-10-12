//! What the face walkback (`SurfaceFaceWalkback`) hands the strip builder: the mesh with every
//! touched face region walked back to its tangent lines, and the vertices standing on each of
//! those lines ([`SurfaceWalkbackResult`]). Ports `Bevel/Surface/SurfaceWalkbackResult.cs`.
//!
//! The mesh is deliberately OPEN. Where the strip goes there is now a gap, bounded by the tangent
//! lines and by whatever the walkback left of the faces the chain ran out onto, and phase 3 closes
//! it by sewing to the very vertices [`SurfaceWalkbackResult::tangent_runs`] names - no new points
//! along the seam, so the coincident-surface problem a boolean has there cannot arise.
//!
//! The three per-cut tables are C# `IReadOnlyDictionary`s keyed by (edge index in the plan's
//! `Edges`, left side); they are [`IndexMap`]s here so that any walk over them keeps the order the
//! walkback filled them in, as C#'s `Dictionary` does.

use std::rc::Rc;
use std::sync::Arc;

use indexmap::IndexMap;
use polygon_mesh::Mesh;

use super::SurfaceStandingFeatureWithheld;

/// One per-cut table: for each (edge index, left side), contiguous runs of vertex indices.
pub type SurfaceWalkbackRuns = IndexMap<(i32, bool), Vec<Vec<i32>>>;

/// C# `SurfaceWalkbackResult`.
#[derive(Debug, Clone)]
pub struct SurfaceWalkbackResult {
    /// The rewritten mesh. A NEW mesh: the one walked back is not touched, and no index of this
    /// one need mean what it meant there, since unused vertices are dropped as the faces are
    /// gathered.
    pub mesh: Arc<Mesh>,
    /// What each tangent line was left of as a boundary of the face it cut, as one or more
    /// contiguous RUNS of vertices, each in the chain's own walk order along the line (From to
    /// To), and the runs themselves in that order too. The left face's tangent line and the right
    /// face's are separate entries, as the strip reads them.
    ///
    /// One run on a plain face; MORE than one where the line leaves the face and comes back - a
    /// notch bitten out of the region's side, or a bore the band runs either side of. Those are
    /// genuinely separate boundaries with no face between them, and the strip has to sew to the
    /// one its own chain edge stands over rather than to their concatenation. Every vertex of a
    /// run is a vertex of the walked-back region's own triangles and every consecutive pair of
    /// them has an edge of one between, so sewing to a run cannot leave a T junction.
    pub tangent_runs: SurfaceWalkbackRuns,
    /// For each of the same keys, the stretches of that cut's rim standing on a rail it BORROWED
    /// from a sibling rather than on its own tangent line - empty for every cut that borrowed
    /// none. Where a band reaches across a crease its outline is drawn on the facet's own rail
    /// (`SurfaceBandOutline.AsDrawnOn`), so a mitre at the end of such a cut has to meet the rim
    /// where the walkback really left it (`SurfaceWalkbackBands.ContinuationsAlong`).
    pub rail_continuations: SurfaceWalkbackRuns,
    /// For each of the same keys, the cut's own BAND CORNERS: the vertices the tangent clip left
    /// standing on one of that cut's end planes, as one list per end in `SurfaceFacetCut.Ends`
    /// order. A corner is where the band's tangent line meets its end plane AS THE BAND WAS
    /// REALLY DRAWN, which on a facet across a crease is the BORROWED rail; the corner published
    /// here is that vertex itself, by identity, so a mitre meets it rather than minting a twin
    /// (`SurfaceMitreSliver`'s foot overrun).
    pub band_corners: SurfaceWalkbackRuns,
    /// The features the bands passed UNDER, whose own facets were held back rather than carried
    /// over: each is finished by extending it through the corner the band took, once the strip
    /// it has to meet is built (`SurfaceStandingFeatureFaces`). Empty where no band passed under
    /// anything.
    pub standing_features: Vec<Rc<SurfaceStandingFeatureWithheld>>,
}

impl SurfaceWalkbackResult {
    /// C#'s constructor: `rail_continuations` and `band_corners` default to empty tables.
    pub fn new(
        mesh: Arc<Mesh>,
        tangent_runs: SurfaceWalkbackRuns,
        standing_features: Vec<Rc<SurfaceStandingFeatureWithheld>>,
        rail_continuations: Option<SurfaceWalkbackRuns>,
        band_corners: Option<SurfaceWalkbackRuns>,
    ) -> Self {
        SurfaceWalkbackResult {
            mesh,
            tangent_runs,
            standing_features,
            rail_continuations: rail_continuations.unwrap_or_default(),
            band_corners: band_corners.unwrap_or_default(),
        }
    }
}
