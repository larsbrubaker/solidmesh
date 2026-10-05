//! One standing feature the walkback held BACK rather than carrying over
//! ([`SurfaceStandingFeatureWithheld`]) and its withheld faces
//! ([`SurfaceStandingFeatureWithheldFace`]). Ports the fields of
//! `Bevel/Surface/SurfaceStandingFeatureWithheld.cs`; the ridge and crease readings land with the
//! standing-feature package as further `impl` blocks.
//!
//! The walkback builds these and then keeps filling them in through shared handles - the merge of
//! two bands that reached one feature appends to its reaches and its faces' creases, and the
//! rewrite gathers each face's rings facet by facet through a by-facet index before taking its
//! outline - while a notch names the faces that cut its rails. So both are held as `Rc`, and every
//! part C# mutates after construction is interior-mutable.

use std::cell::RefCell;
use std::rc::Rc;

use vector_math::Vector3;

/// C# `SurfaceStandingFeatureWithheld`: one standing feature the walkback held BACK rather than
/// carrying over - its own FACES (whole coplanar regions, not single triangles), the creases they
/// lost their footing on, the plane each stands in, and which band undercut them.
///
/// Held back because those faces are not finished. The band took the material under their creases,
/// so each of them has to be EXTENDED through the removed corner until it meets the fillet cylinder
/// (`SurfaceStandingFeatureFaces`) - Lars's ruling, and what the Onshape control builds. Carrying
/// them over as they are and capping the gap flat, which is what the earlier island did, puts a
/// floor where the control has a wall.
///
/// The vertex indices here are the pool's while the walkback is building and the MESH's afterwards:
/// `SurfaceWalkbackRewrite.Assemble` renumbers them along with the tangent runs, since a region
/// held back contributes no face and so has no vertex of its own in the mesh yet.
#[derive(Debug, Clone, PartialEq, Default)]
pub struct SurfaceStandingFeatureWithheld {
    /// Which chain edge's band, and which side of it (`(edge_index, left)`), undercut this
    /// feature - one entry per band that did, in the order they were merged.
    ///
    /// A LIST because one feature can stand under two bands of two different chains, which is the
    /// nose of the nose-in-notch part with both knife tips beveled: its ridge ends stand 7.071
    /// along each slope from that slope's tip edge, and a bead of 3.4 reaches 8.208. Each band
    /// uncovers its own end of the feature, and each face of the feature therefore loses its
    /// footing on TWO runs of crease, one per reach - so
    /// [`SurfaceStandingFeatureWithheldFace::creases`] and
    /// [`SurfaceStandingFeatureWithheldFace::withins`] are indexed by the same number this list is
    /// (`SurfaceFaceWalkback` merges them, and refuses where the two readings do not name the same
    /// faces).
    ///
    /// A feature under two bands of ONE region is a different shape and is still refused before it
    /// gets here: there the two bands uncover overlapping parts of one root, and closing them one
    /// at a time would lay the same triangle down twice (`SurfaceStandingFeatureStretch.TryRead`).
    pub reaches: RefCell<Vec<(i32, bool)>>,
    /// The landing region's outward normal, which the feature stands out of - the FIRST reach's,
    /// where two were merged, since the two bands land on two different regions. Nothing reads it
    /// today.
    pub landing: Vector3,
    /// The withheld faces, one per coplanar region of the feature the band undercut.
    pub faces: Vec<Rc<SurfaceStandingFeatureWithheldFace>>,
}

/// C# `SurfaceStandingFeatureWithheld.Face`: one withheld FACE of the feature - the whole coplanar
/// region, not one triangle of it: the mesh facets it is cut into, its outline, the crease run it
/// stands on, and the plane it lies in.
///
/// The region and not the facet, because the extension is a property of the FACE. A face fanned
/// from a point inside itself - what a boolean leaves - has only one of its triangles across the
/// crease, and withholding that one alone leaves its two coplanar neighbours standing on a ridge
/// end whose material has gone. The whole region goes out as one ring, which is also the only way
/// the old triangulation's diagonals stop being edges of the result.
#[derive(Debug, Clone, PartialEq, Default)]
pub struct SurfaceStandingFeatureWithheldFace {
    /// The region's facets in the source mesh, which name it while the walkback runs.
    pub mesh_faces: Vec<i32>,
    /// The ring of each of those facets as the walkback read it, carrying any crossing put on its
    /// edges - gathered here only so that [`SurfaceStandingFeatureWithheldFace::ring`] can be taken
    /// from them.
    pub rings: RefCell<Vec<Vec<i32>>>,
    /// The region's own outline, which is the boundary of the union of its facets; `None` until the
    /// walkback has taken it.
    pub ring: RefCell<Option<Vec<i32>>>,
    /// The ENDS of the crease run this face lost its footing on (`(from, to)`) - one on the
    /// feature's ridge, one where the run dies - one entry per reach, indexed as
    /// [`SurfaceStandingFeatureWithheld::reaches`] is.
    pub creases: RefCell<Vec<(i32, i32)>>,
    /// The points of each crease run BETWEEN those two ends, which a face whose root line is cut
    /// into several mesh edges has. They go the way the crease does: interior to the extension.
    pub withins: RefCell<Vec<Vec<i32>>>,
    /// A point on the face's own plane.
    pub plane_point: Vector3,
    /// The face's outward normal, which the extension is fanned to face along.
    pub plane_normal: Vector3,
}
