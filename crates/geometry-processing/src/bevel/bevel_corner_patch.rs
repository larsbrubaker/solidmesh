//! Plan 6's spherical corner, both ways up (`Bevel/BevelCornerPatch.cs`): one corner tool and the
//! groups meeting at it ([`BevelCornerPatch`]), every patch a selection asked for with where each
//! ball sits and what an unpatched corner has to say ([`BevelCornerPatchSet`]), and what has been
//! selected at one mesh vertex ([`Corner`], nested in the C# `static class BevelCornerPatches`,
//! whose functions land in this module with the core's corner-patch packages).
//!
//! Where three or more fillet chains of one radius meet at a vertex, the swept tools leave the
//! intersection of their cylinders where a rolling ball would leave a sphere: on an OUTSIDE corner
//! a tricylinder octant of material standing where a ball octant belongs, on an INSIDE one the
//! same shape of void left unfilled. A patch is the solid that settles the difference, to
//! subtract in the first case and to union on in the second.
//!
//! The set's tables are C# `Dictionary`s and `HashSet`s; they are [`IndexMap`]s and
//! [`IndexSet`]s here because `Collect` fills them in a fixed order and C# keeps it.

use std::collections::HashSet;

use indexmap::{IndexMap, IndexSet};
use polygon_mesh::Mesh;
use vector_math::Vector3;

use super::BevelAdvisory;
use crate::topology::FeatureEdge;

/// C# `BevelCornerPatch` (a readonly struct): one corner tool and the groups whose selections
/// meet at it. A patch that built is always used, even where one of those groups then failed to
/// sweep its chain: every chain ending here was laid on this patch's own plane, so withholding
/// the patch would leave them stopped short around a hole.
#[derive(Debug, Clone, Default)]
pub struct BevelCornerPatch {
    /// The corner solid, in the source item's own local space: one to SUBTRACT at an outside
    /// corner and one to UNION on at an inside one - see [`BevelCornerPatch::is_fill`].
    pub mesh: Mesh,
    /// The void this corner owes a FILL, as a solid to clip that fill against, or `None` where
    /// none could be built. Always `None` on a scoop, which takes material away.
    pub void_cone: Option<Mesh>,
    /// The groups whose selections meet at this corner. Also what decides whether the patch
    /// SURVIVES the second pass: `BevelBeadBatch` carries it as a fill's owners and takes the
    /// fill back whenever any one of these groups was re-beaded on the cut part.
    pub groups: Vec<i32>,
    /// Whether this solid FILLS an inside corner rather than scooping an outside one. A fill
    /// joins the BEADS rather than the cutters.
    pub is_fill: bool,
}

/// C# `BevelCornerPatchSet`: every corner patch a selection asked for, where each patched
/// corner's ball sits (which is where the chains ending there stop), and what any corner that
/// could not be patched has to say. `Default` is C#'s `BevelCornerPatchSet.Empty`.
#[derive(Debug, Clone, Default)]
pub struct BevelCornerPatchSet {
    /// The corner solids, one per patched vertex - to subtract or to union on as each says.
    pub patches: Vec<BevelCornerPatch>,
    /// By mesh vertex, where the rolling ball sits in each patched OUTSIDE corner. A convex chain
    /// ending at one of these stops on the plane through that point square to its own last edge.
    pub ball_centres: IndexMap<i32, Vector3>,
    /// The same for each filled INSIDE corner, where the ball sits out in the void. Kept apart so
    /// a chain reads only the corners its own convexity built.
    pub fill_ball_centres: IndexMap<i32, Vector3>,
    /// The warnings per group, in the resolutions' own order, or `None` for a group with no
    /// corner to complain about.
    pub warnings: Vec<Option<Vec<BevelAdvisory>>>,
    /// The patched vertices a run may overshoot straight THROUGH rather than stop on: the
    /// all-convex corners of exactly three runs (see `BevelChainEnds`).
    pub runs_may_drive_through: IndexSet<i32>,
    /// The points each patch put on each run's ball-centre plane, by (vertex, mesh edge) - the
    /// SEAM, which the run carries literally rather than computing its own arc there
    /// (`BevelSeamRing`). By the edge as well as the vertex because one corner hands a different
    /// arc to each run meeting at it. Both convexities share the map: a vertex cannot be both a
    /// patched outside corner and a filled inside one.
    pub seam_rings: IndexMap<(i32, i32), Vec<Vector3>>,
}

/// C# `BevelCornerPatches.Corner`: what has been selected at one mesh vertex.
#[derive(Debug, Clone, Default)]
pub struct Corner {
    /// The mesh vertex this corner is, which is how a chain end recognizes it.
    pub vertex: i32,
    /// Where that vertex is.
    pub position: Vector3,
    /// The selected feature edges at the vertex, kept sorted by mesh edge index so the face ring
    /// and its triangulation do not depend on selection order.
    pub edges: Vec<FeatureEdge>,
    /// The groups those edges were picked into, each once.
    pub groups: Vec<i32>,
    /// The mesh edges already taken in, so one edge picked into two groups is one edge. Only
    /// asked for membership, never iterated.
    pub mesh_edges: HashSet<i32>,
}
