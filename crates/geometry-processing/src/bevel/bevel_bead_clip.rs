//! Trimming a concave run's bead to the part plus the voids it may fill (`Bevel/BevelBeadClip.cs`,
//! a C# `internal static class`, so a module here): the per-rebuild count of which route each
//! chain's trim took ([`Tally`]) and one chain's bead with everything it may occupy
//! ([`Clippable`]). The trims themselves land with the core's bead-clip packages as free
//! functions in this module; `BevelBeadBatch` is what gathers the [`Clippable`]s.

use polygon_mesh::csg::{BooleanError, CsgModes};
use polygon_mesh::Mesh;
use vector_math::Matrix4X4;

use super::BevelEndCap;

/// The rebuild's batched boolean the bead and cap clips run through (C#'s injected
/// `Func<List<(Mesh, Matrix4X4)>, CsgModes, Task<Mesh>> combineAsync`): synchronous, inside the
/// bevel's compute job, per the bevel-core decision; cancellation comes back as
/// [`BooleanError::Cancelled`].
pub type CombineMeshes<'a> =
    dyn Fn(&[(Mesh, Matrix4X4)], CsgModes) -> Result<Mesh, BooleanError> + 'a;

/// C# `BevelBeadClip.Tally`: which chain's bead was actually trimmed to over one rebuild - the
/// corner's, its own wedge's, or none at all.
///
/// Kept because the difference is invisible in the result and matters: a chain that falls back
/// to the wedge alone produces exactly the shape that shipped before the corner void existed.
/// Counted rather than listed because nothing needs to know WHICH chain took which route - a
/// rebuild leaves it for a test to read (`BevelFeatureObject3D.BeadClips`).
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub struct Tally {
    /// How many chains were trimmed to `part + wedge + the corner's void` - the widened
    /// keep-set, which only a chain that reaches a 5.3 corner asks for at all.
    pub to_the_corner: i32,
    /// How many were trimmed to their own wedge and the part alone: every chain with no corner
    /// to reach, plus any whose corner pieces the kernel refused.
    pub to_the_wedge: i32,
    /// How many went on with no trim at all, which is what
    /// `BevelRebuildAdvisories.BeadWentOnUnclipped` says out loud.
    pub unclipped: i32,
}

/// C# `BevelBeadClip.Clippable`: one chain's bead and everything it may fill - its own void
/// wedge, the void of each 5.3 corner it overshoots into, and the wedge to fall back to when the
/// kernel will not build the keep-set those corners belong to.
///
/// A named shape rather than a tuple because the corner voids and the fallback wedge only make
/// sense together: a fallback wedge exists exactly where corner voids do.
#[derive(Debug, Clone)]
pub struct Clippable {
    /// The bead to trim, in the source item's own space.
    pub bead: Mesh,
    /// The void its own chain swept, in the same space - squared off at its 5.3 corners where
    /// [`Clippable::corner_voids`] is `Some`.
    pub wedge: Mesh,
    /// The void of each 5.3 corner this chain overshoots into, or `None` where it reaches none.
    pub corner_voids: Option<Vec<Mesh>>,
    /// The same wedge swept along the overshoot instead of squared (`BevelTool.FallbackWedge`),
    /// or `None` where there was nothing to square.
    pub fallback_wedge: Option<Mesh>,
    /// The faces this bead's run leaves the part through at an end it had to square off
    /// (`BevelBeadExit`), in the same space. Every keep-set is narrowed by them; `None` on a bead
    /// with no squared end, which is most of them.
    pub exit_planes: Option<Vec<BevelEndCap>>,
}

impl Tally {
    /// C#'s constructor.
    pub fn new(to_the_corner: i32, to_the_wedge: i32, unclipped: i32) -> Self {
        Tally {
            to_the_corner,
            to_the_wedge,
            unclipped,
        }
    }
}

impl Clippable {
    /// C#'s constructor; the optional corner voids, fallback wedge and exit planes are `None` for
    /// C#'s defaults.
    pub fn new(
        bead: Mesh,
        wedge: Mesh,
        corner_voids: Option<Vec<Mesh>>,
        fallback_wedge: Option<Mesh>,
        exit_planes: Option<Vec<BevelEndCap>>,
    ) -> Self {
        Clippable {
            bead,
            wedge,
            corner_voids,
            fallback_wedge,
            exit_planes,
        }
    }
}

/// Whether a keep set is worth booleaning with: present, not empty, and manifold.
pub fn is_usable_keep_set(keep: Option<&Mesh>) -> bool {
    keep.is_some_and(|keep| !keep.vertices.is_empty() && keep.is_manifold())
}
