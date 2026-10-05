//! Trimming a concave run's bead to the part plus the voids it may fill (`Bevel/BevelBeadClip.cs`,
//! a C# `internal static class`, so a module here): the per-rebuild count of which route each
//! chain's trim took ([`Tally`]) and one chain's bead with everything it may occupy
//! ([`Clippable`]). The trims themselves land with the core's bead-clip packages as free
//! functions in this module; `BevelBeadBatch` is what gathers the [`Clippable`]s.

use std::rc::Rc;

use polygon_mesh::csg::{BooleanError, CsgModes};
use polygon_mesh::Mesh;
use vector_math::Matrix4X4;

use super::level_line::LevelLineWork;
use super::{bevel_bead_exit, bevel_source_hull, bevel_working_mesh, BevelEndCap};

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
    /// Shared (`Rc`): every bead meeting at a corner holds the SAME void, and the corner pieces
    /// group the beads by that identity (C# reads it by reference), never by geometry.
    pub corner_voids: Option<Vec<Rc<Mesh>>>,
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
        corner_voids: Option<Vec<Rc<Mesh>>>,
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

/// The combiner's answer as C# reads it: a cancel passes up, any other refusal (C#'s caught
/// `InvalidOperationException`s: a partial union, a rejected import, a kernel error) is no mesh.
fn combined(
    combine: &CombineMeshes<'_>,
    operands: &[(Mesh, Matrix4X4)],
    mode: CsgModes,
) -> Result<Option<Mesh>, BooleanError> {
    match combine(operands, mode) {
        Ok(mesh) => Ok(Some(mesh)),
        Err(BooleanError::Cancelled) => Err(BooleanError::Cancelled),
        Err(_) => Ok(None),
    }
}

/// The pieces as one solid: a lone piece as it is, several by a union, `None` where the kernel
/// refuses the union or its answer is no usable operand (C# `JoinAsync`).
pub fn join(pieces: Vec<Mesh>, combine: &CombineMeshes<'_>) -> Result<Option<Mesh>, BooleanError> {
    if pieces.len() == 1 {
        return Ok(pieces.into_iter().next());
    }

    let operands: Vec<(Mesh, Matrix4X4)> = pieces
        .into_iter()
        .map(|piece| (piece, Matrix4X4::IDENTITY))
        .collect();
    let joined = combined(combine, &operands, CsgModes::Union)?;
    Ok(joined.filter(|mesh| bevel_working_mesh::is_usable_operand(Some(mesh), None)))
}

/// The bead trimmed to the part and the voids it may fill, bounded by the part's own padded hull
/// built here (C# `ClipToAsync`; `exit_planes` is C#'s optional, `None` by default).
#[allow(clippy::too_many_arguments)] // C#'s signature.
pub fn clip_to(
    working_mesh: &Mesh,
    item_matrix: Matrix4X4,
    bead: &Mesh,
    voids: &[Mesh],
    combine: &CombineMeshes<'_>,
    exit_planes: Option<&[BevelEndCap]>,
) -> Result<Option<Mesh>, BooleanError> {
    let hull = bevel_source_hull::build(Some(working_mesh));
    clip_to_bounded(
        working_mesh,
        item_matrix,
        hull.as_ref(),
        bead,
        voids,
        combine,
        exit_planes,
    )
}

/// The bead trimmed to the KEEP SET - the part unioned with the voids the bead may fill - and to
/// the source hull and exit planes; `None` where the kernel will not build the keep set or it is
/// unusable (C# `ClipToBoundedAsync`).
#[allow(clippy::too_many_arguments)] // C#'s signature.
pub fn clip_to_bounded(
    working_mesh: &Mesh,
    item_matrix: Matrix4X4,
    source_hull: Option<&Mesh>,
    bead: &Mesh,
    voids: &[Mesh],
    combine: &CombineMeshes<'_>,
    exit_planes: Option<&[BevelEndCap]>,
) -> Result<Option<Mesh>, BooleanError> {
    let mut keepable = Vec::with_capacity(1 + voids.len());
    keepable.push((working_mesh.clone(), item_matrix));
    for may in voids {
        keepable.push((may.clone(), item_matrix));
    }

    let Some(clip) = combined(combine, &keepable, CsgModes::Union)? else {
        return Ok(None);
    };

    if !is_usable_keep_set(Some(&clip)) {
        return Ok(None);
    }

    intersect(
        bead,
        item_matrix,
        source_hull,
        &clip,
        Matrix4X4::IDENTITY,
        combine,
        exit_planes,
    )
}

/// The solid intersected with the keep set, with the source hull where the solid reaches past it,
/// and with the inside of each exit plane; `None` where the kernel refuses or the answer is no
/// usable operand (C# `IntersectAsync`).
#[allow(clippy::too_many_arguments)] // C#'s signature.
pub fn intersect(
    solid: &Mesh,
    solid_matrix: Matrix4X4,
    source_hull: Option<&Mesh>,
    keep: &Mesh,
    keep_matrix: Matrix4X4,
    combine: &CombineMeshes<'_>,
    exit_planes: Option<&[BevelEndCap]>,
) -> Result<Option<Mesh>, BooleanError> {
    let mut operands = vec![(solid.clone(), solid_matrix), (keep.clone(), keep_matrix)];

    if !bevel_source_hull::lies_inside(source_hull, Some(solid)) {
        // Only where it matters: a solid already inside the hull would gain an operand and nothing
        // else.
        if let Some(hull) = source_hull {
            operands.push((hull.clone(), solid_matrix));
        }
    }

    if let Some(exit_planes) = exit_planes {
        let bounds = solid.get_axis_aligned_bounding_box();
        for plane in exit_planes {
            operands.push((bevel_bead_exit::inside(plane, &bounds), solid_matrix));
        }
    }

    let trimmed = combined(combine, &operands, CsgModes::Intersect)?;
    Ok(trimmed.filter(|mesh| bevel_working_mesh::is_usable_operand(Some(mesh), None)))
}

/// Caps every bead going on whole to the source hull where it reaches past it, in place: the
/// intersect runs in the operation's space (`item_matrix`) and the result is carried back to the
/// source item's. A bead already inside the bound - and the part itself, on the callers that put it
/// in this list beside the beads - is left alone, as is one the kernel will not cap. A cancel of
/// the build or of a boolean passes up.
pub fn cap_to_the_hull(
    item_matrix: Matrix4X4,
    source_hull: Option<&Mesh>,
    beads_going_on_whole: &mut [Mesh],
    combine: &CombineMeshes<'_>,
) -> Result<(), BooleanError> {
    let Some(source_hull) = source_hull else {
        return Ok(());
    };
    if beads_going_on_whole.is_empty() {
        return Ok(());
    }

    let back_to_the_source = item_matrix.inverted();
    for bead in beads_going_on_whole.iter_mut() {
        LevelLineWork::poll_cancel().map_err(|_| BooleanError::Cancelled)?;

        if bevel_source_hull::lies_inside(Some(source_hull), Some(bead)) {
            continue;
        }

        let operands = [
            (bead.clone(), item_matrix),
            (source_hull.clone(), item_matrix),
        ];
        let Some(mut capped) = combined(combine, &operands, CsgModes::Intersect)? else {
            continue;
        };

        if !bevel_working_mesh::is_usable_operand(Some(&capped), None) {
            continue;
        }

        capped.transform(back_to_the_source);
        *bead = capped;
    }

    Ok(())
}
