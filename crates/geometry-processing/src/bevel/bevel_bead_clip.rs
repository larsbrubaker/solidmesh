//! Trimming a concave run's bead to the part plus the voids it may fill (`Bevel/BevelBeadClip.cs`,
//! a C# `internal static class`, so a module here): the per-rebuild count of which route each
//! chain's trim took ([`Tally`]) and one chain's bead with everything it may occupy
//! ([`Clippable`]), the trims, and the whole clip ([`apply_all`]); `BevelBeadBatch` is what
//! gathers the [`Clippable`]s.
//!
//! C# asks whether a bead is already going on whole by REFERENCE (`AlreadyGoingOnWhole`); the
//! beads here are owned values, so the driver keeps the index of each clippable it sent on whole in
//! the first pass and asks that instead ([`already_going_on_whole`]).

use std::rc::Rc;

use polygon_mesh::csg::{BooleanError, CsgModes};
use polygon_mesh::Mesh;
use vector_math::Matrix4X4;

use super::level_line::LevelLineWork;
use super::{
    bevel_bead_corner_pieces, bevel_bead_exit, bevel_source_hull, bevel_working_mesh, BevelEndCap,
    BevelRebuildAdvisories,
};

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

/// C# `ApplyAllAsync`: every clippable bead trimmed (corners first, then each bead to its corners,
/// its own corner pieces or its wedge, and the batch joined - or, where the batch will not join,
/// every bead again without its corners), then every bead going on whole capped to the part's
/// padded hull. Returns the clipped meshes and the tally. A cancel passes up.
#[allow(clippy::too_many_arguments)]
pub fn apply_all(
    working_mesh: &Mesh,
    item_matrix: Matrix4X4,
    clippable_beads: &[Clippable],
    beads_going_on_whole: &mut Vec<Mesh>,
    advisories: &mut BevelRebuildAdvisories,
    combine: &CombineMeshes<'_>,
) -> Result<(Vec<Mesh>, Tally), BooleanError> {
    // Once per rebuild: every keep set is a void solid that can run out past the end of the part,
    // and this is the one bound that cannot.
    let source_hull = bevel_source_hull::build(Some(working_mesh));

    let applied = apply_each(
        working_mesh,
        item_matrix,
        source_hull.as_ref(),
        clippable_beads,
        beads_going_on_whole,
        advisories,
        combine,
    )?;

    // And the beads that ended up going on WITH NO TRIM are capped to it too.
    cap_to_the_hull(
        item_matrix,
        source_hull.as_ref(),
        beads_going_on_whole,
        combine,
    )?;

    Ok(applied)
}

/// C# `ApplyEachAsync`: the corners' shared pieces first, then each bead trimmed to its corners
/// (where it kept them), to its own corner pieces, or to its wedge - and where none of those go,
/// sent on whole (and advised once); the batch joined into one solid where any corner was kept, or
/// every bead trimmed again without corners where the batch will not join.
#[allow(clippy::too_many_arguments)]
pub fn apply_each(
    working_mesh: &Mesh,
    item_matrix: Matrix4X4,
    source_hull: Option<&Mesh>,
    clippable_beads: &[Clippable],
    beads_going_on_whole: &mut Vec<Mesh>,
    advisories: &mut BevelRebuildAdvisories,
    combine: &CombineMeshes<'_>,
) -> Result<(Vec<Mesh>, Tally), BooleanError> {
    let mut clipped = Vec::with_capacity(clippable_beads.len());

    // The corners first: whether a corner's piece could be built decides each bead's wedge.
    let refused_corners = bevel_bead_corner_pieces::apply_corners(
        working_mesh,
        item_matrix,
        source_hull,
        clippable_beads,
        &mut clipped,
        combine,
    )?;
    let corner_pieces = clipped.len();

    let mut to_the_corner = 0;
    let mut to_the_wedge = 0;
    let mut unclipped = 0;
    let mut went_on_whole = vec![false; clippable_beads.len()];

    for (index, clippable) in clippable_beads.iter().enumerate() {
        LevelLineWork::poll_cancel().map_err(|_| BooleanError::Cancelled)?;

        let exit_planes = clippable.exit_planes.as_deref();
        let has_corners = clippable
            .corner_voids
            .as_ref()
            .is_some_and(|voids| !voids.is_empty());
        let mut kept_its_corners =
            has_corners && !bevel_bead_corner_pieces::lost_a_corner(clippable, &refused_corners);

        let mut trimmed = if kept_its_corners {
            clip_to_bounded(
                working_mesh,
                item_matrix,
                source_hull,
                &clippable.bead,
                std::slice::from_ref(&clippable.wedge),
                combine,
                exit_planes,
            )?
        } else {
            None
        };

        if !kept_its_corners && has_corners {
            // Its corners are ones no shared piece could be made of: it takes copies of its own.
            let pieces = bevel_bead_corner_pieces::own_corner_pieces(
                working_mesh,
                item_matrix,
                source_hull,
                clippable,
                combine,
            )?;
            trimmed = match pieces {
                None => None,
                Some(pieces) => join(pieces, combine)?,
            };
            kept_its_corners = trimmed.is_some();
        }

        if trimmed.is_none() {
            // Trimmed to the wedge it WOULD have swept rather than the one squared off at the vertex.
            let wedge = if has_corners {
                clippable
                    .fallback_wedge
                    .as_ref()
                    .unwrap_or(&clippable.wedge)
            } else {
                &clippable.wedge
            };
            trimmed = clip_to_bounded(
                working_mesh,
                item_matrix,
                source_hull,
                &clippable.bead,
                std::slice::from_ref(wedge),
                combine,
                exit_planes,
            )?;
            kept_its_corners = false;
        }

        if let Some(trimmed) = trimmed {
            clipped.push(trimmed);
            if kept_its_corners {
                to_the_corner += 1;
            } else {
                to_the_wedge += 1;
            }
            continue;
        }

        // The clip is a trim and not the feature: this chain's bead goes on as it did before.
        beads_going_on_whole.push(clippable.bead.clone());
        went_on_whole[index] = true;
        advisories.bead_went_on_unclipped();
        unclipped += 1;
    }

    if corner_pieces == 0 && to_the_corner == 0 {
        // Nothing in this batch is doing anything the clip did not do before the corner void.
        return Ok((clipped, Tally::new(to_the_corner, to_the_wedge, unclipped)));
    }

    if let Some(joined) = join(clipped, combine)? {
        return Ok((
            vec![joined],
            Tally::new(to_the_corner, to_the_wedge, unclipped),
        ));
    }

    // The net: the corner is given up and every bead that took one is trimmed again to the wedge
    // that shipped before it existed.
    let (second, gave_up) = without_corners(
        working_mesh,
        item_matrix,
        source_hull,
        clippable_beads,
        &went_on_whole,
        beads_going_on_whole,
        advisories,
        combine,
    )?;

    let count = second.len() as i32;
    Ok((second, Tally::new(0, count, unclipped + gave_up)))
}

/// C# `WithoutCornersAsync`: every bead not already going on whole trimmed to its fallback wedge
/// (or its wedge) alone; one the kernel will not trim goes on whole (and is advised once). Returns
/// the trimmed beads and how many gave up. `went_on_whole` names, by index, the clippables the
/// first pass already sent on whole (C#'s `AlreadyGoingOnWhole` by reference).
#[allow(clippy::too_many_arguments)]
pub fn without_corners(
    working_mesh: &Mesh,
    item_matrix: Matrix4X4,
    source_hull: Option<&Mesh>,
    clippable_beads: &[Clippable],
    went_on_whole: &[bool],
    beads_going_on_whole: &mut Vec<Mesh>,
    advisories: &mut BevelRebuildAdvisories,
    combine: &CombineMeshes<'_>,
) -> Result<(Vec<Mesh>, i32), BooleanError> {
    let mut clipped = Vec::with_capacity(clippable_beads.len());
    let mut gave_up = 0;

    for (index, clippable) in clippable_beads.iter().enumerate() {
        LevelLineWork::poll_cancel().map_err(|_| BooleanError::Cancelled)?;

        if already_going_on_whole(went_on_whole, index) {
            // Already given up on in the first pass and already advised about.
            continue;
        }

        let wedge = clippable
            .fallback_wedge
            .as_ref()
            .unwrap_or(&clippable.wedge);
        let trimmed = clip_to_bounded(
            working_mesh,
            item_matrix,
            source_hull,
            &clippable.bead,
            std::slice::from_ref(wedge),
            combine,
            clippable.exit_planes.as_deref(),
        )?;

        if let Some(trimmed) = trimmed {
            clipped.push(trimmed);
            continue;
        }

        beads_going_on_whole.push(clippable.bead.clone());
        advisories.bead_went_on_unclipped();
        gave_up += 1;
    }

    Ok((clipped, gave_up))
}

/// C# `AlreadyGoingOnWhole`: whether clippable `index`'s bead is already in the beads going on
/// whole - read off the indices the first pass sent there, where C# compares references.
pub fn already_going_on_whole(went_on_whole: &[bool], index: usize) -> bool {
    went_on_whole.get(index).copied().unwrap_or(false)
}
