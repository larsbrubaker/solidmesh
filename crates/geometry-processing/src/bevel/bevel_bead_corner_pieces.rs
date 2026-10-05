//! The pieces of beads that meet at a 5.3 corner. Ports `Bevel/BevelBeadCornerPieces.cs` (a C#
//! static class): the beads gathered by the corner void they share ([`beads_by_corner`]), the
//! corner pieces taken ONCE per corner over the union of the beads that share it
//! ([`apply_corners`], [`corner_piece`]) and one bead's piece over two voids ([`piece`],
//! [`intersect_piece`]).

use std::rc::Rc;

use polygon_mesh::csg::{BooleanError, CsgModes};
use polygon_mesh::Mesh;
use vector_math::Matrix4X4;

use super::bevel_bead_clip::{is_usable_keep_set, Clippable, CombineMeshes};
use super::level_line::LevelLineWork;
use super::{bevel_bead_exit, bevel_source_hull, bevel_working_mesh, BevelEndCap};

/// Each corner void the clippable beads reach, in first-met order, with the beads that reach it.
/// A void is ONE corner by identity (`Rc::ptr_eq`, C#'s reference equality): every bead meeting
/// at a corner holds the same void, and two corners never merge for having equal geometry.
pub fn beads_by_corner(clippable_beads: &[Clippable]) -> Vec<(Rc<Mesh>, Vec<&Mesh>)> {
    let mut order: Vec<(Rc<Mesh>, Vec<&Mesh>)> = Vec::new();
    for clippable in clippable_beads {
        let Some(corner_voids) = &clippable.corner_voids else {
            continue;
        };

        for corner in corner_voids {
            match order.iter_mut().find(|(seen, _)| Rc::ptr_eq(seen, corner)) {
                Some((_, beads)) => beads.push(&clippable.bead),
                None => order.push((Rc::clone(corner), vec![&clippable.bead])),
            }
        }
    }

    order
}

/// The bead's piece over two voids: the bead intersected with their union (the keep set), the
/// source hull where the bead reaches past it, and its exit planes; `None` where the kernel
/// refuses either step, the keep set is unusable or the piece comes back empty. A cancel passes up.
#[allow(clippy::too_many_arguments)] // C#'s signature.
pub fn piece(
    bead: &Mesh,
    item_matrix: Matrix4X4,
    source_hull: Option<&Mesh>,
    first: &Mesh,
    second: &Mesh,
    combine: &CombineMeshes<'_>,
    exit_planes: Option<&[BevelEndCap]>,
) -> Result<Option<Mesh>, BooleanError> {
    let keep = match combine(
        &[(first.clone(), item_matrix), (second.clone(), item_matrix)],
        CsgModes::Union,
    ) {
        Ok(keep) => keep,
        Err(BooleanError::Cancelled) => return Err(BooleanError::Cancelled),
        Err(_) => return Ok(None),
    };

    if !is_usable_keep_set(Some(&keep)) {
        return Ok(None);
    }

    let piece = match intersect_piece(bead, item_matrix, source_hull, &keep, combine, exit_planes) {
        Ok(piece) => piece,
        Err(BooleanError::Cancelled) => return Err(BooleanError::Cancelled),
        Err(_) => return Ok(None),
    };

    Ok((!piece.vertices.is_empty()).then_some(piece))
}

/// The bead intersected with a keep set, with the source hull (in the bead's own space) only where
/// the bead actually reaches past it - otherwise the operand is a no-op the kernel still has to
/// integrate - and with the inside of each exit plane. The combiner's answer, refusal included.
pub fn intersect_piece(
    bead: &Mesh,
    item_matrix: Matrix4X4,
    source_hull: Option<&Mesh>,
    keep: &Mesh,
    combine: &CombineMeshes<'_>,
    exit_planes: Option<&[BevelEndCap]>,
) -> Result<Mesh, BooleanError> {
    let mut operands = vec![
        (bead.clone(), item_matrix),
        (keep.clone(), Matrix4X4::IDENTITY),
    ];

    if !bevel_source_hull::lies_inside(source_hull, Some(bead)) {
        if let Some(hull) = source_hull {
            operands.push((hull.clone(), item_matrix));
        }
    }

    if let Some(exit_planes) = exit_planes {
        let bounds = bead.get_axis_aligned_bounding_box();
        for plane in exit_planes {
            operands.push((bevel_bead_exit::inside(plane, &bounds), item_matrix));
        }
    }

    combine(&operands, CsgModes::Intersect)
}

/// Takes each 5.3 corner's piece ONCE, over the union of the beads that reach it, adding each
/// piece to `clipped`; returns the corners whose piece the kernel would not make (by identity), for
/// the caller to fall back on. Two beads each taking their own corner piece lay the same surface
/// twice, which the batch union cannot integrate. A cancel of the build (its work budget) or of a
/// boolean passes up as [`BooleanError::Cancelled`].
pub fn apply_corners(
    working_mesh: &Mesh,
    item_matrix: Matrix4X4,
    source_hull: Option<&Mesh>,
    clippable_beads: &[Clippable],
    clipped: &mut Vec<Mesh>,
    combine: &CombineMeshes<'_>,
) -> Result<Vec<Rc<Mesh>>, BooleanError> {
    let mut refused = Vec::new();
    for (corner, beads) in beads_by_corner(clippable_beads) {
        LevelLineWork::poll_cancel().map_err(|_| BooleanError::Cancelled)?;

        match corner_piece(
            working_mesh,
            item_matrix,
            source_hull,
            &corner,
            &beads,
            combine,
        )? {
            Some(piece) => clipped.push(piece),
            None => refused.push(corner),
        }
    }

    Ok(refused)
}

/// One corner's piece: the union of the beads that reach it, cut by the KEEP SET (the part with
/// the corner's void) and by the source hull where any bead reaches past it; `None` where the
/// kernel refuses a step or a result is no usable operand.
pub fn corner_piece(
    working_mesh: &Mesh,
    item_matrix: Matrix4X4,
    source_hull: Option<&Mesh>,
    corner: &Mesh,
    beads: &[&Mesh],
    combine: &CombineMeshes<'_>,
) -> Result<Option<Mesh>, BooleanError> {
    let refused = |error: BooleanError| match error {
        BooleanError::Cancelled => Err(BooleanError::Cancelled),
        _ => Ok(None),
    };

    let keep = match combine(
        &[
            (working_mesh.clone(), item_matrix),
            (corner.clone(), item_matrix),
        ],
        CsgModes::Union,
    ) {
        Ok(keep) => keep,
        Err(error) => return refused(error),
    };

    let reaching = if beads.len() == 1 {
        beads[0].clone()
    } else {
        let operands: Vec<(Mesh, Matrix4X4)> = beads
            .iter()
            .map(|&bead| (bead.clone(), item_matrix))
            .collect();
        match combine(&operands, CsgModes::Union) {
            Ok(reaching) => reaching,
            Err(error) => return refused(error),
        }
    };

    // The beads' union lands in the operation's space when it was booleaned and in the source
    // item's when there was only one of them to take - and it is classified in whichever of those
    // it is in, because a mirroring matrix turns every triangle of it inside out.
    let reaching_matrix = if beads.len() == 1 {
        item_matrix
    } else {
        Matrix4X4::IDENTITY
    };

    if !is_usable_keep_set(Some(&keep))
        || !bevel_working_mesh::is_usable_operand(Some(&reaching), Some(reaching_matrix))
    {
        return Ok(None);
    }

    let mut cut_by = vec![(reaching, reaching_matrix), (keep, Matrix4X4::IDENTITY)];
    // Asked of the BEADS rather than of the union, because the union may already have been carried
    // into the operation's space while the beads and the hull are both still in the source item's.
    if beads
        .iter()
        .any(|bead| !bevel_source_hull::lies_inside(source_hull, Some(bead)))
    {
        if let Some(hull) = source_hull {
            cut_by.push((hull.clone(), item_matrix));
        }
    }

    let piece = match combine(&cut_by, CsgModes::Intersect) {
        Ok(piece) => piece,
        Err(error) => return refused(error),
    };

    Ok(bevel_working_mesh::is_usable_operand(Some(&piece), None).then_some(piece))
}
