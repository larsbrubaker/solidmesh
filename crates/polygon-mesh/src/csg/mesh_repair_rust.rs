//! Mesh repairs performed with the boolean kernel's import, for callers that want a repair and
//! not a boolean. Port of agg-sharp `Csg/MeshRepairRust.cs` (static class `MeshRepair`):
//! `RewindBackwardPatches`, which the model's boolean operations use to retry an operand the
//! kernel refused. `TryRepairOrientation` (the Repair design tool's kernel repair) is not here
//! yet.

use super::{consistent_winding, manifold_import};
use crate::Mesh;

/// C# `MeshRepair.RewindBackwardPatches`: a copy of `source_mesh` with the triangles of a
/// closed surface that are wound against the rest of it rewound - the backward patch the kernel
/// import refuses as NotClosed although every edge has its two faces - or `None` when there is
/// nothing to rewind.
///
/// Only the minority winding of each connected surface flips, so no shape changes and a shell
/// that is inside out as a whole stays that way. Surfaces are joined by exact position first,
/// then across seams a rounding step apart. The copy keeps the original faces' order, colours
/// and textures; it is meant to be handed to the kernel as an operand after the kernel refused
/// the original. `source_mesh` is left unmodified.
pub fn rewind_backward_patches(source_mesh: &Mesh) -> Option<Mesh> {
    if source_mesh.faces.is_empty() {
        return None;
    }

    if let Some(rewound) = consistent_winding::rewind(source_mesh) {
        return Some(rewound);
    }

    let welded = manifold_import::weld_seams(source_mesh)?;
    consistent_winding::rewind(&welded)
}
