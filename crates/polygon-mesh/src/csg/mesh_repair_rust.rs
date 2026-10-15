//! Mesh repairs performed with the boolean kernel's import, for callers that want a repair and not
//! a boolean: [`rewind_backward_patches`], which the model's boolean operations use to retry an
//! operand the kernel refused, and [`try_repair_orientation`], the Repair tool's shell-level face
//! orientation (`g3::advanced_repair`).

use manifold_rust::manifold::Manifold;
use manifold_rust::types::Error as KernelStatus;

use super::{consistent_winding, manifold_import, manifold_kernel, manifold_result_reader};
use crate::Mesh;

/// A copy of `source_mesh` with the triangles of a closed surface that are wound against the rest
/// of it rewound - the backward patch the kernel import refuses as NotClosed although every edge
/// has its two faces - or `None` when there is nothing to rewind.
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

/// Rewinds inside-out shells so every body reads as solid material, using the kernel's exact
/// shell-level orientation repair. `None` when the kernel could not take the mesh (empty input, an
/// import or repair error status, or an empty result), in which case the caller uses its own
/// fallback.
///
/// The kernel decides per shell with exact predicates: outermost shells end up wound outward,
/// legitimate cavity shells end up wound inward, and coincident or doubled sheets are left alone.
/// That last point is why this is preferred to a flood fill that makes each connected component's
/// winding consistent and then picks a global sign: the kernel is deliberately conservative and
/// never removes or reverses material it has no evidence about, so a single shell whose triangles
/// disagree with each other comes back unchanged rather than guessed at.
///
/// Import is the robust one, so closed-but-non-manifold soup is welded rather than rejected.
/// Geometry that is not closed at all cannot be imported, and that is the main reason this
/// returns `None`. Vertices may be welded and reindexed by the round trip, so anything keyed on
/// vertex or face index (per-face colours, for instance) has to be carried across by position.
/// `source_mesh` is left unmodified.
pub fn try_repair_orientation(source_mesh: &Mesh) -> Option<Mesh> {
    if source_mesh.faces.is_empty() || source_mesh.vertices.is_empty() {
        return None;
    }

    // This is the one entry point into the kernel that does not go through a boolean, so it is
    // the one that has to ask for the kernel's process-global configuration itself: without it
    // a Repair performed before any boolean in the session would run on different settings than
    // the same Repair performed after one.
    manifold_kernel::ensure_configured();

    let imported =
        Manifold::from_mesh_gl64_robust(&manifold_import::to_rust_mesh_data(source_mesh));
    if imported.status() != KernelStatus::NoError {
        return None;
    }

    let repaired = imported.repair_orientation();
    if repaired.status() != KernelStatus::NoError {
        return None;
    }

    // An empty result from a non-empty input means the kernel lost the solid. Handing that back
    // would silently delete the user's model, so it counts as a failure and the caller falls
    // back instead. A result with fewer than three properties per vertex is refused too.
    let result = manifold_result_reader::to_mesh(&repaired, "orientation repair").ok()?;
    if result.faces.is_empty() {
        return None;
    }
    Some(result)
}
