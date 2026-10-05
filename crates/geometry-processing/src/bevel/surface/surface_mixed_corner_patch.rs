//! The patch that closes a mixed corner between the cut's end ring and the bead's
//! (`Bevel/Surface/SurfaceMixedCornerPatch.cs`, a static class: free functions here). The ring
//! readers and the shared face's chord sagitta land here first; the patch's own construction
//! (`TryRing`, `TryCutTheRing`) lands with a later package.

/// How far off its own circle the shared face's chording already stands - the sagitta of one of
/// its chords, `(chord / 2) tan(turn / 4)` - which is what a ring standing on those chords may be
/// off the ideal torus by without being wrong about anything the mesh knows.
///
/// Read off the section the bead's own roll walked, so it is that wall's chording and not a
/// constant. A roll that never happened has no section and no sagitta, and the part's own ruler
/// is what is left.
#[allow(clippy::neg_cmp_op_on_partial_ord)] // C#'s `!(x > 0)`, which refuses a NaN as well.
pub fn the_shared_faces_own_sagitta(
    corner: &super::SurfaceMixedCorner,
    pool: &super::SurfaceVertexPool,
) -> f64 {
    let walk = match corner.rolled_section() {
        Some(walk) if walk.facets.len() >= 2 => walk,
        _ => return pool.near_enough(),
    };

    let mut widest = 0.0;

    for facet in 1..walk.facets.len() {
        let turned = walk.facets[facet]
            .normal
            .dot(walk.facets[facet - 1].normal)
            .clamp(-1.0, 1.0)
            .acos();

        if !(turned > 0.0) {
            continue;
        }

        widest = f64::max(
            widest,
            walk.facets[facet].crossed() / 2.0 * (turned / 4.0).tan(),
        );
    }

    f64::max(pool.near_enough(), widest)
}

/// The cut's ring read from the cap face round to the shared face, whichever way its own rails
/// run. Rail 0 lies on the LEFT of the walk, so which end is which is the plan's own left and
/// right.
pub fn from_the_cap_face(corner: &super::SurfaceMixedCorner, ring: &[i32]) -> Vec<i32> {
    if corner.cut.left_region_key() == corner.shared_region_key {
        reversed(ring)
    } else {
        ring.to_vec()
    }
}

/// The bead's ring read from the handover - its rail on the shared face - round to the wall.
pub fn from_the_handover(corner: &super::SurfaceMixedCorner, ring: &[i32]) -> Vec<i32> {
    if corner.bead.left_region_key() == corner.shared_region_key {
        ring.to_vec()
    } else {
        reversed(ring)
    }
}

/// `ring` back to front, as a new array.
fn reversed(ring: &[i32]) -> Vec<i32> {
    ring.iter().rev().copied().collect()
}
