//! The patch that closes a mixed corner between the cut's end ring and the bead's
//! (`Bevel/Surface/SurfaceMixedCornerPatch.cs`, a static class: free functions here). The ring
//! readers land here first; the patch's own construction (`TryRing`, `TryCutTheRing`) and the
//! shared face's sagitta land with later packages.

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
