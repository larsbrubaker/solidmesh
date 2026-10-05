//! The pieces of beads that meet at a 5.3 corner. Ports `Bevel/BevelBeadCornerPieces.cs` (a C#
//! static class): so far the beads gathered by the corner void they share ([`beads_by_corner`]);
//! the corner pieces themselves land with later runs of the package.

use std::rc::Rc;

use polygon_mesh::Mesh;

use super::bevel_bead_clip::Clippable;

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
