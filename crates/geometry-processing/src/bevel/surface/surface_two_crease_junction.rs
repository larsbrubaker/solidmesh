//! A junction where TWO creases meet one bead at one corner
//! (`Bevel/Surface/SurfaceTwoCreaseJunction.cs`, a static class: free functions here). The tests
//! the junction's reading is made of land here first; the reading itself lands with a later
//! package.

use super::{SurfaceMixedCorner, SurfaceMixedCornerDualFrame};

/// How near the two arms' readings of the SAME turn have to agree, as a dimensionless angle and
/// so a fixed epsilon - the same reasoning as [`SurfaceMixedCorner::SQUARE_ENOUGH`].
pub const SAME_TURN: f64 = 1e-6;

/// How near the two walls' outward normals have to stand to be one plane, as the length between
/// two unit vectors. Dimensionless, so fixed rather than the part's own ruler.
pub const SAME_WALL: f64 = 1e-6;

/// Whether the two creases stand against ONE face - the same region, and one plane. Both,
/// because two coplanar faces of a part are still two faces to walk back, and two facets of one
/// region can stand at an angle where a bead rolled along it.
pub fn the_creases_stand_against_one_wall(
    first: &SurfaceMixedCorner,
    second: &SurfaceMixedCorner,
) -> bool {
    wall_key_of(first) == wall_key_of(second)
        && (first.out_of_the_wall - second.out_of_the_wall).length() <= SAME_WALL
}

/// Whether both arms read ONE canal: the same turn, and the ball each hands over at is the ball
/// the other starts from. Asked both ways about, because the two arms trace the one ellipse in
/// opposite senses and a reading that agreed at one end only would be two arcs of it rather than
/// one.
pub fn the_two_duals_are_one_canal(
    first: &SurfaceMixedCornerDualFrame,
    second: &SurfaceMixedCornerDualFrame,
    near_enough: f64,
) -> bool {
    (first.turn_radians - second.turn_radians).abs() <= SAME_TURN
        && (first.spine.centre_at(first.turn_radians) - second.spine.centre_at(0.0)).length()
            <= near_enough
        && (second.spine.centre_at(second.turn_radians) - first.spine.centre_at(0.0)).length()
            <= near_enough
}

/// The face the bead stands against that nobody picked - its side that is not the shared one.
pub fn wall_key_of(mixed: &SurfaceMixedCorner) -> i32 {
    if mixed.bead.left_region_key() == mixed.shared_region_key {
        mixed.bead.right_region_key()
    } else {
        mixed.bead.left_region_key()
    }
}
