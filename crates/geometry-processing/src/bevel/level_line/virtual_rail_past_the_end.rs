//! A side's rail carried on past a chain's end until the whole row has left the end plane. Ports
//! `Bevel/LevelLine/VirtualRailPastTheEnd.cs` (a C# static class): so far its constant and where
//! the row last touches the end plane ([`last_to_leave`]); the carrying lands with later packages.
//!
//! Where the end face leans OUT over the end (the section's top meets the end plane beyond P), the
//! rows must run on past P until the whole row has passed the end plane; the end plane then clips
//! them. Assumes a STRAIGHT end segment, a flat side across the stretch and a FILLET or chamfer row
//! laid as the fill lays it.

use vector_math::{cs_math, Vector3};

use super::BandProfile;

/// How far past the last row's touch of the end plane the rail is carried, in setbacks.
pub const PAST_THE_LAST_TOUCH: f64 = 0.25;

/// How far behind the end plane (through `end`, normal `normal`) the row between the two rail
/// points reaches at most: its ends, and for a fillet the top of its arc where that stands on the
/// end vertex's side of the chord.
pub fn last_to_leave(
    rail_left: Vector3,
    rail_right: Vector3,
    end: Vector3,
    outward: Vector3,
    normal: Vector3,
    profile: BandProfile,
) -> f64 {
    let behind = |q: Vector3| -(q - end).dot(normal);
    let mut best = cs_math::max_f64(behind(rail_left), behind(rail_right));
    if profile != BandProfile::Fillet {
        return best;
    }

    let to_left = rail_left - end;
    let to_right = rail_right - end;
    let cos = to_left
        .get_normal()
        .dot(to_right.get_normal())
        .clamp(-1.0, 1.0);
    let half = cos.acos() / 2.0;
    let bisector = (to_left.get_normal() + to_right.get_normal()).get_normal();
    let centre = end + (bisector * (to_left.length() / half.cos()));
    let radius = to_left.length() * half.tan();

    // The function's direction in the section plane (square to the chain).
    let direction = -normal - (outward * (-normal).dot(outward));
    if direction.length_squared() == 0.0 {
        return best;
    }

    let top = centre + (direction.get_normal() * radius);

    // On the arc when it stands on the end vertex's side of the chord between the rail points.
    let chord_mid = (rail_left + rail_right) / 2.0;
    if (top - chord_mid).dot(end - chord_mid) >= 0.0 {
        best = cs_math::max_f64(best, behind(top));
    }

    best
}
