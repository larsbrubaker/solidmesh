//! The band gap's room reading. Ports `Bevel/LevelLine/BandGap.Reading.cs` (a partial of the C#
//! static class `BandGap`): so far the margin a room reading keeps under the room
//! ([`room_margin`]); the reading lands with later packages.

/// How much under the room a size has to stand for a kept strip to exist at all, as a fraction of
/// the size: two snaps over the largest unit setback (C#'s optional `snap = 1e-9` passed
/// explicitly); zero for a pick with no setback.
pub fn room_margin(largest_unit_setback: f64, snap: f64) -> f64 {
    if largest_unit_setback > 0.0 {
        2.0 * snap / largest_unit_setback
    } else {
        0.0
    }
}
