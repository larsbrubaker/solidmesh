//! The size a refused row offers, as the row shows it ([`the_shown_size`]): every size the part
//! builds search for (`surface_part_builder::size_that_fits`, `point_groups`) is first floored to
//! it, so the number a caller puts in front of the user is a size that was actually built. The
//! words around the number are the caller's.

/// `size` floored to three decimal places, the precision a row quotes, so 2.9885 is offered as
/// 2.988 rather than rounded up to a 2.989 that never built. Rounded to six places before the
/// floor so a size already on the grid, whose double is a hair under it (2.988 is
/// 2987.9999999999995 thousandths), stays where it is. The rounding is banker's rounding.
pub fn the_shown_size(size: f64) -> f64 {
    (round_to_places(size * 1000.0, 6)).floor() / 1000.0
}

/// Rounds to `digits` places, ties to even: scales by the power of ten, rounds to even, and
/// divides back.
fn round_to_places(value: f64, digits: i32) -> f64 {
    let power = 10f64.powi(digits);
    (value * power).round_ties_even() / power
}
