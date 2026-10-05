//! Where a roll across a curved wall RUNS OUT of wall before the profile fits
//! (`Bevel/Surface/SurfaceWallRanOut.cs`, a static class: free functions here). The bound one
//! linear condition puts on the size lands here first; the reading itself lands with later
//! packages.

/// Narrows the window `lowest..highest` of sizes to where the linear condition
/// `per_size * size + at_nothing >= 0` holds: a condition that does not move with the size either
/// always holds or never does (then the window closes).
#[allow(clippy::neg_cmp_op_on_partial_ord)] // C#'s `!(x > y)`, which refuses a NaN as well.
pub fn bound(per_size: f64, at_nothing: f64, lowest: &mut f64, highest: &mut f64) {
    if !(per_size.abs() > 1e-12) {
        // The condition does not move with the size: it either always holds or never does.
        if at_nothing < 0.0 {
            *highest = f64::NEG_INFINITY;
        }

        return;
    }

    let crosses_at = -at_nothing / per_size;

    // C#'s Math.Max / Math.Min, which pass a NaN through.
    if per_size > 0.0 {
        *lowest = if lowest.is_nan() || crosses_at.is_nan() {
            f64::NAN
        } else {
            lowest.max(crosses_at)
        };
    } else {
        *highest = if highest.is_nan() || crosses_at.is_nan() {
            f64::NAN
        } else {
            highest.min(crosses_at)
        };
    }
}
