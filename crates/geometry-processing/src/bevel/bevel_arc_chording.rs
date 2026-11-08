//! How many segments an arc of a given angle is chorded into - the one place that reads the
//! operation's `Segments` as a count per QUARTER TURN rather than a count per arc
//! ([`segments_for`]); the surface construction's strips chord their arcs with it.
//!
//! A fixed count per arc chords a sharp corner far more coarsely than a blunt one: a 45 degree
//! knife tip turns its fillet through 135 degrees, so eight segments there step 16.875 degrees
//! against a 90 degree edge's 11.25, and the sagitta grows with the square of the step. Reading
//! the number as segments per quarter turn makes the chord error the same everywhere.

/// How much arc one segment of the operation's own count stands for.
const QUARTER_TURN_DEGREES: f64 = 90.0;

/// The segment count for an arc of `angle_degrees`, given the operation's segments per quarter
/// turn. Never less than one: an arc still has to be crossed even when it is a sliver.
pub fn segments_for(angle_degrees: f64, segments_per_quarter: i32) -> i32 {
    // `<= 0 || NaN` refuses every angle that is not positive, a NaN included.
    if segments_per_quarter < 1 || angle_degrees <= 0.0 || angle_degrees.is_nan() {
        return 1.max(segments_per_quarter);
    }

    // Half rounds away from zero (f64::round). `as` saturates past i32's range, which only an
    // absurd angle can reach.
    let rounded =
        (f64::from(segments_per_quarter) * angle_degrees / QUARTER_TURN_DEGREES).round() as i32;
    1.max(rounded)
}
