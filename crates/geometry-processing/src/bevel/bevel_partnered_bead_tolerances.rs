//! The partnered bead's tolerances: every distance the build compares against, each scaled to what
//! it is a distance OF - the bead's size, or the part's own diagonal.

use super::bevel_partnered_bead::BevelPartneredBeadRun;

/// How far past its crease a run-out is measured, in sizes.
pub const RUN_OUT_LIMIT_IN_SIZES: f64 = 10.0;

/// How far a run's faces may turn along it and still be one face.
pub const SAME_FACE_WITHIN_DEGREES: f64 = 0.1;

/// How far the two flat sides are carried inside the part, in sizes.
pub const CLEARANCE_IN_SIZES: f64 = 0.01;

/// How far past a partner's tube the bead's own reaches, in sizes.
pub const PARTNER_OVERLAP_IN_SIZES: f64 = 2e-4;

/// How deep inside a partner a point has to be to count as covered.
pub const COVERED_BY_IN_SIZES: f64 = PARTNER_OVERLAP_IN_SIZES + 2e-6;

/// How near two runs' ends are the same place, in part diagonals.
pub const MEETING_WITHIN_THE_PARTS: f64 = 3e-6;

/// The least run-out, in sizes, that is a run-out at all.
pub const RUNS_OUT_AT_LEAST: f64 = 0.1;

/// How many clearances the corner point may be buried at most.
pub const DEEPEST_BURIAL: f64 = 4.0;

/// How near a face a point has to be to reach it, in sizes.
pub const REACHES_THE_FACE_IN_SIZES: f64 = 2e-4;

/// How far short of the across plane an end stops, in sizes.
pub const ACROSS_PLANE_TIP_IN_SIZES: f64 = 1e-3;

/// How near a station has to be to a cut to stand on it.
pub const CUT_BY: f64 = 1e-9;

/// How far short of the crease's end the second face has to stop to count as stopping, in sizes.
pub const SECOND_FACE_STOPS_SHORT_IN_SIZES: f64 = 2e-4;

/// How deep inside a partner a point has to be to count as covered, for this run.
pub fn covered_by(run: &BevelPartneredBeadRun) -> f64 {
    run.size * COVERED_BY_IN_SIZES
}

/// How near two runs' ends are the same place, for this run's part.
pub fn meeting_within(run: &BevelPartneredBeadRun) -> f64 {
    run.run_out.diagonal * MEETING_WITHIN_THE_PARTS
}

/// How near a face a point has to be to reach it, for this run.
pub fn reaches_the_face(run: &BevelPartneredBeadRun) -> f64 {
    run.size * REACHES_THE_FACE_IN_SIZES
}

/// How far short of the crease's end the second face has to stop, for this run.
pub fn second_face_stops_short_by(run: &BevelPartneredBeadRun) -> f64 {
    run.size * SECOND_FACE_STOPS_SHORT_IN_SIZES
}

/// How far past a partner's tube the bead's own reaches, at `size`.
pub fn partner_overlap(size: f64) -> f64 {
    size * PARTNER_OVERLAP_IN_SIZES
}
