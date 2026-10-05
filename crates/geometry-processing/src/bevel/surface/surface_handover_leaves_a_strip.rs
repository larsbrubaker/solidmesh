//! Whether a handover between two band ends still leaves a strip of band between them. Ports
//! `Bevel/Surface/SurfaceHandoverLeavesAStrip.cs` (a C# static class): so far the band test along a
//! line ([`leaves_a_band`]); the handover reading lands with later packages.

use vector_math::cs_math;

use super::{SurfaceBandEnd, SurfaceLine};

/// True when some stretch of `line` stands on the kept side of every band end (each end's
/// `inward` half-space through its point): the half-spaces cut the line to an interval, open when
/// it is empty. An end parallel to the line keeps all of it or none.
pub fn leaves_a_band(ends: &[SurfaceBandEnd], line: &SurfaceLine) -> bool {
    let mut from = f64::NEG_INFINITY;
    let mut to = f64::INFINITY;

    for end in ends {
        let slope = line.direction.dot(end.inward);
        let standing = (end.point - line.point).dot(end.inward);

        if slope.abs() <= 1e-9 {
            if standing > 0.0 {
                return false;
            }

            continue;
        }

        let at = standing / slope;

        if slope > 0.0 {
            from = cs_math::max_f64(from, at);
        } else {
            to = cs_math::min_f64(to, at);
        }
    }

    from < to
}
