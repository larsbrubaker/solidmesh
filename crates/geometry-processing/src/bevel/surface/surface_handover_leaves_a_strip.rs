//! Whether a handover between two band ends still leaves a strip of band between them: the check
//! of a whole plan
//! ([`try_check`]), the band test along a line ([`leaves_a_band`]) and its plan form
//! ([`leaves_a_band_of`]).

use vector_math::ieee_math;

use super::{SurfaceBandEnd, SurfaceBevelPlan, SurfaceLine};
use crate::bevel::{BevelAdvisory, BevelAdvisoryCode};

/// Checks every corner of `plan` that swallows an edge - that the edge carrying the
/// corner's ring still has a band to hand over on both its sides. A plan that swallows nothing is
/// not measured at all. `Err` is the refusal, named at the corner.
pub fn try_check(plan: &SurfaceBevelPlan) -> Result<(), BevelAdvisory> {
    if plan.roles().swallowed_count() == 0 {
        return Ok(());
    }

    // The plan's OWN corner keys and not 0..edges: a junction's corner stands at an end of the
    // chain and is named by a sentinel no edge index can be (SurfaceBevelPlan::junction_key).
    for corner in plan.mixed_corner_keys() {
        let Some(cut) = plan.roles().try_the_edge_carrying_the_ring_at(corner) else {
            continue;
        };
        let cut = cut as usize;

        for left in [true, false] {
            let line = if left {
                &plan.edges[cut].left_tangent_line
            } else {
                &plan.edges[cut].right_tangent_line
            };

            if leaves_a_band_of(plan, cut, left, line) {
                continue;
            }

            let at = if corner >= 0 {
                plan.edges[corner as usize].to()
            } else {
                plan.edges[cut].from()
            };

            return Err(BevelAdvisory::new(
                BevelAdvisoryCode::SurfaceCornerNotSupported,
                vec![at.into()],
            ));
        }
    }

    Ok(())
}

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
            from = ieee_math::max_f64(from, at);
        } else {
            to = ieee_math::min_f64(to, at);
        }
    }

    from < to
}

/// Whether the band ends of one side of
/// one edge of `plan` ([`SurfaceBevelPlan::band_ends_of`]) leave any of `line` standing.
pub fn leaves_a_band_of(
    plan: &SurfaceBevelPlan,
    edge_index: usize,
    left: bool,
    line: &SurfaceLine,
) -> bool {
    leaves_a_band(&plan.band_ends_of(edge_index, left), line)
}
