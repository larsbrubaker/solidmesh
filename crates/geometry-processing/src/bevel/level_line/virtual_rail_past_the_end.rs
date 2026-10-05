//! A side's rail carried on past a chain's end until the whole row has left the end plane. Ports
//! `Bevel/LevelLine/VirtualRailPastTheEnd.cs` (a C# static class): so far its constant and where
//! the row last touches the end plane ([`last_to_leave`]) and where a run leaves the rail at the
//! end ([`to_the_rail`]); the carrying lands with later packages.
//!
//! Where the end face leans OUT over the end (the section's top meets the end plane beyond P), the
//! rows must run on past P until the whole row has passed the end plane; the end plane then clips
//! them. Assumes a STRAIGHT end segment, a flat side across the stretch and a FILLET or chamfer row
//! laid as the fill lays it.

use vector_math::{cs_math, Vector3};

use super::{BandProfile, ChainField, GapLoop, LevelLineMesh, WorkError};

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

/// Where a run leaves the rail at an end ([`to_the_rail`]).
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct ToTheRail {
    /// How many vertices are kept, counted from the far end.
    pub keep_count: i32,
    /// The side's direction off the chain at P.
    pub away: Vector3,
    /// The chain parameter at P.
    pub parameter_at_p: f64,
    /// Whether the end is at the run's tail.
    pub at_tail: bool,
}

/// C# `VirtualRailPastTheEnd.ToTheRail`: where the run leaves the rail at the end: the run is cut
/// back to its last vertex ON the rail (P, where the rail meets the side-to-end crease), dropping
/// the crease stretch beyond it. `None` where nothing stands beyond P - the run already stops on
/// the rail (or has no rail at all) - unless `may_end_at_p`: a back-facing wall's side run ends AT
/// P (nothing of the crease runs on past it), so P is its last vertex.
pub fn to_the_rail(
    mesh: &LevelLineMesh,
    chain: &ChainField,
    run: &GapLoop,
    last: bool,
    setback: f64,
    may_end_at_p: bool,
) -> Result<Option<ToTheRail>, WorkError> {
    let at_tail = last == (run.parameters[run.parameters.len() - 1] >= run.parameters[0]);
    let mut ordered = run.vertices.clone();
    if !at_tail {
        ordered.reverse();
    }

    let on_rail = 1e-9 * cs_math::max_f64(1.0, setback) + (4.0 * mesh.same_place);
    let mut keep = ordered.len();
    while keep > 1
        && chain
            .evaluate(mesh.position(ordered[keep - 1]))?
            .value
            .abs()
            > on_rail
    {
        keep -= 1;
    }

    if (keep == ordered.len() && !may_end_at_p) || keep < 2 {
        return Ok(None);
    }

    let p = mesh.position(ordered[keep - 1]);
    let sample = chain.evaluate(p)?;
    let away = p - sample.nearest;
    Ok(Some(ToTheRail {
        keep_count: keep as i32,
        away: away.get_normal(),
        parameter_at_p: sample.parameter,
        at_tail,
    }))
}
