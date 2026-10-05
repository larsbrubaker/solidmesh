//! A band's end cut where a mixed corner's run-out CROSSES the band before its end: the rails the
//! crossing took stop on the crossing, and only the rest close against the face beyond. Ports
//! `Bevel/Surface/SurfaceEndCutAboveTheCrossing.cs` (C# `internal static class`; a module of free
//! functions here). What is here are the readings the close is built from - the end plane the
//! uncrossed rails close against ([`try_the_end_plane`]), whether the crossing's foot stands on it
//! ([`the_foot_stands_on_the_end_plane`]), the cut back of the section at the run-out's rim
//! ([`cut_back_at_the_run_out`]), what each chord carries ([`the_betweens`]) and whether a point
//! stands clear of every face ([`nothing_stands_at`]); the close itself (`TryClose`) lands with a
//! later package. The trim it reads is
//! [`SurfaceStripTrimmedAtTheCrossing`].
//!
//! C#'s `IReadOnlyList<int>[] alongTheProfile` is `Option<&[Vec<i32>]>` here: the array may be null,
//! but every list the end cuts hand over is a list (empty where nothing stands along a chord), so a
//! null ENTRY - which C# also skips - has no Rust spelling.

use vector_math::Vector3;

use super::{SurfaceStrip, SurfaceStripTrimmedAtTheCrossing, SurfaceVertexPool};

/// C# `TheFoot`: what `rail_of` says of the entry that is the crossing's own foot.
pub const THE_FOOT: i32 = -1;

/// C# `TheDiePoint`: what `rail_of` says of the entry that is the run-out's die point.
const THE_DIE_POINT: i32 = -2;

/// C# `CutBackAtTheRunOut`: drops whatever of the section stands BEYOND the run-out's own rim on
/// this face - the straight side from the crossing's foot to the die point - keeping the side the
/// band's own far rail stands on. That rail is the one the crossing never reached, so it is the
/// section's anchor: whatever stands with it is face the run-out left, and whatever stands across
/// the line is face it took. `the_apex` and `the_die_is_the_apex` are C#'s `ref` parameters.
#[allow(clippy::neg_cmp_op_on_partial_ord, clippy::too_many_arguments)]
pub fn cut_back_at_the_run_out(
    pool: &SurfaceVertexPool,
    profile: &mut Vec<i32>,
    rail_of: &mut Vec<i32>,
    trimmed: &SurfaceStripTrimmedAtTheCrossing,
    normal: Vector3,
    the_apex: &mut Vector3,
    the_die_is_the_apex: &mut bool,
) {
    let anchor = if trimmed.lowest_rail_crossed() == 0 {
        rail_of.len() - 1
    } else {
        0
    };
    let foot = pool.position_of(trimmed.foot);
    let along = pool.position_of(trimmed.the_die_vertex.get()) - foot;

    if !(along.length_squared() > 0.0) {
        return;
    }

    let away = normal.cross(along).get_normal();
    let keeps = (pool.position_of(profile[anchor]) - foot).dot(away);

    if !(keeps.abs() > pool.near_enough()) {
        // The anchor itself stands on the line, so there is no side to keep by: the run-out's rim
        // runs along the section's own edge and nothing of it is beyond.
        return;
    }

    for at in (0..profile.len()).rev() {
        if rail_of[at] == THE_FOOT || rail_of[at] == THE_DIE_POINT {
            continue;
        }

        let side = (pool.position_of(profile[at]) - foot).dot(away);

        if side * keeps < 0.0 && side.abs() > pool.near_enough() {
            profile.remove(at);
            rail_of.remove(at);
        }
    }

    // And the apex with them, where it stands beyond the line as well: the wedge the section is cut
    // as is drawn from a point of the face, and the part's own corner is not one any more. The die
    // point is, and it stands on the same edge of the face between the corner and what is left, so
    // the wedge from it is the same wedge with what the run-out took already off it.
    let the_corner_stands = (*the_apex - foot).dot(away);

    if the_corner_stands * keeps < 0.0 && the_corner_stands.abs() > pool.near_enough() {
        *the_apex = pool.position_of(trimmed.the_die_vertex.get());
        *the_die_is_the_apex = true;
    }
}

/// C# `TheFootStandsOnTheEndPlane`: whether the crossing's foot stands on the END PLANE itself
/// rather than inside the section - the param the rail beside it meets that plane at, against the
/// param the foot stands at, read to the chord between those two rails. A foot on the plane is a
/// corner of the section's own rim; one inside it is a point the curve passes through the section at.
pub fn the_foot_stands_on_the_end_plane(
    strip: &SurfaceStrip,
    corner: Vector3,
    normal: Vector3,
    trimmed: &SurfaceStripTrimmedAtTheCrossing,
) -> bool {
    let crossed = if trimmed.lowest_rail_crossed() == 0 {
        trimmed.highest_rail_crossed()
    } else {
        trimmed.lowest_rail_crossed()
    };
    let under = if trimmed.lowest_rail_crossed() == 0 {
        crossed + 1
    } else {
        crossed - 1
    };

    if under < 0 || under >= strip.rail_count() {
        return false;
    }

    let Some(param) = strip.try_cross(under, corner, normal) else {
        return false;
    };

    let apart = (strip.point_on(under, 0.0) - strip.point_on(crossed, 0.0)).length();

    (trimmed.foot_param - param).abs() <= apart * 1e-6
}

/// C# `TheBetweens`: what each of the strip's own chords carries between its two rails - whatever
/// the end cut left standing along it, and on the chord the FOOT stands on, the foot itself - in the
/// order the chord runs, from the lower rail to the higher.
///
/// Read off the RAIL each corner of the section's rim came from (`rail_of`, or [`THE_FOOT`] / the
/// die point) rather than by counting corners: the rim carries points no rail of the strip ever had,
/// and where the run-out has taken part of the face the rim is shorter than the strip as well. What
/// a chord carries is then whatever the cut left along the rim between the two entries those two
/// rails stand at, and nothing at all where either of them is gone.
pub fn the_betweens(
    rail_count: i32,
    along_the_profile: Option<&[Vec<i32>]>,
    rail_of: &[i32],
    foot: i32,
    the_die_vertex: i32,
    lowest_rail_crossed: i32,
    highest_rail_crossed: i32,
) -> Vec<Vec<i32>> {
    let rail_count = rail_count.max(0) as usize;
    let mut betweens: Vec<Vec<i32>> = vec![Vec::new(); rail_count.saturating_sub(1)];

    // Where each rail stands in the rim, so that the stretch between two of them can be read off.
    let mut stands_at = vec![-1i32; rail_count];

    for (at, &rail) in rail_of.iter().enumerate() {
        if rail >= 0 && (rail as usize) < stands_at.len() {
            stands_at[rail as usize] = at as i32;
        }
    }

    let mut the_walk_carried_the_foot = false;

    let mut chord = 0;
    while chord + 1 < rail_count {
        let from = stands_at[chord];
        let to = stands_at[chord + 1];

        if from < 0 || to < 0 {
            chord += 1;
            continue;
        }

        // Everything the rim carries between the two, in the order the CHORD runs - which is from the
        // lower rail to the higher, whichever way round the rim itself was walked.
        let mut carrying = Vec::new();
        let forwards = from < to;
        let first = from.min(to) as usize;
        let last = from.max(to) as usize;

        for at in first..last {
            if at > first {
                // A corner of the rim standing between the two rails: the crossing's own foot, or the
                // die point where it stands there.
                let own = the_rims_own(rail_of, at, foot, the_die_vertex);

                the_walk_carried_the_foot |= own == foot && own >= 0;

                carrying.push(own);
            }

            if let Some(along) = along_the_profile.and_then(|along| along.get(at)) {
                carrying.extend_from_slice(along);
            }
        }

        carrying.retain(|&vertex| vertex >= 0);

        if !forwards {
            carrying.reverse();
        }

        betweens[chord] = carrying;
        chord += 1;
    }

    // And the FOOT where the walk above has not already carried it - because it is no corner of the
    // rim at all, the curve passing THROUGH the section rather than ending on its edge; or because
    // the rail on its far side is one the run-out took the face from. Either way that chord has to
    // carry it, or the two surfaces meet along a line with a hole in it. Nearest the rail the
    // crossing reached, which is the end it goes on.
    if the_walk_carried_the_foot {
        return betweens;
    }

    let foot_chord = if lowest_rail_crossed == 0 {
        highest_rail_crossed
    } else {
        lowest_rail_crossed - 1
    };

    if foot_chord >= 0 && (foot_chord as usize) < betweens.len() {
        let foot_chord = foot_chord as usize;
        let mut carrying = betweens[foot_chord].clone();

        // AND WHAT THE END CUT LEFT ON THE WAY TO IT. The foot is a corner of the RIM here, and the
        // stretch of rim between it and the rail beside it is a chord like any other; without this
        // the strip sews straight from the rail to the foot while the face beyond carries the same
        // run split at the crossing - a T junction on the seam.
        let along_the_way_to_the_foot =
            what_the_rim_carries_to_the_foot(along_the_profile, rail_of, lowest_rail_crossed);

        if lowest_rail_crossed == 0 {
            carrying.insert(0, foot);
            carrying.splice(1..1, along_the_way_to_the_foot.iter().copied());
        } else {
            carrying.extend_from_slice(along_the_way_to_the_foot);
            carrying.push(foot);
        }

        betweens[foot_chord] = carrying;
    }

    betweens
}

/// C# `WhatTheRimCarriesToTheFoot`: what the end cut left standing along the one stretch of rim
/// that runs between the FOOT and the rail beside it, in the order that stretch is walked from the
/// rail towards the foot. Its own order already runs that way in both cases, so nothing is reversed.
fn what_the_rim_carries_to_the_foot<'a>(
    along_the_profile: Option<&'a [Vec<i32>]>,
    rail_of: &[i32],
    lowest_rail_crossed: i32,
) -> &'a [i32] {
    let Some(along_the_profile) = along_the_profile else {
        return &[];
    };

    let Some(foot_at) = rail_of.iter().position(|&rail| rail == THE_FOOT) else {
        return &[];
    };

    // The rim chord on the rail side of the foot: the one ending at it where the foot is last, the
    // one starting from it where the foot is first.
    let chord = if lowest_rail_crossed == 0 {
        foot_at as i64
    } else {
        foot_at as i64 - 1
    };

    if chord < 0 {
        return &[];
    }

    along_the_profile
        .get(chord as usize)
        .map_or(&[], |along| along.as_slice())
}

/// C# `TheRimsOwn`: the vertex a rim corner that is no rail of the strip stands at, or -1.
fn the_rims_own(rail_of: &[i32], at: usize, foot: i32, the_die_vertex: i32) -> i32 {
    if rail_of[at] == THE_FOOT {
        foot
    } else if rail_of[at] == THE_DIE_POINT {
        the_die_vertex
    } else {
        -1
    }
}

/// C# `TryTheEndPlane`: the plane the uncrossed rails close against, as its unit normal - through
/// the corner, square to nothing in particular but holding the tangent line vertex the crossing left
/// alone and the edge's own direction. `None` where both tangent lines were taken, so nothing of
/// this end stands against the face beyond it at all.
#[allow(clippy::neg_cmp_op_on_partial_ord)]
pub fn try_the_end_plane(
    pool: &SurfaceVertexPool,
    strip: &SurfaceStrip,
    corner: Vector3,
    on_the_left: i32,
    on_the_right: i32,
    trimmed: &SurfaceStripTrimmedAtTheCrossing,
) -> Option<Vector3> {
    let left_is_crossed = trimmed.crosses(0);
    let right_is_crossed = trimmed.crosses(strip.rail_count() - 1);

    if left_is_crossed && right_is_crossed {
        return None;
    }

    let normal = if !left_is_crossed && !right_is_crossed {
        (pool.position_of(on_the_left) - corner).cross(pool.position_of(on_the_right) - corner)
    } else {
        // One tangent line left: the end still stands in the plane through the corner holding that
        // line's own vertex and the profile's own direction there, which is the chord between the
        // two rails nearest the untaken side.
        let standing = if left_is_crossed {
            on_the_right
        } else {
            on_the_left
        };
        let rail = if left_is_crossed {
            strip.rail_count() - 1
        } else {
            0
        };
        let next = if left_is_crossed {
            strip.rail_count() - 2
        } else {
            1
        };

        (pool.position_of(standing) - corner)
            .cross(strip.point_on(next, 0.0) - strip.point_on(rail, 0.0))
    };

    if !(normal.length_squared() > 0.0) {
        return None;
    }

    Some(normal.get_normal())
}

/// Whether no corner of any of `faces` stands within the pool's ruler of `at`.
pub fn nothing_stands_at(pool: &SurfaceVertexPool, faces: &[(i32, i32, i32)], at: Vector3) -> bool {
    !faces.iter().any(|&(v0, v1, v2)| {
        [v0, v1, v2]
            .into_iter()
            .any(|corner| (pool.position_of(corner) - at).length() < pool.near_enough())
    })
}
