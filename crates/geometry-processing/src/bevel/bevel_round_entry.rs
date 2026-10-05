//! Where the rolling ball enters a round while the crease is still on the flat: cuts the one
//! crease edge that straddles that moment, so no swept segment has to stand for both sides of it
//! at once. Ports `Bevel/BevelRoundEntry.cs`. Reads [`BevelRoundFaces`] (which faces are a
//! round's tangent planes) and [`BevelRoundDescriptor::try_find_station`]; cuts with
//! `FeatureEdge::over_part_of_its_length`. Its caller is `BevelChainSweep.TryCreate`.
//!
//! **The lead.** A round runs out tangentially onto a flat NOSE face, and the second pass's crease
//! crosses that face obliquely before it ever reaches the round's own facets. The ball rolling in
//! the crease does not wait: its contact sits a radius away from the crease, so it climbs onto the
//! round's angular span while the crease point is still up to a radius back on the flat - 1.1 mm
//! on the "Inner Right Sides and Center" fixture at a size of 2. The flat's crease edge is a
//! single mesh edge 3.9 mm long carrying ONE planar section anchored at its midpoint, so that last
//! 1.1 mm is drawn against a plane the ball has already left, and the mitre at the round's first
//! vertex then averages that stale planar section with the arc section next to it. The lower bead
//! of that fixture read 0.033/0.045 off the canal for exactly this, having been 0.009/0.015 before
//! the round sections existed at all.
//!
//! **The cut.** The edge is split at the point where the ball's station leaves the span, found by
//! bisecting the edge parameter against [`BevelRoundDescriptor::try_find_station`] - which answers
//! `None` precisely when the ball has rolled off onto the flat, and whose station runs
//! monotonically along the edge, so there is one crossing. The flat side stays one piece, still
//! exact against its own plane; the on-round stretch is chopped into pieces no longer than the
//! round's own facet chord, so a single section per piece is no coarser than the tessellation it
//! sits on.
//!
//! **The end edges are cut too, and their budget is re-derived after the cut.** A run's two free
//! ends are budgeted before this runs: `BevelChainEnds` holds each uncapped end's overshoot to half
//! the length of the edge that end sits on (`BevelChainEndBudget.MaximumOvershoot`), measured on
//! the whole edge. Cutting that edge would otherwise leave the bound quoting a length the sweep no
//! longer has, so `BevelChainSweep.TryCreate` takes the bounds down to half of the first and last
//! PIECES as soon as this hands back a chain it changed. Leaving the two end edges uncut was the
//! other way to keep the budget honest and it costs too much to take: on the nose-in-notch fixture
//! the crease ENTERS the round on its last edge, and left whole that edge carries one stale planar
//! section which stands 0.043 off the canal - four times the whole run's error - and differs from
//! the arc section next to it by exactly the quarter of the size at which `BevelMitreFit` would cut
//! the run in two.
//!
//! **What it is safe to cut.** Only the geometry of a chain edge changes - the pieces keep the
//! original's faces, normals, convexity, mesh edge and vertex ids (see
//! `FeatureEdge::over_part_of_its_length`). That is safe because this runs inside
//! `BevelChainSweep.TryCreate`, one call before `TryBuildSegments` and after everything that reads
//! the chain topologically has already read it: `BevelChainEnds.Resolve` (run's first and last edge
//! only, whose outer ends are untouched), `BevelSharedEdges` and `BevelMitreFit` (both on
//! resolutions and chains upstream of here), `BevelSweepBackDepth.RetryDepth` (given the caller's
//! own undivided chain). Everything downstream of `TryBuildSegments` works off the swept segments,
//! not the chain.

use std::f64::consts::PI;

use super::BevelRoundFaces;
use crate::topology::{ChainEdge, EdgeConvexity};

/// How far in from an edge's ends the on-round test is taken, and how near an end the split
/// point may land before the piece it would cut off is dropped as nothing.
const AT_THE_ENDS: f64 = 1e-6;

/// How many samples the edge is walked with, looking for the one place the ball leaves the round.
/// Only enough to catch a second crossing before the bisection is trusted - the crossing itself is
/// then found to machine precision by [`entry`] - and each sample is one station solve.
const EDGE_SAMPLES: i32 = 16;

/// The same chain with every edge that straddles a round's entry cut at the entry, or `None` where
/// none does and the chain stands as it came - which is every chain whose caller has no rounds at
/// all, and every chain that never crosses a round's tangent plane. (C# hands back the very list
/// it was given there, and `BevelChainSweep.TryCreate` tells the two apart by reference.)
///
/// `chain` is the run about to be swept, each edge oriented along the walk. `rounds` the rounds
/// known to be in the part - cut there by the first pass, or fitted off the source mesh where the
/// part arrived with them (`BevelSourceRounds`) - or `None` where the caller has none, which is
/// what keeps a caller with no rounds byte for byte where it was. `size` is the operation's size,
/// which is the rolling ball's radius. `round_profile` is whether the tool's profile is an arc: a
/// chamfer's section is not the rolling ball's at all, so there is no station to cut at - the same
/// gate `BevelChainSweep.TryStandOnTheRound` is behind.
pub fn subdivide(
    chain: &[ChainEdge],
    rounds: Option<&BevelRoundFaces>,
    size: f64,
    round_profile: bool,
) -> Option<Vec<ChainEdge>> {
    let rounds = rounds?;

    if !round_profile || chain.is_empty() {
        return None;
    }

    let mut cut: Option<Vec<ChainEdge>> = None;

    for (i, edge) in chain.iter().enumerate() {
        let Some(pieces) = pieces(edge, rounds, size) else {
            if let Some(cut) = cut.as_mut() {
                cut.push(*edge);
            }

            continue;
        };

        // Nothing was cut until now, so the run so far is copied over as it stands. Allocating
        // only here is what keeps this free on the chains - nearly all of them - that never cross
        // a round's tangent plane at all.
        let cut = cut.get_or_insert_with(|| {
            let mut so_far = Vec::with_capacity(chain.len() + pieces.len());
            so_far.extend_from_slice(&chain[..i]);
            so_far
        });

        cut.extend(pieces);
    }

    cut
}

/// What one edge becomes, or `None` where it is left alone: not a bead, not against a tangent
/// plane, or wholly on one side of the round's entry.
fn pieces(edge: &ChainEdge, rounds: &BevelRoundFaces, size: f64) -> Option<Vec<ChainEdge>> {
    if edge.edge.convexity != EdgeConvexity::Concave {
        return None;
    }

    let on_the_left = rounds.try_read_face(edge.left_face());
    let on_the_right = rounds.try_read_face(edge.right_face());

    // Exactly one side, as in TryStandOnTheRound: the section this is preparing for is a round
    // against a PLANE, and a crease between two rounds is a different problem.
    let (round, is_tangent_plane, round_is_on_the_left) = match (on_the_left, on_the_right) {
        (Some((round, tangent)), None) => (round, tangent, true),
        (None, Some((round, tangent))) => (round, tangent, false),
        _ => return None,
    };

    // A FACET's edge is already at most one chord long and lies wholly on the round, so there is
    // nothing to cut: only the flat face the round runs out onto straddles the entry.
    if !is_tangent_plane {
        return None;
    }

    let slope_normal = if round_is_on_the_left {
        edge.right_normal()
    } else {
        edge.left_normal()
    };
    let from = edge.from();
    let along = edge.to() - from;
    let length = along.length();

    if length.is_nan() || length <= 0.0 {
        return None;
    }

    // The round-side face's own plane, which is the facet the crease lies in - and, for a SOURCE
    // round, the second half of what "standing on the round" means.
    let round_face_normal = if round_is_on_the_left {
        edge.left_normal()
    } else {
        edge.right_normal()
    };

    // THE SAME QUESTION THE SECTION ASKS. BevelChainSweep.TryStandOnTheRound takes the round's own
    // section only where a station is found AND (for a source round) the contact has rolled off
    // the crease's facet; asking only the first here would leave an edge whole across the
    // second's own change and let one section stand for a stretch of crease built two different
    // ways. The facet half is gated on read_from_the_source_mesh exactly as it is there, so a
    // first-pass round's chain is cut in precisely the places it always was.
    let on_the_round = |at: f64| -> bool {
        let Some(station) = round.try_find_station(from + (along * at), from, slope_normal, size)
        else {
            return false;
        };

        !rounds.read_from_the_source_mesh()
            || round.contact_has_rolled_off_the_creases_facet(
                station.contact,
                from,
                round_face_normal,
            )
    };

    // SCANNED RATHER THAN ASSUMED. The station slides along the spine as the anchor slides along
    // the edge, so the stretch of the edge whose ball stands on the round is ONE interval - but
    // that rests on the station solve itself being single valued, which it is only while the
    // anchor is inside the spine's own radius of curvature (try_find_station). Where the scan
    // sees the answer change more than once that premise has failed somewhere along this edge,
    // and the edge is handed back uncut: its midpoint's own section still stands, and a cut made
    // at one of several crossings would be a guess.
    let mut transitions = 0;
    let mut low = 0.0;
    let mut high = 1.0;
    let mut was_on = on_the_round(AT_THE_ENDS);
    let at_from = was_on;
    let mut crossing_low = 0.0;
    let mut crossing_high = 0.0;

    for i in 1..=EDGE_SAMPLES {
        let at = AT_THE_ENDS + ((1.0 - (2.0 * AT_THE_ENDS)) * i as f64 / EDGE_SAMPLES as f64);
        let is_on = on_the_round(at);

        if is_on != was_on {
            transitions += 1;
            crossing_low =
                AT_THE_ENDS + ((1.0 - (2.0 * AT_THE_ENDS)) * (i - 1) as f64 / EDGE_SAMPLES as f64);
            crossing_high = at;
        }

        was_on = is_on;
    }

    if transitions == 0 {
        if !at_from {
            // The ball never touches the round anywhere along this edge, so the plain planar
            // section against the two faces is exact for the whole of it.
            return None;
        }
    } else if transitions == 1 {
        let entry = entry(&on_the_round, crossing_low, crossing_high, at_from);

        low = if at_from { 0.0 } else { entry };
        high = if at_from { entry } else { 1.0 };
    } else {
        // More than one crossing - the premise above has failed. Handed back uncut.
        return None;
    }

    // The round's own facet chord. A section standing for a piece no longer than that is no
    // coarser than the tessellation the crease is read off, which is the most that can be asked
    // of it.
    let chord = 2.0 * round.radius * (PI / round.segments as f64).sin();
    // C# `(int)Math.Ceiling(...)`; `as` saturates where C# would be undefined on a NaN or a
    // huge count, which only a degenerate round could produce.
    let pieces = 1.max((((high - low) * length) / chord).ceil() as i32);

    let mut cuts = vec![0.0];

    if low > AT_THE_ENDS {
        cuts.push(low);
    }

    for i in 1..pieces {
        cuts.push(low + ((high - low) * i as f64 / pieces as f64));
    }

    if high < 1.0 - AT_THE_ENDS {
        cuts.push(high);
    }

    cuts.push(1.0);

    if cuts.len() < 3 {
        return None;
    }

    let mut cut = Vec::with_capacity(cuts.len() - 1);

    for i in 1..cuts.len() {
        let a = from + (along * cuts[i - 1]);
        let b = from + (along * cuts[i]);

        // ChainEdge reads its from and to off the FeatureEdge THROUGH is_reversed, so a piece of
        // an edge the chain walks backwards has to be stored with its two ends swapped - the same
        // flip the linker made when it put the whole edge in the chain this way round.
        let piece = if edge.is_reversed {
            edge.edge.over_part_of_its_length(b, a)
        } else {
            edge.edge.over_part_of_its_length(a, b)
        };

        cut.push(ChainEdge::new(edge.edge_index, piece, edge.is_reversed));
    }

    Some(cut)
}

/// Where along the edge the ball's station crosses the end of the round's span, as a parameter in
/// [0, 1]. Plain bisection inside a bracket the caller's scan has already found the one crossing
/// in: `on_the_round` says whether the ball anchored at a parameter is standing on the round,
/// `bracket_low`/`bracket_high` the scanned interval the answer changes across, and
/// `from_is_on_the_round` which end of the edge is the one inside the span.
fn entry(
    on_the_round: &dyn Fn(f64) -> bool,
    bracket_low: f64,
    bracket_high: f64,
    from_is_on_the_round: bool,
) -> f64 {
    let mut on = if from_is_on_the_round {
        bracket_low
    } else {
        bracket_high
    };
    let mut off = if from_is_on_the_round {
        bracket_high
    } else {
        bracket_low
    };

    for _ in 0..60 {
        let middle = (on + off) / 2.0;

        if on_the_round(middle) {
            on = middle;
        } else {
            off = middle;
        }
    }

    (on + off) / 2.0
}
