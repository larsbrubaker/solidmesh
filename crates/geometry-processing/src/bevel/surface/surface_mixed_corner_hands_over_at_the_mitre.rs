//! A mixed corner that hands over on a MITRE ring the chain already had: the plane the band stops
//! on where an edge meets its neighbour ([`mitre_with`]) and the reading of which chord carries the
//! corner's ring ([`try_read`]).

use super::{surface_handover_leaves_a_strip, surface_plan_band_ends};
use super::{SurfaceBandEnd, SurfaceEdgePlan, SurfaceMixedCorner};

/// The plane `edge`'s band stops on where it meets `neighbour`: the chain's own mitre, written
/// the way the plan writes it so that the ring and the band stop on one plane rather than on two
/// that nearly agree. `ahead` is whether the neighbour is the edge the chain LEAVES this one for.
pub fn mitre_with(
    planned: &[SurfaceEdgePlan],
    edge: usize,
    neighbour: usize,
    ahead: bool,
) -> SurfaceBandEnd {
    // Through the CORNER, and not through the crossing of the two tangent lines an ordinary
    // mitre's plane stands at (SurfaceMitreSeam): this is a MIXED corner, where the convexity
    // turns over and the two bands do not meet on a chain mitre at all. Measured: reading the seam
    // here moves the hand-over ring 3.8 mm along one of its own chords, from (28.91, -2.54, 20) to
    // (25.55, -0.83, 20).
    let here = planned[edge].edge.direction();
    let there = planned[neighbour].edge.direction();

    if ahead {
        surface_plan_band_ends::band_end_where_the_chain_leaves_along(
            planned[edge].to(),
            here,
            there,
        )
    } else {
        surface_plan_band_ends::band_end_where_the_chain_arrives_along(
            planned[edge].from(),
            there,
            here,
        )
    }
}

/// Whether the chord that touches the bead (`touching`, the edge standing `2r` from the bead's
/// axis) can carry the corner's ring itself, and where the ring goes when it cannot - as
/// `(carries_the_ring, ring_stops_at)`, the second `Some` only where the ring is the mitre ring.
/// The reading always succeeds, so this is no `Result`. `towards_the_corner` is whether the corner
/// lies FORWARD along the chain from the touching chord (so the chord before it has a lower index);
/// `read` is the corner as read with the touching chord.
///
/// Read as a BAND, not as a distance: the question is whether the touching chord has any of its
/// tangent line left between the handover plane and its own mitre (on "Every Chamfer Works" the
/// handover stands 0.0547 past that mitre, leaving the cap a strip of one rail). Where it has none
/// the mitre ring the chain already has is very nearly the ring the blend wants (0.0077 off the
/// ideal torus against chords that sag 0.0308), so the chord before carries the ring.
pub fn try_read(
    planned: &[SurfaceEdgePlan],
    is_closed: bool,
    touching: usize,
    towards_the_corner: bool,
    read: &SurfaceMixedCorner,
) -> (usize, Option<SurfaceBandEnd>) {
    let count = planned.len() as i64;
    let mut before = if towards_the_corner {
        touching as i64 - 1
    } else {
        touching as i64 + 1
    };

    if before < 0 || before >= count {
        if !is_closed {
            return (touching, None);
        }

        before = (before + count) % count;
    }

    let before = before as usize;

    // The touching chord's band on the face it does NOT share with the bead - the cap - which is
    // the side a handover end is written on and so the side that can be left with nothing.
    let side = planned[touching].left_region_key() != read.shared_region_key;
    let line = if side {
        planned[touching].left_tangent_line
    } else {
        planned[touching].right_tangent_line
    };

    let ends = [
        read.band_end_for(&planned[touching]),
        mitre_with(planned, touching, before, !towards_the_corner),
    ];

    if surface_handover_leaves_a_strip::leaves_a_band(&ends, &line) {
        return (touching, None);
    }

    // It has none, so the ring is the mitre ring: the chord BEFORE it carries the ring, cut at the
    // very plane its own band already stops on, and the touching chord goes under the blend with
    // the chords between.
    (
        before,
        Some(mitre_with(planned, before, touching, towards_the_corner)),
    )
}
