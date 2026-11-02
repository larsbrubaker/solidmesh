//! What a rolling ball may reach: the part only where its round can meet it, and its own sides
//! never. The walk that asks is `walk.rs` ([`Walk::fault`](super::walk) and `Walk::checked`); the
//! overview is the module header (`rolling_ball/mod.rs`).
//!
//! The part. A round only meets what crosses its arc, and anything crossing the arc passes through
//! the ball's circular segment between the chord through its two contacts and the arc
//! ([`round_segment`]); what the ball overlaps on the centre's side of that chord the round never
//! touches, so only the part within the segment can refuse a ball
//! (`StationFault::ReachesPastThePart`).
//!
//! Its own sides. A rolled ball's contact is followed over its side's surface from the last
//! station's (`contact_tracking.rs`), so it never leaps to another part of the side. But a side can
//! hold another part the ball comes upon as it rolls - a ridge or bump on the wall ahead, the far
//! face of a dent - which a contact sliding over the surface cannot reach: the distance rises
//! between them. The ball cannot pass through it either: once that part stands nearer the centre
//! than the radius, the ball rests on it ([`Walk::onto_the_sides`]), the station solved again with
//! that side's contact followed from there. Where the contact a station settles on depended on the
//! path its solve took, a part placed elsewhere in space (its numbers rounded differently) would
//! otherwise settle on the other surface and build a different round. A side's other part standing
//! no nearer than the radius - a side wrapping round behind the other, where a whole reading of the
//! side would answer with it - is never taken: only a ball that would pass into it moves onto it.

use vector_math::Vector3;

use super::ball_station::{ball_station_reading, BallStation, SideReading};
use super::side_surface::{HalfSpace, SideContact};
use super::walk::{StationFailure, StationFault, Walk, REACH_RELATIVE};

/// How many times a station is solved again on a side's nearer part before it is refused: each
/// move lands on surface strictly nearer than the last contact, so a ball that keeps moving is
/// caught between parts of its sides, a corner it cannot rest in.
const MOST_SIDE_MOVES: usize = 4;

/// How far from parallel, as the sine of the angle between them, the directions from a ball's
/// centre to its two contacts must stand for the round's segment between them to bound the part's
/// reach ([`round_segment`]).
const SEGMENT_LEAST_SINE: f64 = 1e-9;

/// The part of space the round at `station` can meet: the half-space beyond the chord through its
/// two contacts, away from the centre, bounded by the plane holding the chord and the chain's
/// direction. Within the ball that is the circular segment between the chord and the arc beyond it
/// (the round is that arc), so the part within the ball and this half is all of it the round can
/// meet: any surface crossing the arc passes through the segment just inside it, and what the ball
/// overlaps on the centre's side of the chord the round never touches. `None`, so the whole ball
/// counts, where the two contacts stand in line with the centre (or one stands on the chain's line
/// through it): no chord; and where the chain point does not stand beyond the chord, so the round
/// is the arc on the centre's side (longer than a half circle) and the segment beyond the chord is
/// not its own.
#[allow(clippy::neg_cmp_op_on_partial_ord)] // `!(x > y)` on purpose: a NaN counts the whole ball
pub(super) fn round_segment(station: &BallStation) -> Option<HalfSpace> {
    let along = station.tangent;
    let across = |v: Vector3| v - (along * v.dot(along));
    let to_a = across(station.contact_a.point - station.centre);
    let to_b = across(station.contact_b.point - station.centre);
    let sine = to_a.cross(to_b).length();
    let lengths = to_a.length() * to_b.length();
    if !(sine > SEGMENT_LEAST_SINE * lengths) {
        return None;
    }
    // The chord's plane holds the chain's direction and the chord; kept away from the centre,
    // the side contact A's direction from the centre points to.
    let normal = along.cross(to_b - to_a);
    let normal = if normal.dot(to_a) < 0.0 {
        -normal
    } else {
        normal
    };
    let beyond = HalfSpace {
        origin: station.contact_a.point,
        normal: normal.get_normal(),
    };
    if !(beyond.depth(station.chain_point) > 0.0) {
        return None;
    }
    Some(beyond)
}

impl Walk<'_, '_> {
    /// `station`, solved in the plane through `origin` square to `normal` with its contacts
    /// followed within `halves`, moved onto any part of either side that stands nearer its centre
    /// than `radius` (see the file header), and solved again there; unchanged where neither side
    /// does. A ball still passing into a side after [`MOST_SIDE_MOVES`] moves is refused, the
    /// side's nearest point named as what it reaches.
    pub(super) fn onto_the_sides(
        &self,
        mut station: BallStation,
        (origin, normal): (Vector3, Vector3),
        radius: f64,
        (halves_a, halves_b): (&[HalfSpace], &[HalfSpace]),
    ) -> Result<BallStation, StationFailure> {
        let inside = radius - (REACH_RELATIVE * (1.0 + radius.abs()));
        let (chain_point, tangent) = (station.chain_point, station.tangent);
        for moves in 0..=MOST_SIDE_MOVES {
            let nearer = |side: &super::side_surface::RollingBallSide,
                          halves: &[HalfSpace],
                          contact: SideContact| {
                side.closest_within(station.centre, halves)
                    .filter(|nearest| nearest.distance < inside)
                    .unwrap_or(contact)
            };
            let from_a = nearer(self.ball.side_a, halves_a, station.contact_a);
            let from_b = nearer(self.ball.side_b, halves_b, station.contact_b);
            if from_a == station.contact_a && from_b == station.contact_b {
                return Ok(station);
            }
            if moves == MOST_SIDE_MOVES {
                let reach = if from_a == station.contact_a {
                    from_b
                } else {
                    from_a
                };
                return Err(StationFailure::Fault {
                    centre: station.centre,
                    fault: StationFault::ReachesPastThePart { reach },
                });
            }
            station = ball_station_reading(
                (
                    self.ball.side_a,
                    SideReading::TrackedFrom {
                        contact: from_a,
                        touched: None,
                        halves: halves_a,
                    },
                ),
                (
                    self.ball.side_b,
                    SideReading::TrackedFrom {
                        contact: from_b,
                        touched: None,
                        halves: halves_b,
                    },
                ),
                origin,
                normal,
                station.centre,
                radius,
            )
            .map_err(StationFailure::Unsolved)?;
            station.chain_point = chain_point;
            station.tangent = tangent;
        }
        Ok(station)
    }
}
