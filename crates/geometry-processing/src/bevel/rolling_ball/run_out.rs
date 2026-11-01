//! The round running out at a free end of its chain ([`WalkEnd::RunOut`](super::WalkEnd)): where
//! one side narrows to nothing at the chain's end, as the T Handle's pocket wall does at the tip
//! between the chain and its own far rim, the ball rolls on and the round thins to nothing, its
//! two contacts converging onto the chain. The overview is the module header
//! (`rolling_ball/mod.rs`); the walk that calls this is `walk.rs`.
//!
//! Where the contacts meet, the two sides' distances grow the same way across the station's plane,
//! so no station there has a single centre, and the walk's step onto it fails. On exact geometry
//! they meet at the chain's end vertex, the width closing smoothly; on a mesh the narrowing side's
//! last triangles bring its rim onto the chain's last edge, so they meet a little before the end,
//! where the width can drop to nothing at once as a station's plane passes a vertex. Either way the
//! walk accepts the chain's end there only when the geometry says the round has run out, not on
//! whatever made the step fail:
//!
//! - **The contacts meet on the chain.** The failing step is tried whole, then halved toward the
//!   last ball that solves until the shares can no longer be told apart. Some ball along it, solved
//!   or not, must touch both sides at one point (the walk's own tolerance for one point) that lies
//!   on the chain's edge between the last solved station and the failing one. A failure for any
//!   other reason - a jump to another answer, a ball reaching into the part, contacts that meet on
//!   a rim away from the chain - finds no such ball.
//! - **The round does not come back.** Every chain point past the meeting point, to the end, has no
//!   ball: none rolled on there from the zero-width ball at the meeting point, its contacts
//!   followed over each side's surface as the walk follows them, and none grown afresh there. A
//!   round that thins to a point mid-chain and widens again past it is no run-out, and the walk
//!   still refuses it. Both are asked because each sees what the other cannot: a ball grown afresh
//!   reads each side only within its half of the corner, so it misses a round that comes back
//!   resting across that plane on a curved side, while one rolled on misses a round that comes
//!   back only out of its reach.
//!
//! The end station is then the ball at the meeting point: its `chain_point` that point, both
//! contacts that one point at the radius from the centre, and the centre a radius from it toward
//! the ball that found it. Its contacts keep the features and triangles that ball read there.

use super::ball_station::{BallStation, BallStationError};
use super::side_surface::SideContact;
use super::walk::{jumped, meeting_tolerance, ChainPoint, StationFailure, Walk};
use crate::bevel::level_line::ChainField;

/// The most times the failing step is halved: past this an f64 share stops changing.
const MOST_BISECTIONS: usize = 64;

impl Walk<'_, '_> {
    /// The zero-width end station where the round runs out between `rest[0]`, whose ball is
    /// `solved`, and `rest[1]`, whose step failed, with `rest[2..]` the chain's points beyond, to
    /// its free end; `None` when the geometry shows no run-out there (see the file header).
    pub(super) fn run_out(&self, rest: &[ChainPoint], solved: BallStation) -> Option<BallStation> {
        let (from, to) = (rest.first()?, rest.get(1)?);
        let radius = self.ball.radius;

        // The failing step whole, then halved: `low` is the share whose ball solves, `high` one
        // whose ball does not.
        let (mut low, mut high, mut share) = (0.0_f64, 1.0_f64, 1.0_f64);
        let (mut low_point, mut ball) = (*from, solved);
        let mut meeting = None;
        for _ in 0..=MOST_BISECTIONS {
            let point = from.toward(to, share);
            let moved = point.point - low_point.point;
            let outcome = self.checked(&point, ball.centre + moved, radius, Some(&ball));
            let centre = match &outcome {
                Ok(next) => Some(next.centre),
                Err(StationFailure::Fault { centre, .. })
                | Err(StationFailure::Unsolved(
                    BallStationError::SidesParallel { centre }
                    | BallStationError::DidNotConverge { centre, .. },
                )) => Some(*centre),
                Err(_) => None,
            };
            meeting = centre
                .and_then(|c| self.meeting(&point, (from, to), c, &ball))
                .map(|station| (point, station));
            if meeting.is_some() {
                break;
            }
            match outcome {
                Ok(next) if !jumped(&ball, &next, moved.length(), radius) => {
                    low = share;
                    low_point = point;
                    ball = next;
                }
                _ => high = share,
            }
            share = (low + high) * 0.5;
            if share <= low || share >= high {
                break;
            }
        }
        let (at, end) = meeting?;

        // The round must not come back before the end: no ball rolled on from the meeting point,
        // and none grown afresh, at any chain point past it.
        let one_point = meeting_tolerance(radius);
        let meeting_point = ChainPoint {
            point: end.chain_point,
            ..at
        };
        let beyond = rest[1..]
            .iter()
            .filter(|p| (p.point - end.chain_point).length() > one_point);
        for point in beyond {
            if self.carried(&meeting_point, end, point, 0).is_ok()
                || self.grown_station(point).is_ok()
            {
                return None;
            }
        }

        Some(end)
    }

    /// The zero-width ball where a ball centred at `centre`, reading the sides as a station at
    /// `point` rolled on from `rolled_from` does, touches both at one point on the chain's edge
    /// `step` (the last solved station's point to the failing one's); `None` where its contacts
    /// are two points or off that edge.
    #[allow(clippy::neg_cmp_op_on_partial_ord)] // `!(x > 0)` on purpose: a NaN has no meeting
    fn meeting(
        &self,
        point: &ChainPoint,
        step: (&ChainPoint, &ChainPoint),
        centre: vector_math::Vector3,
        rolled_from: &BallStation,
    ) -> Option<BallStation> {
        let radius = self.ball.radius;
        let one_point = meeting_tolerance(radius);
        let (a, b) = self.contacts(point, centre, radius, Some(rolled_from))?;
        if (a.point - b.point).length() > one_point {
            return None;
        }
        let middle = (a.point + b.point) * 0.5;
        let on_chain = ChainField::closest_on_segment(middle, step.0.point, step.1.point);
        let outward = centre - on_chain;
        if (middle - on_chain).length() > one_point || !(outward.length() > 0.0) {
            return None;
        }

        let at = |contact: SideContact| SideContact {
            point: on_chain,
            distance: radius,
            ..contact
        };
        Some(BallStation {
            chain_point: on_chain,
            tangent: point.tangent.get_normal(),
            centre: on_chain + (outward.get_normal() * radius),
            contact_a: at(a),
            contact_b: at(b),
            steps: 0,
        })
    }
}
