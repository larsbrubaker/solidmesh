//! The rolling ball walked along a whole chain ([`RollingBall::walk`]): each station solved from
//! the one before it rather than from the two faces' flat estimate, and every station checked to
//! be a ball the fillet can use ([`RollingBall::fault`]). The overview is the module header
//! (`rolling_ball/mod.rs`); one station's solve is `ball_station.rs`.
//!
//! Why a walk. Away from the edge the flat estimate can be far off - where the face beside the edge
//! runs out, where a side curves - and Newton's method started there can settle on a centre that
//! satisfies both distances but is no fillet's ball: one resting on a rim both sides share (every
//! point a radius from such a rim is a radius from both sides), or one reaching round a rim from
//! behind. The flat estimate IS good close to the edge, so the walk starts there: at the chain's
//! middle station it solves a ball a sixteenth of the size, where the two faces beside the edge are
//! all the ball can reach, and grows it to the size asked in doublings, each solve starting from
//! the last centre scaled out from the chain point (exact for flat faces). It then carries the
//! solved centre to each next station in turn, both ways to the chain's ends, moved along with the
//! chain point. A step that fails, or whose centre jumps further than rolling can move it, is
//! halved along the chain until it holds; a station the carried centre still cannot reach is grown
//! afresh there.
//!
//! A ball seeded from nothing (the middle station, and one grown afresh where halving runs out)
//! reads each side only within its half of the corner ([`RollingBall::halves`]): the plane
//! through the chain holding its tangent and the bisector of the two faces beside the edge splits
//! space, and a side answers only with surface on its own side of it. A side region can wrap round
//! the other - a narrow wall with the same smooth surface beyond its far rim, as on the T Handle -
//! and read whole it would answer with that far surface or with the rim the two regions share,
//! where every point a radius off it is a radius off both sides and the solve has nothing to hold.
//! A ball rolled on from a neighbour has no corner half: each side's contact is followed over the
//! side's surface from the neighbour's (`contact_tracking.rs`), because a wide round on curved
//! sides touches a side across that plane (the T Handle's R12 ball near the web rests on the
//! pocket wall about 18 down its curve).
//!
//! What makes a ball usable ([`StationFault`]): nothing of the part is nearer its centre than the
//! radius (the ball does not reach into the solid - for a concave edge - or out of it - for a
//! convex one), its centre stands on the free-space side (concave) or solid side (convex) of each
//! side where it touches it, neither contact is only where its half was cut, and its two contacts
//! are not one point on a rim the sides share.
//!
//! Where a chain ends on a face that is neither of its sides ([`EndFace`], given per end to
//! [`RollingBall::walk_ending_on`]) - the T Handle's junction chains end on the pocket's floor -
//! one rule holds. The end face is neither a side nor an obstacle: the ball touches only the two
//! sides, each read as cut at the end face's plane and extended past it, each of its triangles
//! meeting the plane carried on as its own plane as far as the chain's line carried on past its
//! end vertex, where the trim at the chain ends it (`side_extension.rs`: what lies past the plane
//! of a side that wraps on past it, as the T Handle's pocket wall runs on through the web, is
//! never read), and nothing of the part on the end face or beyond its plane refuses it. Its
//! centre is kept on the chain's side of
//! the end face's plane, the side the chain approaches from: where the free centre would cross the
//! plane, the station is solved in the end plane instead of square to the chain, still at the
//! radius from both sides, so the plane takes the place of the station's own. The round keeps its
//! full radius right to the end, and the end face cuts it flat: the round a part gets by extending
//! the sides and trimming them at the end face. Where the sides meet the end face square, the ball
//! held in the plane touches each side on its trace in the plane, so the round's section there is
//! the end face outline's own fillet; where the face leans, the end stations touch the sides'
//! extensions, so on prism walls every station's contacts stand the walls' own distances from the
//! chain. An end face that is not flat near the chain's end is
//! read as its tangent plane at the chain's end vertex, which the caller gives.
//!
//! A free end (no end face) where one side narrows to nothing can end the round with no width: the
//! ball's two contacts meet on the chain ([`StationFault::ContactsMeet`]), and toward a free end,
//! with no ball anywhere past that point, the walk closes there with a zero-width end station
//! (`run_out.rs`). The result says how each end finished ([`WalkEnd`]). Each side is read a hair
//! past its half of the corner, because the contacts converge onto the chain there and the chain
//! lies in the corner's plane.

use vector_math::Vector3;

use super::ball_station::{
    ball_station_reading, flat_face_centre, BallStation, BallStationError, SideReading,
    SOLVED_RELATIVE,
};
use super::side_extension::{extended_past, ChainEnd};
use super::side_surface::{HalfSpace, RollingBallSide, SideContact};

/// How far beyond an end face's plane a free centre may stand, relative to one plus the radius,
/// and still count as in it: a hundred times the tolerance the solve holds its distances to
/// ([`SOLVED_RELATIVE`]) and far above the rounding of a centre solved in the plane, so a centre
/// that stands in the plane to within what the solve can tell is not held again, and only a real
/// crossing is.
const END_PLANE_RELATIVE: f64 = 1e-9;

/// How far inside the chain's side of an end face's plane the part starts to count as an obstacle,
/// relative to one plus the radius: wide enough that the end face's own triangles, flat only to the
/// precision the mesh was stored at, fall wholly outside, and far below any feature of a part.
const END_FACE_GAP_RELATIVE: f64 = 1e-6;

/// How far past its half of the corner a side is still read, relative to one plus the radius, and
/// how near an end face's plane a side's vertex counts as on it (`side_extension.rs`). A side
/// meets the corner's plane on its own surface - the chain lies in it - and a contact there is
/// surface, not cut; a plane a blended direction tilts by a
/// hair would otherwise hide it, as where the round runs out and the contacts converge onto the
/// chain. Its size is the end face gap's ([`END_FACE_GAP_RELATIVE`]): wide against the tilt a
/// blended direction gives a plane between stations, and far below any feature of a part.
const READING_PAST_RELATIVE: f64 = END_FACE_GAP_RELATIVE;

/// How square to the chain an end face's normal must stand (the cosine between them) for the
/// chain to approach it from one side.
const END_FACE_LEAST_COSINE: f64 = 1e-9;

/// The ball the walk grows first, as a share of the radius asked.
const FIRST_SHARE: f64 = 1.0 / 16.0;

/// The most times a growing step is halved before the station gives up.
const MOST_GROWTH_HALVINGS: usize = 6;

/// The most sizes a ball grows through on its way to the radius. Four doublings take it from a
/// sixteenth to the whole; the rest leave room for doublings that succeed only in part. A ball that
/// fails only at sizes very near the radius would otherwise creep toward it by halves, each pass
/// gaining half of what is left, for some fifty passes of up to seven solves each, and then fail.
const MOST_GROWTH_PASSES: usize = 32;

/// The most times a step along the chain is halved before the walk regrows the station.
const MOST_STEP_HALVINGS: usize = 6;

/// How much nearer than the radius the part may come to the centre, relative to one plus the
/// radius: the solve's own tolerance with room to spare, so only a real reach counts.
const REACH_RELATIVE: f64 = 1e-9;

/// How close two contacts must be, relative to one plus the radius, to count as one point.
pub(super) const SAME_POINT_RELATIVE: f64 = 1e-7;

/// How close a ball's two contacts stand when they meet, for a ball of `radius`: where they meet,
/// each side's distance from the centre grows only as the square of the gap between them over
/// twice the radius, so a solve held to its tolerance on the distances places the contacts only to
/// within the square root of twice the radius times that tolerance.
pub(super) fn meeting_tolerance(radius: f64) -> f64 {
    (2.0 * radius.abs() * (SOLVED_RELATIVE * (1.0 + radius.abs()))).sqrt()
}

/// How far behind a side its contact's centre may stand, relative to one plus the radius.
const WRONG_SIDE_RELATIVE: f64 = 1e-9;

/// How far a centre may move in one step along the chain: twice the chain point's move plus this
/// share of the radius. Rolling moves the centre about as far as the chain point, more where a
/// side curves or the contact slides over a rim; a step past this is a jump to another answer.
const JUMP_RADIUS_SHARE: f64 = 0.25;

/// One point of the chain the ball rolls along.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct ChainPoint {
    /// The point.
    pub point: Vector3,
    /// The chain's unit direction there.
    pub tangent: Vector3,
    /// Side A's outward unit normal beside the edge there, for the first estimate.
    pub normal_a: Vector3,
    /// Side B's.
    pub normal_b: Vector3,
}

impl ChainPoint {
    /// The point `share` of the way from `self` to `to`, its directions blended and made unit
    /// again (or `to`'s where the blend vanishes).
    pub(super) fn toward(&self, to: &ChainPoint, share: f64) -> ChainPoint {
        let blend = |a: Vector3, b: Vector3| {
            let mixed = (a * (1.0 - share)) + (b * share);
            if mixed.length() > 0.0 {
                mixed.get_normal()
            } else {
                b
            }
        };
        ChainPoint {
            point: self.point + ((to.point - self.point) * share),
            tangent: blend(self.tangent, to.tangent),
            normal_a: blend(self.normal_a, to.normal_a),
            normal_b: blend(self.normal_b, to.normal_b),
        }
    }
}

/// The face a chain ends on that is neither of its sides, read as its plane at the chain's end: a
/// point of the plane (the chain's end vertex) and its normal, either way round. The walk keeps
/// the ball's centre on the side of the plane the chain approaches from (see the file header).
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct EndFace {
    /// A point of the plane.
    pub point: Vector3,
    /// The plane's normal; its length and direction do not matter.
    pub normal: Vector3,
}

/// The ball of one fillet chain: its two sides, the whole part it must keep clear of, its radius,
/// and whether the edge is convex (the ball inside the solid) or concave (in free space).
#[derive(Debug, Clone, Copy)]
pub struct RollingBall<'a> {
    /// Side A: everything across the edge on one side.
    pub side_a: &'a RollingBallSide,
    /// Side B.
    pub side_b: &'a RollingBallSide,
    /// The whole part, which the ball may not reach past; `None` reads the two sides only.
    pub part: Option<&'a RollingBallSide>,
    /// The fillet's radius.
    pub radius: f64,
    /// Whether the edge is convex.
    pub convex: bool,
}

/// Why a solved station is no ball the fillet can use.
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum StationFault {
    /// Something of the part is nearer the centre than the radius: the ball reaches into the
    /// solid (concave) or out of it (convex). `reach` is the part's nearest point.
    ReachesPastThePart { reach: SideContact },
    /// The centre stands on the wrong side of a side where it touches it - reaching round a rim
    /// from behind. `side_b` names the side; `by` is how far (negative).
    WrongSideOf { side_b: bool, by: f64 },
    /// Both contacts are one point on a rim the two sides share: every point a radius from it is
    /// a radius from both sides, so the ball is pinned by nothing.
    SharedRimPoint { point: Vector3 },
    /// A side's nearest point is where its half of the corner ends, not its surface: the ball
    /// touches nothing of that side. `side_b` names the side.
    OnTheCut { side_b: bool },
    /// Both contacts are one point at the station's chain point: the round has no width there.
    /// Toward a free end it is where the round runs out (`run_out.rs`); anywhere else the walk is
    /// refused there.
    ContactsMeet { point: Vector3 },
}

/// Why a station along the walk has no ball.
#[derive(Debug, Clone, PartialEq)]
pub enum StationFailure {
    /// The two faces beside the edge fold flat on each other, so there is no first estimate.
    NoFirstEstimate,
    /// The solve did not find a centre.
    Unsolved(BallStationError),
    /// It found one the fillet cannot use.
    Fault {
        centre: Vector3,
        fault: StationFault,
    },
    /// The centre moved further in one step than rolling can move it, however short the step.
    Jumped { from: Vector3, to: Vector3 },
    /// The ball grown at a station reached `size` and could grow no nearer the radius within the
    /// passes a growth is allowed.
    DidNotGrow { size: f64 },
}

/// How a walk finished at one end of its chain.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum WalkEnd {
    /// A free end the ball reached with width: the end station is the ball at the chain's end
    /// point.
    LastStation,
    /// The chain ends on a face that is neither side ([`EndFace`]): the end station is the ball at
    /// the chain's end point, its centre kept on the chain's side of the face's plane.
    EndFace,
    /// The round runs out to nothing at a free end (`run_out.rs`): the end station has zero width,
    /// both its contacts the one point of the chain where they met, its `chain_point`. The `past`
    /// chain points beyond the meeting point, toward the end, have no station.
    RunOut { past: usize },
}

/// A chain walked: its stations in chain order and how each end finished.
#[derive(Debug, Clone, PartialEq)]
pub struct ChainWalk {
    /// One station per chain point, except at a run-out end, where the zero-width end station
    /// stands in for the chain points past the meeting point.
    pub stations: Vec<BallStation>,
    /// How the chain's first end (`ends[0]`) and last end finished.
    pub ends: [WalkEnd; 2],
}

/// Why a chain has no walk.
#[derive(Debug, Clone, PartialEq)]
pub enum WalkError {
    /// The chain has no points.
    NoPoints,
    /// The radius is not a positive finite size.
    NotARadius,
    /// An end face's plane holds the chain's direction at that end (or has no normal), so the
    /// chain approaches it from neither side. `last` names the chain's last end.
    EndFaceAlongTheChain { last: bool },
    /// Station `index` has no ball.
    At {
        index: usize,
        failure: StationFailure,
    },
}

impl RollingBall<'_> {
    /// The ball at every point of the chain, in order, each solved from its neighbour's centre
    /// (see the file header); the first station that has none ends the walk with the reason.
    /// Both ends are free: [`walk_ending_on`](Self::walk_ending_on) with no end faces.
    pub fn walk(&self, points: &[ChainPoint]) -> Result<ChainWalk, WalkError> {
        self.walk_ending_on(points, [None, None])
    }

    /// [`walk`](Self::walk) for a chain ending on a face that is neither side at either end:
    /// `ends[0]` the face its first point lies on, `ends[1]` its last's. Each end face is no
    /// obstacle, and the ball's centre stays on the chain's side of its plane, held in the plane
    /// where it would cross (see the file header). A station so held has its centre in the end
    /// face's plane; its `chain_point` and `tangent` are still the chain's. With any end face,
    /// each side is read as extended past it (`side_extension.rs`), and every contact's triangle
    /// and feature indices name the extended side's own triangle list and vertices (the part's
    /// vertices keep their indices; the cut's and the strips' come after them), not the side's.
    #[allow(clippy::neg_cmp_op_on_partial_ord)] // `!(x > 0)` on purpose: a NaN refuses
    pub fn walk_ending_on(
        &self,
        points: &[ChainPoint],
        ends: [Option<EndFace>; 2],
    ) -> Result<ChainWalk, WalkError> {
        if points.is_empty() {
            return Err(WalkError::NoPoints);
        }
        if !(self.radius > 0.0) || !self.radius.is_finite() {
            return Err(WalkError::NotARadius);
        }

        // Each end's plane as the half of space the chain approaches it from: from the first
        // point the chain runs on along its tangent, into the last against it.
        // The chain's end vertex and its direction carried on past it bound each side's
        // extension there (`side_extension.rs`).
        let mut kept: Vec<HalfSpace> = Vec::new();
        let mut chain_ends: Vec<ChainEnd> = Vec::new();
        for (end, into, at, last) in [
            (ends[0], points[0].tangent, points[0].point, false),
            (
                ends[1],
                -points[points.len() - 1].tangent,
                points[points.len() - 1].point,
                true,
            ),
        ] {
            let Some(end) = end else { continue };
            let length = end.normal.length();
            if !(length > 0.0) || !length.is_finite() {
                return Err(WalkError::EndFaceAlongTheChain { last });
            }
            let normal = end.normal / length;
            let cosine = normal.dot(into.get_normal());
            if !(cosine.abs() > END_FACE_LEAST_COSINE) {
                return Err(WalkError::EndFaceAlongTheChain { last });
            }
            let plane = HalfSpace {
                origin: end.point,
                normal: if cosine > 0.0 { normal } else { -normal },
            };
            kept.push(plane);
            chain_ends.push(ChainEnd {
                plane,
                vertex: at,
                along: -into,
            });
        }
        // Each side read as cut at every end face's plane and extended past it as its own planes
        // (`side_extension.rs`): the round the sides would make run on past the end face.
        let scale = 1.0 + self.radius.abs();
        let extended = (!kept.is_empty()).then(|| {
            let tolerance = READING_PAST_RELATIVE * scale;
            (
                extended_past(self.side_a, &chain_ends, self.radius, tolerance),
                extended_past(self.side_b, &chain_ends, self.radius, tolerance),
            )
        });
        let ball = match &extended {
            Some((side_a, side_b)) => RollingBall {
                side_a,
                side_b,
                ..*self
            },
            None => *self,
        };
        let walk = Walk {
            ball: &ball,
            ends: &kept,
        };

        let at = |index: usize| move |failure| WalkError::At { index, failure };
        let seed = points.len() / 2;
        let first = walk.grown_station(&points[seed]).map_err(at(seed))?;
        let mut stations: Vec<Option<BallStation>> = vec![None; points.len()];
        stations[seed] = Some(first);

        // Toward each end in turn, from the seed out; a free end may run out (`run_out.rs`).
        let mut finished = [WalkEnd::LastStation; 2];
        let mut run_outs: [Option<BallStation>; 2] = [None, None];
        let toward: [Vec<usize>; 2] = [
            (0..seed).rev().collect(),
            (seed + 1..points.len()).collect(),
        ];
        for end in [1, 0] {
            if ends[end].is_some() {
                finished[end] = WalkEnd::EndFace;
            }
            let mut last = first;
            let mut previous = seed;
            for (step, &index) in toward[end].iter().enumerate() {
                match walk.carried(&points[previous], last, &points[index], 0) {
                    Ok(station) => {
                        stations[index] = Some(station);
                        last = station;
                        previous = index;
                    }
                    Err(failure) => {
                        let rest: Vec<ChainPoint> = std::iter::once(previous)
                            .chain(toward[end][step..].iter().copied())
                            .map(|i| points[i])
                            .collect();
                        let run_out = match ends[end] {
                            None => walk.run_out(&rest, last),
                            Some(_) => None,
                        };
                        let Some(station) = run_out else {
                            return Err(at(index)(failure));
                        };
                        run_outs[end] = Some(station);
                        finished[end] = WalkEnd::RunOut {
                            past: toward[end].len() - step,
                        };
                        break;
                    }
                }
            }
        }

        let [head, tail] = run_outs;
        let stations = head
            .into_iter()
            .chain(stations.into_iter().flatten())
            .chain(tail)
            .collect();
        Ok(ChainWalk {
            stations,
            ends: finished,
        })
    }

    /// Why `station`, solved for a ball of `radius`, is no ball the fillet can use; `None` when
    /// it is one.
    pub fn fault(&self, station: &BallStation, radius: f64) -> Option<StationFault> {
        Walk {
            ball: self,
            ends: &[],
        }
        .fault(station, radius)
    }

    /// The ball at `point` grown from a small one: solved at a sixteenth of the radius from the
    /// flat estimate, then doubled to the radius, each size started from the last centre scaled
    /// out from the chain point, a size that fails approached in halved steps.
    pub fn grown_station(&self, point: &ChainPoint) -> Result<BallStation, StationFailure> {
        Walk {
            ball: self,
            ends: &[],
        }
        .grown_station(point)
    }
}

/// A walk's ball with the halves of space its chain's end faces keep it in (none for free ends).
pub(super) struct Walk<'w, 'a> {
    pub(super) ball: &'w RollingBall<'a>,
    pub(super) ends: &'w [HalfSpace],
}

impl Walk<'_, '_> {
    /// [`RollingBall::fault`], with the part read only on the chain's side of each end face, a
    /// little way in, so neither the end face nor anything beyond it counts as reached.
    fn fault(&self, station: &BallStation, radius: f64) -> Option<StationFault> {
        let scale = 1.0 + radius.abs();
        let gap = END_FACE_GAP_RELATIVE * scale;
        let clear_of_ends: Vec<HalfSpace> = self
            .ends
            .iter()
            .map(|end| HalfSpace {
                origin: end.origin + (end.normal * gap),
                normal: end.normal,
            })
            .collect();
        if let Some(reach) = self
            .ball
            .part
            .and_then(|part| part.closest_within(station.centre, &clear_of_ends))
        {
            if reach.distance < radius - (REACH_RELATIVE * scale) {
                return Some(StationFault::ReachesPastThePart { reach });
            }
        }

        let behind = self.ball.convex;
        for (side, contact, side_b) in [
            (self.ball.side_a, &station.contact_a, false),
            (self.ball.side_b, &station.contact_b, true),
        ] {
            // A contact held only by triangles with no area faces no way, so it gives no sign of
            // the ball reaching round a rim; the other checks still read it.
            let Some(by) = side.standing_off(contact, station.centre, behind) else {
                continue;
            };
            if by < -(WRONG_SIDE_RELATIVE * scale) {
                return Some(StationFault::WrongSideOf { side_b, by });
            }
        }

        let (a, b) = (station.contact_a, station.contact_b);
        for (contact, side_b) in [(a, false), (b, true)] {
            if contact.feature.is_cut() {
                return Some(StationFault::OnTheCut { side_b });
            }
        }
        let meet = meeting_tolerance(radius);
        let middle = (a.point + b.point) * 0.5;
        if (a.point - b.point).length() <= meet && (middle - station.chain_point).length() <= meet {
            return Some(StationFault::ContactsMeet { point: middle });
        }
        if a.feature.is_rim()
            && b.feature.is_rim()
            && (a.point - b.point).length() <= SAME_POINT_RELATIVE * scale
        {
            return Some(StationFault::SharedRimPoint { point: a.point });
        }

        None
    }

    /// [`RollingBall::grown_station`] within the end faces.
    pub(super) fn grown_station(&self, point: &ChainPoint) -> Result<BallStation, StationFailure> {
        let first_radius = self.ball.radius * FIRST_SHARE;
        let start = flat_face_centre(
            point.point,
            point.normal_a,
            point.normal_b,
            first_radius,
            self.ball.convex,
        )
        .ok_or(StationFailure::NoFirstEstimate)?;
        let mut station = self.checked(point, start, first_radius, None)?;
        let mut size = first_radius;
        let mut passes = 0;
        while size < self.ball.radius {
            let mut next = vector_math::ieee_math::min_f64(size * 2.0, self.ball.radius);
            let mut halvings = 0;
            loop {
                let start = point.point + ((station.centre - point.point) * (next / size));
                match self.checked(point, start, next, None) {
                    Ok(grown) => {
                        station = grown;
                        size = next;
                        break;
                    }
                    Err(failure) if halvings >= MOST_GROWTH_HALVINGS => return Err(failure),
                    Err(failure) => {
                        halvings += 1;
                        next = (size + next) * 0.5;
                        // Halved down onto the size already solved: no size between them is left
                        // to try, so this one cannot grow.
                        if next <= size {
                            return Err(failure);
                        }
                    }
                }
            }
            passes += 1;
            if size < self.ball.radius && passes >= MOST_GROWTH_PASSES {
                return Err(StationFailure::DidNotGrow { size });
            }
        }

        Ok(station)
    }

    /// The ball at `to`, carried from `from`'s solved `station`: started from its centre moved
    /// along with the chain point, the step halved where it fails or jumps, and the station grown
    /// afresh where halving runs out.
    pub(super) fn carried(
        &self,
        from: &ChainPoint,
        station: BallStation,
        to: &ChainPoint,
        depth: usize,
    ) -> Result<BallStation, StationFailure> {
        let moved = to.point - from.point;
        let start = station.centre + moved;
        let radius = self.ball.radius;
        let failure = match self.checked(to, start, radius, Some(&station)) {
            Ok(next) if !jumped(&station, &next, moved.length(), radius) => return Ok(next),
            Ok(next) => StationFailure::Jumped {
                from: station.centre,
                to: next.centre,
            },
            Err(failure) => failure,
        };

        if depth < MOST_STEP_HALVINGS {
            let middle = from.toward(to, 0.5);
            let halfway = self.carried(from, station, &middle, depth + 1)?;
            return self.carried(&middle, halfway, to, depth + 1);
        }

        // Halving ran out: a ball grown afresh here is used only if rolling reaches it.
        match self.grown_station(to) {
            Ok(grown) if !jumped(&station, &grown, moved.length(), radius) => Ok(grown),
            _ => Err(failure),
        }
    }

    /// The halves of space each side is read within at `point` for a ball of `radius`: its half of
    /// the corner, read a hair past its plane. A side touches that plane on its own surface - the
    /// chain lies in it - and a contact there is surface, not cut: where the round runs out the
    /// contacts converge onto the chain, and a plane through it that a blended direction tilts by
    /// a hair would otherwise hide them. A ball rolled on from a neighbour's (`track`) has no
    /// corner half: its contacts are followed over each side's surface (`contact_tracking.rs`).
    /// The end faces bound no reading: the walk's sides are already cut at them and extended past
    /// them (`side_extension.rs`).
    fn reading_halves(
        &self,
        point: &ChainPoint,
        radius: f64,
        track: Option<&BallStation>,
    ) -> (Vec<HalfSpace>, Vec<HalfSpace>) {
        let (half_a, half_b) = match track {
            None => self.ball.halves(point),
            Some(_) => (None, None),
        };
        let past = READING_PAST_RELATIVE * (1.0 + radius.abs());
        let within = |half: Option<HalfSpace>| -> Vec<HalfSpace> {
            half.into_iter()
                .map(|half| HalfSpace {
                    origin: half.origin - (half.normal * past),
                    normal: half.normal,
                })
                .collect()
        };
        (within(half_a), within(half_b))
    }

    /// How a station at `point` reads each side within `halves_a` and `halves_b`: from nothing
    /// (the nearest point within them) for a seed, followed from `track`'s contacts for a ball
    /// rolled on from it.
    fn readings<'h>(
        halves_a: &'h [HalfSpace],
        halves_b: &'h [HalfSpace],
        track: Option<&BallStation>,
    ) -> (SideReading<'h>, SideReading<'h>) {
        match track {
            None => (
                SideReading::Nearest(halves_a),
                SideReading::Nearest(halves_b),
            ),
            Some(from) => (
                SideReading::TrackedFrom {
                    contact: from.contact_a,
                    halves: halves_a,
                },
                SideReading::TrackedFrom {
                    contact: from.contact_b,
                    halves: halves_b,
                },
            ),
        }
    }

    /// Where a ball centred at `centre` touches each side, read as a station at `point` reads
    /// them (followed from `track`'s contacts where given); `None` for a side with no triangles
    /// there.
    pub(super) fn contacts(
        &self,
        point: &ChainPoint,
        centre: Vector3,
        radius: f64,
        track: Option<&BallStation>,
    ) -> Option<(SideContact, SideContact)> {
        let (halves_a, halves_b) = self.reading_halves(point, radius, track);
        let (reading_a, reading_b) = Self::readings(&halves_a, &halves_b, track);
        let read = |reading: SideReading<'_>, side: &RollingBallSide| match reading {
            SideReading::Nearest(halves) => side.closest_within(centre, halves),
            SideReading::TrackedFrom { contact, halves } => {
                side.closest_tracked(centre, &contact, halves)
            }
        };
        Some((
            read(reading_a, self.ball.side_a)?,
            read(reading_b, self.ball.side_b)?,
        ))
    }

    /// One station solved from `start` at `radius` and checked: in the plane square to the chain,
    /// or, where that centre would cross an end face's plane, in the end plane (the one it would
    /// cross furthest, the first given of two equally far). A free solve that fails is tried in
    /// an end plane `start` already stands in or beyond, which is where a walk held at an end
    /// carries its centre. With `track`, the ball is rolled on from that neighbouring ball and
    /// each side's contact followed from its contact there; without, it is seeded from nothing.
    pub(super) fn checked(
        &self,
        point: &ChainPoint,
        start: Vector3,
        radius: f64,
        track: Option<&BallStation>,
    ) -> Result<BallStation, StationFailure> {
        let (halves_a, halves_b) = self.reading_halves(point, radius, track);
        let (reading_a, reading_b) = Self::readings(&halves_a, &halves_b, track);
        let solve = |origin: Vector3, normal: Vector3, from: Vector3| {
            ball_station_reading(
                (self.ball.side_a, reading_a),
                (self.ball.side_b, reading_b),
                origin,
                normal,
                from,
                radius,
            )
        };
        let free = solve(point.point, point.tangent, start);
        let tolerance = END_PLANE_RELATIVE * (1.0 + radius.abs());
        let deepest_beyond = |p: Vector3, beyond: f64| {
            let mut found: Option<(f64, HalfSpace)> = None;
            for end in self.ends {
                let depth = end.depth(p);
                if depth < beyond && found.is_none_or(|(d, _)| depth < d) {
                    found = Some((depth, *end));
                }
            }
            found
        };
        let held = match &free {
            Ok(station) => deepest_beyond(station.centre, -tolerance).map(|f| (f, station.centre)),
            Err(_) => deepest_beyond(start, tolerance).map(|f| (f, start)),
        };
        let station = match (held, free) {
            (None, free) => free.map_err(StationFailure::Unsolved)?,
            (Some(((depth, end), from)), _) => {
                let on_plane = from - (end.normal * depth);
                let mut station =
                    solve(end.origin, end.normal, on_plane).map_err(StationFailure::Unsolved)?;
                station.chain_point = point.point;
                station.tangent = point.tangent.get_normal();
                station
            }
        };
        match self.fault(&station, radius) {
            Some(fault) => Err(StationFailure::Fault {
                centre: station.centre,
                fault,
            }),
            None => Ok(station),
        }
    }
}

impl RollingBall<'_> {
    /// Each side's half of space at `point`: split by the plane through the chain containing its
    /// tangent and the bisector of the two faces beside the edge, side A's half the one its face
    /// beside the edge runs into. `None` for both where the faces give no plane.
    #[allow(clippy::neg_cmp_op_on_partial_ord)] // `!(x > 0)` on purpose: a NaN gives no halves
    pub fn halves(&self, point: &ChainPoint) -> (Option<HalfSpace>, Option<HalfSpace>) {
        // A concave corner's face A runs off toward B's normal; a convex one's away from it.
        let across = if self.convex {
            point.normal_a - point.normal_b
        } else {
            point.normal_b - point.normal_a
        };
        let length = point.tangent.length();
        if !(length > 0.0) || !length.is_finite() {
            return (None, None);
        }
        let t = point.tangent / length;
        let square = across - (t * across.dot(t));
        if !(square.length() > 0.0) {
            return (None, None);
        }

        let normal = square.get_normal();
        (
            Some(HalfSpace {
                origin: point.point,
                normal,
            }),
            Some(HalfSpace {
                origin: point.point,
                normal: -normal,
            }),
        )
    }
}

/// Whether the centre moved from `before` to `after` further than rolling a chain point `moved`
/// along can move it.
pub(super) fn jumped(before: &BallStation, after: &BallStation, moved: f64, radius: f64) -> bool {
    let reach = (moved * 2.0) + (radius * JUMP_RADIUS_SHARE);
    (after.centre - before.centre).length() > reach
}
