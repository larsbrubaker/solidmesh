//! The rolling-ball fillet's geometry (`docs/rolling-ball-fillet.md`): a round decided by where a
//! ball of the fillet's radius actually touches each side of the picked edge as it rolls along
//! it. A side is everything across the edge on that side - the face beside it, the rim past it,
//! the face beyond - not only the face next to the edge. That one rule is why the rolling ball
//! needs no special cases: where the face beside the edge narrows to nothing, the ball's contact
//! simply moves onto the rim; where both faces curve, it touches where it touches.
//!
//! Built beside the level line's band, which reads a setback per chain point from the angle at
//! the edge (`level_line::ChainField`). Behind its switch ([`rolling_ball_routing`], off unless
//! `MATTERCAD_BEVEL_ROLLINGBALL` is on), a level-line pass takes its band's setbacks from the
//! ball's contacts instead, where every chain of the pass walks
//! (`level_line::rolling_ball_setbacks`); no build the app makes by default uses it. What is here:
//!
//! - [`RollingBallSide`] (`side_surface.rs`): one side's surface - triangles of the part - with a
//!   box tree, answering the side's closest point to a point in space and what that point lies on
//!   ([`ContactFeature`]: inside a face, on an edge, at a vertex, and whether that edge or vertex
//!   is the side's own boundary, a rim).
//! - [`ball_station`] (`ball_station.rs`): the ball's centre at one point of the chain - in the
//!   plane square to the chain there, at the fillet's radius from both sides - and where it
//!   touches each, found by Newton's method from the two flat faces' estimate
//!   ([`flat_face_centre`]).
//! - [`RollingBall::walk`] (`walk.rs`): the ball along a whole chain, each station solved from its
//!   neighbour's centre, starting from a small ball grown to the size asked where the flat
//!   estimate is good, that seed reading each side only within its half of the corner
//!   ([`HalfSpace`]) and every later ball following each side's contact over the side's surface
//!   from its neighbour's ([`RollingBallSide::closest_tracked`], `contact_tracking.rs`), and
//!   every station checked to be a usable ball ([`StationFault`]: not reaching past the part, on
//!   the right side of each contact, not resting on a rim both sides share). A chain ending on a
//!   face that is neither side ([`EndFace`]) keeps the ball's centre on its own side of that
//!   face's plane, held in the plane where it would cross, reads each side as extended past the
//!   face (`side_extension.rs`), and the end face never refuses it.
//!   A free end where the contacts meet on the chain closes with a zero-width end station
//!   (`run_out.rs`); [`ChainWalk`] says how each end finished ([`WalkEnd`]).
//!
//! A CONCAVE edge's ball sits in free space and fills the corner it cannot reach; a CONVEX edge's
//! sits inside the solid and the round cuts away what it cannot reach. Either way the centre is at
//! the radius from both sides, so one solve serves both; the starting estimate picks the side.
//!
//! Numbers follow the crate's rules: `f64`, a fixed order of operations, no fused multiply-add,
//! and every tie broken by triangle index so a search reads the same on every run.

mod ball_station;
mod contact_tracking;
pub mod rolling_ball_routing;
mod run_out;
mod side_extension;
mod side_surface;
mod walk;

pub use ball_station::{
    ball_station, ball_station_reading, ball_station_within, flat_face_centre, BallStation,
    BallStationError, SideReading, MAX_NEWTON_STEPS,
};
pub use side_surface::{ContactFeature, HalfSpace, RollingBallSide, SideContact};
pub use walk::{
    ChainPoint, ChainWalk, EndFace, RollingBall, StationFailure, StationFault, WalkEnd, WalkError,
};
