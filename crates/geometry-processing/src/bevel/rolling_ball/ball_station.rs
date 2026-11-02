//! The rolling ball at one point of the chain: its centre, in the plane square to the chain at that
//! point, at the fillet's radius from both sides, and where it touches each
//! ([`ball_station`]), from the estimate two flat faces give ([`flat_face_centre`]). The overview
//! is the module header (`rolling_ball/mod.rs`).
//!
//! The centre solves `distance(X, side A) = r` and `distance(X, side B) = r` for X in the plane, by
//! Newton's method in the plane's two coordinates. Each distance's gradient is the unit vector
//! from the side's closest point to X, so the step needs nothing but the two contacts; a step that
//! does not shrink the larger of the two errors is halved until it does. A side's distance stays
//! smooth as its contact slides from a face onto a rim, so the same solve carries the ball over
//! the edge of a face that runs out.

use vector_math::Vector3;

use super::contact_tracking::TouchedFrom;
use super::side_surface::{HalfSpace, RollingBallSide, SideContact};

/// The most Newton steps a station takes.
pub const MAX_NEWTON_STEPS: usize = 60;

/// The most times one step is halved before the solve gives up.
const MAX_HALVINGS: usize = 40;

/// How far both distances may stand from the radius, relative to one plus the radius, for the
/// centre to count as found: a few hundred float steps at a part's sizes.
pub(super) const SOLVED_RELATIVE: f64 = 1e-11;

/// The least the two distance gradients' determinant may be in the plane before the sides count as
/// parallel there (no single centre).
const PARALLEL_DETERMINANT: f64 = 1e-12;

/// The least `1 + na . nb` two faces' unit normals may give before [`flat_face_centre`] counts the
/// faces as folded flat on each other. Near the fold the solve's determinant for flat faces is the
/// sine of the angle between their normals, and `sin^2 = (1 - cos)(1 + cos)`, which is
/// `2 (1 + cos)` to first order there; so this is the fold at which the solve itself would call the
/// sides parallel ([`PARALLEL_DETERMINANT`]), and no estimate is refused that the solve could use.
const FOLDED_FLAT: f64 = PARALLEL_DETERMINANT * PARALLEL_DETERMINANT * 0.5;

/// The ball at one chain point.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct BallStation {
    /// The chain point the station's plane passes through.
    pub chain_point: Vector3,
    /// The chain's unit direction there, the plane's normal.
    pub tangent: Vector3,
    /// The ball's centre.
    pub centre: Vector3,
    /// Where the ball touches side A.
    pub contact_a: SideContact,
    /// Where it touches side B.
    pub contact_b: SideContact,
    /// How many Newton steps the solve took.
    pub steps: usize,
}

/// Why a station has no ball.
#[derive(Debug, Clone, PartialEq)]
pub enum BallStationError {
    /// A side holds no triangles.
    EmptySide,
    /// The tangent is zero or not finite.
    NotATangent,
    /// The two sides' distances change the same way across the plane, so no single centre stands
    /// at the radius from both (the sides are parallel there, or the solve stood on a side).
    SidesParallel { centre: Vector3 },
    /// Newton's method stopped short of the radius.
    DidNotConverge { centre: Vector3, error: f64 },
    /// A side's contact, followed from a neighbouring ball's ([`SideReading::TrackedFrom`]), has
    /// nothing of its surface around it within the halves (or names no triangle of the side): the
    /// contact cannot move on from there without leaping to another part of the side, which a
    /// rolling ball never does. `side_b` names the side.
    ContactNotFollowed { side_b: bool },
}

/// The centre two FLAT faces would give: on the bisector of their normals, at the radius from
/// both faces' planes, `C ± r (na + nb) / (1 + na . nb)` - outward (into free space, where a
/// concave edge's ball rolls) unless `convex` (inside the solid). `normal_a` and `normal_b` are the
/// faces' unit outward normals at the chain point. `None` where the faces fold back flat on each
/// other, which no ball fits between.
#[allow(clippy::neg_cmp_op_on_partial_ord)] // `!(x > y)` on purpose: NaN folds flat
pub fn flat_face_centre(
    chain_point: Vector3,
    normal_a: Vector3,
    normal_b: Vector3,
    radius: f64,
    convex: bool,
) -> Option<Vector3> {
    let denominator = 1.0 + normal_a.dot(normal_b);
    if !(denominator > FOLDED_FLAT) {
        return None;
    }

    let sign = if convex { -1.0 } else { 1.0 };
    Some(chain_point + ((normal_a + normal_b) * ((sign * radius) / denominator)))
}

/// One point of the plane, read: the point, both contacts and both errors.
struct Reading {
    centre: Vector3,
    a: SideContact,
    b: SideContact,
    error_a: f64,
    error_b: f64,
}

impl Reading {
    /// The larger of the two errors, which every step must shrink.
    fn error(&self) -> f64 {
        vector_math::ieee_math::max_f64(self.error_a.abs(), self.error_b.abs())
    }
}

/// The ball of `radius` at `chain_point`: its centre in the plane through `chain_point` square to
/// `tangent`, at the radius from both sides, found from `start` (usually [`flat_face_centre`]).
/// The start also picks which of the centres at the radius from both sides is meant - free space
/// for a concave edge, the solid for a convex one - since the solve keeps to the one it starts
/// beside.
pub fn ball_station(
    side_a: &RollingBallSide,
    side_b: &RollingBallSide,
    chain_point: Vector3,
    tangent: Vector3,
    start: Vector3,
    radius: f64,
) -> Result<BallStation, BallStationError> {
    ball_station_within(
        (side_a, &[]),
        (side_b, &[]),
        chain_point,
        tangent,
        start,
        radius,
    )
}

/// [`ball_station`] with each side read only within every one of its halves of space
/// ([`RollingBallSide::closest_within`]; all of it for none): the walk keeps each side to its own
/// side of the corner, so a side that wraps round the other (a narrow wall with the surface beyond
/// it on both sides) cannot answer with a point across the corner, and to the chain's side of each
/// end face, so a side that wraps past an end face cannot answer from beyond it.
pub fn ball_station_within(
    (side_a, halves_a): (&RollingBallSide, &[HalfSpace]),
    (side_b, halves_b): (&RollingBallSide, &[HalfSpace]),
    chain_point: Vector3,
    tangent: Vector3,
    start: Vector3,
    radius: f64,
) -> Result<BallStation, BallStationError> {
    ball_station_reading(
        (side_a, SideReading::Nearest(halves_a)),
        (side_b, SideReading::Nearest(halves_b)),
        chain_point,
        tangent,
        start,
        radius,
    )
}

/// How a station reads one side.
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum SideReading<'h> {
    /// The side's nearest point within every one of the halves
    /// ([`RollingBallSide::closest_within`]): how a ball is seeded from nothing.
    Nearest(&'h [HalfSpace]),
    /// The side's nearest point reached downhill over its surface from `contact`, a contact the
    /// side gave a neighbouring ball (`touched`, where given: where the contact's face ends, also
    /// from the other places that ball touched), within every one of the halves
    /// ([`RollingBallSide::closest_tracked_from_any`]): how a rolling ball's contact moves on.
    TrackedFrom {
        contact: SideContact,
        touched: Option<TouchedFrom>,
        halves: &'h [HalfSpace],
    },
}

impl<'h> SideReading<'h> {
    /// How a station reads both sides within `halves` (side A's, side B's): from nothing (the
    /// nearest point within them) for a seed, followed from `track`'s contacts for a ball rolled
    /// on from it to the chain point `to` (where a contact's face ends there, also from the other
    /// places `track` touched, [`RollingBallSide::touched_elsewhere`]).
    pub(super) fn both(
        halves: [&'h [HalfSpace]; 2],
        track: Option<&BallStation>,
        to: Vector3,
    ) -> (SideReading<'h>, SideReading<'h>) {
        let touched = track.map(|from| TouchedFrom {
            centre: from.centre,
            ahead: to - from.chain_point,
        });
        match track {
            None => (
                SideReading::Nearest(halves[0]),
                SideReading::Nearest(halves[1]),
            ),
            Some(from) => (
                SideReading::TrackedFrom {
                    contact: from.contact_a,
                    touched,
                    halves: halves[0],
                },
                SideReading::TrackedFrom {
                    contact: from.contact_b,
                    touched,
                    halves: halves[1],
                },
            ),
        }
    }
}

impl SideReading<'_> {
    /// The side's contact for a ball centred at `centre`; a tracked reading follows on from
    /// `latest`, the contact the solve last accepted, so the contact slides with the centre over
    /// every Newton step rather than starting each from the neighbouring ball's.
    ///
    /// A followed contact that finds nothing is [`BallStationError::ContactNotFollowed`], not an
    /// empty side: the side may well hold surface within the halves, but only somewhere the
    /// contact would have to leap to, and reading the whole side there would let it. The other
    /// places `touched` gave are starts only for the first reading: after it the contact slides on
    /// from `latest`.
    pub(super) fn read(
        &self,
        side: &RollingBallSide,
        centre: Vector3,
        latest: Option<&SideContact>,
        side_b: bool,
    ) -> Result<SideContact, BallStationError> {
        match self {
            SideReading::Nearest(halves) => side
                .closest_within(centre, halves)
                .ok_or(BallStationError::EmptySide),
            SideReading::TrackedFrom {
                contact,
                touched,
                halves,
            } => match latest {
                Some(latest) => side.closest_tracked(centre, latest, halves),
                None => side.closest_tracked_from_any(centre, contact, *touched, halves),
            }
            .ok_or(BallStationError::ContactNotFollowed { side_b }),
        }
    }
}

/// [`ball_station`] with each side read as `reading_a` and `reading_b` say: within halves of
/// space from nothing, or followed over the side's surface from a neighbouring ball's contact
/// (`contact_tracking.rs`).
#[allow(clippy::neg_cmp_op_on_partial_ord)] // `!(x > y)` on purpose: a NaN refuses
pub fn ball_station_reading(
    (side_a, reading_a): (&RollingBallSide, SideReading<'_>),
    (side_b, reading_b): (&RollingBallSide, SideReading<'_>),
    chain_point: Vector3,
    tangent: Vector3,
    start: Vector3,
    radius: f64,
) -> Result<BallStation, BallStationError> {
    let length = tangent.length();
    if !(length > 0.0) || !length.is_finite() {
        return Err(BallStationError::NotATangent);
    }

    let t = tangent / length;
    let offset = start - chain_point;
    let in_plane = offset - (t * offset.dot(t));
    let e1 = if in_plane.length() > 0.0 {
        in_plane.get_normal()
    } else {
        // A start on the chain point: any direction square to the chain.
        let across = if t.x.abs() < 0.9 {
            Vector3::UNIT_X
        } else {
            Vector3::UNIT_Y
        };
        (across - (t * across.dot(t))).get_normal()
    };
    let e2 = t.cross(e1);

    let read = |u: f64, v: f64, latest: Option<&Reading>| -> Result<Reading, BallStationError> {
        let centre = chain_point + (e1 * u) + (e2 * v);
        let a = reading_a.read(side_a, centre, latest.map(|r| &r.a), false)?;
        let b = reading_b.read(side_b, centre, latest.map(|r| &r.b), true)?;
        Ok(Reading {
            centre,
            error_a: a.distance - radius,
            error_b: b.distance - radius,
            a,
            b,
        })
    };

    let solved = SOLVED_RELATIVE * (1.0 + radius.abs());
    let (mut u, mut v) = (in_plane.dot(e1), in_plane.dot(e2));
    let mut here = read(u, v, None)?;
    for steps in 0..=MAX_NEWTON_STEPS {
        if here.error() <= solved {
            return Ok(BallStation {
                chain_point,
                tangent: t,
                centre: here.centre,
                contact_a: here.a,
                contact_b: here.b,
                steps,
            });
        }

        if !(here.a.distance > 0.0) || !(here.b.distance > 0.0) {
            return Err(BallStationError::SidesParallel {
                centre: here.centre,
            });
        }

        // Each distance grows along the unit vector from its contact to the centre.
        let along_a = (here.centre - here.a.point) / here.a.distance;
        let along_b = (here.centre - here.b.point) / here.b.distance;
        let (j00, j01) = (along_a.dot(e1), along_a.dot(e2));
        let (j10, j11) = (along_b.dot(e1), along_b.dot(e2));
        let determinant = (j00 * j11) - (j01 * j10);
        if !(determinant.abs() > PARALLEL_DETERMINANT) {
            return Err(BallStationError::SidesParallel {
                centre: here.centre,
            });
        }

        let du = -((j11 * here.error_a) - (j01 * here.error_b)) / determinant;
        let dv = -((j00 * here.error_b) - (j10 * here.error_a)) / determinant;
        let mut scale = 1.0;
        let mut next = None;
        for _ in 0..MAX_HALVINGS {
            let trial = read(u + (du * scale), v + (dv * scale), Some(&here))?;
            if trial.error() < here.error() {
                next = Some((u + (du * scale), v + (dv * scale), trial));
                break;
            }

            scale *= 0.5;
        }

        let Some((nu, nv, reading)) = next else {
            return Err(BallStationError::DidNotConverge {
                centre: here.centre,
                error: here.error(),
            });
        };
        u = nu;
        v = nv;
        here = reading;
    }

    Err(BallStationError::DidNotConverge {
        centre: here.centre,
        error: here.error(),
    })
}
