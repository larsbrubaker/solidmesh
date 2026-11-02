//! A side's contact followed from where it was ([`RollingBallSide::closest_tracked`]): the walk's
//! way of reading a side once the first ball stands, so a contact moves only continuously over the
//! side's own surface. The overview is the module header (`rolling_ball/mod.rs`); the side and its
//! whole-side search are `side_surface.rs`, and the walk that uses this is `walk.rs`.
//!
//! Why follow the contact rather than search the side. A side region is everything across the
//! edge, and it can hold surface the ball must never touch from where it rolls: the T Handle's
//! larger region wraps round the pocket wall's far rim, and read whole it answers with that far
//! surface or the rim both regions share. A plane through the chain (the corner's bisector) keeps
//! each side to its own half, which is right where the ball is born, beside the edge, but wrong
//! for a wide round on curved sides: near the T Handle's web the R12 ball touches the pocket wall
//! about 18 down its curve, across the bisector plane, so the half hides the very surface the ball
//! rests on. The ball rolls: its contact on each side slides over that side's surface and never
//! leaps to another part of it. So once a station has a contact, the next station's contact is the
//! nearest point found by walking downhill over the side's triangles from it.
//!
//! The descent. From the old contact's triangle, read every side triangle sharing a vertex with
//! it (its star) and move to the nearest of them while that is strictly nearer the new point,
//! ties to the lower triangle index; where nothing in the star is nearer, the contact is the
//! nearest point of the star. That is a true local minimum of the distance over the side's
//! surface: the star covers every way off the triangle (through its edges and its corners), so no
//! nearby surface is closer. The distance falls (or, at equal distance, the index does) at every
//! move, so the descent ends.
//!
//! What it decides:
//! - **At a rim the contact stops.** The star holds only the side's own triangles, so where the
//!   side ends the descent ends too, and the contact rests on the rim edge or vertex, as the ball
//!   resting on a face that ran out does. Surface past the rim belongs to another region.
//! - **A jump never wins.** Where the local answer and the nearest point of the whole side (within
//!   any half) disagree, the local one is the contact: the other is a different part of the side
//!   the ball cannot have rolled to without lifting off the surface it was on, which is exactly
//!   the wrap or far rim the halves were there to hide. The bisector halves are kept only for
//!   seeding a station from nothing.
//! - **Planes still bound.** The halves given (the end faces' planes) cut each triangle as the
//!   whole-side search does, and a contact on a cut is still [`ContactFeature::Cut`].
//! - **Where the contact's face ends, the ball rolls on from its other touches.** The three-face
//!   ball where a chain walked whole turns at a cube's corner touches its one side (right, bottom
//!   and left faces of the front rim) on the bottom and on the left at once, and records one
//!   contact. Rolled on up the left edge from the bottom contact, the descent's star reached the
//!   right face (1.1 off) before the left's triangle holding the contact (0.9 off), and settled on
//!   that far wall. The bottom face does not carry the ball on: the descent from it left it across
//!   the side's crease. Only there ([`TURN_CREASE_DEGREES`]: the descent's answer stands on a
//!   triangle turned from the old contact's by more than that) the descent also starts from every
//!   other place ahead that the neighbouring ball touched ([`RollingBallSide::touched_elsewhere`])
//!   and keeps the nearest answer; each start is surface the ball was touching, so this is no
//!   jump. Where the old contact's surface carries on smoothly the contact is followed from it
//!   alone, even where the ball touches its side somewhere else too: the T Handle's R12 ball is
//!   wedged against two places of its sphere side 8 to 17 apart, equally near within 1e-11, and
//!   taking the nearer of the two at every station flipped the contact between them each station
//!   (its walk three times as slow, its contacts no longer the ones it rolled on).

use vector_math::Vector3;

use super::ball_station::SOLVED_RELATIVE;
use super::side_surface::{
    closest_on_any_triangle, closest_on_cut_triangle, ContactFeature, HalfSpace, RollingBallSide,
    SideContact,
};
use super::walk::SAME_POINT_RELATIVE;

/// The best reading so far: (squared distance, triangle, point, on a cut).
type Best = (f64, usize, Vector3, bool);

/// How far, in degrees, the triangle a followed contact reaches may be turned from the one it
/// left before the contact's face counts as ended there (see the file header): the bend at which
/// the app splits a part's faces for the user (the bevel's default bend threshold), so a crease
/// sharper than that is where one face of the side ends and another begins, while a curved
/// surface's facets, turning a few degrees each, carry the contact on.
pub const TURN_CREASE_DEGREES: f64 = 20.0;

/// The neighbouring ball a contact is followed from: its centre, and the step its chain point
/// takes to the station being read (`ahead`), which [`RollingBallSide::touched_elsewhere`] reads.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct TouchedFrom {
    /// The neighbouring ball's centre.
    pub centre: Vector3,
    /// The station's chain point less the neighbour's.
    pub ahead: Vector3,
}

impl RollingBallSide {
    /// The side's nearest point to `point` reached by walking downhill over its surface from
    /// `from`, a contact this side gave earlier, each triangle read within every one of `halves`
    /// (see the file header). `None` where `from` names no triangle of the side or nothing of its
    /// star lies within the halves.
    pub fn closest_tracked(
        &self,
        point: Vector3,
        from: &SideContact,
        halves: &[HalfSpace],
    ) -> Option<SideContact> {
        self.closest_tracked_from_any(point, from, None, halves)
    }

    /// [`closest_tracked`](Self::closest_tracked) from `from`, the contact the ball `touched` gave
    /// this side; where that descent leaves the contact's face across a crease
    /// ([`TURN_CREASE_DEGREES`]), also from each place ahead that ball touched
    /// ([`touched_elsewhere`](Self::touched_elsewhere)), the nearest answer kept (ties to the
    /// lower triangle index). Every start is surface the ball was touching, so no reading leaps
    /// (see the file header).
    pub fn closest_tracked_from_any(
        &self,
        point: Vector3,
        from: &SideContact,
        touched: Option<TouchedFrom>,
        halves: &[HalfSpace],
    ) -> Option<SideContact> {
        let start = usize::try_from(from.triangle)
            .ok()
            .filter(|&t| t < self.triangles.len())?;
        let mut best = self.descend(point, start, halves);
        let face_ended = best.is_some_and(|(_, t, _, _)| self.turned_past_a_crease(start, t));
        let elsewhere = match touched {
            Some(ball) if face_ended => self.touched_elsewhere(ball.centre, from, ball.ahead),
            _ => Vec::new(),
        };
        for &other in &elsewhere {
            if let Some(found) = self.descend(point, other, halves) {
                if best.is_none_or(|(d, t, _, _)| found.0 < d || (found.0 == d && found.1 < t)) {
                    best = Some(found);
                }
            }
        }
        let (d, t, on, cut) = best?;
        // A side's list position; a side has far fewer than `i32::MAX` triangles.
        let triangle = t as i32;
        Some(SideContact {
            point: on,
            distance: d.sqrt(),
            feature: if cut {
                ContactFeature::Cut { triangle }
            } else {
                self.feature_of(t, on)
            },
            triangle,
        })
    }

    /// The side's triangles a ball centred at `centre` touches at another place than `contact`,
    /// its contact on this side, ahead of it as the ball rolls on along `ahead`: each triangle
    /// whose nearest point stands no further from the centre than the contact does, give or take
    /// the solve's own tolerance, is not the contact's own point (the contact's triangle and the
    /// ones sharing its edge or vertex are its star, which the descent reads anyway), and lies
    /// beyond the contact along `ahead`. The solve holds a station's distances to the radius only
    /// to `SOLVED_RELATIVE` of one plus the radius, so surface within that of the contact's
    /// distance is touched as far as the station can tell: the three-face ball where a chain
    /// walked whole turns at a cube's corner touches its one side on both faces there. Only a
    /// touch ahead can carry on: rolling on, the ball leaves behind the face of the edge it came
    /// along and rolls onto the face of the edge it goes along. Started from a touch behind (an
    /// L-shaped boss's rim at its outer corner, rolling on toward the inner corner), the descent
    /// ran along the walls the ball had left to the far wall of the L. In index order, from the
    /// side's box tree.
    pub fn touched_elsewhere(
        &self,
        centre: Vector3,
        contact: &SideContact,
        ahead: Vector3,
    ) -> Vec<usize> {
        let scale = 1.0 + contact.distance.abs();
        let reach = contact.distance + (SOLVED_RELATIVE * scale);
        let same_place = SAME_POINT_RELATIVE * scale;
        let mut touched: Vec<usize> = self
            .within(centre, reach)
            .into_iter()
            .filter(|&t| {
                let [a, b, c] = self.corners(t);
                let on = closest_on_any_triangle(centre, a, b, c);
                (on - centre).length() <= reach
                    && (on - contact.point).length() > same_place
                    && (on - contact.point).dot(ahead) > 0.0
            })
            .collect();
        touched.sort_unstable();
        touched
    }

    /// Whether triangle `to` is turned from triangle `from` by more than [`TURN_CREASE_DEGREES`]:
    /// their normals' angle, read as the cosine against both lengths so no normal is divided by
    /// (a triangle with no area turns nowhere).
    fn turned_past_a_crease(&self, from: usize, to: usize) -> bool {
        let normal = |t: usize| {
            let [a, b, c] = self.corners(t);
            (b - a).cross(c - a)
        };
        let (n_from, n_to) = (normal(from), normal(to));
        let least = TURN_CREASE_DEGREES.to_radians().cos() * (n_from.length() * n_to.length());
        n_from.dot(n_to) < least
    }

    /// Walks downhill over the side from triangle `start` toward `point` (see the file header):
    /// the reading it settles on, or `None` where nothing of its star lies within the halves.
    fn descend(&self, point: Vector3, start: usize, halves: &[HalfSpace]) -> Option<Best> {
        let count = self.triangles.len();
        let read = |t: usize| -> Option<Best> {
            closest_on_cut_triangle(point, self.corners(t), halves)
                .map(|(on, cut)| ((on - point).length_squared(), t, on, cut))
        };
        let better = |candidate: &Best, best: &Option<Best>| match best {
            None => true,
            Some((d, t, _, _)) => candidate.0 < *d || (candidate.0 == *d && candidate.1 < *t),
        };

        let mut best = read(start);
        let mut here = start;
        // Every move lowers (distance, triangle) strictly, so at most one move per triangle.
        for _ in 0..=count {
            let mut next = best;
            for t in self.star(here) {
                if let Some(reading) = read(t) {
                    if better(&reading, &next) {
                        next = Some(reading);
                    }
                }
            }
            match next {
                Some(found) if found.1 != here && better(&found, &best) => {
                    best = Some(found);
                    here = found.1;
                }
                _ => break,
            }
        }
        best
    }

    /// The side's triangles sharing a vertex with `triangle`, itself included, in index order.
    fn star(&self, triangle: usize) -> Vec<usize> {
        let mut around: Vec<usize> = self.triangles[triangle]
            .iter()
            .filter_map(|v| self.vertex_triangles.get(v))
            .flatten()
            .copied()
            .collect();
        around.sort_unstable();
        around.dedup();
        around
    }
}
