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

use vector_math::Vector3;

use super::side_surface::{
    closest_on_cut_triangle, ContactFeature, HalfSpace, RollingBallSide, SideContact,
};

/// The best reading so far: (squared distance, triangle, point, on a cut).
type Best = (f64, usize, Vector3, bool);

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
        let count = self.triangles.len();
        let start = usize::try_from(from.triangle).ok().filter(|&t| t < count)?;
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
