//! The oblique-end reading behind `oblique_ends`: whether an end is oblique, whether both its
//! corners lie past the end, and whether its creases fold opposite at the end, split out of
//! `end_face_sense.rs` (which holds the entry point, `oblique_ends`) for length.

use vector_math::Vector3;

use super::corner_cap_flat_ends::use_flat_ends;
use super::curved_end_face::bends_on_softly;
use super::end_face_sense::{
    face_normal, is_convex, leans_off_square, shared_corners, Corners, COPLANAR_TOLERANCE,
    SEPARATE_END_FACE_DEGREES,
};

/// The mesh and bend threshold `ObliqueEnds`' local functions close over.
pub(super) struct ObliqueEndReader<'a> {
    pub positions: &'a [Vector3],
    pub indices: &'a [Corners],
    pub sharp_turn_degrees: f64,
}

fn has(t: Corners, v: i32) -> bool {
    t.0 == v || t.1 == v || t.2 == v
}

fn in_a_side_plane(sides: &[Vector3], normal: Vector3) -> bool {
    sides
        .iter()
        .any(|s| s.dot(normal) >= 1.0 - COPLANAR_TOLERANCE)
}

fn add_corners(set: &mut Vec<i32>, t: Corners) {
    for c in [t.0, t.1, t.2] {
        if !set.contains(&c) {
            set.push(c);
        }
    }
}

impl ObliqueEndReader<'_> {
    fn at(&self, i: i32) -> Vector3 {
        self.positions[i as usize]
    }

    /// Whether the chain's end at `end` (with `next` its neighbour along the chain) ends on one
    /// flat, oblique end face.
    pub fn is_oblique_end(&self, end: i32, next: i32) -> bool {
        let outward = (self.at(end) - self.at(next)).get_normal();
        let mut sides = Vec::new();
        let mut side_triangles = Vec::new();
        let mut others = Vec::new();
        // Corner sets, only added to and asked membership of, so a small Vec does.
        let mut side_corners = Vec::new();
        let mut other_corners = Vec::new();
        let mut other_triangles: Vec<(Vector3, Corners)> = Vec::new();
        for &t in self.indices {
            if !has(t, end) {
                continue;
            }

            let normal = face_normal(self.positions, t);
            if has(t, next) {
                sides.push(normal);
                side_triangles.push(t);
                add_corners(&mut side_corners, t);
            } else {
                others.push(normal);
                other_triangles.push((normal, t));
            }
        }

        if sides.len() != 2 {
            return false;
        }

        // A triangle in a side's plane is that side (a fanned cap has many at the end vertex).
        for &(normal, t) in &other_triangles {
            if in_a_side_plane(&sides, normal) {
                add_corners(&mut side_corners, t);
            } else {
                add_corners(&mut other_corners, t);
            }
        }

        // A crease that CARRIES ON past the end at a turn under the operation's bend threshold (a
        // faceted rim picked in part) is a fade, not an end face: its end keeps the plain field.
        // The carrying edge runs from the end vertex to a corner shared by a side and the face past
        // it. With no threshold (+infinity, the kernel's default) every such edge carries on, so no
        // end takes a ray.
        for &w in &side_corners {
            if w == end || w == next || !other_corners.contains(&w) {
                continue;
            }

            let cos = (self.at(w) - self.at(end))
                .get_normal()
                .dot(outward)
                .clamp(-1.0, 1.0);
            if !self.sharp_turn_degrees.is_finite()
                || cos.acos() * 180.0 / std::f64::consts::PI < self.sharp_turn_degrees
            {
                return false;
            }
        }

        let mut end_face: Option<Vector3> = None;
        for &normal in &others {
            if in_a_side_plane(&sides, normal) {
                continue;
            }

            match end_face {
                None => end_face = Some(normal),
                Some(face) if face.dot(normal) < 1.0 - COPLANAR_TOLERANCE => return false,
                Some(_) => {}
            }
        }

        let Some(end_face) = end_face else {
            return false;
        };

        // A face that folds off a side by less than SeparateEndFaceDegrees is that side CARRYING ON
        // (the next facet of a faceted curve), not an end face: the pick covers part of a longer
        // crease, which fades. The least fold is NaN if any is NaN, so a broken normal is never
        // hidden.
        let folds = sides
            .iter()
            .map(|s| s.dot(end_face).clamp(-1.0, 1.0).acos() * 180.0 / std::f64::consts::PI);
        let mut fold = f64::INFINITY;
        for f in folds {
            if f.is_nan() {
                fold = f64::NAN;
                break;
            }

            if f < fold {
                fold = f;
            }
        }

        if fold < SEPARATE_END_FACE_DEGREES {
            return false;
        }

        // A CURVED end face - one that bends on, under the threshold, into the next facet of a
        // round (a D prism's flat edge ending on its round) - is no plane to stand a ray on: the
        // ray bounds the room at the one facet touching the end, where the end region reads the
        // round to 45 degrees (EndRegionShapesTests). It keeps the plain field, as it does with no
        // threshold.
        if self.sharp_turn_degrees.is_finite()
            && bends_on_softly(
                self.positions,
                self.indices,
                end_face,
                other_triangles
                    .iter()
                    .filter(|o| o.0.dot(end_face) >= 1.0 - COPLANAR_TOLERANCE)
                    .map(|o| o.1),
                self.sharp_turn_degrees,
            )
        {
            return false;
        }

        let fanned_sides: Vec<Corners> = other_triangles
            .iter()
            .filter(|o| in_a_side_plane(&sides, o.0))
            .map(|o| o.1)
            .collect();
        let end_triangles: Vec<Corners> = other_triangles
            .iter()
            .filter(|o| !in_a_side_plane(&sides, o.0))
            .map(|o| o.1)
            .collect();

        // Facing on along the chain, and not square to it (relative 1e-6).
        let facing = end_face.dot(outward);
        if use_flat_ends() {
            // Except a wall facing BACK nearly square across a CONVEX chain's end (Every Chamfer's
            // bottom rim ending on the x = -1 wall at (-1, 40, 0)): a ray there reads through the
            // chain meeting that wall on the far side and the pair room falls to 0, exactly as the
            // flag-off rule below says of Top & Round - so it keeps the plain field there too. A
            // wall leaning back further takes the ray only where it folds opposite at every side
            // crease, as flag off.
            return 1.0 - facing.abs() > COPLANAR_TOLERANCE
                && (facing > 0.0
                    || !is_convex(self.positions, side_triangles[0], side_triangles[1], end)
                    || (leans_off_square(facing)
                        && self.folds_opposite_at_the_end(
                            &side_triangles,
                            &fanned_sides,
                            &end_triangles,
                            end,
                        )));
        }

        if facing > 0.0 {
            return 1.0 - facing > COPLANAR_TOLERANCE;
        }

        // Facing BACK over the chain, and not square to it: taken only where the end face folds the
        // OPPOSITE way to the chain at every side-to-end crease through the end (Fillet Thin Front:
        // a convex fin apex into 45 degree walls it meets concavely). The round then runs on toward
        // the end vertex past where the rails meet the crease, and the wall's plane clips it (the
        // wall grows). Or where a CONCAVE chain meets it the SAME way (Center Flows Through): the
        // round runs on past the end vertex to the face's plane, and the ray is bounded there
        // (EndRayBounds) so it never reads through the part behind the face. The opposite-sense
        // reading asks the wall to lean off square by more than BackFacingWallLeanDegrees.
        1.0 + facing > COPLANAR_TOLERANCE
            && ((!is_convex(self.positions, side_triangles[0], side_triangles[1], end)
                && self.both_corners_past_the_end(&side_triangles, end, outward, end_face))
                || (leans_off_square(facing)
                    && self.folds_opposite_at_the_end(
                        &side_triangles,
                        &fanned_sides,
                        &end_triangles,
                        end,
                    )))
    }

    /// Where each side's rail meets the end face's plane, both
    /// on or past the end vertex along the chain: the rail from the end vertex runs square to the
    /// chain into its side, r, and meets the plane at t = -(r . n) / (outward . n) along the chain.
    /// Scale-free, so read at unit setback. An end face leaning back over BOTH rails (Center Flows
    /// Through's webs) is the concave same-sense end the bounded ray and the end-plane trim are
    /// for; one leaning ACROSS the chain (UpperLeftInner's rim end: one corner short of the end
    /// vertex) keeps the plain field it has always built with.
    #[allow(clippy::neg_cmp_op_on_partial_ord)]
    fn both_corners_past_the_end(
        &self,
        side_triangles: &[Corners],
        end: i32,
        outward: Vector3,
        end_normal: Vector3,
    ) -> bool {
        let v = self.at(end);
        let slant = end_normal.dot(outward);
        if slant.abs() <= COPLANAR_TOLERANCE {
            return false;
        }

        for &t in side_triangles.iter().take(2) {
            // The first longest offset, a NaN length sorting last.
            let mut off: Option<Vector3> = None;
            for c in [t.0, t.1, t.2] {
                let w = self.at(c) - v;
                let w = w - (outward * w.dot(outward));
                let longer = match off {
                    None => true,
                    Some(best) => {
                        let (l, b) = (w.length_squared(), best.length_squared());
                        (b.is_nan() && !l.is_nan()) || l > b
                    }
                };
                if longer {
                    off = Some(w);
                }
            }

            let Some(off) = off else {
                return false;
            };
            if !(off.length_squared() > 0.0) {
                return false;
            }

            let along = -off.get_normal().dot(end_normal) / slant;
            if along < -1e-9 {
                return false;
            }
        }

        true
    }

    /// Every side-to-end crease through the end folds the other
    /// way to the chain's own fold; at least one. A side FANNED at the end counts with the two
    /// triangles on the chain edge: its crease with the end face is as much a side-to-end crease as
    /// theirs, and which one a side's triangulation puts it on must not decide the ray ("Fillet
    /// Should go to Sides and Stay Within Bounds.mcx" c17).
    fn folds_opposite_at_the_end(
        &self,
        side_triangles: &[Corners],
        fanned_sides: &[Corners],
        end_triangles: &[Corners],
        end: i32,
    ) -> bool {
        let chain_is_convex = is_convex(self.positions, side_triangles[0], side_triangles[1], end);
        let mut creases = 0;
        for &side in side_triangles.iter().chain(fanned_sides) {
            for &other in end_triangles {
                let shared = shared_corners(side, other);
                if shared.len() != 2 || !shared.contains(&end) {
                    continue;
                }

                creases += 1;
                if is_convex(self.positions, side, other, end) == chain_is_convex {
                    return false;
                }
            }
        }

        creases > 0
    }
}
