//! Sewing the patch a [`SurfaceMitreSliver`] read, and the outline it fans: out along the narrower
//! foot's step to its crossing, over the mitre polyline the wider run carries there, back along the
//! wider foot's step to ITS rim, and home along the crease between the two rims - wound opposite to
//! the wall's own use of the rim where the wall has drawn it, and by the strip's outward reference
//! only where nothing has. The reading itself is `surface_mitre_sliver`.

// The negated comparisons are deliberate: each is false for NaN, which is the point.
#![allow(clippy::neg_cmp_op_on_partial_ord)]

use solidmesh_math::Vector3;

use super::{SurfaceMitreSliver, SurfaceVertexPool};
use crate::bevel::{BevelAdvisory, BevelAdvisoryCode};

impl SurfaceMitreSliver {
    /// The patch itself, fanned from the narrower run's rim: out along its foot's step to its
    /// crossing, over the mitre polyline the wider run carries there, back along the wider foot's
    /// step to ITS rim, and home along the crease between the two rims. Wound to face the way the
    /// narrower run's far face does, which is the way the patch leaves it.
    pub fn try_sew(
        &self,
        pool: &SurfaceVertexPool,
        faces: &mut Vec<(i32, i32, i32)>,
    ) -> Result<(), BevelAdvisory> {
        // The rim run, kept apart: it is the one side of the outline the WALL is already sewn
        // along, so it says both where the fan may not stand and which way round the patch goes.
        let mut along_the_rim = vec![self.wider_rim];

        along_the_rim.extend(standing_along_the_rim(
            pool,
            pool.position_of(self.wider_rim),
            pool.position_of(self.narrower_rim),
        ));
        along_the_rim.push(self.narrower_rim);

        let (outline, area) = self.the_outline(pool, &along_the_rim);

        if !(area.length_squared() > 0.0) && self.took_the_rims {
            // NOTHING TO SEW AT THIS CORNER, and nothing to mint either. Both crossings stood
            // within the ruler of their own rims and were taken as them, and with them taken every
            // point of the outline stands on the crease: the whole of this patch's area was the two
            // steps off the rim, and those steps are a ruler wide. The crossings cannot be given
            // back their own places at THIS point in the build - the strips' end rings were laid
            // from `the_crossing_of` before `try_sew` is called, so they already carry the RIMS,
            // and a crossing re-minted afterwards is a second vertex a ruler off a vertex the wall
            // and the stitch are sewn to. At the hanger stand-off that minted the pair 754/755
            // beside the run terminals 544/545 and left a quadrilateral whose two long sides belong
            // to two different vertex pairs, which no polygon can close. The rims stand, the wall
            // and the stitch own the whole corner between them, and the joint's own step closes
            // what is left (`surface_mitre_joint_step`).
            return Ok(());
        }

        if !(area.length_squared() > 0.0) {
            // A patch with no area at all: the two rims and everything between them stand on one
            // line, so there is no piece of surface here to hand over.
            return Err(BevelAdvisory::new(
                BevelAdvisoryCode::SurfaceStripEndNotSupported,
                vec![pool.position_of(self.narrower_rim).into()],
            ));
        }

        // Which way round, taken from the WALL where the wall has already said - the rim the patch
        // closes along is a rim the walkback left, and its own triangles are in this very list -
        // and from the strip's bisector only where nothing has.
        let wound = match the_wall_used_the_rim(
            faces,
            &self.the_rim_side_as_the_patch_draws_it(&along_the_rim),
        ) {
            Some(forward) => !forward,
            None => area.dot(self.outward) >= 0.0,
        };

        for at in 1..outline.len().saturating_sub(1) {
            if outline[at] == outline[at + 1]
                || outline[0] == outline[at]
                || outline[0] == outline[at + 1]
            {
                continue;
            }

            faces.push(if wound {
                (outline[0], outline[at], outline[at + 1])
            } else {
                (outline[0], outline[at + 1], outline[at])
            });
        }

        try_the_ear(
            faces,
            self.narrower_rim,
            self.narrower_named,
            self.narrower_crossing,
        );
        try_the_ear(faces, self.wider_rim, self.wider_named, self.wider_crossing);

        Ok(())
    }

    /// The rim side with the NAMED band corner standing where the rim does - which is the side the
    /// patch actually lays, since [`Self::the_outline`] substitutes the corner for the rim at a
    /// named foot.
    ///
    /// The winding is read from the wall's use of the rim side, and a run read with the RIM's own
    /// vertex at its end answers for a segment the patch never draws: at a named foot the wall
    /// turns at the corner, so (rim, first) and (corner, first) are different edges that the wall
    /// may well walk opposite ways. Measured at the hanger stand-off, six of the twenty-nine named
    /// feet came out wound the wrong way for it - the patch walked both its crease segments the
    /// SAME way the wall did and its (corner, crossing) side the same way its own ear did, twelve
    /// doubled edges - and reading the substituted run instead is what puts the patch opposite the
    /// wall on the segments it lays. It settles the ear at the same time, because the ear's winding
    /// is taken from the wall's use of (corner, rim), the very edge that turn is made of.
    fn the_rim_side_as_the_patch_draws_it(&self, along_the_rim: &[i32]) -> Vec<i32> {
        let mut drawn = along_the_rim.to_vec();
        let last = drawn.len() - 1;

        drawn[0] = if self.wider_named >= 0 {
            self.wider_named
        } else {
            self.wider_rim
        };
        drawn[last] = if self.narrower_named >= 0 {
            self.narrower_named
        } else {
            self.narrower_rim
        };

        drawn
    }

    /// The patch's outline, fanned from its far corner, and the doubled area it encloses: out along
    /// the narrower foot's step, over the mitre polyline, back to the wider rim and home along the
    /// crease.
    fn the_outline(&self, pool: &SurfaceVertexPool, along_the_rim: &[i32]) -> (Vec<i32>, Vector3) {
        // At the CROSSING end the outline reads the NAMED corner where it read the rim: the
        // crossing is computed on the straight foot rail, and where the band was drawn on a
        // borrowed rail across a crease the wall's rim turns at the named corner instead, so a
        // patch drawn rim to crossing lays its side beside the wall's rather than on it. The rim
        // itself stays the end of the rim side, which is what the wall is sewn along and what the
        // winding is read from; the triangle between rim, corner and crossing is the ear `try_sew`
        // lays after the fan.
        let mut outline = Vec::with_capacity(self.over_the_narrowers_arc.len() + 3);
        outline.push(if self.narrower_named >= 0 {
            self.narrower_named
        } else {
            self.narrower_rim
        });
        outline.push(self.narrower_crossing);

        outline.extend_from_slice(&self.over_the_narrowers_arc);

        outline.push(if self.wider_named >= 0 {
            self.wider_named
        } else {
            self.wider_rim
        });

        // And home along the crease, through everything already standing on it between the two
        // rims: the band ends split that rim where their own end lines crossed it - the mitre plane
        // leaves one there - and a patch sewn straight from rim to rim would draw its edge across
        // those vertices and leave a T junction the shell cannot close over.
        outline.extend_from_slice(&along_the_rim[1..along_the_rim.len() - 1]);

        self.turn_the_outline_to_the_far_corner(pool, &mut outline);

        let mut area = Vector3::ZERO;

        for at in 1..outline.len().saturating_sub(1) {
            area += (pool.position_of(outline[at]) - pool.position_of(outline[0]))
                .cross(pool.position_of(outline[at + 1]) - pool.position_of(outline[0]));
        }

        (outline, area)
    }

    /// Turns the outline so that it is fanned from the WIDER foot's own crossing - the far corner
    /// where the mitre plane, that run's far landing plane and the narrower run's cylinder all
    /// meet - falling back to the furthest point off the rim where that corner stands on the rim
    /// itself.
    ///
    /// **Why that corner and not the fattest one.** Which corner the patch is fanned from decides
    /// which CHORDS of it are drawn, and a chord the neighbouring piece also draws is a triangle
    /// laid twice. Fanned from the narrower run's crossing - which
    /// [`Self::turn_the_outline_off_the_rim`] picks, because on a sliver whose two long sides lie
    /// eight ten-thousandths apart it is the point furthest off the rim - the patch's first ear is
    /// (narrower crossing, wider crossing, wider rim). Those three stand CONSECUTIVE on the wider
    /// strip's own last-column ring, whose near side reads rail, narrower crossing, wider crossing,
    /// rim, so the strip's fan can cut the very same ear: measured on the raked bore, 21 of the 38
    /// surplus corners drew that triangle twice, once each way round, and every edge of it carried
    /// four faces. Fanned from the WIDER crossing instead, no triangle of the patch has three
    /// consecutive corners of that ring, the 21 go, and the mouth's open count falls from 30 edges
    /// to 9.
    ///
    /// The corner is off the rim by construction - `the_foot_overruns` is what read it, and it only
    /// answers where the crossing stands further from its rim than the weld - but off the rim POINT
    /// is not off the rim LINE, so the fallback is kept for a patch whose far corner happens to
    /// stand along the crease.
    fn turn_the_outline_to_the_far_corner(&self, pool: &SurfaceVertexPool, outline: &mut [i32]) {
        let at = outline.iter().position(|&v| v == self.wider_crossing);

        let Some(at) = at.filter(|_| {
            !(self.off_the_rim(pool, pool.position_of(self.wider_crossing)) <= pool.near_enough())
        }) else {
            self.turn_the_outline_off_the_rim(pool, outline);

            return;
        };

        if at == 0 {
            return;
        }

        outline.rotate_left(at);
    }

    /// How far a point stands off the straight rim the patch closes along.
    fn off_the_rim(&self, pool: &SurfaceVertexPool, point: Vector3) -> f64 {
        let from = pool.position_of(self.narrower_rim);
        let mut along = pool.position_of(self.wider_rim) - from;
        let length = along.length();

        if !(length > 0.0) {
            return 0.0;
        }

        along /= length;

        let offset = point - from;

        (offset - (offset.dot(along) * along)).length()
    }

    /// Turns the outline so that the vertex it is FANNED from is the one standing furthest off the
    /// rim the patch closes along - and so off the one straight run the outline is known to have.
    ///
    /// The patch's last side is the rim between the two runs' own rims, and
    /// [`standing_along_the_rim`] collects the vertices already standing ON it, so that side is a
    /// STRAIGHT run of three points or more by construction. Fanned from the first of those - which
    /// the narrower rim is - every triangle covering that run has its apex on the very line its
    /// other two corners stand on, so it has no area: at the raked bore's crease step the patch
    /// drew (3.3925, 0.4635, 1.375), (3.3858, 0.4635, 1.3683) and (3.3788, 0.4635, 1.3613), three
    /// points of one crease, and the step was covered by nothing at all. Turning the fan is enough
    /// because the rim is the only run the outline is known to be straight along; the rest is the
    /// narrower run's own arc, which turns at every station. A patch with nothing standing on its
    /// rim is a triangle either way round, so nothing without a step to cover is moved by this.
    fn turn_the_outline_off_the_rim(&self, pool: &SurfaceVertexPool, outline: &mut [i32]) {
        let from = pool.position_of(self.narrower_rim);
        let mut along = pool.position_of(self.wider_rim) - from;
        let length = along.length();

        if !(length > 0.0) {
            return;
        }

        along /= length;

        let mut apex = 0;
        let mut furthest = -1.0;

        for (at, &vertex) in outline.iter().enumerate() {
            let offset = pool.position_of(vertex) - from;
            let off = (offset - (offset.dot(along) * along)).length();

            if off > furthest {
                furthest = off;
                apex = at;
            }
        }

        if apex == 0 {
            return;
        }

        outline.rotate_left(apex);
    }
}

/// The one triangle left over where a foot NAMED a band corner it could not take: the fan now draws
/// corner to crossing and the wall already draws rim to corner, so rim, corner and crossing bound a
/// sliver nobody covers. The ear covers it, wound opposite to the wall's own use of (corner, rim) -
/// read out of the laid faces the way [`the_wall_used_the_rim`] reads the crease - so the rim
/// segment it shares with the wall is walked the other way and closes.
fn try_the_ear(faces: &mut Vec<(i32, i32, i32)>, rim: i32, named: i32, crossing: i32) {
    if named < 0 || named == rim || named == crossing || rim == crossing {
        return;
    }

    // Nothing having drawn rim to corner, there is no wall side here for an ear to close against
    // and no winding to take from one.
    let Some(forward) = the_wall_used_the_rim(faces, &[named, rim]) else {
        return;
    };

    faces.push(if forward {
        (rim, named, crossing)
    } else {
        (named, rim, crossing)
    });
}

/// Whether a face already laid uses one of the rim's own segments, and if so (`Some`) whether it
/// walks that segment the way the patch's outline runs along it - from the wider rim toward the
/// narrower.
///
/// **Why the mesh answers this and a direction does not.** The patch is wound to face out, and what
/// that means was read from [`super::SurfaceStrip::outward_reference`], the bisector of the strip's
/// two tangencies. That is right to within a right angle for the strip's own triangles; this patch
/// stands PAST the strip's far tangent line and at a chorded wall's mitre it lies all but square to
/// that bisector - measured on the raked bore, 0.000032 to 0.043, turning sign between corners that
/// are each other's mirror - so the winding taken from it is the arithmetic's and not the shape's.
/// The radius from the run's own centre line is no better: at a crease step the patch is a web
/// standing ACROSS the crease, facing neither along the ball's radius nor against it, and read
/// either way round it takes the raked mouth from 52 pieces laid alongside their neighbours to 72
/// and 92.
///
/// What has no such trouble is the rim itself. The patch closes along a rim the WALKBACK left,
/// whose triangles are already in the list being added to, and two faces meeting on an edge walk it
/// opposite ways. So where the wall has used one of the rim's segments the patch's winding is that
/// use reversed, exactly, and no reference direction is needed. The bisector stays as the answer
/// for a rim nothing has reached yet, which is what a patch at a corner the walkback did not touch
/// has.
fn the_wall_used_the_rim(faces: &[(i32, i32, i32)], along_the_rim: &[i32]) -> Option<bool> {
    for pair in along_the_rim.windows(2) {
        let (from, to) = (pair[0], pair[1]);

        for &(v0, v1, v2) in faces {
            for (first, second) in [(v0, v1), (v1, v2), (v2, v0)] {
                if first == from && second == to {
                    return Some(true);
                }

                if first == to && second == from {
                    return Some(false);
                }
            }
        }
    }

    None
}

/// The pool's vertices standing strictly between two points on the straight rim between them, in
/// order from `from` onward. Read over the whole pool rather than out of its buckets, which answer
/// which vertex is nearest a PLACE and not which stand along a LINE; a patch is rare enough to pay
/// for one pass.
fn standing_along_the_rim(pool: &SurfaceVertexPool, from: Vector3, to: Vector3) -> Vec<i32> {
    let mut along = to - from;
    let length = along.length();
    let mut found: Vec<(f64, i32)> = Vec::new();

    if !(length > 0.0) {
        return Vec::new();
    }

    along /= length;

    for vertex in 0..pool.count() {
        let offset = pool.position_of(vertex) - from;
        let at = offset.dot(along);

        if at > pool.near_enough()
            && at < length - pool.near_enough()
            && (offset - (at * along)).length() <= pool.near_enough()
        {
            found.push((at, vertex));
        }
    }

    // Ties in `at` are broken by vertex index, which is the order the pass found them in, so the
    // walk is the same on every run.
    found.sort_by(|left, right| left.0.total_cmp(&right.0).then(left.1.cmp(&right.1)));

    found.into_iter().map(|(_, vertex)| vertex).collect()
}
