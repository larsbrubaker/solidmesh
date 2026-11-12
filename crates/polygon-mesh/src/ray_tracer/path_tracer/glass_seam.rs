//! What a path does at a seam: a hit on a face of a glass part that a face of another glass part
//! lies on (`glass_contacts` finds those faces when the scene is built). The path tracer's header
//! says what light does there; this file decides it for one hit.
//!
//! **Which glass is on which side.** Two faces lying on each other are found in either order by a
//! trace, since they are at the same distance. Arriving at the back of the hit face, the path is
//! leaving the hit part into the covering one; arriving at its front, it is leaving the covering
//! part into the hit one. Either way the answer is the same.
//!
//! **Whether each part is there.** A part only partly there is decided per straight line, as at
//! any other hit (`see_through`): the path loop decides the hit part before this file is asked,
//! and the covering part is decided here by the same rule, at the same point of the line, so the
//! two faces are one boundary whichever of them the trace returned. Both there: a seam, below.
//! Only one there: that part's glass against air, its face oriented as that part's own; a ray
//! leaving that interface into the other part starts a line that finds the other part not there
//! either, so a path never leaves a glass it did not enter. Neither: the line goes on past both.
//!
//! **Going on along the line** ([`Seam::Straight`], for the same glass on both sides or neither
//! part there) changes nothing but the origin. It starts `SKIP_TOLERANCES` contact tolerances off
//! the hit face's plane on the far side, measured along the normal as every continuing ray is:
//! both faces are within one tolerance of that plane, so neither is hit again, and a real face
//! further than that is still found. Every interface at a seam leaves by the same skip. Each pass
//! counts against the line's crossings, so a line ends after `MAX_CROSSINGS_PER_LINE` of them.
//!
//! **Different glasses** meet at one interface ([`Seam::Interface`]) of their relative index, the
//! glass entered over the glass left. The scattering model takes an index of at least 1 from the
//! outside, so a step down to a thinner glass is given as the inverse ratio with the frame turned
//! round: the path is then leaving the denser glass, as it is. The interface's colour is the two
//! glasses' colours multiplied, since one crossing tints by the square root of its surface's
//! colour and the path leaves one glass and enters the other at once; it is as rough, as metallic
//! and as little transmissive as the more so of the two. A path reflected there stays in the
//! glass it was in.
//!
//! **Not exact at partial opacity.** A line that scatters starts a new line, and a new line
//! decides a partly there part afresh, so a path that entered a part found there can leave it
//! through a face where the part is not: that part's glass then neither bends nor scales what the
//! path carries on the way out, and the path loses the light refraction had concentrated in it.
//! This is so for a single partly there glass part as much as for touching ones, and loses light,
//! never gains it.

use solidmesh_math::Vector3;

use super::pixel_random::PixelRandom;
use super::render_scene::RenderScene;
use super::see_through::{AtHit, StraightLine};
use crate::ray_tracer::{
    MinimalTriangle, PartHit, RayTracerError, ScatteringSurface, SurfacePoint,
};

/// How many contact tolerances off the seam a continuing ray starts.
pub(crate) const SKIP_TOLERANCES: f64 = 2.0;

/// What a hit on a seam is.
pub(super) enum Seam {
    /// No interface: the path goes on from `origin` along the same direction.
    Straight { origin: Vector3 },
    /// One interface, between two glasses or between one glass and air.
    Interface(Interface),
    /// The line has gone through as many surfaces as it may.
    Ended,
}

/// The one interface at a seam.
pub(super) struct Interface {
    /// The surface the scattering model sees, its index the relative one.
    pub(super) surface: ScatteringSurface,
    /// Whether the side the path came from is the thinner, the scattering model's outside.
    pub(super) into_denser: bool,
    /// How far off the seam a continuing ray starts.
    skip: f64,
    /// The part of the two not there on the line that reached the seam, and whether its side is
    /// the one the hit face's normal points to.
    absent: Option<(usize, bool)>,
}

impl Interface {
    /// The part not there on the line that reached the seam, when a ray leaving the seam along
    /// `direction` goes into it: the new line must find it not there too, or the path would
    /// leave a glass it never entered.
    pub(super) fn absent_part_toward(
        &self,
        point: &SurfacePoint,
        direction: Vector3,
    ) -> Option<usize> {
        let (part, on_normal_side) = self.absent?;
        let toward_normal_side = direction.dot(point.geometric_normal) >= 0.0;
        (toward_normal_side == on_normal_side).then_some(part)
    }

    /// Where a ray leaving the seam along `direction` starts: past both faces.
    pub(super) fn next_ray_origin(&self, point: &SurfacePoint, direction: Vector3) -> Vector3 {
        off_seam(point, direction, self.skip)
    }
}

fn off_seam(point: &SurfacePoint, direction: Vector3, skip: f64) -> Vector3 {
    if direction.dot(point.geometric_normal) >= 0.0 {
        point.position + point.geometric_normal * skip
    } else {
        point.position - point.geometric_normal * skip
    }
}

/// The seam at `hit`, for a path arriving along `direction` with the hit part there on this line
/// when `here_present`; `None` where the hit face meets air (or anything but glass).
pub(super) fn glass_seam(
    scene: &RenderScene,
    hit: &PartHit<'_>,
    direction: Vector3,
    here_present: bool,
    line: &mut StraightLine,
    random: &mut PixelRandom,
) -> Result<Option<Seam>, RayTracerError> {
    let Some(leaf) = hit.info.closest_hit_as::<MinimalTriangle>() else {
        return Ok(None);
    };
    let Ok(face) = usize::try_from(leaf.face_index) else {
        return Ok(None);
    };
    let part = hit.part.index();
    let Some(contact) = scene
        .glass_contacts()
        .covering(part, face, hit.info.hit_position)
    else {
        return Ok(None);
    };
    let point = hit.surface_point()?;
    let Some(other_part) = scene.parts().parts().get(contact.part) else {
        return Ok(None);
    };
    let (Some(here), Some(there)) = (
        scene.surface_at(part, point.base_color),
        scene.surface_at(contact.part, other_part.surface().face_color(contact.face)),
    ) else {
        return Ok(None);
    };
    let there_present = if scene.has_see_through_parts() {
        match line.at_hit(scene, contact.part, random) {
            AtHit::Stops => true,
            AtHit::PassesThrough => false,
            AtHit::Ended => return Ok(Some(Seam::Ended)),
        }
    } else {
        true
    };
    let skip = SKIP_TOLERANCES * contact.tolerance;
    let leaving_here = point.facing(direction).back_face;
    let (surface, into_denser) = match (here_present, there_present) {
        (false, false) => {
            return Ok(Some(Seam::Straight {
                origin: off_seam(&point, direction, skip),
            }))
        }
        (true, true) if here == there => {
            if !line.count_pass() {
                return Ok(Some(Seam::Ended));
            }
            return Ok(Some(Seam::Straight {
                origin: off_seam(&point, direction, skip),
            }));
        }
        // One glass against air: entering it from the side the path came from, or leaving it.
        // The covering part lies on the side the hit face's normal points to, the hit part on
        // the other.
        (true, false) => (here, !leaving_here),
        (false, true) => (there, leaving_here),
        (true, true) => {
            let (from, into) = if leaving_here {
                (here, there)
            } else {
                (there, here)
            };
            (seam_surface(&from, &into), into.ior() >= from.ior())
        }
    };
    let absent = match (here_present, there_present) {
        (true, false) => Some((contact.part, true)),
        (false, true) => Some((part, false)),
        _ => None,
    };
    Ok(Some(Seam::Interface(Interface {
        surface,
        into_denser,
        skip,
        absent,
    })))
}

/// The surface of the interface leaving glass `from` into glass `into`.
pub(crate) fn seam_surface(
    from: &ScatteringSurface,
    into: &ScatteringSurface,
) -> ScatteringSurface {
    let ior = if into.ior() >= from.ior() {
        into.ior() / from.ior()
    } else {
        from.ior() / into.ior()
    };
    let (a, b) = (from.base_color(), into.base_color());
    ScatteringSurface::new(
        [a[0] * b[0], a[1] * b[1], a[2] * b[2]],
        from.metallic().max(into.metallic()),
        from.roughness().max(into.roughness()),
        from.transmission().min(into.transmission()),
        ior,
    )
}
