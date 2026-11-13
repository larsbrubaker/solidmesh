//! What a line does where a see-through part rests on an opaque surface: a face of a part less
//! than fully there lying on a face of a fully there part that is not glass, a floor most of all
//! (`glass_contacts` finds those faces when the part opacities are set, by the rule it uses for
//! glass). The path loop asks this file at every traced hit in a scene that has such a contact,
//! and after every bounce from a vertex on one.
//!
//! **Why the trace can't decide it.** The two faces lie at the same distance along a line, so
//! which of them a trace returns is down to `f32` rounding, and a line going on past the
//! see-through part starts a rounding step beyond the face it went through, which puts it past
//! the surface lying there: half the lines leaving such a part through its bottom went on under
//! the floor and met nothing. So the order the line meets the two faces is fixed here, by the
//! contact and never by the trace:
//!
//! - **Each face is met from its own side first.** A traced hit on the front of either face of
//!   the contact (arriving from the side its normal points to, which is the other part's side),
//!   by a line that started beyond the other face, is taken as the hit on the other face at the
//!   same point: a line coming down onto the floor inside the part's footprint went through the
//!   part's bottom first, and a line coming up into the part's bottom from under the floor went
//!   through the floor first. A line that started between the two planes (one going on up from
//!   the floor's own continuing-ray start) crossed nothing else and keeps its hit. So the part's
//!   presence is decided at its bottom on either tie outcome, once per crossing as at any face
//!   (`see_through`), and a fully there face asks nothing.
//! - **A line passing the part out through the face lands on the surface** at the same point, as
//!   the next vertex, with no ray traced ([`onto_rest`]).
//! - **From that vertex, a bounce toward the side the surface's normal points to** (light sent up
//!   by the floor) goes to the part's face across the contact at the same point, with no ray
//!   traced, and the line meets the part there: its presence is decided as at a traced hit.
//!   Where it is there, the vertex is its face, and a bounce from it toward its own normal goes
//!   back across to the surface the same way. Where it is not, the line goes on from the
//!   surface's own continuing-ray start, a rounding step off the surface along the bounce, with
//!   the part already passed: the start lies past the surface, and on either side of the part's
//!   face within the contact's tolerance, and the part's face met from either side is passed
//!   with no draw. A bounce the other way is traced as any other.
//!
//! **Random numbers.** Nothing here draws a number of its own. A face of a fully there part asks
//! the line nothing and counts no crossing, so taking it in place of a traced hit, or reaching it
//! across the contact, moves no draw; the see-through part is decided exactly where a line
//! meeting it from the same side would decide it with the two faces a gap apart. A scene with no
//! see-through part resting on anything never asks.
//!
//! **Found there.** A see-through part found there at its resting face is a vertex there as at
//! any face. Where the part is glass and the line leaves it through the face, that vertex is the
//! glass's own exit event and a bounce out of the face (refracted out of its bottom) steps across
//! onto the surface, as for glass at opacity 1 (`glass_rest`).
//!
//! **No seam on a contact.** The path loop looks for a glass-on-glass seam (`glass_seam`) at no
//! traced hit on either face of such a contact, whichever face the trace returned. A third face
//! in that plane is on no such contact (the top of glass reaching up to the floor's plane from
//! below, under see-through glass resting on the floor): a trace can still return it there, and
//! its seam with the glass above is then found. The GPU loop still looks for one where it decides
//! the see-through part's face (found not there, or there and not leaving through it), so the two
//! routes differ at a see-through glass face on the floor that another glass part's face also
//! covers: glass reaching up to the floor's plane from below with glass on top.
//!
//! **Limits.** A see-through part resting on glass or on another see-through part is traced as
//! any two faces are. A line meeting the surface inside the part's footprint within rounding of
//! the footprint's edge is taken as having crossed the part's face there.

use solidmesh_math::{Ray, Vector3};

use super::glass_rest::{resting_vertex, Across, Partner, PathVertexAt};
use super::render_scene::RenderScene;
use super::see_through::{continued_ray, AtHit, StraightLine};
use crate::ray_tracer::{RayTracerError, SurfacePoint};

/// Where a straight line in a scene first meets a part that is there, as
/// [`RenderScene::line_vertex_at`] reports it.
#[derive(Debug, Clone, PartialEq)]
pub struct LineVertexAt {
    /// The vertex: its part, face, side, and where a see-through part rests on the surface there,
    /// that part as the one a bounce up from it meets across the contact.
    pub vertex: PathVertexAt,
    /// The vertex's position.
    pub position: Vector3,
    /// The parts whose presence the line decided, in order: one entry per draw a path makes.
    pub decided: Vec<usize>,
}

impl RenderScene {
    /// The vertex a path in this scene takes where `ray` first meets a part that is there, the
    /// parts in `absent` found not there wherever the line asks and every other part found there;
    /// `None` when the line meets nothing or ends at the crossing cap. The line is followed by the
    /// path loop's own decisions at a see-through part resting on an opaque surface
    /// (`see_through_rest`'s header), so a caller can check them ray by ray against another
    /// renderer's. A test hook, not part of the stable interface.
    #[doc(hidden)]
    pub fn line_vertex_at(
        &self,
        mut ray: Ray,
        absent: &[usize],
    ) -> Result<Option<LineVertexAt>, RayTracerError> {
        let direction = ray.direction_normal;
        let mut line = StraightLine::default();
        let mut decided = Vec::new();
        loop {
            let Some(hit) = self.parts().trace(&mut ray) else {
                return Ok(None);
            };
            let traced = hit.surface_point()?;
            let (part, point) = met_first(self, hit.part.index(), &traced, &ray)?
                .unwrap_or((hit.part.index(), traced));
            let verdict = line.at_hit_with(self, part, |_| {
                decided.push(part);
                !absent.contains(&part)
            });
            let (part, point, across) = match verdict {
                AtHit::Ended => return Ok(None),
                AtHit::PassesThrough => match onto_rest(self, part, &point, direction)? {
                    Some(next) => (next.part, next.point, Some(next.partner.part)),
                    None => {
                        ray = continued_ray(&point, direction);
                        continue;
                    }
                },
                AtHit::Stops if self.glass_rests().is_empty() => (part, point, None),
                AtHit::Stops => {
                    let (part, point, partner) = resting_vertex(self, part, point, direction)?;
                    (part, point, partner.map(|partner| partner.part))
                }
            };
            return Ok(Some(LineVertexAt {
                vertex: PathVertexAt {
                    part,
                    face: point.face_index,
                    back_face: direction.dot(point.geometric_normal) > 0.0,
                    across,
                },
                position: point.position,
                decided,
            }));
        }
    }
}

/// The part and point `ray` meets first at its traced hit at `point` on a face of the scene,
/// when that is not the hit: where the hit is on the front of a face of a see-through rest
/// contact and the ray started beyond the other face, that face at the same point (see the
/// header). `None` where the hit is what the line meets.
pub(super) fn met_first(
    scene: &RenderScene,
    part: usize,
    point: &SurfacePoint,
    ray: &Ray,
) -> Result<Option<(usize, SurfacePoint)>, RayTracerError> {
    let rests = scene.see_through_rests();
    if rests.is_empty() || ray.direction_normal.dot(point.geometric_normal) >= 0.0 {
        return Ok(None);
    }
    let Ok(face) = usize::try_from(point.face_index) else {
        return Ok(None);
    };
    let Some(contact) = rests.covering(part, face, point.position) else {
        return Ok(None);
    };
    let Some(other) = scene.parts().parts().get(contact.part) else {
        return Ok(None);
    };
    let other_point = other.surface_point(face_number(contact.face)?, point.position)?;
    // Beyond the other face is behind it: on the side its normal points away from.
    if (ray.origin - other_point.position).dot(other_point.geometric_normal) >= 0.0 {
        return Ok(None);
    }
    Ok(Some((contact.part, other_point)))
}

/// Whether `point` on `part` lies on a face of a see-through rest contact, on either side of it.
pub(super) fn on_rest_contact(scene: &RenderScene, part: usize, point: &SurfacePoint) -> bool {
    usize::try_from(point.face_index).is_ok_and(|face| {
        scene
            .see_through_rests()
            .covering(part, face, point.position)
            .is_some()
    })
}

/// The vertex a line that found `part` not there reaches, leaving it at `point` along
/// `direction`, when the face there rests on an opaque surface: that surface at the same point,
/// with the part across from it. `None` where the line is to be traced on.
pub(super) fn onto_rest(
    scene: &RenderScene,
    part: usize,
    point: &SurfacePoint,
    direction: Vector3,
) -> Result<Option<Across>, RayTracerError> {
    let rests = scene.see_through_rests();
    if rests.is_empty() || direction.dot(point.geometric_normal) <= 0.0 {
        return Ok(None);
    }
    let Ok(face) = usize::try_from(point.face_index) else {
        return Ok(None);
    };
    let Some(contact) = rests.covering(part, face, point.position) else {
        return Ok(None);
    };
    let Some(surface) = scene.parts().parts().get(contact.part) else {
        return Ok(None);
    };
    Ok(Some(Across {
        part: contact.part,
        point: surface.surface_point(face_number(contact.face)?, point.position)?,
        partner: Partner {
            part,
            face: point.face_index,
        },
    }))
}

fn face_number(face: usize) -> Result<i32, RayTracerError> {
    i32::try_from(face).map_err(|_| {
        RayTracerError::InvalidArgument("a contact face's index is beyond what a mesh can hold")
    })
}
