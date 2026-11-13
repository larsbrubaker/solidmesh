//! What a path does where glass rests on an opaque surface: a glass face lying on a face of a part
//! that is not glass, a floor most of all (`glass_contacts` finds those faces when the scene is
//! built, by the rule it uses for glass on glass). The path loop asks this file at every traced
//! hit in a scene that has such a contact, and after every bounce from one. How close counts as
//! resting follows the size of the larger triangle of the pair, the `f32` trace's own error on it
//! (`glass_contacts` gives the magnitudes): over a floor quad of half side 40,000 glass lifted
//! 0.05 is taken as resting, so a caller keeps the triangles under its parts small.
//!
//! **A thin gap of air.** A real object resting on a surface touches it at a few points and has a
//! thin gap of air everywhere else, so light leaving the glass there meets the glass's own surface
//! first (reflected back in or refracted out, by Fresnel), and only light that got out meets the
//! opaque surface, right there. Light the opaque surface sends back up meets the glass's surface
//! again, from outside. Tracing cannot get this from the two faces alone: they lie at the same
//! distance along a ray, which of them a trace returns is down to `f32` rounding and part order,
//! and a continuing ray starts a rounding step off its face, which puts it past the other face.
//! So a contact vertex is decided here and never by a trace:
//!
//! - A hit on the back of a glass face resting on an opaque face is the glass's own exit event.
//! - A hit on the front of an opaque face a glass face rests on is the same event: the ray was in
//!   the glass and the trace happened to return the opaque face first. The vertex becomes the
//!   glass face at the same point, so which face a trace returns never matters.
//! - From a contact vertex, a bounce toward the side its face's normal points to (glass refracted
//!   out, glass reflected from outside, light sent up by the opaque surface) goes straight to the
//!   face across the contact at the same point, with no ray traced and nothing passed below either
//!   face. A bounce the other way (glass reflected back in, glass refracted in) is traced from the
//!   vertex as any other.
//!
//! A ray arriving from air at the very edge of the glass's footprint, where it meets the opaque
//! face inside the footprint's rounding, is taken into the glass's surface the same way, so light
//! never reaches the opaque surface under the glass without crossing the glass's surface.
//!
//! **Glass there on this crossing, on a fully there surface.** A contact is used only where the
//! part under the glass is at opacity 1, and the glass is there on this crossing: the hit is
//! asked about only after the line found its part there, which at opacity 1 is a stop that draws
//! no random number and counts no crossing, and below 1 is the line's one draw at that face
//! (`see_through_rest` makes the glass's face the one met first on either tie outcome). So
//! see-through glass found there leaves through its resting face by the same exit event and step
//! across as fully there glass. A hit on the opaque face is taken as the glass's face only for
//! glass at opacity 1, whose presence needs no draw. A part elsewhere in the scene being partly
//! there changes nothing, and a surface less than fully there under glass is no contact: the path
//! decides at whichever face a trace returns, as any two faces.
//!
//! **Random draws.** Glass at opacity 1 on a surface at opacity 1 passes the gate exactly where it
//! always did, so every draw stays where it was. For see-through glass on a fully there surface
//! the presence is drawn once per crossing at the glass's face as before; found there, its exit
//! event draws as that vertex always drew, and what moves is the path after it: the bounce out of
//! the face now goes across to the surface at the same point, untraced, where before it was traced
//! from past the surface into whatever lay below, and the surface's vertex then draws as any
//! vertex. Light the surface sends back up meets the glass across the contact, its presence drawn
//! there (one draw, where glass a modelled gap above would draw it), and found there enters it by
//! Fresnel from outside.
//!
//! **A seam comes first.** At a traced hit the path loop decides a glass-on-glass seam
//! (`glass_seam`) first and takes the rest only where there is none; the GPU loop (`path.wgsl`)
//! takes the rest on its floor first and then skips the seam. The two orders differ only at a hit
//! that is both a glass face resting on the floor's plane and covered there by another glass
//! part's face, a glass part reaching up to the floor from below with glass on top: there this
//! route takes the seam and the GPU route the rest.
//!
//! **Shadow rays** are not changed: a shadow ray from an opaque face under the glass starts its
//! rounding step above that face, inside the glass, and is stopped by the glass as any shadow ray
//! is.

use solidmesh_math::{Ray, Vector3};

use super::glass_contacts::is_glass;
use super::render_scene::RenderScene;
use crate::ray_tracer::{RayTracerError, SurfacePoint};

/// The vertex a path takes where a ray first meets the scene, as [`RenderScene::path_vertex_at`]
/// reports it.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct PathVertexAt {
    /// The part the vertex is on.
    pub part: usize,
    /// The face of that part.
    pub face: i32,
    /// Whether the path arrives at the back of the face, from inside a solid.
    pub back_face: bool,
    /// The part a bounce toward the side the face's normal points to meets next, at the same
    /// point and without a trace, where the face is glass resting on an opaque surface or the
    /// opaque surface under it; `None` where such a bounce is traced.
    pub across: Option<usize>,
}

impl RenderScene {
    /// The vertex a path in this scene takes at the first hit of `ray`, `None` when the ray
    /// meets nothing: which part and face it scatters at and, where glass rests on an opaque
    /// surface, which part across the contact a bounce out of the face meets next. This is what
    /// the path loop decides at that hit where the part hit is there (always, at opacity 1), at a
    /// face no other glass lies on; a caller can check it ray by ray against another renderer's
    /// decisions. A test hook, not part of the stable interface.
    #[doc(hidden)]
    pub fn path_vertex_at(&self, mut ray: Ray) -> Result<Option<PathVertexAt>, RayTracerError> {
        let direction = ray.direction_normal;
        let Some(hit) = self.parts().trace(&mut ray) else {
            return Ok(None);
        };
        let point = hit.surface_point()?;
        let (part, point, partner) = if self.glass_rests().is_empty() {
            (hit.part.index(), point, None)
        } else {
            resting_vertex(self, hit.part.index(), point, direction)?
        };
        let across = partner.map(|partner| partner.part);
        Ok(Some(PathVertexAt {
            part,
            face: point.face_index,
            back_face: direction.dot(point.geometric_normal) > 0.0,
            across,
        }))
    }
}

/// The face across a contact from a vertex.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub(super) struct Partner {
    pub(super) part: usize,
    pub(super) face: i32,
}

/// A vertex reached across a contact, without a trace.
#[derive(Debug, Clone, Copy)]
pub(super) struct Across {
    /// The part the vertex is on.
    pub(super) part: usize,
    /// The point on its face.
    pub(super) point: SurfacePoint,
    /// The face across the contact from it, the one the path came from.
    pub(super) partner: Partner,
}

/// The vertex a traced hit on face `point.face_index` of `part` along `direction` is: the hit
/// itself with no partner where no glass rests there, else the glass face's exit or entry event
/// and the face across from it (see the header).
pub(super) fn resting_vertex(
    scene: &RenderScene,
    part: usize,
    point: SurfacePoint,
    direction: Vector3,
) -> Result<(usize, SurfacePoint, Option<Partner>), RayTracerError> {
    let Ok(face) = usize::try_from(point.face_index) else {
        return Ok((part, point, None));
    };
    let Some(contact) = scene.glass_rests().covering(part, face, point.position) else {
        return Ok((part, point, None));
    };
    let here_glass = scene.surface(part).is_some_and(is_glass);
    // The hit part was found there by the line before this is asked; the part across, never asked
    // here, has to be there for everyone (see the header).
    let (glass_there, under_there) = if here_glass {
        (true, fully_there(scene, contact.part))
    } else {
        (fully_there(scene, contact.part), fully_there(scene, part))
    };
    if !glass_there || !under_there {
        return Ok((part, point, None));
    }
    let toward_normal = direction.dot(point.geometric_normal);
    let partner_face = face_number(contact.face)?;
    if here_glass && toward_normal > 0.0 {
        return Ok((
            part,
            point,
            Some(Partner {
                part: contact.part,
                face: partner_face,
            }),
        ));
    }
    if !here_glass && toward_normal < 0.0 {
        let glass =
            scene.parts().parts()[contact.part].surface_point(partner_face, point.position)?;
        return Ok((
            contact.part,
            glass,
            Some(Partner {
                part,
                face: point.face_index,
            }),
        ));
    }
    Ok((part, point, None))
}

/// The vertex across the contact from the one at `point` on `part`, when a bounce along
/// `outgoing` goes to the side the face's normal points to; `None` when it is to be traced.
pub(super) fn across_contact(
    scene: &RenderScene,
    part: usize,
    point: &SurfacePoint,
    partner: Partner,
    outgoing: Vector3,
) -> Result<Option<Across>, RayTracerError> {
    if outgoing.dot(point.geometric_normal) <= 0.0 {
        return Ok(None);
    }
    let Some(other) = scene.parts().parts().get(partner.part) else {
        return Ok(None);
    };
    Ok(Some(Across {
        part: partner.part,
        point: other.surface_point(partner.face, point.position)?,
        partner: Partner {
            part,
            face: point.face_index,
        },
    }))
}

/// Whether part `part` is at opacity 1, so the line never decides anything at it.
fn fully_there(scene: &RenderScene, part: usize) -> bool {
    scene
        .part_opacity(part)
        .is_some_and(|opacity| opacity >= 1.0)
}

fn face_number(face: usize) -> Result<i32, RayTracerError> {
    i32::try_from(face).map_err(|_| {
        RayTracerError::InvalidArgument("a contact face's index is beyond what a mesh can hold")
    })
}
