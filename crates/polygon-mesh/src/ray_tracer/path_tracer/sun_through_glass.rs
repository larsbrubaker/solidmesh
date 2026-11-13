//! The lines toward the sun: where a line starts, what it goes through, and what reaches the
//! vertex. `integrator` asks this file for a vertex's one shadow ray; `glass_chain` says which
//! share of the vertex's light counts what the line found, and why each light path is counted
//! once. A line answers whether it reached the sun, what the glass along it passed and how many
//! faces of glass it crossed (`SunReach`); a vertex that only needs Exact's answer (reached with
//! no crossing) asks in `LineMode::StopAtAny`, which stops at the first surface as Exact's line
//! does but starts where a straight line would.
//!
//! **Straight through glass** (`SunThroughGlass::Straight`). A line goes on through every face
//! with a glass share (`ScatteringSurface::passes_sun_straight`), smooth or frosted, along the
//! same line, and multiplies what it carries by that face's `straight_transmittance`: the glass
//! share times `1 - Fresnel` at the line's angle to the face's shading normal times the
//! one-crossing tint, which is what a smooth face's delta refraction gives a camera path there,
//! less the lobe's choice probability and the radiance scaling. It stops, the sun unseen, at
//! anything else: an opaque surface, a part found there by the line's decision (`see_through`,
//! drawn as a path draws it), or the `MAX_GLASS_CROSSINGS`-th glass face past the last. The line
//! is not bent, so glass casts a light shadow (tinted by tinted glass) and focuses nothing (no
//! caustics), and frosted glass does not spread what it passes. What a pane's refraction would
//! do is approximated in these cases, each named: a face not parallel to the one the light
//! entered by moves the shadow and reads its Fresnel at the unbent angle (`straight_transmittance`
//! says why that is reciprocal only for parallel faces); light that went back and forth inside
//! the glass is left out (`glass_chain` says how much).
//!
//! **No `ior^2` on the shadow ray.** Radiance inside glass of index `n` is `n^2 (1 - F)` times
//! the sun's, but the sun's disc seen from inside is `n^2` times smaller in solid angle, so the
//! irradiance at a point inside is `(1 - F)` times the irradiance the unbent disc gives:
//! `L_in cos_in dw_in = (1 - F) L cos_out dw_out`. The unbent line already integrates over the
//! outside disc, so it takes `(1 - F)` and no more, for one crossing (a point inside glass) and
//! for two (a pane, whose `n^2` and `1 / n^2` would cancel anyway). The solid-angle statement is
//! exact for a disc near the face's normal; the `cos_in dw_in = cos_out dw_out / n^2` it rests on
//! holds at every angle, but the irradiance it gives is the receiver's only when the receiver
//! lies parallel to the face crossed, since the unbent line meets a tilted receiver at a cosine
//! the bent one would not. The camera path keeps its own `n^2` factors for its own crossings. A
//! receiver inside glass reads its lobe along the bent direction instead (`glass_chain`, "A
//! receiver inside glass"), with the `n^2` and the narrowed disc both counted, which gives the
//! same irradiance; `sun_reach_inside` walks its line and reports the first face crossed from
//! inside its solid (`FirstFace`), found by the walk's first trace at no extra cost.
//!
//! **Exact's line** (`exact_reach`) stops at any surface: the plain shadow ray, with no start rule
//! at a seam or a contact. Under Straight the glass lobe's share of a vertex counts only a line
//! that reached the sun crossing no glass, read from the same walk as the other lobes' share, or
//! asked alone in `LineMode::StopAtAny`; either starts where a straight line does (below), so a
//! seam's line toward the sun's side starts past both faces there, and a contact's crosses the
//! glass first, which the glass share does not count.
//!
//! **Where a straight line starts.** At a seam between two glass parts, past both faces on the
//! side the sun is, the line starting past a part the path found not there, as a bounce from the
//! seam does. At a vertex on a contact (a part resting on an opaque face, `glass_rest` and
//! `see_through_rest`), a sun on the side of the face across meets that face first, at the same
//! point and with no trace, as the path's own bounce does: the part across is decided there by
//! the line's own draw, as at a traced face (the opaque face or a part at opacity 1 draws
//! nothing). Found there, glass is crossed from the thin gap of air between them and the line
//! starts inside it, a fresh line that decides the glass again at its far face as a path does
//! after refracting in; anything else found there (the opaque face across from glass, a
//! see-through part that is not glass) stops the line. Found not there, the line starts at the
//! vertex's own continuing-ray start with the part passed. The sun line re-decides the part
//! rather than carrying the decision the path made arriving at the vertex, because a vertex
//! starts a fresh line: a lifted part a modelled gap above the vertex is decided afresh by the
//! sun line that meets it, and the part resting there must read the same. A line meeting a glass
//! face resting on an opaque part, or that part's face under the glass, is stopped; a face the
//! line would not pass is not asked whether it rests on another.
//!
//! **What a line costs.** A straight line is walked nearest hit first from its first trace, so a
//! clear line costs one trace, a line through glass one more than the faces it crosses, and a line
//! blocked behind glass one per face before the blocker and one for it. A first any-hit trace
//! would save the faces before a blocker it happened to find, but costs a line through glass, the
//! common case under glass, one trace that finds the first face again. A line that only needs
//! Exact's answer (`LineMode::StopAtAny`) is one any-hit trace.
//!
//! **Same work without glass.** A scene with no glass takes the one walk it always did: a single
//! any-hit trace with no part partly there, or the line walked past see-through parts, drawing
//! the same random numbers in the same order. Its image is unchanged to the bit.

use solidmesh_math::{IntersectionType, Ray, Vector3};

use super::glass_rest::{across_contact, resting_vertex, Partner};
use super::glass_seam::{glass_seam, Interface, Seam};
use super::pixel_random::PixelRandom;
use super::render_scene::RenderScene;
use super::render_work::RenderWork;
use super::see_through::{continued_ray, AtHit, StraightLine};
use super::see_through_rest::{met_first, onto_rest};
use crate::ray_tracer::{RayTracerError, ScatteringSurface, SurfacePoint};

/// The most faces of glass a shadow ray goes through; at the next one it counts as blocked. Eight
/// panes, or a solid glass part folded back on itself several times; a line through more carries
/// little of the sun past them and would cost a trace per face.
pub(crate) const MAX_GLASS_CROSSINGS: u32 = 16;

/// The vertex a shadow ray leaves from.
pub(super) struct ShadowVertex<'a> {
    pub(super) part: usize,
    pub(super) point: &'a SurfacePoint,
    pub(super) seam: Option<&'a Interface>,
    pub(super) partner: Option<Partner>,
}

/// What reaches a vertex from the sun along a shadow ray that reached it.
pub(super) struct SunReach {
    /// What the glass along the line passes on, 1 with none.
    pub(super) transmittance: [f64; 3],
    /// How many faces of glass the line crossed, the start's included.
    pub(super) crossed: u32,
}

/// The first face of glass a straight line crossed from inside its solid (hit on its back), which
/// a receiver inside glass reads the sun through (`glass_chain`, "A receiver inside glass").
#[derive(Clone, Copy, Debug, PartialEq)]
pub(super) struct FirstFace {
    /// The face's shading normal, pointing out of the solid along the line.
    pub(super) outward: Vector3,
    /// The glass's index of refraction.
    pub(super) ior: f64,
}

/// The direction inside glass that leaves through a face along a line, and the two cosines.
#[derive(Clone, Copy, Debug, PartialEq)]
pub(super) struct Bent {
    /// The direction inside the glass, toward the face.
    pub(super) inside: Vector3,
    /// The line's cosine to the face's normal outside.
    pub(super) cos_out: f64,
    /// The bent direction's cosine to it inside.
    pub(super) cos_in: f64,
}

impl FirstFace {
    /// The direction inside the glass that refracts out through this face along `line` (Snell's
    /// law, which from outside in never reflects totally), or `None` when the line leaves the
    /// face on its inside (a shading normal bent past the line) or the glass is less dense than
    /// air, where some lines have no direction inside.
    pub(super) fn bent(&self, line: Vector3) -> Option<Bent> {
        let cos_out = self.outward.dot(line);
        if cos_out <= 0.0 || self.ior < 1.0 {
            return None;
        }
        let tangent = line - self.outward * cos_out;
        let sin2_in = (1.0 - cos_out * cos_out) / (self.ior * self.ior);
        let cos_in = (1.0 - sin2_in).max(0.0).sqrt();
        let inside = (tangent / self.ior + self.outward * cos_in).get_normal();
        Some(Bent {
            inside,
            cos_out,
            cos_in,
        })
    }
}

/// What a line toward the sun from a vertex found, as [`RenderScene::sun_line_at`] reports it.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct SunLineAt {
    /// What reached the vertex (the glass's transmittance and how many faces of glass the line
    /// crossed), `None` when the line was blocked.
    pub reach: Option<([f64; 3], u32)>,
    /// The first face of glass the line crossed from inside its solid: its outward shading normal
    /// and the glass's index.
    pub first_face: Option<(Vector3, f64)>,
    /// The direction inside that face's glass that leaves along the line, the line's cosine to
    /// the face outside and the bent direction's inside (`FirstFace::bent`).
    pub bent: Option<(Vector3, f64, f64)>,
}

impl RenderScene {
    /// The line toward the sun along `direction` from the vertex a path in this scene takes where
    /// `ray` first meets a part that is there, the parts in `absent` found not there wherever the
    /// line to the vertex asks and every other part found there: the vertex's seam and its
    /// contact partner as the path loop finds them and, when `step_across`, the vertex across the
    /// contact a bounce along `ray`'s direction reaches. The line toward the sun draws its own
    /// decisions from `random` (after a seam's covering part in a scene with see-through parts,
    /// decided from it there), straight through glass unless `stop_at_any`: the line
    /// `SunThroughGlass::Straight` walks from that vertex, which in a scene with no glass a line
    /// passes is Exact's line from a straight line's start. `None` when `ray` meets nothing that
    /// is there. A test hook, not part of the stable interface.
    #[doc(hidden)]
    pub fn sun_line_at(
        &self,
        mut ray: Ray,
        absent: &[usize],
        step_across: bool,
        direction: Vector3,
        stop_at_any: bool,
        random: &mut PixelRandom,
    ) -> Result<Option<SunLineAt>, RayTracerError> {
        let along = ray.direction_normal;
        let mut line = StraightLine::default();
        let (part, point, seam, partner) = loop {
            let Some(hit) = self.parts().trace(&mut ray) else {
                return Ok(None);
            };
            let traced = hit.surface_point()?;
            let (part, point) = met_first(self, hit.part.index(), &traced, &ray)?
                .unwrap_or((hit.part.index(), traced));
            match line.at_hit_with(self, part, |_| !absent.contains(&part)) {
                AtHit::Ended => return Ok(None),
                AtHit::PassesThrough => match onto_rest(self, part, &point, along)? {
                    Some(next) => break (next.part, next.point, None, Some(next.partner)),
                    None => {
                        ray = continued_ray(&point, along);
                        continue;
                    }
                },
                AtHit::Stops => {}
            }
            if !self.glass_contacts().is_empty() {
                match glass_seam(self, &hit, along, true, &mut line, random)? {
                    Some(Seam::Straight { origin }) => {
                        ray = ray_from(origin, along);
                        continue;
                    }
                    Some(Seam::Interface(interface)) => break (part, point, Some(interface), None),
                    Some(Seam::Ended) => return Ok(None),
                    None => {}
                }
            }
            if self.glass_rests().is_empty() {
                break (part, point, None, None);
            }
            let (part, point, partner) = resting_vertex(self, part, point, along)?;
            break (part, point, None, partner);
        };
        let (part, point, partner) = match partner {
            Some(partner) if step_across => {
                match across_contact(self, part, &point, partner, along)? {
                    Some(next) => (next.part, next.point, Some(next.partner)),
                    None => (part, point, Some(partner)),
                }
            }
            _ => (part, point, partner),
        };
        let vertex = ShadowVertex {
            part,
            point: &point,
            seam: seam.as_ref(),
            partner,
        };
        let mode = if stop_at_any {
            LineMode::StopAtAny
        } else {
            LineMode::Straight
        };
        let mut first_face = None;
        let mut work = RenderWork::default();
        let reach = sun_reach_from(
            self,
            &vertex,
            direction,
            mode,
            &mut first_face,
            random,
            &mut work,
        )?;
        Ok(Some(SunLineAt {
            reach: reach.map(|reach| (reach.transmittance, reach.crossed)),
            first_face: first_face.map(|face| (face.outward, face.ior)),
            bent: first_face
                .and_then(|face| face.bent(direction))
                .map(|bent| (bent.inside, bent.cos_out, bent.cos_in)),
        }))
    }
}

/// How far a line toward the sun goes.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub(super) enum LineMode {
    /// Straight through every face of glass.
    Straight,
    /// Only as far as the first surface, glass included: the line reaches the sun only crossing
    /// nothing. Its start rules are a straight line's.
    StopAtAny,
}

fn multiply(a: [f64; 3], b: [f64; 3]) -> [f64; 3] {
    [a[0] * b[0], a[1] * b[1], a[2] * b[2]]
}

fn ray_from(origin: Vector3, direction: Vector3) -> Ray {
    Ray::with_limits(
        origin,
        direction,
        0.0,
        f64::INFINITY,
        IntersectionType::Both,
    )
}

/// The cosine of `direction` to the shading normal at `point`, as the scattering frame reads it.
fn crossing_cosine(point: &SurfacePoint, direction: Vector3) -> f64 {
    point.facing(direction).shading.dot(direction).abs()
}

/// What `surface` at `point` passes of a line along `direction`; `None` stops the line.
fn crossing(
    surface: &ScatteringSurface,
    point: &SurfacePoint,
    direction: Vector3,
) -> Option<[f64; 3]> {
    surface.straight_transmittance(crossing_cosine(point, direction))
}

/// What reaches `vertex` from the sun along `direction`, or `None` when it is blocked: in
/// `LineMode::StopAtAny`, also when the line meets glass, its start's included.
pub(super) fn sun_reach(
    scene: &RenderScene,
    vertex: &ShadowVertex<'_>,
    direction: Vector3,
    mode: LineMode,
    random: &mut PixelRandom,
    work: &mut RenderWork,
) -> Result<Option<SunReach>, RayTracerError> {
    let mut first_face = None;
    sun_reach_from(
        scene,
        vertex,
        direction,
        mode,
        &mut first_face,
        random,
        work,
    )
}

/// [`sun_reach`] in `LineMode::Straight` for a receiver inside glass, with the first face of
/// glass the walk crossed from inside its solid, found by the walk's first trace, whether or not
/// the line then reached the sun. A face the start crossed (a contact) or a seam is not one.
pub(super) fn sun_reach_inside(
    scene: &RenderScene,
    vertex: &ShadowVertex<'_>,
    direction: Vector3,
    random: &mut PixelRandom,
    work: &mut RenderWork,
) -> Result<(Option<SunReach>, Option<FirstFace>), RayTracerError> {
    let mut first_face = None;
    let reach = sun_reach_from(
        scene,
        vertex,
        direction,
        LineMode::Straight,
        &mut first_face,
        random,
        work,
    )?;
    Ok((reach, first_face))
}

fn sun_reach_from(
    scene: &RenderScene,
    vertex: &ShadowVertex<'_>,
    direction: Vector3,
    mode: LineMode,
    first_face: &mut Option<FirstFace>,
    random: &mut PixelRandom,
    work: &mut RenderWork,
) -> Result<Option<SunReach>, RayTracerError> {
    let mut transmittance = [1.0; 3];
    let mut crossed = 0_u32;
    let mut line = StraightLine::default();
    let origin = if let Some(interface) = vertex.seam {
        if let Some(part) = interface.absent_part_toward(vertex.point, direction) {
            line.starts_past(part);
        }
        interface.next_ray_origin(vertex.point, direction)
    } else if let Some(partner) = vertex.partner {
        match across_contact(scene, vertex.part, vertex.point, partner, direction)? {
            None => vertex.point.next_ray_origin(direction),
            Some(next) => {
                // The face across is met as a traced hit on it would be: a part less than fully
                // there is decided first, by this line's own draw, as a fresh line from the
                // vertex decides any see-through part it meets.
                let mut present = true;
                if scene.has_see_through_parts() {
                    match line.at_hit(scene, next.part, random) {
                        AtHit::Stops => {}
                        AtHit::Ended => return Ok(None),
                        AtHit::PassesThrough => present = false,
                    }
                }
                if present {
                    // A surface, which a line stopping at any surface stops at.
                    if mode == LineMode::StopAtAny {
                        return Ok(None);
                    }
                    // Glass passes the line into itself; anything else across (the opaque
                    // surface under glass, a see-through part found there that is not glass)
                    // stops it.
                    let passed = scene
                        .surface_at(next.part, next.point.base_color)
                        .and_then(|surface| crossing(&surface, &next.point, direction));
                    let Some(passed) = passed else {
                        return Ok(None);
                    };
                    transmittance = passed;
                    crossed = 1;
                    // The crossing is a vertex of the line, as in `through_glass`.
                    line.restart();
                    next.point.next_ray_origin(direction)
                } else {
                    // Not there: the line goes on from the vertex's own continuing-ray start, the
                    // part already passed, as a path's bounce across the contact does.
                    vertex.point.next_ray_origin(direction)
                }
            }
        }
    } else {
        vertex.point.next_ray_origin(direction)
    };
    let ray = ray_from(origin, direction);
    if mode == LineMode::StopAtAny || !scene.has_straight_glass() {
        let reached = sun_reachable(scene, ray, line, random, work)?;
        return Ok(reached.then_some(SunReach {
            transmittance,
            crossed,
        }));
    }
    through_glass(
        scene,
        ray,
        line,
        (transmittance, crossed),
        first_face,
        random,
        work,
    )
}

/// Whether the sun is seen along `direction` from `point` the way Exact sees it
/// (`SunThroughGlass::Exact`): the line stops at any surface, glass included.
pub(super) fn exact_reach(
    scene: &RenderScene,
    point: &SurfacePoint,
    direction: Vector3,
    random: &mut PixelRandom,
    work: &mut RenderWork,
) -> Result<bool, RayTracerError> {
    let ray = ray_from(point.next_ray_origin(direction), direction);
    sun_reachable(scene, ray, StraightLine::default(), random, work)
}

/// Whether the sun is seen along `shadow_ray`, for a scene with no glass a shadow ray passes.
/// With no see-through part in the scene the first hit found answers it. With one, the line is
/// followed nearest hit first, deciding at each part as a path would, so the chance the sun is
/// seen is the product of what each part leaves. `line` is what the line went through at its
/// start.
fn sun_reachable(
    scene: &RenderScene,
    mut shadow_ray: Ray,
    mut line: StraightLine,
    random: &mut PixelRandom,
    work: &mut RenderWork,
) -> Result<bool, RayTracerError> {
    if !scene.has_see_through_parts() {
        shadow_ray.is_shadow_ray = true;
        return Ok(scene.parts().trace(&mut shadow_ray).is_none());
    }
    let direction = shadow_ray.direction_normal;
    loop {
        let Some(hit) = scene.parts().trace(&mut shadow_ray) else {
            return Ok(true);
        };
        match line.at_hit(scene, hit.part.index(), random) {
            AtHit::Stops | AtHit::Ended => return Ok(false),
            AtHit::PassesThrough => {
                shadow_ray = continued_ray(&hit.surface_point()?, direction);
                work.pass_through_rays += 1;
            }
        }
    }
}

/// The line walked nearest hit first through glass, in a scene that has glass a shadow ray
/// passes: at each hit the part's presence, then a seam, then a rest, then the glass's crossing,
/// in the order a path decides them. `line` and `carried` are what the start already went through
/// and crossed; `first_face` is set to the first face crossed from inside its solid when the start
/// crossed none.
fn through_glass(
    scene: &RenderScene,
    mut ray: Ray,
    mut line: StraightLine,
    carried: ([f64; 3], u32),
    first_face: &mut Option<FirstFace>,
    random: &mut PixelRandom,
    work: &mut RenderWork,
) -> Result<Option<SunReach>, RayTracerError> {
    let (mut transmittance, mut crossed) = carried;
    let direction = ray.direction_normal;
    let rests = !scene.glass_rests().is_empty();
    loop {
        let Some(hit) = scene.parts().trace(&mut ray) else {
            return Ok(Some(SunReach {
                transmittance,
                crossed,
            }));
        };
        let mut present = true;
        if scene.has_see_through_parts() {
            match line.at_hit(scene, hit.part.index(), random) {
                AtHit::Stops => {}
                AtHit::Ended => return Ok(None),
                AtHit::PassesThrough => present = false,
            }
        }
        let mut seam = None;
        if !scene.glass_contacts().is_empty() {
            match glass_seam(scene, &hit, direction, present, &mut line, random)? {
                Some(Seam::Straight { origin }) => {
                    ray = ray_from(origin, direction);
                    work.pass_through_rays += 1;
                    continue;
                }
                Some(Seam::Interface(interface)) => {
                    seam = Some(interface);
                    present = true;
                }
                Some(Seam::Ended) => return Ok(None),
                None => {}
            }
        }
        let point = hit.surface_point()?;
        if !present {
            ray = continued_ray(&point, direction);
            work.pass_through_rays += 1;
            continue;
        }
        let surface = match &seam {
            Some(interface) => Some(interface.surface),
            None => scene.surface_at(hit.part.index(), point.base_color),
        };
        let Some(surface) = surface else {
            return Ok(None);
        };
        let Some(passed) = crossing(&surface, &point, direction) else {
            return Ok(None);
        };
        if crossed == 0 && seam.is_none() {
            let facing = point.facing(direction);
            if facing.back_face {
                *first_face = Some(FirstFace {
                    outward: -facing.shading,
                    ior: surface.ior(),
                });
            }
        }
        // Only a surface the line would pass is asked whether it rests on another.
        if rests && seam.is_none() {
            let (_, _, partner) = resting_vertex(scene, hit.part.index(), point, direction)?;
            if partner.is_some() {
                return Ok(None);
            }
        }
        if crossed >= MAX_GLASS_CROSSINGS {
            return Ok(None);
        }
        crossed += 1;
        transmittance = multiply(transmittance, passed);
        if transmittance.iter().all(|&channel| channel <= 0.0) {
            return Ok(None);
        }
        // A crossing is a vertex of the line, as a refraction is of a path: the line after it
        // decides partly there parts afresh.
        line.restart();
        let origin = match &seam {
            Some(interface) => {
                if let Some(part) = interface.absent_part_toward(&point, direction) {
                    line.starts_past(part);
                }
                interface.next_ray_origin(&point, direction)
            }
            None => point.next_ray_origin(direction),
        };
        ray = ray_from(origin, direction);
        work.sun_line_rays += 1;
    }
}
