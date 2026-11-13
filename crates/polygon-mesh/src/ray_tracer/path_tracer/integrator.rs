//! The path loop: one camera ray followed through the scene, gathering sun and environment light
//! at each surface and multiplying a running throughput by what each surface passes on. The
//! header of `path_tracer` says what the estimator is; this file is how a bounce is carried out.
//!
//! **Which way is up at a surface.** The scattering model wants the shading normal as +z of its
//! frame. An opaque surface is one sided, so its frame uses the normals turned toward the ray
//! (`facing_normals`), which is also what keeps a bent interpolated normal from leaving the ray
//! below the surface. A surface that can transmit needs the true outside as +z, since that is how
//! the model tells a ray entering the solid (index of refraction 1 to `ior`) from one leaving it
//! (`ior` to 1): the facing normals get flipped back for a ray that hit the inside of the solid.
//!
//! **Light leaks at bent normals.** A direction can be fine against the smooth shading normal and
//! wrong against the actual triangle: a reflection dipping below a face, or a refraction that
//! comes out on the side it came in from. Such a direction is thrown away, both for the sun
//! (no contribution) and for a sampled bounce (the path ends): the shadow ray and continuing ray
//! would otherwise start on the wrong side of the wall from where the surface said they should.
//! The energy lost scales with how far smooth normals stray from their faces: below 0.2% on a
//! 160 by 80 sphere, but in a white furnace with smooth normals a 12 by 6 sphere loses 0.59%
//! (Lambert), 4.36% (metal at roughness 0.1) and 3.22% (glass), and an 8 by 4 sphere 1.35%, 9.11%
//! and 6.10%. A coarse mesh with smooth normals is visibly darker than a fine one.
//!
//! **See-through parts.** At a hit on a part that is only partly there, `see_through` decides
//! whether the path finds it. A path that doesn't goes on along the same line with nothing else
//! changed: the throughput, the bounce count, the direction and the record of how it got here.
//! Weighting the sun's two estimators stays right because of that last one. The direction was
//! chosen at the last real vertex, and whether a shadow ray along it reaches the sun past
//! see-through parts is the same question, with the same chance, as whether a sampled ray along
//! it does. Both estimators are scaled by the same visibility, so the weights stay the ratio of
//! the two densities at the last vertex, and a shadow ray walks its line by the same rule.
//!
//! **Glass lying on glass.** `glass_seam` says what a hit on a glass face covered by another
//! glass part's face is; the path tracer's header says what light does there.
//!
//! **Glass resting on an opaque surface.** `glass_rest` says which vertex a hit there is and when
//! a bounce goes straight across the contact to the next vertex, which the loop then takes
//! without tracing. A scene with no glass resting on anything never asks.
//!
//! **A see-through part resting on an opaque surface.** `see_through_rest` says which of the two
//! faces a traced hit there is met as, and where a line passing the part lands; a vertex reached
//! across a contact is met as a traced hit would be, its part's presence decided first, so the
//! line decides a see-through part across the contact exactly as it would at a traced face.
//!
//! **The sun through glass.** [`PathTraceSettings::sun_through_glass`] says how sunlight reaches
//! what lies behind glass. `glass_chain` keeps the record of how the path got to its ray, which
//! weighs the sun's disc on escape and a vertex's glass share of the sun, and says why each light
//! path is counted once; `sun_toward` shares a vertex's BSDF toward a sun direction between the
//! other lobes and the glass lobe; `sun_through_glass` walks a vertex's one line toward the sun.

use solidmesh_math::{IntersectionType, Ray, Vector3};

use super::glass_chain::{power_heuristic, Event, SunPath, SunThroughGlass};
use super::glass_rest::{across_contact, resting_vertex, Across, Partner};
use super::glass_seam::{glass_seam, Interface, Seam};
use super::lights::Sun;
use super::pixel_random::PixelRandom;
use super::render_scene::RenderScene;
use super::render_work::RenderWork;
use super::see_through::{continued_ray, AtHit, StraightLine};
use super::see_through_rest::{met_first, on_rest_contact, onto_rest};
use super::sun_through_glass::{exact_reach, sun_reach, sun_reach_inside, ShadowVertex};
use super::sun_toward::{bent_sun, in_glass_after, Toward};
use crate::ray_tracer::{LocalFrame, RayTracerError, ScatteringSurface, SurfacePoint};

/// Which of the ways of reaching the sun a render uses. [`LightStrategy::Combined`] is the one to
/// render with; the other two exist so the three can be checked against each other. Under
/// [`SunThroughGlass::Straight`] the split between the straight line through glass and exact
/// transport through it is the model's, which no strategy changes (`glass_chain`): a shadow ray
/// whose line crosses glass counts its share under all three.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub enum LightStrategy {
    /// Next-event estimation and surface sampling, each weighted by the other's density.
    #[default]
    Combined,
    /// Only next-event estimation reaches the sun (a delta lobe still reaches it by sampling, the
    /// only way it can).
    NextEventOnly,
    /// Only surface sampling reaches the sun along a line that crosses nothing; under
    /// [`SunThroughGlass::Straight`] a shadow ray still brings its share through glass.
    SurfaceSamplingOnly,
}

/// The integrator's settings.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct PathTraceSettings {
    /// The seed every random number derives from.
    pub seed: u64,
    /// The most scattering events a path may have; any energy still carried then is lost.
    pub max_bounces: u32,
    /// After this many bounces a path may be ended at random in proportion to how little it still
    /// carries, unbiased by reweighting the survivors; `None` turns this off.
    pub russian_roulette_after: Option<u32>,
    /// How paths reach the sun.
    pub strategy: LightStrategy,
    /// How sunlight reaches what lies behind glass. The GPU route reads it too
    /// (`SCENE_FLAG_SUN_STRAIGHT`): its shader mirrors both settings in `f32`.
    pub sun_through_glass: SunThroughGlass,
}

impl Default for PathTraceSettings {
    fn default() -> Self {
        PathTraceSettings {
            seed: 0,
            max_bounces: 32,
            russian_roulette_after: Some(3),
            strategy: LightStrategy::Combined,
            sun_through_glass: SunThroughGlass::Straight,
        }
    }
}

/// The highest chance a path survives roulette, so that even a fully white path is eventually
/// ended rather than running to the bounce limit.
const MAX_SURVIVAL: f64 = 0.95;

fn multiply(a: [f64; 3], b: [f64; 3]) -> [f64; 3] {
    [a[0] * b[0], a[1] * b[1], a[2] * b[2]]
}

fn scaled(a: [f64; 3], s: f64) -> [f64; 3] {
    [a[0] * s, a[1] * s, a[2] * s]
}

fn add_to(sum: &mut [f64; 3], add: [f64; 3]) {
    sum[0] += add[0];
    sum[1] += add[1];
    sum[2] += add[2];
}

fn largest_channel(c: [f64; 3]) -> f64 {
    c[0].max(c[1]).max(c[2])
}

/// The radiance a ray that left the scene brings back along `direction`.
fn escaped_radiance(
    scene: &RenderScene,
    settings: &PathTraceSettings,
    direction: Vector3,
    path: SunPath,
) -> [f64; 3] {
    let lights = scene.lights();
    let mut radiance = lights.environment.radiance(direction);
    if let Some(sun) = &lights.sun {
        if sun.contains(direction) {
            let weight = path.sun_weight(settings.strategy, sun.pdf());
            add_to(&mut radiance, scaled(sun.radiance(), weight));
        }
    }
    radiance
}

/// True when the shading frame and the triangle agree on which side `wi_world` leaves from
/// relative to `wo_world`: the local frame's z values say what the frame thinks, the geometric
/// normal says what the triangle does.
fn sides_agree(
    point: &SurfacePoint,
    wo_world: Vector3,
    wi_world: Vector3,
    local_wo_z: f64,
    local_wi_z: f64,
) -> bool {
    let local_same_side = local_wo_z * local_wi_z > 0.0;
    let geometric_same_side =
        wo_world.dot(point.geometric_normal) * wi_world.dot(point.geometric_normal) > 0.0;
    local_same_side == geometric_same_side
}

/// What one vertex needs to shade the sun: its surface and frame, and the direction back along
/// the path.
struct Vertex<'a> {
    part: usize,
    point: &'a SurfacePoint,
    seam: Option<&'a Interface>,
    partner: Option<Partner>,
    surface: &'a ScatteringSurface,
    frame: &'a LocalFrame,
    toward_camera: Vector3,
    wo: Vector3,
    is_first_hit: bool,
    /// Whether the ray that arrived here travels inside glass.
    in_glass: bool,
    /// How the path got here.
    path: SunPath,
}

impl Vertex<'_> {
    /// Where a line toward the sun leaves this vertex from.
    fn shadow(&self) -> ShadowVertex<'_> {
        ShadowVertex {
            part: self.part,
            point: self.point,
            seam: self.seam,
            partner: self.partner,
        }
    }
}

/// Traces a shadow ray toward the sun and returns what it brings, when the strategy uses
/// next-event estimation or, with `straight`, the line crosses glass (`glass_chain`). The
/// surface is evaluated first so a direction it passes nothing along costs no ray. `straight` is
/// whether the render mixes the straight line in for this scene's glass. `receiver_sun_pdf`, the
/// sun's density the vertex's own `w_b` is taken against, is set where it is not the sun's own
/// (a receiver inside glass).
fn sample_sun(
    scene: &RenderScene,
    settings: &PathTraceSettings,
    straight: bool,
    vertex: &Vertex<'_>,
    receiver_sun_pdf: &mut f64,
    random: &mut PixelRandom,
    work: &mut RenderWork,
) -> Result<Option<[f64; 3]>, RayTracerError> {
    let Some(sun) = scene.lights().sun.as_ref() else {
        return Ok(None);
    };
    // Under Straight surface sampling still walks the line, whose share through glass is the
    // model's, not the strategy's.
    if settings.strategy == LightStrategy::SurfaceSamplingOnly && !straight {
        return Ok(None);
    }
    if straight && vertex.in_glass && vertex.seam.is_none() && vertex.partner.is_none() {
        return sample_sun_inside(scene, settings, sun, vertex, receiver_sun_pdf, random, work);
    }
    let wi_world = sun.sample([random.next_unit(), random.next_unit()]);
    let wi = vertex.frame.to_local(wi_world);
    if !sides_agree(
        vertex.point,
        vertex.toward_camera,
        wi_world,
        vertex.wo.z,
        wi.z,
    ) {
        return Ok(None);
    }
    let bsdf = vertex.surface.evaluate(vertex.wo, wi);
    if largest_channel(bsdf) <= 0.0 {
        return Ok(None);
    }
    let pdf_light = sun.pdf();
    if !straight {
        if vertex.is_first_hit {
            work.first_hit_shadow_rays += 1;
        } else {
            work.later_shadow_rays += 1;
        }
        if !exact_reach(scene, vertex.point, wi_world, random, work)? {
            return Ok(None);
        }
        let weight = if settings.strategy == LightStrategy::Combined {
            power_heuristic(pdf_light, vertex.surface.pdf(vertex.wo, wi))
        } else {
            1.0
        };
        return Ok(Some(scaled(
            multiply(bsdf, sun.radiance()),
            wi.z.abs() / pdf_light * weight,
        )));
    }
    let toward = Toward::of(
        settings.strategy,
        vertex.wo.z,
        wi.z,
        bsdf,
        pdf_light,
        sun.radiance(),
        lobe_densities(vertex, wi),
        vertex.path.chain(),
    );
    let Some(mode) = toward.mode() else {
        return Ok(None);
    };
    count_shadow_ray(vertex, work);
    let Some(reach) = sun_reach(scene, &vertex.shadow(), wi_world, mode, random, work)? else {
        return Ok(None);
    };
    Ok(toward.light(settings.strategy, &reach))
}

/// `sample_sun` under Straight for a receiver whose arriving ray travels inside glass
/// (`glass_chain`, "A receiver inside glass"): the line is walked first, and where its first
/// face of glass is crossed from inside, the receiver reads the sun along the direction that
/// refracts out through that face along the line, over the disc as seen inside, `n^2 cos_in /
/// cos_out` times denser and `n^2` times brighter. Returns what the line brings, and sets
/// `receiver_sun_pdf` to the sun's density the receiver's own `w_b` is taken against.
fn sample_sun_inside(
    scene: &RenderScene,
    settings: &PathTraceSettings,
    sun: &Sun,
    vertex: &Vertex<'_>,
    receiver_sun_pdf: &mut f64,
    random: &mut PixelRandom,
    work: &mut RenderWork,
) -> Result<Option<[f64; 3]>, RayTracerError> {
    let line = sun.sample([random.next_unit(), random.next_unit()]);
    let line_local = vertex.frame.to_local(line);
    if !sides_agree(
        vertex.point,
        vertex.toward_camera,
        line,
        vertex.wo.z,
        line_local.z,
    ) {
        return Ok(None);
    }
    // A surface that passes nothing along the line (only delta lobes, or the line below an
    // opaque surface) costs no walk, as on any other vertex; a lobe narrow enough to miss the
    // line but catch the bent direction still has a tail along the line.
    if largest_channel(vertex.surface.evaluate(vertex.wo, line_local)) <= 0.0 {
        return Ok(None);
    }
    count_shadow_ray(vertex, work);
    let (reach, first_face) = sun_reach_inside(scene, &vertex.shadow(), line, random, work)?;
    let bent = first_face.and_then(|face| face.bent(line).map(|bent| (face, bent)));
    let (wi_world, pdf_light, radiance) = match bent {
        Some((face, bent)) => {
            let (pdf, radiance) = bent_sun(
                sun.pdf(),
                sun.radiance(),
                face.ior,
                bent.cos_in,
                bent.cos_out,
            );
            (bent.inside, pdf, radiance)
        }
        None => (line, sun.pdf(), sun.radiance()),
    };
    *receiver_sun_pdf = pdf_light;
    let Some(reach) = reach else {
        return Ok(None);
    };
    let wi = vertex.frame.to_local(wi_world);
    if bent.is_some()
        && !sides_agree(
            vertex.point,
            vertex.toward_camera,
            wi_world,
            vertex.wo.z,
            wi.z,
        )
    {
        return Ok(None);
    }
    let bsdf = vertex.surface.evaluate(vertex.wo, wi);
    if largest_channel(bsdf) <= 0.0 {
        return Ok(None);
    }
    let toward = Toward::of(
        settings.strategy,
        vertex.wo.z,
        wi.z,
        bsdf,
        pdf_light,
        radiance,
        lobe_densities(vertex, wi),
        vertex.path.chain(),
    );
    Ok(toward.light(settings.strategy, &reach))
}

/// The vertex's surface's density toward local `wi` in its two cleaned parts, other lobes and
/// glass lobe (`glass_chain`).
fn lobe_densities(vertex: &Vertex<'_>, wi: Vector3) -> (f64, f64) {
    let densities = vertex.surface.lobe_densities(vertex.wo, wi);
    (densities.other, densities.glass)
}

/// Counts one shadow ray of `vertex`.
fn count_shadow_ray(vertex: &Vertex<'_>, work: &mut RenderWork) {
    if vertex.is_first_hit {
        work.first_hit_shadow_rays += 1;
    } else {
        work.later_shadow_rays += 1;
    }
}

/// The radiance brought back along `ray` (a camera ray), by one path. Random numbers come only
/// from `random`, in the order the path uses them.
pub(super) fn trace_path(
    scene: &RenderScene,
    settings: &PathTraceSettings,
    mut ray: Ray,
    random: &mut PixelRandom,
    work: &mut RenderWork,
) -> Result<[f64; 3], RayTracerError> {
    let mut radiance = [0.0; 3];
    let mut throughput = [1.0; 3];
    // The radiance scaling of refraction, kept apart so roulette can read how much energy the
    // path really carries: an ordinary path inside glass has its throughput divided by `ior^2`
    // only until it leaves, and roulette must not kill it for that.
    let mut eta_scale = 1.0;
    let mut path = SunPath::camera();
    // The camera is taken to be in air.
    let mut in_glass = false;
    let mut scatters = 0_u32;
    let mut line = StraightLine::default();
    // Glass resting on an opaque surface is decided only in a scene that has some, so every other
    // scene runs exactly the operations it always did; `glass_rest` keeps a contact with a partly
    // there part out of it.
    let rests = !scene.glass_rests().is_empty();
    // The vertex a bounce across a contact reached, taken next without a trace.
    let mut across: Option<Across> = None;
    // Whether the sun goes straight through this scene's glass (`glass_chain`); a scene with no
    // glass takes Exact's path, which is the plain one.
    let straight =
        settings.sun_through_glass == SunThroughGlass::Straight && scene.has_straight_glass();
    let sun_pdf = scene.lights().sun.as_ref().map_or(0.0, |sun| sun.pdf());

    work.camera_rays += 1;
    loop {
        let (part_index, point, seam, partner) = if let Some(next) = across.take() {
            // The part across the contact is met as a traced hit on it would be; a fully there
            // part asks nothing.
            if scene.has_see_through_parts() {
                match line.at_hit(scene, next.part, random) {
                    AtHit::Stops => {}
                    AtHit::Ended => break,
                    AtHit::PassesThrough => {
                        // `ray` starts where the line goes on, past both faces of the contact.
                        work.pass_through_rays += 1;
                        continue;
                    }
                }
            }
            if scatters >= settings.max_bounces {
                break;
            }
            (next.part, next.point, None, Some(next.partner))
        } else {
            let Some(hit) = scene.parts().trace(&mut ray) else {
                let escaped = escaped_radiance(scene, settings, ray.direction_normal, path);
                add_to(&mut radiance, multiply(throughput, escaped));
                break;
            };
            // Where a see-through part rests on an opaque surface, which of the two faces the
            // line meets first is the contact's to say, not the trace's; every other scene skips
            // this and runs exactly the operations it always did.
            let mut met = None;
            // On either face of such a contact, whichever the trace returned.
            let mut at_rest = false;
            if !scene.see_through_rests().is_empty() {
                let traced = hit.surface_point()?;
                at_rest = on_rest_contact(scene, hit.part.index(), &traced);
                met = met_first(scene, hit.part.index(), &traced, &ray)?;
            }
            let hit_part = met.as_ref().map_or(hit.part.index(), |(part, _)| *part);
            // Whether the part is there at all comes before the bounce limit: a path past its last
            // bounce still sees through what isn't there.
            let mut present = true;
            if scene.has_see_through_parts() {
                match line.at_hit(scene, hit_part, random) {
                    AtHit::Stops => {}
                    AtHit::Ended => break,
                    AtHit::PassesThrough => present = false,
                }
            }
            // A seam between two glass parts is looked at only in a scene that has one, so every
            // other scene runs exactly the operations it always did, and never on a face of a
            // see-through part's resting contact, so the tie outcome doesn't decide it.
            let mut seam = None;
            if !at_rest && !scene.glass_contacts().is_empty() {
                match glass_seam(
                    scene,
                    &hit,
                    ray.direction_normal,
                    present,
                    &mut line,
                    random,
                )? {
                    Some(Seam::Straight { origin }) => {
                        ray = Ray::with_limits(
                            origin,
                            ray.direction_normal,
                            0.0,
                            f64::INFINITY,
                            IntersectionType::Both,
                        );
                        work.pass_through_rays += 1;
                        continue;
                    }
                    Some(Seam::Interface(interface)) => {
                        seam = Some(interface);
                        present = true;
                    }
                    Some(Seam::Ended) => break,
                    None => {}
                }
            }
            if !present {
                let point = match met {
                    Some((_, point)) => point,
                    None => hit.surface_point()?,
                };
                // A line leaving the part onto an opaque surface it rests on lands there.
                if let Some(next) = onto_rest(scene, hit_part, &point, ray.direction_normal)? {
                    across = Some(next);
                    continue;
                }
                ray = continued_ray(&point, ray.direction_normal);
                work.pass_through_rays += 1;
                continue;
            }
            if scatters >= settings.max_bounces {
                break;
            }
            let point = match met {
                Some((_, point)) => point,
                None => hit.surface_point()?,
            };
            if rests && seam.is_none() {
                let (part, point, partner) =
                    resting_vertex(scene, hit_part, point, ray.direction_normal)?;
                (part, point, seam, partner)
            } else {
                (hit_part, point, seam, None)
            }
        };

        let surface = match &seam {
            Some(interface) => interface.surface,
            None => match scene.surface_at(part_index, point.base_color) {
                Some(surface) => surface,
                None => break,
            },
        };
        let toward_camera = -ray.direction_normal;
        let facing = point.facing(ray.direction_normal);
        let transmissive = surface.transmission() > 0.0 && surface.metallic() < 1.0;
        // See the header on why a transmissive surface has its true outside as +z; at a seam the
        // denser glass's side is the inside.
        let frame_normal = match &seam {
            Some(interface) if interface.into_denser => facing.shading,
            Some(_) => -facing.shading,
            None => {
                if transmissive && facing.back_face {
                    -facing.shading
                } else {
                    facing.shading
                }
            }
        };
        let Some(frame) = LocalFrame::from_normal(frame_normal) else {
            break;
        };
        let wo = frame.to_local(toward_camera);

        let vertex = Vertex {
            part: part_index,
            point: &point,
            seam: seam.as_ref(),
            partner,
            surface: &surface,
            frame: &frame,
            toward_camera,
            wo,
            is_first_hit: scatters == 0,
            in_glass,
            path,
        };
        // The sun's density this vertex's `w_b` is taken against (`glass_chain`).
        let mut receiver_sun_pdf = sun_pdf;
        if let Some(direct) = sample_sun(
            scene,
            settings,
            straight,
            &vertex,
            &mut receiver_sun_pdf,
            random,
            work,
        )? {
            add_to(&mut radiance, multiply(throughput, direct));
        }

        let Some((sample, draw)) = surface.sample_by_lobe(wo, random.next_unit3()) else {
            break;
        };
        let wi_world = frame.to_world(sample.direction).get_normal();
        if !sides_agree(&point, toward_camera, wi_world, wo.z, sample.direction.z) {
            break;
        }
        let event = Event::of_draw(
            straight,
            sample.is_specular,
            draw.from_glass,
            sample.pdf,
            (draw.densities.other, draw.densities.glass),
        );
        throughput = multiply(throughput, sample.weight);
        if wo.z * sample.direction.z < 0.0 {
            let ior_squared = surface.ior() * surface.ior();
            if wo.z > 0.0 {
                eta_scale *= ior_squared;
            } else {
                eta_scale /= ior_squared;
            }
        }
        scatters += 1;
        // Whether the ray leaving this event travels inside glass (`glass_chain`).
        in_glass = in_glass_after(in_glass, seam.is_some(), transmissive, sample.direction.z);
        path = path.after(event, straight, receiver_sun_pdf, in_glass);

        if let Some(after) = settings.russian_roulette_after {
            if scatters >= after {
                let survival = (largest_channel(throughput) * eta_scale).min(MAX_SURVIVAL);
                if survival <= 0.0 || random.next_unit() >= survival {
                    break;
                }
                throughput = scaled(throughput, 1.0 / survival);
            }
        }

        if let Some(partner) = partner {
            if let Some(next) = across_contact(scene, part_index, &point, partner, wi_world)? {
                // The next vertex is at this same point; the ray says which way the path arrives
                // there, and starts where the line goes on should a see-through part across the
                // contact not be there.
                ray = Ray::with_limits(
                    point.next_ray_origin(wi_world),
                    wi_world,
                    0.0,
                    f64::INFINITY,
                    IntersectionType::Both,
                );
                line.restart();
                across = Some(next);
                continue;
            }
        }
        let origin = match &seam {
            Some(interface) => interface.next_ray_origin(&point, wi_world),
            None => point.next_ray_origin(wi_world),
        };
        ray = Ray::with_limits(origin, wi_world, 0.0, f64::INFINITY, IntersectionType::Both);
        work.bounce_rays += 1;
        line.restart();
        if let Some(part) = seam
            .as_ref()
            .and_then(|interface| interface.absent_part_toward(&point, wi_world))
        {
            line.starts_past(part);
        }
    }
    work.record_path(scatters);
    Ok(radiance)
}
