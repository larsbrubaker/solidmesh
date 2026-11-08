//! The Ray Tracing page's shader: what one camera ray brings back for one pixel.
//!
//! - **Shaded** is one sun with a shadow ray, ambient light gathered by a single
//!   cosine-weighted hemisphere ray per sample (so ambient occlusion converges with the samples),
//!   and a reflectivity that adds a slightly glossy mirror bounce on the part.
//! - **Normals** and **Depth** are the hit data itself, which is what makes it visible that these
//!   are real intersections with the mesh and not a picture of it.
//!
//! Every random choice comes from an `shapes::Lcg` seeded by (seed, sample, pixel), so a seed and
//! a sample index always give the same image, however the work is sliced into passes. Sample 0
//! goes through the pixel centre; later samples jitter across the pixel, which is the
//! anti-aliasing.
//!
//! A ray leaving a surface starts offset along the normal, by a small fraction of the part's
//! size: the triangle hit test computes in single precision, so a smaller offset would let a ray
//! find the triangle it just left.

use std::f64::consts::TAU;

use solidmesh::math::{IntersectionType, Ray, Vector3};

use crate::ray_trace_scene::{Camera, TraceScene};
use crate::shapes::Lcg;

/// What a pixel shows, by the index the page's menu sends.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub enum Mode {
    Shaded,
    Normals,
    Depth,
}

impl Mode {
    /// The mode a menu index names, or an error naming the index.
    pub fn from_index(index: u32) -> Result<Mode, String> {
        Ok(match index {
            0 => Mode::Shaded,
            1 => Mode::Normals,
            2 => Mode::Depth,
            _ => return Err(format!("There is no view {index} on this page.")),
        })
    }
}

/// The offset of a ray leaving a surface, as a fraction of the part's radius.
const SURFACE_OFFSET_RADII: f64 = 0.0015;

/// How far an ambient-occlusion ray looks, in part radii: nearby geometry darkens a surface, the
/// far side of the scene does not.
const AO_RANGE_RADII: f64 = 1.5;

/// The bounces a reflection follows. Bounded, so the worst pixel's cost is bounded.
const MAX_BOUNCES: u32 = 3;

/// How far a glossy reflection strays from the mirror direction.
const GLOSS: f64 = 0.03;

const SUN_LIGHT: [f64; 3] = [1.05, 1.0, 0.92];
const SKY_LIGHT: [f64; 3] = [0.42, 0.46, 0.55];
const PART_ALBEDO: [f64; 3] = [0.16, 0.56, 0.50];
const GROUND_ALBEDO: [f64; 3] = [0.74, 0.72, 0.68];
const GROUND_ALBEDO_DARK: [f64; 3] = [0.64, 0.62, 0.59];
const SKY_ZENITH: [f64; 3] = [0.36, 0.55, 0.84];
const SKY_HORIZON: [f64; 3] = [0.82, 0.87, 0.93];
const BACKGROUND_DATA: [f64; 3] = [0.08, 0.09, 0.11];

/// How bright the sun's disc is in a reflection.
const SUN_DISC: f64 = 7.0;
const SUN_DISC_COS: f64 = 0.9994;

/// The direction toward the sun.
pub fn sun_direction() -> Vector3 {
    Vector3::new(-0.42, -0.55, 0.82).get_normal()
}

fn scale(c: [f64; 3], s: f64) -> [f64; 3] {
    [c[0] * s, c[1] * s, c[2] * s]
}

fn add(a: [f64; 3], b: [f64; 3]) -> [f64; 3] {
    [a[0] + b[0], a[1] + b[1], a[2] + b[2]]
}

fn multiply(a: [f64; 3], b: [f64; 3]) -> [f64; 3] {
    [a[0] * b[0], a[1] * b[1], a[2] * b[2]]
}

fn mix(a: [f64; 3], b: [f64; 3], t: f64) -> [f64; 3] {
    add(scale(a, 1.0 - t), scale(b, t))
}

/// A generator for one pixel's one sample, seeded from all four numbers so neighbouring pixels
/// and samples are uncorrelated.
pub fn sample_rng(seed: u32, sample: u32, x: u32, y: u32) -> Lcg {
    let mut hash: u32 = 2_166_136_261;
    for value in [seed, sample, x, y] {
        hash = (hash ^ value).wrapping_mul(16_777_619);
        hash ^= hash >> 15;
    }
    let mut rng = Lcg::new(hash);
    // The LCG's first outputs still echo a sequential state; two steps wash that out.
    rng.next_f64();
    rng.next_f64();
    rng
}

/// A unit vector perpendicular to `n` and the one making them a basis, for hemisphere sampling.
fn basis(n: Vector3) -> (Vector3, Vector3) {
    let helper = if n.x.abs() > 0.9 {
        Vector3::UNIT_Y
    } else {
        Vector3::UNIT_X
    };
    let tangent = n.cross(helper).get_normal();
    (tangent, n.cross(tangent))
}

/// A cosine-weighted direction in the hemisphere about `n`.
fn cosine_hemisphere(n: Vector3, rng: &mut Lcg) -> Vector3 {
    let u1 = rng.next_f64();
    let u2 = rng.next_f64();
    let r = u1.sqrt();
    let phi = TAU * u2;
    let (t, b) = basis(n);
    (t * (r * phi.cos()) + b * (r * phi.sin()) + n * (1.0 - u1).sqrt()).get_normal()
}

/// The sky along `direction`. The sun's disc shows only in reflections, so a camera looking
/// toward it does not get a white hole.
fn sky(direction: Vector3, in_reflection: bool, sun: Vector3) -> [f64; 3] {
    let t = direction.z.max(0.0).sqrt();
    let color = mix(SKY_HORIZON, SKY_ZENITH, t);
    if in_reflection && direction.dot(sun) > SUN_DISC_COS {
        return add(color, scale(SUN_LIGHT, SUN_DISC));
    }
    color
}

/// The ground's color at `position`: a checker a third of the part wide, so reflections, motion
/// and the size of the shadow are easy to read, fading into the horizon.
fn ground_albedo(position: Vector3, radius: f64) -> [f64; 3] {
    let square = radius * 0.65;
    let even =
        ((position.x / square).floor() as i64 + (position.y / square).floor() as i64) % 2 == 0;
    if even {
        GROUND_ALBEDO
    } else {
        GROUND_ALBEDO_DARK
    }
}

/// What one camera ray and its bounces bring back.
struct Tracer<'a> {
    scene: &'a TraceScene,
    sun: Vector3,
    offset: f64,
    ao_range: f64,
    reflectivity: f64,
}

impl Tracer<'_> {
    /// The light arriving back along a ray from `origin` going `direction`.
    fn radiance(
        &self,
        origin: Vector3,
        direction: Vector3,
        depth: u32,
        rng: &mut Lcg,
        rays: &mut u64,
    ) -> [f64; 3] {
        let mut ray = Ray::new(origin, direction);
        let hit = self.scene.trace(&mut ray, rays);
        if hit.hit_type == IntersectionType::None {
            return sky(direction, depth > 0, self.sun);
        }
        let n = hit.normal_at_hit;
        let is_ground = self.scene.is_ground(&hit);
        let albedo = if is_ground {
            ground_albedo(hit.hit_position, self.scene.radius)
        } else {
            PART_ALBEDO
        };
        let at = hit.hit_position + n * self.offset;

        let mut light = [0.0; 3];
        let facing_sun = n.dot(self.sun);
        if facing_sun > 0.0 && !self.in_shadow(at, rays) {
            light = scale(SUN_LIGHT, facing_sun);
        }
        // A cosine-weighted ray that escapes counts as 1: the cosine is in where the rays go.
        let mut ao = Ray::new(at, cosine_hemisphere(n, rng));
        ao.max_distance_to_consider = self.ao_range;
        if !self.scene.occluded(&mut ao, rays) {
            light = add(light, SKY_LIGHT);
        }

        let diffuse = multiply(albedo, light);
        let color = if is_ground || self.reflectivity <= 0.0 || depth >= MAX_BOUNCES {
            diffuse
        } else {
            let mirror = direction - n * (2.0 * direction.dot(n));
            let reflected = self.radiance(at, self.glossy(mirror, n, rng), depth + 1, rng, rays);
            mix(diffuse, reflected, self.reflectivity)
        };
        if is_ground {
            self.fade_to_horizon(color, hit.distance_to_hit)
        } else {
            color
        }
    }

    fn in_shadow(&self, at: Vector3, rays: &mut u64) -> bool {
        let mut shadow = Ray::new(at, self.sun);
        self.scene.occluded(&mut shadow, rays)
    }

    /// `mirror` strayed a little at random, kept above the surface.
    fn glossy(&self, mirror: Vector3, n: Vector3, rng: &mut Lcg) -> Vector3 {
        let stray = Vector3::new(
            rng.next_f64() - 0.5,
            rng.next_f64() - 0.5,
            rng.next_f64() - 0.5,
        ) * (GLOSS * 2.0);
        let strayed = (mirror + stray).get_normal();
        if strayed.dot(n) > 0.0 {
            strayed
        } else {
            mirror
        }
    }

    /// The ground blended into the sky far away, so the horizon is not a hard line.
    fn fade_to_horizon(&self, color: [f64; 3], distance: f64) -> [f64; 3] {
        let fog = 1.0 - (-distance / (self.scene.radius * 14.0)).exp();
        mix(color, SKY_HORIZON, fog)
    }
}

/// The depth readout: 1 at the near end of the view's depth range, 0 at the far end and beyond.
pub fn depth_value(distance: f64, near: f64, far: f64) -> f64 {
    1.0 - ((distance - near) / (far - near)).clamp(0.0, 1.0)
}

/// The depth range the Depth mode spreads across the gray scale, for a part `radius` in size seen
/// from `camera_distance` away.
pub fn depth_range(camera_distance: f64, radius: f64) -> (f64, f64) {
    (
        (camera_distance - radius * 1.3).max(0.0),
        camera_distance + radius * 1.6,
    )
}

/// The linear color of pixel (`x`, `y`) for sample number `sample`, adding the rays it fired to
/// `rays`.
#[allow(clippy::too_many_arguments)]
pub fn pixel_color(
    scene: &TraceScene,
    camera: &Camera,
    mode: Mode,
    reflectivity: f64,
    seed: u32,
    sample: u32,
    x: u32,
    y: u32,
    rays: &mut u64,
) -> [f64; 3] {
    let mut rng = sample_rng(seed, sample, x, y);
    let (jitter_x, jitter_y) = if sample == 0 {
        (0.5, 0.5)
    } else {
        (rng.next_f64(), rng.next_f64())
    };
    let direction = camera.ray_direction(f64::from(x) + jitter_x, f64::from(y) + jitter_y);
    match mode {
        Mode::Shaded => {
            let tracer = Tracer {
                scene,
                sun: sun_direction(),
                offset: scene.radius * SURFACE_OFFSET_RADII,
                ao_range: scene.radius * AO_RANGE_RADII,
                reflectivity,
            };
            tracer.radiance(camera.eye, direction, 0, &mut rng, rays)
        }
        Mode::Normals | Mode::Depth => {
            let mut ray = Ray::new(camera.eye, direction);
            let hit = scene.trace(&mut ray, rays);
            if hit.hit_type == IntersectionType::None {
                return BACKGROUND_DATA;
            }
            if mode == Mode::Normals {
                let n = hit.normal_at_hit;
                return [(n.x + 1.0) * 0.5, (n.y + 1.0) * 0.5, (n.z + 1.0) * 0.5];
            }
            let distance = (scene.center - camera.eye).length();
            let (near, far) = depth_range(distance, scene.radius);
            let v = depth_value(hit.distance_to_hit, near, far);
            [v, v, v]
        }
    }
}
