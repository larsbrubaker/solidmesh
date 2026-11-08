//! The Ray Tracing page's looks and data views: what the page's Material and Light menus mean in
//! the library's terms, and what the Normals and Depth views read off a hit.
//!
//! A [`Look`] is the part's material and color and the light around it. It turns into the
//! library's [`ScatteringSurface`] (the named presets, or the page's Custom sliders) and
//! [`LightSetup`] (a sky gradient plus a soft sun, or an overcast sky with no sun). The picture
//! itself comes from the library's path tracer, run in time-boxed passes by `ray_trace`. Changing
//! a look changes only these two descriptions, so the mesh and its tree are never rebuilt.
//!
//! The sun's brightness is set by the light it delivers (its irradiance), not by the radiance of
//! its disc, so the Sun size slider softens the shadows and leaves the scene equally bright.
//!
//! **Normals** and **Depth** are the hit data itself, which is what makes it visible that these
//! are real intersections with the mesh and not a picture of it. Normals shows the smooth shading
//! normal the path tracer uses, read from the same surface table. Their sample 0 goes through the
//! pixel centre; later samples jitter across the pixel, which is the anti-aliasing.

use solidmesh::math::Vector3;
use solidmesh::mesh::ray_tracer::{
    pixel_jitter, Environment, LightSetup, RayTracerError, ScatteringSurface, Sun,
};

use crate::ray_trace_scene::{Camera, TraceScene};

/// What a pixel shows, by the index the page's menu sends.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub enum Mode {
    /// The library's physically based path tracer.
    PathTraced,
    Normals,
    Depth,
}

impl Mode {
    /// The mode a menu index names, or an error naming the index.
    pub fn from_index(index: u32) -> Result<Mode, String> {
        Ok(match index {
            0 => Mode::PathTraced,
            1 => Mode::Normals,
            2 => Mode::Depth,
            _ => return Err(format!("There is no view {index} on this page.")),
        })
    }
}

/// The page's Material menu, by the index it sends.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub enum Material {
    MattePlastic,
    GlossyPlastic,
    BrushedMetal,
    PolishedMetal,
    Glass,
    Rubber,
    Custom,
}

impl Material {
    /// The material a menu index names, or an error naming the index.
    pub fn from_index(index: u32) -> Result<Material, String> {
        Ok(match index {
            0 => Material::MattePlastic,
            1 => Material::GlossyPlastic,
            2 => Material::BrushedMetal,
            3 => Material::PolishedMetal,
            4 => Material::Glass,
            5 => Material::Rubber,
            6 => Material::Custom,
            _ => return Err(format!("There is no material {index} on this page.")),
        })
    }
}

/// The page's Light menu, by the index it sends.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub enum Light {
    /// A sky gradient plus a soft sun.
    Studio,
    /// A bright even sky and no sun.
    Overcast,
}

impl Light {
    /// The light a menu index names, or an error naming the index.
    pub fn from_index(index: u32) -> Result<Light, String> {
        Ok(match index {
            0 => Light::Studio,
            1 => Light::Overcast,
            _ => return Err(format!("There is no light {index} on this page.")),
        })
    }
}

/// The Custom material's sliders, each 0..=1.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct CustomSliders {
    pub roughness: f64,
    pub metallic: f64,
    pub transmission: f64,
}

/// The part's look: what the page's Material and Light controls say. Two equal looks are the same
/// image, so accumulation continues; a different look starts it over.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Look {
    pub material: Material,
    /// The color control's 8-bit sRGB value, as a color picker gives it.
    pub color: [u8; 3],
    pub custom: CustomSliders,
    pub light: Light,
    /// The sun's angular radius in degrees (Studio only).
    pub sun_degrees: f64,
}

/// The index of refraction the Custom material uses: ordinary glass and plastic, as the presets.
const CUSTOM_IOR: f64 = 1.5;

/// The sky of the Studio light (linear RGB): straight up, level, and straight down.
const STUDIO_ZENITH: [f64; 3] = [0.34, 0.49, 0.82];
const STUDIO_HORIZON: [f64; 3] = [0.85, 0.90, 0.98];
const STUDIO_NADIR: [f64; 3] = [0.30, 0.31, 0.32];

/// The Studio sun: its color, and the irradiance it delivers head on. The display's tone map
/// compresses a lit diffuse surface into the middle of its range at about this value.
const STUDIO_SUN_COLOR: [f64; 3] = [1.0, 0.95, 0.86];
const STUDIO_SUN_IRRADIANCE: f64 = 3.0;

/// The Overcast sky: bright and nearly even, a little darker at the horizon and below.
const OVERCAST_ZENITH: [f64; 3] = [1.10, 1.12, 1.15];
const OVERCAST_HORIZON: [f64; 3] = [0.80, 0.82, 0.85];
const OVERCAST_NADIR: [f64; 3] = [0.35, 0.36, 0.37];

/// The background of the data views.
const BACKGROUND_DATA: [f64; 3] = [0.08, 0.09, 0.11];

/// The direction toward the sun.
pub fn sun_direction() -> Vector3 {
    Vector3::new(-0.42, -0.55, 0.82).get_normal()
}

/// An 8-bit sRGB value as linear light, what a surface description wants. The library decodes
/// face colors the same way but keeps that step to itself.
fn linear_from_srgb(code: u8) -> f64 {
    let value = f64::from(code) / 255.0;
    if value <= 0.040_45 {
        value / 12.92
    } else {
        ((value + 0.055) / 1.055).powf(2.4)
    }
}

impl Look {
    /// The library's surface for this look's material and color.
    pub fn surface(&self) -> ScatteringSurface {
        let color = self.color.map(linear_from_srgb);
        match self.material {
            Material::MattePlastic => ScatteringSurface::matte_plastic(color),
            Material::GlossyPlastic => ScatteringSurface::glossy_plastic(color),
            Material::BrushedMetal => ScatteringSurface::brushed_metal(color),
            Material::PolishedMetal => ScatteringSurface::polished_metal(color),
            Material::Rubber => ScatteringSurface::rubber(color),
            // The library's glass preset is clear; the page tints it with the color control,
            // keeping everything else about the preset.
            Material::Glass => {
                let glass = ScatteringSurface::glass();
                ScatteringSurface::new(
                    color,
                    glass.metallic(),
                    glass.roughness(),
                    glass.transmission(),
                    glass.ior(),
                )
            }
            Material::Custom => ScatteringSurface::new(
                color,
                self.custom.metallic,
                self.custom.roughness,
                self.custom.transmission,
                CUSTOM_IOR,
            ),
        }
    }

    /// The library's lights for this look.
    pub fn lights(&self) -> Result<LightSetup, RayTracerError> {
        match self.light {
            Light::Studio => {
                let environment =
                    Environment::gradient(STUDIO_ZENITH, STUDIO_HORIZON, STUDIO_NADIR)?;
                let radius = self.sun_degrees.to_radians();
                // The size in steradians comes from the library's own answer for this sun.
                let solid_angle = Sun::new(sun_direction(), radius, [1.0; 3])?.solid_angle();
                let radiance = STUDIO_SUN_COLOR.map(|c| c * STUDIO_SUN_IRRADIANCE / solid_angle);
                Ok(LightSetup {
                    environment,
                    sun: Some(Sun::new(sun_direction(), radius, radiance)?),
                })
            }
            Light::Overcast => Ok(LightSetup::environment_only(Environment::gradient(
                OVERCAST_ZENITH,
                OVERCAST_HORIZON,
                OVERCAST_NADIR,
            )?)),
        }
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

/// The Normals or Depth color of pixel (`x`, `y`, top-left origin) for sample number `sample`.
pub fn data_color(
    scene: &TraceScene,
    camera: &Camera,
    mode: Mode,
    seed: u32,
    sample: u32,
    x: u32,
    y: u32,
) -> Result<[f64; 3], String> {
    let (jitter_x, jitter_y) = if sample == 0 {
        (0.5, 0.5)
    } else {
        pixel_jitter(u64::from(seed), x, y, sample)
    };
    let mut ray = camera.ray(f64::from(x) + jitter_x, f64::from(y) + jitter_y);
    let Some(hit) = scene.hit(&mut ray) else {
        return Ok(BACKGROUND_DATA);
    };
    Ok(match mode {
        Mode::Normals => {
            let n = hit
                .surface_point()
                .map_err(|e| e.to_string())?
                .shading_normal;
            [(n.x + 1.0) * 0.5, (n.y + 1.0) * 0.5, (n.z + 1.0) * 0.5]
        }
        Mode::Depth => {
            let distance = (scene.center - camera.eye).length();
            let (near, far) = depth_range(distance, scene.radius);
            let v = depth_value(hit.info.distance_to_hit, near, far);
            [v, v, v]
        }
        Mode::PathTraced => return Err("The path traced view has no data color.".to_string()),
    })
}
