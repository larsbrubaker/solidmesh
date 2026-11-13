//! The render check's scenes: small enough that the CPU reference stays within seconds, and
//! together reaching every branch of the GPU path loop. Each is one [`CheckScene`], chosen by
//! index, and `gpu_bench_render` renders them on the GPU and the CPU.
//!
//! **The closed forms.** Three scenes have an exact answer, independent of either renderer: a
//! white furnace of radiance 1 around a surface that loses nothing. Every path that ends by
//! escaping brings exactly 1, so the image is exactly 1 at every pixel, concave part or not, and a
//! mistake shared by the CPU and GPU would still show. The concave scenes interreflect, which the
//! convex cube can't, and the second uses index-matched glass, which has to pass light straight
//! through both surfaces of the U.
//!
//! The closed form holds only if no energy is lost on the way. So those paths have Russian roulette
//! off (it would be unbiased but noisy, and the check wants exact agreement) and a bounce limit
//! of 1024. A path in the U-shaped channel has at least a tenth of a chance of escaping each
//! bounce, so `0.9^1024`, about 1e-47, of its energy is left to the cap. Their faces are flat
//! with crease-sharp normals, so no bent-normal rejection can lose any either.

use std::f64::consts::PI;
use std::sync::Arc;

use solidmesh::math::{Color, Matrix4X4, Vector2, Vector3};
use solidmesh::mesh::ray_tracer::{
    Environment, LightSetup, LightStrategy, MeshSurface, PathTraceSettings, PlacedPart,
    PlacedParts, RenderCamera, RenderScene, ScatteringSurface, Sun,
};
use solidmesh::mesh::solids::platonic_solids::create_cube;
use solidmesh::mesh::Mesh;

use crate::fillet_pick::build_bvh;
use crate::shapes::{prism, sphere};

/// The seed every render here uses, chosen before any result.
pub const RENDER_SEED: u64 = 0x2545_f491_4f6c_dd1d;

/// A square image's side, pixels, unless a scene says otherwise.
pub const RENDER_SIZE: u32 = 48;

/// Samples for a scene whose noise is ordinary.
const ORDINARY_SAMPLES: u32 = 64;
/// Samples for the glossy and glass scenes, whose few bright directions make the noise large.
const NOISY_SAMPLES: u32 = 384;
/// Samples for the scenes a delta lobe or surface sampling reaches the sun through.
const SPECULAR_SAMPLES: u32 = 256;
/// Samples for a closed-form scene: it has no noise, so it needs only a few.
const CLOSED_FORM_SAMPLES: u32 = 16;

/// The bounces of a closed-form scene: see the module header.
const CLOSED_FORM_BOUNCES: u32 = 1024;

/// Where a far part sits, as the app's build plate can be from the world origin.
const FAR_OFFSET: f64 = 1.0e7;

/// The sun's radius in the sampled-only scene, wide enough that surface sampling finds it often.
const WIDE_SUN_DEGREES: f64 = 15.0;

const SCENE_NAMES: [&str; 15] = [
    "White furnace, convex (closed form)",
    "Diffuse box under the sun",
    "Glossy metal sphere",
    "Rough glass sphere",
    "Part at half opacity",
    "Colored-face part",
    "Perfect mirror sphere (delta lobe)",
    "Smooth glass sphere (delta lobes)",
    "Part under a mirrored, non-uniform scale",
    "Orthographic camera, wide image",
    "Sun by next-event estimation only, tall image",
    "Sun by surface sampling only",
    "Part 1e7 mm from the origin",
    "White furnace, concave lossless part (closed form)",
    "White furnace, concave index-matched glass (closed form)",
];

/// How many scenes the check renders.
pub const SCENE_COUNT: u32 = SCENE_NAMES.len() as u32;

/// One scene of the check: what to render, how much, and the answer when it is known exactly.
pub struct CheckScene {
    pub name: &'static str,
    pub scene: RenderScene,
    pub camera: RenderCamera,
    pub width: u32,
    pub height: u32,
    pub samples: u32,
    pub settings: PathTraceSettings,
    pub closed_form: Option<[f64; 3]>,
}

fn part(mesh: &Mesh, crease: f64, matrix: Matrix4X4) -> Result<PlacedPart, String> {
    let tree = build_bvh(mesh).ok_or_else(|| "The scene's mesh has no faces.".to_string())?;
    let surface = MeshSurface::new(mesh, crease).map_err(|e| e.to_string())?;
    PlacedPart::new(tree, Arc::new(surface), matrix).map_err(|e| e.to_string())
}

fn perspective_camera(offset: Vector3) -> Result<RenderCamera, String> {
    RenderCamera::perspective(
        Vector3::new(4.0, -5.0, 3.0) + offset,
        Vector3::new(0.0, 0.0, 0.3) + offset,
        Vector3::UNIT_Z,
        40.0,
    )
    .map_err(|e| e.to_string())
}

fn sky() -> Result<Environment, String> {
    Environment::gradient([0.6, 0.7, 0.9], [0.5, 0.5, 0.5], [0.15, 0.13, 0.1])
        .map_err(|e| e.to_string())
}

fn sun_of_radius(radius_degrees: f64, radiance: [f64; 3]) -> Result<Sun, String> {
    Sun::new(
        Vector3::new(0.3, -0.5, 0.8),
        radius_degrees.to_radians(),
        radiance,
    )
    .map_err(|e| e.to_string())
}

fn sun() -> Result<Sun, String> {
    sun_of_radius(2.5, [8000.0, 7600.0, 7000.0])
}

fn floor_mesh() -> Mesh {
    let half = 8.0;
    let vertices = [
        -half, -half, -0.5, half, -half, -0.5, half, half, -0.5, -half, half, -0.5,
    ];
    Mesh::from_arrays(&vertices, &[0, 1, 2, 0, 2, 3])
}

/// A U-shaped solid: a cross section open toward +Y, extruded along Z, the simplest shape whose
/// inside faces see each other.
fn u_channel() -> Result<Mesh, String> {
    let contour = [
        Vector2::new(-1.0, -1.0),
        Vector2::new(1.0, -1.0),
        Vector2::new(1.0, 1.0),
        Vector2::new(0.5, 1.0),
        Vector2::new(0.5, -0.4),
        Vector2::new(-0.5, -0.4),
        Vector2::new(-0.5, 1.0),
        Vector2::new(-1.0, 1.0),
    ];
    prism(&contour, 2.0)
}

/// A camera looking into the open side of [`u_channel`], so the inside walls fill the view.
fn camera_into_channel() -> Result<RenderCamera, String> {
    RenderCamera::perspective(
        Vector3::new(1.5, 4.0, 1.5),
        Vector3::new(0.0, 0.2, 0.0),
        Vector3::UNIT_Z,
        35.0,
    )
    .map_err(|e| e.to_string())
}

fn white_furnace() -> Result<LightSetup, String> {
    Ok(LightSetup::environment_only(
        Environment::constant([1.0; 3]).map_err(|e| e.to_string())?,
    ))
}

fn closed_form_settings() -> PathTraceSettings {
    PathTraceSettings {
        seed: RENDER_SEED,
        max_bounces: CLOSED_FORM_BOUNCES,
        russian_roulette_after: None,
        strategy: LightStrategy::Combined,
        ..PathTraceSettings::default()
    }
}

/// Which scene of the check, by index.
pub fn check_scene(index: u32) -> Result<CheckScene, String> {
    let name = *SCENE_NAMES
        .get(index as usize)
        .ok_or_else(|| format!("There is no render check scene {index}."))?;
    let sharp = 20.0_f64.to_radians();
    let smooth = 80.0_f64.to_radians();
    let identity = Matrix4X4::IDENTITY;
    let floor_surface = ScatteringSurface::matte_plastic([0.8, 0.8, 0.8]);

    let mut parts = Vec::new();
    let mut surfaces = Vec::new();
    let mut opacities: Option<Vec<f64>> = None;
    let mut closed_form = None;
    let mut lights = LightSetup {
        environment: sky()?,
        sun: Some(sun()?),
    };
    let mut settings = PathTraceSettings {
        seed: RENDER_SEED,
        ..PathTraceSettings::default()
    };
    let mut camera = perspective_camera(Vector3::ZERO)?;
    let (mut width, mut height) = (RENDER_SIZE, RENDER_SIZE);
    let mut samples = ORDINARY_SAMPLES;
    let mut with_floor = true;

    match index {
        0 => {
            parts.push(part(&create_cube(1.5, 1.5, 1.5), sharp, identity)?);
            surfaces.push(ScatteringSurface::new([1.0; 3], 0.0, 1.0, 0.0, 1.0));
            lights = white_furnace()?;
            closed_form = Some([1.0; 3]);
            samples = CLOSED_FORM_SAMPLES;
            with_floor = false;
        }
        1 => {
            parts.push(part(&create_cube(1.5, 1.5, 1.5), sharp, identity)?);
            surfaces.push(ScatteringSurface::new([0.6, 0.6, 0.6], 0.0, 0.8, 0.0, 1.5));
        }
        2 => {
            parts.push(part(&sphere(1.0, 32)?, smooth, identity)?);
            surfaces.push(ScatteringSurface::new([0.9, 0.7, 0.3], 1.0, 0.25, 0.0, 1.5));
            samples = NOISY_SAMPLES;
        }
        3 => {
            parts.push(part(&sphere(1.0, 32)?, smooth, identity)?);
            surfaces.push(ScatteringSurface::new([1.0; 3], 0.0, 0.3, 1.0, 1.5));
            samples = NOISY_SAMPLES;
        }
        4 => {
            parts.push(part(&sphere(1.0, 24)?, smooth, identity)?);
            surfaces.push(ScatteringSurface::new([0.2, 0.5, 0.8], 0.0, 0.5, 0.0, 1.5));
            opacities = Some(vec![0.5, 1.0]);
        }
        5 => {
            let mut cube = create_cube(1.5, 1.5, 1.5);
            let palette = [
                Color::new(230, 40, 30, 255),
                Color::new(40, 200, 60, 255),
                Color::new(40, 60, 220, 255),
                Color::new(230, 200, 40, 255),
                Color::new(220, 60, 200, 255),
                Color::new(60, 220, 220, 255),
            ];
            cube.face_colors = Some(
                (0..cube.faces.len())
                    .map(|face| palette[face / 2 % palette.len()])
                    .collect(),
            );
            parts.push(part(&cube, sharp, identity)?);
            surfaces.push(ScatteringSurface::matte_plastic([1.0; 3]));
        }
        6 => {
            parts.push(part(&sphere(1.0, 32)?, smooth, identity)?);
            surfaces.push(ScatteringSurface::new(
                [0.95, 0.93, 0.88],
                1.0,
                0.0,
                0.0,
                1.5,
            ));
            samples = SPECULAR_SAMPLES;
        }
        7 => {
            parts.push(part(&sphere(1.0, 32)?, smooth, identity)?);
            surfaces.push(ScatteringSurface::glass());
            samples = SPECULAR_SAMPLES;
        }
        8 => {
            let matrix = Matrix4X4::create_scale(1.4, -0.7, 1.0)
                * Matrix4X4::create_rotation_z(0.6)
                * Matrix4X4::create_translation(0.2, -0.1, 0.1);
            parts.push(part(&sphere(1.0, 24)?, smooth, matrix)?);
            surfaces.push(ScatteringSurface::new([0.7, 0.3, 0.2], 0.0, 0.4, 0.0, 1.5));
        }
        9 => {
            parts.push(part(&create_cube(1.5, 1.5, 1.5), sharp, identity)?);
            surfaces.push(ScatteringSurface::new([0.5, 0.6, 0.7], 0.0, 0.6, 0.0, 1.5));
            camera = RenderCamera::orthographic(
                Vector3::new(4.0, -5.0, 3.0),
                Vector3::new(0.0, 0.0, 0.3),
                Vector3::UNIT_Z,
                5.0,
            )
            .map_err(|e| e.to_string())?;
            (width, height) = (96, 48);
        }
        10 => {
            parts.push(part(&sphere(1.0, 24)?, smooth, identity)?);
            surfaces.push(ScatteringSurface::new([0.8, 0.5, 0.3], 0.0, 0.7, 0.0, 1.5));
            settings.strategy = LightStrategy::NextEventOnly;
            (width, height) = (48, 96);
        }
        11 => {
            parts.push(part(&create_cube(1.5, 1.5, 1.5), sharp, identity)?);
            surfaces.push(ScatteringSurface::matte_plastic([0.6, 0.5, 0.4]));
            // A white face turned to this sun reflects radiance * solid_angle / pi, as bright as
            // the small sun's face, so the picture has the same exposure.
            let half = (WIDE_SUN_DEGREES.to_radians() / 2.0).sin();
            let scale = PI / (2.0 * PI * 2.0 * half * half);
            lights.sun = Some(sun_of_radius(WIDE_SUN_DEGREES, [scale, scale, scale])?);
            settings.strategy = LightStrategy::SurfaceSamplingOnly;
            samples = SPECULAR_SAMPLES;
        }
        12 => {
            let offset = Vector3::new(FAR_OFFSET, 0.0, 0.0);
            let matrix = Matrix4X4::create_translation(FAR_OFFSET, 0.0, 0.0);
            parts.push(part(&create_cube(1.5, 1.5, 1.5), sharp, matrix)?);
            surfaces.push(ScatteringSurface::new([0.6, 0.6, 0.6], 0.0, 0.8, 0.0, 1.5));
            parts.push(part(&floor_mesh(), smooth, matrix)?);
            surfaces.push(floor_surface);
            camera = perspective_camera(offset)?;
            with_floor = false;
        }
        13 => {
            parts.push(part(&u_channel()?, sharp, identity)?);
            surfaces.push(ScatteringSurface::new([1.0; 3], 0.0, 1.0, 0.0, 1.0));
            lights = white_furnace()?;
            closed_form = Some([1.0; 3]);
            samples = CLOSED_FORM_SAMPLES;
            settings = closed_form_settings();
            camera = camera_into_channel()?;
            with_floor = false;
        }
        _ => {
            parts.push(part(&u_channel()?, sharp, identity)?);
            surfaces.push(ScatteringSurface::new([1.0; 3], 0.0, 0.3, 1.0, 1.0));
            lights = white_furnace()?;
            closed_form = Some([1.0; 3]);
            samples = CLOSED_FORM_SAMPLES;
            settings = closed_form_settings();
            camera = camera_into_channel()?;
            with_floor = false;
        }
    }

    if with_floor {
        parts.push(part(&floor_mesh(), smooth, identity)?);
        surfaces.push(floor_surface);
    }

    let mut scene =
        RenderScene::new(PlacedParts::new(parts), surfaces, lights).map_err(|e| e.to_string())?;
    if let Some(opacities) = opacities {
        scene = scene
            .with_part_opacities(opacities)
            .map_err(|e| e.to_string())?;
    }
    Ok(CheckScene {
        name,
        scene,
        camera,
        width,
        height,
        samples,
        settings,
        closed_form,
    })
}
