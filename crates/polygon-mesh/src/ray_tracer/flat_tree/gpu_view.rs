//! What a GPU render looks at: everything that changes when the camera orbits or a light moves and
//! nothing that the geometry buffers hold. [`GpuView`] is the camera, the lights, the settings and
//! how the shader composites its samples; [`ViewRecord`] is the `PathView` uniform of `path.wgsl`,
//! written field by field in little-endian order as `layout.rs` does, and a test pins it against
//! naga's own layout. `gpu_pack.rs` narrows it when it packs a scene and
//! [`GpuRenderScene::set_view`](super::GpuRenderScene::set_view) rewrites it alone, so orbiting
//! never touches the buffers `gpu_scene.rs` builds from the parts.
//!
//! **The floor is a shape in the shader, not a quad in the tree.** The floor's extent grows with
//! the distance to the eye (a hundred view distances out), so a floor quad's vertices would move
//! whenever the camera orbits and every orbit would rebuild the tree over them; a quad a hundred
//! view distances out also spreads the tree's `f32` coordinates so far that a scene of ordinary
//! parts would need coarser vertices than `max_vertex_step` allows. Analytic, the floor is just
//! a plane height, a square extent and a surface, all in this record: `catcher.wgsl` intersects
//! it directly, keeps the point's height exact (`z` is the plane, not a rounded `origin + t *
//! direction`), and adds no triangles to the tree.
//!
//! **Coordinates are relative to the packed scene's origin** (`gpu_scene.rs`): every position
//! here is subtracted from the scene's center in `f64` before it narrows. Directions and radiances
//! need no recentring.

use solidmesh_math::Vector3;

use super::gpu_scene::{narrow3, ScenePlacement};
use super::{FlatTreeError, SurfaceRecord, COORDINATE_LIMIT};
use crate::ray_tracer::{
    LightSetup, LightStrategy, PathTraceSettings, RenderCamera, RenderScene, ScatteringSurface,
    SunThroughGlass,
};

/// The scene uniform's size in bytes (`PathView` in `path.wgsl`).
pub const VIEW_RECORD_BYTES: usize = 272;
/// The sRGB decoding table's size in bytes (`PathSrgb` in `path.wgsl`): a uniform, fixed at pack.
pub const SRGB_TABLE_BYTES: usize = 1024;

/// [`ViewRecord::flags`]: the camera is orthographic.
pub const SCENE_FLAG_ORTHOGRAPHIC: u32 = 1;
/// [`ViewRecord::flags`]: some part is less than fully present.
pub const SCENE_FLAG_SEE_THROUGH: u32 = 2;
/// [`ViewRecord::flags`]: there is a sun.
pub const SCENE_FLAG_SUN: u32 = 4;
/// [`ViewRecord::flags`]: the environment is a gradient.
pub const SCENE_FLAG_GRADIENT: u32 = 8;
/// [`ViewRecord::flags`]: samples go through the coverage channels ([`GpuCompositing`]).
pub const SCENE_FLAG_COMPOSITE: u32 = 16;
/// [`ViewRecord::flags`]: the view has a shadow-catching floor.
pub const SCENE_FLAG_FLOOR: u32 = 32;
/// [`ViewRecord::flags`]: some glass face lies on another (`gpu_contacts.rs`); clear, the shader
/// never reads the contact buffer.
pub const SCENE_FLAG_GLASS_CONTACTS: u32 = 64;
/// [`ViewRecord::flags`]: the sun goes straight through this scene's glass, as the CPU path
/// tracer's `straight` is: [`SunThroughGlass::Straight`] and some part with a glass share. The
/// shader then walks the line toward the sun straight through glass and mixes it with exact
/// transport by the CPU's weights (`sun.wgsl`); clear, it runs Exact's operations.
pub const SCENE_FLAG_SUN_STRAIGHT: u32 = 128;

/// The word in the part buffer where the glass contact block (`gpu_contacts.rs`) starts, right
/// after the part records; 0 for a scene without contacts, which appends no block.
fn contact_block_start(scene: &RenderScene) -> Result<u32, FlatTreeError> {
    if scene.glass_contacts().is_empty() {
        return Ok(0);
    }
    u32::try_from(scene.parts().parts().len() * (super::PART_STRIDE / 4)).map_err(|_| {
        FlatTreeError::NotRepresentable {
            reason: "more parts than a 32-bit word index into the part buffer can reach",
        }
    })
}

/// [`ViewRecord::roulette_after`] when Russian roulette is off.
pub const ROULETTE_OFF: u32 = u32::MAX;

/// The smallest angular radius a sun may have here, radians. A direction carries about 1.5e-7 of
/// f32 rounding, and `path.wgsl` asks whether a sampled direction lands on the disc from that
/// rounded direction: at this radius the rounding is 0.15% of the disc's size.
pub const MIN_GPU_SUN_ANGULAR_RADIUS: f64 = 1e-4;

/// What a floor sample's sun jitter is salted with, so its directions are unrelated to the
/// camera's and the sky's (`catcher.wgsl`).
pub const GPU_SUN_SEED_SALT: u64 = 0x5375_6e5f_6c69_6768;
/// What a floor sample's sky jitter is salted with.
pub const GPU_SKY_SEED_SALT: u64 = 0x536b_795f_6c69_6768;

/// The largest radiance channel, density or color the shader may hold: `f32::MAX` with a factor
/// of a thousand of headroom for the arithmetic it goes through before the accumulation checks.
const LARGEST_GPU_VALUE: f64 = f32::MAX as f64 / 1000.0;

/// How far behind the scene's bounding sphere an orthographic eye slides, as a multiple of the
/// sphere's radius: just outside it, so no corner can be on the eye's side of the plane.
const ORTHOGRAPHIC_EYE_MARGIN: f64 = 1.01;

/// A shadow-catching floor: a plane at height `z` in world coordinates, the square of half-size
/// `half_extent` around `center`. Camera rays that reach it return no radiance of their own; they
/// report what the sun and sky lost ([`GpuCompositing::Floor`]).
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct GpuFloor {
    pub center: (f64, f64),
    pub half_extent: f64,
    pub z: f64,
    /// How much of what is below the floor it lets through, in `0..=1`: `1 - alpha` of the bed's
    /// texture; 0 skips the look past it.
    pub show_through: f64,
    /// What paths see where they meet the floor. Its base color is also the light the floor sends
    /// back along a camera ray that reaches it past a see-through part.
    pub surface: ScatteringSurface,
}

/// How the shader treats a camera ray's sample.
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum GpuCompositing {
    /// Every ray's radiance adds into `accumulation`, the way the CPU's `PathTracer` averages its
    /// passes; the coverage channels are left alone.
    Plain,
    /// Radiance over a coverage: the parts' color adds only where the line meets a part, and the
    /// coverage channels carry the alpha. No floor.
    Coverage,
    /// As [`GpuCompositing::Coverage`], with a shadow-catching floor at the end of the line.
    Floor(GpuFloor),
}

/// Camera, lights, settings and compositing of a render: the part of a packed scene that can
/// change without packing the geometry again.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct GpuView {
    pub camera: RenderCamera,
    /// The image's size, pixels: which pixel a dispatch's index means.
    pub image: (u32, u32),
    /// The size the camera measures its projection against. A caller whose image width differs
    /// from the camera's reference width (a side panel) passes them apart; pixels of the image
    /// past `trace.0` keep projecting along the same camera.
    pub trace: (u32, u32),
    pub lights: LightSetup,
    pub settings: PathTraceSettings,
    pub compositing: GpuCompositing,
}

impl GpuView {
    /// A plain view: every pixel of a `width` by `height` image, measured against that same size,
    /// with no coverage channels.
    pub fn plain(
        camera: RenderCamera,
        width: u32,
        height: u32,
        lights: LightSetup,
        settings: PathTraceSettings,
    ) -> GpuView {
        GpuView {
            camera,
            image: (width, height),
            trace: (width, height),
            lights,
            settings,
            compositing: GpuCompositing::Plain,
        }
    }
}

/// The per-view uniform: camera, lights, settings and floor. See `PathView` in `path.wgsl` for the
/// field-by-field meaning. The bytes this record is the only thing orbiting rewrites.
#[derive(Debug, Clone, PartialEq)]
pub struct ViewRecord {
    pub eye: [f32; 4],
    /// The view direction.
    pub forward: [f32; 3],
    /// The word in the part buffer where the glass contact block (`gpu_contacts.rs`) starts, 0
    /// without contacts; it fills the fourth lane after `forward`.
    pub contact_block: u32,
    pub right: [f32; 4],
    pub up: [f32; 4],
    pub width: u32,
    pub height: u32,
    pub max_bounces: u32,
    pub roulette_after: u32,
    pub seed_low: u32,
    pub seed_high: u32,
    pub strategy: u32,
    pub flags: u32,
    pub env_zenith: [f32; 4],
    pub env_horizon: [f32; 4],
    pub env_ground: [f32; 4],
    pub sun_direction: [f32; 4],
    pub sun_tangent: [f32; 4],
    pub sun_bitangent: [f32; 4],
    pub sun_radiance: [f32; 4],
    pub trace_width: u32,
    pub trace_height: u32,
    pub floor_surface_flags: u32,
    pub floor_show_through: f32,
    /// xy the floor's center, z its height, w its half extent, in packed coordinates.
    pub floor_plane: [f32; 4],
    pub floor_base: [f32; 4],
    pub floor_rest: [f32; 4],
}

impl ViewRecord {
    /// This record's bytes, in uniform-buffer order.
    pub fn to_le_bytes(&self) -> [u8; VIEW_RECORD_BYTES] {
        let mut bytes = [0u8; VIEW_RECORD_BYTES];
        let mut offset = 0;
        let mut put = |word: u32| {
            bytes[offset..offset + 4].copy_from_slice(&word.to_le_bytes());
            offset += 4;
        };
        for value in self.eye {
            put(value.to_bits());
        }
        for value in self.forward {
            put(value.to_bits());
        }
        put(self.contact_block);
        for lane in [self.right, self.up] {
            for value in lane {
                put(value.to_bits());
            }
        }
        for word in [
            self.width,
            self.height,
            self.max_bounces,
            self.roulette_after,
            self.seed_low,
            self.seed_high,
            self.strategy,
            self.flags,
        ] {
            put(word);
        }
        let lights = [
            self.env_zenith,
            self.env_horizon,
            self.env_ground,
            self.sun_direction,
            self.sun_tangent,
            self.sun_bitangent,
            self.sun_radiance,
        ];
        for lane in lights {
            for value in lane {
                put(value.to_bits());
            }
        }
        for word in [
            self.trace_width,
            self.trace_height,
            self.floor_surface_flags,
            self.floor_show_through.to_bits(),
        ] {
            put(word);
        }
        for lane in [self.floor_plane, self.floor_base, self.floor_rest] {
            for value in lane {
                put(value.to_bits());
            }
        }
        bytes
    }
}

const CAMERA_TOO_FAR: &str = "the camera is more than 1e18 units from the scene, too far for the \
32-bit floats a GPU traces with: move it closer, or render on the CPU";

const CAMERA_PROJECTION_TOO_LARGE: &str = "the camera's view is too wide for the 32-bit floats a \
GPU traces with: narrow its field of view or orthographic extent, or render on the CPU";

const LIGHT_TOO_BRIGHT: &str = "a light's radiance or density is too large for the 32-bit floats \
a GPU traces with; turn the light down or make the sun's disc larger";

const FLOOR_NOT_REPRESENTABLE: &str = "the floor's height or extent can't be carried in the \
GPU's 32-bit floats: bring it within 1e18 units of the scene, with a positive extent and a \
see-through share between 0 and 1, or render on the CPU";

const IMAGE_EMPTY: &str = "a render needs an image of at least one pixel";

const SUN_TOO_SMALL: &str = "the sun's disc is narrower than the GPU's 32-bit directions can \
resolve: make it at least 1e-4 radians (0.006 degrees) across, or render on the CPU";

pub(super) fn checked_value(value: f64, reason: &'static str) -> Result<f32, FlatTreeError> {
    if value.is_finite() && value.abs() <= LARGEST_GPU_VALUE {
        Ok(value as f32)
    } else {
        Err(FlatTreeError::NotRepresentable { reason })
    }
}

/// The floor's fields as the shader reads them.
#[derive(Default)]
struct FloorLanes {
    plane: [f32; 4],
    base: [f32; 4],
    rest: [f32; 4],
    surface_flags: u32,
    show_through: f32,
}

/// The floor's lanes, with the plane relative to the packed scene's origin.
fn floor_lanes(floor: &GpuFloor, placement: &ScenePlacement) -> Result<FloorLanes, FlatTreeError> {
    let refused = FlatTreeError::NotRepresentable {
        reason: FLOOR_NOT_REPRESENTABLE,
    };
    let center_x = floor.center.0 - placement.origin.x;
    let center_y = floor.center.1 - placement.origin.y;
    let z = floor.z - placement.origin.z;
    let in_range = |value: f64| value.is_finite() && value.abs() <= COORDINATE_LIMIT;
    if !in_range(center_x)
        || !in_range(center_y)
        || !in_range(z)
        || !in_range(floor.half_extent)
        || floor.half_extent <= 0.0
        || !(0.0..=1.0).contains(&floor.show_through)
    {
        return Err(refused);
    }
    let record = SurfaceRecord::from_surface(&floor.surface);
    let plane = [
        checked_value(center_x, FLOOR_NOT_REPRESENTABLE)?,
        checked_value(center_y, FLOOR_NOT_REPRESENTABLE)?,
        // The same cast a part's recentred corner goes through, so a part resting on the floor
        // shares its height with the plane to the bit.
        z as f32,
        checked_value(floor.half_extent, FLOOR_NOT_REPRESENTABLE)?,
    ];
    let base = [
        record.base_color[0],
        record.base_color[1],
        record.base_color[2],
        record.metallic,
    ];
    let rest = [
        record.roughness,
        record.transmission,
        record.ior,
        record.ior_minus_one,
    ];
    Ok(FloorLanes {
        plane,
        base,
        rest,
        surface_flags: record.flags,
        show_through: floor.show_through as f32,
    })
}

/// The view's record, for a pack placed at `placement`.
pub(super) fn view_record(
    scene: &RenderScene,
    view: &GpuView,
    placement: &ScenePlacement,
) -> Result<ViewRecord, FlatTreeError> {
    if view.image.0 == 0 || view.image.1 == 0 || view.trace.0 == 0 || view.trace.1 == 0 {
        return Err(FlatTreeError::NotRepresentable {
            reason: IMAGE_EMPTY,
        });
    }
    let lane = |v: [f64; 3], w: f64, reason: &'static str| -> Result<[f32; 4], FlatTreeError> {
        Ok([
            checked_value(v[0], reason)?,
            checked_value(v[1], reason)?,
            checked_value(v[2], reason)?,
            checked_value(w, reason)?,
        ])
    };
    let direction = |v: Vector3| -> [f32; 4] {
        let [x, y, z] = narrow3(v);
        [x, y, z, 0.0]
    };

    let lights = &view.lights;
    let (zenith, horizon, ground, gradient) = lights.environment.as_gradient();
    let mut flags = 0;
    if view.camera.is_orthographic() {
        flags |= SCENE_FLAG_ORTHOGRAPHIC;
    }
    if scene.has_see_through_parts() {
        flags |= SCENE_FLAG_SEE_THROUGH;
    }
    if !scene.glass_contacts().is_empty() {
        flags |= SCENE_FLAG_GLASS_CONTACTS;
    }
    if gradient {
        flags |= SCENE_FLAG_GRADIENT;
    }
    if view.settings.sun_through_glass == SunThroughGlass::Straight && scene.has_straight_glass() {
        flags |= SCENE_FLAG_SUN_STRAIGHT;
    }

    let mut sun_direction = [0.0; 4];
    let mut sun_tangent = [0.0; 4];
    let mut sun_bitangent = [0.0; 4];
    let mut sun_radiance = [0.0; 4];
    if let Some(sun) = &lights.sun {
        flags |= SCENE_FLAG_SUN;
        if sun.angular_radius() < MIN_GPU_SUN_ANGULAR_RADIUS {
            return Err(FlatTreeError::NotRepresentable {
                reason: SUN_TOO_SMALL,
            });
        }
        let frame = sun.local_frame();
        let versine = sun.versine();
        let pdf = sun.pdf();
        sun_direction = lane(
            [frame.normal.x, frame.normal.y, frame.normal.z],
            versine,
            LIGHT_TOO_BRIGHT,
        )?;
        sun_tangent = lane(
            [frame.tangent.x, frame.tangent.y, frame.tangent.z],
            pdf,
            LIGHT_TOO_BRIGHT,
        )?;
        sun_bitangent = lane(
            [frame.bitangent.x, frame.bitangent.y, frame.bitangent.z],
            2.0 * versine,
            LIGHT_TOO_BRIGHT,
        )?;
        sun_radiance = lane(sun.radiance(), 0.0, LIGHT_TOO_BRIGHT)?;
    }

    let mut floor_fields = FloorLanes::default();
    match &view.compositing {
        GpuCompositing::Plain => {}
        GpuCompositing::Coverage => flags |= SCENE_FLAG_COMPOSITE,
        GpuCompositing::Floor(floor) => {
            flags |= SCENE_FLAG_COMPOSITE | SCENE_FLAG_FLOOR;
            floor_fields = floor_lanes(floor, placement)?;
        }
    }

    let basis = view.camera.basis();
    let mut eye_position = basis.eye - placement.origin;
    if view.camera.is_orthographic() {
        // How far along the view the scene's center is: see `gpu_scene.rs`.
        let slide = -eye_position.dot(basis.forward) - placement.radius * ORTHOGRAPHIC_EYE_MARGIN;
        if slide > 0.0 {
            eye_position += basis.forward * slide;
        }
    }
    let too_far = (0..3)
        .any(|axis| !eye_position[axis].is_finite() || eye_position[axis].abs() > COORDINATE_LIMIT);
    if too_far {
        return Err(FlatTreeError::NotRepresentable {
            reason: CAMERA_TOO_FAR,
        });
    }
    let mut eye = direction(eye_position);
    eye[3] = checked_value(basis.projection_scale, CAMERA_PROJECTION_TOO_LARGE)?;

    let settings = &view.settings;
    Ok(ViewRecord {
        eye,
        forward: narrow3(basis.forward),
        contact_block: contact_block_start(scene)?,
        right: direction(basis.right),
        up: direction(basis.up),
        width: view.image.0,
        height: view.image.1,
        max_bounces: settings.max_bounces,
        roulette_after: settings.russian_roulette_after.unwrap_or(ROULETTE_OFF),
        // Truncating casts on purpose: the seed's low and high halves.
        seed_low: settings.seed as u32,
        seed_high: (settings.seed >> 32) as u32,
        strategy: match settings.strategy {
            LightStrategy::Combined => 0,
            LightStrategy::NextEventOnly => 1,
            LightStrategy::SurfaceSamplingOnly => 2,
        },
        flags,
        env_zenith: lane(zenith, 0.0, LIGHT_TOO_BRIGHT)?,
        env_horizon: lane(horizon, 0.0, LIGHT_TOO_BRIGHT)?,
        env_ground: lane(ground, 0.0, LIGHT_TOO_BRIGHT)?,
        sun_direction,
        sun_tangent,
        sun_bitangent,
        sun_radiance,
        trace_width: view.trace.0,
        trace_height: view.trace.1,
        floor_surface_flags: floor_fields.surface_flags,
        floor_show_through: floor_fields.show_through,
        floor_plane: floor_fields.plane,
        floor_base: floor_fields.base,
        floor_rest: floor_fields.rest,
    })
}

/// The sRGB decoding uniform's bytes: 256 values narrowed once from the f64 table `scene` holds,
/// four to a `vec4` as `PathSrgb` lays them out.
pub(super) fn srgb_table_bytes(scene: &RenderScene) -> [u8; SRGB_TABLE_BYTES] {
    let mut bytes = [0u8; SRGB_TABLE_BYTES];
    for (index, value) in scene.srgb_table().iter().enumerate() {
        let at = index * 4;
        bytes[at..at + 4].copy_from_slice(&(*value as f32).to_le_bytes());
    }
    bytes
}
