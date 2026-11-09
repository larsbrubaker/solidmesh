//! The Ray Tracing page's GPU route, the WGSL path loop's side of it in the worker: packing the
//! page's part and floor with the library's `GpuRenderScene`, the bytes a new camera or look
//! writes, and turning an accumulation the GPU has filled into the page's image with the library's
//! own display code. The page's WebGPU calls and slicing are `demo/src/ray-trace-gpu.ts`; the rule
//! for which route to take is `demo/src/ray-trace-rules.ts`.
//!
//! The tree over a part and its floor is built once per part. A new camera or light changes only
//! the view uniform (`GpuRenderScene::set_view`), and a new material only the part records, so
//! changing either costs a small buffer write and no rebuild, as on the CPU route. Both rewrites
//! come from here so they can't drift from a fresh pack; `tests/ray_trace_gpu.rs` holds them to
//! that.
//!
//! The picture matches the CPU route's because the numbers do: the same camera, lights, bounces
//! and seed (`ray_trace::settings_of`) go into the shader, and the same exposure, tone map and
//! sRGB encoding (`display_byte`) come out. A pixel with no samples yet keeps what was on screen,
//! stretched to the new size if there is one, exactly as the CPU route's `Accumulation` does; the
//! GPU's accumulation is row order from the bottom left, the page's image from the top left.
//!
//! A scene this GPU can't hold is a [`GpuOutcome::Refused`] with the library's own words saying
//! why and what would fit; the page renders it on the CPU and says so. Only an error that is a
//! bug comes back as `Err`.

use solidmesh::mesh::ray_tracer::flat_tree::{
    FlatTreeError, GpuRenderScene, GpuSceneLimits, GpuView, PartRecord, SurfaceRecord,
    ACCUMULATION_STRIDE, PART_STRIDE, VIEW_RECORD_BYTES,
};
use solidmesh::mesh::ray_tracer::{display_byte, RenderScene};

use crate::ray_trace::{settings_of, stretched_display, TraceRequest, EXPOSURE};
use crate::ray_trace_scene::{Camera, TraceScene};
use crate::ray_trace_shade::{CustomSliders, Light, Look, Material, Mode};

/// How many numbers the page sends for one request (`ray_trace_pass`'s arguments, the budget aside).
pub const TRACE_ARGS: usize = 17;

/// The coarsest f32 step a corner may have, as a fraction of the part's radius. The floor reaches
/// 120 radii out, so its corners round to about 2e-5 of a radius; this leaves room and is still far
/// below a pixel of the part, so only a scene broken in some other way is refused.
const VERTEX_STEP_RADII: f64 = 1e-3;

/// What a GPU call answers: done, or this scene doesn't fit this GPU, in the library's own words.
pub enum GpuOutcome<T> {
    Fits(T),
    Refused(String),
}

/// The bytes a change of view or look writes.
pub struct GpuViewBytes {
    pub view: [u8; VIEW_RECORD_BYTES],
    /// One [`PartRecord`] per placed part.
    pub parts: Vec<u8>,
}

/// The request the page's 17 numbers describe: the arguments of `ray_trace_pass` in order, the
/// budget aside, the preview flag as 0 or 1.
pub fn request_from_args(args: &[f64]) -> Result<TraceRequest, String> {
    if args.len() != TRACE_ARGS {
        return Err(format!(
            "A trace request is {TRACE_ARGS} numbers and this one has {}.",
            args.len()
        ));
    }
    // `as` truncates and saturates: a menu index stays a small whole number, a color channel
    // stays 0..=255 and a bad value is refused by `from_index` or clamped by the color.
    let index = |at: usize| args[at] as u32;
    let byte = |at: usize| args[at] as u8;
    Ok(TraceRequest {
        yaw: args[0],
        pitch: args[1],
        zoom: args[2],
        mode: Mode::from_index(index(3))?,
        look: Look {
            material: Material::from_index(index(4))?,
            color: [byte(5), byte(6), byte(7)],
            custom: CustomSliders {
                roughness: args[8],
                metallic: args[9],
                transmission: args[10],
            },
            light: Light::from_index(index(11))?,
            sun_degrees: args[12],
        },
        seed: index(13),
        width: index(14),
        height: index(15),
        preview: args[16] != 0.0,
    })
}

/// The library's words for a pack or view this GPU cannot hold, or `None` for an error that is a
/// bug rather than something about the scene.
pub fn fallback_reason(error: &FlatTreeError) -> Option<String> {
    match error {
        FlatTreeError::PackedBufferTooLarge { .. }
        | FlatTreeError::BufferTooLarge { .. }
        | FlatTreeError::TooManyTriangles { .. }
        | FlatTreeError::SceneTooLarge { .. }
        | FlatTreeError::TreeTooDeep { .. }
        | FlatTreeError::InvalidCoordinate { .. }
        | FlatTreeError::NotRepresentable { .. } => Some(error.to_string()),
        FlatTreeError::NoTriangles | FlatTreeError::UnexpectedBvhLeaf => None,
    }
}

fn view_of(trace: &TraceScene, request: &TraceRequest) -> Result<GpuView, String> {
    let camera = Camera::new(
        trace,
        request.yaw,
        request.pitch,
        request.zoom,
        request.width,
        request.height,
    )?;
    let lights = request.look.lights().map_err(|e| e.to_string())?;
    Ok(GpuView::plain(
        camera.render_camera(),
        request.width,
        request.height,
        lights,
        settings_of(request),
    ))
}

/// What a GPU refusal means here: refuse with the reason, or report the bug.
fn refuse<T>(error: FlatTreeError) -> Result<GpuOutcome<T>, String> {
    match fallback_reason(&error) {
        Some(reason) => Ok(GpuOutcome::Refused(reason)),
        None => Err(error.to_string()),
    }
}

/// One part packed for the GPU, and what the worker has shown from its accumulation.
pub struct GpuSession {
    scene: GpuRenderScene,
    rendering: (Look, RenderScene),
    width: u32,
    height: u32,
    /// The image last shown, RGBA8 from the top left.
    display: Vec<u8>,
}

impl GpuSession {
    /// Packs `trace` for `request`'s look and view.
    pub fn pack(
        trace: &TraceScene,
        request: TraceRequest,
        max_binding: u64,
    ) -> Result<GpuOutcome<GpuSession>, String> {
        let rendering = trace.render_scene(&request.look)?;
        let view = view_of(trace, &request)?;
        let limits = GpuSceneLimits {
            max_storage_binding: max_binding,
            max_vertex_step: trace.radius * VERTEX_STEP_RADII,
        };
        match GpuRenderScene::pack_view(&rendering, &view, limits) {
            Ok(scene) => Ok(GpuOutcome::Fits(GpuSession {
                scene,
                rendering: (request.look, rendering),
                width: 0,
                height: 0,
                display: Vec::new(),
            })),
            Err(error) => refuse(error),
        }
    }

    /// The pack, for the buffers the page uploads once.
    pub fn packed(&self) -> &GpuRenderScene {
        &self.scene
    }

    /// The view uniform and part records of `request`: all a new camera or look writes.
    pub fn view(
        &mut self,
        trace: &TraceScene,
        request: TraceRequest,
    ) -> Result<GpuOutcome<GpuViewBytes>, String> {
        if self.rendering.0 != request.look {
            self.rendering = (request.look, trace.render_scene(&request.look)?);
        }
        let view = view_of(trace, &request)?;
        if let Err(error) = self.scene.set_view(&self.rendering.1, &view) {
            return refuse(error);
        }
        Ok(GpuOutcome::Fits(GpuViewBytes {
            view: self.scene.view_bytes(),
            parts: part_bytes(trace, &self.rendering),
        }))
    }

    /// Decodes `accumulation` into the image shown, keeping the old image under unsampled pixels.
    pub fn show(&mut self, accumulation: &[u8], width: u32, height: u32) -> Result<&[u8], String> {
        if (width, height) != (self.width, self.height) {
            self.display = stretched_display(
                &self.display,
                self.width,
                self.height,
                width as usize,
                height as usize,
            );
            self.width = width;
            self.height = height;
        }
        decode_accumulation(accumulation, width, height, &mut self.display)?;
        Ok(&self.display)
    }
}

/// The part records of `rendering`'s look, built the way the pack builds them.
fn part_bytes(trace: &TraceScene, rendering: &(Look, RenderScene)) -> Vec<u8> {
    let mut bytes = Vec::with_capacity(2 * PART_STRIDE);
    for (index, surface) in trace.surfaces(&rendering.0).iter().enumerate() {
        let record = PartRecord {
            surface: SurfaceRecord::from_surface(surface),
            opacity: rendering.1.part_opacity(index).unwrap_or(1.0) as f32,
        };
        bytes.extend_from_slice(&record.to_le_bytes());
    }
    bytes
}

/// Writes an accumulation of `width` x `height` `vec4<f32>` (sum of RGB, counted samples; row
/// order from the bottom left) into RGBA8 `display` (top left first) with the library's display
/// code. A pixel with nothing counted keeps what `display` holds. `display` is `width * height * 4`
/// bytes.
pub fn decode_accumulation(
    accumulation: &[u8],
    width: u32,
    height: u32,
    display: &mut [u8],
) -> Result<(), String> {
    let (w, h) = (width as usize, height as usize);
    let expected = w * h * ACCUMULATION_STRIDE;
    if accumulation.len() != expected {
        return Err(format!(
            "The GPU returned {} bytes of accumulation and a {width} x {height} image is {expected}.",
            accumulation.len()
        ));
    }
    if display.len() != w * h * 4 {
        return Err("The image to draw is the wrong size for this accumulation.".to_string());
    }
    let word = |record: &[u8], at: usize| {
        f32::from_le_bytes([record[at], record[at + 1], record[at + 2], record[at + 3]])
    };
    for (index, record) in accumulation
        .as_chunks::<ACCUMULATION_STRIDE>()
        .0
        .iter()
        .enumerate()
    {
        let count = word(record, 12);
        if count <= 0.0 {
            continue;
        }
        let (x, y) = (index % w, h - 1 - index / w);
        let at = (y * w + x) * 4;
        for channel in 0..3 {
            let mean = f64::from(word(record, channel * 4)) / f64::from(count);
            display[at + channel] = display_byte(mean, EXPOSURE);
        }
        display[at + 3] = 255;
    }
    Ok(())
}
