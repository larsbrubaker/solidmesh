//! The GPU Ray Tracing page's render check: the scenes of `gpu_bench_render_scenes` rendered by
//! the WGSL path loop (`PATH_WGSL`, on the GPU by `demo/src/gpu-bench.ts`) and by the production
//! CPU `PathTracer` here, the same sample count each, and compared by `gpu_bench_render_compare`
//! at three scales of pixel group. The GPU's accumulation arrives as bytes of `vec4<f32>` per
//! pixel after each pass (sum and counted samples), so a pass's increment is the difference of
//! two snapshots, the same per-pass measurement the CPU's own samples give.
//!
//! **What the check can detect, measured** on an RTX 5070 at the fixed seed: the smallest bias each
//! check can tell from noise, as a fraction of the group's mean, for a difference that shifts one
//! channel. The whole-image figure is the worst channel at the pooled threshold; the region
//! figures are over every region and channel at the region threshold.
//!
//! ```text
//! scene                                samples  uniform  whole image  region median   p75    max
//! white furnaces (3, closed forms)          16     exact    exact        exact        exact  exact
//! diffuse box under the sun                 64     0.14%    0.42%        1.81%        3.11%  17.9%
//! glossy metal sphere                      384     0.37%    1.40%        3.13%        7.59%  23.6%
//! rough glass sphere                       384     0.12%    2.47%        1.35%        7.21%  78.5%
//! part at half opacity                      64     0.14%    0.42%        1.08%        2.69%  17.9%
//! colored-face part                         64     0.15%    0.51%        1.33%        3.63%  24.3%
//! perfect mirror sphere                    256     0.08%    7.69%        1.38%       37.7%   178%
//! smooth glass sphere                      256     0.09%    8.03%        0.58%        3.00%  278%
//! mirrored, non-uniform scale              64      0.21%    0.56%        1.68%        3.77%  17.9%
//! orthographic, wide image                  64     0.05%    0.18%        0.47%        1.05%  10.2%
//! next-event only, tall image               64     0.17%    0.27%        0.64%        1.20%  15.5%
//! surface sampling only                    256     1.64%    2.09%       16.9%        17.9%   19.5%
//! part 1e7 mm from the origin               64     0.14%    0.42%        1.81%        3.11%  17.9%
//! ```
//!
//! The uniform column is the weighted fit of one relative bias over the regions, and the most
//! sensitive check of a difference that reaches most of the picture: a uniform 0.1% is visible
//! in most scenes. A bias confined to the glossy or glass part alone shows only at region and line
//! scale, where the p75 and max columns are the honest figures. Measured with a shader that scaled
//! its sun term by 0.97, every scene the sun reaches by next-event estimation failed (the uniform
//! fit reached z = 78 on the metal sphere and 110 on rough glass), and the surface-sampling scene,
//! which never uses that term, passed as it should.
//!
//! A one-pixel-wide leak keeps its full size in its row or column (the line tier) instead of a
//! sixth of it in a region.

use js_sys::{Object, Uint8Array};
use solidmesh::mesh::ray_tracer::flat_tree::{
    albedo_table_bytes, GpuRenderScene, GpuSceneLimits, ACCUMULATION_STRIDE, PATH_ENTRY_POINT,
    PATH_WGSL, WEBGPU_DEFAULT_MAX_STORAGE_BINDING,
};
use solidmesh::mesh::ray_tracer::{display_byte, PathTracer, RenderWork};
use wasm_bindgen::prelude::*;

pub use crate::gpu_bench_render_compare::{
    compare_groups, with_dropped_samples, GroupLayout, PassGroups, RenderCheckResult,
    POOLED_Z_THRESHOLD, Z_THRESHOLD,
};
pub use crate::gpu_bench_render_scenes::{check_scene, CheckScene, SCENE_COUNT};

use crate::gpu_bench::fields;

/// The finest grid the check needs: a thousandth of a unit, far below a pixel of these scenes.
pub const CHECK_VERTEX_STEP: f64 = 1e-3;

/// One scene packed for the GPU.
pub fn packed(check: &CheckScene, max_binding: u64) -> Result<GpuRenderScene, String> {
    GpuRenderScene::pack(
        &check.scene,
        &check.camera,
        check.width,
        check.height,
        &check.settings,
        GpuSceneLimits {
            max_storage_binding: max_binding,
            max_vertex_step: CHECK_VERTEX_STEP,
        },
    )
    .map_err(|e| e.to_string())
}

/// A CPU reference render: per-pass group means and the final accumulation.
pub struct CpuRender {
    pub passes: PassGroups,
    pub sums: Vec<[f64; 4]>,
    pub dropped: u64,
}

/// A scene's group layout.
pub fn layout_of(check: &CheckScene) -> GroupLayout {
    GroupLayout {
        width: check.width,
        height: check.height,
    }
}

/// The CPU reference: `check.samples` passes of the production `PathTracer`.
pub fn render_cpu(check: &CheckScene) -> Result<CpuRender, String> {
    let tracer = PathTracer::new(
        &check.scene,
        check.camera,
        check.width,
        check.height,
        check.settings,
    )
    .map_err(|e| e.to_string())?;
    let layout = layout_of(check);
    let count = (check.width * check.height) as usize;
    let mut sums = vec![[0.0_f64; 4]; count];
    let mut passes: PassGroups = Vec::with_capacity(check.samples as usize);
    let mut work = RenderWork::default();
    let mut dropped = 0_u64;
    let mut pass_values = vec![[0.0_f64; 3]; count];
    for pass in 0..check.samples {
        for y in 0..check.height {
            for x in 0..check.width {
                let value = tracer
                    .sample_pixel(x, y, pass, &mut work)
                    .map_err(|e| e.to_string())?;
                let index = (y * check.width + x) as usize;
                if value.iter().all(|c| c.is_finite() && *c >= 0.0) {
                    pass_values[index] = value;
                    for (sum, channel) in sums[index].iter_mut().zip(value) {
                        *sum += channel;
                    }
                    sums[index][3] += 1.0;
                } else {
                    pass_values[index] = [0.0; 3];
                    dropped += 1;
                }
            }
        }
        passes.push(layout.aggregate(&pass_values));
    }
    Ok(CpuRender {
        passes,
        sums,
        dropped,
    })
}

/// One snapshot's little-endian `vec4<f32>` accumulation as doubles.
fn snapshot(bytes: &[u8]) -> Vec<[f64; 4]> {
    bytes
        .as_chunks::<ACCUMULATION_STRIDE>()
        .0
        .iter()
        .map(|record| {
            let word = |at: usize| {
                f64::from(f32::from_le_bytes([
                    record[at],
                    record[at + 1],
                    record[at + 2],
                    record[at + 3],
                ]))
            };
            [word(0), word(4), word(8), word(12)]
        })
        .collect()
}

/// A GPU render: per-pass group means, the final accumulation and how many samples it dropped.
pub struct GpuRender {
    pub passes: PassGroups,
    pub last: Vec<[f64; 4]>,
    pub dropped: u64,
}

/// Reads the GPU's snapshots, one after each pass, back into per-pass group means.
pub fn gpu_passes(
    snapshots: &[u8],
    layout: GroupLayout,
    samples: u32,
) -> Result<GpuRender, String> {
    let pixels = (layout.width * layout.height) as usize;
    let snapshot_bytes = pixels * ACCUMULATION_STRIDE;
    if snapshots.len() != snapshot_bytes * samples as usize {
        return Err(format!(
            "The GPU returned {} bytes of accumulation, and {samples} passes of {pixels} pixels \
             are {} bytes: run the check again.",
            snapshots.len(),
            snapshot_bytes * samples as usize
        ));
    }
    let mut passes: PassGroups = Vec::with_capacity(samples as usize);
    let mut previous = vec![[0.0_f64; 4]; pixels];
    let mut dropped = 0_u64;
    let mut last = previous.clone();
    let mut pass_values = vec![[0.0_f64; 3]; pixels];
    for chunk in snapshots.chunks(snapshot_bytes) {
        let current = snapshot(chunk);
        for index in 0..pixels {
            for channel in 0..3 {
                pass_values[index][channel] = current[index][channel] - previous[index][channel];
            }
            if current[index][3] == previous[index][3] {
                dropped += 1;
            }
        }
        passes.push(layout.aggregate(&pass_values));
        last = current.clone();
        previous = current;
    }
    Ok(GpuRender {
        passes,
        last,
        dropped,
    })
}

/// The comparison of one scene, given the GPU's cumulative snapshots.
pub fn check_render(index: u32, snapshots: &[u8]) -> Result<RenderCheckOutcome, String> {
    let check = check_scene(index)?;
    let layout = layout_of(&check);
    let gpu = gpu_passes(snapshots, layout, check.samples)?;
    let cpu = render_cpu(&check)?;
    let compared = compare_groups(&gpu.passes, &cpu.passes, &layout, check.closed_form);
    let mut result = with_dropped_samples(compared, gpu.dropped, cpu.dropped);
    result.name = check.name.to_string();
    Ok(RenderCheckOutcome {
        gpu_image: display_image(&gpu.last, layout),
        cpu_image: display_image(&cpu.sums, layout),
        result,
    })
}

/// A check's numbers and its two pictures.
pub struct RenderCheckOutcome {
    pub result: RenderCheckResult,
    pub gpu_image: Vec<u8>,
    pub cpu_image: Vec<u8>,
}

/// RGBA8, top row first, of the mean of an accumulation.
fn display_image(accumulation: &[[f64; 4]], layout: GroupLayout) -> Vec<u8> {
    let mut bytes = Vec::with_capacity(accumulation.len() * 4);
    for y in (0..layout.height).rev() {
        for x in 0..layout.width {
            let pixel = accumulation[(y * layout.width + x) as usize];
            let count = pixel[3].max(1.0);
            for channel in pixel.iter().take(3) {
                bytes.push(display_byte(*channel / count, 1.0));
            }
            bytes.push(255);
        }
    }
    bytes
}

/// GPU Ray Tracing page: the shader and buffers of render check scene `index`.
#[wasm_bindgen]
pub fn gpu_bench_render_prepare(index: u32, max_binding: f64) -> Result<Object, JsError> {
    let binding = if max_binding > 0.0 {
        // Bounded to the JS number's own integer range; a device limit is far below it.
        max_binding as u64
    } else {
        WEBGPU_DEFAULT_MAX_STORAGE_BINDING
    };
    let check = check_scene(index).map_err(|e| JsError::new(&e))?;
    let pack = packed(&check, binding).map_err(|e| JsError::new(&e))?;
    fields(&[
        ("shader", PATH_WGSL.into()),
        ("entryPoint", PATH_ENTRY_POINT.into()),
        (
            "nodes",
            Uint8Array::from(pack.tree().node_bytes().as_slice()).into(),
        ),
        (
            "triangles",
            Uint8Array::from(pack.tree().triangle_bytes().as_slice()).into(),
        ),
        (
            "faces",
            Uint8Array::from(pack.face_bytes().as_slice()).into(),
        ),
        (
            "parts",
            Uint8Array::from(pack.part_bytes().as_slice()).into(),
        ),
        (
            "scene",
            Uint8Array::from(pack.view_bytes().as_slice()).into(),
        ),
        (
            "srgb",
            Uint8Array::from(pack.srgb_table_bytes().as_slice()).into(),
        ),
        (
            "albedo",
            Uint8Array::from(albedo_table_bytes().as_slice()).into(),
        ),
        ("width", f64::from(check.width).into()),
        ("height", f64::from(check.height).into()),
        ("samples", f64::from(check.samples).into()),
        ("name", check.name.into()),
        ("zThreshold", Z_THRESHOLD.into()),
        ("pooledZThreshold", POOLED_Z_THRESHOLD.into()),
        ("sceneCount", f64::from(SCENE_COUNT).into()),
    ])
}

/// GPU Ray Tracing page: compares `snapshots` (the GPU's accumulation after each pass) with the
/// CPU reference, and answers the counts and the two pictures.
#[wasm_bindgen]
pub fn gpu_bench_render_check(index: u32, snapshots: &[u8]) -> Result<Object, JsError> {
    let outcome = check_render(index, snapshots).map_err(|e| JsError::new(&e))?;
    let result = outcome.result;
    fields(&[
        ("name", result.name.into()),
        ("passed", result.passed.into()),
        ("regions", f64::from(result.regions.groups).into()),
        ("exceeded", f64::from(result.regions.exceeded).into()),
        ("maxZ", result.regions.max_z.into()),
        ("worst", result.regions.worst.into()),
        ("lineGroups", f64::from(result.lines.groups).into()),
        ("linesExceeded", f64::from(result.lines.exceeded).into()),
        ("linesMaxZ", result.lines.max_z.into()),
        ("linesWorst", result.lines.worst.into()),
        ("pooledGroups", f64::from(result.pooled.groups).into()),
        ("pooledExceeded", f64::from(result.pooled.exceeded).into()),
        ("pooledMaxZ", result.pooled.max_z.into()),
        ("pooledWorst", result.pooled.worst.into()),
        ("uniformExceeded", f64::from(result.uniform.exceeded).into()),
        ("uniformMaxZ", result.uniform.max_z.into()),
        ("uniformWorst", result.uniform.worst.into()),
        (
            "meanRelativeDifference",
            result.mean_relative_difference.into(),
        ),
        ("gpuDropped", (result.gpu_dropped as f64).into()),
        ("cpuDropped", (result.cpu_dropped as f64).into()),
        (
            "closedFormExceeded",
            f64::from(result.closed_form_exceeded).into(),
        ),
        (
            "cpuClosedFormExceeded",
            f64::from(result.cpu_closed_form_exceeded).into(),
        ),
        ("detectableWhole", result.detectable.whole_image.into()),
        ("detectableUniform", result.detectable.uniform_bias.into()),
        (
            "detectableRegionMedian",
            result.detectable.region_median.into(),
        ),
        ("detectableRegionMax", result.detectable.region_max.into()),
        (
            "gpuImage",
            Uint8Array::from(outcome.gpu_image.as_slice()).into(),
        ),
        (
            "cpuImage",
            Uint8Array::from(outcome.cpu_image.as_slice()).into(),
        ),
    ])
}
