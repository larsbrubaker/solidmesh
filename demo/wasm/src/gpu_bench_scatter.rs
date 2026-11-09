//! The GPU Ray Tracing page's scattering check: a fixed batch of cases, every kind of surface the
//! CPU model has across a spread of roughness and view angles, run through the WGSL scattering
//! model (`SCATTER_CHECK_WGSL`, on the GPU by `demo/src/gpu-bench.ts`) and through the CPU model
//! here, reporting how many answers disagree beyond what `f32` allows. The same `Agreement` the
//! traversal check reports, with the case and both answers named for any disagreement.
//!
//! **Why this tolerance.** The shader is `f32`, the CPU model `f64`, so "agree" must mean "differ
//! by no more than `f32` arithmetic can explain". Two things bound that, and they are added:
//!
//! - The chain of arithmetic. A lobe value is about 30 `f32` operations, of which WGSL allows
//!   division 2.5 ULP, so about `40 * f32::EPSILON` accumulates at worst; the diffuse term's
//!   `1 - plastic` is near 0.0175 at grazing (index of refraction 4), amplifying that error about
//!   60 times. [`RELATIVE_TOLERANCE`] is `256 * f32::EPSILON`, that estimate with a factor of two
//!   of margin; [`DIRECTION_TOLERANCE`] is `64 * f32::EPSILON` for a unit direction, where nothing
//!   amplifies. [`ABSOLUTE_FLOOR`] covers values far below anything a path carries.
//! - How sharply the answer depends on its inputs, measured on the CPU model itself rather than
//!   assumed: each input component is nudged by [`NUDGE_ULPS`] (a shader's own direction carries
//!   about that much rounding after normalizing), and the change in each output is added up. This
//!   is where the narrow lobes (a half vector's cancellation) and glass near the critical angle
//!   (the square root at total internal reflection) ask for more room than the first bound, and
//!   only there, instead of a loose constant applied to every case. A direction's f32 *length*
//!   error does not show up in these nudges, because the CPU model renormalizes its inputs and a
//!   nudge's length change vanishes; the shader removes that error itself (`bisector`
//!   in `scatter.wgsl`), where a half vector would otherwise amplify it past any tolerance. The sampled direction is
//!   nudged separately ([`DRAWN_DIRECTION_STEP`]): its rounding is independent of `wo`, and a
//!   nudge of `wo` carries the sampled direction along with it.
//!
//! **What agreement does not mean here.** The two measurements above are added, so a case the CPU
//! model answers with extreme sensitivity to its inputs is checked very loosely, and at view cosines
//! at or below about 1e-3 on narrow lobes `evaluate` and `pdf` are effectively unchecked: on this
//! batch 276 of the 33000 outputs have a sensitivity above 10% of their value, 164 of them at those
//! views. That conditioning is real (a sample there depends on digits an f32 direction cannot
//! carry) and the report says so beside "0 disagreements" for the same reason.
//!
//! A nudge that makes the CPU model change its mind about whether a sample exists at all (a
//! direction rounding across the horizon) marks the case unstable, and a shader that takes the
//! other side there counts as a tie, not a disagreement; see `gpu_bench_compare`'s ties.

use js_sys::{Object, Uint8Array};
use solidmesh::math::Vector3;
use solidmesh::mesh::ray_tracer::flat_tree::{
    albedo_table_bytes, ScatterCheckCase, ScatterCheckResult, SurfaceRecord, SAMPLE_FOUND,
    SAMPLE_SPECULAR, SCATTER_CASE_STRIDE, SCATTER_CHECK_WGSL, SCATTER_RESULT_STRIDE,
    SURFACE_RECORD_STRIDE,
};
use solidmesh::mesh::ray_tracer::{ScatteringSample, ScatteringSurface};
use wasm_bindgen::prelude::*;

use crate::gpu_bench::fields;
use crate::gpu_bench_compare::Agreement;

/// See the module header.
pub const RELATIVE_TOLERANCE: f64 = 256.0 * f32::EPSILON as f64;

/// See the module header.
pub const DIRECTION_TOLERANCE: f64 = 64.0 * f32::EPSILON as f64;

/// See the module header.
pub const ABSOLUTE_FLOOR: f64 = 1e-12;

/// How far each input component is nudged to measure how sharply the answer depends on it.
pub const NUDGE_ULPS: i32 = 4;

/// The absolute step a drawn direction is nudged by, on each axis. Its rounding comes from
/// `2 (w . h) h - w`, whose terms are each near 1 even where the result's z is near 0, so its
/// error is about this big whatever the size of its own smallest component.
pub const DRAWN_DIRECTION_STEP: f64 = 4.0 * f32::EPSILON as f64;

/// How many disagreements a result lists.
const MAX_EXAMPLES: usize = 5;

/// The roughness spread of the batch: smooth, just under and just over the smooth threshold
/// (`sqrt(MIN_ALPHA)`), then narrow to fully rough.
const ROUGHNESSES: [f64; 9] = [0.0, 0.02, 0.0316, 0.0317, 0.05, 0.15, 0.35, 0.6, 1.0];

/// The view cosines of the batch: normal incidence, broad, grazing, near the horizon within and
/// beyond `MIN_COS`, and from below the surface (which a glass share sees).
const VIEW_COSINES: [f64; 9] = [1.0, 0.7, 0.3, 0.05, 1e-3, 1e-6, 1e-10, -0.7, -0.05];

/// An azimuth for the view, off any axis so no component is exactly zero by symmetry.
const VIEW_AZIMUTH: f64 = 0.7;

/// The seed of the fixed random inputs.
const RANDOM_SEED: u64 = 0x2545_f491_4f6c_dd1d;

/// One entry of the batch's surface list: a name for reports, and the surface.
#[derive(Debug, Clone)]
pub struct NamedSurface {
    pub name: String,
    pub surface: ScatteringSurface,
}

/// The fixed batch of the check.
#[derive(Debug, Clone)]
pub struct ScatterBatch {
    pub surfaces: Vec<NamedSurface>,
    pub cases: Vec<ScatterCheckCase>,
}

/// The CPU model's answer to one case, in `f64`.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct CpuAnswer {
    pub evaluate: [f64; 3],
    pub pdf: f64,
    pub sample: Option<ScatteringSample>,
}

/// How much of a difference [`CpuAnswer`]'s outputs may show for one case from `f32` rounding of
/// its inputs alone.
#[derive(Debug, Clone, Copy, Default, PartialEq)]
pub struct Sensitivity {
    pub evaluate: [f64; 3],
    pub pdf: f64,
    pub sample_pdf: f64,
    pub sample_weight: [f64; 3],
    pub sample_direction: [f64; 3],
    /// A nudge decided whether a sample exists, or which kind it is.
    pub unstable: bool,
}

fn vector(v: [f32; 3]) -> Vector3 {
    Vector3::new(f64::from(v[0]), f64::from(v[1]), f64::from(v[2]))
}

/// The CPU model's answer to `case` on `surface`, its inputs the same `f32`s the shader reads.
pub fn cpu_answer(surface: &ScatteringSurface, case: &ScatterCheckCase) -> CpuAnswer {
    let wo = vector(case.wo);
    let wi = vector(case.wi);
    CpuAnswer {
        evaluate: surface.evaluate(wo, wi),
        pdf: surface.pdf(wo, wi),
        sample: surface.sample(wo, case.u.map(f64::from)),
    }
}

/// `value` moved `ulps` steps along the `f32` grid, away from zero when positive; zero stays
/// zero, since a component that is exactly zero stays exactly zero in the shader as well.
fn nudged(value: f32, ulps: i32) -> f32 {
    if value == 0.0 {
        return value;
    }
    let bits = value.abs().to_bits();
    let moved = if ulps >= 0 {
        bits + ulps.unsigned_abs()
    } else {
        bits - ulps.unsigned_abs()
    };
    f32::from_bits(moved).copysign(value)
}

/// `case` with input `component` (0 to 2 `wo`, 3 to 5 `wi`, 6 to 8 the random numbers) nudged.
fn with_nudge(case: &ScatterCheckCase, component: usize, ulps: i32) -> ScatterCheckCase {
    let mut nudged_case = *case;
    let slot = match component {
        0..=2 => &mut nudged_case.wo[component],
        3..=5 => &mut nudged_case.wi[component - 3],
        _ => &mut nudged_case.u[component - 6],
    };
    *slot = nudged(*slot, ulps);
    nudged_case
}

fn max_assign(sum: &mut f64, value: f64) {
    if value.is_finite() && value > *sum {
        *sum = value;
    }
}

/// Measures [`Sensitivity`] for one case on the CPU model.
pub fn sensitivity(
    surface: &ScatteringSurface,
    case: &ScatterCheckCase,
    base: &CpuAnswer,
) -> Sensitivity {
    let mut total = Sensitivity::default();
    for component in 0..9 {
        let mut local = Sensitivity::default();
        for ulps in [NUDGE_ULPS, -NUDGE_ULPS] {
            let moved = cpu_answer(surface, &with_nudge(case, component, ulps));
            for channel in 0..3 {
                max_assign(
                    &mut local.evaluate[channel],
                    (moved.evaluate[channel] - base.evaluate[channel]).abs(),
                );
            }
            max_assign(&mut local.pdf, (moved.pdf - base.pdf).abs());
            match (&moved.sample, &base.sample) {
                (Some(a), Some(b)) if a.is_specular == b.is_specular => {
                    max_assign(&mut local.sample_pdf, (a.pdf - b.pdf).abs());
                    for channel in 0..3 {
                        max_assign(
                            &mut local.sample_weight[channel],
                            (a.weight[channel] - b.weight[channel]).abs(),
                        );
                    }
                    max_assign(
                        &mut local.sample_direction[0],
                        (a.direction.x - b.direction.x).abs(),
                    );
                    max_assign(
                        &mut local.sample_direction[1],
                        (a.direction.y - b.direction.y).abs(),
                    );
                    max_assign(
                        &mut local.sample_direction[2],
                        (a.direction.z - b.direction.z).abs(),
                    );
                }
                (None, None) => {}
                _ => total.unstable = true,
            }
        }
        for channel in 0..3 {
            total.evaluate[channel] += local.evaluate[channel];
            total.sample_weight[channel] += local.sample_weight[channel];
            total.sample_direction[channel] += local.sample_direction[channel];
        }
        total.pdf += local.pdf;
        total.sample_pdf += local.sample_pdf;
    }
    if let Some(sample) = &base.sample {
        add_drawn_direction_sensitivity(surface, case, sample, &mut total);
    }
    total
}

/// Adds how much a sample's pdf and weight move when the sampled direction is nudged on its own.
/// A sample's `f32` direction rounds independently of `wo`, while a nudge of `wo` moves the
/// sampled direction with it; only this measures the rounding of the direction a narrow lobe
/// must then find `h` from, where `wo + wi` nearly cancels.
fn add_drawn_direction_sensitivity(
    surface: &ScatteringSurface,
    case: &ScatterCheckCase,
    sample: &ScatteringSample,
    total: &mut Sensitivity,
) {
    if sample.is_specular {
        return;
    }
    let wo = vector(case.wo);
    let direction = [sample.direction.x, sample.direction.y, sample.direction.z];
    let base_value = surface.evaluate(wo, sample.direction);
    let base_pdf = surface.pdf(wo, sample.direction);
    let weight_of = |value: [f64; 3], pdf: f64, cosine: f64| value.map(|f| f * cosine / pdf);
    let base_weight = weight_of(base_value, base_pdf, sample.direction.z.abs());
    for axis in 0..3 {
        let mut local_pdf = 0.0_f64;
        let mut local_weight = [0.0_f64; 3];
        for step in [DRAWN_DIRECTION_STEP, -DRAWN_DIRECTION_STEP] {
            let mut moved = direction;
            moved[axis] += step;
            let moved = Vector3::new(moved[0], moved[1], moved[2]);
            let pdf = surface.pdf(wo, moved);
            max_assign(&mut local_pdf, (pdf - base_pdf).abs());
            if pdf > 0.0 {
                let weight = weight_of(surface.evaluate(wo, moved), pdf, moved.z.abs());
                for channel in 0..3 {
                    max_assign(
                        &mut local_weight[channel],
                        (weight[channel] - base_weight[channel]).abs(),
                    );
                }
            } else {
                total.unstable = true;
            }
        }
        total.sample_pdf += local_pdf;
        for (total_channel, local_channel) in total.sample_weight.iter_mut().zip(local_weight) {
            *total_channel += local_channel;
        }
    }
}

/// How the shader's answer to one case compares with the CPU's.
#[derive(Debug, Clone, PartialEq)]
pub enum ScatterMatch {
    Agree,
    /// The shader and the CPU differ only on whether a sample exists, on a case where the CPU
    /// model itself changes its mind for a nudge of its inputs.
    Tie,
    Differ(String),
}

fn within(reference: f64, shader: f64, sensitive: f64) -> bool {
    (reference - shader).abs()
        <= RELATIVE_TOLERANCE * reference.abs().max(shader.abs()) + ABSOLUTE_FLOOR + sensitive
}

/// The shader's answer `result` against the CPU's `answer` for `case`.
pub fn classify(
    case: &ScatterCheckCase,
    answer: &CpuAnswer,
    margin: &Sensitivity,
    result: &ScatterCheckResult,
) -> ScatterMatch {
    let widen3 = |v: [f32; 3]| [f64::from(v[0]), f64::from(v[1]), f64::from(v[2])];
    let shader_evaluate = widen3(result.evaluate);
    for (channel, shader_value) in shader_evaluate.iter().enumerate() {
        if !within(
            answer.evaluate[channel],
            *shader_value,
            margin.evaluate[channel],
        ) {
            return ScatterMatch::Differ(format!(
                "evaluate channel {channel}: CPU {}, GPU {shader_value}",
                answer.evaluate[channel]
            ));
        }
    }
    if !within(answer.pdf, f64::from(result.pdf), margin.pdf) {
        return ScatterMatch::Differ(format!("pdf: CPU {}, GPU {}", answer.pdf, result.pdf));
    }
    let found = result.flags & SAMPLE_FOUND != 0;
    let is_specular = result.flags & SAMPLE_SPECULAR != 0;
    match (&answer.sample, found) {
        (None, false) => ScatterMatch::Agree,
        (Some(cpu), true) => {
            if cpu.is_specular != is_specular {
                return if margin.unstable {
                    ScatterMatch::Tie
                } else {
                    ScatterMatch::Differ(format!(
                        "sample kind: CPU is_specular {}, GPU {is_specular}",
                        cpu.is_specular
                    ))
                };
            }
            let direction = widen3(result.sample_direction);
            let reference = [cpu.direction.x, cpu.direction.y, cpu.direction.z];
            for axis in 0..3 {
                if (reference[axis] - direction[axis]).abs()
                    > DIRECTION_TOLERANCE + margin.sample_direction[axis]
                {
                    return ScatterMatch::Differ(format!(
                        "sample direction axis {axis}: CPU {}, GPU {}",
                        reference[axis], direction[axis]
                    ));
                }
            }
            if !within(cpu.pdf, f64::from(result.sample_pdf), margin.sample_pdf) {
                return ScatterMatch::Differ(format!(
                    "sample pdf: CPU {}, GPU {}",
                    cpu.pdf, result.sample_pdf
                ));
            }
            let weight = widen3(result.sample_weight);
            for (channel, shader_weight) in weight.iter().enumerate() {
                if !within(
                    cpu.weight[channel],
                    *shader_weight,
                    margin.sample_weight[channel],
                ) {
                    return ScatterMatch::Differ(format!(
                        "sample weight channel {channel}: CPU {}, GPU {shader_weight}",
                        cpu.weight[channel]
                    ));
                }
            }
            ScatterMatch::Agree
        }
        _ if margin.unstable => ScatterMatch::Tie,
        (cpu, gpu) => ScatterMatch::Differ(format!(
            "sample: CPU found {}, GPU found {gpu} (case wo {:?})",
            cpu.is_some(),
            case.wo
        )),
    }
}

/// The CPU's answer as the record a shader would write, narrowed to `f32`.
pub fn result_of(answer: &CpuAnswer) -> ScatterCheckResult {
    let narrow = |v: [f64; 3]| [v[0] as f32, v[1] as f32, v[2] as f32];
    let mut result = ScatterCheckResult {
        evaluate: narrow(answer.evaluate),
        pdf: answer.pdf as f32,
        ..ScatterCheckResult::default()
    };
    if let Some(sample) = &answer.sample {
        result.sample_direction =
            narrow([sample.direction.x, sample.direction.y, sample.direction.z]);
        result.sample_pdf = sample.pdf as f32;
        result.sample_weight = narrow(sample.weight);
        result.flags = SAMPLE_FOUND
            | if sample.is_specular {
                SAMPLE_SPECULAR
            } else {
                0
            };
    }
    result
}

fn surfaces() -> Vec<NamedSurface> {
    let colored = [0.8, 0.3, 0.1];
    let mut surfaces = vec![
        NamedSurface {
            name: "matte plastic preset".to_string(),
            surface: ScatteringSurface::matte_plastic(colored),
        },
        NamedSurface {
            name: "glossy plastic preset".to_string(),
            surface: ScatteringSurface::glossy_plastic(colored),
        },
        NamedSurface {
            name: "brushed metal preset".to_string(),
            surface: ScatteringSurface::brushed_metal(colored),
        },
        NamedSurface {
            name: "polished metal preset".to_string(),
            surface: ScatteringSurface::polished_metal(colored),
        },
        NamedSurface {
            name: "glass preset".to_string(),
            surface: ScatteringSurface::glass(),
        },
        NamedSurface {
            name: "rubber preset".to_string(),
            surface: ScatteringSurface::rubber(colored),
        },
    ];
    // Base color, metallic, transmission, index of refraction: each row is a kind of surface.
    let kinds: [(&str, [f64; 3], f64, f64, f64); 16] = [
        ("colored metal", colored, 1.0, 0.0, 1.5),
        ("white metal", [1.0, 1.0, 1.0], 1.0, 0.0, 1.5),
        ("colored plastic", colored, 0.0, 0.0, 1.5),
        ("black plastic", [0.0, 0.0, 0.0], 0.0, 0.0, 1.5),
        ("diffuse, no interface (ior 1)", colored, 0.0, 0.0, 1.0),
        ("plastic, ior 4", colored, 0.0, 0.0, 4.0),
        ("clear glass", [1.0, 1.0, 1.0], 0.0, 1.0, 1.5),
        ("tinted glass", colored, 0.0, 1.0, 1.5),
        ("glass, ior 2.4", [1.0, 1.0, 1.0], 0.0, 1.0, 2.4),
        ("glass, ior 4", [1.0, 1.0, 1.0], 0.0, 1.0, 4.0),
        (
            "index-matched glass, ior 1.0005",
            [1.0, 1.0, 1.0],
            0.0,
            1.0,
            1.0005,
        ),
        ("glass, ior 1", [1.0, 1.0, 1.0], 0.0, 1.0, 1.0),
        ("glass, ior 1.0011", [1.0, 1.0, 1.0], 0.0, 1.0, 1.0011),
        ("glass, ior 1.002", [1.0, 1.0, 1.0], 0.0, 1.0, 1.002),
        ("glass, ior 1.03", [1.0, 1.0, 1.0], 0.0, 1.0, 1.03),
        ("metal, plastic and glass mixed", colored, 0.3, 0.5, 1.5),
    ];
    for (name, base, metallic, transmission, ior) in kinds {
        for roughness in ROUGHNESSES {
            surfaces.push(NamedSurface {
                name: format!("{name}, roughness {roughness}"),
                surface: ScatteringSurface::new(base, metallic, roughness, transmission, ior),
            });
        }
    }
    surfaces
}

fn direction_at(cosine: f64, azimuth: f64) -> [f32; 3] {
    let sine = (1.0 - cosine * cosine).max(0.0).sqrt();
    [
        (sine * azimuth.cos()) as f32,
        (sine * azimuth.sin()) as f32,
        cosine as f32,
    ]
}

fn next_random(state: &mut u64) -> f32 {
    *state = state.wrapping_mul(0x5851_f42d_4c95_7f2d).wrapping_add(1);
    // In [0, 1) on the f32 grid. `as` is exact for 23 bits.
    ((*state >> 40) as f32) / (1u32 << 24) as f32
}

/// The batch: for every surface, every view cosine, six light directions (the mirror, near the
/// mirror, along the normal, off the lobe, through the surface, near the horizon) and fixed
/// random numbers, plus a view of no length.
pub fn standard_batch() -> ScatterBatch {
    let surfaces = surfaces();
    let mut cases = Vec::new();
    let mut state = RANDOM_SEED;
    for (index, _) in surfaces.iter().enumerate() {
        let surface = index as u32;
        let mut push = |wo: [f32; 3], wi: [f32; 3], state: &mut u64| {
            let u = [next_random(state), next_random(state), next_random(state)];
            cases.push(ScatterCheckCase { surface, wo, wi, u });
        };
        for view_cos in VIEW_COSINES {
            let wo = direction_at(view_cos, VIEW_AZIMUTH);
            let lights: [[f32; 3]; 6] = [
                [-wo[0], -wo[1], wo[2]],
                {
                    let near = direction_at(view_cos * 0.98, VIEW_AZIMUTH + std::f64::consts::PI);
                    [near[0], near[1], view_cos.abs().max(0.01) as f32]
                },
                [0.0, 0.0, 1.0],
                direction_at(0.4, 2.1),
                [-wo[0], -wo[1], -wo[2]],
                direction_at(1e-3, 1.0),
            ];
            for wi in lights {
                push(wo, wi, &mut state);
            }
        }
        push([0.0, 0.0, 0.0], [0.0, 0.0, 1.0], &mut state);
    }
    ScatterBatch { surfaces, cases }
}

impl ScatterBatch {
    /// The surface buffer.
    pub fn surface_bytes(&self) -> Vec<u8> {
        let mut bytes = Vec::with_capacity(self.surfaces.len() * SURFACE_RECORD_STRIDE);
        for named in &self.surfaces {
            bytes.extend_from_slice(&SurfaceRecord::from_surface(&named.surface).to_le_bytes());
        }
        bytes
    }

    /// The case buffer.
    pub fn case_bytes(&self) -> Vec<u8> {
        let mut bytes = Vec::with_capacity(self.cases.len() * SCATTER_CASE_STRIDE);
        for case in &self.cases {
            bytes.extend_from_slice(&case.to_le_bytes());
        }
        bytes
    }

    /// The shader's results (one [`SCATTER_RESULT_STRIDE`]-byte record per case) against the CPU
    /// model.
    pub fn compare(&self, results: &[u8]) -> Result<Agreement, String> {
        if results.len() != self.cases.len() * SCATTER_RESULT_STRIDE {
            return Err(format!(
                "The GPU returned {} bytes of scattering results for {} cases, and each result \
                 is {SCATTER_RESULT_STRIDE} bytes: run the benchmark again.",
                results.len(),
                self.cases.len()
            ));
        }
        let mut agreement = Agreement {
            compared: self.cases.len() as u64,
            exact: 0,
            ties: 0,
            disagreed: 0,
            examples: Vec::new(),
        };
        for (index, case) in self.cases.iter().enumerate() {
            let mut record = [0u8; SCATTER_RESULT_STRIDE];
            record.copy_from_slice(
                &results[index * SCATTER_RESULT_STRIDE..(index + 1) * SCATTER_RESULT_STRIDE],
            );
            let result = ScatterCheckResult::from_le_bytes(&record);
            let surface = &self.surfaces[case.surface as usize];
            let answer = cpu_answer(&surface.surface, case);
            let margin = sensitivity(&surface.surface, case, &answer);
            match classify(case, &answer, &margin, &result) {
                ScatterMatch::Agree => agreement.exact += 1,
                ScatterMatch::Tie => agreement.ties += 1,
                ScatterMatch::Differ(wording) => {
                    agreement.disagreed += 1;
                    if agreement.examples.len() < MAX_EXAMPLES {
                        agreement.examples.push(format!(
                            "case {index} ({}, wo {:?}, wi {:?}, u {:?}): {wording}",
                            surface.name, case.wo, case.wi, case.u
                        ));
                    }
                }
            }
        }
        Ok(agreement)
    }
}

/// GPU Ray Tracing page: the scattering check's shader and buffers, built on this worker.
#[wasm_bindgen]
pub fn gpu_bench_scatter_prepare() -> Result<Object, JsError> {
    let batch = standard_batch();
    fields(&[
        ("shader", SCATTER_CHECK_WGSL.into()),
        (
            "surfaces",
            Uint8Array::from(batch.surface_bytes().as_slice()).into(),
        ),
        (
            "cases",
            Uint8Array::from(batch.case_bytes().as_slice()).into(),
        ),
        (
            "albedo",
            Uint8Array::from(albedo_table_bytes().as_slice()).into(),
        ),
        ("caseCount", (batch.cases.len() as f64).into()),
    ])
}

/// GPU Ray Tracing page: compares the GPU's scattering results with the CPU model and answers the
/// counts.
#[wasm_bindgen]
pub fn gpu_bench_scatter_compare(results: &[u8]) -> Result<Object, JsError> {
    let agreement = standard_batch()
        .compare(results)
        .map_err(|e| JsError::new(&e))?;
    fields(&[
        ("compared", (agreement.compared as f64).into()),
        ("exact", (agreement.exact as f64).into()),
        ("ties", (agreement.ties as f64).into()),
        ("disagreed", (agreement.disagreed as f64).into()),
        ("examples", agreement.examples.join("\n").into()),
    ])
}
