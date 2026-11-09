// The entry point of the scattering check: for each case in a slice of the case buffer, runs
// `surface_evaluate`, `surface_pdf` and `surface_sample` (scatter.wgsl, which this file follows when
// `SCATTER_CHECK_WGSL` is concatenated) and writes one result. The demo's GPU page compares the
// results with the CPU model. The records are `scatter_shader.rs`'s, pinned by a test.
//
// Bindings: group 0: 0 the surfaces, 1 the cases, 2 the results (`read_write`, one per case),
// 3 a uniform `{ first: u32, count: u32 }`; group 1: the albedo table (scatter.wgsl).

struct ScatterCase {
    surface: u32,
    wo_x: f32,
    wo_y: f32,
    wo_z: f32,
    wi_x: f32,
    wi_y: f32,
    wi_z: f32,
    u0: f32,
    u1: f32,
    u2: f32,
}

struct ScatterResult {
    eval_r: f32,
    eval_g: f32,
    eval_b: f32,
    pdf: f32,
    sample_x: f32,
    sample_y: f32,
    sample_z: f32,
    sample_pdf: f32,
    weight_r: f32,
    weight_g: f32,
    weight_b: f32,
    flags: u32,
}

struct CheckSlice {
    first: u32,
    count: u32,
}

const SAMPLE_FOUND: u32 = 1u;
const SAMPLE_SPECULAR: u32 = 2u;

@group(0) @binding(0) var<storage, read> surfaces: array<Surface>;
@group(0) @binding(1) var<storage, read> cases: array<ScatterCase>;
@group(0) @binding(2) var<storage, read_write> results: array<ScatterResult>;
@group(0) @binding(3) var<uniform> check_slice: CheckSlice;

@compute @workgroup_size(64)
fn scatter_check_main(@builtin(global_invocation_id) id: vec3<u32>) {
    if id.x >= check_slice.count {
        return;
    }
    let index = check_slice.first + id.x;
    let case_ = cases[index];
    let s = surfaces[case_.surface];
    let wo = vec3<f32>(case_.wo_x, case_.wo_y, case_.wo_z);
    let wi = vec3<f32>(case_.wi_x, case_.wi_y, case_.wi_z);
    let value = surface_evaluate(s, wo, wi);
    let density = surface_pdf(s, wo, wi);
    let drawn = surface_sample(s, wo, vec3<f32>(case_.u0, case_.u1, case_.u2));
    var flags = 0u;
    if drawn.found {
        flags = SAMPLE_FOUND;
    }
    if drawn.is_specular {
        flags = flags | SAMPLE_SPECULAR;
    }
    results[index] = ScatterResult(
        value.x, value.y, value.z,
        density,
        drawn.direction.x, drawn.direction.y, drawn.direction.z,
        drawn.pdf,
        drawn.weight.x, drawn.weight.y, drawn.weight.z,
        flags,
    );
}
