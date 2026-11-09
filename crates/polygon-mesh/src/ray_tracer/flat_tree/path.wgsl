// The path loop in WGSL: one sample per pixel of a pixel range, added into an RGBA accumulation
// buffer, the same estimator `path_tracer/integrator.rs` computes on the CPU. It follows
// `trace.wgsl` and `scatter.wgsl` (`PATH_WGSL` is the three concatenated); `gpu_scene.rs` packs the
// buffers and pins these records. Bindings:
//
//   group 0: 0 the tree's nodes, 1 its triangles (`trace.wgsl`)
//   group 1: 0 the albedo table (`scatter.wgsl`)
//   group 2: 0 `faces`, 1 `parts`, 2 `accumulation` (`read_write`), 3 `path_view` (uniform,
//            the only buffer a change of view writes), 4 `path_slice` (uniform: `{ first_pixel,
//            pixel_count, sample_index }`), 5 `channels` (`read_write`, the coverage channels),
//            6 `path_srgb` (uniform, fixed at pack)
//
// Entry point `path_trace_main`, workgroup size 64: invocation `id.x` handles pixel
// `first_pixel + id.x` (row order from the bottom left, `y * width + x`), so a caller time-slices by
// choosing the range and dispatching `ceil(count / 64)` workgroups. Limits used: 7 storage buffers
// in the compute stage, 3 uniform buffers, 3 bind groups, 64 invocations per workgroup, about
// 2.1 KB of private memory and 1.2 KB of function memory per invocation. WebGPU's defaults are 8, 12, 4,
// 256 and the 8 KB its compilers must accept, so nothing here asks for more than a phone gives.
//
// The color and the coverage channels are two buffers and not one 48-byte record: a browser's
// default 128 MB storage binding holds a 1440p image's 16-byte colors and 32-byte channels apart,
// where one record of 48 bytes would stop at 1080p.
//
// The accumulation record is `vec4<f32>(sum of the counted samples' linear RGB, how many counted)`:
// the sum divided by the count is the pixel's mean. A sample is dropped, its pass still spent, when
// it is not finite, negative, or would overflow the running sum, as `AccumulatedPixel::add_sample`
// does. WGSL lets a compiler assume no float is NaN or infinite, so "finite" is read from the bits
// (`path_is_finite`), which no such assumption may change. A dispatch covers each pixel of its range
// exactly once, and a caller gives every pixel its samples in order; unlike the CPU's surface, the
// buffer cannot refuse a repeat.
//
// Where this differs from `integrator.rs`, and why, in f32 terms:
//
// - The random stream is a 32-bit LCG per sample with an output permutation and a per-stream
//   increment, keyed by a hash of the seed, pixel and sample index: a pure function of those, as
//   on the CPU, but a different stream, since WGSL has no 64-bit integer to run SplitMix64.
// - The pixel jitter is the same sequence (steps of 1 and 1 over the plastic number squared) in
//   32-bit fixed point rather than f64, so it stays uniform for any sample index: `index / p` in
//   f32 runs out of fractional bits within a few thousand samples.
// - Every position here is relative to the scene's center (`gpu_scene.rs`), which keeps a part far
//   from the world origin precise. The offset a continuing ray starts by is the triangle's largest
//   L1 magnitude in those coordinates times `SURFACE_OFFSET_F32_STEPS` f32 steps, stored per face.
//   The CPU offset is sized in the mesh's own space because its triangles are tested there; here
//   every triangle is tested in the scene's space narrowed to f32, so a part far from the scene's
//   center has a proportionally larger offset, and a very thin wall out there can leak where the
//   CPU's cannot. `GpuSceneLimits::max_vertex_step` bounds how far that can go.
// - Face colors arrive decoded through `path_scene`'s table of every 8-bit value, the same f64
//   decoding `RenderScene` holds, narrowed once.
//
// Everything else is the CPU loop in the CPU's order. A see-through part's verdict comes first,
// then the bounce limit, then the sun, then the sampled bounce, the leak rejections, roulette and
// the next ray, and every value the CPU takes as a `max`, `min` or density guard is guarded the
// same way (see `path_power_heuristic` for the one guard WGSL needs beyond it). The loop is a
// `loop` with bounded iterations: each ends by escape, a stop, the bounce limit, or the per-line
// crossing cap of `MAX_CROSSINGS_PER_LINE`, and each shadow ray likewise.

// ---- Constants, equal to their Rust counterparts in `gpu_scene.rs` and `see_through.rs`.

const PATH_FACE_BOTH: u32 = 3u;
const PATH_FAR: f32 = 3.0e38f;
const PATH_ROULETTE_OFF: u32 = 0xFFFFFFFFu;
const PATH_MAX_SURVIVAL: f32 = 0.95f;
// The most surfaces of see-through parts one straight line goes through.
const MAX_CROSSINGS_PER_LINE: u32 = 512u;
// The sample index of the stream the pixel jitter's offset comes from, as on the CPU.
const PATH_JITTER_STREAM: u32 = 0xFFFFFFFFu;
// 2^-24 exactly: a 24-bit integer times this is a float in [0, 1) with no rounding.
const PATH_UNIT_FROM_24_BITS: f32 = 5.9604644775390625e-8f;
// `2^32 / plastic` and `2^32 / plastic^2`, the plastic-number jitter step in fixed point.
const PATH_JITTER_STEP_X: u32 = 0xC13FA9A9u;
const PATH_JITTER_STEP_Y: u32 = 0x91E10DA5u;

// `LightStrategy`, as `PathScene.strategy` carries it.
const PATH_STRATEGY_COMBINED: u32 = 0u;
const PATH_STRATEGY_NEXT_EVENT_ONLY: u32 = 1u;
const PATH_STRATEGY_SURFACE_ONLY: u32 = 2u;

// `PathScene.flags`.
const PATH_FLAG_ORTHOGRAPHIC: u32 = 1u;
const PATH_FLAG_SEE_THROUGH: u32 = 2u;
const PATH_FLAG_SUN: u32 = 4u;
const PATH_FLAG_GRADIENT: u32 = 8u;
const PATH_FLAG_COMPOSITE: u32 = 16u;
const PATH_FLAG_FLOOR: u32 = 32u;

// What a line does at a hit.
const LINE_STOPS: u32 = 0u;
const LINE_PASSES: u32 = 1u;
const LINE_ENDED: u32 = 2u;

// ---- The records `gpu_scene.rs` writes.

// One per face, indexed by a hit's `source_index`: the corner normals for shading, not unit
// length (see `gpu_scene.rs`), the world geometric normal, the continuing ray's offset, which part
// the face belongs to, and its color as `0xBBGGRR` bytes.
struct Face {
    n0: array<f32, 3>,
    n1: array<f32, 3>,
    n2: array<f32, 3>,
    geometric: array<f32, 3>,
    ray_offset: f32,
    part: u32,
    color: u32,
}

struct Part {
    surface: Surface,
    opacity: f32,
}

struct PathView {
    // xyz the eye, w `tan(fov / 2)` or the orthographic half extent.
    eye: vec4<f32>,
    // Each xyz, w unused.
    forward: vec4<f32>,
    right: vec4<f32>,
    up: vec4<f32>,
    // The image's size in pixels: which pixel an index means.
    width: u32,
    height: u32,
    max_bounces: u32,
    roulette_after: u32,
    seed_low: u32,
    seed_high: u32,
    strategy: u32,
    flags: u32,
    // A constant environment repeats its one radiance in all three.
    env_zenith: vec4<f32>,
    env_horizon: vec4<f32>,
    env_ground: vec4<f32>,
    sun_direction: vec4<f32>, // w: the versine `1 - cos(radius)`.
    sun_tangent: vec4<f32>,   // w: the sun's density, one over its solid angle.
    sun_bitangent: vec4<f32>, // w: the squared chord of the disc's edge.
    sun_radiance: vec4<f32>,
    // The size the camera measures its projection against (`GpuView::trace`).
    trace_width: u32,
    trace_height: u32,
    floor_surface_flags: u32,
    floor_show_through: f32,
    // xy the floor's center, z its height, w its half extent, relative to the scene's center.
    floor_plane: vec4<f32>,
    floor_base: vec4<f32>,
    floor_rest: vec4<f32>,
}

struct PathSrgb {
    to_linear: array<vec4<f32>, 64>,
}

struct PathSlice {
    first_pixel: u32,
    pixel_count: u32,
    sample_index: u32,
}

// The coverage channels of a composited pixel, summed over its samples.
struct PathChannels {
    // xyz the share of its color the sky would have brought had the line gone through every part;
    // w the coverage of the samples that counted in `accumulation`.
    backdrop: vec4<f32>,
    // x the coverage of every sample, y the sun's loss on the floor, z the sky's, w how many samples.
    coverage: vec4<f32>,
}

@group(2) @binding(0) var<storage, read> faces: array<Face>;
@group(2) @binding(1) var<storage, read> parts: array<Part>;
@group(2) @binding(2) var<storage, read_write> accumulation: array<vec4<f32>>;
@group(2) @binding(3) var<uniform> path_view: PathView;
@group(2) @binding(4) var<uniform> path_slice: PathSlice;
@group(2) @binding(5) var<storage, read_write> channels: array<PathChannels>;
@group(2) @binding(6) var<uniform> path_srgb: PathSrgb;

// ---- The sample's random stream and the parts one straight line has gone through. Private, not
// passed around, because `surface_sample` and the shadow ray both need them in the same order.

var<private> path_rng_state: u32;
var<private> path_rng_increment: u32;

fn path_mix(value: u32) -> u32 {
    let state = value * 747796405u + 2891336453u;
    let word = ((state >> ((state >> 28u) + 4u)) ^ state) * 277803737u;
    return (word >> 22u) ^ word;
}

// The stream of `sample_index` at pixel (x, y): the seed's two words and the three integers folded
// through the mixer, one at a time, so neighbouring pixels and samples start unrelated.
fn path_rng_start(x: u32, y: u32, sample_index: u32) {
    path_rng_start_salted(0u, 0u, x, y, sample_index);
}

// As `path_rng_start`, with the seed's two words first XORed with a salt: a stream unrelated to the
// unsalted one for the same pixel and sample.
fn path_rng_start_salted(salt_low: u32, salt_high: u32, x: u32, y: u32, sample_index: u32) {
    var state = path_mix(path_view.seed_low ^ salt_low);
    state = path_mix(state ^ (path_view.seed_high ^ salt_high));
    state = path_mix(state ^ x);
    state = path_mix(state ^ y);
    state = path_mix(state ^ sample_index);
    path_rng_state = state;
    path_rng_increment = path_mix(state ^ 0x9E3779B9u) | 1u;
}

fn path_next_bits() -> u32 {
    path_rng_state = path_rng_state * 747796405u + path_rng_increment;
    let state = path_rng_state;
    let word = ((state >> ((state >> 28u) + 4u)) ^ state) * 277803737u;
    return (word >> 22u) ^ word;
}

// A number in `[0, 1)`: 24 bits fit an f32 exactly, so the conversion cannot round up to 1.
fn path_next_unit() -> f32 {
    return f32(path_next_bits() >> 8u) * PATH_UNIT_FROM_24_BITS;
}

fn path_next_unit3() -> vec3<f32> {
    let a = path_next_unit();
    let b = path_next_unit();
    let c = path_next_unit();
    return vec3<f32>(a, b, c);
}

// One straight line's crossings (`see_through.rs`'s `StraightLine`). A path and the shadow rays
// from its vertices never use this at the same time: a shadow ray runs between the verdict that
// ended the last straight line and the restart for the next.
var<private> line_passed: array<u32, MAX_CROSSINGS_PER_LINE>;
var<private> line_passed_count: u32;
var<private> line_crossings: u32;

fn line_restart() {
    line_passed_count = 0u;
    line_crossings = 0u;
}

fn line_has_passed(part: u32) -> bool {
    for (var index = 0u; index < line_passed_count; index += 1u) {
        if line_passed[index] == part {
            return true;
        }
    }
    return false;
}

// `StraightLine::at_hit`: whether the part is there, deciding once per part along the line.
fn line_at_hit(part: u32) -> u32 {
    let opacity = parts[part].opacity;
    if opacity >= 1.0f {
        return LINE_STOPS;
    }
    if line_crossings >= MAX_CROSSINGS_PER_LINE {
        return LINE_ENDED;
    }
    line_crossings += 1u;
    if line_has_passed(part) {
        return LINE_PASSES;
    }
    line_passed[line_passed_count] = part;
    line_passed_count += 1u;
    if opacity > 0.0f && path_next_unit() < opacity {
        line_passed_count -= 1u;
        return LINE_STOPS;
    }
    return LINE_PASSES;
}

// ---- A point on a surface (`surface_point.rs`, `placed_part.rs`).

struct PathPoint {
    position: vec3<f32>,
    geometric: vec3<f32>,
    shading: vec3<f32>,
    ray_offset: f32,
    part: u32,
    color: u32,
}

fn path_vec3(values: array<f32, 3>) -> vec3<f32> {
    return vec3<f32>(values[0], values[1], values[2]);
}

// The point is rebuilt from the triangle's corners, on the triangle's own plane, as on the CPU.
fn path_surface_point(hit: Hit) -> PathPoint {
    let face = faces[hit.source_index];
    let triangle = triangles[hit.triangle];
    let position = path_vec3(triangle.v0) * hit.u + path_vec3(triangle.v1) * hit.v
        + path_vec3(triangle.v2) * hit.w;
    let geometric = path_vec3(face.geometric);
    let mixed = path_vec3(face.n0) * hit.u + path_vec3(face.n1) * hit.v + path_vec3(face.n2) * hit.w;
    var shading = unit_direction(mixed);
    if !has_direction(shading) {
        shading = geometric;
    }
    return PathPoint(position, geometric, shading, face.ray_offset, face.part, face.color);
}

struct PathFacing {
    geometric: vec3<f32>,
    shading: vec3<f32>,
    back_face: bool,
}

// `facing_normals`: both normals on the ray's side, the shading normal replaced by the geometric
// one where interpolation bent it below the surface or away from the ray.
fn path_facing(point: PathPoint, incoming: vec3<f32>) -> PathFacing {
    let toward = -incoming;
    let back_face = dot(point.geometric, toward) < 0.0f;
    let geometric = select(point.geometric, -point.geometric, back_face);
    var shading = select(point.shading, -point.shading, back_face);
    if !(dot(shading, geometric) > 0.0f && dot(shading, toward) > 0.0f) {
        shading = geometric;
    }
    return PathFacing(geometric, shading, back_face);
}

fn path_next_origin(point: PathPoint, outgoing: vec3<f32>) -> vec3<f32> {
    let side = select(-point.ray_offset, point.ray_offset, dot(outgoing, point.geometric) >= 0.0f);
    return point.position + point.geometric * side;
}

fn path_ray_from(origin: vec3<f32>, direction: vec3<f32>) -> Ray {
    return Ray(origin, 0.0f, direction, PATH_FAR, PATH_FACE_BOTH);
}

// ---- The frame around a normal (`scattering/frame.rs`).

struct PathFrame {
    found: bool,
    tangent: vec3<f32>,
    bitangent: vec3<f32>,
    normal: vec3<f32>,
}

fn path_frame(normal_in: vec3<f32>) -> PathFrame {
    let n = unit_direction(normal_in);
    if !has_direction(n) {
        return PathFrame(false, vec3<f32>(0.0f), vec3<f32>(0.0f), vec3<f32>(0.0f));
    }
    let sign = select(-1.0f, 1.0f, n.z >= 0.0f);
    let a = -1.0f / (sign + n.z);
    let b = n.x * n.y * a;
    let tangent = vec3<f32>(1.0f + sign * n.x * n.x * a, sign * b, -sign * n.x);
    let bitangent = vec3<f32>(b, sign + n.y * n.y * a, -n.y);
    return PathFrame(true, tangent, bitangent, n);
}

fn path_to_local(frame: PathFrame, world: vec3<f32>) -> vec3<f32> {
    return vec3<f32>(dot(world, frame.tangent), dot(world, frame.bitangent), dot(world, frame.normal));
}

fn path_to_world(frame: PathFrame, local: vec3<f32>) -> vec3<f32> {
    return frame.tangent * local.x + frame.bitangent * local.y + frame.normal * local.z;
}

// ---- Lights (`lights.rs`).

fn path_smoothstep(t_in: f32) -> f32 {
    let t = clamp(t_in, 0.0f, 1.0f);
    return t * t * (3.0f - 2.0f * t);
}

fn path_environment(direction: vec3<f32>) -> vec3<f32> {
    if (path_view.flags & PATH_FLAG_GRADIENT) == 0u {
        return path_view.env_zenith.xyz;
    }
    if direction.z >= 0.0f {
        return mix(path_view.env_horizon.xyz, path_view.env_zenith.xyz, path_smoothstep(direction.z));
    }
    return mix(path_view.env_horizon.xyz, path_view.env_ground.xyz, path_smoothstep(-direction.z));
}

fn path_has_sun() -> bool {
    return (path_view.flags & PATH_FLAG_SUN) != 0u;
}

fn path_sun_contains(direction: vec3<f32>) -> bool {
    let chord = direction - path_view.sun_direction.xyz;
    return dot(chord, chord) <= path_view.sun_bitangent.w;
}

fn path_sun_sample(u: vec2<f32>) -> vec3<f32> {
    let versine = path_view.sun_direction.w;
    let height = u.x * versine;
    let sin_angle = sqrt(max(height * (2.0f - height), 0.0f));
    let turn = cos_sin_of_turn(u.y);
    return path_view.sun_tangent.xyz * (sin_angle * turn.x)
        + path_view.sun_bitangent.xyz * (sin_angle * turn.y)
        + path_view.sun_direction.xyz * (1.0f - height);
}

// `pdf^2 / (pdf^2 + other^2)` from their ratio, as `integrator.rs` does. WGSL may assume no float
// is infinite, so a ratio whose square would overflow f32 is answered directly: its weight is at
// most 1e-36, nothing an f32 sum can keep.
fn path_power_heuristic(pdf: f32, other: f32) -> f32 {
    if pdf > 0.0f {
        let ratio = other / pdf;
        if ratio >= 1e18f {
            return 0.0f;
        }
        return 1.0f / (1.0f + ratio * ratio);
    }
    return 0.0f;
}

// The sun radiance a ray that left the scene sees, `escaped_radiance`.
fn path_escaped(direction: vec3<f32>, sampled: bool, pdf: f32) -> vec3<f32> {
    var radiance = path_environment(direction);
    if path_has_sun() && path_sun_contains(direction) {
        var weight = 1.0f;
        if sampled && path_view.strategy != PATH_STRATEGY_SURFACE_ONLY {
            if path_view.strategy == PATH_STRATEGY_NEXT_EVENT_ONLY {
                weight = 0.0f;
            } else {
                weight = path_power_heuristic(pdf, path_view.sun_tangent.w);
            }
        }
        radiance += path_view.sun_radiance.xyz * weight;
    }
    return radiance;
}

// ---- One bounce's pieces (`integrator.rs`).

fn path_largest_channel(c: vec3<f32>) -> f32 {
    return max(max(c.x, c.y), c.z);
}

fn path_sides_agree(geometric: vec3<f32>, wo_world: vec3<f32>, wi_world: vec3<f32>, wo_z: f32, wi_z: f32) -> bool {
    let local_same_side = wo_z * wi_z > 0.0f;
    let geometric_same_side = dot(wo_world, geometric) * dot(wi_world, geometric) > 0.0f;
    return local_same_side == geometric_same_side;
}

fn path_sun_reachable(origin_ray: Ray) -> bool {
    if path_floor_distance(origin_ray) >= 0.0f {
        return false;
    }
    if (path_view.flags & PATH_FLAG_SEE_THROUGH) == 0u {
        return trace_any(origin_ray).triangle == NO_HIT;
    }
    line_restart();
    var ray = origin_ray;
    let direction = origin_ray.direction;
    loop {
        let hit = trace_closest(ray);
        if hit.triangle == NO_HIT {
            return true;
        }
        let point = path_surface_point(hit);
        let verdict = line_at_hit(point.part);
        if verdict != LINE_PASSES {
            return false;
        }
        ray = path_ray_from(path_next_origin(point, direction), direction);
    }
    return false;
}

struct PathVertex {
    point: PathPoint,
    surface: Surface,
    frame: PathFrame,
    toward_camera: vec3<f32>,
    wo: vec3<f32>,
}

// The sun's contribution at this vertex, or zero.
fn path_sample_sun(vertex: PathVertex) -> vec3<f32> {
    if !path_has_sun() || path_view.strategy == PATH_STRATEGY_SURFACE_ONLY {
        return vec3<f32>(0.0f);
    }
    let u0 = path_next_unit();
    let u1 = path_next_unit();
    let wi_world = path_sun_sample(vec2<f32>(u0, u1));
    let wi = path_to_local(vertex.frame, wi_world);
    if !path_sides_agree(vertex.point.geometric, vertex.toward_camera, wi_world, vertex.wo.z, wi.z) {
        return vec3<f32>(0.0f);
    }
    let bsdf = surface_evaluate(vertex.surface, vertex.wo, wi);
    if path_largest_channel(bsdf) <= 0.0f {
        return vec3<f32>(0.0f);
    }
    let shadow = path_ray_from(path_next_origin(vertex.point, wi_world), wi_world);
    if !path_sun_reachable(shadow) {
        return vec3<f32>(0.0f);
    }
    let pdf_light = path_view.sun_tangent.w;
    var weight = 1.0f;
    if path_view.strategy == PATH_STRATEGY_COMBINED {
        weight = path_power_heuristic(pdf_light, surface_pdf(vertex.surface, vertex.wo, wi));
    }
    return bsdf * path_view.sun_radiance.xyz * (abs(wi.z) / pdf_light * weight);
}

// The surface of `part` at a face of `color`: base times the decoded face color, for a colored
// face (the CPU's white-face shortcut gives the same answer, since white decodes to 1).
fn path_surface_at(part: u32, color: u32) -> Surface {
    if part == PATH_FLOOR_PART {
        return path_floor_surface();
    }
    var surface = parts[part].surface;
    if (color & 0xFFFFFFu) != 0xFFFFFFu {
        let r = color & 255u;
        let g = (color >> 8u) & 255u;
        let b = (color >> 16u) & 255u;
        surface.base_r *= path_srgb.to_linear[r >> 2u][r & 3u];
        surface.base_g *= path_srgb.to_linear[g >> 2u][g & 3u];
        surface.base_b *= path_srgb.to_linear[b >> 2u][b & 3u];
    }
    return surface;
}

// ---- The path (`trace_path`).

fn path_radiance(camera_ray: Ray) -> vec3<f32> {
    var ray = camera_ray;
    var radiance = vec3<f32>(0.0f);
    var throughput = vec3<f32>(1.0f);
    // The radiance scaling of refraction, kept apart so roulette reads how much energy the path
    // really carries: a path inside glass is only divided by `ior^2` until it leaves.
    var eta_scale = 1.0f;
    var arrival_sampled = false;
    var arrival_pdf = 0.0f;
    var scatters = 0u;
    line_restart();

    loop {
        let hit = trace_closest(ray);
        let floor_t = path_floor_distance(ray);
        let floor_first = floor_t >= 0.0f && (hit.triangle == NO_HIT || floor_t < hit.t);
        if hit.triangle == NO_HIT && !floor_first {
            radiance += throughput * path_escaped(ray.direction, arrival_sampled, arrival_pdf);
            break;
        }
        var point: PathPoint;
        if floor_first {
            point = path_floor_point(ray, floor_t);
        } else {
            point = path_surface_point(hit);
        }
        // Whether the part is there at all comes before the bounce limit: a path past its last
        // bounce still sees through what isn't there. The floor is always there.
        if !floor_first && (path_view.flags & PATH_FLAG_SEE_THROUGH) != 0u {
            let verdict = line_at_hit(point.part);
            if verdict == LINE_ENDED {
                break;
            }
            if verdict == LINE_PASSES {
                ray = path_ray_from(path_next_origin(point, ray.direction), ray.direction);
                continue;
            }
        }
        if scatters >= path_view.max_bounces {
            break;
        }

        let surface = path_surface_at(point.part, point.color);
        let toward_camera = -ray.direction;
        let facing = path_facing(point, ray.direction);
        // A transmissive surface keeps its true outside as +z: that is how the model tells a ray
        // entering the solid from one leaving it.
        let frame_normal = select(facing.shading, -facing.shading, has_glass(surface) && facing.back_face);
        let frame = path_frame(frame_normal);
        if !frame.found {
            break;
        }
        let wo = path_to_local(frame, toward_camera);

        let vertex = PathVertex(point, surface, frame, toward_camera, wo);
        radiance += throughput * path_sample_sun(vertex);

        let drawn = surface_sample(surface, wo, path_next_unit3());
        if !drawn.found {
            break;
        }
        let wi_world = unit_direction(path_to_world(frame, drawn.direction));
        if !has_direction(wi_world) {
            break;
        }
        if !path_sides_agree(point.geometric, toward_camera, wi_world, wo.z, drawn.direction.z) {
            break;
        }
        throughput *= drawn.weight;
        if wo.z * drawn.direction.z < 0.0f {
            let ior2 = surface.ior * surface.ior;
            if wo.z > 0.0f {
                eta_scale *= ior2;
            } else {
                eta_scale /= ior2;
            }
        }
        scatters += 1u;
        arrival_sampled = !drawn.is_specular;
        arrival_pdf = drawn.pdf;

        if path_view.roulette_after != PATH_ROULETTE_OFF && scatters >= path_view.roulette_after {
            let survival = min(path_largest_channel(throughput) * eta_scale, PATH_MAX_SURVIVAL);
            if survival <= 0.0f || path_next_unit() >= survival {
                break;
            }
            throughput /= survival;
        }

        ray = path_ray_from(path_next_origin(point, wi_world), wi_world);
        line_restart();
    }
    return radiance;
}

// ---- Accumulation (`render_surface.rs`'s `add_sample`).

// True when the bits are a finite number. Read from the bits and not with `x == x`, which a WGSL
// compiler may fold to true.
fn path_is_finite(value: f32) -> bool {
    return (bitcast<u32>(value) & 0x7F800000u) != 0x7F800000u;
}

fn path_finite3(value: vec3<f32>) -> bool {
    return path_is_finite(value.x) && path_is_finite(value.y) && path_is_finite(value.z);
}

// Where sample `sample_index` of pixel (x, y) falls inside it, a stream that the salt makes
// unrelated to any other: a pure function of those, so it never disturbs the path's random numbers.
fn path_jitter(salt_low: u32, salt_high: u32, x: u32, y: u32, sample_index: u32) -> vec2<f32> {
    path_rng_start_salted(salt_low, salt_high, x, y, PATH_JITTER_STREAM);
    let offset_x = path_next_bits() + sample_index * PATH_JITTER_STEP_X;
    let offset_y = path_next_bits() + sample_index * PATH_JITTER_STEP_Y;
    return vec2<f32>(
        f32(offset_x >> 8u) * PATH_UNIT_FROM_24_BITS,
        f32(offset_y >> 8u) * PATH_UNIT_FROM_24_BITS,
    );
}

fn path_camera_ray(x: f32, y: f32) -> Ray {
    let width = f32(path_view.trace_width);
    let height = f32(path_view.trace_height);
    let shorter = min(width, height);
    let horizontal = (2.0f * x - width) / shorter;
    let vertical = (2.0f * y - height) / shorter;
    let scale = path_view.eye.w;
    if (path_view.flags & PATH_FLAG_ORTHOGRAPHIC) != 0u {
        let origin = path_view.eye.xyz + path_view.right.xyz * (horizontal * scale)
            + path_view.up.xyz * (vertical * scale);
        return path_ray_from(origin, path_view.forward.xyz);
    }
    let direction = path_view.forward.xyz + path_view.right.xyz * (horizontal * scale)
        + path_view.up.xyz * (vertical * scale);
    // `normalize` and not `unit_direction`, whose early return for a vector already close to unit
    // length is taken only by the rays within a tenth of a pixel of the view's axis. On the NVIDIA
    // Vulkan driver (RTX 5070, 610.60) a pixel taking that branch at this call site loses the
    // device in the dispatch that holds it, with no loop of this shader spinning, and it does not
    // on Intel's driver. This direction doesn't need the shortcut: it is freshly summed, so it
    // arrives with no shared rounding worth keeping, and its length is at least 1 from the
    // orthonormal basis, so `normalize` is safe where `unit_direction` guards small inputs.
    return path_ray_from(path_view.eye.xyz, normalize(direction));
}

@compute @workgroup_size(64)
fn path_trace_main(@builtin(global_invocation_id) id: vec3<u32>) {
    if id.x >= path_slice.pixel_count {
        return;
    }
    let index = path_slice.first_pixel + id.x;
    let x = index % path_view.width;
    let y = index / path_view.width;
    let sample_index = path_slice.sample_index;

    // The jitter stream is apart from the path's so it doesn't reuse the numbers the path scatters
    // with, and it is a pure function of the pixel and sample index as on the CPU.
    let jitter = path_jitter(0u, 0u, x, y, sample_index);
    let ray = path_camera_ray(f32(x) + jitter.x, f32(y) + jitter.y);

    if (path_view.flags & PATH_FLAG_COMPOSITE) != 0u {
        path_composite_sample(index, x, y, sample_index, ray);
        return;
    }

    path_rng_start(x, y, sample_index);
    let value = path_radiance(ray);

    let prior = accumulation[index];
    let sum = prior.xyz + value;
    let usable = path_finite3(value) && value.x >= 0.0f && value.y >= 0.0f && value.z >= 0.0f
        && path_finite3(sum);
    if usable {
        accumulation[index] = vec4<f32>(sum, prior.w + 1.0f);
    }
}
