// The scattering model in WGSL: `evaluate`, `surface_pdf` and `surface_sample` for one surface, a
// function-by-function mirror of `scattering/model.rs`, `microfacet.rs` and `albedo.rs`, in the
// local frame +z = the shading normal, `wo` and `wi` pointing away from the surface. `scatter_shader.rs`
// exposes this file as `SCATTER_WGSL`; it declares no entry point, so an integrator includes it.
// The one binding it owns is the albedo table, group 1 binding 0, so it can sit beside the
// traversal's group 0 (`scatter_shader.rs` writes the bytes).
//
// The f32 port is the f64 model's arithmetic with these differences, each for the reason given.
//
// - The branch points of the model (`smooth`, index-matched, has-glass) are decided once in f64
//   when `SurfaceRecord` is written and arrive as `flags`, so a roughness or index of refraction
//   that rounds across a threshold cannot pick a different branch here than on the CPU.
// - `1 - E`, the energy a lobe loses to multiple scattering, is stored in the albedo table
//   (`loss`, read with `A` and `B` from an `AlbedoEntry`). In f32 `1 - (A + B)` would be mostly
//   rounding noise at the narrow end, where the true value is ~1e-5 and the f32 resolution near 1
//   is 1.2e-7. Interpolating the stored `loss` needs no such subtraction.
// - `sample_visible_normal` writes `half.z` as `(1 - v) * (1 + s.z)`, the exact value of
//   `z + s.z`, rather than the sum, which cancels in f32 when the sampled point is near the cap's
//   edge; and `sin_theta` as `sqrt((1 - z) * (1 + z))` for the same reason near `z = ±1`.
// - The half vector is `bisector`, `wo + wi` with its component along `wo - wi` removed, and the
//   Fresnel term's cosine is `half_vector_cosine`, half its length, rather than `dot(wo, h)`. At a
//   mirror reflection near grazing `wo + wi` is short and a direction's f32 length error (a
//   rounding the f64 model does not make) is a large part of it, amplified by `1 / |wo + wi|`.
//   True unit vectors have `wo + wi` exactly perpendicular to `wo - wi`, so that error can be
//   projected out exactly.
// - The refraction half vector is `refraction_sum`, the same repair for the eta-weighted sum, and
//   its squared length stands in for `(eta_wo cos_o + eta_wi cos_i)^2`, which cancels in f32 near
//   index 1. The sum is short there for the same reason a mirror reflection's is, and the two
//   length errors weigh the same in it.
// - The azimuth of a sampled direction is `cos_sin_of_turn`, not the built-in `sin` and `cos`: WGSL
//   only requires those to be within 2^-11 of the truth on `[-pi, pi]` and says nothing outside
//   it, and a device at that limit would aim a sampled direction up to 1e-3 off, far more than
//   f32 costs anywhere else in this file. The reduction is exact in `u` and the series is good to
//   about `f32::EPSILON` on any device.
// - `unit_direction` keeps the scaling by the largest component (f32 squares of small inputs
//   vanish), and a direction with no length comes back as the zero vector, which the callers test
//   instead of an `Option`. A vector already within `UNIT_LENGTH_SLACK` of unit length is returned
//   as it is: normalizing again would round each component on its own, which `bisector` cannot
//   remove because it is not along `wo - wi`, and every direction a sample hands to `evaluate` and
//   `pdf` would pay for it. The length error left behind is what `bisector` and `refraction_sum`
//   project out.
// - The index step is `ior_minus_one`, a separate field rather than `ior - 1`: a step near index
//   1 subtracted from two f32s keeps only a few bits, and the half vector, `dielectric_f0` and
//   Fresnel are all made of it. Fresnel and Snell's cosine are rewritten in `eta_step` so that
//   their near-equal terms are multiplied out and nothing cancels near index 1.
//
// One limit is the f32 direction itself. Near index 1 the half vector is `eta_wo wo + eta_wi wi`,
//   a sum about as short as the step, and a sampled direction's rounding of 6e-8 per component
//   moves it by about `6e-8 / |sum|`, up to a few percent of a narrow lobe's density. That is the
//   f64 model's own answer moving with the direction it is asked about: at ior 1.0011, roughness
//   0.0317, the CPU's pdf at the shader's direction differs from its pdf at its own direction by
//   a mean of 0.4% and at most 2.8%, while the shader matches the CPU at that same direction to a
//   mean of 2e-5 and at most 2e-4. The check's measured sensitivity allows for it.
// - WGSL may assume no NaN or infinity, so `clean` keeps a value only when it is positive and
//   below f32's largest; a division that blew up comes out as 0, as `clean` in `model.rs` does.
//   Non-finite input is out of contract, the same precondition `trace.wgsl` has: a tracer's
//   directions and random numbers are finite.
//
// Everything else is the CPU model's formula in the CPU model's order. `albedo.rs` gives the
// table axes; `scattering/mod.rs` the conventions.

// ---- Constants, equal to their Rust counterparts in microfacet.rs, model.rs and albedo.rs.

const SCATTER_PI: f32 = 3.14159265358979323846f;
const MIN_COS: f32 = 1e-9f;
const MIN_ALPHA: f32 = 1e-3f;
// The cosine-weighted mean of Schlick's grazing term.
const MEAN_SCHLICK_GRAZING: f32 = 0.047619047619047619f;
const MIN_SINGLE_SCATTER_LOSS: f32 = 1e-5f;
// The largest f32 below 1: 1 - f32::EPSILON / 2.
const ALMOST_ONE: f32 = 0.99999994f;
// `dot(v, v)` of a vector this close to unit length is left alone by `unit_direction`.
const UNIT_LENGTH_SLACK: f32 = 9.5367431640625e-7f;
const LARGEST_F32: f32 = 3.4028234e38f;
const ALBEDO_TABLE_SIZE: u32 = 32u;
const QUARTER_TURN: f32 = 1.5707963267948966f;
// The fourth root of MIN_ALPHA, where the table's row axis starts.
const ROW_LOWEST: f32 = 0.1778279410038923f;

// `SurfaceRecord::flags` bits.
const SURFACE_SMOOTH: u32 = 1u;
const SURFACE_INDEX_MATCHED: u32 = 2u;
const SURFACE_HAS_GLASS: u32 = 4u;

// ---- The two records this module adds (`scatter_shader.rs` and `scattering/gpu_record.rs`).

struct Surface {
    base_r: f32,
    base_g: f32,
    base_b: f32,
    metallic: f32,
    roughness: f32,
    transmission: f32,
    ior: f32,
    ior_minus_one: f32,
    flags: u32,
}

// One albedo table entry: `A` and `B`, and `1 - A - B` kept exactly (see the header).
struct AlbedoEntry {
    a: f32,
    b: f32,
    loss: f32,
}

@group(1) @binding(0) var<storage, read> albedo_table: array<AlbedoEntry>;

// ---- The results of a lookup or a sample.

struct Shares {
    metal: f32,
    plastic: f32,
    glass: f32,
}

struct Energy {
    multiple_scatter: bool,
    mean_loss: f32,
    multiple_scatter_fresnel: vec3<f32>,
    plastic_multiple_scatter: f32,
    plastic_mean: f32,
    plastic_f0: f32,
    plastic_f90: f32,
}

struct Directional {
    a: f32,
    b: f32,
    loss: f32,
    plastic: f32,
}

struct LobeChoice {
    specular: f32,
    cosine: f32,
    glass: f32,
}

struct GlassSide {
    flipped: bool,
    wo: vec3<f32>,
    eta_wo: f32,
    eta_wi: f32,
    // `eta_wi - eta_wo`, carried exactly so a step near index 1 is not the difference of two f32s.
    eta_step: f32,
}

struct TransmissionHalf {
    found: bool,
    h: vec3<f32>,
    cos_o: f32,
    cos_i: f32,
    // `(eta_wo * cos_o + eta_wi * cos_i)^2`: the refraction denominator, from the sum's length.
    denominator2: f32,
}

// One sampled scattering event, `found` false when nothing scatters. See `ScatteringSample`.
struct ScatterSample {
    found: bool,
    direction: vec3<f32>,
    pdf: f32,
    weight: vec3<f32>,
    is_specular: bool,
}

// ---- Small helpers.

fn no_sample() -> ScatterSample {
    return ScatterSample(false, vec3<f32>(0.0f), 0.0f, vec3<f32>(0.0f), false);
}

fn mirror_z(v: vec3<f32>) -> vec3<f32> {
    return vec3<f32>(v.x, v.y, -v.z);
}

fn mirror_about_z(wo: vec3<f32>) -> vec3<f32> {
    return vec3<f32>(-wo.x, -wo.y, wo.z);
}

fn channel_mean(rgb: vec3<f32>) -> f32 {
    return (rgb.x + rgb.y + rgb.z) / 3.0f;
}

// `value`, or zero when it is not a positive finite number: a division that blew up.
fn clean(value: f32) -> f32 {
    if value > 0.0f && value < LARGEST_F32 {
        return value;
    }
    return 0.0f;
}

fn clean3(value: vec3<f32>) -> vec3<f32> {
    return vec3<f32>(clean(value.x), clean(value.y), clean(value.z));
}

// `value` as a random number in `[0, 1)`.
fn random_unit(value: f32) -> f32 {
    return clamp(value, 0.0f, ALMOST_ONE);
}

fn has_direction(v: vec3<f32>) -> bool {
    return v.x != 0.0f || v.y != 0.0f || v.z != 0.0f;
}

// `v` scaled to unit length, or the zero vector when it has no direction. Scaling by the largest
// component first keeps a very small direction from losing itself in the squares.
fn unit_direction(v: vec3<f32>) -> vec3<f32> {
    if abs(dot(v, v) - 1.0f) < UNIT_LENGTH_SLACK {
        return v;
    }
    let largest = max(max(abs(v.x), abs(v.y)), abs(v.z));
    if largest <= 0.0f {
        return vec3<f32>(0.0f);
    }
    let scaled = v / largest;
    let length_of = sqrt(dot(scaled, scaled));
    if length_of > 0.0f {
        return scaled / length_of;
    }
    return vec3<f32>(0.0f);
}

// `cos` and `sin` of `2 pi u`, for `u` in `[0, 1)`: `u` is reduced to the nearest quarter turn with
// no `pi` in the arithmetic, then a short series on at most an eighth of a turn. See the header.
fn cos_sin_of_turn(u: f32) -> vec2<f32> {
    let quarters = u * 4.0f;
    let nearest = floor(quarters + 0.5f);
    let a = (quarters - nearest) * QUARTER_TURN;
    let a2 = a * a;
    let s = a * (1.0f + a2 * (-1.66666667e-1f + a2 * (8.33333333e-3f + a2 * (-1.98412698e-4f + a2 * 2.75573192e-6f))));
    let c = 1.0f + a2 * (-0.5f + a2 * (4.16666667e-2f + a2 * (-1.38888889e-3f + a2 * 2.48015873e-5f)));
    let quadrant = u32(nearest) % 4u;
    if quadrant == 0u {
        return vec2<f32>(c, s);
    }
    if quadrant == 1u {
        return vec2<f32>(-s, c);
    }
    if quadrant == 2u {
        return vec2<f32>(-c, -s);
    }
    return vec2<f32>(s, -c);
}

// ---- microfacet.rs.

fn ggx_density(h: vec3<f32>, alpha: f32) -> f32 {
    let sin2 = h.x * h.x + h.y * h.y;
    let cos2 = h.z * h.z;
    let alpha2 = alpha * alpha;
    let denominator = sin2 + alpha2 * cos2;
    return alpha2 / (SCATTER_PI * denominator * denominator);
}

fn smith_lambda(cos_theta: f32, alpha: f32) -> f32 {
    let cos2 = cos_theta * cos_theta;
    let tan2 = (1.0f - cos2) / cos2;
    return (sqrt(1.0f + alpha * alpha * tan2) - 1.0f) * 0.5f;
}

fn smith_masking(cos_theta: f32, alpha: f32) -> f32 {
    return 1.0f / (1.0f + smith_lambda(cos_theta, alpha));
}

fn smith_masking_shadowing(cos_o: f32, cos_i: f32, alpha: f32) -> f32 {
    return 1.0f / (1.0f + smith_lambda(cos_o, alpha) + smith_lambda(cos_i, alpha));
}

fn visible_normal_density(wo: vec3<f32>, h: vec3<f32>, alpha: f32) -> f32 {
    let cos_h = max(dot(wo, h), 0.0f);
    return smith_masking(wo.z, alpha) * cos_h * ggx_density(h, alpha) / wo.z;
}

// See the header for the two places this keeps f32 from cancelling.
fn sample_visible_normal(wo: vec3<f32>, alpha: f32, u: f32, v: f32) -> vec3<f32> {
    let stretched = unit_direction(vec3<f32>(alpha * wo.x, alpha * wo.y, wo.z));
    if !has_direction(stretched) {
        return vec3<f32>(0.0f);
    }
    let turn = cos_sin_of_turn(u);
    let z = (1.0f - v) * (1.0f + stretched.z) - stretched.z;
    let sin_theta = sqrt(max((1.0f - z) * (1.0f + z), 0.0f));
    let on_cap = vec3<f32>(sin_theta * turn.x, sin_theta * turn.y, z);
    let half_vector = vec3<f32>(
        on_cap.x + stretched.x,
        on_cap.y + stretched.y,
        (1.0f - v) * (1.0f + stretched.z),
    );
    return unit_direction(vec3<f32>(
        alpha * half_vector.x,
        alpha * half_vector.y,
        max(half_vector.z, 0.0f),
    ));
}

fn reflect_about(w: vec3<f32>, h: vec3<f32>) -> vec3<f32> {
    return h * (2.0f * dot(w, h)) - w;
}

// The zero vector at total internal reflection.
fn refract_through(w: vec3<f32>, h: vec3<f32>, side: GlassSide) -> vec3<f32> {
    let a = side.eta_wo;
    let b = side.eta_wi;
    let cos_incident = dot(w, h);
    let cos2_transmit = (side.eta_step * (a + b) + a * a * cos_incident * cos_incident) / (b * b);
    if cos2_transmit <= 0.0f {
        return vec3<f32>(0.0f);
    }
    let eta_ratio = a / b;
    return h * (eta_ratio * cos_incident - sqrt(cos2_transmit)) - w * eta_ratio;
}

fn fifth_power(value: f32) -> f32 {
    let square = value * value;
    return value * (square * square);
}

fn grazing_reflectance(f0: f32) -> f32 {
    return clamp(50.0f * f0, 0.0f, 1.0f);
}

fn dielectric_fresnel(cos_incident: f32, side: GlassSide) -> f32 {
    let a = side.eta_wo;
    let b = side.eta_wi;
    let c = min(abs(cos_incident), 1.0f);
    let step_sum = side.eta_step * (a + b);
    let cos2_t = (step_sum + a * a * c * c) / (b * b);
    if cos2_t <= 0.0f {
        return 1.0f;
    }
    let cos_t = sqrt(cos2_t);
    // `(a c - b cos_t) / (a c + b cos_t)` and its parallel partner, each with its numerator's two
    // near-equal terms multiplied out so that nothing cancels near index 1.
    let perpendicular_sum = a * c + b * cos_t;
    let perpendicular = -step_sum / (perpendicular_sum * perpendicular_sum);
    let parallel_sum = b * c + a * cos_t;
    let parallel = step_sum * (c * c * (a * a + b * b) - a * a) / (b * b * parallel_sum * parallel_sum);
    return 0.5f * (parallel * parallel + perpendicular * perpendicular);
}

// ---- albedo.rs: `albedo_table` is the 32 by 32 grid in row order, then the 32 row means.

fn column_coordinate(cos_in: f32, alpha: f32) -> f32 {
    let c = clamp(cos_in, 0.0f, 1.0f);
    return 0.5f * sqrt(c) + 0.5f * (c * (1.0f + alpha) / (c + alpha));
}

fn row_coordinate(alpha: f32) -> f32 {
    let root = sqrt(sqrt(max(alpha, MIN_ALPHA)));
    return clamp((root - ROW_LOWEST) / (1.0f - ROW_LOWEST), 0.0f, 1.0f);
}

fn albedo_entry_rgb(index: u32) -> vec3<f32> {
    let entry = albedo_table[index];
    return vec3<f32>(entry.a, entry.b, entry.loss);
}

// `[A, B, loss]` at this cosine and GGX width, interpolated as `directional_albedo_from` does.
fn directional_albedo(cos_in: f32, alpha: f32) -> vec3<f32> {
    let last = f32(ALBEDO_TABLE_SIZE - 1u);
    let row = row_coordinate(alpha) * last;
    let column = column_coordinate(cos_in, alpha) * last;
    let row_low = min(u32(row), ALBEDO_TABLE_SIZE - 2u);
    let column_low = min(u32(column), ALBEDO_TABLE_SIZE - 2u);
    let row_t = row - f32(row_low);
    let column_t = column - f32(column_low);
    let row_first = row_low * ALBEDO_TABLE_SIZE + column_low;
    let near = albedo_entry_rgb(row_first) * (1.0f - column_t)
        + albedo_entry_rgb(row_first + 1u) * column_t;
    let far = albedo_entry_rgb(row_first + ALBEDO_TABLE_SIZE) * (1.0f - column_t)
        + albedo_entry_rgb(row_first + ALBEDO_TABLE_SIZE + 1u) * column_t;
    return near * (1.0f - row_t) + far * row_t;
}

// `[A, B, loss]`, the cosine-weighted mean at this GGX width.
fn mean_albedo(alpha: f32) -> vec3<f32> {
    let last = f32(ALBEDO_TABLE_SIZE - 1u);
    let row = row_coordinate(alpha) * last;
    let row_low = min(u32(row), ALBEDO_TABLE_SIZE - 2u);
    let row_t = row - f32(row_low);
    let first_mean = ALBEDO_TABLE_SIZE * ALBEDO_TABLE_SIZE + row_low;
    return albedo_entry_rgb(first_mean) * (1.0f - row_t)
        + albedo_entry_rgb(first_mean + 1u) * row_t;
}

// ---- model.rs: the parts of a surface a lobe depends on.

fn base_of(s: Surface) -> vec3<f32> {
    return vec3<f32>(s.base_r, s.base_g, s.base_b);
}

fn is_smooth(s: Surface) -> bool {
    return (s.flags & SURFACE_SMOOTH) != 0u;
}

fn is_index_matched(s: Surface) -> bool {
    return (s.flags & SURFACE_INDEX_MATCHED) != 0u;
}

fn has_glass(s: Surface) -> bool {
    return (s.flags & SURFACE_HAS_GLASS) != 0u;
}

fn is_rough_glass(s: Surface) -> bool {
    return has_glass(s) && !is_smooth(s) && !is_index_matched(s);
}

fn shares_of(s: Surface) -> Shares {
    let dielectric = 1.0f - s.metallic;
    return Shares(s.metallic, dielectric * (1.0f - s.transmission), dielectric * s.transmission);
}

fn alpha_of(s: Surface) -> f32 {
    return max(s.roughness * s.roughness, MIN_ALPHA);
}

fn dielectric_f0(s: Surface) -> f32 {
    let ratio = s.ior_minus_one / (s.ior + 1.0f);
    return ratio * ratio;
}

fn tint_of(s: Surface) -> vec3<f32> {
    return sqrt(base_of(s));
}

fn specular_reflectance(s: Surface, shares: Shares, cos_theta: f32) -> vec3<f32> {
    let plastic_f0 = dielectric_f0(s);
    let plastic_f90 = grazing_reflectance(plastic_f0);
    let f0 = shares.metal * base_of(s) + vec3<f32>(shares.plastic * plastic_f0);
    let f90 = vec3<f32>(shares.metal + shares.plastic * plastic_f90);
    let grazing = fifth_power(1.0f - clamp(cos_theta, 0.0f, 1.0f));
    return f0 + (f90 - f0) * grazing;
}

fn single_scatter(s: Surface, shares: Shares, wo: vec3<f32>, wi: vec3<f32>, alpha: f32) -> vec3<f32> {
    let h = unit_direction(bisector(wo, wi));
    if !has_direction(h) {
        return vec3<f32>(0.0f);
    }
    let term = ggx_density(h, alpha) * smith_masking_shadowing(wo.z, wi.z, alpha)
        / (4.0f * wo.z * wi.z);
    return specular_reflectance(s, shares, half_vector_cosine(wo, wi)) * term;
}

// `wo + wi`, the direction of the half vector, with the two directions' f32 length errors removed.
// For true unit vectors `wo + wi` is exactly perpendicular to `wo - wi`, since `(wo + wi) . (wo - wi)`
// is `|wo|^2 - |wi|^2`; a component along `wo - wi` can only be length error. Near a mirror reflection
// `wo + wi` is short and an f32 length error of 6e-8 is a large part of it, so it is removed. When
// `wo - wi` is the short one (light back along the view) the sum is long and needs no correction.
fn bisector(wo: vec3<f32>, wi: vec3<f32>) -> vec3<f32> {
    let sum = wo + wi;
    let difference = wo - wi;
    let difference2 = dot(difference, difference);
    if dot(sum, sum) < difference2 {
        return sum - difference * (dot(sum, difference) / difference2);
    }
    return sum;
}

// `dot(wo, h)` for `h` the unit bisector of `wo` and `wi`, as `length(bisector) / 2`, which equals
// it for unit vectors: `dot(wo, h)` would pick up what remains of the length error as `eps / |wo + wi|`.
fn half_vector_cosine(wo: vec3<f32>, wi: vec3<f32>) -> f32 {
    return 0.5f * length(bisector(wo, wi));
}

fn multiple_scatter_fresnel_of(f0: f32, f90: f32, mean_white: f32, mean_loss: f32) -> f32 {
    let mean_fresnel = f0 + (f90 - f0) * MEAN_SCHLICK_GRAZING;
    let denominator = 1.0f - mean_fresnel * mean_loss;
    if denominator > 0.0f {
        return mean_fresnel * mean_fresnel * mean_white / denominator;
    }
    return 0.0f;
}

fn reflective_energy(s: Surface, shares: Shares, alpha: f32) -> Energy {
    let mean = mean_albedo(alpha);
    let mean_a = mean.x;
    let mean_b = mean.y;
    let mean_white = min(mean_a + mean_b, 1.0f);
    let mean_loss = max(mean.z, 0.0f);
    let multiple_scatter = !is_smooth(s) && mean_loss >= MIN_SINGLE_SCATTER_LOSS;
    let plastic_f0 = dielectric_f0(s);
    let plastic_f90 = grazing_reflectance(plastic_f0);
    var plastic_multiple_scatter = 0.0f;
    var multiple_scatter_color = vec3<f32>(0.0f);
    if multiple_scatter {
        plastic_multiple_scatter = multiple_scatter_fresnel_of(plastic_f0, plastic_f90, mean_white, mean_loss);
        let base = base_of(s);
        multiple_scatter_color = vec3<f32>(
            shares.metal * multiple_scatter_fresnel_of(base.x, 1.0f, mean_white, mean_loss)
                + shares.plastic * plastic_multiple_scatter,
            shares.metal * multiple_scatter_fresnel_of(base.y, 1.0f, mean_white, mean_loss)
                + shares.plastic * plastic_multiple_scatter,
            shares.metal * multiple_scatter_fresnel_of(base.z, 1.0f, mean_white, mean_loss)
                + shares.plastic * plastic_multiple_scatter,
        );
    }
    let plastic_mean = plastic_f0 * mean_a + plastic_f90 * mean_b + plastic_multiple_scatter * mean_loss;
    return Energy(
        multiple_scatter,
        mean_loss,
        multiple_scatter_color,
        plastic_multiple_scatter,
        plastic_mean,
        plastic_f0,
        plastic_f90,
    );
}

fn directional_energy(energy: Energy, cos_in: f32, alpha: f32) -> Directional {
    let table = directional_albedo(cos_in, alpha);
    let loss = max(table.z, 0.0f);
    let plastic = energy.plastic_f0 * table.x + energy.plastic_f90 * table.y
        + energy.plastic_multiple_scatter * loss;
    return Directional(table.x, table.y, loss, plastic);
}

fn lobe_choice(s: Surface, shares: Shares, cos_o: f32) -> LobeChoice {
    var specular = 0.0f;
    var cosine = 0.0f;
    if cos_o >= MIN_COS {
        let alpha = alpha_of(s);
        let energy = reflective_energy(s, shares, alpha);
        let at_o = directional_energy(energy, cos_o, alpha);
        let base = base_of(s);
        let single = shares.metal * (base * at_o.a + vec3<f32>(at_o.b))
            + vec3<f32>(shares.plastic * (energy.plastic_f0 * at_o.a + energy.plastic_f90 * at_o.b));
        let multiple = channel_mean(energy.multiple_scatter_fresnel) * at_o.loss;
        let diffuse = shares.plastic * channel_mean(base) * max(1.0f - at_o.plastic, 0.0f);
        specular = channel_mean(single);
        cosine = multiple + diffuse;
    }
    let total = specular + cosine + shares.glass;
    if total > 0.0f {
        return LobeChoice(specular / total, cosine / total, shares.glass / total);
    }
    return LobeChoice(0.0f, 0.0f, 0.0f);
}

fn glass_side(s: Surface, wo: vec3<f32>) -> GlassSide {
    if wo.z >= 0.0f {
        return GlassSide(false, wo, 1.0f, s.ior, s.ior_minus_one);
    }
    return GlassSide(true, mirror_z(wo), s.ior, 1.0f, -s.ior_minus_one);
}

fn reflected_glass(side: GlassSide, up_wi: vec3<f32>, alpha: f32, glass_share: f32) -> f32 {
    let h = unit_direction(bisector(side.wo, up_wi));
    if !has_direction(h) {
        return 0.0f;
    }
    let fresnel = dielectric_fresnel(half_vector_cosine(side.wo, up_wi), side);
    let term = ggx_density(h, alpha) * smith_masking_shadowing(side.wo.z, up_wi.z, alpha)
        / (4.0f * side.wo.z * up_wi.z);
    return glass_share * fresnel * term;
}

fn reflected_glass_density(side: GlassSide, up_wi: vec3<f32>, alpha: f32) -> f32 {
    let h = unit_direction(bisector(side.wo, up_wi));
    if !has_direction(h) {
        return 0.0f;
    }
    let cos_o = half_vector_cosine(side.wo, up_wi);
    let fresnel = dielectric_fresnel(cos_o, side);
    return fresnel * visible_normal_density(side.wo, h, alpha) / (4.0f * dot(side.wo, h));
}

// `eta_wo * wo + eta_wi * wi`, the direction of the refraction half vector, the analog of `bisector`
// for an index step (see the header). The short form of the sum keeps its two near-opposite terms
// together, so it does not round on the way to cancelling, and the length error of `wo` and `wi`
// is projected out against `eta_wo * wo - eta_wi * wi`: for true unit vectors the dot of the two is
// exactly `eta_wo^2 - eta_wi^2`, and anything else along the difference is that error.
fn refraction_sum(side: GlassSide, up_wi: vec3<f32>) -> vec3<f32> {
    let a = side.eta_wo;
    let b = side.eta_wi;
    let sum = (side.wo + up_wi) * a + up_wi * side.eta_step;
    let difference = side.wo * a - up_wi * b;
    let difference2 = dot(difference, difference);
    if dot(sum, sum) < difference2 {
        let expected = -side.eta_step * (a + b);
        return sum + difference * ((expected - dot(sum, difference)) / difference2);
    }
    return sum;
}

fn transmission_half(side: GlassSide, up_wi: vec3<f32>) -> TransmissionHalf {
    let none = TransmissionHalf(false, vec3<f32>(0.0f), 0.0f, 0.0f, 0.0f);
    let sum = refraction_sum(side, up_wi);
    let unoriented = unit_direction(sum);
    if !has_direction(unoriented) {
        return none;
    }
    let h = select(unoriented, -unoriented, unoriented.z < 0.0f);
    let cos_o = dot(side.wo, h);
    let cos_i = dot(up_wi, h);
    if cos_o <= 0.0f || cos_i >= 0.0f {
        return none;
    }
    return TransmissionHalf(true, h, cos_o, cos_i, dot(sum, sum));
}

fn transmitted_glass(side: GlassSide, up_wi: vec3<f32>, alpha: f32, glass_share: f32, tint: vec3<f32>) -> vec3<f32> {
    let half_of = transmission_half(side, up_wi);
    if !half_of.found {
        return vec3<f32>(0.0f);
    }
    let denominator2 = half_of.denominator2;
    let fresnel = dielectric_fresnel(half_of.cos_o, side);
    let term = ggx_density(half_of.h, alpha)
        * smith_masking_shadowing(side.wo.z, -up_wi.z, alpha)
        * abs(half_of.cos_o * half_of.cos_i)
        / (side.wo.z * abs(up_wi.z))
        * (side.eta_wo * side.eta_wo)
        / denominator2;
    return tint * (glass_share * (1.0f - fresnel) * term);
}

fn transmitted_glass_density(side: GlassSide, up_wi: vec3<f32>, alpha: f32) -> f32 {
    let half_of = transmission_half(side, up_wi);
    if !half_of.found {
        return 0.0f;
    }
    let denominator2 = half_of.denominator2;
    let fresnel = dielectric_fresnel(half_of.cos_o, side);
    let jacobian = side.eta_wi * side.eta_wi * abs(half_of.cos_i) / denominator2;
    return (1.0f - fresnel) * visible_normal_density(side.wo, half_of.h, alpha) * jacobian;
}

// ---- The three operations a tracer calls: model.rs's `evaluate`, `pdf` and `sample`.

// The BSDF toward unit directions `wo` and `wi`, per channel.
fn surface_evaluate(s: Surface, wo_in: vec3<f32>, wi_in: vec3<f32>) -> vec3<f32> {
    let wo = unit_direction(wo_in);
    let wi = unit_direction(wi_in);
    if !has_direction(wo) || !has_direction(wi) {
        return vec3<f32>(0.0f);
    }
    let shares = shares_of(s);
    let alpha = alpha_of(s);
    var value = vec3<f32>(0.0f);
    let flipped = has_glass(s) && wo.z < 0.0f;
    let opaque_wo = select(wo, mirror_z(wo), flipped);
    let opaque_wi = select(wi, mirror_z(wi), flipped);
    if opaque_wo.z >= MIN_COS && opaque_wi.z >= MIN_COS {
        let energy = reflective_energy(s, shares, alpha);
        let at_o = directional_energy(energy, opaque_wo.z, alpha);
        let at_i = directional_energy(energy, opaque_wi.z, alpha);
        if !is_smooth(s) {
            value += single_scatter(s, shares, opaque_wo, opaque_wi, alpha);
        }
        if energy.multiple_scatter {
            let scale = at_o.loss * at_i.loss / (SCATTER_PI * energy.mean_loss);
            value += energy.multiple_scatter_fresnel * scale;
        }
        let diffuse = shares.plastic * max(1.0f - at_o.plastic, 0.0f) * max(1.0f - at_i.plastic, 0.0f)
            / (SCATTER_PI * (1.0f - energy.plastic_mean));
        value += base_of(s) * diffuse;
    }
    if is_rough_glass(s) && abs(wo.z) >= MIN_COS && abs(wi.z) >= MIN_COS {
        let side = glass_side(s, wo);
        let up_wi = select(wi, mirror_z(wi), side.flipped);
        if up_wi.z > 0.0f {
            value += vec3<f32>(reflected_glass(side, up_wi, alpha, shares.glass));
        } else {
            value += transmitted_glass(side, up_wi, alpha, shares.glass, tint_of(s));
        }
    }
    return clean3(value);
}

// The solid-angle density `surface_sample` draws `wi` from for this `wo`.
fn surface_pdf(s: Surface, wo_in: vec3<f32>, wi_in: vec3<f32>) -> f32 {
    let wo = unit_direction(wo_in);
    let wi = unit_direction(wi_in);
    if !has_direction(wo) || !has_direction(wi) {
        return 0.0f;
    }
    let shares = shares_of(s);
    let alpha = alpha_of(s);
    let flipped = has_glass(s) && wo.z < 0.0f;
    let opaque_wo = select(wo, mirror_z(wo), flipped);
    let opaque_wi = select(wi, mirror_z(wi), flipped);
    let choice = lobe_choice(s, shares, opaque_wo.z);
    var density = 0.0f;
    if opaque_wo.z >= MIN_COS && opaque_wi.z >= MIN_COS {
        if choice.specular > 0.0f && !is_smooth(s) {
            let h = unit_direction(bisector(opaque_wo, opaque_wi));
            if has_direction(h) {
                density += choice.specular * visible_normal_density(opaque_wo, h, alpha)
                    / (4.0f * dot(opaque_wo, h));
            }
        }
        density += choice.cosine * opaque_wi.z / SCATTER_PI;
    }
    if choice.glass > 0.0f && is_rough_glass(s) && abs(wo.z) >= MIN_COS && abs(wi.z) >= MIN_COS {
        let side = glass_side(s, wo);
        let up_wi = select(wi, mirror_z(wi), side.flipped);
        if up_wi.z > 0.0f {
            density += choice.glass * reflected_glass_density(side, up_wi, alpha);
        } else {
            density += choice.glass * transmitted_glass_density(side, up_wi, alpha);
        }
    }
    return clean(density);
}

fn finish_sample(s: Surface, wo: vec3<f32>, wi: vec3<f32>) -> ScatterSample {
    let density = surface_pdf(s, wo, wi);
    if density <= 0.0f {
        return no_sample();
    }
    let cosine = abs(wi.z);
    let weight = clean3(surface_evaluate(s, wo, wi) * cosine / density);
    return ScatterSample(true, wi, density, weight, false);
}

fn delta_event(direction_in: vec3<f32>, probability: f32, throughput: vec3<f32>, density: f32) -> ScatterSample {
    let direction = unit_direction(direction_in);
    if !has_direction(direction) || probability <= 0.0f {
        return no_sample();
    }
    return ScatterSample(true, direction, clean(density), clean3(throughput / probability), true);
}

fn sample_specular(s: Surface, shares: Shares, wo: vec3<f32>, probability: f32, u1: f32, u2: f32) -> ScatterSample {
    let flipped = has_glass(s) && wo.z < 0.0f;
    let opaque_wo = select(wo, mirror_z(wo), flipped);
    if is_smooth(s) {
        let fresnel = specular_reflectance(s, shares, opaque_wo.z);
        let direction = mirror_about_z(opaque_wo);
        return delta_event(select(direction, mirror_z(direction), flipped), probability, fresnel, probability);
    }
    let h = sample_visible_normal(opaque_wo, alpha_of(s), u1, u2);
    if !has_direction(h) {
        return no_sample();
    }
    let wi = unit_direction(reflect_about(opaque_wo, h));
    if !has_direction(wi) || wi.z < MIN_COS {
        return no_sample();
    }
    return finish_sample(s, wo, select(wi, mirror_z(wi), flipped));
}

fn sample_cosine(s: Surface, wo: vec3<f32>, u1: f32, u2: f32) -> ScatterSample {
    let flipped = has_glass(s) && wo.z < 0.0f;
    let radius = sqrt(u1);
    let turn = cos_sin_of_turn(u2);
    let wi = vec3<f32>(radius * turn.x, radius * turn.y, sqrt(max(1.0f - u1, 0.0f)));
    if wi.z < MIN_COS {
        return no_sample();
    }
    return finish_sample(s, wo, select(wi, mirror_z(wi), flipped));
}

fn sample_glass(s: Surface, shares: Shares, wo: vec3<f32>, probability: f32, pick: f32, u1: f32, u2: f32) -> ScatterSample {
    let side = glass_side(s, wo);
    if is_smooth(s) || is_index_matched(s) {
        let fresnel = dielectric_fresnel(side.wo.z, side);
        if pick < fresnel {
            let event = probability * fresnel;
            let direction = mirror_about_z(side.wo);
            return delta_event(
                select(direction, mirror_z(direction), side.flipped),
                event,
                vec3<f32>(shares.glass) * fresnel,
                event,
            );
        }
        let eta_ratio = side.eta_wo / side.eta_wi;
        let up = refract_through(side.wo, vec3<f32>(0.0f, 0.0f, 1.0f), side);
        if !has_direction(up) {
            return no_sample();
        }
        let radiance = eta_ratio * eta_ratio;
        let event = probability * (1.0f - fresnel);
        let throughput = tint_of(s) * (shares.glass * (1.0f - fresnel) * radiance);
        return delta_event(select(up, mirror_z(up), side.flipped), event, throughput, event);
    }
    let h = sample_visible_normal(side.wo, alpha_of(s), u1, u2);
    if !has_direction(h) {
        return no_sample();
    }
    let fresnel = dielectric_fresnel(dot(side.wo, h), side);
    var up_wi: vec3<f32>;
    if pick < fresnel {
        let reflected = unit_direction(reflect_about(side.wo, h));
        if !has_direction(reflected) || reflected.z < MIN_COS {
            return no_sample();
        }
        up_wi = reflected;
    } else {
        let refracted_raw = refract_through(side.wo, h, side);
        if !has_direction(refracted_raw) {
            return no_sample();
        }
        let refracted = unit_direction(refracted_raw);
        if !has_direction(refracted) || refracted.z > -MIN_COS {
            return no_sample();
        }
        up_wi = refracted;
    }
    return finish_sample(s, wo, select(up_wi, mirror_z(up_wi), side.flipped));
}

// Draws the next direction of a path along `wo` from three random numbers in `[0, 1)`; `found`
// false when nothing scatters.
fn surface_sample(s: Surface, wo_in: vec3<f32>, u: vec3<f32>) -> ScatterSample {
    let wo = unit_direction(wo_in);
    if !has_direction(wo) || abs(wo.z) < MIN_COS {
        return no_sample();
    }
    let pick = random_unit(u.x);
    let u1 = random_unit(u.y);
    let u2 = random_unit(u.z);
    let shares = shares_of(s);
    let flipped = has_glass(s) && wo.z < 0.0f;
    let choice = lobe_choice(s, shares, select(wo.z, -wo.z, flipped));
    if pick < choice.specular && choice.specular > 0.0f {
        return sample_specular(s, shares, wo, choice.specular, u1, u2);
    }
    if pick < choice.specular + choice.cosine && choice.cosine > 0.0f {
        return sample_cosine(s, wo, u1, u2);
    }
    if choice.glass > 0.0f {
        let remapped = (pick - choice.specular - choice.cosine) / choice.glass;
        return sample_glass(s, shares, wo, choice.glass, random_unit(remapped), u1, u2);
    }
    if choice.cosine > 0.0f {
        return sample_cosine(s, wo, u1, u2);
    }
    if choice.specular > 0.0f {
        return sample_specular(s, shares, wo, choice.specular, u1, u2);
    }
    return no_sample();
}
