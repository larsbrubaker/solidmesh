// Pictures on faces in WGSL: the factor a face's picture multiplies its color by at a hit, the
// CPU's lookup (`path_tracer/trace_texture.rs`) in `f32`. It reads the picture block
// `gpu_texture_block.rs` packs at the end of `parts` (a face's `Face.texture` word points to its
// entry) and the atlas `gpu_textures.rs` lays out, bound here as group 2 binding 7: a
// `texture_2d<u32>` of `Rgba8Uint` texels, straight sRGB, row 0 the bottom, read with
// `textureLoad` (level 0, no sampler). Part of `PATH_WGSL` after `path.wgsl`, whose `faces`,
// `parts`, `path_srgb` and `path_is_finite` it reads (`path_shader.rs` concatenates them).
//
// `path_picture_at` follows `texture_lookup_f32` (`gpu_texture_block.rs`) step for step, a test
// pinning the text: the coordinate is the face's packed corner coordinates weighted by the hit's
// `u`, `v`, `w` (the weights `path_surface_point` gives corners `v0`, `v1`, `v2`); each side wraps
// (`t - floor(t)`, a non-finite coordinate reading as 0), splits into `x = wrapped * size - 0.5`,
// `x0 = floor(x)` and the fraction `x - x0`, and takes columns `x0` and `x0 + 1` wrapped into the
// picture (`x0` is at least -1, so `x0 + size` is never negative and the remainder is taken in
// `u32`, never of a negative value); only then is the picture's atlas origin added, so a read
// never leaves the picture's own rectangle. Each of the four texels decodes through the face
// colors' sRGB table, lies over white by its alpha (`alpha * decoded + (1 - alpha)`), and the four
// blend bottom, top, then up, each `a + t * (b - a)` (written out: `mix` may be evaluated another
// way).
//
// A face without a picture answers exactly 1 before reading anything, and the path loop multiplies
// the base color by that 1 after the face color, one more multiply with no other operation moved.
// A product with exactly 1 is the base color itself, so a scene without pictures computes, without
// fusing, exactly the values it did before; a driver may still contract a multiply and an add into
// one fused step anywhere in the shader, as WGSL allows, and could do so before pictures too.
//
// Where a face shows a picture the card's factor can differ from the mirror's, and the CPU's, in
// four small ways: the `f32` hit, the `f32` coordinate, a driver contracting `alpha * decoded +
// clear`, and WGSL's `f32` division, which need not round correctly: `f32(texel.a) / 255.0f` may be
// a few units in the last place from the mirror's. That last is harmless: an alpha of 0 still
// divides to 0, so a see-through texel still shows exactly the face's own color; a white texel
// (decoded 1) still gives exactly 1 whatever its alpha rounds to, `alpha + (1 - alpha)` rounding
// back to 1 in `f32`; and elsewhere a few units in the last place of alpha move the factor by less
// than 1e-6. `gpu_texture_pack_tests` and `gpu_texture_shader_tests` bound the rest on the
// mirror.
//
// **Where it is called.** A driver copies a function into every place it is called, and four texel
// loads copied five times grew the compiled path loop by 8%, so the lookup is called from three
// places, each one face's: once a path vertex (`path_radiance`, at the vertex's own face, or, on
// the floor under glass resting there, the glass face across, which the vertex hands the shadow
// ray in `rest_picture`), once a crossing of the line toward the sun (`path_sun_line`), and once
// a seam, at the covering face (`seam_at`, which both of those call, so a driver copies that
// lookup twice: four copies in all; `seam.wgsl` says why it cannot share the hit face's). The one
// `textureLoad` is in `path_picture_texel`, which `path_picture_at` calls four times. The seam's
// lookup, its point on the covering face and its comparison grew the joined source by 7,389
// bytes, and naga's translation of the path entry point by 7,412 bytes of SPIR-V (5.1%), 8,584 of
// HLSL and 8,703 of MSL. naga writes each function once, so a driver that copies `seam_at` into
// both its callers grows by more.

@group(2) @binding(7) var path_atlas: texture_2d<u32>;

// `PathView.flags`: the contact block carries covering triangles (some face shows a picture and
// some glass lies on glass), so a seam reads the covering face's picture (`seam.wgsl`).
const PATH_FLAG_PICTURES: u32 = 256u;

// The picture block's layout (`gpu_texture_block.rs`): a face entry is six coordinate words, then
// the word where its picture record starts; a picture record is its atlas origin, then its size.
// A face's coordinates are shifted by whole repeats of its own: wrap them before use, and never
// compare them with another face's.
const PATH_NO_TEXTURE: u32 = 0u;
const PATH_TEXTURE_FACE_PICTURE: u32 = 6u;
const PATH_TEXTURE_PICTURE_SIZE: u32 = 2u;

// One side of a lookup: the two texel indices inside the picture and how far between them.
struct PathPictureSide {
    first: u32,
    second: u32,
    fraction: f32,
}

// `at` modulo `count` (positive), in [0, count), for `at` in [-1, count]: the only values a side
// hands it. `at + count` is then in [count - 1, 2 count], never negative, so the remainder is
// taken of unsigned values only. A signed `%` of a negative `at` is never taken: on a card
// (Vulkan, RTX 5070) that form read column `(2^32 - 1) mod count` for `at = -1`, the wrong texel
// on every picture side that is not a power of two.
fn path_picture_index(at: i32, count: i32) -> u32 {
    return u32(at + count) % u32(count);
}

// `side_spot_f32`: one side's texel indices and fraction in a picture `size_in` texels long.
fn path_picture_side(coordinate: f32, size_in: u32) -> PathPictureSide {
    let size = max(size_in, 1u);
    var wrapped = 0.0f;
    if path_is_finite(coordinate) {
        wrapped = coordinate - floor(coordinate);
    }
    // A side is at most the atlas's, far below 2^24, so it is exact in f32 and fits an i32.
    let x = wrapped * f32(size) - 0.5f;
    let x0 = floor(x);
    let fraction = x - x0;
    // `x0` is a whole number in [-1, size - 1].
    let first = i32(x0);
    let count = i32(size);
    return PathPictureSide(path_picture_index(first, count), path_picture_index(first + 1, count),
        fraction);
}

// One atlas texel's factor: its color decoded, laid over white by its alpha.
fn path_picture_texel(at: vec2<u32>) -> vec3<f32> {
    let texel = textureLoad(path_atlas, at, 0);
    let alpha = f32(texel.a) / 255.0f;
    let clear = 1.0f - alpha;
    return vec3<f32>(
        alpha * path_srgb.to_linear[texel.r >> 2u][texel.r & 3u] + clear,
        alpha * path_srgb.to_linear[texel.g >> 2u][texel.g & 3u] + clear,
        alpha * path_srgb.to_linear[texel.b >> 2u][texel.b & 3u] + clear,
    );
}

// What the picture on `hit`'s face multiplies the face's color by there: exactly 1 on a face
// without one.
fn path_picture_at(hit: Hit) -> vec3<f32> {
    let entry = faces[hit.source_index].texture;
    if entry == PATH_NO_TEXTURE {
        return vec3<f32>(1.0f);
    }
    let u = bitcast<f32>(parts[entry]) * hit.u + bitcast<f32>(parts[entry + 2u]) * hit.v
        + bitcast<f32>(parts[entry + 4u]) * hit.w;
    let v = bitcast<f32>(parts[entry + 1u]) * hit.u + bitcast<f32>(parts[entry + 3u]) * hit.v
        + bitcast<f32>(parts[entry + 5u]) * hit.w;
    let picture = parts[entry + PATH_TEXTURE_FACE_PICTURE];
    let columns = path_picture_side(u, parts[picture + PATH_TEXTURE_PICTURE_SIZE]);
    let rows = path_picture_side(v, parts[picture + PATH_TEXTURE_PICTURE_SIZE + 1u]);
    let x = parts[picture];
    let y = parts[picture + 1u];
    let bottom_left = path_picture_texel(vec2<u32>(columns.first + x, rows.first + y));
    let bottom_right = path_picture_texel(vec2<u32>(columns.second + x, rows.first + y));
    let top_left = path_picture_texel(vec2<u32>(columns.first + x, rows.second + y));
    let top_right = path_picture_texel(vec2<u32>(columns.second + x, rows.second + y));
    let bottom = bottom_left + columns.fraction * (bottom_right - bottom_left);
    let top = top_left + columns.fraction * (top_right - top_left);
    return bottom + rows.fraction * (top - bottom);
}
