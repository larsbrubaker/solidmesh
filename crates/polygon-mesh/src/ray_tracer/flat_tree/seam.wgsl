// What a path does at a seam, in WGSL: a hit on a face of a glass part that a face of another
// glass part lies on. It mirrors `path_tracer/glass_seam.rs` branch by branch, reading the contact
// block `gpu_contacts.rs` packs after the part records (its header gives the words). `seam_at` is
// asked only when `PATH_FLAG_GLASS_CONTACTS` is set: by `path.wgsl`'s path loop at the path's own
// hits, and by `sun.wgsl`'s line toward the sun at its hits when it walks straight through glass
// (`PATH_FLAG_SUN_STRAIGHT`), as the CPU's `through_glass` does; a shadow ray that stops at any
// surface and the coverage line never look at a seam, on the CPU too. Nothing here traverses the
// tree. It follows `catcher.wgsl` in `PATH_WGSL`.
//
// The point test is the CPU's `covers` on the packed edge equations, evaluated in f32 in the order
// `GlassContactRecord::covers_f32` evaluates them, so a test can check the equations in Rust. Every
// surface comparison (same glass, which is denser, the interface's surface) was decided in f64
// when the scene was packed and arrives as flags and a `Surface` record, except, in a scene with
// pictures, where a picture tints either face (below).
//
// **Pictures.** The hit face's picture reaches a seam on the point (`picture.wgsl`). In a scene
// with pictures (`PATH_FLAG_PICTURES`) the seam reads the covering face's too, as the CPU's
// `glass_seam.rs` does: the block carries each entry's covering triangle, the point is placed on
// that triangle's own corners (`seam_cover_hit`, the CPU's `barycentric_at` in f32) and its
// picture looked up there, once the covering part is found there (the only cases that read it).
// Where either face's factor then differs from 1 the pack's color-only decisions give way to the
// surfaces themselves, as the CPU compares them: one glass when the materials are the same apart
// from color (`CONTACT_SAME_MATERIAL`, decided in f64) and the two tinted base colors are equal,
// and two glasses meet at the pack's interface with its color the two tinted colors multiplied.
// The covering part's glass against air takes its face's picture. Each face's coordinates are
// read on its own face only, and only the factors meet. What is left between the routes is f32
// and the frame. The hit and the lookups are f32. The tinted colors are compared in f32 where the
// CPU compares them in f64, so two colors closer than f32 resolves can be one glass here and two
// there; the same picture on both faces, read at two coordinates, is noisy on both routes but not
// on the same pixels. And the covering point is placed in the scene's frame, on the face's packed
// corners, where the CPU places it in the part's own mesh space: the two agree on the face and
// differ only off it, by about one contact tolerance, under a non-uniform scale.
//
// **Where the lookup is called.** `seam_at` adds one call of `path_picture_at`, three in all. A
// driver copies `seam_at` into the path loop and the line toward the sun, so the lookup's four
// texel loads are copied four times where they were twice; the seam's lookup cannot share the
// path vertex's or the crossing's, which have read the hit face's picture by then and need it
// beside the covering one's.

const CONTACT_ENTRY_WORDS: u32 = 29u;
// Where an entry's fields sit, in words from its first, and an edge's within its five.
const CONTACT_EDGE_WORDS: u32 = 5u;
const CONTACT_EDGE_START: u32 = 0u;
const CONTACT_EDGE_DIRECTION: u32 = 2u;
const CONTACT_EDGE_ALLOWANCE: u32 = 4u;
const CONTACT_WORD_DROP: u32 = 15u;
const CONTACT_WORD_TOLERANCE: u32 = 16u;
const CONTACT_WORD_PART: u32 = 17u;
const CONTACT_WORD_FLAGS: u32 = 18u;
const CONTACT_WORD_COLOR: u32 = 19u;
const CONTACT_WORD_SURFACE: u32 = 20u;
const CONTACT_FULLY_COVERED: u32 = 0x80000000u;
const CONTACT_SAME_GLASS: u32 = 1u;
const CONTACT_HIT_DENSER: u32 = 2u;
const CONTACT_COVER_DENSER: u32 = 4u;
const CONTACT_NO_SEAM: u32 = 8u;
const CONTACT_SAME_MATERIAL: u32 = 16u;
// `glass_seam.rs`'s `SKIP_TOLERANCES`.
const CONTACT_SKIP_TOLERANCES: f32 = 2.0f;

// `Seam`: no seam at all, `Straight`, `Interface` and `Ended`.
const SEAM_NONE: u32 = 0u;
const SEAM_STRAIGHT: u32 = 1u;
const SEAM_INTERFACE: u32 = 2u;
const SEAM_ENDED: u32 = 3u;
const SEAM_NO_PART: u32 = 0xFFFFFFFFu;
const SEAM_NO_ENTRY: u32 = 0xFFFFFFFFu;

// `Seam` and its `Interface`: the surface the scattering model sees, whether the side the path
// came from is the thinner, the skip off the seam, and the part not there on the line that reached
// it with whether its side is the one the hit face's normal points to.
struct PathSeam {
    kind: u32,
    surface: Surface,
    into_denser: bool,
    skip: f32,
    absent_part: u32,
    absent_on_normal_side: bool,
}

fn seam_none() -> PathSeam {
    var seam: PathSeam;
    seam.kind = SEAM_NONE;
    seam.into_denser = false;
    seam.skip = 0.0f;
    seam.absent_part = SEAM_NO_PART;
    seam.absent_on_normal_side = false;
    return seam;
}

fn seam_f32(at: u32) -> f32 {
    return bitcast<f32>(parts[at]);
}

// `covers`: the entry at word `at` covers `position`, each edge's allowance already the tolerance
// times its length on a widened edge and 0 on any other.
fn seam_covers(at: u32, position: vec3<f32>) -> bool {
    let drop = parts[at + CONTACT_WORD_DROP];
    var point = vec2<f32>(position.x, position.y);
    if drop == 0u {
        point = vec2<f32>(position.y, position.z);
    } else if drop == 1u {
        point = vec2<f32>(position.z, position.x);
    }
    for (var edge = 0u; edge < 3u; edge += 1u) {
        let base = at + edge * CONTACT_EDGE_WORDS;
        let start = base + CONTACT_EDGE_START;
        let direction = base + CONTACT_EDGE_DIRECTION;
        let side = seam_f32(direction) * (point.y - seam_f32(start + 1u))
            - seam_f32(direction + 1u) * (point.x - seam_f32(start));
        if !(side >= -seam_f32(base + CONTACT_EDGE_ALLOWANCE)) {
            return false;
        }
    }
    return true;
}

// `GlassContacts::covering`: the word of the first entry of `triangle` covering `position`, in
// list order, or `SEAM_NO_ENTRY` where the face meets air there. A face its first entry covers
// whole needs no point test.
fn seam_covering(triangle: u32, position: vec3<f32>) -> u32 {
    let block = path_view.contact_block;
    let count = parts[block];
    if triangle >= count {
        return SEAM_NO_ENTRY;
    }
    let start = parts[block + 1u + triangle];
    let first = start & ~CONTACT_FULLY_COVERED;
    let end = parts[block + 2u + triangle] & ~CONTACT_FULLY_COVERED;
    if first >= end {
        return SEAM_NO_ENTRY;
    }
    let entries = block + 2u + count;
    if (start & CONTACT_FULLY_COVERED) != 0u {
        return entries + first * CONTACT_ENTRY_WORDS;
    }
    for (var entry = first; entry < end; entry += 1u) {
        let at = entries + entry * CONTACT_ENTRY_WORDS;
        if seam_covers(at, position) {
            return at;
        }
    }
    return SEAM_NO_ENTRY;
}

// The covering triangle of the entry at word `at`: in a scene with pictures one word per entry
// follows the entries, the covering face's index in `triangles`.
fn seam_cover_triangle(at: u32) -> u32 {
    let block = path_view.contact_block;
    let count = parts[block];
    let entries = block + 2u + count;
    let total = parts[block + 1u + count] & ~CONTACT_FULLY_COVERED;
    return parts[entries + total * CONTACT_ENTRY_WORDS + (at - entries) / CONTACT_ENTRY_WORDS];
}

// How far the point the weights `w` rebuild on the corners lies from `position`, squared.
fn seam_rebuilt_distance(c0: vec3<f32>, c1: vec3<f32>, c2: vec3<f32>, w: vec3<f32>,
    position: vec3<f32>) -> f32 {
    let at = c0 * w.x + c1 * w.y + c2 * w.z - position;
    return dot(at, at);
}

// How far along the edge from `start` to `end` the point nearest `position` lies, in [0, 1]; 0 on
// an edge of no length.
fn seam_edge_along(start: vec3<f32>, end: vec3<f32>, position: vec3<f32>) -> f32 {
    let edge = end - start;
    let edge_squared = dot(edge, edge);
    var t = 0.0f;
    if edge_squared > 0.0f {
        t = clamp(dot(position - start, edge) / edge_squared, 0.0f, 1.0f);
    }
    return t;
}

// `cover_weights_f32` (the CPU's `barycentric_at`): a hit on packed triangle `cover` at
// `position`, its weights the sub-triangle areas' ratios where they place the point inside, else
// (or where it rebuilds a nearer point) the nearest edge point's, for the picture lookup, which
// reads only the face and the weights. The corners are three values and the edges three straight
// steps with their weights written out, so nothing is indexed by a variable: a local array or a
// vector written by a variable index costs some drivers scratch memory.
fn seam_cover_hit(cover: u32, position: vec3<f32>) -> Hit {
    let triangle = triangles[cover];
    let c0 = path_vec3(triangle.v0);
    let c1 = path_vec3(triangle.v1);
    let c2 = path_vec3(triangle.v2);
    let normal = cross(c1 - c0, c2 - c0);
    let normal_squared = dot(normal, normal);
    var best = vec3<f32>(1.0f / 3.0f);
    var nearest = 0.0f;
    var found = false;
    if normal_squared > 0.0f && path_is_finite(normal_squared) {
        let w0 = dot(normal, cross(c1 - position, c2 - position)) / normal_squared;
        let w1 = dot(normal, cross(c2 - position, c0 - position)) / normal_squared;
        let w2 = dot(normal, cross(c0 - position, c1 - position)) / normal_squared;
        let sum = w0 + w1 + w2;
        if w0 >= 0.0f && w1 >= 0.0f && w2 >= 0.0f && sum > 0.0f && path_is_finite(sum) {
            let weights = vec3<f32>(w0 / sum, w1 / sum, w2 / sum);
            let distance = seam_rebuilt_distance(c0, c1, c2, weights, position);
            if path_is_finite(distance) {
                best = weights;
                nearest = distance;
                found = true;
            }
        }
    }
    // The edges from corner 0 to 1, 1 to 2 and 2 to 0, each taken where it is strictly nearer.
    let t01 = seam_edge_along(c0, c1, position);
    let w01 = vec3<f32>(1.0f - t01, t01, 0.0f);
    let d01 = seam_rebuilt_distance(c0, c1, c2, w01, position);
    if path_is_finite(d01) && (!found || d01 < nearest) {
        best = w01;
        nearest = d01;
        found = true;
    }
    let t12 = seam_edge_along(c1, c2, position);
    let w12 = vec3<f32>(0.0f, 1.0f - t12, t12);
    let d12 = seam_rebuilt_distance(c0, c1, c2, w12, position);
    if path_is_finite(d12) && (!found || d12 < nearest) {
        best = w12;
        nearest = d12;
        found = true;
    }
    let t20 = seam_edge_along(c2, c0, position);
    let w20 = vec3<f32>(t20, 0.0f, 1.0f - t20);
    let d20 = seam_rebuilt_distance(c0, c1, c2, w20, position);
    if path_is_finite(d20) && (!found || d20 < nearest) {
        best = w20;
    }
    return Hit(0.0f, cover, triangle.source_index, 0u, best.x, best.y, best.z);
}

// The interface surface the pack worked out with `seam_surface`.
fn seam_interface(at: u32) -> Surface {
    let s = at + CONTACT_WORD_SURFACE;
    return Surface(
        seam_f32(s), seam_f32(s + 1u), seam_f32(s + 2u), seam_f32(s + 3u), seam_f32(s + 4u),
        seam_f32(s + 5u), seam_f32(s + 6u), seam_f32(s + 7u), parts[s + 8u],
    );
}

// `glass_seam`: the seam at `point` on packed triangle `triangle`, for a path arriving along
// `direction` with the hit part there on this line when `here_present`.
fn seam_at(point: PathPoint, triangle: u32, direction: vec3<f32>, here_present: bool) -> PathSeam {
    var seam = seam_none();
    let at = seam_covering(triangle, point.position);
    if at == SEAM_NO_ENTRY {
        return seam;
    }
    let flags = parts[at + CONTACT_WORD_FLAGS];
    // A surface the pack could not form: the CPU answers no seam for the first covering entry.
    if (flags & CONTACT_NO_SEAM) != 0u {
        return seam;
    }
    let there_part = parts[at + CONTACT_WORD_PART];
    // Whether the covering part is there, by the same per-line rule at the same point of the line.
    var there_present = true;
    if (path_view.flags & PATH_FLAG_SEE_THROUGH) != 0u {
        let verdict = line_at_hit(there_part);
        if verdict == LINE_ENDED {
            seam.kind = SEAM_ENDED;
            return seam;
        }
        there_present = verdict == LINE_STOPS;
    }
    seam.skip = CONTACT_SKIP_TOLERANCES * seam_f32(at + CONTACT_WORD_TOLERANCE);
    let leaving_here = path_facing(point, direction).back_face;
    // `(false, false)`: neither part there, the line goes on past both.
    if !here_present && !there_present {
        seam.kind = SEAM_STRAIGHT;
        return seam;
    }
    // The covering face's picture, where the covering part is there to show it.
    let there_color = parts[at + CONTACT_WORD_COLOR];
    var there_picture = vec3<f32>(1.0f);
    var pictured = false;
    if there_present && (path_view.flags & PATH_FLAG_PICTURES) != 0u {
        there_picture = path_picture_at(seam_cover_hit(seam_cover_triangle(at), point.position));
        pictured = any(there_picture != vec3<f32>(1.0f)) || any(point.picture != vec3<f32>(1.0f));
    }
    var same = (flags & CONTACT_SAME_GLASS) != 0u;
    var tint = vec3<f32>(0.0f);
    if pictured && here_present {
        let here_surface = path_surface_at(point.part, point.color, point.picture);
        let there_surface = path_surface_at(there_part, there_color, there_picture);
        same = (flags & CONTACT_SAME_MATERIAL) != 0u && here_surface.base_r == there_surface.base_r
            && here_surface.base_g == there_surface.base_g
            && here_surface.base_b == there_surface.base_b;
        tint = vec3<f32>(here_surface.base_r * there_surface.base_r,
            here_surface.base_g * there_surface.base_g, here_surface.base_b * there_surface.base_b);
    }
    // `(true, true) if here == there`: one glass, counted as one crossing (`count_pass`).
    if here_present && there_present && same {
        if line_crossings >= MAX_CROSSINGS_PER_LINE {
            seam.kind = SEAM_ENDED;
            return seam;
        }
        line_crossings += 1u;
        seam.kind = SEAM_STRAIGHT;
        return seam;
    }
    seam.kind = SEAM_INTERFACE;
    if here_present && !there_present {
        // `(true, false)`: the hit part's glass against air; the covering part, on the side the
        // normal points to, is absent.
        seam.surface = path_surface_at(point.part, point.color, point.picture);
        seam.into_denser = !leaving_here;
        seam.absent_part = there_part;
        seam.absent_on_normal_side = true;
    } else if !here_present && there_present {
        // `(false, true)`: the covering part's glass against air; the hit part is absent.
        seam.surface = path_surface_at(there_part, there_color, there_picture);
        seam.into_denser = leaving_here;
        seam.absent_part = point.part;
        seam.absent_on_normal_side = false;
    } else {
        // `(true, true)`, different glasses: one interface of their relative index, denser on
        // the side the path goes into when that glass's index is at least the other's.
        seam.surface = seam_interface(at);
        if pictured {
            seam.surface.base_r = tint.x;
            seam.surface.base_g = tint.y;
            seam.surface.base_b = tint.z;
        }
        seam.into_denser = select(
            (flags & CONTACT_HIT_DENSER) != 0u,
            (flags & CONTACT_COVER_DENSER) != 0u,
            leaving_here,
        );
    }
    return seam;
}

// `off_seam` (`Interface::next_ray_origin` and the straight pass): past both faces, on the side
// `direction` leaves by.
fn seam_origin(point: PathPoint, direction: vec3<f32>, skip: f32) -> vec3<f32> {
    if dot(direction, point.geometric) >= 0.0f {
        return point.position + point.geometric * skip;
    }
    return point.position - point.geometric * skip;
}

// `Interface::absent_part_toward`: the part not there on the line that reached the seam, when a
// ray leaving along `direction` goes into it; `SEAM_NO_PART` otherwise.
fn seam_absent_toward(seam: PathSeam, point: PathPoint, direction: vec3<f32>) -> u32 {
    if seam.absent_part == SEAM_NO_PART {
        return SEAM_NO_PART;
    }
    let toward_normal_side = dot(direction, point.geometric) >= 0.0f;
    if toward_normal_side == seam.absent_on_normal_side {
        return seam.absent_part;
    }
    return SEAM_NO_PART;
}

// `StraightLine::starts_past`: the new line starts inside `part`, which the line before found not
// there, so its far surface is passed as the rest of it was.
fn line_starts_past(part: u32) {
    if line_passed_count < MAX_CROSSINGS_PER_LINE {
        line_passed[line_passed_count] = part;
        line_passed_count += 1u;
    }
}
