// What a path does at a seam, in WGSL: a hit on a face of a glass part that a face of another
// glass part lies on. It mirrors `path_tracer/glass_seam.rs` branch by branch, reading the contact
// block `gpu_contacts.rs` packs after the part records (its header gives the words); `path.wgsl`'s
// path loop calls `seam_at` only when `PATH_FLAG_GLASS_CONTACTS` is set, and only for the path's
// own hits: on the CPU too, shadow rays and coverage never look at a seam. Nothing here traverses
// the tree. It follows `catcher.wgsl` in `PATH_WGSL`.
//
// The point test is the CPU's `covers` on the packed edge equations, evaluated in f32 in the order
// `GlassContactRecord::covers_f32` evaluates them, so a test can check the equations in Rust. Every
// surface comparison (same glass, which is denser, the interface's surface) was decided in f64
// when the scene was packed and arrives as flags and a `Surface` record.

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
    // `(true, true) if here == there`: one glass, counted as one crossing (`count_pass`).
    if here_present && there_present && (flags & CONTACT_SAME_GLASS) != 0u {
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
        seam.surface = path_surface_at(point.part, point.color);
        seam.into_denser = !leaving_here;
        seam.absent_part = there_part;
        seam.absent_on_normal_side = true;
    } else if !here_present && there_present {
        // `(false, true)`: the covering part's glass against air; the hit part is absent.
        seam.surface = path_surface_at(there_part, parts[at + CONTACT_WORD_COLOR]);
        seam.into_denser = leaving_here;
        seam.absent_part = point.part;
        seam.absent_on_normal_side = false;
    } else {
        // `(true, true)`, different glasses: one interface of their relative index, denser on
        // the side the path goes into when that glass's index is at least the other's.
        seam.surface = seam_interface(at);
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
