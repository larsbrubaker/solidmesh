// The flat tree's WGSL compute traversal: closest-hit and any-hit over the node and triangle
// storage buffers, mirroring traverse.rs function by function so a reviewer can read the two side
// by side. The byte layouts are the ones `flat_tree`'s module header specifies; the two records
// here that are the shader's own (a ray, a hit) are `shader.rs`'s and are pinned by a test.
//
// WGSL may assume no NaN or infinities, so nothing here relies on them: the reciprocal direction
// is clamped finite (`safe_inverse`), stored bounds are finite, and a ray's direction must be a
// non-zero unit vector (the same precondition traverse.rs has).
//
// Each invocation traces one ray and writes one hit at the ray's index, so a dispatch of any size
// can cover a contiguous slice of the ray buffer.
//
// Two tolerances are WGSL's own and differ from traverse.rs. WGSL allows a compiler to reassociate
// or fuse `a * b - c * d`, and allows 2.5 ULP of error in a division where Rust's is correctly
// rounded, so this shader does not promise the exact negatives on a shared edge that traverse.rs
// gets. Instead an edge value within `EDGE_SNAP` of its two products counts as zero, which means
// inside, so a ray on a shared edge is taken by a triangle on one side or the other however the
// expression was evaluated. `SLAB_WIDEN` budgets the bound subtraction (half an ULP), the
// reciprocal (2.5 ULP of division under WGSL's accuracy rules), the product (half an ULP) and the
// widening's own rounding, about 4 epsilons of relative error, and takes 8 for margin; traverse.rs
// gets away with 4.

// ---- Constants, equal to their Rust counterparts in layout.rs and traverse.rs.

const EMPTY: u32 = 0xFFFFFFFFu;
const LEAF_BIT: u32 = 0x80000000u;
const COUNT_BITS: u32 = 2u;
const COUNT_MASK: u32 = 3u;
const FACE_FRONT: u32 = 1u;
const FACE_BACK: u32 = 2u;
const STACK_CAPACITY: u32 = 128u;
// f32::EPSILON spelled exactly: the shader's tolerances are multiples of it.
const F32_EPSILON: f32 = 1.1920928955078125e-7f;
// traverse.rs's SLAB_WIDEN doubled, for WGSL's division accuracy; the budget is in the header.
const SLAB_WIDEN: f32 = 8.0f * F32_EPSILON;
// An edge value within this fraction of its products' magnitudes counts as zero: a fused or
// reassociated `a * b - c * d` differs from the rounded one by about an ULP of each product.
const EDGE_SNAP: f32 = 4.0f * F32_EPSILON;
const MIN_DIRECTION: f32 = 1e-18f;
// Stands in for "any distance" when a ray runs parallel to a slab (traverse.rs, PARALLEL_SPAN).
const PARALLEL_SPAN: f32 = 1e37f;
// A hit's `triangle` when the ray found nothing.
const NO_HIT: u32 = 0xFFFFFFFFu;

// ---- Records, field for field the Rust layouts.

struct Node {
    min_x: vec4<f32>,
    max_x: vec4<f32>,
    min_y: vec4<f32>,
    max_y: vec4<f32>,
    min_z: vec4<f32>,
    max_z: vec4<f32>,
    child: vec4<u32>,
}

struct Triangle {
    v0: array<f32, 3>,
    v1: array<f32, 3>,
    v2: array<f32, 3>,
    source_index: u32,
}

// `FlatRay::to_le_bytes` writes this: 48 bytes.
struct Ray {
    origin: vec3<f32>,
    t_min: f32,
    direction: vec3<f32>,
    t_max: f32,
    face_mask: u32,
}

// `FlatHit` in 28 bytes; `triangle` is NO_HIT for a miss.
struct Hit {
    t: f32,
    triangle: u32,
    source_index: u32,
    front_face: u32,
    u: f32,
    v: f32,
    w: f32,
}

// The slice of the ray buffer one dispatch covers.
struct Params {
    first_ray: u32,
    ray_count: u32,
}

@group(0) @binding(0) var<storage, read> nodes: array<Node>;
@group(0) @binding(1) var<storage, read> triangles: array<Triangle>;
@group(0) @binding(2) var<storage, read> rays: array<Ray>;
@group(0) @binding(3) var<storage, read_write> hits: array<Hit>;
@group(0) @binding(4) var<uniform> params: Params;

// ---- layout.rs's child_code.

fn is_leaf(code: u32) -> bool {
    return (code & LEAF_BIT) != 0u && code != EMPTY;
}

fn first_triangle(code: u32) -> u32 {
    return (code & ~LEAF_BIT) >> COUNT_BITS;
}

fn leaf_count(code: u32) -> u32 {
    return (code & COUNT_MASK) + 1u;
}

// ---- traverse.rs.

fn safe_inverse(direction: f32) -> f32 {
    var clamped = direction;
    if abs(direction) < MIN_DIRECTION {
        if direction >= 0.0f {
            clamped = MIN_DIRECTION;
        } else {
            clamped = -MIN_DIRECTION;
        }
    }
    return 1.0f / clamped;
}

// traverse.rs's `slab`, all four children at once.
struct Slab {
    near: vec4<f32>,
    far: vec4<f32>,
}

fn slab(min_bound: vec4<f32>, max_bound: vec4<f32>, origin: f32, direction: f32, inverse: f32) -> Slab {
    if abs(direction) < MIN_DIRECTION {
        let inside = (vec4<f32>(origin) >= min_bound) & (vec4<f32>(origin) <= max_bound);
        return Slab(
            select(vec4<f32>(PARALLEL_SPAN), vec4<f32>(-PARALLEL_SPAN), inside),
            select(vec4<f32>(-PARALLEL_SPAN), vec4<f32>(PARALLEL_SPAN), inside),
        );
    }
    let t0 = (min_bound - vec4<f32>(origin)) * vec4<f32>(inverse);
    let t1 = (max_bound - vec4<f32>(origin)) * vec4<f32>(inverse);
    return Slab(min(t0, t1), max(t0, t1));
}

// `p * q - r * s`, zero when it is within EDGE_SNAP of its two products (see the header).
fn edge_value(p: f32, q: f32, r: f32, s: f32) -> f32 {
    let first = p * q;
    let second = r * s;
    let difference = first - second;
    if abs(difference) <= EDGE_SNAP * (abs(first) + abs(second)) {
        return 0.0f;
    }
    return difference;
}

struct Shear {
    kx: u32,
    ky: u32,
    kz: u32,
    sx: f32,
    sy: f32,
    sz: f32,
}

fn shear_of(direction: vec3<f32>) -> Shear {
    var kz = 0u;
    if abs(direction[1]) > abs(direction[kz]) {
        kz = 1u;
    }
    if abs(direction[2]) > abs(direction[kz]) {
        kz = 2u;
    }
    var kx = (kz + 1u) % 3u;
    var ky = (kx + 1u) % 3u;
    // Keeps the projected winding the same when the ray runs against its major axis.
    if direction[kz] < 0.0f {
        let swap = kx;
        kx = ky;
        ky = swap;
    }
    let sz = 1.0f / direction[kz];
    return Shear(kx, ky, kz, direction[kx] * sz, direction[ky] * sz, sz);
}

// The Option<(t, barycentric, front)> hit_triangle returns.
struct TriangleHit {
    found: bool,
    t: f32,
    u: f32,
    v: f32,
    w: f32,
    front_face: bool,
}

fn no_triangle_hit() -> TriangleHit {
    return TriangleHit(false, 0.0f, 0.0f, 0.0f, 0.0f, false);
}

fn hit_triangle(triangle: Triangle, ray: Ray, shear: Shear, t_best: f32) -> TriangleHit {
    let kx = shear.kx;
    let ky = shear.ky;
    let kz = shear.kz;
    let a = vec3<f32>(
        triangle.v0[0] - ray.origin[0],
        triangle.v0[1] - ray.origin[1],
        triangle.v0[2] - ray.origin[2],
    );
    let b = vec3<f32>(
        triangle.v1[0] - ray.origin[0],
        triangle.v1[1] - ray.origin[1],
        triangle.v1[2] - ray.origin[2],
    );
    let c = vec3<f32>(
        triangle.v2[0] - ray.origin[0],
        triangle.v2[1] - ray.origin[1],
        triangle.v2[2] - ray.origin[2],
    );

    let ax = a[kx] - shear.sx * a[kz];
    let ay = a[ky] - shear.sy * a[kz];
    let bx = b[kx] - shear.sx * b[kz];
    let by = b[ky] - shear.sy * b[kz];
    let cx = c[kx] - shear.sx * c[kz];
    let cy = c[ky] - shear.sy * c[kz];

    let u = edge_value(cx, by, cy, bx);
    let v = edge_value(ax, cy, ay, cx);
    let w = edge_value(bx, ay, by, ax);

    if (u < 0.0f || v < 0.0f || w < 0.0f) && (u > 0.0f || v > 0.0f || w > 0.0f) {
        return no_triangle_hit();
    }

    let det = u + v + w;
    if det == 0.0f {
        return no_triangle_hit();
    }

    let az = shear.sz * a[kz];
    let bz = shear.sz * b[kz];
    let cz = shear.sz * c[kz];
    // The weights first, as in traverse.rs: u, v and w share a sign here, so each is in 0..=1.
    let u_weight = u / det;
    let v_weight = v / det;
    let w_weight = w / det;
    let t = u_weight * az + v_weight * bz + w_weight * cz;
    if !(t >= ray.t_min && t <= t_best) {
        return no_triangle_hit();
    }

    let front_face = det > 0.0f;
    var wanted = FACE_BACK;
    if front_face {
        wanted = FACE_FRONT;
    }
    if (ray.face_mask & wanted) == 0u {
        return no_triangle_hit();
    }

    return TriangleHit(true, t, u_weight, v_weight, w_weight, front_face);
}

fn trace_closest(ray: Ray) -> Hit {
    return trace(ray, false);
}

// The hit found first; `triangle` is NO_HIT when the ray is unoccluded.
fn trace_any(ray: Ray) -> Hit {
    return trace(ray, true);
}

fn trace(ray: Ray, stop_at_first_hit: bool) -> Hit {
    let inverse = vec3<f32>(
        safe_inverse(ray.direction[0]),
        safe_inverse(ray.direction[1]),
        safe_inverse(ray.direction[2]),
    );
    let shear = shear_of(ray.direction);

    var stack_code: array<u32, STACK_CAPACITY>;
    var stack_near: array<f32, STACK_CAPACITY>;
    var stack_size = 1u;
    stack_code[0] = 0u;
    stack_near[0] = ray.t_min;

    var best = Hit(0.0f, NO_HIT, 0u, 0u, 0.0f, 0.0f, 0.0f);
    var t_best = ray.t_max;

    while stack_size > 0u {
        stack_size -= 1u;
        let code = stack_code[stack_size];
        // A nearer hit may have been found since this entry was pushed.
        if stack_near[stack_size] > t_best {
            continue;
        }

        if is_leaf(code) {
            let first = first_triangle(code);
            let count = leaf_count(code);
            for (var offset = 0u; offset < count; offset += 1u) {
                let triangle = triangles[first + offset];
                let found = hit_triangle(triangle, ray, shear, t_best);
                if found.found {
                    t_best = found.t;
                    var front = 0u;
                    if found.front_face {
                        front = 1u;
                    }
                    best = Hit(found.t, first + offset, triangle.source_index, front, found.u,
                        found.v, found.w);
                    if stop_at_first_hit {
                        return best;
                    }
                }
            }
            continue;
        }

        let node = nodes[code];
        // All four slabs at once (the record is structure-of-arrays for this); traverse.rs does
        // the same arithmetic per slot and skips empty slots first. Slot order is unchanged.
        let slab_x = slab(node.min_x, node.max_x, ray.origin[0], ray.direction[0], inverse[0]);
        let slab_y = slab(node.min_y, node.max_y, ray.origin[1], ray.direction[1], inverse[1]);
        let slab_z = slab(node.min_z, node.max_z, ray.origin[2], ray.direction[2], inverse[2]);
        let near_all = max(max(slab_x.near, slab_y.near), slab_z.near);
        let far_all = min(min(slab_x.far, slab_y.far), slab_z.far);
        let near_widened = near_all - abs(near_all) * SLAB_WIDEN;
        let far_widened = far_all + abs(far_all) * SLAB_WIDEN;

        // The entered children, farthest first, so the nearest ends up on top of the stack.
        var order: array<u32, 4>;
        var order_near: array<f32, 4>;
        var order_count = 0u;
        for (var slot = 0u; slot < 4u; slot += 1u) {
            if node.child[slot] == EMPTY {
                continue;
            }
            let near = near_widened[slot];
            let far = far_widened[slot];
            if near <= far && far >= ray.t_min && near <= t_best {
                // Insertion by descending distance; ties keep slot order, deterministically.
                var at = order_count;
                while at > 0u && order_near[at - 1u] < near {
                    order[at] = order[at - 1u];
                    order_near[at] = order_near[at - 1u];
                    at -= 1u;
                }
                order[at] = node.child[slot];
                order_near[at] = near;
                order_count += 1u;
            }
        }
        for (var position = 0u; position < order_count; position += 1u) {
            stack_code[stack_size] = order[position];
            stack_near[stack_size] = order_near[position];
            stack_size += 1u;
        }
    }

    return best;
}

// ---- The entry points: one ray per invocation, written back at the ray's index.

@compute @workgroup_size(64)
fn trace_closest_main(@builtin(global_invocation_id) id: vec3<u32>) {
    if id.x >= params.ray_count {
        return;
    }
    let index = params.first_ray + id.x;
    hits[index] = trace_closest(rays[index]);
}

@compute @workgroup_size(64)
fn trace_any_main(@builtin(global_invocation_id) id: vec3<u32>) {
    if id.x >= params.ray_count {
        return;
    }
    let index = params.first_ray + id.x;
    hits[index] = trace_any(rays[index]);
}
