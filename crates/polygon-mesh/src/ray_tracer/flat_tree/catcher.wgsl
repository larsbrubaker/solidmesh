// The shadow-catcher floor and the coverage channels in WGSL, the part of `PATH_WGSL` that runs when
// `PathView.flags` carries `PATH_FLAG_COMPOSITE`. It follows `path.wgsl` (`path_shader.rs`
// concatenates them) and shares its records; `gpu_view.rs` packs the floor into `PathView`.
//
// What a composited sample produces is a coverage (how much of the line the parts cover: 1 on an
// opaque part, 0 on nothing, in between where the line goes through see-through parts) and the
// radiance of the path where there is something on the line. A camera ray that reaches the floor
// past whatever it covers adds no color of its own: it reports how much sun and sky light the parts
// took from that floor point, as a share of the light that would have reached it, and a compositor
// darkens what is under the picture by that share.
//
// **The floor is a shape here, not a part in the tree.** Its extent grows with the distance to the
// eye, so a quad would rebuild the whole tree on every orbit, and a quad a hundred view distances
// out would spread the tree's coordinates so far that ordinary parts lose precision. So
// `path_floor_distance` intersects the plane directly and `path_floor_point` keeps the point's
// height exactly the plane's, instead of a rounded `origin + t * direction`. Paths meet the floor
// through that function, on both sides, in the same order as the CPU's quad; the sun and sky rays
// that ask what the parts took use only the parts' tree, because a catcher's light is the light
// the parts can block.
//
// **Contact shadows.** The rays that ask what the parts took start at the floor's point lifted by a
// `SURFACE_OFFSET_F32_STEPS`-sized margin over the rounding of that point's own coordinates, and
// the plane's height is exact, so the lift always clears the plane: any height above it starts the
// ray on the correct side of a part resting on the floor. A part's bottom face sits at the plane's
// own height to the bit (`gpu_view.rs` narrows both alike), so under the part the ray starts inside
// it and is blocked, and just outside it the ray goes up past the wall exactly as it would from a
// point on the floor.
//
// **An orthographic eye slides** in front of the parts (`gpu_scene.rs`) and that can put the floor
// behind the ray's start: all rays of an orthographic camera are the same lines either way, so the
// catcher reads a distance to the floor that may be negative there. A perspective eye never slides
// and uses the ordinary test.

// The `Part` index a `PathPoint` carries when it is on the floor; not an entry of `parts`.
const PATH_FLOOR_PART: u32 = 0xFFFFFFFEu;
// What `path_catcher_distance` answers when the floor is not on the line.
const PATH_NO_FLOOR: f32 = 3.1e38f;
// `SURFACE_OFFSET_F32_STEPS` in `surface_point.rs`: how many f32 steps of its own rounding a point
// off a surface starts away from it.
const PATH_FLOOR_OFFSET_STEPS: f32 = 8.0f;

// The salts `GPU_SUN_SEED_SALT` and `GPU_SKY_SEED_SALT` in `gpu_view.rs`, in halves.
const PATH_SUN_SALT_LOW: u32 = 0x6c696768u;
const PATH_SUN_SALT_HIGH: u32 = 0x53756e5fu;
const PATH_SKY_SALT_LOW: u32 = 0x6c696768u;
const PATH_SKY_SALT_HIGH: u32 = 0x536b795fu;

// Rec. 709 luminance: a shadow's alpha is a brightness.
const PATH_LUMA: vec3<f32> = vec3<f32>(0.2126f, 0.7152f, 0.0722f);
const PATH_PI: f32 = 3.14159265358979f;

// Whether a path may meet the floor at all right now: a floor is in the view and this path is not
// the look past it through the parts-only scene.
var<private> path_floor_active: bool;

// ---- The floor plane.

// The nearest distance along `ray` (within its window, both sides) to the floor's square, or a
// negative number when the ray misses it or the floor isn't active.
fn path_floor_distance(ray: Ray) -> f32 {
    if !path_floor_active {
        return -1.0f;
    }
    let dz = ray.direction.z;
    if abs(dz) < MIN_DIRECTION {
        return -1.0f;
    }
    let t = (path_view.floor_plane.z - ray.origin.z) / dz;
    if !(t >= ray.t_min && t <= ray.t_max) {
        return -1.0f;
    }
    let at = ray.origin.xy + ray.direction.xy * t;
    let from_center = abs(at - path_view.floor_plane.xy);
    if from_center.x > path_view.floor_plane.w || from_center.y > path_view.floor_plane.w {
        return -1.0f;
    }
    return t;
}

// The margin a ray leaving the floor at `position` starts by: rounding-sized in f32.
fn path_floor_offset(position: vec3<f32>) -> f32 {
    return (abs(position.x) + abs(position.y) + abs(position.z)) * F32_EPSILON
        * PATH_FLOOR_OFFSET_STEPS;
}

// The point on the floor at distance `t` along `ray`. The height is the plane's own, not the
// rounded sum: that is what keeps the offset above a floor it would otherwise round into.
fn path_floor_point(ray: Ray, t: f32) -> PathPoint {
    let at = ray.origin.xy + ray.direction.xy * t;
    let position = vec3<f32>(at.x, at.y, path_view.floor_plane.z);
    let up = vec3<f32>(0.0f, 0.0f, 1.0f);
    return PathPoint(position, up, up, path_floor_offset(position), PATH_FLOOR_PART, 0xFFFFFFu);
}

fn path_floor_surface() -> Surface {
    let base = path_view.floor_base;
    let rest = path_view.floor_rest;
    return Surface(base.x, base.y, base.z, base.w, rest.x, rest.y, rest.z, rest.w,
        path_view.floor_surface_flags);
}

// How far along a camera ray it reaches the floor from above, or `PATH_NO_FLOOR`. A floor behind
// the slid start of an orthographic ray still counts (see the header).
fn path_catcher_distance(ray: Ray) -> f32 {
    if !(ray.direction.z < 0.0f) {
        return PATH_NO_FLOOR;
    }
    let t = (path_view.floor_plane.z - ray.origin.z) / ray.direction.z;
    if t <= 0.0f && (path_view.flags & PATH_FLAG_ORTHOGRAPHIC) == 0u {
        return PATH_NO_FLOOR;
    }
    let at = ray.origin.xy + ray.direction.xy * t;
    let from_center = abs(at - path_view.floor_plane.xy);
    if from_center.x > path_view.floor_plane.w || from_center.y > path_view.floor_plane.w {
        return PATH_NO_FLOOR;
    }
    return t;
}

// ---- Coverage along a line, deterministic.

// How much of `origin_ray` the parts cover (`RenderScene::coverage_along`): a part counted once
// however many of its surfaces the line crosses, and its limit `t_max` applying to the first
// stretch only, since a continuing line starts anew past the surface it went through.
fn path_coverage_along(origin_ray: Ray) -> f32 {
    line_restart();
    var ray = origin_ray;
    let direction = origin_ray.direction;
    var uncovered = 1.0f;
    loop {
        let hit = trace_closest(ray);
        if hit.triangle == NO_HIT {
            return 1.0f - uncovered;
        }
        let point = path_surface_point(hit);
        let opacity = parts[point.part].opacity;
        if opacity >= 1.0f {
            return 1.0f;
        }
        if line_crossings >= MAX_CROSSINGS_PER_LINE {
            return 1.0f;
        }
        line_crossings += 1u;
        if !line_has_passed(point.part) {
            line_passed[line_passed_count] = point.part;
            line_passed_count += 1u;
            uncovered *= 1.0f - opacity;
        }
        ray = path_ray_from(path_next_origin(point, direction), direction);
    }
    return 1.0f;
}

// ---- The light that reaches a floor point.

struct FloorLight {
    reached: vec3<f32>,
    open: vec3<f32>,
    sun_lost: f32,
    sky_lost: f32,
}

// ---- A composited sample.

// What a composited sample learned before its path, and whether it shows one: `path_trace_main`
// traces that path with `floor_active` as the scene's floor, then `path_composite_finish` adds the
// sample. The path is traced from that one place for both the plain and the composited sample (see
// `path.wgsl`'s header on call sites).
struct CompositePlan {
    coverage: f32,
    found_light: bool,
    light: FloorLight,
    // The share of a part past the floor the bed shows through, 0 when there is none.
    past_seen: f32,
    traces_path: bool,
    floor_active: bool,
}

fn path_composite_plan_none() -> CompositePlan {
    return CompositePlan(0.0f, false, FloorLight(vec3<f32>(0.0f), vec3<f32>(0.0f), 0.0f, 0.0f),
        0.0f, false, false);
}

// The lines a composited sample asks the coverage of, in the order it asks them: the camera's line
// up to the floor, then from the floor point toward the sun and the sky, then the camera's line past
// the floor. Each line after the first is asked only when the one before says so.
const PATH_LINE_CAMERA: u32 = 0u;
const PATH_LINE_SUN: u32 = 1u;
const PATH_LINE_SKY: u32 = 2u;
const PATH_LINE_BEYOND: u32 = 3u;
const PATH_LINE_DONE: u32 = 4u;

// One sample of a pixel up to its path, in the order `tile_work.rs` documents: the line's coverage
// first, and the floor's light only when the line reaches the floor past what it covers. Radiance
// is wanted only where it is shown, so a sample that covers nothing and sees no part past the floor
// costs two shadow rays and no path.
//
// The floor's light is one sample of the sun and one of the sky, each along the same direction
// with and without the parts in the way, so the ratio is exactly 1 on open floor with no noise and
// no reference to trace. Every line is asked through the one `path_coverage_along` call in the
// loop, its answer read by the stage that asked it.
fn path_composite_plan(x: u32, y: u32, sample_index: u32, ray: Ray) -> CompositePlan {
    let has_floor = (path_view.flags & PATH_FLAG_FLOOR) != 0u;
    let show_through = path_view.floor_show_through;
    var plan = path_composite_plan_none();
    var floor_t = PATH_NO_FLOOR;
    if has_floor {
        floor_t = path_catcher_distance(ray);
    }
    let reaches = floor_t != PATH_NO_FLOOR;

    var line = ray;
    if reaches {
        line.t_max = floor_t;
    }
    var stage = PATH_LINE_CAMERA;
    // A sun sample stands for its whole solid angle (one over its density); what a white floor
    // reflects of it is that times the cosine over pi.
    var sun_reflected = vec3<f32>(0.0f);
    var sky = vec3<f32>(0.0f);
    var sky_line = ray;
    loop {
        let covered = path_coverage_along(line);
        var next = PATH_LINE_DONE;
        if stage == PATH_LINE_CAMERA {
            plan.coverage = covered;
            if covered < 1.0f && reaches {
                let at = ray.origin.xy + ray.direction.xy * floor_t;
                let height = path_view.floor_plane.z;
                let point = vec3<f32>(at.x, at.y, height);
                let origin = vec3<f32>(at.x, at.y, height + path_floor_offset(point));
                plan.found_light = true;
                next = PATH_LINE_SKY;
                if path_has_sun() {
                    let u = path_jitter(PATH_SUN_SALT_LOW, PATH_SUN_SALT_HIGH, x, y, sample_index);
                    let direction = path_sun_sample(u);
                    if direction.z > 0.0f {
                        sun_reflected = path_view.sun_radiance.xyz
                            * (direction.z / path_view.sun_tangent.w / PATH_PI);
                        line = path_ray_from(origin, direction);
                        next = PATH_LINE_SUN;
                    }
                }
                let u = path_jitter(PATH_SKY_SALT_LOW, PATH_SKY_SALT_HIGH, x, y, sample_index);
                let radius = sqrt(u.x);
                let turn = cos_sin_of_turn(u.y);
                let direction = vec3<f32>(radius * turn.x, radius * turn.y, sqrt(max(1.0f - u.x, 0.0f)));
                sky = path_environment(direction);
                sky_line = path_ray_from(origin, direction);
                if next == PATH_LINE_SKY {
                    line = sky_line;
                }
            }
        } else if stage == PATH_LINE_SUN {
            let seen = 1.0f - covered;
            plan.light.sun_lost = dot(PATH_LUMA, sun_reflected) * (1.0f - seen);
            plan.light.reached += sun_reflected * seen;
            plan.light.open += sun_reflected;
            line = sky_line;
            next = PATH_LINE_SKY;
        } else if stage == PATH_LINE_SKY {
            let seen = 1.0f - covered;
            plan.light.sky_lost = dot(PATH_LUMA, sky) * (1.0f - seen);
            plan.light.reached += sky * seen;
            plan.light.open += sky;
            if plan.coverage == 0.0f && show_through > 0.0f {
                line = ray;
                line.t_min = max(floor_t, 0.0f);
                line.t_max = PATH_FAR;
                next = PATH_LINE_BEYOND;
            }
        } else {
            let seen = covered * show_through;
            if seen > 0.0f {
                // The parts' own scene, with no floor in the way: the part below it.
                plan.past_seen = seen;
                plan.traces_path = true;
                plan.floor_active = false;
            }
        }
        if next == PATH_LINE_DONE {
            break;
        }
        stage = next;
    }

    if plan.coverage > 0.0f {
        plan.traces_path = true;
        plan.floor_active = has_floor;
    }
    return plan;
}

// The rest of the sample once `traced` holds its path's radiance (zero when `plan` traced none).
// One that covers nothing and sees no part past the floor adds only its floor loss. One with a part
// past the floor adds that part at the bed's see-through share, over the floor it stands on.
fn path_composite_finish(index: u32, plan: CompositePlan, traced: vec3<f32>, ray: Ray) {
    var radiance = vec3<f32>(0.0f);
    var added = 0.0f;
    if plan.coverage > 0.0f {
        radiance = traced;
        added = plan.coverage;
    } else if plan.past_seen > 0.0f && plan.found_light {
        let floor_radiance = plan.light.reached * path_view.floor_base.xyz;
        radiance = traced * plan.past_seen + floor_radiance * (1.0f - plan.past_seen);
        added = plan.past_seen;
    }

    var sun_shadow = 0.0f;
    var sky_shadow = 0.0f;
    if plan.found_light {
        let open = dot(PATH_LUMA, plan.light.open);
        if open > 0.0f {
            sun_shadow = clamp(plan.light.sun_lost / open, 0.0f, 1.0f);
            sky_shadow = clamp(plan.light.sky_lost / open, 0.0f, 1.0f);
        }
    }
    var backdrop = path_escaped(ray.direction, false, 0.0f);
    if plan.found_light {
        backdrop = plan.light.reached * path_view.floor_base.xyz;
    }

    path_composite_add(index, radiance, added, sun_shadow, sky_shadow, backdrop);
}

// `PixelAccum::add`: the sample's shadow is the share of its line the parts don't cover, the
// covered color adds under the same checks a plain sample does, and only a usable sample's
// coverage counts toward the color's mean.
fn path_composite_add(index: u32, radiance: vec3<f32>, coverage: f32, sun_shadow: f32, sky_shadow: f32, backdrop: vec3<f32>) {
    let left = 1.0f - coverage;
    var channel = channels[index];
    channel.coverage.y += left * sun_shadow;
    channel.coverage.z += left * sky_shadow;
    channel.coverage.w += 1.0f;
    if coverage > 0.0f {
        channel.coverage.x += coverage;
        let prior = accumulation[index];
        let sum = prior.xyz + radiance;
        let usable = path_finite3(radiance) && radiance.x >= 0.0f && radiance.y >= 0.0f
            && radiance.z >= 0.0f && path_finite3(sum);
        if usable {
            accumulation[index] = vec4<f32>(sum, prior.w + 1.0f);
            channel.backdrop.w += coverage;
            if coverage < 1.0f {
                channel.backdrop = vec4<f32>(channel.backdrop.xyz + backdrop * left, channel.backdrop.w);
            }
        }
    }
    channels[index] = channel;
}
