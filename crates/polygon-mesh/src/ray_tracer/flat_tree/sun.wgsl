// The sun and the sky in WGSL: the environment a path that leaves the scene sees, the sun's disc
// and its sampling, and next-event estimation toward the sun with its shadow ray, the
// `lights.rs` and `sample_sun` parts of `integrator.rs`. It follows `path.wgsl` and
// `see_through.wgsl` in `PATH_WGSL` (`path_shader.rs`) and uses their records, the path's random
// stream and its straight line; the path loop in `path.wgsl` calls `path_escaped` when a path
// leaves and `path_sample_sun` at each vertex, and `catcher.wgsl` asks `path_environment` and
// `path_sun_sample` for a floor's light.
//
// The shadow ray runs one `trace` call whether it stops at the first hit, walks see-through parts
// or walks straight through glass (`path.wgsl`'s header says why each traversing function has
// one call site): `path_sun_line`, the CPU's `sun_reach_from`, `sun_reachable` and
// `through_glass` in `sun_through_glass.rs` in one loop.
//
// **Straight through glass** (`PATH_FLAG_SUN_STRAIGHT`). With the flag the line starts where the
// CPU's straight line starts (at a seam past both faces; at a vertex on a contact with the floor,
// a sun on the far side of the face across meets that face first, the part there decided by the
// line's own draw: found there, glass is crossed and the line goes on from inside it and anything
// else stops it; found not there, the line goes on from the vertex with the part passed) and
// decides, at each hit nearest first, the part's presence, the seam (`seam.wgsl`), whether the
// surface passes the sun (`surface_passes_sun_straight`), the first face crossed from inside its
// solid, a glass face resting on the floor being left (which stops it), the crossing cap
// `PATH_MAX_GLASS_CROSSINGS` and the face's `surface_straight_transmittance`, in the CPU's order.
// The CPU's rest rule covers glass resting on any opaque part; here only the analytic floor has
// one, so a line leaving glass through a face lying on an opaque part of the tree crosses that
// face and meets the part's own face by the trace, which stops it unless the continuing-ray start
// already lies past a single sheet. The walk ends within `PATH_MAX_GLASS_CROSSINGS + 1` stretches
// between crossings, each within `MAX_CROSSINGS_PER_LINE + 1` traces.
//
// **The two routes' weights** (`glass_chain.rs`: the rule, the state and when each part is
// written and read). With the flag `path_sample_sun` is the CPU's `sample_sun` under Straight: one
// sun sample, the BSDF toward it shared between the other lobes and the glass lobe by their two
// densities (`path_sun_toward`), the line walked straight when the other share wants it and
// stopped at any surface when only the glass share does, the other share at Exact's strategy
// weight with a clear line and `PH(p_sun, p_o)` through glass, the glass share only with a clear
// line and scaled by `chain`; a receiver inside glass reads the sun along the direction bent at the
// line's first face (`path_sun_bent`) over the denser disc seen from inside, and says that disc's
// density back as the one its own `w_b` is taken against. The path loop keeps `chain`, `in_chain`,
// `last_glass` and `in_glass` and updates them after each event as `SunPath::after` does, and
// `path_escaped` scales the sun's disc by `chain` after a glass event. The view sets the flag for
// `SunThroughGlass::Straight` in a scene with glass (`gpu_view.rs`); without it every operation is
// the plain shadow ray's and the plain escape's.

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

// The sun radiance a ray that left the scene sees, `escaped_radiance`: Exact's weight for the
// arrival, times `chain` when a glass event sent the ray (`SunPath::sun_weight`).
fn path_escaped(direction: vec3<f32>, sampled: bool, pdf: f32, last_glass: bool, chain: f32) -> vec3<f32> {
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
        if last_glass {
            weight = weight * chain;
        }
        radiance += path_view.sun_radiance.xyz * weight;
    }
    return radiance;
}

// `shadow_weight`: the shadow ray's share of a sun direction of density `sun_pdf` against a lobe
// that would find it with density `pdf`, under the view's strategy.
fn path_shadow_weight(sun_pdf: f32, pdf: f32) -> f32 {
    if path_view.strategy == PATH_STRATEGY_COMBINED {
        return path_power_heuristic(sun_pdf, pdf);
    }
    if path_view.strategy == PATH_STRATEGY_SURFACE_ONLY {
        return 0.0f;
    }
    return 1.0f;
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

// `MAX_GLASS_CROSSINGS` in `sun_through_glass.rs`: the most faces of glass a line toward the sun
// goes through; at the next one it counts as blocked.
const PATH_MAX_GLASS_CROSSINGS: u32 = 16u;

// What a line toward the sun found (`SunReach` and `FirstFace` in `sun_through_glass.rs`): whether
// it reached the sun, what the glass along it passed (1 with none), how many faces of glass it
// crossed, the start's included, and the first face of glass it crossed from inside its solid
// (hit on its back), with that face's shading normal pointing out of the solid along the line and
// the glass's index. The first face is kept whether or not the line then reached the sun.
struct SunLine {
    reached: bool,
    transmittance: vec3<f32>,
    crossed: u32,
    first_found: bool,
    first_outward: vec3<f32>,
    first_ior: f32,
}

// `SunLine` ended short of the sun.
fn path_sun_blocked(line: SunLine) -> SunLine {
    var blocked = line;
    blocked.reached = false;
    return blocked;
}

// `crossing_cosine`: the line's cosine to the shading normal at `point`, as the scattering frame
// reads it.
fn path_crossing_cosine(point: PathPoint, direction: vec3<f32>) -> f32 {
    return abs(dot(path_facing(point, direction).shading, direction));
}

// The line toward the sun along `direction` from `vertex` (`sun_reach_from` in
// `sun_through_glass.rs`). Without `PATH_FLAG_SUN_STRAIGHT` it is the plain shadow ray: from the
// vertex's own continuing-ray start, stopped by the floor or the first hit, or with see-through
// parts walked hit by hit past the parts the line finds not there. With the flag the line starts
// where a straight line does (at a seam past both faces, at a contact with the floor across the
// contact first) and, with `straight`, goes nearest hit first through every face of glass
// (`LineMode::Straight`); without `straight` it stops at the first surface met past its start
// (`LineMode::StopAtAny`). One `trace` serves every mode (`path.wgsl`'s header on call sites).
fn path_sun_line(vertex: PathVertex, direction: vec3<f32>, straight: bool) -> SunLine {
    var line = SunLine(false, vec3<f32>(1.0f), 0u, false, vec3<f32>(0.0f), 1.0f);
    let see_through = (path_view.flags & PATH_FLAG_SEE_THROUGH) != 0u;
    let sun_straight = (path_view.flags & PATH_FLAG_SUN_STRAIGHT) != 0u;
    let walk = sun_straight && straight;
    if see_through || sun_straight {
        line_restart();
    }
    var origin = path_next_origin(vertex.point, direction);
    if sun_straight && vertex.seam.kind == SEAM_INTERFACE {
        // A seam: past both faces, starting past a part the path found not there.
        let absent = seam_absent_toward(vertex.seam, vertex.point, direction);
        if absent != SEAM_NO_PART {
            line_starts_past(absent);
        }
        origin = seam_origin(vertex.point, direction, vertex.seam.skip);
    } else if sun_straight && vertex.resting && dot(direction, vertex.point.geometric) > 0.0f {
        // A contact with the sun on the far side of the face across: that face is met first, at
        // the same point and with no trace, and the part there decided by this line's own draw.
        var across: PathPoint;
        if vertex.floor_first {
            across = path_surface_point(vertex.rest_hit);
        } else {
            across = path_floor_point_under(vertex.point.position);
        }
        var present = true;
        if see_through && across.part != PATH_FLOOR_PART {
            let verdict = line_at_hit(across.part);
            if verdict == LINE_ENDED {
                return path_sun_blocked(line);
            }
            present = verdict != LINE_PASSES;
        }
        if present {
            // Glass passes the line into itself; anything else across stops it, and a line
            // stopping at any surface stops at glass too.
            if !walk {
                return path_sun_blocked(line);
            }
            let surface = path_surface_at(across.part, across.color);
            if !surface_passes_sun_straight(surface) {
                return path_sun_blocked(line);
            }
            line.transmittance = surface_straight_transmittance(surface,
                path_crossing_cosine(across, direction));
            line.crossed = 1u;
            line_restart();
            origin = path_next_origin(across, direction);
        }
    }
    var ray = path_ray_from(origin, direction);
    if path_floor_distance(ray) >= 0.0f {
        return path_sun_blocked(line);
    }
    let stop_at_any = !walk && !see_through;
    loop {
        let hit = trace(ray, stop_at_any);
        if hit.triangle == NO_HIT {
            line.reached = true;
            return line;
        }
        if stop_at_any {
            return path_sun_blocked(line);
        }
        let point = path_surface_point(hit);
        var present = true;
        if see_through {
            let verdict = line_at_hit(point.part);
            if verdict == LINE_ENDED {
                return path_sun_blocked(line);
            }
            present = verdict != LINE_PASSES;
        }
        if !walk {
            if present {
                return path_sun_blocked(line);
            }
            ray = path_ray_from(path_next_origin(point, direction), direction);
            continue;
        }
        var seam = seam_none();
        if (path_view.flags & PATH_FLAG_GLASS_CONTACTS) != 0u {
            seam = seam_at(point, hit.source_index, direction, present);
            if seam.kind == SEAM_ENDED {
                return path_sun_blocked(line);
            }
            if seam.kind == SEAM_STRAIGHT {
                ray = path_ray_from(seam_origin(point, direction, seam.skip), direction);
                continue;
            }
            if seam.kind == SEAM_INTERFACE {
                present = true;
            }
        }
        if !present {
            ray = path_ray_from(path_next_origin(point, direction), direction);
            continue;
        }
        let at_seam = seam.kind == SEAM_INTERFACE;
        var surface = seam.surface;
        if !at_seam {
            surface = path_surface_at(point.part, point.color);
        }
        if !surface_passes_sun_straight(surface) {
            return path_sun_blocked(line);
        }
        let facing = path_facing(point, direction);
        let passed = surface_straight_transmittance(surface, abs(dot(facing.shading, direction)));
        if line.crossed == 0u && !at_seam && facing.back_face {
            line.first_found = true;
            line.first_outward = -facing.shading;
            line.first_ior = surface.ior;
        }
        // Glass left through its face resting on the floor: the floor across stops the line.
        if !at_seam && dot(direction, point.geometric) > 0.0f && path_rests_on_floor(point) {
            return path_sun_blocked(line);
        }
        if line.crossed >= PATH_MAX_GLASS_CROSSINGS {
            return path_sun_blocked(line);
        }
        line.crossed += 1u;
        line.transmittance *= passed;
        if path_largest_channel(line.transmittance) <= 0.0f {
            return path_sun_blocked(line);
        }
        // A crossing is a vertex of the line: the line after it decides partly there parts afresh.
        line_restart();
        if at_seam {
            let absent = seam_absent_toward(seam, point, direction);
            if absent != SEAM_NO_PART {
                line_starts_past(absent);
            }
            ray = path_ray_from(seam_origin(point, direction, seam.skip), direction);
        } else {
            ray = path_ray_from(path_next_origin(point, direction), direction);
        }
    }
    return path_sun_blocked(line);
}

// `FirstFace::bent`: the direction inside glass of index `ior` that refracts out through a face of
// outward shading normal `outward` along `line`, with the line's cosine to the normal outside and
// the bent direction's inside; `found` false where the line leaves the face on its inside or the
// glass is less dense than air.
struct SunBent {
    found: bool,
    inside: vec3<f32>,
    cos_out: f32,
    cos_in: f32,
}

fn path_sun_bent(outward: vec3<f32>, ior: f32, line: vec3<f32>) -> SunBent {
    let cos_out = dot(outward, line);
    if cos_out <= 0.0f || ior < 1.0f {
        return SunBent(false, vec3<f32>(0.0f), 0.0f, 0.0f);
    }
    let tangent = line - outward * cos_out;
    let sin2_in = (1.0f - cos_out * cos_out) / (ior * ior);
    let cos_in = sqrt(max(1.0f - sin2_in, 0.0f));
    let inside = unit_direction(tangent / ior + outward * cos_in);
    return SunBent(true, inside, cos_out, cos_in);
}

// A vertex of the path, as its shadow ray reads it: the point, its surface and frame, and where a
// straight line toward the sun starts from it (`ShadowVertex`): the seam's interface there, and
// whether it is on a contact with the floor (`resting`), the floor under a part (`floor_first`,
// the part's face hit in `rest_hit`) or a part's face over the floor.
struct PathVertex {
    point: PathPoint,
    surface: Surface,
    frame: PathFrame,
    toward_camera: vec3<f32>,
    wo: vec3<f32>,
    seam: PathSeam,
    resting: bool,
    floor_first: bool,
    rest_hit: Hit,
}

// What a vertex's BSDF sends toward one sun direction under Straight, in its two shares (`Toward`
// in `integrator.rs`): the direction's cosine, the sun's density and radiance along it, the other
// lobes' density, the shares `lobe_shares` gives, and the glass share's weight with `chain` in.
struct SunToward {
    bsdf: vec3<f32>,
    cos: f32,
    pdf_light: f32,
    radiance: vec3<f32>,
    other_pdf: f32,
    other: f32,
    glass: f32,
    glass_weight: f32,
}

// `Toward::of`: the densities' two parts at `wi`, cleaned, shared all to glass through the surface,
// in proportion where both are there, all to the one that is, and all to the other lobes where
// neither is.
fn path_sun_toward(vertex: PathVertex, wi: vec3<f32>, bsdf: vec3<f32>, pdf_light: f32, radiance: vec3<f32>, chain: f32) -> SunToward {
    let through_surface = vertex.wo.z * wi.z < 0.0f;
    let parts = surface_pdf_by_lobe(vertex.surface, vertex.wo, wi);
    let other_pdf = clean(parts.x);
    let glass_pdf = clean(parts.y);
    var other = 1.0f;
    var glass = 0.0f;
    if through_surface {
        other = 0.0f;
        glass = 1.0f;
    } else if glass_pdf > 0.0f && other_pdf > 0.0f {
        let total = other_pdf + glass_pdf;
        other = other_pdf / total;
        glass = glass_pdf / total;
    } else if glass_pdf > 0.0f {
        other = 0.0f;
        glass = 1.0f;
    }
    return SunToward(bsdf, abs(wi.z), pdf_light, radiance, other_pdf, other, glass,
        path_shadow_weight(pdf_light, glass_pdf) * chain);
}

// `Toward::wants_glass`.
fn path_sun_wants_glass(toward: SunToward) -> bool {
    return toward.glass > 0.0f && toward.glass_weight > 0.0f;
}

// `Toward::light`: the light a line that reached the sun brings, each share's term `bsdf * passed *
// radiance * (cos / pdf_light * weight)`.
fn path_sun_toward_light(toward: SunToward, line: SunLine) -> vec3<f32> {
    var direct = vec3<f32>(0.0f);
    if toward.other > 0.0f {
        var weight: f32;
        if line.crossed == 0u {
            weight = path_shadow_weight(toward.pdf_light, toward.other_pdf);
        } else {
            weight = path_power_heuristic(toward.pdf_light, toward.other_pdf);
        }
        if weight > 0.0f {
            var passed = line.transmittance;
            if toward.other < 1.0f {
                passed = line.transmittance * toward.other;
            }
            direct = toward.bsdf * passed * toward.radiance * (toward.cos / toward.pdf_light * weight);
        }
    }
    if path_sun_wants_glass(toward) && line.crossed == 0u {
        direct += toward.bsdf * vec3<f32>(toward.glass) * toward.radiance
            * (toward.cos / toward.pdf_light * toward.glass_weight);
    }
    return direct;
}

// What a vertex's shadow ray brings, and the sun's density its own `w_b` is taken against: the
// sun's own, but for a receiver inside glass the disc as seen inside (`sample_sun_inside`).
struct SunShadow {
    light: vec3<f32>,
    receiver_pdf: f32,
}

// The sun's contribution at this vertex (`sample_sun`), `chain` and `in_glass` the path's record of
// how it got here. Without `PATH_FLAG_SUN_STRAIGHT` it is Exact's shadow ray, operation for
// operation; with it the header's two routes.
fn path_sample_sun(vertex: PathVertex, chain: f32, in_glass: bool) -> SunShadow {
    var shadow = SunShadow(vec3<f32>(0.0f), path_view.sun_tangent.w);
    let straight = (path_view.flags & PATH_FLAG_SUN_STRAIGHT) != 0u;
    if !path_has_sun() || (path_view.strategy == PATH_STRATEGY_SURFACE_ONLY && !straight) {
        return shadow;
    }
    // A receiver inside glass, not at a seam or a contact, reads the sun bent at the glass's face.
    let inside = straight && in_glass && vertex.seam.kind != SEAM_INTERFACE && !vertex.resting;
    let u0 = path_next_unit();
    let u1 = path_next_unit();
    let wi_world = path_sun_sample(vec2<f32>(u0, u1));
    var wi = path_to_local(vertex.frame, wi_world);
    if !path_sides_agree(vertex.point.geometric, vertex.toward_camera, wi_world, vertex.wo.z, wi.z) {
        return shadow;
    }
    var bsdf = surface_evaluate(vertex.surface, vertex.wo, wi);
    if path_largest_channel(bsdf) <= 0.0f {
        return shadow;
    }
    var toward: SunToward;
    var walk_straight = true;
    if straight && !inside {
        toward = path_sun_toward(vertex, wi, bsdf, path_view.sun_tangent.w,
            path_view.sun_radiance.xyz, chain);
        if toward.other <= 0.0f && !path_sun_wants_glass(toward) {
            return shadow;
        }
        walk_straight = toward.other > 0.0f;
    }
    let line = path_sun_line(vertex, wi_world, walk_straight);
    if !straight {
        if !line.reached {
            return shadow;
        }
        let pdf_light = path_view.sun_tangent.w;
        var weight = 1.0f;
        if path_view.strategy == PATH_STRATEGY_COMBINED {
            weight = path_power_heuristic(pdf_light, surface_pdf(vertex.surface, vertex.wo, wi));
        }
        shadow.light = bsdf * path_view.sun_radiance.xyz * (abs(wi.z) / pdf_light * weight);
        return shadow;
    }
    if inside {
        var bent = SunBent(false, vec3<f32>(0.0f), 0.0f, 0.0f);
        if line.first_found {
            bent = path_sun_bent(line.first_outward, line.first_ior, wi_world);
        }
        var direction = wi_world;
        var radiance = path_view.sun_radiance.xyz;
        if bent.found {
            let ior2 = line.first_ior * line.first_ior;
            direction = bent.inside;
            shadow.receiver_pdf = path_view.sun_tangent.w * ior2 * bent.cos_in / bent.cos_out;
            radiance = path_view.sun_radiance.xyz * ior2;
        }
        if !line.reached {
            return shadow;
        }
        wi = path_to_local(vertex.frame, direction);
        if bent.found && !path_sides_agree(vertex.point.geometric, vertex.toward_camera, direction,
            vertex.wo.z, wi.z) {
            return shadow;
        }
        bsdf = surface_evaluate(vertex.surface, vertex.wo, wi);
        if path_largest_channel(bsdf) <= 0.0f {
            return shadow;
        }
        toward = path_sun_toward(vertex, wi, bsdf, shadow.receiver_pdf, radiance, chain);
    } else if !line.reached {
        return shadow;
    }
    shadow.light = path_sun_toward_light(toward, line);
    return shadow;
}

// The record a path keeps of how it got to its ray beside Exact's arrival (`SunPath` in
// `glass_chain.rs`): the share of exact transport through glass its chain keeps, whether it is in
// a chain, whether a glass lobe drew the last event, and whether the ray travels inside glass.
struct PathSunState {
    chain: f32,
    in_chain: bool,
    last_glass: bool,
    in_glass: bool,
}

// The record after the event `drawn` (`in_glass`, then `SunPath::after` under Straight): a seam
// has glass on both sides, a surface with a glass share is left on the side the ray goes (+z its
// true outside), and an opaque one leaves the ray in what it arrived through; `receiver_pdf` is the
// sun's density the vertex's own `w_b` is taken against.
fn path_sun_after(state: PathSunState, drawn: ScatterSample, at_seam: bool, glass_surface: bool, receiver_pdf: f32) -> PathSunState {
    var next = state;
    if at_seam {
        next.in_glass = true;
    } else if glass_surface {
        next.in_glass = drawn.direction.z < 0.0f;
    }
    if drawn.from_glass {
        next.in_chain = true;
        next.last_glass = true;
        return next;
    }
    let carried = select(1.0f, state.chain, state.in_chain);
    if drawn.is_specular {
        next.chain = carried;
    } else {
        next.chain = carried * path_power_heuristic(drawn.other_pdf, receiver_pdf);
    }
    next.in_chain = next.in_glass;
    next.last_glass = false;
    return next;
}
