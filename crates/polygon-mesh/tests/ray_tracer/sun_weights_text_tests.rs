//! The text of the GPU route's weights for sunlight through glass, pinned to their Rust mirror's
//! steps (`flat_tree/sun_weights.rs`): the whole of `path_sample_sun`, `path_sun_toward`,
//! `path_sun_toward_light`, `path_sun_wants_glass`, `path_shadow_weight`, `path_sun_after` and
//! `path_escaped` (`sun.wgsl`), the density and its two parts a sample carries (`surface_pdf`,
//! `finish_sample`, the glass lobe's mark in `surface_sample`, `scatter.wgsl`), the declarations
//! of the records they read and make (`SunToward`, `SunShadow`, `PathSunState`, `ScatterSample`),
//! and how the path loop keeps and updates the record (`path.wgsl`), so a change to any of them fails here until
//! the mirror and its case-by-case checks against the CPU (`sun_weights_tests`) follow it.

use solidmesh_mesh::ray_tracer::flat_tree::PATH_WGSL;

use crate::see_through_on_floor_tests::{function_text, struct_text};

/// `surface_pdf`, flattened by `function_text`.
const SURFACE_PDF: &str =
    "fn surface_pdf(s: Surface, wo: vec3<f32>, wi: vec3<f32>) -> f32 { let parts = \
    surface_pdf_by_lobe(s, wo, wi); return clean(parts.x + parts.y);";

/// `finish_sample`, flattened by `function_text`.
const FINISH_SAMPLE: &str =
    "fn finish_sample(s: Surface, wo: vec3<f32>, wi: vec3<f32>) -> ScatterSample { let parts = \
    surface_pdf_by_lobe(s, wo, wi); let density = clean(parts.x + parts.y); if density <= 0.0f \
    { return no_sample(); } let cosine = abs(wi.z); let weight = clean3(surface_evaluate(s, wo, \
    wi) * cosine / density); return ScatterSample(true, wi, density, weight, false, false, \
    clean(parts.x), clean(parts.y));";

/// `path_shadow_weight`, flattened by `function_text`.
const PATH_SHADOW_WEIGHT: &str =
    "fn path_shadow_weight(sun_pdf: f32, pdf: f32) -> f32 { if path_view.strategy == \
    PATH_STRATEGY_COMBINED { return path_power_heuristic(sun_pdf, pdf); } if path_view.strategy \
    == PATH_STRATEGY_SURFACE_ONLY { return 0.0f; } return 1.0f;";

/// `path_escaped`, flattened by `function_text`.
const PATH_ESCAPED: &str =
    "fn path_escaped(direction: vec3<f32>, sampled: bool, pdf: f32, last_glass: bool, chain: \
    f32) -> vec3<f32> { var radiance = path_environment(direction); if path_has_sun() && \
    path_sun_contains(direction) { var weight = 1.0f; if sampled && path_view.strategy != \
    PATH_STRATEGY_SURFACE_ONLY { if path_view.strategy == PATH_STRATEGY_NEXT_EVENT_ONLY { \
    weight = 0.0f; } else { weight = path_power_heuristic(pdf, path_view.sun_tangent.w); } } if \
    last_glass { weight = weight * chain; } radiance += path_view.sun_radiance.xyz * weight; } \
    return radiance;";

/// `path_sun_toward`, flattened by `function_text`.
const PATH_SUN_TOWARD: &str =
    "fn path_sun_toward(vertex: PathVertex, wi: vec3<f32>, bsdf: vec3<f32>, pdf_light: f32, \
    radiance: vec3<f32>, chain: f32) -> SunToward { let through_surface = vertex.wo.z * wi.z < \
    0.0f; let parts = surface_pdf_by_lobe(vertex.surface, vertex.wo, wi); let other_pdf = \
    clean(parts.x); let glass_pdf = clean(parts.y); var other = 1.0f; var glass = 0.0f; if \
    through_surface { other = 0.0f; glass = 1.0f; } else if glass_pdf > 0.0f && other_pdf > \
    0.0f { let total = other_pdf + glass_pdf; other = other_pdf / total; glass = glass_pdf / \
    total; } else if glass_pdf > 0.0f { other = 0.0f; glass = 1.0f; } return SunToward(bsdf, \
    abs(wi.z), pdf_light, radiance, other_pdf, other, glass, path_shadow_weight(pdf_light, \
    glass_pdf) * chain);";

/// `path_sun_wants_glass`, flattened by `function_text`.
const PATH_SUN_WANTS_GLASS: &str =
    "fn path_sun_wants_glass(toward: SunToward) -> bool { return toward.glass > 0.0f && \
    toward.glass_weight > 0.0f;";

/// `path_sun_toward_light`, flattened by `function_text`.
const PATH_SUN_TOWARD_LIGHT: &str =
    "fn path_sun_toward_light(toward: SunToward, line: SunLine) -> vec3<f32> { var direct = \
    vec3<f32>(0.0f); if toward.other > 0.0f { var weight: f32; if line.crossed == 0u { weight = \
    path_shadow_weight(toward.pdf_light, toward.other_pdf); } else { weight = \
    path_power_heuristic(toward.pdf_light, toward.other_pdf); } if weight > 0.0f { var passed = \
    line.transmittance; if toward.other < 1.0f { passed = line.transmittance * toward.other; } \
    direct = toward.bsdf * passed * toward.radiance * (toward.cos / toward.pdf_light * weight); \
    } } if path_sun_wants_glass(toward) && line.crossed == 0u { direct += toward.bsdf * \
    vec3<f32>(toward.glass) * toward.radiance * (toward.cos / toward.pdf_light * \
    toward.glass_weight); } return direct;";

/// `path_sample_sun`, flattened by `function_text`.
const PATH_SAMPLE_SUN: &str =
    "fn path_sample_sun(vertex: PathVertex, chain: f32, in_glass: bool) -> SunShadow { var \
    shadow = SunShadow(vec3<f32>(0.0f), path_view.sun_tangent.w); let straight = \
    (path_view.flags & PATH_FLAG_SUN_STRAIGHT) != 0u; if !path_has_sun() || (path_view.strategy \
    == PATH_STRATEGY_SURFACE_ONLY && !straight) { return shadow; } let inside = straight && \
    in_glass && vertex.seam.kind != SEAM_INTERFACE && !vertex.resting; let u0 = \
    path_next_unit(); let u1 = path_next_unit(); let wi_world = path_sun_sample(vec2<f32>(u0, \
    u1)); var wi = path_to_local(vertex.frame, wi_world); if \
    !path_sides_agree(vertex.point.geometric, vertex.toward_camera, wi_world, vertex.wo.z, \
    wi.z) { return shadow; } var bsdf = surface_evaluate(vertex.surface, vertex.wo, wi); if \
    path_largest_channel(bsdf) <= 0.0f { return shadow; } var toward: SunToward; var \
    walk_straight = true; if straight && !inside { toward = path_sun_toward(vertex, wi, bsdf, \
    path_view.sun_tangent.w, path_view.sun_radiance.xyz, chain); if toward.other <= 0.0f && \
    !path_sun_wants_glass(toward) { return shadow; } walk_straight = toward.other > 0.0f; } let \
    line = path_sun_line(vertex, wi_world, walk_straight); if !straight { if !line.reached { \
    return shadow; } let pdf_light = path_view.sun_tangent.w; var weight = 1.0f; if \
    path_view.strategy == PATH_STRATEGY_COMBINED { weight = path_power_heuristic(pdf_light, \
    surface_pdf(vertex.surface, vertex.wo, wi)); } shadow.light = bsdf * \
    path_view.sun_radiance.xyz * (abs(wi.z) / pdf_light * weight); return shadow; } if inside { \
    var bent = SunBent(false, vec3<f32>(0.0f), 0.0f, 0.0f); if line.first_found { bent = \
    path_sun_bent(line.first_outward, line.first_ior, wi_world); } var direction = wi_world; \
    var radiance = path_view.sun_radiance.xyz; if bent.found { let ior2 = line.first_ior * \
    line.first_ior; direction = bent.inside; shadow.receiver_pdf = path_view.sun_tangent.w * \
    ior2 * bent.cos_in / bent.cos_out; radiance = path_view.sun_radiance.xyz * ior2; } if \
    !line.reached { return shadow; } wi = path_to_local(vertex.frame, direction); if bent.found \
    && !path_sides_agree(vertex.point.geometric, vertex.toward_camera, direction, vertex.wo.z, \
    wi.z) { return shadow; } bsdf = surface_evaluate(vertex.surface, vertex.wo, wi); if \
    path_largest_channel(bsdf) <= 0.0f { return shadow; } toward = path_sun_toward(vertex, wi, \
    bsdf, shadow.receiver_pdf, radiance, chain); } else if !line.reached { return shadow; } \
    shadow.light = path_sun_toward_light(toward, line); return shadow;";

/// `path_sun_after`, flattened by `function_text`.
const PATH_SUN_AFTER: &str =
    "fn path_sun_after(state: PathSunState, drawn: ScatterSample, at_seam: bool, glass_surface: \
    bool, receiver_pdf: f32) -> PathSunState { var next = state; if at_seam { next.in_glass = \
    true; } else if glass_surface { next.in_glass = drawn.direction.z < 0.0f; } if \
    drawn.from_glass { next.in_chain = true; next.last_glass = true; return next; } let carried \
    = select(1.0f, state.chain, state.in_chain); if drawn.is_specular { next.chain = carried; } \
    else { next.chain = carried * path_power_heuristic(drawn.other_pdf, receiver_pdf); } \
    next.in_chain = next.in_glass; next.last_glass = false; return next;";

/// The records the weights read and make, declared field by field. Their constructors are pinned
/// by position, so two fields of one type swapped in a declaration (`other` and `glass` in
/// `SunToward`, say) would pass every other pin.
const WEIGHT_RECORDS: [(&str, &str); 4] = [
    (
        "SunToward",
        "struct SunToward { bsdf: vec3<f32>, cos: f32, pdf_light: f32, radiance: vec3<f32>, \
         other_pdf: f32, other: f32, glass: f32, glass_weight: f32,",
    ),
    (
        "SunShadow",
        "struct SunShadow { light: vec3<f32>, receiver_pdf: f32,",
    ),
    (
        "PathSunState",
        "struct PathSunState { chain: f32, in_chain: bool, last_glass: bool, in_glass: bool,",
    ),
    (
        "ScatterSample",
        "struct ScatterSample { found: bool, direction: vec3<f32>, pdf: f32, weight: vec3<f32>, \
         is_specular: bool, from_glass: bool, other_pdf: f32, glass_pdf: f32,",
    ),
];

#[test]
fn the_shader_weight_records_are_declared_as_their_mirrors_read_them() {
    let source = PATH_WGSL;
    for (name, text) in WEIGHT_RECORDS {
        assert_eq!(
            struct_text(source, name),
            text,
            "{name} is not declared as its mirror reads it"
        );
    }
}

#[test]
fn the_shader_weights_are_their_mirrors() {
    let pieces = [
        ("surface_pdf", SURFACE_PDF),
        ("finish_sample", FINISH_SAMPLE),
        ("path_shadow_weight", PATH_SHADOW_WEIGHT),
        ("path_escaped", PATH_ESCAPED),
        ("path_sun_toward", PATH_SUN_TOWARD),
        ("path_sun_wants_glass", PATH_SUN_WANTS_GLASS),
        ("path_sun_toward_light", PATH_SUN_TOWARD_LIGHT),
        ("path_sample_sun", PATH_SAMPLE_SUN),
        ("path_sun_after", PATH_SUN_AFTER),
    ];
    for (name, text) in pieces {
        assert_eq!(
            function_text(PATH_WGSL, name),
            text,
            "{name} is not its mirror's"
        );
    }
}

#[test]
fn a_sample_says_when_the_glass_lobe_drew_it() {
    let sample = function_text(PATH_WGSL, "surface_sample");
    assert!(
        sample.contains(
            "var drawn = sample_glass(s, shares, wo, choice.glass, random_unit(remapped), u1, \
             u2); drawn.from_glass = true; return drawn;"
        ),
        "{sample}"
    );
    // A delta event and no event carry no density parts and no glass mark of their own.
    assert!(function_text(PATH_WGSL, "no_sample").contains(
        "return ScatterSample(false, vec3<f32>(0.0f), 0.0f, vec3<f32>(0.0f), false, false, \
         0.0f, 0.0f);"
    ));
    assert!(function_text(PATH_WGSL, "delta_event").contains(
        "return ScatterSample(true, direction, clean(density), clean3(throughput / \
         probability), true, false, 0.0f, 0.0f);"
    ));
}

#[test]
fn the_path_loop_keeps_the_record_as_the_mirror_does() {
    let path = function_text(PATH_WGSL, "path_radiance");
    for step in [
        // The camera's record, in air, and the flag that writes it.
        "let straight = (path_view.flags & PATH_FLAG_SUN_STRAIGHT) != 0u; var sun_path = \
         PathSunState(1.0f, false, false, false);",
        // The escape reads the record.
        "radiance += throughput * path_escaped(ray.direction, arrival_sampled, arrival_pdf, \
         sun_path.last_glass, sun_path.chain); break;",
        // The shadow ray reads it, and says the density its own weight was taken against.
        "let shadow = path_sample_sun(vertex, sun_path.chain, sun_path.in_glass); radiance += \
         throughput * shadow.light;",
        // After the event, its shadow ray, and before roulette and the next ray: the arrival's
        // density by the drawn lobe's part, then `in_glass` and the rest of the record.
        "scatters += 1u; arrival_sampled = !drawn.is_specular; arrival_pdf = drawn.pdf; if \
         straight { if !drawn.is_specular { arrival_pdf = select(drawn.other_pdf, \
         drawn.glass_pdf, drawn.from_glass); } sun_path = path_sun_after(sun_path, drawn, \
         at_seam, has_glass(surface), shadow.receiver_pdf); } if path_view.roulette_after",
    ] {
        assert!(path.contains(step), "the path loop lacks `{step}`");
    }
    // Nothing else writes the record (its declaration and the one update): a pass-through, a seam
    // passed straight and a step across a contact `continue` before the event.
    assert_eq!(path.matches("sun_path =").count(), 2, "{path}");
}
