//! The text of the GPU route's line toward the sun, pinned to its Rust mirror's steps: the whole
//! of `path_sun_line` (`sun.wgsl`) and the pieces it reads (`surface_passes_sun_straight` and
//! `surface_straight_transmittance` in `scatter.wgsl`, the crossing cosine, the blocked record,
//! the bent direction), where `path_sample_sun` walks the line, the declarations of the records
//! it reads and makes (`PathVertex`, `SunLine`, `SunBent`) and the one place the path loop builds
//! a `PathVertex`, so a change to any of them fails here until `sun_line_f32`
//! (`flat_tree/sun_line.rs`) and its line-for-line checks against the CPU
//! (`sun_line_shader_tests`) follow it.

use solidmesh_mesh::ray_tracer::flat_tree::PATH_WGSL;

use crate::see_through_on_floor_tests::{function_text, struct_text};

/// `path_sun_line`'s body, flattened by `function_text`: the mirror's steps.
const SUN_LINE_WALK: &str =
    "fn path_sun_line(vertex: PathVertex, direction: vec3<f32>, straight: bool) -> SunLine { var \
    line = SunLine(false, vec3<f32>(1.0f), 0u, false, vec3<f32>(0.0f), 1.0f); let see_through = \
    (path_view.flags & PATH_FLAG_SEE_THROUGH) != 0u; let sun_straight = (path_view.flags & \
    PATH_FLAG_SUN_STRAIGHT) != 0u; let walk = sun_straight && straight; if see_through || \
    sun_straight { line_restart(); } var origin = path_next_origin(vertex.point, direction); if \
    sun_straight && vertex.seam.kind == SEAM_INTERFACE { let absent = \
    seam_absent_toward(vertex.seam, vertex.point, direction); if absent != SEAM_NO_PART { \
    line_starts_past(absent); } origin = seam_origin(vertex.point, direction, vertex.seam.skip); \
    } else if sun_straight && vertex.resting && dot(direction, vertex.point.geometric) > 0.0f { \
    var across: PathPoint; if vertex.floor_first { across = path_surface_point(vertex.rest_hit); \
    } else { across = path_floor_point_under(vertex.point.position); } var present = true; if \
    see_through && across.part != PATH_FLOOR_PART { let verdict = line_at_hit(across.part); if \
    verdict == LINE_ENDED { return path_sun_blocked(line); } present = verdict != LINE_PASSES; } \
    if present { if !walk { return path_sun_blocked(line); } let surface = \
    path_surface_at(across.part, across.color); if !surface_passes_sun_straight(surface) { \
    return path_sun_blocked(line); } line.transmittance = \
    surface_straight_transmittance(surface, path_crossing_cosine(across, direction)); \
    line.crossed = 1u; line_restart(); origin = path_next_origin(across, direction); } } var ray \
    = path_ray_from(origin, direction); if path_floor_distance(ray) >= 0.0f { return \
    path_sun_blocked(line); } let stop_at_any = !walk && !see_through; loop { let hit = \
    trace(ray, stop_at_any); if hit.triangle == NO_HIT { line.reached = true; return line; } if \
    stop_at_any { return path_sun_blocked(line); } let point = path_surface_point(hit); var \
    present = true; if see_through { let verdict = line_at_hit(point.part); if verdict == \
    LINE_ENDED { return path_sun_blocked(line); } present = verdict != LINE_PASSES; } if !walk { \
    if present { return path_sun_blocked(line); } ray = path_ray_from(path_next_origin(point, \
    direction), direction); continue; } var seam = seam_none(); if (path_view.flags & \
    PATH_FLAG_GLASS_CONTACTS) != 0u { seam = seam_at(point, hit.source_index, direction, \
    present); if seam.kind == SEAM_ENDED { return path_sun_blocked(line); } if seam.kind == \
    SEAM_STRAIGHT { ray = path_ray_from(seam_origin(point, direction, seam.skip), direction); \
    continue; } if seam.kind == SEAM_INTERFACE { present = true; } } if !present { ray = \
    path_ray_from(path_next_origin(point, direction), direction); continue; } let at_seam = \
    seam.kind == SEAM_INTERFACE; var surface = seam.surface; if !at_seam { surface = \
    path_surface_at(point.part, point.color); } if !surface_passes_sun_straight(surface) { \
    return path_sun_blocked(line); } let facing = path_facing(point, direction); let passed = \
    surface_straight_transmittance(surface, abs(dot(facing.shading, direction))); if \
    line.crossed == 0u && !at_seam && facing.back_face { line.first_found = true; \
    line.first_outward = -facing.shading; line.first_ior = surface.ior; } if !at_seam && \
    dot(direction, point.geometric) > 0.0f && path_rests_on_floor(point) { return \
    path_sun_blocked(line); } if line.crossed >= PATH_MAX_GLASS_CROSSINGS { return \
    path_sun_blocked(line); } line.crossed += 1u; line.transmittance *= passed; if \
    path_largest_channel(line.transmittance) <= 0.0f { return path_sun_blocked(line); } \
    line_restart(); if at_seam { let absent = seam_absent_toward(seam, point, direction); if \
    absent != SEAM_NO_PART { line_starts_past(absent); } ray = path_ray_from(seam_origin(point, \
    direction, seam.skip), direction); } else { ray = path_ray_from(path_next_origin(point, \
    direction), direction); } } return path_sun_blocked(line);";

/// The records the line reads and makes, declared field by field. Their constructors are pinned
/// by position, so two fields of one type swapped in a declaration or a constructor would pass
/// every other pin.
const LINE_RECORDS: [(&str, &str); 3] = [
    (
        "PathVertex",
        "struct PathVertex { point: PathPoint, surface: Surface, frame: PathFrame, toward_camera: \
         vec3<f32>, wo: vec3<f32>, seam: PathSeam, resting: bool, floor_first: bool, rest_hit: \
         Hit,",
    ),
    (
        "SunLine",
        "struct SunLine { reached: bool, transmittance: vec3<f32>, crossed: u32, first_found: \
         bool, first_outward: vec3<f32>, first_ior: f32,",
    ),
    (
        "SunBent",
        "struct SunBent { found: bool, inside: vec3<f32>, cos_out: f32, cos_in: f32,",
    ),
];

#[test]
fn the_shader_line_records_are_declared_as_their_mirrors_read_them() {
    let source = PATH_WGSL;
    for (name, text) in LINE_RECORDS {
        assert_eq!(
            struct_text(source, name),
            text,
            "{name} is not declared as its mirror reads it"
        );
    }
}

#[test]
fn the_path_loop_builds_its_vertex_field_for_field() {
    let source = PATH_WGSL;
    let path = function_text(source, "path_radiance");
    assert!(
        path.contains(
            "let vertex = PathVertex(point, surface, frame, toward_camera, wo, seam, resting, \
             floor_first, rest_hit);"
        ),
        "the path loop does not build its vertex from its own fields in order: {path}"
    );
    // That is the only vertex the shader builds.
    assert_eq!(source.matches("PathVertex(").count(), 1);
}

#[test]
fn the_shader_sun_line_walks_as_its_mirror_does() {
    assert_eq!(
        function_text(PATH_WGSL, "path_sun_line"),
        SUN_LINE_WALK,
        "the sun's line is not its mirror's walk"
    );
}

#[test]
fn the_shader_pieces_of_the_line_are_the_mirrors() {
    let pieces = [
        (
            "surface_passes_sun_straight",
            "fn surface_passes_sun_straight(s: Surface) -> bool { return has_glass(s);",
        ),
        (
            "surface_straight_transmittance",
            "fn surface_straight_transmittance(s: Surface, cos: f32) -> vec3<f32> { let from_air = \
             GlassSide(false, vec3<f32>(0.0f, 0.0f, 1.0f), 1.0f, s.ior, s.ior_minus_one); let \
             fresnel = dielectric_fresnel(cos, from_air); return tint_of(s) * (shares_of(s).glass \
             * (1.0f - fresnel));",
        ),
        (
            "path_crossing_cosine",
            "fn path_crossing_cosine(point: PathPoint, direction: vec3<f32>) -> f32 { return \
             abs(dot(path_facing(point, direction).shading, direction));",
        ),
        (
            "path_sun_blocked",
            "fn path_sun_blocked(line: SunLine) -> SunLine { var blocked = line; blocked.reached = \
             false; return blocked;",
        ),
        (
            "path_sun_bent",
            "fn path_sun_bent(outward: vec3<f32>, ior: f32, line: vec3<f32>) -> SunBent { let \
             cos_out = dot(outward, line); if cos_out <= 0.0f || ior < 1.0f { return \
             SunBent(false, vec3<f32>(0.0f), 0.0f, 0.0f); } let tangent = line - outward * \
             cos_out; let sin2_in = (1.0f - cos_out * cos_out) / (ior * ior); let cos_in = \
             sqrt(max(1.0f - sin2_in, 0.0f)); let inside = unit_direction(tangent / ior + outward \
             * cos_in); return SunBent(true, inside, cos_out, cos_in);",
        ),
    ];
    for (name, text) in pieces {
        assert_eq!(
            function_text(PATH_WGSL, name),
            text,
            "{name} is not its mirror's"
        );
    }
    // The shadow term walks the line once, straight through glass unless only the glass share
    // wants it (`sun_weights_text_tests` pins the rest of `path_sample_sun`).
    let sample = function_text(PATH_WGSL, "path_sample_sun");
    assert_eq!(sample.matches("path_sun_line(").count(), 1, "{sample}");
    assert!(
        sample.contains("let line = path_sun_line(vertex, wi_world, walk_straight);"),
        "{sample}"
    );
}
