//! The GPU path loop's WGSL and scene packing: the module validates with its entry point and its
//! three pieces concatenate with no name or binding clash, the WGSL records are byte for byte the
//! Rust ones and the constants equal theirs, packing a small real scene writes each buffer where
//! the shader reads it, and refusals say what fits. Whether its picture matches the CPU is the
//! demo's GPU page, which runs it (`demo/wasm/src/gpu_bench_render.rs`). What a change of view may
//! touch and the stepped pack are `gpu_view_tests`.

use std::f64::consts::PI;
use std::sync::Arc;

use naga::valid::{Capabilities, ValidationFlags, Validator};
use naga::{ArraySize, Literal, Module, ShaderStage, TypeInner};
use solidmesh_math::{Matrix4X4, Vector3};
use solidmesh_mesh::ray_tracer::flat_tree::{
    gpu_scene_size_bound, pixel_slice_bytes, FlatTreeError, GpuRenderScene, GpuSceneLimits,
    ACCUMULATION_STRIDE, CHANNEL_STRIDE, FACE_STRIDE, GPU_SKY_SEED_SALT, GPU_SUN_SEED_SALT,
    MIN_GPU_SUN_ANGULAR_RADIUS, PART_STRIDE, PATH_ENTRY_POINT, PATH_WGSL, PIXEL_SLICE_BYTES,
    ROULETTE_OFF, SCATTER_WGSL, SCENE_FLAG_COMPOSITE, SCENE_FLAG_FLOOR, SCENE_FLAG_GRADIENT,
    SCENE_FLAG_ORTHOGRAPHIC, SCENE_FLAG_SEE_THROUGH, SCENE_FLAG_SUN, SRGB_TABLE_BYTES, TRACE_WGSL,
    VIEW_RECORD_BYTES, WEBGPU_DEFAULT_MAX_STORAGE_BINDING,
};
use solidmesh_mesh::ray_tracer::{
    Environment, LightSetup, LightStrategy, MeshSurface, PathTraceSettings, PlacedPart,
    PlacedParts, RenderCamera, RenderScene, ScatteringSurface, Sun, MAX_CROSSINGS_PER_LINE,
};
use solidmesh_mesh::solids::platonic_solids::create_cube;

use super::flat_tree_shader_tests::{assert_layout, literal};
use super::path_tracer_support::{scene_of, top_down_camera, SHARP_CREASE};
use super::surface_hit_support::tree_of;

/// The limits every packing here runs under: a micrometre of grid in millimetre units.
const LIMITS: GpuSceneLimits = GpuSceneLimits {
    max_storage_binding: WEBGPU_DEFAULT_MAX_STORAGE_BINDING,
    max_vertex_step: 1e-3,
};

fn parsed() -> Module {
    naga::front::wgsl::parse_str(PATH_WGSL).expect("the path loop parses as WGSL")
}

fn word(bytes: &[u8], index: usize) -> u32 {
    u32::from_le_bytes(bytes[index * 4..index * 4 + 4].try_into().expect("a word"))
}

fn lights() -> LightSetup {
    LightSetup {
        environment: Environment::gradient([0.1, 0.2, 0.3], [0.4, 0.5, 0.6], [0.7, 0.8, 0.9])
            .expect("a gradient"),
        sun: Some(Sun::new(Vector3::new(0.0, 0.0, 1.0), 0.05, [10.0, 20.0, 30.0]).expect("a sun")),
    }
}

fn small_scene() -> RenderScene {
    let mut cube = create_cube(2.0, 2.0, 2.0);
    cube.face_colors = Some(
        (0..cube.faces.len())
            .map(|i| solidmesh_math::Color::new(10 + i as i32, 20, 30, 255))
            .collect(),
    );
    scene_of(
        &cube,
        SHARP_CREASE,
        Matrix4X4::IDENTITY,
        ScatteringSurface::matte_plastic([0.25, 0.5, 0.75]),
        lights(),
    )
}

#[test]
fn the_path_loop_validates_with_its_entry_point() {
    let module = parsed();
    Validator::new(ValidationFlags::all(), Capabilities::empty())
        .validate(&module)
        .expect("the three shaders together validate");
    let compute: Vec<&str> = module
        .entry_points
        .iter()
        .filter(|e| e.stage == ShaderStage::Compute)
        .map(|e| e.name.as_str())
        .collect();
    assert!(compute.contains(&PATH_ENTRY_POINT), "{compute:?}");
    assert!(compute.contains(&"trace_closest_main"), "{compute:?}");
}

#[test]
fn the_three_pieces_are_the_same_text_and_share_no_binding() {
    assert!(PATH_WGSL.starts_with(TRACE_WGSL));
    assert!(PATH_WGSL.contains(SCATTER_WGSL));
    let module = parsed();
    let mut seen = std::collections::HashSet::new();
    for (_, global) in module.global_variables.iter() {
        if let Some(binding) = &global.binding {
            assert!(
                seen.insert((binding.group, binding.binding)),
                "two resources share group {} binding {}",
                binding.group,
                binding.binding
            );
        }
    }
}

#[test]
fn the_records_match_the_buffers_byte_for_byte() {
    let module = parsed();
    assert_layout(
        &module,
        "Face",
        &[
            ("n0", 0),
            ("n1", 12),
            ("n2", 24),
            ("geometric", 36),
            ("ray_offset", 48),
            ("part", 52),
            ("color", 56),
        ],
        FACE_STRIDE as u32,
    );
    assert_layout(
        &module,
        "Part",
        &[("surface", 0), ("opacity", 36)],
        PART_STRIDE as u32,
    );
    assert_layout(
        &module,
        "PathView",
        &[
            ("eye", 0),
            ("forward", 16),
            ("right", 32),
            ("up", 48),
            ("width", 64),
            ("height", 68),
            ("max_bounces", 72),
            ("roulette_after", 76),
            ("seed_low", 80),
            ("seed_high", 84),
            ("strategy", 88),
            ("flags", 92),
            ("env_zenith", 96),
            ("env_horizon", 112),
            ("env_ground", 128),
            ("sun_direction", 144),
            ("sun_tangent", 160),
            ("sun_bitangent", 176),
            ("sun_radiance", 192),
            ("trace_width", 208),
            ("trace_height", 212),
            ("floor_surface_flags", 216),
            ("floor_show_through", 220),
            ("floor_plane", 224),
            ("floor_base", 240),
            ("floor_rest", 256),
        ],
        VIEW_RECORD_BYTES as u32,
    );
    assert_layout(
        &module,
        "PathSrgb",
        &[("to_linear", 0)],
        SRGB_TABLE_BYTES as u32,
    );
    assert_layout(
        &module,
        "PathChannels",
        &[("backdrop", 0), ("coverage", 16)],
        CHANNEL_STRIDE as u32,
    );
    assert_layout(
        &module,
        "PathSlice",
        &[("first_pixel", 0), ("pixel_count", 4), ("sample_index", 8)],
        12,
    );
    let stride_of = |var: &str| {
        let (_, global) = module
            .global_variables
            .iter()
            .find(|(_, g)| g.name.as_deref() == Some(var))
            .unwrap_or_else(|| panic!("the shader declares {var}"));
        match module.types[global.ty].inner {
            TypeInner::Array {
                size: ArraySize::Dynamic,
                stride,
                ..
            } => stride,
            ref other => panic!("{var} is {other:?}"),
        }
    };
    assert_eq!(stride_of("faces"), FACE_STRIDE as u32);
    assert_eq!(stride_of("parts"), PART_STRIDE as u32);
    assert_eq!(stride_of("accumulation"), ACCUMULATION_STRIDE as u32);
    assert_eq!(stride_of("channels"), CHANNEL_STRIDE as u32);
}

#[test]
fn the_shader_constants_are_the_rust_constants() {
    let module = parsed();
    assert_eq!(
        literal(&module, "MAX_CROSSINGS_PER_LINE"),
        Literal::U32(MAX_CROSSINGS_PER_LINE)
    );
    assert_eq!(
        literal(&module, "PATH_ROULETTE_OFF"),
        Literal::U32(ROULETTE_OFF)
    );
    assert_eq!(literal(&module, "PATH_MAX_SURVIVAL"), Literal::F32(0.95));
    assert_eq!(
        literal(&module, "PATH_JITTER_STREAM"),
        Literal::U32(u32::MAX)
    );
    assert_eq!(
        literal(&module, "PATH_FLAG_ORTHOGRAPHIC"),
        Literal::U32(SCENE_FLAG_ORTHOGRAPHIC)
    );
    assert_eq!(
        literal(&module, "PATH_FLAG_SEE_THROUGH"),
        Literal::U32(SCENE_FLAG_SEE_THROUGH)
    );
    assert_eq!(
        literal(&module, "PATH_FLAG_SUN"),
        Literal::U32(SCENE_FLAG_SUN)
    );
    assert_eq!(
        literal(&module, "PATH_FLAG_GRADIENT"),
        Literal::U32(SCENE_FLAG_GRADIENT)
    );
    assert_eq!(
        literal(&module, "PATH_FLAG_COMPOSITE"),
        Literal::U32(SCENE_FLAG_COMPOSITE)
    );
    assert_eq!(
        literal(&module, "PATH_FLAG_FLOOR"),
        Literal::U32(SCENE_FLAG_FLOOR)
    );
    let halves = |salt: u64| [salt as u32, (salt >> 32) as u32];
    assert_eq!(
        [
            literal(&module, "PATH_SUN_SALT_LOW"),
            literal(&module, "PATH_SUN_SALT_HIGH")
        ],
        halves(GPU_SUN_SEED_SALT).map(Literal::U32)
    );
    assert_eq!(
        [
            literal(&module, "PATH_SKY_SALT_LOW"),
            literal(&module, "PATH_SKY_SALT_HIGH")
        ],
        halves(GPU_SKY_SEED_SALT).map(Literal::U32)
    );
    // The jitter steps are the CPU's plastic-number sequence in 32-bit fixed point.
    let plastic = 1.324_717_957_244_746_f64;
    let fixed = |value: f64| (value * 4_294_967_296.0).floor() as u32;
    assert_eq!(
        literal(&module, "PATH_JITTER_STEP_X"),
        Literal::U32(fixed(1.0 / plastic))
    );
    assert_eq!(
        literal(&module, "PATH_JITTER_STEP_Y"),
        Literal::U32(fixed(1.0 / (plastic * plastic)))
    );
}

#[test]
fn the_packed_buffers_hold_each_field_where_the_shader_reads_it() {
    let scene = small_scene();
    let camera = RenderCamera::orthographic(
        Vector3::new(0.0, 0.0, 5.0),
        Vector3::ZERO,
        Vector3::UNIT_Y,
        4.0,
    )
    .expect("a camera");
    let settings = PathTraceSettings {
        seed: 0x1122_3344_5566_7788,
        max_bounces: 7,
        russian_roulette_after: Some(2),
        strategy: LightStrategy::NextEventOnly,
    };
    let pack =
        GpuRenderScene::pack(&scene, &camera, 20, 10, &settings, LIMITS).expect("the scene packs");
    assert_eq!(pack.faces().len(), 12);
    assert_eq!(pack.parts().len(), 1);
    assert_eq!(pack.face_bytes().len(), 12 * FACE_STRIDE);
    assert_eq!(pack.part_bytes().len(), PART_STRIDE);
    assert_eq!(pack.sizes().accumulation_bytes, 20 * 10 * 16);

    let face = pack.faces()[3];
    let bytes = pack.face_bytes();
    let record = &bytes[3 * FACE_STRIDE..4 * FACE_STRIDE];
    assert_eq!(
        word(record, 12),
        face.ray_offset.to_bits(),
        "geometric at 36, ray offset at 48"
    );
    assert_eq!(word(record, 13), face.part);
    assert_eq!(
        word(record, 14),
        u32::from(10 + 3u8) | (20 << 8) | (30 << 16),
        "BBGGRR"
    );

    let part_bytes = pack.part_bytes();
    assert_eq!(word(&part_bytes, 0), 0.25_f32.to_bits());
    assert_eq!(
        word(&part_bytes, 9),
        1.0_f32.to_bits(),
        "opacity after the surface"
    );

    let scene_bytes = pack.view_bytes();
    assert_eq!(scene_bytes.len(), VIEW_RECORD_BYTES);
    assert_eq!(word(&scene_bytes, 16), 20, "width");
    assert_eq!(word(&scene_bytes, 17), 10, "height");
    assert_eq!(word(&scene_bytes, 18), 7, "max bounces");
    assert_eq!(word(&scene_bytes, 19), 2, "roulette after");
    assert_eq!(word(&scene_bytes, 20), 0x5566_7788, "seed low");
    assert_eq!(word(&scene_bytes, 21), 0x1122_3344, "seed high");
    assert_eq!(word(&scene_bytes, 22), 1, "next event only");
    assert_eq!(
        word(&scene_bytes, 23),
        SCENE_FLAG_ORTHOGRAPHIC | SCENE_FLAG_SUN | SCENE_FLAG_GRADIENT
    );
    assert_eq!(word(&scene_bytes, 24), 0.1_f32.to_bits(), "zenith");
    // The srgb table: white decodes to exactly 1, code 3 is the fourth slot of the first vec4.
    let srgb = pack.srgb_table_bytes();
    assert_eq!(srgb.len(), SRGB_TABLE_BYTES);
    assert_eq!(word(&srgb, 255), 1.0_f32.to_bits(), "white");
    let srgb3 = (3.0_f64 / 255.0 / 12.92) as f32;
    let stored = f32::from_bits(word(&srgb, 3));
    assert!((stored - srgb3).abs() < 1e-6, "{stored} vs {srgb3}");
    assert_eq!(pixel_slice_bytes(5, 6, 7), {
        let mut expected = [0u8; PIXEL_SLICE_BYTES];
        expected[0] = 5;
        expected[4] = 6;
        expected[8] = 7;
        expected
    });
}

#[test]
fn the_tree_source_index_reaches_the_part_and_face_it_came_from() {
    let scene = small_scene();
    let pack = GpuRenderScene::pack(
        &scene,
        &top_down_camera(10.0, 4.0),
        4,
        4,
        &PathTraceSettings::default(),
        LIMITS,
    )
    .expect("packs");
    let tree = pack.tree();
    let mut seen = [false; 12];
    for triangle in tree.triangles() {
        seen[triangle.source_index as usize] = true;
        let part = pack.faces()[triangle.source_index as usize].part;
        assert_eq!(part, 0, "every face belongs to the one part");
    }
    assert!(seen.iter().all(|&s| s), "the tree covers every face");
}

#[test]
fn a_non_uniform_scale_gives_the_cpu_interpolated_direction() {
    // A cube smoothed all round has unequal corner normals under this scale, which is where unit
    // corner normals would interpolate to a different direction than the CPU's. A negative scale
    // mirrors the part, which reverses its world winding, so the normals must still come out
    // on the outside.
    for matrix in [
        Matrix4X4::create_scale(1.0, 5.0, 0.25),
        Matrix4X4::create_scale(1.0, -5.0, 0.25),
        Matrix4X4::create_scale(-3.0, 0.5, 2.0)
            * Matrix4X4::create_rotation_z(0.7)
            * Matrix4X4::create_translation(4.0, -2.0, 1.0),
    ] {
        check_normals_under(matrix);
    }
}

fn check_normals_under(matrix: Matrix4X4) {
    let cube = create_cube(2.0, 2.0, 2.0);
    let surface = Arc::new(MeshSurface::new(&cube, PI).expect("smoothed"));
    let scene = RenderScene::new(
        PlacedParts::new(vec![
            PlacedPart::new(tree_of(&cube), surface, matrix).expect("placed")
        ]),
        vec![ScatteringSurface::matte_plastic([0.5; 3])],
        lights(),
    )
    .expect("a scene");
    let pack = GpuRenderScene::pack(
        &scene,
        &top_down_camera(10.0, 4.0),
        4,
        4,
        &PathTraceSettings::default(),
        LIMITS,
    )
    .expect("packs");
    let part = &scene.parts().parts()[0];
    for face in 0..cube.faces.len() {
        let record = pack.faces()[face];
        let corners = [
            cube.faces[face].v0,
            cube.faces[face].v1,
            cube.faces[face].v2,
        ]
        .map(|v| Vector3::from_vector3_float(cube.vertices[v as usize]));
        let weights = [0.2, 0.5, 0.3];
        let mesh_point =
            corners[0] * weights[0] + corners[1] * weights[1] + corners[2] * weights[2];
        let world = mesh_point.transform_position(matrix);
        let point = part.surface_point(face as i32, world).expect("a point");
        let mixed: [f32; 3] = std::array::from_fn(|axis| {
            record.normals[0][axis] * weights[0] as f32
                + record.normals[1][axis] * weights[1] as f32
                + record.normals[2][axis] * weights[2] as f32
        });
        let length = (mixed[0] * mixed[0] + mixed[1] * mixed[1] + mixed[2] * mixed[2]).sqrt();
        let unit = mixed.map(|c| f64::from(c / length));
        let cpu = point.shading_normal;
        assert!(
            (unit[0] - cpu.x).abs() < 1e-6
                && (unit[1] - cpu.y).abs() < 1e-6
                && (unit[2] - cpu.z).abs() < 1e-6,
            "face {face}: stored {unit:?}, CPU {cpu:?}"
        );
        let world_normal = point.geometric_normal;
        assert!(
            (f64::from(record.geometric[0]) - world_normal.x).abs() < 1e-6
                && (f64::from(record.geometric[2]) - world_normal.z).abs() < 1e-6
        );
    }
}

#[test]
fn a_scene_too_big_for_the_binding_limit_says_what_would_fit() {
    let scene = small_scene();
    let camera = top_down_camera(10.0, 4.0);
    let settings = PathTraceSettings::default();
    let tiny = 512;
    let error = GpuRenderScene::pack(
        &scene,
        &camera,
        100,
        100,
        &settings,
        GpuSceneLimits {
            max_storage_binding: tiny,
            ..LIMITS
        },
    )
    .expect_err("the image is too big");
    assert_eq!(
        error,
        FlatTreeError::PackedBufferTooLarge {
            buffer: "accumulation",
            needed_bytes: 100 * 100 * 16,
            limit_bytes: tiny,
            what: "pixels",
            largest_that_fits: tiny / 16,
        }
    );
    assert!(error
        .to_string()
        .contains("reduce this scene to at most 32 pixels"));

    let bound = gpu_scene_size_bound(12, 1, 4, false);
    let pack = GpuRenderScene::pack(&scene, &camera, 2, 2, &settings, LIMITS).expect("packs");
    let actual = pack.sizes();
    assert!(actual.node_bytes <= bound.node_bytes);
    assert_eq!(actual.triangle_bytes, bound.triangle_bytes);
    assert_eq!(actual.face_bytes, bound.face_bytes);
    assert_eq!(actual.part_bytes, bound.part_bytes);
    assert_eq!(actual.accumulation_bytes, bound.accumulation_bytes);
    assert_eq!(actual.channel_bytes, bound.channel_bytes);
    assert!(pack.sizes().tree.depth <= bound.tree.depth);

    let error = GpuRenderScene::pack(
        &scene,
        &camera,
        2,
        2,
        &settings,
        GpuSceneLimits {
            max_storage_binding: 400,
            ..LIMITS
        },
    )
    .expect_err("too little room for the faces");
    assert!(matches!(
        error,
        FlatTreeError::PackedBufferTooLarge { buffer: "face", .. }
    ));
}

#[test]
fn a_sun_too_small_for_f32_directions_and_an_empty_scene_are_refused() {
    let cube = create_cube(2.0, 2.0, 2.0);
    let tiny_sun = Sun::new(
        Vector3::new(0.0, 0.0, 1.0),
        MIN_GPU_SUN_ANGULAR_RADIUS * 0.5,
        [1.0; 3],
    )
    .expect("a valid CPU sun");
    let scene = scene_of(
        &cube,
        SHARP_CREASE,
        Matrix4X4::IDENTITY,
        ScatteringSurface::matte_plastic([0.5; 3]),
        LightSetup::sun_only(tiny_sun),
    );
    let error = GpuRenderScene::pack(
        &scene,
        &top_down_camera(10.0, 4.0),
        4,
        4,
        &PathTraceSettings::default(),
        LIMITS,
    )
    .expect_err("refused");
    assert!(error.to_string().contains("sun's disc"), "{error}");

    let empty = RenderScene::empty(LightSetup::environment_only(Environment::black()));
    let error = GpuRenderScene::pack(
        &empty,
        &top_down_camera(10.0, 4.0),
        4,
        4,
        &PathTraceSettings::default(),
        LIMITS,
    )
    .expect_err("nothing to bind");
    assert_eq!(error, FlatTreeError::NoTriangles);
}

#[test]
fn a_part_far_from_the_origin_keeps_its_corners_and_a_small_offset() {
    let cube = create_cube(2.0, 2.0, 2.0);
    let far = 1e7;
    let scene = scene_of(
        &cube,
        SHARP_CREASE,
        Matrix4X4::create_translation(far, 0.0, 0.0),
        ScatteringSurface::matte_plastic([0.5; 3]),
        lights(),
    );
    let camera = RenderCamera::perspective(
        Vector3::new(far + 5.0, -6.0, 4.0),
        Vector3::new(far, 0.0, 0.0),
        Vector3::UNIT_Z,
        40.0,
    )
    .expect("a camera");
    let pack = GpuRenderScene::pack(&scene, &camera, 4, 4, &PathTraceSettings::default(), LIMITS)
        .expect("packs");
    for triangle in pack.tree().triangles() {
        for corner in [triangle.v0, triangle.v1, triangle.v2] {
            for axis in corner {
                assert!(axis.abs() <= 1.0 + 1e-5, "corner coordinate {axis}");
            }
        }
    }
    for face in pack.faces() {
        assert!(face.ray_offset < 1e-4, "offset {}", face.ray_offset);
    }
}

/// The line of pixel (`x`, `y`) as the shader builds it from the packed scene, in world space.
fn packed_line(pack: &GpuRenderScene, x: f32, y: f32, size: (f32, f32)) -> (Vector3, Vector3) {
    let scene = pack.view();
    let shorter = size.0.min(size.1);
    let horizontal = (2.0 * x - size.0) / shorter;
    let vertical = (2.0 * y - size.1) / shorter;
    let along = |lane: [f32; 4], k: f32| [lane[0] * k, lane[1] * k, lane[2] * k];
    let right = along(scene.right, horizontal * scene.eye[3]);
    let up = along(scene.up, vertical * scene.eye[3]);
    let orthographic = scene.flags & SCENE_FLAG_ORTHOGRAPHIC != 0;
    let origin: [f32; 3] = if orthographic {
        std::array::from_fn(|axis| scene.eye[axis] + right[axis] + up[axis])
    } else {
        std::array::from_fn(|axis| scene.eye[axis])
    };
    let direction: [f32; 3] = if orthographic {
        std::array::from_fn(|axis| scene.forward[axis])
    } else {
        std::array::from_fn(|axis| scene.forward[axis] + right[axis] + up[axis])
    };
    let widen = |v: [f32; 3]| Vector3::new(f64::from(v[0]), f64::from(v[1]), f64::from(v[2]));
    (
        widen(origin) + pack.scene_origin(),
        widen(direction).get_normal(),
    )
}

/// How far `point` is from the line through `origin` along unit `direction`.
fn distance_from_line(point: Vector3, origin: Vector3, direction: Vector3) -> f64 {
    let relative = point - origin;
    (relative - direction * relative.dot(direction)).length()
}

fn far_eye_scene() -> (RenderScene, Vector3) {
    // A part well away from the origin, as a build plate's corner would be.
    let center = Vector3::new(137.3, 211.9, 20.0);
    let cube = create_cube(40.0, 40.0, 40.0);
    let scene = scene_of(
        &cube,
        SHARP_CREASE,
        Matrix4X4::create_translation(center.x, center.y, center.z),
        ScatteringSurface::matte_plastic([0.5; 3]),
        lights(),
    );
    (scene, center)
}

#[test]
fn an_orthographic_eye_pulled_back_1e5_keeps_the_rays_on_their_lines() {
    let (scene, center) = far_eye_scene();
    // The app's pull-back, about 1e5 out, on a diagonal so every coordinate is a long one.
    let toward_eye = Vector3::new(0.6, -0.7, 0.3873).get_normal();
    let eye = center + toward_eye * 1.0e5;
    let camera = RenderCamera::orthographic(eye, center, Vector3::UNIT_Z, 80.0).expect("a camera");
    let (width, height) = (200_u32, 100_u32);
    let pack = GpuRenderScene::pack(
        &scene,
        &camera,
        width,
        height,
        &PathTraceSettings::default(),
        LIMITS,
    )
    .expect("packs");
    let packed_eye = pack.view().eye;
    let eye_distance = (f64::from(packed_eye[0]).powi(2)
        + f64::from(packed_eye[1]).powi(2)
        + f64::from(packed_eye[2]).powi(2))
    .sqrt();
    assert!(eye_distance < 200.0, "the eye came in to {eye_distance}");
    let size = (width as f32, height as f32);
    for (x, y) in [(0.5, 0.5), (100.25, 50.75), (199.5, 99.5), (17.5, 83.25)] {
        let cpu = camera.ray_through(f64::from(x), f64::from(y), width, height);
        let (origin, direction) = packed_line(&pack, x, y, size);
        // Where the ray crosses the scene's middle, on the CPU's line.
        let along = (center - cpu.origin).dot(cpu.direction_normal);
        let point = cpu.origin + cpu.direction_normal * along;
        let gap = distance_from_line(point, origin, direction);
        // A pixel here is 0.8 units: this is well under a thousandth of one.
        assert!(gap < 5e-4, "pixel ({x}, {y}) is {gap} off its CPU line");
    }
}

#[test]
fn a_perspective_eye_1e5_out_stays_within_its_pixel() {
    let (scene, center) = far_eye_scene();
    let toward_eye = Vector3::new(0.6, -0.7, 0.3873).get_normal();
    let eye = center + toward_eye * 1.0e5;
    let camera = RenderCamera::perspective(eye, center, Vector3::UNIT_Z, 0.05).expect("a camera");
    let (width, height) = (200_u32, 100_u32);
    let pack = GpuRenderScene::pack(
        &scene,
        &camera,
        width,
        height,
        &PathTraceSettings::default(),
        LIMITS,
    )
    .expect("packs");
    let size = (width as f32, height as f32);
    let footprint = 2.0 * 1.0e5 * 0.05_f64.to_radians().tan() / f64::from(height) / 2.0;
    for (x, y) in [(100.0, 50.0), (17.5, 83.25)] {
        let cpu = camera.ray_through(f64::from(x), f64::from(y), width, height);
        let (origin, direction) = packed_line(&pack, x, y, size);
        let along = (center - cpu.origin).dot(cpu.direction_normal);
        let point = cpu.origin + cpu.direction_normal * along;
        let gap = distance_from_line(point, origin, direction);
        assert!(
            gap < footprint * 0.1,
            "pixel ({x}, {y}) is {gap} off a {footprint} footprint"
        );
    }
}

#[test]
fn orbiting_the_camera_does_not_move_the_packed_triangles() {
    let (scene, center) = far_eye_scene();
    let at = |eye: Vector3| {
        let camera =
            RenderCamera::perspective(eye, center, Vector3::UNIT_Z, 40.0).expect("a camera");
        GpuRenderScene::pack(&scene, &camera, 4, 4, &PathTraceSettings::default(), LIMITS)
            .expect("packs")
    };
    let first = at(center + Vector3::new(100.0, 0.0, 40.0));
    let second = at(center + Vector3::new(-30.0, 90.0, 10.0));
    assert_eq!(first.scene_origin(), second.scene_origin());
    assert_eq!(
        first.tree().triangle_bytes(),
        second.tree().triangle_bytes()
    );
    assert_eq!(first.scene_origin(), center);
}

#[test]
fn a_scene_spread_too_far_for_the_precision_says_what_size_would_fit() {
    let cube = create_cube(2.0, 2.0, 2.0);
    let near = PlacedPart::new(
        tree_of(&cube),
        Arc::new(MeshSurface::new(&cube, SHARP_CREASE).expect("a table")),
        Matrix4X4::IDENTITY,
    )
    .expect("placed");
    let far = PlacedPart::new(
        tree_of(&cube),
        Arc::new(MeshSurface::new(&cube, SHARP_CREASE).expect("a table")),
        Matrix4X4::create_translation(2e7, 0.0, 0.0),
    )
    .expect("placed");
    let scene = RenderScene::new(
        PlacedParts::new(vec![near, far]),
        vec![ScatteringSurface::matte_plastic([0.5; 3]); 2],
        lights(),
    )
    .expect("a scene");
    let error = GpuRenderScene::pack(
        &scene,
        &top_down_camera(10.0, 4.0),
        4,
        4,
        &PathTraceSettings::default(),
        LIMITS,
    )
    .expect_err("two parts 2e7 apart can't hold a micrometre");
    let FlatTreeError::SceneTooLarge {
        reach,
        step,
        required_step,
        largest_reach,
    } = error
    else {
        panic!("{error:?}");
    };
    assert_eq!(reach, 1e7 + 1.0);
    assert_eq!(step, reach * f64::from(f32::EPSILON));
    assert_eq!(required_step, 1e-3);
    assert_eq!(largest_reach, 1e-3 / f64::from(f32::EPSILON));
    let words = error.to_string();
    assert!(words.contains("keep every part within"), "{words}");
    assert!(words.contains("render it on the CPU"), "{words}");
}

#[test]
fn a_camera_the_gpu_cannot_carry_says_so_in_its_own_words() {
    let scene = small_scene();
    let far_eye = RenderCamera::perspective(
        Vector3::new(0.0, 0.0, 1e30),
        Vector3::ZERO,
        Vector3::UNIT_Y,
        40.0,
    )
    .expect("a valid CPU camera");
    let error = GpuRenderScene::pack(
        &scene,
        &far_eye,
        4,
        4,
        &PathTraceSettings::default(),
        LIMITS,
    )
    .expect_err("an eye past the coordinate limit");
    assert!(
        error.to_string().contains("the camera is more than"),
        "{error}"
    );

    let huge_extent = RenderCamera::orthographic(
        Vector3::new(0.0, 0.0, 5.0),
        Vector3::ZERO,
        Vector3::UNIT_Y,
        1e300,
    )
    .expect("a valid CPU camera");
    let error = GpuRenderScene::pack(
        &scene,
        &huge_extent,
        4,
        4,
        &PathTraceSettings::default(),
        LIMITS,
    )
    .expect_err("an extent past f32");
    let words = error.to_string();
    assert!(words.contains("the camera's view is too wide"), "{words}");
    assert!(!words.contains("radiance"), "{words}");
}
