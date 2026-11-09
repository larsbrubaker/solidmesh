//! The Ray Tracing page's GPU route, the parts that run without a GPU: the numbers the page sends
//! become the same request the CPU route gets, a new camera or look rewrites only what a fresh
//! pack would hold, a scene that doesn't fit is refused in words, and an accumulation decodes to
//! the CPU route's picture through the library's display code.

use solidmesh::mesh::ray_tracer::display_byte;
use solidmesh::mesh::ray_tracer::flat_tree::{
    FlatTreeError, ACCUMULATION_STRIDE, PART_STRIDE, ROULETTE_OFF,
};
use solidmesh_wasm::ray_trace::TraceRequest;
use solidmesh_wasm::ray_trace_gpu::{
    decode_accumulation, fallback_reason, request_from_args, GpuOutcome, GpuSession, TRACE_ARGS,
};
use solidmesh_wasm::ray_trace_scene::{Part, TraceScene};
use solidmesh_wasm::ray_trace_shade::{CustomSliders, Light, Look, Material, Mode};

/// The 17 numbers the page sends for an image, in `ray_trace_pass`'s order.
fn args() -> Vec<f64> {
    vec![
        0.85, 0.42, 1.0, 0.0, 2.0, 91.0, 194.0, 180.0, 0.3, 0.5, 0.25, 1.0, 5.0, 1.0, 40.0, 30.0,
        0.0,
    ]
}

fn look() -> Look {
    Look {
        material: Material::MattePlastic,
        color: [91, 194, 180],
        custom: CustomSliders {
            roughness: 0.3,
            metallic: 0.0,
            transmission: 0.0,
        },
        light: Light::Studio,
        sun_degrees: 5.0,
    }
}

fn request() -> TraceRequest {
    TraceRequest {
        yaw: 0.85,
        pitch: 0.42,
        zoom: 1.0,
        mode: Mode::PathTraced,
        look: look(),
        seed: 1,
        width: 40,
        height: 30,
        preview: false,
    }
}

fn scene() -> TraceScene {
    let mesh = Part::RoundedBox.build(12).expect("the part builds");
    TraceScene::new(&mesh, String::new()).expect("the part traces")
}

fn packed(trace: &TraceScene, request: TraceRequest, max_binding: u64) -> GpuSession {
    match GpuSession::pack(trace, request, max_binding).expect("no bug") {
        GpuOutcome::Fits(session) => session,
        GpuOutcome::Refused(reason) => panic!("expected the scene to fit: {reason}"),
    }
}

const BINDING: u64 = 128 * 1024 * 1024;

#[test]
fn the_page_numbers_become_the_same_request_the_cpu_route_gets() {
    let request = request_from_args(&args()).expect("well formed");
    assert_eq!(request.yaw, 0.85);
    assert_eq!(request.mode, Mode::PathTraced);
    assert_eq!(request.look.material, Material::BrushedMetal);
    assert_eq!(request.look.color, [91, 194, 180]);
    assert_eq!(request.look.custom.metallic, 0.5);
    assert_eq!(request.look.light, Light::Overcast);
    assert_eq!(request.look.sun_degrees, 5.0);
    assert_eq!((request.seed, request.width, request.height), (1, 40, 30));
    assert!(!request.preview);

    let mut preview = args();
    preview[16] = 1.0;
    assert!(request_from_args(&preview).expect("well formed").preview);
}

#[test]
fn a_malformed_request_is_refused_in_words() {
    let error = request_from_args(&args()[..TRACE_ARGS - 1]).expect_err("too short");
    assert!(error.contains("17 numbers"), "{error}");
    let mut bad = args();
    bad[3] = 9.0;
    assert!(request_from_args(&bad)
        .expect_err("no such view")
        .contains("no view 9"));
}

#[test]
fn only_the_scenes_this_gpu_cant_hold_fall_back_to_the_cpu() {
    let fallbacks = [
        FlatTreeError::InvalidCoordinate { triangle_index: 3 },
        FlatTreeError::NotRepresentable {
            reason: "a light is too bright",
        },
        FlatTreeError::TreeTooDeep {
            depth: 90,
            limit: 64,
        },
    ];
    for error in fallbacks {
        assert_eq!(fallback_reason(&error), Some(error.to_string()));
    }
    assert_eq!(fallback_reason(&FlatTreeError::NoTriangles), None);
    assert_eq!(fallback_reason(&FlatTreeError::UnexpectedBvhLeaf), None);
}

#[test]
fn a_binding_too_small_for_the_scene_is_a_refusal_not_an_error() {
    let trace = scene();
    match GpuSession::pack(&trace, request(), 1024).expect("not a bug") {
        GpuOutcome::Refused(reason) => {
            assert!(reason.contains("bytes"), "says what would fit: {reason}");
            assert!(reason.contains("1024"), "{reason}");
        }
        GpuOutcome::Fits(_) => panic!("a 1 KiB binding can't hold this scene"),
    }
}

#[test]
fn refining_paths_are_the_cpu_routes_and_a_preview_is_short() {
    let trace = scene();
    let mut preview = request();
    preview.preview = true;
    let session = packed(&trace, request(), BINDING);
    let view = session.packed().view();
    assert_eq!(
        (view.max_bounces, view.roulette_after, view.seed_low),
        (24, 3, 1)
    );
    let session = packed(&trace, preview, BINDING);
    let view = session.packed().view();
    assert_eq!((view.max_bounces, view.roulette_after), (3, ROULETTE_OFF));
}

#[test]
fn orbiting_rewrites_only_what_a_fresh_pack_would_hold() {
    let trace = scene();
    let mut session = packed(&trace, request(), BINDING);
    let mut orbited = request();
    orbited.yaw = 2.1;
    orbited.pitch = 0.9;
    orbited.zoom = 2.0;
    orbited.width = 64;
    orbited.height = 48;
    let bytes = match session.view(&trace, orbited).expect("no bug") {
        GpuOutcome::Fits(bytes) => bytes,
        GpuOutcome::Refused(reason) => panic!("{reason}"),
    };
    let fresh = packed(&trace, orbited, BINDING);
    assert_eq!(bytes.view, fresh.packed().view_bytes());
    assert_eq!(bytes.parts, fresh.packed().part_bytes());
}

#[test]
fn a_material_or_light_change_needs_no_new_pack() {
    let trace = scene();
    let mut session = packed(&trace, request(), BINDING);
    let mut changed = request();
    changed.look = Look {
        material: Material::Glass,
        color: [255, 240, 200],
        custom: changed.look.custom,
        light: Light::Overcast,
        sun_degrees: 12.0,
    };
    let bytes = match session.view(&trace, changed).expect("no bug") {
        GpuOutcome::Fits(bytes) => bytes,
        GpuOutcome::Refused(reason) => panic!("{reason}"),
    };
    let fresh = packed(&trace, changed, BINDING);
    assert_eq!(bytes.parts.len(), 2 * PART_STRIDE);
    assert_eq!(bytes.parts, fresh.packed().part_bytes());
    assert_eq!(bytes.view, fresh.packed().view_bytes());
}

/// One accumulation record: sum of RGB, counted samples.
fn record(sum: [f32; 3], count: f32) -> [u8; ACCUMULATION_STRIDE] {
    let mut bytes = [0u8; ACCUMULATION_STRIDE];
    for (at, value) in sum.into_iter().chain([count]).enumerate() {
        bytes[at * 4..at * 4 + 4].copy_from_slice(&value.to_le_bytes());
    }
    bytes
}

#[test]
fn an_accumulation_decodes_through_the_librarys_display_code_with_rows_flipped() {
    // Row 0 of the accumulation is the bottom of the image; the page's image starts at the top.
    let bottom = record([1.0, 0.5, 0.25], 2.0);
    let top = record([3.0, 0.0, 6.0], 3.0);
    let mut accumulation = Vec::new();
    accumulation.extend_from_slice(&bottom);
    accumulation.extend_from_slice(&top);
    let mut display = vec![9u8; 8];
    decode_accumulation(&accumulation, 1, 2, &mut display).expect("right size");
    let byte = |linear: f64| display_byte(linear, 1.0);
    assert_eq!(
        display,
        vec![
            byte(1.0),
            byte(0.0),
            byte(2.0),
            255,
            byte(0.5),
            byte(0.25),
            byte(0.125),
            255
        ]
    );
}

#[test]
fn a_pixel_with_no_samples_yet_keeps_the_old_picture() {
    let accumulation = record([0.0; 3], 0.0);
    let mut display = vec![10, 20, 30, 255];
    decode_accumulation(&accumulation, 1, 1, &mut display).expect("right size");
    assert_eq!(display, vec![10, 20, 30, 255]);
}

#[test]
fn a_wrong_sized_accumulation_is_an_error_in_words() {
    let mut display = vec![0u8; 4];
    let error = decode_accumulation(&[0u8; 10], 1, 1, &mut display).expect_err("wrong size");
    assert!(error.contains("10 bytes"), "{error}");
}

#[test]
fn a_session_shows_a_smaller_image_after_a_larger_one_and_stretches_it() {
    let trace = scene();
    let mut session = packed(&trace, request(), BINDING);
    let big: Vec<u8> = (0..4).flat_map(|_| record([0.75; 3], 1.0)).collect();
    let shown = session.show(&big, 2, 2).expect("right size").to_vec();
    assert_eq!(shown.len(), 16);
    let empty = record([0.0; 3], 0.0);
    let small = session.show(&empty, 1, 1).expect("right size").to_vec();
    assert_eq!(
        small[..4],
        shown[..4],
        "the old picture shows through at the new size"
    );
}

#[test]
fn limits_are_sized_from_the_part_so_a_part_of_any_size_fits() {
    // A part far larger than the demo's own: the f32 grid scales with it, so it still packs.
    let mesh = Part::RoundedBox.build(12).expect("the part builds");
    let mut big = mesh.clone();
    for corner in big.vertices.iter_mut() {
        corner.x *= 1000.0;
        corner.y *= 1000.0;
        corner.z *= 1000.0;
    }
    let trace = TraceScene::new(&big, String::new()).expect("the part traces");
    packed(&trace, request(), BINDING);
}
