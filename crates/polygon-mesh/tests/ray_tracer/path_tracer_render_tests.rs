//! The render loop's contract: results that are bit-identical across tile sizes, order and
//! threads, a seed that matters, cancellation, an empty scene, bad values, the camera's rays
//! and the display conversion.

use solidmesh_math::{Matrix4X4, Vector2, Vector3, WorldView};
use solidmesh_mesh::ray_tracer::{
    display_byte, linear_to_srgb, tone_map, AccumulatedPixel, Environment, LightSetup,
    PathTraceSettings, PathTracer, RenderCamera, RenderScene, RenderSurface, RenderWork,
    ScatteringSurface, Sun, TileRect,
};
use solidmesh_mesh::CancelToken;

use crate::path_tracer_support::{fine_sphere, scene_of, top_down_camera, SMOOTH_CREASE};

const SIZE: (u32, u32) = (17, 11);

/// A scene with some of everything: sun, gradient, a colored part with a shine.
fn busy_scene() -> RenderScene {
    let surface = ScatteringSurface::new([0.6, 0.3, 0.2], 0.0, 0.5, 0.0, 1.5);
    let lights = LightSetup {
        environment: Environment::gradient([0.3, 0.4, 0.9], [0.9, 0.85, 0.8], [0.15, 0.1, 0.1])
            .unwrap(),
        sun: Some(Sun::new(Vector3::new(0.5, 0.3, 0.8), 0.1, [6.0, 5.5, 5.0]).unwrap()),
    };
    scene_of(
        &fine_sphere(10.0),
        SMOOTH_CREASE,
        Matrix4X4::IDENTITY,
        surface,
        lights,
    )
}

fn tracer(scene: &RenderScene, seed: u64) -> PathTracer<'_> {
    let settings = PathTraceSettings {
        seed,
        ..PathTraceSettings::default()
    };
    PathTracer::new(scene, top_down_camera(40.0, 30.0), SIZE.0, SIZE.1, settings).unwrap()
}

fn assert_bit_identical(a: &RenderSurface, b: &RenderSurface) {
    for y in 0..SIZE.1 {
        for x in 0..SIZE.0 {
            let (pa, pb) = (a.pixel(x, y).unwrap(), b.pixel(x, y).unwrap());
            assert_eq!(pa.samples(), pb.samples(), "pixel {x},{y} samples");
            for channel in 0..3 {
                assert_eq!(
                    pa.sum()[channel].to_bits(),
                    pb.sum()[channel].to_bits(),
                    "pixel {x},{y} channel {channel}"
                );
            }
        }
    }
}

fn render_whole(tracer: &PathTracer<'_>) -> RenderSurface {
    let mut surface = RenderSurface::new(SIZE.0, SIZE.1).unwrap();
    for sample in 0..4 {
        tracer
            .render_pass(&mut surface, sample, &CancelToken::new())
            .unwrap();
    }
    surface
}

/// However the image is cut into tiles, in whatever order, the bits are the same.
#[test]
fn the_result_does_not_depend_on_tile_size_or_order() {
    let scene = busy_scene();
    let renderer = tracer(&scene, 7);
    let expected = render_whole(&renderer);

    for (tile_w, tile_h) in [(1, 1), (3, 7), (5, 2), (100, 3)] {
        let mut surface = RenderSurface::new(SIZE.0, SIZE.1).unwrap();
        for sample in 0..4 {
            let mut tiles = surface.tiles_mut(tile_w, tile_h).unwrap();
            // Reverse every other pass, so tile order differs pass to pass as well.
            if sample % 2 == 1 {
                tiles.reverse();
            }
            for tile in &mut tiles {
                renderer
                    .render_tile(tile, sample, &CancelToken::new())
                    .unwrap();
            }
        }
        assert_bit_identical(&expected, &surface);
    }
}

/// Tiles spread across real threads, through the same disjoint windows, give the same bits.
#[test]
fn the_result_is_bit_identical_across_threads() {
    let scene = busy_scene();
    let renderer = tracer(&scene, 7);
    let expected = render_whole(&renderer);

    let mut surface = RenderSurface::new(SIZE.0, SIZE.1).unwrap();
    let cancel = CancelToken::new();
    let renderer = &renderer;
    let cancel = &cancel;
    for sample in 0..4 {
        let tiles = surface.tiles_mut(4, 3).unwrap();
        std::thread::scope(|scope| {
            for mut tile in tiles {
                scope.spawn(move || renderer.render_tile(&mut tile, sample, cancel).unwrap());
            }
        });
    }
    assert_bit_identical(&expected, &surface);
}

/// A different seed gives a different image, and the same seed the same one: both halves of
/// "the seed is what decides".
#[test]
fn the_seed_decides_the_noise() {
    let scene = busy_scene();
    let first = render_whole(&tracer(&scene, 11));
    let again = render_whole(&tracer(&scene, 11));
    assert_bit_identical(&first, &again);
    let other = render_whole(&tracer(&scene, 12));
    let differs = (0..SIZE.0).any(|x| {
        first.pixel(x, SIZE.1 / 2).unwrap().sum() != other.pixel(x, SIZE.1 / 2).unwrap().sum()
    });
    assert!(differs, "seed 12 reproduced seed 11's image");
}

/// A cancelled token stops the pass with the buffer valid, and repeating the pass completes it
/// and adds nothing to pixels that already had the sample.
#[test]
fn cancelling_leaves_a_valid_buffer_and_the_pass_can_be_finished() {
    let scene = busy_scene();
    let renderer = tracer(&scene, 3);
    let expected = render_whole(&renderer);

    let mut surface = RenderSurface::new(SIZE.0, SIZE.1).unwrap();
    let cancel = CancelToken::new();
    cancel.cancel();
    let work = renderer.render_pass(&mut surface, 0, &cancel).unwrap();
    assert!(work.cancelled);
    assert_eq!(work.pixels_rendered, 0, "a cancelled token still rendered");

    // A pass cut off a third of the way along the first row: finish it, then the rest.
    let mut partial = surface.tile_mut(TileRect {
        x: 0,
        y: 0,
        width: 5,
        height: 1,
    });
    renderer
        .render_tile(partial.as_mut().unwrap(), 0, &CancelToken::new())
        .unwrap();
    drop(partial);
    let work = renderer
        .render_pass(&mut surface, 0, &CancelToken::new())
        .unwrap();
    assert_eq!(
        work.pixels_rendered,
        u64::from(SIZE.0 * SIZE.1) - 5,
        "the repeat rendered pixels that already had the sample"
    );
    for sample in 1..4 {
        renderer
            .render_pass(&mut surface, sample, &CancelToken::new())
            .unwrap();
    }
    assert_bit_identical(&expected, &surface);
}

/// A cancel arriving during a pass stops it, leaves every pixel with whole samples and a finite
/// sum, and can be raced from another thread without making the buffer unreadable.
#[test]
fn a_cancel_during_a_pass_stops_it_promptly() {
    let scene = busy_scene();
    let renderer = PathTracer::new(
        &scene,
        top_down_camera(40.0, 30.0),
        400,
        400,
        PathTraceSettings::default(),
    )
    .unwrap();
    let mut surface = RenderSurface::new(400, 400).unwrap();
    let cancel = CancelToken::new();
    let work = std::thread::scope(|scope| {
        let canceller = cancel.clone();
        scope.spawn(move || canceller.cancel());
        renderer.render_pass(&mut surface, 0, &cancel).unwrap()
    });
    assert!(cancel.is_cancelled());
    if work.cancelled {
        assert!(work.pixels_rendered < 400 * 400);
    }
    for y in 0..400 {
        for x in 0..400 {
            let pixel = surface.pixel(x, y).unwrap();
            assert!(pixel.samples() <= 1);
            assert!(pixel.mean().iter().all(|c| c.is_finite()));
        }
    }
}

/// No parts: every pixel is the environment, exactly, and the camera ray hits nothing.
#[test]
fn an_empty_scene_shows_the_environment() {
    let environment =
        Environment::gradient([0.2, 0.4, 0.8], [0.9, 0.8, 0.7], [0.1, 0.1, 0.1]).unwrap();
    let scene = RenderScene::empty(LightSetup::environment_only(environment));
    let camera = RenderCamera::orthographic(
        Vector3::new(0.0, 0.0, 10.0),
        Vector3::ZERO,
        Vector3::UNIT_Y,
        4.0,
    )
    .unwrap();
    let tracer = PathTracer::new(&scene, camera, 2, 2, PathTraceSettings::default()).unwrap();
    let mut work = RenderWork::default();
    let value = tracer.sample_pixel(0, 0, 0, &mut work).unwrap();
    assert_eq!(value, environment.radiance(-Vector3::UNIT_Z));
    assert_eq!(work.total_rays(), 1);
}

/// A NaN or an infinity from a broken path costs that one sample, and the pixel keeps counting.
#[test]
fn a_bad_sample_is_dropped_and_counted_and_never_poisons_the_pixel() {
    let mut pixel = AccumulatedPixel::default();
    assert_eq!(pixel.add_sample(0, [0.5, 0.25, 0.75]), Some(true));
    assert_eq!(pixel.add_sample(1, [f64::NAN, 1.0, 1.0]), Some(false));
    assert_eq!(pixel.add_sample(2, [1.0, f64::INFINITY, 1.0]), Some(false));
    assert_eq!(pixel.add_sample(3, [1.0, -1.0, 1.0]), Some(false));
    assert_eq!(pixel.add_sample(4, [0.5, 0.25, 0.75]), Some(true));
    assert_eq!(pixel.samples(), 5);
    assert_eq!(pixel.dropped(), 3);
    assert_eq!(pixel.mean(), [0.5, 0.25, 0.75]);
    // A sample out of its turn is refused and changes nothing.
    assert_eq!(pixel.add_sample(9, [9.0; 3]), None);
    assert_eq!(pixel.samples(), 5);
}

/// A huge but legal radiance overflows the running sum, which is dropped and counted like a NaN.
#[test]
fn an_overflowing_sum_drops_the_sample() {
    let huge = 1.7e308;
    let scene = RenderScene::empty(LightSetup::environment_only(
        Environment::constant([huge; 3]).unwrap(),
    ));
    let tracer = PathTracer::new(
        &scene,
        top_down_camera(5.0, 2.0),
        1,
        1,
        PathTraceSettings::default(),
    )
    .unwrap();
    let mut surface = RenderSurface::new(1, 1).unwrap();
    let cancel = CancelToken::new();
    tracer.render_pass(&mut surface, 0, &cancel).unwrap();
    let work = tracer.render_pass(&mut surface, 1, &cancel).unwrap();
    assert_eq!(work.dropped_samples, 1);
    let pixel = surface.pixel(0, 0).unwrap();
    assert_eq!(pixel.dropped(), 1);
    assert_eq!(pixel.mean(), [huge; 3]);
}

/// Perspective rays through known screen positions reach the expected points of a plane at the
/// target, whichever side of the image is shorter.
#[test]
fn perspective_rays_hit_known_points_both_ways_round() {
    let eye = Vector3::new(0.0, 0.0, 10.0);
    let camera = RenderCamera::perspective(eye, Vector3::ZERO, Vector3::UNIT_Y, 90.0).unwrap();
    let hit_plane = |x: f64, y: f64, width: u32, height: u32| {
        let ray = camera.ray_through(x, y, width, height);
        assert!((ray.direction_normal.length() - 1.0).abs() < 1e-12);
        let distance = -ray.origin.z / ray.direction_normal.z;
        ray.origin + ray.direction_normal * distance
    };
    // Landscape: height is shorter, so the 90 degree field is across y.
    let top_right = hit_plane(200.0, 100.0, 200, 100);
    assert!((top_right.x - 20.0).abs() < 1e-9 && (top_right.y - 10.0).abs() < 1e-9);
    // Portrait: width is shorter, so 90 degrees is across x.
    let top_right = hit_plane(100.0, 200.0, 100, 200);
    assert!((top_right.x - 10.0).abs() < 1e-9 && (top_right.y - 20.0).abs() < 1e-9);
    // Bottom-left origin, y up: the screen's upper half is the world's +Y.
    assert!(hit_plane(50.0, 150.0, 100, 200).y > 0.0);
    // The center is the view axis, whatever the aspect.
    let center = hit_plane(100.0, 50.0, 200, 100);
    assert!(center.x.abs() < 1e-9 && center.y.abs() < 1e-9);
}

/// Orthographic rays are parallel, start on the eye plane and frame the shorter side exactly.
#[test]
fn orthographic_rays_hit_known_points_both_ways_round() {
    let camera = RenderCamera::orthographic(
        Vector3::new(1.0, 2.0, 10.0),
        Vector3::new(1.0, 2.0, 0.0),
        Vector3::UNIT_Y,
        4.0,
    )
    .unwrap();
    let wide = camera.ray_through(200.0, 100.0, 200, 100);
    assert_eq!(wide.direction_normal, -Vector3::UNIT_Z);
    assert!((wide.origin.x - 5.0).abs() < 1e-12 && (wide.origin.y - 4.0).abs() < 1e-12);
    let tall = camera.ray_through(100.0, 200.0, 100, 200);
    assert!((tall.origin.x - 3.0).abs() < 1e-12 && (tall.origin.y - 6.0).abs() < 1e-12);
}

// The camera agrees with the view MatterCAD draws in landscape, where `WorldView`'s vertical field
/// of view is also the shorter side's. The portrait case is the next test.
#[test]
fn a_perspective_camera_matches_world_view() {
    let (width, height) = (640.0, 480.0);
    let view = WorldView::new(width, height);
    let eye = view.eye_position();
    let camera = RenderCamera::perspective(eye, Vector3::ZERO, Vector3::UNIT_Y, 45.0).unwrap();
    for (x, y) in [(0.0, 0.0), (640.0, 0.0), (320.0, 240.0), (100.0, 350.0)] {
        let ours = camera.ray_through(x, y, 640, 480);
        let theirs = view.get_ray_for_local_bounds(Vector2::new(x, y));
        assert!(
            (ours.origin - theirs.origin).length() < 1e-9,
            "origin at {x},{y}"
        );
        assert!(
            (ours.direction_normal - theirs.direction_normal).length() < 1e-9,
            "direction at {x},{y}: {:?} against {:?}",
            ours.direction_normal,
            theirs.direction_normal
        );
    }
}

/// The tone map is monotonic and the display conversion has exact sRGB endpoints; display bytes
/// are ordered top row first.
#[test]
fn the_display_conversion_is_monotonic_with_exact_endpoints() {
    let mut last = -1.0;
    for i in 0..4000 {
        let value = f64::from(i) * 0.01;
        let mapped = tone_map(value);
        assert!(mapped >= last, "tone map fell at {value}");
        assert!((0.0..1.0).contains(&mapped));
        last = mapped;
    }
    assert_eq!(tone_map(0.0), 0.0);
    assert_eq!(tone_map(f64::INFINITY), 1.0);
    assert_eq!(tone_map(f64::NAN), 0.0);
    assert_eq!(linear_to_srgb(0.0), 0.0);
    assert_eq!(linear_to_srgb(1.0), 1.0);
    assert_eq!(linear_to_srgb(2.0), 1.0);
    assert_eq!(display_byte(0.0, 1.0), 0);
    assert_eq!(display_byte(f64::INFINITY, 1.0), 255);
    assert_eq!(display_byte(1e300, 1.0), 255);

    let scene = RenderScene::empty(LightSetup::environment_only(
        Environment::gradient([1.0; 3], [0.0; 3], [0.0; 3]).unwrap(),
    ));
    let tracer = PathTracer::new(
        &scene,
        RenderCamera::orthographic(
            Vector3::new(0.0, 0.0, 5.0),
            Vector3::ZERO,
            Vector3::UNIT_Y,
            2.0,
        )
        .unwrap(),
        1,
        2,
        PathTraceSettings::default(),
    )
    .unwrap();
    let mut surface = RenderSurface::new(1, 2).unwrap();
    tracer
        .render_pass(&mut surface, 0, &CancelToken::new())
        .unwrap();
    let bytes = surface.to_display_rgba(1.0);
    assert_eq!(bytes.len(), 8);
    assert_eq!(bytes[3], 255, "alpha");
}

/// In portrait the two conventions part: `WorldView`'s field of view is vertical, `RenderCamera`'s
/// is on the shorter side, here the width. Passing the view's horizontal field reproduces its
/// rays; passing its vertical field does not.
#[test]
fn a_portrait_camera_takes_world_views_horizontal_field() {
    let (width, height) = (480.0, 640.0);
    let view = WorldView::new(width, height);
    let eye = view.eye_position();
    let matching =
        RenderCamera::perspective(eye, Vector3::ZERO, Vector3::UNIT_Y, view.h_fov_degrees())
            .unwrap();
    let vertical = RenderCamera::perspective(eye, Vector3::ZERO, Vector3::UNIT_Y, 45.0).unwrap();
    let mut vertical_differs = false;
    for (x, y) in [(0.0, 0.0), (480.0, 0.0), (240.0, 320.0), (100.0, 500.0)] {
        let theirs = view.get_ray_for_local_bounds(Vector2::new(x, y));
        let ours = matching.ray_through(x, y, 480, 640);
        assert!(
            (ours.direction_normal - theirs.direction_normal).length() < 1e-9,
            "direction at {x},{y}"
        );
        let other = vertical.ray_through(x, y, 480, 640);
        vertical_differs |= (other.direction_normal - theirs.direction_normal).length() > 1e-3;
    }
    assert!(
        vertical_differs,
        "45 degrees on the shorter side matched the view"
    );
}

/// Sizes that could never be allocated are refused with the limit named, without trying.
#[test]
fn a_render_surface_too_large_to_allocate_is_refused_with_its_limit() {
    let largest = RenderSurface::MAX_PIXELS;
    for (width, height) in [(u32::MAX, u32::MAX), (1 << 20, 1 << 20), (u32::MAX, 2)] {
        let error = RenderSurface::new(width, height).unwrap_err();
        let text = error.to_string();
        assert!(
            text.contains(&largest.to_string()),
            "{width} by {height}: the refusal should name the limit {largest}: {text}"
        );
    }
    // One past the limit, with no allocation.
    let side = 1 << 15;
    let past = RenderSurface::new(side, (largest / u64::from(side)) as u32 + 1);
    assert!(past.is_err());
    // A side of zero is refused the same as ever, and an ordinary surface is fine.
    assert!(RenderSurface::new(0, 16).is_err());
    let small = RenderSurface::new(64, 32).unwrap();
    assert_eq!(small.width(), 64);
    assert_eq!(small.height(), 32);
    assert_eq!(small.pixel(63, 31).unwrap().samples(), 0);
}

/// A sun too small for its squared density to stay finite is refused with the limit named, and
/// the smallest allowed sun renders a finite image through every part of the weighting.
#[test]
fn a_sun_with_an_unusable_solid_angle_is_refused_at_the_stated_limit() {
    let refused = Sun::new(Vector3::UNIT_Z, 1e-80, [1.0; 3]).unwrap_err();
    let text = refused.to_string();
    assert!(
        text.contains("1e-150"),
        "the refusal should name the limit: {text}"
    );
    assert!(Sun::new(Vector3::UNIT_Z, 5.5e-76, [1.0; 3]).is_err());

    let smallest = Sun::new(Vector3::UNIT_Z, 5.7e-76, [1.0; 3]).unwrap();
    assert!(smallest.solid_angle() >= 1e-150);
    let scene = scene_of(
        &fine_sphere(10.0),
        SMOOTH_CREASE,
        Matrix4X4::IDENTITY,
        ScatteringSurface::new([0.8; 3], 0.0, 0.1, 0.0, 1.5),
        LightSetup::sun_only(smallest),
    );
    let tracer = PathTracer::new(
        &scene,
        top_down_camera(40.0, 12.0),
        4,
        4,
        PathTraceSettings::default(),
    )
    .unwrap();
    let mut work = RenderWork::default();
    for sample in 0..40 {
        let value = tracer.sample_pixel(1, 2, sample, &mut work).unwrap();
        assert!(
            value
                .iter()
                .all(|channel| channel.is_finite() && *channel >= 0.0),
            "sample {sample}: {value:?}"
        );
    }
}
