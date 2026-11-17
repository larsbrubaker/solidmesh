//! How precisely the GPU path loop's scene packing keeps what is far away (`flat_tree/gpu_scene.rs`,
//! recentring): a part far from the origin keeps its corners and a small ray offset, an eye pulled
//! far back keeps every pixel's ray on its CPU line, orbiting does not move the packed triangles,
//! and a scene spread too far for the asked precision says what size would fit.

use std::sync::Arc;

use solidmesh_math::{Matrix4X4, Vector3};
use solidmesh_mesh::ray_tracer::flat_tree::{
    FlatTreeError, GpuRenderScene, SCENE_FLAG_ORTHOGRAPHIC,
};
use solidmesh_mesh::ray_tracer::{
    MeshSurface, PathTraceSettings, PlacedPart, PlacedParts, RenderCamera, RenderScene,
    ScatteringSurface,
};
use solidmesh_mesh::solids::platonic_solids::create_cube;

use super::path_shader_tests::{lights, LIMITS};
use super::path_tracer_support::{scene_of, top_down_camera, SHARP_CREASE};
use super::surface_hit_support::tree_of;

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
