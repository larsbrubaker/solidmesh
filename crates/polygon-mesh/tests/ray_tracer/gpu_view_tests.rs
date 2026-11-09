//! What a GPU view may and may not change, and the stepped pack. A view change must rewrite the
//! view uniform and no other buffer, because orbiting a composited view is meant to cost one small
//! write and no rebuild; the stepped pack must produce the one-call pack's bytes, because a host
//! with no threads builds a scene through it and a different scene would be a silent wrong answer.
//! The floor's light and the channels themselves are measured against the CPU on a real GPU (the
//! demo's render check and the app's own GPU test); here only the bytes that reach the shader.

use solidmesh_math::{Matrix4X4, Vector3};
use solidmesh_mesh::ray_tracer::flat_tree::{
    FlatTreeError, GpuCompositing, GpuFloor, GpuRenderScene, GpuSceneBuilder, GpuSceneLimits,
    GpuView, CHANNEL_STRIDE, SCENE_FLAG_COMPOSITE, SCENE_FLAG_FLOOR,
    WEBGPU_DEFAULT_MAX_STORAGE_BINDING,
};
use solidmesh_mesh::ray_tracer::{
    Environment, LightSetup, PathTraceSettings, RenderCamera, ScatteringSurface, Sun,
};
use solidmesh_mesh::solids::platonic_solids::create_cube;

use super::path_tracer_support::{scene_of, SHARP_CREASE};

const LIMITS: GpuSceneLimits = GpuSceneLimits {
    max_storage_binding: WEBGPU_DEFAULT_MAX_STORAGE_BINDING,
    max_vertex_step: 1e-3,
};

fn lights() -> LightSetup {
    LightSetup {
        environment: Environment::gradient([0.1, 0.2, 0.3], [0.4, 0.5, 0.6], [0.7, 0.8, 0.9])
            .expect("a gradient"),
        sun: Some(Sun::new(Vector3::new(0.3, 0.2, 1.0), 0.05, [10.0, 20.0, 30.0]).expect("a sun")),
    }
}

fn scene() -> solidmesh_mesh::ray_tracer::RenderScene {
    let cube = create_cube(40.0, 40.0, 40.0);
    scene_of(
        &cube,
        SHARP_CREASE,
        Matrix4X4::create_translation(137.3, 211.9, 20.0),
        ScatteringSurface::matte_plastic([0.5; 3]),
        lights(),
    )
}

fn floor() -> GpuFloor {
    GpuFloor {
        center: (137.3, 211.9),
        half_extent: 1.0e5,
        z: 0.0,
        show_through: 0.25,
        surface: ScatteringSurface::matte_plastic([0.3; 3]),
    }
}

fn view_at(eye: Vector3, compositing: GpuCompositing) -> GpuView {
    GpuView {
        camera: RenderCamera::perspective(
            eye,
            Vector3::new(137.3, 211.9, 20.0),
            Vector3::UNIT_Z,
            40.0,
        )
        .expect("a camera"),
        image: (24, 16),
        trace: (30, 16),
        lights: lights(),
        settings: PathTraceSettings::default(),
        compositing,
    }
}

const CENTER: Vector3 = Vector3::new(137.3, 211.9, 20.0);

#[test]
fn moving_the_camera_changes_only_the_view_record() {
    let scene = scene();
    let first = view_at(
        CENTER + Vector3::new(100.0, 0.0, 40.0),
        GpuCompositing::Floor(floor()),
    );
    let second = view_at(
        CENTER + Vector3::new(-30.0, 90.0, 10.0),
        GpuCompositing::Floor(floor()),
    );
    let mut pack = GpuRenderScene::pack_view(&scene, &first, LIMITS).expect("packs");
    let before = (
        pack.tree().node_bytes(),
        pack.tree().triangle_bytes(),
        pack.face_bytes(),
        pack.part_bytes(),
        pack.srgb_table_bytes(),
    );
    let first_view = pack.view_bytes();
    let other = GpuRenderScene::pack_view(&scene, &second, LIMITS).expect("packs");

    pack.set_view(&scene, &second).expect("the view changes");

    assert_ne!(pack.view_bytes(), first_view, "the view record moved");
    assert_eq!(pack.view_bytes(), other.view_bytes());
    assert_eq!(pack.tree().node_bytes(), before.0);
    assert_eq!(pack.tree().triangle_bytes(), before.1);
    assert_eq!(pack.face_bytes(), before.2);
    assert_eq!(pack.part_bytes(), before.3);
    assert_eq!(pack.srgb_table_bytes(), before.4);
    assert_eq!(pack.scene_origin(), other.scene_origin());
    assert_eq!(
        pack.tree().triangle_bytes(),
        other.tree().triangle_bytes(),
        "a re-pack from a new view holds the same triangles"
    );
}

#[test]
fn a_floor_is_carried_only_in_the_view_record() {
    let scene = scene();
    let eye = CENTER + Vector3::new(100.0, -60.0, 80.0);
    let floor_view = view_at(eye, GpuCompositing::Floor(floor()));
    let plain = view_at(eye, GpuCompositing::Plain);
    let with_floor = GpuRenderScene::pack_view(&scene, &floor_view, LIMITS).expect("packs");
    let without = GpuRenderScene::pack_view(&scene, &plain, LIMITS).expect("packs");

    assert_eq!(
        with_floor.tree().triangle_bytes(),
        without.tree().triangle_bytes()
    );
    assert_eq!(with_floor.face_bytes(), without.face_bytes());
    assert_eq!(with_floor.parts().len(), 1, "the floor is not a part");
    // A floor 1e5 out changes neither the origin nor the precision of the parts' coordinates.
    assert_eq!(with_floor.scene_origin(), without.scene_origin());

    let record = with_floor.view();
    assert_eq!(record.trace_width, 30);
    assert_eq!(record.width, 24);
    assert_ne!(record.flags & SCENE_FLAG_FLOOR, 0);
    assert_ne!(record.flags & SCENE_FLAG_COMPOSITE, 0);
    // The height is relative to the scene's center, the plane's own bits.
    assert_eq!(record.floor_plane[2], -20.0);
    assert_eq!(record.floor_plane[0], 0.0);
    assert_eq!(record.floor_plane[3], 1.0e5);
    assert_eq!(record.floor_show_through, 0.25);
}

#[test]
fn the_stepped_pack_is_the_one_call_pack_bit_for_bit() {
    let scene = scene();
    let view = view_at(
        CENTER + Vector3::new(100.0, 40.0, 70.0),
        GpuCompositing::Floor(floor()),
    );
    let one_call = GpuRenderScene::pack_view(&scene, &view, LIMITS).expect("packs");
    for units in [1_usize, 2, 3, 7, 25] {
        let mut builder = GpuSceneBuilder::new(&scene, view, LIMITS).expect("starts");
        let mut steps = 0;
        let stepped = loop {
            steps += 1;
            assert!(steps < 10_000, "units {units}: the pack never finished");
            if let Some(packed) = builder.step(units).expect("steps") {
                break packed;
            }
        };
        assert!(steps > 1, "units {units}: the work was not sliced");
        assert_eq!(
            stepped.tree().node_bytes(),
            one_call.tree().node_bytes(),
            "units {units}"
        );
        assert_eq!(
            stepped.tree().triangle_bytes(),
            one_call.tree().triangle_bytes(),
            "units {units}"
        );
        assert_eq!(stepped.face_bytes(), one_call.face_bytes(), "units {units}");
        assert_eq!(stepped.part_bytes(), one_call.part_bytes(), "units {units}");
        assert_eq!(stepped.view_bytes(), one_call.view_bytes(), "units {units}");
        assert_eq!(
            stepped.srgb_table_bytes(),
            one_call.srgb_table_bytes(),
            "units {units}"
        );
        assert_eq!(
            stepped.scene_origin(),
            one_call.scene_origin(),
            "units {units}"
        );
    }
}

#[test]
fn a_finished_builder_refuses_to_answer_twice_and_a_host_can_defer_the_tree() {
    let scene = scene();
    let view = view_at(
        CENTER + Vector3::new(100.0, 0.0, 40.0),
        GpuCompositing::Plain,
    );
    let mut builder = GpuSceneBuilder::new(&scene, view, LIMITS).expect("starts");
    while !builder.next_step_builds_tree() {
        assert!(
            builder.step(1).expect("steps").is_none(),
            "the tree comes last"
        );
    }
    assert!(builder.step(1).expect("the tree").is_some());
    let again = builder.step(1).expect_err("already finished");
    assert!(
        matches!(again, FlatTreeError::NotRepresentable { .. }),
        "{again}"
    );
}

#[test]
fn a_composited_image_too_big_for_its_channel_buffer_is_refused_before_any_work() {
    let scene = scene();
    let view = GpuView {
        image: (1000, 1000),
        trace: (1000, 1000),
        ..view_at(
            CENTER + Vector3::new(100.0, 0.0, 40.0),
            GpuCompositing::Floor(floor()),
        )
    };
    let limits = GpuSceneLimits {
        max_storage_binding: 1000 * 1000 * 16,
        ..LIMITS
    };
    match GpuSceneBuilder::new(&scene, view, limits) {
        Err(FlatTreeError::PackedBufferTooLarge {
            buffer,
            largest_that_fits,
            ..
        }) => {
            assert_eq!(buffer, "coverage channel");
            assert_eq!(largest_that_fits, 1000 * 1000 * 16 / CHANNEL_STRIDE as u64);
        }
        Err(other) => panic!("{other}"),
        Ok(_) => panic!("the channels can't fit"),
    }
    // The same image with no compositing fits the same limit.
    let plain = GpuView {
        compositing: GpuCompositing::Plain,
        ..view
    };
    assert!(GpuSceneBuilder::new(&scene, plain, limits).is_ok());
}

#[test]
fn a_floor_the_shader_cannot_carry_says_so() {
    let scene = scene();
    for bad in [
        GpuFloor {
            half_extent: 0.0,
            ..floor()
        },
        GpuFloor {
            show_through: 1.5,
            ..floor()
        },
        GpuFloor {
            z: f64::NAN,
            ..floor()
        },
    ] {
        let view = view_at(
            CENTER + Vector3::new(100.0, 0.0, 40.0),
            GpuCompositing::Floor(bad),
        );
        let error = GpuRenderScene::pack_view(&scene, &view, LIMITS).expect_err("refused");
        assert!(error.to_string().contains("floor's height"), "{error}");
    }
}
