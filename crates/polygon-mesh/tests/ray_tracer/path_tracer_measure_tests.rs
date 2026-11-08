//! How fast the path tracer runs on real scenes: rays by kind per second, single-threaded and on
//! 8 threads, and samples per second for a 1920x1080 image at about 100k and 1M triangles, with
//! the ray count per sample and the coherent fraction. Ignored by default, since it reports
//! rather than checks; run it in a release build.

use std::time::Instant;

use solidmesh_math::{Matrix4X4, Vector3};
use solidmesh_mesh::ray_tracer::{
    Environment, LightSetup, PathTraceSettings, PathTracer, RenderCamera, RenderScene,
    RenderSurface, RenderWork, ScatteringSurface, Sun,
};
use solidmesh_mesh::CancelToken;

use crate::path_tracer_support::{scene_of, SMOOTH_CREASE};
use crate::surface_hit_support::sphere;

fn scene_with(segments: usize, rings: usize) -> RenderScene {
    let mut mesh = sphere(50.0, Vector3::ZERO, segments, rings);
    // A large floor under the part, so most camera rays hit something and bounce, as a real
    // render does; an image of mostly sky would flatter the numbers.
    let floor = crate::path_tracer_support::flat_patch(4000.0, -50.0);
    mesh = merged(mesh, floor);
    println!("faces: {}", mesh.faces.len());
    let lights = LightSetup {
        environment: Environment::gradient([0.3, 0.45, 0.9], [0.9, 0.9, 0.85], [0.2, 0.18, 0.15])
            .unwrap(),
        sun: Some(Sun::new(Vector3::new(0.4, -0.5, 0.75), 0.05, [40.0, 38.0, 34.0]).unwrap()),
    };
    scene_of(
        &mesh,
        SMOOTH_CREASE,
        Matrix4X4::IDENTITY,
        ScatteringSurface::matte_plastic([0.5, 0.45, 0.4]),
        lights,
    )
}

fn camera() -> RenderCamera {
    RenderCamera::perspective(
        Vector3::new(85.0, -120.0, 75.0),
        Vector3::new(0.0, 0.0, -5.0),
        Vector3::UNIT_Z,
        40.0,
    )
    .unwrap()
}

/// One pass of a `width` by `height` image spread over `threads`: the work, and seconds taken.
fn timed_pass(
    scene: &RenderScene,
    size: (u32, u32),
    settings: PathTraceSettings,
    threads: usize,
) -> (RenderWork, f64) {
    let tracer = PathTracer::new(scene, camera(), size.0, size.1, settings).unwrap();
    let mut surface = RenderSurface::new(size.0, size.1).unwrap();
    let tiles = surface.tiles_mut(64, 64).unwrap();
    let mut buckets: Vec<Vec<_>> = (0..threads).map(|_| Vec::new()).collect();
    for (index, tile) in tiles.into_iter().enumerate() {
        buckets[index % threads].push(tile);
    }
    let cancel = CancelToken::new();
    let start = Instant::now();
    let tracer = &tracer;
    let cancel = &cancel;
    let works: Vec<RenderWork> = std::thread::scope(|scope| {
        let handles: Vec<_> = buckets
            .into_iter()
            .map(|tiles| {
                scope.spawn(move || {
                    let mut work = RenderWork::default();
                    for mut tile in tiles {
                        work.add(&tracer.render_tile(&mut tile, 0, cancel).unwrap());
                    }
                    work
                })
            })
            .collect();
        handles
            .into_iter()
            .map(|handle| handle.join().unwrap())
            .collect()
    });
    let seconds = start.elapsed().as_secs_f64();
    let mut work = RenderWork::default();
    for each in &works {
        work.add(each);
    }
    (work, seconds)
}

fn report(label: &str, work: &RenderWork, seconds: f64) {
    let total = work.total_rays() as f64;
    println!(
        "{label}: {total} rays in {seconds:.3}s = {:.0} rays/s; camera {} first-shadow {} later-shadow {} bounce {}; rays per sample {:.2}; coherent {:.3}; samples/s {:.1}",
        total / seconds,
        work.camera_rays,
        work.first_hit_shadow_rays,
        work.later_shadow_rays,
        work.bounce_rays,
        total / work.camera_rays as f64,
        work.coherent_fraction(),
        work.pixels_rendered as f64 / seconds,
    );
}

#[test]
#[ignore = "reports timings; run in a release build"]
fn measure_rays_and_samples_per_second() {
    let full_paths = PathTraceSettings::default();
    let camera_only = PathTraceSettings {
        max_bounces: 0,
        ..PathTraceSettings::default()
    };
    let scene = scene_with(224, 224);
    let (work, seconds) = timed_pass(&scene, (1920, 1080), full_paths, 1);
    report("100k 1920x1080 full paths, 1 thread", &work, seconds);
    let (work, seconds) = timed_pass(&scene, (1920, 1080), full_paths, 8);
    report("100k 1920x1080 full paths, 8 threads", &work, seconds);
    let (work, seconds) = timed_pass(&scene, (1920, 1080), camera_only, 1);
    report("100k camera rays only, 1 thread", &work, seconds);
    let (work, seconds) = timed_pass(&scene, (1920, 1080), camera_only, 8);
    report("100k camera rays only, 8 threads", &work, seconds);

    let scene = scene_with(707, 707);
    let (work, seconds) = timed_pass(&scene, (1920, 1080), full_paths, 8);
    report("1M 1920x1080 full paths, 8 threads", &work, seconds);
    let (work, seconds) = timed_pass(&scene, (1920, 1080), full_paths, 1);
    report("1M 1920x1080 full paths, 1 thread", &work, seconds);
}

/// `first` with `second` appended, one mesh: the measurement only needs a scene with a part and
/// a floor, and one tree over both is what a merged mesh gives.
fn merged(first: solidmesh_mesh::Mesh, second: solidmesh_mesh::Mesh) -> solidmesh_mesh::Mesh {
    let mut vertices = Vec::new();
    let mut faces = Vec::new();
    let mut offset = 0;
    for mesh in [&first, &second] {
        for vertex in &mesh.vertices {
            vertices.extend_from_slice(&[
                f64::from(vertex.x),
                f64::from(vertex.y),
                f64::from(vertex.z),
            ]);
        }
        for face in mesh.faces.iter() {
            faces.extend_from_slice(&[face.v0 + offset, face.v1 + offset, face.v2 + offset]);
        }
        offset += mesh.vertices.len() as i32;
    }
    solidmesh_mesh::Mesh::from_arrays(&vertices, &faces)
}
