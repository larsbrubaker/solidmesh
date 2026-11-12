//! What finding where glass parts touch costs: for a scene of 96 glass walls standing apart, the
//! same walls touching end to end as a floor plan's do, two glass slabs of 50,000 triangles each
//! lying face on face, and a glass slab 200 and 2000 across under a fine 10 mm glass sheet in
//! either order, how long building the scene takes and how long a render does; and for one glass
//! part of 720,000 triangles, building its scene as glass against as matte. Ignored;
//! run in a release build with `--nocapture` to read the numbers.

use std::time::Instant;

use solidmesh_math::{Matrix4X4, Vector3};
use solidmesh_mesh::ray_tracer::{
    PathTraceSettings, PathTracer, PlacedParts, RenderCamera, RenderScene, RenderSurface,
    ScatteringSurface,
};
use solidmesh_mesh::{CancelToken, Mesh};

use crate::glass_tray_support::{box_meshes, part_of, rig_lights};

/// 96 walls 10 long, 2 thick and 10 tall in 12 rows of 8, `touching` end to end along each row,
/// or with a gap of 4 between them.
pub(crate) fn walls(touching: bool) -> Vec<Mesh> {
    let step = if touching { 10.0 } else { 14.0 };
    let boxes: Vec<_> = (0..12)
        .flat_map(|row| {
            (0..8).map(move |at| {
                (
                    [10.0, 2.0, 10.0],
                    [f64::from(at) * step, f64::from(row) * 12.0, 5.0],
                )
            })
        })
        .collect();
    box_meshes(&boxes)
}

/// A slab 100 square and 2 thick from `z`, its top and bottom each a grid of `cells` by `cells`
/// squares shifted by `shift` of a cell, closed by four side quads.
pub(crate) fn grid_slab(z: f64, cells: u32, shift: f64) -> Mesh {
    let mut vertices = Vec::new();
    let mut faces = Vec::new();
    let size = 100.0;
    let cell = size / f64::from(cells);
    for (height, up) in [(z + 2.0, true), (z, false)] {
        let base = u32::try_from(vertices.len() / 3).unwrap();
        for j in 0..=cells {
            for i in 0..=cells {
                let edge = |k: u32| {
                    if k == 0 || k == cells {
                        f64::from(k) * cell
                    } else {
                        (f64::from(k) + shift) * cell
                    }
                };
                vertices.extend([edge(i), edge(j), height]);
            }
        }
        let at = |i: u32, j: u32| base + j * (cells + 1) + i;
        for j in 0..cells {
            for i in 0..cells {
                let (a, b, c, d) = (at(i, j), at(i + 1, j), at(i + 1, j + 1), at(i, j + 1));
                if up {
                    faces.extend([a, b, c, a, c, d]);
                } else {
                    faces.extend([a, c, b, a, d, c]);
                }
            }
        }
    }
    let corners = [(0.0, 0.0), (size, 0.0), (size, size), (0.0, size)];
    let base = u32::try_from(vertices.len() / 3).unwrap();
    for (x, y) in corners {
        vertices.extend([x, y, z, x, y, z + 2.0]);
    }
    for side in 0..4 {
        let (low, high) = (base + side * 2, base + (side + 1) % 4 * 2);
        faces.extend([low, high, high + 1, low, high + 1, low + 1]);
    }
    let faces: Vec<i32> = faces.into_iter().map(|f| f as i32).collect();
    Mesh::from_arrays(&vertices, &faces)
}

fn scene_of(meshes: &[Mesh]) -> RenderScene {
    let parts = meshes
        .iter()
        .map(|mesh| part_of(mesh, Matrix4X4::IDENTITY))
        .collect();
    RenderScene::new(
        PlacedParts::new(parts),
        vec![ScatteringSurface::glass(); meshes.len()],
        rig_lights(),
    )
    .unwrap()
}

/// Seconds to build the scene (best of three) and to render it 160 by 120 at 8 samples.
fn timed(meshes: &[Mesh], eye: Vector3, target: Vector3) -> (f64, f64) {
    let mut build = f64::INFINITY;
    let mut scene = None;
    for _ in 0..3 {
        let start = Instant::now();
        let built = scene_of(meshes);
        build = build.min(start.elapsed().as_secs_f64());
        scene = Some(built);
    }
    let scene = scene.unwrap();
    let camera = RenderCamera::perspective(eye, target, Vector3::UNIT_Z, 40.0).unwrap();
    let tracer = PathTracer::new(&scene, camera, 160, 120, PathTraceSettings::default()).unwrap();
    let mut surface = RenderSurface::new(160, 120).unwrap();
    let start = Instant::now();
    for sample in 0..8 {
        tracer
            .render_pass(&mut surface, sample, &CancelToken::new())
            .unwrap();
    }
    (build, start.elapsed().as_secs_f64())
}

/// Seconds to build a scene of `meshes` placed by `matrices`, all of `surface`, best of three.
fn build_seconds(meshes: &[(Mesh, Matrix4X4)], surface: ScatteringSurface) -> f64 {
    let mut best = f64::INFINITY;
    for _ in 0..3 {
        let start = Instant::now();
        let parts = meshes
            .iter()
            .map(|(mesh, matrix)| part_of(mesh, *matrix))
            .collect();
        let scene = RenderScene::new(
            PlacedParts::new(parts),
            vec![surface; meshes.len()],
            rig_lights(),
        )
        .unwrap();
        best = best.min(start.elapsed().as_secs_f64());
        drop(scene);
    }
    best
}

#[test]
#[ignore = "reports what finding touching glass costs; run in a release build"]
fn measure_glass_contact_build_cost() {
    let sheet = (
        grid_slab(0.0, 70, 0.3),
        Matrix4X4::mult(
            Matrix4X4::create_scale(0.1, 0.1, 0.5),
            Matrix4X4::create_translation(-5.0, -5.0, 0.0),
        ),
    );
    for half in [100.0, 1000.0] {
        let mut slab =
            solidmesh_mesh::solids::platonic_solids::create_cube(2.0 * half, 2.0 * half, 2.0);
        slab.translate_xyz(0.0, 0.0, -1.0);
        let slab = (slab, Matrix4X4::IDENTITY);
        for (order, meshes) in [
            ("slab first", vec![slab.clone(), sheet.clone()]),
            ("sheet first", vec![sheet.clone(), slab.clone()]),
        ] {
            let glass = build_seconds(&meshes, ScatteringSurface::glass());
            let matte = build_seconds(
                &meshes,
                ScatteringSurface::new([0.5; 3], 0.0, 1.0, 0.0, 1.0),
            );
            println!(
                "slab half {half}, {order}: build as glass {glass:.4} s, as matte {matte:.4} s"
            );
        }
    }
    let large = vec![(grid_slab(0.0, 424, 0.0), Matrix4X4::IDENTITY)];
    let glass = build_seconds(&large, ScatteringSurface::glass());
    let matte = build_seconds(&large, ScatteringSurface::new([0.5; 3], 0.0, 1.0, 0.0, 1.0));
    println!("one part of 719,352 triangles: build as glass {glass:.4} s, as matte {matte:.4} s");
}

#[test]
#[ignore = "reports what finding touching glass costs; run in a release build"]
fn measure_glass_contact_cost() {
    let eye = Vector3::new(50.0, -120.0, 120.0);
    let target = Vector3::new(50.0, 66.0, 0.0);
    for (name, meshes) in [
        ("96 walls apart", walls(false)),
        ("96 walls touching", walls(true)),
    ] {
        let (build, render) = timed(&meshes, eye, target);
        println!("{name}: build {:.4} s, render {:.3} s", build, render);
    }
    let slabs = vec![grid_slab(0.0, 112, 0.0), grid_slab(2.0, 112, 0.5)];
    // Each slab is two grids of 2 * 112 * 112 triangles and 8 side triangles.
    let triangles = 2 * (2 * 2 * 112 * 112 + 8);
    let (build, render) = timed(
        &slabs,
        Vector3::new(50.0, -80.0, 90.0),
        Vector3::new(50.0, 50.0, 2.0),
    );
    println!(
        "two slabs face on face, {triangles} triangles: build {:.4} s, render {:.3} s",
        build, render
    );
}
