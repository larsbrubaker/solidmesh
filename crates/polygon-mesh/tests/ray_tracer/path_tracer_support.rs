//! Shared fixtures for the path tracer tests: meshes with known surfaces wrapped into render
//! scenes, the seeded sample loop that returns a mean and a measured standard error, and the
//! tolerance the assertions read.

use std::f64::consts::PI;
use std::sync::Arc;

use solidmesh_math::{Matrix4X4, Vector3};
use solidmesh_mesh::ray_tracer::{
    LightSetup, MeshSurface, PathTraceSettings, PathTracer, PlacedPart, PlacedParts, RenderCamera,
    RenderScene, RenderWork, ScatteringSurface,
};
use solidmesh_mesh::Mesh;

use crate::scattering_support::Stats;
use crate::surface_hit_support::{sphere, tree_of};

/// A crease wide enough that a fine sphere is smooth everywhere.
pub(crate) const SMOOTH_CREASE: f64 = 89.0 * PI / 180.0;

/// A flat crease so a box's faces stay sharp.
pub(crate) const SHARP_CREASE: f64 = 10.0 * PI / 180.0;

/// `surface` on one part, `mesh` placed by `matrix`.
pub(crate) fn scene_of(
    mesh: &Mesh,
    crease: f64,
    matrix: Matrix4X4,
    surface: ScatteringSurface,
    lights: LightSetup,
) -> RenderScene {
    let part = PlacedPart::new(
        tree_of(mesh),
        Arc::new(MeshSurface::new(mesh, crease).unwrap()),
        matrix,
    )
    .unwrap();
    RenderScene::new(PlacedParts::new(vec![part]), vec![surface], lights).unwrap()
}

/// A latitude-longitude sphere at the origin, fine enough that a shading normal and its face
/// differ by about a degree.
pub(crate) fn fine_sphere(radius: f64) -> Mesh {
    sphere(radius, Vector3::ZERO, 160, 80)
}

/// A square of half side `half` at height `z`, facing +Z.
pub(crate) fn flat_patch(half: f64, z: f64) -> Mesh {
    let vertices = [
        -half, -half, z, half, -half, z, half, half, z, -half, half, z,
    ];
    Mesh::from_arrays(&vertices, &[0, 1, 2, 0, 2, 3])
}

/// An orthographic camera looking straight down, framing a 2 unit square at the origin.
pub(crate) fn top_down_camera(height: f64, extent: f64) -> RenderCamera {
    RenderCamera::orthographic(
        Vector3::new(0.0, 0.0, height),
        Vector3::ZERO,
        Vector3::UNIT_Y,
        extent,
    )
    .unwrap()
}

/// `samples` per pixel of a `width` by `height` image: each channel's mean and standard error,
/// over every sample of every pixel, and the work counted.
pub(crate) fn estimate(
    scene: &RenderScene,
    camera: RenderCamera,
    size: (u32, u32),
    settings: PathTraceSettings,
    samples: u32,
) -> ([Stats; 3], RenderWork) {
    let tracer = PathTracer::new(scene, camera, size.0, size.1, settings).unwrap();
    let mut stats = [Stats::default(); 3];
    let mut work = RenderWork::default();
    for y in 0..size.1 {
        for x in 0..size.0 {
            for sample in 0..samples {
                let value = tracer.sample_pixel(x, y, sample, &mut work).unwrap();
                for (stat, channel) in stats.iter_mut().zip(value) {
                    stat.add(channel);
                }
            }
        }
    }
    (stats, work)
}

/// Five standard errors of each estimate, combined, plus an absolute margin for the rounding of
/// the arithmetic itself, so a test with almost no variance still has a nonzero window.
pub(crate) fn agreement_tolerance(a: &Stats, b: &Stats) -> f64 {
    5.0 * (a.standard_error().powi(2) + b.standard_error().powi(2)).sqrt() + 1e-9
}
