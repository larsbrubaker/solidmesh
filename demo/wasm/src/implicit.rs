//! The Implicit / SDF page: solids described by signed distance functions and meshed by the
//! library's two meshers - marching cubes over a sampled grid (`sdf::extract` on a `DenseGrid3f`)
//! or octree dual contouring (`dual_contouring::build_octree` and `generate_mesh_from_octree`).
//!
//! The primitives are the library's (`dual_contouring::density`: `Sphere`, `Box`, `Cylinder`,
//! placed by `Transform`). The library's own `Union` / `Subtraction` are sharp (min / max); the
//! smooth blend here is the polynomial smooth minimum, a few lines of demo code over those
//! fields, so the page can show a fillet-like blend growing with its radius.
//!
//! The third scene is the boolean's implicit processing mode on meshes
//! (`BooleanOptions::processing_mode`): two mesh operands resampled as distance grids, combined
//! and meshed again, which never fails whatever the operands.

use polygon_mesh::csg::boolean_processing::do_array;
use polygon_mesh::csg::{BooleanOptions, CsgModes, ProcessingModes, ProcessingResolution};
use polygon_mesh::dual_contouring::{
    build_octree, generate_mesh_from_octree, Box as SdfBox, Cylinder, Sdf, Sphere, Transform,
};
use polygon_mesh::sdf::{extract, DenseGrid3f};
use polygon_mesh::{CancelToken, Mesh};
use vector_math::{AxisAlignedBoundingBox, Matrix4X4, Vector3};

use crate::shapes::Shape;
use crate::Scene;

/// The polynomial smooth minimum of two distances: `min(a, b)` rounded over a band `k` wide.
pub fn smooth_min(a: f64, b: f64, k: f64) -> f64 {
    if k <= 0.0 {
        return a.min(b);
    }
    let h = (k - (a - b).abs()).max(0.0) / k;
    a.min(b) - h * h * k * 0.25
}

/// The smooth maximum, for intersections and subtractions.
pub fn smooth_max(a: f64, b: f64, k: f64) -> f64 {
    -smooth_min(-a, -b, k)
}

/// A field and the box it lives in.
pub struct Field {
    pub f: std::boxed::Box<dyn Fn(Vector3) -> f64>,
    pub bounds: AxisAlignedBoundingBox,
}

fn at(sdf: impl Sdf + 'static, x: f64, y: f64, z: f64) -> Transform {
    Transform::new(
        std::boxed::Box::new(sdf),
        Matrix4X4::create_translation(x, y, z),
    )
}

/// Scene 0: a sphere, a box and a cylinder smooth-unioned into one blob.
pub fn blob(blend: f64) -> Field {
    let sphere = at(Sphere { radius: 9.0 }, -6.0, 0.0, 4.0);
    let block = at(SdfBox::new(Vector3::new(22.0, 14.0, 8.0)), 4.0, 0.0, -4.0);
    let post = at(Cylinder::new(26.0, 4.0), 10.0, 0.0, 6.0);
    let bounds = [sphere.bounds(), block.bounds(), post.bounds()]
        .iter()
        .fold(AxisAlignedBoundingBox::empty(), |b, c| {
            AxisAlignedBoundingBox::union(&b, c)
        });
    let f = move |p: Vector3| {
        let d = smooth_min(sphere.sdf(p), block.sdf(p), blend);
        smooth_min(d, post.sdf(p), blend)
    };
    Field {
        f: std::boxed::Box::new(f),
        bounds,
    }
}

/// Scene 1: a plate with a smooth-blended boss and a hole drilled through both, its edges
/// softened by the same blend.
pub fn drilled_boss(blend: f64) -> Field {
    let plate = at(SdfBox::new(Vector3::new(36.0, 24.0, 6.0)), 0.0, 0.0, 0.0);
    let boss = at(Cylinder::new(14.0, 8.0), 0.0, 0.0, 6.0);
    let hole = at(Cylinder::new(40.0, 4.0), 0.0, 0.0, 0.0);
    let slot = at(SdfBox::new(Vector3::new(8.0, 30.0, 10.0)), 14.0, 0.0, 0.0);
    let bounds = AxisAlignedBoundingBox::union(&plate.bounds(), &boss.bounds());
    let f = move |p: Vector3| {
        let body = smooth_min(plate.sdf(p), boss.sdf(p), blend);
        let cut = smooth_max(body, -hole.sdf(p), blend * 0.5);
        smooth_max(cut, -slot.sdf(p), blend * 0.5)
    };
    Field {
        f: std::boxed::Box::new(f),
        bounds,
    }
}

/// The field meshed with marching cubes over a grid `cells` across its padded bounds.
pub fn mesh_marching_cubes(field: &Field, cells: u32, pad: f64) -> Result<Mesh, String> {
    let min = field.bounds.min_xyz - Vector3::new(pad, pad, pad);
    let size = field.bounds.size() + Vector3::new(2.0 * pad, 2.0 * pad, 2.0 * pad);
    let cell = size.x.max(size.y).max(size.z) / f64::from(cells.max(8));
    let count = |extent: f64| (extent / cell).ceil() as usize + 1;
    let (ni, nj, nk) = (count(size.x), count(size.y), count(size.z));
    let mut grid = DenseGrid3f::new(ni, nj, nk, 0.0);
    for k in 0..nk {
        for j in 0..nj {
            for i in 0..ni {
                let p = min + Vector3::new(i as f64 * cell, j as f64 * cell, k as f64 * cell);
                grid.buffer[i + ni * (j + nj * k)] = (field.f)(p) as f32;
            }
        }
    }
    extract(&grid, min, cell, 0.0, &CancelToken::new())
        .ok_or_else(|| "Marching cubes was cancelled.".to_string())
}

/// The field meshed with octree dual contouring at `2^level` finest cells across.
pub fn mesh_dual_contouring(field: &Field, level: i32, pad: f64) -> Mesh {
    let size = field.bounds.size() + Vector3::new(2.0 * pad, 2.0 * pad, 2.0 * pad);
    let side = size.x.max(size.y).max(size.z);
    let center = field.bounds.center();
    let min = center - Vector3::new(side / 2.0, side / 2.0, side / 2.0);
    let f = |p: Vector3| (field.f)(p);
    let root = build_octree(&f, min, Vector3::new(side, side, side), level, 1e-4);
    generate_mesh_from_octree(&root)
}

/// The cells across the largest side a slider value asks for, kept to 16..=192: a finer grid costs
/// the cube of its size in samples.
fn cells_for(resolution: u32) -> u32 {
    resolution.clamp(16, 192)
}

/// The implicit scene. `scene` 0 blob, 1 drilled boss (both SDFs smooth-blended by `blend` mm),
/// 2 the implicit boolean of a cube and a sphere mesh. `mesher` 0 marching cubes, 1 dual
/// contouring; `resolution` cells across the largest side.
pub fn implicit_scene(
    scene: u32,
    blend: f64,
    resolution: u32,
    mesher: u32,
) -> Result<Scene, String> {
    let cells = cells_for(resolution);
    if scene == 2 {
        return implicit_mesh_boolean(cells, mesher);
    }
    let field = match scene {
        0 => blob(blend),
        1 => drilled_boss(blend),
        other => return Err(format!("There is no implicit scene {other}.")),
    };
    let pad = 2.0 + blend;
    let mesh = match mesher {
        0 => mesh_marching_cubes(&field, cells, pad)?,
        _ => {
            // Dual contouring's lattice is a power of two across.
            let level = (f64::from(cells).log2().round() as i32).clamp(4, 7);
            mesh_dual_contouring(&field, level, pad)
        }
    };
    let how = if mesher == 0 {
        format!("marching cubes, {cells} cells")
    } else {
        "dual contouring".to_string()
    };
    Ok(Scene::new(
        mesh,
        format!("Smooth blend {blend:.1} mm, {how}."),
    ))
}

/// A cube minus a sphere through the n-ary boolean's implicit processing mode (the two-operand
/// spelling always meshes with marching cubes; `do_array` honours dual contouring too).
fn implicit_mesh_boolean(cells: u32, mesher: u32) -> Result<Scene, String> {
    let cube = Shape::Cube.build(24.0, 32)?;
    let sphere = Shape::Sphere.build(30.0, 48)?;
    let resolution = if cells > 96 {
        ProcessingResolution::R128
    } else {
        ProcessingResolution::R64
    };
    let options = BooleanOptions {
        processing_mode: if mesher == 0 {
            ProcessingModes::MarchingCubes
        } else {
            ProcessingModes::DualContouring
        },
        input_resolution: resolution,
        output_resolution: resolution,
        ..BooleanOptions::default()
    };
    let lift = Matrix4X4::create_translation(6.0, 6.0, 6.0);
    let mesh = do_array(
        &[(&cube, Matrix4X4::IDENTITY), (&sphere, lift)],
        CsgModes::Subtract,
        &CancelToken::new(),
        &options,
    )
    .map_err(|e| format!("The implicit boolean failed: {e}"))?;
    let mut ghost = cube.copy();
    ghost.copy_all_faces(&sphere, lift);
    Ok(Scene {
        mesh,
        ghost: Some(ghost),
        note: format!(
            "Cube minus sphere as implicit grids ({} cells).",
            resolution.cells()
        ),
    })
}
