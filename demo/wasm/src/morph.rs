//! The Minkowski & Hull page: `csg::minkowski_morph`'s sum (dilation: every point of the solid
//! swept by the tool) and difference (erosion: where the tool still fits inside), and
//! `Mesh::get_convex_hull`.
//!
//! The hull's sources are meshes made of several separate shells copied into one mesh with no
//! boolean (`Mesh::copy_all_faces`): a scattering of small octahedra (a point cloud with
//! volume), a few spheres and boxes, or one star prism.

use solidmesh::math::{Matrix4X4, Vector2};
use solidmesh::mesh::csg::minkowski_morph::{minkowski_difference, minkowski_sum};
use solidmesh::mesh::csg::minkowski_processing::sphere_mesh;
use solidmesh::mesh::processors::outline_to_mesh::extrude_contours;
use solidmesh::mesh::solids::platonic_solids::create_octahedron;
use solidmesh::mesh::Mesh;

use crate::shapes::{circle, prism, star, Lcg, Shape};
use crate::Scene;

/// The solids the Minkowski page offers: 0 star prism, 1 L bracket, 2 cube, 3 hex nut. They are
/// kept to tens of faces: in the browser the kernel has one thread, and a sum costs about one
/// tool hull per solid face.
fn solid(index: u32) -> Result<Mesh, String> {
    match index {
        0 => Shape::Star.build(30.0, 32),
        1 => {
            let l = [
                Vector2::new(0.0, 0.0),
                Vector2::new(28.0, 0.0),
                Vector2::new(28.0, 8.0),
                Vector2::new(8.0, 8.0),
                Vector2::new(8.0, 24.0),
                Vector2::new(0.0, 24.0),
            ];
            prism(&l, 10.0)
        }
        2 => Shape::Cube.build(20.0, 32),
        3 => {
            let hex = circle(Vector2::new(0.0, 0.0), 14.0, 6);
            let hole: Vec<Vector2> = circle(Vector2::new(0.0, 0.0), 7.0, 12)
                .into_iter()
                .rev()
                .collect();
            extrude_contours(&[hex, hole], 10.0).map_err(|e| format!("extrude failed: {e}"))
        }
        other => Err(format!("There is no Minkowski solid {other}.")),
    }
}

/// The tools: 0 the kernel's ball (`minkowski_processing::sphere_mesh`, `segments` around), 1 cube,
/// 2 octahedron, each `size` mm across.
fn tool(index: u32, size: f64, segments: u32) -> Result<Mesh, String> {
    match index {
        0 => sphere_mesh(size / 2.0, segments.clamp(4, 32) as i32)
            .map_err(|e| format!("The ball failed: {e}")),
        1 => Shape::Cube.build(size, 8),
        2 => Ok(crate::shapes::centered(create_octahedron(size / 2.0))),
        other => Err(format!("There is no Minkowski tool {other}.")),
    }
}

/// The solid dilated (or, with `erode`, eroded) by the tool; the solid is the ghost. `segments` is
/// the ball's detail (other tools ignore it).
pub fn minkowski_scene(
    solid_index: u32,
    tool_index: u32,
    size: f64,
    erode: bool,
    segments: u32,
) -> Result<Scene, String> {
    let solid = solid(solid_index)?;
    let tool = tool(tool_index, size.max(0.5), segments)?;
    let mesh = if erode {
        minkowski_difference(&solid, &tool)
    } else {
        minkowski_sum(&solid, &tool)
    }
    .map_err(|e| format!("The Minkowski operation failed: {e}"))?;
    let what = if erode { "difference" } else { "sum" };
    Ok(Scene {
        mesh,
        ghost: Some(solid),
        note: format!("Minkowski {what} with a {size:.1} mm tool."),
    })
}

/// Shells scattered in one mesh: `count` small octahedra in a 30 mm ball (source 0), a few
/// spheres and cubes (source 1) or a star prism (source 2).
fn hull_source(source: u32, count: u32, seed: u32) -> Result<Mesh, String> {
    let mut rng = Lcg::new(seed);
    let mut mesh = Mesh::new();
    match source {
        0 => {
            let dot = create_octahedron(0.8);
            for _ in 0..count.clamp(4, 400) {
                // Rejection sampling keeps the cloud a ball rather than a box.
                let p = loop {
                    let v = [rng.next_f64(), rng.next_f64(), rng.next_f64()].map(|t| 2.0 * t - 1.0);
                    if v[0] * v[0] + v[1] * v[1] + v[2] * v[2] <= 1.0 {
                        break v;
                    }
                };
                let at = Matrix4X4::create_translation(p[0] * 15.0, p[1] * 15.0, p[2] * 10.0);
                mesh.copy_all_faces(&dot, at);
            }
        }
        1 => {
            let sphere = Shape::Sphere.build(10.0, 24)?;
            let cube = Shape::Cube.build(8.0, 8)?;
            for i in 0..count.clamp(2, 12) {
                let p = [rng.next_f64(), rng.next_f64(), rng.next_f64()].map(|t| 30.0 * t - 15.0);
                let piece = if i % 2 == 0 { &sphere } else { &cube };
                let at = Matrix4X4::create_rotation_z(rng.next_f64() * 3.0)
                    * Matrix4X4::create_translation(p[0], p[1], p[2] * 0.5);
                mesh.copy_all_faces(piece, at);
            }
        }
        2 => {
            let outer = 15.0;
            mesh = prism(&star(count.clamp(3, 12), outer, outer * 0.4), 10.0)?;
        }
        other => return Err(format!("There is no hull source {other}.")),
    }
    Ok(mesh)
}

/// The convex hull of the source; the source is the ghost.
pub fn hull_scene(source: u32, count: u32, seed: u32) -> Result<Scene, String> {
    let shells = hull_source(source, count, seed)?;
    let hull = shells
        .get_convex_hull(false)
        .ok_or_else(|| "The hull could not be built (fewer than four faces).".to_string())?
        .mesh()
        .copy();
    let note = format!(
        "Hull of {} source triangles: {} hull triangles.",
        shells.faces.len(),
        hull.faces.len()
    );
    Ok(Scene {
        mesh: hull,
        ghost: Some(shells),
        note,
    })
}
