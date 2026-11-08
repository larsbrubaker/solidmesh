//! A boolean-based "is this point inside the solid" probe for kernel-side bevel tests. It is pure
//! polygon-mesh, so the kernel tests need no application.

use solidmesh_math::{Matrix4X4, Vector3};
use solidmesh_mesh::csg::boolean_processing::do_array;
use solidmesh_mesh::csg::{BooleanOptions, CsgModes};
use solidmesh_mesh::solids::platonic_solids;
use solidmesh_mesh::{CancelToken, Mesh};

use crate::mesh_measurements::signed_volume;

/// A small cube about a point, for asking whether that point is inside a solid: intersect it and
/// read the volume back, which is `side^3` where the point is well inside and zero where it is
/// well outside. Deliberately not a ray cast - the solids these are asked about are the tools
/// themselves, and a boolean is what every other measurement here is taken with.
pub fn probe_cube(at: Vector3, side: f64) -> Mesh {
    let mut cube = platonic_solids::create_cube(side, side, side);

    cube.translate(at);

    cube
}

/// The volume of `result` inside `probe` (the polygon boolean at its default resolutions).
pub fn volume_inside(result: &Mesh, probe: &Mesh) -> f64 {
    let intersection = do_array(
        &[(result, Matrix4X4::IDENTITY), (probe, Matrix4X4::IDENTITY)],
        CsgModes::Intersect,
        &CancelToken::new(),
        &BooleanOptions::default(),
    );

    match intersection {
        Ok(mesh) if !mesh.vertices.is_empty() => signed_volume(&mesh),
        _ => 0.0,
    }
}
