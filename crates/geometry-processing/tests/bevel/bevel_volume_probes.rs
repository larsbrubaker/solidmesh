//! Ports `GeometryProcessingTests/BevelVolumeProbes.cs`: a boolean-based "is this point inside the
//! solid" probe for kernel-side bevel tests. Like the C# (which copies MatterCADTests'
//! `BevelCornerPatchProbes` so the kernel tests need no application), it is pure polygon-mesh.

use polygon_mesh::csg::boolean_processing::do_array;
use polygon_mesh::csg::{BooleanOptions, CsgModes};
use polygon_mesh::solids::platonic_solids;
use polygon_mesh::{CancelToken, Mesh};
use vector_math::{Matrix4X4, Vector3};

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

/// The volume of `result` inside `probe` (C#'s `Polygons` mode at its default resolutions).
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
