//! The implicit-surface processing modes of the boolean (the n-ary implicit boolean, the implicit
//! half of the two-operand boolean, and the operand field): each operand becomes a narrow-band
//! signed distance grid sampled trilinearly (g3 `MeshSignedDistanceGrid` +
//! `DenseGridTrilinearImplicit`), the operands combine as min/max of their fields (g3's implicit
//! operators), and the result is meshed again - with g3's `MarchingCubesPro` (n-ary) or
//! `MarchingCubes` (two-operand), or with octree dual contouring. No kernel is involved, so the
//! answer is approximate and only checked by volume.
//!
//! The operand field is always the sampled grid: an exact fast-winding-number field would only
//! matter in `Polygons` mode, which never reaches this path.

use solidmesh_math::{Matrix4X4, Vector3};

use super::{CsgModes, ProcessingModes, ProcessingResolution};
use crate::dual_contouring::{build_octree, generate_mesh_from_octree};
use crate::g3::dmesh3::{AxisAlignedBox3d, DMesh3};
use crate::g3::implicit3d::{
    BoundedImplicitFunction3d, BoxedImplicit, DenseGridTrilinearImplicit, ImplicitDifference3d,
    ImplicitIntersection3d, ImplicitNaryDifference3d, ImplicitNaryIntersection3d,
    ImplicitNaryUnion3d, ImplicitUnion3d,
};
use crate::g3::marching_cubes::{MarchingCubes, RootfindingModes};
use crate::g3::marching_cubes_pro::MarchingCubesPro;
use crate::g3::signed_distance_grid::SignedDistanceGrid;
use crate::Mesh;

/// The mesh's narrow-band signed distance grid, `num_cells` across its largest dimension, as a
/// trilinear field.
fn implicit_function(mesh: &Mesh, num_cells: i32) -> BoxedImplicit {
    let mesh_a3 = mesh.to_dmesh3();
    let cell_size = mesh_a3.cached_bounds().max_dim() / num_cells as f64;
    let sdf = SignedDistanceGrid::compute(&mesh_a3, cell_size);
    Box::new(DenseGridTrilinearImplicit::from_sdf(sdf))
}

/// The operand transformed into place, on a copy.
fn placed(mesh: &Mesh, matrix: Matrix4X4) -> Mesh {
    let mut copy = mesh.copy();
    copy.transform(matrix);
    copy
}

/// Marching cubes over the field (the Pro variant for the n-ary boolean, the plain one for two
/// operands): cubes of `max_dim / num_cells` over the field's bounds grown by three cubes, roots
/// found by five lerp steps.
fn marching_cubes_mesh(root: &dyn BoundedImplicitFunction3d, num_cells: i32, pro: bool) -> DMesh3 {
    let mut bounds = root.bounds();
    let cube_size = bounds.max_dim() / num_cells as f64;
    // Leave a buffer of cells.
    bounds.expand(3.0 * cube_size);
    if pro {
        MarchingCubesPro {
            implicit: root,
            iso_value: 0.0,
            bounds,
            cube_size,
            root_mode: RootfindingModes::LerpSteps,
            root_mode_steps: 5,
        }
        .generate()
    } else {
        MarchingCubes {
            implicit: root,
            iso_value: 0.0,
            bounds,
            cube_size,
            root_mode: RootfindingModes::LerpSteps,
            root_mode_steps: 5,
        }
        .generate()
    }
    // No vertex normals are computed here: converting to a `Mesh` drops them anyway.
}

/// The dual contouring call of the n-ary implicit boolean: the octree over the field's bounds,
/// `level` deep, simplified at 0.001.
fn dual_contouring_mesh(root: &dyn BoundedImplicitFunction3d, level: i32) -> Mesh {
    let bounds: AxisAlignedBox3d = root.bounds();
    let min = Vector3::new(bounds.min[0], bounds.min[1], bounds.min[2]);
    let size = Vector3::new(
        bounds.max[0] - bounds.min[0],
        bounds.max[1] - bounds.min[1],
        bounds.max[2] - bounds.min[2],
    );
    let f = |pos: Vector3| root.value([pos.x, pos.y, pos.z]);
    let octree = build_octree(&f, min, size, level, 0.001);
    generate_mesh_from_octree(&octree)
}

/// The n-ary boolean in an implicit mode. An empty operand list gives an empty mesh rather than an
/// error.
pub(crate) fn as_implicit_meshes(
    items: &[(&Mesh, Matrix4X4)],
    operation: CsgModes,
    processing_mode: ProcessingModes,
    input_resolution: ProcessingResolution,
    output_resolution: ProcessingResolution,
) -> Mesh {
    if items.is_empty() {
        return Mesh::new();
    }
    let mut implicits: Vec<BoxedImplicit> = items
        .iter()
        .map(|(mesh, matrix)| implicit_function(&placed(mesh, *matrix), input_resolution.cells()))
        .collect();

    let root: BoxedImplicit = match operation {
        CsgModes::Union => Box::new(ImplicitNaryUnion3d {
            children: implicits,
        }),
        CsgModes::Subtract => {
            // The first operand with every later one removed.
            let b_set = implicits.split_off(1);
            let a = implicits.remove(0);
            Box::new(ImplicitNaryDifference3d { a, b_set })
        }
        CsgModes::Intersect => Box::new(ImplicitNaryIntersection3d {
            children: implicits,
        }),
    };

    if processing_mode == ProcessingModes::DualContouring {
        dual_contouring_mesh(root.as_ref(), output_resolution.exponent())
    } else {
        marching_cubes_mesh(root.as_ref(), output_resolution.cells(), true).to_mesh()
    }
}

/// The implicit half of the two-operand boolean: always marching cubes (`MarchingCubes`, not the
/// Pro), whatever the implicit mode. An operand with fewer than four faces is no solid, so the
/// other one is returned as placed.
pub(crate) fn two_operand_implicit(
    mesh_a: &Mesh,
    matrix_a: Matrix4X4,
    mesh_b: &Mesh,
    matrix_b: Matrix4X4,
    operation: CsgModes,
    input_resolution: ProcessingResolution,
    output_resolution: ProcessingResolution,
) -> Mesh {
    let mesh_a = placed(mesh_a, matrix_a);
    let mesh_b = placed(mesh_b, matrix_b);
    if mesh_a.faces.len() < 4 {
        return mesh_b;
    } else if mesh_b.faces.len() < 4 {
        return mesh_a;
    }

    let a = implicit_function(&mesh_a, input_resolution.cells());
    let b = implicit_function(&mesh_b, input_resolution.cells());
    let root: BoxedImplicit = match operation {
        CsgModes::Union => Box::new(ImplicitUnion3d { a, b }),
        CsgModes::Subtract => Box::new(ImplicitDifference3d { a, b }),
        CsgModes::Intersect => Box::new(ImplicitIntersection3d { a, b }),
    };
    marching_cubes_mesh(root.as_ref(), output_resolution.cells(), false).to_mesh()
}
