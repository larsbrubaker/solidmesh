//! Shared fixtures for the glass tray scenes: the tray as boxes that touch, or unioned into one
//! solid, MatterCAD's sun and sky rig, and a scene of glass boxes on their own, each a part.

use std::sync::Arc;

use solidmesh_math::{Matrix4X4, Vector3};
use solidmesh_mesh::csg::csg_operations::union;
use solidmesh_mesh::ray_tracer::{
    Environment, LightSetup, MeshSurface, PlacedPart, PlacedParts, RenderScene, ScatteringSurface,
    Sun,
};
use solidmesh_mesh::solids::platonic_solids::create_cube;
use solidmesh_mesh::{CancelToken, Mesh};

use crate::path_tracer_support::SHARP_CREASE;
use crate::surface_hit_support::tree_of;

/// The tray as boxes that touch and never overlap, (size, center) with the bottom at z = 0: a
/// 100 by 100 by 2 base, a rim 20 tall and 2 thick, two long interior walls and a short cross
/// wall between them.
pub(crate) fn tray_boxes() -> Vec<([f64; 3], [f64; 3])> {
    vec![
        ([100.0, 100.0, 2.0], [0.0, 0.0, 1.0]),
        ([100.0, 2.0, 20.0], [0.0, -49.0, 12.0]),
        ([100.0, 2.0, 20.0], [0.0, 49.0, 12.0]),
        ([2.0, 96.0, 20.0], [-49.0, 0.0, 12.0]),
        ([2.0, 96.0, 20.0], [49.0, 0.0, 12.0]),
        ([2.0, 96.0, 20.0], [-16.0, 0.0, 12.0]),
        ([2.0, 96.0, 20.0], [16.0, 0.0, 12.0]),
        ([30.0, 2.0, 20.0], [0.0, 0.0, 12.0]),
    ]
}

/// MatterCAD's light rig at a typical view: a 2.5 degree sun of radiance 184 above and to the
/// side, and a grey sky gradient.
pub(crate) fn rig_lights() -> LightSetup {
    LightSetup {
        environment: Environment::gradient([0.55; 3], [0.35; 3], [0.21; 3]).unwrap(),
        sun: Some(
            Sun::new(
                Vector3::new(0.5, -0.35, 0.8),
                2.5_f64.to_radians(),
                [184.0; 3],
            )
            .unwrap(),
        ),
    }
}

/// Boxes of (size, center), each its own mesh.
pub(crate) fn box_meshes(boxes: &[([f64; 3], [f64; 3])]) -> Vec<Mesh> {
    boxes
        .iter()
        .map(|&(size, center)| {
            let mut mesh = create_cube(size[0], size[1], size[2]);
            mesh.translate_xyz(center[0], center[1], center[2]);
            mesh
        })
        .collect()
}

/// `meshes` unioned into one closed solid.
pub(crate) fn unioned(meshes: &[Mesh]) -> Mesh {
    let cancel = CancelToken::new();
    let mut solid = meshes[0].clone();
    for piece in &meshes[1..] {
        solid = union(&solid, piece, &cancel).unwrap();
    }
    solid
}

/// The tray's boxes, each placed, or unioned into one closed solid when `one_solid`.
pub(crate) fn tray_meshes(one_solid: bool) -> Vec<Mesh> {
    let boxes = box_meshes(&tray_boxes());
    if one_solid {
        vec![unioned(&boxes)]
    } else {
        boxes
    }
}

/// A placed part of `mesh` with sharp creases, moved by `matrix`.
pub(crate) fn part_of(mesh: &Mesh, matrix: Matrix4X4) -> PlacedPart {
    PlacedPart::new(
        tree_of(mesh),
        Arc::new(MeshSurface::new(mesh, SHARP_CREASE).unwrap()),
        matrix,
    )
    .unwrap()
}

/// Each of `meshes` its own part of `surface`, in place, under `lights`.
pub(crate) fn parts_scene(
    meshes: &[Mesh],
    surface: ScatteringSurface,
    lights: LightSetup,
) -> RenderScene {
    let parts = meshes
        .iter()
        .map(|mesh| part_of(mesh, Matrix4X4::IDENTITY))
        .collect();
    RenderScene::new(PlacedParts::new(parts), vec![surface; meshes.len()], lights).unwrap()
}
