//! A picture on a face where two parts of the same glass touch: the two faces lying on each other
//! are one boundary, so the seam must read the same whichever of them a trace returns. Putting
//! the picture on the lower box's top or on the upper box's bottom, and listing the boxes in
//! either order, renders the same bits, and not those of the same boxes with no picture.

use std::sync::Arc;

use solidmesh_math::{Matrix4X4, Vector2Float, Vector3};
use solidmesh_mesh::ray_tracer::{
    Environment, LightSetup, PathTraceSettings, PathTracer, PlacedParts, RenderCamera, RenderScene,
    RenderWork, ScatteringSurface, Sun, TraceTexture,
};
use solidmesh_mesh::{FaceTextureData, Mesh};

use crate::glass_tray_support::{box_meshes, part_of};

/// Which side of the shared plane carries the picture.
#[derive(Clone, Copy)]
enum Pictured {
    LowerTop,
    UpperBottom,
    Neither,
}

/// The faces of `mesh` lying in the plane z = `z`.
pub(super) fn faces_at(mesh: &Mesh, z: f64) -> Vec<usize> {
    (0..mesh.faces.len())
        .filter(|&face| {
            let f = &mesh.faces[face];
            [f.v0, f.v1, f.v2]
                .iter()
                .all(|&vertex| (f64::from(mesh.vertices[vertex as usize].z) - z).abs() < 1e-9)
        })
        .collect()
}

/// Two 2 unit glass boxes stacked on the plane z = 1 (the lower one first unless `upper_first`),
/// one picture, dark red, on the side `pictured` names (none for [`Pictured::Neither`]).
fn stacked_glass(pictured: Pictured, upper_first: bool) -> RenderScene {
    let mut meshes = box_meshes(&[([2.0; 3], [0.0, 0.0, 0.0]), ([2.0; 3], [0.0, 0.0, 2.0])]);
    let (box_index, z) = match pictured {
        Pictured::LowerTop => (0, 1.0),
        Pictured::UpperBottom => (1, 1.0),
        Pictured::Neither => (0, f64::NAN),
    };
    let faces = faces_at(&meshes[box_index], z);
    if !matches!(pictured, Pictured::Neither) {
        assert_eq!(faces.len(), 2, "a box side is two triangles");
    }
    let mid = Vector2Float::new(0.5, 0.5);
    for &face in &faces {
        meshes[box_index]
            .face_textures
            .insert(face as i32, FaceTextureData::new(None, mid, mid, mid));
    }
    let mut assignment: Vec<Vec<Option<u32>>> = vec![Vec::new(), Vec::new()];
    assignment[box_index] = (0..meshes[box_index].faces.len())
        .map(|face| faces.contains(&face).then_some(0))
        .collect();
    if upper_first {
        meshes.reverse();
        assignment.reverse();
    }
    let parts = meshes
        .iter()
        .map(|mesh| part_of(mesh, Matrix4X4::IDENTITY))
        .collect();
    let lights = LightSetup {
        environment: Environment::constant([0.5, 0.55, 0.6]).unwrap(),
        sun: Some(Sun::new(Vector3::new(0.2, 0.1, 1.0), 0.05, [3.0; 3]).unwrap()),
    };
    let glass = ScatteringSurface::glass();
    let picture = TraceTexture::new(1, 1, vec![150, 20, 20, 255]).unwrap();
    let scene = RenderScene::new(PlacedParts::new(parts), vec![glass; 2], lights).unwrap();
    if matches!(pictured, Pictured::Neither) {
        return scene;
    }
    scene
        .with_face_textures(vec![Arc::new(picture)], assignment)
        .unwrap()
}

/// Every sample's bits of a small render looking down through both boxes.
fn render_bits(scene: &RenderScene) -> Vec<[u64; 3]> {
    let camera = RenderCamera::perspective(
        Vector3::new(0.6, -0.4, 9.0),
        Vector3::new(0.0, 0.0, 0.0),
        Vector3::UNIT_Y,
        20.0,
    )
    .unwrap();
    let tracer = PathTracer::new(scene, camera, 6, 6, PathTraceSettings::default()).unwrap();
    let mut work = RenderWork::default();
    let mut bits = Vec::new();
    for y in 0..6 {
        for x in 0..6 {
            for sample in 0..8 {
                let value = tracer.sample_pixel(x, y, sample, &mut work).unwrap();
                bits.push(value.map(f64::to_bits));
            }
        }
    }
    bits
}

/// The picture on either face of the seam, with the boxes listed in either order, renders the
/// same bits, which differ from the same boxes' with no picture: the seam reads both faces'
/// pictures whichever face the trace returned.
#[test]
fn a_pictured_face_on_a_same_glass_seam_reads_the_same_from_either_face() {
    for upper_first in [false, true] {
        let plain = render_bits(&stacked_glass(Pictured::Neither, upper_first));
        let lower = render_bits(&stacked_glass(Pictured::LowerTop, upper_first));
        let upper = render_bits(&stacked_glass(Pictured::UpperBottom, upper_first));
        assert_eq!(
            lower, upper,
            "upper box listed first: {upper_first}: the render depends on which face holds the picture"
        );
        assert_ne!(
            lower, plain,
            "upper box listed first: {upper_first}: the seam ignores the picture"
        );
    }
}
