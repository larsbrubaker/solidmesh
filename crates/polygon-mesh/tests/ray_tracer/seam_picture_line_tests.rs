//! A seam where one face of the touching pair shows a picture, on the GPU route's mirror of the
//! line toward the sun (`sun_line_f32`, whose seam is `seam.wgsl`'s `seam_at`) against the CPU's
//! line (`RenderScene::sun_line_at`, whose seam is `glass_seam`), on two glass boxes stacked on
//! the plane z = 25 with the picture on the lower box's top or the upper box's bottom and the
//! boxes listed in either order, so the trace returns the pictured face in some runs and the face
//! it covers in others. Three seams: one glass on both sides, where the picture makes the two
//! surfaces differ and so an interface; two different glasses, whose interface is tinted by the
//! picture; and the covering part's glass against air where the hit part is not there. Each line
//! crosses and passes what the CPU's does. Last, a picture that tints its face to exactly the
//! other face's color: one glass where the two materials are the same, an interface where they
//! differ only in roughness or index.

use std::sync::Arc;

use solidmesh_math::{Color, Matrix4X4, Vector2Float, Vector3};
use solidmesh_mesh::ray_tracer::flat_tree::{
    GpuCompositing, GpuFloor, GpuRenderScene, GpuSceneLimits, GpuView,
    WEBGPU_DEFAULT_MAX_STORAGE_BINDING,
};
use solidmesh_mesh::ray_tracer::{
    PathTraceSettings, PlacedPart, PlacedParts, RenderCamera, RenderScene, ScatteringSurface,
    TraceTexture,
};
use solidmesh_mesh::FaceTextureData;
use solidmesh_mesh::Mesh;

use crate::glass_on_floor_tests::furnace;
use crate::glass_tray_support::{box_meshes, part_of};
use crate::gpu_texture_pack_tests::picture;
use crate::gpu_texture_shader_tests::{agree_pictured, PICTURED_BOUND_PER_CROSSING};
use crate::see_through_on_floor_tests::wide_floor;
use crate::sun_line_shader_tests::{both, down_from, glass_of, gpu_floor_vertex, gpu_hit, random};
use crate::texture_glass_seam_tests::faces_at;

/// Which face of the seam shows the picture.
#[derive(Debug, Clone, Copy, PartialEq)]
pub(super) enum Pictured {
    LowerTop,
    UpperBottom,
}

/// The stack: the lower box's surface and opacity, the upper's, the pictured face, and whether the
/// upper box is listed first.
#[derive(Debug, Clone, Copy)]
pub(super) struct Stack {
    pub(super) lower: (ScatteringSurface, f64),
    pub(super) upper: (ScatteringSurface, f64),
    pub(super) pictured: Pictured,
    pub(super) upper_first: bool,
}

/// Every stack of `lower` under `upper`: the picture on either face, the boxes in either order.
pub(super) fn stacks(
    lower: (ScatteringSurface, f64),
    upper: (ScatteringSurface, f64),
) -> Vec<Stack> {
    let mut all = Vec::new();
    for pictured in [Pictured::LowerTop, Pictured::UpperBottom] {
        for upper_first in [false, true] {
            all.push(Stack {
                lower,
                upper,
                pictured,
                upper_first,
            });
        }
    }
    all
}

/// The CPU scene of `stack` over the wide matte floor and its GPU pack over the analytic floor at
/// z = 0: two boxes of side 20, the lower from z = 5 to 25, the upper from 25 to 45, the pictured
/// face's coordinates running across it so the picture's factor changes along the seam.
pub(super) fn stacked(stack: Stack) -> (RenderScene, GpuRenderScene) {
    stacked_with(stack, true)
}

/// [`stacked`], with the picture given to the scene only when `pictures` (the face keeps its
/// coordinates either way).
pub(super) fn stacked_with(stack: Stack, pictures: bool) -> (RenderScene, GpuRenderScene) {
    stacked_from(stack, pictures.then(|| picture(4, 3, 31)), None)
}

/// [`stacked`] showing `texture` (none at all for `None`), the other face of the seam colored
/// `plain_color` when given.
pub(super) fn stacked_from(
    stack: Stack,
    texture: Option<TraceTexture>,
    plain_color: Option<Color>,
) -> (RenderScene, GpuRenderScene) {
    let (cpu, gpu) = stacked_scenes(stack, texture, plain_color);
    let limits = GpuSceneLimits {
        max_storage_binding: WEBGPU_DEFAULT_MAX_STORAGE_BINDING,
        max_vertex_step: 1e-3,
    };
    let pack = GpuRenderScene::pack_view(&gpu, &stack_view(), limits).unwrap();
    (cpu, pack)
}

/// The two scenes [`stacked_from`] uses: the CPU's over the wide matte floor, and the one it packs
/// (the floor being the view's).
pub(super) fn stacked_scenes(
    stack: Stack,
    texture: Option<TraceTexture>,
    plain_color: Option<Color>,
) -> (RenderScene, RenderScene) {
    let mut meshes: Vec<Mesh> =
        box_meshes(&[([20.0; 3], [0.0, 0.0, 15.0]), ([20.0; 3], [0.0, 0.0, 35.0])]);
    let box_index = match stack.pictured {
        Pictured::LowerTop => 0,
        Pictured::UpperBottom => 1,
    };
    let faces = faces_at(&meshes[box_index], 25.0);
    assert_eq!(faces.len(), 2, "a box side is two triangles");
    let mesh = &mut meshes[box_index];
    let at = |mesh: &Mesh, v: i32| {
        let p = Vector3::from_vector3_float(mesh.vertices[v as usize]);
        Vector2Float::new(0.041 * p.x + 0.37, 0.033 * p.y - 0.21)
    };
    for &face in &faces {
        let f = &mesh.faces[face];
        let data = FaceTextureData::new(None, at(mesh, f.v0), at(mesh, f.v1), at(mesh, f.v2));
        mesh.face_textures.insert(face as i32, data);
    }
    if let Some(color) = plain_color {
        let other = &mut meshes[1 - box_index];
        let plain = faces_at(other, 25.0);
        let colors = (0..other.faces.len())
            .map(|face| {
                if plain.contains(&face) {
                    color
                } else {
                    Color::rgba(255, 255, 255, 255)
                }
            })
            .collect();
        other.face_colors = Some(colors);
    }
    let mut assignment: Vec<Vec<Option<u32>>> = vec![Vec::new(), Vec::new()];
    assignment[box_index] = (0..meshes[box_index].faces.len())
        .map(|face| faces.contains(&face).then_some(0))
        .collect();
    let mut pieces = [stack.lower, stack.upper];
    if stack.upper_first {
        meshes.reverse();
        assignment.reverse();
        pieces.reverse();
    }
    let texture = texture.map(Arc::new);
    let scene_of = |with_floor: bool| {
        let mut placed: Vec<PlacedPart> = meshes
            .iter()
            .map(|mesh| part_of(mesh, Matrix4X4::IDENTITY))
            .collect();
        let mut surfaces: Vec<ScatteringSurface> = pieces.iter().map(|piece| piece.0).collect();
        let mut opacities: Vec<f64> = pieces.iter().map(|piece| piece.1).collect();
        let mut faces = assignment.clone();
        if with_floor {
            placed.push(part_of(&wide_floor(), Matrix4X4::IDENTITY));
            surfaces.push(ScatteringSurface::matte_plastic([0.3; 3]));
            opacities.push(1.0);
            faces.push(Vec::new());
        }
        let scene = RenderScene::new(PlacedParts::new(placed), surfaces, furnace())
            .unwrap()
            .with_part_opacities(opacities)
            .unwrap();
        match &texture {
            Some(texture) => scene
                .with_face_textures(vec![texture.clone()], faces)
                .unwrap(),
            None => scene,
        }
    };
    (scene_of(true), scene_of(false))
}

/// The view every stack is packed with: a small orthographic image over the floor at z = 0.
pub(super) fn stack_view() -> GpuView {
    let camera = RenderCamera::orthographic(
        Vector3::new(0.0, -80.0, 60.0),
        Vector3::new(0.0, 0.0, 10.0),
        Vector3::UNIT_Z,
        40.0,
    )
    .unwrap();
    GpuView {
        camera,
        image: (4, 4),
        trace: (4, 4),
        lights: furnace(),
        settings: PathTraceSettings::default(),
        compositing: GpuCompositing::Floor(GpuFloor {
            center: (0.0, 0.0),
            half_extent: 200.0,
            z: 0.0,
            show_through: 0.0,
            surface: ScatteringSurface::matte_plastic([0.3; 3]),
        }),
    }
}

/// Whether the packed face the `f32` traversal returns at the seam above `(x, y)` is the lower
/// box's (rather than the upper box's it lies on).
pub(super) fn seam_hit_is_lower(pack: &GpuRenderScene, stack: Stack, (x, y): (f64, f64)) -> bool {
    let hit = gpu_hit(pack, Vector3::new(x, y, 15.0), Vector3::UNIT_Z);
    let part = pack.faces()[hit.source_index as usize].part;
    (part == 0) != stack.upper_first
}

/// The lines toward the sun from the floor under the stack, straight up and tilted, held to agree
/// with the CPU's; answers each line's crossings and the faces the traversal returned at the seam
/// (whether the lower box's, whether the pictured one).
fn lines_agree(stack: Stack) -> Vec<(u32, bool, bool)> {
    lines_agree_on(stack, stacked(stack))
}

/// [`lines_agree`] on a stack already built.
fn lines_agree_on(
    stack: Stack,
    (scene, pack): (RenderScene, GpuRenderScene),
) -> Vec<(u32, bool, bool)> {
    let mut found = Vec::new();
    for (x, y) in [(0.0, 0.0), (2.3, -1.9), (-3.1, 2.7)] {
        let lower = seam_hit_is_lower(&pack, stack, (x, y));
        let pictured_hit = lower == (stack.pictured == Pictured::LowerTop);
        let vertex = gpu_floor_vertex(&pack, x, y);
        let cpu_ray = down_from(Vector3::new(x, y, 3.0));
        for direction in [
            Vector3::UNIT_Z,
            Vector3::new(0.1, 0.05, 1.0).get_normal(),
            Vector3::new(-0.2, 0.15, 1.0).get_normal(),
        ] {
            let (cpu, gpu) = both(
                &scene,
                &pack,
                (&cpu_ray, &[], false),
                &vertex,
                direction,
                false,
                &random(),
            );
            let (_, crossed, gap) = agree_pictured(&cpu, &gpu)
                .unwrap_or_else(|| panic!("{stack:?}: the stack passes the sun"));
            assert!(
                gap <= f64::from(crossed) * PICTURED_BOUND_PER_CROSSING,
                "{stack:?} from ({x}, {y}) toward {direction:?}: the GPU's line passes {gap} more \
                 or less than the CPU's"
            );
            found.push((crossed, lower, pictured_hit));
        }
    }
    found
}

/// Both faces of the pair were returned at the seam, and so were the pictured one and the plain.
fn assert_both_faces_hit(found: &[(u32, bool, bool)]) {
    for (what, pick) in [
        ("the lower box's face", 1usize),
        ("the pictured face", 2usize),
    ] {
        let of = |line: &(u32, bool, bool)| if pick == 1 { line.1 } else { line.2 };
        assert!(found.iter().any(of), "{what} was never the one hit");
        assert!(!found.iter().all(of), "{what} was always the one hit");
    }
}

#[test]
fn a_picture_on_either_face_of_one_glass_makes_the_seam_an_interface_as_on_the_cpu() {
    // Of one glass the plain seam is passed straight (two faces in all); with a picture on one
    // face the two surfaces differ, so the seam is a third crossing whichever face was hit.
    let glass = (glass_of([1.0; 3], 0.0, 1.5), 1.0);
    let mut found = Vec::new();
    for stack in stacks(glass, glass) {
        found.extend(lines_agree(stack));
    }
    assert!(found.iter().all(|line| line.0 == 3), "{found:?}");
    assert_both_faces_hit(&found);
}

#[test]
fn a_picture_on_a_seam_of_two_glasses_tints_their_interface_as_on_the_cpu() {
    // 1.7 glass on 1.5: one interface at the seam, tinted by the picture on either face.
    let lower = (glass_of([0.9, 0.95, 1.0], 0.0, 1.5), 1.0);
    let upper = (glass_of([1.0, 0.9, 0.85], 0.0, 1.7), 1.0);
    let mut found = Vec::new();
    for stack in stacks(lower, upper) {
        found.extend(lines_agree(stack));
    }
    assert!(found.iter().all(|line| line.0 == 3), "{found:?}");
    assert_both_faces_hit(&found);
}

#[test]
fn a_covering_pictured_face_against_air_reads_its_picture_as_on_the_cpu() {
    // The lower box is never there (opacity 0): the line passes its bottom, then meets the seam
    // as the upper box's glass against air, its bottom's picture tinting it whether the trace
    // returned that face or the absent lower box's top lying on it; then the upper box's top.
    let lower = (glass_of([1.0; 3], 0.0, 1.5), 0.0);
    let upper = (glass_of([1.0; 3], 0.0, 1.5), 1.0);
    let mut found = Vec::new();
    for upper_first in [false, true] {
        found.extend(lines_agree(Stack {
            lower,
            upper,
            pictured: Pictured::UpperBottom,
            upper_first,
        }));
    }
    assert!(found.iter().all(|line| line.0 == 2), "{found:?}");
    assert_both_faces_hit(&found);
}

#[test]
fn a_picture_tinting_its_face_to_the_other_faces_color_is_one_glass_only_of_one_material() {
    // The upper box's bottom shows a uniform mid grey, which tints it to exactly the color of the
    // lower box's top lying on it (the same sRGB byte, decoded through the same table). Of one
    // material the two surfaces are then equal and the seam is passed straight (two faces in
    // all); differing only in roughness, or only in index, they are two glasses (three).
    let grey = Color::rgba(128, 128, 128, 255);
    let uniform = TraceTexture::new(1, 1, vec![128, 128, 128, 255]).unwrap();
    let lower = glass_of([1.0; 3], 0.0, 1.5);
    for (name, upper, faces) in [
        ("one material", lower, 2),
        ("rougher", glass_of([1.0; 3], 0.3, 1.5), 3),
        ("denser", glass_of([1.0; 3], 0.0, 1.6), 3),
    ] {
        let mut found = Vec::new();
        for upper_first in [false, true] {
            let stack = Stack {
                lower: (lower, 1.0),
                upper: (upper, 1.0),
                pictured: Pictured::UpperBottom,
                upper_first,
            };
            let built = stacked_from(stack, Some(uniform.clone()), Some(grey));
            found.extend(lines_agree_on(stack, built));
        }
        assert!(
            found.iter().all(|line| line.0 == faces),
            "{name}: {found:?}"
        );
        assert_both_faces_hit(&found);
    }
}
