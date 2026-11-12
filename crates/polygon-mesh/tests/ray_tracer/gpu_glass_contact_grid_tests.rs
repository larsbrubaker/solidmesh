//! The GPU pack's glass contacts for nine clear glass boxes, 20 on a side, in a 3 by 3 grid on a
//! 60 by 60 footprint over a floor: the centre box lies face on face on four neighbours and
//! touches the four diagonal ones only along an edge. For every packed triangle, the entries the
//! shader reads (through the per-triangle starts, with the floor's triangles first) name exactly
//! the box across that face, none for an edge-only neighbour, and the shader's `f32` point test
//! covers every point of a touching face, its edges and corners included, and nothing of a face
//! that meets air or the floor.

use solidmesh_math::Vector3;
use solidmesh_mesh::ray_tracer::flat_tree::{
    GpuRenderScene, GpuSceneLimits, CONTACT_COVER_DENSER, CONTACT_HIT_DENSER, CONTACT_SAME_GLASS,
    WEBGPU_DEFAULT_MAX_STORAGE_BINDING,
};
use solidmesh_mesh::ray_tracer::{PathTraceSettings, RenderCamera, ScatteringSurface};

use crate::path_tracer_glass_contact_tests::{boxes_scene, furnace};

const SIDE: f64 = 20.0;

/// The grid cell (column along x, row along y) of box `index`, in the order the boxes are placed.
fn cell(index: usize) -> (i64, i64) {
    ((index / 3) as i64, (index % 3) as i64)
}

fn grid_pack() -> GpuRenderScene {
    let boxes: Vec<_> = (0..9)
        .map(|index| {
            let (column, row) = cell(index);
            (
                [SIDE; 3],
                [
                    SIDE * (column - 1) as f64,
                    SIDE * (row - 1) as f64,
                    SIDE / 2.0,
                ],
                ScatteringSurface::glass(),
            )
        })
        .collect();
    let scene = boxes_scene(&boxes, true, furnace());
    let camera = RenderCamera::orthographic(
        Vector3::new(0.0, -80.0, 60.0),
        Vector3::new(0.0, 0.0, 10.0),
        Vector3::UNIT_Z,
        40.0,
    )
    .expect("a camera");
    let limits = GpuSceneLimits {
        max_storage_binding: WEBGPU_DEFAULT_MAX_STORAGE_BINDING,
        max_vertex_step: 1e-3,
    };
    GpuRenderScene::pack(&scene, &camera, 4, 4, &PathTraceSettings::default(), limits)
        .expect("the grid packs")
}

/// The part across a face of box part `part` whose outward normal is `normal`, when a box lies
/// face on face there: the floor is part 0, box `i` is part `i + 1`.
fn neighbour_across(part: u32, normal: [f32; 3]) -> Option<u32> {
    let (column, row) = cell(part as usize - 1);
    let [x, y, z] = normal;
    let (dc, dr) = if x.abs() == 1.0 && y == 0.0 && z == 0.0 {
        (x as i64, 0)
    } else if y.abs() == 1.0 && x == 0.0 && z == 0.0 {
        (0, y as i64)
    } else {
        return None;
    };
    let (c, r) = (column + dc, row + dr);
    ((0..3).contains(&c) && (0..3).contains(&r)).then(|| (c * 3 + r) as u32 + 1)
}

#[test]
fn gpu_contact_pack_of_a_three_by_three_grid_names_the_box_across_each_face() {
    let pack = grid_pack();
    let contacts = pack.glass_contacts();
    // The floor's 2 triangles, then 12 for each box.
    assert_eq!(pack.faces().len(), 2 + 9 * 12);
    let mut touching_triangles = 0;
    for (t, face) in pack.faces().iter().enumerate() {
        let (entries, _) = contacts.of_triangle(t);
        if face.part == 0 {
            assert!(entries.is_empty(), "the floor is not glass");
            continue;
        }
        let across = neighbour_across(face.part, face.geometric);
        assert_eq!(
            !entries.is_empty(),
            across.is_some(),
            "triangle {t} of part {} facing {:?}",
            face.part,
            face.geometric
        );
        let Some(across) = across else {
            continue;
        };
        touching_triangles += 1;
        let axis = face.geometric.iter().position(|v| *v != 0.0).unwrap() as u32;
        assert!(entries.len() <= 2, "the face across is two triangles");
        for entry in entries {
            assert_eq!(
                entry.part, across,
                "triangle {t}: only the box across the face"
            );
            assert_eq!(entry.drop_axis, axis);
            assert_eq!(
                entry.flags,
                CONTACT_SAME_GLASS | CONTACT_HIT_DENSER | CONTACT_COVER_DENSER
            );
        }
    }
    // 12 shared faces in the grid, each seen from both sides, two triangles each.
    assert_eq!(touching_triangles, 12 * 2 * 2);
}

#[test]
fn gpu_contact_pack_of_a_three_by_three_grid_covers_every_touching_face_to_its_edges() {
    let pack = grid_pack();
    let contacts = pack.glass_contacts();
    let steps = 24;
    let mut tested = 0;
    for triangle in pack.tree().triangles() {
        let t = triangle.source_index as usize;
        let face = pack.faces()[t];
        if face.part == 0 {
            continue;
        }
        let expected = neighbour_across(face.part, face.geometric).is_some();
        let (entries, fully) = contacts.of_triangle(t);
        let corners = [triangle.v0, triangle.v1, triangle.v2].map(|c| c.map(f64::from));
        for i in 0..=steps {
            for j in 0..=steps - i {
                let (u, v) = (
                    f64::from(i) / f64::from(steps),
                    f64::from(j) / f64::from(steps),
                );
                let w = 1.0 - u - v;
                let point: [f32; 3] = std::array::from_fn(|axis| {
                    (corners[0][axis] * w + corners[1][axis] * u + corners[2][axis] * v) as f32
                });
                let covered = fully || entries.iter().any(|entry| entry.covers_f32(point));
                assert_eq!(
                    covered, expected,
                    "triangle {t} of part {} facing {:?} at {point:?}",
                    face.part, face.geometric
                );
                tested += 1;
            }
        }
    }
    assert_eq!(tested, 9 * 12 * (steps + 1) * (steps + 2) / 2);
}
