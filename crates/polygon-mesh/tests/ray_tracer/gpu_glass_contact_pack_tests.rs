//! The GPU pack's glass contacts (`flat_tree/gpu_contacts.rs`): which packed triangles carry
//! contact entries and what an entry holds, for two glass boxes stacked face on face, a small
//! glass box of another index on a larger one, and 96 glass walls touching end to end, the block
//! riding after the part records where the view says; that a scene where no glass lies on glass
//! appends nothing, its flag clear and its block start 0;
//! and that the packed edge equations, evaluated in `f32` as the shader will, cover exactly the
//! region the covering box stands on, away from a tolerance-wide strip at its edge.

use solidmesh_math::Vector3;
use solidmesh_mesh::ray_tracer::flat_tree::{
    GpuRenderScene, GpuSceneLimits, CONTACT_COVER_DENSER, CONTACT_ENTRY_WORDS,
    CONTACT_FULLY_COVERED, CONTACT_HIT_DENSER, CONTACT_SAME_GLASS, PART_STRIDE,
    SCENE_FLAG_GLASS_CONTACTS, WEBGPU_DEFAULT_MAX_STORAGE_BINDING,
};
use solidmesh_mesh::ray_tracer::{PathTraceSettings, RenderCamera, RenderScene, ScatteringSurface};

use crate::glass_tray_support::parts_scene;
use crate::path_tracer_glass_contact_measure_tests::walls;
use crate::path_tracer_glass_contact_tests::{boxes_scene, furnace};

const LIMITS: GpuSceneLimits = GpuSceneLimits {
    max_storage_binding: WEBGPU_DEFAULT_MAX_STORAGE_BINDING,
    max_vertex_step: 1e-3,
};

fn pack(scene: &RenderScene) -> GpuRenderScene {
    let camera = RenderCamera::orthographic(
        Vector3::new(0.0, -20.0, 20.0),
        Vector3::ZERO,
        Vector3::UNIT_Z,
        10.0,
    )
    .expect("a camera");
    GpuRenderScene::pack(scene, &camera, 4, 4, &PathTraceSettings::default(), LIMITS)
        .expect("the scene packs")
}

fn glass(color: [f64; 3], roughness: f64, ior: f64) -> ScatteringSurface {
    ScatteringSurface::new(color, 0.0, roughness, 1.0, ior)
}

/// The packed triangles that carry contact entries, by `source_index`.
fn with_contacts(pack: &GpuRenderScene) -> Vec<usize> {
    (0..pack.faces().len())
        .filter(|&t| !pack.glass_contacts().of_triangle(t).0.is_empty())
        .collect()
}

/// Two glass boxes 2 on a side, one on the other: the lower box's top and the upper box's bottom
/// are each two triangles lying on the other box's two.
#[test]
fn gpu_contact_pack_of_two_stacked_boxes_lists_the_shared_faces() {
    let clear = ScatteringSurface::glass();
    let scene = boxes_scene(
        &[
            ([2.0, 2.0, 2.0], [0.0, 0.0, 1.0], clear),
            ([2.0, 2.0, 2.0], [0.0, 0.0, 3.0], clear),
        ],
        false,
        furnace(),
    );
    let pack = pack(&scene);
    let contacts = pack.glass_contacts();
    assert_eq!(pack.faces().len(), 24);
    assert_eq!(
        contacts.starts.len(),
        25,
        "a start per triangle and the count"
    );
    let carrying = with_contacts(&pack);
    assert_eq!(carrying.len(), 4, "two triangles of each shared face");
    for &t in &carrying {
        let face = pack.faces()[t];
        let lower = t < 12;
        assert_eq!(face.part, u32::from(!lower));
        assert_eq!(face.geometric[2], if lower { 1.0 } else { -1.0 });
        let (entries, fully) = contacts.of_triangle(t);
        // Whatever the diagonals, the two triangles of the other face together cover this one.
        assert!(!entries.is_empty() && entries.len() <= 2);
        for entry in entries {
            assert_eq!(entry.part, u32::from(lower), "the other box covers it");
            assert_eq!(entry.drop_axis, 2, "a horizontal face drops z");
            assert_eq!(
                entry.flags,
                CONTACT_SAME_GLASS | CONTACT_HIT_DENSER | CONTACT_COVER_DENSER
            );
            assert!(entry.tolerance >= face.ray_offset);
            assert_eq!(
                entry.interface.ior, 1.0,
                "the same glass is index 1 to itself"
            );
            assert_eq!(entry.color, 0x00FF_FFFF);
        }
        // A first entry that is the same triangle covers all of this one.
        if entries.len() == 1 {
            assert!(fully, "triangle {t}'s one contact covers it whole");
        }
    }
    let total: usize = carrying
        .iter()
        .map(|&t| contacts.of_triangle(t).0.len())
        .sum();
    assert_eq!(contacts.entries.len(), total);
    assert_eq!(contacts.starts[24], total as u32);
    let words = pack.contact_bytes().len() / 4;
    assert_eq!(
        words,
        1 + 25 + total * CONTACT_ENTRY_WORDS,
        "the count, starts and entries"
    );
    // The block follows the two part records in the part buffer, where the view says it starts.
    let part_bytes = pack.part_bytes();
    assert_eq!(part_bytes.len(), 2 * PART_STRIDE + words * 4);
    assert_eq!(pack.view().contact_block, (2 * PART_STRIDE / 4) as u32);
    assert_eq!(&part_bytes[2 * PART_STRIDE..], &pack.contact_bytes()[..]);
    assert_eq!(
        &part_bytes[2 * PART_STRIDE..2 * PART_STRIDE + 4],
        &24u32.to_le_bytes()
    );
    assert_eq!(pack.sizes().part_bytes, part_bytes.len() as u64);
    assert_ne!(pack.view().flags & SCENE_FLAG_GLASS_CONTACTS, 0);
    // Packing is deterministic: the same scene gives the same words.
    assert_eq!(pack.contact_bytes(), self::pack(&scene).contact_bytes());
}

/// A box of index 1.7, tinted and rough, standing on a quarter of a clear box of index 1.5.
fn small_on_large() -> RenderScene {
    boxes_scene(
        &[
            ([4.0, 4.0, 2.0], [0.0, 0.0, 1.0], glass([1.0; 3], 0.0, 1.5)),
            (
                [2.0, 2.0, 2.0],
                [1.0, 1.0, 3.0],
                glass([0.8, 0.9, 0.5], 0.3, 1.7),
            ),
        ],
        false,
        furnace(),
    )
}

#[test]
fn gpu_contact_pack_of_two_glasses_carries_their_interface() {
    let pack = pack(&small_on_large());
    let contacts = pack.glass_contacts();
    let lower: Vec<usize> = with_contacts(&pack)
        .into_iter()
        .filter(|&t| t < 12)
        .collect();
    assert!(!lower.is_empty() && lower.len() <= 2, "the large box's top");
    let (entries, fully) = contacts.of_triangle(lower[0]);
    assert!(!fully, "the small box covers only a quarter of the top");
    let entry = entries[0];
    assert_eq!(entry.part, 1);
    assert_eq!(
        entry.flags, CONTACT_COVER_DENSER,
        "1.7 over 1.5: the cover is denser"
    );
    assert_eq!(entry.interface.ior, (1.7_f64 / 1.5) as f32);
    assert_eq!(entry.interface.ior_minus_one, (1.7_f64 / 1.5 - 1.0) as f32);
    assert_eq!(entry.interface.base_color, [0.8_f32, 0.9, 0.5]);
    assert_eq!(entry.interface.roughness, 0.3);
    assert_eq!(entry.interface.transmission, 1.0);
    // The small box's bottom sees the large one: the hit glass is the denser one there.
    let upper: Vec<usize> = with_contacts(&pack)
        .into_iter()
        .filter(|&t| t >= 12)
        .collect();
    let (upper_entries, upper_fully) = contacts.of_triangle(upper[0]);
    assert_eq!(upper_entries[0].flags, CONTACT_HIT_DENSER);
    assert_eq!(
        upper_entries[0].interface, entry.interface,
        "the same interface either way"
    );
    assert!(
        upper_fully || upper_entries.len() == 2,
        "the large top covers the small bottom whole"
    );
}

/// The packed edges, in `f32` as the shader tests them, cover a point of the large box's top
/// exactly where the small box stands on it, at every point of a grid across the top away from
/// a strip twice the tolerance wide along the small box's edges.
#[test]
fn gpu_contact_pack_f32_point_test_agrees_with_the_covered_region() {
    let pack = pack(&small_on_large());
    let contacts = pack.glass_contacts();
    let origin = pack.scene_origin();
    let top: Vec<_> = pack
        .tree()
        .triangles()
        .iter()
        .filter(|t| {
            (t.source_index as usize) < 12
                && pack.faces()[t.source_index as usize].geometric[2] == 1.0
        })
        .copied()
        .collect();
    assert_eq!(top.len(), 2);
    let tolerance = contacts
        .entries
        .iter()
        .fold(0.0_f64, |big, e| big.max(f64::from(e.tolerance)));
    let strip = 2.0 * tolerance;
    let inside = |t: &solidmesh_mesh::ray_tracer::flat_tree::FlatTriangle, p: [f64; 2]| {
        let corners = [t.v0, t.v1, t.v2].map(|c| [f64::from(c[0]), f64::from(c[1])]);
        let side = |a: [f64; 2], b: [f64; 2]| {
            (b[0] - a[0]) * (p[1] - a[1]) - (b[1] - a[1]) * (p[0] - a[0])
        };
        let s = [
            side(corners[0], corners[1]),
            side(corners[1], corners[2]),
            side(corners[2], corners[0]),
        ];
        s.iter().all(|&v| v > 1e-6) || s.iter().all(|&v| v < -1e-6)
    };
    let (mut checked, mut covered) = (0, 0);
    let steps = 81;
    for i in 0..steps {
        for j in 0..steps {
            let world = [
                -2.0 + 4.0 * f64::from(i) / f64::from(steps - 1),
                -2.0 + 4.0 * f64::from(j) / f64::from(steps - 1),
            ];
            let near_edge = world
                .iter()
                .any(|&v| (v - 0.0).abs() <= strip || (v - 2.0).abs() <= strip);
            if near_edge {
                continue;
            }
            let expected = world.iter().all(|&v| v > 0.0 && v < 2.0);
            let packed = [world[0] - origin.x, world[1] - origin.y, 2.0 - origin.z];
            let point = packed.map(|v| v as f32);
            let flat = [f64::from(point[0]), f64::from(point[1])];
            for triangle in top.iter().filter(|t| inside(t, flat)) {
                let (entries, fully) = contacts.of_triangle(triangle.source_index as usize);
                let found = fully || entries.iter().any(|e| e.covers_f32(point));
                assert_eq!(
                    found, expected,
                    "at {world:?} in triangle {}",
                    triangle.source_index
                );
                checked += 1;
                covered += usize::from(found);
            }
        }
    }
    assert!(
        checked > 5000,
        "most of the grid is away from the edges: {checked}"
    );
    assert!(
        covered > 1000 && covered < checked / 2,
        "a quarter is covered: {covered} of {checked}"
    );
}

/// 96 walls touching end to end in 12 rows of 8: 7 joins a row, each two end faces of two
/// triangles lying on each other.
#[test]
fn gpu_contact_pack_of_96_touching_walls_lists_every_join() {
    let scene = parts_scene(&walls(true), ScatteringSurface::glass(), furnace());
    let pack = pack(&scene);
    let carrying = with_contacts(&pack);
    assert_eq!(pack.faces().len(), 96 * 12);
    assert_eq!(carrying.len(), 12 * 7 * 2 * 2);
    for &t in &carrying {
        let face = pack.faces()[t];
        assert_eq!(face.geometric[1], 0.0, "only end faces, along x, touch");
        assert_eq!(face.geometric[2], 0.0);
        let (entries, _) = pack.glass_contacts().of_triangle(t);
        for entry in entries {
            assert_eq!(entry.drop_axis, 0, "an end face drops x");
            let neighbour = if face.geometric[0] > 0.0 {
                face.part + 1
            } else {
                face.part - 1
            };
            assert_eq!(entry.part, neighbour, "the next wall along the row");
        }
    }
    let apart = parts_scene(&walls(false), ScatteringSurface::glass(), furnace());
    let apart_pack = self::pack(&apart);
    assert!(with_contacts(&apart_pack).is_empty());
}

/// A scene where no glass lies on glass: the gapped walls, and the stacked boxes with the upper
/// one matte. Nothing is appended to the part buffer, the flag is clear and the block start is 0,
/// so every buffer is what it was before any contact existed: every other pack test pins those.
#[test]
fn gpu_contact_pack_without_contacts_appends_nothing() {
    let matte = ScatteringSurface::new([0.5; 3], 0.0, 1.0, 0.0, 1.0);
    let scene = boxes_scene(
        &[
            ([2.0, 2.0, 2.0], [0.0, 0.0, 1.0], ScatteringSurface::glass()),
            ([2.0, 2.0, 2.0], [0.0, 0.0, 3.0], matte),
        ],
        false,
        furnace(),
    );
    for scene in [
        scene,
        parts_scene(&walls(false), ScatteringSurface::glass(), furnace()),
    ] {
        let pack = pack(&scene);
        assert!(pack.contact_bytes().is_empty());
        assert_eq!(pack.part_bytes().len(), pack.parts().len() * PART_STRIDE);
        assert_eq!(pack.view().contact_block, 0);
        assert!(pack.glass_contacts().starts.is_empty());
        assert_eq!(pack.view().flags & SCENE_FLAG_GLASS_CONTACTS, 0);
        for t in 0..pack.faces().len() {
            assert_eq!(pack.glass_contacts().of_triangle(t).0.len(), 0);
        }
    }
    assert_eq!(CONTACT_FULLY_COVERED, 1 << 31);
}
