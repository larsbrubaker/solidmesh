//! What a seam with a picture on one face reads, packed and in WGSL: in a scene with pictures the
//! contact block ends with one word per entry, the covering face's triangle in the tree, which
//! names the covering face of the right part lying on the seam, and the entry's flags say whether
//! the two glasses are the same apart from color; a scene without pictures packs neither, nor the
//! view flag. The shader's constants are the pack's, its seam functions are their mirrors'
//! (`seam_f32.rs`) step for step, and the mirror's placing of a point on the covering face lands
//! where it should, inside and outside the face, and where the CPU's own placing does. The view's
//! picture flag is the pack's, whatever scene a view is later set with.

use naga::Literal;
use solidmesh_math::{Matrix4X4, Vector3};
use solidmesh_mesh::ray_tracer::flat_tree::{
    cover_weights_f32, GpuRenderScene, GpuSceneLimits, CONTACT_ENTRY_WORDS, CONTACT_SAME_GLASS,
    CONTACT_SAME_MATERIAL, PATH_WGSL, SCENE_FLAG_PICTURES, WEBGPU_DEFAULT_MAX_STORAGE_BINDING,
};
use solidmesh_mesh::Mesh;

use crate::flat_tree_shader_tests::literal;
use crate::glass_tray_support::part_of;
use crate::gpu_texture_pack_tests::picture;
use crate::seam_picture_line_tests::{stack_view, stacked, stacked_scenes, stacked_with, stacks};
use crate::see_through_on_floor_tests::function_text;
use crate::sun_line_shader_tests::glass_of;

/// The part buffer as words.
fn part_words(pack: &GpuRenderScene) -> Vec<u32> {
    pack.part_bytes()
        .as_chunks::<4>()
        .0
        .iter()
        .map(|word| u32::from_le_bytes(*word))
        .collect()
}

#[test]
fn a_pictured_scene_packs_each_entrys_covering_triangle_after_the_entries() {
    let one = (glass_of([1.0; 3], 0.0, 1.5), 1.0);
    let other = (glass_of([1.0; 3], 0.0, 1.7), 1.0);
    for (lower, upper, same) in [(one, one, true), (one, other, false)] {
        for stack in stacks(lower, upper) {
            let (_, pack) = stacked(stack);
            assert_ne!(pack.view().flags & SCENE_FLAG_PICTURES, 0, "{stack:?}");
            let contacts = pack.glass_contacts();
            assert!(
                !contacts.entries.is_empty(),
                "{stack:?}: the seam has contacts"
            );
            assert_eq!(contacts.covering_triangles.len(), contacts.entries.len());
            // The block as the shader reads it: count, starts, entries, then the covering list.
            let words = contacts.to_words();
            let listed = 1 + contacts.starts.len() + contacts.entries.len() * CONTACT_ENTRY_WORDS;
            assert_eq!(words.len(), listed + contacts.entries.len());
            assert_eq!(&words[listed..], &contacts.covering_triangles[..]);
            let block = pack.view().contact_block as usize;
            assert_eq!(&part_words(&pack)[block..block + words.len()], &words[..]);
            // Each names a face of the covering part lying on the seam's plane.
            let seam_z = (25.0 - pack.scene_origin().z) as f32;
            for triangle in 0..contacts.starts.len() - 1 {
                let (first, entries, _) = contacts.of_triangle_from(triangle);
                for (offset, entry) in entries.iter().enumerate() {
                    let cover = contacts.covering_triangles[first + offset] as usize;
                    let packed = pack.tree().triangles()[cover];
                    let face = pack.faces()[packed.source_index as usize];
                    assert_eq!(face.part, entry.part, "{stack:?}: the covering part's face");
                    assert_ne!(face.part, pack.faces()[triangle].part);
                    for corner in [packed.v0, packed.v1, packed.v2] {
                        assert!((corner[2] - seam_z).abs() < 1e-5, "{stack:?}: {corner:?}");
                    }
                }
            }
            for entry in &contacts.entries {
                assert_eq!(entry.flags & CONTACT_SAME_MATERIAL != 0, same, "{stack:?}");
                assert_eq!(entry.flags & CONTACT_SAME_GLASS != 0, same, "{stack:?}");
            }
        }
    }
}

#[test]
fn a_scene_without_pictures_packs_no_covering_triangles_and_no_picture_flags() {
    let glass = (glass_of([1.0; 3], 0.0, 1.5), 1.0);
    for stack in stacks(glass, glass) {
        let (_, plain) = stacked_with(stack, false);
        let (_, pictured) = stacked(stack);
        assert_eq!(plain.view().flags & SCENE_FLAG_PICTURES, 0);
        let contacts = plain.glass_contacts();
        assert!(contacts.covering_triangles.is_empty());
        assert!(contacts
            .entries
            .iter()
            .all(|entry| entry.flags & CONTACT_SAME_MATERIAL == 0));
        // The entries themselves are the pictured scene's but for the flag.
        let mut flagged = contacts.entries.clone();
        for entry in &mut flagged {
            entry.flags |= CONTACT_SAME_MATERIAL;
        }
        assert_eq!(flagged, pictured.glass_contacts().entries, "{stack:?}");
    }
}

#[test]
fn the_seam_picture_constants_are_the_packs() {
    let module = naga::front::wgsl::parse_str(PATH_WGSL).expect("the path loop parses");
    assert_eq!(
        literal(&module, "CONTACT_SAME_MATERIAL"),
        Literal::U32(CONTACT_SAME_MATERIAL)
    );
    assert_eq!(
        literal(&module, "PATH_FLAG_PICTURES"),
        Literal::U32(SCENE_FLAG_PICTURES)
    );
}

/// The shader's seam pieces for pictures, flattened by `function_text`: the mirror's steps.
const SEAM_PICTURE_PIECES: [(&str, &str); 4] = [
    (
        "seam_cover_triangle",
        "fn seam_cover_triangle(at: u32) -> u32 { let block = path_view.contact_block; let count = \
         parts[block]; let entries = block + 2u + count; let total = parts[block + 1u + count] & \
         ~CONTACT_FULLY_COVERED; return parts[entries + total * CONTACT_ENTRY_WORDS + (at - \
         entries) / CONTACT_ENTRY_WORDS];",
    ),
    (
        "seam_rebuilt_distance",
        "fn seam_rebuilt_distance(c0: vec3<f32>, c1: vec3<f32>, c2: vec3<f32>, w: vec3<f32>, \
         position: vec3<f32>) -> f32 { let at = c0 * w.x + c1 * w.y + c2 * w.z - position; return \
         dot(at, at);",
    ),
    (
        "seam_edge_along",
        "fn seam_edge_along(start: vec3<f32>, end: vec3<f32>, position: vec3<f32>) -> f32 { let \
         edge = end - start; let edge_squared = dot(edge, edge); var t = 0.0f; if edge_squared > \
         0.0f { t = clamp(dot(position - start, edge) / edge_squared, 0.0f, 1.0f); } return t;",
    ),
    (
        "seam_cover_hit",
        "fn seam_cover_hit(cover: u32, position: vec3<f32>) -> Hit { let triangle = \
         triangles[cover]; let c0 = path_vec3(triangle.v0); let c1 = path_vec3(triangle.v1); let \
         c2 = path_vec3(triangle.v2); let normal = cross(c1 - c0, c2 - c0); let normal_squared = \
         dot(normal, normal); var best = vec3<f32>(1.0f / 3.0f); var nearest = 0.0f; var found = \
         false; if normal_squared > 0.0f && path_is_finite(normal_squared) { let w0 = dot(normal, \
         cross(c1 - position, c2 - position)) / normal_squared; let w1 = dot(normal, cross(c2 - \
         position, c0 - position)) / normal_squared; let w2 = dot(normal, cross(c0 - position, c1 \
         - position)) / normal_squared; let sum = w0 + w1 + w2; if w0 >= 0.0f && w1 >= 0.0f && w2 \
         >= 0.0f && sum > 0.0f && path_is_finite(sum) { let weights = vec3<f32>(w0 / sum, w1 / \
         sum, w2 / sum); let distance = seam_rebuilt_distance(c0, c1, c2, weights, position); if \
         path_is_finite(distance) { best = weights; nearest = distance; found = true; } } } let \
         t01 = seam_edge_along(c0, c1, position); let w01 = vec3<f32>(1.0f - t01, t01, 0.0f); let \
         d01 = seam_rebuilt_distance(c0, c1, c2, w01, position); if path_is_finite(d01) && \
         (!found || d01 < nearest) { best = w01; nearest = d01; found = true; } let t12 = \
         seam_edge_along(c1, c2, position); let w12 = vec3<f32>(0.0f, 1.0f - t12, t12); let d12 = \
         seam_rebuilt_distance(c0, c1, c2, w12, position); if path_is_finite(d12) && (!found || \
         d12 < nearest) { best = w12; nearest = d12; found = true; } let t20 = \
         seam_edge_along(c2, c0, position); let w20 = vec3<f32>(t20, 0.0f, 1.0f - t20); let d20 = \
         seam_rebuilt_distance(c0, c1, c2, w20, position); if path_is_finite(d20) && (!found || \
         d20 < nearest) { best = w20; } return Hit(0.0f, cover, triangle.source_index, 0u, \
         best.x, best.y, best.z);",
    ),
];

/// `seam_at`'s steps for pictures, in order: the covering face's lookup once the covering part
/// is there, the one-glass test from the surfaces where a factor differs from 1, and the two
/// surfaces that read the covering face's picture.
const SEAM_AT_PICTURE_STEPS: [&str; 3] = [
    "if !here_present && !there_present { seam.kind = SEAM_STRAIGHT; return seam; } let \
     there_color = parts[at + CONTACT_WORD_COLOR]; var there_picture = vec3<f32>(1.0f); var \
     pictured = false; if there_present && (path_view.flags & PATH_FLAG_PICTURES) != 0u { \
     there_picture = path_picture_at(seam_cover_hit(seam_cover_triangle(at), point.position)); \
     pictured = any(there_picture != vec3<f32>(1.0f)) || any(point.picture != vec3<f32>(1.0f)); } \
     var same = (flags & CONTACT_SAME_GLASS) != 0u; var tint = vec3<f32>(0.0f); if pictured && \
     here_present { let here_surface = path_surface_at(point.part, point.color, point.picture); \
     let there_surface = path_surface_at(there_part, there_color, there_picture); same = (flags & \
     CONTACT_SAME_MATERIAL) != 0u && here_surface.base_r == there_surface.base_r && \
     here_surface.base_g == there_surface.base_g && here_surface.base_b == there_surface.base_b; \
     tint = vec3<f32>(here_surface.base_r * there_surface.base_r, here_surface.base_g * \
     there_surface.base_g, here_surface.base_b * there_surface.base_b); } if here_present && \
     there_present && same {",
    "} else if !here_present && there_present { seam.surface = path_surface_at(there_part, \
     there_color, there_picture);",
    "} else { seam.surface = seam_interface(at); if pictured { seam.surface.base_r = tint.x; \
     seam.surface.base_g = tint.y; seam.surface.base_b = tint.z; }",
];

#[test]
fn the_shader_seam_reads_the_covering_picture_as_its_mirror_does() {
    for (name, text) in SEAM_PICTURE_PIECES {
        assert_eq!(
            function_text(PATH_WGSL, name),
            text,
            "{name} is not its mirror's"
        );
    }
    let seam = function_text(PATH_WGSL, "seam_at");
    let mut from = 0;
    for step in SEAM_AT_PICTURE_STEPS {
        let at = seam[from..]
            .find(step)
            .unwrap_or_else(|| panic!("seam_at no longer has, in order, `{step}`: {seam}"));
        from += at + step.len();
    }
}

#[test]
fn the_covering_point_lands_on_the_face_inside_and_outside_it() {
    let corners = [[0.0, 0.0, 0.0], [4.0, 0.0, 0.0], [0.0, 2.0, 0.0]];
    // Inside, and a little off the face's plane, as an f32 hit on the other face of a seam is.
    assert_eq!(
        cover_weights_f32(corners, [1.0, 0.5, 0.0]),
        [0.5, 0.25, 0.25]
    );
    assert_eq!(
        cover_weights_f32(corners, [1.0, 0.5, 1e-3]),
        [0.5, 0.25, 0.25]
    );
    // Past an edge, the nearest point on it.
    assert_eq!(
        cover_weights_f32(corners, [2.0, -1.0, 0.0]),
        [0.5, 0.5, 0.0]
    );
    assert_eq!(
        cover_weights_f32(corners, [-3.0, 5.0, 0.0]),
        [0.0, 0.0, 1.0]
    );
    // A face with no area places the point on its one edge; with nothing finite, a third each.
    let sliver = [[0.0, 0.0, 0.0], [2.0, 0.0, 0.0], [2.0, 0.0, 0.0]];
    assert_eq!(
        cover_weights_f32(sliver, [0.5, 1.0, 0.0]),
        [0.75, 0.25, 0.0]
    );
    let broken = [[f32::NAN; 3], [1.0, 0.0, 0.0], [0.0, 1.0, 0.0]];
    assert_eq!(cover_weights_f32(broken, [0.2, 0.2, 0.0]), [1.0 / 3.0; 3]);
}

/// How far the mirror's weights may be from the CPU's: the corners are within 8 of the origin and
/// the triangle's area about 20, so each `f32` cross and dot rounds the area ratios by a few
/// 1e-7; 2e-6 is several times that. The worst measured is 8.0e-8.
const WEIGHT_BOUND: f64 = 2e-6;

#[test]
fn the_covering_point_lands_where_the_cpus_own_placing_puts_it() {
    // A slanted triangle the CPU's surface point places points on (`barycentric_at`, through a
    // part at the identity), and the mirror's weights on the same f32 corners and f32 points.
    let corners = [
        Vector3::new(1.5, -2.25, 0.75),
        Vector3::new(7.0, 1.0, -1.5),
        Vector3::new(-0.5, 4.5, 2.0),
    ];
    let mut mesh = Mesh::new();
    mesh.create_face(&corners);
    let part = part_of(&mesh, Matrix4X4::IDENTITY);
    let normal = (corners[1] - corners[0])
        .cross(corners[2] - corners[0])
        .get_normal();
    let at = |w: [f64; 3]| corners[0] * w[0] + corners[1] * w[1] + corners[2] * w[2];
    let mut points = Vec::new();
    // Inside, on each edge and at the corners, then past each edge and past each corner.
    for (a, b) in [(0.2, 0.3), (0.6, 0.15), (0.05, 0.9), (0.33, 0.33)] {
        points.push(at([1.0 - a - b, a, b]));
    }
    for t in [0.0, 0.25, 0.7, 1.0] {
        points.push(at([1.0 - t, t, 0.0]));
        points.push(at([0.0, 1.0 - t, t]));
        points.push(at([t, 0.0, 1.0 - t]));
    }
    for (a, b) in [
        (-0.3, 0.4),
        (0.5, 0.8),
        (0.4, -0.2),
        (1.4, -0.1),
        (-0.2, -0.3),
        (-0.4, 1.5),
    ] {
        points.push(at([1.0 - a - b, a, b]));
    }
    // Off the plane on either side, as an f32 hit on the other face of a seam lies, and far off.
    let near: Vec<Vector3> = points.clone();
    for offset in [1e-3, -1e-3, 0.4, -2.5] {
        for point in &near {
            points.push(*point + normal * offset);
        }
    }
    let narrow = |v: Vector3| [v.x as f32, v.y as f32, v.z as f32];
    let packed = corners.map(narrow);
    let mut worst = 0.0_f64;
    for point in points {
        let point32 = narrow(point);
        let exact = Vector3::new(
            f64::from(point32[0]),
            f64::from(point32[1]),
            f64::from(point32[2]),
        );
        let cpu = part.surface_point(0, exact).unwrap().barycentric;
        let gpu = cover_weights_f32(packed, point32);
        for (cpu, gpu) in cpu.iter().zip(gpu) {
            worst = worst.max((cpu - f64::from(gpu)).abs());
        }
        assert!(
            (0..3).all(|c| (cpu[c] - f64::from(gpu[c])).abs() <= WEIGHT_BOUND),
            "at {exact:?} the mirror's weights {gpu:?} against the CPU's {cpu:?}"
        );
    }
    assert!(worst <= WEIGHT_BOUND, "worst {worst}");
}

#[test]
fn the_picture_flag_is_the_packs_whatever_scene_the_view_is_set_with() {
    let glass = (glass_of([1.0; 3], 0.0, 1.5), 1.0);
    let stack = stacks(glass, glass)[0];
    let limits = GpuSceneLimits {
        max_storage_binding: WEBGPU_DEFAULT_MAX_STORAGE_BINDING,
        max_vertex_step: 1e-3,
    };
    let (_, pictured) = stacked_scenes(stack, Some(picture(4, 3, 31)), None);
    let (_, plain) = stacked_scenes(stack, None, None);
    // A pack without pictures never promises covering words, even for a pictured scene's view.
    let mut plain_pack = GpuRenderScene::pack_view(&plain, &stack_view(), limits).unwrap();
    assert_eq!(plain_pack.view().flags & SCENE_FLAG_PICTURES, 0);
    plain_pack.set_view(&pictured, &stack_view()).unwrap();
    assert_eq!(plain_pack.view().flags & SCENE_FLAG_PICTURES, 0);
    // And a pack with them keeps the flag whatever the scene says.
    let mut pictured_pack = GpuRenderScene::pack_view(&pictured, &stack_view(), limits).unwrap();
    assert_ne!(pictured_pack.view().flags & SCENE_FLAG_PICTURES, 0);
    pictured_pack.set_view(&plain, &stack_view()).unwrap();
    assert_ne!(pictured_pack.view().flags & SCENE_FLAG_PICTURES, 0);
}
