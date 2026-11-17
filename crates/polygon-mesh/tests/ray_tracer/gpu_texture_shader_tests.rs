//! The GPU path loop reading pictures (`picture.wgsl`), checked without a graphics card: the atlas
//! is bound where the header says, the shader's lookup is its Rust mirror's text step for step
//! (`texture_lookup_f32`, which `gpu_texture_pack_tests` holds against the CPU's lookup), the path
//! loop and the line toward the sun look a picture up where the CPU reads one and carry it into the
//! surface in the CPU's order, a hit the `f32` reference traversal finds reads the picture the
//! CPU's hit reads (so the packed corner weights are the coordinates' corners), and the mirror's
//! line toward the sun through pictured glass, and from the floor under pictured glass resting
//! there, passes what the CPU's line passes. The shader's own arithmetic runs first on a card in
//! the app.

use naga::{ImageClass, ImageDimension, ScalarKind, TypeInner};
use solidmesh_math::{IntersectionType, Matrix4X4, Ray, Vector2Float, Vector3};
use solidmesh_mesh::ray_tracer::flat_tree::{
    FlatRay, GpuCompositing, GpuFloor, GpuRenderScene, GpuSceneLimits, GpuTextureLimits, GpuView,
    SunLineF32, SunLineScene, FACE_BOTH, PATH_WGSL, WEBGPU_DEFAULT_MAX_STORAGE_BINDING,
};
use solidmesh_mesh::ray_tracer::{
    PathTraceSettings, PlacedPart, PlacedParts, RenderCamera, RenderScene, ScatteringSurface,
    SunLineAt, TraceTexture,
};
use solidmesh_mesh::{FaceTextureData, Mesh};
use std::sync::Arc;

use crate::glass_on_floor_tests::furnace;
use crate::glass_tray_support::{box_meshes, part_of};
use crate::gpu_texture_pack_tests::{pack_with, picture, pictured_scene};
use crate::see_through_on_floor_tests::{function_text, wide_floor};
use crate::sun_line_shader_tests::{
    both, down_from, glass_of, gpu_floor_under, gpu_floor_vertex, narrow, packed, random,
    sun_directions,
};

/// The lookup's pieces, flattened by `function_text`: `texture_lookup_f32` and its `side_spot_f32`
/// in the shader's words.
const LOOKUP: [(&str, &str); 4] = [
    (
        "path_picture_index",
        "fn path_picture_index(at: i32, count: i32) -> u32 { return u32(at + count) % \
         u32(count);",
    ),
    (
        "path_picture_side",
        "fn path_picture_side(coordinate: f32, size_in: u32) -> PathPictureSide { let size = \
         max(size_in, 1u); var wrapped = 0.0f; if path_is_finite(coordinate) { wrapped = \
         coordinate - floor(coordinate); } let x = wrapped * f32(size) - 0.5f; let x0 = floor(x); \
         let fraction = x - x0; let first = i32(x0); let count = i32(size); return \
         PathPictureSide(path_picture_index(first, count), path_picture_index(first + 1, count), \
         fraction);",
    ),
    (
        "path_picture_texel",
        "fn path_picture_texel(at: vec2<u32>) -> vec3<f32> { let texel = textureLoad(path_atlas, \
         at, 0); let alpha = f32(texel.a) / 255.0f; let clear = 1.0f - alpha; return vec3<f32>( \
         alpha * path_srgb.to_linear[texel.r >> 2u][texel.r & 3u] + clear, alpha * \
         path_srgb.to_linear[texel.g >> 2u][texel.g & 3u] + clear, alpha * \
         path_srgb.to_linear[texel.b >> 2u][texel.b & 3u] + clear, );",
    ),
    (
        "path_picture_at",
        "fn path_picture_at(hit: Hit) -> vec3<f32> { let entry = \
         faces[hit.source_index].texture; if entry == PATH_NO_TEXTURE { return vec3<f32>(1.0f); } \
         let u = bitcast<f32>(parts[entry]) * hit.u + bitcast<f32>(parts[entry + 2u]) * hit.v + \
         bitcast<f32>(parts[entry + 4u]) * hit.w; let v = bitcast<f32>(parts[entry + 1u]) * hit.u \
         + bitcast<f32>(parts[entry + 3u]) * hit.v + bitcast<f32>(parts[entry + 5u]) * hit.w; let \
         picture = parts[entry + PATH_TEXTURE_FACE_PICTURE]; let columns = path_picture_side(u, \
         parts[picture + PATH_TEXTURE_PICTURE_SIZE]); let rows = path_picture_side(v, \
         parts[picture + PATH_TEXTURE_PICTURE_SIZE + 1u]); let x = parts[picture]; let y = \
         parts[picture + 1u]; let bottom_left = path_picture_texel(vec2<u32>(columns.first + x, \
         rows.first + y)); let bottom_right = path_picture_texel(vec2<u32>(columns.second + x, \
         rows.first + y)); let top_left = path_picture_texel(vec2<u32>(columns.first + x, \
         rows.second + y)); let top_right = path_picture_texel(vec2<u32>(columns.second + x, \
         rows.second + y)); let bottom = bottom_left + columns.fraction * (bottom_right - \
         bottom_left); let top = top_left + columns.fraction * (top_right - top_left); return \
         bottom + rows.fraction * (top - bottom);",
    ),
];

#[test]
fn the_path_loop_reads_the_atlas_as_an_unsigned_texture_at_group_2_binding_7() {
    let module = naga::front::wgsl::parse_str(PATH_WGSL).expect("the path loop parses");
    let (_, atlas) = module
        .global_variables
        .iter()
        .find(|(_, global)| global.name.as_deref() == Some("path_atlas"))
        .expect("the shader declares the atlas");
    let binding = atlas.binding.as_ref().expect("the atlas is bound");
    assert_eq!((binding.group, binding.binding), (2, 7));
    assert_eq!(
        module.types[atlas.ty].inner,
        TypeInner::Image {
            dim: ImageDimension::D2,
            arrayed: false,
            class: ImageClass::Sampled {
                kind: ScalarKind::Uint,
                multi: false,
            },
        },
        "an Rgba8Uint atlas read texel by texel"
    );
}

#[test]
fn the_shader_lookup_is_its_mirror_step_for_step() {
    for (name, text) in LOOKUP {
        assert_eq!(
            function_text(PATH_WGSL, name),
            text,
            "{name} is not the mirror's step"
        );
    }
}

/// The path loop looks a picture up once a vertex where the CPU reads one, the line toward the sun
/// at each face it crosses and at the glass across a contact (the vertex's own lookup), and the
/// surface takes it after the face's color; a seam takes the hit face's from the point and looks
/// the covering face's up itself (`seam_picture_pack_tests` pins how).
#[test]
fn the_shader_carries_a_picture_into_the_surface_where_the_cpu_reads_one() {
    let surface_at = function_text(PATH_WGSL, "path_surface_at");
    assert!(
        surface_at.ends_with(
            "surface.base_b *= path_srgb.to_linear[b >> 2u][b & 3u]; } surface.base_r *= \
             picture.x; surface.base_g *= picture.y; surface.base_b *= picture.z; return surface;"
        ),
        "the picture is not taken after the face's color: {surface_at}"
    );
    let path = function_text(PATH_WGSL, "path_radiance");
    for step in [
        "var rest_picture = vec3<f32>(1.0f); if present && hit.triangle != NO_HIT && (!floor_first \
         || resting) { let picture = path_picture_at(hit); if floor_first { rest_picture = \
         picture; } else { point.picture = picture; } } var seam = seam_none();",
        "surface = path_surface_at(point.part, point.color, point.picture);",
    ] {
        assert!(path.contains(step), "the path loop no longer has `{step}`");
    }
    let seam = function_text(PATH_WGSL, "seam_at");
    for step in [
        "seam.surface = path_surface_at(point.part, point.color, point.picture);",
        "seam.surface = path_surface_at(there_part, there_color, there_picture);",
    ] {
        assert!(seam.contains(step), "the seam no longer has `{step}`");
    }
    // Only the two lookups give a point its picture; the seam's gives the covering face's.
    let text = PATH_WGSL.replace("\r\n", "\n");
    assert_eq!(text.matches("path_picture_at(hit);").count(), 2);
}

/// Hits the `f32` reference traversal finds on every pictured face of `pictured_scene` (and on
/// the cube's faces without coordinates) read, through the shader's lookup at the hit's own
/// weights, the factor the CPU reads at its hit of the same ray: the packed triangle's corner
/// weights are the picture coordinates' corners (corners out of order would put the factor tenths
/// off). The gap is the `f32` hit's: the ray starts 30 out, where an `f32` step is 1.9e-6, and its
/// hit lies up to 4e-6 from the CPU's; that moves the coordinate by up to 1.5 times as much (the
/// faces' coordinate gradient), the texel position by the picture's side (5) times that, and the
/// factor, which bilinear filtering keeps continuous, by at most that (neighbouring texel factors
/// differ by less than 1): 3e-5. The worst measured is 1.65e-5; at the CPU's own weights the
/// lookup is within 3e-7 (`gpu_texture_pack_tests` holds that).
#[test]
fn a_traced_hit_reads_the_picture_the_cpu_reads_at_its_hit() {
    let textures = vec![picture(5, 4, 1), picture(3, 4, 2), picture(2, 2, 3)];
    let (scene, _) = pictured_scene(textures);
    let pack = pack_with(&scene, GpuTextureLimits::DEFAULT);
    let walk = SunLineScene::new(&pack, false, false);
    let mut first_triangle = Vec::new();
    let mut total = 0u32;
    for part in scene.parts().parts() {
        first_triangle.push(total);
        total += part.surface().face_count() as u32;
    }
    let directions = [
        Vector3::new(0.0, 0.0, -1.0),
        Vector3::new(0.31, -0.22, -1.0).get_normal(),
        Vector3::new(-1.0, 0.13, -0.4).get_normal(),
        Vector3::new(0.2, 1.0, -0.35).get_normal(),
    ];
    let (mut pictured, mut plain, mut worst) = (0, 0, 0.0_f64);
    for direction in directions {
        for i in 0..23 {
            for j in 0..17 {
                let target = Vector3::new(
                    -0.93 + 0.2531 * f64::from(i),
                    -0.97 + 0.1173 * f64::from(j),
                    0.6,
                );
                let origin = target - direction * 30.0;
                let mut ray = Ray::with_limits(
                    origin,
                    direction,
                    0.0,
                    f64::INFINITY,
                    IntersectionType::Both,
                );
                let Some(hit) = scene.parts().trace(&mut ray) else {
                    continue;
                };
                let point = hit.surface_point().expect("a point");
                let part = hit.part.index();
                let cpu = scene
                    .face_texture(part, point.face_index as usize)
                    .zip(point.texture_coordinate)
                    .and_then(|(texture, uv)| scene.texture_factor_at(texture, uv));
                let flat = FlatRay {
                    origin: packed(&pack, origin),
                    direction: narrow(direction),
                    t_min: 0.0,
                    t_max: 3.0e38,
                    face_mask: FACE_BOTH,
                };
                let gpu_hit = pack.tree().trace_closest(&flat).expect("the GPU hits too");
                assert_eq!(
                    gpu_hit.source_index,
                    first_triangle[part] + point.face_index as u32,
                    "the same face at {target:?} along {direction:?}"
                );
                let gpu = walk.picture_at(&gpu_hit);
                match cpu {
                    None => {
                        assert_eq!(gpu, [1.0; 3], "no picture, exactly 1");
                        plain += 1;
                    }
                    Some(factor) => {
                        for channel in 0..3 {
                            let gap = (f64::from(gpu[channel]) - factor[channel]).abs();
                            worst = worst.max(gap);
                        }
                        pictured += 1;
                    }
                }
            }
        }
    }
    assert!(worst <= 3e-5, "the GPU's factor is {worst} from the CPU's");
    assert!(
        pictured > 200 && plain > 20,
        "{pictured} pictured, {plain} plain"
    );
}

/// One box of a pictured scene: its size, center and surface.
type PicturedPiece = ([f64; 3], [f64; 3], ScatteringSurface);

/// The CPU scene of `pieces` over the wide matte floor and the GPU pack of `pieces` over its
/// analytic floor at z = 0, every face of every piece showing `texture` at coordinates that run
/// across each face (several repeats of the picture over a box).
fn pictured_pair(
    pieces: &[PicturedPiece],
    texture: &TraceTexture,
) -> (RenderScene, GpuRenderScene) {
    let meshes: Vec<Mesh> = pieces
        .iter()
        .map(|&(size, center, _)| {
            let mut mesh = box_meshes(&[(size, center)])[0].clone();
            let at = |v: i32| {
                let p = Vector3::from_vector3_float(mesh.vertices[v as usize]);
                Vector2Float::new(
                    0.07 * p.x + 0.05 * p.z + 0.31,
                    0.09 * p.y - 0.04 * p.z - 1.7,
                )
            };
            let corners: Vec<_> = mesh
                .faces
                .iter()
                .map(|f| (at(f.v0), at(f.v1), at(f.v2)))
                .collect();
            for (face, (a, b, c)) in corners.into_iter().enumerate() {
                mesh.face_textures
                    .insert(face as i32, FaceTextureData::new(None, a, b, c));
            }
            mesh
        })
        .collect();
    let scene_of = |with_floor: bool| {
        let mut placed: Vec<PlacedPart> = meshes
            .iter()
            .map(|mesh| part_of(mesh, Matrix4X4::IDENTITY))
            .collect();
        let mut surfaces: Vec<ScatteringSurface> = pieces.iter().map(|piece| piece.2).collect();
        let mut faces: Vec<Vec<Option<u32>>> = meshes
            .iter()
            .map(|mesh| vec![Some(0); mesh.faces.len()])
            .collect();
        if with_floor {
            placed.push(part_of(&wide_floor(), Matrix4X4::IDENTITY));
            surfaces.push(ScatteringSurface::matte_plastic([0.3; 3]));
            faces.push(Vec::new());
        }
        RenderScene::new(PlacedParts::new(placed), surfaces, furnace())
            .unwrap()
            .with_face_textures(vec![Arc::new(texture.clone())], faces)
            .unwrap()
    };
    let camera = RenderCamera::orthographic(
        Vector3::new(0.0, -80.0, 60.0),
        Vector3::new(0.0, 0.0, 10.0),
        Vector3::UNIT_Z,
        40.0,
    )
    .unwrap();
    let view = GpuView {
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
    };
    let limits = GpuSceneLimits {
        max_storage_binding: WEBGPU_DEFAULT_MAX_STORAGE_BINDING,
        max_vertex_step: 1e-3,
    };
    let pack = GpuRenderScene::pack_view(&scene_of(false), &view, limits).unwrap();
    (scene_of(true), pack)
}

/// Holds the two lines to agree with pictures on the faces they cross, reached and crossed
/// exactly, and answers, when the line reached the sun, the CPU's transmittance, its crossings and
/// the largest gap between the two transmittances in a channel.
pub(super) fn agree_pictured(cpu: &SunLineAt, gpu: &SunLineF32) -> Option<([f64; 3], u32, f64)> {
    let Some((transmittance, crossed)) = cpu.reach else {
        assert!(
            !gpu.reached,
            "the CPU's line is blocked, the GPU's reached: {gpu:?}"
        );
        return None;
    };
    assert!(
        gpu.reached,
        "the CPU's line reached the sun, the GPU's is blocked"
    );
    assert_eq!(gpu.crossed, crossed, "faces of glass crossed");
    let gap = (0..3).fold(0.0_f64, |worst, channel| {
        worst.max((f64::from(gpu.transmittance[channel]) - transmittance[channel]).abs())
    });
    Some((transmittance, crossed, gap))
}

/// How far the line's transmittance may be from the CPU's through pictured glass, a crossing. Each
/// route's line leaves its own start (the `f32` vertex and its offset against the CPU's `f64`
/// ones, up to about 2e-5 apart here, 30 out where an `f32` step is 1.9e-6), so at 50 degrees the
/// two cross a face up to about 3e-5 apart; the coordinates move by 0.14 of that (their gradient
/// over these boxes), the texel position by the picture's side (6) times that, the factor by at
/// most that, and the crossing's tint, the square root of the base color times the factor, by
/// half: about 1.3e-5, on top of the plain line's 2e-7, which the bound rounds up to 1.5e-5. The
/// worst measured is 4.7e-6 a crossing through a pane and 8e-7 from under resting glass.
pub(super) const PICTURED_BOUND_PER_CROSSING: f64 = 1.5e-5;

#[test]
fn a_line_through_a_pictured_pane_is_tinted_as_the_cpu_line_is() {
    // A pane 2 thick lifted 10 over the floor, its faces showing a picture: the line from the
    // floor crosses its bottom and its top, each tinted by its own spot of the picture.
    let texture = picture(5, 4, 11);
    let tint = [0.9, 0.8, 0.7];
    let (scene, pack) = pictured_pair(
        &[(
            [60.0, 60.0, 2.0],
            [0.0, 0.0, 11.0],
            glass_of(tint, 0.0, 1.5),
        )],
        &texture,
    );
    let (mut tinted_by_picture, mut worst) = (0, 0.0_f64);
    for (x, y) in [(0.0, 0.0), (3.7, -5.1), (-11.3, 7.9), (17.2, 4.4)] {
        let vertex = gpu_floor_vertex(&pack, x, y);
        let cpu_ray = down_from(Vector3::new(x, y, 5.0));
        for direction in sun_directions() {
            let (cpu, gpu) = both(
                &scene,
                &pack,
                (&cpu_ray, &[], false),
                &vertex,
                direction,
                false,
                &random(),
            );
            let (transmittance, crossed, gap) =
                agree_pictured(&cpu, &gpu).expect("the pane passes the sun");
            assert_eq!(crossed, 2);
            worst = worst.max(gap);
            if direction.z == 1.0 {
                let fresnel = (0.5_f64 / 2.5).powi(2);
                let plain = (1.0 - fresnel).powi(2);
                if (0..3).any(|c| (transmittance[c] - plain * tint[c]).abs() > 1e-3) {
                    tinted_by_picture += 1;
                }
            }
        }
    }
    assert_eq!(
        tinted_by_picture, 4,
        "the picture tints every straight-up line"
    );
    assert!(
        worst <= 2.0 * PICTURED_BOUND_PER_CROSSING,
        "the GPU's line passes {worst} more or less than the CPU's"
    );
}

#[test]
fn a_line_from_under_pictured_glass_resting_on_the_floor_is_tinted_by_the_glass_across() {
    // A box of side 20 resting on the floor, its faces showing a picture: from the floor under it
    // the line crosses the bottom across the contact, read from the vertex's own lookup, then the
    // top.
    let texture = picture(4, 6, 23);
    let (scene, pack) = pictured_pair(
        &[([20.0; 3], [0.0, 0.0, 10.0], ScatteringSurface::glass())],
        &texture,
    );
    let mut worst = 0.0_f64;
    for (x, y) in [(0.0, 0.0), (6.1, -4.4), (-7.7, 2.9)] {
        let inside = Vector3::new(x, y, 5.0);
        let under = gpu_floor_under(&pack, inside);
        for direction in sun_directions() {
            let (cpu, gpu) = both(
                &scene,
                &pack,
                (&down_from(inside), &[], true),
                &under,
                direction,
                false,
                &random(),
            );
            let reach = agree_pictured(&cpu, &gpu);
            assert_eq!(reach.map(|(_, crossed, _)| crossed), Some(2));
            worst = worst.max(reach.map_or(0.0, |(_, _, gap)| gap));
        }
    }
    assert!(
        worst <= 2.0 * PICTURED_BOUND_PER_CROSSING,
        "the GPU's line passes {worst} more or less than the CPU's"
    );
}
