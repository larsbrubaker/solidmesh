//! Pictures packed for the GPU path loop (`flat_tree/gpu_textures.rs`, `gpu_texture_block.rs`):
//! a scene without pictures packs the same bytes as before the face record grew its picture word,
//! the shader's constants for the picture block are the Rust ones, the packed lookup in `f32`
//! reads the texels the CPU's lookup reads with the same weights and factor (pictures of
//! different sizes at different atlas origins, coordinates well outside 0..1, a large picture a
//! thousand repeats out, a face centred on 0), the block refuses a storage binding it passes, and
//! the mirror refuses words that point nowhere. The atlas itself is `gpu_texture_atlas_tests`.

use std::sync::Arc;

use naga::Literal;
use solidmesh_math::{Color, Matrix4X4, Vector2Float, Vector3};
use solidmesh_mesh::ray_tracer::flat_tree::{
    texture_lookup_f32, AtlasPlacement, FlatTreeError, GpuRenderScene, GpuSceneLimits,
    GpuTextureAtlas, GpuTextureLimits, GpuView, FACE_STRIDE, NO_TEXTURE, PATH_WGSL,
    TEXTURE_FACE_PICTURE, TEXTURE_FACE_WORDS, TEXTURE_PICTURE_SIZE, TEXTURE_PICTURE_WORDS,
    WEBGPU_DEFAULT_MAX_STORAGE_BINDING,
};
use solidmesh_mesh::ray_tracer::{
    MeshSurface, PathTraceSettings, PlacedPart, PlacedParts, RenderCamera, RenderScene,
    ScatteringSurface, TraceTexture,
};
use solidmesh_mesh::solids::platonic_solids::create_cube;
use solidmesh_mesh::{FaceTextureData, Mesh};

use crate::flat_tree_shader_tests::literal;
use crate::path_tracer_glass_contact_tests::{boxes_scene, furnace};
use crate::path_tracer_support::{flat_patch, scene_of, SHARP_CREASE};
use crate::surface_hit_support::tree_of;

const LIMITS: GpuSceneLimits = GpuSceneLimits {
    max_storage_binding: WEBGPU_DEFAULT_MAX_STORAGE_BINDING,
    max_vertex_step: 1e-3,
};

/// The bytes a face record held before the picture word: each record less its last word, which
/// must be the new one.
const OLD_FACE_STRIDE: usize = 60;

pub(super) fn try_pack(
    scene: &RenderScene,
    texture_limits: GpuTextureLimits,
) -> Result<GpuRenderScene, FlatTreeError> {
    try_pack_in(scene, LIMITS, texture_limits)
}

pub(super) fn try_pack_in(
    scene: &RenderScene,
    limits: GpuSceneLimits,
    texture_limits: GpuTextureLimits,
) -> Result<GpuRenderScene, FlatTreeError> {
    let camera = RenderCamera::orthographic(
        Vector3::new(0.0, -20.0, 20.0),
        Vector3::ZERO,
        Vector3::UNIT_Z,
        10.0,
    )
    .expect("a camera");
    let view = GpuView::plain(camera, 4, 4, *scene.lights(), PathTraceSettings::default());
    GpuRenderScene::pack_view_with_texture_limits(scene, &view, limits, texture_limits)
}

pub(super) fn pack_with(scene: &RenderScene, texture_limits: GpuTextureLimits) -> GpuRenderScene {
    try_pack(scene, texture_limits).expect("the scene packs")
}

fn fnv(bytes: impl IntoIterator<Item = u8>) -> u64 {
    let mut hash = 0xcbf2_9ce4_8422_2325_u64;
    for byte in bytes {
        hash ^= u64::from(byte);
        hash = hash.wrapping_mul(0x0000_0100_0000_01b3);
    }
    hash
}

pub(super) fn words(bytes: &[u8]) -> Vec<u32> {
    bytes
        .as_chunks::<4>()
        .0
        .iter()
        .map(|w| u32::from_le_bytes(*w))
        .collect()
}

/// `width` by `height` texels of varied color and alpha, the same for the same seed.
pub(super) fn picture(width: u32, height: u32, seed: u32) -> TraceTexture {
    let mut state = seed.wrapping_mul(747_796_405).wrapping_add(2_891_336_453);
    let bytes = (0..width * height * 4)
        .map(|_| {
            state = state.wrapping_mul(1_664_525).wrapping_add(1_013_904_223);
            // The top byte of an LCG step.
            (state >> 24) as u8
        })
        .collect();
    TraceTexture::new(width, height, bytes).expect("a picture")
}

/// A square showing picture 0 on face 0 and picture 1 on face 1 (coordinates far outside 0..1,
/// among them u = 1.012, v = -1.3381), and a cube whose faces show the last picture, the first
/// six with coordinates and the rest with none (so they pack no entry). The meshes and their
/// placements come back too.
pub(super) fn pictured_scene(textures: Vec<TraceTexture>) -> (RenderScene, Vec<(Mesh, Matrix4X4)>) {
    let uv = Vector2Float::new;
    let mut square = flat_patch(1.0, 0.0);
    let square_uvs = [
        [uv(1.012, -1.3381), uv(3.25, -1.3381), uv(1.012, 0.77)],
        [uv(-0.4, 2.6), uv(1.7, -0.9), uv(3.3, 1.1)],
    ];
    for (face, [a, b, c]) in square_uvs.into_iter().enumerate() {
        square
            .face_textures
            .insert(face as i32, FaceTextureData::new(None, a, b, c));
    }
    let mut cube = create_cube(2.0, 2.0, 2.0);
    cube.face_colors = Some(vec![Color::rgba(200, 180, 160, 255); cube.faces.len()]);
    for face in 0..6 {
        let f = f64::from(face);
        cube.face_textures.insert(
            face,
            FaceTextureData::new(
                None,
                uv(0.1 + 0.37 * f, -2.2 + 0.5 * f),
                uv(1.9 - 0.21 * f, 0.3 * f),
                uv(-1.1 + 0.4 * f, 2.05 - 0.6 * f),
            ),
        );
    }
    let last = textures.len() as u32 - 1;
    let faces = vec![
        vec![Some(0), Some(1.min(last))],
        vec![Some(last); cube.faces.len()],
    ];
    let meshes = vec![
        (square, Matrix4X4::IDENTITY),
        (cube, Matrix4X4::create_translation(3.0, 0.0, 1.0)),
    ];
    let parts = meshes
        .iter()
        .map(|(mesh, matrix)| {
            PlacedPart::new(
                tree_of(mesh),
                Arc::new(MeshSurface::new(mesh, SHARP_CREASE).expect("a table")),
                *matrix,
            )
            .expect("placed")
        })
        .collect();
    let scene = RenderScene::new(
        PlacedParts::new(parts),
        vec![ScatteringSurface::matte_plastic([0.8, 0.7, 0.6]); 2],
        furnace(),
    )
    .expect("a scene")
    .with_face_textures(textures.into_iter().map(Arc::new).collect(), faces)
    .expect("pictures");
    (scene, meshes)
}

/// The texels of `atlas` that picture `index` covers, rows bottom first.
pub(super) fn region(atlas: &GpuTextureAtlas, index: usize) -> Vec<u8> {
    let place = atlas.placements()[index];
    let row = atlas.width() as usize * 4;
    (0..place.height as usize)
        .flat_map(|r| {
            let at = (place.y as usize + r) * row + place.x as usize * 4;
            atlas.bytes().expect("the atlas's bytes")[at..at + place.width as usize * 4].to_vec()
        })
        .collect()
}

#[test]
fn the_shader_reads_the_picture_block_with_the_rust_constants() {
    let module = naga::front::wgsl::parse_str(PATH_WGSL).expect("parses");
    assert_eq!(
        literal(&module, "PATH_NO_TEXTURE"),
        Literal::U32(NO_TEXTURE)
    );
    assert_eq!(
        literal(&module, "PATH_TEXTURE_FACE_PICTURE"),
        Literal::U32(TEXTURE_FACE_PICTURE as u32)
    );
    assert_eq!(
        literal(&module, "PATH_TEXTURE_PICTURE_SIZE"),
        Literal::U32(TEXTURE_PICTURE_SIZE as u32)
    );
}

/// Every buffer a scene without pictures packs is the bytes it packed when a face record was 60
/// bytes (digests taken from the library before the picture word existed), with the new word 0
/// and no picture block: one colored cube, and two glass boxes stacked so a contact block follows
/// the part records.
#[test]
fn a_scene_without_pictures_packs_the_bytes_it_did_before_pictures() {
    let mut cube = create_cube(2.0, 2.0, 2.0);
    cube.face_colors = Some(
        (0..cube.faces.len())
            .map(|i| Color::new(10 + i as i32, 20, 30, 255))
            .collect(),
    );
    let colored = scene_of(
        &cube,
        SHARP_CREASE,
        Matrix4X4::IDENTITY,
        ScatteringSurface::matte_plastic([0.25, 0.5, 0.75]),
        furnace(),
    );
    let clear = ScatteringSurface::glass();
    let stacked = boxes_scene(
        &[
            ([2.0, 2.0, 2.0], [0.0, 0.0, 1.0], clear),
            ([2.0, 2.0, 2.0], [0.0, 0.0, 3.0], clear),
        ],
        false,
        furnace(),
    );
    let expected: [[u64; 3]; 2] = [
        [
            0x6fab_3886_b556_a7e5,
            0x34c6_a088_1c7d_d25f,
            0x3fdf_1d90_9cb6_ccb3,
        ],
        [
            0x8adf_6f1c_08a3_46ed,
            0x71f9_6b5d_7e0d_5101,
            0x2b7c_481c_e459_fa58,
        ],
    ];
    for (scene, expected) in [colored, stacked].iter().zip(expected) {
        let pack = pack_with(scene, GpuTextureLimits::DEFAULT);
        let faces = pack.face_bytes();
        assert_eq!(faces.len(), pack.faces().len() * FACE_STRIDE);
        let mut old = Vec::new();
        for record in faces.chunks(FACE_STRIDE) {
            old.extend_from_slice(&record[..OLD_FACE_STRIDE]);
            assert_eq!(&record[OLD_FACE_STRIDE..], &NO_TEXTURE.to_le_bytes());
        }
        assert_eq!(
            [fnv(old), fnv(pack.part_bytes()), fnv(pack.view_bytes())],
            expected
        );
        assert!(pack.texture_block().words().is_empty());
        let atlas = pack.texture_atlas();
        assert_eq!((atlas.width(), atlas.height()), (1, 1), "a bindable atlas");
        assert_eq!(atlas.bytes(), Some(&[255u8; 4][..]));
        assert!(atlas.placements().is_empty() && atlas.shrunk().is_empty());
    }
}

/// At points across every pictured face, the packed lookup in `f32` reads the very texels the
/// CPU's lookup reads (indices exactly, after taking off the picture's atlas origin), with
/// weights within `f32` rounding and the factor within 1e-5.
#[test]
fn the_packed_lookup_reads_the_texels_the_cpu_lookup_reads() {
    // Of equal height, the first two share a shelf only on a wider atlas: the least-area one
    // stacks them and puts the third beside the second, so pictures sit at origins with a
    // non-zero y and a non-zero x, which the lookup must add after wrapping.
    let textures = vec![picture(5, 4, 1), picture(3, 4, 2), picture(2, 2, 3)];
    let (scene, meshes) = pictured_scene(textures);
    let pack = pack_with(&scene, GpuTextureLimits::DEFAULT);
    let part_words = words(&pack.part_bytes());
    let atlas = pack.texture_atlas();
    assert!(atlas.shrunk().is_empty(), "all fit as they are");
    let place = |x, y, width, height| AtlasPlacement {
        x,
        y,
        width,
        height,
    };
    assert_eq!(
        atlas.placements(),
        &[place(0, 0, 5, 4), place(0, 4, 3, 4), place(3, 4, 2, 2)]
    );
    assert_eq!((atlas.width(), atlas.height()), (5, 8));
    for (index, place) in atlas.placements().iter().enumerate() {
        let texture = &scene.textures()[index];
        assert_eq!(region(atlas, index), texture.bytes());
        assert_eq!(
            (place.width, place.height),
            (texture.width(), texture.height())
        );
    }
    let weights = [
        [1.0, 0.0, 0.0],
        [0.0, 1.0, 0.0],
        [0.0, 0.0, 1.0],
        [0.2, 0.5, 0.3],
        [0.61, 0.13, 0.26],
        [0.05, 0.05, 0.9],
        [0.34, 0.33, 0.33],
    ];
    let mut looked_up = 0;
    let mut triangle = 0;
    for (part_index, (part, (mesh, matrix))) in
        scene.parts().parts().iter().zip(&meshes).enumerate()
    {
        for face in 0..mesh.faces.len() {
            let record = pack.faces()[triangle];
            triangle += 1;
            let picture_index = scene
                .face_texture(part_index, face)
                .expect("every face here names a picture");
            let f = mesh.faces[face];
            let corners = [f.v0, f.v1, f.v2]
                .map(|v| Vector3::from_vector3_float(mesh.vertices[v as usize]))
                .map(|c| c.transform_position(*matrix));
            if !mesh.face_textures.contains_key(&(face as i32)) {
                assert_eq!(record.texture, NO_TEXTURE, "no coordinates, no entry");
                continue;
            }
            let entry = record.texture as usize;
            let picture_record = part_words[entry + TEXTURE_FACE_PICTURE] as usize;
            let place = atlas.placements()[picture_index as usize];
            assert_eq!(
                &part_words[picture_record..picture_record + TEXTURE_PICTURE_WORDS],
                &[place.x, place.y, place.width, place.height]
            );
            let texture = &scene.textures()[picture_index as usize];
            for w in weights {
                let at = corners[0] * w[0] + corners[1] * w[1] + corners[2] * w[2];
                let point = part.surface_point(face as i32, at).expect("a point");
                let uv = point.texture_coordinate.expect("coordinates");
                let spot = texture.spot(uv);
                let factor = scene
                    .texture_factor_at(picture_index, uv)
                    .expect("a factor");
                // Narrowed once, as the shader receives its hit's weights.
                let narrow = point.barycentric.map(|b| b as f32);
                let gpu = texture_lookup_f32(
                    &part_words,
                    record.texture,
                    narrow,
                    atlas.bytes().expect("the atlas's bytes"),
                    atlas.width(),
                    &pack.srgb_table_bytes(),
                )
                .expect("a lookup");
                let local = gpu.texels.map(|[x, y]| [x - place.x, y - place.y]);
                assert_eq!(
                    local,
                    [
                        [spot.columns[0], spot.rows[0]],
                        [spot.columns[1], spot.rows[0]],
                        [spot.columns[0], spot.rows[1]],
                        [spot.columns[1], spot.rows[1]],
                    ],
                    "part {part_index} face {face} at {w:?}, uv {uv:?}"
                );
                // The coordinate's own f32 rounding, a few steps of its size, times the
                // picture's side.
                let size = f64::from(texture.width().max(texture.height()));
                let tolerance =
                    8.0 * f64::from(f32::EPSILON) * (1.0 + uv.x.abs().max(uv.y.abs())) * size;
                assert!(
                    (f64::from(gpu.across) - spot.across).abs() <= tolerance
                        && (f64::from(gpu.up) - spot.up).abs() <= tolerance,
                    "weights {} {} against {} {}",
                    gpu.across,
                    gpu.up,
                    spot.across,
                    spot.up
                );
                for channel in 0..3 {
                    let gap = (f64::from(gpu.factor[channel]) - factor[channel]).abs();
                    assert!(gap <= 1e-5, "factor {:?} against {factor:?}", gpu.factor);
                }
                looked_up += 1;
            }
        }
    }
    // Two square faces and six of the cube's twelve carry coordinates.
    let entries = 8;
    assert_eq!(looked_up, entries * weights.len());
    assert_eq!(
        pack.texture_block().words().len(),
        3 * TEXTURE_PICTURE_WORDS + entries * TEXTURE_FACE_WORDS
    );
}

/// A part buffer with room for the faces but not for a hundred picture records says the pictures
/// are what to reduce; one with room for the records but not every face entry says the pictured
/// faces are, and how many fit.
#[test]
fn a_picture_block_past_the_storage_binding_says_what_to_reduce() {
    let many: Vec<TraceTexture> = (0..100).map(|seed| picture(1, 1, seed)).collect();
    let (scene, _) = pictured_scene(many);
    // 14 faces of 64 bytes fit in 1000; 2 part records, 100 picture records and 8 face entries
    // need 20 + 400 + 56 words.
    let tight = GpuSceneLimits {
        max_storage_binding: 1000,
        ..LIMITS
    };
    assert_eq!(
        try_pack_in(&scene, tight, GpuTextureLimits::DEFAULT).err(),
        Some(FlatTreeError::PackedBufferTooLarge {
            buffer: "part (with its pictures)",
            needed_bytes: 476 * 4,
            limit_bytes: 1000,
            what: "pictures",
            largest_that_fits: (250 - 20) / 4,
        })
    );
    let some: Vec<TraceTexture> = (0..50).map(|seed| picture(1, 1, seed)).collect();
    let (scene, _) = pictured_scene(some);
    // 20 + 200 records leave 30 of 250 words: four 7-word entries of the eight.
    assert_eq!(
        try_pack_in(&scene, tight, GpuTextureLimits::DEFAULT).err(),
        Some(FlatTreeError::PackedBufferTooLarge {
            buffer: "part (with its pictures)",
            needed_bytes: (20 + 200 + 56) * 4,
            limit_bytes: 1000,
            what: "pictured faces",
            largest_that_fits: 4,
        })
    );
}

/// The lookup mirror answers `None`, never panics, for words that point anywhere: past the
/// buffer, at a picture past the atlas, or with offsets that would overflow.
#[test]
fn the_lookup_mirror_refuses_words_that_point_nowhere() {
    let (scene, _) = pictured_scene(vec![picture(2, 2, 1)]);
    let pack = pack_with(&scene, GpuTextureLimits::DEFAULT);
    let srgb = pack.srgb_table_bytes();
    let atlas = pack.texture_atlas();
    let look = |words: &[u32], texture: u32| {
        texture_lookup_f32(
            words,
            texture,
            [0.2, 0.3, 0.5],
            atlas.bytes().expect("the atlas's bytes"),
            atlas.width(),
            &srgb,
        )
    };
    let words = words(&pack.part_bytes());
    let real = pack.faces()[0].texture;
    assert!(look(&words, real).is_some());
    assert_eq!(look(&words, NO_TEXTURE), None);
    assert_eq!(look(&words, u32::MAX), None);
    assert_eq!(look(&words, words.len() as u32 - 3), None);
    let mut broken = words.clone();
    let entry = real as usize;
    broken[entry + TEXTURE_FACE_PICTURE] = u32::MAX - 1;
    assert_eq!(look(&broken, real), None, "a record past the buffer");
    let record = words[entry + TEXTURE_FACE_PICTURE] as usize;
    let mut broken = words.clone();
    broken[record] = u32::MAX;
    assert_eq!(look(&broken, real), None, "an origin that overflows");
    let mut broken = words.clone();
    broken[record + 1] = 1 << 20;
    assert_eq!(look(&broken, real), None, "a row past the atlas");
    let mut broken = words.clone();
    broken[record] = atlas.width();
    assert_eq!(look(&broken, real), None, "a column past the atlas");
}

/// A 1024 by 1024 picture on a face whose coordinates are about a thousand repeats out. As the
/// mesh holds them, an `f32` coordinate there is coarse by about 6e-5, a sixteenth of a texel of
/// this picture; packed shifted by whole repeats (`gpu_texture_block.rs`), the lookup reads the
/// CPU's texels away from a texel edge, and its weights and factor keep to the rounding of a
/// face spanning a dozen repeats.
#[test]
fn a_large_picture_far_out_in_coordinates_keeps_its_texels() {
    let uv = Vector2Float::new;
    let corners = [
        uv(1234.567, -987.6543),
        uv(1241.25, -980.5),
        uv(1229.75, -990.125),
    ];
    let mut square = flat_patch(1.0, 0.0);
    for face in 0..2 {
        square.face_textures.insert(
            face,
            FaceTextureData::new(None, corners[0], corners[1], corners[2]),
        );
    }
    let part = PlacedPart::new(
        tree_of(&square),
        Arc::new(MeshSurface::new(&square, SHARP_CREASE).expect("a table")),
        Matrix4X4::IDENTITY,
    )
    .expect("placed");
    let scene = RenderScene::new(
        PlacedParts::new(vec![part]),
        vec![ScatteringSurface::matte_plastic([0.8; 3])],
        furnace(),
    )
    .expect("a scene")
    .with_face_textures(
        vec![Arc::new(picture(1024, 1024, 20))],
        vec![vec![Some(0); 2]],
    )
    .expect("a picture");
    let pack = pack_with(&scene, GpuTextureLimits::DEFAULT);
    let part_words = words(&pack.part_bytes());
    let record = pack.faces()[0];
    let texture = &scene.textures()[0];
    let part = &scene.parts().parts()[0];
    // Shifted, the coordinates are under 12: a few f32 steps of that, times the side.
    let tolerance = 8.0 * f64::from(f32::EPSILON) * 12.0 * 1024.0;
    let f = square.faces[0];
    let world =
        [f.v0, f.v1, f.v2].map(|v| Vector3::from_vector3_float(square.vertices[v as usize]));
    let (mut worst_weight, mut worst_factor, mut exact) = (0.0_f64, 0.0_f64, 0);
    for step in 0..50 {
        let a = 0.013 + 0.0191 * f64::from(step);
        let b = 0.9 - 0.017 * f64::from(step);
        let w = [a, b * (1.0 - a), (1.0 - a) * (1.0 - b)];
        let at = world[0] * w[0] + world[1] * w[1] + world[2] * w[2];
        let point = part.surface_point(0, at).expect("a point");
        let cpu_uv = point.texture_coordinate.expect("coordinates");
        let spot = texture.spot(cpu_uv);
        let factor = scene.texture_factor_at(0, cpu_uv).expect("a factor");
        let gpu = texture_lookup_f32(
            &part_words,
            record.texture,
            point.barycentric.map(|b| b as f32),
            pack.texture_atlas().bytes().expect("the atlas's bytes"),
            pack.texture_atlas().width(),
            &pack.srgb_table_bytes(),
        )
        .expect("a lookup");
        let near_edge = |t: f64| t < tolerance || t > 1.0 - tolerance;
        if near_edge(spot.across) || near_edge(spot.up) {
            continue;
        }
        exact += 1;
        assert_eq!(
            gpu.texels,
            [
                [spot.columns[0], spot.rows[0]],
                [spot.columns[1], spot.rows[0]],
                [spot.columns[0], spot.rows[1]],
                [spot.columns[1], spot.rows[1]],
            ],
            "uv {cpu_uv:?}"
        );
        worst_weight = worst_weight
            .max((f64::from(gpu.across) - spot.across).abs())
            .max((f64::from(gpu.up) - spot.up).abs());
        for (gpu_channel, cpu_channel) in gpu.factor.iter().zip(factor) {
            worst_factor = worst_factor.max((f64::from(*gpu_channel) - cpu_channel).abs());
        }
    }
    eprintln!("{exact} points: weights within {worst_weight:e}, factors within {worst_factor:e}");
    assert!(exact >= 40, "only {exact} points away from a texel edge");
    assert!(worst_weight <= tolerance, "weights off by {worst_weight}");
    assert!(worst_factor <= tolerance, "factors off by {worst_factor}");
}

/// A face's coordinates are shifted by the whole number nearest 0 at its midpoint: a face
/// spanning -500 to 500 packs exactly the mesh's coordinates, and one far out packs them less
/// that whole number, both reading the CPU's texels.
#[test]
fn a_face_centred_on_zero_packs_its_own_coordinates() {
    let uv = Vector2Float::new;
    let centred = [uv(-500.0, -500.0), uv(500.0, -480.0), uv(-490.0, 500.0)];
    let far = [
        uv(1234.567, -987.6543),
        uv(1241.25, -980.5),
        uv(1229.75, -990.125),
    ];
    let mut square = flat_patch(1.0, 0.0);
    for (face, corners) in [centred, far].into_iter().enumerate() {
        square.face_textures.insert(
            face as i32,
            FaceTextureData::new(None, corners[0], corners[1], corners[2]),
        );
    }
    let part = PlacedPart::new(
        tree_of(&square),
        Arc::new(MeshSurface::new(&square, SHARP_CREASE).expect("a table")),
        Matrix4X4::IDENTITY,
    )
    .expect("placed");
    let scene = RenderScene::new(
        PlacedParts::new(vec![part]),
        vec![ScatteringSurface::matte_plastic([0.8; 3])],
        furnace(),
    )
    .expect("a scene")
    .with_face_textures(vec![Arc::new(picture(6, 5, 22))], vec![vec![Some(0); 2]])
    .expect("a picture");
    let pack = pack_with(&scene, GpuTextureLimits::DEFAULT);
    let part_words = words(&pack.part_bytes());
    let packed = |face: usize| {
        let entry = pack.faces()[face].texture as usize;
        let at = |w: usize| f32::from_bits(part_words[entry + w]);
        [0, 1, 2].map(|corner| [at(2 * corner), at(2 * corner + 1)])
    };
    assert_eq!(packed(0), centred.map(|c| [c.x, c.y]), "not shifted at all");
    // u spans 1229.75 to 1241.25 (midpoint 1235.5), v -990.125 to -980.5 (midpoint -985.3125).
    assert_eq!(packed(1), far.map(|c| [c.x - 1235.0, c.y + 985.0]));
    let part = &scene.parts().parts()[0];
    let texture = &scene.textures()[0];
    for face in 0..2 {
        let f = square.faces[face];
        let world =
            [f.v0, f.v1, f.v2].map(|v| Vector3::from_vector3_float(square.vertices[v as usize]));
        let at = world[0] * 0.2 + world[1] * 0.45 + world[2] * 0.35;
        let point = part.surface_point(face as i32, at).expect("a point");
        let spot = texture.spot(point.texture_coordinate.expect("coordinates"));
        let gpu = texture_lookup_f32(
            &part_words,
            pack.faces()[face].texture,
            point.barycentric.map(|b| b as f32),
            pack.texture_atlas().bytes().expect("the atlas's bytes"),
            pack.texture_atlas().width(),
            &pack.srgb_table_bytes(),
        )
        .expect("a lookup");
        assert_eq!(
            gpu.texels[0],
            [spot.columns[0], spot.rows[0]],
            "face {face}"
        );
        assert_eq!(
            gpu.texels[3],
            [spot.columns[1], spot.rows[1]],
            "face {face}"
        );
    }
}
