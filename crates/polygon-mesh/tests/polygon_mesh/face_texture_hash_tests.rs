//! A textured mesh's long hash keys the stored assets saved designs point at, so it is pinned:
//! the faces, their UVs and each picture's size and sampled bytes must hash to the same value on
//! every build, whatever type the mesh holds its pictures through.

use std::sync::Arc;

use agg_path::image::ImageBuffer;
use polygon_mesh::solids::platonic_solids::create_cube;
use polygon_mesh::{FaceTextureData, Mesh};
use vector_math::Vector2Float;

/// A `width` by `height` picture whose every byte depends on its position and `seed`, so a
/// sampled byte that moved or changed changes the hash.
fn patterned_image(width: i32, height: i32, seed: u8) -> Arc<ImageBuffer> {
    let mut image = ImageBuffer::new(width, height);
    for (index, byte) in image.get_buffer_mut().iter_mut().enumerate() {
        // Wrapping on purpose: only a repeatable byte pattern is wanted.
        *byte = (index as u8).wrapping_mul(31).wrapping_add(seed);
    }
    image.mark_image_changed();
    Arc::new(image)
}

fn uv(x: f64, y: f64) -> Vector2Float {
    Vector2Float::new(x, y)
}

/// A cube wearing two pictures (one large enough that its bytes are sampled, one small enough to
/// hash whole) on alternating faces, plus one texture entry with no picture.
fn textured_cube() -> (Mesh, Arc<ImageBuffer>, Arc<ImageBuffer>) {
    let mut mesh = create_cube(10.0, 20.0, 30.0);
    let large = patterned_image(64, 48, 7);
    let small = patterned_image(5, 3, 101);
    for face in 0..mesh.faces.len() as i32 {
        let picture = match face % 3 {
            0 => Some(Arc::clone(&large)),
            1 => Some(Arc::clone(&small)),
            _ => None,
        };
        let shift = face as f64 * 0.125;
        mesh.face_textures.insert(
            face,
            FaceTextureData::new(
                picture.map(|image| image as _),
                uv(shift, 0.0),
                uv(1.0, shift),
                uv(0.5, 1.0 - shift),
            ),
        );
    }
    (mesh, large, small)
}

#[test]
fn a_textured_meshs_long_hash_is_pinned() {
    let (mesh, _, _) = textured_cube();
    assert_eq!(mesh.get_long_hash_code(), 10_198_547_439_505_253_836);
}

#[test]
fn a_face_texture_keeps_the_identity_of_the_picture_it_was_given() {
    let (mesh, large, small) = textured_cube();
    assert!(mesh.face_textures[&0].same_image(Some(&large)));
    assert!(mesh.face_textures[&1].same_image(Some(&small)));
    assert!(!mesh.face_textures[&0].same_image(Some(&small)));
    assert!(mesh.face_textures[&2].same_image(None::<&Arc<ImageBuffer>>));
    // An equal copy of the pixels is another picture.
    let copy = Arc::new(ImageBuffer::from_image(&large));
    assert!(!mesh.face_textures[&0].same_image(Some(&copy)));
}
