//! The GPU path loop's picture atlas (`flat_tree/gpu_textures.rs`): pictures over the texel
//! budget shrink largest first (one past the largest side before any) to exactly what
//! `TraceTexture::shrunk` gives the CPU, halving averages light and keeps a see-through texel's
//! color out of its neighbours, the fallible shrink is the same picture, the layout breaks ties
//! by index, a scene whose pictures cannot fit even at one texel each is refused, the plan agrees
//! with the built atlas, a taken atlas says so, and packing twice gives the same bytes.

use solidmesh_mesh::ray_tracer::flat_tree::{
    fit_texture_atlas, plan_texture_atlas, FlatTreeError, GpuTextureLimits, ShrunkTexture,
    TEXTURE_PICTURE_SIZE, TEXTURE_PICTURE_WORDS,
};
use solidmesh_mesh::ray_tracer::TraceTexture;

use crate::gpu_texture_pack_tests::{pack_with, picture, pictured_scene, region, try_pack, words};

/// Three pictures of 336 texels over a 200-texel budget: the largest, which is not the first,
/// is halved once (its 256 texels to 64), and then all three fit. The report names it, and the
/// atlas holds exactly `TraceTexture::shrunk` of it and the other two as given.
#[test]
fn pictures_over_the_budget_halve_the_largest_first_to_what_the_cpu_shrinks() {
    let textures = vec![picture(8, 8, 3), picture(16, 16, 4), picture(4, 4, 5)];
    let limits = GpuTextureLimits {
        max_texels: 200,
        max_side: 64,
    };
    let atlas = fit_texture_atlas(&textures, limits).expect("fits once shrunk");
    assert_eq!(
        atlas.shrunk(),
        &[ShrunkTexture {
            texture: 1,
            halvings: 1,
            width: 8,
            height: 8,
        }]
    );
    assert!(u64::from(atlas.width()) * u64::from(atlas.height()) <= limits.max_texels);
    assert!(atlas.width() <= limits.max_side && atlas.height() <= limits.max_side);
    let halved = textures[1].halved();
    assert_eq!(textures[1].shrunk(1), halved);
    assert_eq!((halved.width(), halved.height()), (8, 8));
    assert_eq!(region(&atlas, 1), halved.bytes(), "byte for byte");
    assert_eq!(region(&atlas, 0), textures[0].bytes());
    assert_eq!(region(&atlas, 2), textures[2].bytes());

    // The pack fits the same way and its picture records name the shrunk size.
    let (scene, _) = pictured_scene(textures);
    let pack = pack_with(&scene, limits);
    assert_eq!(pack.texture_atlas(), &atlas);
    let part_words = words(&pack.part_bytes());
    let first = part_words.len() - pack.texture_block().words().len();
    let record = first + TEXTURE_PICTURE_WORDS + TEXTURE_PICTURE_SIZE;
    assert_eq!(&part_words[record..record + 2], &[8, 8]);
}

/// A picture wider than the largest texture side is halved before a larger one that is not, as
/// it can never fit as it is.
#[test]
fn a_picture_past_the_largest_side_is_halved_first() {
    let textures = vec![picture(40, 40, 6), picture(70, 2, 7)];
    let limits = GpuTextureLimits {
        max_texels: 1 << 20,
        max_side: 64,
    };
    let atlas = fit_texture_atlas(&textures, limits).expect("fits");
    assert_eq!(
        atlas.shrunk(),
        &[ShrunkTexture {
            texture: 1,
            halvings: 1,
            width: 35,
            height: 1,
        }]
    );
    assert_eq!(region(&atlas, 1), textures[1].halved().bytes());
}

/// Halving averages light, not sRGB codes, and weights each texel by its alpha: a 2 by 2 block
/// of white and black is the code whose decoded value is nearest one half (188, where averaging
/// codes would give 128), a red texel among three fully see-through ones keeps its red exactly
/// at a quarter of its alpha, and an odd side keeps its last column by averaging it with itself.
#[test]
fn halving_averages_light_and_keeps_see_through_colors_out() {
    let white = [255, 255, 255, 255];
    let black = [0, 0, 0, 255];
    let checker = TraceTexture::new(2, 2, [white, black, black, white].concat()).unwrap();
    assert_eq!(checker.halved().bytes(), &[188, 188, 188, 255]);

    let clear_blue = [0, 0, 255, 0];
    let red = [255, 0, 0, 255];
    let edge = TraceTexture::new(2, 2, [red, clear_blue, clear_blue, clear_blue].concat()).unwrap();
    assert_eq!(edge.halved().bytes(), &[255, 0, 0, 64]);

    let row = TraceTexture::new(3, 1, [black, black, white].concat()).unwrap();
    let halved = row.halved();
    assert_eq!((halved.width(), halved.height()), (2, 1));
    assert_eq!(halved.bytes(), &[0, 0, 0, 255, 255, 255, 255, 255]);

    let one = TraceTexture::new(1, 1, red.to_vec()).unwrap();
    assert_eq!(one.halved(), one);
    assert_eq!(one.shrunk(5), one);
}

/// Pictures that cannot fit even shrunk to one texel each are refused, saying what would help;
/// a pack of such a scene is refused the same way.
#[test]
fn pictures_that_cannot_fit_at_one_texel_are_refused() {
    let textures = vec![picture(8, 8, 8), picture(4, 4, 9), picture(2, 2, 10)];
    let limits = GpuTextureLimits {
        max_texels: 2,
        max_side: 64,
    };
    let expected = FlatTreeError::TexturesTooLarge {
        pictures: 3,
        max_texels: 2,
        max_side: 64,
    };
    assert_eq!(fit_texture_atlas(&textures, limits), Err(expected.clone()));
    let words = expected.to_string();
    assert!(words.contains("render it on the CPU"), "{words}");
    let (scene, _) = pictured_scene(textures);
    assert_eq!(try_pack(&scene, limits), Err(expected));
    let no_side = GpuTextureLimits {
        max_texels: 1 << 20,
        max_side: 0,
    };
    assert!(matches!(
        fit_texture_atlas(&[picture(1, 1, 11)], no_side),
        Err(FlatTreeError::TexturesTooLarge { .. })
    ));
}

/// The same scene packs to the same bytes every time, shrinking and all.
#[test]
fn packing_pictures_twice_gives_the_same_bytes() {
    let textures = || vec![picture(9, 5, 12), picture(16, 3, 13), picture(6, 6, 14)];
    let limits = GpuTextureLimits {
        max_texels: 100,
        max_side: 32,
    };
    let (first_scene, _) = pictured_scene(textures());
    let (second_scene, _) = pictured_scene(textures());
    let first = pack_with(&first_scene, limits);
    let second = pack_with(&second_scene, limits);
    assert!(!first.texture_atlas().shrunk().is_empty(), "shrinking ran");
    assert_eq!(first.face_bytes(), second.face_bytes());
    assert_eq!(first.part_bytes(), second.part_bytes());
    assert_eq!(first.texture_atlas(), second.texture_atlas());
    assert_eq!(
        fit_texture_atlas(&textures(), limits),
        fit_texture_atlas(&textures(), limits)
    );
}

/// Pictures of the same size lie in index order, and of the same height the wider first.
#[test]
fn the_layout_breaks_ties_by_index_after_height_and_width() {
    let same = vec![picture(4, 4, 15), picture(4, 4, 16), picture(4, 4, 17)];
    let atlas = fit_texture_atlas(&same, GpuTextureLimits::DEFAULT).expect("fits");
    let origins: Vec<[u32; 2]> = atlas.placements().iter().map(|p| [p.x, p.y]).collect();
    assert_eq!(origins, [[0, 0], [0, 4], [0, 8]]);
    let wide_second = vec![picture(3, 4, 18), picture(5, 4, 19)];
    let atlas = fit_texture_atlas(&wide_second, GpuTextureLimits::DEFAULT).expect("fits");
    assert_eq!([atlas.placements()[1].x, atlas.placements()[1].y], [0, 0]);
}

/// Taking the atlas's bytes out of a pack leaves its size, placements and shrink report, and the
/// sizes still count the texture; the plan alone gives the same layout and shrink report as
/// building the atlas, and builds only the pictures it was made for.
#[test]
fn the_atlas_bytes_can_be_taken_and_the_plan_reads_no_texels() {
    let textures = vec![picture(8, 8, 3), picture(16, 16, 4), picture(4, 4, 5)];
    let limits = GpuTextureLimits {
        max_texels: 200,
        max_side: 64,
    };
    let plan = plan_texture_atlas(&textures, limits).expect("a plan");
    let built = plan.build(&textures).expect("an atlas");
    assert_eq!(built, fit_texture_atlas(&textures, limits).expect("fits"));
    assert_eq!(plan.placements(), built.placements());
    assert_eq!(plan.shrunk(), built.shrunk());
    assert_eq!(
        (plan.width(), plan.height()),
        (built.width(), built.height())
    );
    assert_eq!(plan.byte_len(), built.bytes().expect("bytes").len() as u64);
    assert!(plan.build(&textures[..2]).is_err(), "only its own pictures");

    let (scene, _) = pictured_scene(textures);
    let mut pack = pack_with(&scene, limits);
    let bytes = pack.take_texture_atlas_bytes().expect("the bytes, once");
    assert_eq!(Some(&bytes[..]), built.bytes());
    assert_eq!(pack.texture_atlas().bytes(), None, "a taken atlas says so");
    assert_eq!(
        pack.take_texture_atlas_bytes(),
        None,
        "and gives nothing twice"
    );
    assert_eq!(pack.texture_atlas().byte_len(), bytes.len() as u64);
    assert_eq!(pack.texture_atlas().placements(), built.placements());
    assert_eq!(pack.texture_atlas().shrunk(), built.shrunk());
    assert_eq!(pack.sizes().atlas_bytes, bytes.len() as u64);
}

/// The fallible shrink is the infallible one, and a texture budget of nothing still bounds the
/// 1 by 1 atlas a scene without pictures gets.
#[test]
fn the_fallible_shrink_matches_and_the_bound_covers_the_white_atlas() {
    let texture = picture(13, 6, 21);
    assert_eq!(texture.try_halved().expect("memory"), texture.halved());
    assert_eq!(texture.try_shrunk(0).expect("memory"), texture);
    assert_eq!(texture.try_shrunk(3).expect("memory"), texture.shrunk(3));
    let nothing = GpuTextureLimits {
        max_texels: 0,
        max_side: 0,
    };
    assert_eq!(nothing.max_atlas_bytes(), 4);
    let empty = fit_texture_atlas::<TraceTexture>(&[], nothing).expect("the white atlas");
    assert_eq!(empty.byte_len(), nothing.max_atlas_bytes());
}
