//! Picture work in bounded pieces, for a host without threads: a halving and a shrink stepped a
//! few texels at a time (`path_tracer/texture_halving.rs`), the atlas built a few units at a time
//! (`flat_tree/gpu_atlas_build.rs`) and a pack sliced through its atlas (`gpu_pack.rs`) give the
//! very bytes the one-call forms give, whatever the piece and however odd the pictures' sizes;
//! each step stays within its piece (an atlas or pack step within one halved texel past it); a
//! re-pack handed its plan builds no atlas and writes the same picture block; a plan made for
//! other pictures is refused; and a pack step after an error is refused as finished.

use std::sync::Arc;

use solidmesh_math::Vector3;
use solidmesh_mesh::ray_tracer::flat_tree::{
    fit_texture_atlas, plan_texture_atlas, FlatTreeError, GpuRenderScene, GpuSceneBuilder,
    GpuSceneLimits, GpuTextureAtlas, GpuTextureLimits, GpuView, TextureAtlasBuild,
    TextureAtlasPlan, ATLAS_WORK_PER_HALVED_TEXEL, WEBGPU_DEFAULT_MAX_STORAGE_BINDING,
};
use solidmesh_mesh::ray_tracer::{
    PathTraceSettings, RenderCamera, RenderScene, ScatteringSurface, TextureHalving,
    TextureShrinking, TraceTexture,
};

use crate::gpu_texture_pack_tests::{picture, pictured_scene};
use crate::path_tracer_glass_contact_tests::{boxes_scene, furnace};

const LIMITS: GpuSceneLimits = GpuSceneLimits {
    max_storage_binding: WEBGPU_DEFAULT_MAX_STORAGE_BINDING,
    max_vertex_step: 1e-3,
};

/// The pieces every picture stepper is cut into: one texel, two, odd sizes that end a row
/// part-way, one past a 37-wide halving's 19-texel row, and everything at once.
const PIECES: [usize; 7] = [1, 2, 7, 20, 31, 1000, usize::MAX];

/// The pieces an atlas build is cut into: those of [`PIECES`] and the sizes around one halved
/// texel's cost, where a step can only just afford one or cannot afford one at all.
const ATLAS_PIECES: [usize; 11] = [
    1,
    2,
    7,
    31,
    ATLAS_WORK_PER_HALVED_TEXEL - 1,
    ATLAS_WORK_PER_HALVED_TEXEL,
    ATLAS_WORK_PER_HALVED_TEXEL + 1,
    3 * ATLAS_WORK_PER_HALVED_TEXEL + 5,
    1000,
    100_000,
    usize::MAX,
];

/// Odd-sized pictures, a single texel, a row of three, a column of three, an even size that is
/// no power of two, and one whose texels are all or partly see-through (alpha 0 in every other
/// texel, the rest of `picture`'s varied alphas).
fn odd_pictures() -> Vec<TraceTexture> {
    let mut see_through = picture(9, 7, 41).bytes().to_vec();
    for texel in see_through.chunks_mut(4).step_by(2) {
        texel[3] = 0;
    }
    vec![
        picture(37, 23, 37),
        picture(1, 1, 38),
        picture(3, 1, 39),
        picture(1, 3, 40),
        picture(12, 14, 42),
        TraceTexture::new(9, 7, see_through).expect("a picture"),
    ]
}

/// A halving stepped in each piece is the one-call halving byte for byte, every step but the last
/// writes exactly its piece (so a step never runs past what it was given), and a finished halving
/// answers nothing more.
#[test]
fn a_halving_in_pieces_is_the_one_call_halving() {
    for source in odd_pictures() {
        let whole = source.try_halved().expect("memory");
        assert_eq!(whole, source.halved());
        for piece in PIECES {
            let mut halving = TextureHalving::new(&source);
            let total = halving.texel_count();
            let mut steps = 0u64;
            let halved = loop {
                steps += 1;
                if let Some(halved) = halving.step(piece).expect("memory") {
                    break halved;
                }
            };
            assert_eq!(
                halved,
                whole,
                "{}x{} in pieces of {piece}",
                source.width(),
                source.height()
            );
            // The piece fits a u64; a step writes exactly it until the last.
            assert_eq!(steps, total.div_ceil((piece as u64).min(total)));
            assert!(halving.is_finished());
            assert_eq!(halving.step(piece), Ok(None), "nothing after the picture");
        }
    }
}

/// The texels a shrink of `source` by `halvings` writes: the copy's, or each halving's in turn
/// until the picture is 1 by 1, as `TraceTexture::shrunk` stops.
fn shrink_texels(source: &TraceTexture, halvings: u32) -> u64 {
    let mut size = (source.width(), source.height());
    if halvings == 0 {
        return u64::from(size.0) * u64::from(size.1);
    }
    let mut texels = 0;
    for done in 0..halvings {
        if done > 0 && size == (1, 1) {
            break;
        }
        size = (size.0.div_ceil(2), size.1.div_ceil(2));
        texels += u64::from(size.0) * u64::from(size.1);
    }
    texels
}

/// A shrink by 0 (a copy), 1, 2, 3 and 9 halvings (past 1 by 1) stepped in each piece, the
/// source shared as a scene shares it, is the one-call shrink byte for byte; no step writes more
/// texels than it was given, across the end of one halving and the start of the next too, and the
/// steps together write each halving's texels once.
#[test]
fn a_shrink_in_pieces_is_the_one_call_shrink() {
    for source in odd_pictures() {
        let source = Arc::new(source);
        for halvings in [0, 1, 2, 3, 9] {
            let whole = source.try_shrunk(halvings).expect("memory");
            assert_eq!(whole, source.shrunk(halvings));
            for piece in PIECES {
                let mut shrinking = TextureShrinking::new(Arc::clone(&source), halvings);
                let shrunk = loop {
                    let before = shrinking.texels_written();
                    let shrunk = shrinking.step(piece).expect("memory");
                    // A piece fits a u64.
                    assert!(shrinking.texels_written() - before <= piece as u64);
                    if let Some(shrunk) = shrunk {
                        break shrunk;
                    }
                };
                let size = format!("{}x{}", source.width(), source.height());
                assert_eq!(
                    shrunk, whole,
                    "{size} {halvings} halvings in pieces of {piece}"
                );
                assert_eq!(
                    shrinking.texels_written(),
                    shrink_texels(&source, halvings),
                    "{size} {halvings} halvings in pieces of {piece}"
                );
                assert!(shrinking.is_finished());
                assert_eq!(shrinking.step(piece), Ok(None), "nothing after the picture");
            }
        }
    }
}

/// `textures` built to `plan` in pieces of `piece`: each step ends within one halved texel's cost
/// of its piece and the work done ends at the plan's count; a build stepped after its atlas is
/// refused.
fn build_in_pieces(
    plan: &Arc<TextureAtlasPlan>,
    textures: &[TraceTexture],
    piece: usize,
) -> GpuTextureAtlas {
    let mut build = TextureAtlasBuild::new(Arc::clone(plan));
    let atlas = loop {
        let before = build.work_done();
        let atlas = build.step(textures, piece).expect("memory");
        let spent = build.work_done() - before;
        // Pieces and costs fit a u64: a step ends within one halved texel of its piece.
        let most = (piece as u64).saturating_add(ATLAS_WORK_PER_HALVED_TEXEL as u64 - 1);
        assert!(spent >= 1, "a step of {piece} did nothing");
        assert!(spent <= most, "a step of {piece} did {spent}");
        if let Some(atlas) = atlas {
            break atlas;
        }
    };
    assert_eq!(build.work_done(), plan.build_work(), "in pieces of {piece}");
    assert!(matches!(
        build.step(textures, piece),
        Err(FlatTreeError::NotRepresentable { .. })
    ));
    atlas
}

/// Odd-sized pictures over budgets the plan meets by halving: once one picture once, and once
/// two pictures, one of them three times. The atlas built in each piece is the one-call atlas byte
/// for byte (`build_in_pieces` checks each step's work), and a build stepped with other pictures
/// is refused.
#[test]
fn an_atlas_in_pieces_is_the_one_call_atlas() {
    let one_halved = vec![picture(9, 5, 51), picture(37, 23, 52), picture(3, 1, 53)];
    let several_halved = vec![
        picture(37, 23, 54),
        picture(12, 14, 55),
        picture(1, 1, 56),
        picture(3, 1, 57),
    ];
    let cases = [(one_halved.clone(), 400), (several_halved, 60)];
    for (case, (textures, max_texels)) in cases.into_iter().enumerate() {
        let limits = GpuTextureLimits {
            max_texels,
            max_side: 64,
        };
        let plan = Arc::new(plan_texture_atlas(&textures, limits).expect("a plan"));
        let halvings: Vec<(u32, u32)> = plan
            .shrunk()
            .iter()
            .map(|s| (s.texture, s.halvings))
            .collect();
        if case == 0 {
            assert_eq!(halvings, [(1, 1)], "the largest picture is halved once");
        } else {
            assert!(halvings.len() >= 2, "two pictures are halved: {halvings:?}");
            assert!(
                halvings.iter().any(|&(_, count)| count >= 3),
                "one is halved three times: {halvings:?}"
            );
        }
        let whole = fit_texture_atlas(&textures, limits).expect("fits");
        assert_eq!(plan.build(&textures).expect("an atlas"), whole);
        for piece in ATLAS_PIECES {
            let atlas = build_in_pieces(&plan, &textures, piece);
            assert_eq!(atlas, whole, "case {case} in pieces of {piece}");
        }
    }

    let plan = Arc::new(
        plan_texture_atlas(
            &one_halved,
            GpuTextureLimits {
                max_texels: 400,
                max_side: 64,
            },
        )
        .expect("a plan"),
    );
    let mut build = TextureAtlasBuild::new(Arc::clone(&plan));
    assert!(matches!(
        build.step(&one_halved[..2], 1),
        Err(FlatTreeError::NotRepresentable { .. })
    ));
    let resized = vec![picture(9, 5, 51), picture(36, 23, 52), picture(3, 1, 53)];
    assert!(!plan.fits_sizes_of(&resized));
    assert!(matches!(
        build.step(&resized, 1),
        Err(FlatTreeError::NotRepresentable { .. })
    ));
}

fn view_of(scene: &RenderScene) -> GpuView {
    let camera = RenderCamera::orthographic(
        Vector3::new(0.0, -20.0, 20.0),
        Vector3::ZERO,
        Vector3::UNIT_Z,
        10.0,
    )
    .expect("a camera");
    GpuView::plain(camera, 4, 4, *scene.lights(), PathTraceSettings::default())
}

/// `builder` stepped `units` at a time to its pack, with how many steps that took.
fn step_to_end(mut builder: GpuSceneBuilder<'_>, units: usize) -> (GpuRenderScene, u64, usize) {
    let mut steps = 0;
    loop {
        steps += 1;
        if let Some(pack) = builder.step(units).expect("the scene packs") {
            return (pack, builder.atlas_work_done(), steps);
        }
    }
}

const HALVING_LIMITS: GpuTextureLimits = GpuTextureLimits {
    max_texels: 400,
    max_side: 64,
};

/// A pack whose atlas halves a picture, sliced one unit and seven units at a time, is the pack
/// built in one go (faces, parts, view, picture block and atlas bytes), its atlas work the plan's;
/// the tree's step builds no atlas texels, which start in a step of their own.
#[test]
fn a_pack_sliced_through_its_atlas_is_the_pack_in_one_go() {
    let textures = vec![picture(9, 5, 61), picture(37, 23, 62), picture(3, 1, 63)];
    let (scene, _) = pictured_scene(textures.clone());
    let view = view_of(&scene);
    let whole =
        GpuRenderScene::pack_view_with_texture_limits(&scene, &view, LIMITS, HALVING_LIMITS)
            .expect("the scene packs");
    assert!(
        !whole.texture_atlas().shrunk().is_empty(),
        "the atlas halves a picture"
    );
    let plan = plan_texture_atlas(&textures, HALVING_LIMITS).expect("a plan");
    for units in [1, 7] {
        let builder = GpuSceneBuilder::new(&scene, view, LIMITS)
            .expect("a builder")
            .with_texture_limits(HALVING_LIMITS);
        let (sliced, work, _) = step_to_end(builder, units);
        assert_eq!(sliced, whole, "sliced {units} units at a time");
        assert_eq!(sliced.face_bytes(), whole.face_bytes());
        assert_eq!(sliced.part_bytes(), whole.part_bytes());
        assert_eq!(sliced.view_bytes(), whole.view_bytes());
        assert_eq!(
            sliced.texture_atlas().bytes(),
            whole.texture_atlas().bytes()
        );
        assert_eq!(work, plan.build_work());
    }

    let mut builder = GpuSceneBuilder::new(&scene, view, LIMITS)
        .expect("a builder")
        .with_texture_limits(HALVING_LIMITS);
    while !builder.next_step_builds_tree() {
        assert_eq!(builder.step(1), Ok(None));
    }
    assert_eq!(
        builder.step(usize::MAX),
        Ok(None),
        "the tree's step ends at the atlas"
    );
    assert_eq!(builder.atlas_work_done(), 0);
    let pack = builder.step(usize::MAX).expect("packs").expect("the pack");
    assert_eq!(pack, whole);
}

/// A stepped pack spends its atlas work within the units each step is given, plus less than one
/// halved texel: the largest step's atlas work stays under the piece and one halved texel's cost,
/// so neither the halving of the 37 by 23 picture nor the whole atlas lands in one step, and
/// the pack is the pack built in one go.
#[test]
fn a_stepped_packs_atlas_work_stays_within_each_steps_units() {
    let textures = vec![picture(9, 5, 71), picture(37, 23, 72), picture(3, 1, 73)];
    let (scene, _) = pictured_scene(textures.clone());
    let view = view_of(&scene);
    let whole =
        GpuRenderScene::pack_view_with_texture_limits(&scene, &view, LIMITS, HALVING_LIMITS)
            .expect("the scene packs");
    let plan = plan_texture_atlas(&textures, HALVING_LIMITS).expect("a plan");
    // The 37 by 23 picture halves to 19 by 12.
    let halving_work = 19 * 12 * ATLAS_WORK_PER_HALVED_TEXEL as u64;
    assert_eq!(plan.shrunk()[0].texture, 1);
    for units in [1, 7, ATLAS_WORK_PER_HALVED_TEXEL, 100, 1000] {
        let mut builder = GpuSceneBuilder::new(&scene, view, LIMITS)
            .expect("a builder")
            .with_texture_limits(HALVING_LIMITS);
        let (mut largest, mut atlas_steps) = (0, 0);
        let pack = loop {
            let before = builder.atlas_work_done();
            let pack = builder.step(units).expect("the scene packs");
            let spent = builder.atlas_work_done() - before;
            if spent > 0 {
                atlas_steps += 1;
            }
            largest = largest.max(spent);
            if let Some(pack) = pack {
                break pack;
            }
        };
        // A unit count fits a u64.
        let most = units as u64 + ATLAS_WORK_PER_HALVED_TEXEL as u64 - 1;
        assert!(
            largest <= most,
            "{units} units a step: a step did {largest}"
        );
        assert!(largest < halving_work, "{units} units a step halved in one");
        assert!(
            atlas_steps > 1,
            "{units} units a step built the atlas in one"
        );
        assert_eq!(builder.atlas_work_done(), plan.build_work());
        assert_eq!(pack, whole, "{units} units a step");
    }
}

/// A scene without pictures still finishes in the tree's step, with its 1 by 1 white atlas.
#[test]
fn a_scene_without_pictures_finishes_in_the_trees_step() {
    let bare = boxes_scene(
        &[(
            [1.0; 3],
            [0.0, 0.0, 0.5],
            ScatteringSurface::matte_plastic([0.8, 0.7, 0.6]),
        )],
        false,
        furnace(),
    );
    let mut builder = GpuSceneBuilder::new(&bare, view_of(&bare), LIMITS).expect("a builder");
    while !builder.next_step_builds_tree() {
        assert_eq!(builder.step(1), Ok(None));
    }
    let pack = builder
        .step(1)
        .expect("packs")
        .expect("the pack, in the tree's step");
    assert_eq!(pack.texture_atlas().bytes(), Some(&[255u8; 4][..]));
    assert_eq!(builder.atlas_work_done(), 1);
}

/// A re-pack handed the first pack's plan builds no atlas texels and writes the same picture
/// block, faces and parts; its atlas has the plan's size and placements and no bytes. A plan made
/// for other pictures is refused when handed over.
#[test]
fn a_repack_handed_its_plan_builds_no_atlas() {
    let textures = vec![picture(9, 5, 81), picture(37, 23, 82), picture(3, 1, 83)];
    let (scene, _) = pictured_scene(textures);
    let view = view_of(&scene);
    let first_builder = GpuSceneBuilder::new(&scene, view, LIMITS)
        .expect("a builder")
        .with_texture_limits(HALVING_LIMITS);
    let (first, first_work, _) = step_to_end(first_builder, 5);
    assert!(first_work > 0);
    let plan = Arc::clone(first.texture_atlas().plan());
    for units in [1, usize::MAX] {
        let builder = GpuSceneBuilder::new(&scene, view, LIMITS)
            .expect("a builder")
            .with_texture_plan(Arc::clone(&plan))
            .expect("the plan is for these pictures");
        let (again, work, _) = step_to_end(builder, units);
        assert_eq!(work, 0, "no atlas texel was built");
        assert_eq!(again.texture_block(), first.texture_block());
        assert_eq!(again.part_bytes(), first.part_bytes());
        assert_eq!(again.face_bytes(), first.face_bytes());
        assert_eq!(again.view_bytes(), first.view_bytes());
        assert_eq!(again.texture_atlas().bytes(), None);
        assert_eq!(again.texture_atlas().plan(), &plan);
        assert_eq!(
            again.texture_atlas().placements(),
            first.texture_atlas().placements()
        );
        assert_eq!(
            (
                again.texture_atlas().width(),
                again.texture_atlas().height()
            ),
            (
                first.texture_atlas().width(),
                first.texture_atlas().height()
            )
        );
        assert_eq!(again.sizes(), first.sizes());
    }

    let other = Arc::new(
        plan_texture_atlas(&[picture(9, 5, 81), picture(3, 1, 83)], HALVING_LIMITS)
            .expect("a plan"),
    );
    let refused = GpuSceneBuilder::new(&scene, view, LIMITS)
        .expect("a builder")
        .with_texture_plan(other);
    assert!(matches!(
        refused.err(),
        Some(FlatTreeError::NotRepresentable { .. })
    ));
}

/// A pack that answered an error is done: the next step answers the "already finished" refusal,
/// never a retry of a half-built pack. Here the error is the scene's reach, too large for a
/// placement step of a picometre.
#[test]
fn a_step_after_an_error_is_refused_as_finished() {
    let (scene, _) = pictured_scene(vec![picture(3, 1, 91)]);
    let tight = GpuSceneLimits {
        max_vertex_step: 1e-12,
        ..LIMITS
    };
    let mut builder = GpuSceneBuilder::new(&scene, view_of(&scene), tight).expect("a builder");
    assert!(matches!(
        builder.step(usize::MAX),
        Err(FlatTreeError::SceneTooLarge { .. })
    ));
    assert_eq!(
        builder.step(usize::MAX),
        Err(FlatTreeError::NotRepresentable {
            reason: "this GPU pack was already finished",
        })
    );
}
