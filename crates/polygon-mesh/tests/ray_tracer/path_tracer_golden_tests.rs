//! A golden image of an all-opaque scene: its digest is pinned so a change to the path loop that
//! moves one bit of an opaque render is caught, the guarantee that see-through parts cost a scene
//! without them nothing. The scene is built from exact coordinates only, so the digest holds on
//! every platform the library runs on.

use solidmesh_math::{Matrix4X4, Vector3};
use solidmesh_mesh::ray_tracer::{
    Environment, LightSetup, MeshSurface, PathTraceSettings, PathTracer, PlacedPart, PlacedParts,
    RenderCamera, RenderScene, RenderSurface, ScatteringSurface, Sun, SunThroughGlass,
};
use solidmesh_mesh::solids::platonic_solids::create_cube;
use solidmesh_mesh::CancelToken;

use crate::path_tracer_support::{flat_patch, SHARP_CREASE};
use crate::surface_hit_support::tree_of;

const SIZE: (u32, u32) = (17, 11);

/// The opaque scene the digest pins: a colored floor and a shiny translated cube under a sun and
/// a gradient sky, rendered 4 passes.
pub(crate) fn golden_scene() -> RenderScene {
    let floor = flat_patch(30.0, 0.0);
    let cube = create_cube(10.0, 10.0, 10.0);
    let parts = vec![
        PlacedPart::new(
            tree_of(&floor),
            std::sync::Arc::new(MeshSurface::new(&floor, SHARP_CREASE).unwrap()),
            Matrix4X4::IDENTITY,
        )
        .unwrap(),
        PlacedPart::new(
            tree_of(&cube),
            std::sync::Arc::new(MeshSurface::new(&cube, SHARP_CREASE).unwrap()),
            Matrix4X4::create_translation(0.0, 0.0, 7.0),
        )
        .unwrap(),
    ];
    RenderScene::new(
        PlacedParts::new(parts),
        vec![
            ScatteringSurface::new([0.6, 0.55, 0.5], 0.0, 1.0, 0.0, 1.0),
            ScatteringSurface::new([0.7, 0.2, 0.3], 0.0, 0.35, 0.0, 1.5),
        ],
        LightSetup {
            environment: Environment::gradient([0.3, 0.4, 0.9], [0.9, 0.85, 0.8], [0.15, 0.1, 0.1])
                .unwrap(),
            sun: Some(Sun::new(Vector3::new(0.5, 0.3, 0.8), 0.1, [6.0, 5.5, 5.0]).unwrap()),
        },
    )
    .unwrap()
}

/// A digest of a whole render's accumulated sums, in pixel order.
pub(crate) fn render_digest(scene: &RenderScene) -> u64 {
    render_digest_with(scene, SunThroughGlass::default())
}

/// [`render_digest`] with sunlight reaching what lies behind glass as `through` says.
pub(crate) fn render_digest_with(scene: &RenderScene, through: SunThroughGlass) -> u64 {
    let camera = RenderCamera::orthographic(
        Vector3::new(0.0, 0.0, 40.0),
        Vector3::ZERO,
        Vector3::UNIT_Y,
        26.0,
    )
    .unwrap();
    let settings = PathTraceSettings {
        seed: 42,
        sun_through_glass: through,
        ..PathTraceSettings::default()
    };
    let tracer = PathTracer::new(scene, camera, SIZE.0, SIZE.1, settings).unwrap();
    let mut surface = RenderSurface::new(SIZE.0, SIZE.1).unwrap();
    for sample in 0..4 {
        tracer
            .render_pass(&mut surface, sample, &CancelToken::new())
            .unwrap();
    }
    let mut digest = 0xcbf2_9ce4_8422_2325_u64;
    for y in 0..SIZE.1 {
        for x in 0..SIZE.0 {
            for channel in surface.pixel(x, y).unwrap().sum() {
                for byte in channel.to_bits().to_le_bytes() {
                    digest = (digest ^ u64::from(byte)).wrapping_mul(0x0000_0100_0000_01b3);
                }
            }
        }
    }
    digest
}

/// Pinned from the path tracer as it was before parts could be see-through: moving it means an
/// opaque image changed.
const OPAQUE_RENDER_DIGEST: u64 = 0xc435_c1f3_58d8_6094;

/// An all-opaque scene renders exactly as it always did: the part-opacity machinery draws no
/// random number and changes no ray for a scene that has none of it.
#[test]
fn an_opaque_scene_renders_as_it_always_did() {
    let digest = render_digest(&golden_scene());
    println!("opaque digest {digest:#018x}");
    assert_eq!(digest, OPAQUE_RENDER_DIGEST, "got {digest:#018x}");
    // With no glass, how the sun reaches what lies behind glass changes nothing.
    let exact = render_digest_with(&golden_scene(), SunThroughGlass::Exact);
    assert_eq!(exact, OPAQUE_RENDER_DIGEST, "got {exact:#018x} exactly");
}
