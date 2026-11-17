//! Pictures on faces in the CPU path tracer: what `RenderScene::with_face_textures` makes a hit
//! read. A white or fully see-through picture leaves a render exactly as it was, Repeat wrapping
//! reads the texels a real design's out-of-range coordinates name, bilinear filtering blends
//! decoded (linear) colors, each texel's bytes show at their place on the face, a picture's first
//! row shows at the bottom of a rendered face, a colored face
//! under a picture multiplies the two, a rendered checker shows each half's expected color, and
//! a picture or face assignment of the wrong size is refused, and two scenes given the same
//! pictures hold them, not copies.

use std::sync::Arc;

use solidmesh_math::{Color, Matrix4X4, Vector2, Vector2Float, Vector3};
use solidmesh_mesh::ray_tracer::{
    blend_factors, Environment, LightSetup, MeshSurface, PathTraceSettings, PathTracer, PlacedPart,
    PlacedParts, RayTracerError, RenderScene, RenderWork, ScatteringSurface, Sun, TraceTexture,
};
use solidmesh_mesh::{FaceTextureData, Mesh};

use crate::path_tracer_support::{flat_patch, top_down_camera, SHARP_CREASE};
use crate::scattering_support::Stats;
use crate::surface_hit_support::tree_of;

/// A diffuse surface of `albedo`.
fn lambert(albedo: [f64; 3]) -> ScatteringSurface {
    ScatteringSurface::new(albedo, 0.0, 1.0, 0.0, 1.0)
}

/// A 2 by 2 square at the origin facing +Z, its texture coordinates (0, 0) at the corner
/// (-1, -1) and (1, 1) at (1, 1), each face colored `color`.
fn mapped_square(color: Color) -> Mesh {
    let mut mesh = flat_patch(1.0, 0.0);
    mesh.face_colors = Some(vec![color; mesh.faces.len()]);
    let uv = |u: f64, v: f64| Vector2Float::new(u, v);
    let corners = [uv(0.0, 0.0), uv(1.0, 0.0), uv(1.0, 1.0), uv(0.0, 1.0)];
    for (face, [a, b, c]) in [[0, 1, 2], [0, 2, 3]].into_iter().enumerate() {
        mesh.face_textures.insert(
            face as i32,
            FaceTextureData::new(None, corners[a], corners[b], corners[c]),
        );
    }
    mesh
}

/// `mesh` under `lights`, its faces showing `texture` when there is one.
fn scene_of(
    mesh: &Mesh,
    surface: ScatteringSurface,
    lights: LightSetup,
    texture: Option<TraceTexture>,
) -> RenderScene {
    let part = PlacedPart::new(
        tree_of(mesh),
        Arc::new(MeshSurface::new(mesh, SHARP_CREASE).unwrap()),
        Matrix4X4::IDENTITY,
    )
    .unwrap();
    let scene = RenderScene::new(PlacedParts::new(vec![part]), vec![surface], lights).unwrap();
    match texture {
        None => scene,
        Some(texture) => scene
            .with_face_textures(
                vec![Arc::new(texture)],
                vec![vec![Some(0); mesh.faces.len()]],
            )
            .unwrap(),
    }
}

/// A sun and a sky, so a render reads the surface through both the sun and the bounces.
fn sun_and_sky() -> LightSetup {
    LightSetup {
        environment: Environment::constant([0.4, 0.5, 0.6]).unwrap(),
        sun: Some(Sun::new(Vector3::new(0.3, -0.2, 1.0), 0.05, [3.0, 2.8, 2.5]).unwrap()),
    }
}

/// Every sample of a small render of `scene`, in order.
fn render(scene: &RenderScene) -> Vec<[f64; 3]> {
    let tracer = PathTracer::new(
        scene,
        top_down_camera(10.0, 2.5),
        6,
        6,
        PathTraceSettings::default(),
    )
    .unwrap();
    let mut work = RenderWork::default();
    let mut samples = Vec::new();
    for y in 0..6 {
        for x in 0..6 {
            for sample in 0..4 {
                samples.push(tracer.sample_pixel(x, y, sample, &mut work).unwrap());
            }
        }
    }
    samples
}

/// The bits of every channel of every sample, for an exact comparison.
fn bits(samples: &[[f64; 3]]) -> Vec<[u64; 3]> {
    samples
        .iter()
        .map(|sample| sample.map(f64::to_bits))
        .collect()
}

/// One texel's bytes repeated over a `width` by `height` picture.
fn uniform(width: u32, height: u32, texel: [u8; 4]) -> TraceTexture {
    TraceTexture::new(width, height, texel.repeat((width * height) as usize)).unwrap()
}

/// The factor `scene`'s texture 0 gives at `uv`.
fn factor(scene: &RenderScene, uv: Vector2) -> [f64; 3] {
    scene.texture_factor_at(0, uv).unwrap()
}

/// A white opaque picture leaves a colored face exactly as it was: the factor is 1 and the render
/// matches the untextured one to the bit.
#[test]
fn a_white_opaque_picture_renders_exactly_as_no_picture() {
    let mesh = mapped_square(Color::rgba(200, 90, 30, 255));
    let surface = lambert([0.9, 0.8, 0.7]);
    let plain = scene_of(&mesh, surface, sun_and_sky(), None);
    let white = scene_of(
        &mesh,
        surface,
        sun_and_sky(),
        Some(uniform(3, 2, [255, 255, 255, 255])),
    );
    assert_eq!(factor(&white, Vector2::new(0.37, -1.3381)), [1.0; 3]);
    assert_eq!(bits(&render(&white)), bits(&render(&plain)));
}

/// A fully see-through picture, whatever its color, shows the face's own color: the factor is
/// exactly 1 and the render matches the untextured one to the bit.
#[test]
fn a_fully_see_through_picture_renders_exactly_as_no_picture() {
    let mesh = mapped_square(Color::rgba(40, 160, 220, 255));
    let surface = lambert([0.6, 0.7, 0.8]);
    let plain = scene_of(&mesh, surface, sun_and_sky(), None);
    let bytes: Vec<u8> = [
        [10, 200, 30, 0],
        [250, 5, 90, 0],
        [0, 0, 0, 0],
        [77, 1, 2, 0],
    ]
    .concat();
    let clear = scene_of(
        &mesh,
        surface,
        sun_and_sky(),
        Some(TraceTexture::new(2, 2, bytes).unwrap()),
    );
    assert_eq!(factor(&clear, Vector2::new(0.3, 0.6)), [1.0; 3]);
    assert_eq!(bits(&render(&clear)), bits(&render(&plain)));
}

/// Coordinates outside 0..1, as a real design has them (v down to -1.3381, u up to 1.012), wrap
/// by Repeat: u = 1.012 on a 4 wide picture falls between the last column and the first, and
/// v = -1.3381 on a 4 high picture between rows 2 and 3, with the weights the wrapped position
/// gives, and the factor is the blend of exactly those texels.
#[test]
fn repeat_wrapping_reads_the_texels_out_of_range_coordinates_name() {
    let bytes: Vec<u8> = (0..16u8)
        .flat_map(|texel| [texel * 16, 255 - texel * 8, texel * 5 + 3, 255])
        .collect();
    let texture = TraceTexture::new(4, 4, bytes).unwrap();
    let uv = Vector2::new(1.012, -1.3381);
    let spot = texture.spot(uv);
    assert_eq!(spot.columns, [3, 0]);
    assert_eq!(spot.rows, [2, 3]);
    // u wraps to 0.012: 0.012 * 4 - 0.5 = -0.452, 0.548 past column -1 (column 3).
    assert!((spot.across - 0.548).abs() < 1e-9, "{}", spot.across);
    // v wraps to 0.6619: 0.6619 * 4 - 0.5 = 2.1476, 0.1476 past row 2.
    assert!((spot.up - 0.1476).abs() < 1e-9, "{}", spot.up);

    let mesh = mapped_square(Color::rgba(255, 255, 255, 255));
    let scene = scene_of(&mesh, lambert([1.0; 3]), sun_and_sky(), Some(texture));
    // A texel's centre reads that texel alone.
    let at = |column: u32, row: u32| {
        factor(
            &scene,
            Vector2::new(
                (f64::from(column) + 0.5) / 4.0,
                (f64::from(row) + 0.5) / 4.0,
            ),
        )
    };
    let expected = blend_factors(
        [at(3, 2), at(0, 2), at(3, 3), at(0, 3)],
        spot.across,
        spot.up,
    );
    assert_eq!(factor(&scene, uv), expected);
}

/// Halfway between two texels the factor is the mean of their decoded (linear) colors, not the
/// decoded mean of their bytes: black beside white is exactly 0.5, where the byte mean (127.5)
/// decodes to about 0.21.
#[test]
fn bilinear_filtering_blends_decoded_colors() {
    let mesh = mapped_square(Color::rgba(255, 255, 255, 255));
    let pair = |left: [u8; 4], right: [u8; 4]| {
        let texture = TraceTexture::new(2, 1, [left, right].concat()).unwrap();
        scene_of(&mesh, lambert([1.0; 3]), sun_and_sky(), Some(texture))
    };
    let middle = Vector2::new(0.5, 0.5);
    let black_white = pair([0, 0, 0, 255], [255, 255, 255, 255]);
    assert_eq!(factor(&black_white, middle), [0.5; 3]);

    let colored = pair([200, 30, 250, 255], [10, 240, 10, 255]);
    let left = factor(&colored, Vector2::new(0.25, 0.5));
    let right = factor(&colored, Vector2::new(0.75, 0.5));
    let blended = factor(&colored, middle);
    let byte_mean = factor(
        &scene_of(
            &mesh,
            lambert([1.0; 3]),
            sun_and_sky(),
            Some(uniform(1, 1, [105, 135, 130, 255])),
        ),
        middle,
    );
    for channel in 0..3 {
        let mean = (left[channel] + right[channel]) / 2.0;
        assert!(
            (blended[channel] - mean).abs() < 1e-15,
            "channel {channel}: {} against {mean}",
            blended[channel]
        );
        assert!(
            (blended[channel] - byte_mean[channel]).abs() > 0.02,
            "channel {channel}: the blend matches the byte mean's color"
        );
    }
}

/// A picture's rows run bottom first up the face: a 1 by 2 picture rendered on the square, in a
/// white furnace, shows its first row's bytes over the lower half of the render (v from 0 to 0.5)
/// and its second row's over the upper half. The filter gives each half's own row three
/// quarters of the weight and the other row (wrapped round) one quarter; the expected colors come
/// from the bytes by the sRGB formula.
#[test]
fn a_pictures_first_row_shows_at_the_bottom_of_the_face() {
    let albedo = [0.9, 0.8, 0.7];
    let rows = [[250, 30, 10, 255], [10, 60, 240, 255]];
    let mesh = mapped_square(Color::rgba(255, 255, 255, 255));
    let scene = scene_of(
        &mesh,
        lambert(albedo),
        LightSetup::environment_only(Environment::constant([1.0; 3]).unwrap()),
        Some(TraceTexture::new(1, 2, rows.concat()).unwrap()),
    );
    let size = 8;
    let tracer = PathTracer::new(
        &scene,
        top_down_camera(10.0, 2.0),
        size,
        size,
        PathTraceSettings::default(),
    )
    .unwrap();
    let mut work = RenderWork::default();
    let mut halves = [[Stats::default(); 3]; 2];
    for y in 0..size {
        // Pixel rows count up from the bottom of the render, as v does on the square.
        let half = usize::from(y >= size / 2);
        for x in 0..size {
            for sample in 0..32 {
                let value = tracer.sample_pixel(x, y, sample, &mut work).unwrap();
                for (stat, channel) in halves[half].iter_mut().zip(value) {
                    stat.add(channel);
                }
            }
        }
    }
    for (half, (near, far)) in [(rows[0], rows[1]), (rows[1], rows[0])]
        .into_iter()
        .enumerate()
    {
        for channel in 0..3 {
            let expected =
                albedo[channel] * (0.75 * decode(near[channel]) + 0.25 * decode(far[channel]));
            let stat = halves[half][channel];
            assert!(
                (stat.mean() - expected).abs() <= 5.0 * stat.standard_error() + 0.002,
                "half {half} channel {channel}: {} against {expected}",
                stat.mean()
            );
        }
    }
}

/// A 2 by 1 checker on the square, in a white furnace: each half of the render averages its
/// filtered factor times the part's color. Across the left half (u from 0 to 0.5) the filter
/// gives the left texel three quarters of the weight and the right (wrapped round) one quarter,
/// and the right half the reverse.
#[test]
fn a_rendered_checker_shows_each_halfs_expected_color() {
    let albedo = [0.9, 0.8, 0.6];
    let mesh = mapped_square(Color::rgba(255, 255, 255, 255));
    let texture =
        TraceTexture::new(2, 1, [[255, 20, 128, 255], [0, 230, 40, 255]].concat()).unwrap();
    let scene = scene_of(
        &mesh,
        lambert(albedo),
        LightSetup::environment_only(Environment::constant([1.0; 3]).unwrap()),
        Some(texture),
    );
    let first = factor(&scene, Vector2::new(0.25, 0.5));
    let second = factor(&scene, Vector2::new(0.75, 0.5));

    let size = 8;
    let tracer = PathTracer::new(
        &scene,
        top_down_camera(10.0, 2.0),
        size,
        size,
        PathTraceSettings::default(),
    )
    .unwrap();
    let mut work = RenderWork::default();
    let mut halves = [[Stats::default(); 3]; 2];
    for y in 0..size {
        for x in 0..size {
            let half = usize::from(x >= size / 2);
            for sample in 0..32 {
                let value = tracer.sample_pixel(x, y, sample, &mut work).unwrap();
                for (stat, channel) in halves[half].iter_mut().zip(value) {
                    stat.add(channel);
                }
            }
        }
    }
    for (half, (near, far)) in [(first, second), (second, first)].into_iter().enumerate() {
        for channel in 0..3 {
            let expected = albedo[channel] * (0.75 * near[channel] + 0.25 * far[channel]);
            let stat = halves[half][channel];
            assert!(
                (stat.mean() - expected).abs() <= 5.0 * stat.standard_error() + 0.002,
                "half {half} channel {channel}: {} against {expected}",
                stat.mean()
            );
        }
    }
}

/// The sRGB decode of an 8-bit channel value, by its formula.
fn decode(code: u8) -> f64 {
    let value = f64::from(code) / 255.0;
    if value <= 0.04045 {
        value / 12.92
    } else {
        ((value + 0.055) / 1.055).powf(2.4)
    }
}

/// Each texel's bytes land where the picture's layout puts them: on a 2 by 2 picture, rows
/// bottom first, bytes 0..4 at the bottom-left texel's centre, 4..8 bottom-right, 8..12
/// top-left and 12..16 top-right, each read as its decoded color alone.
#[test]
fn each_texels_bytes_show_at_its_place_on_the_face() {
    let bytes: Vec<u8> = vec![
        200, 10, 90, 255, // bottom-left
        30, 220, 140, 255, // bottom-right
        120, 60, 250, 255, // top-left
        5, 160, 45, 255, // top-right
    ];
    let mesh = mapped_square(Color::rgba(255, 255, 255, 255));
    let texture = TraceTexture::new(2, 2, bytes.clone()).unwrap();
    let scene = scene_of(&mesh, lambert([1.0; 3]), sun_and_sky(), Some(texture));
    let places = [(0.25, 0.25), (0.75, 0.25), (0.25, 0.75), (0.75, 0.75)];
    for (texel, &(u, v)) in places.iter().enumerate() {
        let read = factor(&scene, Vector2::new(u, v));
        for channel in 0..3 {
            let expected = decode(bytes[texel * 4 + channel]);
            assert!(
                (read[channel] - expected).abs() < 1e-12,
                "texel {texel} channel {channel}: {} against {expected}",
                read[channel]
            );
        }
    }
}

/// A colored face under a colored picture shows the part's color times the face's decoded color
/// times the picture's decoded color: in a white furnace a flat Lambert face returns exactly
/// that product.
#[test]
fn a_colored_face_under_a_picture_multiplies_part_face_and_picture() {
    let albedo = [0.9, 0.7, 0.5];
    let face = [128, 64, 200];
    let texel = [60, 180, 255];
    let mesh = mapped_square(Color::rgba(face[0], face[1], face[2], 255));
    let scene = scene_of(
        &mesh,
        lambert(albedo),
        LightSetup::environment_only(Environment::constant([1.0; 3]).unwrap()),
        Some(uniform(2, 2, [texel[0], texel[1], texel[2], 255])),
    );
    let size = 4;
    let tracer = PathTracer::new(
        &scene,
        top_down_camera(10.0, 1.5),
        size,
        size,
        PathTraceSettings::default(),
    )
    .unwrap();
    let mut work = RenderWork::default();
    let mut stats = [Stats::default(); 3];
    for y in 0..size {
        for x in 0..size {
            for sample in 0..16 {
                let value = tracer.sample_pixel(x, y, sample, &mut work).unwrap();
                for (stat, channel) in stats.iter_mut().zip(value) {
                    stat.add(channel);
                }
            }
        }
    }
    for (channel, stat) in stats.iter().enumerate() {
        let expected = albedo[channel] * decode(face[channel]) * decode(texel[channel]);
        assert!(
            (stat.mean() - expected).abs() <= 5.0 * stat.standard_error() + 1e-6,
            "channel {channel}: {} against {expected}",
            stat.mean()
        );
    }
}

/// Whether `result` is refused as an invalid argument whose text says `why`.
fn refused_with<T>(result: Result<T, RayTracerError>, why: &str) -> bool {
    matches!(result, Err(RayTracerError::InvalidArgument(text)) if text.contains(why))
}

/// A picture with no texels, or with a byte count that is not width x height x 4, is refused,
/// saying which; so are face textures for the wrong number of parts, a part's list for the wrong
/// number of faces, and a face naming a picture the scene doesn't have.
#[test]
fn a_picture_of_the_wrong_size_is_refused() {
    let size = "width x height x 4 bytes";
    let empty = "at least one texel";
    assert!(refused_with(TraceTexture::new(2, 2, vec![0; 15]), size));
    assert!(refused_with(TraceTexture::new(2, 2, vec![0; 17]), size));
    assert!(refused_with(TraceTexture::new(0, 4, Vec::new()), empty));
    assert!(refused_with(TraceTexture::new(4, 0, Vec::new()), empty));
    assert!(TraceTexture::new(2, 2, vec![0; 16]).is_ok());

    let mesh = mapped_square(Color::rgba(255, 255, 255, 255));
    let scene = || scene_of(&mesh, lambert([1.0; 3]), sun_and_sky(), None);
    let picture = || vec![Arc::new(uniform(1, 1, [255; 4]))];
    assert!(refused_with(
        scene().with_face_textures(picture(), Vec::new()),
        "one list of face textures for each part",
    ));
    assert!(refused_with(
        scene().with_face_textures(picture(), vec![Vec::new(), Vec::new()]),
        "one list of face textures for each part",
    ));
    assert!(refused_with(
        scene().with_face_textures(picture(), vec![vec![Some(0)]]),
        "one texture (or none) per face",
    ));
    assert!(refused_with(
        scene().with_face_textures(picture(), vec![vec![Some(1), None]]),
        "past the end",
    ));
    assert!(scene()
        .with_face_textures(picture(), vec![vec![Some(0), None]])
        .is_ok());
}

/// Two scenes given the same pictures hold those very pictures, not copies: a caller that builds
/// a new scene at every camera stop keeps one decoded copy of a design's pictures however many
/// scenes it builds from them, and each scene reads them as the one built first does.
#[test]
fn scenes_given_the_same_pictures_share_their_texels() {
    let mesh = mapped_square(Color::rgba(255, 255, 255, 255));
    let scene = || scene_of(&mesh, lambert([1.0; 3]), sun_and_sky(), None);
    let bytes = [[255, 20, 128, 255], [0, 230, 40, 255]].concat();
    let pictures = vec![
        Arc::new(TraceTexture::new(2, 1, bytes).unwrap()),
        Arc::new(uniform(1, 1, [90, 90, 90, 255])),
    ];
    let faces = vec![vec![Some(0), Some(1)]];
    let first = scene()
        .with_face_textures(pictures.clone(), faces.clone())
        .unwrap();
    let second = scene().with_face_textures(pictures.clone(), faces).unwrap();
    assert_eq!(first.textures().len(), 2);
    for (index, picture) in pictures.iter().enumerate() {
        for scene in [&first, &second] {
            assert!(Arc::ptr_eq(&scene.textures()[index], picture));
            assert_eq!(
                scene.textures()[index].bytes().as_ptr(),
                picture.bytes().as_ptr()
            );
        }
    }
    let uv = Vector2::new(0.25, 0.5);
    assert_eq!(
        first.texture_factor_at(0, uv),
        second.texture_factor_at(0, uv)
    );
    assert!(first.texture_factor_at(0, uv).is_some());
}
