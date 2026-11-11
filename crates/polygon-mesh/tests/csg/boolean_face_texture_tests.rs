//! Face textures through booleans: a textured part combined with, cut by or intersected with
//! another part keeps its picture on every face that came from it. The picture is measured, not
//! eyeballed: points on each result face look their colour up through the result's UVs and are
//! compared with the colour at the same point of the original textured surface, read through the
//! original UVs. Faces a plain part contributes stay untextured, and two textured parts each keep
//! their own picture.

use std::sync::Arc;

use crate::test_picture::TestPicture;
use solidmesh_math::{Color, Matrix4X4, TexturePixels, Vector2Float, Vector3, Vector3Float};
use solidmesh_mesh::csg::{boolean_processing, BooleanOptions, CsgModes};
use solidmesh_mesh::solids::platonic_solids::create_cube;
use solidmesh_mesh::{CancelToken, FaceTextureData, Mesh};

#[path = "boolean_face_texture_chart_tests.rs"]
mod chart_cases;
#[path = "boolean_face_texture_edge_tests.rs"]
mod edge_cases;

const PICTURE_SIZE: i32 = 256;
const RED: Color = Color::rgb(0xE0, 0x30, 0x30);
const BLUE: Color = Color::rgb(0x30, 0x40, 0xE0);

/// Smooth bands of colour, several per atlas cell in each direction, so a UV that slides along
/// the surface shows as a colour change. `phase` makes a second, different picture.
fn banded_picture(phase: f64) -> Arc<TestPicture> {
    let mut picture = TestPicture::new(PICTURE_SIZE, PICTURE_SIZE);
    let size = PICTURE_SIZE as usize;
    let bytes = picture.bytes_mut();
    let tau = std::f64::consts::TAU;
    for y in 0..size {
        for x in 0..size {
            let u = (x as f64 + 0.5) / size as f64;
            let v = (y as f64 + 0.5) / size as f64;
            let r = 127.5 + 127.5 * libm::sin(tau * 12.0 * u + phase);
            let g = 127.5 + 127.5 * libm::sin(tau * 9.0 * v + 2.0 * phase);
            let b = 127.5 + 127.5 * libm::sin(tau * 6.0 * (u + v));
            let at = (y * size + x) * 4;
            // Each channel lies in [0, 255], so the casts cannot saturate.
            bytes[at] = r.round() as u8;
            bytes[at + 1] = g.round() as u8;
            bytes[at + 2] = b.round() as u8;
            bytes[at + 3] = 255;
        }
    }
    Arc::new(picture)
}

/// The picture's colour at `(u, v)`, bilinearly filtered, with clamped edges.
fn sample(picture: &TestPicture, u: f64, v: f64) -> [f64; 3] {
    let size = PICTURE_SIZE as f64;
    let x = (u * size - 0.5).clamp(0.0, size - 1.0);
    let y = (v * size - 0.5).clamp(0.0, size - 1.0);
    // Clamped into the picture, so the casts stay in range.
    let x0 = x.floor() as usize;
    let y0 = y.floor() as usize;
    let x1 = (x0 + 1).min(PICTURE_SIZE as usize - 1);
    let y1 = (y0 + 1).min(PICTURE_SIZE as usize - 1);
    let fx = x - x0 as f64;
    let fy = y - y0 as f64;
    let bytes = picture.pixel_bytes();
    let at =
        |px: usize, py: usize, c: usize| bytes[(py * PICTURE_SIZE as usize + px) * 4 + c] as f64;
    let mut out = [0.0; 3];
    for (c, channel) in out.iter_mut().enumerate() {
        let top = at(x0, y0, c) * (1.0 - fx) + at(x1, y0, c) * fx;
        let bottom = at(x0, y1, c) * (1.0 - fx) + at(x1, y1, c) * fx;
        *channel = top * (1.0 - fy) + bottom * fy;
    }
    out
}

/// A 10 mm cube whose six sides are six UV islands of a 3 by 2 atlas: each side's picture is the
/// side projected onto its own plane, scaled into its own cell with a margin.
fn textured_cube(picture: &Arc<TestPicture>) -> Mesh {
    let mut cube = create_cube(10.0, 10.0, 10.0);
    let image: Arc<dyn TexturePixels> = picture.clone();
    for face in 0..cube.faces.len() {
        let corners = corners_of(&cube, face);
        let normal = cube.faces[face].normal;
        let uvs = corners.map(|p| atlas_uv(p, normal));
        cube.face_textures.insert(
            face as i32,
            FaceTextureData::new(Some(image.clone()), uvs[0], uvs[1], uvs[2]),
        );
    }
    cube
}

/// The atlas UV of point `p` on the cube side facing `normal`.
fn atlas_uv(p: Vector3, normal: Vector3Float) -> Vector2Float {
    let n = [normal.x, normal.y, normal.z];
    let axis = (0..3)
        .max_by(|&a, &b| n[a].abs().total_cmp(&n[b].abs()))
        .unwrap_or(0);
    let cell = axis * 2 + usize::from(n[axis] > 0.0);
    let p = [p.x, p.y, p.z];
    let (a, b) = ((axis + 1) % 3, (axis + 2) % 3);
    // The cube spans -5..5 on every axis; each cell is a third by a half, with a margin.
    let s = (p[a] + 5.0) / 10.0;
    let t = (p[b] + 5.0) / 10.0;
    let u = ((cell % 3) as f64 + 0.05 + 0.9 * s) / 3.0;
    let v = ((cell / 3) as f64 + 0.05 + 0.9 * t) / 2.0;
    Vector2Float::new(u, v)
}

fn corners_of(mesh: &Mesh, face: usize) -> [Vector3; 3] {
    let f = &mesh.faces[face];
    [f.v0, f.v1, f.v2].map(|v| mesh.vertices[v as usize].as_vector3())
}

/// The barycentric weights of `p` in triangle `t`, and how far `p` is off its plane.
fn barycentric(t: [Vector3; 3], p: Vector3) -> ([f64; 3], f64) {
    let e0 = t[1] - t[0];
    let e1 = t[2] - t[0];
    let n = e0.cross(e1);
    let area2 = n.length();
    let off_plane = (p - t[0]).dot(n) / area2;
    let w1 = (p - t[0]).cross(e1).dot(n) / (area2 * area2);
    let w2 = e0.cross(p - t[0]).dot(n) / (area2 * area2);
    ([1.0 - w1 - w2, w1, w2], off_plane.abs())
}

/// The face of `surface` that `p` lies on, with its weights, if any.
fn face_under(surface: &Mesh, p: Vector3) -> Option<(usize, [f64; 3])> {
    (0..surface.faces.len())
        .filter_map(|face| {
            let (w, off) = barycentric(corners_of(surface, face), p);
            (off < 1e-4 && w.iter().all(|&w| w > -1e-6)).then_some((face, w, off))
        })
        .min_by(|a, b| a.2.total_cmp(&b.2))
        .map(|(face, w, _)| (face, w))
}

fn uv_at(texture: &FaceTextureData, w: [f64; 3]) -> (f64, f64) {
    let uvs = [texture.uv0, texture.uv1, texture.uv2];
    let mut u = 0.0;
    let mut v = 0.0;
    for (uv, w) in uvs.iter().zip(w) {
        u += f64::from(uv.x) * w;
        v += f64::from(uv.y) * w;
    }
    (u, v)
}

/// Points inside a result face: its centre, between centre and corners, and just inside each
/// corner, where a face that wrongly spans a UV seam shows the other side's mapping.
const SAMPLE_WEIGHTS: [[f64; 3]; 7] = [
    [1.0 / 3.0, 1.0 / 3.0, 1.0 / 3.0],
    [0.6, 0.2, 0.2],
    [0.2, 0.6, 0.2],
    [0.2, 0.2, 0.6],
    [0.96, 0.02, 0.02],
    [0.02, 0.96, 0.02],
    [0.02, 0.02, 0.96],
];

/// One textured source placed in the scene: its surface where the boolean saw it, and its
/// picture.
struct Source<'a> {
    placed: Mesh,
    picture: &'a Arc<TestPicture>,
}

impl Source<'_> {
    fn new<'a>(mesh: &Mesh, matrix: Matrix4X4, picture: &'a Arc<TestPicture>) -> Source<'a> {
        let mut placed = mesh.copy();
        placed.transform(matrix);
        Source { placed, picture }
    }
}

/// What a check over a result found.
#[derive(Debug, Default)]
struct Findings {
    textured_faces: usize,
    untextured_faces: usize,
    worst_uv: f64,
    worst_colour: f64,
}

/// The pictured face of `source` that `p` lies on, with its texture and weights.
fn pictured<'a>(source: &'a Source, p: Vector3) -> Option<(&'a FaceTextureData, [f64; 3])> {
    face_under(&source.placed, p).and_then(|(under, w)| {
        source
            .placed
            .face_textures
            .get(&(under as i32))
            .filter(|t| t.image.is_some())
            .map(|t| (t, w))
    })
}

/// Every result face lying on a pictured face of a textured source must wear that source's
/// picture, with UVs that match the source's at every sample point; every face lying on no
/// pictured source face (the plain part's surface) must be untextured.
fn check(result: &Mesh, sources: &[Source]) -> Findings {
    let mut findings = Findings::default();
    for face in 0..result.faces.len() {
        let corners = corners_of(result, face);
        let centre = (corners[0] + corners[1] + corners[2]) / 3.0;
        let source = sources
            .iter()
            .find(|source| face_under(&source.placed, centre).is_some());
        let texture = result.face_textures.get(&(face as i32));
        // A face on a source face with no picture is as plain as the plain part's.
        let source = source.filter(|source| pictured(source, centre).is_some());
        let Some(source) = source else {
            assert!(
                texture.is_none(),
                "face {face} of the plain part has a picture"
            );
            findings.untextured_faces += 1;
            continue;
        };
        let texture = texture.unwrap_or_else(|| panic!("face {face} lost its picture"));
        let expected: Arc<dyn TexturePixels> = source.picture.clone();
        assert!(
            texture.same_image(Some(&expected)),
            "face {face} wears another picture"
        );
        findings.textured_faces += 1;
        for w in SAMPLE_WEIGHTS {
            let p = corners[0] * w[0] + corners[1] * w[1] + corners[2] * w[2];
            let (got_u, got_v) = uv_at(texture, w);
            // A sample on a pictureless face means the result face spans onto it.
            let Some((source_texture, source_w)) = pictured(source, p) else {
                findings.worst_uv = f64::INFINITY;
                continue;
            };
            let (want_u, want_v) = uv_at(source_texture, source_w);
            findings.worst_uv = findings
                .worst_uv
                .max((got_u - want_u).abs().max((got_v - want_v).abs()));
            let got = sample(source.picture, got_u, got_v);
            let want = sample(source.picture, want_u, want_v);
            for (g, w) in got.iter().zip(want) {
                findings.worst_colour = findings.worst_colour.max((g - w).abs());
            }
        }
    }
    findings
}

/// UVs agree to about 16 f32 steps at UV 1 (measured: 2e-8, one step): the result's corners are
/// f32 positions and its UVs the kernel's f64 interpolation along cut edges narrowed to f32, so a
/// sample point and its UVs each carry a rounding the original surface's do not. In colour
/// (0-255) that is a hundredth of a level (measured: 1.4e-4).
const UV_TOLERANCE: f64 = 1e-6;
const COLOUR_TOLERANCE: f64 = 0.01;

fn tilted_cutter_matrix() -> Matrix4X4 {
    Matrix4X4::create_rotation(Vector3::new(0.3, 0.2, 0.5))
        * Matrix4X4::create_translation_vector(Vector3::new(4.0, 3.0, 2.3))
}

/// The textured cube is itself moved, so the check also covers its UVs riding the transform
/// unchanged.
fn textured_matrix() -> Matrix4X4 {
    Matrix4X4::create_rotation_z(0.1) * Matrix4X4::create_translation(0.5, -0.25, 0.0)
}

fn run(
    a: (&Mesh, Matrix4X4),
    b: (&Mesh, Matrix4X4),
    operation: CsgModes,
    mesh_colors: Option<Vec<Color>>,
) -> Mesh {
    let options = BooleanOptions {
        mesh_colors,
        ..BooleanOptions::default()
    };
    boolean_processing::do_boolean(a.0, a.1, b.0, b.1, operation, &CancelToken::new(), &options)
        .expect("the boolean runs")
}

#[test]
fn textured_part_keeps_its_picture_through_every_boolean() {
    let picture = banded_picture(0.0);
    let cube = textured_cube(&picture);
    let cutter = create_cube(7.0, 7.0, 7.0);
    for operation in [CsgModes::Union, CsgModes::Subtract, CsgModes::Intersect] {
        let result = run(
            (&cube, textured_matrix()),
            (&cutter, tilted_cutter_matrix()),
            operation,
            None,
        );
        let findings = check(&result, &[Source::new(&cube, textured_matrix(), &picture)]);
        assert!(findings.textured_faces > 0, "{operation:?}: {findings:?}");
        assert!(findings.untextured_faces > 0, "{operation:?}: {findings:?}");
        assert!(
            findings.worst_uv <= UV_TOLERANCE,
            "{operation:?}: {findings:?}"
        );
        assert!(
            findings.worst_colour <= COLOUR_TOLERANCE,
            "{operation:?}: {findings:?}"
        );
        assert!(result.face_colors.is_none(), "no colours were asked for");
        assert!(
            result.is_manifold(),
            "{operation:?}: the result is not closed"
        );
    }
}

#[test]
fn textured_boolean_keeps_its_face_colours() {
    let picture = banded_picture(0.0);
    let cube = textured_cube(&picture);
    let cutter = create_cube(7.0, 7.0, 7.0);
    let result = run(
        (&cube, textured_matrix()),
        (&cutter, tilted_cutter_matrix()),
        CsgModes::Subtract,
        Some(vec![RED, BLUE]),
    );
    let colors = result.face_colors.as_ref().expect("colours were asked for");
    assert_eq!(colors.len(), result.faces.len());
    for (face, &colour) in colors.iter().enumerate() {
        let want = if result.face_textures.contains_key(&(face as i32)) {
            RED
        } else {
            BLUE
        };
        assert_eq!(colour, want, "face {face}");
    }
    let findings = check(&result, &[Source::new(&cube, textured_matrix(), &picture)]);
    assert!(findings.worst_uv <= UV_TOLERANCE, "{findings:?}");
}

#[test]
fn two_textured_parts_each_keep_their_own_picture() {
    let first = banded_picture(0.0);
    let second = banded_picture(1.7);
    let a = textured_cube(&first);
    let b = textured_cube(&second);
    for operation in [CsgModes::Union, CsgModes::Subtract, CsgModes::Intersect] {
        let result = run(
            (&a, textured_matrix()),
            (&b, tilted_cutter_matrix()),
            operation,
            None,
        );
        let findings = check(
            &result,
            &[
                Source::new(&a, textured_matrix(), &first),
                Source::new(&b, tilted_cutter_matrix(), &second),
            ],
        );
        assert_eq!(findings.untextured_faces, 0, "{operation:?}: {findings:?}");
        assert!(
            findings.worst_uv <= UV_TOLERANCE,
            "{operation:?}: {findings:?}"
        );
        assert!(
            findings.worst_colour <= COLOUR_TOLERANCE,
            "{operation:?}: {findings:?}"
        );
    }
}

#[test]
fn textured_boolean_is_the_same_on_every_run() {
    let picture = banded_picture(0.0);
    let cube = textured_cube(&picture);
    let cutter = create_cube(7.0, 7.0, 7.0);
    let once = || {
        run(
            (&cube, textured_matrix()),
            (&cutter, tilted_cutter_matrix()),
            CsgModes::Subtract,
            Some(vec![RED, BLUE]),
        )
    };
    let first = once();
    let first_hash = first.get_long_hash_code_with(0);
    for _ in 0..3 {
        let again = once();
        assert_eq!(again.get_long_hash_code_with(0), first_hash);
        assert_eq!(again.vertices, first.vertices);
        let uvs = |mesh: &Mesh| -> Vec<(i32, [u32; 6])> {
            mesh.face_textures
                .iter()
                .map(|(&face, t)| {
                    (
                        face,
                        [t.uv0, t.uv1, t.uv2]
                            .map(|uv| [uv.x.to_bits(), uv.y.to_bits()])
                            .concat()
                            .try_into()
                            .unwrap_or([0; 6]),
                    )
                })
                .collect()
        };
        assert_eq!(uvs(&again), uvs(&first));
    }
}
