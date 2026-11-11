//! Face textures through booleans where the mapping is not one picture laid flat per side: a flat
//! side split into two UV charts, a flat side whose one chart bends, a self-intersecting textured
//! part (the kernel's robust path), a textured part that also wears face colours, a face whose
//! texture entry has UVs but no picture, and a non-finite UV. Uses the parent file's cube, picture
//! and surface check.

use super::*;

/// The faces of the cube's flat top side (y = +5), coplanar and wearing one picture.
const TOP: [usize; 2] = [8, 9];

/// The top side as two UV charts meeting along its diagonal: face 9's mapping is shifted, so
/// a groove across the diagonal makes new vertices on the seam.
fn two_chart_top(picture: &Arc<TestPicture>) -> Mesh {
    let mut cube = textured_cube(picture);
    let t = cube.face_textures[&9].clone();
    let shift = |uv: Vector2Float| Vector2Float::new(f64::from(uv.x), f64::from(uv.y) + 0.4);
    cube.face_textures.insert(
        9,
        FaceTextureData::new(t.image.clone(), shift(t.uv0), shift(t.uv1), shift(t.uv2)),
    );
    cube
}

/// The top side as one continuous chart that is not one affine map: face 9's corner off the
/// shared diagonal is moved in UV.
fn bent_chart_top(picture: &Arc<TestPicture>) -> Mesh {
    let mut cube = textured_cube(picture);
    let t = cube.face_textures[&9].clone();
    let moved = Vector2Float::new(f64::from(t.uv2.x) + 0.03, f64::from(t.uv2.y) + 0.05);
    cube.face_textures.insert(
        9,
        FaceTextureData::new(t.image.clone(), t.uv0, t.uv1, moved),
    );
    cube
}

fn grooves() -> [Matrix4X4; 3] {
    [
        Matrix4X4::create_translation(1.3, 5.0, 0.0),
        Matrix4X4::create_rotation_y(0.3) * Matrix4X4::create_translation(1.3, 5.0, 0.0),
        Matrix4X4::create_rotation_y(0.785) * Matrix4X4::create_translation(-0.7, 5.0, 0.9),
    ]
}

/// Whether some textured result face has points just inside its corners on both top faces: one
/// face stretched over the seam between them.
fn spans_the_top_seam(result: &Mesh, source: &Mesh) -> bool {
    (0..result.faces.len())
        .filter(|face| result.face_textures.contains_key(&(*face as i32)))
        .any(|face| {
            let corners = corners_of(result, face);
            let mut on = [false; 2];
            for w in [[0.96, 0.02, 0.02], [0.02, 0.96, 0.02], [0.02, 0.02, 0.96]] {
                let p = corners[0] * w[0] + corners[1] * w[1] + corners[2] * w[2];
                if let Some((under, _)) = face_under(source, p) {
                    for (side, &top) in on.iter_mut().zip(&TOP) {
                        *side |= under == top;
                    }
                }
            }
            on[0] && on[1]
        })
}

fn assert_exact(findings: &Findings, what: &str) {
    assert!(findings.textured_faces > 0, "{what}: {findings:?}");
    assert!(findings.worst_uv <= UV_TOLERANCE, "{what}: {findings:?}");
    assert!(
        findings.worst_colour <= COLOUR_TOLERANCE,
        "{what}: {findings:?}"
    );
}

#[test]
fn a_groove_across_a_uv_seam_on_a_flat_side_keeps_both_charts() {
    let picture = banded_picture(0.0);
    let cube = two_chart_top(&picture);
    let groove = create_cube(2.0, 2.0, 20.0);
    for (index, matrix) in grooves().into_iter().enumerate() {
        for operation in [CsgModes::Subtract, CsgModes::Union] {
            let result = run(
                (&cube, Matrix4X4::IDENTITY),
                (&groove, matrix),
                operation,
                None,
            );
            let what = format!("groove {index} {operation:?}");
            assert!(!spans_the_top_seam(&result, &cube), "{what}");
            let findings = check(
                &result,
                &[Source::new(&cube, Matrix4X4::IDENTITY, &picture)],
            );
            assert_exact(&findings, &what);
        }
    }
}

#[test]
fn a_groove_across_a_bent_chart_on_a_flat_side_keeps_its_mapping() {
    let picture = banded_picture(0.0);
    let cube = bent_chart_top(&picture);
    let groove = create_cube(2.0, 2.0, 20.0);
    for (index, matrix) in grooves().into_iter().enumerate() {
        for operation in [CsgModes::Subtract, CsgModes::Union] {
            let result = run(
                (&cube, Matrix4X4::IDENTITY),
                (&groove, matrix),
                operation,
                None,
            );
            let what = format!("groove {index} {operation:?}");
            assert!(!spans_the_top_seam(&result, &cube), "{what}");
            let findings = check(
                &result,
                &[Source::new(&cube, Matrix4X4::IDENTITY, &picture)],
            );
            assert_exact(&findings, &what);
        }
    }
}

#[test]
fn a_self_intersecting_textured_part_keeps_its_picture() {
    // Two overlapping cubes in one mesh intersect themselves, which sends the boolean down the
    // kernel's robust path.
    let picture = banded_picture(0.0);
    let mut both = textured_cube(&picture);
    let mut second = textured_cube(&picture);
    second.transform(
        Matrix4X4::create_rotation_z(0.4) * Matrix4X4::create_translation(3.0, 2.0, 1.0),
    );
    let offset = both.vertices.len() as i32;
    both.vertices.extend(second.vertices.iter().copied());
    for (face, f) in second.faces.iter().enumerate() {
        let index = both.faces.len() as i32;
        both.faces
            .add(f.v0 + offset, f.v1 + offset, f.v2 + offset, &both.vertices);
        both.face_textures
            .insert(index, second.face_textures[&(face as i32)].clone());
    }
    let cutter = create_cube(7.0, 7.0, 7.0);
    for operation in [CsgModes::Union, CsgModes::Subtract, CsgModes::Intersect] {
        let result = run(
            (&both, Matrix4X4::IDENTITY),
            (&cutter, tilted_cutter_matrix()),
            operation,
            None,
        );
        let findings = check(
            &result,
            &[Source::new(&both, Matrix4X4::IDENTITY, &picture)],
        );
        assert_exact(&findings, &format!("{operation:?}"));
    }
}

#[test]
fn a_textured_part_with_face_colours_keeps_both() {
    let picture = banded_picture(0.0);
    let mut cube = textured_cube(&picture);
    let green = Color::rgb(0x30, 0xC0, 0x40);
    let colour_of = |face: usize| if face.is_multiple_of(4) { green } else { RED };
    cube.face_colors = Some((0..cube.faces.len()).map(colour_of).collect());
    let result = run(
        (&cube, textured_matrix()),
        (&create_cube(7.0, 7.0, 7.0), tilted_cutter_matrix()),
        CsgModes::Subtract,
        Some(vec![BLUE, BLUE]),
    );
    let source = Source::new(&cube, textured_matrix(), &picture);
    let colours = result.face_colors.as_ref().expect("colours were asked for");
    assert_eq!(colours.len(), result.faces.len());
    for (face, &colour) in colours.iter().enumerate() {
        let corners = corners_of(&result, face);
        let centre = (corners[0] + corners[1] + corners[2]) / 3.0;
        let want = face_under(&source.placed, centre).map_or(BLUE, |(under, _)| colour_of(under));
        assert_eq!(colour, want, "face {face}");
    }
    assert_exact(&check(&result, &[source]), "coloured");
}

#[test]
fn a_texture_entry_without_a_picture_stays_untextured() {
    let picture = banded_picture(0.0);
    let mut cube = textured_cube(&picture);
    let bare = 4;
    let t = cube.face_textures[&bare].clone();
    cube.face_textures
        .insert(bare, FaceTextureData::new(None, t.uv0, t.uv1, t.uv2));
    let result = run(
        (&cube, Matrix4X4::IDENTITY),
        (&create_cube(7.0, 7.0, 7.0), tilted_cutter_matrix()),
        CsgModes::Union,
        None,
    );
    let mut bare_faces = 0;
    for face in 0..result.faces.len() {
        let corners = corners_of(&result, face);
        let centre = (corners[0] + corners[1] + corners[2]) / 3.0;
        if face_under(&cube, centre).is_some_and(|(under, _)| under == bare as usize) {
            bare_faces += 1;
            assert!(
                !result.face_textures.contains_key(&(face as i32)),
                "face {face} came from a face with no picture"
            );
        }
    }
    assert!(bare_faces > 0);
    // Every other face of the cube keeps its picture exactly.
    let findings = check(
        &result,
        &[Source::new(&cube, Matrix4X4::IDENTITY, &picture)],
    );
    assert_exact(&findings, "no-picture entry");
}

#[test]
fn a_non_finite_uv_goes_in_as_zero_and_the_boolean_still_runs() {
    let picture = banded_picture(0.0);
    let mut cube = textured_cube(&picture);
    let t = cube.face_textures[&0].clone();
    let nan = Vector2Float {
        x: f32::NAN,
        y: f32::INFINITY,
    };
    cube.face_textures
        .insert(0, FaceTextureData::new(t.image.clone(), nan, t.uv1, t.uv2));
    let result = run(
        (&cube, Matrix4X4::IDENTITY),
        (&create_cube(7.0, 7.0, 7.0), tilted_cutter_matrix()),
        CsgModes::Union,
        None,
    );
    assert!(!result.face_textures.is_empty());
    for (face, t) in &result.face_textures {
        for uv in [t.uv0, t.uv1, t.uv2] {
            assert!(uv.x.is_finite() && uv.y.is_finite(), "face {face}: {uv:?}");
        }
    }
    let zero_corner = result.face_textures.values().any(|t| {
        [t.uv0, t.uv1, t.uv2]
            .iter()
            .any(|uv| uv.x == 0.0 && uv.y == 0.0)
    });
    assert!(zero_corner, "the non-finite corner came back as (0, 0)");
}
