//! Port of `LevelLine/CreaseRunOutTests.cs`: where a fillet's run-out along an unpicked crease
//! ends, against the Onshape control.

use geometry_processing::bevel::level_line::CreaseRunOut;
use vector_math::math_helper::radians_to_degrees;
use vector_math::Vector3;

use crate::level_line_feature_edges::feature_edges_over;
use crate::level_line_fixture::LevelLineFixture;
use crate::nose_in_notch_test_meshes as nose;

const CONTROL_REACH: f64 = 4.5846;

const CONTROL_END_DEGREES: f64 = 158.5036;

fn find(fixture: &LevelLineFixture) -> Vec<CreaseRunOut> {
    CreaseRunOut::find(
        &fixture.positions,
        &fixture.indices,
        &fixture.chains,
        &fixture.closed,
        &|a, b| fixture.is_feature_edge(a, b),
        fixture.size,
        16,
    )
}

/// C# `TheNoseFixtureRunsOutAtBothRidgeEndsLikeTheControl`.
#[test]
fn the_nose_fixture_runs_out_at_both_ridge_ends_like_the_control() {
    let fixture = LevelLineFixture::load("top-center-face-nose-r1");
    let run_outs = find(&fixture);
    assert_the_controls_run_outs(&run_outs, fixture.size);
    let mut corners: Vec<i32> = run_outs.iter().map(|r| r.corner).collect();
    corners.sort();
    assert_eq!(corners, vec![1, 9]);
}

/// C# `TheBuiltNoseRunsOutAtBothRidgeEndsLikeTheControl`.
#[test]
fn the_built_nose_runs_out_at_both_ridge_ends_like_the_control() {
    let (positions, indices, chain) = built_nose();
    let is_feature = feature_edges_over(&positions, &indices, 20.0);
    let run_outs = CreaseRunOut::find(
        &positions,
        &indices,
        &[chain],
        &[true],
        &*is_feature,
        1.0,
        16,
    );
    assert_the_controls_run_outs(&run_outs, 1.0);
    let ends: Vec<Vector3> = run_outs.iter().map(|r| r.corner_point).collect();
    assert!(ends
        .iter()
        .any(|&p| (p - nose::left_ridge_end()).length() < 1e-4));
    assert!(ends
        .iter()
        .any(|&p| (p - nose::right_ridge_end()).length() < 1e-4));
}

/// C# `TheMirroredNoseReadsTheMirroredRunOuts`.
#[test]
fn the_mirrored_nose_reads_the_mirrored_run_outs() {
    let fixture = LevelLineFixture::load("top-center-face-nose-r1");
    let run_outs = find(&fixture);

    let mirror = |p: Vector3| Vector3::new(-p.x, p.y, p.z);
    let mirrored_positions: Vec<Vector3> = fixture.positions.iter().map(|&p| mirror(p)).collect();
    let mirrored_indices: Vec<(i32, i32, i32)> =
        fixture.indices.iter().map(|&(a, b, c)| (a, c, b)).collect();
    let mirrored = CreaseRunOut::find(
        &mirrored_positions,
        &mirrored_indices,
        &fixture.chains,
        &fixture.closed,
        &|a, b| fixture.is_feature_edge(a, b),
        fixture.size,
        16,
    );

    assert_eq!(mirrored.len(), run_outs.len());
    for run_out in &run_outs {
        let twins: Vec<&CreaseRunOut> = mirrored
            .iter()
            .filter(|m| m.corner == run_out.corner)
            .collect();
        assert_eq!(twins.len(), 1);
        let twin = twins[0];
        assert!((twin.end() - mirror(run_out.end())).length() < 1e-9);
        assert!((twin.end_turn_radians - run_out.end_turn_radians).abs() < 1e-9);
        for i in 0..run_out.wall_contact.len() {
            assert!((twin.wall_contact[i] - mirror(run_out.wall_contact[i])).length() < 1e-9);
            assert!((twin.tube_contact[i] - mirror(run_out.tube_contact[i])).length() < 1e-9);
        }
    }
}

/// C# `NoRunOutWhereNoUnpickedCreaseEndsAMixedCorner`, one test per C# case.
fn no_run_out_where_no_unpicked_crease_ends_a_mixed_corner(name: &str) {
    let fixture = LevelLineFixture::load(name);
    let run_outs = find(&fixture);
    let said: Vec<String> = run_outs
        .iter()
        .map(|r| format!("v{} along v{} reach {:.4}", r.corner, r.crease_end, r.reach))
        .collect();
    assert_eq!(run_outs.len(), 0, "{}", said.join("; "));
}

#[test]
fn no_run_out_where_no_unpicked_crease_ends_a_mixed_corner_inner_right_sides_and_center_op0() {
    no_run_out_where_no_unpicked_crease_ends_a_mixed_corner("inner-right-sides-and-center-op0");
}

#[test]
fn no_run_out_where_no_unpicked_crease_ends_a_mixed_corner_full_front_ridge_and_hips() {
    no_run_out_where_no_unpicked_crease_ends_a_mixed_corner("full-front-ridge-and-hips");
}

#[test]
fn no_run_out_where_no_unpicked_crease_ends_a_mixed_corner_top_center_face_f6_r01() {
    no_run_out_where_no_unpicked_crease_ends_a_mixed_corner("top-center-face-f6-r01");
}

fn assert_the_controls_run_outs(run_outs: &[CreaseRunOut], size: f64) {
    assert_eq!(run_outs.len(), 2);
    for run_out in run_outs {
        let said = format!(
            "at v{} {:?}: canal {:.4}, end {:.4}, reach {:.4} of {:.4}",
            run_out.corner,
            run_out.corner_point,
            radians_to_degrees(run_out.canal_turn_radians),
            radians_to_degrees(run_out.end_turn_radians),
            run_out.reach,
            run_out.crease_length
        );
        assert!(
            (run_out.reach - CONTROL_REACH * size).abs() <= 1e-3,
            "{said}"
        );
        assert!(
            (radians_to_degrees(run_out.end_turn_radians) - CONTROL_END_DEGREES).abs() <= 1e-3,
            "{said}"
        );
        assert!(
            (radians_to_degrees(run_out.canal_turn_radians) - 112.62).abs() <= 1e-2,
            "{said}"
        );

        // The ball dies TANGENT to the crease's line: its centre stands one radius from it.
        assert!(
            (run_out.distance_to_crease_line(run_out.centre_at(run_out.end_turn_radians)) - size)
                .abs()
                <= 1e-6,
            "{said}"
        );

        // The wall contact ends on the crease at P*.
        assert!(
            (run_out.wall_contact[run_out.wall_contact.len() - 1] - run_out.end()).length() < 1e-6,
            "{said}"
        );
        assert_eq!(run_out.wall_contact.len(), run_out.tube_contact.len());
    }
}

/// Positions, triangles and one picked chain.
type BuiltPick = (Vec<Vector3>, Vec<(i32, i32, i32)>, Vec<i32>);

/// C# `Nose()`: the built nose, with the pick the fixture walks round its upper face.
fn built_nose() -> BuiltPick {
    let mut mesh = nose::create();
    mesh.clean_and_merge();
    let positions: Vec<Vector3> = mesh
        .vertices
        .iter()
        .map(|&v| Vector3::from_vector3_float(v))
        .collect();
    let indices: Vec<(i32, i32, i32)> = mesh.faces.iter().map(|f| (f.v0, f.v1, f.v2)).collect();
    let find = |p: Vector3| {
        positions
            .iter()
            .position(|&q| (q - p).length() < 1e-4)
            .map_or(-1, |i| i as i32)
    };
    let chain = vec![
        find(nose::upper_crease_end()),
        find(nose::left_ridge_end()),
        find(nose::right_ridge_end()),
    ];

    (positions, indices, chain)
}
