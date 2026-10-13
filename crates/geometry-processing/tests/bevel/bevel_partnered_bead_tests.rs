//! Port of `BevelPartneredBeadTests.cs`: the nose-in-notch part's four creases, each a bead built
//! from its rolling balls and its partners' tubes.

use std::rc::Rc;

use polygon_mesh::csg::boolean_processing::do_array;
use polygon_mesh::csg::{BooleanOptions, CsgModes};
use polygon_mesh::{CancelToken, Mesh};

use geometry_processing::bevel::bevel_face_fit::DEFAULT_SURFACE_TOLERANCE_DEGREES;
use geometry_processing::bevel::bevel_partnered_bead::try_build;
use geometry_processing::bevel::bevel_run_partners::{for_chain, try_ball_centre_line};
use geometry_processing::bevel::{BevelFaceRunOut, BevelPartnerTube, BevelSweptChain};
use geometry_processing::topology::{ChainEdge, EdgeConvexity, FeatureEdgeSet};
use vector_math::{Matrix4X4, Vector3};

use crate::mesh_measurements::{shell_count, signed_volume};
use crate::nose_in_notch_test_meshes as nose;

const RADIUS: f64 = 5.0;

const SEGMENTS: i32 = 15;

const OVERLAP: f64 = 1e-3 + 1e-6;

fn corner_edge_tangent() -> Vector3 {
    Vector3::new(0.0, 0.0, 10.3189)
}

fn centre_crease_tangent() -> Vector3 {
    Vector3::new(10.0, 10.0, 16.9856)
}

fn setup() -> (Vec<BevelSweptChain>, Vec<String>, Rc<BevelFaceRunOut>) {
    let part = nose::create();
    let (runs, names) = nose::creases(&part);
    let run_out = Rc::new(BevelFaceRunOut::create(
        std::sync::Arc::new(part),
        None,
        DEFAULT_SURFACE_TOLERANCE_DEGREES,
    ));
    (runs, names, run_out)
}

fn is_a_ridge_end(vertex: Vector3) -> bool {
    (vertex - nose::left_ridge_end()).length() < 1e-6
        || (vertex - nose::right_ridge_end()).length() < 1e-6
}

fn sagitta_of(run: &BevelSweptChain) -> f64 {
    let edge = run.edges[0].edge;
    let sweep = edge.normal0.dot(edge.normal1).clamp(-1.0, 1.0).acos();

    RADIUS * (1.0 - (sweep / (2.0 * f64::from(SEGMENTS + 2))).cos())
}

/// C# `EveryOneOfTheFourCreasesBuildsItsBead`.
#[test]
fn every_one_of_the_four_creases_builds_its_bead() {
    let (runs, names, run_out) = setup();

    for i in 0..runs.len() {
        if let Err(refusal) = try_build(
            &runs[i],
            &runs,
            RADIUS,
            SEGMENTS,
            Some(run_out.clone()),
            None,
        ) {
            panic!(
                "the {} crease has a straight partner at both of its ends, so its bead is one this builds. Instead: {refusal}",
                names[i]
            );
        }
    }
}

/// C# `PastTheEndsOfItsCreaseNoBeadReachesIntoAPartnersTube`.
#[test]
fn past_the_ends_of_its_crease_no_bead_reaches_into_a_partners_tube() {
    let (runs, names, run_out) = setup();

    for i in 0..runs.len() {
        let bead = try_build(
            &runs[i],
            &runs,
            RADIUS,
            SEGMENTS,
            Some(run_out.clone()),
            None,
        )
        .unwrap_or_else(|refusal| panic!("{refusal}"));

        let (axis_point, direction) = try_ball_centre_line(&runs[i], RADIUS).unwrap();
        let along = |point: Vector3| (point - axis_point).dot(direction);

        let crease_start = along(runs[i].edges[0].from());
        let crease_end = along(runs[i].edges[runs[i].edges.len() - 1].to());
        let (partners, _) = for_chain(&runs[i], &runs, RADIUS);

        let at_the_start = is_a_ridge_end(runs[i].edges[0].from());
        let past_the_ends: Vec<Vector3> = bead
            .vertices
            .iter()
            .map(|&vertex| Vector3::from_vector3_float(vertex))
            .filter(|&vertex| {
                if at_the_start {
                    along(vertex) < crease_start.min(crease_end) - 1e-6
                } else {
                    along(vertex) > crease_start.max(crease_end) + 1e-6
                }
            })
            .collect();

        assert!(
            !past_the_ends.is_empty(),
            "the {} bead runs 6.26 mm past the ridge end of its crease",
            names[i]
        );

        let allowed = OVERLAP
            + runs
                .iter()
                .map(sagitta_of)
                .fold(f64::NEG_INFINITY, f64::max);

        for partner in &partners {
            let (depth, at) = past_the_ends
                .iter()
                .map(|&vertex| (partner.depth(vertex), vertex))
                .min_by(|a, b| a.0.total_cmp(&b.0))
                .unwrap();

            assert!(
                depth > -allowed,
                "the {} bead has a ridge run-out vertex {at:?} {} mm inside a partner's cylinder, deeper than the {allowed} mm allowed",
                names[i],
                -depth
            );
        }
    }
}

/// C# `TheBeadRunsPastBothEndsOfItsCreaseAsFarAsTheControlDoes`.
#[test]
fn the_bead_runs_past_both_ends_of_its_crease_as_far_as_the_control_does() {
    let (runs, names, run_out) = setup();

    let upper_left = &runs[names.iter().position(|n| n == "upper left").unwrap()];
    let bead = try_build(upper_left, &runs, RADIUS, SEGMENTS, Some(run_out), None)
        .unwrap_or_else(|refusal| panic!("{refusal}"));

    let (axis_point, direction) = try_ball_centre_line(upper_left, RADIUS).unwrap();
    let along = |point: Vector3| (point - axis_point).dot(direction);

    let reached: Vec<f64> = bead
        .vertices
        .iter()
        .map(|&vertex| along(Vector3::from_vector3_float(vertex)))
        .collect();
    let min = reached.iter().copied().fold(f64::INFINITY, f64::min);
    let max = reached.iter().copied().fold(f64::NEG_INFINITY, f64::max);

    assert!(
        min <= along(corner_edge_tangent()) + 0.01,
        "the control's bead reaches (0, 0, 10.3189) on the cube's front-left corner edge: {min}"
    );
    assert!(
        max >= along(centre_crease_tangent()) - 0.01,
        "and it reaches (10, 10, 16.9856) on the notch's centre crease: {max}"
    );

    let spacing = 2.0 * RADIUS * (std::f64::consts::PI / 2.0 / f64::from(SEGMENTS + 3)).sin();

    assert!(
        max <= along(centre_crease_tangent()) + (2.0 * spacing),
        "the bead is carried one station past the wall to bury its cap, and no further"
    );
}

/// C# `APartnerRidingAnotherFaceCoversNothingOnThisRunsFace`.
#[test]
fn a_partner_riding_another_face_covers_nothing_on_this_runs_face() {
    let (runs, names, run_out) = setup();
    let index_of = |name: &str| names.iter().position(|n| n == name).unwrap();

    let upper_left = &runs[index_of("upper left")];
    let bead = try_build(upper_left, &runs, RADIUS, SEGMENTS, Some(run_out), None)
        .unwrap_or_else(|refusal| panic!("{refusal}"));

    let (axis_point, direction) = try_ball_centre_line(upper_left, RADIUS).unwrap();
    let along = |point: Vector3| (point - axis_point).dot(direction);

    let crease_end = along(upper_left.edges[0].from())
        .max(along(upper_left.edges[upper_left.edges.len() - 1].to()));

    let upper_right = &runs[index_of("upper right")];
    let (other_point, other_direction) = try_ball_centre_line(upper_right, RADIUS).unwrap();

    assert!(
        other_direction.dot(nose::left_slope_normal()).abs() > 0.01,
        "the partner does not ride this run's own slope"
    );

    let tube = BevelPartnerTube::new(other_point, other_direction, RADIUS);

    let run_out_vertices: Vec<Vector3> = bead
        .vertices
        .iter()
        .map(|&vertex| Vector3::from_vector3_float(vertex))
        .filter(|&vertex| along(vertex) > crease_end + 1e-6)
        .collect();

    assert!(
        !run_out_vertices.is_empty(),
        "the upper left bead runs 1.556 mm past its crease onto the centre crease"
    );

    let deepest = run_out_vertices
        .iter()
        .map(|&vertex| tube.depth(vertex))
        .fold(f64::INFINITY, f64::min);

    assert!(
        deepest < -0.1,
        "this run-out is SUPPOSED to reach well inside the other-face partner's cylinder: {deepest}"
    );

    let proud = run_out_vertices
        .iter()
        .map(|&vertex| (vertex - axis_point - (direction * along(vertex))).length() - RADIUS)
        .fold(f64::NEG_INFINITY, f64::max);

    assert!(
        proud < 2.0 * RADIUS,
        "the run-out closes on the wall it runs out onto: {proud}"
    );
}

/// C# `NoBeadStandsPastTheKnifeTipItsFaceEndsOn`.
#[test]
fn no_bead_stands_past_the_knife_tip_its_face_ends_on() {
    let (runs, names, run_out) = setup();
    let clearance = RADIUS * 0.01;

    for i in 0..runs.len() {
        let bead = try_build(
            &runs[i],
            &runs,
            RADIUS,
            SEGMENTS,
            Some(run_out.clone()),
            None,
        )
        .unwrap_or_else(|refusal| panic!("{refusal}"));

        let proud = bead
            .vertices
            .iter()
            .map(|&vertex| {
                let x = f64::from(vertex.x);
                (-x).max(x - nose::SIZE)
            })
            .fold(f64::NEG_INFINITY, f64::max);

        assert!(
            proud <= clearance,
            "the {} bead reaches {proud} mm outside a part that runs from 0 to {} in x",
            names[i],
            nose::SIZE
        );
    }
}

fn union(operands: &[&Mesh]) -> Option<Mesh> {
    let items: Vec<(&Mesh, Matrix4X4)> =
        operands.iter().map(|&m| (m, Matrix4X4::IDENTITY)).collect();
    do_array(
        &items,
        CsgModes::Union,
        &CancelToken::new(),
        &BooleanOptions::default(),
    )
    .ok()
    .filter(|mesh| !mesh.vertices.is_empty())
}

/// C# `TheFourBeadsAndThePartMakeOneSolidWithNoVoidBetweenThem`.
#[test]
fn the_four_beads_and_the_part_make_one_solid_with_no_void_between_them() {
    let part = nose::create();
    let (runs, _, run_out) = setup();

    let beads: Vec<Mesh> = runs
        .iter()
        .map(|run| {
            try_build(run, &runs, RADIUS, SEGMENTS, Some(run_out.clone()), None)
                .unwrap_or_else(|refusal| panic!("{refusal}"))
        })
        .collect();

    let mut operands = vec![&part];
    operands.extend(beads.iter());

    let union =
        union(&operands).expect("four beads added to one part have to union into something");

    assert_eq!(
        shell_count(&union),
        1,
        "the four beads all meet the part and each other"
    );
    assert!(
        signed_volume(&union) > nose::VOLUME,
        "every one of the four runs is concave, so the fillet can only ADD material"
    );
}

/// C# `TheClearanceIsBuriedAndChangesNothingOutsideThePart`.
#[test]
fn the_clearance_is_buried_and_changes_nothing_outside_the_part() {
    let part = nose::create();
    let (runs, names, run_out) = setup();
    const A_HAIR: f64 = 1e-3;
    const PROUD_ALLOWANCE: f64 = 0.05;

    for i in 0..runs.len() {
        let buried = try_build(
            &runs[i],
            &runs,
            RADIUS,
            SEGMENTS,
            Some(run_out.clone()),
            None,
        )
        .unwrap_or_else(|refusal| panic!("{refusal}"));
        let bare = try_build(
            &runs[i],
            &runs,
            RADIUS,
            SEGMENTS,
            Some(run_out.clone()),
            Some(A_HAIR),
        )
        .unwrap_or_else(|refusal| panic!("{refusal}"));

        let with_clearance = union(&[&part, &buried])
            .unwrap_or_else(|| panic!("the {} bead has to UNION with the part", names[i]));
        let without = union(&[&part, &bare]).expect("the bare-sided bead has to union too");

        assert!(
            (signed_volume(&with_clearance) - signed_volume(&without)).abs() <= PROUD_ALLOWANCE,
            "the {} bead's clearance runs into material the part already has",
            names[i]
        );
    }
}

/// C# `TheSameFourBeadsBuildAndUnionAtAHundredthAndAtTenTimesTheRadius`, one test per C# case.
fn the_same_four_beads_build_and_union_at_a_hundredth_and_at_ten_times_the_radius(
    scale: f64,
    radius: f64,
) {
    let part = nose::create_scaled(scale);
    let (runs, names) = nose::creases_scaled(&part, scale);
    let run_out = Rc::new(BevelFaceRunOut::create(
        std::sync::Arc::new(part.clone()),
        None,
        DEFAULT_SURFACE_TOLERANCE_DEGREES,
    ));

    let beads: Vec<Mesh> = (0..runs.len())
        .map(|i| {
            try_build(&runs[i], &runs, radius, SEGMENTS, Some(run_out.clone()), None).unwrap_or_else(
                |refusal| {
                    panic!(
                        "the {} crease is the same crease at radius {radius} on a part {scale} times the size. Instead: {refusal}",
                        names[i]
                    )
                },
            )
        })
        .collect();

    let mut operands = vec![&part];
    operands.extend(beads.iter());

    let union = union(&operands).expect("four beads added to one part have to union at any size");

    assert_eq!(shell_count(&union), 1);
    assert!(signed_volume(&union) > nose::VOLUME * scale * scale * scale);
}

#[test]
fn the_same_four_beads_build_and_union_at_a_hundredth_and_at_ten_times_the_radius_0_1_0_5() {
    the_same_four_beads_build_and_union_at_a_hundredth_and_at_ten_times_the_radius(0.1, 0.5);
}

#[test]
fn the_same_four_beads_build_and_union_at_a_hundredth_and_at_ten_times_the_radius_10_50() {
    the_same_four_beads_build_and_union_at_a_hundredth_and_at_ten_times_the_radius(10.0, 50.0);
}

/// C# `ABossCornerIsHandedBackBecauseNothingRunsOutThere`.
#[test]
fn a_boss_corner_is_handed_back_because_nothing_runs_out_there() {
    const BOSS_RADIUS: f64 = 2.0;

    let boss = crate::topology_test_meshes::create_square_boss_on_plate();
    let features = FeatureEdgeSet::create(&boss);
    let run_out = Rc::new(BevelFaceRunOut::create(
        std::sync::Arc::new(boss.clone()),
        None,
        DEFAULT_SURFACE_TOLERANCE_DEGREES,
    ));

    let feet: Vec<BevelSweptChain> = features
        .edges()
        .iter()
        .enumerate()
        .filter(|(_, edge)| edge.convexity == EdgeConvexity::Concave)
        .map(|(i, edge)| BevelSweptChain::new(vec![ChainEdge::new(i as i32, *edge, false)], false))
        .collect();

    assert_eq!(
        feet.len(),
        4,
        "the boss's foot is the only inside corner on the part"
    );

    for foot in &feet {
        assert!(
            try_build(foot, &feet, BOSS_RADIUS, SEGMENTS, Some(run_out.clone()), None).is_err(),
            "nothing runs out at a boss corner, so this run belongs to the sweep - refused in words"
        );
    }
}
