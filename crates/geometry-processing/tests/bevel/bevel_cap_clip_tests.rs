//! The end cap the sweep cannot carry, and what `bevel_cap_clip` is allowed to do about it. Every
//! test is arithmetic on a cube or a hand-made plane - no kernel - because the three things worth
//! pinning are decisions: where the half space sits, that only one cap is ever demoted, and that a
//! clip the kernel refuses costs the group rather than shipping the untrimmed tool.
//!
//! The combiner returns a `Result<Mesh, _>`, and "the answer it will not vouch for" is an empty
//! mesh, which is no usable operand.

use geometry_processing::bevel::bevel_cap_clip::{self, Candidate, ClippedCap};
use geometry_processing::bevel::bevel_chain_linker;
use geometry_processing::bevel::bevel_chain_sweep::{self, BevelSweptSection};
use geometry_processing::bevel::{
    BevelAdvisoryArg, BevelAdvisoryCode, BevelChainEnds, BevelEndCap,
};
use geometry_processing::topology::bevel_edge_group_resolver;
use geometry_processing::topology::{BevelSelectionKind, ChainEdge, FaceGroup, FeatureEdgeSet};
use polygon_mesh::csg::{BooleanError, CsgModes, KernelStatus};
use polygon_mesh::solids::platonic_solids;
use polygon_mesh::Mesh;
use std::cell::Cell;
use vector_math::{math_helper, AxisAlignedBoundingBox, Matrix4X4, Vector3};

const CUBE_SIZE: f64 = 20.0;

/// How far off square the hand-made cap planes lean: inside the 80 degrees the chain ends will
/// still cap against, and steep enough that the slide onto one of them outruns the whole run.
const LEAN_DEGREES: f64 = 79.0;

/// Slack for a point that is meant to be exactly on a plane: mesh vertices are float.
const ON_PLANE: f64 = 1e-4;

#[test]
fn the_half_space_starts_on_the_cap_plane_and_fills_the_side_the_cap_keeps_the_tool_from() {
    // A plane leaning well off any axis, so a rotation that only half works cannot pass by luck
    let normal = Vector3::new(1.0, -2.0, 3.0).get_normal();
    let through = Vector3::new(4.0, 5.0, 6.0);
    let cap = BevelEndCap::new(normal, normal.dot(through));

    let bounds = AxisAlignedBoundingBox::new(Vector3::ZERO, Vector3::new(8.0, 10.0, 12.0));
    let cube = bevel_cap_clip::half_space(&cap, &bounds);

    let span = bevel_cap_clip::SPAN_IN_DIAGONALS * bounds.size().length();

    // Every vertex of the box lies between the plane and one span in front of it.
    let distances: Vec<f64> = cube
        .vertices
        .iter()
        .map(|vertex| normal.dot(Vector3::from_vector3_float(*vertex)) - cap.plane_distance)
        .collect();
    let min = distances.iter().copied().fold(f64::INFINITY, f64::min);
    let max = distances.iter().copied().fold(f64::NEG_INFINITY, f64::max);

    assert!(
        min.abs() <= span * 1e-5,
        "the box's near face is the cap plane itself, so its nearest vertices sit on it"
    );
    assert!(
        (max - span).abs() <= span * 1e-5,
        "and the far face is one full span forward, on the side the cap keeps the tool out of"
    );
    assert_eq!(
        distances
            .iter()
            .filter(|&&distance| distance < span / 2.0)
            .count(),
        4,
        "a cube turned onto the normal presents one square face to the plane and one away from it"
    );

    // And the probe either side of it, which is what a tilted box would fail
    let foot_of_the_plane = through - (normal * normal.dot(through - bounds.center()));

    assert!(
        inside(&cube, foot_of_the_plane + (normal * 0.01)),
        "a point just past the plane is material the cap exists to keep the tool out of"
    );
    assert!(
        !inside(&cube, foot_of_the_plane - (normal * 0.01)),
        "and a point just short of it is the tool's own side, which the clip must not touch"
    );
}

#[test]
fn the_half_space_swallows_everything_of_the_tool_that_lies_past_the_plane() {
    let normal = Vector3::new(0.6, 0.8, 0.0).get_normal();
    let bounds =
        AxisAlignedBoundingBox::new(Vector3::new(-5.0, -5.0, -5.0), Vector3::new(5.0, 5.0, 5.0));

    // Through the middle of the tool, so half of its corners are on each side.
    let cap = BevelEndCap::new(normal, normal.dot(bounds.center()));
    let cube = bevel_cap_clip::half_space(&cap, &bounds);

    let mut past = 0;
    for corner in corners(&bounds) {
        if normal.dot(corner) - cap.plane_distance <= 0.0 {
            continue;
        }

        past += 1;
        assert!(
            inside(&cube, corner),
            "the corner at {corner:?} is past the cap plane, so the clip has to reach it"
        );
    }

    assert!(
        past > 0,
        "a plane through the middle of a box leaves corners on both sides of it, or this test proves nothing"
    );
}

#[test]
fn neither_cap_is_ever_dropped_without_the_other_surviving() {
    let start = BevelEndCap::new(Vector3::new(-1.0, 0.0, 0.0), 3.0);
    let end = BevelEndCap::new(Vector3::new(1.0, 0.0, 0.0), 7.0);

    let attempts =
        bevel_cap_clip::attempts_for(&BevelChainEnds::new(start.clone(), end.clone()), &run());

    assert_eq!(
        attempts.len(),
        2,
        "two capped ends offer exactly two demotions - this one, or that one, and never the pair"
    );

    for (ends, clips) in &attempts {
        assert_eq!(clips.len(), 1, "one attempt hands the boolean one cap");
        assert!(
            ends.start.is_capped() || ends.end.is_capped(),
            "the end that was not demoted is still swept, which is what the sweep has to survive"
        );
    }

    assert!(!attempts[0].0.start.is_capped());
    assert_eq!(attempts[0].1[0].plane.plane_distance, start.plane_distance);
    assert!(!attempts[1].0.end.is_capped());
    assert_eq!(attempts[1].1[0].plane.plane_distance, end.plane_distance);
}

#[test]
fn a_run_no_demotion_rescues_keeps_the_first_attempts_refusal() {
    const RADIUS: f64 = 12.0;
    const SEGMENTS: i32 = 8;

    let run = run();
    let last = run.len() - 1;

    // Two planes leaning 79 degrees off square at the two ends of a 20 mm edge: ONE of these
    // alone drags a ring 17 * tan(79) = 87 mm along a 20 mm run, so every demotion folds as surely
    // as the pair did, and the loop below says so out loud.
    let ends = BevelChainEnds::new(
        leaning_cap(run[0].from(), -run[0].direction(), run[0].direction()),
        leaning_cap(run[last].to(), run[last].direction(), run[last].direction()),
    );

    let sweep = |ends: &BevelChainEnds| {
        bevel_chain_sweep::try_create(
            &run,
            false,
            Some(ends),
            RADIUS,
            SEGMENTS,
            true,
            BevelSweptSection::Tool,
            0.0,
            0.0,
            None,
            0.0,
            None,
        )
    };

    let both_caps = match sweep(&ends) {
        Ok(_) => panic!(
            "this fixture only means anything while the sweep with both caps really is refused"
        ),
        Err(reason) => reason,
    };

    for (attempt_ends, _) in bevel_cap_clip::attempts_for(&ends, &run) {
        assert!(
            sweep(&attempt_ends).is_err(),
            "and while every single-cap retry folds as well - that is what makes it a run no demotion rescues"
        );
    }

    let tool =
        bevel_cap_clip::try_create(&run, false, Some(&ends), RADIUS, SEGMENTS, true, None, 0.0);

    // `Err` is no tool, with no clips owed.
    match tool {
        Ok(_) => panic!("neither single-cap retry can sweep either, so there is no tool to have"),
        Err(reason) => assert_eq!(
            reason, both_caps,
            "the refusal has to describe the tool that was asked for - instead: {reason:?}"
        ),
    }
}

/// The cases "partial", "rejected" and "null": all three of the ways a subtract can decline, one
/// remedy for all three.
fn a_clip_the_kernel_will_not_run_withdraws_the_cutter_and_fails_its_group(refusal: &str) {
    let other = platonic_solids::create_cube(4.0, 4.0, 4.0);
    let mut cutters = vec![platonic_solids::create_cube(2.0, 2.0, 2.0), other.clone()];

    let candidates = vec![Candidate::new(
        0,
        3,
        2.5,
        vec![ClippedCap::new(
            BevelEndCap::new(Vector3::new(1.0, 0.0, 0.0), 0.0),
            Vector3::new(7.0, 8.0, 9.0),
        )],
    )];

    let combine = |_: &[(Mesh, Matrix4X4)], _: CsgModes| -> Result<Mesh, BooleanError> {
        match refusal {
            "partial" => Err(BooleanError::Partial {
                message: "the kernel refused an operand".into(),
                partial_result: Box::default(),
                skipped_operands: Vec::new(),
            }),
            "rejected" => Err(BooleanError::ImportRejected {
                message: "the kernel would not import an operand".into(),
                status: KernelStatus::NotManifold,
            }),
            _ => Ok(Mesh::default()),
        }
    };

    let withdrawals =
        bevel_cap_clip::apply(&mut cutters, &candidates, &combine).expect("not cancelled");

    assert_eq!(
        withdrawals.len(),
        1,
        "the one candidate could not be clipped, so its group has to hear about it"
    );
    assert_eq!(
        withdrawals[0].group_index, 3,
        "the row that is failed is the group the withdrawn cutter came from"
    );
    assert_eq!(
        withdrawals[0].reason.code(),
        BevelAdvisoryCode::CapClipCouldNotCut,
        "the wording follows the sweep's own cap refusal - instead: {:?}",
        withdrawals[0].reason
    );
    assert_eq!(
        withdrawals[0].reason.args()[1],
        BevelAdvisoryArg::Double(2.5),
        "and it quotes the size, so retyping is an option the user can see - instead: {:?}",
        withdrawals[0].reason
    );
    assert_eq!(
        cutters.len(),
        1,
        "the cutter that could not be clipped is out of the batch, not in it untrimmed"
    );
    assert_eq!(
        cutters[0].vertices, other.vertices,
        "and the OTHER cutter is untouched - a withdrawal closes the gap it leaves, it does not shift the batch"
    );
}

#[test]
fn a_clip_the_kernel_will_not_run_withdraws_the_cutter_and_fails_its_group_partial() {
    a_clip_the_kernel_will_not_run_withdraws_the_cutter_and_fails_its_group("partial");
}

#[test]
fn a_clip_the_kernel_will_not_run_withdraws_the_cutter_and_fails_its_group_rejected() {
    a_clip_the_kernel_will_not_run_withdraws_the_cutter_and_fails_its_group("rejected");
}

#[test]
fn a_clip_the_kernel_will_not_run_withdraws_the_cutter_and_fails_its_group_null() {
    a_clip_the_kernel_will_not_run_withdraws_the_cutter_and_fails_its_group("null");
}

#[test]
fn a_clip_the_kernel_runs_replaces_the_cutter_in_place() {
    let clipped = platonic_solids::create_cube(3.0, 3.0, 3.0);
    let mut cutters = vec![platonic_solids::create_cube(2.0, 2.0, 2.0)];

    let candidates = vec![Candidate::new(
        0,
        0,
        1.0,
        vec![ClippedCap::new(
            BevelEndCap::new(Vector3::new(0.0, 0.0, 1.0), 0.0),
            Vector3::ZERO,
        )],
    )];

    let operand_count = Cell::new(0);
    let combine = |items: &[(Mesh, Matrix4X4)], mode: CsgModes| -> Result<Mesh, BooleanError> {
        operand_count.set(items.len());
        Ok(if mode == CsgModes::Subtract {
            clipped.clone()
        } else {
            Mesh::default()
        })
    };

    let withdrawals =
        bevel_cap_clip::apply(&mut cutters, &candidates, &combine).expect("not cancelled");

    assert!(withdrawals.is_empty(), "nothing was refused");
    assert_eq!(cutters.len(), 1);
    assert_eq!(
        cutters[0].vertices, clipped.vertices,
        "the clipped solid is what the batch unions, in the slot the overshooting one had"
    );
    assert_eq!(
        operand_count.get(),
        2,
        "the cutter and the one half space it owes - the clip is a subtract, not an intersect"
    );
}

/// A cap plane through a point, leaning [`LEAN_DEGREES`] off square to the run.
fn leaning_cap(through: Vector3, outward: Vector3, along_the_run: Vector3) -> BevelEndCap {
    let mut sideways = along_the_run.cross(Vector3::new(0.0, 0.0, 1.0));
    if sideways.length() < 1e-9 {
        sideways = along_the_run.cross(Vector3::new(0.0, 1.0, 0.0));
    }

    let lean = math_helper::degrees_to_radians(LEAN_DEGREES);
    let normal =
        ((outward.get_normal() * lean.cos()) + (sideways.get_normal() * lean.sin())).get_normal();
    BevelEndCap::new(normal, normal.dot(through))
}

/// One cube edge as a run, which is the shortest real chain there is.
fn run() -> Vec<ChainEdge> {
    let cube = platonic_solids::create_cube(CUBE_SIZE, CUBE_SIZE, CUBE_SIZE);
    let features = FeatureEdgeSet::create(&cube);

    let group = bevel_edge_group_resolver::create_from_edge(
        &features,
        top_edge_index(&features),
        BevelSelectionKind::Edge,
        FaceGroup::DEFAULT_CURVE_TOLERANCE_DEGREES,
    )
    .expect("the top edge is an edge of the set");

    let resolution = bevel_edge_group_resolver::resolve(&group, &cube);
    bevel_chain_linker::link(&resolution.edges)[0].edges.clone()
}

fn top_edge_index(features: &FeatureEdgeSet) -> i32 {
    let half = CUBE_SIZE / 2.0;
    for (i, edge) in features.edges().iter().enumerate() {
        if (edge.start.y - half).abs() < 1e-6
            && (edge.end.y - half).abs() < 1e-6
            && (edge.start.z - half).abs() < 1e-6
            && (edge.end.z - half).abs() < 1e-6
        {
            return i as i32;
        }
    }

    panic!("the cube has no top edge along x");
}

fn corners(bounds: &AxisAlignedBoundingBox) -> Vec<Vector3> {
    (0..8)
        .map(|i| {
            Vector3::new(
                if i & 1 == 0 {
                    bounds.min_xyz.x
                } else {
                    bounds.max_xyz.x
                },
                if i & 2 == 0 {
                    bounds.min_xyz.y
                } else {
                    bounds.max_xyz.y
                },
                if i & 4 == 0 {
                    bounds.min_xyz.z
                } else {
                    bounds.max_xyz.z
                },
            )
        })
        .collect()
}

/// Whether a point is inside a CONVEX mesh - every face's plane has it on the inward side. Only
/// ever asked about the half-space box, which is a cube.
fn inside(cube: &Mesh, point: Vector3) -> bool {
    for face in cube.faces.iter() {
        let a = Vector3::from_vector3_float(cube.vertices[face.v0 as usize]);
        let b = Vector3::from_vector3_float(cube.vertices[face.v1 as usize]);
        let c = Vector3::from_vector3_float(cube.vertices[face.v2 as usize]);

        let normal = (b - a).cross(c - a);
        if normal.length_squared().is_nan() || normal.length_squared() <= 0.0 {
            continue;
        }

        if normal.get_normal().dot(point - a) > ON_PLANE {
            return false;
        }
    }

    true
}
