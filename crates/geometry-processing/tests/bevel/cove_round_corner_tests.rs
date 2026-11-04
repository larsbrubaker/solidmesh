//! A picked closed rim whose crease flips from a cove (concave) to a round (convex) at a corner:
//! "Unexpected Refusals.mcx"'s half cylinder cut floor, a D whose straight edge along y = -25.21
//! meets the cut's end wall rising from the floor (concave) and whose arc meets the cylinder's
//! outer wall falling away (convex), the two turning into each other at (58.69, -25.21, 6.65);
//! and at (44.45, -36.34, 6.65) the cut's curved wall (concave) turns into the outer wall again.
//! Built as one chain, the rim's fill refuses it because a rail runs backward next to such a
//! corner; that refusal, and only that one, is the trigger for the fallback that cuts the rim into
//! runs at its flips (a build naming them in `WorkBudget::sense_flips_to_cut`, found by
//! `chain_on_the_mesh::sense_flips_near`), as it is cut at its setback steps.

use std::rc::Rc;

use geometry_processing::bevel::level_line::band_gap_entry_points::BandGapBuildOptions;
use geometry_processing::bevel::level_line::chain_on_the_mesh::sense_flips_near;
use geometry_processing::bevel::level_line::{gap_fill, GapFillRefusal};
use vector_math::Vector3;

use crate::end_face_builds::{as_saved, build_chains, default_budget};
use crate::fillet_thin_front_fin::profile_of;
use crate::level_line_fixture::LevelLineFixture;

/// The rim's two cove-to-round corners' vertices in the fixture.
const FLIP_VERTICES: [i32; 2] = [115, 10];

/// The rim's two cove-to-round corners.
const FLIPS: [Vector3; 2] = [
    Vector3 {
        x: 58.68540573120117,
        y: -25.210336685180664,
        z: 6.6508283615112305,
    },
    Vector3 {
        x: 44.451480865478516,
        y: -36.34162902832031,
        z: 6.6508283615112305,
    },
];

#[test]
fn the_floor_rim_is_cut_into_runs_at_its_cove_to_round_corners() {
    let fixture = Rc::new(LevelLineFixture::load(
        "unexpected-refusals-half-cut-floor-rim-r1",
    ));
    let budget = default_budget(&fixture, false);
    *budget.sense_flips_to_cut.borrow_mut() = FLIP_VERTICES.to_vec();
    let options = BandGapBuildOptions {
        budget: Some(budget),
        ..as_saved(&fixture, fixture.known_room)
    };
    let gap = build_chains(&fixture, &[0], 1.0, &options);
    let ends: Vec<Vector3> = gap
        .chains
        .iter()
        .filter(|chain| !chain.is_closed)
        .flat_map(|chain| {
            let points = chain.points();
            [points[0], points[points.len() - 1]]
        })
        .collect();
    for flip in FLIPS {
        assert!(
            ends.contains(&flip),
            "no run ends at {flip:?}: {} chains, ends {ends:?}",
            gap.chains.len()
        );
    }
}

/// Built as one chain, the rim's fill refuses it because a rail runs backward, and the refusal
/// stands within a setback of one of its cove-to-round corners at every size: the fallback's
/// trigger. A point of the rim far from both corners is no trigger.
#[test]
fn the_one_chain_refusal_stands_next_to_a_cove_to_round_corner() {
    for size in [0.5, 1.0, 2.0] {
        let fixture = Rc::new(LevelLineFixture::load(
            "unexpected-refusals-half-cut-floor-rim-r1",
        ));
        let options = BandGapBuildOptions {
            budget: Some(default_budget(&fixture, false)),
            ..as_saved(&fixture, fixture.known_room)
        };
        let gap = build_chains(&fixture, &[0], size, &options);
        assert!(gap.built(), "band {:?} at {size}", gap.refusal);
        let filled =
            gap_fill::fill(&gap, profile_of(&fixture), fixture.segments).expect("the fill runs");
        assert_eq!(
            filled.refusal,
            GapFillRefusal::RailIsNotMonotone,
            "at {size}"
        );
        let at = gap
            .mesh
            .as_ref()
            .expect("a mesh")
            .borrow()
            .position(filled.offending_vertex);
        let reach = gap
            .chains
            .iter()
            .map(|chain| chain.max_setback)
            .fold(0.0, f64::max);
        let feature = Rc::clone(&fixture);
        let is_feature_edge = move |a: i32, b: i32| feature.is_feature_edge(a, b);
        let near = |point: Vector3| {
            sense_flips_near(
                &fixture.positions,
                &fixture.indices,
                &fixture.chains,
                &fixture.closed,
                Some(&is_feature_edge),
                point,
                reach,
            )
            .expect("the rim stands on two triangles per edge")
        };
        // The refusal stands next to the round's corner with the end wall, and only that flip is
        // offered for cutting.
        assert_eq!(
            near(at),
            vec![115],
            "the refusal at {at:?} at {size} (reach {reach})"
        );

        // The middle of the arc, far from both corners.
        assert!(near(Vector3::new(56.03, -34.49, 6.65)).is_empty());
    }
}

/// Cut at its flips, each run stopping flush on the other run's side face at the corner, the rim
/// builds one loop per side and fills closed in one shell, uncrossed, at 0.3 to 1 - the corner
/// patch at the cove-to-cove setback step (44.45, -25.21, 6.65) held on the band side of the walls
/// it meets (`corner_cap_hold::held_within_the_faces`).
#[test]
fn the_floor_rim_cut_at_its_flips_builds_closed() {
    use crate::round_mouth_fillet_crossings::crossing_pairs;
    use crate::sweep_mesh_measures;
    for size in [0.3, 0.5, 1.0] {
        let fixture = Rc::new(LevelLineFixture::load(
            "unexpected-refusals-half-cut-floor-rim-r1",
        ));
        let budget = default_budget(&fixture, false);
        *budget.sense_flips_to_cut.borrow_mut() = FLIP_VERTICES.to_vec();
        let options = BandGapBuildOptions {
            budget: Some(budget),
            ..as_saved(&fixture, fixture.known_room)
        };
        let gap = build_chains(&fixture, &[0], size, &options);
        assert!(gap.built(), "band {:?} at {size}", gap.refusal);
        let filled =
            gap_fill::fill(&gap, profile_of(&fixture), fixture.segments).expect("the fill runs");
        assert_eq!(filled.refusal, GapFillRefusal::None, "at {size}");
        let mesh = filled.mesh.as_ref().expect("a filled mesh").borrow();
        let kept = sweep_mesh_measures::kept(&mesh);
        assert!(
            sweep_mesh_measures::is_closed_and_consistent(mesh.vertices(), &kept),
            "open at {size}"
        );
        assert_eq!(
            sweep_mesh_measures::shell_count(mesh.vertex_count(), &kept),
            1
        );
        assert_eq!(crossing_pairs(&mesh), 0, "crossing at {size}");
    }
}

/// At 2 the corner patch cannot be held within the walls it meets: even at the closest point on
/// or behind every wall, one triangle turns over and one is flattened, so the fill is refused
/// (`FillLeavesThePart`) rather than answering a closed but folded patch.
#[test]
fn the_floor_rim_cut_at_its_flips_refuses_a_folded_corner_at_two() {
    let fixture = Rc::new(LevelLineFixture::load(
        "unexpected-refusals-half-cut-floor-rim-r1",
    ));
    let budget = default_budget(&fixture, false);
    *budget.sense_flips_to_cut.borrow_mut() = FLIP_VERTICES.to_vec();
    let options = BandGapBuildOptions {
        budget: Some(budget),
        ..as_saved(&fixture, fixture.known_room)
    };
    let gap = build_chains(&fixture, &[0], 2.0, &options);
    assert!(gap.built(), "band {:?}", gap.refusal);
    let filled =
        gap_fill::fill(&gap, profile_of(&fixture), fixture.segments).expect("the fill runs");
    assert_eq!(filled.refusal, GapFillRefusal::FillLeavesThePart);
}

/// Which rule set each flush end's plane on the floor rim: at both corners each run stops on the
/// other run's side face (the cut's end wall or curved wall for the round, the outer facet for the
/// cove), which parts the runs and cuts into no face past the corner.
#[test]
fn the_floor_rims_flush_ends_stop_on_the_other_runs_side_face() {
    use geometry_processing::bevel::level_line::flip_corner_clip::{
        flip_corner_plane_paths, SeamPlanePath,
    };
    use geometry_processing::bevel::level_line::{chain_on_the_mesh, BandProfile, LevelLineWork};

    let fixture = Rc::new(LevelLineFixture::load(
        "unexpected-refusals-half-cut-floor-rim-r1",
    ));
    let budget = default_budget(&fixture, false);
    *budget.sense_flips_to_cut.borrow_mut() = FLIP_VERTICES.to_vec();
    let _work = LevelLineWork::begin(Some(budget));
    let feature = Rc::clone(&fixture);
    let is_feature_edge = move |a: i32, b: i32| feature.is_feature_edge(a, b);
    let (chains, closed) = chain_on_the_mesh::split_at_setback_steps(
        &fixture.positions,
        &fixture.indices,
        &fixture.chains,
        &fixture.closed,
        BandProfile::Fillet,
        fixture.sharp_turn_degrees,
        Some(&is_feature_edge),
    )
    .expect("the rim stands on two triangles per edge");
    let paths = flip_corner_plane_paths(
        &fixture.positions,
        &fixture.indices,
        &chains,
        &closed,
        &is_feature_edge,
    );
    let read: Vec<SeamPlanePath> = paths
        .iter()
        .flat_map(|&(first, last)| [first, last])
        .flatten()
        .collect();
    assert_eq!(read, vec![SeamPlanePath::OtherSideFace; 4], "{paths:?}");
}

/// The other run's side face at a corner V = 0, the runs leaving along +x (this one, onto its floor
/// facet) and +y (the other one, along its wall x = 0). The wall's facets about V lean `lean_up`
/// and `lean_down` off the plane x = 0 on either side of the facet the other run's edge stands on.
fn faceted_wall_at_a_corner(lean_up: f64, lean_down: f64) -> (Vec<Vector3>, Vec<(i32, i32, i32)>) {
    let positions = vec![
        Vector3::new(0.0, 0.0, 0.0),        // 0: the corner V
        Vector3::new(1.0, 0.0, 0.0),        // 1: this run's next vertex
        Vector3::new(0.0, 1.0, 0.0),        // 2: the other run's next vertex
        Vector3::new(1.0, 1.0, 0.0),        // 3: the floor
        Vector3::new(0.0, 1.0, 1.0),        // 4: the wall above the other run's edge
        Vector3::new(lean_up, 0.0, 1.5),    // 5: the next wall facet up
        Vector3::new(lean_down, 0.0, -1.5), // 6: the next wall facet down
        Vector3::new(0.0, 1.0, -1.0),       // 7: the wall below the other run's edge
    ];
    let indices = vec![
        (0, 1, 3), // this run's floor facet
        (0, 3, 2), // the floor on the other run's edge
        (0, 2, 4), // the other run's side face, x = 0
        (0, 4, 5), // the next wall facet up
        (0, 7, 2), // the wall below
        (0, 6, 7), // the next wall facet down
    ];
    (positions, indices)
}

/// A wall whose facets about the corner both lean the same way off its facet's plane is one
/// faceted curve on one side of it: the run stops on that facet's plane. Where they lean opposite
/// ways the plane would cut into the wall past the corner, and it is refused, so the clip falls back
/// to the crease's plane.
#[test]
fn a_side_face_whose_facets_straddle_its_plane_is_refused() {
    use geometry_processing::bevel::level_line::flip_corner_clip::against_the_other_side_face;

    let (positions, indices) = faceted_wall_at_a_corner(-0.1, -0.1);
    let one_side = against_the_other_side_face(&positions, &indices, 0, 1, 2);
    assert!(one_side.is_some(), "a wall curving one way takes the plane");
    let (normal, _) = one_side.expect("checked");
    assert!(
        normal.x.abs() > 1.0 - 1e-12,
        "the plane is the wall's facet x = 0: {normal:?}"
    );

    let (positions, indices) = faceted_wall_at_a_corner(0.1, -0.1);
    assert!(
        against_the_other_side_face(&positions, &indices, 0, 1, 2).is_none(),
        "a wall whose facets straddle the plane is cut into by it"
    );
}

/// Whether a set of flips cuts the floor rim into more pieces than the last set did
/// (`chain_on_the_mesh::flips_cut_more`): its two flips change the build from none; the flip at
/// vertex 10 is cut already at the rim's setback step there, but naming it still changes the build
/// (its end stops flush as a flip's, not as a step's); a rim vertex that is no flip, added to the
/// two, changes nothing, so the offer would not build again for it.
#[test]
fn a_flip_that_cuts_nothing_new_asks_for_no_build() {
    use geometry_processing::bevel::level_line::{chain_on_the_mesh, BandProfile};
    let fixture = Rc::new(LevelLineFixture::load(
        "unexpected-refusals-half-cut-floor-rim-r1",
    ));
    let features = Rc::clone(&fixture);
    let is_feature_edge = move |a: i32, b: i32| features.is_feature_edge(a, b);
    let rim = &fixture.chains[0];
    let not_a_flip = *rim
        .iter()
        .find(|v| !FLIP_VERTICES.contains(v))
        .expect("a rim vertex that is no flip");
    let cuts_more = |last: &[i32], flips: &[i32]| {
        chain_on_the_mesh::flips_cut_more(
            &fixture.positions,
            &fixture.indices,
            &fixture.chains,
            &fixture.closed,
            BandProfile::Fillet,
            fixture.sharp_turn_degrees,
            Some(&is_feature_edge),
            last,
            flips,
        )
    };
    assert!(cuts_more(&[], &FLIP_VERTICES));
    // Vertex 10 is cut already at its setback step, but named it ends flush as a flip: a change.
    assert!(cuts_more(&[], &FLIP_VERTICES[1..]));
    assert!(cuts_more(&FLIP_VERTICES[..1], &FLIP_VERTICES));
    let with_a_non_flip = [FLIP_VERTICES[0], FLIP_VERTICES[1], not_a_flip];
    assert!(!cuts_more(&FLIP_VERTICES, &with_a_non_flip));
}
