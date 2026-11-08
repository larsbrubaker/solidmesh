//! The refinement rules' sliver guard
//! (`refinement_splits::would_thin_a_child*`, `refinement_splits_enqueue::bisect_instead_of_thinning*`):
//! a split that would leave a child far thinner than its parent bisects that child's longest-edge
//! path instead.

use std::collections::{HashMap, VecDeque};
use std::rc::Rc;

use solidmesh_math::Vector3;

use solidmesh::bevel::level_line::band_gap_entry_points::{self, BandGapBuildOptions};
use solidmesh::bevel::level_line::field_counters::{ALL_RULES, WOULD_THIN_LIMITS};
use solidmesh::bevel::level_line::point_bevel::PointBevelKind;
use solidmesh::bevel::level_line::refinement_splits::{
    would_thin_a_child, would_thin_a_child_on_edge,
};
use solidmesh::bevel::level_line::refinement_splits_enqueue::{
    bisect_instead_of_thinning_on_edge, SLIVER_ASPECT_LIMIT,
};
use solidmesh::bevel::level_line::{
    BandProfile, FieldCounters, LevelLineMesh, LevelLineRule, WorkBudget,
};

use crate::end_face_builds::build_chains;
use crate::level_line_feature_edges::feature_edges_over;
use crate::level_line_fixture::LevelLineFixture;
use crate::level_line_room_geometry::{hourglass_solid, l_prism};
use crate::point_bevel_tests::build_for_meter;
use crate::ridge_search_cost_tests::append_to_cost_log;
use crate::{grid_hole_plate_fixture, several_chains_fixtures};

const LIMIT: f64 = SLIVER_ASPECT_LIMIT;

#[test]
fn splitting_a_strip_well_inside_its_long_edge_thins_a_child() {
    // The child (P, 1, 2) keeps the 16.008 edge 1-2 with 0.3 of the area: aspect 107 against the
    // parent's 32.
    assert!(would_thin_a_child_on_edge(
        &strip(),
        0,
        1,
        Vector3::new(11.2, 0.0, 0.0),
        LIMIT,
        0.0
    ));
}

#[test]
fn splitting_a_well_shaped_triangle_at_a_midpoint_does_not() {
    assert!(!would_thin_a_child_on_edge(
        &well_shaped(),
        0,
        1,
        Vector3::new(2.0, 0.0, 0.0),
        LIMIT,
        0.0
    ));
}

#[test]
fn a_nick_within_one_over_the_limit_of_an_end_is_left_to_the_rules() {
    // Half of 1/64 of the way in from vertex 1: the child (P, 1, 2) is a real sliver, but that is
    // what any crossing near a corner makes, and it is not the regress.
    let near_the_end = Vector3::new(16.0 * (1.0 - (0.5 / LIMIT)), 0.0, 0.0);
    assert!(!would_thin_a_child_on_edge(
        &strip(),
        0,
        1,
        near_the_end,
        LIMIT,
        0.0
    ));
}

#[test]
fn a_triangle_no_longer_than_the_gate_is_not_guarded() {
    assert!(!would_thin_a_child_on_edge(
        &strip(),
        0,
        1,
        Vector3::new(11.2, 0.0, 0.0),
        LIMIT,
        17.0
    ));
}

#[test]
fn every_triangle_on_the_edge_is_asked() {
    // The strip's long edge 0-1 is shared with a well-shaped triangle below it, whose children at
    // 0.7 are fine; the strip's are not, so the split is guarded.
    let mesh = LevelLineMesh::from_triangles(
        &[
            Vector3::new(0.0, 0.0, 0.0),
            Vector3::new(16.0, 0.0, 0.0),
            Vector3::new(0.0, 0.5, 0.0),
            Vector3::new(8.0, -14.0, 0.0),
        ],
        &[(0, 1, 2), (1, 0, 3)],
        None,
    )
    .expect("the mesh builds");

    assert!(would_thin_a_child_on_edge(
        &mesh,
        0,
        1,
        Vector3::new(11.2, 0.0, 0.0),
        LIMIT,
        0.0
    ));
}

#[test]
fn an_interior_point_is_asked_of_its_three_star_children() {
    // 0.01 over the strip's long edge: star child (0, 1, P) is 16 long and 0.01 high. That is
    // farther than the triangle's own millionth from the edge, so it is an interior insertion.
    assert!(would_thin_a_child(
        &strip(),
        0,
        Vector3::new(4.0, 0.01, 0.0),
        LIMIT,
        0.0
    ));

    // The centroid of a well-shaped triangle makes three well-shaped children.
    assert!(!would_thin_a_child(
        &well_shaped(),
        0,
        Vector3::new(2.0, 3.5 / 3.0, 0.0),
        LIMIT,
        0.0
    ));

    // A point on an edge is asked as that edge's split, and at a corner nothing is minted.
    assert!(would_thin_a_child(
        &strip(),
        0,
        Vector3::new(11.2, 0.0, 0.0),
        LIMIT,
        0.0
    ));
    assert!(!would_thin_a_child(
        &strip(),
        0,
        Vector3::new(16.0, 0.0, 0.0),
        LIMIT,
        0.0
    ));
}

#[test]
fn the_guard_leaves_a_triangle_of_another_tag_alone() {
    // Edge 0-1 is a fold: the strip above it is tag 1, the well-shaped triangle below is tag 0. A
    // split at 0.7 thins only the strip, so it is guarded when the strip asks and not when the
    // tag-0 triangle does.
    let fold = || {
        LevelLineMesh::from_triangles(
            &[
                Vector3::new(0.0, 0.0, 0.0),
                Vector3::new(16.0, 0.0, 0.0),
                Vector3::new(0.0, 0.5, 0.0),
                Vector3::new(8.0, -14.0, 0.0),
            ],
            &[(0, 1, 2), (1, 0, 3)],
            Some(&[1, 0]),
        )
        .expect("the mesh builds")
    };
    let split = Vector3::new(11.2, 0.0, 0.0);

    let mut from_below = fold();
    assert!(!bisect_instead_of_thinning_on_edge(
        &mut from_below,
        1,
        0,
        1,
        split,
        0.0,
        &mut VecDeque::new(),
        &mut HashMap::new()
    )
    .expect("the guard reads"));
    assert_eq!(from_below.vertex_count(), 4);

    let mut from_above = fold();
    assert!(bisect_instead_of_thinning_on_edge(
        &mut from_above,
        0,
        0,
        1,
        split,
        0.0,
        &mut VecDeque::new(),
        &mut HashMap::new()
    )
    .expect("the guard reads"));
    assert_eq!(from_above.vertex_count(), 5);
}

/// Switches the per-rule meter back off however the test leaves.
struct MeterOff;

impl Drop for MeterOff {
    fn drop(&mut self) {
        FieldCounters::set_meter_on(false);
    }
}

#[test]
fn the_guard_answers_false_when_the_bisection_divides_nothing() {
    // The strip's long edge is the pick, so the bisection cannot start and the rule must split as
    // it asked.
    let mut mesh = strip();
    mesh.is_chain_edge = Some(Rc::new(|a, b| (a == 1 && b == 2) || (a == 2 && b == 1)));
    let mut pending = VecDeque::new();

    FieldCounters::reset();
    FieldCounters::set_meter_on(true);
    {
        let _off = MeterOff;
        assert!(!bisect_instead_of_thinning_on_edge(
            &mut mesh,
            0,
            0,
            1,
            Vector3::new(11.2, 0.0, 0.0),
            0.0,
            &mut pending,
            &mut HashMap::new()
        )
        .expect("the guard reads"));
    }

    assert_eq!(mesh.vertex_count(), 3);
    assert_eq!(pending.len(), 0);
    assert_eq!(FieldCounters::fired_count(LevelLineRule::SliverGuard), 0);
}

/// The would-fire meter over the land reproducers and the pinned fixtures, at every candidate limit.
/// The guard must fire where the land regresses (pass A's 2, 3, 9 and the face rim's 2, 3, 8) and
/// never on a build whose mesh a pin records, so `SLIVER_ASPECT_LIMIT` is the smallest candidate
/// silent on all of those.
#[test]
fn the_guard_would_fire_on_the_land_and_nowhere_on_the_pinned_fixtures() {
    let mut table = String::new();
    table.push_str(&format!("{:<22}{:<12}", "build", "rule"));
    for l in WOULD_THIN_LIMITS {
        table.push_str(&format!("{:>8}", format!("A{l}")));
    }
    table.push('\n');

    let mut at_the_limit: Vec<(String, i64, i64)> = Vec::new();
    for (name, build) in builds() {
        FieldCounters::reset();
        FieldCounters::set_meter_on(true);
        {
            let _off = MeterOff;
            build();
        }

        let mut any = false;
        for rule in ALL_RULES {
            let row: Vec<i64> = WOULD_THIN_LIMITS
                .iter()
                .map(|&l| FieldCounters::would_thin(rule, l))
                .collect();
            if row.iter().any(|&n| n > 0) {
                any = true;
                table.push_str(&format!("{:<22}{:<12}", name, format!("{rule:?}")));
                for n in row {
                    table.push_str(&format!("{n:>8}"));
                }
                table.push('\n');
            }
        }

        if !any {
            table.push_str(&format!("{name:<22}(none)\n"));
        }

        // The meter reads a rule's split BEFORE the guard takes it, so where the guard is in force
        // these rows count the splits it was asked about, and this line how many it bisected
        // instead.
        let fired = FieldCounters::fired_count(LevelLineRule::SliverGuard);
        table.push_str(&format!("{:<22}{:<12}{:>8}\n", name, "guard fired", fired));

        let count: i64 = ALL_RULES
            .iter()
            .map(|&rule| FieldCounters::would_thin(rule, LIMIT))
            .sum();
        at_the_limit.push((name, count, fired));
    }

    println!("{table}");
    append_to_cost_log(&table);

    for (name, count, fired) in at_the_limit {
        if name.starts_with("land") {
            assert!(count > 0, "{name} is the regress the guard is for\n{table}");
            assert!(fired > 0, "the guard bisects on {name}\n{table}");
        } else {
            assert_eq!(
                count, 0,
                "{name} is pinned, so the guard must stay silent there\n{table}"
            );
            assert_eq!(fired, 0, "{name} is pinned\n{table}");
        }
    }
}

/// The builds the meter reads, each the way the test that pins or reproduces it
/// builds it. Only the meter is read, so each build's own result is dropped.
fn builds() -> Vec<(String, Box<dyn Fn()>)> {
    let mut builds: Vec<(String, Box<dyn Fn()>)> = vec![
        (
            "land passA 2,3,9".to_string(),
            Box::new(|| trio("many-holes-passA-37chains-r05", &[2, 3, 9])),
        ),
        (
            "land facerim 2,3,8".to_string(),
            Box::new(|| trio("many-holes-facerim-25chains-r05", &[2, 3, 8])),
        ),
    ];

    for (fixture, size) in [("A", 0.3), ("B", 0.5), ("C", 0.3)] {
        builds.push((
            format!("SeveralChains {fixture} {size}"),
            Box::new(move || {
                let (positions, indices, chains) = several_chains_fixtures::named(fixture);
                let feature =
                    feature_edges_over(&positions, &indices, several_chains_fixtures::FOLD_DEGREES);
                let closed = vec![true; chains.len()];
                band_gap_entry_points::build_pick(
                    &positions,
                    &indices,
                    &chains,
                    &closed,
                    Some(feature),
                    size,
                    BandProfile::Chamfer,
                    &BandGapBuildOptions::default(),
                )
                .expect("a valid pick");
            }),
        ));
    }

    for (which, size) in [
        ("hourglass", 0.4),
        ("hourglass", 0.1),
        ("L", 0.2),
        ("L", 0.05),
    ] {
        builds.push((
            format!("ridge {which} {size}"),
            Box::new(move || {
                let (positions, indices, chain) = if which == "hourglass" {
                    hourglass_solid(10.0, 2.0, 4.0, 4.0)
                } else {
                    l_prism()
                };
                let budget = WorkBudget::new(WorkBudget::default_limit(indices.len() as i32))
                    .expect("a positive limit");
                band_gap_entry_points::build(
                    &positions,
                    &indices,
                    &chain,
                    true,
                    Some(feature_edges_over(&positions, &indices, 45.0)),
                    size,
                    BandProfile::Fillet,
                    &BandGapBuildOptions {
                        chord_tolerance: 0.0,
                        snap: 1e-9,
                        known_room: None,
                        budget: Some(Rc::new(budget)),
                        ..BandGapBuildOptions::default()
                    },
                )
                .expect("a valid pick");
            }),
        ));
    }

    // Point bevels refine through the same rules with no chain edges and a chord tolerance of a
    // thousandth of the setback - a cone tip and a cube corner, as PointBevelTests builds them.
    builds.push((
        "point cone fillet".to_string(),
        Box::new(|| build_for_meter(true, PointBevelKind::Fillet)),
    ));
    builds.push((
        "point cone chamfer".to_string(),
        Box::new(|| build_for_meter(true, PointBevelKind::Chamfer)),
    ));
    builds.push((
        "point cube chamfer".to_string(),
        Box::new(|| build_for_meter(false, PointBevelKind::Chamfer)),
    ));

    for holes in [2, 4, 6] {
        builds.push((
            format!("grid {holes}"),
            Box::new(move || {
                let (positions, indices, chains) = grid_hole_plate_fixture::pick_with_span(
                    holes,
                    grid_hole_plate_fixture::DEFAULT_SPAN,
                );
                let closed = vec![true; chains.len()];
                let budget = WorkBudget::new(500 * WorkBudget::default_limit(indices.len() as i32))
                    .expect("a positive limit");
                band_gap_entry_points::build_pick(
                    &positions,
                    &indices,
                    &chains,
                    &closed,
                    Some(grid_hole_plate_fixture::feature_edges(&positions, &indices)),
                    grid_hole_plate_fixture::SIZE,
                    BandProfile::Chamfer,
                    &BandGapBuildOptions {
                        chord_tolerance: 0.0,
                        snap: 1e-9,
                        known_room: None,
                        budget: Some(Rc::new(budget)),
                        ..BandGapBuildOptions::default()
                    },
                )
                .expect("a valid pick");
            }),
        ));
    }

    builds
}

/// Three chains of a captured Many Holes pick, built as the capture records.
fn trio(capture: &str, picked: &[usize]) {
    let fixture = Rc::new(LevelLineFixture::load(capture));
    let mut budget = WorkBudget::new(WorkBudget::default_limit(fixture.indices.len() as i32))
        .expect("a positive limit");
    budget.input_is_float_stored = true;
    let options = BandGapBuildOptions {
        chord_tolerance: 0.0,
        snap: 1e-9,
        known_room: None,
        budget: Some(Rc::new(budget)),
        seam_epsilon: fixture.seam_epsilon,
        sharp_turn_degrees: fixture.sharp_turn_degrees,
        fillet_segments: fixture.segments,
    };
    build_chains(&fixture, picked, fixture.size, &options);
}

fn strip() -> LevelLineMesh {
    LevelLineMesh::from_triangles(
        &[
            Vector3::new(0.0, 0.0, 0.0),
            Vector3::new(16.0, 0.0, 0.0),
            Vector3::new(0.0, 0.5, 0.0),
        ],
        &[(0, 1, 2)],
        None,
    )
    .expect("the strip builds")
}

fn well_shaped() -> LevelLineMesh {
    LevelLineMesh::from_triangles(
        &[
            Vector3::new(0.0, 0.0, 0.0),
            Vector3::new(4.0, 0.0, 0.0),
            Vector3::new(2.0, 3.5, 0.0),
        ],
        &[(0, 1, 2)],
        None,
    )
    .expect("the triangle builds")
}
