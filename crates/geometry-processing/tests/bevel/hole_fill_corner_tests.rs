//! The corner picks with `CornerCap.UseHoleFill` on, so every corner hole closes by the one hole
//! filler, each held to the requirement set - closed, consistently oriented, one shell, no
//! zero-area triangle, wound as the input, the volume moved the right way. Also carries the shared
//! `fill_with_the_filler`, which the other hole-fill test files call. Parameter cases run as one
//! loop per test.

#![allow(dead_code)]

use std::rc::Rc;
use std::time::Instant;

use geometry_processing::bevel::level_line::band_gap_entry_points::{self, BandGapBuildOptions};
use geometry_processing::bevel::level_line::corner_cap_hole_fill::last_hole_fill_failure;
use geometry_processing::bevel::level_line::corner_cap_hole_fill_switch::set_use_hole_fill_override;
use geometry_processing::bevel::level_line::seam_edges::FeatureEdgeTest;
use geometry_processing::bevel::level_line::work_budget::WatchHoleFills;
use geometry_processing::bevel::level_line::{
    band_gap_reading, corner_fill, gap_fill, BandGapResult, BandProfile, GapFillRefusal,
    GapFillResult, WorkBudget,
};
use vector_math::Vector3;

use crate::level_line_feature_edges::feature_edges_over;
use crate::level_line_fixture::LevelLineFixture;
use crate::level_line_test_geometry::{cube_solid, signed_volume};
use crate::several_chains_fixtures::FOLD_DEGREES;
use crate::sweep_mesh_measures;

/// The top-front edge 4-5, the rest of the top rim and the rest of the front rim.
pub fn two_faces() -> Vec<Vec<i32>> {
    vec![vec![4, 5], vec![5, 6, 7, 4], vec![5, 1, 0, 4]]
}

const ALL_OPEN: [bool; 3] = [false, false, false];

const CUBE_VOLUME: f64 = 8.0;

/// One band build of the chains at the size, every optional setting at its default.
pub fn build(
    positions: &[Vector3],
    indices: &[(i32, i32, i32)],
    chains: &[Vec<i32>],
    closed: &[bool],
    feature: &FeatureEdgeTest,
    size: f64,
    profile: BandProfile,
) -> BandGapResult {
    band_gap_entry_points::build_pick(
        positions,
        indices,
        chains,
        closed,
        Some(Rc::clone(feature)),
        size,
        profile,
        &BandGapBuildOptions::default(),
    )
    .expect("the pick builds or refuses by name")
}

/// The room of the chains: the largest size their band builds at.
pub fn room_of(
    positions: &[Vector3],
    indices: &[(i32, i32, i32)],
    chains: &[Vec<i32>],
    closed: &[bool],
    feature: &FeatureEdgeTest,
    profile: BandProfile,
) -> f64 {
    band_gap_reading::room_of_pick(
        positions,
        indices,
        chains,
        closed,
        Some(Rc::clone(feature)),
        profile,
        1e-9,
    )
    .expect("the room reads")
}

/// The fixture's feature edges as the kernel's test.
pub fn fixture_feature(fixture: &Rc<LevelLineFixture>) -> FeatureEdgeTest {
    let fixture = Rc::clone(fixture);
    Rc::new(move |a, b| fixture.is_feature_edge(a, b))
}

/// The fixture's profile.
pub fn fixture_profile(fixture: &LevelLineFixture) -> BandProfile {
    if fixture.chamfer {
        BandProfile::Chamfer
    } else {
        BandProfile::Fillet
    }
}

/// Fills with the filler on for this call only, on this thread.
pub fn fill_with_the_filler(
    gap: &BandGapResult,
    profile: BandProfile,
    segments: i32,
) -> GapFillResult {
    fill_with_the_filler_watched(gap, profile, segments, None)
}

/// [`fill_with_the_filler`] with the per-flow hole-fill watcher set: the watcher rides
/// on the fill's budget (the default one for the mesh's size, as `GapFill.Fill` takes).
pub fn fill_with_the_filler_watched(
    gap: &BandGapResult,
    profile: BandProfile,
    segments: i32,
    watch: Option<WatchHoleFills>,
) -> GapFillResult {
    let triangle_count = gap
        .mesh
        .as_ref()
        .map_or(0, |mesh| mesh.borrow().triangle_count());
    let budget = Rc::new(
        WorkBudget::new(WorkBudget::default_limit(triangle_count)).expect("a positive limit"),
    );
    *budget.watch_hole_fills.borrow_mut() = watch;
    set_use_hole_fill_override(Some(true));
    let filled = gap_fill::fill_with_budget(gap, profile, segments, Some(budget));
    set_use_hole_fill_override(None);
    filled.expect("the fill fills or refuses by name")
}

/// Two adjacent cube faces close by the filler.
#[test]
fn two_adjacent_cube_faces_close_by_the_filler() {
    for (profile, fraction_of_room) in [
        (BandProfile::Fillet, 0.1),
        (BandProfile::Fillet, 0.5),
        (BandProfile::Fillet, 0.9),
        (BandProfile::Fillet, 0.99),
        (BandProfile::Chamfer, 0.1),
        (BandProfile::Chamfer, 0.5),
        (BandProfile::Chamfer, 0.9),
        (BandProfile::Chamfer, 0.99),
    ] {
        let (positions, indices) = cube_solid(1.0);
        let feature = feature_edges_over(&positions, &indices, FOLD_DEGREES);
        let room = room_of(
            &positions,
            &indices,
            &two_faces(),
            &ALL_OPEN,
            &feature,
            profile,
        );
        let size = fraction_of_room * room;
        let gap = build(
            &positions,
            &indices,
            &two_faces(),
            &ALL_OPEN,
            &feature,
            size,
            profile,
        );
        println!(
            "two faces {profile:?} at {size} (room {room}): band {:?}",
            gap.refusal
        );
        assert!(gap.built());

        let filled = fill_with_the_filler(&gap, profile, 4);
        meets_the_requirement(
            &filled,
            &format!("two faces {profile:?} at {size}"),
            CUBE_VOLUME,
            true,
        );
    }
}

/// Every edge of the cube chamfers closed by the filler.
#[test]
fn every_edge_of_the_cube_chamfers_closed_by_the_filler() {
    for fraction_of_room in [0.1, 0.5, 0.9, 0.99] {
        let (positions, indices) = cube_solid(1.0);
        let chains = every_edge();
        let open = vec![false; chains.len()];
        let feature = feature_edges_over(&positions, &indices, FOLD_DEGREES);
        let size = fraction_of_room
            * room_of(
                &positions,
                &indices,
                &chains,
                &open,
                &feature,
                BandProfile::Chamfer,
            );
        let gap = build(
            &positions,
            &indices,
            &chains,
            &open,
            &feature,
            size,
            BandProfile::Chamfer,
        );
        println!("every edge at {size}: band {:?}", gap.refusal);
        assert!(gap.built());

        let filled = fill_with_the_filler(&gap, BandProfile::Chamfer, 4);
        meets_the_requirement(&filled, &format!("every edge at {size}"), CUBE_VOLUME, true);
    }
}

/// The cube's twelve edges, each its own chain.
pub fn every_edge() -> Vec<Vec<i32>> {
    vec![
        vec![0, 1],
        vec![1, 2],
        vec![2, 3],
        vec![3, 0],
        vec![4, 5],
        vec![5, 6],
        vec![6, 7],
        vec![7, 4],
        vec![0, 4],
        vec![1, 5],
        vec![2, 6],
        vec![3, 7],
    ]
}

/// A chamfer's corner is FLAT - its strips are planes,
/// so a corner they meet at is mitred flat by the fan to the corner's axis, and the filler is never
/// handed a chamfer's hole, even with the filler on.
#[test]
fn a_chamfers_corner_holes_are_never_faired() {
    for fraction_of_room in [0.1, 0.5, 0.9] {
        let fixture = Rc::new(LevelLineFixture::load("outer-faces-r04"));
        let feature = fixture_feature(&fixture);
        let room = room_of(
            &fixture.positions,
            &fixture.indices,
            &fixture.chains,
            &fixture.closed,
            &feature,
            BandProfile::Chamfer,
        );
        let gap = build(
            &fixture.positions,
            &fixture.indices,
            &fixture.chains,
            &fixture.closed,
            &feature,
            fraction_of_room * room,
            BandProfile::Chamfer,
        );
        assert!(gap.built());

        let faired = Rc::new(std::cell::Cell::new(0));
        let count = Rc::clone(&faired);
        let filled = fill_with_the_filler_watched(
            &gap,
            BandProfile::Chamfer,
            fixture.segments,
            Some(Rc::new(move |_, _, _, _, _| count.set(count.get() + 1))),
        );

        assert_eq!(filled.refusal, GapFillRefusal::None);
        assert_eq!(faired.get(), 0);
    }
}

/// Outer faces close by the filler.
#[test]
fn outer_faces_close_by_the_filler() {
    for (profile, fraction_of_room) in [
        (BandProfile::Fillet, 0.25),
        (BandProfile::Fillet, 0.5),
        (BandProfile::Fillet, 0.9),
        (BandProfile::Fillet, 0.99),
        (BandProfile::Chamfer, 0.1),
        (BandProfile::Chamfer, 0.5),
        (BandProfile::Chamfer, 0.9),
    ] {
        let fixture = Rc::new(LevelLineFixture::load("outer-faces-r04"));
        let feature = fixture_feature(&fixture);
        let positions = &fixture.positions;
        let indices = &fixture.indices;
        let before = sweep_mesh_measures::signed_volume(positions, indices);
        let room = room_of(
            positions,
            indices,
            &fixture.chains,
            &fixture.closed,
            &feature,
            profile,
        );
        let size = fraction_of_room * room;
        let gap = build(
            positions,
            indices,
            &fixture.chains,
            &fixture.closed,
            &feature,
            size,
            profile,
        );
        println!(
            "outer faces {profile:?} at {size} (room {room}): band {:?}",
            gap.refusal
        );
        assert!(gap.built());

        let filled = fill_with_the_filler(&gap, profile, fixture.segments);
        meets_the_requirement(
            &filled,
            &format!("outer faces {profile:?} at {size}"),
            before,
            true,
        );
    }
}

/// At a cube corner of the two-face pick every edge is
/// rounded at one radius, so the true blend is the rolling ball's octant; the filler is fair, not
/// exact, and is held within 5% of that radius. Also prints the cost per hole in this build.
#[test]
fn the_cube_corner_stays_near_the_rolling_ball() {
    for size in [0.2, 0.5] {
        let (positions, indices) = cube_solid(1.0);
        let feature = feature_edges_over(&positions, &indices, FOLD_DEGREES);
        let gap_of = || {
            build(
                &positions,
                &indices,
                &two_faces(),
                &ALL_OPEN,
                &feature,
                size,
                BandProfile::Fillet,
            )
        };

        let gap = gap_of();
        let watch = Instant::now();
        let filled = fill_with_the_filler(&gap, BandProfile::Fillet, 4);
        let with_filler = watch.elapsed().as_secs_f64() * 1000.0;
        let old_gap = gap_of();
        let watch = Instant::now();
        gap_fill::fill(&old_gap, BandProfile::Fillet, 4).expect("the fill");
        let with_rings = watch.elapsed().as_secs_f64() * 1000.0;
        println!(
            "size {size}: fill {with_filler:.3} ms with the filler, {with_rings:.3} ms with rings; {:.3} ms more per hole (2 holes)",
            (with_filler - with_rings) / 2.0
        );
        assert_eq!(filled.refusal, GapFillRefusal::None);

        let mesh = filled.mesh.as_ref().expect("a filled mesh").borrow();
        let mut deviation: f64 = 0.0;
        let mut cap_vertices = 0;
        for corner in corner_fill::corner_points(&gap.chains) {
            let outward = Vector3::new(sign(corner.x), sign(corner.y), sign(corner.z));
            let inset = corner - (outward * size);
            for &v in &filled.added_vertices {
                let off = mesh.position(v) - inset;
                if off.x * outward.x < -1e-9
                    || off.y * outward.y < -1e-9
                    || off.z * outward.z < -1e-9
                {
                    continue;
                }

                cap_vertices += 1;
                deviation = deviation.max((off.length() - size).abs());
            }
        }

        println!(
            "size {size}: {cap_vertices} corner vertices, max radial deviation {deviation} = {:.3}% of the radius",
            100.0 * deviation / size
        );
        assert!(cap_vertices > 0);
        assert!(deviation / size <= 0.05);
    }
}

/// The sign of `x` as a double: -1, 0 or 1, zero for either zero.
pub fn sign(x: f64) -> f64 {
    if x > 0.0 {
        1.0
    } else if x < 0.0 {
        -1.0
    } else {
        0.0
    }
}

/// Holds a fill to the requirement: filled, one closed and consistent shell, no zero-area
/// triangle, and wound as the input (a convex pick takes material).
fn meets_the_requirement(
    filled: &GapFillResult,
    label: &str,
    source_volume: f64,
    convex_takes_material: bool,
) {
    println!(
        "{label}: fill {:?} ({:?}), chain {}, vertex {}, edge {:?}",
        filled.refusal,
        last_hole_fill_failure(),
        filled.offending_chain,
        filled.offending_vertex,
        filled.open_edge
    );
    assert_eq!(filled.refusal, GapFillRefusal::None, "{label}");

    let mesh = filled.mesh.as_ref().expect("a filled mesh").borrow();
    let kept = sweep_mesh_measures::kept(&mesh);
    assert!(sweep_mesh_measures::is_closed_and_consistent(
        mesh.vertices(),
        &kept
    ));
    assert_eq!(
        sweep_mesh_measures::shell_count(mesh.vertex_count(), &kept),
        1
    );

    let smallest_area = smallest_kept_area(&mesh, &kept);
    let volume = signed_volume(&mesh);
    println!(
        "{label}: volume {volume} of {source_volume}, smallest area {smallest_area}, {} fill triangles",
        filled.added_triangles.len()
    );
    assert!(smallest_area > 1e-12, "{label}");

    // Wound as the input: a closed, consistent shell whose signed volume keeps the source's sign; a
    // pick that is convex overall takes material.
    assert!(volume > 0.0, "{label}");
    if convex_takes_material {
        assert!(volume < source_volume, "{label}");
    }
}

/// The smallest area of the kept triangles.
pub fn smallest_kept_area(
    mesh: &geometry_processing::bevel::level_line::LevelLineMesh,
    kept: &[(i32, i32, i32)],
) -> f64 {
    kept.iter()
        .map(|&(a, b, c)| {
            (mesh.position(b) - mesh.position(a))
                .cross(mesh.position(c) - mesh.position(a))
                .length()
                / 2.0
        })
        .fold(f64::INFINITY, f64::min)
}
