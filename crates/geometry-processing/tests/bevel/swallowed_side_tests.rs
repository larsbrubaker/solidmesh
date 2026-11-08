//! Flow rung 3's swallowed side, on Fillet Thin Front's fin (see
//! `oblique_end_face_owner_tests::fillet_thin_fronts_fin_rounds_to_the_cylinder_cut_back_by_the_walls`):
//! both fin faces are 6.01 along the slope, the fillet's setback is 1.5 r, so at 4.9 (7.35) each
//! face lies wholly in the strip, its far corner on the V crease where the two growing walls meet;
//! at 3 (4.5) it does not.

use std::rc::Rc;

use solidmesh::bevel::level_line::band_gap_entry_points::{self, BandGapBuildOptions};
use solidmesh::bevel::level_line::band_side_tagging_ends::{tag_sides, SideReach};
use solidmesh::bevel::level_line::chain_on_the_mesh;
use solidmesh::bevel::level_line::end_face_sense;
use solidmesh::bevel::level_line::field_counters::FieldCounters;
use solidmesh::bevel::level_line::swallowed_side_runs::{
    growing_walls, is_swallowed, row_span, virtual_run, virtual_run_over,
};
use solidmesh::bevel::level_line::{gap_fill, strip_clip};
use solidmesh::bevel::level_line::{
    BandGapRefusal, BandGapResult, BandProfile, ChainField, GapFillRefusal, LevelLineMesh,
};
use solidmesh_math::Vector3;

use crate::fillet_thin_front_fin::{assert_the_fin_rounds_to_the_control, profile_of};
use crate::level_line_fixture::LevelLineFixture;
use crate::sweep_mesh_measures;

const BEND_DEGREES: f64 = 20.0;

/// The union's corner in the fixture's space; the box frame is read from it.
const CORNER: Vector3 = Vector3::new(-20.0, -118.5, 0.0);

fn sign(x: f64) -> f64 {
    // The sign, 0 for 0.
    if x > 0.0 {
        1.0
    } else if x < 0.0 {
        -1.0
    } else {
        0.0
    }
}

#[test]
fn fin_faces_are_swallowed_at_the_saved_size_and_not_at3() {
    for (size, swallowed) in [(4.9, true), (3.0, false)] {
        let (mesh, chain) = uncut_at(size);

        // Both walls are read at both sizes, so a false at 3 is the side standing past the strip.
        assert!(growing_walls(&mesh, &chain).is_some());
        assert_eq!(
            is_swallowed(&mesh, &chain, 1).expect("no budget"),
            swallowed
        );
        assert_eq!(
            is_swallowed(&mesh, &chain, 2).expect("no budget"),
            swallowed
        );
    }
}

/// Each swallowed side's run is the rail off the mesh: every point at the setback from the chain,
/// square to it, in its side's plane (the fin face's, at 33.69 degrees to the fin's mid plane
/// z = 10); the run ends on the two wall planes (x = y and x + y = 20, box frame), parameters
/// ascending.
#[test]
fn swallowed_sides_run_is_the_rail_between_the_walls() {
    for side in [1, 2] {
        let size = 4.9;
        let (mesh, chain) = uncut_at(size);
        let run = virtual_run(&mesh, &chain, side).expect("a run");
        assert!(run.len() >= 2);

        // tan 33.69 = 2 / 3, so the fillet's setback is 1.5 r and the rail stands at (3, +-2) /
        // sqrt 13 of it.
        let setback = 1.5 * size;
        let rail_y = 5.0 + (setback * 3.0 / 13f64.sqrt());
        let rail_dz = setback * 2.0 / 13f64.sqrt();
        const TOLERANCE: f64 = 1e-5;
        let mut side_sign = 0.0;
        for i in 0..run.len() {
            let (point, parameter) = run[i];
            let on_chain = chain.point_at(parameter);
            let direction = (chain.points[chain.points.len() - 1] - chain.points[0]).get_normal();
            assert!(((point - on_chain).length() - chain.setback_at(parameter)).abs() <= TOLERANCE);
            assert!((point - on_chain).dot(direction).abs() <= TOLERANCE);

            let p = point - CORNER;
            assert!((p.y - rail_y).abs() <= TOLERANCE, "{p:?} (box frame)");
            assert!(
                ((p.z - 10.0).abs() - rail_dz).abs() <= TOLERANCE,
                "{p:?} (box frame)"
            );

            // One side's rail is above the fin's mid plane, the other's below, the same at every
            // point.
            side_sign = if i == 0 { sign(p.z - 10.0) } else { side_sign };
            assert_eq!(sign(p.z - 10.0), side_sign);
            if i > 0 {
                assert!(parameter > run[i - 1].1);
            }
        }

        // The ends stand one on each wall plane.
        let on_wall = |q: Vector3, wall: i32| {
            if wall == 0 {
                (q.x - q.y).abs() / 2f64.sqrt()
            } else {
                (q.x + q.y - 20.0).abs() / 2f64.sqrt()
            }
        };
        let start = run[0].0 - CORNER;
        let end = run[run.len() - 1].0 - CORNER;
        let start_wall = if on_wall(start, 0) <= TOLERANCE { 0 } else { 1 };
        println!(
            "swallowed side {side}: run {start:?} .. {end:?} (box frame), parameters {} .. {}",
            run[0].1,
            run[run.len() - 1].1
        );
        assert!(on_wall(start, start_wall) <= TOLERANCE);
        assert!(on_wall(end, 1 - start_wall) <= TOLERANCE);
    }
}

/// The rows span every row that reaches the notch's air, a quarter setback past the last to touch
/// each wall: the cylinder's front (y = 5 + r / sin 33.69 - r, box frame) touches x = y at
/// x = front and x + y = 20 at x = 20 - front; the chain runs from v9 (x = 15), so those are
/// t = 15 - x. The rows at the span's two ends stand wholly behind a wall, read on the control's
/// circle.
#[test]
fn swallowed_strips_rows_span_past_the_last_touch_of_each_wall() {
    let size = 4.9;
    let (mesh, chain) = uncut_at(size);
    let span = row_span(&mesh, &chain, BandProfile::Fillet).expect("a span");

    let axis_y = 5.0 + (size * 13f64.sqrt() / 2.0);
    let front = axis_y - size;
    let margin = 0.25 * 1.5 * size;
    println!(
        "swallowed strip rows: {} .. {}, front {front}",
        span.0, span.1
    );
    assert!((span.0 - (15.0 - (20.0 - front) - margin)).abs() <= 1e-5);
    assert!((span.1 - (15.0 - front + margin)).abs() <= 1e-5);

    // The end rows, on the control's circle between its contact lines: every point behind a wall.
    let contact = 1.5 * size * 2.0 / 13f64.sqrt();
    for t in [span.0, span.1] {
        let x = 15.0 - t;
        let half_angle = (contact / size).asin();
        for k in 0..=16 {
            let angle = -half_angle + (2.0 * half_angle * k as f64 / 16.0);
            let y = axis_y - (size * angle.cos());
            assert!(
                y > x || x + y > 20.0,
                "row at x {x}: ({x}, {y}) stands in the notch's air"
            );
        }
    }

    let run = virtual_run_over(&mesh, &chain, 1, Some(span)).expect("a run");
    assert_eq!(run[0].1, span.0);
    assert_eq!(run[run.len() - 1].1, span.1);
}

/// Under a handed room (the room reading still stops at the far corner until rung 4), the swallowed
/// fin builds to the 4.9 control: closed, one shell, every added vertex on the cylinder or a wall,
/// the volume the part's less 51.7.
#[test]
fn swallowed_fin_builds_to_the_control_under_a_handed_room() {
    assert_the_fin_rounds_to_the_control(4.9, 51.7, Some(6.0));
}

/// The swallowed fin's room is where the round stops reaching the V crease (the ties merge): the
/// axis at y = 5 + r sqrt 13 / 2 stands r past the crease's y = 10 at r = 5 / (sqrt 13 / 2 - 1),
/// less the kept strip's margin (the fixture's float moves it more than the margin does); 1.1
/// times it refuses by size and names the same room.
#[test]
fn swallowed_fins_room_is_where_the_ties_merge() {
    let built = gap_at(4.9);
    let merge = 5.0 / ((13f64.sqrt() / 2.0) - 1.0);
    println!(
        "swallowed fin: room {} set by {:?} against the merge {merge}",
        built.room, built.room_set_by
    );
    assert!(
        built.built(),
        "{:?}, room {} set by {:?}",
        built.refusal,
        built.room,
        built.room_set_by
    );
    // The fixture's corners are float, a few 1e-7 off the analytic part.
    assert!((built.room - merge).abs() <= 1e-5);

    let refused = gap_at(1.1 * built.room);
    assert_eq!(refused.refusal, BandGapRefusal::Size);
    assert_eq!(refused.room, built.room);
}

/// The room is a size that builds: at 0.9 of it and at the room itself - where the two ties on the
/// V crease all but meet - the fin fills closed, one shell.
#[test]
fn swallowed_fin_builds_up_to_its_room() {
    for fraction in [0.9, 1.0] {
        let room = gap_at(4.9).room;
        let size = fraction * room;
        let gap = gap_at(size);
        assert!(
            gap.built(),
            "{:?}, room {} at {size}",
            gap.refusal,
            gap.room
        );
        let meter_was = FieldCounters::meter_on();
        FieldCounters::set_meter_on(true);
        strip_clip::set_last_outcome(None);
        let fixture = LevelLineFixture::load("fillet-thin-front-op0");
        let filled = gap_fill::fill(&gap, profile_of(&fixture), fixture.segments);
        let clip = strip_clip::last_outcome();
        FieldCounters::set_meter_on(meter_was);
        let filled = filled.expect("the fill runs");
        assert_eq!(
            filled.refusal,
            GapFillRefusal::None,
            "{:?} at {size}: clip {clip:?}",
            filled.refusal
        );
        let mesh = filled.mesh.as_ref().expect("the filled mesh").borrow();
        let kept = sweep_mesh_measures::kept(&mesh);
        assert!(sweep_mesh_measures::is_closed_and_consistent(
            mesh.vertices(),
            &kept
        ));
        assert_eq!(
            sweep_mesh_measures::shell_count(mesh.vertex_count(), &kept),
            1
        );
    }
}

/// The pick as the room reading sees it before any cut: the part tagged into its sides and the
/// chain's field at the size, both built by the production calls BandRoom makes.
fn uncut_at(size: f64) -> (LevelLineMesh, ChainField) {
    let fixture = LevelLineFixture::load("fillet-thin-front-op0");
    let chain = &fixture.chains[0];
    let closed = fixture.closed[0];
    let chain_edges = chain_on_the_mesh::chain_edges(chain, closed);
    let end_rays = end_face_sense::oblique_ends(
        &fixture.positions,
        &fixture.indices,
        chain,
        closed,
        BEND_DEGREES,
    );
    let end_ray_bounds =
        end_face_sense::end_ray_bounds(&fixture.positions, &fixture.indices, chain, end_rays);
    let (tags, _) = tag_sides(
        &fixture.indices,
        &chain_edges,
        &|a, b| fixture.is_feature_edge(a, b),
        true,
        Some(&fixture.positions),
        end_rays,
        SideReach::default(),
    )
    .expect("the sides tag");
    let mesh = LevelLineMesh::from_triangles(&fixture.positions, &fixture.indices, Some(&tags))
        .expect("the mesh");
    let unit = chain_on_the_mesh::unit_setbacks(
        &fixture.positions,
        &fixture.indices,
        chain,
        closed,
        &chain_edges,
        profile_of(&fixture),
    )
    .expect("the setbacks");
    let points: Vec<Vector3> = chain
        .iter()
        .map(|&v| fixture.positions[v as usize])
        .collect();
    let setbacks: Vec<f64> = unit.iter().map(|u| u * size).collect();
    let field = ChainField::from_polyline_bounded(
        &points,
        closed,
        &setbacks,
        BEND_DEGREES,
        end_rays,
        end_ray_bounds,
    )
    .expect("the field");
    (mesh, field)
}

/// The pick's cut at the size: at 4.9 it is refused by size today (the room stops at the far
/// corner), and the refusal carries the tagged mesh and the chain's field at that size, which is
/// what the check reads.
fn gap_at(size: f64) -> BandGapResult {
    let fixture = Rc::new(LevelLineFixture::load("fillet-thin-front-op0"));
    let feature = {
        let fixture = Rc::clone(&fixture);
        Rc::new(move |a, b| fixture.is_feature_edge(a, b))
    };
    let options = BandGapBuildOptions {
        chord_tolerance: 0.0,
        snap: 1e-9,
        sharp_turn_degrees: BEND_DEGREES,
        ..BandGapBuildOptions::default()
    };
    band_gap_entry_points::build_pick(
        &fixture.positions,
        &fixture.indices,
        &[fixture.chains[0].clone()],
        &[fixture.closed[0]],
        Some(feature),
        size,
        profile_of(&fixture),
        &options,
    )
    .expect("a valid pick")
}
