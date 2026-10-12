//! Port of `LevelLine/ObliqueEndFaceOwnerTests.cs`: flow rung 3's owner rows - catalogue picks
//! whose open chain ends on an OBLIQUE end face, each held to its Onshape control within size / 50.
//! The fin's control (`AssertTheFinRoundsToTheControl`) lives in `fillet_thin_front_fin.rs`, which
//! `swallowed_side_tests.rs` shares.

use std::cell::RefCell;
use std::rc::Rc;

use geometry_processing::bevel::level_line::band_gap_in_turn::{
    build_open_chains_in_turn, takes_open_chains_in_turn,
};
use geometry_processing::bevel::level_line::corner_cap::{WatchEndHoleAt, WATCH_END_HOLE_AT};
use geometry_processing::bevel::level_line::end_face_sense::oblique_ends;
use geometry_processing::bevel::level_line::field_counters::FieldCounters;
use geometry_processing::bevel::level_line::{gap_fill, hull_clip, virtual_rail_past_the_end};
use geometry_processing::bevel::level_line::{BandGapRefusal, GapFillRefusal};
use vector_math::Vector3;

use crate::end_face_builds::{at_bend, build_chains, feature_of, FlatEnds};
use crate::fillet_thin_front_fin::{
    assert_the_fin_rounds_to_the_control, profile_of, BEND_DEGREES,
};
use crate::level_line_fixture::LevelLineFixture;
use crate::level_line_test_geometry::{signed_volume, signed_volume_of};
use crate::sweep_mesh_measures;

fn normal_of(positions: &[Vector3], t: (i32, i32, i32)) -> Vector3 {
    let at = |v: i32| positions[v as usize];
    (at(t.1) - at(t.0)).cross(at(t.2) - at(t.0)).get_normal()
}

fn has(t: (i32, i32, i32), v: i32) -> bool {
    t.0 == v || t.1 == v || t.2 == v
}

/// Fillet Thin Front G0: a convex 4.9 fillet on a fin's apex edge, ending at each end on a 45
/// degree wall of the V notch; held to the cylinder cut back by the walls.
#[test]
fn fillet_thin_fronts_fin_rounds_to_the_cylinder_cut_back_by_the_walls() {
    assert_the_fin_rounds_to_the_control(
        LevelLineFixture::load("fillet-thin-front-op0").size,
        51.7,
        None,
    );
}

/// The same pick at radius 3, under the room, against the "Fillet 3" control.
#[test]
fn fillet_thin_fronts_fin_rounds_to_the_radius3_control() {
    assert_the_fin_rounds_to_the_control(3.0, 29.9, None);
}

/// Center Flows Through with BOTH chains picked, as the app picks them, reads room for the saved 6
/// and cuts at it.
#[test]
fn center_flows_through_both_chains_read_their_room() {
    let fixture = Rc::new(LevelLineFixture::load("center-flows-through-op0"));
    let gap = build_chains(&fixture, &[0, 1], fixture.size, &at_bend(BEND_DEGREES));
    let at = if gap.offending_vertex >= 0 {
        let mesh = gap.mesh.as_ref().expect("the mesh").borrow();
        format!("{:?}", mesh.position(gap.offending_vertex))
    } else {
        "-".to_string()
    };
    println!(
        "center flows through both: {:?}, room {} set by {:?}, triangle {}, edge {:?}, vertex {} {at}, chain {} other {}",
        gap.refusal,
        gap.room,
        gap.room_set_by,
        gap.offending_triangle,
        gap.offending_edge,
        gap.offending_vertex,
        gap.offending_chain,
        gap.other_chain
    );
    assert_eq!(
        gap.refusal,
        BandGapRefusal::None,
        "{:?}, room {} set by {:?}, edge {:?}, vertex {at}, chains {}/{}",
        gap.refusal,
        gap.room,
        gap.room_set_by,
        gap.offending_edge,
        gap.offending_chain,
        gap.other_chain
    );
    assert!(gap.built());
}

/// Center Flows Through with BOTH chains picked fills closed, one shell, and the two beads
/// together add what each adds alone.
#[test]
fn center_flows_through_both_chains_fill_like_each_alone() {
    let fixture = Rc::new(LevelLineFixture::load("center-flows-through-op0"));
    let profile = profile_of(&fixture);
    let gap = build_chains(&fixture, &[0, 1], fixture.size, &at_bend(BEND_DEGREES));
    assert!(
        gap.built(),
        "{:?}, room {} set by {:?}",
        gap.refusal,
        gap.room,
        gap.room_set_by
    );
    assert!(takes_open_chains_in_turn(Some(&gap), &fixture.closed));
    let filled = build_open_chains_in_turn(
        &fixture.positions,
        &fixture.indices,
        &fixture.chains,
        Some(feature_of(&fixture)),
        fixture.size,
        profile,
        fixture.segments,
        Some(gap.room),
        1e-9,
        0.0,
        BEND_DEGREES,
        None,
        None,
    )
    .expect("the build runs");
    assert_eq!(
        filled.refusal,
        GapFillRefusal::None,
        "on chain {} at {} {:?}",
        filled.offending_chain,
        filled.offending_vertex,
        filled.open_edge
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

    // Each chain's bead alone, as the app would build it picked by itself.
    let source = signed_volume_of(&fixture.positions, &fixture.indices);
    let mut alone_sum = 0.0;
    for chain in 0..fixture.chains.len() {
        let gap = build_chains(&fixture, &[chain], fixture.size, &at_bend(BEND_DEGREES));
        let alone = gap_fill::fill(&gap, profile, fixture.segments).expect("the fill runs");
        let alone_mesh = alone.mesh.as_ref().expect("the filled mesh").borrow();
        alone_sum += signed_volume(&alone_mesh) - source;
    }

    let added = signed_volume(&mesh) - source;
    println!("center flows through both: added {added} against the chains alone {alone_sum}");
    assert!(
        (added - alone_sum).abs() <= 1e-6 * source.abs(),
        "added {added} against {alone_sum}"
    );
}

/// The fin's end walls face BACK over the chain and meet the fin faces at concave creases: both
/// ends carry the end ray.
#[test]
fn fillet_thin_fronts_back_facing_opposite_sense_walls_carry_the_end_ray() {
    let fixture = LevelLineFixture::load("fillet-thin-front-op0");
    let rays = oblique_ends(
        &fixture.positions,
        &fixture.indices,
        &fixture.chains[0],
        fixture.closed[0],
        BEND_DEGREES,
    );
    assert_eq!(rays, (true, true));
}

/// Hull end, part (1): at UpperLeftInner's rim end both side rails carry on virtually past where
/// they run out of the part - the left out through the recess mouth, the right up through the top.
#[test]
fn upper_left_inners_rim_rails_carry_on_past_the_mouth_and_the_top() {
    let fixture = Rc::new(LevelLineFixture::load("upperleftinner-should-bevel-op0"));
    let gap = build_chains(&fixture, &[0], fixture.size, &at_bend(BEND_DEGREES));
    let mut minted = Vec::new();
    virtual_rail_past_the_end::extended(&gap, &gap.loops, profile_of(&fixture), &mut minted, false)
        .expect("the rails carry");
    let mesh = gap.mesh.as_ref().expect("the mesh").borrow();
    let points: Vec<Vector3> = minted.iter().map(|&v| mesh.position(v)).collect();
    println!("upperleftinner rim: minted {points:?}");
    assert_eq!(points.len(), 2);
    assert!(points.iter().map(|p| p.y).fold(f64::INFINITY, f64::min) < 95.0);
    assert!(points.iter().map(|p| p.z).fold(f64::NEG_INFINITY, f64::max) > 0.0);
}

/// UpperLeftInner G0: a concave 3 fillet on the recess's upper-left crease whose rim end stands on
/// an OPPOSITE-sense end face; the pick builds closed, one shell, and adds the control's bead.
#[test]
fn upper_left_inners_bead_adds_the_controls_volume() {
    let fixture = Rc::new(LevelLineFixture::load("upperleftinner-should-bevel-op0"));
    let gap = build_chains(&fixture, &[0], fixture.size, &at_bend(BEND_DEGREES));
    assert!(
        gap.built(),
        "{:?}, room {} set by {:?}",
        gap.refusal,
        gap.room,
        gap.room_set_by
    );
    let rays = oblique_ends(
        &fixture.positions,
        &fixture.indices,
        &fixture.chains[0],
        fixture.closed[0],
        BEND_DEGREES,
    );
    let meter_was = FieldCounters::meter_on();
    FieldCounters::set_meter_on(true);
    let filled = gap_fill::fill(&gap, profile_of(&fixture), fixture.segments);
    let hull = hull_clip::last_outcome();
    FieldCounters::set_meter_on(meter_was);
    let filled = filled.expect("the fill runs");
    let mesh = filled.mesh.as_ref().expect("the filled mesh").borrow();
    assert_eq!(
        filled.refusal,
        GapFillRefusal::None,
        "at {} {:?}; hull {hull:?}; rays {rays:?}",
        filled.offending_vertex,
        filled.open_edge
    );

    let kept = sweep_mesh_measures::kept(&mesh);
    assert!(sweep_mesh_measures::is_closed_and_consistent(
        mesh.vertices(),
        &kept
    ));
    assert_eq!(
        sweep_mesh_measures::shell_count(mesh.vertex_count(), &kept),
        1
    );
    let source = signed_volume_of(&fixture.positions, &fixture.indices);
    let added = signed_volume(&mesh) - source;
    println!("upperleftinner: source {source}, added {added} against the control's 12.917");
    assert!((added - (6346.26 - 6333.343)).abs() <= 0.5, "added {added}");
}

/// Center Flows Through G0/G1: a concave 6 fillet on the V notch's vertical crease; each chain ends
/// on a fin face (the web). Every added vertex lies on the cylinder or in the web's plane, and none
/// stands past the web into the fin's material.
#[test]
fn center_flows_throughs_bead_runs_straight_on_to_the_webs_plane() {
    for chain in [0usize, 1] {
        center_flows_throughs_bead_runs_straight_on_to_the_webs_plane_case(chain);
    }
}

fn center_flows_throughs_bead_runs_straight_on_to_the_webs_plane_case(chain: usize) {
    let fixture = Rc::new(LevelLineFixture::load("center-flows-through-op0"));
    let size = fixture.size;
    let gap = build_chains(&fixture, &[chain], size, &at_bend(BEND_DEGREES));
    assert!(
        gap.built(),
        "{:?}, room {} set by {:?}",
        gap.refusal,
        gap.room,
        gap.room_set_by
    );
    let filled = gap_fill::fill(&gap, profile_of(&fixture), fixture.segments).expect("the fill");
    assert_eq!(
        filled.refusal,
        GapFillRefusal::None,
        "{} {:?}",
        filled.offending_vertex,
        filled.open_edge
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

    // Read the geometry off the fixture: the two walls on the chain edge (outward normals, into
    // the air of the notch), and the web - the face at the inner end vertex that is not vertical.
    let positions = &fixture.positions;
    let from = fixture.chains[chain][0];
    let to = *fixture.chains[chain].last().expect("a chain");
    let walls: Vec<Vector3> = fixture
        .indices
        .iter()
        .filter(|&&t| has(t, from) && has(t, to))
        .map(|&t| normal_of(positions, t))
        .collect();
    let crease = positions[from as usize];
    let up = (positions[to as usize] - crease).get_normal();
    let axis = crease + (size * (walls[0] + walls[1]) / (1.0 + walls[0].dot(walls[1])));
    let inner =
        if (positions[from as usize].z - 10.0).abs() < (positions[to as usize].z - 10.0).abs() {
            from
        } else {
            to
        };
    let web = fixture
        .indices
        .iter()
        .filter(|&&t| has(t, inner) && !(has(t, from) && has(t, to)))
        .map(|&t| normal_of(positions, t))
        .find(|n| n.dot(up).abs() > 0.1)
        .expect("the web");
    let web_point = positions[inner as usize];
    let outer = if inner == from { to } else { from };

    let mut worst = 0.0;
    let mut worst_at = Vector3::new(0.0, 0.0, 0.0);
    let mut past: f64 = 0.0;
    for &t in &filled.added_triangles {
        let face = mesh.triangles()[t as usize];
        for p in [face.a, face.b, face.c].map(|i| mesh.position(i)) {
            let from_axis = p - axis;
            let radial = (from_axis - (up * from_axis.dot(up))).length();
            let on_web = (p - web_point).dot(web);
            // The web's outward normal points into the notch's air, where the bead is: past it is
            // below zero.
            past = past.max(-on_web);

            // The outer end (the part's bottom or top) is square to the chain and grows its cap.
            let on_outer_end = (p - positions[outer as usize]).dot(up).abs();
            let off = f64::min(f64::min((radial - size).abs(), on_web.abs()), on_outer_end);
            if off > worst {
                worst = off;
                worst_at = p;
            }
        }
    }

    println!(
        "center flows through chain {chain} at {size}: off the control {worst} at {worst_at:?}, past the web {past}"
    );
    // The control's own bar, as the app test holds it (BevelControlMeshComparison.ChordTolerance).
    assert!(worst <= 0.05, "off the control {worst} at {worst_at:?}");
    assert!(past <= size / 50.0, "past the web {past}");
}

/// Bottom Center Edge with flat ends OFF: the apex end's growing cap is not star-shaped about the
/// end vertex, so the fill refuses by name.
#[test]
fn bottom_center_edge_refuses_by_name_at_its_apex_end() {
    let fixture = Rc::new(LevelLineFixture::load("bottom-center-edge-op0"));
    let _flat_ends = FlatEnds::set(false);
    let gap = build_chains(&fixture, &[0], fixture.size, &at_bend(BEND_DEGREES));
    assert!(
        gap.built(),
        "{:?}, room {} set by {:?}",
        gap.refusal,
        gap.room,
        gap.room_set_by
    );
    let refused = gap_fill::fill(&gap, profile_of(&fixture), fixture.segments).expect("the fill");
    assert_eq!(refused.refusal, GapFillRefusal::CornerCapNotStarShaped);
}

/// Bottom Center Edge with flat ends: the apex end is an end face end, so its last row lies in
/// that plane and on the round within the control's bar.
#[test]
fn bottom_center_edge_apex_row_lies_in_its_end_plane_with_flat_ends() {
    let fixture = Rc::new(LevelLineFixture::load("bottom-center-edge-op0"));
    let apex = Vector3::new(-10.0, -108.5, 6.6667);
    // The round: the arc's axis runs along the chain, r off both sides (the wall 10-9-5, the
    // ceiling 1-5-9).
    let positions = &fixture.positions;
    let wall = normal_of(positions, (10, 9, 5));
    let ceiling = normal_of(positions, (1, 5, 9));
    let along = (positions[9] - positions[5]).get_normal();
    let axis = positions[5] + (fixture.size * (wall + ceiling) / (1.0 + wall.dot(ceiling)));
    let r = fixture.size;
    let off_the_round = move |p: Vector3| {
        let d = p - axis;
        ((d - (along * d.dot(along))).length() - r).abs()
    };

    let normal = Vector3::new(1.0, -1.0, 0.0).get_normal();
    let apex_rows = Rc::new(RefCell::new(Vec::<f64>::new()));
    let off_round = Rc::new(RefCell::new(Vec::<f64>::new()));
    let _flat_ends = FlatEnds::set(true);
    {
        let apex_rows = Rc::clone(&apex_rows);
        let off_round = Rc::clone(&off_round);
        let watch: WatchEndHoleAt = Rc::new(move |mesh, row, _run, v| {
            if (v - apex).length() < 1e-3 {
                apex_rows.borrow_mut().push(
                    row.iter()
                        .map(|&i| (mesh.position(i) - v).dot(normal).abs())
                        .fold(f64::NEG_INFINITY, f64::max),
                );
                off_round.borrow_mut().push(
                    row.iter()
                        .map(|&i| off_the_round(mesh.position(i)))
                        .fold(f64::NEG_INFINITY, f64::max),
                );
            }
        });
        WATCH_END_HOLE_AT.with(|w| *w.borrow_mut() = Some(watch));
    }
    let gap = build_chains(&fixture, &[0], fixture.size, &at_bend(BEND_DEGREES));
    assert!(
        gap.built(),
        "{:?}, room {} set by {:?}",
        gap.refusal,
        gap.room,
        gap.room_set_by
    );
    let filled = gap_fill::fill(&gap, profile_of(&fixture), fixture.segments).expect("the fill");
    assert_eq!(filled.refusal, GapFillRefusal::None);
    drop(_flat_ends);

    let apex_rows = apex_rows.borrow();
    let off_round = off_round.borrow();
    println!(
        "bottom center edge apex row: off the end plane {apex_rows:?}, off the round {off_round:?}"
    );
    assert_eq!(apex_rows.len(), 1);
    assert!(apex_rows[0] <= 1e-6 * fixture.size, "{}", apex_rows[0]);
    // On the round within the control's own bar (BevelControlMeshComparison.ChordTolerance).
    assert!(off_round[0] <= 0.05, "{}", off_round[0]);
}
