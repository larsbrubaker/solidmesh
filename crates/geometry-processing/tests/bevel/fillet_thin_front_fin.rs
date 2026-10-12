//! Port of `ObliqueEndFaceOwnerTests.AssertTheFinRoundsToTheControl` (`LevelLine/ObliqueEndFaceOwnerTests.cs`):
//! Fillet Thin Front's fin built at a size (under a handed room, when given) and held to its
//! control - the cylinder cut back by the walls. Shared by `swallowed_side_tests.rs`.

#![allow(dead_code)]

use std::rc::Rc;

use geometry_processing::bevel::level_line::band_gap_entry_points::{self, BandGapBuildOptions};
use geometry_processing::bevel::level_line::field_counters::FieldCounters;
use geometry_processing::bevel::level_line::open_gap_fill::ends_site;
use geometry_processing::bevel::level_line::{gap_fill, strip_clip};
use geometry_processing::bevel::level_line::{BandProfile, GapFillRefusal};
use vector_math::Vector3;

use crate::level_line_fixture::LevelLineFixture;
use crate::level_line_test_geometry::signed_volume;
use crate::sweep_mesh_measures;

/// C# `ObliqueEndFaceOwnerTests.BendDegrees`.
pub const BEND_DEGREES: f64 = 20.0;

/// The fixture's profile (C# `LevelLineFixture.Profile`).
pub fn profile_of(fixture: &LevelLineFixture) -> BandProfile {
    if fixture.chamfer {
        BandProfile::Chamfer
    } else {
        BandProfile::Fillet
    }
}

/// C# `AssertTheFinRoundsToTheControl`: builds the fin at the size (under a handed room, when
/// given) and holds it to the control.
pub fn assert_the_fin_rounds_to_the_control(size: f64, removed: f64, known_room: Option<f64>) {
    let fixture = Rc::new(LevelLineFixture::load("fillet-thin-front-op0"));
    let feature = {
        let fixture = Rc::clone(&fixture);
        Rc::new(move |a, b| fixture.is_feature_edge(a, b))
    };
    let options = BandGapBuildOptions {
        chord_tolerance: 0.0,
        snap: 1e-9,
        known_room,
        sharp_turn_degrees: BEND_DEGREES,
        ..BandGapBuildOptions::default()
    };
    let gap = band_gap_entry_points::build_pick(
        &fixture.positions,
        &fixture.indices,
        &[fixture.chains[0].clone()],
        &[fixture.closed[0]],
        Some(feature),
        size,
        profile_of(&fixture),
        &options,
    )
    .expect("a valid pick");
    assert!(
        gap.built(),
        "{:?}, room {} set by {:?}, triangle {}, edge {:?}, vertex {}, pinch {}",
        gap.refusal,
        gap.room,
        gap.room_set_by,
        gap.offending_triangle,
        gap.offending_edge,
        gap.offending_vertex,
        gap.pinches_at_every_size_tried
    );
    let meter_was = FieldCounters::meter_on();
    FieldCounters::set_meter_on(true);
    strip_clip::set_last_outcome(None);
    let filled = gap_fill::fill(&gap, profile_of(&fixture), fixture.segments);
    let clip = strip_clip::last_outcome();
    FieldCounters::set_meter_on(meter_was);
    let filled = filled.expect("the fill runs");
    let mesh = filled.mesh.as_ref().expect("the filled mesh").borrow();
    let at = if filled.offending_vertex >= 0 {
        format!(
            "{:?}",
            mesh.position(filled.offending_vertex) - Vector3::new(-20.0, -118.5, 0.0)
        )
    } else {
        "-".to_string()
    };
    assert_eq!(
        filled.refusal,
        GapFillRefusal::None,
        "{:?} at {} {at} (box frame) {:?}, clip {clip:?}, ends {:?}",
        filled.refusal,
        filled.offending_vertex,
        filled.open_edge,
        ends_site()
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

    // The box frame: the union's corner is at (-20, -118.5, 0) in the fixture's space.
    let corner = Vector3::new(-20.0, -118.5, 0.0);
    let axis_y = 5.0 + (size / (2.0f64 / 3.0).atan().sin());
    let mut worst = 0.0;
    let mut worst_at = Vector3::new(0.0, 0.0, 0.0);
    for &t in &filled.added_triangles {
        let face = mesh.triangles()[t as usize];
        for p in [face.a, face.b, face.c].map(|i| mesh.position(i) - corner) {
            let on_round = (((p.y - axis_y).powi(2) + (p.z - 10.0).powi(2)).sqrt() - size).abs();
            let on_wall = f64::min((p.x - p.y).abs(), (p.x + p.y - 20.0).abs()) / 2f64.sqrt();
            let outside = f64::max(0.0, f64::max(p.y - p.x, p.x + p.y - 20.0) / 2f64.sqrt());
            let off = f64::max(f64::min(on_round, on_wall), outside);
            if off > worst {
                worst = off;
                worst_at = p;
            }
        }
    }

    let volume = signed_volume(&mesh);
    let control = 8000.0 - 2000.0 + 55.5556 - removed;
    println!(
        "fillet thin front at {size}: off the control {worst} at {worst_at:?}, volume {volume} against {control}"
    );
    assert!(
        worst <= size / 50.0,
        "off the control {worst} at {worst_at:?} (box frame)"
    );
    assert!(
        (volume - control).abs() <= 1.0,
        "volume {volume} against {control}"
    );
}
