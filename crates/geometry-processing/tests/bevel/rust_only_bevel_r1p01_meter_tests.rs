//! Rust-only checks of the bevel-core round-1 meters and witnesses (package R1.P01, run 2): the
//! report number formats, the field counters' bands and switches, the needle meter's tallies, the
//! split meter's edge distance, the hull clip's supporting planes and the side witness's triangle
//! lookup. The whole-build tests also cover them.

use std::collections::HashSet;

use geometry_processing::bevel::level_line::stopwatch::{e3, g4};
use geometry_processing::bevel::level_line::{
    hull_clip, side_witness, FieldCounters, LevelLineMesh, LevelLineTriangle, NeedleMeter,
    SplitMeter,
};
use vector_math::number_format::format_i64_number;
use vector_math::Vector3;

fn v(x: f64, y: f64, z: f64) -> Vector3 {
    Vector3::new(x, y, z)
}

/// A unit square in z = 0 (triangles 0 and 1, normal +z) and a strip triangle (2) rising to z = 1.
fn square_and_strip() -> LevelLineMesh {
    LevelLineMesh {
        vertices: vec![
            v(0.0, 0.0, 0.0),
            v(1.0, 0.0, 0.0),
            v(1.0, 1.0, 0.0),
            v(0.0, 1.0, 0.0),
            v(0.5, 0.5, 1.0),
        ],
        triangles: vec![
            LevelLineTriangle {
                a: 0,
                b: 1,
                c: 2,
                tag: 1,
            },
            LevelLineTriangle {
                a: 0,
                b: 2,
                c: 3,
                tag: 1,
            },
            LevelLineTriangle {
                a: 0,
                b: 1,
                c: 4,
                tag: 2,
            },
        ],
        removed: vec![false, false, false],
        ..Default::default()
    }
}

#[test]
fn rust_only_meter_reports_write_their_pinned_number_formats() {
    assert_eq!(format_i64_number(0), "0");
    assert_eq!(format_i64_number(999), "999");
    assert_eq!(format_i64_number(1234567), "1,234,567");
    assert_eq!(format_i64_number(-1000), "-1,000");

    assert_eq!(g4(0.0), "0");
    assert_eq!(g4(1.5), "1.5");
    assert_eq!(g4(123.456), "123.5");
    assert_eq!(g4(12345.0), "1.234E+04");
    assert_eq!(g4(0.0001234), "0.0001234");
    assert_eq!(g4(0.000001234), "1.234E-06");
    assert_eq!(g4(f64::INFINITY), "∞");

    assert_eq!(e3(0.00002), "2.000E-005");
    assert_eq!(e3(12345.0), "1.234E+004");
    assert_eq!(e3(-1.0), "-1.000E+000");
}

#[test]
fn rust_only_field_counters_file_spans_and_reaches_into_bands() {
    assert_eq!(FieldCounters::bucket_of(0.25), 0);
    assert_eq!(FieldCounters::bucket_of(1.0), 2);
    assert_eq!(FieldCounters::bucket_of(100.0), 6);
    assert_eq!(FieldCounters::buckets(), 7);
    assert_eq!(FieldCounters::reach_of(0.5), 0);
    assert_eq!(FieldCounters::reach_of(1.0), 1);
    assert_eq!(FieldCounters::reach_of(16.0), 4);
    assert_eq!(FieldCounters::reaches(), 5);

    // Per thread: this test's thread starts from zero whatever other tests count.
    FieldCounters::with(|t| t.evaluate = 7);
    assert!(FieldCounters::report().starts_with("Evaluate 7 (pick queries 0)"));
    assert_eq!(FieldCounters::report().lines().count(), 3);
}

#[test]
fn rust_only_needle_meter_counts_a_cascade_only_while_begun_and_metered() {
    let _meter = crate::meter_lock::meter_lock();
    FieldCounters::set_meter_on(true);
    NeedleMeter::begin(2, 10.0);
    assert!(NeedleMeter::on());
    NeedleMeter::start_call();
    NeedleMeter::note_repair(0, 5, 1, 2, 3, 0.5, 2.0, 1.0);
    NeedleMeter::note_repair_left(0, true);
    NeedleMeter::note_repair(0, 6, 1, 2, 3, 50.0, 2.0, 1.0);
    NeedleMeter::end_call(true);
    NeedleMeter::note_no_needle();
    let report = NeedleMeter::report();
    NeedleMeter::end();
    FieldCounters::set_meter_on(false);
    assert!(!NeedleMeter::on());

    assert!(report.starts_with("needle meter: 2 repairs; 2 per-split calls, 1 of them"));
    assert!(report.contains("2-10: 1"));
    assert!(report.contains("longest cascade 2 repairs in call 1, through 2 distinct apexes"));
    assert!(report.contains("repaired needles made by Input: 1; height/floor < 1x: 1"));
    assert!(report.contains("repaired needles made by Repair: 1;"));
    assert!(
        report.contains("made by Input, apex 5 on long edge (1,2) re-tiling 3, height/floor 0.5")
    );
}

#[test]
fn rust_only_split_meter_reads_a_point_off_an_edge() {
    let (a, b) = (v(0.0, 0.0, 0.0), v(2.0, 0.0, 0.0));
    assert_eq!(SplitMeter::off_the_edge(a, b, v(1.0, 3.0, 0.0)), 3.0);
    assert_eq!(SplitMeter::off_the_edge(a, b, v(5.0, 4.0, 0.0)), 5.0);
    assert_eq!(SplitMeter::off_the_edge(a, a, v(0.0, 0.0, 2.0)), 2.0);
}

#[test]
fn rust_only_hull_clip_keeps_the_plane_a_strip_rises_out_of() {
    let mesh = square_and_strip();
    let planes = hull_clip::crossed_supporting_planes(&mesh, &HashSet::from([2]));
    assert_eq!(planes.len(), 1);
    assert_eq!(planes[0].normal, v(0.0, 0.0, 1.0));
    assert_eq!(planes[0].on, HashSet::from([0, 1, 2, 3]));

    // With the strip below the plane instead, no strip vertex stands outside it.
    let mut below = square_and_strip();
    below.vertices[4] = v(0.5, 0.5, -1.0);
    assert!(hull_clip::crossed_supporting_planes(&below, &HashSet::from([2])).is_empty());
}

#[test]
fn rust_only_side_witness_finds_the_triangle_holding_a_point() {
    let mesh = square_and_strip();
    let side = side_witness::live_triangles_of_side(&mesh, 1);
    assert_eq!(side, vec![0, 1]);
    assert_eq!(
        side_witness::triangle_holding(&mesh, &side, v(0.8, 0.2, 0.0)),
        0
    );
    assert_eq!(
        side_witness::triangle_holding(&mesh, &side, v(0.2, 0.8, 0.0)),
        1
    );
    assert_eq!(
        side_witness::triangle_holding(&mesh, &side, v(0.2, 0.8, 0.1)),
        -1
    );

    side_witness::update_tally(|t| t.entered += 2);
    assert_eq!(side_witness::tally().entered, 2);
    side_witness::reset();
    assert_eq!(side_witness::tally().entered, 0);
}
