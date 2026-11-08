//! Level-line helpers that take input from outside their own module answer edge cases without
//! panicking: the would-thin tally read at a limit it never counted, and a valley cap fanned over
//! an empty outline.

use std::collections::HashSet;

use solidmesh::bevel::level_line::field_counters::{FieldCounters, LevelLineRule};
use solidmesh::bevel::level_line::{strip_clip_valley, LevelLineMesh};
use solidmesh_math::Vector3;

#[test]
fn would_thin_at_a_limit_never_tallied_reads_zero() {
    let _meter = crate::meter_lock::meter_lock();
    FieldCounters::reset();

    // The edge dip's last limit is the cell just before the edge bump's first one, which an
    // unknown limit used to borrow.
    FieldCounters::note_would_thin(LevelLineRule::EdgeDip, 2);
    assert_eq!(FieldCounters::would_thin(LevelLineRule::EdgeDip, 128.0), 1);
    assert_eq!(FieldCounters::would_thin(LevelLineRule::EdgeBump, 50.0), 0);

    // The first rule has no cell before it: an unknown limit read there used to panic.
    assert_eq!(FieldCounters::would_thin(LevelLineRule::None, 50.0), 0);
    FieldCounters::reset();
}

#[test]
fn fanning_an_empty_outline_lays_nothing_and_refuses() {
    let mut mesh = LevelLineMesh::new();
    let mut strip = HashSet::new();
    let fanned =
        strip_clip_valley::fan_facing(&mut mesh, &mut strip, &[], Vector3::new(0.0, 0.0, 1.0))
            .expect("no work to run out of");
    assert!(!fanned);
    assert!(strip.is_empty());
}
