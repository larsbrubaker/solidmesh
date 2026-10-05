//! Rust-only tests for the bevel-core package R1.P04's methods that no ported C# test class covers
//! yet: the strip's rail readings (`SurfaceStrip`), the strip station (`SurfaceStripStation`),
//! the chamfer mitre's rail-end pairing (`SurfaceCornerMitre.RailEndsPairUp`), a band's trim at a
//! run-out's crossing (`SurfaceStripTrimmedAtTheCrossing`) and the mitre hand-over's staircases
//! (`SurfaceCrossingAtTheMitre.TryTheStaircases`). Each pins the C# rule on a hand-built case.

use std::cell::{Cell, RefCell};
use std::rc::Rc;

use geometry_processing::bevel::surface::{
    surface_corner_mitre, SurfaceCrossingAtTheMitre, SurfaceStrip, SurfaceStripStation,
    SurfaceStripTrimmedAtTheCrossing, SurfaceVertexPool,
};
use vector_math::Vector3;

/// A strip's rails run along its edge from their own feet; a point's parameter is read from the
/// strip's origin; a plane the rails run along is no crossing.
#[test]
fn rust_only_a_strip_reads_its_rails() {
    let strip = SurfaceStrip {
        origin: Vector3::new(0.0, 0.0, 0.0),
        along: Vector3::new(1.0, 0.0, 0.0),
        rail_points: vec![Vector3::new(0.0, 1.0, 0.0), Vector3::new(0.0, 0.0, 1.0)],
        rail_step_degrees: 0.0,
        outward_reference: Vector3::new(0.0, 1.0, 1.0).get_normal(),
    };

    assert_eq!(strip.rail_count(), 2);
    assert_eq!(strip.along(), Vector3::new(1.0, 0.0, 0.0));
    assert_eq!(strip.point_on(1, 3.0), Vector3::new(3.0, 0.0, 1.0));
    assert_eq!(strip.param_of(Vector3::new(4.5, 7.0, -2.0)), 4.5);
    assert_eq!(
        strip.try_cross(0, Vector3::new(2.0, 0.0, 0.0), Vector3::new(1.0, 0.0, 0.0)),
        Some(2.0)
    );
    assert_eq!(
        strip.try_cross(0, Vector3::new(2.0, 0.0, 0.0), Vector3::new(0.0, 1.0, 0.0)),
        None
    );
}

/// A clean station has an empty between for each pair of rails, and none for a lone rail.
#[test]
fn rust_only_a_clean_station_has_nothing_between() {
    let station = SurfaceStripStation::clean(vec![1, 2, 3], vec![0.0, 0.5, 1.0]);
    assert_eq!(station.between, vec![Vec::<i32>::new(); 2]);
    assert!(SurfaceStripStation::clean(vec![7], vec![0.0])
        .between
        .is_empty());
}

/// Three strips' ends chain into one rim only when every end is shared exactly twice and no strip
/// is pinched shut.
#[test]
fn rust_only_rail_ends_pair_up_two_at_a_time() {
    assert!(surface_corner_mitre::rail_ends_pair_up(&[1, 2, 2, 3, 3, 1]));
    assert!(!surface_corner_mitre::rail_ends_pair_up(&[
        1, 1, 2, 3, 3, 2
    ]));
    assert!(!surface_corner_mitre::rail_ends_pair_up(&[
        1, 2, 2, 3, 3, 4
    ]));
}

/// A trim answers which rails it crosses, where, and what the curve carries between two of them;
/// its table answers by strip and end.
#[test]
fn rust_only_a_trim_answers_its_rail_questions() {
    let trim = SurfaceStripTrimmedAtTheCrossing::new(
        3,
        true,
        vec![2, 1],
        vec![0.5, 0.25],
        vec![20, 10],
        vec![vec![], vec![7, 8]],
        30,
        0.1,
    );

    assert!(trim.crosses(1) && trim.crosses(2) && !trim.crosses(0));
    assert_eq!(trim.vertex_on(2), Some(20));
    assert_eq!(trim.param_on(1), Some(0.25));
    assert_eq!(trim.vertex_on(0), None);
    assert_eq!(
        (trim.lowest_rail_crossed(), trim.highest_rail_crossed()),
        (1, 2)
    );
    // The curve ran from rail 2 down to rail 1, so the chord from rail 1 reads it backwards.
    assert_eq!(trim.the_betweens_on_the_chord_from(1), vec![8, 7]);
    assert!(trim.the_betweens_on_the_chord_from(0).is_empty());
    assert_eq!(trim.the_die_vertex.get(), -1);

    let trim = Rc::new(trim);
    let table = vec![[None, None], [None, Some(trim.clone())]];
    assert!(SurfaceStripTrimmedAtTheCrossing::the_band_is_already_crossed(Some(&table), 1, false));
    assert!(!SurfaceStripTrimmedAtTheCrossing::the_band_is_already_crossed(Some(&table), 1, true));
    assert!(SurfaceStripTrimmedAtTheCrossing::standing_at(None, 0, true).is_none());
    assert!(SurfaceStripTrimmedAtTheCrossing::standing_at(Some(&table), 5, true).is_none());

    let mut pool = SurfaceVertexPool::new(&[]);
    let a = pool.add(Vector3::new(0.0, 0.0, 0.0));
    let b = pool.add(Vector3::new(4.0, 0.0, 0.0));
    let c = pool.add(Vector3::new(1.0, 0.0, 0.0));
    assert!(SurfaceStripTrimmedAtTheCrossing::stands_on_the_chord(
        &pool, a, b, c
    ));
    assert!(!SurfaceStripTrimmedAtTheCrossing::stands_on_the_chord(
        &pool, a, c, b
    ));

    let strip = SurfaceStrip {
        origin: Vector3::ZERO,
        along: Vector3::new(0.0, 0.0, 1.0),
        rail_points: vec![Vector3::new(0.0, 0.0, 0.0), Vector3::new(1.0, 0.0, 0.0)],
        rail_step_degrees: 0.0,
        outward_reference: Vector3::new(0.0, 1.0, 0.0),
    };
    let on_rail_one = pool.add(Vector3::new(1.0, 0.0, 3.0));
    assert_eq!(
        SurfaceStripTrimmedAtTheCrossing::try_the_rail_under(&pool, &strip, on_rail_one),
        Some((1, 3.0))
    );
    let off = pool.add(Vector3::new(0.5, 0.0, 3.0));
    assert_eq!(
        SurfaceStripTrimmedAtTheCrossing::try_the_rail_under(&pool, &strip, off),
        None
    );
}

/// The two staircases step down from the shared ring to each band's own crossings, carry the
/// hand-over (and the cap's column tops) on the split chord, and close both trims; trims on the
/// wrong bands are refused.
#[test]
fn rust_only_the_mitre_hands_over_on_two_staircases() {
    let mut pool = SurfaceVertexPool::new(&[]);
    let hand_over = pool.add(Vector3::new(9.0, 9.0, 9.0));
    let crossing = SurfaceCrossingAtTheMitre {
        given_up: vec![false, true, true],
        ring: vec![
            Vector3::new(0.0, 0.0, 0.0),
            Vector3::new(1.0, 0.0, 0.0),
            Vector3::new(2.0, 0.0, 0.0),
        ],
        param_on_the_cap: vec![0.0, 0.1, 0.2],
        param_on_the_wall: vec![1.0, 1.1, 1.2],
        the_cap_band: Cell::new(0),
        the_wall_band: Cell::new(1),
        chord: 0,
        vertex: Cell::new(hand_over),
        along_the_chord: RefCell::new(Some(vec![40, 41])),
        ..SurfaceCrossingAtTheMitre::default()
    };
    assert_eq!(crossing.the_first_rail_under(), 1);

    let trim = |strip: i32, at: Vec<i32>| {
        SurfaceStripTrimmedAtTheCrossing::new(
            strip,
            false,
            vec![1, 2],
            vec![5.0, 6.0],
            at,
            vec![vec![], vec![]],
            -1,
            0.0,
        )
    };
    let wall = trim(1, vec![21, 22]);
    let cap = trim(0, vec![11, 12]);

    // Handed over wall first: the stations come back in that order.
    let (for_the_wall, for_the_cap) = crossing
        .try_the_staircases(&mut pool, Some(&wall), Some(&cap))
        .expect("both trims step down to the hand-over");
    let ring_vertex = pool.add(Vector3::new(0.0, 0.0, 0.0));
    assert_eq!(for_the_cap.rail, vec![ring_vertex, 11, 12]);
    assert_eq!(for_the_cap.param, vec![0.0, 5.0, 6.0]);
    assert_eq!(for_the_cap.between, vec![vec![hand_over, 41, 40], vec![]]);
    assert_eq!(for_the_wall.rail, vec![ring_vertex, 21, 22]);
    assert_eq!(for_the_wall.between, vec![vec![hand_over], vec![]]);
    assert!(wall.closed_the_end.get() && cap.closed_the_end.get());

    let stray = trim(5, vec![1, 2]);
    assert!(crossing
        .try_the_staircases(&mut pool, Some(&stray), Some(&cap))
        .is_err());
    assert!(crossing
        .try_the_staircases(&mut pool, None, Some(&cap))
        .is_err());
}
