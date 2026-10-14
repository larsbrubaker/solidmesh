//! Rust-only tests for the bevel-core package R1.P04's methods that no other test covers on its
//! own: the strip's rail readings (`SurfaceStrip`), the strip station (`SurfaceStripStation`), the
//! chamfer mitre (`SurfaceCornerMitre`), a band's trim at a run-out's crossing
//! (`SurfaceStripTrimmedAtTheCrossing`) and the mitre hand-over's staircases
//! (`SurfaceCrossingAtTheMitre`), the corner patch's end cut (`SurfaceCornerPatch.TryCutTheEnd`),
//! the end cut above a crossing (`SurfaceEndCutAboveTheCrossing`), the trim by the face beyond's
//! rings (`SurfaceStripTrimmedByTheFaceBeyond`) and the tail and landing-notch readings. The corner
//! closings use the square edge plan R1.P06's tests build; the standing-feature and notch readings
//! are in `rust_only_bevel_r1p04_notch_tests.rs`. Each pins the rule on a hand-built case.

use std::cell::{Cell, RefCell};
use std::rc::Rc;

use geometry_processing::bevel::surface::{
    surface_corner_mitre, surface_corner_patch, surface_end_cut_above_the_crossing,
    surface_strip_ends, SurfaceCornerPatchArrival, SurfaceCrossingAtTheMitre, SurfaceStrip,
    SurfaceStripStation, SurfaceStripTrimmedAtTheCrossing, SurfaceStripTrimmedByTheFaceBeyond,
    SurfaceVertexPool,
};
use geometry_processing::bevel::BevelAdvisoryCode;
use vector_math::Vector3;

use crate::rust_only_bevel_r1p06_tests::square_edge_plan;

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

/// A chamfer strip along `along` from the origin, ruled by two feet.
fn chamfer(along: Vector3, left: Vector3, right: Vector3) -> Rc<SurfaceStrip> {
    Rc::new(SurfaceStrip {
        origin: Vector3::ZERO,
        along,
        rail_points: vec![left, right],
        rail_step_degrees: 0.0,
        outward_reference: (left + right).get_normal() * -1.0,
    })
}

/// Three equal chamfers at a cube corner close on the one point their planes share, each strip
/// ending on its two walkback vertices with that point between them; a pinched set of ends is
/// refused.
#[test]
fn rust_only_three_chamfers_mitre_at_a_cube_corner() {
    let size = 1.0;
    let mut pool = SurfaceVertexPool::new(&[]);
    let on_x = pool.add(Vector3::new(-size, 0.0, 0.0));
    let on_y = pool.add(Vector3::new(0.0, -size, 0.0));
    let on_z = pool.add(Vector3::new(0.0, 0.0, -size));
    let plan = square_edge_plan(size, false);

    let arrival = |strip: Rc<SurfaceStrip>, left: i32, right: i32| SurfaceCornerPatchArrival {
        strip,
        edge: plan.clone(),
        on_the_left: left,
        on_the_right: right,
    };
    let x = Vector3::new(1.0, 0.0, 0.0);
    let y = Vector3::new(0.0, 1.0, 0.0);
    let z = Vector3::new(0.0, 0.0, 1.0);
    let arrivals = vec![
        arrival(chamfer(-x, -y * size, -z * size), on_y, on_z),
        arrival(chamfer(-y, -z * size, -x * size), on_z, on_x),
        arrival(chamfer(-z, -x * size, -y * size), on_x, on_y),
    ];

    let stations = surface_corner_mitre::try_close(&mut pool, &arrivals, Vector3::ZERO)
        .expect("three equal chamfers meet in one point");
    let meeting = pool.add(Vector3::new(-0.5, -0.5, -0.5) * size);
    assert_eq!(stations.len(), 3);
    for (station, arrival) in stations.iter().zip(&arrivals) {
        assert_eq!(
            station.rail,
            vec![arrival.on_the_left, arrival.on_the_right]
        );
        assert!(station.param.iter().all(|param| param.abs() < 1e-12));
        assert_eq!(station.between, vec![vec![meeting]]);
    }

    let mut pinched = arrivals.clone();
    pinched[0].on_the_right = pinched[0].on_the_left;
    assert_eq!(
        surface_corner_mitre::try_close(&mut pool, &pinched, Vector3::ZERO)
            .unwrap_err()
            .code(),
        BevelAdvisoryCode::SurfaceCornerNotSupported
    );
}

/// A round strip cut at the ball's plane lands its outer rails on the walkback's touch points and
/// mints the inner ones; outer rails that miss those points are refused.
#[test]
fn rust_only_a_round_strip_is_cut_at_the_balls_plane() {
    let plan = square_edge_plan(1.0, true);
    let strip = Rc::new(SurfaceStrip::try_create(&plan, 4).expect("a quarter round"));
    let rails = strip.rail_count();
    let centre = Vector3::new(5.0, 1.0, -1.0);

    let mut pool = SurfaceVertexPool::new(&[]);
    let left = pool.add(strip.point_on(0, 5.0));
    let right = pool.add(strip.point_on(rails - 1, 5.0));
    let arrival = SurfaceCornerPatchArrival {
        strip: strip.clone(),
        edge: plan.clone(),
        on_the_left: left,
        on_the_right: right,
    };

    let (ring, meets_at) =
        surface_corner_patch::try_cut_the_end(&mut pool, &arrival, centre, Vector3::ZERO)
            .expect("the outer rails land on the touch points");
    assert_eq!(ring.len() as i32, rails);
    assert_eq!((ring[0], ring[ring.len() - 1]), (left, right));
    assert!(meets_at.iter().all(|param| (param - 5.0).abs() < 1e-12));
    for (rail, &vertex) in ring.iter().enumerate() {
        assert!((pool.position_of(vertex) - strip.point_on(rail as i32, 5.0)).length() < 1e-5);
    }

    let missed = SurfaceCornerPatchArrival {
        on_the_left: pool.add(Vector3::new(9.0, 9.0, 9.0)),
        ..arrival
    };
    assert_eq!(
        surface_corner_patch::try_cut_the_end(&mut pool, &missed, centre, Vector3::ZERO)
            .unwrap_err()
            .code(),
        BevelAdvisoryCode::SurfaceStripSeamDidNotMeet
    );
}

/// Each chord carries what the rim holds between its two rails, the foot goes on the chord beside
/// the crossing with what the cut left on the way to it, and the end plane holds both tangent
/// vertices where neither was crossed.
#[test]
fn rust_only_the_end_cut_above_a_crossing_reads_its_chords() {
    let foot = 50;
    // Rim: rail 0, rail 1, the foot, rail 2 crossed away (absent); the crossing took rails 2..3.
    let rail_of = [0, 1, surface_end_cut_above_the_crossing::THE_FOOT];
    let along = vec![vec![7], vec![8], vec![]];
    let betweens =
        surface_end_cut_above_the_crossing::the_betweens(4, Some(&along), &rail_of, foot, -1, 2, 3);
    // Chord 0 carries what stands between rails 0 and 1; chord 1 (rail 1 to the crossed rail 2)
    // carries what the rim left on the way to the foot, then the foot.
    assert_eq!(betweens, vec![vec![7], vec![8, foot], vec![]]);

    let mut pool = SurfaceVertexPool::new(&[]);
    let strip = SurfaceStrip {
        origin: Vector3::ZERO,
        along: Vector3::new(1.0, 0.0, 0.0),
        rail_points: vec![Vector3::new(0.0, 1.0, 0.0), Vector3::new(0.0, 0.0, -1.0)],
        rail_step_degrees: 0.0,
        outward_reference: Vector3::new(0.0, -1.0, 1.0).get_normal(),
    };
    let left = pool.add(Vector3::new(3.0, 1.0, 0.0));
    let right = pool.add(Vector3::new(3.0, 0.0, -1.0));
    let untouched = SurfaceStripTrimmedAtTheCrossing::new(
        0,
        false,
        vec![5],
        vec![0.0],
        vec![0],
        vec![vec![]],
        -1,
        0.0,
    );
    let normal = surface_end_cut_above_the_crossing::try_the_end_plane(
        &pool,
        &strip,
        Vector3::new(3.0, 0.0, 0.0),
        left,
        right,
        &untouched,
    )
    .expect("neither tangent line was crossed");
    assert!((normal.cross(Vector3::new(1.0, 0.0, 0.0))).length() < 1e-12);

    let both = SurfaceStripTrimmedAtTheCrossing::new(
        0,
        false,
        vec![0, 1],
        vec![0.0, 0.0],
        vec![0, 0],
        vec![vec![], vec![]],
        -1,
        0.0,
    );
    assert!(surface_end_cut_above_the_crossing::try_the_end_plane(
        &pool,
        &strip,
        Vector3::ZERO,
        left,
        right,
        &both
    )
    .is_none());
    assert_eq!(
        surface_strip_ends::meets_the_seam(&pool),
        pool.near_enough()
    );
}

/// The trim by the face beyond collapses the rails it took onto its shared point, walks the rim from
/// one step outside the first kept rail, and reads the profile from its corner beyond.
#[test]
fn rust_only_the_trim_by_the_face_beyond_reads_its_rings() {
    let mut pool = SurfaceVertexPool::new(&[]);
    let strip = SurfaceStrip {
        origin: Vector3::ZERO,
        along: Vector3::new(1.0, 0.0, 0.0),
        rail_points: (0..4)
            .map(|rail| Vector3::new(0.0, rail as f64, 0.0))
            .collect(),
        rail_step_degrees: 30.0,
        outward_reference: Vector3::new(0.0, 0.0, 1.0),
    };
    let shared = pool.add(Vector3::new(9.0, 9.0, 9.0));
    let seam = pool.add(Vector3::new(2.0, 3.0, 0.0));
    let trim = SurfaceStripTrimmedByTheFaceBeyond::new(
        0,
        true,
        5.0,
        vec![0.0, 0.0, 1.0, 2.0],
        2,
        shared,
        99,
        Vector3::ZERO,
        Vector3::new(1.0, 0.0, 0.0),
    );

    assert_eq!(SurfaceStripTrimmedByTheFaceBeyond::rail_at(1, 4, 0), 3);
    let (ring, meets_at) = trim.ring_at(&mut pool, &strip, true, seam);
    assert_eq!((ring[0], ring[1], ring[3]), (shared, shared, seam));
    assert_eq!(meets_at, vec![5.0, 5.0, 1.0, 2.0]);
    assert!((pool.position_of(ring[2]) - Vector3::new(1.0, 2.0, 0.0)).length() < 1e-6);
    assert_eq!(trim.walk_of_the_rim(4), vec![1, 2, 3]);
    assert_eq!(trim.profile_of(&ring), vec![99, shared, ring[2], seam]);
}

/// The section is cut back at the run-out's rim: what stands across the line from the foot to the
/// die point, away from the anchor rail, goes - and the corner with it, the die point standing in.
/// The foot stands on the end plane exactly when its param is where the rail beside it meets it.
#[test]
fn rust_only_the_end_cut_cuts_back_at_the_run_out_and_finds_its_foot() {
    let mut pool = SurfaceVertexPool::new(&[]);
    let foot = pool.add(Vector3::new(0.0, 0.0, 0.0));
    let die = pool.add(Vector3::new(0.0, 1.0, 0.0));
    let kept = pool.add(Vector3::new(-2.0, 0.5, 0.0));
    let beyond = pool.add(Vector3::new(2.0, 0.5, 0.0));

    let trimmed = SurfaceStripTrimmedAtTheCrossing::new(
        0,
        false,
        vec![1, 2],
        vec![2.0, 2.0],
        vec![0, 0],
        vec![vec![], vec![]],
        foot,
        2.0,
    );
    trimmed.the_die_vertex.set(die);

    let mut profile = vec![kept, beyond, foot];
    let mut rail_of = vec![0, 1, surface_end_cut_above_the_crossing::THE_FOOT];
    let mut apex = Vector3::new(3.0, 3.0, 0.0);
    let mut the_die_is_the_apex = false;
    surface_end_cut_above_the_crossing::cut_back_at_the_run_out(
        &pool,
        &mut profile,
        &mut rail_of,
        &trimmed,
        Vector3::new(0.0, 0.0, 1.0),
        &mut apex,
        &mut the_die_is_the_apex,
    );
    assert_eq!(profile, vec![kept, foot]);
    assert_eq!(
        rail_of,
        vec![0, surface_end_cut_above_the_crossing::THE_FOOT]
    );
    assert_eq!(apex, Vector3::new(0.0, 1.0, 0.0));
    assert!(the_die_is_the_apex);

    let strip = SurfaceStrip {
        origin: Vector3::ZERO,
        along: Vector3::new(1.0, 0.0, 0.0),
        rail_points: (0..4)
            .map(|rail| Vector3::new(0.0, rail as f64, 0.0))
            .collect(),
        rail_step_degrees: 30.0,
        outward_reference: Vector3::new(0.0, 0.0, 1.0),
    };
    let on_the_plane = |foot_param: f64| {
        let trimmed = SurfaceStripTrimmedAtTheCrossing::new(
            0,
            false,
            vec![1, 2],
            vec![0.0, 0.0],
            vec![0, 0],
            vec![vec![], vec![]],
            foot,
            foot_param,
        );
        surface_end_cut_above_the_crossing::the_foot_stands_on_the_end_plane(
            &strip,
            Vector3::new(2.0, 0.0, 0.0),
            Vector3::new(1.0, 0.0, 0.0),
            &trimmed,
        )
    };
    assert!(on_the_plane(2.0));
    assert!(!on_the_plane(2.5));
}

/// The tail's cross section stands half way between the notch's exit and the furthest the other end
/// reaches, and the rings past it take the four shapes of a chord; a landing end measures its sliver
/// by a crease vertex standing on the straight side.
#[test]
fn rust_only_the_tail_and_landing_readings() {
    use geometry_processing::bevel::surface::{
        surface_strip_ends_at_a_landing_notch, surface_strip_tail_around_a_feature,
    };

    assert_eq!(
        surface_strip_tail_around_a_feature::try_the_section(10.0, &[2.0, 4.0], 0.1),
        Some(7.0)
    );
    assert_eq!(
        surface_strip_tail_around_a_feature::try_the_section(10.0, &[2.0, 12.0], 0.1),
        None
    );
    assert_eq!(
        surface_strip_tail_around_a_feature::try_the_section(10.0, &[], 0.1),
        None
    );

    // Rails 1 and 2 notched; the ridge leaves between them through the exit.
    let rings = surface_strip_tail_around_a_feature::try_the_rings(
        &[10, 11, 12, 13],
        &[false, true, true, false],
        &[-1, 21, 22, -1],
        1,
        99,
        &[30, -1, 32],
    )
    .expect("every chord beside a notched rail has its Q");
    assert_eq!(
        rings,
        vec![
            vec![10, 11, 21, 30],
            vec![11, 12, 22, 99, 21],
            vec![12, 13, 32, 22]
        ]
    );
    assert_eq!(
        surface_strip_tail_around_a_feature::try_the_rings(
            &[10, 11],
            &[false, true],
            &[-1, 21],
            -1,
            99,
            &[-1]
        ),
        Err(0)
    );

    let mut pool = SurfaceVertexPool::new(&[]);
    let end = pool.add(Vector3::new(0.0, 0.0, 0.0));
    let on_the_side = pool.add(Vector3::new(2.0, 0.0, 0.0));
    let off_the_side = pool.add(Vector3::new(2.0, 1.0, 0.0));
    let corner = Vector3::new(5.0, 0.0, 0.0);
    assert!(surface_end_cut_crease(&pool, end, on_the_side, corner));
    assert!(!surface_end_cut_crease(&pool, end, off_the_side, corner));

    fn surface_end_cut_crease(
        pool: &SurfaceVertexPool,
        end: i32,
        crease: i32,
        corner: Vector3,
    ) -> bool {
        surface_strip_ends_at_a_landing_notch::the_crease_stands_on_the_straight_side(
            pool, end, crease, corner,
        )
    }
}
