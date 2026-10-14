//! A chain vertex turning sharper than the bend threshold is MITRED on the outside of its turn -
//! each edge's line carried on to the bisector plane, the offset polygon's corner - and one at or
//! below the threshold keeps the vertex's ball bit for bit. The Around Center cases fill
//! `around-center-r5` at the app's bend threshold and measure it against `around-center-control`.
//! Each test's cases run as a loop inside it.

use std::cell::RefCell;
use std::collections::HashSet;
use std::rc::Rc;

use geometry_processing::bevel::level_line::band_gap_entry_points::{self, BandGapBuildOptions};
use geometry_processing::bevel::level_line::{gap_fill, ChainField, LevelLineMesh};
use vector_math::dotnet_random::DotNetRandom;
use vector_math::{Matrix4X4, Vector3};

use crate::end_face_builds::feature_of;
use crate::fillet_thin_front_fin::profile_of;
use crate::level_line_fixture::LevelLineFixture;
use crate::round_mouth_fillet_crossings::crossing_pairs;

const THRESHOLD: f64 = 20.0;

/// An L on a plate's top rim: along +x to (10, 0, 0), then a 90-degree left turn along +y.
fn l_chain() -> [Vector3; 3] {
    [
        Vector3::new(0.0, 0.0, 0.0),
        Vector3::new(10.0, 0.0, 0.0),
        Vector3::new(10.0, 10.0, 0.0),
    ]
}

fn within(actual: f64, expected: f64, tolerance: f64) -> bool {
    (actual - expected).abs() <= tolerance
}

/// The outside of a sharp turn reads the offset polygon's corner.
#[test]
fn the_outside_of_a_sharp_turn_reads_the_offset_polygons_corner() {
    let plain = ChainField::from_polyline(&l_chain(), false, &[1.0, 1.0, 1.0]).expect("a chain");
    let mitred = ChainField::from_polyline_sharp(&l_chain(), false, &[1.0, 1.0, 1.0], THRESHOLD)
        .expect("a chain");

    // The rail on the outside: y = -1 up to the mitre point (11, -1), then x = 11.
    for on_rail in [
        Vector3::new(10.5, -1.0, 0.0),
        Vector3::new(11.0, -1.0, 0.0),
        Vector3::new(11.0, -0.5, 0.0),
        Vector3::new(10.001, -1.0, 0.0),
    ] {
        let sample = mitred.evaluate(on_rail).expect("no budget");
        assert!(
            within(sample.value, 0.0, 1e-9),
            "{on_rail:?} is on the offset polygon"
        );
        assert!(
            within(sample.parameter, 10.0, 1e-12),
            "the wedge reads the vertex's parameter"
        );
    }

    assert!(
        within(
            plain
                .evaluate(Vector3::new(11.0, -1.0, 0.0))
                .expect("no budget")
                .value,
            2f64.sqrt() - 1.0,
            1e-12
        ),
        "without a threshold the corner is the vertex's ball, as before"
    );

    // Inside the turn nothing moves.
    for inside in [
        Vector3::new(9.0, 0.5, 0.0),
        Vector3::new(12.0, 0.5, 0.0),
        Vector3::new(9.5, 3.0, 0.0),
        Vector3::new(-1.0, -1.0, 0.0),
    ] {
        assert_eq!(
            mitred.evaluate(inside).expect("no budget").value,
            plain.evaluate(inside).expect("no budget").value,
            "{inside:?} is not in the outside wedge"
        );
    }

    // The least-value queries read the same mitre: a triangle in the wedge, on the in-edge's side.
    let least = mitred
        .min_over_triangle(
            Vector3::new(10.2, -3.0, 0.0),
            Vector3::new(10.8, -3.0, 0.0),
            Vector3::new(10.5, -2.5, 0.0),
        )
        .expect("no budget");
    assert!(
        within(least.value, 1.5, 1e-9),
        "max(0.5, 2.5) - 1 at (10.5, -2.5)"
    );
    let along = mitred
        .min_along_segment(Vector3::new(13.0, -0.5, 0.0), Vector3::new(13.0, -2.0, 0.0))
        .expect("no budget");
    assert!(
        within(along.value, 2.0, 1e-9),
        "the out-edge's line reads 3 - 1 across x = 13"
    );
}

/// A turn below the threshold is bit identical.
#[test]
fn a_turn_below_the_threshold_is_bit_identical() {
    // A 40-sided cylinder rim: every turn is 9 degrees, below the threshold.
    const SIDES: i32 = 40;
    let rim: Vec<Vector3> = (0..SIDES)
        .map(|i| {
            let angle = 2.0 * std::f64::consts::PI * i as f64 / SIDES as f64;
            Vector3::new(20.0 * angle.cos(), 20.0 * angle.sin(), 5.0)
        })
        .collect();
    let setbacks = vec![2.0; SIDES as usize];
    let plain = ChainField::from_polyline(&rim, true, &setbacks).expect("a chain");
    let thresholded =
        ChainField::from_polyline_sharp(&rim, true, &setbacks, THRESHOLD).expect("a chain");

    let mut random = DotNetRandom::new(7);
    for _ in 0..400 {
        let x = (random.next_double() * 60.0) - 30.0;
        let y = (random.next_double() * 60.0) - 30.0;
        let z = random.next_double() * 10.0;
        let p = Vector3::new(x, y, z);
        let a = plain.evaluate(p).expect("no budget");
        let b = thresholded.evaluate(p).expect("no budget");
        assert_eq!(b.value.to_bits(), a.value.to_bits());
        assert_eq!(b.parameter.to_bits(), a.parameter.to_bits());
        let q = p + Vector3::new(1.0, 0.5, 0.25);
        let r = p + Vector3::new(-0.5, 1.0, 0.0);
        assert_eq!(
            thresholded
                .min_over_triangle(p, q, r)
                .expect("no budget")
                .value
                .to_bits(),
            plain
                .min_over_triangle(p, q, r)
                .expect("no budget")
                .value
                .to_bits()
        );
    }
}

/// The closed chain's wrap is mitred like any other vertex.
#[test]
fn the_closed_chains_wrap_is_mitred_like_any_other_vertex() {
    // A closed square, starting at a corner: the wrap vertex (0, 0, 0) turns 90 degrees like the
    // rest.
    let square = [
        Vector3::new(0.0, 0.0, 0.0),
        Vector3::new(10.0, 0.0, 0.0),
        Vector3::new(10.0, 10.0, 0.0),
        Vector3::new(0.0, 10.0, 0.0),
    ];
    let field = ChainField::from_polyline_sharp(&square, true, &[1.0, 1.0, 1.0, 1.0], THRESHOLD)
        .expect("a chain");
    let wrap = field
        .evaluate(Vector3::new(-1.0, -1.0, 0.0))
        .expect("no budget");
    let mid = field
        .evaluate(Vector3::new(11.0, -1.0, 0.0))
        .expect("no budget");
    assert!(within(wrap.value, 0.0, 1e-9));
    assert!(within(mid.value, 0.0, 1e-9));
    assert_eq!(
        wrap.parameter, 0.0,
        "the wrap reads its canonical parameter on both sides of its bisector"
    );
    assert_eq!(
        field
            .evaluate(Vector3::new(-1.0, -0.5, 0.0))
            .expect("no budget")
            .parameter,
        0.0
    );
    assert_eq!(
        field
            .evaluate(Vector3::new(-0.5, -1.0, 0.0))
            .expect("no budget")
            .parameter,
        0.0
    );
}

/// Size / 50 and not the old construction's
/// 0.05 - CAD exactness is not the goal (plan, decision 4).
#[test]
fn around_center_follows_the_control_at_its_sharp_turns() {
    let (worst, worst_at, size) = around_center_worst_off_the_control_shifted(Vector3::ZERO, false);
    assert!(
        worst <= size / 50.0,
        "worst published vertex {worst_at:?} stands {worst:.3} off the control"
    );
}

/// Around Center's tip spokes are drawn on the partner's leg.
#[test]
fn around_center_tip_spokes_are_drawn_on_the_partners_leg() {
    // Tighter than size / 50, so a slip back is seen before the test above fails: the tip spoke
    // v21-v63 stood 0.124 off the control while its seam-plane end read the incoming leg
    // (FilletArc.OnThePartnersLeg), and the far tip's in-seam row v23-v26 0.108 while it was drawn
    // about one chain point (FilletArc.InTheSeam); both mended, the worst is 0.072, still on that
    // in-seam row.
    let (worst, worst_at, _) = around_center_worst_off_the_control_shifted(Vector3::ZERO, false);
    assert!(
        worst <= 0.075,
        "worst published vertex {worst_at:?} stands {worst:.3} off the control"
    );
}

/// Around Center builds without crossing cells.
#[test]
fn around_center_builds_without_crossing_cells() {
    // The seam rows (FilletArc.InTheSeam) and the re-read tip spokes sit between rows still drawn
    // about a leg's chain point; none of their cells may cross another.
    let (mesh, _, _) = around_center_fill(&|p| p, false);
    assert_eq!(crossing_pairs(&mesh.borrow()), 0);
}

/// Around Center far from the origin reads the same worst.
#[test]
fn around_center_far_from_the_origin_reads_the_same_worst() {
    // Moved 1000 mm off in each axis: the seam rows must still be taken, which a tolerance scaled
    // by the chain's length alone does not promise (FilletArc.SeamTolerance scales with the
    // coordinates).
    let (worst, _, _) = around_center_worst_off_the_control_shifted(Vector3::ZERO, false);
    let shift = Vector3::new(1000.0, 1000.0, 1000.0);
    let (far_worst, far_worst_at, _) = around_center_worst_off_the_control_shifted(shift, false);
    assert!(
        within(far_worst, worst, 1e-6),
        "far from the origin the worst is {far_worst:.4} at {:?}",
        far_worst_at - shift
    );
}

/// Around Center rounded through floats reads the same worst.
#[test]
fn around_center_rounded_through_floats_reads_the_same_worst() {
    // Rails come through floats: ~4e-6 of noise at these coordinates, above a chain-length 1e-7
    // tolerance.
    let (worst, _, _) = around_center_worst_off_the_control_shifted(Vector3::ZERO, false);
    let (rounded, rounded_at, _) = around_center_worst_off_the_control_shifted(Vector3::ZERO, true);
    assert!(
        within(rounded, worst, 2e-3),
        "rounded through floats the worst is {rounded:.4} at {rounded_at:?}"
    );
}

/// Around Center rounded through floats far from the origin (100, 300 and 1000 out) reads the
/// same worst distance off the control.
#[test]
fn around_center_rounded_through_floats_far_from_the_origin_reads_the_same_worst() {
    // The inner diamond's vertex on the wrap joint's bisector is on both legs, and the float
    // rounding picks which one its reading names; the merge measures its spokes at the incoming
    // leg's reading instead (GapFill.IncomingReadingAt).
    let (worst, _, _) = around_center_worst_off_the_control_shifted(Vector3::ZERO, false);
    for offset in [100.0, 300.0, 1000.0] {
        let shift = Vector3::new(offset, offset, offset);
        let (far_worst, far_worst_at, _) = around_center_worst_off_the_control_shifted(shift, true);
        assert!(
            within(far_worst, worst, 2e-3),
            "{offset}: far from the origin the worst is {far_worst:.4} at {:?}",
            far_worst_at - shift
        );
    }
}

/// For seeds 1-4: Around Center placed
/// by a rigid motion (rotated about a seeded random axis, then moved a seeded random distance out)
/// and rounded through floats must build the same fill as at the origin: the same worst distance
/// off the control, and the same triangle count - the merge's spokes and the rails' starts do not
/// turn on which leg float noise makes a tie vertex read.
#[test]
fn around_center_rotated_and_moved_through_floats_builds_the_same_fill() {
    let (worst, _, _, triangles) = around_center_worst_off_the_control(&|p| p, false);
    for seed in 1..=4 {
        let mut random = DotNetRandom::new(seed);
        let axis = unit_from(&mut random).get_normal();
        let angle = random_angle(&mut random);
        let shift = unit_from(&mut random) * 2000.0;
        let rotation = Matrix4X4::create_rotation_axis_angle(axis, angle);
        let place = move |p: Vector3| p.transform(rotation) + shift;

        let (placed_worst, placed_worst_at, _, placed_triangles) =
            around_center_worst_off_the_control(&place, true);
        assert!(
            within(placed_worst, worst, 2e-3),
            "seed {seed}: placed, the worst is {placed_worst:.4} at {placed_worst_at:?}"
        );
        assert_eq!(placed_triangles, triangles, "seed {seed}");
    }
}

/// A seeded random angle in [0, 2 pi).
fn random_angle(random: &mut DotNetRandom) -> f64 {
    random.next_double() * 2.0 * std::f64::consts::PI
}

/// A seeded random unit direction.
fn unit_from(random: &mut DotNetRandom) -> Vector3 {
    let x = random.next_double() - 0.5;
    let y = random.next_double() - 0.5;
    let z = random.next_double() - 0.5;
    Vector3::new(x, y, z)
}

/// The Around Center fill with every vertex moved by `place`, rounded through floats when
/// `through_floats`.
fn around_center_fill(
    place: &dyn Fn(Vector3) -> Vector3,
    through_floats: bool,
) -> (Rc<RefCell<LevelLineMesh>>, LevelLineFixture, f64) {
    let moved = |p: Vector3| {
        let m = place(p);
        if through_floats {
            Vector3::new(m.x as f32 as f64, m.y as f32 as f64, m.z as f32 as f64)
        } else {
            m
        }
    };

    let fixture = Rc::new(LevelLineFixture::load("around-center-r5"));
    let control = LevelLineFixture::load("around-center-control");
    let positions: Vec<Vector3> = fixture.positions.iter().map(|&p| moved(p)).collect();
    let gap = band_gap_entry_points::build_pick(
        &positions,
        &fixture.indices,
        &fixture.chains,
        &fixture.closed,
        Some(feature_of(&fixture)),
        fixture.size,
        profile_of(&fixture),
        &BandGapBuildOptions {
            chord_tolerance: 0.0,
            snap: 1e-9,
            known_room: None,
            seam_epsilon: 0.0,
            // the app's default bend threshold
            sharp_turn_degrees: 10.0,
            ..BandGapBuildOptions::default()
        },
    )
    .expect("a valid pick");
    assert!(gap.built(), "refusal {:?}, room {}", gap.refusal, gap.room);

    let filled =
        gap_fill::fill(&gap, profile_of(&fixture), fixture.segments).expect("the fill runs");
    let mesh = filled.mesh.expect("a filled mesh");
    (mesh, control, fixture.size)
}

/// The Around Center fill's worst distance off the control, both moved by `shift`.
fn around_center_worst_off_the_control_shifted(
    shift: Vector3,
    through_floats: bool,
) -> (f64, Vector3, f64) {
    let (worst, worst_at, size, _) =
        around_center_worst_off_the_control(&move |p| p + shift, through_floats);
    (worst, worst_at, size)
}

/// The published vertex standing
/// furthest off the control's surface, and the fill's live triangle count.
fn around_center_worst_off_the_control(
    place: &dyn Fn(Vector3) -> Vector3,
    through_floats: bool,
) -> (f64, Vector3, f64, i32) {
    let (mesh, control, size) = around_center_fill(place, through_floats);
    let mesh = mesh.borrow();
    let control_positions: Vec<Vector3> = control.positions.iter().map(|&p| place(p)).collect();

    let mut used = HashSet::new();
    let mut triangles = 0;
    for (t, triangle) in mesh.triangles().iter().enumerate() {
        if !mesh.is_removed(t as i32) {
            used.insert(triangle.a);
            used.insert(triangle.b);
            used.insert(triangle.c);
            triangles += 1;
        }
    }

    let mut worst = 0.0;
    let mut worst_at = Vector3::ZERO;
    for v in used {
        let p = mesh.position(v);
        let mut best = f64::INFINITY;
        for &(a, b, c) in &control.indices {
            best = best.min(
                (ChainField::closest_on_triangle(
                    p,
                    control_positions[a as usize],
                    control_positions[b as usize],
                    control_positions[c as usize],
                ) - p)
                    .length(),
            );
        }

        if best > worst {
            worst = best;
            worst_at = p;
        }
    }

    (worst, worst_at, size, triangles)
}
