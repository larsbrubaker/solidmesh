//! The speed requirement, and the proof that meeting it changed no answer. A live drag rebuilds the
//! band every step, so a 20,000 triangle part with a 256 segment rim has to build in a fraction of
//! a second, and the indexed field must answer exactly what the plain loop over every chain segment
//! answered. The class's cylinder is `tessellated_cylinder.rs`. It runs alone (a `threads-required`
//! override in `.config/nextest.toml`), so its timings read undisturbed. Each test's cases run as a
//! loop inside it.

use std::rc::Rc;
use std::time::Instant;

use solidmesh::bevel::level_line::band_gap_entry_points::{self, BandGapBuildOptions};
use solidmesh::bevel::level_line::band_gap_reading;
use solidmesh::bevel::level_line::{
    gap_fill, BandGapRefusal, BandProfile, ChainField, FieldCounters, WorkBudget,
};
use solidmesh_math::seeded_random::SeededRandom;
use solidmesh_math::Vector3;

use crate::level_line_feature_edges::feature_edges_over;
use crate::level_line_room_geometry::SolidWithChain;
use crate::tessellated_cylinder::tessellated_cylinder;

const FOLD_DEGREES: f64 = 45.0;

/// The wall-clock hang guard for the drag budget row: today's Debug build+fill reads 0.54 s on a
/// quiet machine and 1.8 s on a loaded one, so 10 s never trips on load yet still trips on the 13 s
/// and 77 s regressions that row was written against.
const HANG_GUARD_MILLISECONDS: f64 = 10_000.0;

/// THE DRAG BUDGET IN WORK UNITS: what one drag step on the 20,000 triangle cylinder's top rim may
/// charge, build and fill together. Read 2026-09-30 as 82,089 build + 8,662 fill = 90,751 for the
/// fillet and 82,089 + 768 = 82,857 for the chamfer, identical run to run; each ceiling sits a
/// tenth above, so a change that makes the step cost noticeably more work fails here whatever the
/// machine's load, and one that makes it cheaper lowers the reading without breaking anything.
fn unit_ceiling(profile: BandProfile) -> i64 {
    if profile == BandProfile::Fillet {
        100_000
    } else {
        91_000
    }
}

fn within(actual: f64, expected: f64, tolerance: f64) -> bool {
    (actual - expected).abs() <= tolerance
}

/// The index may only make the same query
/// cheaper, never different - over random points, segments and triangles round a wiggly chain with
/// a varying setback, every indexed answer equals the exhaustive one to the exactness the rest of
/// the module is pinned to.
#[test]
fn indexed_queries_answer_what_the_exhaustive_search_answers() {
    let mut random = SeededRandom::new(20260918);
    let mut points = Vec::new();
    let mut setbacks = Vec::new();
    for i in 0..40 {
        let angle = (i as f64 / 40.0) * std::f64::consts::PI * 2.0;
        points.push(Vector3::new(
            10.0 * angle.cos(),
            10.0 * angle.sin(),
            2.0 * (3.0 * angle).sin(),
        ));

        // A varying setback is the case the bound has to give away the most to, so it is the one
        // tested.
        setbacks.push(1.0 + (0.5 * (5.0 * angle).sin()));
    }

    let field = ChainField::from_polyline(&points, true, &setbacks).expect("a chain");

    let somewhere = |random: &mut SeededRandom| {
        let x = (random.next_double() * 30.0) - 15.0;
        let y = (random.next_double() * 30.0) - 15.0;
        let z = (random.next_double() * 10.0) - 5.0;
        Vector3::new(x, y, z)
    };

    for _ in 0..300 {
        let p = somewhere(&mut random);
        let indexed = field.evaluate(p).expect("no budget");
        let exhaustive = field.evaluate_exhaustive(p);
        assert!(within(indexed.distance, exhaustive.distance, 1e-9));
        assert!(within(indexed.parameter, exhaustive.parameter, 1e-9));
        assert!(within(indexed.value, exhaustive.value, 1e-9));
        assert!((indexed.nearest - exhaustive.nearest).length() < 1e-9);
    }

    for _ in 0..60 {
        let a = somewhere(&mut random);
        let b = somewhere(&mut random);
        let c = somewhere(&mut random);
        assert_segment_and_triangle_agree(&field, a, b, c, true, true, "random");
    }
}

/// The closed form used where a chain
/// segment's setback is constant may only make the same query cheaper, never different - held to
/// the convex search it replaces, over the configurations closed forms are usually got wrong in:
/// parallel, touching, crossing, coplanar and degenerate.
///
/// The VALUE is compared everywhere, to the exactness the module is pinned to. The POINT is only
/// compared where the minimum is unique: a mesh piece parallel to a chain segment attains its least
/// value over a whole interval, and which point of that interval a search lands on is arbitrary.
/// The reference - `least_over_piece_exhaustive`, handed an opaque closest-point function - has no
/// closed form to reach for, so it really is the old search answering.
#[test]
fn the_closed_form_answers_what_the_convex_search_answers() {
    // A constant setback everywhere is what puts every chain segment down the closed-form path.
    let square = [
        Vector3::new(0.0, 0.0, 0.0),
        Vector3::new(10.0, 0.0, 0.0),
        Vector3::new(10.0, 10.0, 0.0),
        Vector3::new(0.0, 10.0, 0.0),
    ];
    let field = ChainField::from_polyline(&square, true, &[1.0; 4]).expect("a chain");
    let v = Vector3::new;

    // Each row: the piece, and whether its least value is attained at ONE point.
    let segments = [
        (
            v(2.0, 0.0, 3.0),
            v(8.0, 0.0, 3.0),
            false,
            "parallel to a chain segment",
        ),
        (
            v(2.0, 0.0, 0.0),
            v(8.0, 0.0, 0.0),
            false,
            "lying along a chain segment",
        ),
        (
            v(5.0, 0.0, 0.0),
            v(5.0, 0.0, 5.0),
            true,
            "touching the chain",
        ),
        (
            v(5.0, -5.0, 0.0),
            v(5.0, 5.0, 0.0),
            true,
            "crossing the chain",
        ),
        (
            v(3.0, 2.0, 1.0),
            v(3.0, 2.0, 1.0),
            true,
            "a zero length segment",
        ),
        (
            v(5.0, 5.0, 2.0),
            v(6.0, 4.0, 2.0),
            true,
            "skew to every chain segment",
        ),
        (
            v(-4.0, -4.0, 0.0),
            v(-4.0, 14.0, 0.0),
            true,
            "outside the square, parallel to a side",
        ),
        (
            v(0.0, 0.0, 4.0),
            v(10.0, 10.0, 4.0),
            true,
            "over the square's diagonal",
        ),
    ];

    for (a, b, unique, what) in segments {
        let indexed = field.min_along_segment(a, b).expect("no budget");
        let reference = field
            .least_over_piece_exhaustive(Rc::new(move |p| closest_on_segment(p, a, b)))
            .expect("no budget");
        assert!(within(indexed.value, reference.value, 1e-9), "{what}");
        if unique {
            assert!((indexed.point - reference.point).length() < 1e-6, "{what}");
        }
    }

    let triangles = [
        (
            v(2.0, -2.0, 0.0),
            v(8.0, -2.0, 0.0),
            v(5.0, 4.0, 0.0),
            false,
            "coplanar, swallowing a chain segment",
        ),
        (
            v(2.0, 2.0, 3.0),
            v(8.0, 2.0, 3.0),
            v(5.0, 6.0, 3.0),
            false,
            "in a plane parallel to the chain's, near edge parallel too",
        ),
        (
            v(2.0, 0.0, 2.0),
            v(8.0, 0.0, 2.0),
            v(5.0, 0.0, 6.0),
            false,
            "standing on a chain segment, edge parallel to it",
        ),
        (
            v(5.0, 0.0, 0.0),
            v(7.0, 3.0, 4.0),
            v(3.0, 3.0, 4.0),
            true,
            "touching the chain at a corner",
        ),
        (
            v(5.0, -3.0, 0.0),
            v(5.0, 3.0, 2.0),
            v(5.0, 3.0, -2.0),
            true,
            "pierced by a chain segment",
        ),
        (
            v(1.0, 1.0, 1.0),
            v(3.0, 3.0, 3.0),
            v(5.0, 5.0, 5.0),
            true,
            "degenerate - three collinear points",
        ),
        (
            v(4.0, 4.0, 4.0),
            v(4.0, 4.0, 4.0),
            v(4.0, 4.0, 4.0),
            true,
            "degenerate - one point",
        ),
        (
            v(-4.0, -4.0, 1.0),
            v(-2.0, -6.0, 3.0),
            v(-6.0, -8.0, 2.0),
            true,
            "clear of the square",
        ),
    ];

    for (a, b, c, unique, what) in triangles {
        let indexed = field.min_over_triangle(a, b, c).expect("no budget");
        let reference = field
            .least_over_piece_exhaustive(Rc::new(move |p| closest_on_triangle(p, a, b, c)))
            .expect("no budget");
        assert!(within(indexed.value, reference.value, 1e-9), "{what}");
        if unique {
            assert!((indexed.point - reference.point).length() < 1e-6, "{what}");
        }
    }

    // And the general case, where a random piece's minimum is unique with probability one.
    let mut random = SeededRandom::new(20260918);
    let somewhere = |random: &mut SeededRandom| {
        let x = (random.next_double() * 24.0) - 7.0;
        let y = (random.next_double() * 24.0) - 7.0;
        let z = (random.next_double() * 12.0) - 6.0;
        Vector3::new(x, y, z)
    };

    for _ in 0..120 {
        let a = somewhere(&mut random);
        let b = somewhere(&mut random);
        let c = somewhere(&mut random);
        assert_segment_and_triangle_agree(&field, a, b, c, true, true, "random");
    }
}

/// The random rows of both tests: segment `a`-`b` and triangle `a b c`, each held to the
/// exhaustive reference.
fn assert_segment_and_triangle_agree(
    field: &ChainField,
    a: Vector3,
    b: Vector3,
    c: Vector3,
    segment_unique: bool,
    triangle_unique: bool,
    what: &str,
) {
    let along_indexed = field.min_along_segment(a, b).expect("no budget");
    let along_reference = field
        .least_over_piece_exhaustive(Rc::new(move |p| closest_on_segment(p, a, b)))
        .expect("no budget");
    assert!(
        within(along_indexed.value, along_reference.value, 1e-9),
        "{what}"
    );
    if segment_unique {
        assert!((along_indexed.point - along_reference.point).length() < 1e-6);
    }

    let over_indexed = field.min_over_triangle(a, b, c).expect("no budget");
    let over_reference = field
        .least_over_piece_exhaustive(Rc::new(move |p| closest_on_triangle(p, a, b, c)))
        .expect("no budget");
    assert!(
        within(over_indexed.value, over_reference.value, 1e-9),
        "{what}"
    );
    if triangle_unique {
        assert!((over_indexed.point - over_reference.point).length() < 1e-6);
    }
}

/// For a fillet and a chamfer: the
/// drag budget, as a ceiling a regression trips over - a fillet and a chamfer on the top rim of a
/// 20,000 triangle cylinder, each built and filled inside [`unit_ceiling`] work units, with a
/// wall-clock hang guard behind it.
#[test]
fn a_rim_of_a_twenty_thousand_triangle_cylinder_builds_inside_the_drag_budget() {
    for profile in [BandProfile::Fillet, BandProfile::Chamfer] {
        let (positions, indices, rim) = tessellated_cylinder(256, 38, 10.0, 20.0);
        assert!(indices.len() > 19000);

        let feature = feature_edges_over(&positions, &indices, FOLD_DEGREES);

        FieldCounters::reset();
        let build_budget = Rc::new(
            WorkBudget::new(WorkBudget::default_limit(indices.len() as i32)).expect("a limit"),
        );
        let watch = Instant::now();
        let gap = band_gap_entry_points::build(
            &positions,
            &indices,
            &rim,
            true,
            Some(feature),
            1.0,
            profile,
            &BandGapBuildOptions {
                chord_tolerance: 0.0,
                snap: 1e-9,
                known_room: None,
                budget: Some(Rc::clone(&build_budget)),
                ..BandGapBuildOptions::default()
            },
        )
        .expect("a valid pick");
        let built = watch.elapsed().as_secs_f64() * 1000.0;
        assert_eq!(gap.refusal, BandGapRefusal::None, "{profile:?}");

        let triangle_count = gap
            .mesh
            .as_ref()
            .expect("a built band")
            .borrow()
            .triangle_count();
        let fill_budget =
            Rc::new(WorkBudget::new(WorkBudget::default_limit(triangle_count)).expect("a limit"));
        let fill = gap_fill::fill_with_budget(
            &gap,
            profile,
            if profile == BandProfile::Fillet { 4 } else { 1 },
            Some(Rc::clone(&fill_budget)),
        )
        .expect("the fill runs");
        let total = watch.elapsed().as_secs_f64() * 1000.0;
        let units = build_budget.spent.get() + fill_budget.spent.get();
        println!(
            "cylinder rim {profile:?}: build {built:.0} ms, build+fill {total:.0} ms, {} + {} = {units} units, {} triangles added",
            build_budget.spent.get(),
            fill_budget.spent.get(),
            fill.added_triangles.len()
        );
        println!("{}", FieldCounters::report());

        assert!(units < unit_ceiling(profile), "{profile:?}: {units} units");

        // A HANG GUARD, not the budget: the claim above is in work units because they repeat to
        // the digit whatever else the machine runs, and a wall-clock ceiling near the honest
        // 0.54 s failed under load with nothing wrong. This one sits ten times over today's Debug
        // reading so only a regression back towards the old 77 seconds (or the 13, 5.6 or 2.1
        // before the bounds, closed forms and bracketed roots) trips it - including one in work
        // the budget does not charge.
        assert!(total < HANG_GUARD_MILLISECONDS, "{profile:?}: {total} ms");
    }
}

/// The drag itself - the room is a statement about
/// the part and the pick, so it is read ONCE and every size after that is a bare build handed that
/// room. The row printed is what a user feels per step.
#[test]
fn a_drag_over_ten_sizes_pays_for_its_room_only_once() {
    let (positions, indices, rim) = tessellated_cylinder(256, 38, 10.0, 20.0);
    let feature = feature_edges_over(&positions, &indices, FOLD_DEGREES);

    let watch = Instant::now();
    let room = band_gap_reading::room_of(
        &positions,
        &indices,
        &rim,
        true,
        Some(Rc::clone(&feature)),
        BandProfile::Fillet,
        1e-9,
    )
    .expect("the room reads");
    let room_milliseconds = watch.elapsed().as_secs_f64() * 1000.0;

    // The room a build reads for itself must be the room asked for without one - otherwise a
    // drag's first step would answer a different question from every step after it.
    let read_by_a_build = band_gap_entry_points::build(
        &positions,
        &indices,
        &rim,
        true,
        Some(Rc::clone(&feature)),
        0.5,
        BandProfile::Fillet,
        &BandGapBuildOptions::default(),
    )
    .expect("a valid pick")
    .room;
    assert_eq!(room, read_by_a_build);

    let mut per_build = Vec::new();
    for step in 0..10 {
        let size = 0.5 + (step as f64 * 0.05);
        let at = Instant::now();
        let gap = band_gap_entry_points::build(
            &positions,
            &indices,
            &rim,
            true,
            Some(Rc::clone(&feature)),
            size,
            BandProfile::Fillet,
            &BandGapBuildOptions {
                known_room: Some(room),
                ..BandGapBuildOptions::default()
            },
        )
        .expect("a valid pick");
        per_build.push(at.elapsed().as_secs_f64() * 1000.0);
        assert_eq!(gap.refusal, BandGapRefusal::None, "{size}");
    }

    let average = per_build.iter().sum::<f64>() / per_build.len() as f64;
    println!(
        "drag row: room {room_milliseconds:.0} ms once, then {} builds averaging {average:.0} ms (least {:.0}, greatest {:.0})",
        per_build.len(),
        per_build.iter().cloned().fold(f64::INFINITY, f64::min),
        per_build.iter().cloned().fold(f64::NEG_INFINITY, f64::max)
    );
}

/// A build handed its room costs what the BAND costs
/// and not what the MODEL costs - the same top rim on a cylinder of ten times the triangles costs
/// only a little more, because the walk out from the chain never enters the surface the line
/// cannot reach.
///
/// **The GROWTH, not the ratio.** What the model's share costs is `big - small`; the ratio `big /
/// small` is that same growth divided by a number that has nothing to do with the model, so it
/// moves whenever the BAND gets cheaper - which is a thing to want, not a regression. Pruning the
/// ridge rule's peak search (see `ridge_search_cost_tests`) took 12,288 work units off each row
/// alike, leaving the growth untouched at 98,574 while the ratio rose from 3.16 to 3.96 and broke a
/// `< 4x` claim. The growth is the quantity the requirement is about, and it is read the same on
/// both.
///
/// **In work units, not milliseconds.** A work unit is an integer the algorithm decides, so the two
/// rows repeat to the digit whatever else the machine is running. The times are still printed for
/// the record and nothing is asserted on them.
///
/// **Why the growth is not zero.** Two things still grow with the model and neither is the field:
/// every build rebuilds the whole `LevelLineMesh` and its standing map from all the triangles, and
/// this bigger cylinder's rim is itself twice as long (512 sides against 256), so twice the band.
/// The ceiling comes down when the working mesh is built once per model rather than once per build.
///
/// **The caveat.** Work units cannot see UNCHARGED work, and `LevelLineMesh::from_triangles`
/// charges nothing - only `split_edge` and `split_triangle` do. So the per-build mesh rebuild is
/// invisible to this measurement: the number below understates what the model actually costs. The
/// milliseconds printed beside it are the only witness to it today.
#[test]
fn a_known_room_build_does_not_grow_with_the_model() {
    let small = tessellated_cylinder(256, 38, 10.0, 20.0);
    let big = tessellated_cylinder(512, 190, 10.0, 20.0);
    assert!(big.1.len() > 4 * small.1.len());

    let small_cost = build_cost(&small);
    let big_cost = build_cost(&big);
    let growth = big_cost.0 - small_cost.0;
    println!(
        "known-room build: {} triangles {} units / {:.0} ms, {} triangles {} units / {:.0} ms, growth {growth} units",
        small.1.len(),
        small_cost.0,
        small_cost.1,
        big.1.len(),
        big_cost.0,
        big_cost.1
    );

    // Measured 98,574 units of growth, both before the ridge-rule pruning and after it. The
    // ceiling is set a tenth above that - tighter than the 8% the old wall-clock ratio had, and it
    // does not move when the band gets cheaper. See the doc comment for why this is the growth and
    // not the ratio.
    assert!(growth < 110_000, "growth {growth}");
}

/// The work units and milliseconds of one known-room build.
fn build_cost(model: &SolidWithChain) -> (i64, f64) {
    let (positions, indices, rim) = model;
    let feature = feature_edges_over(positions, indices, FOLD_DEGREES);
    let room = band_gap_reading::room_of(
        positions,
        indices,
        rim,
        true,
        Some(Rc::clone(&feature)),
        BandProfile::Fillet,
        1e-9,
    )
    .expect("the room reads");

    // One build first, so neither row pays for the other's warm-up; its budget is thrown away.
    band_gap_entry_points::build(
        positions,
        indices,
        rim,
        true,
        Some(Rc::clone(&feature)),
        0.5,
        BandProfile::Fillet,
        &BandGapBuildOptions {
            known_room: Some(room),
            ..BandGapBuildOptions::default()
        },
    )
    .expect("a valid pick");

    let budget =
        Rc::new(WorkBudget::new(WorkBudget::default_limit(indices.len() as i32)).expect("a limit"));
    let watch = Instant::now();
    let gap = band_gap_entry_points::build(
        positions,
        indices,
        rim,
        true,
        Some(feature),
        0.5,
        BandProfile::Fillet,
        &BandGapBuildOptions {
            chord_tolerance: 0.0,
            snap: 1e-9,
            known_room: Some(room),
            budget: Some(Rc::clone(&budget)),
            ..BandGapBuildOptions::default()
        },
    )
    .expect("a valid pick");
    let elapsed = watch.elapsed().as_secs_f64() * 1000.0;
    assert_eq!(gap.refusal, BandGapRefusal::None, "the rim refused");

    (budget.spent.get(), elapsed)
}

/// The closest point of a segment to a point - the reference query's own
/// piece.
fn closest_on_segment(point: Vector3, a: Vector3, b: Vector3) -> Vector3 {
    let along = b - a;
    let length_squared = along.length_squared();
    if length_squared <= 0.0 {
        return a;
    }

    let t = ((point - a).dot(along) / length_squared).clamp(0.0, 1.0);
    a + (along * t)
}

/// The closest point of a triangle to a point, by projection onto its plane
/// then its edges.
fn closest_on_triangle(point: Vector3, a: Vector3, b: Vector3, c: Vector3) -> Vector3 {
    let mut normal = (b - a).cross(c - a);
    if normal.length_squared() > 0.0 {
        normal = normal.get_normal();
        let on_plane = point - (normal * (point - a).dot(normal));

        // Inside the triangle when it is on the same side of all three edges.
        let mut inside = true;
        let corners = [a, b, c];
        for i in 0..3 {
            let from = corners[i];
            let to = corners[(i + 1) % 3];
            if (to - from).cross(on_plane - from).dot(normal) < 0.0 {
                inside = false;
                break;
            }
        }

        if inside {
            return on_plane;
        }
    }

    let mut best = closest_on_segment(point, a, b);
    for candidate in [
        closest_on_segment(point, b, c),
        closest_on_segment(point, c, a),
    ] {
        if (candidate - point).length_squared() < (best - point).length_squared() {
            best = candidate;
        }
    }

    best
}
