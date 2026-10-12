//! Ports `Tests/GeometryProcessingTests/LevelLine/ConvexCellBoundTests.cs`: what `ConvexCellBound`
//! (the union-of-balls bound read from the corners' own chain points) is worth to the
//! refinement's bounded searches, and that it is sound. For any chain point c, `|q - c| - s(c)` is
//! convex and stands above the field everywhere, so over a cell it is greatest at a corner; it is
//! also never looser than the Lipschitz bound it joins. The cost rows' ceilings are work units
//! (deterministic, not clock time) and `FieldCounters` is per thread here, so C#'s
//! `[NotInParallel]` needs no nextest override. C# `[Arguments]` rows are a loop over the
//! same-named test.

use std::fmt::Write as _;
use std::rc::Rc;

use geometry_processing::bevel::level_line::band_gap_entry_points::{self, BandGapBuildOptions};
use geometry_processing::bevel::level_line::{
    bounded_maximum, convex_cell_bound, field_bounds, BandProfile, ChainField, FieldCounters,
    LevelLineMesh, PickField, RoomField, WorkBudget,
};
use sha2::{Digest, Sha256};
use vector_math::dotnet_random::DotNetRandom;
use vector_math::number_format::format_f64;
use vector_math::Vector3;

use crate::grid_hole_plate_fixture;

/// C# `Cost`: one build's cost, with the mesh digest that says the output did not move.
struct Cost {
    spent: i64,
    evaluates: i64,
    steps: i64,
    digest: String,
    meter: String,
}

/// Turns the meter off however the measure leaves (C#'s `finally`).
struct MeterOff;

impl Drop for MeterOff {
    fn drop(&mut self) {
        FieldCounters::set_meter_on(false);
    }
}

/// C# `Measure`.
fn measure(span: f64, size: f64) -> Cost {
    let (positions, indices, chains) = grid_hole_plate_fixture::pick_with_span(4, span);
    let budget = Rc::new(
        WorkBudget::new(500 * WorkBudget::default_limit(indices.len() as i32))
            .expect("a positive limit"),
    );

    FieldCounters::reset();
    FieldCounters::set_meter_on(true);
    let _off = MeterOff;
    let closed = vec![true; chains.len()];
    let built = band_gap_entry_points::build_pick(
        &positions,
        &indices,
        &chains,
        &closed,
        Some(grid_hole_plate_fixture::feature_edges(&positions, &indices)),
        size,
        BandProfile::Chamfer,
        &BandGapBuildOptions {
            chord_tolerance: 0.0,
            snap: 1e-9,
            known_room: None,
            budget: Some(Rc::clone(&budget)),
            ..BandGapBuildOptions::default()
        },
    )
    .expect("the pick builds or refuses by name");

    assert!(
        built.built(),
        "the 4 x 4 plate at span {span} refused by {:?}",
        built.refusal
    );

    let mesh = built.mesh.as_ref().expect("a built mesh").borrow();
    Cost {
        spent: budget.spent.get(),
        evaluates: FieldCounters::with(|t| t.evaluate),
        steps: FieldCounters::total_steps(),
        digest: digest(&mesh),
        meter: FieldCounters::rule_report(),
    }
}

/// C# `Digest`: every live triangle's corners to the bit (C# "R" text), SHA-256, the first 16 hex
/// digits upper case - the same reading the cost tests pin.
fn digest(mesh: &LevelLineMesh) -> String {
    let mut text = String::new();
    for t in 0..mesh.triangle_count() {
        if mesh.is_removed(t) {
            continue;
        }

        let triangle = mesh.triangles()[t as usize];
        for vertex in [triangle.a, triangle.b, triangle.c] {
            let p = mesh.position(vertex);
            let _ = write!(
                text,
                "{},{},{};",
                format_f64(p.x),
                format_f64(p.y),
                format_f64(p.z)
            );
        }

        text.push('\n');
    }

    let hash = Sha256::digest(text.as_bytes());
    let hex: String = hash.iter().map(|b| format!("{b:02X}")).collect();
    hex[..16].to_string()
}

/// C# `TheSharperBoundCostsLessAndBuildsTheSameMesh`: one build at a stated span over size must
/// cost no more than the ceiling the sharper bound measured, and must build the mesh recorded here
/// (at span 2 byte for byte the digest `ManyChainBuildCostTests` pinned BEFORE the bound existed).
#[test]
fn the_sharper_bound_costs_less_and_builds_the_same_mesh() {
    let rows: [(f64, f64, i64, &str); 2] = [
        (2.0, 0.5, 133_000, "442C79AFA13B3132"),
        (20.0, 0.5, 290_000, "12652E3B23185206"),
    ];
    for (span, size, ceiling, digest) in rows {
        let measured = measure(span, size);

        let line = format!(
            "span/size {:.0}: spent {}, evaluates {}, steps {}, mesh {}\n{}",
            span / size,
            measured.spent,
            measured.evaluates,
            measured.steps,
            measured.digest,
            measured.meter
        );
        println!("{line}");
        if let Ok(log) = std::env::var("LEVELLINE_COST_LOG") {
            if !log.is_empty() {
                use std::io::Write as _;
                if let Ok(mut file) = std::fs::OpenOptions::new()
                    .create(true)
                    .append(true)
                    .open(&log)
                {
                    let _ = writeln!(file, "{line}");
                }
            }
        }

        assert_eq!(measured.digest, digest, "span {span}");
        assert!(
            measured.spent <= ceiling,
            "span {span}: spent {} over {ceiling}",
            measured.spent
        );
    }
}

/// C# `FieldsToBeSoundOver`: one chain whose setback varies, a pick of several such chains, and
/// the same field with its setbacks scaled.
fn fields_to_be_sound_over() -> Vec<(&'static str, RoomField)> {
    let square = [
        Vector3::new(-2.0, -2.0, 0.0),
        Vector3::new(0.0, -2.0, 0.0),
        Vector3::new(2.0, -2.0, 0.0),
        Vector3::new(2.0, 2.0, 0.0),
        Vector3::new(0.0, 2.0, 0.0),
        Vector3::new(-2.0, 2.0, 0.0),
    ];
    let varying = Rc::new(
        ChainField::from_polyline(&square, true, &[1.0, 0.4142, 1.0, 0.7, 1.0, 0.4142])
            .expect("a valid polyline"),
    );
    let moved: Vec<Vector3> = square
        .iter()
        .map(|&p| p + Vector3::new(6.0, 1.0, 0.5))
        .collect();
    let second = Rc::new(
        ChainField::from_polyline(&moved, true, &[0.5, 1.2, 0.5, 1.2, 0.5, 1.2])
            .expect("a valid polyline"),
    );

    vec![
        (
            "one varying chain",
            RoomField::from_chain(Rc::clone(&varying)),
        ),
        (
            "a pick of two",
            RoomField::from_pick(Rc::new(
                PickField::new(vec![Rc::clone(&varying), second], &[0, 1]).expect("a valid pick"),
            )),
        ),
        (
            "setbacks scaled",
            RoomField::from_chain(varying)
                .with_setbacks_scaled(0.37)
                .expect("no budget in force"),
        ),
    ]
}

/// C# `TheBoundStandsOverTheFieldAndUnderTheLipschitzBound`: over seeded random cells the bound
/// never stands below a value the field really takes inside the cell, and never above the
/// Lipschitz bound the refinement used before it.
#[test]
fn the_bound_stands_over_the_field_and_under_the_lipschitz_bound() {
    let mut random = DotNetRandom::new(20260921);
    for (name, field) in fields_to_be_sound_over() {
        let evaluate = |p: Vector3| field.evaluate(p).expect("no budget in force");
        for _cell in 0..300 {
            let a = somewhere_about(&mut random);
            let b = a + spread(&mut random);
            let c = a + spread(&mut random);
            let sa = evaluate(a);
            let sb = evaluate(b);
            let sc = evaluate(c);

            let convex = convex_cell_bound::over_triangle(&sa, &sb, &sc, a, b, c, None);
            let lipschitz =
                field_bounds::greatest_over_triangle(sa.value, sb.value, sc.value, a, b, c);
            let along_convex = convex_cell_bound::over_segment(&sa, &sb, a, b, None);
            let along_lipschitz =
                field_bounds::greatest_over_triangle(sa.value, sb.value, sb.value, a, b, b);

            let mut highest = f64::NEG_INFINITY;
            let mut highest_on_edge = f64::NEG_INFINITY;
            for _sample in 0..200 {
                let u = random.next_double();
                let v = random.next_double() * (1.0 - u);
                highest = highest.max(evaluate(a + ((b - a) * u) + ((c - a) * v)).value);
                highest_on_edge =
                    highest_on_edge.max(evaluate(a + ((b - a) * random.next_double())).value);
            }

            assert!(
                convex >= highest - 1e-9,
                "{name}: a bound under the field retires the cell that holds the peak"
            );
            assert!(
                convex <= lipschitz + 1e-9,
                "{name}: the bound is taken as the lesser of the two, so it can never be looser"
            );
            assert!(along_convex >= highest_on_edge - 1e-9, "{name}");
            assert!(along_convex <= along_lipschitz + 1e-9, "{name}");
        }
    }
}

/// C# `EverySampleSaysItsValueIsItsDistanceLessItsSetback`: the value IS the distance less the
/// setback at the chain point the sample names - the bound rebuilds the value from `distance` and
/// `nearest`.
#[test]
fn every_sample_says_its_value_is_its_distance_less_its_setback() {
    let mut random = DotNetRandom::new(4919);
    for (name, field) in fields_to_be_sound_over() {
        for _trial in 0..300 {
            let q = somewhere_about(&mut random);
            let sample = field.evaluate(q).expect("no budget in force");

            assert!(
                ((q - sample.nearest).length() - sample.distance).abs() <= 1e-9,
                "{name}"
            );
            assert!(
                ((sample.distance - field.setback_of(&sample)) - sample.value).abs() <= 1e-9,
                "{name}: the bound rebuilds the value from Distance and Nearest"
            );
        }
    }
}

/// C# `SeedingASearchChangesItsCostAndNotItsAnswer`: a floor it must beat changes a search's COST
/// and not its answer.
#[test]
fn seeding_a_search_changes_its_cost_and_not_its_answer() {
    let mut random = DotNetRandom::new(773);
    let field = fields_to_be_sound_over().remove(0).1;
    const TOLERANCE: f64 = 1e-3;
    let evaluate = |p: Vector3| field.evaluate(p);

    for _cell in 0..200 {
        let a = somewhere_about(&mut random);
        let b = a + spread(&mut random);
        let c = a + spread(&mut random);

        let unseeded = bounded_maximum::over_triangle(
            &evaluate,
            a,
            b,
            c,
            TOLERANCE,
            f64::INFINITY,
            f64::NEG_INFINITY,
            None,
            None,
        )
        .expect("no budget in force");

        // The contract's own band: a floor more than a tolerance under the true maximum can never
        // prune the branch holding it, so the seeded search must answer the same number.
        let seeded = bounded_maximum::over_triangle(
            &evaluate,
            a,
            b,
            c,
            TOLERANCE,
            f64::INFINITY,
            unseeded.value - (2.0 * TOLERANCE),
            None,
            None,
        )
        .expect("no budget in force");

        assert!(
            (seeded.value - unseeded.value).abs() <= TOLERANCE,
            "a floor under the answer changes which cells are opened, not what is found"
        );
    }
}

/// C# `SomewhereAbout`: a point somewhere about the chains above - inside the band, outside it,
/// and over the gap.
fn somewhere_about(random: &mut DotNetRandom) -> Vector3 {
    Vector3::new(
        (random.next_double() * 12.0) - 4.0,
        (random.next_double() * 10.0) - 4.0,
        (random.next_double() * 4.0) - 2.0,
    )
}

/// C# `Spread`: a cell edge of a size the refinement really works at, from a hundredth up to a
/// whole chain.
fn spread(random: &mut DotNetRandom) -> Vector3 {
    let scale = 10f64.powf((random.next_double() * 2.5) - 2.0);
    Vector3::new(
        (random.next_double() - 0.5) * scale,
        (random.next_double() - 0.5) * scale,
        (random.next_double() - 0.5) * scale,
    )
}
