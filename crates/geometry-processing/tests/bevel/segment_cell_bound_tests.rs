//! Ports `Tests/GeometryProcessingTests/LevelLine/SegmentCellBoundTests.cs`: the island search's
//! chain-segment cell bound is SOUND - over seeded random cells it never stands under a value the
//! field really takes inside the cell - and it bites: it stands below the Lipschitz bound often.

use std::rc::Rc;

use geometry_processing::bevel::level_line::{field_bounds, ChainField, FieldSample, PickField};
use vector_math::dotnet_random::DotNetRandom;
use vector_math::Vector3;

use super::clipped_cell_bound_tests::greatest_by_sampling;

const SQUARE: [Vector3; 6] = [
    Vector3::new(-2.0, -2.0, 0.0),
    Vector3::new(0.0, -2.0, 0.0),
    Vector3::new(2.0, -2.0, 0.0),
    Vector3::new(2.0, 2.0, 0.0),
    Vector3::new(0.0, 2.0, 0.0),
    Vector3::new(-2.0, 2.0, 0.0),
];

type Evaluate = Box<dyn Fn(Vector3) -> FieldSample>;
type Bound =
    Box<dyn Fn(&FieldSample, &FieldSample, &FieldSample, Vector3, Vector3, Vector3) -> f64>;

fn one_chain(field: Rc<ChainField>) -> (Evaluate, Bound) {
    let reader = field.clone();
    (
        Box::new(move |p| reader.evaluate(p).expect("no budget")),
        Box::new(move |s0, s1, s2, v0, v1, v2| {
            field
                .segment_cell_bound(0, s0, s1, s2, v0, v1, v2)
                .expect("no budget")
        }),
    )
}

/// The shapes the bound must hold over: a closed chain whose setback varies (its wrap included),
/// a zig-zag sharp enough to be mitred, an open chain carried on past both ends by rays, and a
/// pick of several.
fn fields_to_be_sound_over() -> Vec<(&'static str, Evaluate, Bound)> {
    let mut fields = Vec::new();

    let varying = Rc::new(
        ChainField::from_polyline(&SQUARE, true, &[1.0, 0.4142, 1.0, 0.7, 1.0, 0.4142])
            .expect("a valid chain"),
    );
    let (evaluate, bound) = one_chain(varying.clone());
    fields.push(("closed chain, varying setback", evaluate, bound));

    let zigzag = Rc::new(
        ChainField::from_polyline_sharp(
            &[
                Vector3::new(-4.0, 0.0, 0.0),
                Vector3::new(-1.0, 3.0, 0.0),
                Vector3::new(1.0, -1.0, 0.0),
                Vector3::new(4.0, 2.0, 1.0),
            ],
            false,
            &[0.8, 1.2, 0.6, 1.0],
            30.0,
        )
        .expect("a valid chain"),
    );
    let (evaluate, bound) = one_chain(zigzag);
    fields.push(("mitred vertices", evaluate, bound));

    let rays = Rc::new(
        ChainField::from_polyline_rayed(
            &[
                Vector3::new(-3.0, -1.0, 0.0),
                Vector3::new(0.0, 1.0, 0.0),
                Vector3::new(3.0, 0.0, 0.5),
            ],
            false,
            &[1.0, 0.5, 1.5],
            f64::INFINITY,
            (true, true),
        )
        .expect("a valid chain"),
    );
    let (evaluate, bound) = one_chain(rays.clone());
    fields.push(("end rays", evaluate, bound));

    let shifted_points: Vec<Vector3> = SQUARE
        .iter()
        .map(|&p| p + Vector3::new(3.0, 1.0, 0.5))
        .collect();
    let shifted = Rc::new(
        ChainField::from_polyline(&shifted_points, true, &[0.5, 1.2, 0.5, 1.2, 0.5, 1.2])
            .expect("a valid chain"),
    );
    let pick =
        Rc::new(PickField::new(vec![varying, shifted, rays], &[0, 1, 2]).expect("a valid pick"));
    let reader = pick.clone();
    fields.push((
        "a pick of three",
        Box::new(move |p| reader.evaluate(p).expect("no budget")),
        Box::new(move |s0, s1, s2, v0, v1, v2| {
            pick.segment_cell_bound(s0, s1, s2, v0, v1, v2)
                .expect("no budget")
        }),
    ));

    fields
}

#[test]
fn the_bound_stands_over_the_field() {
    let mut random = DotNetRandom::new(20260924);
    for (name, evaluate, bound) in fields_to_be_sound_over() {
        let mut biting = 0;
        for _ in 0..300 {
            let a = {
                let x = (random.next_double() * 10.0) - 5.0;
                let y = (random.next_double() * 10.0) - 5.0;
                let z = (random.next_double() * 2.0) - 1.0;
                Vector3::new(x, y, z)
            };
            let spread = if random.next_double() < 0.5 { 0.5 } else { 3.0 };
            let b = a + offset(&mut random, spread);
            let c = a + offset(&mut random, spread);

            let cell_bound = bound(&evaluate(a), &evaluate(b), &evaluate(c), a, b, c);
            let greatest = greatest_by_sampling(&|q| evaluate(q).value, a, b, c, 24);
            assert!(
                cell_bound >= greatest - 1e-9,
                "{name}: over ({a:?}), ({b:?}), ({c:?}) a bound under the field retires the cell holding the peak"
            );

            // A bound under the Lipschitz one is one that could retire a cell the search would not
            // have.
            let sa = evaluate(a);
            if cell_bound
                < field_bounds::greatest_over_triangle(
                    sa.value,
                    evaluate(b).value,
                    evaluate(c).value,
                    a,
                    b,
                    c,
                )
            {
                biting += 1;
            }
        }

        assert!(
            biting > 30,
            "{name}: a bound that never bites proves nothing sound ({biting})"
        );
    }
}

fn offset(random: &mut DotNetRandom, spread: f64) -> Vector3 {
    let x = (random.next_double() - 0.5) * spread;
    let y = (random.next_double() - 0.5) * spread;
    let z = (random.next_double() - 0.5) * spread * 0.3;
    Vector3::new(x, y, z)
}
