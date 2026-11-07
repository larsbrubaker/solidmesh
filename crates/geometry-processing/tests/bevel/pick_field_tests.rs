//! One field over a GROUP of chains. The pick's value is the least of its members', it is still
//! exactly 1-Lipschitz, a tie between chains is settled by a RULE, and a pick of ONE answers what
//! that one chain answers to the bit.

use std::rc::Rc;

use geometry_processing::bevel::level_line::{ChainField, PickField, ScalarField};
use vector_math::seeded_random::SeededRandom;
use vector_math::Vector3;

/// A three-point open chain in a box round `centre`, with varying setbacks.
fn random_chain(random: &mut SeededRandom, centre: Vector3) -> Rc<ChainField> {
    let mut points = Vec::new();
    let mut setbacks = Vec::new();
    for _ in 0..4 {
        let x = (random.next_double() - 0.5) * 6.0;
        let y = (random.next_double() - 0.5) * 6.0;
        let z = (random.next_double() - 0.5) * 6.0;
        points.push(centre + Vector3::new(x, y, z));
        setbacks.push(0.5 + random.next_double());
    }

    Rc::new(ChainField::from_polyline(&points, false, &setbacks).expect("a valid chain"))
}

fn random_point(random: &mut SeededRandom, half: f64) -> Vector3 {
    let x = (random.next_double() - 0.5) * 2.0 * half;
    let y = (random.next_double() - 0.5) * 2.0 * half;
    let z = (random.next_double() - 0.5) * 2.0 * half;
    Vector3::new(x, y, z)
}

/// Three chains far enough apart that each owns a region, near enough that their fields meet.
fn three_chains(random: &mut SeededRandom) -> Vec<Rc<ChainField>> {
    vec![
        random_chain(random, Vector3::new(-5.0, 0.0, 0.0)),
        random_chain(random, Vector3::new(0.0, 5.0, 0.0)),
        random_chain(random, Vector3::new(5.0, 0.0, 0.0)),
    ]
}

fn pick(chains: &[Rc<ChainField>], members: &[i32]) -> PickField {
    PickField::new(chains.to_vec(), members).expect("a valid pick")
}

/// The answer the pick MUST give, read the slow way: every member chain searched exhaustively
/// (`ChainField::evaluate_exhaustive`), the least taken, a tie going to the lower chain index.
/// Returns (value, chain, parameter, runner-up).
fn brute_force(chains: &[Rc<ChainField>], members: &[i32], point: Vector3) -> (f64, i32, f64, f64) {
    let mut best_value = f64::INFINITY;
    let mut runner_up = f64::INFINITY;
    let mut best_chain = -1;
    let mut best_parameter = 0.0;
    for &k in members {
        let sample = chains[k as usize].evaluate_exhaustive(point);
        if sample.value < best_value {
            runner_up = best_value;
            best_value = sample.value;
            best_chain = k;
            best_parameter = sample.parameter;
        } else {
            runner_up = vector_math::ieee_math::min_f64(runner_up, sample.value);
        }
    }

    (best_value, best_chain, best_parameter, runner_up)
}

fn value(field: &dyn ScalarField, point: Vector3) -> f64 {
    field.value_at(point).expect("no budget")
}

#[test]
fn value_chain_and_local_parameter_agree_with_brute_force() {
    let mut random = SeededRandom::new(20260919);
    let chains = three_chains(&mut random);
    let members = [0, 1, 2];
    let field = pick(&chains, &members);

    for _ in 0..2000 {
        let point = random_point(&mut random, 12.0);
        let (expected_value, expected_chain, expected_parameter, runner) =
            brute_force(&chains, &members, point);
        let sample = field.evaluate(point).expect("no budget");

        assert!(
            (sample.value - expected_value).abs() <= 1e-12,
            "the pick's value is the least of its members'"
        );

        // Which chain (and so which local parameter) is only settled where the winner wins
        // CLEARLY: nearer than that the two answers tie and the tie rule decides.
        if runner - expected_value > 1e-7 {
            assert_eq!(
                sample.chain, expected_chain,
                "the minimising chain is named"
            );
            assert!(
                (sample.parameter - expected_parameter).abs() <= 1e-9,
                "the parameter is local to the chain that won"
            );
            assert!(sample.parameter >= 0.0);
            assert!(
                sample.parameter <= chains[sample.chain as usize].length + 1e-9,
                "a local parameter never runs past its own chain"
            );
        }
    }
}

#[test]
fn the_field_is_one_lipschitz() {
    let mut random = SeededRandom::new(4242);
    let field = pick(&three_chains(&mut random), &[0, 1, 2]);

    for _ in 0..2000 {
        let p = random_point(&mut random, 12.0);
        let q = random_point(&mut random, 12.0);
        let apart = (q - p).length();

        assert!(
            (value(&field, q) - value(&field, p)).abs() <= apart + 1e-9,
            "a minimum of 1-Lipschitz fields is 1-Lipschitz, which every bounded search leans on"
        );
    }
}

#[test]
fn a_tie_between_chains_goes_to_the_lower_index() {
    // Two parallel chains either side of the X axis, the same setback: every point of the axis is
    // exactly as far from one as from the other.
    let lower = Rc::new(
        ChainField::from_polyline(
            &[Vector3::new(0.0, 1.0, 0.0), Vector3::new(10.0, 1.0, 0.0)],
            false,
            &[1.0, 1.0],
        )
        .expect("a valid chain"),
    );
    let upper = Rc::new(
        ChainField::from_polyline(
            &[Vector3::new(0.0, -1.0, 0.0), Vector3::new(10.0, -1.0, 0.0)],
            false,
            &[1.0, 1.0],
        )
        .expect("a valid chain"),
    );

    let as_written = pick(&[lower.clone(), upper.clone()], &[0, 1]);
    let reversed = pick(&[upper, lower], &[0, 1]);

    for i in 1..10 {
        let on_the_midline = Vector3::new(i as f64, 0.0, 0.0);

        assert_eq!(
            as_written
                .evaluate(on_the_midline)
                .expect("no budget")
                .chain,
            0,
            "a tie between chains goes to the lower chain index, whichever chain that is"
        );
        assert_eq!(
            reversed.evaluate(on_the_midline).expect("no budget").chain,
            0,
            "the rule is the index, not the geometry - the two orders disagree about which chain wins"
        );
    }
}

#[test]
fn exact_minima_over_a_piece_are_the_least_of_the_members() {
    let mut random = SeededRandom::new(777);
    let chains = three_chains(&mut random);
    let field = pick(&chains, &[0, 1, 2]);

    for _ in 0..60 {
        let a = random_point(&mut random, 8.0);
        let b = random_point(&mut random, 8.0);
        let c = random_point(&mut random, 8.0);

        // A minimum over the piece of a minimum over the chains is the same number either way
        // round, so the pick's exact minimum must be ATTAINED - no sample of the piece may beat it.
        let along_segment = field.min_along_segment(a, b).expect("no budget");
        let over_triangle = field.min_over_triangle(a, b, c).expect("no budget");
        let peak = field.max_over_triangle(a, b, c, 1e-4).expect("no budget");

        let mut sampled_low = f64::INFINITY;
        let mut sampled_high = f64::NEG_INFINITY;
        const STEPS: i32 = 40;
        for i in 0..=STEPS {
            sampled_low = sampled_low.min(value(&field, a + ((b - a) * (i as f64 / STEPS as f64))));
            for j in 0..=(STEPS - i) {
                let u = i as f64 / STEPS as f64;
                let v = j as f64 / STEPS as f64;
                let at = value(&field, a + ((b - a) * u) + ((c - a) * v));
                sampled_high = sampled_high.max(at);
            }
        }

        assert!(
            along_segment.value <= sampled_low + 1e-9,
            "the exact least over the segment cannot be beaten by a sample of it"
        );
        assert!(
            (value(&field, along_segment.point) - along_segment.value).abs() <= 1e-7,
            "the least is attained at the point reported"
        );
        assert!(
            over_triangle.value <= along_segment.value + 1e-9,
            "the triangle holds the segment"
        );
        assert!(
            peak.value >= sampled_high - 1e-9,
            "the bounded greatest-value search finds a peak no sample beats"
        );
    }
}

#[test]
fn a_pick_of_one_answers_its_chain_to_the_bit() {
    let mut random = SeededRandom::new(31415);
    for _ in 0..200 {
        let chain = random_chain(&mut random, Vector3::ZERO);
        let field = PickField::of(chain.clone());

        for _ in 0..10 {
            let point = random_point(&mut random, 8.0);
            let mine = field.evaluate(point).expect("no budget");
            let theirs = chain.evaluate(point).expect("no budget");

            assert_eq!(mine.value, theirs.value);
            assert_eq!(mine.distance, theirs.distance);
            assert_eq!(mine.parameter, theirs.parameter);
            assert_eq!(mine.nearest.x, theirs.nearest.x);
            assert_eq!(mine.nearest.y, theirs.nearest.y);
            assert_eq!(mine.nearest.z, theirs.nearest.z);
            assert_eq!(
                mine.chain, 0,
                "the single chain of a pick of one is chain 0"
            );
            assert_eq!(value(&field, point), value(chain.as_ref(), point));
            assert_eq!(field.setback_of(&mine), chain.setback_at(theirs.parameter));
        }

        let a = random_point(&mut random, 8.0);
        let b = random_point(&mut random, 8.0);
        let c = random_point(&mut random, 8.0);

        assert_eq!(
            field.min_along_segment(a, b).expect("no budget").value,
            chain.min_along_segment(a, b).expect("no budget").value
        );
        assert_eq!(
            field.min_over_triangle(a, b, c).expect("no budget").value,
            chain.min_over_triangle(a, b, c).expect("no budget").value
        );
        assert_eq!(
            field
                .max_along_segment(a, b, 1e-3)
                .expect("no budget")
                .value,
            chain
                .max_along_segment(a, b, 1e-3)
                .expect("no budget")
                .value
        );
        assert_eq!(
            field
                .max_over_triangle(a, b, c, 1e-3)
                .expect("no budget")
                .value,
            chain
                .max_over_triangle(a, b, c, 1e-3)
                .expect("no budget")
                .value
        );
        assert_eq!(
            field.nothing_reaches_below(a, b, 0.0),
            chain.nothing_reaches_below(a, b, 0.0)
        );
        assert_eq!(field.min_setback, chain.min_setback);
        assert_eq!(field.max_setback, chain.max_setback);
    }
}

#[test]
fn value_and_chain_are_invariant_to_a_rigid_motion() {
    let mut random = SeededRandom::new(90210);
    let chains = three_chains(&mut random);
    let members = [0, 1, 2];
    let here = pick(&chains, &members);

    // A turn about Z by a third of a radian and a shove, applied to the chains and to the query.
    let angle: f64 = 0.3333333333333333;
    let shove = Vector3::new(13.5, -4.25, 7.125);
    let move_it = |v: Vector3| {
        Vector3::new(
            (v.x * angle.cos()) - (v.y * angle.sin()),
            (v.x * angle.sin()) + (v.y * angle.cos()),
            v.z,
        ) + shove
    };

    let moved: Vec<Rc<ChainField>> = chains
        .iter()
        .map(|chain| {
            let points: Vec<Vector3> = chain.points().iter().map(|&p| move_it(p)).collect();
            Rc::new(
                ChainField::from_polyline(&points, chain.is_closed, &setbacks_of(chain))
                    .expect("a valid chain"),
            )
        })
        .collect();

    let there = pick(&moved, &members);

    for _ in 0..500 {
        let point = random_point(&mut random, 12.0);
        let a = here.evaluate(point).expect("no budget");
        let b = there.evaluate(move_it(point)).expect("no budget");

        assert!(
            (b.value - a.value).abs() <= 1e-9,
            "the same cut wherever the part stands"
        );
        assert_eq!(
            b.chain, a.chain,
            "which chain owns a point is a property of the part, not of its placement"
        );
    }
}

/// The setbacks a chain was built from, read back one chain point at a time.
fn setbacks_of(chain: &ChainField) -> Vec<f64> {
    let points = chain.points();
    let mut setbacks = Vec::new();
    let mut parameter = 0.0;
    for i in 0..points.len() {
        setbacks.push(chain.setback_at(parameter));
        if i + 1 < points.len() {
            parameter += (points[i + 1] - points[i]).length();
        }
    }

    setbacks
}

#[test]
fn a_group_ignores_every_chain_outside_it() {
    let mut random = SeededRandom::new(2468);
    let chains = three_chains(&mut random);
    let subset = [0, 2];
    let group = pick(&chains, &subset);

    for _ in 0..1000 {
        let point = random_point(&mut random, 12.0);
        let (expected_value, _, _, _) = brute_force(&chains, &subset, point);
        let sample = group.evaluate(point).expect("no budget");

        assert_ne!(
            sample.chain, 1,
            "a chain outside the group is not in the group's field at all"
        );
        assert!(
            (sample.value - expected_value).abs() <= 1e-12,
            "the group's value is the least over ITS chains only"
        );
    }

    // And the point is that this is a DIFFERENT field from the whole pick's: somewhere the
    // left-out chain would have won.
    let every_chain = pick(&chains, &[0, 1, 2]);
    let mut differs_somewhere = false;
    let mut i = 0;
    while i < 1000 && !differs_somewhere {
        let point = random_point(&mut random, 12.0);
        differs_somewhere = (value(&every_chain, point) - value(&group, point)).abs() > 1e-6;
        i += 1;
    }

    assert!(
        differs_somewhere,
        "chain 1 owns some of space, so leaving it out has to change the field"
    );
}
