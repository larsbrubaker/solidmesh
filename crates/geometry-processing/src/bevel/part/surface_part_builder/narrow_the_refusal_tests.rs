// Pins of `narrow_the_refusal`'s search, asked of a yes/no question with a known answer so the
// set it lands on, the order it asks in and its attempt cap are read without a mesh. The builder
// asks the same function of real attempts (`the_loops_that_still_refuse`).

use std::cell::Cell;
use std::convert::Infallible;

use super::narrow_the_refusal::the_smallest_refusing_set;
use super::BevelSurfacePartBuilder;

/// A question that refuses wherever every one of `culprits` is in the set, saying the set's size.
fn refuses_with<'a>(
    culprits: &[usize],
    asked: &'a Cell<usize>,
) -> impl FnMut(&[usize]) -> Result<Option<usize>, Infallible> + 'a {
    let culprits = culprits.to_vec();
    move |set: &[usize]| {
        asked.set(asked.get() + 1);
        Ok(culprits
            .iter()
            .all(|culprit| set.contains(culprit))
            .then_some(set.len()))
    }
}

#[test]
fn one_corner_among_fifty_six_loops_is_narrowed_to_its_own_loop() {
    let pick: Vec<usize> = (0..56).collect();
    let asked = Cell::new(0);
    let (set, why) = the_smallest_refusing_set(
        &pick,
        56,
        BevelSurfacePartBuilder::NARROWING_ATTEMPTS_ALLOWED,
        refuses_with(&[37], &asked),
    )
    .unwrap();

    assert_eq!(set, vec![37]);
    // The reason is the one the narrowed set gave on its own.
    assert_eq!(why, 1);
    assert!(asked.get() <= BevelSurfacePartBuilder::NARROWING_ATTEMPTS_ALLOWED);
}

#[test]
fn a_corner_shared_by_loops_far_apart_in_the_pick_is_narrowed_to_that_pair() {
    // A ramp meeting a post: two loops refuse together and build apart, one in each half of the
    // pick's order, so halving alone would refuse everybody.
    let pick: Vec<usize> = (0..56).collect();
    let asked = Cell::new(0);
    let (set, why) = the_smallest_refusing_set(
        &pick,
        56,
        BevelSurfacePartBuilder::NARROWING_ATTEMPTS_ALLOWED,
        refuses_with(&[5, 44], &asked),
    )
    .unwrap();

    assert_eq!(set, vec![5, 44]);
    assert_eq!(why, 2);
    assert!(asked.get() <= BevelSurfacePartBuilder::NARROWING_ATTEMPTS_ALLOWED);
}

#[test]
fn a_refusal_that_needs_every_loop_stays_the_whole_sets() {
    let pick: Vec<usize> = (0..4).collect();
    let asked = Cell::new(0);
    let (set, why) =
        the_smallest_refusing_set(&pick, 4, 100, refuses_with(&[0, 1, 2, 3], &asked)).unwrap();

    assert_eq!(set, pick);
    assert_eq!(why, 4);
}

#[test]
fn the_attempt_cap_stops_the_search_on_a_set_that_really_refused() {
    let pick: Vec<usize> = (0..56).collect();
    let asked = Cell::new(0);
    let (set, why) =
        the_smallest_refusing_set(&pick, 56, 3, refuses_with(&[5, 44], &asked)).unwrap();

    assert_eq!(asked.get(), 3);
    assert!(set.contains(&5) && set.contains(&44));
    assert!(
        set.len() < pick.len(),
        "three attempts already narrowed the set"
    );
    assert_eq!(why, set.len());
}

#[test]
fn the_search_asks_in_the_same_order_every_time() {
    let pick: Vec<usize> = (0..20).collect();
    let mut first_run = Vec::new();
    let mut second_run = Vec::new();

    for log in [&mut first_run, &mut second_run] {
        the_smallest_refusing_set(&pick, 0, 24, |set: &[usize]| -> Result<_, Infallible> {
            log.push(set.to_vec());
            Ok((set.contains(&3) && set.contains(&17)).then_some(0))
        })
        .unwrap();
    }

    assert_eq!(first_run, second_run);
    // The first question is the first half, in the pick's own order.
    assert_eq!(first_run[0], (0..10).collect::<Vec<_>>());
}
