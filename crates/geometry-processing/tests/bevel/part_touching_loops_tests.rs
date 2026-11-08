//! Which loop of a touching pair a many-loop bevel rim leaves alone (`part::touching_loops`), on
//! loops made of bare vertex numbers - no mesh, so the rule is read in milliseconds where the
//! many-hole plate build that motivated it takes half a minute in Release.
//!
//! The answer is a set, so `equivalent` sorts both sides before comparing.

use geometry_processing::bevel::part::touching_loops::the_ones_to_leave_alone;

type Loop = Vec<(i32, i32)>;

/// Two loops nowhere near anything, which lift a pick over the many-loop floor.
fn bystanders() -> [Loop; 2] {
    [ring(&[100, 101, 102]), ring(&[200, 201, 202])]
}

/// The loops given, then the bystanders, so the touching ones keep indices 0, 1, ....
fn pick(loops: &[Loop]) -> Vec<Loop> {
    loops.iter().cloned().chain(bystanders()).collect()
}

/// A closed loop through these vertices, in order.
fn ring(vertices: &[i32]) -> Loop {
    vertices
        .iter()
        .enumerate()
        .map(|(i, &vertex)| (vertex, vertices[(i + 1) % vertices.len()]))
        .collect()
}

fn equivalent(actual: Vec<(usize, usize, i32)>, expected: &[(usize, usize, i32)]) {
    let mut actual = actual;
    let mut expected = expected.to_vec();
    actual.sort_unstable();
    expected.sort_unstable();
    assert_eq!(actual, expected);
}

#[test]
fn of_a_touching_pair_the_larger_loop_stays() {
    let big = ring(&[0, 1, 2, 3, 4, 5]);
    let small = ring(&[5, 10, 11]);

    equivalent(
        the_ones_to_leave_alone(&pick(&[big.clone(), small.clone()])),
        &[(1, 0, 5)],
    );

    // And whichever was picked first.
    equivalent(the_ones_to_leave_alone(&pick(&[small, big])), &[(0, 1, 5)]);
}

#[test]
fn on_a_tie_the_loop_picked_first_stays() {
    equivalent(
        the_ones_to_leave_alone(&pick(&[ring(&[0, 1, 2]), ring(&[2, 3, 4])])),
        &[(1, 0, 2)],
    );
}

#[test]
fn an_open_run_in_any_touch_leaves_the_route_as_it_was() {
    // A closed pair that would be answered on its own, beside an open run touching a third loop:
    // the open run's touch belongs to the older path, so nothing is taken out from under it.
    let open = vec![(20, 21), (21, 22)];

    assert!(the_ones_to_leave_alone(&pick(&[
        ring(&[0, 1, 2, 3]),
        ring(&[3, 4, 5]),
        open,
        ring(&[22, 23, 24]),
    ]))
    .is_empty());
}

#[test]
fn a_loop_touching_several_is_kept_and_each_smaller_one_left() {
    let middle = ring(&[0, 1, 2, 3, 4, 5, 6, 7]);

    equivalent(
        the_ones_to_leave_alone(&pick(&[middle, ring(&[0, 10, 11]), ring(&[4, 12, 13])])),
        &[(1, 0, 0), (2, 0, 4)],
    );

    // A run of three, largest to smallest: the middle one goes to the first, and with it gone the
    // smallest touches nothing that stays, so it is kept rather than taken for nothing.
    equivalent(
        the_ones_to_leave_alone(&pick(&[
            ring(&[0, 1, 2, 3, 4]),
            ring(&[4, 5, 6, 7]),
            ring(&[7, 8, 9]),
        ])),
        &[(1, 0, 4)],
    );
}

#[test]
fn a_loop_picked_twice_is_not_paired_with_its_own_copy() {
    let ring_loop = ring(&[0, 1, 2, 3]);

    // Alone with its copy, nothing touches.
    assert!(the_ones_to_leave_alone(&pick(&[ring_loop.clone(), ring(&[3, 2, 1, 0])])).is_empty());

    // And where the loop is the one left alone, its copy goes with it.
    equivalent(
        the_ones_to_leave_alone(&pick(&[
            ring(&[3, 10, 11, 12, 13]),
            ring_loop.clone(),
            ring_loop,
        ])),
        &[(1, 0, 3), (2, 0, 3)],
    );
}

#[test]
fn a_pick_of_fewer_than_four_loops_is_not_judged() {
    // Below the many-loop floor a pick is the owner's corner pick, not a rim.
    assert!(the_ones_to_leave_alone(
        &[ring(&[0, 1, 2]), ring(&[2, 3, 4]), ring(&[100, 101, 102]),]
    )
    .is_empty());
}
