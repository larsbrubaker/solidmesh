// Which flip the cove-to-round offer cuts next when the cut build's fill still refuses on a run
// (`cove_round_retry::the_next_flip`): the run's end nearer the refusal, where it is a flip not yet
// cut, and never an end of a closed run, whose first point is only where the pick started it.

use solidmesh::bevel::part::surface_part_builder::cove_round_retry::the_next_flip;
use solidmesh_math::Vector3;

const A: Vector3 = Vector3::new(0.0, 0.0, 0.0);
const B: Vector3 = Vector3::new(10.0, 0.0, 0.0);

/// The flips at both ends of a run, vertex 3 at A and vertex 7 at B, A listed first.
fn both_ends() -> Vec<(i32, Vector3)> {
    vec![(3, A), (7, B)]
}

/// A run cut at neither end, refused next to its far end B: only B's flip is cut, though A's is
/// listed first.
#[test]
fn a_refusal_next_to_the_far_end_cuts_only_that_flip() {
    let run = [A, Vector3::new(5.0, 0.0, 0.0), B];
    let at = Vector3::new(9.5, 0.2, 0.0);
    assert_eq!(the_next_flip(&run, false, at, &both_ends(), &[]), Some(7));
}

/// Refused next to A, the run's first end: A's flip.
#[test]
fn a_refusal_next_to_the_first_end_cuts_its_flip() {
    let run = [A, Vector3::new(5.0, 0.0, 0.0), B];
    let at = Vector3::new(0.5, 0.2, 0.0);
    assert_eq!(the_next_flip(&run, false, at, &both_ends(), &[]), Some(3));
}

/// The nearer end's flip already cut: nothing more to cut there, so no flip at all.
#[test]
fn a_nearer_end_already_cut_names_no_flip() {
    let run = [A, Vector3::new(5.0, 0.0, 0.0), B];
    let at = Vector3::new(9.5, 0.2, 0.0);
    assert_eq!(the_next_flip(&run, false, at, &both_ends(), &[7]), None);
}

/// A closed run has no ends: its first point is where the pick started it.
#[test]
fn a_closed_run_names_no_flip() {
    let run = [A, Vector3::new(5.0, 0.0, 0.0), Vector3::new(5.0, 5.0, 0.0)];
    let at = Vector3::new(0.5, 0.2, 0.0);
    assert_eq!(the_next_flip(&run, true, at, &both_ends(), &[]), None);
}
