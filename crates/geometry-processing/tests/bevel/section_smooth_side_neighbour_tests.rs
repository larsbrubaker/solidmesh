//! The smooth side's boundary neighbours: a closed boundary wraps round, an open one's two ends
//! have no pair of neighbours, so a straight-run reading never lifts an end out of a kept ring.

use geometry_processing::bevel::sections::section_smooth_side::SmoothSideCut;
use geometry_processing::bevel::sections::section_smooth_side_triangulate::{
    lies_on_the_segment_between_its_neighbours, the_polyline_neighbours,
};

fn cut(boundary: &[i32], closed: bool) -> SmoothSideCut {
    let mut cut = SmoothSideCut {
        boundary: boundary.to_vec(),
        boundary_is_closed: closed,
        ..Default::default()
    };
    cut.join_placed.extend(boundary.iter().copied());
    cut
}

#[test]
fn a_closed_boundary_wraps_its_neighbours_round() {
    let along = the_polyline_neighbours(&cut(&[1, 2, 3, 4], true));
    assert_eq!(along.get(&1), Some(&(4, 2)));
    assert_eq!(along.get(&4), Some(&(3, 1)));
}

#[test]
fn an_open_boundarys_ends_are_not_each_others_neighbours() {
    let open = cut(&[1, 2, 3, 4], false);
    let along = the_polyline_neighbours(&open);
    assert_eq!(along.get(&1), None);
    assert_eq!(along.get(&4), None);
    assert_eq!(along.get(&2), Some(&(1, 3)));
    assert_eq!(along.get(&3), Some(&(2, 4)));

    // A ring that runs 4 -> 1 -> 2 would read the end 1 as standing between the far end and its
    // successor if the ends wrapped; an end never lies on a segment between two neighbours.
    let ring = [4, 1, 2];
    assert!(!lies_on_the_segment_between_its_neighbours(
        &open, &along, &ring, 1
    ));
    // The same reading on an interior vertex still lifts it.
    let ring = [1, 2, 3];
    assert!(lies_on_the_segment_between_its_neighbours(
        &open, &along, &ring, 1
    ));
}
