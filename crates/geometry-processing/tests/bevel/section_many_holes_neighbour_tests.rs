//! ONE MOUTH OF "Many Holes" AND ITS NEIGHBOUR, read
//! off the same fixture the loop table does ([`crate::many_holes_test_mesh`]) but asked a
//! different question: not what the loop answers, but whether what it emits leaves the mouth
//! BESIDE it whole.
//!
//! **Why the row exists.** The same loop, measured on an unmerged branch that moved the section
//! boundaries onto the canal, assembled with 66 OPEN EDGES, every one on the rim of the
//! NEIGHBOURING mouth: the smooth side's kept region had been drawn without that neighbour's hole
//! ring. The invariant: an UNTOUCHED hole ring of the facet is a hole of whatever region encloses
//! it - either the region is drawn WITH it or the side refuses by name. A kept region that
//! swallowed an untouched ring and was emitted anyway is the one outcome barred here.

use std::collections::HashSet;

use geometry_processing::topology::bevel_edge_group_resolver;

use crate::many_holes_rim_reading::{self, LoopAnswer};
use crate::many_holes_test_mesh;

/// The loop this file is about, by the loop table's own name.
const THE_LOOP: &str = "n64@(-111.2,97.5,-74.9)";

/// Its mirror twin, the loop that stayed sound where the one above went open.
const THE_TWIN: &str = "n64@(-111.2,125.5,-74.9)";

/// THE MOUTH AT (-111.2, 97.5, -74.9), BUILT AND ASSEMBLED ALONE AT 0.5, IS NEVER UNSOUND: it is
/// either closed and manifold and one shell, or it is REFUSED BY NAME. On main it is SOUND - the
/// room reading 1.052 for the 0.5 the file asks - so the claim is pinned at its strong end.
#[test]
fn the_mouth_by_its_neighbour_is_sound_or_refused_by_name() {
    let answer = the_answer_for(THE_LOOP);

    assert_ne!(
        answer.outcome(),
        "unsound",
        "open {}, non manifold {}, shells {} - an emitted mesh leaving the neighbouring mouth's rim \
         open is the one answer this row bars",
        answer.open_edges,
        answer.non_manifold_edges,
        answer.shells
    );

    assert_eq!(
        answer.outcome(),
        "sound",
        "on main this loop takes its fillet whole: {}",
        many_holes_rim_reading::row(&answer)
    );

    assert_eq!(answer.open_edges, 0);
    assert_eq!(answer.non_manifold_edges, 0);
    assert_eq!(answer.shells, 1);
}

/// THE MIRROR TWIN at (-111.2, 125.5, -74.9) is sound too, so a failure of the row above can be
/// told from a failure of the fixture.
#[test]
fn the_mirror_twin_is_sound_too() {
    let answer = the_answer_for(THE_TWIN);

    assert_eq!(
        answer.outcome(),
        "sound",
        "{}",
        many_holes_rim_reading::row(&answer)
    );
}

fn the_answer_for(name: &str) -> LoopAnswer {
    let part = many_holes_test_mesh::the_part();

    let (resolutions, _) =
        bevel_edge_group_resolver::resolve_all(&[many_holes_test_mesh::the_outer_wall()], &part);
    let resolution = &resolutions[0];

    // The loop table's index for a loop named by its edge count and centre.
    let only: HashSet<usize> = many_holes_rim_reading::the_loops_in_order(&part, resolution)
        .iter()
        .enumerate()
        .filter(|(_, one)| many_holes_rim_reading::name_of(&part, one) == name)
        .map(|(i, _)| i)
        .collect();

    let mut read = many_holes_rim_reading::read_every_loop(
        &part,
        resolution,
        many_holes_test_mesh::SIZE,
        many_holes_test_mesh::SEGMENTS,
        None,
        Some(&only),
    );

    assert_eq!(read.len(), 1, "the table has exactly one loop named {name}");

    read.remove(0)
}
