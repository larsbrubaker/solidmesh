//! A band
//! whose end stands over a run-out's crossing sews its last chord to the crossing's FOOT, and the
//! stretch of the section's rim between that foot and the rail beside it is a chord like any other:
//! the end face's own triangulation crosses it, and the end cut leaves a vertex where it does. The
//! strip has to carry that vertex as well as the foot, or it sews the run whole while the face beyond
//! carries it split - a T junction on the seam, and an open cap
//! (`SurfaceEndCutAboveTheCrossing.TheBetweens`).
//!
//! The numbers are the ones measured on "Every Chamfer Works.mcx" at r = 3.756. They
//! are checked as exact equality, which holds the order the strip sews in too.

use solidmesh::bevel::surface::surface_end_cut_above_the_crossing::{the_betweens, THE_FOOT};

/// The crossing's foot, as a vertex of the pool.
const THE_FOOT_VERTEX: i32 = 245;

/// The vertex the end cut left standing on the rim between rail two and the foot.
const ON_THE_WAY_TO_THE_FOOT: i32 = 608;

/// The crossing took the HIGH rails, so the rim ends at the foot and the chord into it is the last
/// one the strip sews: what stands along it comes before the foot.
#[test]
fn the_chord_into_the_foot_carries_what_stands_on_the_rim_before_it() {
    // Rails zero to two stand on the rim, then the foot; the crossing took rails three to eight.
    let rail_of = [0, 1, 2, THE_FOOT];

    let along_the_profile = vec![vec![], vec![], vec![ON_THE_WAY_TO_THE_FOOT]];

    let betweens = the_betweens(
        9,
        Some(&along_the_profile),
        &rail_of,
        THE_FOOT_VERTEX,
        -1,
        3,
        8,
    );

    assert_eq!(
        betweens[2],
        vec![ON_THE_WAY_TO_THE_FOOT, THE_FOOT_VERTEX],
        "the chord from rail two runs through what the cut left on the rim and then into the foot"
    );
}

/// And the mirror: the crossing took the LOW rails, so the rim starts at the foot and what stands
/// along that first chord comes AFTER it, in the order the strip's chord runs.
#[test]
fn the_chord_out_of_the_foot_carries_what_stands_on_the_rim_after_it() {
    // The crossing took rails zero and one, so the rim is the foot and then rails two and three.
    let rail_of = [THE_FOOT, 2, 3];

    let along_the_profile = vec![vec![ON_THE_WAY_TO_THE_FOOT], vec![]];

    let betweens = the_betweens(
        4,
        Some(&along_the_profile),
        &rail_of,
        THE_FOOT_VERTEX,
        -1,
        0,
        1,
    );

    assert_eq!(
        betweens[1],
        vec![THE_FOOT_VERTEX, ON_THE_WAY_TO_THE_FOOT],
        "the chord into rail two leaves the foot and runs through what the cut left after it"
    );
}
