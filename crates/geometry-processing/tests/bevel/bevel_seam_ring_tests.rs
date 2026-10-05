//! Ports `GeometryProcessingTests/BevelSeamRingTests.cs`: the seam a corner patch and the runs that
//! end on it meet along, which both solids have to carry as the SAME points, so the test is EXACT
//! equality rather than a tolerance. Ported: `TheSnapTakesTheWholeSeamOrLeavesTheRingAlone`. The
//! two wired tests (`ThePatchAndItsRunsCarryTheSamePointsOnEveryBallCentrePlane`,
//! `TheFillPatchAndItsBeadsCarryTheSamePointsOnTheirBallCentrePlane`) need `BevelCornerPatches`
//! and `BevelGroupTools.TryBuild` from the cyclic core and land with it.

use geometry_processing::bevel::bevel_seam_ring::{points, snap};
use vector_math::Vector3;

const RADIUS: f64 = 3.4;

/// The snap's own contract, which the wired tests read only through its consequences: it takes the
/// WHOLE ring or none of it, and what it takes it takes literally.
#[test]
fn the_snap_takes_the_whole_seam_or_leaves_the_ring_alone() {
    let centre = Vector3::new(3.0, -4.0, 5.0);
    let seam = points(
        centre,
        RADIUS,
        Vector3::new(1.0, 0.0, 0.0),
        Vector3::new(0.0, 0.0, 1.0),
        4,
    );

    // A ring as a sweep builds one: the arc, a rounding away, with the tool's clearance points
    // around it. The order is deliberately not the seam's - which is the whole reason the match is
    // by nearness rather than by index.
    let mut ring = vec![Vector3::ZERO; seam.len() + 2];

    ring[0] = centre + Vector3::new(9.0, 9.0, 9.0);

    for i in 0..seam.len() {
        ring[seam.len() - i] = seam[i] + Vector3::new(1e-9, -1e-9, 1e-9);
    }

    let last = ring.len() - 1;
    ring[last] = centre + Vector3::new(-9.0, -9.0, -9.0);

    let mut carried = ring.clone();

    assert!(
        snap(&mut carried, &seam, RADIUS),
        "a ring that presents the whole arc, however it is ordered, carries the seam"
    );

    for i in 0..seam.len() {
        assert_eq!(
            carried[seam.len() - i],
            seam[i],
            "and it carries it LITERALLY - the published point, not the ring's own near miss"
        );
    }

    assert_eq!(carried[0], ring[0], "nothing that is not the seam is moved");

    // One point of the arc genuinely somewhere else. A ten-thousandth of the radius is a hundredth
    // of a chord and a thousand times any rounding - small enough that the tolerance this file
    // used to carry (a thousandth of the radius) would have swallowed it and moved the ring, which
    // is the thing that must not happen quietly.
    let mut wrong = ring.clone();

    wrong[2] += Vector3::new(1e-4 * RADIUS, 0.0, 0.0);

    let before = wrong.clone();

    assert!(
        !snap(&mut wrong, &seam, RADIUS),
        "a ring missing one of the seam's points is a different ring, and half a seam is worse than \
         none - the two solids would then meet along part of one polyline and part of another"
    );

    for i in 0..wrong.len() {
        assert_eq!(
            wrong[i], before[i],
            "and a refused snap leaves the ring exactly as the sweep made it"
        );
    }
}
