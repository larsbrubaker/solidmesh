//! Port of `SectionTaperedBarrelTests.cs`: many holes at a workable size - the two-bore tapered
//! barrel ([`crate::tapered_barrel_test_meshes`]), its wall picked as a face group. Every reading
//! is taken on the working part, the form the app hands the kernel; the raw boolean appears once,
//! as the row this fixture's size is stated against. The rows that plan, route or build the wall
//! need the chain join, the section routing and the surface builder, and land with them.

use geometry_processing::bevel::bevel_chain_linker::link;
use geometry_processing::topology::FeatureEdgeChain;

use crate::tapered_barrel_test_meshes;

/// THE FIXTURE AND THE PICK: a barrel whose wall, picked as a face group, resolves to 220 edges in
/// six closed loops - four bore mouths and the two end rims.
#[test]
fn the_wall_pick_is_six_closed_loops() {
    let raw = tapered_barrel_test_meshes::the_part();

    // The RAW boolean's own row, kept because it is the source this fixture's size claim is made
    // against - the conditioned part below is what every other reading here is taken on. With Pi
    // exact the drills are exactly level, so the bore rings land ON the wall's folds and the boolean
    // leaves no doubled fold vertices.
    assert_eq!(raw.vertices.len(), 222);
    assert_eq!(raw.faces.len(), 448);

    let part = tapered_barrel_test_meshes::the_working_part();

    // Nothing for the conditioning to take off: a level drill leaves no doubled fold vertices or
    // slivers.
    assert_eq!(part.vertices.len(), 222);
    assert_eq!(part.faces.len(), 448);

    let resolution = tapered_barrel_test_meshes::the_wall(&part);

    assert!(
        resolution.is_resolved(),
        "{:?}",
        resolution.failure_reason.as_ref().map(|why| why.code())
    );

    assert_eq!(resolution.edges.len(), 220);

    let loops = link(&resolution.edges);

    assert_eq!(loops.len(), 6);
    // There is no open run in this pick - four mouths and two end rims.
    assert_eq!(loops.iter().filter(|chain| chain.is_closed).count(), 6);

    // The two 48s are the end rims, one edge per facet; the four mouths are a 24-gon drill's ring
    // re-cut by the wall's own folds - fewer cuts than under float Pi's tilt (36, 36, 39, 39),
    // because a level ring's vertices land ON folds instead of beside them.
    let mut counts: Vec<usize> = loops.iter().map(|chain| chain.edges.len()).collect();
    counts.sort_unstable();
    assert_eq!(counts, vec![30, 30, 32, 32, 48, 48]);
}

/// WHY THE PER-MOUTH READING IS NOT HERE YET: tangent propagation cannot name a mouth. Seeded on a
/// mouth edge it dies at the wall's first fold.
#[test]
fn a_mouth_cannot_be_named_by_tangent_propagation() {
    let part = tapered_barrel_test_meshes::the_working_part();
    let resolution = tapered_barrel_test_meshes::the_wall(&part);
    // C#'s `OrderBy(..).First()`: a stable sort, so the FIRST smallest closed loop.
    let mouth = link(&resolution.edges)
        .into_iter()
        .filter(|chain| chain.is_closed)
        .min_by_key(|chain| chain.edges.len())
        .expect("the wall has closed loops");

    let propagated = FeatureEdgeChain::propagate(&resolution.features, mouth.edges[0].edge_index)
        .expect("a mouth edge seeds a chain");

    // The smallest mouth: 30 edges with the drill exactly level (36 under float Pi's tilt).
    assert_eq!(mouth.edges.len(), 30);

    // One edge, not two: with the drill level the mouth's first edge already ends ON a fold - the
    // tangent run dies at the wall's first 7.5 degree fold, so SectionBuilder's chain entry point
    // cannot be handed this mouth.
    assert_eq!(propagated.edges().len(), 1);

    assert!(!propagated.is_closed());
}
