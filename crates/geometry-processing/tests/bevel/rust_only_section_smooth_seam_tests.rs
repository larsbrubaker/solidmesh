//! Rust-only tests of `section_smooth_seam`, which the Many Holes and open-mouth arc cut section
//! tests reach only in context: the seam reading on [`crate::two_mouths_in_one_facet_mesh`]'s lower
//! mouth, whose smooth side `SectionSmoothSideBridgedHolesTests` cuts directly.

use std::rc::Rc;

use geometry_processing::bevel::sections::section_builder_run::try_build_loop;
use geometry_processing::bevel::sections::section_fan_walk::edges_by_vertex_pair;
use geometry_processing::bevel::sections::section_smooth_seam::{
    a_walk_crossed_a_fold, side_is_smooth, try_cut_every_smooth_side, SmoothSeamFailure,
};
use geometry_processing::bevel::sections::{SectionPool, SectionRun};
use geometry_processing::bevel::{BevelAdvisory, BevelAdvisoryCode};
use indexmap::IndexMap;
use polygon_mesh::Mesh;

use crate::two_mouths_in_one_facet_mesh as fixture;

/// The mouth's walks fold on the tessellated wall (its LEFT) and nowhere on the bore's side, so the
/// left alone reads smooth, and the assembly's cut takes that one side - the same three facets
/// `SectionSmoothSideBridgedHolesTests` cuts when asked directly.
#[test]
fn rust_only_the_mouth_is_cut_on_the_one_side_that_folds() {
    let part = fixture::the_working_part();
    let (run, mut pool) = the_lower_mouth(&part);
    let graph = part.get_mesh_edge_graph();
    let edge_at_pair = edges_by_vertex_pair(&graph);
    let mut regions = IndexMap::new();

    assert!(a_walk_crossed_a_fold(&run, true), "the wall side folds");
    assert!(
        !a_walk_crossed_a_fold(&run, false),
        "the other side does not"
    );
    assert!(side_is_smooth(
        &part,
        &graph,
        &edge_at_pair,
        &run,
        true,
        &mut regions
    ));
    assert!(!side_is_smooth(
        &part,
        &graph,
        &edge_at_pair,
        &run,
        false,
        &mut regions
    ));

    let runs = vec![run];
    let seams = try_cut_every_smooth_side(&part, &graph, &mut pool, &runs)
        .unwrap_or_else(|failure| panic!("the one smooth side cuts: {failure:?}"));

    assert_eq!(seams.len(), 1, "one run, one smooth side");
    assert_eq!(seams[0].run, 0, "the seam names its run by its index");
    assert!(seams[0].is_left);
    assert_eq!(seams[0].cut.facets.len(), 3);
}

/// A run whose walks fold on BOTH sides is refused by name, at its first station, with the refusal
/// carried as the advisory's second argument (the refusal's text).
#[test]
fn rust_only_a_run_smooth_on_both_sides_is_refused_by_name() {
    let part = fixture::the_working_part();
    let (mut run, mut pool) = the_lower_mouth(&part);
    let graph = part.get_mesh_edge_graph();

    let mut folded = (*run.sections[0]).clone();
    folded.right_walk_crossed_a_fold = true;
    run.sections[0] = Rc::new(folded);

    let station = run.sections[0].station;
    let runs = vec![run];

    assert_eq!(
        try_cut_every_smooth_side(&part, &graph, &mut pool, &runs).err(),
        Some(SmoothSeamFailure::Refused(BevelAdvisory::new(
            BevelAdvisoryCode::SurfaceSmoothSideNotCut,
            vec![station.into(), "BothSidesAreSmooth".into()],
        )))
    );
}

/// The lower of the two mouths, built as a closed run of sections.
fn the_lower_mouth(part: &Mesh) -> (SectionRun, SectionPool) {
    let resolution = fixture::the_wall(part);
    let graph = part.get_mesh_edge_graph();
    let mouths = fixture::the_mouths(part, &resolution);
    let ring: Vec<i32> = mouths[0]
        .edges
        .iter()
        .map(|edge| edge.from_vertex())
        .collect();
    let mut pool = SectionPool::new(&part.vertices);

    let run = try_build_loop(
        &ring,
        Some(&resolution.features),
        part,
        &graph,
        &mut pool,
        fixture::SIZE,
        fixture::SEGMENTS,
        true,
    )
    .unwrap_or_else(|why| panic!("the mouth builds its run: {why:?}"));

    (run, pool)
}
