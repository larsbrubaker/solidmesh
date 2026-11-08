//! A FACET WITH HOLES IN IT, CUT.
//! [`crate::two_mouths_in_one_facet_mesh`] is Many Holes' refused facet at a size that can be worked
//! on - two bore mouths standing wholly inside one flat piece of a tessellated wall - and the smooth
//! side's cut has to take that piece, which is a disc with holes rather than a disc.

use std::collections::HashSet;

use solidmesh::bevel::sections::section_builder_run::try_build_loop;
use solidmesh::bevel::sections::section_fan_walk::edges_by_vertex_pair;
use solidmesh::bevel::sections::section_smooth_side::{
    try_cut_the_smooth_side, SmoothSideCut, SmoothSideRefusal,
};
use solidmesh::bevel::sections::section_smooth_side_boundary::SmoothSideFailure;
use solidmesh::bevel::sections::{SectionPool, SectionSide};

use crate::two_mouths_in_one_facet_mesh as fixture;

/// The fixture lands the mouth's smooth side on a flat piece with a WHOLE second bore mouth inside
/// it, and the cut takes that piece from its rings AS THEY STAND, so the outline reading no longer
/// stops it. The one facet is 100 faces with outline rings 47/4/47; the cut's two pieces both end on
/// the OUTER quad, the two mouths are untouched rings, and each goes to the region enclosing it.
#[test]
fn the_piece_with_a_hole_in_it_is_cut_from_its_rings_as_they_stand() {
    let read = the_lower_mouth();

    println!(
        "lower mouth: refusal {:?}, rings {:?}, facets cut {:?}",
        read.refusal,
        read.rings,
        read.cut.as_ref().map(|cut| cut.facets.len())
    );

    assert_ne!(
        read.refusal,
        SmoothSideRefusal::TheOutlineIsNotOneRing,
        "the kept region is the outer ring, the boundary loop and the untouched mouth"
    );
    assert!(
        read.rings.is_empty(),
        "nothing is left for the outline refusal to report"
    );
}

/// THE DELIVERABLE: the mouth cuts. Each untouched ring of the piece goes to the region enclosing it
/// and a kept region with holes is spliced back to one ring, so the ears read one ring and the split
/// and the siding go as they do on a facet with no hole.
#[test]
fn the_mouth_in_a_piece_with_holes_cuts() {
    let read = the_lower_mouth();

    assert_eq!(
        read.refusal,
        SmoothSideRefusal::None,
        "a facet with holes is cut and its kept region spliced, refused {:?} at {}",
        read.refusal,
        read.where_
    );
}

/// The lower of the two mouths, taken as a chain and its smooth side cut.
fn the_lower_mouth() -> Reading {
    let part = fixture::the_working_part();
    let resolution = fixture::the_wall(&part);
    let graph = part.get_mesh_edge_graph();
    let edge_at_pair = edges_by_vertex_pair(&graph);
    let feature_edges: HashSet<i32> = resolution
        .features
        .edges()
        .iter()
        .map(|feature| feature.mesh_edge_index)
        .collect();
    let mouths = fixture::the_mouths(&part, &resolution);

    assert_eq!(
        mouths.len(),
        2,
        "the fixture has two mouths, not {}",
        mouths.len()
    );

    let ring: Vec<i32> = mouths[0]
        .edges
        .iter()
        .map(|edge| edge.from_vertex())
        .collect();
    let mut pool = SectionPool::new(&part.vertices);

    let run = try_build_loop(
        &ring,
        Some(&resolution.features),
        &part,
        &graph,
        &mut pool,
        fixture::SIZE,
        fixture::SEGMENTS,
        true,
    )
    .unwrap_or_else(|why| panic!("the mouth builds its run: {why:?}"));

    let left_is_smooth = run
        .sections
        .iter()
        .filter(|section| section.left_kind == SectionSide::Offset)
        .count()
        >= run
            .sections
            .iter()
            .filter(|section| section.right_kind == SectionSide::Offset)
            .count();

    let is_a_feature = |edge: i32| feature_edges.contains(&edge);
    let (cut, done) = try_cut_the_smooth_side(
        &part,
        &graph,
        &edge_at_pair,
        &mut pool,
        &run,
        left_is_smooth,
        Some(&is_a_feature),
    );

    let (refusal, where_) = match done {
        Ok(()) => (SmoothSideRefusal::None, "0, 0, 0".to_string()),
        Err(SmoothSideFailure::Refused(refusal, at)) => (refusal, format!("{at:?}")),
        Err(SmoothSideFailure::NotANumber(nan)) => panic!("the cut minted a NaN point: {nan}"),
    };

    Reading {
        rings: cut.rings_of_the_refused_facet.clone(),
        cut: (refusal == SmoothSideRefusal::None).then_some(cut),
        refusal,
        where_,
    }
}

struct Reading {
    cut: Option<SmoothSideCut>,
    refusal: SmoothSideRefusal,
    where_: String,
    rings: Vec<i32>,
}
