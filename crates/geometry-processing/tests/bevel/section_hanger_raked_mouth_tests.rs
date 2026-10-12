//! Port of `SectionHangerRakedMouthTests.cs`: THE PICTURE HANGER'S RAKED MOUTH, on the hanger's own
//! mesh ([`crate::picture_hanger_test_mesh`]) - the 48 edge tangent chain of group 0, whose room is
//! millimetres rather than a split vertex's own facet, taking its 0.4 fillet through the sections
//! alone and together with the top rim, with the split vertices blended and absorbed. Built through
//! the shared `SurfaceTestBuild` port with the sections flag pinned on.

use geometry_processing::bevel::sections::{section_room, section_routing, section_split_blend};
use geometry_processing::bevel::BevelAdvisory;
use geometry_processing::topology::{
    bevel_edge_group_resolver, BevelEdgeGroupResolution, FeatureEdgeChain, FeatureEdgeSet,
};
use polygon_mesh::{CancelToken, Mesh, MeshEdgeGraph};
use vector_math::Vector3;

use crate::picture_hanger_test_mesh as fixture;
use crate::surface_test_build::{
    pinned_to_the_sections, station, the_closed_readable_mesh, try_bevel, try_bevel_resolutions,
    TryBevelOptions, TryBevelOutcome,
};

/// THE ROOM ALONE, with nothing built: the mouth has at least a millimetre of room, and the 0.4 the
/// file asks fits. This is the reading the whole row turned on, and it costs no build.
#[test]
fn the_raked_mouths_room_is_not_limited_by_a_split_vertexs_own_facet() {
    let part = fixture::the_part();
    let graph = part.get_mesh_edge_graph();
    let chain = the_mouth_chain(&part, &graph);

    let (fits, largest, failure) =
        section_room::try_fit(Some(&part), Some(&graph), Some(&chain), fixture::SIZE);

    assert!(
        fits,
        "the 0.4 fits - instead {:?} at {}, offering {largest}",
        failure.as_ref().map(BevelAdvisory::code),
        station(failure.as_ref())
    );

    // The mouth has 4.5 mm of plate on the flat side and 5.66 mm of bore down the wall; a facet's
    // own ruling, 0.121 from the split vertex, is not a limit of the part.
    assert!(
        largest > 1.0,
        "and the room is millimetres, not a facet's width - it read {largest}"
    );
}

/// THE CLAIM: the mouth alone takes its 0.4 fillet through the sections, closed, manifold and in
/// one shell, with none of its original vertices left standing.
#[test]
fn the_raked_mouth_takes_its_fillet_through_the_sections() {
    let _sections = pinned_to_the_sections();

    let part = fixture::the_part();
    let mouth = the_mouth(&part);

    let outcome = try_bevel_resolutions(
        &part,
        std::slice::from_ref(&mouth),
        fixture::SIZE,
        fixture::SEGMENTS,
        &CancelToken::new(),
        TryBevelOptions::default(),
    );

    let mesh = the_closed_readable_mesh(&outcome);

    assert_eq!(section_routing::last_route().as_deref(), Some("sections"));

    // Every station of the picked chain is consumed by the fillet: a mouth vertex still standing in
    // the published mesh is a piece of the original crease that was never rounded.
    for edge in &mouth.edges {
        assert!(
            !stands_in(mesh, edge.start),
            "the mouth vertex {:?} is still in the published mesh",
            edge.start
        );
    }
}

/// BOTH of the file's picks in one pass, at 0.4 - the top rim and the raked mouth together - build.
#[test]
fn both_picks_together_build_in_one_pass() {
    let _sections = pinned_to_the_sections();

    let outcome = both_picks();

    the_closed_readable_mesh(&outcome);
}

/// The same two picks with the split vertices ABSORBED into their gap's boundary rows rather than
/// blended into stations of their own - the reading that says the absorbed loft is not a fix for the
/// barrel alone.
#[test]
fn both_picks_together_build_with_the_split_vertices_absorbed() {
    // C#'s `finally`: put the flag back even when an assertion fails.
    struct PutBack(bool);
    impl Drop for PutBack {
        fn drop(&mut self) {
            section_split_blend::set_absorb_split_vertices(self.0);
        }
    }

    let _sections = pinned_to_the_sections();
    let _put_back = PutBack(section_split_blend::absorb_split_vertices());
    section_split_blend::set_absorb_split_vertices(true);

    let outcome = both_picks();

    the_closed_readable_mesh(&outcome);
}

/// The top rim and the raked mouth, resolved together and built in one pass.
fn both_picks() -> TryBevelOutcome {
    let part = fixture::the_part();

    try_bevel(
        &part,
        &[fixture::the_top_face_rim(), fixture::the_raked_mouth()],
        fixture::SIZE,
        fixture::SEGMENTS,
        &CancelToken::new(),
        TryBevelOptions::default(),
    )
}

/// C# `TheMouth`: the mouth as the file picks it - 48 edges of a tangent chain.
fn the_mouth(part: &Mesh) -> BevelEdgeGroupResolution {
    let (mut resolutions, _) =
        bevel_edge_group_resolver::resolve_all(&[fixture::the_raked_mouth()], part);
    resolutions.remove(0)
}

/// C# `TheMouthChain`: the same mouth as a feature chain, which is what the room measures - walked
/// out from the feature edge standing where the group's first resolved edge stands.
fn the_mouth_chain(part: &Mesh, graph: &MeshEdgeGraph) -> FeatureEdgeChain {
    let picked = the_mouth(part);
    let wanted = (picked.edges[0].start + picked.edges[0].end) / 2.0;
    let features = FeatureEdgeSet::create_with_graph(part, graph, fixture::BEND_THRESHOLD_DEGREES);

    for (i, edge) in features.edges().iter().enumerate() {
        if ((edge.start + edge.end) / 2.0 - wanted).length() < 1e-4 {
            let chain = FeatureEdgeChain::propagate(&features, i as i32)
                .expect("the seed is an edge of the set");

            assert!(
                chain.is_closed(),
                "the mouth walks closed, and it walked {} open",
                chain.count()
            );

            return chain;
        }
    }

    panic!("no feature edge stands where the mouth's first picked edge does");
}

/// C# `StandsIn`: whether any vertex of a mesh stands at a given position.
fn stands_in(mesh: &Mesh, at: Vector3) -> bool {
    mesh.vertices
        .iter()
        .any(|vertex| (Vector3::from_vector3_float(*vertex) - at).length() < 1e-5)
}
