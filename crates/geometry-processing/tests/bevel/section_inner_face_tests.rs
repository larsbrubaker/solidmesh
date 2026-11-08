//! "Inner Face Should Bevel.mcx" on the part's own mesh
//! ([`crate::inner_face_test_mesh`]) - the bore wall's two mouths, each riding over a convex edge of
//! the cube with two ninety degree turns, routed to the sections and built.
//!
//! The turn is not what sends it to legacy: the mouth riding over the cube's edge reads as the mitre
//! it is (`SectionCorner`'s shared side need only be one SURFACE). Built through the shared
//! `surface_test_build` with the sections flag pinned on.

use std::rc::Rc;

use solidmesh::bevel::sections::{section_corner, section_routing, section_split_blend};
use solidmesh::bevel::surface::SurfaceBevelPlan;
use solidmesh::bevel::BevelAdvisory;
use solidmesh::topology::{
    bevel_edge_group_resolver, BevelEdgeGroupResolution, EdgeConvexity, FeatureEdge,
};
use solidmesh_math::Vector3;
use solidmesh_mesh::CancelToken;

use crate::inner_face_test_mesh as fixture;
use crate::section_turn_readings::{
    chain_dihedral_degrees, feature_dihedral_degrees, the_creases_at, the_sharp_turns_of,
};
use crate::surface_test_build::{
    pinned_to_the_sections, station, the_closed_readable_mesh, try_bevel_resolutions,
    TryBevelOptions, TryBevelOutcome,
};

/// THE TURN IS NOT WHAT SENDS IT TO LEGACY: the mouth riding over the cube's edge reads as the
/// mitre it is - `SectionCorner`'s shared side need only be one SURFACE - so the router keeps the
/// pick and the sections carry it through.
#[test]
fn the_bore_wall_is_routed_through_the_sections() {
    let _sections = pinned_to_the_sections();

    let outcome = the_bore_wall_at(fixture::SIZE);

    assert_eq!(
        section_routing::last_route().as_deref(),
        Some("sections"),
        "the ninety degree turn is a corner the sections can draw"
    );

    if let Err(reason) = outcome.mesh_or_reason() {
        panic!(
            "{:?} at {}",
            reason.map(BevelAdvisory::code),
            station(reason)
        );
    }
}

/// THE READING of the two turning vertices the router names. Every number the reading quotes
/// is asserted here.
#[test]
fn the_turning_vertices_are_a_mouth_running_over_a_convex_part_edge() {
    let part = fixture::the_part();

    let plans: Vec<Rc<SurfaceBevelPlan>> = {
        let _sections = pinned_to_the_sections();
        the_bore_wall_at(fixture::SIZE).plans_built
    };

    assert_eq!(plans.len(), 2, "the pick is the bore's two mouths");
    assert!(
        plans
            .iter()
            .all(|plan| plan.is_closed && plan.edges.len() == 76),
        "seventy-six edges each, both closed - 152 in two loops, as the panel's row says"
    );

    let turns = the_sharp_turns_of(&plans);

    let mut turning: Vec<i32> = turns
        .iter()
        .map(|(_, leaving)| leaving.from_vertex())
        .collect();
    turning.sort_unstable();
    assert_eq!(
        turning,
        vec![19, 59, 100, 140],
        "two ninety degree turns on each mouth, of which the app names 59 and 100"
    );

    for vertex in [59, 100] {
        let matching: Vec<_> = turns
            .iter()
            .filter(|(_, leaving)| leaving.from_vertex() == vertex)
            .collect();
        assert_eq!(matching.len(), 1);
        let (arriving, leaving) = matching[0];

        assert!(
            (section_corner::turn_degrees(arriving, leaving) - 90.176).abs() <= 0.002,
            "vertex {vertex} turns a real ninety degrees, not noise"
        );

        // NEITHER SIDE IS FLAT, which is the whole of it. The shared side is the bore WALL, but as
        // two facets of the tessellation about nine degrees apart; the other side is two different
        // walls of the cube meeting square.
        let along_the_wall = arriving
            .right_normal()
            .get_normal()
            .dot(leaving.right_normal().get_normal());
        let across_the_part_edge = arriving
            .left_normal()
            .get_normal()
            .dot(leaving.left_normal().get_normal());

        assert!(
            (along_the_wall - 0.98769).abs() <= 1e-4,
            "the bore wall's own fold at vertex {vertex} is nine degrees, not flat"
        );

        assert!(
            along_the_wall < FeatureEdge::COPLANAR_FACE_DOT,
            "so the 'shared' side does not read as one flat side at all"
        );

        assert!(
            across_the_part_edge.abs() < 1e-4,
            "and the other two faces at vertex {vertex} are square to each other - two different \
             walls of the cube, which is the part edge the mouth rides over"
        );

        // And the two wedges differ by ten times the tolerance a corner would be read at.
        let arriving_wedge = arriving
            .left_normal()
            .get_normal()
            .dot(arriving.right_normal().get_normal());
        let leaving_wedge = leaving
            .left_normal()
            .get_normal()
            .dot(leaving.right_normal().get_normal());

        assert!(
            ((arriving_wedge - leaving_wedge).abs() - 1.1e-5).abs() <= 2e-6,
            "the second clause would decline it too, against a SameWedgeToWithin of 1e-6"
        );

        assert!(
            section_corner::looks_like_a_corner(arriving, leaving),
            "and the corner rule reads that as ONE SURFACE shared - the bore's wall, held as two \
             facets - with the two cube faces as the sides that differ"
        );

        // THE THIRD CREASE: the cube's own part edge, one millimetre long, square and convex.
        let creases: Vec<FeatureEdge> =
            the_creases_at(&part, vertex, fixture::BEND_THRESHOLD_DEGREES)
                .into_iter()
                .filter(|crease| {
                    crease.start_vertex != arriving.from_vertex()
                        && crease.end_vertex != arriving.from_vertex()
                        && crease.start_vertex != leaving.to_vertex()
                        && crease.end_vertex != leaving.to_vertex()
                })
                .collect();

        assert_eq!(
            creases.len(),
            1,
            "exactly one crease leaves vertex {vertex} that is not the chain"
        );

        let part_edge = &creases[0];

        assert!(
            (part_edge.direction.length() - 1.0).abs() <= 1e-6,
            "a millimetre long"
        );

        assert!(
            (part_edge.direction.get_normal().y.abs() - 1.0).abs() <= 1e-4,
            "running straight along Y - the cube's edge, not the bore's"
        );

        assert!(
            (feature_dihedral_degrees(part_edge) - 90.0).abs() <= 1e-3,
            "square"
        );

        assert_eq!(
            part_edge.convexity,
            EdgeConvexity::Convex,
            "and convex - the outside of the cube"
        );

        // The chain's own two creases are the bore wall's folds, which are NOT square.
        assert!((chain_dihedral_degrees(arriving) - 93.18).abs() <= 0.01);
        assert!((chain_dihedral_degrees(leaving) - 93.18).abs() <= 0.01);
    }

    // AND THE TWO LOOPS DO NOT MEET: they are the bore's two mouths, at opposite ends of the part.
    let far: Vec<&Rc<SurfaceBevelPlan>> = plans
        .iter()
        .filter(|plan| !plan.edges.iter().any(|edge| edge.edge.from_vertex() == 59))
        .collect();
    assert_eq!(far.len(), 1);

    let vertex_59 = Vector3::from_vector3_float(part.vertices[59]);
    let near_vertex_59 = far[0]
        .edges
        .iter()
        .map(|edge| (edge.edge.from() - vertex_59).length())
        .fold(f64::MAX, f64::min);

    assert!(
        near_vertex_59 > 20.0,
        "the far mouth's nearest point to vertex 59 is a whole part away, so nothing here is two \
         loops touching - it is {near_vertex_59}"
    );
}

/// THE CORNER STANDS: the same pick builds through the sections - closed, manifold, one shell - at
/// the largest size measured to get all the way through, 0.1 mm.
#[test]
fn the_bore_wall_takes_a_small_fillet_through_the_sections() {
    the_bore_wall_builds(0.1);
}

/// LARS'S CLAIM ON THIS FILE, MET: the bore wall's rim takes its 2 mm fillet, closed, manifold and
/// in one shell. What used to stop it was the split vertex beside each mitre.
#[test]
fn the_bore_wall_takes_its_fillet_through_the_sections() {
    the_bore_wall_builds(fixture::SIZE);
}

/// The same 2 mm build with the split vertices ABSORBED into their gap's boundary rows - the split
/// vertex beside each mitre laying no station of its own.
#[test]
fn the_bore_wall_takes_its_fillet_with_the_split_vertices_absorbed() {
    let found = section_split_blend::absorb_split_vertices();
    section_split_blend::set_absorb_split_vertices(true);

    // Put the flag back even when the build's assertions fail, so later tests see the default.
    struct PutBack(bool);
    impl Drop for PutBack {
        fn drop(&mut self) {
            section_split_blend::set_absorb_split_vertices(self.0);
        }
    }
    let _put_back = PutBack(found);

    the_bore_wall_builds(fixture::SIZE);
}

fn the_bore_wall_builds(size: f64) {
    let _sections = pinned_to_the_sections();

    the_closed_readable_mesh(&the_bore_wall_at(size));
}

/// The bore wall alone, built the part builder's own way - see
/// `surface_test_build`.
fn the_bore_wall_at(size: f64) -> TryBevelOutcome {
    let part = fixture::the_part();

    try_bevel_resolutions(
        &part,
        &[the_rim(&part)],
        size,
        fixture::SEGMENTS,
        &CancelToken::new(),
        TryBevelOptions::default(),
    )
}

fn the_rim(part: &solidmesh_mesh::Mesh) -> BevelEdgeGroupResolution {
    let (mut resolutions, _) =
        bevel_edge_group_resolver::resolve_all(&[fixture::the_bore_wall()], part);
    resolutions.remove(0)
}
