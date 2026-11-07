//! Many holes at a workable size - the two-bore tapered
//! barrel ([`crate::tapered_barrel_test_meshes`]), its wall picked as a face group. Every reading
//! is taken on the working part, the form the app hands the kernel; the raw boolean appears once,
//! as the row this fixture's size is stated against. The routed rows pin the sections router on
//! (`pinned_to_the_sections`) and the legacy rows off, around their own builds.
//!
//! `TheWallTakesItsFilletThroughTheSections` is not run: the section assembly still refuses the
//! routed pass as not closed, six seams cutting one wall.

use std::rc::Rc;
use std::sync::Arc;

use geometry_processing::bevel::bevel_chain_linker::link;
use geometry_processing::bevel::sections::section_routing;
use geometry_processing::bevel::surface::{
    surface_mesh_closure, surface_shared_edge_network, SurfaceBevelPlan,
};
use geometry_processing::topology::{BevelEdgeGroupResolution, FaceRegionCache, FeatureEdgeChain};
use polygon_mesh::{CancelToken, Mesh};

use crate::surface_test_build::{
    pinned_to_the_legacy_construction, pinned_to_the_sections, try_bevel_resolutions,
    TryBevelOptions, TryBevelOutcome,
};
use crate::surface_test_chains::join_and_open;
use crate::tapered_barrel_test_meshes;

/// A size neither room refuses on this pick.
const A_SIZE_THE_ROOM_ALLOWS: f64 = 0.35;

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
    // A stable sort, so the FIRST smallest closed loop.
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

#[test]
fn both_stations_pass_the_room_at_the_asked_size() {
    let part = tapered_barrel_test_meshes::the_working_part();
    let resolution = tapered_barrel_test_meshes::the_wall(&part);

    {
        let _legacy = pinned_to_the_legacy_construction();
        let legacy = build(&part, &resolution, tapered_barrel_test_meshes::SIZE);

        assert!(!legacy.built());
        assert_eq!(
            code_of(&legacy).as_deref(),
            Some("SurfaceResultNotClosed"),
            "the room no longer answers at all on this pick, so the legacy leg reaches the builder \
             and its own construction is what stops it"
        );
        assert_eq!(
            legacy.plans_built.len(),
            6,
            "every loop of the pick plans on the conditioned part - see \
             EveryLoopOfTheWallPickPlans - so nothing returns early and all six are published"
        );
    }

    let _sections = pinned_to_the_sections();
    section_routing::set_last_route(None);

    let routed = build(&part, &resolution, tapered_barrel_test_meshes::SIZE);

    assert!(!routed.built());
    assert_eq!(
        section_routing::last_route().as_deref(),
        Some("sections"),
        "at the fixture's own size the pass now reaches the router, and the router takes it: {:?}",
        code_of(&routed)
    );
    // The same refusal the pick meets at 0.35: the assembly lays a mesh that does not close - read
    // on the answer by SectionAssembly and refused there rather than handed back as built.
    assert_eq!(
        code_of(&routed).as_deref(),
        Some("SurfaceResultNotClosed"),
        "the same not-closed refusal the pick meets at 0.35 - see \
         TheWallPickIsRoutedPastBothRoomsAndEndsAtACorner"
    );
    assert_eq!(routed.plans_built.len(), 6);
}

#[test]
fn the_wall_pick_is_routed_past_both_rooms_and_ends_at_a_corner() {
    let part = tapered_barrel_test_meshes::the_working_part();
    let resolution = tapered_barrel_test_meshes::the_wall(&part);

    let _sections = pinned_to_the_sections();
    section_routing::set_last_route(None);

    let outcome = build(&part, &resolution, A_SIZE_THE_ROOM_ALLOWS);

    assert_eq!(
        section_routing::last_route().as_deref(),
        Some("sections"),
        "a plan whose wall the sections can cut is routed, not handed back: {:?}",
        code_of(&outcome)
    );
    assert!(!outcome.built());
    assert_eq!(
        code_of(&outcome).as_deref(),
        Some("SurfaceResultNotClosed"),
        "neither room refuses this size any more and the smooth cut's join does not refuse either: \
         the assembly publishes, the answer is OPEN, and SectionAssembly refuses it by name"
    );
}

#[test]
fn the_routed_wall_pick_never_reports_an_open_result_as_built() {
    let part = tapered_barrel_test_meshes::the_working_part();
    let resolution = tapered_barrel_test_meshes::the_wall(&part);

    let _sections = pinned_to_the_sections();

    for size in [A_SIZE_THE_ROOM_ALLOWS, tapered_barrel_test_meshes::SIZE] {
        let outcome = build(&part, &resolution, size);

        match outcome.mesh_or_reason() {
            Ok(mesh) => {
                let (open, open_at) = surface_mesh_closure::open_edge_count(mesh);

                assert_eq!(
                    open, 0,
                    "at {size} the build said it built, but the mesh is open at {open_at:?}"
                );
            }
            Err(why) => assert!(why.is_some(), "at {size} a refusal names itself"),
        }
    }
}

#[test]
fn every_loop_of_the_wall_pick_plans() {
    let part = Arc::new(tapered_barrel_test_meshes::the_working_part());
    let resolution = tapered_barrel_test_meshes::the_wall(&part);
    let graph = part.get_mesh_edge_graph();
    let regions = Arc::new(FaceRegionCache::new(
        part.clone(),
        Some(graph.clone()),
        resolution.features.flat_angle_degrees(),
    ));
    let curve_regions = Arc::new(FaceRegionCache::new(
        part.clone(),
        Some(graph.clone()),
        resolution.group.curve_tolerance_degrees,
    ));

    let verdicts: Vec<(usize, String)> = join_and_open(
        &surface_shared_edge_network::link_together(std::slice::from_ref(&resolution)),
    )
    .iter()
    .map(|chain| {
        let planned: Result<Rc<SurfaceBevelPlan>, _> =
            SurfaceBevelPlan::try_create_with_curve_regions(
                chain,
                &regions,
                Some(&curve_regions),
                tapered_barrel_test_meshes::SIZE,
                true,
            );

        (
            chain.edges.len(),
            match planned {
                Ok(_) => "planned".to_string(),
                Err(why) => format!("{:?}", why.code()),
            },
        )
    })
    .collect();

    let reading = verdicts
        .iter()
        .map(|(edges, code)| format!("{edges}:{code}"))
        .collect::<Vec<_>>()
        .join(", ");

    assert_eq!(verdicts.len(), 6, "{reading}");
    assert_eq!(
        verdicts
            .iter()
            .filter(|(_, code)| code == "planned")
            .count(),
        6,
        "{reading}"
    );
}

/// The wall pick through `surface_test_build::try_bevel`, carrying the pick's own
/// feature reading down the plan path.
fn build(part: &Mesh, resolution: &BevelEdgeGroupResolution, size: f64) -> TryBevelOutcome {
    try_bevel_resolutions(
        part,
        std::slice::from_ref(resolution),
        size,
        tapered_barrel_test_meshes::SEGMENTS,
        &CancelToken::new(),
        TryBevelOptions {
            round_profile: true,
            carry_the_feature_reading: true,
        },
    )
}

/// The refusal's code as text, `None` where the build did not refuse.
fn code_of(outcome: &TryBevelOutcome) -> Option<String> {
    outcome.reason().map(|why| format!("{:?}", why.code()))
}
