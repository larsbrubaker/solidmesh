//! Port of `SectionRoutedPassTests.cs`: WHOLE PASSES THROUGH THE ROUTER on the straight bore's
//! plate: both mouths routed in one pass and assembled closed, a pass mixing the mouth with an open rim
//! edge left to the legacy construction untouched, and a routed pass refused by its OWN room reading
//! past the land with the most that fits. Built through the shared `SurfaceTestBuild` port, except
//! the room row, which calls `surface_bevel_builder::try_build` directly so the legacy room does not
//! answer first.

use std::sync::Arc;

use geometry_processing::bevel::sections::section_routing;
use geometry_processing::bevel::surface::{
    surface_bevel_builder, surface_mesh_closure, surface_shared_edge_network, SurfaceBevelPlan,
    SurfaceBuildFailure, SurfaceBuildOptions, SurfaceBuilt,
};
use geometry_processing::bevel::{BevelAdvisoryArg, BevelAdvisoryCode};
use geometry_processing::topology::{
    bevel_edge_group_resolver, BevelEdgeGroupResolution, BevelSelectionKind, FaceGroup,
    FaceRegionCache, FeatureEdgeSet, MatchTolerances,
};
use polygon_mesh::{CancelToken, Mesh};
use vector_math::Vector3;

use crate::bore_test_meshes::create_plate_with_a_round_bore;
use crate::mesh_measurements::shell_count;
use crate::rigid_motion_meshes::most_faces_on_an_edge;
use crate::surface_test_build::{pinned_to_the_sections, try_bevel_resolutions, TryBevelOptions};
use crate::surface_test_chains::join_and_open;

const SIDES: i32 = 24;

const BORE_RADIUS: f64 = 3.0;

const THICKNESS: f64 = 10.0;

/// The plate fixture's land - the flat ring from the bore's mouth out to the plate's rim.
const LAND: f64 = 6.0;

const SEGMENTS: i32 = 8;

const BEND_THRESHOLD_DEGREES: f64 = 20.0;

/// A size both the land and the thickness leave room for.
const SIZE: f64 = 0.4;

/// Both mouths of the straight bore, picked in one pass: two plans, one pool, one assembly - and
/// what comes back is closed, manifold and one shell.
#[test]
fn both_mouths_of_the_bore_route_in_one_pass() {
    let _sections = pinned_to_the_sections();

    let part = the_plate();
    let built = try_bevel_both_mouths(&part, SIZE);

    assert_eq!(
        section_routing::last_route().as_deref(),
        Some("sections"),
        "every plan of the pass passes every clause on its own"
    );

    let mesh = built
        .unwrap_or_else(|failure| panic!("{:?}", failure.advisory().map(|a| a.code())))
        .mesh;

    let (open, open_at) = surface_mesh_closure::open_edge_count(&mesh);
    assert_eq!(
        open, 0,
        "one rebuild and two lofts share one face list - open at {open_at:?}"
    );

    assert!(most_faces_on_an_edge(&mesh) <= 2);
    assert_eq!(shell_count(&mesh), 1);
}

/// A pass carrying one simple chain and one that is not - the bore's mouth and a single picked edge
/// of the plate's rim, which is an OPEN chain - does not route AT ALL: the sections take whole
/// passes or none, so the answer is the legacy construction's own, bit for bit.
#[test]
fn a_pass_mixing_a_simple_chain_with_an_open_one_does_not_route() {
    let off = try_bevel_the_mouth_and_one_rim_edge(SIZE);

    assert_eq!(
        section_routing::last_route().as_deref(),
        Some("legacy: the flag is off")
    );

    let _sections = pinned_to_the_sections();

    let on = try_bevel_the_mouth_and_one_rim_edge(SIZE);

    assert_eq!(
        section_routing::last_route().as_deref(),
        Some("legacy: a chain is not closed"),
        "one plan of the pass is not simple, so no plan of it routes"
    );

    assert_eq!(on.is_ok(), off.is_ok());
    assert_eq!(
        on.as_ref()
            .err()
            .and_then(|f| f.advisory())
            .map(|a| a.code()),
        off.as_ref()
            .err()
            .and_then(|f| f.advisory())
            .map(|a| a.code())
    );

    if let (Ok(on_mesh), Ok(off_mesh)) = (&on, &off) {
        assert!(
            the_same_mesh(&on_mesh.mesh, &off_mesh.mesh),
            "a pass that does not route is the legacy construction's answer untouched"
        );
    }
}

/// A routed pass reads the ROOM before it assembles: past the land the mouth's fillet has nowhere
/// to go, and the pass is refused with the size advisory quoting the land - not built by the legacy
/// construction, whose room runs out at a different number.
#[test]
fn past_the_land_the_routed_pass_is_refused_with_the_most_that_fits() {
    let _sections = pinned_to_the_sections();

    let over = try_bevel_the_upper_mouth_past_the_legacy_room(1.001 * LAND);

    assert_eq!(
        section_routing::last_route().as_deref(),
        Some("sections"),
        "the pass routed and was then refused by its own room reading"
    );

    let refusal = match over {
        Ok(_) => panic!("a fillet wider than the land does not fit"),
        Err(failure) => failure
            .advisory()
            .cloned()
            .expect("the room names its advisory"),
    };

    assert_eq!(refusal.code(), BevelAdvisoryCode::SizeExceedsFaceWidth);

    match refusal.args().get(2) {
        Some(BevelAdvisoryArg::Double(most)) => assert!(
            (most - LAND).abs() <= 1e-3,
            "the advisory carries the most that fits, which is the land - it carries {most}"
        ),
        other => panic!("the advisory's third argument is a size, not {other:?}"),
    }

    let under =
        try_bevel_the_upper_mouth_past_the_legacy_room(0.999 * LAND).unwrap_or_else(|why_not| {
            panic!(
                "a tenth of a percent inside the land it builds - {:?}",
                why_not.advisory().map(|a| a.code())
            )
        });

    assert_eq!(surface_mesh_closure::open_edge_count(&under.mesh).0, 0);
}

fn the_plate() -> Mesh {
    create_plate_with_a_round_bore(SIDES, BORE_RADIUS, THICKNESS, LAND)
}

/// C# `TryBevelBothMouths`: both mouth loops handed to ONE build, picked through `resolve_crease`,
/// which at 24 sides chooses among the three feature edges meeting at every mouth vertex.
fn try_bevel_both_mouths(part: &Mesh, size: f64) -> Result<SurfaceBuilt, SurfaceBuildFailure> {
    let features = Arc::new(FeatureEdgeSet::create_with_graph(
        part,
        &part.get_mesh_edge_graph(),
        BEND_THRESHOLD_DEGREES,
    ));

    let resolutions = [
        the_mouth(&features, THICKNESS / 2.0),
        the_mouth(&features, -THICKNESS / 2.0),
    ];

    try_bevel_resolutions(
        part,
        &resolutions,
        size,
        SEGMENTS,
        &CancelToken::new(),
        TryBevelOptions::default(),
    )
    .result
}

/// C# `TryBevelTheMouthAndOneRimEdge`: the bore's upper mouth and one edge of the plate's top rim,
/// in one pass.
fn try_bevel_the_mouth_and_one_rim_edge(size: f64) -> Result<SurfaceBuilt, SurfaceBuildFailure> {
    let part = the_plate();
    let features = Arc::new(FeatureEdgeSet::create_with_graph(
        &part,
        &part.get_mesh_edge_graph(),
        BEND_THRESHOLD_DEGREES,
    ));

    let rim = bevel_edge_group_resolver::create_from_edge(
        &features,
        the_rim_edge(&features),
        BevelSelectionKind::Edge,
        FaceGroup::DEFAULT_CURVE_TOLERANCE_DEGREES,
    )
    .expect("the rim edge is a crease of the plate");

    let resolutions = [
        the_mouth(&features, THICKNESS / 2.0),
        bevel_edge_group_resolver::resolve(&rim, &part),
    ];

    try_bevel_resolutions(
        &part,
        &resolutions,
        size,
        SEGMENTS,
        &CancelToken::new(),
        TryBevelOptions::default(),
    )
    .result
}

/// C# `TryBevelTheUpperMouthPastTheLegacyRoom`: the upper mouth alone, built through
/// `surface_bevel_builder` DIRECTLY rather than through `SurfaceTestBuild` - the legacy room
/// (`surface_room::try_fit`) runs ahead of the builder there and would refuse a size past the land
/// before the router was ever asked, and what is read here is the ROUTED pass's own refusal.
fn try_bevel_the_upper_mouth_past_the_legacy_room(
    size: f64,
) -> Result<SurfaceBuilt, SurfaceBuildFailure> {
    let part = the_plate();
    let graph = part.get_mesh_edge_graph();
    let features = Arc::new(FeatureEdgeSet::create_with_graph(
        &part,
        &graph,
        BEND_THRESHOLD_DEGREES,
    ));
    let resolutions = [the_mouth(&features, THICKNESS / 2.0)];

    let regions = Arc::new(FaceRegionCache::new(
        Arc::new(part.clone()),
        Some(graph.clone()),
        features.flat_angle_degrees(),
    ));
    let curve_regions = Arc::new(FaceRegionCache::new(
        Arc::new(part.clone()),
        Some(graph.clone()),
        resolutions[0].group.curve_tolerance_degrees,
    ));

    let mut plans = Vec::new();

    for chain in join_and_open(&surface_shared_edge_network::link_together(&resolutions)) {
        let plan = SurfaceBevelPlan::try_create_with_curve_regions(
            &chain,
            &regions,
            Some(&curve_regions),
            size,
            true,
        )
        .map_err(SurfaceBuildFailure::Refused)?;

        plans.push(plan);
    }

    surface_bevel_builder::try_build(
        &part,
        &graph,
        &regions,
        &plans,
        SEGMENTS,
        &CancelToken::new(),
        SurfaceBuildOptions {
            crossings: None,
            ran_off_over_a_stretch: None,
            lay_the_oblique_rows_anyway: false,
            planless_loops: &[],
            features: None,
        },
    )
}

/// C# `TheMouth`: the mouth loop at `height`, resolved as a crease so it walks all 24 sides.
fn the_mouth(features: &Arc<FeatureEdgeSet>, height: f64) -> BevelEdgeGroupResolution {
    for (i, edge) in features.edges().iter().enumerate() {
        if !on_the_mouth(edge.start, height) || !on_the_mouth(edge.end, height) {
            continue;
        }

        let group = bevel_edge_group_resolver::create_from_edge(
            features,
            i as i32,
            BevelSelectionKind::TangentChain,
            FaceGroup::DEFAULT_CURVE_TOLERANCE_DEGREES,
        )
        .expect("the mouth edge is a crease of the plate");

        let resolved = bevel_edge_group_resolver::resolve_crease(
            &group,
            features.clone(),
            FaceGroup::DEFAULT_CURVE_TOLERANCE_DEGREES,
            FaceGroup::DEFAULT_CURVE_TOLERANCE_DEGREES,
            MatchTolerances::default(),
        );

        match &resolved.chain {
            Some(chain) if chain.is_closed() && chain.count() == SIDES as usize => return resolved,
            other => panic!(
                "the mouth at {height} walks a closed loop of {SIDES} - it walked {}",
                other.as_ref().map_or(0, |c| c.count())
            ),
        }
    }

    panic!("the fixture has no mouth at {height}");
}

/// C# `TheRimEdge`: an edge of the plate's top rim - the outer boundary of the land.
fn the_rim_edge(features: &FeatureEdgeSet) -> i32 {
    for (i, edge) in features.edges().iter().enumerate() {
        if on_the_rim(edge.start) && on_the_rim(edge.end) {
            return i as i32;
        }
    }

    panic!("the fixture has no top rim");
}

fn on_the_mouth(at: Vector3, height: f64) -> bool {
    (at.z - height).abs() < 1e-4
        && (Vector3::new(at.x, at.y, 0.0).length() - BORE_RADIUS).abs() < 1e-4
}

fn on_the_rim(at: Vector3) -> bool {
    (at.z - (THICKNESS / 2.0)).abs() < 1e-4
        && (Vector3::new(at.x, at.y, 0.0).length() - (BORE_RADIUS + LAND)).abs() < 1e-3
}

/// C# `TheSameMesh`: bit for bit - the same points in the same order making the same faces in the
/// same order.
fn the_same_mesh(left: &Mesh, right: &Mesh) -> bool {
    left.vertices.len() == right.vertices.len()
        && left.faces.len() == right.faces.len()
        && left
            .vertices
            .iter()
            .zip(right.vertices.iter())
            .all(|(l, r)| l == r)
        && left
            .faces
            .iter()
            .zip(right.faces.iter())
            .all(|(l, r)| l.v0 == r.v0 && l.v1 == r.v1 && l.v2 == r.v2)
}
