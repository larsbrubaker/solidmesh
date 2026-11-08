//! WHICH PASSES THE ROUTER SENDS TO THE SECTIONS, read off
//! `section_routing::last_route` after a whole build through the shared `surface_test_build` -
//! the straight forty-side bore mouth wherever the part stands, a box's top rim with its four
//! corners, the raked bore cube's open top mouth arc, a plain cube edge alone, a chamfer, and the
//! flag left off. Each parameter row is one pass of a loop in the test of the same name.

use solidmesh::bevel::sections::{section_end, section_routing};
use solidmesh::bevel::surface::surface_mesh_closure;
use solidmesh::bevel::BevelAdvisory;
use solidmesh::topology::{
    bevel_edge_group_resolver, BevelEdgeGroupResolution, BevelSelectionKind, FaceGroup,
    FeatureEdgeSet,
};
use solidmesh_math::{Matrix4X4, Vector3};
use solidmesh_mesh::solids::platonic_solids;
use solidmesh_mesh::{CancelToken, Mesh};

use crate::bore_test_meshes::create_plate_with_a_raked_bore;
use crate::inner_face_test_mesh as inner_face;
use crate::mesh_measurements::shell_count;
use crate::open_mouth_arc_walk::the_top_mouth_arc;
use crate::rigid_motion_meshes::{
    most_faces_on_an_edge, move_mesh, signature_of, the_motions, TopologySignature,
};
use crate::surface_bevel_robustness_tests as robustness;
use crate::surface_test_build::{
    pinned_to_the_sections, try_bevel, try_bevel_resolutions, TryBevelOptions,
};

/// A size the bore fixture's 1.5 radius and 4 thickness both leave room for.
pub(crate) const SIZE: f64 = 0.2;

/// The straight forty-side bore's mouth is the router's own case - closed, convex, two edges at
/// every vertex, nine degrees of turn - so with the flag on the whole pass goes through the
/// sections, and what comes back is closed, manifold and one shell wherever the part stands.
#[test]
fn the_straight_forty_side_bore_mouth_routes_through_the_sections() {
    let _sections = pinned_to_the_sections();

    let mut at_the_origin: Option<TopologySignature> = None;

    for (motion_name, motion) in the_motions() {
        let result = try_bevel_the_bore_mouth(40, motion, SIZE, true);

        assert_eq!(
            section_routing::last_route().as_deref(),
            Some("sections"),
            "the {motion_name} bore mouth is the router's own case"
        );

        let mesh = match result {
            Ok(mesh) => mesh,
            Err(reason) => panic!(
                "{motion_name}: {:?}",
                reason.as_ref().map(BevelAdvisory::code)
            ),
        };

        assert_eq!(surface_mesh_closure::open_edge_count(&mesh).0, 0);
        assert!(most_faces_on_an_edge(&mesh) <= 2);
        assert_eq!(shell_count(&mesh), 1);

        let signature = signature_of(Some(&mesh));

        if motion_name == "identity" {
            at_the_origin = Some(signature);
        } else {
            assert_eq!(
                Some(&signature),
                at_the_origin.as_ref(),
                "{motion_name} published a different topology from the origin's"
            );
        }
    }
}

/// A box's top rim turns ninety degrees at every corner, which is a CORNER and not a curve - and a
/// corner has its own section (`SectionCorner`), so the turn no longer refuses the pass and the rim
/// routes. What the corner's own mitre draws there is read in `SectionCornerTests`; the claim here
/// is only that the router lets it through.
#[test]
fn the_box_top_rims_corners_no_longer_refuse_the_pass() {
    let with_the_flag_off = try_bevel_the_box_top_rim();

    assert_eq!(
        section_routing::last_route().as_deref(),
        Some("legacy: the flag is off")
    );
    if let Err(reason) = &with_the_flag_off {
        panic!("{:?}", reason.as_ref().map(BevelAdvisory::code));
    }

    let _sections = pinned_to_the_sections();

    let with_the_flag_on = try_bevel_the_box_top_rim();

    assert_eq!(
        section_routing::last_route().as_deref(),
        Some("sections"),
        "every one of the rim's four corners draws its mitre"
    );

    if let Err(reason) = &with_the_flag_on {
        panic!("{:?}", reason.as_ref().map(BevelAdvisory::code));
    }
}

/// AN OPEN CHAIN ROUTES WHEN BOTH OF ITS ENDS CAN BE BUILT. The raked bore cube's top mouth arc
/// (`InnerFaceTestMesh`, 40 edges) stops where the mouth crosses the cube's top/-X edge, and the
/// side beyond each of those ends is the -X wall - ONE FLAT region, sharing one crease with each of
/// the arc's two sides - which is exactly what `SectionEnd` draws an End section on. So the router
/// asks `SectionEnd.CanEndAt` at both ends and lets the plan through, at either size.
///
/// Routed is not published: what each size then builds is pinned in
/// `SectionEveryPickMatrixTests`; what is claimed HERE is the router's own answer.
#[test]
fn the_top_mouth_arc_routes_through_the_sections() {
    for size in [0.5, 2.0] {
        let part = inner_face::the_part();
        let graph = part.get_mesh_edge_graph();
        let features =
            FeatureEdgeSet::create_with_graph(&part, &graph, inner_face::BEND_THRESHOLD_DEGREES);

        let arc = the_top_mouth_arc(&part, &features);

        // The predicate the router now asks, read on the arc's own two ends.
        assert!(
            section_end::can_end_at(&part, &graph, &arc[0], true, None).is_ok(),
            "the arc's start dies on the -X wall, one flat region"
        );

        assert!(
            section_end::can_end_at(&part, &graph, &arc[arc.len() - 1], false, None).is_ok(),
            "and so does its finish"
        );

        let _sections = pinned_to_the_sections();

        try_bevel_the_top_mouth_arc(&part, &features, size);

        assert_eq!(
            section_routing::last_route().as_deref(),
            Some("sections"),
            "an open chain whose two ends both build End sections is the sections' own case \
             (size {size})"
        );
    }
}

/// AND EVERY OTHER OPEN CHAIN STILL KEEPS THE SENTENCE. One of the cube's plain edges - the -Y/-Z
/// edge, picked alone - is an open chain of ONE edge, and the router declines it with "a chain is
/// not closed" as it always has.
///
/// The clause it fails is the ENDS clause, not the length one: an open plan is read at its two ends
/// and a chain of one edge cannot be a run at all, so the ends are never asked.
#[test]
fn a_plain_cube_edge_picked_alone_still_does_not_route() {
    let _sections = pinned_to_the_sections();

    let part = inner_face::the_part();
    let graph = part.get_mesh_edge_graph();
    let features =
        FeatureEdgeSet::create_with_graph(&part, &graph, inner_face::BEND_THRESHOLD_DEGREES);

    let picked = the_chain_of(&part, &features, 1, Vector3::new(-9.0, -29.0, 0.0));

    try_bevel_resolutions(
        &part,
        &[picked],
        0.5,
        inner_face::SEGMENTS,
        &CancelToken::new(),
        TryBevelOptions::default(),
    );

    assert_eq!(
        section_routing::last_route().as_deref(),
        Some("legacy: a chain is not closed")
    );
}

/// A chamfer's section is a chord and nothing draws it yet, so no chamfer routes.
#[test]
fn a_chamfer_does_not_route() {
    let _sections = pinned_to_the_sections();

    let _ = try_bevel_the_bore_mouth(40, Matrix4X4::IDENTITY, SIZE, false);

    assert_eq!(
        section_routing::last_route().as_deref(),
        Some("legacy: the operation is a chamfer")
    );
}

/// With the flag off nothing routes, whatever the pass is - the router is not even asked, which is
/// what makes the whole kernel suite's meshes unchanged by this step.
#[test]
fn with_the_flag_off_the_bore_mouth_does_not_route_either() {
    let built = try_bevel_the_bore_mouth(40, Matrix4X4::IDENTITY, SIZE, true);

    if let Err(reason) = &built {
        panic!("{:?}", reason.as_ref().map(BevelAdvisory::code));
    }
    assert_eq!(
        section_routing::last_route().as_deref(),
        Some("legacy: the flag is off")
    );
}

/// The top mouth arc picked as a tangent chain and built through the
/// part builder's pipeline.
fn try_bevel_the_top_mouth_arc(part: &Mesh, features: &FeatureEdgeSet, size: f64) {
    let picked = the_chain_of(part, features, 40, Vector3::new(-18.94, -34.0, 20.0));

    try_bevel_resolutions(
        part,
        &[picked],
        size,
        inner_face::SEGMENTS,
        &CancelToken::new(),
        TryBevelOptions::default(),
    );
}

/// The resolved tangent-chain pick of `edges` edges standing at `lowest` - the same
/// geometric naming `SectionEveryPickMatrixTests` gives its rows, so a pick is named by where it is
/// on the part and not by an index.
pub fn the_chain_of(
    part: &Mesh,
    features: &FeatureEdgeSet,
    edges: usize,
    lowest: Vector3,
) -> BevelEdgeGroupResolution {
    for index in 0..features.count() as i32 {
        let group = bevel_edge_group_resolver::create_from_edge(
            features,
            index,
            BevelSelectionKind::TangentChain,
            FaceGroup::DEFAULT_CURVE_TOLERANCE_DEGREES,
        )
        .expect("every index of the set is a crease of the part");

        let resolution = bevel_edge_group_resolver::resolve(&group, part);

        if !resolution.is_resolved() || resolution.edges.len() != edges {
            continue;
        }

        for edge in &resolution.edges {
            // A one-edge pick is named by its MIDPOINT and a chain by an end of it, so a point that
            // names the pick may be either.
            for at in [edge.start, edge.end, (edge.start + edge.end) / 2.0] {
                if (at - lowest).length() < 0.06 {
                    return resolution;
                }
            }
        }
    }

    panic!("no {edges} edge chain stands at {lowest:?}");
}

/// The bore fixtures' own mouth build, stood by `motion` and picked by
/// the geometric question `SurfaceBevelRobustnessTests.TheUpperMouth` asks, so the pick does not
/// depend on an index surviving the move. The mesh, or the advisory (`None` where nothing was
/// named).
pub(crate) fn try_bevel_the_bore_mouth(
    sides: i32,
    motion: Matrix4X4,
    size: f64,
    round_profile: bool,
) -> Result<Mesh, Option<BevelAdvisory>> {
    // The fixture's default land is 6.
    let mut part =
        create_plate_with_a_raked_bore(sides, robustness::RADIUS, robustness::THICKNESS, 0.0, 6.0);

    move_mesh(&mut part, motion);

    let features = FeatureEdgeSet::create_with_graph(
        &part,
        &part.get_mesh_edge_graph(),
        robustness::BEND_THRESHOLD_DEGREES,
    );

    let group = bevel_edge_group_resolver::create_from_edge(
        &features,
        robustness::the_upper_mouth(&features, motion.inverted(), 0.0),
        BevelSelectionKind::TangentChain,
        FaceGroup::DEFAULT_CURVE_TOLERANCE_DEGREES,
    )
    .expect("the upper mouth is a crease of the part");

    try_bevel(
        &part,
        &[group],
        size,
        robustness::SEGMENTS,
        &CancelToken::new(),
        TryBevelOptions {
            round_profile,
            ..TryBevelOptions::default()
        },
    )
    .result
    .map(|built| built.mesh)
    .map_err(|failure| failure.advisory().cloned())
}

/// "Bevel the top of this box" - one face pick, four rim edges, four
/// square corners.
fn try_bevel_the_box_top_rim() -> Result<Mesh, Option<BevelAdvisory>> {
    let part = platonic_solids::create_cube(20.0, 20.0, 20.0);

    let features = FeatureEdgeSet::create_with_graph(&part, &part.get_mesh_edge_graph(), 10.0);

    let group = bevel_edge_group_resolver::create_from_edge(
        &features,
        the_top_rim(&features),
        BevelSelectionKind::FaceGroup,
        FaceGroup::DEFAULT_CURVE_TOLERANCE_DEGREES,
    )
    .expect("the top rim is a crease of the cube");

    try_bevel(
        &part,
        &[group],
        2.0,
        8,
        &CancelToken::new(),
        TryBevelOptions::default(),
    )
    .result
    .map(|built| built.mesh)
    .map_err(|failure| failure.advisory().cloned())
}

/// An edge of the cube's top rim - both ends on the top face.
fn the_top_rim(features: &FeatureEdgeSet) -> i32 {
    for (i, edge) in features.edges().iter().enumerate() {
        if edge.start.z > 10.0 - 1e-6 && edge.end.z > 10.0 - 1e-6 {
            return i as i32;
        }
    }

    panic!("the cube has no top rim");
}
