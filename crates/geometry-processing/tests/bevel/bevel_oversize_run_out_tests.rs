//! Ports `GeometryProcessingTests/BevelOversizeRunOutTests.cs`: a round BIGGER than the face it has
//! to land on, run over the nose-in-notch part's top edge - "15mm Bevel Top Edge.mcx", where the
//! Onshape control neither refuses nor shrinks but eats the whole of that face on the cut's side and
//! runs the cylinder out on the walls either end. The pick is the LEFT slope's top edge from the 45
//! degree knife tip at (0, 0, 20) to the centre crease at (10, 10, 20): 14.142 long, shorter than
//! the radius 15. Both old refusals (face width, and `CapPullsToolBackPastSegment`) are about the
//! one plane x = 0 the run ends on, and both are answered by it: the group is admitted on the cap,
//! and the cap is built by the boolean (`bevel_cap_clip`) where the sweep cannot carry it.

use std::rc::Rc;
use std::sync::Arc;

use geometry_processing::bevel::bevel_chain_end_cap::ResolveOptions;
use geometry_processing::bevel::bevel_chain_linker;
use geometry_processing::bevel::bevel_chain_sweep::{self, BevelSweptSection};
use geometry_processing::bevel::bevel_face_fit::DEFAULT_SURFACE_TOLERANCE_DEGREES;
use geometry_processing::bevel::bevel_group_tools::{self, BevelGroupToolsOptions};
use geometry_processing::bevel::{BevelAdvisoryCode, BevelChainEnds, BevelFaceFit};
use geometry_processing::topology::bevel_edge_group_resolver;
use geometry_processing::topology::{
    BevelEdgeGroupResolution, BevelSelectionKind, ChainEdge, FaceGroup, FeatureEdgeSet,
};
use polygon_mesh::Mesh;
use vector_math::Vector3;

use crate::bevel_shadow_body_tests::the_oversize_round;
use crate::kernel_cut_helpers::edge_index;
use crate::mesh_measurements::{shell_count, signed_volume};
use crate::nose_in_notch_test_meshes as nose;

/// The radius the fixture asks for - more than the 14.142 of run it has to fit on.
const RADIUS: f64 = 15.0;
const SEGMENTS: i32 = 16;

/// How far a vertex may be off a plane the shape puts it exactly on.
const ON_IT: f64 = 1e-4;

/// The convex corner the run starts at: the 45 degree knife tip's top vertex.
fn tip_corner() -> Vector3 {
    Vector3::new(0.0, 0.0, 20.0)
}

/// The concave corner the run ends at, on top of the notch's centre crease.
fn crease_corner() -> Vector3 {
    Vector3::new(10.0, 10.0, 20.0)
}

#[test]
fn the_oversize_round_is_admitted_on_the_wall_its_run_ends_on_and_clipped_there() {
    let part = nose::create();
    let resolution = top_edge_of_the_left_slope(&part);
    let check = BevelFaceFit::create(
        Arc::new(part.clone()),
        Rc::new(vec![resolution.clone()]),
        true,
        RADIUS,
        DEFAULT_SURFACE_TOLERANCE_DEGREES,
        None,
    )
    .try_check_group_fully(0);

    assert!(
        check.fits(),
        "the top face runs out onto the side face x = 0, which is the wall this run's own start \
         vertex sits on - so the width it measured is not a bound on the cut, and the refusal it \
         used to carry ({:?}) was measuring the wrong thing",
        check.failure.as_ref().map(|failure| failure.code())
    );
    assert!(
        check.trimmed_by_a_wall_it_ends_on,
        "a group here on the strength of a cap is a group whose cap has to be built at all costs - \
         this is what entitles it to BevelCapClip, and without it the sweep's own \
         CapPullsToolBackPastSegment refusal stands"
    );
    assert_eq!(
        check.standing_on_a_face.len(),
        1,
        "a cut this deep uncovers the buried nose, which stands on the slope and is not the cut's \
         to take"
    );

    // The sweep on its own still cannot carry the cap, which is the whole reason the clip is
    // needed: 15 of drag over 14.142 of run. Pinned rather than assumed.
    let run = one_run(&resolution);
    let ends = ends_of(&resolution, &run);
    let swept = bevel_chain_sweep::try_create(
        &run,
        false,
        Some(&ends),
        RADIUS,
        SEGMENTS,
        true,
        BevelSweptSection::Tool,
        0.0,
        0.0,
        None,
        0.0,
        None,
    );

    match swept {
        Ok(_) => panic!("the cap at the tip pulls the ring back 15 along a run of 14.142"),
        Err(failure) => {
            assert_eq!(
                failure.code(),
                BevelAdvisoryCode::CapPullsToolBackPastSegment
            )
        }
    }

    let tools = bevel_group_tools::try_build(
        &resolution,
        true,
        RADIUS,
        SEGMENTS,
        BevelGroupToolsOptions {
            may_clip_caps: check.trimmed_by_a_wall_it_ends_on,
            ..BevelGroupToolsOptions::default()
        },
    )
    .unwrap_or_else(|refusal| {
        panic!(
            "the clip is what carries the cap the sweep cannot: {:?}",
            refusal.reason.code()
        )
    })
    .tools;

    assert_eq!(tools.len(), 1);
    assert_eq!(
        tools[0].cap_clips.as_ref().map_or(0, |clips| clips.len()),
        1,
        "one end - the tip - is demoted to a clip; the crease end's cap on the right slope is still \
         swept"
    );

    let clip = &tools[0].cap_clips.as_ref().expect("one clip")[0];

    assert!(
        (clip.at - tip_corner()).length() < ON_IT,
        "the demoted end is the knife tip"
    );
    assert!(
        (clip.plane.plane_normal - Vector3::new(-1.0, 0.0, 0.0)).length() < 1e-6,
        "the plane the tool is stopped on is the part's own side face x = 0, pointing the way the \
         sweep leaves the end - which is the face the control's ellipse is trimmed by"
    );
}

/// The finished solid: the top face is gone on the cut's side of the plane `x + y = 20`, the tip's
/// vertical edge is cut down to z = 5, and the run-out at the crease lies IN that plane.
#[test]
fn the_round_eats_the_near_half_of_the_top_face_and_runs_out_on_both_walls() {
    let result = the_oversize_round(true);
    let vertices: Vec<Vector3> = result.vertices.iter().map(|v| v.as_vector3()).collect();

    assert_eq!(
        shell_count(&result),
        1,
        "a cut this big must not sever the part or leave the nose it uncovers floating"
    );
    assert!(
        signed_volume(&result) < nose::VOLUME,
        "one convex round can only ever take material off"
    );

    // The top face: the control leaves only the triangle (0, 20), (20, 0), (20, 20), so nothing of
    // z = 20 may survive on the cut's side of the line x + y = 20.
    let on_top: Vec<Vector3> = vertices
        .iter()
        .copied()
        .filter(|vertex| (vertex.z - 20.0).abs() < ON_IT)
        .collect();

    assert!(
        !on_top.is_empty(),
        "the far triangle of the top face is untouched and has to still be there"
    );
    assert!(
        on_top
            .iter()
            .all(|vertex| vertex.x + vertex.y >= 20.0 - 1e-3),
        "the round eats the whole top face on its own side of x + y = 20, leaving the triangle \
         (0, 20), (20, 0), (20, 20): {:?}",
        on_top
            .iter()
            .map(|vertex| (vertex.x, vertex.y))
            .collect::<Vec<_>>()
    );

    // The tip end. The control does not cap on the corner: it runs over it and is trimmed by
    // x = 0, taking the tip's own vertical edge down to z = 5 with it.
    assert!(
        vertices
            .iter()
            .any(|&vertex| (vertex - Vector3::new(0.0, 0.0, 5.0)).length() < 0.05),
        "the tip's vertical edge at (0, 0) is cut down to z = 5, which is the round's lowest reach - \
         a run that stopped AT the corner would leave that edge whole to z = 20"
    );
    assert!(
        !vertices.iter().any(|vertex| vertex.x < -ON_IT),
        "the clip stops the tool at the side face; nothing may hang outside the part"
    );

    // The crease end. The far cap is the cylinder's own cross section lying in the plane
    // x + y = 20, the full 90 degrees of it from (10, 10, 5) up to (0, 20, 19.975).
    assert!(
        vertices
            .iter()
            .any(|&vertex| (vertex - Vector3::new(10.0, 10.0, 5.0)).length() < 0.05),
        "the arc's low end is on the centre crease at z = 5"
    );
    assert!(
        vertices
            .iter()
            .any(|&vertex| (vertex - Vector3::new(0.0, 20.0, 19.975)).length() < 0.05),
        "and its high end is at the back left top corner, 0.025 below the top face"
    );
}

/// The one-edge group on the left slope's top edge, resolved as an edge pick.
fn top_edge_of_the_left_slope(part: &Mesh) -> BevelEdgeGroupResolution {
    let features = FeatureEdgeSet::create(part);
    let edge = edge_index(&features, tip_corner(), crease_corner(), ON_IT);
    let group = bevel_edge_group_resolver::create_from_edge(
        &features,
        edge,
        BevelSelectionKind::Edge,
        FaceGroup::DEFAULT_CURVE_TOLERANCE_DEGREES,
    )
    .expect("an edge of the set");

    bevel_edge_group_resolver::resolve(&group, part)
}

/// The resolution's single run, head to tail.
fn one_run(resolution: &BevelEdgeGroupResolution) -> Vec<ChainEdge> {
    let mut chains = bevel_chain_linker::link_resolution(resolution);
    assert_eq!(chains.len(), 1, "one chain");
    let mut runs = bevel_chain_linker::split_by_convexity(chains.remove(0));
    assert_eq!(runs.len(), 1, "one run");
    runs.remove(0).edges
}

/// That run's ends as the builder reads them - both capped, which is what the sweep chokes on.
fn ends_of(resolution: &BevelEdgeGroupResolution, run: &[ChainEdge]) -> BevelChainEnds {
    let mut warnings = Vec::new();
    BevelChainEnds::resolve(
        run,
        false,
        RADIUS,
        Some(&resolution.features),
        Some(&resolution.edges),
        Some(&mut warnings),
        ResolveOptions::default(),
    )
}
