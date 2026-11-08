//! Where the strip over "UpperLeftInner should bevel.mcx"'s upper left crease leaves the PART at
//! its rim end - the seven rails of a 3 mm round at 8 segments, read against the six planes the
//! cube is bounded by.
//!
//! The band on the left wall runs off it over the last 1.8866 mm of the crease, so there is no face
//! left at that end for the strip to be cut back against: what ends it is the part's own outside.
//! Six rails leave through the recess's MOUTH, y = 0, and the last leaves through the cube's TOP,
//! z = 20 - the design's own table, rail for rail - and the end turns between rails 5 and 6 at one
//! point on the band, X56 = (2.0745, 0, 20). The corner the two pieces of the cap both die at is
//! D = (0, 0, 20), a vertex the part already has, so it is LOOKED UP.

use std::rc::Rc;
use std::sync::Arc;

use solidmesh::bevel::bevel_chain_linker;
use solidmesh::bevel::surface::{
    surface_chain_join, surface_room_measure, SurfaceBandRunsOffOverAStretch, SurfaceBevelPlan,
    SurfaceEdgePlan, SurfaceLine, SurfaceStrip, SurfaceStripEndsOnTheHull,
    SurfaceStripEndsOnTheHullSupportingPlane, SurfaceVertexPool,
};
use solidmesh::bevel::BevelAdvisoryCode;
use solidmesh::topology::{FaceRegionCache, FeatureEdge, FeatureEdgeSet};
use solidmesh_math::Vector3;
use solidmesh_mesh::Mesh;

use crate::pyramid_recess_test_meshes;

const RADIUS: f64 = 3.0;

const SEGMENTS: i32 = 8;

/// The design's own table, rail by rail: how far back from the rim end each rail leaves the part,
/// through which plane, and where.
fn the_table() -> [(f64, Vector3, Vector3); 7] {
    [
        (1.8866, -Vector3::UNIT_Y, Vector3::new(0.0, 0.0, 17.1306)),
        (1.3755, -Vector3::UNIT_Y, Vector3::new(0.0724, 0.0, 17.9335)),
        (0.9733, -Vector3::UNIT_Y, Vector3::new(0.2865, 0.0, 18.6438)),
        (0.6975, -Vector3::UNIT_Y, Vector3::new(0.6331, 0.0, 19.2308)),
        (0.5600, -Vector3::UNIT_Y, Vector3::new(1.0972, 0.0, 19.6692)),
        (0.5666, -Vector3::UNIT_Y, Vector3::new(1.6587, 0.0, 19.9400)),
        (0.7645, Vector3::UNIT_Z, Vector3::new(2.3092, 0.0316, 20.0)),
    ]
}

/// Where the end turns from the mouth plane to the top face, on the band between rails 5 and 6.
fn between() -> Vector3 {
    Vector3::new(2.0745, 0.0, 20.0)
}

/// The part is bounded by its own cube and by nothing of the recess cut into it.
#[test]
fn the_supporting_planes_are_the_cubes_six_faces() {
    let read = the_crease();
    let planes =
        SurfaceStripEndsOnTheHull::supporting_planes_of(&read.part, read.pool.near_enough());

    assert_eq!(
        planes.len(),
        6,
        "a cube has six, and every face of the recess has part on both sides of it - instead {}",
        planes.len()
    );

    for plane in &planes {
        let axis = plane.normal.x.abs() + plane.normal.y.abs() + plane.normal.z.abs();
        let offset = plane.at.dot(plane.normal);

        assert!(
            (axis - 1.0).abs() <= 1e-9,
            "each is square to an axis - instead {:?}",
            plane.normal
        );

        assert!(
            offset.abs() < 1e-9 || (offset - pyramid_recess_test_meshes::SIZE).abs() < 1e-9,
            "and stands at 0 or at 20 - instead {offset} along {:?}",
            plane.normal
        );
    }
}

/// The whole reading at the rim end: every rail's plane and exit, the turn between the two of
/// them, and the corner it dies at - looked up in the part rather than minted.
#[test]
fn the_rim_end_leaves_through_the_mouth_and_the_top_face() {
    let read = the_crease();
    let planes =
        SurfaceStripEndsOnTheHull::supporting_planes_of(&read.part, read.pool.near_enough());
    let minted = read.pool.vertices().len();

    assert_eq!(
        read.strip.rail_count(),
        7,
        "71.565 degrees of sweep at 8 segments a quarter is 6 chords - instead {}",
        read.strip.rail_count()
    );

    let ends = SurfaceStripEndsOnTheHull::try_read(
        &read.pool,
        &read.strip,
        &read.edge,
        read.the_rim_is_the_edges_start,
        &planes,
        Some(read.knife_line),
    )
    .unwrap_or_else(|failure| {
        panic!(
            "the end really does leave through two of the part's planes - instead {:?}",
            failure.code()
        )
    });

    let length = (read.edge.to() - read.edge.from()).dot(read.strip.along());
    let table = the_table();

    for rail in 0..read.strip.rail_count() {
        // The design's table is walked from the LEFT wall's tangent rail in, which is rail zero's
        // own order (SurfaceStrip rules the left tangent first).
        let expected = table[rail as usize];
        let exit = ends.exit_at[rail as usize];
        let back = if read.the_rim_is_the_edges_start {
            exit
        } else {
            length - exit
        };

        assert!(
            (back - expected.0).abs() <= 1e-3,
            "rail {rail} leaves {} back from the rim end - instead {back}",
            expected.0
        );

        assert!(
            (ends.plane_of[rail as usize].normal.dot(expected.1) - 1.0).abs() <= 1e-6,
            "and through the plane {:?} - instead {:?}",
            expected.1,
            ends.plane_of[rail as usize].normal
        );

        assert!(
            (read.strip.point_on(rail, exit) - expected.2).length() < 1e-3,
            "at {:?} - instead {:?}",
            expected.2,
            read.strip.point_on(rail, exit)
        );
    }

    assert_eq!(
        ends.changes_after, 5,
        "six rails leave through the mouth and the last through the top - instead it changes after {}",
        ends.changes_after
    );

    assert!(
        (ends.between - between()).length() < 1e-3,
        "and the end turns where the band crosses the cube's front-top edge - instead {:?}",
        ends.between
    );

    assert_eq!(
        read.pool.position_of(ends.apex),
        Vector3::new(0.0, 0.0, pyramid_recess_test_meshes::SIZE),
        "the two planes meet the knife edge at the cube's front-top-left corner - instead {:?}",
        read.pool.position_of(ends.apex)
    );

    assert!(
        (ends.apex as usize) < read.part.vertices.len(),
        "which is a vertex the PART has, looked up rather than minted"
    );

    assert_eq!(
        read.pool.vertices().len(),
        minted,
        "and the reading mints nothing at all - it emits no faces and adds no points"
    );
}

/// A rail that leaves through none of the planes, or leaves outside the edge's own span, is refused
/// by name: the strip's APEX end runs out over the lower slope and reaches no plane of the hull
/// inside its span at all, and so does every rail with nothing to leave through.
#[test]
fn an_end_with_no_exit_inside_the_span_is_refused_by_name() {
    for with_no_planes_at_all in [true, false] {
        let read = the_crease();
        let planes = if with_no_planes_at_all {
            Vec::new()
        } else {
            SurfaceStripEndsOnTheHull::supporting_planes_of(&read.part, read.pool.near_enough())
        };

        let failure = SurfaceStripEndsOnTheHull::try_read(
            &read.pool,
            &read.strip,
            &read.edge,
            !read.the_rim_is_the_edges_start,
            &planes,
            Some(read.knife_line),
        )
        .expect_err("the apex end leaves through nothing");

        assert_eq!(
            failure.code(),
            BevelAdvisoryCode::SurfaceHullEndNotSupported,
            "there is nothing at that end for the rails to leave through inside the span - instead {:?} ({with_no_planes_at_all})",
            failure.code()
        );
    }
}

/// And an end over THREE planes is refused by name. Driven directly, with one plane more than the
/// part has: a plane a fifth of a millimetre in front of the cube's left face takes the first two
/// rails, the mouth keeps the next four and the top face keeps the last.
#[test]
fn an_end_over_three_planes_is_refused_by_name() {
    let read = the_crease();
    let mut planes =
        SurfaceStripEndsOnTheHull::supporting_planes_of(&read.part, read.pool.near_enough());
    planes.push(SurfaceStripEndsOnTheHullSupportingPlane::new(
        Vector3::new(0.2, 0.0, 0.0),
        -Vector3::UNIT_X,
    ));

    let failure = SurfaceStripEndsOnTheHull::try_read(
        &read.pool,
        &read.strip,
        &read.edge,
        read.the_rim_is_the_edges_start,
        &planes,
        Some(read.knife_line),
    )
    .expect_err("three planes end the staircase");

    assert_eq!(
        failure.code(),
        BevelAdvisoryCode::SurfaceHullEndOnThreePlanes,
        "three planes end one staircase - instead {:?}",
        failure.code()
    );
}

/// And a knife edge that does not pass through the corner the two planes make leaves no apex, so
/// the end is refused rather than closed on whatever vertex of the part happened to stand near the
/// crossing. Driven directly with a line running up through NEITHER corner - a millimetre in front
/// of the mouth - which meets each plane somewhere the other one is not.
#[test]
fn a_knife_edge_that_misses_the_corner_leaves_no_apex() {
    let read = the_crease();
    let planes =
        SurfaceStripEndsOnTheHull::supporting_planes_of(&read.part, read.pool.near_enough());
    let misses = SurfaceLine::new(Vector3::new(0.0, 1.0, 19.0), Vector3::UNIT_Z);

    let failure = SurfaceStripEndsOnTheHull::try_read(
        &read.pool,
        &read.strip,
        &read.edge,
        read.the_rim_is_the_edges_start,
        &planes,
        Some(misses),
    )
    .expect_err(
        "that line crosses the top face a millimetre in front of the mouth, so the two planes do not meet on it",
    );

    assert_eq!(
        failure.code(),
        BevelAdvisoryCode::SurfaceHullEndNotSupported,
        "named as an end with no corner of the part to die on - instead {:?}",
        failure.code()
    );
}

/// And an end whose two planes do not come in two runs is refused by name. The mouth's own table
/// dips in the middle - rails 4 and 5 leave 0.560 and 0.567 back where rail 6 leaves 0.717 - so a
/// plane square to the edge 0.58 back takes exactly those two and hands the rest back: three runs
/// over two planes, which turns no one corner.
#[test]
fn an_end_that_crosses_the_corner_twice_is_refused_by_name() {
    let read = the_crease();
    let mouths: Vec<SurfaceStripEndsOnTheHullSupportingPlane> =
        SurfaceStripEndsOnTheHull::supporting_planes_of(&read.part, read.pool.near_enough())
            .into_iter()
            .filter(|plane| plane.normal.dot(-Vector3::UNIT_Y) > 0.999)
            .collect();
    assert_eq!(mouths.len(), 1, "the part has one mouth plane");

    let planes = vec![
        mouths[0],
        SurfaceStripEndsOnTheHullSupportingPlane::new(
            read.edge.to() - (read.strip.along() * 0.58),
            read.strip.along(),
        ),
    ];

    let failure = SurfaceStripEndsOnTheHull::try_read(
        &read.pool,
        &read.strip,
        &read.edge,
        read.the_rim_is_the_edges_start,
        &planes,
        Some(read.knife_line),
    )
    .expect_err("the mouth's run comes back");

    assert_eq!(
        failure.code(),
        BevelAdvisoryCode::SurfaceHullEndCrossesTheCorner,
        "the mouth's run comes back after the square plane's - instead {:?}",
        failure.code()
    );
}

/// One reading of the fixture, with everything the end needs to be asked about.
struct Reading {
    part: Mesh,
    pool: SurfaceVertexPool,
    edge: SurfaceEdgePlan,
    strip: SurfaceStrip,
    /// Whether the chain was linked rim first, which is what tells the end this is all about from
    /// the apex end. It is the chain's WALK and nothing about the shape.
    the_rim_is_the_edges_start: bool,
    knife_line: SurfaceLine,
}

/// The recess's upper left crease, planned and ruled at 3 mm over 8 segments a quarter, with the
/// part's own vertices in the pool and the knife edge the left wall runs off.
fn the_crease() -> Reading {
    let part =
        pyramid_recess_test_meshes::create_from_the_fixtures_own_triangles(Vector3::ZERO, false);
    let regions = Arc::new(FaceRegionCache::new(
        Arc::new(part.clone()),
        Some(part.get_mesh_edge_graph()),
        FeatureEdge::default_flat_angle_degrees(),
    ));
    let features = FeatureEdgeSet::create(&part);

    let rim_end = pyramid_recess_test_meshes::rim_end();
    let apex_end = pyramid_recess_test_meshes::apex_end();
    let creases: Vec<FeatureEdge> = features
        .edges()
        .iter()
        .filter(|edge| {
            ((edge.start - rim_end).length() < 1e-3 && (edge.end - apex_end).length() < 1e-3)
                || ((edge.end - rim_end).length() < 1e-3 && (edge.start - apex_end).length() < 1e-3)
        })
        .copied()
        .collect();
    assert_eq!(creases.len(), 1, "the fixture has one upper left crease");

    let mut chains = surface_chain_join::join(&bevel_chain_linker::link(&creases));
    assert_eq!(chains.len(), 1, "one crease is one chain");
    let chain = chains.remove(0);

    let plan: Rc<SurfaceBevelPlan> = SurfaceBevelPlan::try_create(&chain, &regions, RADIUS, true)
        .unwrap_or_else(|reason| panic!("the crease has to plan at {RADIUS}: {:?}", reason.code()));

    let edge = plan.edges[0].clone();

    let strip = SurfaceStrip::try_create(&edge, SEGMENTS)
        .unwrap_or_else(|reason| panic!("and it has to rule: {:?}", reason.code()));

    // The knife edge is the room reading's own: the rim the left wall's band runs off over the
    // stretch at the rim end (SurfaceBandRunsOffOverAStretch).
    let mut stretches: Vec<SurfaceBandRunsOffOverAStretch> = Vec::new();

    surface_room_measure::measure(std::slice::from_ref(&plan), None, Some(&mut stretches));

    assert_eq!(stretches.len(), 1, "one stretch runs off");
    let knife = stretches[0].knife_line;

    Reading {
        pool: SurfaceVertexPool::new(&part.vertices),
        the_rim_is_the_edges_start: (edge.from() - rim_end).length() < 1e-6,
        knife_line: SurfaceLine::new(knife.start, knife.direction),
        part,
        edge,
        strip,
    }
}
