//! Port of `SectionFaceRebuildTests.cs`: the part's own faces REBUILT by VERTEX SUBSTITUTION on the
//! mouth of a straight round bore - nothing cut, only the mouth moved, no triangle inverted, every
//! triangle in its own plane, the two boundary rings the only open edges, and a size that folds a
//! face refused at the station.

use std::collections::{HashMap, HashSet};

use geometry_processing::bevel::sections::section_builder_run::try_build_chain;
use geometry_processing::bevel::sections::section_face_rebuild::try_rebuild_run;
use geometry_processing::bevel::sections::{SectionPool, SectionRun};
use geometry_processing::bevel::{BevelAdvisoryArg, BevelAdvisoryCode};
use geometry_processing::topology::{FeatureEdgeChain, FeatureEdgeSet};
use polygon_mesh::{Face, Mesh};
use vector_math::Vector3;

use crate::bore_test_meshes::create_plate_with_a_round_bore;

const SIDES: i32 = 24;
const BORE_RADIUS: f64 = 3.0;
const THICKNESS: f64 = 10.0;
const RADIUS: f64 = 0.4;
const SEGMENTS: i32 = 8;
const BEND_THRESHOLD_DEGREES: f64 = 20.0;

type Triple = (i32, i32, i32);

struct Built {
    part: Mesh,
    pool: SectionPool,
    run: SectionRun,
    faces: Vec<Triple>,
}

/// C# `TheRebuildCutsNothingAndMovesOnlyTheMouth`.
#[test]
fn the_rebuild_cuts_nothing_and_moves_only_the_mouth() {
    let built = build();

    assert_eq!(
        built.faces.len(),
        built.part.faces.len(),
        "substitution rewrites corners; it never cuts a triangle, so the face count cannot change"
    );

    let mouth: HashSet<i32> = built.run.sections.iter().map(|s| s.chain_vertex).collect();

    for face in &built.faces {
        for corner in [face.0, face.1, face.2] {
            assert!(
                !mouth.contains(&corner),
                "every face around a mouth vertex is in one of its two fans, so after the rebuild no \
                 face anywhere still reads the mouth itself"
            );
        }
    }

    // And nothing else moved: every corner that was not on the mouth is the mesh index it always was.
    for face in 0..built.faces.len() {
        assert!(
            same_where_it_was_not_the_mouth(&built.part.faces[face], built.faces[face], &mouth),
            "face {face} may only have had its mouth corners rewritten"
        );
    }
}

/// C# `NoTriangleInverted`: no rewritten triangle turned itself inside out or lost its area.
#[test]
fn no_triangle_inverted() {
    let built = build();

    for face in 0..built.faces.len() {
        let before = twice_the_area(at_part(&built, &built.part.faces[face]));
        let now = twice_the_area(at_pool(&built, built.faces[face]));

        assert!(
            now.length() > 0.0,
            "face {face} still has three corners that are not in a line"
        );
        assert!(
            now.get_normal().dot(before.get_normal()) > 0.0,
            "face {face} still faces the way it did before its corner moved"
        );
    }
}

/// C# `EveryTriangleStaysInItsOwnPlane`: the OFFSET on the top plane and the SLIDE in each of the
/// two wall facets its crease separates.
#[test]
fn every_triangle_stays_in_its_own_plane() {
    let built = build();
    let top = THICKNESS / 2.0;
    let mut on_the_top = 0;

    for face in 0..built.faces.len() {
        let was = &built.part.faces[face];
        let plane = twice_the_area(at_part(&built, was)).get_normal();
        let through = Vector3::from_vector3_float(built.part.vertices[was.v0 as usize]);
        let is_top = (plane.z - 1.0).abs() < 1e-9;
        let now = built.faces[face];

        for corner in [now.0, now.1, now.2] {
            let at = built.pool.position_of(corner);

            assert!(
                (at - through).dot(plane).abs() < 1e-9,
                "face {face}'s corner at {at:?} left the plane it was rebuilt in"
            );

            if is_top {
                assert!(
                    (at.z - top).abs() < 1e-9,
                    "a top face's offset boundary stays on the top plane exactly"
                );
            }
        }

        if is_top {
            on_the_top += 1;
        }
    }

    assert!(on_the_top > 0, "the fixture has a top face to have checked");
}

/// C# `TheOnlyOpenEdgesAreTheTwoBoundaryRings`.
#[test]
fn the_only_open_edges_are_the_two_boundary_rings() {
    let built = build();
    let used = edge_use(built.faces.iter().copied());

    let original: Vec<Triple> = built
        .part
        .faces
        .iter()
        .map(|f| (f.v0, f.v1, f.v2))
        .collect();
    assert_eq!(
        edge_use(original.into_iter())
            .values()
            .filter(|&&c| c == 1)
            .count(),
        0,
        "the plate went in CLOSED, so every open edge below is one the rebuild opened"
    );

    let mut expected: HashSet<(i32, i32)> = HashSet::new();

    for i in 0..SIDES as usize {
        let here = &built.run.sections[i];
        let next = &built.run.sections[(i + 1) % SIDES as usize];

        expected.insert(ordered(here.left_boundary, next.left_boundary));
        expected.insert(ordered(here.right_boundary, next.right_boundary));
    }

    assert_eq!(
        expected.len(),
        2 * SIDES as usize,
        "the two rings are twenty four edges each, all distinct"
    );

    let open: HashSet<(i32, i32)> = used
        .iter()
        .filter(|(_, &count)| count == 1)
        .map(|(&key, _)| key)
        .collect();

    assert_eq!(
        open.len(),
        2 * SIDES as usize,
        "the plate was closed, so the only open edges are the ones the mouth gave up - and {} are open",
        open.len()
    );

    for edge in &expected {
        assert!(
            open.contains(edge),
            "the boundary ring edge {edge:?} is the gap the strip is lofted into, so it is used once"
        );
    }

    for (key, &count) in &used {
        assert!(
            count <= 2,
            "no edge of the rebuild may be used more than twice, and {key:?} is used {count} times"
        );
    }
}

/// C# `ASizeThatFoldsAFaceIsRefusedAtTheStation`.
#[test]
fn a_size_that_folds_a_face_is_refused_at_the_station() {
    let part = create_plate_with_a_round_bore(SIDES, BORE_RADIUS, THICKNESS, 6.0);
    let graph = part.get_mesh_edge_graph();
    let features = FeatureEdgeSet::create_with_graph(&part, &graph, BEND_THRESHOLD_DEGREES);
    let chain = FeatureEdgeChain::propagate(&features, the_upper_mouth(&features)).unwrap();
    let mut pool = SectionPool::new(&part.vertices);

    let too_big = THICKNESS * 1.5;

    let run = match try_build_chain(
        Some(&chain),
        &part,
        &graph,
        &mut pool,
        too_big,
        SEGMENTS,
        true,
    ) {
        Ok(run) => run,
        Err(refused) => panic!("the sections themselves refused {too_big} - {refused:?}"),
    };

    let refused = try_rebuild_run(&part, &graph, &run, &pool)
        .expect_err("a boundary dragged out through the far side of a triangle is not a rebuild");

    assert_eq!(refused.code(), BevelAdvisoryCode::SurfaceCornerNotSupported);

    let BevelAdvisoryArg::Vector3(at) = refused.args()[0] else {
        panic!("the refusal is named at a point, and it says {refused}");
    };

    assert!(
        (at.z - (THICKNESS / 2.0)).abs() < 1e-6,
        "the refusal is named at the mouth vertex whose fan folded, and it says {at:?}"
    );
}

fn edge_use(faces: impl Iterator<Item = Triple>) -> HashMap<(i32, i32), i32> {
    let mut used = HashMap::new();

    for face in faces {
        for (a, b) in [(face.0, face.1), (face.1, face.2), (face.2, face.0)] {
            *used.entry(ordered(a, b)).or_insert(0) += 1;
        }
    }

    used
}

fn ordered(one: i32, other: i32) -> (i32, i32) {
    if one < other {
        (one, other)
    } else {
        (other, one)
    }
}

fn same_where_it_was_not_the_mouth(was: &Face, now: Triple, mouth: &HashSet<i32>) -> bool {
    (mouth.contains(&was.v0) || was.v0 == now.0)
        && (mouth.contains(&was.v1) || was.v1 == now.1)
        && (mouth.contains(&was.v2) || was.v2 == now.2)
}

fn at_part(built: &Built, face: &Face) -> (Vector3, Vector3, Vector3) {
    let at = |v: i32| Vector3::from_vector3_float(built.part.vertices[v as usize]);
    (at(face.v0), at(face.v1), at(face.v2))
}

fn at_pool(built: &Built, face: Triple) -> (Vector3, Vector3, Vector3) {
    (
        built.pool.position_of(face.0),
        built.pool.position_of(face.1),
        built.pool.position_of(face.2),
    )
}

fn twice_the_area(corners: (Vector3, Vector3, Vector3)) -> Vector3 {
    (corners.1 - corners.0).cross(corners.2 - corners.0)
}

fn build() -> Built {
    let part = create_plate_with_a_round_bore(SIDES, BORE_RADIUS, THICKNESS, 6.0);
    let graph = part.get_mesh_edge_graph();
    let features = FeatureEdgeSet::create_with_graph(&part, &graph, BEND_THRESHOLD_DEGREES);
    let chain = FeatureEdgeChain::propagate(&features, the_upper_mouth(&features)).unwrap();

    assert!(
        chain.is_closed() && chain.count() == SIDES as usize,
        "the fixture's mouth walks as a closed loop of {SIDES}, and it walked {} closed {}",
        chain.count(),
        chain.is_closed()
    );

    let mut pool = SectionPool::new(&part.vertices);

    let run = match try_build_chain(
        Some(&chain),
        &part,
        &graph,
        &mut pool,
        RADIUS,
        SEGMENTS,
        true,
    ) {
        Ok(run) => run,
        Err(failure) => panic!("the straight bore's mouth sections - instead {failure:?}"),
    };

    let faces = match try_rebuild_run(&part, &graph, &run, &pool) {
        Ok(faces) => faces,
        Err(failure) => panic!("the straight bore's faces rebuild - instead {failure}"),
    };

    Built {
        part,
        pool,
        run,
        faces,
    }
}

fn the_upper_mouth(features: &FeatureEdgeSet) -> i32 {
    features
        .edges()
        .iter()
        .position(|edge| on_the_mouth(edge.start) && on_the_mouth(edge.end))
        .expect("the fixture has no upper mouth") as i32
}

fn on_the_mouth(at: Vector3) -> bool {
    (at.z - (THICKNESS / 2.0)).abs() < 1e-6
        && (Vector3::new(at.x, at.y, 0.0).length() - BORE_RADIUS).abs() < 1e-4
}
