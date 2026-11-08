//! An edge's STRIP is the quad ladder between its two end sections'
//! profiles, and nothing else (`section_loft::try_emit`). On a closed run `end_of(i)` and
//! `start_of(i + 1)` are one object, so the seam between two strips is the same integers read twice
//! and the "used exactly twice, in opposite directions" reading walks across it with no tolerance;
//! the winding is decided once per run and the diagonal by index parity, which the rigid-motion
//! reading pins.

use std::collections::{HashMap, HashSet};

use solidmesh::bevel::sections::section_builder_run::try_build_chain;
use solidmesh::bevel::sections::section_loft;
use solidmesh::bevel::sections::{SectionPool, SectionRun};
use solidmesh::topology::{FeatureEdgeChain, FeatureEdgeSet};
use solidmesh_math::{Matrix4X4, Vector3};

use crate::bore_test_meshes::create_plate_with_a_round_bore;
use crate::rigid_motion_meshes;

const SIDES: i32 = 24;
const BORE_RADIUS: f64 = 3.0;
const THICKNESS: f64 = 10.0;
const RADIUS: f64 = 0.4;
const SEGMENTS: i32 = 8;
const BEND_THRESHOLD_DEGREES: f64 = 20.0;

type Faces = Vec<(i32, i32, i32)>;

struct Built {
    pool: SectionPool,
    run: SectionRun,
}

#[test]
fn the_loft_lays_two_triangles_a_quad_and_mints_nothing() {
    let built = build(Matrix4X4::IDENTITY);
    let mut faces = Faces::new();
    let before = built.pool.count();

    if let Err(failure) = section_loft::try_emit(&built.run, &built.pool, &mut faces, None) {
        panic!("a straight bore's run lofts - instead {failure}");
    }

    assert_eq!(
        faces.len() as i32,
        SIDES * SEGMENTS * 2,
        "twenty four strips of eight quads, each quad split in two"
    );

    assert_eq!(
        built.pool.count(),
        before,
        "a loft joins up points the sections already minted; it has none of its own"
    );

    for &face in &faces {
        let area = area_of(&built.pool, face);

        assert!(
            area > 1e-12,
            "no triangle of a loft is degenerate, and {face:?} has area {area}"
        );
    }
}

/// The reading the construction exists for: every edge INSIDE the loft - up a profile, along a
/// step, and ACROSS the seam between one strip and the next - is used by exactly two triangles and
/// in opposite directions, so the tube is a surface everywhere except the two rows it deliberately
/// leaves for the faces.
#[test]
fn every_interior_edge_is_used_twice_the_other_way_round() {
    let built = build(Matrix4X4::IDENTITY);
    let mut faces = Faces::new();

    let _ = section_loft::try_emit(&built.run, &built.pool, &mut faces, None);

    let mut used: HashMap<(i32, i32), i32> = HashMap::new();

    for directed in directed_edges(&faces) {
        *used.entry(directed).or_insert(0) += 1;
    }

    assert_eq!(
        used.values().copied().max(),
        Some(1),
        "a directed edge laid twice is two faces on one side of it"
    );

    let open: Vec<(i32, i32)> = used
        .keys()
        .copied()
        .filter(|&(from, to)| !used.contains_key(&(to, from)))
        .collect();

    assert_eq!(
        open.len() as i32,
        2 * SIDES,
        "the only edges used once are the two boundary rails, one per section per side"
    );

    // And they are two loops, each walked one way round: every rail vertex leaves once and arrives
    // once, which a loop does and a pair of ragged runs does not.
    let mut leaves: HashMap<i32, i32> = HashMap::new();
    let mut arrives: HashMap<i32, i32> = HashMap::new();

    for &(from, to) in &open {
        *leaves.entry(from).or_insert(0) += 1;
        *arrives.entry(to).or_insert(0) += 1;
    }

    assert!(leaves.values().all(|&count| count == 1));
    assert!(arrives.values().all(|&count| count == 1));

    let mut leaving: Vec<i32> = leaves.keys().copied().collect();
    let mut arriving: Vec<i32> = arrives.keys().copied().collect();
    leaving.sort_unstable();
    arriving.sort_unstable();
    assert_eq!(leaving, arriving);

    let boundaries: HashSet<i32> = built
        .run
        .sections
        .iter()
        .flat_map(|section| [section.left_boundary, section.right_boundary])
        .collect();

    assert!(
        leaves.keys().all(|vertex| boundaries.contains(vertex)),
        "the open rails are the sections' own boundary points, the ones the faces are handed"
    );
}

/// That the strip faces OUT of the material: a rolling ball's surface stands a radius from the
/// centre segment it was swept about, so its normal is the way from that centre to the surface.
#[test]
fn every_triangle_faces_away_from_the_ball_that_swept_it() {
    let built = build(Matrix4X4::IDENTITY);
    let mut faces = Faces::new();

    let _ = section_loft::try_emit(&built.run, &built.pool, &mut faces, None);

    for &face in &faces {
        let at = centroid(&built.pool, face);
        let outward = at - nearest_centre(&built.run, at);
        let normal = normal_of(&built.pool, face);

        assert!(
            normal.dot(outward.get_normal()) > 0.5,
            "the fillet at {at:?} points away from the ball's centre line, not into it"
        );
    }
}

/// That the loft is pure topology over the run: the same bore stood anywhere hands back the same
/// face list, index for index.
#[test]
fn where_the_part_stands_changes_no_face_of_the_loft() {
    let at_the_origin = faces_of(build(Matrix4X4::IDENTITY));

    for (name, motion) in rigid_motion_meshes::the_motions() {
        let moved = faces_of(build(motion));

        assert_eq!(
            moved.len(),
            at_the_origin.len(),
            "the loft moved by {name} lays as many faces as the one at the origin"
        );

        for i in 0..moved.len() {
            assert_eq!(
                moved[i], at_the_origin[i],
                "face {i} of the loft moved by {name} is the face it was at the origin"
            );
        }
    }
}

fn faces_of(built: Built) -> Faces {
    let mut faces = Faces::new();

    if let Err(failure) = section_loft::try_emit(&built.run, &built.pool, &mut faces, None) {
        panic!("the bore's run lofts - instead {failure}");
    }

    faces
}

fn directed_edges(faces: &Faces) -> Vec<(i32, i32)> {
    faces
        .iter()
        .flat_map(|&(v0, v1, v2)| [(v0, v1), (v1, v2), (v2, v0)])
        .collect()
}

fn centroid(pool: &SectionPool, face: (i32, i32, i32)) -> Vector3 {
    (pool.position_of(face.0) + pool.position_of(face.1) + pool.position_of(face.2)) / 3.0
}

fn normal_of(pool: &SectionPool, face: (i32, i32, i32)) -> Vector3 {
    let zero = pool.position_of(face.0);

    (pool.position_of(face.1) - zero)
        .cross(pool.position_of(face.2) - zero)
        .get_normal()
}

fn area_of(pool: &SectionPool, face: (i32, i32, i32)) -> f64 {
    let zero = pool.position_of(face.0);

    (pool.position_of(face.1) - zero)
        .cross(pool.position_of(face.2) - zero)
        .length()
        / 2.0
}

/// Where the rolling ball stood nearest to `at`, over every section's centre segment.
fn nearest_centre(run: &SectionRun, at: Vector3) -> Vector3 {
    let mut best = run.sections[0].centre_from;
    let mut nearest = f64::MAX;

    for section in &run.sections {
        let candidate = closest_on(at, section.centre_from, section.centre_to);
        let reach = (at - candidate).length();

        if reach < nearest {
            nearest = reach;
            best = candidate;
        }
    }

    best
}

fn closest_on(at: Vector3, from: Vector3, to: Vector3) -> Vector3 {
    let run = to - from;
    let length_squared = run.length_squared();
    let along = if length_squared < 1e-24 {
        0.0
    } else {
        ((at - from).dot(run) / length_squared).clamp(0.0, 1.0)
    };

    from + (run * along)
}

fn build(motion: Matrix4X4) -> Built {
    let mut part = create_plate_with_a_round_bore(SIDES, BORE_RADIUS, THICKNESS, 6.0);

    // The seed is named on the part where it was made and then the WHOLE part is moved, so the
    // moved build walks the same edges in the same order and any difference downstream is the
    // loft's own.
    let at_the_origin = FeatureEdgeSet::create_with_graph(
        &part,
        &part.get_mesh_edge_graph(),
        BEND_THRESHOLD_DEGREES,
    );
    let seed = the_upper_mouth(&at_the_origin);

    rigid_motion_meshes::move_mesh(&mut part, motion);

    let graph = part.get_mesh_edge_graph();
    let features = FeatureEdgeSet::create_with_graph(&part, &graph, BEND_THRESHOLD_DEGREES);
    let chain = FeatureEdgeChain::propagate(&features, seed).expect("the mouth propagates");

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

    Built { pool, run }
}

fn the_upper_mouth(features: &FeatureEdgeSet) -> i32 {
    for i in 0..features.count() {
        let edge = &features.edges()[i];

        if on_the_mouth(edge.start) && on_the_mouth(edge.end) {
            return i as i32;
        }
    }

    panic!("the fixture has no upper mouth");
}

fn on_the_mouth(at: Vector3) -> bool {
    (at.z - (THICKNESS / 2.0)).abs() < 1e-6
        && (Vector3::new(at.x, at.y, 0.0).length() - BORE_RADIUS).abs() < 1e-4
}
