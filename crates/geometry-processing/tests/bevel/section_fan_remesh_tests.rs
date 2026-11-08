//! The fixture the ears re-mesh exists for - the same straight
//! round bore `SectionFaceRebuildTests` rebuilds, with its top face re-triangulated so that ONE
//! mouth vertex stands behind a chord its own boundary walks straight past.
//!
//! **The shape.** Two extra top points are put at `BORE_RADIUS + 0.2`, two degrees either side of
//! one mouth vertex M, and the two quads of the top annulus around M are drawn again through them.
//! M's top fan is then three triangles - one on the chord in front of it and two thin flanks - and
//! the 0.4 setback moves M out to about `r + 0.4`, past all three.
//!
//! **What that proves that one fan could not.** Each flank triangle carries a SECOND mouth vertex
//! as a corner, so three fans fold and they share faces: exactly the case that refused before the
//! re-mesh took the UNION of the folding fans as one region. The extra points come out on that
//! union's own outline - the triangles between them and the rim are in no fan - so the ring carries
//! them and no interior point has to be split back in here.

use std::collections::{HashMap, HashSet};

use solidmesh::bevel::sections::section_builder_run::try_build_chain;
use solidmesh::bevel::sections::{section_assembly, section_face_rebuild, SectionPool, SectionRun};
use solidmesh::bevel::surface::surface_mesh_closure;
use solidmesh::topology::{FeatureEdgeChain, FeatureEdgeSet};
use solidmesh_math::{math_helper, Vector3, Vector3Float};
use solidmesh_mesh::Mesh;

use crate::bore_test_meshes::create_plate_with_a_round_bore;
use crate::{mesh_measurements, rigid_motion_meshes};

const SIDES: i32 = 24;
const BORE_RADIUS: f64 = 3.0;
const THICKNESS: f64 = 10.0;
const LAND: f64 = 6.0;
/// How far outside the bore the two extra top points stand.
const EXTRA_POINT_OUT: f64 = 0.2;
/// How far round from the mouth vertex they stand - small, so the flanks are thin too.
const EXTRA_POINT_DEGREES: f64 = 2.0;
const RADIUS: f64 = 0.4;
const SEGMENTS: i32 = 8;
const BEND_THRESHOLD_DEGREES: f64 = 20.0;

type Triple = (i32, i32, i32);

struct Built {
    part: Mesh,
    pool: SectionPool,
    run: SectionRun,
    faces: Vec<Triple>,
    extra: Vec<i32>,
}

/// The whole point: what refused before, because two folding fans shared a face, now rebuilds -
/// and rebuilds by DRAWING AGAIN, which is the only thing in the construction that changes the face
/// count.
#[test]
fn the_shared_folding_fans_are_drawn_again_as_one_region() {
    let built = build();

    assert_ne!(
        built.faces.len(),
        built.part.faces.len(),
        "the fixture folds faces, so the ears drew their region again and the count moved"
    );

    let mouth: HashSet<i32> = built.run.sections.iter().map(|s| s.chain_vertex).collect();

    for face in &built.faces {
        for corner in corners(face) {
            assert!(
                !mouth.contains(&corner),
                "no face of the result may still read a mouth vertex, re-meshed or not"
            );
        }
    }
}

/// Nothing came out inside out, and the top stayed flat: every triangle whose corners are all on
/// the top plane faces up, and no corner of a face that was on the top plane left it.
#[test]
fn nothing_is_folded_and_the_top_stays_flat() {
    let built = build();
    let top = THICKNESS / 2.0;
    let mut on_the_top = 0;

    for face in &built.faces {
        let at: Vec<Vector3> = corners(face)
            .iter()
            .map(|&corner| built.pool.position_of(corner))
            .collect();
        let twice_the_area = (at[1] - at[0]).cross(at[2] - at[0]);

        assert!(
            twice_the_area.length() > 0.0,
            "the triangle at {:?} still has three corners that are not in a line",
            at[0]
        );

        if !at.iter().all(|corner| (corner.z - top).abs() < 1e-6) {
            continue;
        }

        on_the_top += 1;

        for corner in &at {
            assert!(
                (corner.z - top).abs() < 1e-9,
                "a top triangle's corner at {corner:?} left the top plane"
            );
        }

        assert!(
            twice_the_area.get_normal().z > 0.0,
            "the top triangle at {:?} came out facing down, which is a fold",
            at[0]
        );
    }

    assert!(on_the_top > 0, "the fixture has a top to have checked");
}

/// What the re-mesh may and may not drop. The two extra points stand at `r + 0.2` and the setback
/// is 0.4, so the fillet has MOVED OVER them - a point the strip now covers is not a point of the
/// result, and leaving it out is the answer rather than a refusal. Every other point of the
/// re-meshed region, which the boundary did not reach, is still there.
#[test]
fn only_the_points_the_fillet_covered_are_left_out() {
    let built = build();
    let used: HashSet<i32> = built.faces.iter().flat_map(corners).collect();

    let mouth: HashSet<i32> = built.run.sections.iter().map(|s| s.chain_vertex).collect();
    let setback: HashSet<i32> = built.extra.iter().copied().collect();

    for point in 0..built.part.vertices.len() as i32 {
        if mouth.contains(&point) || setback.contains(&point) {
            continue;
        }

        assert!(
            used.contains(&point),
            "the point at {:?} is outside the fillet, so the re-mesh may not drop it",
            built.pool.position_of(point)
        );
    }

    for &extra in &built.extra {
        let at = built.pool.position_of(extra);
        let band = BORE_RADIUS + (RADIUS / (std::f64::consts::PI / 4.0).tan());

        assert!(
            Vector3::new(at.x, at.y, 0.0).length() < band,
            "the extra point at {at:?} is inside the setback, which is why the strip covers it"
        );

        assert!(
            !used.contains(&extra),
            "the extra point at {at:?} is under the fillet and is not a point of the result"
        );
    }
}

/// The room for the strip is still exactly the two boundary rings, re-mesh or no re-mesh: a
/// region drawn again must cover the same piece of the part its faces did.
#[test]
fn the_only_open_edges_are_still_the_two_boundary_rings() {
    let built = build();
    let mut used: HashMap<(i32, i32), i32> = HashMap::new();

    for face in &built.faces {
        for (a, b) in sides3(face) {
            *used.entry(ordered(a, b)).or_insert(0) += 1;
        }
    }

    let mut expected = HashSet::new();

    for i in 0..SIDES as usize {
        let here = &built.run.sections[i];
        let next = &built.run.sections[(i + 1) % SIDES as usize];

        expected.insert(ordered(here.left_boundary, next.left_boundary));
        expected.insert(ordered(here.right_boundary, next.right_boundary));
    }

    let open: HashSet<(i32, i32)> = used
        .iter()
        .filter(|(_, &count)| count == 1)
        .map(|(&key, _)| key)
        .collect();

    assert_eq!(
        open.len(),
        2 * SIDES as usize,
        "the plate went in closed and only the mouth opened - {} edges are open",
        open.len()
    );

    for edge in &expected {
        assert!(
            open.contains(edge),
            "the boundary ring edge {edge:?} is the gap the strip is lofted into"
        );
    }

    for (key, &count) in &used {
        assert!(
            count <= 2,
            "{key:?} is used {count} times, and no edge of a surface may be"
        );
    }
}

/// And the whole build through `section_assembly` is still one closed manifold solid: the re-mesh
/// handed its triangles back over the SAME pool, so the strips still meet the faces by index.
#[test]
fn the_region_re_mesh_still_assembles_into_one_closed_solid() {
    let (part, extra) = the_fixture();
    let graph = part.get_mesh_edge_graph();
    let features = FeatureEdgeSet::create_with_graph(&part, &graph, BEND_THRESHOLD_DEGREES);
    let chain = FeatureEdgeChain::propagate(&features, the_upper_mouth(&features))
        .expect("the mouth propagates");

    let assembled =
        section_assembly::try_build(Some(&part), Some(&graph), Some(&chain), RADIUS, SEGMENTS)
            .unwrap_or_else(|failure| {
                panic!("the re-meshed region assembles - instead {failure:?}")
            });

    let (open, open_at) = surface_mesh_closure::open_edge_count(&assembled);
    assert_eq!(
        open, 0,
        "the re-mesh covered its region exactly - open at {open_at:?}"
    );

    assert_eq!(
        rigid_motion_meshes::most_faces_on_an_edge(&assembled),
        2,
        "an edge carrying three faces is a fold no later weld can undo"
    );

    assert_eq!(
        mesh_measurements::shell_count(&assembled),
        1,
        "the fillet is part of the plate, not a second shell standing beside it"
    );

    assert!(
        mesh_measurements::signed_volume(&assembled) > 0.0,
        "the assembled solid faces out"
    );

    assert_eq!(extra.len(), 2);
}

/// The plate with a round bore, its top annulus re-triangulated through two extra points either
/// side of the mouth vertex nearest the x axis. Everything below the top plane is the mesh's own.
/// Returns the mesh and the indices of the two points that were added.
fn the_fixture() -> (Mesh, Vec<i32>) {
    let part = create_plate_with_a_round_bore(SIDES, BORE_RADIUS, THICKNESS, LAND);
    let top = THICKNESS / 2.0;
    let step = 2.0 * std::f64::consts::PI / SIDES as f64;

    // The mouth vertex the extra points stand either side of, and its neighbours round the mouth
    // and round the rim - all read by where they stand, so the revolve's own ordering is not
    // assumed.
    let here = nearest_to(&part, BORE_RADIUS, 0.0, top);
    let mouth_next = nearest_to(&part, BORE_RADIUS, step, top);
    let mouth_previous = nearest_to(
        &part,
        BORE_RADIUS,
        -2.0 * std::f64::consts::PI / SIDES as f64,
        top,
    );
    let rim_here = nearest_to(&part, BORE_RADIUS + LAND, 0.0, top);
    let rim_next = nearest_to(&part, BORE_RADIUS + LAND, step, top);
    let rim_previous = nearest_to(
        &part,
        BORE_RADIUS + LAND,
        -2.0 * std::f64::consts::PI / SIDES as f64,
        top,
    );

    let replaced: HashSet<i32> = [
        here,
        mouth_next,
        mouth_previous,
        rim_here,
        rim_next,
        rim_previous,
    ]
    .into_iter()
    .collect();

    let mut vertices: Vec<Vector3Float> = part.vertices.clone();

    let turn = math_helper::degrees_to_radians(EXTRA_POINT_DEGREES);

    let ahead = vertices.len() as i32;
    vertices.push(at(BORE_RADIUS + EXTRA_POINT_OUT, turn, top));

    let behind = vertices.len() as i32;
    vertices.push(at(BORE_RADIUS + EXTRA_POINT_OUT, -turn, top));

    let extra = vec![ahead, behind];

    let mut faces: Vec<Triple> = Vec::new();
    let mut dropped = 0;

    for drawn in part.faces.iter() {
        if replaced.contains(&drawn.v0)
            && replaced.contains(&drawn.v1)
            && replaced.contains(&drawn.v2)
            && on_the_top_plane(&part, [drawn.v0, drawn.v1, drawn.v2], top)
        {
            dropped += 1;

            continue;
        }

        faces.push((drawn.v0, drawn.v1, drawn.v2));
    }

    assert_eq!(
        dropped, 4,
        "the two top quads either side of the mouth vertex are four triangles, and {dropped} were found"
    );

    // The mouth vertex's own fan: the chord in front of it, and a thin flank to each neighbour.
    add_facing_up(&mut faces, &vertices, here, ahead, behind);
    add_facing_up(&mut faces, &vertices, here, mouth_next, ahead);
    add_facing_up(&mut faces, &vertices, here, behind, mouth_previous);

    // And the rest of the two quads, fanned from the rim point between them, which sees all of it.
    add_facing_up(
        &mut faces,
        &vertices,
        rim_here,
        rim_previous,
        mouth_previous,
    );
    add_facing_up(&mut faces, &vertices, rim_here, mouth_previous, behind);
    add_facing_up(&mut faces, &vertices, rim_here, behind, ahead);
    add_facing_up(&mut faces, &vertices, rim_here, ahead, mouth_next);
    add_facing_up(&mut faces, &vertices, rim_here, mouth_next, rim_next);

    let mut built = Mesh::new();

    built.vertices = vertices;

    for (v0, v1, v2) in faces {
        built.faces.add(v0, v1, v2, &built.vertices);
    }

    built.mark_as_changed();

    (built, extra)
}

fn on_the_top_plane(part: &Mesh, corners: [i32; 3], top: f64) -> bool {
    corners
        .iter()
        .all(|&corner| (part.vertices[corner as usize].z as f64 - top).abs() <= 1e-6)
}

/// Adds a triangle of the top wound so that it faces UP, which is how every face of the top
/// annulus is wound and so is the only thing the replacement has to get right.
fn add_facing_up(faces: &mut Vec<Triple>, vertices: &[Vector3Float], a: i32, b: i32, c: i32) {
    let position = |i: i32| Vector3::from_vector3_float(vertices[i as usize]);
    let at = position(a);
    let twice_the_area = (position(b) - at).cross(position(c) - at);

    if twice_the_area.z <= 0.0 {
        faces.push((a, c, b));
    } else {
        faces.push((a, b, c));
    }
}

fn at(radius: f64, turn: f64, z: f64) -> Vector3Float {
    Vector3Float::new(radius * turn.cos(), radius * turn.sin(), z)
}

/// The mesh vertex nearest a place named in cylindrical terms.
fn nearest_to(part: &Mesh, radius: f64, turn: f64, z: f64) -> i32 {
    let wanted = Vector3::from_vector3_float(at(radius, turn, z));
    let mut found = -1;
    let mut nearest = f64::MAX;

    for (i, &v) in part.vertices.iter().enumerate() {
        let away = (Vector3::from_vector3_float(v) - wanted).length_squared();

        if away < nearest {
            nearest = away;
            found = i as i32;
        }
    }

    assert!(
        nearest <= 1e-6,
        "the fixture has no vertex at {wanted:?} - the nearest is {} away",
        nearest.sqrt()
    );

    found
}

fn corners(face: &Triple) -> [i32; 3] {
    [face.0, face.1, face.2]
}

fn sides3(face: &Triple) -> [(i32, i32); 3] {
    [(face.0, face.1), (face.1, face.2), (face.2, face.0)]
}

fn ordered(one: i32, other: i32) -> (i32, i32) {
    if one < other {
        (one, other)
    } else {
        (other, one)
    }
}

fn build() -> Built {
    let (part, extra) = the_fixture();
    let graph = part.get_mesh_edge_graph();
    let features = FeatureEdgeSet::create_with_graph(&part, &graph, BEND_THRESHOLD_DEGREES);
    let chain = FeatureEdgeChain::propagate(&features, the_upper_mouth(&features))
        .expect("the mouth propagates");

    assert!(
        chain.is_closed() && chain.count() == SIDES as usize,
        "the fixture's mouth walks as a closed loop of {SIDES}, and it walked {} closed {}",
        chain.count(),
        chain.is_closed()
    );

    let mut pool = SectionPool::new(&part.vertices);

    let run = try_build_chain(
        Some(&chain),
        &part,
        &graph,
        &mut pool,
        RADIUS,
        SEGMENTS,
        true,
    )
    .unwrap_or_else(|failure| panic!("the fixture's mouth sections - instead {failure:?}"));

    let faces = section_face_rebuild::try_rebuild_run(&part, &graph, &run, &pool)
        .unwrap_or_else(|failure| panic!("the fixture's faces rebuild - instead {failure:?}"));

    Built {
        part,
        pool,
        run,
        faces,
        extra,
    }
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
