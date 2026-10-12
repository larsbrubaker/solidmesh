//! Port of `SectionAssemblyTests.cs`: the vertex-centric construction as ONE mesh - the part's
//! faces rebuilt onto the sections' boundaries and the strips lofted between the sections, laid into
//! one face list over one pool and published (`section_assembly`).
//!
//! **Why closure is the whole reading.** The faces and the strips meet by INDEX: a section's
//! boundary is one pool integer, handed to the faces on that side and used as a rail by the strips
//! on either side of the vertex. So "the result is closed and manifold" is not a statement about
//! tolerances here; a single mismatched index would open it, and no tolerance could close it.

use std::collections::HashSet;

use geometry_processing::bevel::sections::section_builder_run::try_build_chain;
use geometry_processing::bevel::sections::{
    section_assembly, section_face_rebuild, section_loft, SectionPool,
};
use geometry_processing::bevel::surface::surface_mesh_closure;
use geometry_processing::topology::{FeatureEdgeChain, FeatureEdgeSet};
use polygon_mesh::{Mesh, MeshEdgeGraph};
use vector_math::{Matrix4X4, Vector3};

use crate::bore_test_meshes::create_plate_with_a_round_bore;
use crate::{mesh_measurements, rigid_motion_meshes};

const SIDES: i32 = 24;
const BORE_RADIUS: f64 = 3.0;
const THICKNESS: f64 = 10.0;
const RADIUS: f64 = 0.4;
const SEGMENTS: i32 = 8;
const BEND_THRESHOLD_DEGREES: f64 = 20.0;

#[test]
fn the_straight_bores_mouth_assembles_into_one_closed_manifold_solid() {
    let (part, graph, chain) = the_bore(Matrix4X4::IDENTITY);

    let built = match section_assembly::try_build(
        Some(&part),
        Some(&graph),
        Some(&chain),
        RADIUS,
        SEGMENTS,
    ) {
        Ok(built) => built,
        Err(failure) => panic!("the straight bore's mouth assembles - instead {failure:?}"),
    };

    let (open, open_at) = surface_mesh_closure::open_edge_count(&built);
    assert_eq!(
        open, 0,
        "the faces' rails and the strips' rails are the same pool indices - open at {open_at:?}"
    );

    assert_eq!(
        rigid_motion_meshes::most_faces_on_an_edge(&built),
        2,
        "an edge carrying three faces is a fold no later weld can undo"
    );

    assert_eq!(rigid_motion_meshes::two_cornered_triangle_count(&built), 0);

    assert_eq!(
        mesh_measurements::shell_count(&built),
        1,
        "the fillet is part of the plate, not a second shell standing beside it"
    );

    assert!(
        mesh_measurements::signed_volume(&built) > 0.0,
        "the assembled solid faces out"
    );
}

/// Each ring edge of a section boundary loop is used ONCE by a rebuilt face and ONCE by a strip
/// triangle, the other way round - which closure alone could not tell apart from a mesh that is
/// inside out along the rings.
#[test]
fn every_ring_edge_is_used_once_by_a_face_and_once_by_a_strip_the_other_way_round() {
    let (part, graph, chain) = the_bore(Matrix4X4::IDENTITY);
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
    .unwrap_or_else(|why_not| panic!("the bore's sections build - instead {why_not:?}"));

    let faces = section_face_rebuild::try_rebuild_run(&part, &graph, &run, &pool)
        .unwrap_or_else(|why_not| panic!("the bore's faces rebuild - instead {why_not}"));

    let by_the_faces: HashSet<(i32, i32)> = directed_edges(&faces).collect();
    let mut strips = Vec::new();

    if let Err(why_not) = section_loft::try_emit(&run, &pool, &mut strips, None) {
        panic!("the bore's strips loft - instead {why_not}");
    }

    let by_the_strips: HashSet<(i32, i32)> = directed_edges(&strips).collect();

    // The rings are the boundary rails: consecutive sections' boundaries on each side.
    let mut rings = Vec::new();

    for i in 0..run.count() {
        let start = run.start_of(i);
        let end = run
            .end_of(i)
            .expect("a closed run has an end for every edge");

        rings.push((start.left_boundary, end.left_boundary));
        rings.push((start.right_boundary, end.right_boundary));
    }

    for ring in rings {
        let forward_by_a_face = by_the_faces.contains(&ring);
        let backward_by_a_face = by_the_faces.contains(&(ring.1, ring.0));

        assert!(
            forward_by_a_face ^ backward_by_a_face,
            "ring edge {ring:?} is used by exactly one rebuilt face, one way round"
        );

        let forward_by_a_strip = by_the_strips.contains(&ring);
        let backward_by_a_strip = by_the_strips.contains(&(ring.1, ring.0));

        assert!(
            forward_by_a_strip ^ backward_by_a_strip,
            "ring edge {ring:?} is used by exactly one strip triangle, one way round"
        );

        assert_ne!(
            forward_by_a_face, forward_by_a_strip,
            "the face and the strip meeting at {ring:?} walk it the OPPOSITE way, or the surface \
             folds there"
        );
    }
}

/// The first row of the robustness story for the new path: where the part stands changes nothing
/// about what is assembled.
#[test]
fn where_the_part_stands_changes_none_of_the_assembly() {
    let at_the_origin = assemble(Matrix4X4::IDENTITY);
    let signature = rigid_motion_meshes::signature_of(Some(&at_the_origin));

    for (name, motion) in rigid_motion_meshes::the_motions() {
        let moved = assemble(motion);

        let (open, open_at) = surface_mesh_closure::open_edge_count(&moved);
        assert_eq!(
            open, 0,
            "the build moved by {name} is closed - open at {open_at:?}"
        );

        assert_eq!(
            rigid_motion_meshes::signature_of(Some(&moved)),
            signature,
            "the build moved by {name} has the topology it had at the origin"
        );
    }
}

fn assemble(motion: Matrix4X4) -> Mesh {
    let (part, graph, chain) = the_bore(motion);

    section_assembly::try_build(Some(&part), Some(&graph), Some(&chain), RADIUS, SEGMENTS)
        .unwrap_or_else(|failure| {
            panic!("the bore assembles wherever it stands - instead {failure:?}")
        })
}

fn directed_edges(faces: &[(i32, i32, i32)]) -> impl Iterator<Item = (i32, i32)> + '_ {
    faces
        .iter()
        .flat_map(|&(v0, v1, v2)| [(v0, v1), (v1, v2), (v2, v0)])
}

fn the_bore(motion: Matrix4X4) -> (Mesh, MeshEdgeGraph, FeatureEdgeChain) {
    let mut part = create_plate_with_a_round_bore(SIDES, BORE_RADIUS, THICKNESS, 6.0);

    // Named at the origin and then the WHOLE part moved, so the moved build walks the same edges
    // in the same order and any difference downstream is the assembly's own.
    let seed = the_upper_mouth(&FeatureEdgeSet::create_with_graph(
        &part,
        &part.get_mesh_edge_graph(),
        BEND_THRESHOLD_DEGREES,
    ));

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

    (part, graph, chain)
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
