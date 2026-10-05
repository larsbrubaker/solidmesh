//! Rust-only checks of the tenth bevel slice (`BevelMixedBlendTool`, `BevelPairCornerReach`). The
//! C# classes that exercise them (`BevelMixedBlendTests`, `BevelConcaveTests`,
//! `BevelPairCornerTrimTests`) read whole bevels through `BevelMixedBlends` and the chain sweep, so
//! they land with those; these pin each piece alone, against the closed forms the C# remarks state.
//! The last check pins a slice-11 round-0 record whose C# initializers are not Rust defaults.

use std::collections::{HashMap, HashSet};

use geometry_processing::bevel::bevel_edge_tool_builder::CONTAINMENT_MARGIN;
use geometry_processing::bevel::bevel_mixed_blend_tool;
use geometry_processing::bevel::bevel_pair_corner_reach::to_partner_axis;
use geometry_processing::topology::{ChainEdge, FeatureEdgeSet};
use polygon_mesh::Mesh;
use vector_math::{Vector3, Vector3Float};

// ---------------------------------------------------------------- BevelMixedBlendTool

/// A square mixed corner: the concave edge runs down -z from the cap face z = 0, its axis through
/// the origin, and the bead's rim turns a quarter from +x to +y.
fn square_corner(segments: i32, run_out: bool) -> bevel_mixed_blend_tool::MixedBlendTool {
    bevel_mixed_blend_tool::build(
        Vector3::new(0.0, 0.0, 0.0),
        Vector3::new(0.0, 0.0, 0.0),
        Vector3::new(0.0, 0.0, -1.0),
        Vector3::new(0.0, 0.0, 1.0),
        Vector3::new(1.0, 0.0, 0.0),
        Vector3::new(0.0, 1.0, 0.0),
        std::f64::consts::FRAC_PI_2,
        1.0,
        segments,
        run_out,
    )
}

/// Every directed edge of every face has exactly one reverse: the mesh is a closed 2-manifold.
fn assert_closed(mesh: &Mesh) {
    let mut directed: HashMap<(i32, i32), i32> = HashMap::new();
    for face in mesh.faces.iter() {
        for (a, b) in [(face.v0, face.v1), (face.v1, face.v2), (face.v2, face.v0)] {
            *directed.entry((a, b)).or_insert(0) += 1;
        }
    }
    for (&(a, b), &count) in &directed {
        assert_eq!(count, 1, "edge {a}->{b} used {count} times");
        assert_eq!(directed.get(&(b, a)), Some(&1), "edge {a}->{b} has no twin");
    }
}

fn signed_volume(mesh: &Mesh) -> f64 {
    let at = |i: i32| {
        let v: Vector3Float = mesh.vertices[i as usize];
        Vector3::new(v.x as f64, v.y as f64, v.z as f64)
    };
    mesh.faces
        .iter()
        .map(|f| at(f.v0).dot(at(f.v1).cross(at(f.v2))) / 6.0)
        .sum()
}

#[test]
fn rust_only_square_corner_reaches_match_the_closed_forms() {
    let segments = 4;
    let half_step = std::f64::consts::FRAC_PI_2 / segments as f64 / 2.0;
    let tool = square_corner(segments, true);

    // Square to the cap face the tangent distance is the radius, the widest point stands
    // r + r / cos(step / 2) from the axis, and the runout reaches sqrt(reach^2 - r^2).
    let reach = 1.0 + 1.0 / half_step.cos();
    assert!((tool.cap_face_reach - 1.0).abs() < 1e-12);
    assert!((tool.void_face_reach - (reach - 1.0)).abs() < 1e-12);
    assert!((tool.runout_reach - (reach * reach - 1.0).sqrt()).abs() < 1e-12);

    // The void face reach is the same with the runout or without it; only the runout reach goes.
    let arc_only = square_corner(segments, false);
    assert_eq!(arc_only.void_face_reach, tool.void_face_reach);
    assert_eq!(arc_only.cap_face_reach, tool.cap_face_reach);
    assert_eq!(arc_only.runout_reach, 0.0);
}

#[test]
fn rust_only_the_runout_adds_whole_stations_past_psi_max() {
    let segments = 4;
    let point_count = (segments + 2) as usize;
    let arc_only = square_corner(segments, false)
        .mesh
        .expect("square corner draws");
    let with_runout = square_corner(segments, true)
        .mesh
        .expect("square corner draws");

    // The arc alone is segments + 1 rings.
    assert_eq!(
        arc_only.vertices.len(),
        (segments as usize + 1) * point_count
    );

    // psi_max = acos(r / reach) is about 60.3 degrees against a 22.5 degree step, so the runout is
    // ceil(2.68) + 1 = 4 further rings.
    assert_eq!(
        with_runout.vertices.len(),
        (segments as usize + 5) * point_count
    );

    for mesh in [&arc_only, &with_runout] {
        assert_closed(mesh);
        // Wound with its normals facing out of the tool.
        assert!(signed_volume(mesh) > 0.0);
        // The tool's top lands exactly on the cap face: nothing stands above z = 0.
        assert!(mesh.vertices.iter().all(|v| v.z <= 1e-6));
    }
}

#[test]
fn rust_only_the_handovers_are_the_planes_through_the_axis_of_the_end_rings() {
    let tool = square_corner(4, false);

    // fromShared x alongEdge = (1,0,0) x (0,0,-1) = (0,1,0); fromVoid gives (-1,0,0).
    assert_eq!(tool.handover.plane_normal, Vector3::new(0.0, 1.0, 0.0));
    assert_eq!(tool.handover.plane_distance, 0.0);
    assert_eq!(tool.far_handover.plane_normal, Vector3::new(-1.0, 0.0, 0.0));
}

#[test]
fn rust_only_a_cap_face_along_the_edge_refuses_but_still_hands_over() {
    // A cap face whose normal is square to the edge folds the wedge shut at one end of the arc and
    // flat open at the other.
    let tool = bevel_mixed_blend_tool::build(
        Vector3::new(0.0, 0.0, 0.0),
        Vector3::new(0.0, 0.0, 0.0),
        Vector3::new(0.0, 0.0, -1.0),
        Vector3::new(1.0, -1.0, 0.0).get_normal(),
        Vector3::new(1.0, 0.0, 0.0),
        Vector3::new(0.0, 1.0, 0.0),
        std::f64::consts::FRAC_PI_2,
        1.0,
        4,
        true,
    );

    // The arc's last station (+y) presents a wedge of 180 degrees, past the bound, so the station
    // list is refused before any reach is read; the handover planes are filled regardless.
    assert!(tool.mesh.is_none());
    assert_eq!(tool.cap_face_reach, 0.0);
    assert_eq!(tool.void_face_reach, 0.0);
    assert_eq!(tool.runout_reach, 0.0);
    assert_eq!(tool.handover.plane_normal, Vector3::new(0.0, 1.0, 0.0));
}

// ---------------------------------------------------------------- BevelPairCornerReach

/// An L-shaped prism one unit tall: outline (0,0) (2,0) (2,1) (1,1) (1,2) (0,2), counter-clockwise
/// from above, with the reflex corner at (1,1).
fn l_prism() -> Mesh {
    let outline = [
        (0.0, 0.0),
        (2.0, 0.0),
        (2.0, 1.0),
        (1.0, 1.0),
        (1.0, 2.0),
        (0.0, 2.0),
    ];
    let n = outline.len() as i32;
    let mut mesh = Mesh::new();
    for z in [0.0, 1.0] {
        for &(x, y) in &outline {
            mesh.vertices.push(Vector3Float::new(x, y, z));
        }
    }
    let top = |i: i32| n + i;
    // Fans from the reflex corner (index 3) are inside the L.
    for (a, b) in [(4, 5), (5, 0), (0, 1), (1, 2)] {
        mesh.faces.add(top(3), top(a), top(b), &mesh.vertices);
        mesh.faces.add(3, b, a, &mesh.vertices);
    }
    for i in 0..n {
        let j = (i + 1) % n;
        mesh.faces.add(i, j, top(j), &mesh.vertices);
        mesh.faces.add(i, top(j), top(i), &mesh.vertices);
    }
    mesh
}

fn vertex_at(mesh: &Mesh, x: f32, y: f32, z: f32) -> i32 {
    mesh.vertices
        .iter()
        .position(|v| v.x == x && v.y == y && v.z == z)
        .expect("vertex exists") as i32
}

/// The chain edge from `from` to `to`, walked that way.
fn chain_edge(features: &FeatureEdgeSet, from: i32, to: i32) -> ChainEdge {
    let (index, edge) = features
        .edges()
        .iter()
        .enumerate()
        .find(|(_, e)| {
            (e.start_vertex == from && e.end_vertex == to)
                || (e.start_vertex == to && e.end_vertex == from)
        })
        .expect("feature edge exists");
    ChainEdge::new(index as i32, *edge, edge.start_vertex != from)
}

fn all_mesh_edges(features: &FeatureEdgeSet) -> HashSet<i32> {
    features.edges().iter().map(|e| e.mesh_edge_index).collect()
}

#[test]
fn rust_only_a_square_partner_at_a_reflex_corner_is_reached_at_the_radius() {
    let mesh = l_prism();
    let features = FeatureEdgeSet::create(&mesh);
    let corner = vertex_at(&mesh, 1.0, 1.0, 1.0);
    let end = chain_edge(&features, vertex_at(&mesh, 2.0, 1.0, 1.0), corner);
    assert_eq!(end.to_vertex(), corner);
    let selected = all_mesh_edges(&features);

    // The partner runs along +y with its ball under the top face at x < 1, which is where this
    // chain is heading: a square wedge puts the axis one radius on, plus the containment margin.
    let reach = to_partner_axis(
        Some(corner),
        &end,
        false,
        Some(&features),
        Some(&selected),
        0.5,
    );
    assert!((reach - 0.5 * CONTAINMENT_MARGIN).abs() < 1e-9, "{reach}");

    // The same edge walked the other way and read as the chain's START leaves by the same way out.
    let reversed = end.reversed();
    assert_eq!(
        to_partner_axis(
            Some(corner),
            &reversed,
            true,
            Some(&features),
            Some(&selected),
            0.5
        ),
        reach
    );
}

#[test]
fn rust_only_no_corner_no_selection_or_partners_behind_reach_nothing() {
    let mesh = l_prism();
    let features = FeatureEdgeSet::create(&mesh);
    let corner = vertex_at(&mesh, 1.0, 1.0, 1.0);
    let end = chain_edge(&features, vertex_at(&mesh, 2.0, 1.0, 1.0), corner);
    let selected = all_mesh_edges(&features);

    assert_eq!(
        to_partner_axis(None, &end, false, Some(&features), Some(&selected), 0.5),
        0.0
    );
    assert_eq!(
        to_partner_axis(Some(corner), &end, false, None, Some(&selected), 0.5),
        0.0
    );
    assert_eq!(
        to_partner_axis(Some(corner), &end, false, Some(&features), None, 0.5),
        0.0
    );

    // Only this end itself selected: no partner.
    let only_self: HashSet<i32> = [end.edge.mesh_edge_index].into_iter().collect();
    assert_eq!(
        to_partner_axis(
            Some(corner),
            &end,
            false,
            Some(&features),
            Some(&only_self),
            0.5
        ),
        0.0
    );

    // At a convex box corner every partner's axis sits back the way the chain came.
    let box_corner = vertex_at(&mesh, 2.0, 0.0, 1.0);
    let rim = chain_edge(&features, vertex_at(&mesh, 0.0, 0.0, 1.0), box_corner);
    assert_eq!(
        to_partner_axis(
            Some(box_corner),
            &rim,
            false,
            Some(&features),
            Some(&selected),
            0.5
        ),
        0.0
    );
}

// ---------------------------------------------------------------- Bevel core round 0 records

#[test]
fn rust_only_a_pick_fault_defaults_to_naming_nothing() {
    use geometry_processing::bevel::level_line::{BandGapRefusal, PickFault};

    // C#'s initializers: every index -1, the refusal its first member.
    let fault = PickFault::default();
    assert_eq!(fault.refusal, BandGapRefusal::default());
    assert_eq!(
        (fault.chain, fault.other, fault.vertex, fault.edge),
        (-1, -1, -1, (-1, -1))
    );
}
