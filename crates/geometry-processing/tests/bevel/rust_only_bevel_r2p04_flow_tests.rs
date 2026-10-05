//! Rust-only checks of the bevel core's R2.P04 tagging and flow readings - a point's side of a
//! chain, a both-sided member's mixed corner, the band's candidate triangles, the end region's flat
//! patch and its flow over a narrow end face, the rounds' band, and the foreign room's bracket and
//! reading. The least queries are in `rust_only_bevel_r2p04_tests.rs`.

use std::rc::Rc;

use geometry_processing::bevel::level_line::ChainField;
use vector_math::Vector3;

#[test]
fn rust_only_a_points_side_of_a_chain_and_a_both_sided_members_corners() {
    use geometry_processing::bevel::level_line::band_side_tagging::{
        first_both_sided_at_a_mixed_corner, side_of,
    };
    use geometry_processing::bevel::level_line::triangle_edges::standing_map;
    use geometry_processing::bevel::level_line::BandGroup;
    // The cube's top-front edge 0 -> 1: the top (left, side 1) and the front (right, side 2).
    let positions = vec![
        Vector3::new(0.0, 0.0, 1.0),
        Vector3::new(1.0, 0.0, 1.0),
        Vector3::new(0.5, 0.5, 1.0),
        Vector3::new(0.5, 0.0, 0.0),
    ];
    let indices = vec![(0, 1, 2), (1, 0, 3)];
    let standing = standing_map(&indices);
    let edges = [(0, 1)];
    assert_eq!(
        side_of(
            &positions,
            &indices,
            &standing,
            &edges,
            Vector3::new(0.5, 0.3, 1.0)
        )
        .ok(),
        Some(1)
    );
    assert_eq!(
        side_of(
            &positions,
            &indices,
            &standing,
            &edges,
            Vector3::new(0.5, 0.0, 0.5)
        )
        .ok(),
        Some(2)
    );
    // An edge on one triangle is refused.
    assert!(side_of(
        &positions,
        &indices[..1],
        &standing_map(&indices[..1]),
        &edges,
        Vector3::ZERO
    )
    .is_err());

    let both = BandGroup {
        tag: 1,
        members: vec![0, 1],
        both_sided: vec![0],
        ..BandGroup::default()
    };
    let chains = vec![vec![(0, 1)], vec![(0, 1)]];
    // Every picked edge at the shared corners folds one way: no mixed corner.
    assert_eq!(
        first_both_sided_at_a_mixed_corner(
            Some(&positions),
            &indices,
            &chains,
            std::slice::from_ref(&both)
        )
        .ok(),
        Some(-1)
    );
    assert_eq!(
        first_both_sided_at_a_mixed_corner(None, &indices, &chains, &[both]).ok(),
        Some(-1)
    );
    assert_eq!(
        first_both_sided_at_a_mixed_corner(
            Some(&positions),
            &indices,
            &chains,
            &[BandGroup::default()]
        )
        .ok(),
        Some(-1)
    );
}

#[test]
fn rust_only_a_sides_candidates_are_the_triangles_the_line_dips_into() {
    use geometry_processing::bevel::level_line::band_triangles::candidates;
    use geometry_processing::bevel::level_line::{LevelLineMesh, LevelLineTriangle};
    // A long thin triangle whose corners all stand outside the line (setback 1 about the chain) but
    // whose middle dips across it, a far one, and one of another side.
    let mut mesh = LevelLineMesh::new();
    mesh.vertices = vec![
        Vector3::new(-5.0, 0.5, 0.0),
        Vector3::new(5.0, 0.5, 0.0),
        Vector3::new(0.0, 3.0, 0.0),
        Vector3::new(0.0, 20.0, 0.0),
        Vector3::new(1.0, 20.0, 0.0),
        Vector3::new(0.0, 21.0, 0.0),
    ];
    for (i, t) in [
        LevelLineTriangle::new(0, 1, 2, 1),
        LevelLineTriangle::new(3, 4, 5, 1),
        LevelLineTriangle::new(0, 1, 2, 2),
    ]
    .into_iter()
    .enumerate()
    {
        mesh.triangles.push(t);
        mesh.removed.push(false);
        mesh.attach(i as i32, t);
    }

    // A short chain at the origin: |(±5, 0.5)| and (0, 3) all stand past its setback of 1.
    let field = ChainField::from_polyline(
        &[Vector3::new(-0.1, 0.0, 0.0), Vector3::new(0.1, 0.0, 0.0)],
        false,
        &[1.0, 1.0],
    )
    .expect("a chain");
    assert_eq!(
        candidates(&mesh, &field, 1, 0.0).expect("no budget"),
        vec![0]
    );
    assert_eq!(
        candidates(&mesh, &field, 2, 0.0).expect("no budget"),
        vec![2]
    );
}

#[test]
fn rust_only_a_flat_patch_stops_at_bends_and_keeps_its_soft_rim() {
    use std::collections::HashSet;

    use geometry_processing::bevel::level_line::end_face_flow::flat_patch;
    use geometry_processing::bevel::level_line::triangle_edges::standing_map;
    // Two flat triangles (0, 1) and a third (2) bent gently down across the edge 1-2.
    let positions = vec![
        Vector3::new(0.0, 0.0, 0.0),
        Vector3::new(1.0, 0.0, 0.0),
        Vector3::new(1.0, 1.0, 0.0),
        Vector3::new(0.0, 1.0, 0.0),
        Vector3::new(2.0, 0.5, -0.2),
    ];
    let indices = vec![(0, 1, 2), (0, 2, 3), (1, 4, 2)];
    let standing = standing_map(&indices);
    let none: HashSet<(i32, i32)> = HashSet::new();
    let no_features = |_: i32, _: i32| false;
    let (patch, soft) = flat_patch(
        &positions,
        &indices,
        &standing,
        &none,
        &no_features,
        &[1],
        None,
        99,
    )
    .expect("no budget");
    assert_eq!(patch.iter().copied().collect::<Vec<_>>(), vec![1, 0]);
    assert_eq!(soft, vec![(1, 2)]);
    // A bend at the end vertex is crossed.
    let (through, through_soft) = flat_patch(
        &positions,
        &indices,
        &standing,
        &none,
        &no_features,
        &[1],
        None,
        2,
    )
    .expect("no budget");
    assert_eq!(through.len(), 3);
    assert!(through_soft.is_empty());
}

#[test]
fn rust_only_the_rounds_band_and_its_smooth_joints() {
    use std::cell::RefCell;

    use geometry_processing::bevel::level_line::band_gap_rounds_first::{
        smooth_joints_over_the_rounds, the_rounds_band, the_rounds_chains,
    };
    use geometry_processing::bevel::level_line::{GapFillResult, LevelLineMesh, LevelLineTriangle};
    // The input: one face in z = 0. The rounds added a triangle tiling that face again and one
    // facet of a round standing off it.
    let input = vec![
        Vector3::new(0.0, 0.0, 0.0),
        Vector3::new(1.0, 0.0, 0.0),
        Vector3::new(0.0, 1.0, 0.0),
    ];
    let input_indices = vec![(0, 1, 2)];
    let mut mesh = LevelLineMesh::new();
    mesh.vertices = vec![
        Vector3::new(0.0, 0.0, 0.0),
        Vector3::new(1.0, 0.0, 0.0),
        Vector3::new(0.0, 1.0, 0.0),
        Vector3::new(1.0, 1.0, 1.0),
    ];
    for (i, t) in [
        LevelLineTriangle::new(0, 1, 2, 0),
        LevelLineTriangle::new(1, 3, 2, 0),
    ]
    .into_iter()
    .enumerate()
    {
        mesh.triangles.push(t);
        mesh.removed.push(false);
        mesh.attach(i as i32, t);
    }

    let rounds = GapFillResult {
        mesh: Some(Rc::new(RefCell::new(mesh))),
        added_triangles: vec![0, 1],
        ..GapFillResult::default()
    };
    let (triangles, band) = the_rounds_band(&input, &input_indices, &rounds);
    assert_eq!(triangles, vec![(0, 1, 2), (1, 3, 2)]);
    assert_eq!(band, vec![false, true]);
    // Vertex 3 is interior to a crease both of whose edges there border the band; vertex 1 of the
    // other crease is not (its edge 0-1 is on the re-tiled face).
    let smooth = smooth_joints_over_the_rounds(
        &input,
        &input_indices,
        &[vec![1, 3, 2], vec![0, 1, 3]],
        &rounds,
    );
    assert_eq!(smooth, vec![Vector3::new(1.0, 1.0, 1.0)]);
    // One chain has no knife tip, so pass A builds nothing.
    assert!(the_rounds_chains(&input, &input_indices, &[vec![0, 1]], None).is_empty());
}

#[test]
fn rust_only_an_end_region_flows_over_a_narrow_face_as_far_as_the_band_reaches() {
    use std::collections::HashSet;

    use geometry_processing::bevel::level_line::end_face_flow::flow_over_a_narrow_end_face;
    use geometry_processing::bevel::level_line::triangle_edges::standing_map;
    // A flat strip of four triangles up y, its crossings 1-2 and 3-4 creases; the end region is
    // triangle 0 (tag 5) at the end vertex 0, the band's chain just below the strip.
    let positions = vec![
        Vector3::new(0.0, 0.0, 0.0),
        Vector3::new(1.0, 0.0, 0.0),
        Vector3::new(0.0, 1.0, 0.0),
        Vector3::new(1.0, 1.0, 0.0),
        Vector3::new(0.0, 2.0, 0.0),
        Vector3::new(1.0, 2.0, 0.0),
    ];
    let indices = vec![(0, 1, 2), (1, 3, 2), (2, 3, 4), (3, 5, 4)];
    let standing = standing_map(&indices);
    let none: HashSet<(i32, i32)> = HashSet::new();
    let creases = |a: i32, b: i32| matches!((a.min(b), a.max(b)), (1, 2) | (3, 4));
    let reach = |setback: f64| {
        ChainField::from_polyline(
            &[Vector3::new(0.5, -1.0, 0.0), Vector3::new(0.6, -1.0, 0.0)],
            false,
            &[setback, setback],
        )
        .expect("a chain")
    };
    let flow = |setback: f64| {
        let mut tags = vec![5, 0, 0, 0];
        flow_over_a_narrow_end_face(
            &positions,
            &indices,
            &standing,
            &none,
            &creases,
            0,
            5,
            &mut tags,
            &reach(setback),
            None,
        )
        .expect("no budget");
        tags
    };
    // A band reaching both creases carries the region over the whole strip; one reaching only the
    // first stops at the second.
    assert_eq!(flow(2.5), vec![5, 5, 5, 5]);
    assert_eq!(flow(1.5), vec![5, 5, 5, 0]);
    // A band reaching neither leaves the region as it was.
    assert_eq!(flow(0.5), vec![5, 0, 0, 0]);
}

#[test]
fn rust_only_the_room_is_bracketed_from_one_bare_pass() {
    use geometry_processing::bevel::level_line::foreign_edge_room_readings::bracket_from_one_bare_pass;
    use geometry_processing::bevel::level_line::LevelLineMesh;
    let mut mesh = LevelLineMesh::new();
    mesh.vertices = vec![
        Vector3::new(1.0, 3.0, 0.0),
        Vector3::new(3.0, 3.0, 0.0),
        Vector3::new(1.0, 30.0, 0.0),
        Vector3::new(3.0, 30.0, 0.0),
        Vector3::new(1.0, 0.0, 0.0),
        Vector3::new(3.0, 0.0, 0.0),
    ];
    let field_at = |size: f64| {
        ChainField::from_polyline(
            &[Vector3::new(0.0, 0.0, 0.0), Vector3::new(4.0, 0.0, 0.0)],
            false,
            &[size, size],
        )
        .expect("a chain")
    };
    // Edges 3 and 30 off, unit setbacks 1 .. 2: the room lies in [1.5, 3], and the far edge, whose
    // lower end 15 stands above 3, is pruned.
    let edges = [(0, 1, 0.0), (2, 3, 0.0)];
    let bracket =
        bracket_from_one_bare_pass(&mesh, &edges, &field_at, 1.0, 2.0).expect("no budget");
    assert!(bracket.bracketed && !bracket.touches_the_chain);
    assert_eq!((bracket.lo, bracket.hi), (1.5, 3.0));
    assert_eq!(bracket.searched, Some(vec![(0, 1, 0.0)]));
    // An edge on the chain touches it; a non-positive unit setback does not bracket.
    let touching =
        bracket_from_one_bare_pass(&mesh, &[(4, 5, 0.0)], &field_at, 1.0, 2.0).expect("no budget");
    assert!(!touching.bracketed && touching.touches_the_chain);
    let flat = bracket_from_one_bare_pass(&mesh, &edges, &field_at, 0.0, 2.0).expect("no budget");
    assert!(!flat.bracketed && flat.searched.is_none());
}

#[test]
fn rust_only_the_room_for_a_chain_reads_uniform_setbacks_in_one_pass_and_bisects_the_rest() {
    use geometry_processing::bevel::level_line::foreign_edge_room_readings::{bisect, room_for};
    use geometry_processing::bevel::level_line::LevelLineMesh;
    let mut mesh = LevelLineMesh::new();
    mesh.vertices = vec![
        Vector3::new(1.0, 3.0, 0.0),
        Vector3::new(3.0, 3.0, 0.0),
        Vector3::new(1.0, 0.0, 0.0),
        Vector3::new(3.0, 0.0, 0.0),
    ];
    let uniform = |size: f64| {
        ChainField::from_polyline(
            &[Vector3::new(0.0, 0.0, 0.0), Vector3::new(4.0, 0.0, 0.0)],
            false,
            &[size, size],
        )
        .expect("a chain")
    };
    let edges = [(0, 1, 0.0)];
    assert_eq!(
        room_for(&mesh, &[], &uniform, &[1.0, 1.0], 1.0).ok(),
        Some(f64::INFINITY)
    );
    assert!(
        (room_for(&mesh, &edges, &uniform, &[1.0, 1.0], 1.0).expect("no budget") - 3.0).abs()
            <= 1e-12
    );
    // A setback that varies 1 .. 2 along the chain: bracketed from the bare pass, then bisected to
    // the same room a plain bisection finds.
    let varying = |size: f64| {
        ChainField::from_polyline(
            &[Vector3::new(0.0, 0.0, 0.0), Vector3::new(4.0, 0.0, 0.0)],
            false,
            &[size, 2.0 * size],
        )
        .expect("a chain")
    };
    let room = room_for(&mesh, &edges, &varying, &[1.0, 2.0], 1.0).expect("no budget");
    let plain = bisect(&mesh, &edges, &varying, 0.0, 10.0).expect("no budget");
    assert!(room > 1.5 && room < 3.0);
    assert!((room - plain).abs() <= 1e-9);
    // An edge already on the chain leaves no room.
    assert_eq!(
        room_for(&mesh, &[(2, 3, 0.0)], &varying, &[1.0, 2.0], 1.0).ok(),
        Some(0.0)
    );
}
