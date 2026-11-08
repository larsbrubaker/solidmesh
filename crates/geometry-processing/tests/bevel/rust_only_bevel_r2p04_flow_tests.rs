//! Rust-only checks of the bevel core's R2.P04 tagging and flow readings - a point's side of a
//! chain, a both-sided member's mixed corner, the band's candidate triangles, the end region's flat
//! patch and its flow over a narrow end face, the rounds' band, and the foreign room's bracket and
//! reading. The least queries are in `rust_only_bevel_r2p04_tests.rs`.

use std::rc::Rc;

use solidmesh::bevel::level_line::ChainField;
use solidmesh_math::Vector3;

#[test]
fn rust_only_a_points_side_of_a_chain_and_a_both_sided_members_corners() {
    use solidmesh::bevel::level_line::band_side_tagging::{
        first_both_sided_at_a_mixed_corner, side_of,
    };
    use solidmesh::bevel::level_line::triangle_edges::standing_map;
    use solidmesh::bevel::level_line::BandGroup;
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
    use solidmesh::bevel::level_line::band_triangles::candidates;
    use solidmesh::bevel::level_line::{LevelLineMesh, LevelLineTriangle};
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

    use solidmesh::bevel::level_line::end_face_flow::flat_patch;
    use solidmesh::bevel::level_line::triangle_edges::standing_map;
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

    use solidmesh::bevel::level_line::band_gap_rounds_first::{
        smooth_joints_over_the_rounds, the_rounds_band, the_rounds_chains,
    };
    use solidmesh::bevel::level_line::{GapFillResult, LevelLineMesh, LevelLineTriangle};
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

    use solidmesh::bevel::level_line::end_face_flow::flow_over_a_narrow_end_face;
    use solidmesh::bevel::level_line::triangle_edges::standing_map;
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
    use solidmesh::bevel::level_line::foreign_edge_room_readings::bracket_from_one_bare_pass;
    use solidmesh::bevel::level_line::LevelLineMesh;
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
    use solidmesh::bevel::level_line::foreign_edge_room_readings::{bisect, room_for};
    use solidmesh::bevel::level_line::LevelLineMesh;
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

#[test]
fn rust_only_an_ends_end_face_is_flooded_as_its_end_region() {
    use std::collections::HashSet;

    use solidmesh::bevel::level_line::band_side_tagging_ends::{flood_ends, EndReach};
    use solidmesh::bevel::level_line::triangle_edges::standing_map;
    // The cube's top-front edge 0 -> 1 (top side 1, front side 2) ending on the x = 1 face, whose
    // creases with the sides are folds.
    let positions = vec![
        Vector3::new(0.0, 0.0, 1.0),
        Vector3::new(1.0, 0.0, 1.0),
        Vector3::new(1.0, 0.5, 1.0),
        Vector3::new(1.0, 0.0, 0.0),
    ];
    let indices = vec![(0, 1, 2), (1, 0, 3), (1, 3, 2)];
    let standing = standing_map(&indices);
    let chain_keys: HashSet<(i32, i32)> = [(0, 1)].into_iter().collect();
    let folds = |a: i32, b: i32| a != b;
    let tagged = |end_rays: (bool, bool)| {
        let mut tags = vec![1, 2, 0];
        flood_ends(
            &positions,
            &indices,
            &standing,
            &chain_keys,
            &folds,
            &[(0, 1)],
            &mut tags,
            None,
            end_rays,
            EndReach::default(),
            false,
        )
        .expect("two sides");
        tags
    };
    // The last end's face is its end region (tag 4); the first end has no untagged face.
    assert_eq!(tagged((false, false)), vec![1, 2, 4]);
    // An end carrying the end ray keeps its end face whole and untagged.
    assert_eq!(tagged((false, true)), vec![1, 2, 0]);
    // A flat end face bounded by folds is no round from the end on: it has no soft join to cross.
    let rounds = solidmesh::bevel::level_line::end_face_flow::rounds_from_the_end(
        &positions,
        &indices,
        &standing,
        &chain_keys,
        &folds,
        &[2],
        1,
    )
    .expect("no budget");
    assert!(!rounds);
}

#[test]
fn rust_only_the_sides_band_and_end_regions_of_a_cube_edge() {
    use solidmesh::bevel::level_line::band_side_tagging_ends::{
        end_regions_of, tag_band, tag_sides, SideReach,
    };
    let positions = vec![
        Vector3::new(0.0, 0.0, 1.0),
        Vector3::new(1.0, 0.0, 1.0),
        Vector3::new(1.0, 0.5, 1.0),
        Vector3::new(1.0, 0.0, 0.0),
    ];
    let indices = vec![(0, 1, 2), (1, 0, 3), (1, 3, 2)];
    let folds = |a: i32, b: i32| a != b;
    let none = |_: i32, _: i32| false;
    let (tags, divides) = tag_sides(
        &indices,
        &[(0, 1)],
        &folds,
        true,
        Some(&positions),
        (false, false),
        SideReach::default(),
    )
    .expect("two sides");
    assert!(divides);
    assert_eq!(tags, vec![1, 2, 4]);
    // Without end floods (or positions) the sides alone.
    let (bare, _) = tag_sides(
        &indices,
        &[(0, 1)],
        &folds,
        false,
        None,
        (false, false),
        SideReach::default(),
    )
    .expect("two sides");
    assert_eq!(bare, vec![1, 2, 0]);
    // With no folds at all one region reaches both sides: the chain does not divide them.
    let (joined, divides) = tag_sides(
        &indices,
        &[(0, 1)],
        &none,
        true,
        Some(&positions),
        (false, false),
        SideReach::default(),
    )
    .expect("two sides");
    assert!(!divides);
    assert_eq!(joined[1], 1);

    // The band at a size: the end face the band reaches is its end region.
    let field = ChainField::from_polyline(&[positions[0], positions[1]], false, &[0.5, 0.5])
        .expect("a chain");
    let (band, divides) =
        tag_band(&positions, &indices, &[(0, 1)], &folds, &field, 0.0, true).expect("two sides");
    assert!(divides);
    assert_eq!(band, vec![1, 2, 4]);

    // On a pick's tagging where side 2 is another chain's: still the last end's region.
    let regions = end_regions_of(
        &positions,
        &indices,
        &[(0, 1)],
        &folds,
        &[1, 2, 0],
        &|t| t == 1,
        (false, false),
        None,
        None,
    )
    .expect("two sides");
    assert_eq!(regions, vec![1, -1, 4]);
}

#[test]
fn rust_only_the_room_read_off_the_index_is_the_plain_reading() {
    use solidmesh::bevel::level_line::foreign_edge_room_readings::{room_for, room_from_index};
    use solidmesh::bevel::level_line::{ForeignEdgeIndex, LevelLineMesh};
    let mut mesh = LevelLineMesh::new();
    mesh.vertices = vec![
        Vector3::new(1.0, 3.0, 0.0),
        Vector3::new(3.0, 3.0, 0.0),
        Vector3::new(1.0, 30.0, 0.0),
        Vector3::new(3.0, 30.0, 0.0),
    ];
    let ends = mesh.vertices.clone();
    let mut order = vec![0, 1];
    let mut nodes = Vec::new();
    ForeignEdgeIndex::add(&mut nodes, &ends, &mut order, 0, 2);
    let edges: Vec<(i32, i32)> = order.iter().map(|&i| (2 * i, 2 * i + 1)).collect();
    let sorted: Vec<Vector3> = order
        .iter()
        .flat_map(|&i| [ends[(2 * i) as usize], ends[(2 * i + 1) as usize]])
        .collect();
    let index = Rc::new(ForeignEdgeIndex::new(edges, sorted, nodes));
    let indexes = [index];
    let uniform = |size: f64| {
        ChainField::from_polyline(
            &[Vector3::new(0.0, 0.0, 0.0), Vector3::new(4.0, 0.0, 0.0)],
            false,
            &[size, size],
        )
        .expect("a chain")
    };
    let varying = |size: f64| {
        ChainField::from_polyline(
            &[Vector3::new(0.0, 0.0, 0.0), Vector3::new(4.0, 0.0, 0.0)],
            false,
            &[size, 2.0 * size],
        )
        .expect("a chain")
    };
    let indexed =
        room_from_index(&mesh, &indexes, &uniform, &[1.0, 1.0], 1.0, None).expect("no budget");
    assert!((indexed - 3.0).abs() <= 1e-12);
    let plain_edges = [(0, 1, 0.0), (2, 3, 0.0)];
    let indexed =
        room_from_index(&mesh, &indexes, &varying, &[1.0, 2.0], 1.0, None).expect("no budget");
    let plain = room_for(&mesh, &plain_edges, &varying, &[1.0, 2.0], 1.0).expect("no budget");
    assert!((indexed - plain).abs() <= 1e-9);
    // Every edge exempt: nothing bounds the chain.
    let all = |_: (i32, i32)| true;
    assert_eq!(
        room_from_index(&mesh, &indexes, &varying, &[1.0, 2.0], 1.0, Some(&all)).ok(),
        Some(f64::INFINITY)
    );
}

#[test]
fn rust_only_a_picks_free_end_floods_its_end_face_as_an_end_group() {
    use solidmesh::bevel::level_line::pick_end_floods::flood_pick_ends;
    use solidmesh::bevel::level_line::BandGroup;
    let positions = vec![
        Vector3::new(0.0, 0.0, 1.0),
        Vector3::new(1.0, 0.0, 1.0),
        Vector3::new(1.0, 0.5, 1.0),
        Vector3::new(1.0, 0.0, 0.0),
    ];
    let indices = vec![(0, 1, 2), (1, 0, 3), (1, 3, 2)];
    let folds = |a: i32, b: i32| a != b;
    let side = |tag: i32| {
        let mut side_of = indexmap::IndexMap::new();
        side_of.insert(0, tag);
        BandGroup {
            tag,
            members: vec![0],
            side_of_chain: side_of,
            ..BandGroup::default()
        }
    };
    let mut groups = vec![side(1), side(2)];
    let mut tags = vec![1, 2, 0];
    flood_pick_ends(
        &positions,
        &indices,
        &[vec![(0, 1)]],
        &folds,
        None,
        &mut tags,
        &mut groups,
        None,
        None,
    )
    .expect("the end floods");
    // The end at 1 stands on the untagged face, which becomes the chain's end group; the end at 0
    // stands on no face of its own and joins nothing.
    assert_eq!(tags, vec![1, 2, 3]);
    assert_eq!(groups.len(), 3);
    assert_eq!(groups[2].tag, 3);
    assert_eq!(groups[2].members, vec![0]);
    assert_eq!(groups[2].end_of, 0);
    assert_eq!(groups[2].side_of_chain.get(&0), Some(&3));
    assert!(groups[..2].iter().all(|g| g.end_members.is_empty()));

    // A closed chain has no ends to flood.
    let mut closed_tags = vec![1, 2, 0];
    let mut closed_groups = vec![side(1), side(2)];
    flood_pick_ends(
        &positions,
        &indices,
        &[vec![(0, 1), (1, 2), (2, 0)]],
        &folds,
        None,
        &mut closed_tags,
        &mut closed_groups,
        None,
        None,
    )
    .expect("nothing to flood");
    assert_eq!(closed_tags, vec![1, 2, 0]);
    assert_eq!(closed_groups.len(), 2);
}

#[test]
fn rust_only_the_side_groups_of_one_and_two_chains() {
    use solidmesh::bevel::level_line::band_side_groups::{group_sides, has_a_both_sided_member};
    let positions = vec![
        Vector3::new(0.0, 0.0, 1.0),
        Vector3::new(1.0, 0.0, 1.0),
        Vector3::new(1.0, 0.5, 1.0),
        Vector3::new(1.0, 0.0, 0.0),
    ];
    let indices = vec![(0, 1, 2), (1, 0, 3), (1, 3, 2)];
    let folds = |a: i32, b: i32| a != b;
    let none = |_: i32, _: i32| false;
    // One chain: its two sides, and the end face flooded as the second end's region - what the
    // side tagging reads.
    let (tags, groups, undivided) = group_sides(
        &indices,
        &[vec![(0, 1)]],
        &folds,
        true,
        Some(&positions),
        None,
        None,
    )
    .expect("two sides");
    assert_eq!(tags, vec![1, 2, 4]);
    assert_eq!(undivided, -1);
    assert_eq!(groups.len(), 2);
    assert_eq!(groups[0].side_of_chain.get(&0), Some(&1));
    assert_eq!(groups[1].side_of_chain.get(&0), Some(&2));
    // With no folds one region reaches both sides: the chain is undivided.
    let (_, _, undivided) = group_sides(
        &indices,
        &[vec![(0, 1)]],
        &none,
        true,
        Some(&positions),
        None,
        None,
    )
    .expect("one region");
    assert_eq!(undivided, 0);
    // Two chains sharing the face between them: that face is one group with both as members.
    let (tags, groups, undivided) = group_sides(
        &indices,
        &[vec![(0, 1)], vec![(1, 3)]],
        &folds,
        false,
        None,
        None,
        None,
    )
    .expect("three groups");
    assert_eq!(undivided, -1);
    assert_eq!(tags, vec![1, 2, 3]);
    assert_eq!(groups.len(), 3);
    assert_eq!(groups[1].members, vec![0, 1]);
    assert!(groups.iter().all(|g| g.both_sided.is_empty()));
    let chains = [vec![0, 1], vec![1, 3]];
    assert!(!has_a_both_sided_member(
        &positions,
        &indices,
        Some(&chains),
        &[false, false],
        Some(&folds)
    )
    .expect("grouped"));
    assert!(!has_a_both_sided_member(&positions, &indices, None, &[], None).expect("no pick"));
}
