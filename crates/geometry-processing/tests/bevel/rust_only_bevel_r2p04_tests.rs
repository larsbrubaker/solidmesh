//! Rust-only checks of the bevel core's R2.P04 package: the chain, pick and run-out fields' least
//! queries and the `ChainDistanceField` impls they complete, on the chain fields of
//! `chain_field_fixtures.rs`.

use std::rc::Rc;

use solidmesh::bevel::level_line::{
    BandGroupFieldsWithRunOuts, ChainDistanceField, ChainField, PickField, PointField,
};
use solidmesh_math::Vector3;

use crate::chain_field_fixtures::{l_chain, open_chain};

fn x_leg() -> ChainField {
    open_chain(
        vec![Vector3::new(0.0, 0.0, 0.0), Vector3::new(4.0, 0.0, 0.0)],
        vec![1.0, 1.0],
    )
}

#[test]
fn rust_only_a_chains_least_along_a_segment_and_over_a_triangle() {
    let chain = x_leg();
    // A segment two above the chain: its closed-form least is 2 - 1.
    let along = chain
        .min_along_segment(Vector3::new(0.0, 2.0, 0.0), Vector3::new(4.0, 2.0, 0.0))
        .expect("no budget");
    assert!((along.value - 1.0).abs() <= 1e-12);
    assert!((along.point.y - 2.0).abs() <= 1e-12);
    // A triangle the chain passes through reads -setback inside it.
    let over = chain
        .min_over_triangle(
            Vector3::new(1.0, -1.0, 0.0),
            Vector3::new(3.0, -1.0, 0.0),
            Vector3::new(2.0, 1.0, 0.0),
        )
        .expect("no budget");
    assert!((over.value - -1.0).abs() <= 1e-12);

    // Where the setback varies the convex search answers, and the indexed walk agrees with the
    // exhaustive reference.
    let l = l_chain();
    let (a, b) = (Vector3::new(0.0, -3.0, 0.0), Vector3::new(6.0, -3.0, 0.0));
    let indexed = l.min_along_segment(a, b).expect("no budget");
    let exhaustive = l
        .least_over_piece_exhaustive(Rc::new(move |p: Vector3| {
            ChainField::closest_on_segment(p, a, b)
        }))
        .expect("no budget");
    assert!((indexed.value - exhaustive.value).abs() <= 1e-9);
    assert!((indexed.point - exhaustive.point).length() <= 1e-6);
}

#[test]
fn rust_only_a_pick_and_a_wrapper_read_the_least_of_their_fields() {
    let x = Rc::new(x_leg());
    let y = Rc::new(open_chain(
        vec![Vector3::new(0.0, 5.0, 0.0), Vector3::new(4.0, 5.0, 0.0)],
        vec![1.0, 1.0],
    ));
    let pick = PickField::new(vec![x.clone(), y.clone()], &[0, 1]).expect("two members");
    let near_y = pick
        .min_along_segment(Vector3::new(0.0, 4.0, 0.0), Vector3::new(4.0, 4.0, 0.0))
        .expect("no budget");
    assert!((near_y.value - 0.0).abs() <= 1e-12);
    let over = pick
        .min_over_triangle(
            Vector3::new(0.0, 2.0, 0.0),
            Vector3::new(4.0, 2.0, 0.0),
            Vector3::new(2.0, 3.0, 0.0),
        )
        .expect("no budget");
    assert!((over.value - 1.0).abs() <= 1e-12);

    // Through the trait: a chain, a pick and a wrapper are all chain distance fields.
    let chain_field: Rc<dyn ChainDistanceField> = x.clone();
    assert!(chain_field.nothing_reaches_below(
        Vector3::new(0.0, 10.0, 0.0),
        Vector3::new(1.0, 11.0, 0.0),
        5.0
    ));
    let pick_field: Rc<dyn ChainDistanceField> = Rc::new(pick);
    let ball: Rc<dyn ChainDistanceField> =
        Rc::new(PointField::sphere(Vector3::new(2.0, 3.0, 0.0), 1.0));
    let sample_chain = x.clone();
    let wrapped = BandGroupFieldsWithRunOuts::new(
        chain_field,
        Rc::new(move |p| sample_chain.evaluate(p)),
        None,
        vec![ball],
    );
    // Along y = 2.5 the ball (0.5 off its centre, less 1) reads below the chain's 1.5.
    let lowered = wrapped
        .min_along_segment(Vector3::new(0.0, 2.5, 0.0), Vector3::new(4.0, 2.5, 0.0))
        .expect("no budget");
    assert!((lowered.value - -0.5).abs() <= 1e-12);
    let wrapped: Rc<dyn ChainDistanceField> = Rc::new(wrapped);
    let peak = wrapped
        .max_along_segment(
            Vector3::new(2.0, 0.0, 0.0),
            Vector3::new(2.0, 3.0, 0.0),
            1e-6,
        )
        .expect("no budget");
    assert!((peak.value - 0.5).abs() <= 1e-5);
    assert!(pick_field
        .min_along_segment(Vector3::new(0.0, 4.0, 0.0), Vector3::new(4.0, 4.0, 0.0))
        .is_ok());
}

#[test]
fn rust_only_each_group_reads_its_cutting_members_pick_and_its_run_outs() {
    use indexmap::IndexMap;
    use solidmesh::bevel::level_line::{band_group_fields_of, BandGroup, FieldKind};
    let x = Rc::new(x_leg());
    let y = Rc::new(open_chain(
        vec![Vector3::new(0.0, 5.0, 0.0), Vector3::new(4.0, 5.0, 0.0)],
        vec![1.0, 1.0],
    ));
    let fields = vec![x, y];
    let alone = BandGroup {
        tag: 1,
        members: vec![0],
        ..BandGroup::default()
    };
    // Chain 1 grows over group 2: it owns pieces there but does not cut it.
    let grown = BandGroup {
        tag: 2,
        members: vec![0, 1],
        grow_over_members: vec![1],
        ..BandGroup::default()
    };
    let ball: Rc<dyn ChainDistanceField> =
        Rc::new(PointField::sphere(Vector3::new(2.0, 3.0, 0.0), 1.0));
    let run_outs: IndexMap<i32, Vec<Rc<dyn ChainDistanceField>>> =
        [(1, vec![ball])].into_iter().collect();
    let groups = [alone, grown];
    let (fields_of, samples) =
        band_group_fields_of(&groups, false, &fields, Some(&run_outs)).expect("members");
    assert_eq!(fields_of.keys().copied().collect::<Vec<_>>(), vec![1, 2]);
    assert_eq!(fields_of[&1].kind(), FieldKind::WithRunOuts);
    assert_eq!(fields_of[&2].kind(), FieldKind::PickField);
    let near_y = Vector3::new(2.0, 4.5, 0.0);
    // Group 2 cuts by chain 0 alone, but its sample names chain 1 where that is nearer.
    assert!((fields_of[&2].value_at(near_y).expect("no budget") - 3.5).abs() <= 1e-12);
    assert_eq!(samples[&2](near_y).expect("no budget").chain, 1);
    // Group 1 reads the ball's run-out where it is lower, marked clipped.
    let by_ball = samples[&1](Vector3::new(2.0, 3.0, 0.0)).expect("no budget");
    assert!(by_ball.clipped && (by_ball.value - -1.0).abs() <= 1e-12);

    // A pick of one chain reads the bare chain field.
    let (only, _) = band_group_fields_of(&groups[..1], true, &fields[..1], None).expect("members");
    assert_eq!(only[&1].kind(), FieldKind::ChainField);
}

#[test]
fn rust_only_a_point_is_swallowed_at_its_distance_over_the_setback() {
    use solidmesh::bevel::level_line::RoomField;
    let chain = x_leg();
    // Two above a chain of unit setback: swallowed at size 2.
    assert!(
        (chain
            .swallow_scale(Vector3::new(2.0, 2.0, 0.0))
            .expect("no budget")
            - 2.0)
            .abs()
            <= 1e-12
    );
    // The L's setback climbs to 2 at its corner, so a point beside the corner goes at 2.5 / 2.
    let l = l_chain();
    let near_corner = l
        .swallow_scale(Vector3::new(4.0, -2.5, 0.0))
        .expect("no budget");
    assert!((near_corner - 1.25).abs() <= 1e-12);
    let y = Rc::new(open_chain(
        vec![Vector3::new(0.0, 5.0, 0.0), Vector3::new(4.0, 5.0, 0.0)],
        vec![1.0, 1.0],
    ));
    let pick = Rc::new(PickField::new(vec![Rc::new(x_leg()), y], &[0, 1]).expect("two members"));
    assert!(
        (pick
            .swallow_scale(Vector3::new(2.0, 4.0, 0.0))
            .expect("no budget")
            - 1.0)
            .abs()
            <= 1e-12
    );
    let room = RoomField::from_pick(pick);
    assert!(
        (room
            .swallow_scale(Vector3::new(2.0, 1.5, 0.0))
            .expect("no budget")
            - 1.5)
            .abs()
            <= 1e-12
    );
}

#[test]
fn rust_only_an_end_face_rising_off_a_convex_edges_side_is_opposite_sense() {
    use solidmesh::bevel::level_line::end_face_sense::is_opposite_sense;
    use solidmesh::bevel::level_line::triangle_edges::standing_map;
    // The unit cube's top-front edge along x (convex), its top side 1 and front side 2; at the end
    // vertex 1 a face rises off the top's edge 1-2 - a concave crease, the opposite sense.
    let positions = vec![
        Vector3::new(0.0, 0.0, 1.0),
        Vector3::new(1.0, 0.0, 1.0),
        Vector3::new(1.0, 0.5, 1.0),
        Vector3::new(1.0, 0.0, 0.0),
        Vector3::new(1.0, 0.25, 2.0),
    ];
    let rising = vec![(0, 1, 2), (1, 0, 3), (2, 1, 4)];
    let standing = standing_map(&rising);
    assert!(
        is_opposite_sense(&positions, &rising, &standing, &[1, 2, 0], (0, 1), 1)
            .expect("two sides")
    );
    // Falling away instead, the crease is convex like the chain: the same sense.
    let mut falling_positions = positions.clone();
    falling_positions[4] = Vector3::new(1.0, 0.25, 0.0);
    assert!(!is_opposite_sense(
        &falling_positions,
        &rising,
        &standing,
        &[1, 2, 0],
        (0, 1),
        1
    )
    .expect("two sides"));
    // No untagged face at the end: no crease, so not opposite.
    assert!(
        !is_opposite_sense(&positions, &rising, &standing, &[1, 2, 1], (0, 1), 1)
            .expect("two sides")
    );
    // A chain edge on one triangle is refused.
    let one_side = vec![(0, 1, 2)];
    assert!(is_opposite_sense(
        &positions,
        &one_side,
        &standing_map(&one_side),
        &[1],
        (0, 1),
        1
    )
    .is_err());
}

#[test]
fn rust_only_the_foreign_edge_tree_finds_the_nearest_edge_first() {
    use solidmesh::bevel::level_line::ForeignEdgeIndex;
    // Six unit edges standing parallel to the chain at y = 2 .. 7.
    let mut ends = Vec::new();
    for i in 0..6 {
        let y = 2.0 + i as f64;
        ends.push(Vector3::new(1.0, y, 0.0));
        ends.push(Vector3::new(3.0, y, 0.0));
    }
    let mut order: Vec<i32> = (0..6).collect();
    let mut nodes = Vec::new();
    ForeignEdgeIndex::add(&mut nodes, &ends, &mut order, 0, 6);
    let edges: Vec<(i32, i32)> = order.iter().map(|&i| (2 * i, 2 * i + 1)).collect();
    let sorted_ends: Vec<Vector3> = order
        .iter()
        .flat_map(|&i| [ends[(2 * i) as usize], ends[(2 * i + 1) as usize]])
        .collect();
    let index = Rc::new(ForeignEdgeIndex::new(edges, sorted_ends, nodes));
    let chain = x_leg();
    let found = ForeignEdgeIndex::least_approach(std::slice::from_ref(&index), &chain, 1.0, None)
        .expect("no budget");
    // The nearest edge (y = 2) approaches to 2 - 1; the far leaf is pruned unread.
    assert!((found.least - 1.0).abs() <= 1e-12);
    assert!(found.read < 6);
    assert_eq!(found.read as usize, found.near.len());
    // Skipping the nearest edge leaves the next, at y = 3.
    let nearest = found
        .near
        .iter()
        .find(|(_, a)| (*a - 1.0).abs() <= 1e-12)
        .map(|(e, _)| *e)
        .expect("the nearest edge was read");
    let skip_it = move |edge: (i32, i32)| edge == nearest;
    let next =
        ForeignEdgeIndex::least_approach(&[index], &chain, 1.0, Some(&skip_it)).expect("no budget");
    assert!((next.least - 2.0).abs() <= 1e-12);
}

#[test]
fn rust_only_a_clipped_chains_least_is_lifted_to_its_end_plane() {
    use solidmesh::bevel::level_line::EndRayBound;
    // The chain along x from 0 to 4, its last end bounded by the plane x = 4.
    let bound = EndRayBound {
        plane_normal: Vector3::new(1.0, 0.0, 0.0),
        ..EndRayBound::default()
    };
    let clipped = ChainField::from_polyline_bounded(
        &[Vector3::new(0.0, 0.0, 0.0), Vector3::new(4.0, 0.0, 0.0)],
        false,
        &[1.0, 1.0],
        f64::INFINITY,
        (false, true),
        (None, Some(bound)),
    )
    .expect("a chain");
    // Past the plane the field is the plane's reading wherever the chain's own is lower: across
    // x = 5 the least is 1, not the chain's 0 at (5, 0).
    let past = clipped
        .min_along_segment(Vector3::new(5.0, -0.5, 0.0), Vector3::new(5.0, 0.5, 0.0))
        .expect("no budget");
    assert!((past.value - 1.0).abs() <= 1e-9);
    let unclipped = x_leg()
        .min_along_segment(Vector3::new(5.0, -0.5, 0.0), Vector3::new(5.0, 0.5, 0.0))
        .expect("no budget");
    assert!((unclipped.value - 0.0).abs() <= 1e-12);
    // Inside the plane the chain's own least stands.
    let inside = clipped
        .min_along_segment(Vector3::new(3.0, -0.5, 0.0), Vector3::new(3.0, 0.5, 0.0))
        .expect("no budget");
    assert!((inside.value - -1.0).abs() <= 1e-12);
    // Over a triangle straddling the plane the clipped field max(|y| - 1, x - 4) is least on its
    // left side x = 3.75 + 0.75 y, where the two readings meet: y = -3/7, value -4/7.
    let over = clipped
        .min_over_triangle(
            Vector3::new(3.0, -1.0, 0.0),
            Vector3::new(6.0, -1.0, 0.0),
            Vector3::new(4.5, 1.0, 0.0),
        )
        .expect("no budget");
    assert!((over.value - -4.0 / 7.0).abs() <= 1e-6);
    assert!((over.point.y - -3.0 / 7.0).abs() <= 1e-4);
}

#[test]
fn rust_only_the_foreign_room_is_bisected_to_where_the_band_reaches_an_edge() {
    use solidmesh::bevel::level_line::foreign_edge_room_readings::{
        bisect, every_value_is_above_zero, where_the_room_is_set,
    };
    use solidmesh::bevel::level_line::LevelLineMesh;
    let mut mesh = LevelLineMesh::new();
    mesh.vertices = vec![Vector3::new(1.0, 3.0, 0.0), Vector3::new(3.0, 3.0, 0.0)];
    let field_at = |size: f64| {
        ChainField::from_polyline(
            &[Vector3::new(0.0, 0.0, 0.0), Vector3::new(4.0, 0.0, 0.0)],
            false,
            &[size, size],
        )
        .expect("a chain")
    };
    // An edge three off the chain: the band reaches it at size 3.
    let edges = [(0, 1, 0.0)];
    assert!(every_value_is_above_zero(&mesh, &edges, &field_at, 2.0).expect("no budget"));
    assert!(!every_value_is_above_zero(&mesh, &edges, &field_at, 4.0).expect("no budget"));
    let room = bisect(&mesh, &edges, &field_at, 0.0, 10.0).expect("no budget");
    assert!((room - 3.0).abs() <= 1e-9);
    // The other chain's own half setback taken off too: 3 = 1.5 s.
    let shared = [(0, 1, 0.5)];
    assert!((bisect(&mesh, &shared, &field_at, 0.0, 10.0).expect("no budget") - 2.0).abs() <= 1e-9);
    let at = where_the_room_is_set(&mesh, &edges, &field_at, &[1.0, 1.0], room).expect("no budget");
    assert!((at.y - 3.0).abs() <= 1e-12);
}

#[test]
fn rust_only_a_flood_stops_at_chain_and_feature_edges() {
    use std::collections::HashSet;

    use solidmesh::bevel::level_line::band_side_tagging::{connected_regions, flood};
    use solidmesh::bevel::level_line::triangle_edges::standing_map;
    // A strip of three triangles: 0 | 1 | 2 across the edges 1-2 and 2-3.
    let indices = vec![(0, 1, 2), (1, 3, 2), (2, 3, 4)];
    let standing = standing_map(&indices);
    let no_features = |_: i32, _: i32| false;
    let none: HashSet<(i32, i32)> = HashSet::new();
    assert_eq!(
        connected_regions(&indices, &standing, &none, &no_features).expect("no budget"),
        vec![1, 1, 1]
    );
    // The chain along 1-2 parts triangle 0 from the rest.
    let chain: HashSet<(i32, i32)> = [(1, 2)].into_iter().collect();
    assert_eq!(
        connected_regions(&indices, &standing, &chain, &no_features).expect("no budget"),
        vec![1, 2, 2]
    );
    // A feature edge 2-3 parts triangle 2 as well.
    let feature = |a: i32, b: i32| (a.min(b), a.max(b)) == (2, 3);
    assert_eq!(
        connected_regions(&indices, &standing, &chain, &feature).expect("no budget"),
        vec![1, 2, 3]
    );
    // A flood from triangle 2 that may not enter triangle 1 tags only itself.
    let mut tags = vec![0; 3];
    let not_one = |t: i32| t != 1;
    flood(
        &indices,
        &standing,
        &none,
        &no_features,
        &[2],
        7,
        &mut tags,
        Some(&not_one),
    )
    .expect("no budget");
    assert_eq!(tags, vec![0, 0, 7]);
}

#[test]
fn rust_only_chains_read_their_fold_and_a_knife_tips_end_face() {
    use std::collections::HashSet;

    use solidmesh::bevel::level_line::band_gap_rounds_first::{
        chain_senses, plane_of, takes_rounds_first, the_tips_end_face,
    };
    use solidmesh::bevel::level_line::triangle_edges::standing_map;
    // The cube's top-front edge (convex) and a floor-wall corner (concave), each on two triangles.
    let convex_at = vec![
        Vector3::new(0.0, 0.0, 1.0),
        Vector3::new(1.0, 0.0, 1.0),
        Vector3::new(0.5, 0.5, 1.0),
        Vector3::new(0.5, 0.0, 0.0),
    ];
    let concave_at = vec![
        Vector3::new(0.0, 0.0, 0.0),
        Vector3::new(1.0, 0.0, 0.0),
        Vector3::new(0.5, 1.0, 0.0),
        Vector3::new(0.5, 0.0, 1.0),
    ];
    let indices = vec![(0, 1, 2), (1, 0, 3)];
    let chain = vec![vec![0, 1]];
    assert_eq!(
        chain_senses(&convex_at, &indices, &chain, None),
        vec![Some(true)]
    );
    assert_eq!(
        chain_senses(&concave_at, &indices, &chain, None),
        vec![Some(false)]
    );
    // An edge on one triangle has no fold.
    assert_eq!(
        chain_senses(&convex_at, &indices[..1], &chain, None),
        vec![None]
    );
    // One chain alone is never built rounds first.
    assert!(!takes_rounds_first(&convex_at, &indices, &chain, None));

    // The end face of a chain ending at 1: of the two triangles on its last edge, the one on no
    // convex chain edge - here the front, since the top stands on the convex edge 1-2.
    let standing = standing_map(&indices);
    let convex_edges: HashSet<(i32, i32)> = [(1, 2)].into_iter().collect();
    let face = the_tips_end_face(
        &convex_at,
        &indices,
        &standing,
        &convex_edges,
        1,
        &[vec![0, 1]],
    );
    assert_eq!(face, Some(plane_of(&convex_at, indices[1])));
    assert!(the_tips_end_face(
        &convex_at,
        &indices,
        &standing,
        &HashSet::new(),
        1,
        &[vec![0, 1]]
    )
    .is_none());
}

#[test]
fn rust_only_a_side_flows_only_onto_a_gentle_convex_neighbour_it_reaches() {
    use solidmesh::bevel::level_line::side_flow::flows_onto_a_gentle_neighbour;
    use solidmesh::bevel::level_line::triangle_edges::standing_map;
    let mut positions = vec![
        Vector3::new(0.0, 0.0, 0.0),
        Vector3::new(1.0, 0.0, 0.0),
        Vector3::new(0.5, 1.0, 0.0),
        Vector3::new(0.5, -1.0, -0.5),
    ];
    let indices = vec![(0, 1, 2), (1, 0, 3)];
    let standing = standing_map(&indices);
    let reaching = ChainField::from_polyline(&[positions[0], positions[1]], false, &[1.0, 1.0])
        .expect("a chain");
    // The face across turns down by about 27 degrees: gentle and convex.
    assert!(
        flows_onto_a_gentle_neighbour(&positions, &indices, &standing, &reaching, 0, 1)
            .expect("no budget")
    );
    // A band that does not reach the crease does not flow.
    let far = ChainField::from_polyline(
        &[Vector3::new(0.0, 5.0, 0.0), Vector3::new(1.0, 5.0, 0.0)],
        false,
        &[1.0, 1.0],
    )
    .expect("a chain");
    assert!(
        !flows_onto_a_gentle_neighbour(&positions, &indices, &standing, &far, 0, 1)
            .expect("no budget")
    );
    // Folded back past a right angle it is a knife; turned up it is concave: neither flows.
    positions[3] = Vector3::new(0.5, 1.0, -0.1);
    assert!(
        !flows_onto_a_gentle_neighbour(&positions, &indices, &standing, &reaching, 0, 1)
            .expect("no budget")
    );
    positions[3] = Vector3::new(0.5, -1.0, 0.5);
    assert!(
        !flows_onto_a_gentle_neighbour(&positions, &indices, &standing, &reaching, 0, 1)
            .expect("no budget")
    );
}

#[test]
fn rust_only_an_end_regions_flood_crosses_a_soft_bend_while_the_band_reaches_it() {
    use solidmesh::bevel::level_line::end_face_flow::crosses_a_soft_bend;
    use solidmesh::bevel::level_line::triangle_edges::standing_map;
    let positions = vec![
        Vector3::new(0.0, 0.0, 0.0),
        Vector3::new(1.0, 0.0, 0.0),
        Vector3::new(0.5, 1.0, 0.0),
        Vector3::new(0.5, -1.0, 0.0),
    ];
    let indices = vec![(0, 1, 2), (1, 0, 3)];
    let standing = standing_map(&indices);
    let reach = ChainField::from_polyline(&[positions[0], positions[1]], false, &[1.0, 1.0])
        .expect("a chain");
    let up = [Vector3::new(0.0, 0.0, 1.0)];
    let sideways = [Vector3::new(1.0, 0.0, 0.0)];
    let crosses = |past: bool, rounds: bool, reach: Option<&ChainField>, sides: &[Vector3]| {
        crosses_a_soft_bend(
            &positions, &indices, &standing, 0, 1, past, rounds, reach, 0.0, sides,
        )
        .expect("no budget")
    };
    // At a size, the band's reach decides; neither flow, nothing crosses.
    assert!(crosses(false, true, Some(&reach), &up));
    assert!(crosses(true, false, Some(&reach), &up));
    assert!(!crosses(false, false, Some(&reach), &up));
    // Size-free, it crosses until the run comes round to lie alongside a side.
    assert!(!crosses(false, true, None, &up));
    assert!(crosses(false, true, None, &sideways));
    assert!(crosses(true, false, None, &sideways));
}
