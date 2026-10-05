//! Rust-only checks of the bevel core's R2.P04 package: the chain, pick and run-out fields' least
//! queries and the `ChainDistanceField` impls they complete, on the chain fields of
//! `chain_field_fixtures.rs`.

use std::rc::Rc;

use geometry_processing::bevel::level_line::{
    BandGroupFieldsWithRunOuts, ChainDistanceField, ChainField, PickField, PointField,
};
use vector_math::Vector3;

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
    use geometry_processing::bevel::level_line::{band_group_fields_of, BandGroup, FieldKind};
    use indexmap::IndexMap;
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
    use geometry_processing::bevel::level_line::RoomField;
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
    use geometry_processing::bevel::level_line::end_face_sense::is_opposite_sense;
    use geometry_processing::bevel::level_line::triangle_edges::standing_map;
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
    use geometry_processing::bevel::level_line::ForeignEdgeIndex;
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
